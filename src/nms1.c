#include "nms1/nms1.h"

// zephyr include
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/jy901b.h>
#include <zephyr/drivers/servo.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

// project include
#include "nms1/common.h"
#include "nms1/micro_ros.h"

LOG_MODULE_REGISTER(nms1);

/* static function declaration -----------------------------------------------*/
static int steer_angle_to_servo_duty(float angle);

static int drive_speed_to_servo_duty(float speed);

static void nms1_imu_timer_cb(struct k_timer* timer);

static void nms1_encoder_timer_cb(struct k_timer* timer);

static void nms1_drive_timer_cb(struct k_timer* timer);

static void nms1_imu_work_handler(struct k_work* work);

/* function definition -------------------------------------------------------*/
int nms1_init(struct nms1* nms1, const struct device* pcnt,
              const struct device* imu, const struct device* steer,
              const struct device* drive,
              const struct device* micro_ros_serial) {
  static const float speed_filter_cof[NMS1_SPEED_FIR_SIZE] = {
      [0 ... NMS1_SPEED_FIR_SIZE - 1] = 1.0F / NMS1_SPEED_FIR_SIZE,
  };
  static const float drive_filter_cof[NMS1_DRIVE_FIR_SIZE] = {
      [0 ... NMS1_DRIVE_FIR_SIZE - 1] = 1.0F / NMS1_DRIVE_FIR_SIZE,
  };

  // device
  nms1->imu = imu;

  nms1->steer = steer;
  nms1->drive = drive;

  // kernel object
  k_timer_init(&nms1->imu_timer, nms1_imu_timer_cb, NULL);
  k_timer_init(&nms1->encoder_timer, nms1_encoder_timer_cb, NULL);
  k_timer_init(&nms1->drive_timer, nms1_drive_timer_cb, NULL);

  k_work_init(&nms1->imu_work, nms1_imu_work_handler);

  int ret;
  ret = encoder_pcnt_init(&nms1->encoder, pcnt);
  if (ret != 0) {
    LOG_ERR("Failed to init encoder.");
    return ret;
  }

  ret = micro_ros_init(&nms1->micro_ros, micro_ros_serial);
  if (ret != 0) {
    LOG_ERR("Failed to init micro ros.");
    return ret;
  }

  // project data
  arm_fir_init_f32(&nms1->speed_filter, NMS1_SPEED_FIR_SIZE, speed_filter_cof,
                   nms1->speed_filter_buf, 1);
  arm_fir_init_f32(&nms1->drive_filter, NMS1_DRIVE_FIR_SIZE, drive_filter_cof,
                   nms1->drive_filter_buf, 1);

  // start
  k_timer_start(&nms1->imu_timer, K_NO_WAIT, K_MSEC(NMS1_IMU_UPDATE_PERIOD));
  k_timer_start(&nms1->encoder_timer, K_NO_WAIT,
                K_MSEC(NMS1_ENCODER_UPDATE_PERIOD));
  k_timer_start(&nms1->drive_timer, K_NO_WAIT,
                K_MSEC(NMS1_DRIVE_UPDATE_PERIOD));

  return 0;
}

int nms1_steer_command(const struct nms1* nms1, float angle) {
  int ret;
  ret = servo_set(nms1->steer, steer_angle_to_servo_duty(angle));
  if (ret != 0) {
    LOG_ERR("Failed to set steer angle.");
    return ret;
  }

  return 0;
}

int nms1_drive_command( struct nms1* nms1, float speed) {
  atomic_set(&nms1->drive_command, PUN_TO_LONG(speed));

  return 0;
}

/* static function definition ------------------------------------------------*/
static int steer_angle_to_servo_duty(float angle) {
  return angle / NMS1_STEER_GEAR_RATIO * SERVO_MAX_DUTY;
}

static int drive_speed_to_servo_duty(float speed) {
  return speed * SERVO_MAX_DUTY;
}

static void nms1_imu_timer_cb(struct k_timer* timer) {
  struct nms1* nms1 = CONTAINER_OF(timer, struct nms1, imu_timer);

  // submit imu work to system work queue to avoid running in ISR context
  k_work_submit(&nms1->imu_work);
}

static void nms1_encoder_timer_cb(struct k_timer* timer) {
  struct nms1* nms1 = CONTAINER_OF(timer, struct nms1, encoder_timer);

  int ret;
  int64_t count;
  ret = encoder_pcnt_get_count(&nms1->encoder, &count);
  if (ret != 0) {
    LOG_ERR("Failed to update encoder.");
    return;
  }

  float raw_speed = (float)(count - nms1->encoder_count_last) *
                    NMS1_ENCODER_COUNT_TO_SPEED * 1000 /
                    NMS1_ENCODER_UPDATE_PERIOD;
  nms1->encoder_count_last = count;

  float speed;
  arm_fir_f32(&nms1->speed_filter, &raw_speed, &speed, 1);

  ret = micro_ros_pub_speed(&nms1->micro_ros, speed);
  if (ret != 0) {
    LOG_ERR("Failed to publish encoder data. Return code %d.", ret);
    return;
  }
}

static void nms1_drive_timer_cb(struct k_timer* timer) {
  struct nms1* nms1 = CONTAINER_OF(timer, struct nms1, drive_timer);

  int ret;
  float drive = PUN_TO_FLOAT((nms1->drive_command));

  float filtered_drive;
  arm_fir_f32(&nms1->drive_filter, &drive, &filtered_drive, 1);

  ret = servo_set(nms1->drive, drive_speed_to_servo_duty(filtered_drive));
  if (ret != 0) {
    LOG_ERR("Failed to set drive speed.");
  }
}

static void nms1_imu_work_handler(struct k_work* work) {
  struct nms1* nms1 = CONTAINER_OF(work, struct nms1, imu_work);

  int ret;
  ret = sensor_sample_fetch(nms1->imu);
  if (ret != 0) {
    LOG_WRN("Device %s failed to fetch sample. Return code %d.",
            nms1->imu->name, ret);
    return;
  }

  struct sensor_value val[4];
  sensor_channel_get(nms1->imu, SENSOR_CHAN_ACCEL_XYZ, val);
  nms1->imu_data.accel[0] = sensor_value_to_float(val);
  nms1->imu_data.accel[1] = sensor_value_to_float(val + 1);
  nms1->imu_data.accel[2] = sensor_value_to_float(val + 2);

  sensor_channel_get(nms1->imu, SENSOR_CHAN_GYRO_XYZ, val);
  nms1->imu_data.gyro[0] = sensor_value_to_float(val);
  nms1->imu_data.gyro[1] = sensor_value_to_float(val + 1);
  nms1->imu_data.gyro[2] = sensor_value_to_float(val + 2);

  sensor_channel_get(nms1->imu, SENSOR_CHAN_QUAT_WXYZ, val);
  nms1->imu_data.quat[0] = sensor_value_to_float(val);
  nms1->imu_data.quat[1] = sensor_value_to_float(val + 1);
  nms1->imu_data.quat[2] = sensor_value_to_float(val + 2);
  nms1->imu_data.quat[3] = sensor_value_to_float(val + 3);

  // publish imu data
  ret = micro_ros_pub_imu(&nms1->micro_ros, &nms1->imu_data);
  if (ret != 0) {
    LOG_ERR("Failed to publish imu data. Return code %d.", ret);
    return;
  }
}
