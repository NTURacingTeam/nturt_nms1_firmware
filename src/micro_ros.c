#include "nms1/micro_ros.h"

// glibc include
#include <string.h>

// zephyr include
#include <zephyr/device.h>
#include <zephyr/drivers/sensor/jy901b.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

// micro ros include
#include <microros_transports.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>

// ros msg include
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/float32.h>

// project include
#include "nms1/common.h"
#include "nms1/nms1.h"

LOG_MODULE_REGISTER(micro_ros);

/* static function declation -------------------------------------------------*/
static void micro_ros_entry_point(void* arg1, void* arg2, void* arg3);

static int __micro_ros_init(struct micro_ros* micro_ros,
                            const struct device* serial);

static int __micro_ros_update(struct micro_ros* micro_ros);

static int __micro_ros_pub_imu(const struct micro_ros* micro_ros,
                               const struct imu_data* imu_data);

static int __micro_ros_pub_speed(const struct micro_ros* micro_ros,
                                 float speed);

static void onTargetSteer(const void* msg);

static void onTargetSpeed(const void* msg);

static void micro_ros_timer_cb(struct k_timer* timer);

/* static variable -----------------------------------------------------------*/
/// @todo: Should not be singleton
K_THREAD_STACK_DEFINE(micro_ros_stack_area, MICRO_ROS_STACK_SIZE);

/* function definition -------------------------------------------------------*/
int micro_ros_init(struct micro_ros* micro_ros, const struct device* serial) {
  k_msgq_init(&micro_ros->task_queue, micro_ros->task_queue_buf,
              sizeof(struct micro_ros_task), MICRO_ROS_TASK_QUEUE_SIZE);

  k_tid_t tid = k_thread_create(&micro_ros->micro_ros_thread,
                                micro_ros_stack_area, MICRO_ROS_STACK_SIZE,
                                micro_ros_entry_point, micro_ros, (void*)serial,
                                NULL, MICRO_ROS_THREAD_PRIORITY, 0, K_NO_WAIT);

  k_thread_name_set(tid, "micro_ros");

  // wait for micro ros init to finish in micro_ros thread
  struct micro_ros_task task;
  k_msgq_get(&micro_ros->task_queue, &task, K_FOREVER);

  return task.init.ret;
}

int micro_ros_pub_imu(struct micro_ros* micro_ros,
                      const struct imu_data* imu_data) {
  static struct micro_ros_task task = {
      .id = MICRO_ROS_TASK_PUB_IMU,
  };
  memcpy(&task.pub_imu.imu_data, imu_data, sizeof(struct imu_data));

  if (k_msgq_put(&micro_ros->task_queue, &task, K_NO_WAIT)) {
    return -EBUSY;
  }

  return 0;
}

int micro_ros_pub_speed(struct micro_ros* micro_ros, float speed) {
  static struct micro_ros_task task = {
      .id = MICRO_ROS_TASK_PUB_SPEED,
  };
  task.pub_speed.speed = speed;

  if (k_msgq_put(&micro_ros->task_queue, &task, K_NO_WAIT)) {
    return -EBUSY;
  }

  return 0;
}

/* static function definition ------------------------------------------------*/
static void micro_ros_entry_point(void* arg1, void* arg2, void* arg3) {
  struct micro_ros* micro_ros = arg1;
  const struct device* micro_ros_serial = arg2;

  int ret;
  ret = __micro_ros_init(micro_ros, micro_ros_serial);

  struct micro_ros_task task = {
      .id = MICRO_ROS_TASK_INIT,
      .init.ret = ret,
  };
  k_msgq_put(&micro_ros->task_queue, &task, K_NO_WAIT);

  if (ret != 0) {
    return;
  }

  // wait for init ret to be read from task_queue
  k_sleep(K_MSEC(10));

  while (true) {
    k_msgq_get(&micro_ros->task_queue, &task, K_FOREVER);

    switch (task.id) {
      case MICRO_ROS_TASK_UPDATE:
        ret = __micro_ros_update(micro_ros);
        if (ret != 0) {
          LOG_WRN("Failed to update micro ros. Return code %d.", ret);
        }
        break;

      case MICRO_ROS_TASK_PUB_IMU:
        ret = __micro_ros_pub_imu(micro_ros, &task.pub_imu.imu_data);
        if (ret != 0) {
          LOG_WRN("Failed to publish imu data. Return code %d.", ret);
        }
        break;

      case MICRO_ROS_TASK_PUB_SPEED:
        ret = __micro_ros_pub_speed(micro_ros, task.pub_speed.speed);
        if (ret != 0) {
          LOG_WRN("Failed to publish encoder data. Return code %d.", ret);
        }
        break;

      default:
        break;
    }
  }
}

static int __micro_ros_init(struct micro_ros* micro_ros,
                            const struct device* serial) {
  if (rmw_uros_set_custom_transport(
          MICRO_ROS_FRAMING_REQUIRED, (void*)serial, zephyr_transport_open,
          zephyr_transport_close, zephyr_transport_write,
          zephyr_transport_read) != RCL_RET_OK) {
    LOG_ERR("Failed to set rmw_uros serial transport.");
    goto err;
  }

  micro_ros->allocator = rcl_get_default_allocator();

  if (rclc_support_init(&micro_ros->support, 0, NULL, &micro_ros->allocator) !=
      RCL_RET_OK) {
    LOG_ERR("Failed to init rclc support.");
    goto err;
  }

  if (rclc_node_init_default(&micro_ros->node, MICRO_ROS_NODE_NAME,
                             MICRO_ROS_NODE_NAMESPACE,
                             &micro_ros->support) != RCL_RET_OK) {
    LOG_ERR("Failed to init ros node.");
    goto err_fini_support;
  }

  if (rclc_publisher_init_best_effort(
          &micro_ros->imu_pub, &micro_ros->node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
          "imu") != RCL_RET_OK) {
    LOG_ERR("Failed to init ros publisher \"imu\".");
    goto err_fini_node;
  }

  if (rclc_publisher_init_best_effort(
          &micro_ros->speed_pub, &micro_ros->node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
          "speed") != RCL_RET_OK) {
    LOG_ERR("Failed to init ros publisher \"speed\".");
    goto err_fini_node;
  }

  if (rclc_subscription_init_best_effort(
          &micro_ros->target_steer_sub, &micro_ros->node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
          "target_steer") != RCL_RET_OK) {
    LOG_ERR("Failed to init ros subscriber \"target_steer\".");
    goto err_fini_node;
  }

  if (rclc_subscription_init_best_effort(
          &micro_ros->target_speed_sub, &micro_ros->node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
          "target_speed") != RCL_RET_OK) {
    LOG_ERR("Failed to init ros subscriber \"target_speed\".");
    goto err_fini_node;
  }

  if (rclc_executor_init(&micro_ros->executor, &micro_ros->support.context, 2,
                         &micro_ros->allocator) != RCL_RET_OK) {
    LOG_ERR("Failed to init rclc executor.");
    goto err_fini_trans;
  }

  if (rclc_executor_add_subscription(
          &micro_ros->executor, &micro_ros->target_steer_sub,
          &micro_ros->target_steer_msg, onTargetSteer,
          ON_NEW_DATA) != RCL_RET_OK) {
    LOG_ERR("Failed to add subscription callback.");
    goto err_fini_trans;
  }

  if (rclc_executor_add_subscription(
          &micro_ros->executor, &micro_ros->target_speed_sub,
          &micro_ros->target_speed_msg, onTargetSpeed,
          ON_NEW_DATA) != RCL_RET_OK) {
    LOG_ERR("Failed to add subscription callback.");
    goto err_fini_trans;
  }

  if (rmw_uros_sync_session(5000) != RCL_RET_OK) {
    LOG_WRN("Failed to syncronize ros time.");
  }

  // start timer to trigger micro ros update
  k_timer_init(&micro_ros->update_timer, micro_ros_timer_cb, NULL);
  k_timer_start(&micro_ros->update_timer, K_NO_WAIT,
                K_MSEC(MICRO_ROS_UPDATE_PERIOD));

  return 0;

  // inhibit unused-result warning since we're cleaning up
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"

err_fini_trans:
  (void)rcl_publisher_fini(&micro_ros->imu_pub, &micro_ros->node);
  (void)rcl_publisher_fini(&micro_ros->speed_pub, &micro_ros->node);
  (void)rcl_subscription_fini(&micro_ros->target_steer_sub, &micro_ros->node);
  (void)rcl_subscription_fini(&micro_ros->target_speed_sub, &micro_ros->node);

err_fini_node:
  (void)rcl_node_fini(&micro_ros->node);

err_fini_support:
  rclc_support_fini(&micro_ros->support);

err:
  return -EIO;

#pragma GCC diagnostic pop
}

static int __micro_ros_update(struct micro_ros* micro_ros) {
  int ret;
  ret = rclc_executor_spin_some(&micro_ros->executor, RCL_MS_TO_NS(5));
  if (ret == RCL_RET_ERROR) {
    LOG_ERR("Failed to spin some.");
    return -EIO;
  }

  return 0;
}

static int __micro_ros_pub_imu(const struct micro_ros* micro_ros,
                               const struct imu_data* imu_data) {
  static const float accel_var = (JY901B_ACCEL_STD * SENSOR_G / 1000000) *
                                 (JY901B_ACCEL_STD * SENSOR_G / 1000000);
  static const float gyro_var = (JY901B_GYRO_STD * SENSOR_PI / 180 / 1000000) *
                                (JY901B_GYRO_STD * SENSOR_PI / 180 / 1000000);
  static const float orien_roll_pitch_var =
      JY901B_ORIEN_ROLL_PITCH_STD * JY901B_ORIEN_ROLL_PITCH_STD;
  static const float orien_yaw_var =
      JY901B_ORIEN_YAW_STD * JY901B_ORIEN_YAW_STD;

  static sensor_msgs__msg__Imu msg = {
      .header.frame_id.data = "imu",
      .linear_acceleration_covariance =
          {
              [0] = accel_var,
              [4] = accel_var,
              [8] = accel_var,
          },
      .angular_velocity_covariance =
          {
              [0] = gyro_var,
              [4] = gyro_var,
              [8] = gyro_var,
          },
      .orientation_covariance =
          {
              [0] = orien_roll_pitch_var,
              [4] = orien_roll_pitch_var,
              [8] = orien_yaw_var,
          },
  };

  if (rmw_uros_epoch_synchronized()) {
    int64_t nano = rmw_uros_epoch_nanos();
    msg.header.stamp.sec = nano / 1000000000;
    msg.header.stamp.nanosec = nano % 1000000000;
  }

  msg.orientation.w = imu_data->quat[0];
  msg.orientation.x = imu_data->quat[1];
  msg.orientation.y = imu_data->quat[2];
  msg.orientation.z = imu_data->quat[3];

  msg.angular_velocity.x = imu_data->gyro[0];
  msg.angular_velocity.y = imu_data->gyro[1];
  msg.angular_velocity.z = imu_data->gyro[2];

  msg.linear_acceleration.x = imu_data->accel[0];
  msg.linear_acceleration.y = imu_data->accel[1];
  msg.linear_acceleration.z = imu_data->accel[2];

  if (rcl_publish(&micro_ros->imu_pub, &msg, NULL) != RCL_RET_OK) {
    LOG_ERR("Failed to publish imu data.");
    return -EIO;
  }

  return 0;
}

static int __micro_ros_pub_speed(const struct micro_ros* micro_ros,
                                 float speed) {
  static std_msgs__msg__Float32 msg;

  msg.data = speed;

  if (rcl_publish(&micro_ros->speed_pub, &msg, NULL) != RCL_RET_OK) {
    LOG_ERR("Failed to publish encoder data.");
    return -EIO;
  }

  return 0;
}

static void onTargetSteer(const void* msg) {
  struct micro_ros* micro_ros =
      CONTAINER_OF(msg, struct micro_ros, target_steer_msg);
  struct nms1* nms1 = CONTAINER_OF(micro_ros, struct nms1, micro_ros);

  nms1_steer_command(nms1, micro_ros->target_steer_msg.data);
}

static void onTargetSpeed(const void* msg) {
  struct micro_ros* micro_ros =
      CONTAINER_OF(msg, struct micro_ros, target_speed_msg);
  struct nms1* nms1 = CONTAINER_OF(micro_ros, struct nms1, micro_ros);

  nms1_drive_command(nms1, micro_ros->target_speed_msg.data);
}

static void micro_ros_timer_cb(struct k_timer* timer) {
  struct micro_ros* micro_ros =
      CONTAINER_OF(timer, struct micro_ros, update_timer);

  static const struct micro_ros_task task = {
      .id = MICRO_ROS_TASK_UPDATE,
  };
  k_msgq_put(&micro_ros->task_queue, &task, K_NO_WAIT);
}
