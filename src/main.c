// zephyr include
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/jy901b.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

// micro ros include
#include <microros_transports.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>

// ros msg include
#include <sensor_msgs/msg/imu.h>

// project include
#include "nms1/common.h"
#include "nms1/nms1.h"

LOG_MODULE_REGISTER(main);

static const struct device *const encoder = DEVICE_DT_GET(DT_ALIAS(encoder));
static const struct device *const imu = DEVICE_DT_GET(DT_ALIAS(imu));

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led), gpios);

static const struct device *const steer = DEVICE_DT_GET(DT_ALIAS(steer_servo));
static const struct device *const drive = DEVICE_DT_GET(DT_ALIAS(motor_servo));

static const struct device *const micro_ros_serial =
    DEVICE_DT_GET(DT_ALIAS(micro_ros_serial));

static struct nms1 nms1;

int main(void) {
  int ret;

  // hardware check
  if (!device_is_ready(encoder)) {
    LOG_ERR("Device %s is not ready.", encoder->name);
    return 0;
  }

  if (!device_is_ready(imu)) {
    LOG_ERR("Device %s is not ready.", imu->name);
    return 0;
  }

  if (!gpio_is_ready_dt(&led)) {
    LOG_ERR("Device %s is not ready.", led.port->name);
    return 0;
  }

  if (!device_is_ready(steer)) {
    LOG_ERR("Device %s is not ready.", steer->name);
    return 0;
  }

  if (!device_is_ready(drive)) {
    LOG_ERR("Device %s is not ready.", drive->name);
    return 0;
  }

  if (!device_is_ready(micro_ros_serial)) {
    LOG_ERR("Device %s is not ready.", micro_ros_serial->name);
    return 0;
  }

  // init
  ret = nms1_init(&nms1, encoder, imu, steer, drive, micro_ros_serial);
  if (ret != 0) {
    LOG_ERR("Failed to init nms1.");
    return ret;
  }

  while (true) {
    k_msleep(100);
  }
  return 0;
}
