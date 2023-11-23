/**
 * @file micro_ros.h
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief
 *
 */

#ifndef NMS1_MICRO_ROS_H_
#define NMS1_MICRO_ROS_H_

// zephyr include
#include <zephyr/device.h>
#include <zephyr/kernel.h>

// micro ros include
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

// ros msg include
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/float32.h>

// project include
#include "nms1/common.h"

#define MICRO_ROS_STACK_SIZE 4096
#define MICRO_ROS_THREAD_PRIORITY 5
#define MICRO_ROS_TASK_QUEUE_SIZE 10

#define MICRO_ROS_NODE_NAME "nms1"
#define MICRO_ROS_NODE_NAMESPACE ""
#define MICRO_ROS_UPDATE_PERIOD 50  // ms

/// @brief Micro ros task id.
enum micro_ros_task_id {
  MICRO_ROS_TASK_INIT = 0,
  MICRO_ROS_TASK_UPDATE,
  MICRO_ROS_TASK_PUB_IMU,
  MICRO_ROS_TASK_PUB_SPEED,
};

/// @brief Micro ros task.
struct micro_ros_task {
  enum micro_ros_task_id id;
  union {
    struct {
      int ret;
    } init;
    struct {
    } update;
    struct {
      struct imu_data imu_data;
    } pub_imu;
    struct {
      float speed;
    } pub_speed;
  };
};

/// @brief Control struct for micro_ros.
struct micro_ros {
  /* zephyr kernel object ----------------------------------------------------*/
  /// @brief Thread for micro_ros.
  struct k_thread micro_ros_thread;

  /// @brief Timer for periodically update micro_ros.
  struct k_timer update_timer;

  /// @brief Message queue for triggering micro_ros tasks and transmitting
  /// parameters.
  struct k_msgq task_queue;

  /// @brief Message queue buffer for task_queue.
  char
      task_queue_buf[MICRO_ROS_TASK_QUEUE_SIZE * sizeof(struct micro_ros_task)];

  /* micro ros ---------------------------------------------------------------*/
  /// @brief RCL allocator.
  rcl_allocator_t allocator;

  /// @brief RCL support.
  rclc_support_t support;

  /// @brief RCLC executor.
  rclc_executor_t executor;

  /// @brief RCL node.
  rcl_node_t node;

  /// @brief ROS publisher for IMU data.
  rcl_publisher_t imu_pub;

  /// @brief ROS publisher for speed.
  rcl_publisher_t speed_pub;

  /// @brief ROS subscription for target steer.
  rcl_subscription_t target_steer_sub;

  /// @brief ROS subscription for target speed.
  rcl_subscription_t target_speed_sub;

  /// @brief ROS message for target steer.
  std_msgs__msg__Float32 target_steer_msg;

  /// @brief ROS message for target speed.
  std_msgs__msg__Float32 target_speed_msg;
};

/**
 * @brief Initialize @ref micro_ros.
 *
 * @param[in,out] micro_ros Pointer to @ref micro_ros.
 * @param[in] serial Pointer to serial device.
 * @return 0 on success, negative on error.
 */
int micro_ros_init(struct micro_ros* micro_ros, const struct device* serial);

/**
 * @brief Publish IMU data.
 *
 * @param[in,out] micro_ros Pointer to @ref micro_ros.
 * @param[in] imu_data Pointer to @ref imu_data.
 * @return 0 on success, negative on error.
 */
int micro_ros_pub_imu(struct micro_ros* micro_ros,
                      const struct imu_data* imu_data);

/**
 * @brief Publish speed.
 *
 * @param[in,out] micro_ros Pointer to @ref micro_ros.
 * @param[in] speed Speed.
 * @return 0 on success, negative on error.
 */
int micro_ros_pub_speed(struct micro_ros* micro_ros, float speed);

#endif  // NMS1_MICRO_ROS_H_
