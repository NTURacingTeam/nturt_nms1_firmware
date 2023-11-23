/**
 * @file common.h
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief
 *
 */

#ifndef NMS1_COMMON_H_
#define NMS1_COMMON_H_

// zephyr include
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

/// @brief IMU data.
struct imu_data {
  /// @brief acceleration in m/s^2
  float accel[3];

  /// @brief angular velocity in rad/s
  float gyro[3];

  /// @brief orientation quaternion
  float quat[4];
};

#endif  // NMS1_COMMON_H_
