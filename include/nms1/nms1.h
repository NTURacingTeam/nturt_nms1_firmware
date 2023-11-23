/**
 * @file nms1.h
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief
 */

#ifndef NMS1_NMS1_H_
#define NMS1_NMS1_H_

// glibc include
#include <stdbool.h>
#include <stdint.h>

// zephyr include
#include <zephyr/device.h>
#include <zephyr/kernel.h>

// project include
#include "nms1/common.h"
#include "nms1/encoder_pcnt.h"
#include "nms1/micro_ros.h"

#define NMS1_IMU_UPDATE_PERIOD 50      // ms
#define NMS1_ENCODER_UPDATE_PERIOD 50  // ms

#define NMS1_ENCODER_COUNT_TO_SPEED 0.001F  // 1 count = 0.001 m/s

/// @brief Control struct for nms1.
struct nms1 {
  /* zephyr devices ----------------------------------------------------------*/
  const struct device* imu;

  const struct device* steer;
  const struct device* drive;

  /* zephyr kernel object ----------------------------------------------------*/
  /// @brief Timer for periodically receiving IMU data.
  struct k_timer imu_timer;

  /// @brief Timer for periodically receiving encoder data.
  struct k_timer encoder_timer;

  /// @brief Work for deffering receiving IMU data to system work queue.
  struct k_work imu_work;

  /* project control struct --------------------------------------------------*/
  /// @brief Control struct for encoder.
  struct encoder_pcnt encoder;

  /// @brief Control struct for micro_ros.
  struct micro_ros micro_ros;

  /* project data ------------------------------------------------------------*/
  /// @brief IMU data.
  struct imu_data imu_data;

  /// @brief Last encoder count.
  int64_t encoder_count_last;
};

/**
 * @brief Initialize @ref nms1.
 *
 * @param[in,out] nms1 Pointer to @ref nms1.
 * @param[in] encoder Pointer to encoder device.
 * @param[in] imu Pointer to imu device.
 * @param[in] steer Pointer to steer device.
 * @param[in] drive Pointer to drive device.
 * @param[in] micro_ros_serial Pointer to micro_ros serial device.
 * @return 0 if success, negative errno code if fail.
 */
int nms1_init(struct nms1* nms1, const struct device* encoder,
              const struct device* imu, const struct device* steer,
              const struct device* drive,
              const struct device* micro_ros_serial);

/**
 * @brief Send steer command.
 * 
 * @param[in] nms1 Pointer to @ref nms1.
 * @param[in] angle Steer angle.
 * @return 0 if success, negative errno code if fail.
 */
int nms1_steer_command(const struct nms1* nms1, float angle);

/**
 * @brief Send drive command.
 * 
 * @param[in] nms1 Pointer to @ref nms1.
 * @param[in] speed Drive speed.
 * @return 0 if success, negative errno code if fail.
 */
int nms1_drive_command(const struct nms1* nms1, float speed);

#endif  // NMS1_NMS1_H_
