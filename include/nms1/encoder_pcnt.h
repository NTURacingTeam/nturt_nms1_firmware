/**
 * @file encoder_pcnt.h
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief
 *
 */

#ifndef NMS1_ENCODER_PCNT_H
#define NMS1_ENCODER_PCNT_H

// glibc include
#include <stdbool.h>
#include <stdint.h>

// zephyr include
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#define ENCODER_PCNT_TOTAL_COUNT 65536

/// @brief Control struct for PCNT controlled encoder.
struct encoder_pcnt {
  /// @brief PCNT device.
  const struct device* dev;

  /// @brief Sensor trigger for lower and upper threshold.
  struct sensor_trigger trig;

  /// @brief Flag for trigger.
  bool triggered;

  /// @brief Amount of times the encoder had wrapped around @ref
  /// ENCODER_PCNT_TOTAL_COUNT.
  int64_t wrapped_times;
};

/**
 * @brief Initialize encoder_pcnt struct.
 * 
 * @param[in,out] encoder Pointer to @ref encoder_pcnt.
 * @param[in] pcnt PCNT device.
 * @return int 0 if success, negative error code if failure.
 */
int encoder_pcnt_init(struct encoder_pcnt* encoder, const struct device* pcnt);

/**
 * @brief Get encoder count.
 * 
 * @param[in] encoder Pointer to @ref encoder_pcnt.
 * @param[out] count Encoder count.
 * @return int 0 if success, negative error code if failure.
 */
int encoder_pcnt_get_count(const struct encoder_pcnt* encoder, int64_t* count);

#endif  // NMS1_ENCODER_PCNT_H
