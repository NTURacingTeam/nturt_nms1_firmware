#include "nms1/encoder_pcnt.h"

// glibc include
#include <stdbool.h>
#include <stdint.h>

// zephyr include
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(encoder_pcnt);

/* static function declaration -----------------------------------------------*/
static void encoder_pcnt_trigger_handler(const struct device* dev,
                                         const struct sensor_trigger* trig);

/* function definition -------------------------------------------------------*/
int encoder_pcnt_init(struct encoder_pcnt* encoder, const struct device* pcnt) {
  encoder->dev = pcnt;
  encoder->trig = (struct sensor_trigger){
      .chan = SENSOR_CHAN_ROTATION,
      .type = SENSOR_TRIG_THRESHOLD,
  };
  encoder->triggered = false;
  encoder->wrapped_times = 0;

  int ret;
  struct sensor_value val;

  val.val1 = 32767;
  ret = sensor_attr_set(encoder->dev, SENSOR_CHAN_ROTATION,
                        SENSOR_ATTR_UPPER_THRESH, &val);
  if (ret != 0) {
    LOG_ERR("Device %s failed to set upper threshold.\n", encoder->dev->name);
    return ret;
  }

  val.val1 = -32767;
  ret = sensor_attr_set(encoder->dev, SENSOR_CHAN_ROTATION,
                        SENSOR_ATTR_LOWER_THRESH, &val);
  if (ret != 0) {
    LOG_ERR("Device %s failed to set lower threshold.\n", encoder->dev->name);
    return ret;
  }

  ret = sensor_trigger_set(encoder->dev, &encoder->trig,
                           encoder_pcnt_trigger_handler);
  if (ret != 0) {
    LOG_ERR("Device %s failed to set trigger.\n", encoder->dev->name);
    return ret;
  }

  return 0;
}

int encoder_pcnt_get_count(const struct encoder_pcnt* encoder, int64_t* count) {
  int ret;
  struct sensor_value val;

  ret = sensor_sample_fetch(encoder->dev);
  if (ret != 0) {
    LOG_ERR("Device %s failed to fetch sample.\n", encoder->dev->name);
    return ret;
  }

  ret = sensor_channel_get(encoder->dev, SENSOR_CHAN_ROTATION, &val);
  if (ret != 0) {
    LOG_ERR("Device %s failed to get data.\n", encoder->dev->name);
    return ret;
  }

  *count = encoder->wrapped_times * ENCODER_PCNT_TOTAL_COUNT + val.val1;

  return 0;
}

/* static function definition ------------------------------------------------*/
static void encoder_pcnt_trigger_handler(const struct device* dev,
                                         const struct sensor_trigger* trig) {
  struct encoder_pcnt* encoder = CONTAINER_OF(trig, struct encoder_pcnt, trig);

  int ret;
  struct sensor_value val;

  ret = sensor_sample_fetch(dev);
  if (ret != 0) {
    LOG_WRN("Device %s failed to fetch sample. Return code %d.\n", dev->name,
            ret);
    return;
  }

  ret = sensor_channel_get(dev, SENSOR_CHAN_ROTATION, &val);
  if (ret != 0) {
    LOG_WRN("Device %s failed to get data. Return code %d.\n", dev->name, ret);
    return;
  }

  if (encoder->triggered) {
    if (val.val1 > 0) {
      encoder->wrapped_times--;
    } else {
      encoder->wrapped_times++;
    }
    encoder->triggered = false;
  } else {
    encoder->triggered = true;
  }
}
