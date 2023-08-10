#include "../inc/app_main.h"
#include "../inc/hardware.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zephyr.h>
#include <stdio.h>

#include "../inc/hardware.h"

LOG_MODULE_REGISTER(app_main);

using namespace RbfpidBalbot;

void AppMain(void) {
  LOG_INF("hardware initialization start");
  if (int ret = hardware::CheckHardware() < 0) {
    LOG_ERR("fail to check hardware, ret=%d", ret);
    return;
  }
  if (int ret = hardware::InitHardware() < 0) {
    LOG_ERR("fail to initiate hardware, ret=%d", ret);
    return;
  }
  LOG_INF("hardware initialization complete");

  /* application logic */
  LOG_INF("application started");
  LOG_INF("RBF-PID Balbot");

  for (;;) {
    k_sleep(K_MSEC(500));
    auto dev = hardware::imu;
    struct sensor_value temperature;
    struct sensor_value accel[3];
    struct sensor_value gyro[3];
    int rc = sensor_sample_fetch(dev);

    if (rc == 0) {
      rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
    }
    if (rc == 0) {
      rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
    }
    if (rc == 0) {
      LOG_INF(
          "accel %f %f %f m/s^2 gyro  %f %f %f rad/s\n",
          sensor_value_to_double(&accel[0]), sensor_value_to_double(&accel[1]),
          sensor_value_to_double(&accel[2]), sensor_value_to_double(&gyro[0]),
          sensor_value_to_double(&gyro[1]), sensor_value_to_double(&gyro[2]));
    } else {
      LOG_ERR("sample fetch/get failed: %d\n", rc);
    }
  }
}