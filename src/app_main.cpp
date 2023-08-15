#include "../inc/app_main.h"
#include "../inc/hardware.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdio.h>

#include "../inc/hardware.h"
#include "../inc/posture.h"

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

  const int dt_ms = 2;
  std::array<double, 3> d_accel, d_gyro, d_magn, euler;
  std::array<float, 3> f_accel, f_gyro, f_magn;
  std::array<float, 4> quad;
  posture::MahonyAHRS mahony((float)dt_ms / 1000, 10, 50);
  long i = 0;
  for (;;) {
    k_sleep(K_MSEC(dt_ms));
    i++;

    if (int ret = hardware::ReadIMU(d_accel, d_gyro, d_magn) < 0) {
      LOG_ERR("fail to read IMU, ret=%d", ret);
      continue;
    }

    for (int i = 0; i < 3; i++) {
      f_accel[i] = (float)d_accel[i];
      f_gyro[i] = (float)d_accel[i];
      f_magn[i] = (float)d_magn[i];
    }

    mahony.Update(f_gyro[0], f_gyro[1], f_gyro[2], f_accel[0], f_accel[1], f_accel[2], f_magn[0], f_magn[1], f_magn[2]);
    euler = mahony.GetEuler();
    quad = mahony.GetQuaternion();
    if (i % 3 == 0) {
      LOG_INF("p %f r %f y %f", euler[0], euler[1], euler[2]);
      //LOG_INF("q %f %f %f %f", quad[0], quad[1], quad[2], quad[3]);
    }
  }
}