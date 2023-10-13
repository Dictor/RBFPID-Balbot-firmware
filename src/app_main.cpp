#include "../inc/app_main.h"

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <string>
#include <vector>

#include "../inc/hardware.h"
#include "../inc/posture.h"
#include "../inc/rbfpid.h"

LOG_MODULE_REGISTER(app_main);

using namespace RbfpidBalbot;

void AppMain(void) {
  LOG_INF("hardware initialization start");
  if (int ret = hardware::CheckHardware(); ret < 0) {
    LOG_ERR("fail to check hardware, ret=%d", ret);
    gpio_pin_set_dt(&hardware::err_led, 1);
    return;
  }
  if (int ret = hardware::InitHardware(); ret < 0) {
    LOG_ERR("fail to initiate hardware, ret=%d", ret);
    gpio_pin_set_dt(&hardware::err_led, 1);
    return;
  }
  LOG_INF("hardware initialization complete");

  /* application logic */
  LOG_INF("application started");
  LOG_INF("RBF-PID Balbot");

  const int dt_ms = 2;
  const double u_motor_factor = 2500;
  std::array<double, 3> d_accel, d_gyro, d_magn, euler;
  std::array<float, 3> f_accel, f_gyro, f_magn;
  std::array<float, 4> quad;
  posture::MahonyAHRS mahony((float)dt_ms / 1000, 100, 500);
  control::RBFPID pid(3, 8, pow(10, 6), 0.01, 10000, 0.01, 100);
  long i = 0;
  double u, uf;
  char telemetry[100];
  auto gain = pid.ReadGain();

  for (;;) {
    k_sleep(K_MSEC(dt_ms));
    i++;

    if (int ret = hardware::ReadIMU(d_accel, d_gyro, d_magn); ret < 0) {
      LOG_ERR("fail to read IMU, ret=%d", ret);
      gpio_pin_set_dt(&hardware::err_led, 1);
      continue;
    }

    for (int i = 0; i < 3; i++) {
      f_accel[i] = (float)d_accel[i];
      f_gyro[i] = (float)d_accel[i];
      f_magn[i] = (float)d_magn[i];
    }

    mahony.Update(f_gyro[0], f_gyro[1], f_gyro[2], f_accel[0], f_accel[1],
                  f_accel[2], f_magn[0], f_magn[1], f_magn[2]);
    euler = mahony.GetEuler();
    quad = mahony.GetQuaternion();
    u = pid.Update(-euler[0], euler[0]);
    uf = (u / u_motor_factor);  //+ (u >= 0 ? 0.3 : -0.3);
    hardware::SetMotor(false, uf);
    if (i % 10 == 0) {
      LOG_INF("e %6f u %6f uf %6f", -euler[0], u, uf);
      gpio_pin_toggle_dt(&hardware::run_led);
      gain = pid.ReadGain();
      sprintf(telemetry, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
              (float)k_uptime_get() / 1000.0F, -euler[0], uf, std::get<0>(gain),
              std::get<1>(gain), std::get<2>(gain));
      hardware::SerialTx((uint8_t*)telemetry, strlen(telemetry));
      // LOG_INF("q %f %f %f %f", quad[0], quad[1], quad[2], quad[3]);
    }
  }
}