#include "../inc/app_main.h"
#include "../inc/hardware.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
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

  }
}