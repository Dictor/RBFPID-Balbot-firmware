#include "../inc/hardware.h"

#include <errno.h>
#include <zephyr/logging/log.h>

#include <vector>

LOG_MODULE_REGISTER(hardware);

using namespace RbfpidBalbot;

const struct gpio_dt_spec hardware::run_led =
    GPIO_DT_SPEC_GET(DT_NODELABEL(run_led), gpios);
const struct gpio_dt_spec hardware::err_led =
    GPIO_DT_SPEC_GET(DT_NODELABEL(err_led), gpios);
const struct device* hardware::imu = DEVICE_DT_GET_ONE(invensense_mpu9250);

int hardware::CheckHardware() {
  /*
  check_list is pointer array of device struct and kChecklistSize define its
  size. CPP container is intentionally hestatied because it's system initialize
  code
  */
  std::vector<const device*> check_list = {run_led.port, err_led.port, imu};

  for (const auto l : check_list) {
    if (l == NULL) return -EINVAL;
    if (!device_is_ready(l)) return -ENODEV;
  }
  return 0;
}

int hardware::InitHardware() {
  gpio_pin_configure_dt(&hardware::run_led, GPIO_OUTPUT);
  gpio_pin_configure_dt(&hardware::err_led, GPIO_OUTPUT);

  return 0;
}