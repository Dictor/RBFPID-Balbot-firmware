#ifndef RBFPIDBALBOT_HARDWARE
#define RBFPIDBALBOT_HARDWARE

/* zephyr device and driver */
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

namespace RbfpidBalbot {
namespace hardware {

extern const struct gpio_dt_spec run_led, err_led;
extern const struct device *imu;

int CheckHardware();
int InitHardware();

};  // namespace hardware
};  // namespace RbfpidBalbot

#endif