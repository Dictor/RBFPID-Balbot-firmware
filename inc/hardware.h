#ifndef RBFPIDBALBOT_HARDWARE
#define RBFPIDBALBOT_HARDWARE

/* zephyr device and driver */
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>

/* std headers */
#include <array>

namespace RbfpidBalbot {
namespace hardware {

extern const struct gpio_dt_spec run_led, err_led, m_off, m_fault, m_mode;
extern const struct pwm_dt_spec m_in1, m_in2, m_in3, m_in4;
extern const struct device *imu;

int CheckHardware();
int InitHardware();
int ReadIMU(std::array<double, 3> &accel, std::array<double, 3> &gyro, std::array<double, 3> &magn);
int SetMotor(bool off, double percentile);

};  // namespace hardware
};  // namespace RbfpidBalbot

#endif