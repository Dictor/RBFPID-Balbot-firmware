#include "../inc/hardware.h"

#include <errno.h>
#include <stdint.h>
#include <zephyr/logging/log.h>

#include <cmath>
#include <vector>

LOG_MODULE_REGISTER(hardware);

using namespace RbfpidBalbot;

const struct gpio_dt_spec hardware::run_led =
    GPIO_DT_SPEC_GET(DT_NODELABEL(run_led), gpios);
const struct gpio_dt_spec hardware::err_led =
    GPIO_DT_SPEC_GET(DT_NODELABEL(err_led), gpios);

const struct gpio_dt_spec hardware::m_off =
    GPIO_DT_SPEC_GET(DT_NODELABEL(m_off), gpios);
const struct gpio_dt_spec hardware::m_fault =
    GPIO_DT_SPEC_GET(DT_NODELABEL(m_fault), gpios);
const struct gpio_dt_spec hardware::m_mode =
    GPIO_DT_SPEC_GET(DT_NODELABEL(m_mode), gpios);

const struct pwm_dt_spec hardware::m_in1 = PWM_DT_SPEC_GET(DT_NODELABEL(m_in1));
const struct pwm_dt_spec hardware::m_in2 = PWM_DT_SPEC_GET(DT_NODELABEL(m_in2));
const struct pwm_dt_spec hardware::m_in3 = PWM_DT_SPEC_GET(DT_NODELABEL(m_in3));
const struct pwm_dt_spec hardware::m_in4 = PWM_DT_SPEC_GET(DT_NODELABEL(m_in4));

const struct device *hardware::imu = DEVICE_DT_GET_ONE(invensense_mpu9250);

int hardware::CheckHardware() {
  /*
  check_list is pointer array of device struct and kChecklistSize define its
  size. CPP container is intentionally hestatied because it's system initialize
  code
  */
  std::vector<const device *> check_list = {
      run_led.port, err_led.port, imu,      m_off.port,
      m_fault.port, m_in1.dev,    m_in2.dev};

  for (const auto l : check_list) {
    if (l == NULL) return -EINVAL;
    if (!device_is_ready(l)) return -ENODEV;
  }
  return 0;
}

int hardware::InitHardware() {
  gpio_pin_configure_dt(&hardware::run_led, GPIO_OUTPUT);
  gpio_pin_configure_dt(&hardware::err_led, GPIO_OUTPUT);
  gpio_pin_configure_dt(&hardware::m_off, GPIO_OUTPUT);
  gpio_pin_configure_dt(&hardware::m_fault, GPIO_INPUT);
  gpio_pin_set_dt(&hardware::m_mode, 1);  // m_mode is active low

  return 0;
}

int hardware::ReadIMU(std::array<double, 3> &accel, std::array<double, 3> &gyro,
                      std::array<double, 3> &magn) {
  struct sensor_value tmp_a[3], tmp_g[3], tmp_m[3];
  int rc = sensor_sample_fetch(hardware::imu);

  if (rc == 0) {
    rc = sensor_channel_get(hardware::imu, SENSOR_CHAN_ACCEL_XYZ, tmp_a);
  }
  if (rc == 0) {
    rc = sensor_channel_get(hardware::imu, SENSOR_CHAN_GYRO_XYZ, tmp_g);
  }
  if (rc == 0) {
    rc = sensor_channel_get(hardware::imu, SENSOR_CHAN_MAGN_XYZ, tmp_m);
  }
  if (rc == 0) {
    for (int i = 0; i < 3; i++) {
      accel[i] = sensor_value_to_double(&tmp_a[i]);
      gyro[i] = sensor_value_to_double(&tmp_g[i]);
      magn[i] = sensor_value_to_double(&tmp_m[i]);
    }
  } else {
    LOG_ERR("sample fetch/get failed: %d\n", rc);
  }
  return rc;
}

int hardware::SetMotor(bool off, double percentile) {
  static const double period = 1000.0;
  /* m_off is active low (nSLEEP) */
  if (off) {
    gpio_pin_set_dt(&hardware::m_off, 1);
  } else {
    uint32_t pulse = abs((uint32_t)(period * percentile));
    gpio_pin_set_dt(&hardware::m_off, 0);
    if (pulse >= (uint32_t)period) pulse = (uint32_t)period;

    /*
    IN1, 3 | IN2, 4
    0      | 0      - coast
    0      | 1      - reverse
    1      | 0      - forward
    1      | 1      - brake
    */
    pwm_set_dt(&hardware::m_in1, period, percentile <= 0 ? 0 : pulse);
    pwm_set_dt(&hardware::m_in2, period, percentile >= 0 ? 0 : pulse);
    pwm_set_dt(&hardware::m_in3, period, percentile <= 0 ? 0 : pulse);
    pwm_set_dt(&hardware::m_in4, period, percentile >= 0 ? 0 : pulse);
  }
  return 0;
}