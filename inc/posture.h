#ifndef RBFPIDBALBOT_POSTURE
#define RBFPIDBALBOT_POSTURE

#include <array>

//=====================================================================================================
// MahonyAHRS is from below with GPLv2
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================

using namespace std;

namespace RbfpidBalbot {
namespace posture {
class MahonyAHRS {
 private:
  float dt_;
  float twoKp_, twoKi_;  // 2 * P, I gain
  array<float, 4> q_;  // quaternion of sensor frame relative to auxiliary frame
  array<float, 3> integralFB_;  // integral error terms scaled by Ki
  static float invSqrt(float x);

 public:
  MahonyAHRS(float dt, float Kp = 0.5, float Ki = 0.0);
  void Update(float gx, float gy, float gz, float ax, float ay, float az);
  void Update(float gx, float gy, float gz, float ax, float ay, float az,
              float mx, float my, float mz);
  array<float, 4> GetQuaternion();
};
};  // namespace posture
};  // namespace RbfpidBalbot

#endif