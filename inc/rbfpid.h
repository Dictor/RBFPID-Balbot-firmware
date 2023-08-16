#ifndef RBFPIDBALBOT_RBFPID
#define RBFPIDBALBOT_RBFPID

#include "../Eigen/Dense"

namespace RbfpidBalbot {
namespace control {
class RBFPID {
 private:
 public:
  /**
   * @brief Construct a new RBFPID object
   *
   * @param mg momentum factor
   * @param gain_mg momentum factor for only PID gain changing
   * @param lr learning rate
   * @param init_kp initial P gain
   * @param init_ki initial I gain
   * @param init_kd initial D gain
   * @param u_limit reset threshold of abs(output)
   * @param init_rbf_width initial rbf nn width
   * @param init_rbf_weight initial rbf nn weight
   */
  RBFPID(double gain_mg, double init_kp, double init_ki,
         double init_kd, double u_limit, double init_rbf_width = 3.0,
         double init_rbf_weight = 0.01, double mg = 0.25, double lr = 0.05);
  /**
   * @brief Update controller
   * 
   * @param e error (input of controller)
   * @param y result (output of plant)
   * @return double output (input of plant)
   */
  double Update(double e, double y);
};
};  // namespace control
};  // namespace RbfpidBalbot

#endif