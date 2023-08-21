#ifndef RBFPIDBALBOT_RBFPID
#define RBFPIDBALBOT_RBFPID

#include "../Eigen/Dense"

#include <string>
#include <iostream>

namespace RbfpidBalbot {
namespace control {
class RBFPID {
 private:
  /* PID controller parameters */
  double ek_, ek1_, ek2_;  // e(k), e(k-1), e(k-2)
  double sumek_;
  double xc1_, xc2_, xc3_;  // xc(1), xc(2), xc(3)
  double u_, du_, u_limit_;
  double kp_, ki_, kd_;
  

  /* RBF NN parameters */
  const int n_, m_;
  Eigen::MatrixXd X_;               // input X [n x 1]
  Eigen::MatrixXd Ck_, Ck1_, Ck2_;  // center vector C [m x n]
  Eigen::MatrixXd H_;               // radial vector [m x 1]
  Eigen::MatrixXd Bk_, Bk1_, Bk2_;  // radial width vector B [m x 1]
  Eigen::MatrixXd Wk_, Wk1_, Wk2_;  // weight vector W [m x 1]
  double yk_, yk1_;                 // output of plant
  double ym_;                       // output of RBF NN
  double J_;                        // jacobian information of RBF NN
  double lr_, mg_, gain_mg_;        // learning rate and momentum factor
 public:
  /**
   * @brief Construct a new RBFPID object
   *
   * @param n size of input
   * @param m size of hidden
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
  RBFPID(int n, int m, double gain_mg, double init_kp, double init_ki,
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

  std::string ToString() {
       std::ostringstream ss;
       ss << "kp " << kp_ << " ki " << ki_ << " kd " << kd_;
       return ss.str();
  }
};
};  // namespace control
};  // namespace RbfpidBalbot

#endif