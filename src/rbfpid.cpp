#include "../inc/rbfpid.h"

#include <zephyr/logging/log.h>
#include <zephyr/zephyr.h>

#include <cmath>

LOG_MODULE_REGISTER(control);

using namespace RbfpidBalbot::control;
using namespace Eigen;

RBFPID::RBFPID(int n, int m, double gain_mg, double init_kp, double init_ki,
               double init_kd, double u_limit, double init_rbf_width,
               double init_rbf_weight, double mg, double lr)
    : n_(n),
      m_(m),
      ek1_(0.0),
      ek2_(0.0),
      sumek_(0.0),
      u_(0.0),
      du_(0.0),
      kp_(init_kp),
      ki_(init_ki),
      kd_(init_kd),
      X_(1, n),
      Ck_(m, n),
      Ck1_(m, n),
      Ck2_(m, n),
      H_(m, 1),
      Bk_(m, 1),
      Bk1_(m, 1),
      Bk2_(m, 1),
      Wk_(m, 1),
      Wk1_(m, 1),
      Wk2_(m, 1),
      yk1_(0),
      mg_(mg),
      lr_(lr),
      u_limit_(u_limit),
      gain_mg_(gain_mg) {
  X_ = MatrixXd::Zero(1, n);
  Ck_ = MatrixXd::Zero(m, n);
  Ck1_ = MatrixXd::Zero(m, n);
  Ck2_ = MatrixXd::Zero(m, n);
  H_ = MatrixXd::Zero(m, 1);
  Bk_ = init_rbf_width * MatrixXd::Ones(m, 1) + MatrixXd::Random(m, 1);
  Bk1_ = Bk_;
  Bk2_ = Bk_;
  Wk_ = init_rbf_weight * MatrixXd::Ones(m, 1);
  Wk1_ = Wk_;
  Wk2_ = Wk_;
}

double RBFPID::Update(double e, double y) {
  /* set input vector*/
  yk_ = y;
  X_(0) = u_;
  X_(1) = yk_;
  X_(2) = yk1_;

  /* calculate centor vector */
  for (int j = 0; j < m_; j++) {
    auto dist = (X_ - Ck_.row(j)).squaredNorm();
    H_(j) = exp(-(pow(dist, 2)) / (2 * pow(Bk_(j), 2)));
  }

  /* calculate RBF NN parameters */
  ym_ = (Wk_.transpose() * H_).value();
  for (int j = 0; j < m_; j++) {
    auto dWk = mg_ * (yk_ - ym_) * H_(j);
    Wk_(j) = Wk1_(j) + dWk + lr_ * (Wk1_(j) - Wk2_(j));
    auto dBk = mg_ * (yk_ - ym_) * Wk_(j) * H_(j) *
               ((X_ - Ck_.row(j)).norm() / pow(Bk_(j), 3));
    Bk_(j) = Bk1_(j) + dBk + lr_ * (Bk1_(j) - Bk2_(j));
    for (int i = 0; i < n_; i++) {
      auto dCk =
          mg_ * (yk_ - ym_) * Wk_(j) * ((X_(i) - Ck_(j, i)) / pow(Bk_(j), 2));
      Ck_(j, i) = Ck1_(j, i) + dCk + lr_ * (Ck1_(j, i) - Ck2_(j, i));
    }
  }

  /* calculate RBF NN J */
  J_ = 0;
  for (int j = 0; j < m_; j++) {
    J_ = J_ + Wk_(j) * H_(j) * ((Ck_(j, 1) - X_(1)) / pow(Bk_(j), 2));
  }

  /* ek is current error */
  ek_ = e;
  sumek_ = sumek_ + ek_;

  /* calculate controller input */
  xc1_ = ek_ - ek1_;
  xc2_ = ek_;
  xc3_ = ek_ - 2 * ek1_ + ek2_;

  /* update PID gain */
  kp_ = kp_ + gain_mg_ * mg_ * ek_ * J_ * xc1_;
  ki_ = ki_ + gain_mg_ * mg_ * ek_ * J_ * xc2_;
  kd_ = kd_ + gain_mg_ * mg_ * ek_ * J_ * xc3_;

  if (kp_ < 0) kp_ = 0;
  if (ki_ < 0) ki_ = 0;
  if (kd_ < 0) ki_ = 0;

  /* calculate pid output */
  du_ = kp_ * xc1_ + ki_ * xc2_ + kd_ * xc3_;
  u_ = u_ + du_;
  if (abs(u_) > u_limit_) {
    u_ = du_;
  }

  /* in final, proceed time k, shift */
  ek1_ = ek_;
  ek2_ = ek1_;
  yk1_ = yk_;
  Ck1_ = Ck_;
  Ck2_ = Ck1_;
  Bk1_ = Bk_;
  Bk2_ = Bk1_;
  Wk1_ = Wk_;
  Wk2_ = Wk1_;

  return u_;
}

std::tuple<double, double, double> RBFPID::ReadGain() {
  return std::make_tuple(kp_, ki_, kd_);
}