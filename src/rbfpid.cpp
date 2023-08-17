#include "../inc/rbfpid.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(control);

using namespace RbfpidBalbot::control;
using namespace Eigen;

RBFPID::RBFPID(int n, int m, double gain_mg, double init_kp, double init_ki,
               double init_kd, double u_limit, double init_rbf_width,
               double init_rbf_weight, double mg, double l)
    : ek1_(0.0),
      ek2_(0.0),
      sumek_(0.0),
      u_(0.0),
      du_(0.0),
      kp_(init_kp),
      ki_(init_ki),
      kd_(init_kd),
      X_(n, 1),
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
      yk1_(0) {
        X_ = MatrixXd::Zero(m, 1);
        Ck_ = MatrixXd::Zero(m, n);
        Ck1_ = MatrixXd::Zero(m, n);
        Ck2_ = MatrixXd::Zero(m, n);
        H_ = MatrixXd::Zero(m, 1);
      }

double RBFPID::Update(double e, double y) {}