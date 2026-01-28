#pragma once

#include <Eigen/Dense>
#include <vector>
#include <cstddef>

namespace legged {

/**
 * Per-joint Kalman filter to estimate joint acceleration qdd from (q, qd).
 *
 * State: x = [q, qd, qdd]^T
 * Meas : y = [q_meas, qd_meas]^T
 *
 * Model: constant acceleration + white jerk process noise.
 */
class JointKalmanFilter {
 public:
  struct Params {
    // Measurement noise variances
    double R_q  = 1e-5;   // rad^2
    double R_qd = 1e-2;   // (rad/s)^2

    // Process noise: jerk std (rad/s^3)
    double sigma_jerk = 25.0;

    // Initial covariance (diagonal)
    double P0_q   = 1e-5;
    double P0_qd  = 1e-3;
    double P0_qdd = 1e+1;
  };

  JointKalmanFilter() = default;

  void init(std::size_t nJoints, double dt);

  void setParams(const Params& params);
  void setDt(double dt);

  // reset
  void resetJoint(std::size_t idx, double q, double qd, double qdd = 0.0);
  void resetAll(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, double qdd0 = 0.0);

  // update (no gate)
  void update(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, double dt_override = -1.0);

  // outputs
  Eigen::VectorXd getQ() const;
  Eigen::VectorXd getQd() const;
  Eigen::VectorXd getQdd() const;
  const std::vector<Eigen::Vector3d>& getStates() const { return x_; }

  double getDt() const { return dt_; }
  std::size_t size() const { return x_.size(); }

 private:
  void rebuildModel_();
  Eigen::Matrix3d computeQfromJerk_(double dt, double sigma_jerk) const;

  double dt_ = 0.001;
  Params params_;

  // Shared matrices (same for all joints)
  Eigen::Matrix3d F_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix<double, 2, 3> H_;
  Eigen::Matrix2d R_ = Eigen::Matrix2d::Identity();
  Eigen::Matrix3d Q_ = Eigen::Matrix3d::Zero();

  // Per joint state/cov
  std::vector<Eigen::Vector3d> x_;
  std::vector<Eigen::Matrix3d> P_;
};

}  // namespace legged
