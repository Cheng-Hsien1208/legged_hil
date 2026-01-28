#include "JointKalmanFilter.h"

#include <stdexcept>
#include <algorithm>
#include <cmath>

namespace legged {

void JointKalmanFilter::init(std::size_t nJoints, double dt) {
  if (nJoints == 0) throw std::runtime_error("JointKalmanFilter::init(): nJoints must be > 0");
  if (!(dt > 0.0)) throw std::runtime_error("JointKalmanFilter::init(): dt must be > 0");

  dt_ = dt;
  params_ = Params();

  x_.assign(nJoints, Eigen::Vector3d::Zero());
  P_.assign(nJoints, Eigen::Matrix3d::Zero());

  // H maps state -> measurement: y = [q, qd]
  H_.setZero();
  H_(0, 0) = 1.0;
  H_(1, 1) = 1.0;

  rebuildModel_();

  // default covariance
  for (std::size_t i = 0; i < nJoints; ++i) {
    P_[i].setZero();
    P_[i](0, 0) = std::max(params_.P0_q,   1e-16);
    P_[i](1, 1) = std::max(params_.P0_qd,  1e-16);
    P_[i](2, 2) = std::max(params_.P0_qdd, 1e-16);
  }
}

void JointKalmanFilter::setParams(const Params& params) {
  params_ = params;
  rebuildModel_();

  // Update current P diagonals only if you want (optional).
  // Here we keep existing P_ because it reflects current confidence.
}

void JointKalmanFilter::setDt(double dt) {
  if (!(dt > 0.0)) return;
  dt_ = dt;
  rebuildModel_();
}

void JointKalmanFilter::rebuildModel_() {
  const double dt = dt_;

  // F: constant acceleration
  F_.setIdentity();
  F_(0, 1) = dt;
  F_(0, 2) = 0.5 * dt * dt;
  F_(1, 2) = dt;

  // R: measurement noise
  R_.setZero();
  R_(0, 0) = std::max(params_.R_q,  1e-16);
  R_(1, 1) = std::max(params_.R_qd, 1e-16);

  // Q: jerk-driven process noise
  Q_ = computeQfromJerk_(dt, std::max(params_.sigma_jerk, 1e-12));
}

Eigen::Matrix3d JointKalmanFilter::computeQfromJerk_(double dt, double sigma_jerk) const {
  // jerk j ~ N(0, sigma_j^2), with state [q, qd, qdd]
  // Q = sigma_j^2 * [[dt^5/20, dt^4/8,  dt^3/6],
  //                 [dt^4/8,  dt^3/3,  dt^2/2],
  //                 [dt^3/6,  dt^2/2,  dt      ]]
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt2 * dt2;
  const double dt5 = dt4 * dt;
  const double sj2 = sigma_jerk * sigma_jerk;

  Eigen::Matrix3d Q;
  Q(0, 0) = dt5 / 20.0;
  Q(0, 1) = dt4 / 8.0;
  Q(0, 2) = dt3 / 6.0;

  Q(1, 0) = Q(0, 1);
  Q(1, 1) = dt3 / 3.0;
  Q(1, 2) = dt2 / 2.0;

  Q(2, 0) = Q(0, 2);
  Q(2, 1) = Q(1, 2);
  Q(2, 2) = dt;

  return sj2 * Q;
}

void JointKalmanFilter::resetJoint(std::size_t idx, double q, double qd, double qdd) {
  if (idx >= x_.size()) return;

  x_[idx] << q, qd, qdd;

  P_[idx].setZero();
  P_[idx](0, 0) = std::max(params_.P0_q,   1e-16);
  P_[idx](1, 1) = std::max(params_.P0_qd,  1e-16);
  P_[idx](2, 2) = std::max(params_.P0_qdd, 1e-16);
}

void JointKalmanFilter::resetAll(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, double qdd0) {
  const std::size_t n = x_.size();
  if (static_cast<std::size_t>(q.size()) != n || static_cast<std::size_t>(qd.size()) != n) return;

  for (std::size_t i = 0; i < n; ++i) {
    resetJoint(i, q(static_cast<int>(i)), qd(static_cast<int>(i)), qdd0);
  }
}

void JointKalmanFilter::update(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, double dt_override) {
  const std::size_t n = x_.size();
  if (n == 0) return;

  if (static_cast<std::size_t>(q.size()) != n || static_cast<std::size_t>(qd.size()) != n) {
    throw std::runtime_error("JointKalmanFilter::update(): input size mismatch");
  }

  if (dt_override > 0.0 && std::abs(dt_override - dt_) > 1e-12) {
    dt_ = dt_override;
    rebuildModel_();
  }

  for (std::size_t i = 0; i < n; ++i) {
    // -------- predict --------
    const Eigen::Vector3d x_pred = F_ * x_[i];
    const Eigen::Matrix3d P_pred = F_ * P_[i] * F_.transpose() + Q_;

    // -------- innovation --------
    Eigen::Matrix<double, 2, 1> y;
    y << q(static_cast<int>(i)), qd(static_cast<int>(i));
    const Eigen::Matrix<double, 2, 1> innov = y - (H_ * x_pred);

    // -------- update --------
    Eigen::Matrix2d S = H_ * P_pred * H_.transpose() + R_;

    // If near singular, skip update (rare)
    const double det = S.determinant();
    if (std::abs(det) < 1e-18) {
      x_[i] = x_pred;
      P_[i] = P_pred;
      continue;
    }

    const Eigen::Matrix2d S_inv = S.inverse();
    const Eigen::Matrix<double, 3, 2> K = P_pred * H_.transpose() * S_inv;

    x_[i] = x_pred + K * innov;
    P_[i] = (Eigen::Matrix3d::Identity() - K * H_) * P_pred;
  }
}

Eigen::VectorXd JointKalmanFilter::getQ() const {
  Eigen::VectorXd q(static_cast<int>(x_.size()));
  for (std::size_t i = 0; i < x_.size(); ++i) {
    q(static_cast<int>(i)) = x_[i](0);   // q
  }
  return q;
}

Eigen::VectorXd JointKalmanFilter::getQd() const {
  Eigen::VectorXd qd(static_cast<int>(x_.size()));
  for (std::size_t i = 0; i < x_.size(); ++i) {
    qd(static_cast<int>(i)) = x_[i](1);  // qd
  }
  return qd;
}

Eigen::VectorXd JointKalmanFilter::getQdd() const {
  Eigen::VectorXd qdd(static_cast<int>(x_.size()));
  for (std::size_t i = 0; i < x_.size(); ++i) {
    qdd(static_cast<int>(i)) = x_[i](2); // qdd
  }
  return qdd;
}


}  // namespace legged
