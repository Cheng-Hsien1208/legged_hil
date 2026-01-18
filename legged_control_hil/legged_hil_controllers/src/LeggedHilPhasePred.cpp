#include "legged_hil_controllers/LeggedHilPhasePred.h"

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

#include <iostream>
#include <cmath>
#include <algorithm>

namespace legged {

LeggedHilPhasePred::LeggedHilPhasePred(
    const ocs2::PinocchioInterface& pinocchioInterface,
    const std::vector<ImpedanceJointHandle>& impedanceJointHandles,
    std::vector<std::string> jointNames,
    std::vector<std::string> contactNames3DoF)
    : pinocchioInterface_(pinocchioInterface),
      impedanceJointHandles_(impedanceJointHandles),
      jointNames_(jointNames),
      contactNames3DoF_(contactNames3DoF) {}

void LeggedHilPhasePred::initialize() {
    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();

    for (size_t i = 0; i < model.joints.size(); ++i) {
        jointNames_to_idx_[model.names[i]] = model.joints[i].idx_v();
    }

    forceModels_[0] = {350.0, 170.0, 15.0, 50.0, 50.0, 50.0};  // LF
    forceModels_[1] = {350.0, 170.0, 15.0, 50.0, 50.0, 50.0};  // RF
    forceModels_[2] = {350.0, 170.0, 15.0, 50.0, 50.0, 50.0};  // LH
    forceModels_[3] = {350.0, 170.0, 15.0, 50.0, 50.0, 50.0};  // RH

    for (int i = 0; i < 4; i++) {
        prior_2_legs_stance_[i] = 0.6;
        prior_4_legs_stance_[i] = 0.2;
        prior_swing_[i] = 0.2;
        isStance_[i] = true;
        contactForces_[i] = 0.0;
    }

    initialized_ = true;
}

double LeggedHilPhasePred::normpdf(double x, double mu, double sigma) {
    constexpr double PI = 3.14159265358979323846;

    double diff = x - mu;
    double exponent = -(diff * diff) / (2.0 * sigma * sigma);
    double val = (1.0 / (std::sqrt(2.0 * PI) * sigma)) * std::exp(exponent);
    return std::max(val, 1e-12); 
}

double LeggedHilPhasePred::estimateFootContactForce(const Eigen::Vector3d& jointTorque, const Eigen::Matrix3d& J) {
    Eigen::Vector3d contactForce = J.transpose().inverse() * jointTorque;
    // return contactForce.norm();
    return std::abs(contactForce.z());
}

void LeggedHilPhasePred::decidePhase(int leg, double contact_force) {
    const auto& model = forceModels_[leg];

    double L_2_legs_stance = normpdf(contact_force, model.mu_2_legs_stance, model.sigma_2_legs_stance);
    double L_4_legs_stance = normpdf(contact_force, model.mu_4_legs_stance, model.sigma_4_legs_stance);
    double L_swing  = normpdf(contact_force, model.mu_swing, model.sigma_swing);

    double post_2_legs_stance, post_4_legs_stance, post_swing; 

    // 乘上 prior 再 normalize
    double numerator_2_legs_stance = L_2_legs_stance * prior_2_legs_stance_[leg];
    double numerator_4_legs_stance = L_4_legs_stance * prior_4_legs_stance_[leg];
    double numerator_swing  = L_swing  * prior_swing_[leg];
    double denom = numerator_2_legs_stance + numerator_4_legs_stance + numerator_swing;
    

    if (denom < 1e-12) {
        // post_stance = prior_stance_[leg];
        post_2_legs_stance = prior_2_legs_stance_[leg];
        post_4_legs_stance = prior_4_legs_stance_[leg];
        post_swing  = prior_swing_[leg];
        std::cerr << "Denominator too small, using prior for leg " << leg << std::endl;
    } else {
        // post_stance = numerator_stance / denom;
        post_2_legs_stance = numerator_2_legs_stance / denom;
        post_4_legs_stance = numerator_4_legs_stance / denom;
        post_swing  = numerator_swing / denom;

        // clamp
        // post_stance = std::max(1e-6, std::min(post_stance, 1.0 - 1e-6));
        post_2_legs_stance = std::max(1e-6, std::min(post_2_legs_stance, 1.0 - 1e-6));
        post_4_legs_stance = std::max(1e-6, std::min(post_4_legs_stance, 1.0 - 1e-6));
        post_swing  = std::max(1e-6, std::min(post_swing, 1.0 - 1e-6));
    }

    // 更新 prior
    // prior_stance_[leg] = post_stance;
    prior_2_legs_stance_[leg] = post_2_legs_stance;
    prior_4_legs_stance_[leg] = post_4_legs_stance;
    prior_swing_[leg] = post_swing;

    // 判斷 stance/swing 狀態
    // isStance_[leg] = (prior_stance_[leg] > prior_swing_[leg]);
    if (post_2_legs_stance > post_swing || post_4_legs_stance > post_swing) {
        isStance_[leg] = true;
    } else {
        isStance_[leg] = false;
    }
}

void LeggedHilPhasePred::update() {
    if (!initialized_) return;

    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();

    // 初始化 joint state vectors
    vector_t q = vector_t::Zero(model.nq);
    vector_t v = vector_t::Zero(model.nv);
    vector_t effort = vector_t::Zero(model.nv);

    q.head<6>() << 0, 0, 0, 0, 0, 0;
    v.head<6>().setZero();

    // 填入 ROS handle 中的 12 軸 joint 值
    for (const auto& handle : impedanceJointHandles_) {
        auto it = jointNames_to_idx_.find(handle.getName());
        if (it == jointNames_to_idx_.end()) {
            ROS_WARN_STREAM("Joint " << handle.getName() << " not found in Pinocchio model.");
            continue;
        }

        size_t idx_v = it->second;

        effort(idx_v) = handle.getEffort();
        v(idx_v) = handle.getVelocity();
        q(idx_v) = handle.getPosition();
    }

    // 做 forward kinematics 和 jacobian 準備
    pinocchio::forwardKinematics(model, data, q, v);
    pinocchio::computeJointJacobians(model, data);
    pinocchio::updateFramePlacements(model, data);

    // 對每條腿計算接觸力
    for (int leg = 0; leg < 4; ++leg) {
        auto HAAJointNames = jointNames_[leg * 3];
        auto HFEJointNames = jointNames_[leg * 3 + 1];
        auto KFEJointNames = jointNames_[leg * 3 + 2];
        const std::string& footName = contactNames3DoF_[leg];
        pinocchio::FrameIndex frameId = model.getFrameId(footName);
        pinocchio::FrameIndex rootId = model.getFrameId("base");

        Eigen::MatrixXd J_full(6, model.nv);
        pinocchio::getFrameJacobian(model, data, frameId, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_full);

        // Jacobian
        Eigen::Matrix3d J_leg;
        J_leg.col(0) = J_full.block<3,1>(0, model.joints[model.getJointId(HAAJointNames)].idx_v());
        J_leg.col(1) = J_full.block<3,1>(0, model.joints[model.getJointId(HFEJointNames)].idx_v());
        J_leg.col(2) = J_full.block<3,1>(0, model.joints[model.getJointId(KFEJointNames)].idx_v());
        
        // Torque
        Eigen::Vector3d tau_leg;
        tau_leg(0) = effort(model.joints[model.getJointId(HAAJointNames)].idx_v());
        tau_leg(1) = effort(model.joints[model.getJointId(HFEJointNames)].idx_v());
        tau_leg(2) = effort(model.joints[model.getJointId(KFEJointNames)].idx_v());

        // Normalize the force vector
        contactForces_[leg] = estimateFootContactForce(tau_leg, J_leg);
        decidePhase(leg, contactForces_[leg]);
    }
};

}  // namespace legged
