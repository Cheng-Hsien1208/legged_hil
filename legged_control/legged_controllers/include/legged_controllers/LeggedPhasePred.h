#pragma once

#include <legged_interface/LeggedInterface.h>
#include <legged_estimation/StateEstimateBase.h>

#include <array>
#include <vector>
#include <string>
#include <Eigen/Core>

namespace legged {

class LeggedPhasePred {
public:
    LeggedPhasePred(const ocs2::PinocchioInterface& pinocchioInterface,
                const std::vector<HybridJointHandle>& hybridJointHandles,
                std::vector<std::string> jointNames,
                std::vector<std::string> contactNames3DoF);

    void initialize();
    bool isInitialized() { return initialized_; }

    void update();

    double getContactForceNorm(int leg) const { return contactForces_[leg]; }
    double getSwingProb(int leg) const { return prior_swing_[leg]; }
    bool getLegPhase(int leg) const { return isStance_[leg]; }

private:
    double normpdf(double x, double mu, double sigma);
    double estimateFootContactForce(const Eigen::Vector3d& jointTorque, const Eigen::Matrix3d& Jt);
    void decidePhase(int leg, double contact_force);

    // Pinocchio
    ocs2::PinocchioInterface pinocchioInterface_;
    std::vector<HybridJointHandle> hybridJointHandles_;

    // Joint name to index mapping
    std::vector<std::string> jointNames_;
    std::vector<std::string> contactNames3DoF_;
    std::map<std::string, size_t> jointNames_to_idx_;

    bool initialized_ = false;

    // output 
    std::array<bool, 4> isStance_;
    std::array<double, 4> contactForces_;

    // filter state
    // std::array<double, 4> prior_stance_;
    std::array<double, 4> prior_2_legs_stance_;
    std::array<double, 4> prior_4_legs_stance_;
    std::array<double, 4> prior_swing_;

    // Gaussian probability model parameters
    struct LegForceModel {
        double mu_2_legs_stance;
        double mu_4_legs_stance;
        double mu_swing;
        double sigma_2_legs_stance;
        double sigma_4_legs_stance;
        double sigma_swing;
    };
    std::array<LegForceModel, 4> forceModels_;
};

}  // namespace legged
