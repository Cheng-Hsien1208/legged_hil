#pragma once

#include <Eigen/Core>
#include <array>
#include <unordered_map>
#include <memory>
#include <string>
#include <vector>

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <fstream> 
#include <ros/ros.h>

namespace legged {

  using vector_t = Eigen::VectorXd;

class LeggedFootObs {
  public:
    struct FootState {
      Eigen::Vector3d position_W = Eigen::Vector3d::Zero();      // Foot position in base frame
      Eigen::Vector3d velocity_W = Eigen::Vector3d::Zero();      // Foot linear velocity in world frame
      Eigen::Vector3d contactForce_W = Eigen::Vector3d::Zero();  // Estimated contact force in world frame
      double contactForceNorm_z = 0.0;                           // Norm of contact force z component
      double F_ratio = 0.0;                                      // Normalized contact force ratio (0.0~1.0)
      double F_ratio_delta = 0.0;                                // Delta of F_ratio (LPF-based)
      Eigen::Vector3d contactForce_test = Eigen::Vector3d::Zero(); // For testing
    };

    LeggedFootObs(const std::string& urdfFile, const std::vector<std::string>& jointNames,
                  const ocs2::CentroidalModelInfo& centroidalInfo, const std::vector<std::string>& footNames,
                  const std::string& csvRootDir = "");

    void updateJointStates(const vector_t& qJoints, const vector_t& vJoints, const vector_t& aJoints
      , const vector_t& tauJoints, const vector_t& tauCmdJoints, const Eigen::Vector3d& linearAccel) {
      qJoints_ = qJoints;
      vJoints_ = vJoints;
      aJoints_ = aJoints;
      tauJoints_ = tauJoints;
      tauCmdJoints_ = tauCmdJoints;
      linearAccel_ = linearAccel;
    }

    void updateMeasuredRbdState(const vector_t& measuredRbdState) { measuredRbdState_ = measuredRbdState; }

    void updateContactGt(const std::array<int, 4>& contactGt) { contactGt_ = contactGt; }

    void updateTime(double timeSec) { timeSec_ = timeSec; }

    void update();

    const std::unordered_map<std::string, FootState>& getFootStates() const { return footStates_; }


  public:
    std::vector<std::string> jointNames_;
    std::vector<std::string> footNames_;
    std::unordered_map<std::string, FootState> footStates_;


  private:
    void buildPinocchioQv_(const ocs2::CentroidalModelInfo& info, vector_t& qPinocchio, vector_t& vPinocchio, vector_t& aPinocchio);

    std::string urdfFile_;

    ocs2::CentroidalModelPinocchioMapping mapping_;

    std::unique_ptr<ocs2::PinocchioInterface> pinocchioInterfacePtr_;
    std::shared_ptr<ocs2::PinocchioEndEffectorKinematics> eeKinematicsPtr_;

    vector_t qJoints_;
    vector_t vJoints_;
    vector_t aJoints_;
    vector_t tauJoints_;
    vector_t tauCmdJoints_;
    Eigen::Vector3d linearAccel_ = Eigen::Vector3d::Zero();
    vector_t measuredRbdState_;

    double timeSec_ = 0.0;
    double prevTimeSec_ = -1.0;

    // For delta feature: LPF per-leg of F_ratio
    std::array<double, 4> fRatioLP_{{0.0, 0.0, 0.0, 0.0}};
    bool fRatioLPInit_ = false;
    double fRatioLpTau_ = 0.005;
        
    // Hampel filter (outlier removal) for joint torque
    static constexpr int kHampelWindow = 10;  // window size
    static constexpr double kHampelK = 1.0;  // threshold multiplier

    // per-joint ring buffer for tau
    std::vector<std::array<double, kHampelWindow>> tauWindow_;
    std::vector<int> tauWindowCount_;
    std::vector<int> tauWindowHead_;

    void initHampelBuffers_(int nJoints);
    double hampelFilter_(int jointIdx, double x);
    static double median_(std::array<double, kHampelWindow> a, int count);

    // ===== CSV logging =====
    void initCsvIfNeeded_();
    void logFeatures_();

    std::string csvPath_;
    std::ofstream csv_;
    bool csvHeaderWritten_ = false;

    std::array<int, 4> contactGt_{{0, 0, 0, 0}};
};

}  // namespace legged
