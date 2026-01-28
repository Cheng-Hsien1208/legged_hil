#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <legged_common/hardware_interface/ContactSensorInterface.h>
#include <legged_hil_interface/hardware_interface/ImpedanceJointInterface.h>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>

#include <legged_estimation/StateEstimateBase.h>
#include <legged_interface/LeggedInterface.h>
#include <legged_wbc/WbcBase.h>

#include "legged_controllers/SafetyChecker.h"
#include "legged_controllers/visualization/LeggedSelfCollisionVisualization.h"

#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <legged_hil_controllers/LeggedHilPhasePred.h>

#include <legged_hil_controllers/LeggedFootObs.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <legged_hil_controllers/LeggedFootContactEst.h>

#include <legged_hil_controllers/JointKalmanFilter.h>

namespace legged {
using namespace ocs2;
using namespace legged_robot;

class LeggedHilController : public controller_interface::MultiInterfaceController<ImpedanceJointInterface, hardware_interface::ImuSensorInterface,
                                                                               ContactSensorInterface> {
 public:
  LeggedHilController() = default;
  ~LeggedHilController() override;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& /*time*/) override { mpcRunning_ = false; }

 protected:
  virtual void updateStateEstimation(const ros::Time& time, const ros::Duration& period);

  virtual void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                    bool verbose);
  virtual void setupMpc();
  virtual void setupMrt();
  virtual void setupStateEstimate(const std::string& taskFile, bool verbose);

  // Interface
  std::shared_ptr<LeggedInterface> leggedInterface_;
  std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;
  std::vector<ImpedanceJointHandle> impedanceJointHandles_;
  std::vector<ContactSensorHandle> contactHandles_;
  hardware_interface::ImuSensorHandle imuSensorHandle_;

  // State Estimation
  SystemObservation currentObservation_;
  vector_t measuredRbdState_;
  std::shared_ptr<StateEstimateBase> stateEstimate_;
  std::shared_ptr<CentroidalModelRbdConversions> rbdConversions_;

  // Whole Body Control
  std::shared_ptr<WbcBase> wbc_;
  std::shared_ptr<SafetyChecker> safetyChecker_;

  // Nonlinear MPC
  std::shared_ptr<MPC_BASE> mpc_;
  std::shared_ptr<MPC_MRT_Interface> mpcMrtInterface_;

  // Visualization
  std::shared_ptr<LeggedRobotVisualizer> robotVisualizer_;
  std::shared_ptr<LeggedSelfCollisionVisualization> selfCollisionVisualization_;
  ros::Publisher observationPublisher_;

  // CPU Affinity
  std::string mpcCpuList_;
  bool parseCpuList(const std::string& cpuListStr, cpu_set_t& cpuset);
  bool setThreadAffinity(std::thread& th, const cpu_set_t& cpuset);

  // Gains
  double kp_{0.0};
  double kd_{0.0};

  // Contact 
  contact_flag_t contactFlagGt = {false, false, false, false};
  contact_flag_t contactFlagEstOld = {false, false, false, false};
  contact_flag_t contactFlagEstNew = {false, false, false, false};
  int count = 0;

  // Contact flags Gt
  std::vector<std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Bool>>> footPhaseGtPublishers_;

  // Legged Phase Prediction
  std::shared_ptr<LeggedHilPhasePred> LeggedHilPhasePred_;
  std::vector<std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Bool>>> footPhaseEstPublishers_;


  // Endeffector interfaces
  std::shared_ptr<LeggedFootObs> LeggedFootObs_;
  std::vector<std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Vector3Stamped>>> footPosPublishers_;
  std::vector<std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Vector3Stamped>>> footVelPublishers_;
  std::vector<std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Vector3Stamped>>> footForcePublishers_;
  std::vector<std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64>>> footFratioPublishers_;
  std::vector<std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64>>> footFratioDeltaPublishers_;
  vector_t tau_prev_;
  std::vector<std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Vector3Stamped>>> footForceTestPublishers_;

  // Foot Contact Estimation
  std::shared_ptr<LeggedFootContactEst> footContactEst_;
  std::vector<std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Bool>>> footContactEstPublishers_;
  std::vector<std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64>>> footContactLogOddsPublishers_;
  std::vector<std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64>>> footContactCurrentLogisticPublishers_;
  std::vector<std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64>>> footContactProbStancePublishers_;

  // Joint Kalman Filter for acceleration estimation
  std::shared_ptr<JointKalmanFilter> jointKalmanFilter_;
  std::vector<std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64>>> jointPosEstPublishers_;
  std::vector<std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64>>> jointVelEstPublishers_;
  std::vector<std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64>>> jointAccelEstPublishers_;

  // Joint Ground Truth
  std::vector<std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64>>> jointPosGtPublishers_;
  std::vector<std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64>>> jointVelGtPublishers_;

 private:
  std::thread mpcThread_;
  std::atomic_bool controllerRunning_{}, mpcRunning_{};
  benchmark::RepeatedTimer mpcTimer_;
  benchmark::RepeatedTimer wbcTimer_;

};
}  // namespace legged
