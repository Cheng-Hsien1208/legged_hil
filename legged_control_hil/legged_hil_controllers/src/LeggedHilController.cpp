#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "LeggedHilController.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>

#include <angles/angles.h>
#include <legged_estimation/FromTopiceEstimate.h>
#include <legged_estimation/LinearKalmanFilter.h>
#include <legged_wbc/HierarchicalWbc.h>
#include <legged_wbc/WeightedWbc.h>
#include <pluginlib/class_list_macros.hpp>

namespace legged {
bool LeggedHilController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) {
  // Initialize OCS2
  std::string urdfFile;
  std::string taskFile;
  std::string referenceFile;
  controller_nh.getParam("/urdfFile", urdfFile);
  controller_nh.getParam("/taskFile", taskFile);
  controller_nh.getParam("/referenceFile", referenceFile);
  controller_nh.getParam("/legged_hil_hw/mpc_cpu_list", mpcCpuList_);
  
  controller_nh.getParam("/legged_hil_hw/kp", kp_);
  controller_nh.getParam("/legged_hil_hw/kd", kd_);
  ROS_INFO_STREAM("[LeggedHilController] Using kp: " << kp_ << ", kd: " << kd_);

  bool verbose = false;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);

  setupLeggedInterface(taskFile, urdfFile, referenceFile, verbose);
  setupMpc();
  setupMrt();
  // Visualization
  ros::NodeHandle nh;
  CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo());
  eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(leggedInterface_->getPinocchioInterface(), pinocchioMapping,
                                                                      leggedInterface_->modelSettings().contactNames3DoF);
  robotVisualizer_ = std::make_shared<LeggedRobotVisualizer>(leggedInterface_->getPinocchioInterface(),
                                                             leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, nh);
  selfCollisionVisualization_.reset(new LeggedSelfCollisionVisualization(leggedInterface_->getPinocchioInterface(),
                                                                         leggedInterface_->getGeometryInterface(), pinocchioMapping, nh));

  // Hardware interface
  auto* impedanceJointInterface = robot_hw->get<ImpedanceJointInterface>();
  std::vector<std::string> joint_names{"LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
                                       "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};
  for (const auto& joint_name : joint_names) {
    impedanceJointHandles_.push_back(impedanceJointInterface->getHandle(joint_name));
  }
  auto* contactInterface = robot_hw->get<ContactSensorInterface>();
  for (const auto& name : leggedInterface_->modelSettings().contactNames3DoF) {
    contactHandles_.push_back(contactInterface->getHandle(name));
  }
  imuSensorHandle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("base_imu");

  // State estimation
  setupStateEstimate(taskFile, verbose);

  // Whole body control
  wbc_ = std::make_shared<WeightedWbc>(leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(),
                                       *eeKinematicsPtr_);
  wbc_->loadTasksSetting(taskFile, verbose);

  // Safety Checker
  safetyChecker_ = std::make_shared<SafetyChecker>(leggedInterface_->getCentroidalModelInfo());

  // Legged Phase Prediction
  LeggedHilPhasePred_ = std::make_shared<LeggedHilPhasePred>(
      leggedInterface_->getPinocchioInterface(), impedanceJointHandles_,
      leggedInterface_->modelSettings().jointNames, leggedInterface_->modelSettings().contactNames3DoF);

  footPhaseGtPublishers_.resize(4);
  footPhaseEstPublishers_.resize(4);
  std::vector<std::string> legNames = leggedInterface_->modelSettings().contactNames3DoF;
  for (size_t i = 0; i < 4; ++i) {
    footPhaseGtPublishers_[i] = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::Bool>>(nh, "/legged_robot/foot_phase_gt/" + legNames[i], 1);
    footPhaseEstPublishers_[i] = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::Bool>>(nh, "/legged_robot/foot_phase_est/" + legNames[i], 1);
  }

  // Eedeffector interfaces
  LeggedFootObs_ = std::make_shared<LeggedFootObs>(urdfFile,
                                                     leggedInterface_->modelSettings().jointNames,
                                                     leggedInterface_->getCentroidalModelInfo(),
                                                     leggedInterface_->modelSettings().contactNames3DoF,
                                                     "");

  footPosPublishers_.resize(4);
  footVelPublishers_.resize(4);
  footForcePublishers_.resize(4);
  footFratioPublishers_.resize(4);
  footFratioDeltaPublishers_.resize(4);

  for (size_t i = 0; i < 4; ++i) {
    footPosPublishers_[i] = std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::Vector3Stamped>>(
        nh, "/legged_robot/foot_obs/foot_position_W/" + legNames[i], 1);

    footVelPublishers_[i] = std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::Vector3Stamped>>(
        nh, "/legged_robot/foot_obs/foot_velocity_W/" + legNames[i], 1);

    footForcePublishers_[i] = std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::Vector3Stamped>>(
        nh, "/legged_robot/foot_obs/foot_force_W/" + legNames[i], 1);

    footFratioPublishers_[i] = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::Float64>>(
        nh, "/legged_robot/foot_obs/foot_F_ratio/" + legNames[i], 1);
    
    footFratioDeltaPublishers_[i] = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::Float64>>(
        nh, "/legged_robot/foot_obs/foot_F_ratio_delta/" + legNames[i], 1);
  }

  // Foot Contact Estimation
  LeggedFootContactEst::LogisticParams logisticParams;
  logisticParams.mu = {0.24998293573491237, 0.02435780812149187, -0.012396106084883337};
  logisticParams.sigma = {0.21544671048667338, 0.05257167677599799, 0.5992584679301705};
  logisticParams.w =  {3.2181342486259585, -1.1470960111140018, 0.14000015890435785};
  logisticParams.b = -0.4109456112539159;
  logisticParams.priorLogOdds = -0.2820386688872121;

  LeggedFootContactEst::FilterParams filterParams;
  filterParams.alpha = 0.001;
  filterParams.beta = 0.01;
  filterParams.L_min = -30.0;
  filterParams.L_max = 15.0;
  filterParams.L_enter = 10.0;
  filterParams.L_exit = -1.0;
  filterParams.N_enter = 10;
  filterParams.N_exit = 3;

  footContactEst_ =  std::make_shared<LeggedFootContactEst>(LeggedFootObs_, logisticParams, filterParams);

  footContactEstPublishers_.resize(4);
  footContactLogOddsPublishers_.resize(4);
  footContactCurrentLogisticPublishers_.resize(4);
  footContactProbStancePublishers_.resize(4);
  for (size_t i = 0; i < 4; ++i) {
    footContactEstPublishers_[i] = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::Bool>>(
        nh, "/legged_robot/foot_contact_est/foot_contact_est/" + legNames[i], 1);
    footContactLogOddsPublishers_[i] = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::Float64>>(
        nh, "/legged_robot/foot_contact_est/foot_contact_log_odds/" + legNames[i], 1);
    footContactCurrentLogisticPublishers_[i] = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::Float64>>(
        nh, "/legged_robot/foot_contact_est/foot_contact_current_logistic/" + legNames[i], 1);
    footContactProbStancePublishers_[i] = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::Float64>>(
        nh, "/legged_robot/foot_contact_est/foot_contact_prob_stance/" + legNames[i], 1);
  }

  return true;
}

void LeggedHilController::starting(const ros::Time& time) {
  // Initial state
  currentObservation_.state.setZero(leggedInterface_->getCentroidalModelInfo().stateDim);
  updateStateEstimation(time, ros::Duration(0.002));
  currentObservation_.input.setZero(leggedInterface_->getCentroidalModelInfo().inputDim);
  currentObservation_.mode = ModeNumber::STANCE;

  TargetTrajectories target_trajectories({currentObservation_.time}, {currentObservation_.state}, {currentObservation_.input});

  // Set the first observation and command and wait for optimization to finish
  mpcMrtInterface_->setCurrentObservation(currentObservation_);
  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
  ROS_INFO_STREAM("Waiting for the initial policy ...");
  while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok()) {
    mpcMrtInterface_->advanceMpc();
    ros::WallRate(leggedInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
  }
  ROS_INFO_STREAM("Initial policy has been received.");

  // Start Legged Phase Predictor
  LeggedHilPhasePred_->initialize();
  ROS_INFO_STREAM("LeggedHilPhasePred initialized.");

  // Start Foot Contact Estimator
  footContactEst_->reset(0.0);
  ROS_INFO_STREAM("LeggedFootContactEst initialized.");

  mpcRunning_ = true;
}

void LeggedHilController::update(const ros::Time& time, const ros::Duration& period) {
  wbcTimer_.startTimer();
  // State Estimate
  updateStateEstimation(time, period);

  // Update the current state of the system
  mpcMrtInterface_->setCurrentObservation(currentObservation_);

  // Load the latest MPC policy
  mpcMrtInterface_->updatePolicy();

  // Evaluate the current policy
  vector_t optimizedState, optimizedInput;
  size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
  mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode);

  // Whole body control
  currentObservation_.input = optimizedInput;

  vector_t x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode, period.toSec());

  vector_t torque = x.tail(12);

  vector_t posDes = centroidal_model::getJointAngles(optimizedState, leggedInterface_->getCentroidalModelInfo());
  vector_t velDes = centroidal_model::getJointVelocities(optimizedInput, leggedInterface_->getCentroidalModelInfo());

  // Safety check, if failed, stop the controller
  if (!safetyChecker_->check(currentObservation_, optimizedState, optimizedInput)) {
    ROS_ERROR_STREAM("[Legged Controller] Safety check failed, stopping the controller.");
    stopRequest(time);
  }

  for (size_t j = 0; j < leggedInterface_->getCentroidalModelInfo().actuatedDofNum; ++j) {
    impedanceJointHandles_[j].setCommand(
      impedanceJointHandles_[j].getStateTimestamp()[0],
      impedanceJointHandles_[j].getStateTimestamp()[1],
      time.sec, 
      time.nsec, 
      posDes(j), 
      velDes(j), 
      kp_,
      kd_,
      torque(j));
  }

  // Visualization
  robotVisualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());
  selfCollisionVisualization_->update(currentObservation_);

  // Publish the observation. Only needed for the command interface
  observationPublisher_.publish(ros_msg_conversions::createObservationMsg(currentObservation_));

  wbcTimer_.endTimer();
}

void LeggedHilController::updateStateEstimation(const ros::Time& time, const ros::Duration& period) {
  vector_t jointPos(impedanceJointHandles_.size()), jointVel(impedanceJointHandles_.size()), jointEffort(impedanceJointHandles_.size());
  int contactsSize = leggedInterface_->modelSettings().contactNames3DoF.size();
  Eigen::Quaternion<scalar_t> quat;
  vector3_t angularVel, linearAccel;
  matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;

  for (size_t i = 0; i < impedanceJointHandles_.size(); ++i) {
    jointPos(i) = impedanceJointHandles_[i].getPosition();
    jointVel(i) = impedanceJointHandles_[i].getVelocity();
    jointEffort(i) = impedanceJointHandles_[i].getEffort();
  }

  for (size_t i = 0; i < 4; ++i) {
    quat.coeffs()(i) = imuSensorHandle_.getOrientation()[i];
  }
  for (size_t i = 0; i < 3; ++i) {
    angularVel(i) = imuSensorHandle_.getAngularVelocity()[i];
    linearAccel(i) = imuSensorHandle_.getLinearAcceleration()[i];
  }
  for (size_t i = 0; i < 9; ++i) {
    orientationCovariance(i) = imuSensorHandle_.getOrientationCovariance()[i];
    angularVelCovariance(i) = imuSensorHandle_.getAngularVelocityCovariance()[i];
    linearAccelCovariance(i) = imuSensorHandle_.getLinearAccelerationCovariance()[i];
  }
  for (size_t i = 0; i < contactHandles_.size(); ++i) {
    contactFlagGt[i] = contactHandles_[i].isContact();
  }

  stateEstimate_->updateJointStates(jointPos, jointVel);
  stateEstimate_->updateContact(contactFlagEstNew);
  // stateEstimate_->updateContact(contactFlagEstOld);
  stateEstimate_->updateImu(quat, angularVel, linearAccel, orientationCovariance, angularVelCovariance, linearAccelCovariance);
  measuredRbdState_ = stateEstimate_->update(time, period);
  currentObservation_.time += period.toSec();
  scalar_t yawLast = currentObservation_.state(9);
  currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
  currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
  currentObservation_.mode = stateEstimate_->getMode();

  // Legged Phase Prediction
  LeggedHilPhasePred_->update();
  
  // Get contact sensor data
  for (size_t i = 0; i < contactsSize; ++i) {
    contactFlagEstOld[i] = LeggedHilPhasePred_->getLegPhase(i);
    if (footPhaseGtPublishers_[i]->trylock()) {
      footPhaseGtPublishers_[i]->msg_.data = contactFlagGt[i];
      footPhaseGtPublishers_[i]->unlockAndPublish();
    }

    if (footPhaseEstPublishers_[i]->trylock()) {
      footPhaseEstPublishers_[i]->msg_.data = contactFlagEstOld[i];
      footPhaseEstPublishers_[i]->unlockAndPublish();
    }
  }

  // foot observations
  double timeSec = time.toSec();
  std::array<int, 4> contactGt_;
  for (size_t i = 0; i < contactsSize; ++i) {
    contactGt_[i] = contactFlagGt[i] ? 1 : 0;
  }
  LeggedFootObs_->updateTime(timeSec);
  LeggedFootObs_->updateContactGt(contactGt_);
  LeggedFootObs_->updateJointStates(jointPos, jointVel, jointEffort);
  LeggedFootObs_->updateMeasuredRbdState(measuredRbdState_);
  LeggedFootObs_->update();
  const auto& obsFeet = LeggedFootObs_->getFootStates();

  const auto& legNames = leggedInterface_->modelSettings().contactNames3DoF;

  for (size_t i = 0; i < legNames.size(); ++i) {
    const auto& name = legNames[i];
    auto it = obsFeet.find(name);
    if (it == obsFeet.end()) continue;

    const auto& footState = it->second;

    // --- publish position ---
    if (footPosPublishers_[i] && footPosPublishers_[i]->trylock()) {
      auto& msg = footPosPublishers_[i]->msg_;
      msg.header.stamp = time;
      msg.header.frame_id = "world";  // 或 "odom" 依你的世界座標定義
      msg.vector.x = footState.position_W.x();
      msg.vector.y = footState.position_W.y();
      msg.vector.z = footState.position_W.z();
      footPosPublishers_[i]->unlockAndPublish();
    }

    // --- publish velocity ---
    if (footVelPublishers_[i] && footVelPublishers_[i]->trylock()) {
      auto& msg = footVelPublishers_[i]->msg_;
      msg.header.stamp = time;
      msg.header.frame_id = "world";
      msg.vector.x = footState.velocity_W.x();
      msg.vector.y = footState.velocity_W.y();
      msg.vector.z = footState.velocity_W.z();
      footVelPublishers_[i]->unlockAndPublish();
    }

    // --- publish contact force ---
    if (footForcePublishers_[i] && footForcePublishers_[i]->trylock()) {
      auto& msg = footForcePublishers_[i]->msg_;
      msg.header.stamp = time;
      msg.header.frame_id = "world";
      msg.vector.x = footState.contactForce_W.x();
      msg.vector.y = footState.contactForce_W.y();
      msg.vector.z = footState.contactForce_W.z();
      footForcePublishers_[i]->unlockAndPublish();
    }

    // --- publish F_ratio ---
    if (footFratioPublishers_[i] && footFratioPublishers_[i]->trylock()) {
      footFratioPublishers_[i]->msg_.data = footState.F_ratio;
      footFratioPublishers_[i]->unlockAndPublish();
    }

    // --- publish F_ratio_delta ---
    if (footFratioDeltaPublishers_[i] && footFratioDeltaPublishers_[i]->trylock()) {
      footFratioDeltaPublishers_[i]->msg_.data = footState.F_ratio_delta;
      // ROS_INFO_STREAM("Foot " << legNames[i] << " F_ratio_delta: " << footState.F_ratio_delta);
      footFratioDeltaPublishers_[i]->unlockAndPublish();
    }
  }

  // Update foot contact estimation
  if (count % 1 == 0) {
      footContactEst_-> update();
  }
  count++;
  const auto& feetEstState = footContactEst_-> getLegEstStates();
  for (size_t i=0; i < legNames.size(); ++i) {
    const auto& name = legNames[i];
    auto it = feetEstState.find(name);
    if (it == feetEstState.end()) continue;

    const auto& footEstState = it->second;
    contactFlagEstNew[i] = footEstState.isStance;

    if (footContactEstPublishers_[i]->trylock()) {
      footContactEstPublishers_[i]->msg_.data = contactFlagEstNew[i];
      footContactEstPublishers_[i]->unlockAndPublish();
    }

    if (footContactLogOddsPublishers_[i]->trylock()) {
      footContactLogOddsPublishers_[i]->msg_.data = footEstState.logOdds;
      footContactLogOddsPublishers_[i]->unlockAndPublish();
    }

    if (footContactCurrentLogisticPublishers_[i]->trylock()) {
      footContactCurrentLogisticPublishers_[i]->msg_.data = footEstState.currentLogistic;
      footContactCurrentLogisticPublishers_[i]->unlockAndPublish();
    } 

    if (footContactProbStancePublishers_[i]->trylock()) {
      footContactProbStancePublishers_[i]->msg_.data = footEstState.probStance;
      footContactProbStancePublishers_[i]->unlockAndPublish();
    }
  }
}

LeggedHilController::~LeggedHilController() {
  controllerRunning_ = false;
  if (mpcThread_.joinable()) {
    mpcThread_.join();
  }
  std::cerr << "########################################################################";
  std::cerr << "\n### MPC Benchmarking";
  std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
  std::cerr << "########################################################################";
  std::cerr << "\n### WBC Benchmarking";
  std::cerr << "\n###   Maximum : " << wbcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << wbcTimer_.getAverageInMilliseconds() << "[ms].";
}

void LeggedHilController::setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                            bool verbose) {
  leggedInterface_ = std::make_shared<LeggedInterface>(taskFile, urdfFile, referenceFile);
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
}

void LeggedHilController::setupMpc() {
  mpc_ = std::make_shared<SqpMpc>(leggedInterface_->mpcSettings(), leggedInterface_->sqpSettings(),
                                  leggedInterface_->getOptimalControlProblem(), leggedInterface_->getInitializer());
  rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(leggedInterface_->getPinocchioInterface(),
                                                                    leggedInterface_->getCentroidalModelInfo());

  const std::string robotName = "legged_robot";
  ros::NodeHandle nh;
  // Gait receiver
  auto gaitReceiverPtr =
      std::make_shared<GaitReceiver>(nh, leggedInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);
  // ROS ReferenceManager
  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, leggedInterface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nh);
  mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
  observationPublisher_ = nh.advertise<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1);
}

void LeggedHilController::setupMrt() {
  mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
  mpcMrtInterface_->initRollout(&leggedInterface_->getRollout());
  mpcTimer_.reset();

  controllerRunning_ = true;
  mpcThread_ = std::thread([&]() {
    while (controllerRunning_) {
      try {
        executeAndSleep(
            [&]() {
              if (mpcRunning_) {
                mpcTimer_.startTimer();
                mpcMrtInterface_->advanceMpc();
                mpcTimer_.endTimer();
              }
            },
            leggedInterface_->mpcSettings().mpcDesiredFrequency_);
      } catch (const std::exception& e) {
        controllerRunning_ = false;
        ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
        stopRequest(ros::Time());
      }
    }
  });
  setThreadPriority(leggedInterface_->sqpSettings().threadPriority, mpcThread_);

  if (!mpcCpuList_.empty()) {
    cpu_set_t cpuset;
    if (parseCpuList(mpcCpuList_, cpuset)) {
      if (setThreadAffinity(mpcThread_, cpuset)) {
        ROS_INFO_STREAM("MPC thread affinity set to CPUs: " << mpcCpuList_);
      } else {
        ROS_ERROR_STREAM("Failed to set MPC thread affinity to CPUs: " << mpcCpuList_);
      }
    } else {
      ROS_ERROR_STREAM("Invalid MPC CPU list format: " << mpcCpuList_);
    }
  }
  pthread_setname_np(mpcThread_.native_handle(), "mpc_main");
}

void LeggedHilController::setupStateEstimate(const std::string& taskFile, bool verbose) {
  stateEstimate_ = std::make_shared<KalmanFilterEstimate>(leggedInterface_->getPinocchioInterface(),
                                                          leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
  dynamic_cast<KalmanFilterEstimate&>(*stateEstimate_).loadSettings(taskFile, verbose);
  currentObservation_.time = 0;
}

bool LeggedHilController::parseCpuList(const std::string& cpuListStr, cpu_set_t& cpuset) {
  CPU_ZERO(&cpuset);
  if (cpuListStr.empty()) {
    return false;
  }

  std::stringstream ss(cpuListStr);
  std::string token;

  while (std::getline(ss, token, ',')) {
    try {
      int cpu = std::stoi(token);
      if (cpu < 0) {
        return false;
      }
      CPU_SET(cpu, &cpuset);
    } catch (const std::exception&) {
      return false;
    }
  }
  return true;
}

bool LeggedHilController::setThreadAffinity(std::thread& th, const cpu_set_t& cpuset) {
  int ret = pthread_setaffinity_np(th.native_handle(), sizeof(cpu_set_t), &cpuset);
  return (ret == 0);
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LeggedHilController, controller_interface::ControllerBase)