#pragma once

#include <deque>
#include <unordered_map>

#include <gazebo_ros_control/default_robot_hw_sim.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include <legged_hil_interface/hardware_interface/ContactSensorInterface.h>
#include <legged_hil_interface/hardware_interface/ImpedanceJointInterface.h>

#include <lcm/lcm-cpp.hpp>
#include "legged_hil_lcm/joints_msg_t.hpp"
#include "legged_hil_lcm/imu_msg_t.hpp"
#include "legged_hil_lcm/contacts_msg_t.hpp"
#include "legged_hil_lcm/joints_cmd_t.hpp"

#include <mutex>
#include <thread>
#include <atomic>

namespace legged {
struct ImpedanceJointData {
  hardware_interface::JointHandle joint_;
  ros::Time stamp_;
  double posDes_{}, velDes_{}, kp_{}, kd_{}, ff_{};
};

struct ImpedanceJointCommand {
  ros::Time stamp_;
  double posDes_{}, velDes_{}, kp_{}, kd_{}, ff_{};
};

struct ImuData {
  gazebo::physics::LinkPtr linkPtr_;
  ros::Time stamp_;
  std::string frame_id_;
  double ori_[4];            // NOLINT(modernize-avoid-c-arrays)
  double oriCov_[9];         // NOLINT(modernize-avoid-c-arrays)
  double angularVel_[3];     // NOLINT(modernize-avoid-c-arrays)
  double angularVelCov_[9];  // NOLINT(modernize-avoid-c-arrays)
  double linearAcc_[3];      // NOLINT(modernize-avoid-c-arrays)
  double linearAccCov_[9];   // NOLINT(modernize-avoid-c-arrays)
};

class LeggedHilHWSim : public gazebo_ros_control::DefaultRobotHWSim {
 public:
  ~LeggedHilHWSim() override;
  bool initSim(const std::string& robot_namespace, ros::NodeHandle model_nh, gazebo::physics::ModelPtr parent_model,
               const urdf::Model* urdf_model, std::vector<transmission_interface::TransmissionInfo> transmissions) override;
  void readSim(ros::Time time, ros::Duration period) override;
  void writeSim(ros::Time time, ros::Duration period) override;

 private:
  void parseImu(XmlRpc::XmlRpcValue& imuDatas, const gazebo::physics::ModelPtr& parentModel);
  void parseContacts(XmlRpc::XmlRpcValue& contactNames);

  ImpedanceJointInterface impedanceJointInterface_;
  ContactSensorInterface contactSensorInterface_;
  hardware_interface::ImuSensorInterface imuSensorInterface_;

  gazebo::physics::ContactManager* contactManager_{};

  std::list<ImpedanceJointData> impedanceJointDatas_;
  std::list<ImuData> imuDatas_;
  std::unordered_map<std::string, std::deque<ImpedanceJointCommand> > cmdBuffer_;
  std::unordered_map<std::string, bool> name2contact_;

  double kp_default_{};
  double kd_default_{};

  double delay_{};

  // LCM
  lcm::LCM lcm_;
  std::string ch_joints_{"JOINTS_MSG"};
  std::string ch_imu_{"IMU_MSG"};
  std::string ch_contacts_{"CONTACTS_MSG"};
  std::string ch_cmd_{"JOINT_CMD_MSG"};

  // ---- LCM callbacks ----
  void onJointCmd(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const legged_hil_lcm::joints_cmd_t* msg);
  void lcmSpin();  // handleTimeout loop
  lcm::Subscription* sub_cmd_{nullptr};

  // thread control
  std::atomic<bool> th_running_{false};
  std::thread th_joints_, th_imu_, th_contacts_, th_cmd_;

  double hz_joints_{200.0};
  double hz_imu_{400.0};
  double hz_contacts_{200.0};

  // latest data cache (protected)
  std::mutex mtx_;
  legged_hil_lcm::joints_msg_t joints_msg_;
  legged_hil_lcm::imu_msg_t imu_msg_;
  legged_hil_lcm::contacts_msg_t contacts_msg_;
  legged_hil_lcm::joints_cmd_t joints_cmd_msg_;
};

}  // namespace legged
