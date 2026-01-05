#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#include <controller_manager_msgs/SwitchController.h>

#include <ocs2_msgs/mode_schedule.h>
#include <ocs2_core/misc/LoadData.h>
#include "ocs2_legged_robot_ros/gait/ModeSequenceTemplateRos.h"

#include <thread>
#include <mutex>
#include <atomic>
#include <string>
#include <vector>
#include <unordered_map>

namespace legged_joy {

struct XboxMap {
  // axes
  int LX, LY, RX, RY, LT, RT, DPADX, DPADY;

  // buttons
  int A, B, X, Y, LB, RB, LS, RS;

  // default constructor
  XboxMap()
      : LX(0), LY(1), RX(3), RY(4),
        LT(2), RT(5), DPADX(6), DPADY(7),
        A(0), B(1), X(2), Y(3),
        LB(4), RB(5), LS(9), RS(10) {}
};


class XboxJoyNode {
public:
  XboxJoyNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~XboxJoyNode();

private:
  // ROS callback
  void joyCb(const sensor_msgs::Joy::ConstPtr& msg);

  // Fixed-rate loop
  void controlLoop();

  // Helpers
  bool getButton(const sensor_msgs::Joy& j, int idx) const;
  double getAxis(const sensor_msgs::Joy& j, int idx) const;
  double applyDeadzone(double v) const;

  // Gait (route 1: load file here)
  bool initGaitsOnce();
  bool setGait(const std::string& gait_name);

  // ros_control controller_manager
  bool switchLeggedController(bool start);

private:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber joy_sub_;

  ros::Publisher cmd_pub_;           // geometry_msgs/Twist
  ros::Publisher mode_schedule_pub_; // ocs2_msgs::mode_schedule
  ros::ServiceClient switch_client_; // controller_manager_msgs/SwitchController

  // Params
  std::string joy_topic_{"/joy"};
  std::string cmd_vel_topic_{"/cmd_vel"};

  // OCS2
  std::string robot_name_{"legged_robot"};      // topic: <robot_name>_mpc_mode_schedule
  std::string gait_command_file_;               // from global param "/gaitCommandFile"

  // Controller manager switch
  std::string switch_service_{"/controller_manager/switch_controller"};
  std::string legged_controller_name_{"controllers/legged_hil_controller"};

  int switch_strictness_{2};     // BEST_EFFORT=1, STRICT=2
  bool switch_start_asap_{true};
  double switch_timeout_{0.0};   // sec, 0 = no timeout

  // Loop
  double loop_hz_;
  double deadzone_;

  // Cmd limits (before speed multiplier)
  double max_vx_;
  double max_vy_;
  double max_vz_;

  double max_roll_;
  double max_pitch_;
  double max_yaw_;

  // Speed multiplier
  double speed_mul_;
  double speed_mul_step_;
  double speed_mul_min_;
  double speed_mul_max_;

  XboxMap map_;

  // Latest joy message
  std::mutex mtx_;
  sensor_msgs::Joy last_joy_;
  bool have_joy_{false};

  // Gait templates
  std::vector<std::string> gait_list_;
  std::unordered_map<std::string, ocs2::legged_robot::ModeSequenceTemplate> gait_map_;
  bool gait_ready_{false};

  // Thread
  std::atomic<bool> running_{false};
  std::thread control_thread_;

  // Mapping helpers
  template <typename T>
    void loadMapParam(const ros::NodeHandle& pnh, const std::string& name, T& field) {
      pnh.param<int>(name, field, field);
    }
};

}  // namespace legged_joy
