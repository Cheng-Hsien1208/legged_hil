#include "legged_joy/xbox_joy.h"

#include <algorithm>
#include <cmath>

namespace legged_joy {

XboxJoyNode::XboxJoyNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
: nh_(nh), pnh_(pnh)
{
  // --------------------
  // Params
  // --------------------
  pnh_.param<std::string>("joy_topic", joy_topic_, "/joy");
  pnh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");
  pnh_.param<std::string>("robot_name", robot_name_, "legged_robot");

  pnh_.param<std::string>("switch_service", switch_service_, "/controller_manager/switch_controller");
  pnh_.param<std::string>("/legged_controller_name", legged_controller_name_, "controllers/legged_hil_controller");
  pnh_.param<int>("switch_strictness", switch_strictness_, 2);
  pnh_.param<bool>("switch_start_asap", switch_start_asap_, true);
  pnh_.param<double>("switch_timeout", switch_timeout_, 0.0);

  pnh_.param<double>("loop_hz", loop_hz_, 200.0);
  pnh_.param<double>("deadzone", deadzone_, 0.1);

  pnh_.param<double>("max_vx", max_vx_, 1.0);
  pnh_.param<double>("max_vy", max_vy_, 0.5);
  pnh_.param<double>("max_vz", max_vz_, 1.2);

  pnh_.param<double>("max_roll",  max_roll_,  0.8);
  pnh_.param<double>("max_pitch", max_pitch_, 0.8);
  pnh_.param<double>("max_yaw",   max_yaw_,   1.0);

  pnh_.param<double>("speed_mul_init", speed_mul_, 1.0);
  pnh_.param<double>("speed_mul_step", speed_mul_step_, 0.1);
  pnh_.param<double>("speed_mul_min",  speed_mul_min_, 0.1);
  pnh_.param<double>("speed_mul_max",  speed_mul_max_, 2.0);

  // =====================
  //  Load Xbox Mapping
  // =====================
  ros::NodeHandle map_nh(pnh_, "xbox_map");   // ~/xbox_map
  ros::NodeHandle axes_nh(map_nh, "axes");    // ~/xbox_map/axes
  ros::NodeHandle btn_nh(map_nh,  "buttons"); // ~/xbox_map/buttons

  loadMapParam(axes_nh, "LX", map_.LX);
  loadMapParam(axes_nh, "LY", map_.LY);
  loadMapParam(axes_nh, "RX", map_.RX);
  loadMapParam(axes_nh, "RY", map_.RY);
  loadMapParam(axes_nh, "LT", map_.LT);
  loadMapParam(axes_nh, "RT", map_.RT);
  loadMapParam(axes_nh, "DPADX", map_.DPADX);
  loadMapParam(axes_nh, "DPADY", map_.DPADY);

  loadMapParam(btn_nh, "A",  map_.A);
  loadMapParam(btn_nh, "B",  map_.B);
  loadMapParam(btn_nh, "X",  map_.X);
  loadMapParam(btn_nh, "Y",  map_.Y);
  loadMapParam(btn_nh, "LB", map_.LB);
  loadMapParam(btn_nh, "RB", map_.RB);
  loadMapParam(btn_nh, "LS", map_.LS);
  loadMapParam(btn_nh, "RS", map_.RS);

  ROS_INFO_STREAM("[legged_joy] Xbox mapping:"
    << "\n  AXES"
    << "\n    LX=" << map_.LX << " LY=" << map_.LY
    << "\n    RX=" << map_.RX << " RY=" << map_.RY
    << "\n    LT=" << map_.LT << " RT=" << map_.RT
    << "\n    DPADX=" << map_.DPADX << " DPADY=" << map_.DPADY
    << "\n  BUTTONS"
    << "\n    A=" << map_.A << " B=" << map_.B
    << "\n    X=" << map_.X << " Y=" << map_.Y
    << "\n    LB=" << map_.LB << " RB=" << map_.RB
    << "\n    LS=" << map_.LS << " RS=" << map_.RS);


  // --------------------
  // ROS I/O
  // --------------------
  joy_sub_ = nh_.subscribe(joy_topic_, 1, &XboxJoyNode::joyCb, this);
  cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);

  // OCS2 gait switching topic
  mode_schedule_pub_ =
      nh_.advertise<ocs2_msgs::mode_schedule>(robot_name_ + "_mpc_mode_schedule", 1, true);

  // controller_manager switch service
  switch_client_ = nh_.serviceClient<controller_manager_msgs::SwitchController>(switch_service_);

  // --------------------
  // Load gait templates (route 1)
  // --------------------
  initGaitsOnce();

  // --------------------
  // Start control thread
  // --------------------
  running_.store(true);
  control_thread_ = std::thread(&XboxJoyNode::controlLoop, this);

  ROS_INFO_STREAM("[legged_joy] xbox_joy started"
                  << " joy_topic=" << joy_topic_
                  << " cmd_vel_topic=" << cmd_vel_topic_
                  << " mode_schedule_topic=" << (robot_name_ + "_mpc_mode_schedule")
                  << " switch_service=" << switch_service_
                  << " controller=" << legged_controller_name_);
}

XboxJoyNode::~XboxJoyNode() {
  running_.store(false);
  if (control_thread_.joinable()) control_thread_.join();
}

void XboxJoyNode::joyCb(const sensor_msgs::Joy::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx_);
  last_joy_ = *msg;
  have_joy_ = true;
}

bool XboxJoyNode::getButton(const sensor_msgs::Joy& j, int idx) const {
  if (idx < 0 || idx >= (int)j.buttons.size()) return false;
  return j.buttons[idx] != 0;
}

double XboxJoyNode::getAxis(const sensor_msgs::Joy& j, int idx) const {
  if (idx < 0 || idx >= (int)j.axes.size()) return 0.0;
  return (double)j.axes[idx];
}

double XboxJoyNode::applyDeadzone(double v) const {
  return (std::fabs(v) < deadzone_) ? 0.0 : v;
}

// --------------------
// Gait loading (once)
// --------------------
bool XboxJoyNode::initGaitsOnce() {
  // Keep compatible with OCS2 keyboard publisher: global param "/gaitCommandFile"
  if (!nh_.getParam("/gaitCommandFile", gait_command_file_)) {
    ROS_WARN("[legged_joy] Param /gaitCommandFile not found. Gait switching disabled.");
    gait_ready_ = false;
    return false;
  }

  ROS_INFO_STREAM("[legged_joy] Loading gait file: " << gait_command_file_);

  try {
    ocs2::loadData::loadStdVector(gait_command_file_, "list", gait_list_, true);

    gait_map_.clear();
    for (const auto& gaitName : gait_list_) {
      gait_map_.insert(
          {gaitName, ocs2::legged_robot::loadModeSequenceTemplate(gait_command_file_, gaitName, true)});
    }

    gait_ready_ = true;
    ROS_INFO_STREAM("[legged_joy] Gaits available:");
    for (const auto& g : gait_list_) ROS_INFO_STREAM("  - " << g);

    return true;
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("[legged_joy] Failed to load gait file: " << e.what());
    gait_ready_ = false;
    return false;
  }
}

// --------------------
// Set gait (publish once)
// --------------------
bool XboxJoyNode::setGait(const std::string& gait_name) {
  if (!gait_ready_) {
    ROS_WARN_THROTTLE(1.0, "[legged_joy] Gait not ready (missing /gaitCommandFile or load failed).");
    return false;
  }

  auto it = gait_map_.find(gait_name);
  if (it == gait_map_.end()) {
    ROS_WARN_STREAM("[legged_joy] Gait not found: " << gait_name);
    ROS_WARN_STREAM("[legged_joy] Available gaits:");
    for (const auto& g : gait_list_) ROS_WARN_STREAM("  - " << g);
    return false;
  }

  mode_schedule_pub_.publish(ocs2::legged_robot::createModeSequenceTemplateMsg(it->second));
  ROS_INFO_STREAM("[legged_joy] setGait -> " << gait_name);
  return true;
}

// --------------------
// controller_manager switch
// --------------------
bool XboxJoyNode::switchLeggedController(bool start) {
  if (!switch_client_.exists()) {
    ROS_WARN_STREAM_THROTTLE(1.0, "[legged_joy] switch service not available: " << switch_service_);
    return false;
  }

  controller_manager_msgs::SwitchController srv;
  if (start) {
    srv.request.start_controllers = {legged_controller_name_};
    srv.request.stop_controllers.clear();
  } else {
    srv.request.stop_controllers = {legged_controller_name_};
    srv.request.start_controllers.clear();
  }

  srv.request.strictness = switch_strictness_;
  srv.request.start_asap = switch_start_asap_;
  srv.request.timeout = switch_timeout_;

  if (!switch_client_.call(srv)) {
    ROS_WARN("[legged_joy] switch_controller call failed.");
    return false;
  }

  ROS_INFO_STREAM("[legged_joy] switch_controller start=" << (start ? "true" : "false")
                  << " ok=" << (srv.response.ok ? "true" : "false"));
  return srv.response.ok;
}

// --------------------
// Fixed-rate loop
// --------------------
void XboxJoyNode::controlLoop() {
  ros::Rate rate(loop_hz_);

  // Edge-trigger states (one-shot)
  bool prevY=false, prevX=false, prevA=false, prevB=false;
  bool prevStart=false, prevBack=false;
  int prevDpadY = 0;  // edge for speed multiplier

  // Convert trigger axis to [0,1]
  auto trigger01 = [&](const sensor_msgs::Joy& j, int axisIndex) -> double {
    const double a = getAxis(j, axisIndex);
    // Common ROS xbox mapping: released=+1, pressed=-1
    double t = (1.0 - a) * 0.5;  // [-1,1] -> [1,0]? actually: +1->0, -1->1
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    return t;
  };

  while (ros::ok() && running_.load()) {
    sensor_msgs::Joy joy;
    bool have = false;

    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (have_joy_) {
        joy = last_joy_;
        have = true;
      }
    }

    if (!have) {
      rate.sleep();
      continue;
    }

    // --------------------
    // Read buttons
    // --------------------
    const bool LB    = getButton(joy, map_.LB);      // 5) dead-man for velocity
    const bool RB    = getButton(joy, map_.RB);      // 6) gate for gait switch
    const bool START = getButton(joy, map_.LS);   // 8) start controller
    const bool STOP  = getButton(joy, map_.RS);    // 8) stop controller

    const bool A = getButton(joy, map_.A);
    const bool B = getButton(joy, map_.B);
    const bool X = getButton(joy, map_.X);
    const bool Y = getButton(joy, map_.Y);

    // --------------------
    // 8) Controller start/stop (independent of LB)
    // --------------------
    if (START && !prevStart) (void)switchLeggedController(true);
    if (STOP  && !prevBack)  (void)switchLeggedController(false);
    prevStart = START;
    prevBack  = STOP;

    // --------------------
    // D-pad X/Y are axes: -1 / 0 / +1
    // --------------------
    int dpadX = 0;
    {
      const double raw = getAxis(joy, map_.DPADX);
      if (raw > 0.5) dpadX = +1;
      else if (raw < -0.5) dpadX = -1;
      else dpadX = 0;
    }

    int dpadY = 0;
    {
      const double raw = getAxis(joy, map_.DPADY);
      if (raw > 0.5) dpadY = +1;
      else if (raw < -0.5) dpadY = -1;
      else dpadY = 0;
    }

    // --------------------
    // 7) DPADY controls speed multiplier (edge-trigger)
    // Up: +mul once, Down: -mul once
    // --------------------
    if (dpadY != 0 && prevDpadY == 0) {
      if (dpadY == +1) speed_mul_ = std::min(speed_mul_ + speed_mul_step_, speed_mul_max_);
      if (dpadY == -1) speed_mul_ = std::max(speed_mul_ - speed_mul_step_, speed_mul_min_);
    }
    prevDpadY = dpadY;

    // --------------------
    // 6) Gait switching only when RB held (edge-trigger on face buttons)
    // Y=stance, X=trot, A=static_walk, B=flying_trot
    // --------------------
    if (RB) {
      if (Y && !prevY) (void)setGait("stance");
      if (X && !prevX) (void)setGait("trot");
      if (A && !prevA) (void)setGait("static_walk");
      if (B && !prevB) (void)setGait("flying_trot");
    }
    prevY = Y; prevX = X; prevA = A; prevB = B;

    // --------------------
    // 5) Velocity commands require LB held; else output all zeros
    // --------------------
    geometry_msgs::Twist cmd;
    if (LB) {
      // 1) LX LY -> VX VY
      const double lx = applyDeadzone(getAxis(joy, map_.LX));
      const double ly = applyDeadzone(getAxis(joy, map_.LY));

      // 2) RX RY -> yaw pitch
      const double rx = applyDeadzone(getAxis(joy, map_.RX));
      const double ry = applyDeadzone(getAxis(joy, map_.RY));

      // 4) LT -> -VZ, RT -> +VZ
      const double lt = applyDeadzone(trigger01(joy, map_.LT));  // 0..1
      const double rt = applyDeadzone(trigger01(joy, map_.RT));  // 0..1

      // 3) DPADX -> roll (discrete -1/0/+1)
      const double roll_cmd = static_cast<double>(dpadX);

      const double s = speed_mul_;

      // linear
      cmd.linear.x = (ly) * max_vx_ * s;              // forward (+)
      cmd.linear.y = (lx) * max_vy_ * s;              // left (+)
      cmd.linear.z = (rt - lt) * max_vz_ * s;          // up (+) by RT, down (-) by LT

      // angular
      cmd.angular.z = ( rx) * max_yaw_ * s;            // yaw
      cmd.angular.y = ( ry) * max_pitch_ * s;          // pitch
      cmd.angular.x = (-roll_cmd) * max_roll_ * s;      // roll (D-pad X)
    } else {
      cmd = geometry_msgs::Twist{};
    }

    cmd_pub_.publish(cmd);
    rate.sleep();
  }
}

}  // namespace legged_joy

int main(int argc, char** argv) {
  ros::init(argc, argv, "xbox_joy");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // allow /joy callback concurrently with control thread
  ros::AsyncSpinner spinner(2);
  spinner.start();

  legged_joy::XboxJoyNode node(nh, pnh);
  ros::waitForShutdown();
  return 0;
}
