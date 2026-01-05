#include "legged_hil_gazebo/LeggedHilHWSim.h"
#include <gazebo_ros_control/gazebo_ros_control_plugin.h>

namespace legged {
bool LeggedHilHWSim::initSim(const std::string& robot_namespace, ros::NodeHandle model_nh, gazebo::physics::ModelPtr parent_model,
                          const urdf::Model* urdf_model, std::vector<transmission_interface::TransmissionInfo> transmissions) {
  bool ret = DefaultRobotHWSim::initSim(robot_namespace, model_nh, parent_model, urdf_model, transmissions);

  model_nh.param("hil/kp_default", kp_default_, kp_default_);
  model_nh.param("hil/kd_default", kd_default_, kp_default_);

  // Joint interface
  registerInterface(&impedanceJointInterface_);
  std::vector<std::string> joint_names_ = ej_interface_.getNames();
  for (const auto& name : joint_names_) {
    impedanceJointDatas_.push_back(ImpedanceJointData{.joint_ = ej_interface_.getHandle(name), .kp_ = kp_default_, .kd_ = kd_default_});
    ImpedanceJointData& back = impedanceJointDatas_.back();
    impedanceJointInterface_.registerHandle(ImpedanceJointHandle(back.joint_, 0, 0, 0, 0, &back.posDes_, &back.velDes_, &back.kp_, &back.kd_, &back.ff_));
    cmdBuffer_.insert(std::make_pair(name.c_str(), std::deque<ImpedanceJointCommand>()));
  }

  // IMU interface
  registerInterface(&imuSensorInterface_);
  XmlRpc::XmlRpcValue xmlRpcValue;
  if (!model_nh.getParam("gazebo/imus", xmlRpcValue)) {
    ROS_WARN("No imu specified");
    return false;
  } else {
    parseImu(xmlRpcValue, parent_model);
  }
  if (!model_nh.getParam("gazebo/delay", delay_)) {
    delay_ = 0.;
  }
  if (!model_nh.getParam("gazebo/contacts", xmlRpcValue)) {
    ROS_WARN("No contacts specified");
  } else {
    parseContacts(xmlRpcValue);
  }

  contactManager_ = parent_model->GetWorld()->Physics()->GetContactManager();
  contactManager_->SetNeverDropContacts(true);  // NOTE: If false, we need to select view->contacts in gazebo GUI to
                                                // avoid returning nothing when calling ContactManager::GetContacts()

  // LCM channel / hz
  model_nh.param("hil/ch_joints", ch_joints_, ch_joints_);
  model_nh.param("hil/ch_imu", ch_imu_, ch_imu_);
  model_nh.param("hil/ch_contacts", ch_contacts_, ch_contacts_);
  model_nh.param("hil/hz_joints", hz_joints_, hz_joints_);
  model_nh.param("hil/hz_imu", hz_imu_, hz_imu_);
  model_nh.param("hil/hz_contacts", hz_contacts_, hz_contacts_);
  model_nh.param("hil/state_delay", delay_, delay_);
  
  if (!lcm_.good()) {
    ROS_ERROR("LCM is not good()");
  }
  
  // joints_msg_
  {
    std::lock_guard<std::mutex> lock(mtx_);
    auto names = ej_interface_.getNames();
    joints_msg_.num_joints = static_cast<int32_t>(names.size());

    joints_msg_.name.resize(names.size());
    for (size_t i=0;i<names.size();++i) joints_msg_.name[i] = names[i];
    
    joints_msg_.sec.resize(names.size());
    joints_msg_.nsec.resize(names.size());
    joints_msg_.position.resize(names.size());
    joints_msg_.velocity.resize(names.size());
    joints_msg_.effort.resize(names.size());
  }

  // imu_msg_
  {
    std::lock_guard<std::mutex> lock(mtx_);
    imu_msg_.sec = 0;
    imu_msg_.nsec = 0;

    imu_msg_.frame_id = imuDatas_.front().frame_id_;
  }

  // contacts_msg_
  {
    std::lock_guard<std::mutex> lock(mtx_);

    contacts_msg_.num_contacts = static_cast<int32_t>(name2contact_.size());
    contacts_msg_.sec.resize(name2contact_.size());
    contacts_msg_.nsec.resize(name2contact_.size());
    contacts_msg_.name.resize(name2contact_.size());
    contacts_msg_.isContact.resize(name2contact_.size());
    int idx=0;
    for (auto& kv : name2contact_) {
      contacts_msg_.name[idx] = kv.first;
      contacts_msg_.isContact[idx] = kv.second;
      idx++;
    }
  }

  th_running_.store(true);

  // joints thread
  th_joints_ = std::thread([this](){
    ros::Rate r(hz_joints_);
    while (th_running_.load()) {
      legged_hil_lcm::joints_msg_t msg;
      { std::lock_guard<std::mutex> lock(mtx_); msg = joints_msg_; }
      lcm_.publish(ch_joints_, &msg);
      r.sleep();
    }
  });

  // imu thread
  th_imu_ = std::thread([this](){
    ros::Rate r(hz_imu_);
    while (th_running_.load()) {
      legged_hil_lcm::imu_msg_t msg;
      { std::lock_guard<std::mutex> lock(mtx_); msg = imu_msg_; }
      lcm_.publish(ch_imu_, &msg);
      r.sleep();
    }
  });

  // contacts thread
  th_contacts_ = std::thread([this](){
    ros::Rate r(hz_contacts_);
    while (th_running_.load()) {
      legged_hil_lcm::contacts_msg_t msg;
      { std::lock_guard<std::mutex> lock(mtx_); msg = contacts_msg_; }
      lcm_.publish(ch_contacts_, &msg);
      r.sleep();
    }
  });

  sub_cmd_ = lcm_.subscribe<legged_hil_lcm::joints_cmd_t>(ch_cmd_, &LeggedHilHWSim::onJointCmd, this);
  th_cmd_ = std::thread(&LeggedHilHWSim::lcmSpin, this);

  return ret;
}

void LeggedHilHWSim::lcmSpin() {
  while (th_running_.load()) {
    lcm_.handleTimeout(10);
  }
}


void LeggedHilHWSim::onJointCmd(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const legged_hil_lcm::joints_cmd_t* msg) {
  const int num_joints = msg->num_joints;

  if (num_joints != static_cast<int>(joint_names_.size())) {
    ROS_WARN("Received joints_cmd_t with different number of joints: expected %zu, got %d", joint_names_.size(), num_joints);
    return;
  }

  std::lock_guard<std::mutex> lock(mtx_);

  for (int i = 0; i < num_joints; i++) {
    const std::string& name = msg->name[i];
    if (cmdBuffer_.find(name) == cmdBuffer_.end()) {
      ROS_WARN("Received joints_cmd_t with unknown joint name: %s", name.c_str());
      continue;
    }

    ImpedanceJointCommand cmd;
    cmd.stamp_.sec = msg->sec[i];
    cmd.stamp_.nsec = msg->nsec[i];
    cmd.posDes_ = msg->pos_des[i];
    cmd.velDes_ = msg->vel_des[i];
    cmd.kp_ = msg->kp[i];
    cmd.kd_ = msg->kd[i];
    cmd.ff_ = msg->ff[i];
    cmdBuffer_[name].push_front(cmd);
  }  
}

void LeggedHilHWSim::readSim(ros::Time time, ros::Duration period) {
  //  gazebo_ros_control::DefaultRobotHWSim::readSim(time, period); The DefaultRobotHWSim Provide a bias joint velocity
  for (unsigned int j = 0; j < n_dof_; j++) {
    double position = sim_joints_[j]->Position(0);

    joint_velocity_[j] = (position - joint_position_[j]) / period.toSec();
    if (time == ros::Time(period.toSec())) {
      joint_velocity_[j] = 0;
    }
    if (joint_types_[j] == urdf::Joint::PRISMATIC) {
      joint_position_[j] = position;
    } else {
      joint_position_[j] += angles::shortest_angular_distance(joint_position_[j], position);
    }
    joint_effort_[j] = sim_joints_[j]->GetForce((unsigned int)(0));
  }

  // Imu Sensor
  for (auto& imu : imuDatas_) {
    // TODO Add noise
    ignition::math::Pose3d pose = imu.linkPtr_->WorldPose();
    imu.ori_[0] = pose.Rot().X();
    imu.ori_[1] = pose.Rot().Y();
    imu.ori_[2] = pose.Rot().Z();
    imu.ori_[3] = pose.Rot().W();
    ignition::math::Vector3d rate = imu.linkPtr_->RelativeAngularVel();
    imu.angularVel_[0] = rate.X();
    imu.angularVel_[1] = rate.Y();
    imu.angularVel_[2] = rate.Z();

    ignition::math::Vector3d gravity = {0., 0., -9.81};
    ignition::math::Vector3d accel = imu.linkPtr_->RelativeLinearAccel() - pose.Rot().RotateVectorReverse(gravity);
    imu.linearAcc_[0] = accel.X();
    imu.linearAcc_[1] = accel.Y();
    imu.linearAcc_[2] = accel.Z();
  }

  // Contact Sensor
  for (auto& state : name2contact_) {
    state.second = false;
  }
  for (const auto& contact : contactManager_->GetContacts()) {
    if (static_cast<uint32_t>(contact->time.sec) != (time - period).sec ||
        static_cast<uint32_t>(contact->time.nsec) != (time - period).nsec) {
      continue;
    }
    std::string linkName = contact->collision1->GetLink()->GetName();
    if (name2contact_.find(linkName) != name2contact_.end()) {
      name2contact_[linkName] = true;
    }
    linkName = contact->collision2->GetLink()->GetName();
    if (name2contact_.find(linkName) != name2contact_.end()) {
      name2contact_[linkName] = true;
    }
  }

  // Set cmd to zero to avoid crazy soft limit oscillation when not controller loaded
  for (auto& cmd : joint_effort_command_) {
    cmd = 0;
  }
  for (auto& cmd : joint_velocity_command_) {
    cmd = 0;
  }
  for (auto& joint : impedanceJointDatas_) {
    joint.posDes_ = joint.joint_.getPosition();
    joint.velDes_ = joint.joint_.getVelocity();
    joint.kp_ = 0.;
    joint.kd_ = 0.;
    joint.ff_ = 0.;
  }

  // LCM
  std::lock_guard<std::mutex> lock(mtx_);

  // joints_msg_
  for (int i = 0; i < n_dof_; i++) {
    joints_msg_.sec[i] = time.sec;
    joints_msg_.nsec[i] = time.nsec;
    joints_msg_.position[i] = joint_position_[i];
    joints_msg_.velocity[i] = joint_velocity_[i];
    joints_msg_.effort[i]   = joint_effort_[i];
  }

  // imu_msg_
  imu_msg_.sec = time.sec;
  imu_msg_.nsec = time.nsec;
  for (int k = 0; k < 4; k++) imu_msg_.orientation[k] = imuDatas_.front().ori_[k];
  for (int k = 0; k < 9; k++) imu_msg_.orientation_covariance[k] = imuDatas_.front().oriCov_[k];
  for (int k = 0; k < 3; k++) imu_msg_.angular_velocity[k] = imuDatas_.front().angularVel_[k];
  for (int k = 0; k < 9; k++) imu_msg_.angular_velocity_covariance[k] = imuDatas_.front().angularVelCov_[k];
  for (int k = 0; k < 3; k++) imu_msg_.linear_acceleration[k] = imuDatas_.front().linearAcc_[k];
  for (int k = 0; k < 9; k++) imu_msg_.linear_acceleration_covariance[k] = imuDatas_.front().linearAccCov_[k];

  // contacts_msg_
  for (int i = 0; i < contacts_msg_.num_contacts; i++) {
    const auto& nm = contacts_msg_.name[i];
    contacts_msg_.sec[i] = time.sec;
    contacts_msg_.nsec[i] = time.nsec;
    contacts_msg_.isContact[i] = name2contact_[nm];
  }
}

void LeggedHilHWSim::writeSim(ros::Time time, ros::Duration period) {
  for (auto joint : impedanceJointDatas_) {
    auto& buffer = cmdBuffer_.find(joint.joint_.getName())->second;
    if (time == ros::Time(period.toSec())) {  // Simulation reset
      buffer.clear();
    }

    while (!buffer.empty() && buffer.back().stamp_ + ros::Duration(delay_) < time) {
      buffer.pop_back();
    }

    if (buffer.empty()) {
      buffer.push_front(ImpedanceJointCommand{
        .stamp_ = time, .posDes_ = 0, .velDes_ = 0, .kp_ = kp_default_, .kd_ = kp_default_, .ff_ = 0});
    }

    const auto& cmd = buffer.front();
    joint.joint_.setCommand(cmd.kp_ * (cmd.posDes_ - joint.joint_.getPosition()) + cmd.kd_ * (cmd.velDes_ - joint.joint_.getVelocity()) +
                            cmd.ff_);
  }
  DefaultRobotHWSim::writeSim(time, period);
}

void LeggedHilHWSim::parseImu(XmlRpc::XmlRpcValue& imuDatas, const gazebo::physics::ModelPtr& parentModel) {
  ROS_ASSERT(imuDatas.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  for (auto it = imuDatas.begin(); it != imuDatas.end(); ++it) {
    if (!it->second.hasMember("frame_id")) {
      ROS_ERROR_STREAM("Imu " << it->first << " has no associated frame id.");
      continue;
    } else if (!it->second.hasMember("orientation_covariance_diagonal")) {
      ROS_ERROR_STREAM("Imu " << it->first << " has no associated orientation covariance diagonal.");
      continue;
    } else if (!it->second.hasMember("angular_velocity_covariance")) {
      ROS_ERROR_STREAM("Imu " << it->first << " has no associated angular velocity covariance.");
      continue;
    } else if (!it->second.hasMember("linear_acceleration_covariance")) {
      ROS_ERROR_STREAM("Imu " << it->first << " has no associated linear acceleration covariance.");
      continue;
    }
    XmlRpc::XmlRpcValue oriCov = imuDatas[it->first]["orientation_covariance_diagonal"];
    ROS_ASSERT(oriCov.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(oriCov.size() == 3);
    for (int i = 0; i < oriCov.size(); ++i) {
      ROS_ASSERT(oriCov[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    }
    XmlRpc::XmlRpcValue angularCov = imuDatas[it->first]["angular_velocity_covariance"];
    ROS_ASSERT(angularCov.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(angularCov.size() == 3);
    for (int i = 0; i < angularCov.size(); ++i) {
      ROS_ASSERT(angularCov[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    }
    XmlRpc::XmlRpcValue linearCov = imuDatas[it->first]["linear_acceleration_covariance"];
    ROS_ASSERT(linearCov.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(linearCov.size() == 3);
    for (int i = 0; i < linearCov.size(); ++i) {
      ROS_ASSERT(linearCov[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    }

    std::string frameId = imuDatas[it->first]["frame_id"];
    gazebo::physics::LinkPtr linkPtr = parentModel->GetLink(frameId);
    ROS_ASSERT(linkPtr != nullptr);
    imuDatas_.push_back((ImuData{
        .linkPtr_ = linkPtr,
        .stamp_ = ros::Time(0),
        .frame_id_ = frameId,
        .ori_ = {0., 0., 0., 0.},
        .oriCov_ = {static_cast<double>(oriCov[0]), 0., 0., 0., static_cast<double>(oriCov[1]), 0., 0., 0., static_cast<double>(oriCov[2])},
        .angularVel_ = {0., 0., 0.},
        .angularVelCov_ = {static_cast<double>(angularCov[0]), 0., 0., 0., static_cast<double>(angularCov[1]), 0., 0., 0.,
                           static_cast<double>(angularCov[2])},
        .linearAcc_ = {0., 0., 0.},
        .linearAccCov_ = {static_cast<double>(linearCov[0]), 0., 0., 0., static_cast<double>(linearCov[1]), 0., 0., 0.,
                          static_cast<double>(linearCov[2])}}));
    ImuData& imuData = imuDatas_.back();
    imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle(it->first, frameId, imuData.ori_, imuData.oriCov_,
                                                                           imuData.angularVel_, imuData.angularVelCov_, imuData.linearAcc_,
                                                                           imuData.linearAccCov_));
  }
}

void LeggedHilHWSim::parseContacts(XmlRpc::XmlRpcValue& contactNames) {
  ROS_ASSERT(contactNames.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int i = 0; i < contactNames.size(); ++i) {  // NOLINT(modernize-loop-convert)
    std::string name = contactNames[i];
    name2contact_.insert(std::make_pair(name, false));
    contactSensorInterface_.registerHandle(ContactSensorHandle(name, &name2contact_[name]));
  }
  registerInterface(&contactSensorInterface_);
}

LeggedHilHWSim::~LeggedHilHWSim() {
  th_running_.store(false);
  if (th_joints_.joinable()) th_joints_.join();
  if (th_imu_.joinable()) th_imu_.join();
  if (th_contacts_.joinable()) th_contacts_.join();
  if (th_cmd_.joinable()) th_cmd_.join();
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LeggedHilHWSim, gazebo_ros_control::RobotHWSim)
GZ_REGISTER_MODEL_PLUGIN(gazebo_ros_control::GazeboRosControlPlugin)  // Default plugin
