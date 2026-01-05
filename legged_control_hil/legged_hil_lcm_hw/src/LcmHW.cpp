#include "LcmHW.h"

namespace legged {

bool LcmHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
    if (!LeggedHilHW::init(root_nh, robot_hw_nh)) {
        return false;
    }

    setupJoints();
    setupImu();
    setupContactSensor(robot_hw_nh);

    std::string lcmCpuList_;
    int lcmThreadPriority_{90};

    // param
    root_nh.param("hil/ch_joints", ch_joints_, ch_joints_);
    root_nh.param("hil/ch_imu", ch_imu_, ch_imu_);
    root_nh.param("hil/ch_contacts", ch_contacts_, ch_contacts_);
    root_nh.param("hil/ch_cmd", ch_cmd_, ch_cmd_);
    root_nh.param("hil/state_delay", state_delay_, state_delay_);
    root_nh.param("hil/cmd_delay", cmd_delay_, cmd_delay_);
    root_nh.param<std::string>("legged_hil_hw/lcm_cpu_list", lcmCpuList_, "");
    root_nh.param<int>("legged_hil_hw/lcm_thread_priority", lcmThreadPriority_, 90);

    // LCM setup
    if (!lcm_.good()) {
        ROS_FATAL("LCM is not good()");
        return false;
    }

    sub_joints_ = lcm_.subscribe(ch_joints_, &LcmHW::onJoints, this);
    sub_imu_ = lcm_.subscribe(ch_imu_, &LcmHW::onImu, this);
    sub_contacts_ = lcm_.subscribe(ch_contacts_, &LcmHW::onContacts, this);

    th_running_.store(true);
    th_lcm_ = std::thread(&LcmHW::lcmSpin, this);

    // Set thread priority and affinity
    sched_param sched{.sched_priority = lcmThreadPriority_};
    if (pthread_setschedparam(th_lcm_.native_handle(), SCHED_FIFO, &sched) != 0) {
        ROS_WARN(
            "Failed to set LCM thread priority (one possible reason could be that the user and the group permissions "
            "are not set properly.).\n");
    }
    if (!lcmCpuList_.empty()) {
        cpu_set_t cpuset;
        if (parseCpuList(lcmCpuList_, cpuset)) {
            if (setThreadAffinity(th_lcm_, cpuset)) {
                ROS_INFO_STREAM("LCM thread affinity set to CPUs: " << lcmCpuList_);
            } else {
                ROS_ERROR_STREAM("Failed to set LCM thread affinity to CPUs: " << lcmCpuList_);
            }
        } else {
            ROS_ERROR_STREAM("Invalid LCM CPU list format: " << lcmCpuList_);
        }
    }
    std::string thread_name = "lcm";
    pthread_setname_np(th_lcm_.native_handle(), thread_name.c_str());

    return true;
}

bool LcmHW::parseCpuList(const std::string& cpuListStr, cpu_set_t& cpuset) {
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

bool LcmHW::setThreadAffinity(std::thread& th, const cpu_set_t& cpuset) {
  int ret = pthread_setaffinity_np(th.native_handle(), sizeof(cpu_set_t), &cpuset);
  return (ret == 0);
}

void LcmHW::lcmSpin() {
    while (th_running_.load()) {
        lcm_.handleTimeout(10);
    }
}

void LcmHW::onJoints(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const legged_hil_lcm::joints_msg_t* msg) {
    std::lock_guard<std::mutex> lock(mtx_);

    for (int i = 0; i < msg->num_joints; ++i) {
        const std::string& nm = msg->name[i];
        auto it = jointNameToIndex_.find(nm);
        if (it == jointNameToIndex_.end()) continue;

        const int idx = it->second;

        jointData_[idx].sec  = msg->sec[i];
        jointData_[idx].nsec = msg->nsec[i];
        jointData_[idx].pos_ = msg->position[i];
        jointData_[idx].vel_ = msg->velocity[i];
        jointData_[idx].tau_ = msg->effort[i];
    }

    lastStateTime_ = ros::Time::now();
}

void LcmHW::onImu(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const legged_hil_lcm::imu_msg_t* msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    imuData_.sec = msg->sec;
    imuData_.nsec = msg->nsec;
    imuData_.frame_id = msg->frame_id;
    for(int i = 0; i < 4; ++i) imuData_.orientation[i] = msg->orientation[i];
    for(int i = 0; i < 3; ++i) imuData_.angular_velocity[i] = msg->angular_velocity[i];
    for(int i = 0; i < 3; ++i) imuData_.linear_acceleration[i] = msg->linear_acceleration[i];
    for(int i = 0; i < 9; ++i) imuData_.orientation_covariance[i] = msg->orientation_covariance[i];
    for(int i = 0; i < 9; ++i) imuData_.angular_velocity_covariance[i] = msg->angular_velocity_covariance[i];
    for(int i = 0; i < 9; ++i) imuData_.linear_acceleration_covariance[i] = msg->linear_acceleration_covariance[i];
}

void LcmHW::onContacts(const lcm::ReceiveBuffer* rbuf, const std::string& chan,const legged_hil_lcm::contacts_msg_t* msg) {
    std::lock_guard<std::mutex> lock(mtx_);

    for (int i = 0; i < msg->num_contacts; ++i) {
        const std::string& nm = msg->name[i];
        auto it = contactNameToIndex_.find(nm);
        if (it == contactNameToIndex_.end()) continue;

        const int idx = it->second;
        contactData_[idx].sec = msg->sec[i];
        contactData_[idx].nsec = msg->nsec[i];
        contactData_[idx].isContact = msg->isContact[i];
    }
}

bool LcmHW::setupJoints() {
    jointNameToIndex_.clear();
    
    for (const auto& kv : urdfModel_->joints_) {
        const std::string& name = kv.first;
        const auto& joint = kv.second;

        int leg_index = -1;
        int joint_index = -1;

        if (name.find("LF") != std::string::npos)      leg_index = 0;
        else if (name.find("LH") != std::string::npos) leg_index = 1;
        else if (name.find("RF") != std::string::npos) leg_index = 2;
        else if (name.find("RH") != std::string::npos) leg_index = 3;
        else continue;

        if (name.find("HAA") != std::string::npos)      joint_index = 0;
        else if (name.find("HFE") != std::string::npos) joint_index = 1;
        else if (name.find("KFE") != std::string::npos) joint_index = 2;
        else continue;

        const int index = leg_index * 3 + joint_index;
        if (index < 0 || index >= 12) continue;

        jointNameToIndex_[name] = index;

        hardware_interface::JointStateHandle state_handle(
            name,
            &jointData_[index].pos_,
            &jointData_[index].vel_,
            &jointData_[index].tau_);

        jointStateInterface_.registerHandle(state_handle);

        jointCmd_[index].name = name;
        jointCmd_[index].sec = 0;
        jointCmd_[index].nsec = 0;
        jointCmd_[index].posDes_ = 0.0;
        jointCmd_[index].velDes_ = 0.0;
        jointCmd_[index].kp_ = 0.0;
        jointCmd_[index].kd_ = 0.0;
        jointCmd_[index].ff_ = 0.0;

        impedanceJointInterface_.registerHandle(
            ImpedanceJointHandle(
                state_handle,
                &jointData_[index].sec,
                &jointData_[index].nsec,
                &jointCmd_[index].sec,
                &jointCmd_[index].nsec,
                &jointCmd_[index].posDes_,
                &jointCmd_[index].velDes_,
                &jointCmd_[index].kp_,
                &jointCmd_[index].kd_,
                &jointCmd_[index].ff_));
    }

    if (jointNameToIndex_.size() != 12) {
        ROS_ERROR_STREAM("[LcmHW] Expected 12 leg joints, got " << jointNameToIndex_.size());
        return false;
    }

    return true;
}

bool LcmHW::setupImu() {
    imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle("base_imu","base_imu",
                                                                         imuData_.orientation,
                                                                         imuData_.orientation_covariance,
                                                                         imuData_.angular_velocity,
                                                                         imuData_.angular_velocity_covariance,
                                                                         imuData_.linear_acceleration,
                                                                         imuData_.linear_acceleration_covariance));
    return true;
}

bool LcmHW::setupContactSensor(ros::NodeHandle& nh) {
    contactNameToIndex_.clear();

    for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
        const auto& nm = CONTACT_SENSOR_NAMES[i];
        contactNameToIndex_[nm] = static_cast<int>(i);
        contactSensorInterface_.registerHandle(ContactSensorHandle(nm, &contactData_[i].isContact));
    }
    return true;
}

void LcmHW::read(const ros::Time& time, const ros::Duration& /*period*/) {
    std::lock_guard<std::mutex> lock(mtx_);
    const double dt_state = (time - lastStateTime_).toSec();
    if (dt_state > state_delay_) {
        ROS_WARN_THROTTLE(1.0, "[LcmHW] No new LCM state received for %.3f seconds", dt_state);
        state_timeout_ = true;
        for (int i = 0; i < 12; ++i) {
            jointData_[i].vel_ = 0.0;
            jointData_[i].tau_ = 0.0;
        }
    }else{
        state_timeout_ = false;
    }
}

void LcmHW::write(const ros::Time& time, const ros::Duration& /*period*/) {
    std::lock_guard<std::mutex> lock(mtx_);
    for (int i = 0; i < 12; ++i) {
        const ros::Time t_cmd(jointCmd_[i].sec, jointCmd_[i].nsec);

        const double dt_cmd = (time - t_cmd).toSec();
        if (dt_cmd > cmd_delay_) {
            cmd_timeout_ = true;
            break;
        }else{
            cmd_timeout_ = false;
        }
    }

    if( state_timeout_ || cmd_timeout_ ){
        ROS_WARN_THROTTLE(1.0, "[LcmHW] Skip sending command due to timeout (state_timeout: %d, cmd_timeout: %d)", state_timeout_, cmd_timeout_);
        for (int i = 0; i < 12; ++i) {
            jointCmd_[i].posDes_ = jointData_[i].pos_;
            jointCmd_[i].velDes_ = 0.0;
            jointCmd_[i].kp_ = 0.0;
            jointCmd_[i].kd_ = 0.0;
            jointCmd_[i].ff_ = 0.0;
        }
    }

    legged_hil_lcm::joints_cmd_t cmd_msg;
    
    cmd_msg.num_joints = 12;


    cmd_msg.sec.resize(12);
    cmd_msg.nsec.resize(12);
    cmd_msg.name.resize(12);
    cmd_msg.pos_des.resize(12);
    cmd_msg.vel_des.resize(12);
    cmd_msg.kp.resize(12);
    cmd_msg.kd.resize(12);
    cmd_msg.ff.resize(12);
    
    for (int i = 0; i < 12; ++i) {
        cmd_msg.sec[i] = jointData_[i].sec;
        cmd_msg.nsec[i] = jointData_[i].nsec;
        cmd_msg.name[i] = jointCmd_[i].name;
        cmd_msg.pos_des[i] = jointCmd_[i].posDes_;
        cmd_msg.vel_des[i] = jointCmd_[i].velDes_;
        cmd_msg.kp[i] = jointCmd_[i].kp_;
        cmd_msg.kd[i] = jointCmd_[i].kd_;
        cmd_msg.ff[i] = jointCmd_[i].ff_;
    }

    lcm_.publish(ch_cmd_, &cmd_msg);
}

LcmHW::~LcmHW(){
    th_running_.store(false);
    if (th_lcm_.joinable()) {
        th_lcm_.join();
    }
}

}  // namespace legged
