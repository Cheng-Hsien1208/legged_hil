#pragma once
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_state_interface.h>

namespace legged {
class ImpedanceJointHandle : public hardware_interface::JointStateHandle {
 public:
  ImpedanceJointHandle() = default;

  ImpedanceJointHandle(const JointStateHandle& js, int* state_sec, int* state_nsec, int* cmd_sec, int* cmd_nsec, double* posDes, double* velDes, double* kp, double* kd, double* ff)
      : JointStateHandle(js), state_sec_(state_sec), state_nsec_(state_nsec), cmd_sec_(cmd_sec), cmd_nsec_(cmd_nsec), posDes_(posDes), velDes_(velDes), kp_(kp), kd_(kd), ff_(ff){
    if (posDes_ == nullptr) {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() +
                                                           "'. Position desired data pointer is null.");
    }
    if (velDes_ == nullptr) {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() +
                                                           "'. Velocity desired data pointer is null.");
    }
    if (kp_ == nullptr) {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Kp data pointer is null.");
    }
    if (kd_ == nullptr) {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Kd data pointer is null.");
    }
    if (ff_ == nullptr) {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() +
                                                           "'. Feedforward data pointer is null.");
    }
  }

  void setStateTimestamp(int sec, int nsec) {
    assert(state_sec_);
    assert(state_nsec_);
    *state_sec_ = sec;
    *state_nsec_ = nsec;
  }

  void setCmdTimestamp(int sec, int nsec) {
    assert(cmd_sec_);
    assert(cmd_nsec_);
    *cmd_sec_ = sec;
    *cmd_nsec_ = nsec;
  }

  void setPositionDesired(double cmd) {
    assert(posDes_);
    *posDes_ = cmd;
  }

  void setVelocityDesired(double cmd) {
    assert(velDes_);
    *velDes_ = cmd;
  }

  void setKp(double cmd) {
    assert(kp_);
    *kp_ = cmd;
  }

  void setKd(double cmd) {
    assert(kd_);
    *kd_ = cmd;
  }

  void setFeedforward(double cmd) {
    assert(ff_);
    *ff_ = cmd;
  }

  void setCommand(int state_sec, int state_nsec, int cmd_sec, int cmd_nsec, double pos_des, double vel_des, double kp, double kd, double ff) {
    setStateTimestamp(state_sec, state_nsec);
    setCmdTimestamp(cmd_sec, cmd_nsec);
    setPositionDesired(pos_des);
    setVelocityDesired(vel_des);
    setKp(kp);
    setKd(kd);
    setFeedforward(ff);
  }

  std::array<int, 2> getStateTimestamp(){
    assert(state_sec_);
    assert(state_nsec_);
    return {(*state_sec_), (*state_nsec_)};
  }

  std::array<int, 2> getCmdTimestamp(){
    assert(cmd_sec_);
    assert(cmd_nsec_);
    return {(*cmd_sec_), (*cmd_nsec_)};
  }

  double getPositionDesired() {
    assert(posDes_);
    return *posDes_;
  }

  double getVelocityDesired() {
    assert(velDes_);
    return *velDes_;
  }

  double getKp() {
    assert(kp_);
    return *kp_;
  }

  double getKd() {
    assert(kd_);
    return *kd_;
  }
  
  double getFeedforward() {
    assert(ff_);
    return *ff_;
  }

 private:
  int* state_sec_ = {nullptr};
  int* state_nsec_ = {nullptr};
  int* cmd_sec_ = {nullptr};
  int* cmd_nsec_ = {nullptr};
  double* posDes_ = {nullptr};
  double* velDes_ = {nullptr};
  double* kp_ = {nullptr};
  double* kd_ = {nullptr};
  double* ff_ = {nullptr};
};

class ImpedanceJointInterface : public hardware_interface::HardwareResourceManager<ImpedanceJointHandle, hardware_interface::ClaimResources> {};

}  // namespace legged
