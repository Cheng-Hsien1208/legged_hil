#include "LeggedHilHWLoop.h"
#include <sstream>
#include <vector>
#include <sched.h>


namespace legged {
LeggedHilHWLoop::LeggedHilHWLoop(ros::NodeHandle& nh, std::shared_ptr<LeggedHilHW> hardware_interface)
    : nh_(nh), hardwareInterface_(std::move(hardware_interface)), loopRunning_(true) {
  // Create the controller manager
  controllerManager_.reset(new controller_manager::ControllerManager(hardwareInterface_.get(), nh_));

  // Load ros params
  int error = 0;
  int wbcThreadPriority = 0;
  std::string wbcCpuList_;

  error += static_cast<int>(!nh.getParam("legged_hil_hw/loop_frequency", loopHz_));
  error += static_cast<int>(!nh.getParam("legged_hil_hw/cycle_time_error_threshold", cycleTimeErrorThreshold_));
  error += static_cast<int>(!nh.getParam("legged_hil_hw/wbc_thread_priority", wbcThreadPriority));
  error += static_cast<int>(!nh.getParam("legged_hil_hw/wbc_cpu_list", wbcCpuList_));

  if (error > 0) {
    std::string error_message =
        "could not retrieve one of the required parameters: loop_hz or cycle_time_error_threshold or thread_priority";
    ROS_ERROR_STREAM(error_message);
    throw std::runtime_error(error_message);
  }

  // Get current time for use with first update
  lastTime_ = Clock::now();

  // Setup loop thread
  loopThread_ = std::thread([&]() {
    while (loopRunning_) {
      update();
    }
  });
  sched_param sched{.sched_priority = wbcThreadPriority};
  if (pthread_setschedparam(loopThread_.native_handle(), SCHED_FIFO, &sched) != 0) {
    ROS_WARN(
        "Failed to set threads priority (one possible reason could be that the user and the group permissions "
        "are not set properly.).\n");
  }

  // Set CPU affinity if specified
  if(!wbcCpuList_.empty()) {
    cpu_set_t cpuset;
    if (parseCpuList(wbcCpuList_, cpuset)) {
      if (setThreadAffinity(loopThread_, cpuset)) {
        ROS_INFO_STREAM("WBC thread affinity set to CPUs: " << wbcCpuList_);
      } else {
        ROS_ERROR_STREAM("Failed to set WBC thread affinity to CPUs: " << wbcCpuList_);
      }
    } else {
      ROS_ERROR_STREAM("Invalid WBC CPU list format: " << wbcCpuList_);
    }
  }
  pthread_setname_np(loopThread_.native_handle(), "wbc_thread");
}

void LeggedHilHWLoop::update() {
  const auto currentTime = Clock::now();
  // Compute desired duration rounded to clock decimation
  const Duration desiredDuration(1.0 / loopHz_);

  // Get change in time
  Duration time_span = std::chrono::duration_cast<Duration>(currentTime - lastTime_);
  elapsedTime_ = ros::Duration(time_span.count());
  lastTime_ = currentTime;

  // Check cycle time for excess delay
  const double cycle_time_error = (elapsedTime_ - ros::Duration(desiredDuration.count())).toSec();
  if (cycle_time_error > cycleTimeErrorThreshold_) {
    ROS_WARN_STREAM("Cycle time exceeded error threshold by: " << cycle_time_error - cycleTimeErrorThreshold_ << "s, "
                                                               << "cycle time: " << elapsedTime_ << "s, "
                                                               << "threshold: " << cycleTimeErrorThreshold_ << "s");
  }

  // Input
  // get the hardware's state
  hardwareInterface_->read(ros::Time::now(), elapsedTime_);

  // Control
  // let the controller compute the new command (via the controller manager)
  controllerManager_->update(ros::Time::now(), elapsedTime_);

  // Output
  // send the new command to hardware
  hardwareInterface_->write(ros::Time::now(), elapsedTime_);

  // Sleep
  const auto sleepTill = currentTime + std::chrono::duration_cast<Clock::duration>(desiredDuration);
  std::this_thread::sleep_until(sleepTill);
}

bool LeggedHilHWLoop::parseCpuList(const std::string& cpuListStr, cpu_set_t& cpuset) {
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

bool LeggedHilHWLoop::setThreadAffinity(std::thread& th, const cpu_set_t& cpuset) {
  int ret = pthread_setaffinity_np(th.native_handle(), sizeof(cpu_set_t), &cpuset);
  return (ret == 0);
}

LeggedHilHWLoop::~LeggedHilHWLoop() {
  loopRunning_ = false;
  if (loopThread_.joinable()) {
    loopThread_.join();
  }
}

}  // namespace legged
