#pragma once

#include <legged_hil_lcm/contacts_msg_t.hpp>
#include <legged_hil_lcm/imu_msg_t.hpp>
#include <legged_hil_lcm/joints_msg_t.hpp>
#include <legged_hil_lcm/joints_cmd_t.hpp>
#include <legged_hil_hw/LeggedHilHW.h>
#include <lcm/lcm-cpp.hpp>

#include <vector>
#include <unordered_map>

#include <mutex>
#include <thread>
#include <atomic>

namespace legged {

const std::vector<std::string> CONTACT_SENSOR_NAMES = {"RF_FOOT", "LF_FOOT", "RH_FOOT", "LH_FOOT"};

struct LcmJointData {
    int sec;
    int nsec;
    std::string name;
    double pos_, vel_, tau_;                 // state
};

struct LcmJointCmd {
    int sec;
    int nsec;
    std::string name;
    double posDes_, velDes_, kp_, kd_, ff_;  // command
};

struct LcmImuData {
    int sec;
    int nsec;
    std::string frame_id;
    double orientation[4];
    double orientation_covariance[9];
    double angular_velocity[3];
    double angular_velocity_covariance[9];
    double linear_acceleration[3];
    double linear_acceleration_covariance[9];
};

struct LcmContactData {
    int sec;
    int nsec;
    std::string name;
    bool isContact;
};

class LcmHW : public LeggedHilHW {
    public:
        LcmHW() = default;

        ~LcmHW() override;

        bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;

        void read(const ros::Time& time, const ros::Duration& period) override;

        void write(const ros::Time& time, const ros::Duration& period) override;

    private:
        bool setupJoints();

        bool setupImu();

        bool setupContactSensor(ros::NodeHandle& nh);

        std::mutex mtx_;
        LcmJointData jointData_[12]{};
        LcmJointCmd jointCmd_[12]{};
        LcmImuData imuData_{};
        LcmContactData contactData_[4]{};
        std::unordered_map<std::string, int> jointNameToIndex_;
        std::unordered_map<std::string, int> contactNameToIndex_;
        
        double state_delay_{0.5};
        double cmd_delay_{0.5};
        ros::Time lastStateTime_{ros::Time(0)};
        bool state_timeout_{false};
        bool cmd_timeout_{false};

        // LCM
        lcm::LCM lcm_;
        std::string ch_joints_{"JOINTS_MSG"};
        std::string ch_imu_{"IMU_MSG"};
        std::string ch_contacts_{"CONTACTS_MSG"};
        std::string ch_cmd_{"JOINT_CMD"}; 

        // ---- LCM callbacks ----
        void onJoints(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const legged_hil_lcm::joints_msg_t* msg);
        void onImu(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const legged_hil_lcm::imu_msg_t* msg);
        void onContacts(const lcm::ReceiveBuffer* rbuf, const std::string& chan,const legged_hil_lcm::contacts_msg_t* msg);
        void lcmSpin();  // handleTimeout loop

        lcm::Subscription* sub_joints_{nullptr};
        lcm::Subscription* sub_imu_{nullptr};
        lcm::Subscription* sub_contacts_{nullptr};

        // ---- threading ----
        std::atomic<bool> th_running_{false};
        std::thread th_lcm_;
        bool parseCpuList(const std::string& cpuListStr, cpu_set_t& cpuset);
        bool setThreadAffinity(std::thread& th, const cpu_set_t& cpuset);
};

}  // namespace legged
