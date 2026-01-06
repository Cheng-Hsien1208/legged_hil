# legged_control with HIL
TODO

## Introduction
TODO

## Installation
```
# Clone in src folder
git clone git@github.com:Cheng-Hsien1208/legged_hil.git
```
clone後自行將meshes放入description資料夾中（meshes檔案太大無法上傳github）



### Build
```
# Install dependencies
sudo apt install liburdfdom-dev liboctomap-dev libassimp-dev
sudo apt install ros-noetic-joy
```

```
# ocs2
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin build ocs2_legged_robot_ros ocs2_self_collision_visualization
```

```
# legged contol
catkin build legged_controllers legged_unitree_description
catkin build legged_gazebo
```

```
# legged hil Simulator
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin build legged_hil_gazebo
```

```
# legged hil Controller
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin build legged_hil_hw legged_hil_lcm_hw
```

## Simulator 本機網路設定
```
sudo ufw disable
sudo ip link set 網卡名稱 multicast on
sudo ip route add 224.0.0.0/4 dev 網卡名稱
ip route | grep 224
```

## Controller 本機網路設定
```
sudo ufw disable
sudo ip link set 網卡名稱 multicast on
sudo ip route add 224.0.0.0/4 dev 網卡名稱
ip route | grep 224
```

## Simulator ros_ws 使用
```
export ROS_Master_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1

export CLASSPATH=~/ros_ws/src/legged_control_hil/legged_hil_lcm/lcm_java
export LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=1"

export ROBOT_TYPE=alpha
source /opt/ros/noetic/setup.bash
source ~/ros_ws/devel/setup.bash

# 一般模擬器
roslaunch legged_hil_mirle_description empty_world.launch

# 可以pin core & set priority
roslaunch legged_hil_mirle_description rt_empty_world.launch
```

## Controller ros_ws 使用
```
export ROS_Master_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1

export CLASSPATH=~/ros_ws/src/legged_control_hil/legged_hil_lcm/lcm_java
export LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=1"

export ROBOT_TYPE=alpha
source /opt/ros/noetic/setup.bash
source ~/ros_ws/devel/setup.bash

roslaunch legged_hil_hw legged_hil_hw_mirle.launch
roslaunch legged_hil_controllers load_controller.launch
```

## 如何綁定core & 設定 prioity

### Simulator : rt_empty_world.launch
這裡使用一般sched，nice值範圍 = -20 ~ 19

```
<arg name="ros_cpu_list" default="16-31"/>
<arg name="ros_thread_nice" default="-20"/>
```

### Controller : legged_hil_hw/launch/legged_hil_hw_robot_type.launch
這裡使用一般sched，nice值範圍 = -20 ~ 19

```
<arg name="ros_cpu_list" default="16-31"/>
<arg name="ros_thread_nice" default="-20"/>
```

### Controller : legged_hil_controllers/load_controller.launch
這裡使用一般sched，nice值範圍 = -20 ~ 19

```
<arg name="ros_cpu_list" default="16-31"/>
<arg name="ros_thread_nice" default="-20"/>
```

### Controller :  legged_hil_hw/config/robot_type.yaml
這裡使用RT_sched, priority範圍 = 0 ~ 99
cpu_list 必須一一列出並用逗號分隔, 中間不能有空白或 "-" 
```
legged_hil_hw:
  loop_frequency: 1000
  cycle_time_error_threshold: 0.002
  kp: 50.0
  kd: 40.0

  lcm_cpu_list: "2,3"
  wbc_cpu_list: "4,5" 
  mpc_cpu_list: "6,7"
  lcm_thread_priority: 80
  wbc_thread_priority: 95
  mpc_thread_priority: 50
```

### Controller : legged_hil_controllers/config/robot_type/task.info
這裡使用RT_sched, priority範圍 = 0 ~ 99
cpu_list 必須一一列出並用逗號分隔, 中間不能有空白或 "-" 

```
; Multiple_Shooting SQP settings
sqp
{
  .
  .
  .
  .
  .
  .
  .
  threadPriority                        50
  solverCpuList                         8,9
}
```

## Monitor Tools
```
htop
lscpu -e
pidstat -t -p <pid> 1
pidstat -r -human -p <pid> 1