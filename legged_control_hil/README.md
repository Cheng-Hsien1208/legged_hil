# LeggedHIL 使用說明
內容分作兩部份，模擬器端只需要legged_control_hil單一project即可，控制器端須配合ocs2+legged_control。

## Simulator 本機網路設定
sudo ufw disable
sudo ip link set 網卡名稱 multicast on
sudo ip route add 224.0.0.0/4 dev 網卡名稱
ip route | grep 224

## Simulator ros_ws 編譯
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin build legged_hil_sim

## Simulator ros_ws 使用
export ROS_Master_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1

export CLASSPATH=~/ros_ws/src/legged_control_hil/legged_hil_lcm/lcm_java
export LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=1"

export ROBOT_TYPE=alpha
source /opt/ros/noetic/setup.bash
source ~/ros_ws/devel/setup.bash

roslaunch legged_hil_mirle_description empty_world.launch


## Controller 本機網路設定
sudo ufw disable
sudo ip link set 網卡名稱 multicast on
sudo ip route add 224.0.0.0/4 dev 網卡名稱
ip route | grep 224

## Controller 編譯
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin build legged_hil_hw legged_hil_lcm_hw
 
## Controller ros_ws 使用
export ROS_Master_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1

export CLASSPATH=~/ros_ws/src/legged_control_hil/legged_hil_lcm/lcm_java
export LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=1"

export ROBOT_TYPE=alpha
source /opt/ros/noetic/setup.bash
source ~/ros_ws/devel/setup.bash

roslaunch legged_hil_hw legged_hil_hw_mirle.launch
roslaunch legged_hil_controllers load_controller.launch
