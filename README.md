# R1 Status Monitor

基于C++、Muduo和ROS 2的机器人状态采集与TCP监测项目。

## 当前功能

- TCP状态查询及自动重连
- 长度字段消息协议
- ROS 2里程计订阅
- 线程安全状态缓存
- 数据超时检测
- 多客户端查询
- 按进程独立保存客户端日志

## 目录

- network_demo：独立TCP客户端、服务端和协议测试
- ros_ws/src/r1_status_ros：ROS 2与TCP整合服务端

## 环境

- Ubuntu
- ROS 2 Humble
- C++17
- CMake、colcon
- 官方Muduo网络库

## 当前验证范围

已完成本机模拟ROS数据测试。
尚未接入R1实机。

## 说明

Muduo需要在目标环境中单独准备。
R1_INTERFACE_DIR="/home/rpp/r1_projects/检验文件夹/接口盘点_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$R1_INTERFACE_DIR"

ros2 node list | tee "$R1_INTERFACE_DIR/nodes.txt"
ros2 topic list -t | tee "$R1_INTERFACE_DIR/topics.txt"
ros2 service list -t | tee "$R1_INTERFACE_DIR/services.txt"
ros2 action list -t | tee "$R1_INTERFACE_DIR/actions.txt"


ros2 topic info /odom -v
ros2 topic info /battery_state -v
ros2 topic info /device_state -v
ros2 topic info /error_state -v


ros2 interface show rpp_msgs/msg/BatteryState
ros2 interface show rpp_msgs/msg/DeviceState
ros2 interface show rpp_msgs/msg/ErrorState


timeout 8s ros2 topic echo /battery_state --once
timeout 8s ros2 topic echo /device_state --once
timeout 8s ros2 topic echo /error_state --once


# 创建一个新的采集目录。
CHECK_DIR="$HOME/r1_projects/检验/状态样例_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$CHECK_DIR"

# 查看机械臂消息字段。
ros2 interface show rm_ros_interfaces/msg/Jointerrorcode \
  > "$CHECK_DIR/Jointerrorcode.txt"

ros2 interface show rm_ros_interfaces/msg/Rmerr \
  > "$CHECK_DIR/Rmerr.txt"

ros2 interface show rm_ros_interfaces/msg/Armcurrentstatus \
  > "$CHECK_DIR/Armcurrentstatus.txt"


  for topic in \
  /battery_state \
  /device_state \
  /error_state \
  /diagnostics \
  /joint_states \
  /left/rm_driver/udp_joint_error_code \
  /right/rm_driver/udp_joint_error_code \
  /left/rm_driver/udp_rm_err \
  /right/rm_driver/udp_rm_err \
  /left/rm_driver/udp_arm_current_status \
  /right/rm_driver/udp_arm_current_status
do
  name="${topic#/}"
  name="${name//\//_}"

  # 保存发布者信息、消息类型和QoS。
  ros2 topic info "$topic" -v \
    > "$CHECK_DIR/${name}_info.txt" 2>&1

  # 最多等待8秒，保存一条实际消息。
  timeout 8s ros2 topic echo "$topic" --once \
    > "$CHECK_DIR/${name}_sample.txt" 2>&1
done


ros2 pkg prefix rpp_msgs


cd /home/rpp/r1_projects

wget --timeout=20 --tries=2 \
  -O r1-status-monitor-latest.tar.gz \
  https://codeload.github.com/aifendou/r1-status-monitor/tar.gz/refs/heads/main

  mkdir /home/rpp/r1_projects/r1-status-monitor-test

  tar -xzf /home/rpp/r1_projects/r1-status-monitor-latest.tar.gz \
  -C /home/rpp/r1_projects/r1-status-monitor-test \
  --strip-components=1

  ls /home/rpp/r1_projects/r1-status-monitor-test


  source /opt/ros/humble/setup.bash

source /home/rpp/rpp_ws/install/setup.bash

cd /home/rpp/r1_projects/r1-status-monitor-test/ros_ws

colcon build --packages-select r1_status_ros \
  --cmake-args \
  -DCMAKE_PREFIX_PATH=/home/rpp/r1_projects/r1_deps/release-install-cpp11
``

source /home/rpp/r1_projects/r1-status-monitor-test/ros_ws/install/setup.bash

ros2 run r1_status_ros r1_status_server \
  --ros-args -r /demo/odom:=/odom

cd /home/rpp/r1_projects/r1-status-monitor-test/network_demo

cmake -S . -B build \
  -DCMAKE_PREFIX_PATH=/home/rpp/r1_projects/r1_deps/release-install-cpp11

cmake --build build -j2

./build/r1_status_client logs

source /opt/ros/humble/setup.bash
source /home/rpp/rpp_ws/install/setup.bash

ros2 topic echo /battery_state --once

ros2 topic echo /device_state --once

ros2 topic echo /error_state --once


source /opt/ros/humble/setup.bash
source /home/rpp/rpp_ws/install/setup.bash

timeout 8s ros2 topic echo /left/rm_driver/udp_joint_error_code --once

timeout 8s ros2 topic echo /right/rm_driver/udp_joint_error_code --once
timeout 8s ros2 topic echo /left/rm_driver/udp_rm_err --once
timeout 8s ros2 topic echo /right/rm_driver/udp_rm_err --once

//下次去的要运行的指令
ros2 topic echo /joint_states --once
ros2 topic info /joint_states -v

ros2 param get /left/crt_gripper_driver gripper_joint
ros2 param get /right/crt_gripper_driver gripper_joint
ros2 topic info /joint_states -v


source /opt/ros/humble/setup.bash
source /home/rpp/rpp_ws/install/setup.bash

mkdir -p "$HOME/r1_projects/检验文件夹/demo_20261126_01"

cd "$HOME/r1_projects/检验文件夹/demo_20261126_01"



PYTHONUNBUFFERED=1 ros2 topic echo /rosout rcl_interfaces/msg/Log \
  --qos-reliability reliable \
  --qos-durability volatile \
  --qos-depth 1000 \
  --full-length \
  > rosout_full.txt \
  2> rosout_capture_errors.txt


  source /opt/ros/humble/setup.bash
source /home/rpp/rpp_ws/install/setup.bash

cd "$HOME/r1_projects/检验文件夹/demo_20261126_01"


ros2 node list -a > nodes_before.txt

ros2 topic list -t --include-hidden-topics > topics_before.txt

ros2 service list -t > services_before.txt

ros2 action list -t > actions_before.txt

ros2 node info /bt_demo > bt_demo_info.txt 2>&1

ros2 param dump /bt_demo > bt_demo_parameters.yaml 2>&1

date -Is > before_task_start_time.txt

ros2 node list -a > nodes_running.txt

ros2 topic list -t --include-hidden-topics > topics_running.txt

ros2 service list -t > services_running.txt

ros2 action list -t > actions_running.



date -Is > task_end_observed_time.txt

ros2 node list -a > nodes_after.txt

ros2 topic list -t --include-hidden-topics > topics_after.txt

ros2 service list -t > services_after.txt

ros2 action list -t > actions_after.txt

grep -n -E \
  'Behavior tree started|BT stopped|Conveyor|conveyor|Photoelectric|PLC|PickPallet' \
  rosout_full.txt



rpp<robot>:~/Desktop$ ros2 topic info /joint_states -v
Type: sensor_msgs/msg/JointState

Publisher count: 5

Node name: joint_state_publisher
Node namespace: /
Topic type: sensor_msgs/msg/JointState
Endpoint type: PUBLISHER
GID: 01.0f.6f.3b.28.25.e9.54.00.00.00.00.00.00.13.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Node name: motor_lift_ros_node
Node namespace: /
Topic type: sensor_msgs/msg/JointState
Endpoint type: PUBLISHER
GID: 01.0f.6f.3b.30.25.07.a9.00.00.00.00.00.00.1a.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Node name: motor_pitch_ros_node
Node namespace: /
Topic type: sensor_msgs/msg/JointState
Endpoint type: PUBLISHER
GID: 01.0f.6f.3b.32.25.41.2f.00.00.00.00.00.00.1a.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Node name: _NODE_NAME_UNKNOWN_
Node namespace: _NODE_NAMESPACE_UNKNOWN_
Topic type: sensor_msgs/msg/JointState
Endpoint type: PUBLISHER
GID: 01.0f.ab.f1.f4.7e.d5.fe.00.00.00.00.00.00.bb.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Node name: _NODE_NAME_UNKNOWN_
Node namespace: _NODE_NAMESPACE_UNKNOWN_
Topic type: sensor_msgs/msg/JointState
Endpoint type: PUBLISHER
GID: 01.0f.ab.f1.f8.7e.2e.7c.00.00.00.00.00.00.bb.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Subscription count: 3

Node name: robot_state_publisher
Node namespace: /
Topic type: sensor_msgs/msg/JointState
Endpoint type: SUBSCRIPTION
GID: 01.0f.6f.3b.26.25.6d.03.00.00.00.00.00.00.15.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: BEST_EFFORT
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Node name: joint_state_publisher
Node namespace: /
Topic type: sensor_msgs/msg/JointState
Endpoint type: SUBSCRIPTION
GID: 01.0f.6f.3b.28.25.e9.54.00.00.00.00.00.00.12.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Node name: move_group_private_105768178178576
Node namespace: /
Topic type: sensor_msgs/msg/JointState
Endpoint type: SUBSCRIPTION
GID: 01.0f.6f.3b.6f.26.ad.69.00.00.00.00.00.00.27.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: BEST_EFFORT
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite
timeout 10s ros2 topic echo /joint_states 

ros2 node info /left/crt_gripper_driver
ros2 node info /right/crt_gripper_driver
rpp<robot>:~/Desktop$ ros2 node info /left/crt_gripper_driver
ros2 node info /right/crt_gripper_driver
/left/crt_gripper_driver
  Subscribers:
    /left/set_gripper_pos: std_msgs/msg/Float32
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /joint_states: sensor_msgs/msg/JointState
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /left/crt_gripper_driver/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /left/crt_gripper_driver/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /left/crt_gripper_driver/get_parameters: rcl_interfaces/srv/GetParameters
    /left/crt_gripper_driver/list_parameters: rcl_interfaces/srv/ListParameters
    /left/crt_gripper_driver/set_parameters: rcl_interfaces/srv/SetParameters
    /left/crt_gripper_driver/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
    /left/set_gripper_pos_srv: crt_gripper_driver/srv/CRTGripperset
  Service Clients:

  Action Servers:
    /left/gripper_controller/gripper_cmd: control_msgs/action/GripperCommand
  Action Clients:

/right/crt_gripper_driver
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /right/set_gripper_pos: std_msgs/msg/Float32
  Publishers:
    /joint_states: sensor_msgs/msg/JointState
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /right/crt_gripper_driver/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /right/crt_gripper_driver/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /right/crt_gripper_driver/get_parameters: rcl_interfaces/srv/GetParameters
    /right/crt_gripper_driver/list_parameters: rcl_interfaces/srv/ListParameters
    /right/crt_gripper_driver/set_parameters: rcl_interfaces/srv/SetParameters
    /right/crt_gripper_driver/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
    /right/set_gripper_pos_srv: crt_gripper_driver/srv/CRTGripperset
  Service Clients:

  Action Servers:
    /right/gripper_controller/gripper_cmd: control_msgs/action/GripperCommand
  Action Clients:

rpp<robot>:~/Desktop$ 

