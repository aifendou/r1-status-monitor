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
不提交build、install、运行日志和第三方库二进制文件。