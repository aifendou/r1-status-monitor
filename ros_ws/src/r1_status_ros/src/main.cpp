/**
 * 创建共享对象、启动 ROS 和 Muduo、处理退出
 */
#include "r1_status_ros/state_store.hpp"
#include "r1_status_ros/robot_monitor.hpp"
#include "r1_status_ros/tcp_status_server.hpp"
#include "r1_status_ros/FaultManager.hpp"
#include "FileLogger.h"

#include <rclcpp/rclcpp.hpp>
#include <muduo/net/EventLoop.h>
#include <muduo/net/InetAddress.h>

#include <atomic>
#include <exception>
#include <iostream>
#include <memory>
#include <thread>

int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    // 服务端使用server角色，文件名为r1_server_日期.log。
    // 路径相对于启动程序时的工作目录。
    FileLogger logger("logs", "server");

    logger.writeRuntime(
        r1_monitor::Severity::Info,
        "START",
        "R1监测服务端启动，运行日志=" + logger.path());
    // 当前实验配置，ROS和TCP使用相同超时阈值
    const double timeout_sec = 2.0;
    // 只创建一份缓存，让两个模块共享
    auto store =std::make_shared<r1_monitor::StateStore>();
    // 只创建一份故障管理对象，由ROS节点和TCP服务端共享。
    auto fault_manager =std::make_shared<r1_monitor::FaultManager>(logger);
    auto node =std::make_shared<r1_monitor::RobotMonitor>(store,"/demo/odom",timeout_sec);
    // Muduo主EventLoop必须在运行它的主线程中创建。
    muduo::net::EventLoop loop;
    muduo::net::InetAddress address("0.0.0.0", 9090);
    r1_monitor::TcpStatusServer server(
        &loop,
        address,
        store,//共享缓存
        timeout_sec);//超时事件
    /*
     * Ctrl+C由ROS 2的信号处理机制请求关闭ROS。
     * 主EventLoop定期观察ROS状态，然后结束自身循环。
     * 这样不会出现ROS已经退出，但Muduo仍一直等待的情况。
     */
    loop.runEvery(0.2, [&loop]()
    {
        if (!rclcpp::ok())
        {
            loop.quit();
        }
    });
    server.start();
    // 记录ROS执行线程是否因为异常而退出
    std::atomic<bool> ros_failed{false};
    // ROS回调在这条线程中执行
    std::thread ros_thread([node,&ros_failed](){
        try
        {
            rclcpp::spin(node);
        }
        catch (const std::exception& error){
            ros_failed.store(true);

            std::cerr
                << "[R1服务端] ROS执行异常："
                << error.what()
                << std::endl;
            // 让主EventLoop也能观察到退出请求。
            rclcpp::shutdown();
        }
    });
    std::cout
        << "[R1服务端] 监听0.0.0.0:9090"
        << "，订阅/demo/odom"
        << std::endl;
    // 主线程运行Muduo
    loop.loop();
     // 请求ROS停止，并等待ROS线程结束
    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
    if (ros_thread.joinable())
    {
        ros_thread.join();
    }
    logger.writeRuntime(
        r1_monitor::Severity::Info,
        "STOP",
        "R1监测服务端退出");
    return ros_failed.load() ? 1 : 0;
}