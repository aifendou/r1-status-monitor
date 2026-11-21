#pragma once
#include "r1_status_ros/state_store.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <chrono>

#include <functional>
#include <memory>
#include <string>

using namespace std;
using namespace placeholders;

namespace r1_monitor{
class OdomMonitor : public rclcpp::Node{
public:
    OdomMonitor(shared_ptr<StateStore> store
        ,const string& topic
        ,double timeout_sec)
        :rclcpp::Node("r1_odom_monitor"),
         store_(store),
         timeout_sec_(timeout_sec)
    {
        odom_subscription_ =this->create_subscription<nav_msgs::msg::Odometry>(
            topic,rclcpp::QoS(10),bind(&OdomMonitor::onOdom,this,_1));
        check_timer_ =this->create_wall_timer(
            chrono::seconds(1),bind(&OdomMonitor::checkTimeout,this));
        RCLCPP_INFO(
            this->get_logger(),
            "[WAITING] Topic: %s, timeout: %.1f s",
            topic.c_str(),
            timeout_sec_);
    }
private:
    // 里程计消息到达后，由ROS 2执行器调用，回调函数
    void onOdom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        // 更新共享StateStore
        const bool accepted =store_->update(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->twist.twist.linear.x,
            msg->twist.twist.angular.z
        );
        if(!accepted){
            RCLCPP_WARN(
                this->get_logger(),
                "Rejected odom containing NaN or infinity.");
        }
    }
    // 定时检查数据年龄，并输出缓存状态,回调函数
    void checkTimeout(){
        // 取得完整副本，锁已经在snapshot()返回前释放
        const auto data =store_->snapshot();
        if(!data.received_){
            return;
        }
        //当前时刻减去最后接收时刻，转换为浮点秒数。
        const double age_sec =chrono::duration<double>(
                SteadyClock::now() - data.last_receive_time_).count();
        if(age_sec > timeout_sec_){
            // 同一段超时期间只打印一次告警。
            if(!timed_out_){
                RCLCPP_WARN(
                    this->get_logger(),
                    "[STALE] No accepted odom for %.2f s.",
                    age_sec);
            }
            timed_out_ = true;
            return;
        }
        // 数据重新变得新鲜，打印一次恢复提示
        if(timed_out_){
            RCLCPP_INFO(
                this->get_logger(),
                "[RECOVERED] Odom reception resumed.");
        }
        timed_out_ = false;
        RCLCPP_INFO(
            this->get_logger(),
            "[FRESH] x=%.2f, y=%.2f, vx=%.2f, wz=%.2f, age=%.2f s",
            data.x_,
            data.y_,
            data.linear_velocity_,
            data.angular_velocity_,
            age_sec);
        
    }
    // 与TCP服务端共同持有同一个StateStore
    shared_ptr<StateStore> store_;
    // 超时阈值
    double timeout_sec_;
    // 标记上一轮检查是否已经处于超时状态
    bool timed_out_ = false;
    // 保存订阅对象，让它在节点运行期间持续存在。
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::TimerBase::SharedPtr check_timer_;

};
}