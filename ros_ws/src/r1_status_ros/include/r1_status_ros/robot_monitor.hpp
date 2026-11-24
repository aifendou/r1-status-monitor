/**
 * 	创建 ROS 订阅，通过回调更新缓存
 */
#pragma once
#include "r1_status_ros/state_store.hpp"
#include "r1_status_ros/FaultManager.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rpp_msgs/msg/battery_state.hpp>
#include <rpp_msgs/msg/device_state.hpp>
#include <rpp_msgs/msg/error_state.hpp>
#include <chrono>

#include <functional>
#include <memory>
#include <string>
#include <vector>

using namespace std;
using namespace placeholders;

namespace r1_monitor{
class RobotMonitor : public rclcpp::Node{
public:
    RobotMonitor(shared_ptr<StateStore> store
        ,const string& topic
        ,double timeout_sec
        ,shared_ptr<FaultManager> fault_manager = nullptr)
        :rclcpp::Node("r1_robot_monitor"),
         store_(store),
         fault_manager_(fault_manager),
         timeout_sec_(timeout_sec)
    {
        odom_subscription_ =this->create_subscription<nav_msgs::msg::Odometry>(
            topic,rclcpp::QoS(10),bind(&RobotMonitor::onOdom,this,_1));
        battery_subscription_ = this->create_subscription<rpp_msgs::msg::BatteryState>(
            "/battery_state",rclcpp::QoS(10),bind(&RobotMonitor::onBattery,this,_1));
        device_subscription_ = this->create_subscription<rpp_msgs::msg::DeviceState>(
            "/device_state",rclcpp::QoS(10),bind(&RobotMonitor::onDevice,this,_1));
        check_timer_ =this->create_wall_timer(
            chrono::seconds(1),bind(&RobotMonitor::checkTimeout,this));
        RCLCPP_INFO(
            this->get_logger(),
            "[WAITING] Topic: %s, timeout: %.1f s",
            topic.c_str(),
            timeout_sec_);
        if(fault_manager_){
            // 只注册当前已经接入的四类底盘报警。
            fault_manager_->registerFault("chassis", "motor_alarm");
            fault_manager_->registerFault("chassis", "turn_motor_alarm");
            fault_manager_->registerFault("chassis", "bump_alarm");
            fault_manager_->registerFault("chassis", "communication_alarm");
            error_subscription_ =
            this->create_subscription<rpp_msgs::msg::ErrorState>(
                "/error_state",rclcpp::QoS(10),bind(&RobotMonitor::onError,this,_1));
        }
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
    //电池信息
    void onBattery(
        rpp_msgs::msg::BatteryState::ConstSharedPtr msg)
    {
        const bool accepted = store_->updateBattery(
            static_cast<int>(msg->percentage),
            msg->ts);

        if (!accepted)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "Rejected battery state: invalid percentage or timestamp.");
        }
    }
    // 收到设备状态后，只提取数据并更新缓存。
    void onDevice(
        rpp_msgs::msg::DeviceState::ConstSharedPtr msg)
    {
        const bool accepted = store_->updateDevice(
            static_cast<int>(msg->control_mode),
            static_cast<int>(msg->power_mode),
            static_cast<int>(msg->motor_mode),
            static_cast<int>(msg->e_stop),
            msg->ts);

        if (!accepted)
        {
            // 限制告警频率，避免异常消息导致终端不断刷屏。
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "Rejected device state: invalid value or timestamp.");
        }
    }
    // 只组装观察结果，不立即提交。
    // 因此一次厂家消息中的四项数据可以先组合起来。
    FaultObservation makeChassisFault(
        const char* code,
        int raw_value,
        std::int64_t source_ts,
        Severity severity) const
    {
        FaultObservation observation;

        observation.module_ = "chassis";
        observation.code_ = code;

        observation.state_ =
            raw_value == 0
                ? FaultState::Healthy
                : FaultState::Active;

        observation.severity_ = severity;

        observation.detail_ =
            "raw=" + std::to_string(raw_value)
            + " source_ts=" + std::to_string(source_ts);

        // 时间戳变化不代表故障内容变化。
        observation.fingerprint_ =
            std::to_string(raw_value);

        return observation;
    }


    void onError(rpp_msgs::msg::ErrorState::ConstSharedPtr msg)
    {
        if (!fault_manager_)
        {
            return;
        }

        if (msg->ts <= 0)
        {
            fault_manager_->markModuleUnknown("chassis");

            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "Error state has no valid source timestamp.");

            return;
        }

        if (fault_received_
            && msg->ts == last_fault_source_ts_)
        {
            // 厂家重复发布同一份底层缓存，
            // 不刷新数据年龄，也不重复提交。
            return;
        }

        // 记录本次回调观察到新反馈的时刻。
        const auto receive_time = SteadyClock::now();

        std::vector<FaultObservation> observations;
        observations.reserve(4);

        observations.push_back(
            makeChassisFault(
                "motor_alarm",
                static_cast<int>(msg->motor_error),
                msg->ts,
                Severity::Error));

        observations.push_back(
            makeChassisFault(
                "turn_motor_alarm",
                static_cast<int>(msg->turn_motor_error),
                msg->ts,
                Severity::Error));

        observations.push_back(
            makeChassisFault(
                "bump_alarm",
                static_cast<int>(msg->bump_error),
                msg->ts,
                Severity::Warning));

        observations.push_back(
            makeChassisFault(
                "communication_alarm",
                static_cast<int>(msg->communication_error),
                msg->ts,
                Severity::Error));

        // 四项反馈在同一个批次中更新。
        fault_manager_->observeBatch(observations);

        // 成功处理后更新本地接收记录。
        fault_received_ = true;
        last_fault_source_ts_ = msg->ts;
        last_fault_update_time_ = receive_time;
    }

    void checkFaultTimeout()
    {
        if (!fault_manager_)
        {
            return;
        }

        if (!fault_received_)
        {
            // 已注册项目本来就是Unknown。
            return;
        }

        const double age_sec =
            std::chrono::duration<double>(
                SteadyClock::now() - last_fault_update_time_)
                .count();

        if (age_sec > fault_timeout_sec_)
        {
            fault_manager_->markModuleUnknown("chassis");
        }
    }
    // 定时检查数据年龄，并输出缓存状态,回调函数
    void checkTimeout(){
        // 放在里程计的提前return之前。
        checkFaultTimeout();
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
    shared_ptr<FaultManager> fault_manager_;
    // 超时阈值
    double timeout_sec_;
    // 标记上一轮检查是否已经处于超时状态
    bool timed_out_ = false;
    bool fault_received_ = false;
    std::int64_t last_fault_source_ts_ = 0;
    SteadyClock::time_point last_fault_update_time_{};
    // 暂定5秒，后续结合实机底层更新周期调整。
    const double fault_timeout_sec_ = 5.0;
    // 保存订阅对象，让它在节点运行期间持续存在。
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<rpp_msgs::msg::BatteryState>::SharedPtr battery_subscription_;
    rclcpp::Subscription<rpp_msgs::msg::DeviceState>::SharedPtr device_subscription_;
    rclcpp::Subscription<rpp_msgs::msg::ErrorState>::SharedPtr error_subscription_;
    rclcpp::TimerBase::SharedPtr check_timer_;

};
}