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
#include <rm_ros_interfaces/msg/jointerrorcode.hpp>
#include <rm_ros_interfaces/msg/rmerr.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/message_info.hpp>
#include <chrono>

#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>  // 排序、删除零值、去重。
#include <cstdint>    // std::int32_t。
#include <locale>
#include <sstream>    // 将错误码数组整理为文字。
#include <cmath>
#include <stdexcept>

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
        // 接入左右机械臂。
        // 即使不启用故障管理，订阅仍然可以更新状态缓存。
        initArmMonitoring();
        initGripperMonitoring();
        initCameraMonitoring();
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
    // ============================================================
    // 机械臂监测：注册、订阅、缓存更新和故障提交。
    // ============================================================

    // 将方向转换为统一的故障模块名称。
    static const char* armModule(ArmSide side)
    {
        return side == ArmSide::Left
            ? "left_arm"
            : "right_arm";
    }

    // 创建左右机械臂的四个订阅。
    // 创建左右机械臂的四个订阅。
    // 由RobotMonitor构造函数调用一次。
    void initArmMonitoring()
    {
        if (fault_manager_)
        {
            fault_manager_->registerFault(
                "left_arm", "joint_error");

            fault_manager_->registerFault(
                "right_arm", "joint_error");

            fault_manager_->registerFault(
                "left_arm", "system_error");

            fault_manager_->registerFault(
                "right_arm", "system_error");
        }

        // 左臂：关节错误码。
        left_arm_joint_subscription_ =
            this->create_subscription<
                rm_ros_interfaces::msg::Jointerrorcode>(
                "/left/rm_driver/udp_joint_error_code",
                rclcpp::QoS(10),
                [this](
                    rm_ros_interfaces::msg::Jointerrorcode::ConstSharedPtr msg)
                {
                    // ROS只传入msg。
                    // 左右方向由我们在函数内部明确指定。
                    this->onArmJointError(msg, ArmSide::Left);
                });

        // 右臂：关节错误码。
        right_arm_joint_subscription_ =
            this->create_subscription<
                rm_ros_interfaces::msg::Jointerrorcode>(
                "/right/rm_driver/udp_joint_error_code",
                rclcpp::QoS(10),
                [this](
                    rm_ros_interfaces::msg::Jointerrorcode::ConstSharedPtr msg)
                {
                    this->onArmJointError(msg, ArmSide::Right);
                });

        // 左臂：系统错误码。
        left_arm_system_subscription_ =
            this->create_subscription<
                rm_ros_interfaces::msg::Rmerr>(
                "/left/rm_driver/udp_rm_err",
                rclcpp::QoS(10),
                [this](
                    rm_ros_interfaces::msg::Rmerr::ConstSharedPtr msg)
                {
                    this->onArmSystemError(msg, ArmSide::Left);
                });

        // 右臂：系统错误码。
        right_arm_system_subscription_ =
            this->create_subscription<
                rm_ros_interfaces::msg::Rmerr>(
                "/right/rm_driver/udp_rm_err",
                rclcpp::QoS(10),
                [this](
                    rm_ros_interfaces::msg::Rmerr::ConstSharedPtr msg)
                {
                    this->onArmSystemError(msg, ArmSide::Right);
                });
    }
    // 将错误码数组整理为文字。
    // 例如：[0,0,12,0,0,0]。
    static std::string armCodesText(
        const std::vector<std::int32_t>& codes)
    {
        std::ostringstream text;
        text.imbue(std::locale::classic());

        text << "[";

        for (std::size_t i = 0; i < codes.size(); ++i)
        {
            if (i != 0)
            {
                text << ",";
            }

            text << codes[i];
        }

        text << "]";
        return text.str();
    }

    // 将有效反馈交给故障管理器。
    // accepted表示StateStore是否接受了这条消息。
    void reportArmFeedback(
        ArmSide side,
        const char* code,
        const std::vector<std::int32_t>& raw_codes,
        bool accepted)
    {
        const char* module = armModule(side);

        if (!accepted)
        {
            if (fault_manager_)
            {
                fault_manager_->markFaultUnknown(module, code);
            }

            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "[ARM INVALID] %s/%s: invalid array length or value.",
                module,
                code);

            return;
        }

        if (!fault_manager_)
        {
            return;
        }

        FaultObservation observation;
        observation.module_ = module;
        observation.code_ = code;
        observation.state_ = FaultState::Healthy;
        observation.severity_ = Severity::Error;

        // 当前按是否存在非零错误码判断是否报告报警。
        // 具体错误原因仍保留厂家原始码，不自行解释。
        for (const auto value : raw_codes)
        {
            if (value != 0)
            {
                observation.state_ = FaultState::Active;
                break;
            }
        }

        // 详情保留原始顺序。
        observation.detail_ = "raw=" + armCodesText(raw_codes);

        auto comparison_codes = raw_codes;

        if (observation.code_ == "system_error")
        {
            // 系统错误按集合比较：
            // [12,35]和[35,12]不应被当成两种不同的报警。
            comparison_codes.erase(
                std::remove(
                    comparison_codes.begin(),
                    comparison_codes.end(),
                    0),
                comparison_codes.end());

            std::sort(
                comparison_codes.begin(),
                comparison_codes.end());

            comparison_codes.erase(
                std::unique(
                    comparison_codes.begin(),
                    comparison_codes.end()),
                comparison_codes.end());
        }

        // 关节数组保留顺序：不同下标对应不同关节。
        observation.fingerprint_ = armCodesText(comparison_codes);

        // 内部负责报警出现、变化、解除和故障日志记录。
        fault_manager_->observe(observation);
    }

    // 收到左臂或右臂关节错误码。
    void onArmJointError(
        rm_ros_interfaces::msg::Jointerrorcode::ConstSharedPtr msg,
        ArmSide side)
    {
        const std::vector<std::int32_t> codes(
            msg->joint_error.begin(),
            msg->joint_error.end());

        // 先更新原始状态缓存。
        const bool accepted = store_->updateArmJoint(
            side,
            static_cast<int>(msg->dof),
            codes);

        // 再将本次观察交给故障管理。
        reportArmFeedback(
            side,
            "joint_error",
            codes,
            accepted);
    }

    // 收到左臂或右臂系统错误码。
    void onArmSystemError(
        rm_ros_interfaces::msg::Rmerr::ConstSharedPtr msg,
        ArmSide side)
    {
        const std::vector<std::int32_t> codes(
            msg->err.begin(),
            msg->err.end());

        const bool accepted = store_->updateArmSystem(
            side,
            static_cast<int>(msg->err_len),
            codes);

        reportArmFeedback(
            side,
            "system_error",
            codes,
            accepted);
    }
    // ============================================================
    // 夹爪监测。
    // ============================================================

    static const char* gripperModule(GripperSide side)
    {
        return side == GripperSide::Left
            ? "left_gripper"
            : "right_gripper";
    }

    // 创建订阅并注册反馈超时监测项。
    void initGripperMonitoring()
    {
        // 参数在程序启动时读取。
        // 默认值来自厂家参考源码，部署时可以通过ROS参数覆盖。
        gripper_joint_states_topic_ =
            this->declare_parameter<std::string>(
                "gripper_joint_states_topic",
                "/joint_states");

        left_gripper_joint_ =
            this->declare_parameter<std::string>(
                "left_gripper_joint",
                "left_R2");

        right_gripper_joint_ =
            this->declare_parameter<std::string>(
                "right_gripper_joint",
                "right_R2");

        left_gripper_publisher_ =
            this->declare_parameter<std::string>(
                "left_gripper_publisher",
                "/left/crt_gripper_driver");

        right_gripper_publisher_ =
            this->declare_parameter<std::string>(
                "right_gripper_publisher",
                "/right/crt_gripper_driver");

        if (gripper_joint_states_topic_.empty()
            || left_gripper_joint_.empty()
            || right_gripper_joint_.empty()
            || left_gripper_publisher_.empty()
            || right_gripper_publisher_.empty())
        {
            throw std::invalid_argument(
                "Gripper topic, joint and publisher names must not be empty.");
        }

        if (fault_manager_)
        {
            fault_manager_->registerFault(
                "left_gripper", "feedback_timeout");

            fault_manager_->registerFault(
                "right_gripper", "feedback_timeout");
        }

        gripper_subscription_ =
            this->create_subscription<sensor_msgs::msg::JointState>(
                gripper_joint_states_topic_,
                rclcpp::QoS(10),
                [this](
                    sensor_msgs::msg::JointState::ConstSharedPtr msg,
                    const rclcpp::MessageInfo& info)
                {
                    this->onGripperJointState(msg, info);
                });

        RCLCPP_INFO(
            this->get_logger(),
            "[GRIPPER CONFIG] topic=%s; "
            "left=%s from %s; right=%s from %s",
            gripper_joint_states_topic_.c_str(),
            left_gripper_joint_.c_str(),
            left_gripper_publisher_.c_str(),
            right_gripper_joint_.c_str(),
            right_gripper_publisher_.c_str());
    }

    // 检查这条消息是否来自指定驱动节点。
    //
    // GID是发布者端点标识。
    // 通过“节点名称 + GID”匹配，避免接受其他节点模拟的位置。
    bool fromGripperDriver(
        const rclcpp::MessageInfo& info,
        const std::string& expected_node) const
    {
        const auto publishers =
            this->get_publishers_info_by_topic(
                gripper_joint_states_topic_);

        const auto& received_gid =
            info.get_rmw_message_info().publisher_gid;

        for (const auto& publisher : publishers)
        {
            std::string full_name = publisher.node_namespace();

            if (full_name.empty() || full_name.back() != '/')
            {
                full_name += "/";
            }

            full_name += publisher.node_name();

            if (full_name != expected_node)
            {
                continue;
            }

            const auto& endpoint_gid = publisher.endpoint_gid();

            if (std::equal(
                    endpoint_gid.begin(),
                    endpoint_gid.end(),
                    received_gid.data))
            {
                return true;
            }
        }

        return false;
    }

    // 接收共享话题上的消息，先区分实际发布者。
    void onGripperJointState(
        sensor_msgs::msg::JointState::ConstSharedPtr msg,
        const rclcpp::MessageInfo& info)
    {
        if (fromGripperDriver(info, left_gripper_publisher_))
        {
            updateOneGripper(
                msg,
                GripperSide::Left,
                left_gripper_joint_);
        }
        else if (fromGripperDriver(info, right_gripper_publisher_))
        {
            updateOneGripper(
                msg,
                GripperSide::Right,
                right_gripper_joint_);
        }

        // 其他发布者的消息不用于刷新夹爪反馈时间。
    }

    // 提取指定关节对应的实际位置。
    void updateOneGripper(
        sensor_msgs::msg::JointState::ConstSharedPtr msg,
        GripperSide side,
        const std::string& joint_name)
    {
        // position[i]必须对应name[i]。
        // 本模块需要位置，因此不接受空缺或长度不一致的位置数组。
        if (msg->position.size() != msg->name.size())
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "[GRIPPER INVALID] name/position size mismatch.");

            return;
        }

        // 目标关节必须恰好出现一次。
        if (std::count(
                msg->name.begin(),
                msg->name.end(),
                joint_name) != 1)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "[GRIPPER NAME] Expected joint '%s' once.",
                joint_name.c_str());

            return;
        }

        const auto found = std::find(
            msg->name.begin(),
            msg->name.end(),
            joint_name);

        const std::size_t index =
            static_cast<std::size_t>(
                found - msg->name.begin());

        const bool accepted = store_->updateGripper(
            side,
            joint_name,
            msg->position[index]);

        if (!accepted)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "[GRIPPER INVALID] Non-finite position.");

            return;
        }

        if (fault_manager_)
        {
            FaultObservation observation;

            observation.module_ = gripperModule(side);
            observation.code_ = "feedback_timeout";
            observation.state_ = FaultState::Healthy;
            observation.severity_ = Severity::Warning;

            observation.detail_ =
                "已收到实际夹爪位置反馈，joint=" + joint_name;

            // 位置变化不代表反馈超时故障发生变化。
            observation.fingerprint_ = "feedback_present";

            // 若之前发生过超时，这次有效反馈会产生恢复事件。
            fault_manager_->observe(observation);
        }
    }
    // ============================================================
    // 相机图像反馈监测。
    // ============================================================

    void initCameraMonitoring()
    {
        const auto names =
            this->declare_parameter<std::vector<std::string>>(
                "camera_names",
                std::vector<std::string>{
                    "camera_F",
                    "camera_B",
                    "camera_H",
                    "camera_L",
                    "camera_R"
                });

        const auto topics =
            this->declare_parameter<std::vector<std::string>>(
                "camera_topics",
                std::vector<std::string>{
                    "/camera_F/camera/depth/image_rect_raw",
                    "/camera_B/camera/depth/image_rect_raw",
                    "/camera_H/camera/color/image_raw",
                    "/camera_L/camera/color/image_raw",
                    "/camera_R/camera/color/image_raw"
                });

        if (names.empty()
            || names.size() != topics.size()
            || names.size() > kMaxCameraCount)
        {
            throw std::invalid_argument(
                "Camera names/topics must have matching nonzero sizes.");
        }

        // 传感器数据QoS：BestEffort、Volatile。
        // 队列只保留最近一帧，避免旧帧积压。
        auto qos = rclcpp::SensorDataQoS();
        qos.keep_last(1);

        camera_subscriptions_.reserve(names.size());

        for (std::size_t i = 0; i < names.size(); ++i)
        {
            const std::string name = names[i];
            const std::string topic = topics[i];

            store_->registerCamera(name, topic);

            if (fault_manager_)
            {
                fault_manager_->registerFault(
                    name,
                    "frame_timeout");
            }

            auto subscription =
                this->create_subscription<sensor_msgs::msg::Image>(
                    topic,
                    qos,
                    [this, name](
                        sensor_msgs::msg::Image::ConstSharedPtr msg)
                    {
                        // name按值捕获，每个订阅保存自己的相机名称。
                        this->onCameraImage(msg, name);
                    });

            camera_subscriptions_.push_back(subscription);

            RCLCPP_INFO(
                this->get_logger(),
                "[CAMERA CONFIG] %s -> %s",
                name.c_str(),
                topic.c_str());
        }
    }

    // 接收到图像后，只检查基本结构并提取元数据。
    void onCameraImage(
        sensor_msgs::msg::Image::ConstSharedPtr msg,
        const std::string& name)
    {
        const std::uint64_t expected_bytes =
            static_cast<std::uint64_t>(msg->step)
            * static_cast<std::uint64_t>(msg->height);

        const std::int64_t source_stamp_ns =
            static_cast<std::int64_t>(msg->header.stamp.sec)
                * 1000000000LL
            + static_cast<std::int64_t>(msg->header.stamp.nanosec);

        // Image消息的data长度应等于step * height。
        // 本模块不解码像素，只做基本有效性检查。
        if (msg->width == 0
            || msg->height == 0
            || msg->step == 0
            || msg->encoding.empty()
            || msg->encoding.size() > 64
            || expected_bytes != msg->data.size()
            || msg->header.stamp.nanosec >= 1000000000U
            || source_stamp_ns <= 0)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "[CAMERA INVALID] %s: invalid image structure or timestamp.",
                name.c_str());

            return;
        }

        const bool accepted = store_->updateCamera(
            name,
            msg->width,
            msg->height,
            msg->encoding,
            source_stamp_ns);

        if (!accepted)
        {
            // 包括重复源时间戳。
            // 不刷新缓存时间，也不解除已有超时。
            return;
        }

        if (fault_manager_)
        {
            FaultObservation observation;

            observation.module_ = name;
            observation.code_ = "frame_timeout";
            observation.state_ = FaultState::Healthy;
            observation.severity_ = Severity::Warning;

            observation.detail_ = "已收到有效新图像帧";
            observation.fingerprint_ = "new_frame_received";

            // 如果之前发生过超时，这里会记录恢复事件。
            fault_manager_->observe(observation);
        }
    }

    
    // 检查一侧夹爪反馈。
    void checkOneGripperTimeout(
        const char* module,
        const GripperSnapshot& gripper)
    {
        if (!gripper.received_)
        {
            // 从未收到时，可能尚未启动或配置不匹配。
            // 保持未知，不直接认定夹爪硬件发生故障。
            fault_manager_->markFaultUnknown(
                module,
                "feedback_timeout");

            return;
        }

        const double age_sec =
            std::chrono::duration<double>(
                SteadyClock::now() - gripper.last_receive_time_)
                .count();

        if (age_sec <= kGripperTimeoutSec)
        {
            // 正常判断由有效接收回调提交。
            return;
        }

        FaultObservation observation;

        observation.module_ = module;
        observation.code_ = "feedback_timeout";
        observation.state_ = FaultState::Active;
        observation.severity_ = Severity::Warning;

        observation.detail_ =
            "超过"
            + std::to_string(kGripperTimeoutSec)
            + "秒未收到有效夹爪位置反馈，joint="
            + gripper.joint_name_;

        // 年龄不断增加不属于新的故障内容。
        observation.fingerprint_ = "feedback_missing";

        // 相同超时持续期间，不重复写出现事件。
        fault_manager_->observe(observation);
    }

    void checkGripperTimeouts()
    {
        if (!fault_manager_)
        {
            return;
        }

        const auto grippers = store_->grippersSnapshot();

        checkOneGripperTimeout(
            "left_gripper",
            grippers.left_);

        checkOneGripperTimeout(
            "right_gripper",
            grippers.right_);
    }
    // 检查某一路机械臂反馈是否已经失效。
    void checkOneArmFeedback(
        const char* module,
        const char* code,
        const ArmFeedbackSnapshot& feedback)
    {
        if (!feedback.received_)
        {
            fault_manager_->markFaultUnknown(module, code);
            return;
        }

        const double age_sec =
            std::chrono::duration<double>(
                SteadyClock::now() - feedback.last_receive_time_)
                .count();

        if (age_sec > kArmTimeoutSec)
        {
            // 接收超时表示当前状态未知，不冒充“报警解除”。
            fault_manager_->markFaultUnknown(module, code);
        }
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
    void checkArmFaultTimeouts()
    {
        if (!fault_manager_)
        {
            return;
        }

        const auto arms = store_->armsSnapshot();

        checkOneArmFeedback(
            "left_arm", "joint_error", arms.left_.joint_);

        checkOneArmFeedback(
            "right_arm", "joint_error", arms.right_.joint_);

        checkOneArmFeedback(
            "left_arm", "system_error", arms.left_.system_);

        checkOneArmFeedback(
            "right_arm", "system_error", arms.right_.system_);
    }
    void checkCameraTimeouts()
    {
        if (!fault_manager_)
        {
            return;
        }

        const auto cameras = store_->camerasSnapshot();
        const auto now = SteadyClock::now();

        for (const auto& item : cameras)
        {
            const std::string& name = item.first;
            const CameraSnapshot& camera = item.second;

            if (!camera.received_)
            {
                // 尚未收到有效帧，可能尚未启动或配置不匹配。
                fault_manager_->markFaultUnknown(
                    name,
                    "frame_timeout");

                continue;
            }

            const double age_sec =
                std::chrono::duration<double>(
                    now - camera.last_receive_time_)
                    .count();

            if (age_sec <= kCameraTimeoutSec)
            {
                continue;
            }

            FaultObservation observation;

            observation.module_ = name;
            observation.code_ = "frame_timeout";
            observation.state_ = FaultState::Active;
            observation.severity_ = Severity::Warning;

            observation.detail_ =
                "超过"
                + std::to_string(kCameraTimeoutSec)
                + "秒未接纳有效新图像帧，topic="
                + camera.topic_;

            // 时间继续增长，不算新的故障内容。
            observation.fingerprint_ = "new_frame_missing";

            fault_manager_->observe(observation);
        }
    }
    // 定时检查数据年龄，并输出缓存状态,回调函数
    void checkTimeout(){
        // 放在里程计的提前return之前。
        checkFaultTimeout();
        checkArmFaultTimeouts();
        checkGripperTimeouts();
        checkCameraTimeouts();
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
    // 启动时读取的配置。
    std::string gripper_joint_states_topic_;

    std::string left_gripper_joint_;
    std::string right_gripper_joint_;

    // 实际夹爪驱动节点的完整名称。
    // 用于排除joint_state_publisher等其他发布者。
    std::string left_gripper_publisher_;
    std::string right_gripper_publisher_;
    // 保存订阅对象，让它在节点运行期间持续存在。
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<rpp_msgs::msg::BatteryState>::SharedPtr battery_subscription_;
    rclcpp::Subscription<rpp_msgs::msg::DeviceState>::SharedPtr device_subscription_;
    rclcpp::Subscription<rpp_msgs::msg::ErrorState>::SharedPtr error_subscription_;
    rclcpp::Subscription<rm_ros_interfaces::msg::Jointerrorcode>::SharedPtr left_arm_joint_subscription_;
    rclcpp::Subscription<rm_ros_interfaces::msg::Jointerrorcode>::SharedPtr right_arm_joint_subscription_;
    rclcpp::Subscription<rm_ros_interfaces::msg::Rmerr>::SharedPtr left_arm_system_subscription_;
    rclcpp::Subscription<rm_ros_interfaces::msg::Rmerr>::SharedPtr right_arm_system_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr gripper_subscription_;
    // 每路图像一个订阅对象。
    // 使用vector保存，避免写五个几乎一样的成员。
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> camera_subscriptions_;
    rclcpp::TimerBase::SharedPtr check_timer_;

};
}