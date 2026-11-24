/**
 * 共享缓存
 * 保存里程计、电池、设备快照；加锁；记录更新时间
 * */
#pragma once

#include <chrono>
#include <cstdint>
#include <locale>
#include <cmath>//数学公式
#include <mutex>//锁
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <stdexcept>
#include <cstddef>


using namespace std;
namespace r1_monitor{
using SteadyClock = std::chrono::steady_clock;
// 一份里程计快照：数据与最后接收时刻放在一起。
struct OdomSnapshot
{
    double x_ = 0.0;
    double y_ = 0.0;
    double linear_velocity_ = 0.0;
    double angular_velocity_ = 0.0;

    bool received_ = false;
    SteadyClock::time_point last_receive_time_{};
};
// 电池数据的独立快照
struct BatterySnapshot{
    int percentage_= 0;
    bool received_ = false;
    // 厂家电池消息中的底层更新时间。
    int64_t source_ts_ =0;
    // 本机最近观察到“新的底层电池时间戳”的时刻
    SteadyClock::time_point last_update_time_{};
};
// 设备状态快照。
// 状态值先保存厂家原始编码，确认含义后再转换为中文。
struct DeviceSnapshot
{
    int control_mode_ = 0;//控制模式或控制来源
    int power_mode_ = 0;//供电模式
    int motor_mode_ = 0;//电机模式
    int e_stop_ = 0;//急停状态字段

    // false表示尚未收到有效数据，此时上面的默认值没有业务含义。
    bool received_ = false;

    // 厂家底层状态的更新时间戳。
    int64_t source_ts_ = 0;//厂家状态更新时间戳

    // 本机最近观察到源时间戳变化的时刻。
    SteadyClock::time_point last_update_time_{};
};
// 明确区分左右机械臂，避免用true/false表示方向。
enum class ArmSide
{
    Left,
    Right
};

// 暂定5秒没有有效ROS消息，就认为这路反馈已经过期。
// ROS检查和JSON生成使用同一个阈值。
constexpr double kArmTimeoutSec = 5.0;

// 一路机械臂错误码反馈。
// 关节错误码与系统错误码分别保存、分别计时。
struct ArmFeedbackSnapshot
{
    std::vector<std::int32_t> codes_;

    bool received_ = false;

    // 这两种厂家消息没有源时间戳，因此记录本机接收时刻。
    SteadyClock::time_point last_receive_time_{};
};

// 一侧机械臂的缓存。
struct ArmSnapshot
{
    int dof_ = 0;

    ArmFeedbackSnapshot joint_;
    ArmFeedbackSnapshot system_;
};

// 同时保存左右机械臂。
struct ArmsSnapshot
{
    ArmSnapshot left_;
    ArmSnapshot right_;
};
enum class GripperSide
{
    Left,
    Right
};

// 厂家参考驱动每100ms发布一次。
// 暂定超过2秒未收到有效位置反馈，判为反馈超时。
constexpr double kGripperTimeoutSec = 2.0;

// 一侧夹爪的实际位置反馈。
struct GripperSnapshot
{
    std::string joint_name_;
    double position_rad_ = 0.0;

    bool received_ = false;
    SteadyClock::time_point last_receive_time_{};
};

struct GrippersSnapshot
{
    GripperSnapshot left_;
    GripperSnapshot right_;
};
// 超过2秒没有接纳有效新帧，认为新帧反馈超时。
constexpr double kCameraTimeoutSec = 2.0;

// 本版最多同时监测8路图像流。
constexpr std::size_t kMaxCameraCount = 8;

struct CameraSnapshot
{
    std::string topic_;

    bool received_ = false;

    std::uint32_t width_ = 0;
    std::uint32_t height_ = 0;
    std::string encoding_;

    // 本次监测进程累计接纳的新帧数量。
    std::uint64_t frames_ = 0;

    // 最近接纳帧的源时间戳，用于识别重复帧。
    std::int64_t source_stamp_ns_ = 0;

    SteadyClock::time_point last_receive_time_{};

    // 最近完成的统计窗口中的接收帧率。
    double fps_ = 0.0;
    bool fps_ready_ = false;

    // 以下是缓存内部的统计字段，不发送给客户端。
    SteadyClock::time_point window_start_time_{};
    std::uint64_t window_frames_ = 0;
};

// 用名称区分各路相机，避免为每台相机重复写结构体。
using CamerasSnapshot =
    std::map<std::string, CameraSnapshot>;

// ROS线程和TCP线程共享这个对象
class StateStore
{
public:
    // 更新整份缓存。
    // 非有限数值不写入，避免后续生成非法JSON。
    bool update(double x,double y,double vx,double wz){
        if(!isfinite(x)||
           !isfinite(y)||
           !isfinite(vx)||
           !isfinite(wz))
           {
            return false;
           }
        //lock_guard在创建时加锁，离开作用域时自动解锁，包括执行`return`时。
        lock_guard<mutex> lock(odom_mutex_);

        cache_.x_ = x;
        cache_.y_ = y;
        cache_.linear_velocity_ = vx;
        cache_.angular_velocity_ = wz;
        cache_.last_receive_time_ = SteadyClock::now();
        cache_.received_ = true;
        return true;
    }
    // 更新电池百分比
    // 返回false表示字段无效；重复发布相同时间戳不刷新数据年龄
    bool updateBattery(int percentage,int64_t source_ts)
    { 
        if (percentage < 0 || percentage > 100 || source_ts <= 0)
            {
                return false;
            }
        lock_guard<mutex> lock(battery_mutex_);
        if(battery_.received_ && source_ts ==battery_.source_ts_){
            return true;
        }
        battery_.percentage_ =percentage;
        battery_.source_ts_ =source_ts;
        battery_.last_update_time_ =SteadyClock::now();
        battery_.received_=true;
        return true;
    }
    // 更新设备状态。
    // 消息中的四个状态字段类型都是uint8，先只检查编码范围。
    bool updateDevice(
        int control_mode,
        int power_mode,
        int motor_mode,
        int e_stop,
        std::int64_t source_ts)
    {
        const auto valid_byte = [](int value)
        {
            return value >= 0 && value <= 255;
        };

        if (!valid_byte(control_mode)
            || !valid_byte(power_mode)
            || !valid_byte(motor_mode)
            || !valid_byte(e_stop)
            || source_ts <= 0)
        {
            return false;
        }

        std::lock_guard<std::mutex> lock(device_mutex_);

        if (device_.received_
            && source_ts == device_.source_ts_)
        {
            // 重复发布同一份底层数据，不刷新新鲜度。
            return true;
        }

        device_.control_mode_ = control_mode;
        device_.power_mode_ = power_mode;
        device_.motor_mode_ = motor_mode;
        device_.e_stop_ = e_stop;

        device_.source_ts_ = source_ts;
        device_.last_update_time_ = SteadyClock::now();
        device_.received_ = true;

        return true;
    }
    // 更新一侧机械臂的关节错误码。
    // 当前R1实机查询结果：每侧六个关节。
    bool updateArmJoint(
        ArmSide side,
        int dof,
        const std::vector<std::int32_t>& codes)
    {
        if (dof != 6 || codes.size() != 6)
        {
            return false;
        }

        // 厂家关节错误码类型为uint16。
        for (const auto code : codes)
        {
            if (code < 0 || code > 65535)
            {
                return false;
            }
        }

        std::lock_guard<std::mutex> lock(arms_mutex_);

        ArmSnapshot& arm =
            side == ArmSide::Left ? arms_.left_ : arms_.right_;

        arm.dof_ = dof;
        arm.joint_.codes_ = codes;
        arm.joint_.received_ = true;
        arm.joint_.last_receive_time_ = SteadyClock::now();

        return true;
    }

    // 更新一侧机械臂的系统错误码。
    // error_count对应厂家消息的err_len，是数组长度，不是故障数量。
    bool updateArmSystem(
        ArmSide side,
        int error_count,
        const std::vector<std::int32_t>& codes)
    {
        // 厂家参考SDK的错误码数组容量为24。
        if (error_count < 0
            || error_count > 24
            || static_cast<std::size_t>(error_count) != codes.size())
        {
            return false;
        }

        std::lock_guard<std::mutex> lock(arms_mutex_);

        ArmSnapshot& arm =
            side == ArmSide::Left ? arms_.left_ : arms_.right_;

        arm.system_.codes_ = codes;
        arm.system_.received_ = true;
        arm.system_.last_receive_time_ = SteadyClock::now();

        return true;
    }

    // 保存实际位置反馈。
    // 不根据角度自行推断“抓取成功”或“夹爪故障”。
    bool updateGripper(
        GripperSide side,
        const std::string& joint_name,
        double position_rad)
    {
        if (joint_name.empty() || !std::isfinite(position_rad))
        {
            return false;
        }

        std::lock_guard<std::mutex> lock(grippers_mutex_);

        GripperSnapshot& gripper =
            side == GripperSide::Left
                ? grippers_.left_
                : grippers_.right_;

        gripper.joint_name_ = joint_name;
        gripper.position_rad_ = position_rad;
        gripper.received_ = true;
        gripper.last_receive_time_ = SteadyClock::now();

        return true;
    }
    // 注册实际启用的相机。
    // 注册后默认received=false，不能把默认值当作有效图像信息。
    void registerCamera(
        const std::string& name,
        const std::string& topic)
    {
        const std::string allowed =
            "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
            "abcdefghijklmnopqrstuvwxyz"
            "0123456789_";

        if (name.empty()
            || name.size() > 32
            || name.find_first_not_of(allowed) != std::string::npos
            || topic.empty()
            || topic.size() > 192
            || topic.front() != '/')
        {
            throw std::invalid_argument(
                "Invalid camera name or absolute topic.");
        }

        std::lock_guard<std::mutex> lock(cameras_mutex_);

        if (cameras_.size() >= kMaxCameraCount)
        {
            throw std::invalid_argument(
                "Too many monitored cameras.");
        }

        CameraSnapshot camera;
        camera.topic_ = topic;

        if (!cameras_.emplace(name, camera).second)
        {
            throw std::invalid_argument(
                "Duplicate camera name.");
        }
    }

    // 接纳一帧新的有效图像。
    // 只保存元数据，不复制和保存图像像素。
    //
    // 返回false：字段无效、未注册，或源时间戳与上一帧相同。
    // 重复帧不会刷新新鲜度，也不会用于解除超时报警。
    bool updateCamera(
        const std::string& name,
        std::uint32_t width,
        std::uint32_t height,
        const std::string& encoding,
        std::int64_t source_stamp_ns)
    {
        if (width == 0
            || height == 0
            || encoding.empty()
            || encoding.size() > 64
            || source_stamp_ns <= 0)
        {
            return false;
        }

        std::lock_guard<std::mutex> lock(cameras_mutex_);

        const auto found = cameras_.find(name);

        if (found == cameras_.end())
        {
            return false;
        }

        CameraSnapshot& camera = found->second;

        if (camera.received_
            && source_stamp_ns == camera.source_stamp_ns_)
        {
            return false;
        }

        const auto now = SteadyClock::now();

        // 首帧或长时间中断后，重新开始帧率统计。
        const bool restart_window =
            !camera.received_
            || std::chrono::duration<double>(
                now - camera.last_receive_time_)
                .count() > kCameraTimeoutSec;

        if (restart_window)
        {
            camera.window_start_time_ = now;
            camera.window_frames_ = 0;
            camera.fps_ = 0.0;
            camera.fps_ready_ = false;
        }
        else
        {
            ++camera.window_frames_;

            const double window_sec =
                std::chrono::duration<double>(
                    now - camera.window_start_time_)
                    .count();

            if (window_sec >= 1.0)
            {
                camera.fps_ =
                    static_cast<double>(camera.window_frames_)
                    / window_sec;

                camera.fps_ready_ = true;

                camera.window_start_time_ = now;
                camera.window_frames_ = 0;
            }
        }

        camera.width_ = width;
        camera.height_ = height;
        camera.encoding_ = encoding;
        camera.source_stamp_ns_ = source_stamp_ns;

        camera.last_receive_time_ = now;
        camera.received_ = true;

        ++camera.frames_;

        return true;
    }


    // 返回一份副本，不把内部缓存的引用暴露给其他线程
    OdomSnapshot snapshot() const{
        lock_guard<mutex> lock(odom_mutex_);
        return cache_;
    }
    // 返回副本；调用者取得副本后即可释放锁并处理业务
    BatterySnapshot batterySnapshot() const{
        lock_guard<mutex> lock(battery_mutex_);
        return battery_;
    }
    // 返回独立副本，避免其他线程直接访问内部成员。
    DeviceSnapshot deviceSnapshot() const
    {
        std::lock_guard<std::mutex> lock(device_mutex_);
        return device_;
    }
    // 返回左右机械臂缓存的完整副本。
    ArmsSnapshot armsSnapshot() const
    {
        std::lock_guard<std::mutex> lock(arms_mutex_);

        return arms_;
    }
    // 复制完整快照，返回后释放锁。
    GrippersSnapshot grippersSnapshot() const
    {
        std::lock_guard<std::mutex> lock(grippers_mutex_);

        return grippers_;
    }
    // 返回全部相机的元数据副本。
    CamerasSnapshot camerasSnapshot() const
    {
        std::lock_guard<std::mutex> lock(cameras_mutex_);

        return cameras_;
    }
private:
    // mutable允许const的snapshot()进行加锁操作
    mutable mutex odom_mutex_;
    mutable mutex battery_mutex_;
    mutable mutex device_mutex_;
    mutable mutex arms_mutex_;
    mutable mutex grippers_mutex_;
    mutable mutex cameras_mutex_;
    DeviceSnapshot device_;
    BatterySnapshot battery_;
    OdomSnapshot cache_;
    ArmsSnapshot arms_;
    GrippersSnapshot grippers_;
    CamerasSnapshot cameras_;
};

};
