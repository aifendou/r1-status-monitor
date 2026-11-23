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
private:
    // mutable允许const的snapshot()进行加锁操作
    mutable mutex odom_mutex_;
    mutable mutex battery_mutex_;
    mutable mutex device_mutex_;
    DeviceSnapshot device_;
    BatterySnapshot battery_;
    OdomSnapshot cache_;
};

};
