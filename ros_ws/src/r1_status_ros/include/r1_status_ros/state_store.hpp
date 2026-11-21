//共享缓存
#pragma once
#include <chrono>
#include <cmath>//数学公式
#include <mutex>//锁
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
        lock_guard<mutex> lock(mutex_);

        cache_.x_ = x;
        cache_.y_ = y;
        cache_.linear_velocity_ = vx;
        cache_.angular_velocity_ = wz;
        cache_.last_receive_time_ = SteadyClock::now();
        cache_.received_ = true;
        return true;
    }
    // 返回一份副本，不把内部缓存的引用暴露给其他线程
    OdomSnapshot snapshot() const{
        lock_guard<mutex> lock(mutex_);
        return cache_;
    }
private:
    // mutable允许const的snapshot()进行加锁操作
    mutable mutex mutex_;
    OdomSnapshot cache_;
};

};
