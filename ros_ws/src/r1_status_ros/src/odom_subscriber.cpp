/*
 * 项目：R1机器人状态采集与TCP远程监测
 * 阶段：ROS 2状态缓存与超时检测。
 *
 * 功能：
 * 1. 订阅 /demo/odom。
 * 2. 缓存最近收到的位置、速度和本地接收时刻。
 * 3. 每秒检查一次，超过2秒未收到消息时提示超时。
 * 4. 恢复接收后提示恢复。
 *
 * 当前使用单线程spin，不涉及Muduo和实机控制。
 */
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <chrono>
#include <functional>
#include <memory>

using SteadyClock =std::chrono::steady_clock;
// 保存最近一次收到的里程计数据,用结构体
struct OdomCache
{
    double x_ =0.0;
    double y_ =0.0;
    double linear_velocity_ = 0.0;
    double angular_velocity_ = 0.0;

    // 区分“从未收到消息”和“收到的数值恰好为零”。
    bool received_ =false;
    // 本机最后一次执行接收回调的时刻。
    SteadyClock::time_point last_receive_time_{};

};
class OdomSubscriber : public rclcpp::Node{
public:
    OdomSubscriber():rclcpp::Node("r1_odom_subscriber")
    {   
        // 创建订阅对象，并注册收到里程计后的处理函数。
        odom_subscription_=this->create_subscription<nav_msgs::msg::Odometry>(
                    "/demo/odom",
                    rclcpp::QoS(10),
                    std::bind(&OdomSubscriber::onOdom,this,std::placeholders::_1)
        );
        // 每秒执行一次检查；即使没有新消息，仍会触发
        check_timer_= this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&OdomSubscriber::checkTimeout,
            this)
        );
        RCLCPP_INFO(
            this->get_logger(),
            "Odom subscriber started. Topic: /demo/odom"
        );
    }
private:
    // 里程计消息到达后，由ROS 2执行器调用。
     void onOdom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        cache_.x_ = msg->pose.pose.position.x;
        cache_.y_ = msg->pose.pose.position.y;

        cache_.linear_velocity_ =
            msg->twist.twist.linear.x;

        cache_.angular_velocity_ =
            msg->twist.twist.angular.z;

        cache_.last_receive_time_ = SteadyClock::now();
        cache_.received_ = true;
    }
    // 定时检查数据年龄，并输出缓存状态。
    void checkTimeout(){
        // 从未收到数据，继续等待。
        if(!cache_.received_){
            return;
        }

        //当前时刻减去最后接收时刻，转换为浮点秒数。
        const double age_sec=
                std::chrono::duration<double>(
                    SteadyClock::now()-cache_.last_receive_time_)
                    .count();
        if(age_sec > timeout_sec_){
            // 同一段超时期间只打印一次告警。
            if(!timed_out_){
                RCLCPP_WARN(
                    this->get_logger(),
                    "[STALE] No odom received for %.2f s. "
                    "Cached values are outdated.",
                    age_sec);
            }
            timed_out_ =true;
            return;
        }
        // 数据重新变得新鲜，打印一次恢复提示
        if(timed_out_){
            RCLCPP_INFO(
                this->get_logger(),
                "[RECOVERED] Odom reception resumed.");
        }
        timed_out_ = false;
        // 正常状态下，每次检查打印一份缓存快照。
        RCLCPP_INFO(
            this->get_logger(),
            "[FRESH] x=%.2f m, y=%.2f m, "
            "vx=%.2f m/s, wz=%.2f rad/s, age=%.2f s",
            cache_.x_,
            cache_.y_,
            cache_.linear_velocity_,
            cache_.angular_velocity_,
            age_sec);
    }
    // 最近一次里程计数据
    OdomCache cache_;
    // 标记上一轮检查是否已经处于超时状态
    bool timed_out_ =false;
    // 超时阈值：2秒
    const double timeout_sec_ =2.0;
    // 保存订阅对象，让它在节点运行期间持续存在。
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::TimerBase::SharedPtr check_timer_;
};
int main(int argc,char *argv[]){
    // 初始化ROS 2，处理ROS命令行参数等
    rclcpp::init(argc,argv);
    // 创建节点对象
    auto node =std::make_shared<OdomSubscriber>();
    // 运行执行器，等待消息并执行订阅回调。
    rclcpp::spin(node);
    // 关闭ROS 2上下文。
    rclcpp::shutdown();
    return 0;
}