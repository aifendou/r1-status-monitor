#pragma once
#include "r1_status_ros/state_store.hpp"
#include "r1_status_ros/FrameCodec.h"

#include <muduo/net/TcpServer.h>
#include <muduo/net/TcpConnection.h>
#include <muduo/net/EventLoop.h>
#include <muduo/net/InetAddress.h>
#include <muduo/net/Buffer.h>
#include <muduo/base/Timestamp.h>

#include <functional>
#include <iomanip>
#include <iostream>
#include <locale>
#include <memory>
#include <sstream>
#include <string>
using namespace std;
using namespace muduo;
using namespace muduo::net;
using namespace placeholders;

namespace r1_monitor
{
class TcpStatusServer{
public:
    TcpStatusServer(EventLoop* loop,
                    const InetAddress &address,
                    shared_ptr<StateStore> store,
                    double timeout_sec)
                    :store_(store),
                    timeout_sec_(timeout_sec),
                    server_(loop,address,"R1StatusServer")
    {
        server_.setConnectionCallback(bind(&TcpStatusServer::onConnection,this,_1));
        server_.setMessageCallback(bind(&TcpStatusServer::onMessage,this,_1,_2,_3));
        server_.setThreadNum(3);
    }
    void start(){
        server_.start();
        cout
        << "[R1服务端] TCP状态查询已启动,额外I/O线程数=3"
        << endl;
    }

private:
    void onConnection(const TcpConnectionPtr& conn)
    {
        cout
        << "[R1服务端] 客户端="
        << conn->peerAddress().toIpPort()
        << (conn->connected() ? " 已连接" : " 已断开")
        << endl;
    }
    void onMessage(const TcpConnectionPtr& conn,Buffer *buffer,Timestamp)
    {  
        while(true){
            string request;
            const auto result = r1_protocol::readFrame(buffer, request);
            //数据没读全
            if (result == r1_protocol::ReadResult::NeedMore)
            {
                return;
            }

            if (result == r1_protocol::ReadResult::Invalid)
            {
                std::cerr
                    << "[R1服务端] 非法消息长度，关闭连接"
                    << std::endl;

                conn->forceClose();
                return;
            }
            std::string response;
            if (request == "GET_STATE")
            {
                response = buildStateJson();
            }
            else
            {
                response =
                    R"({"ok":false,"error":"UNKNOWN_COMMAND"})";
            }
            if (!r1_protocol::sendFrame(conn, response))
            {
                std::cerr
                    << "[R1服务端] 响应长度非法"
                    << std::endl;

                conn->forceClose();
                return;
            }
        }
    }
    // 从共享快照生成JSON，不再返回固定位置和速度。
    string buildOdomStateJson() const
    {
        const auto data =store_->snapshot();
        // 从未收到消息时，不能把默认零值冒充真实数据
        if (!data.received_)
        {
            return
                R"({"ok":true,"robot_id":"R1","source":"ros_topic",)"
                R"("odom_received":false,"odom_fresh":false,)"
                R"("odom_age_sec":null,"x":null,"y":null,"vx":null,"wz":null})";
        }
        // 每次查询时重新计算数据年龄
        const double age_sec =
                chrono::duration<double>(
                SteadyClock::now() - data.last_receive_time_)
                .count();
        const bool fresh = age_sec <= timeout_sec_;
        std::ostringstream output;
        // 保证小数点为"."，避免区域设置影响JSON格式
        //像`cout`一样使用`<<`拼接内容，但写进的是内存字符串，不是终端
        output.imbue(locale::classic());
        output << boolalpha//让布尔值显示成JSON需要的`true`或`false`，而不是`1`或`0`
               << fixed
               << setprecision(3);//让浮点数保留三位小数
        output
            << R"({"ok":true,"robot_id":"R1","source":"ros_topic",)"//source:"ros_topic"表示数据来自ROS话题
            << R"("odom_received":true,)"
            << R"("odom_fresh":)" << fresh
            << R"(,"odom_age_sec":)" << age_sec
            << R"(,"x":)" << data.x_
            << R"(,"y":)" << data.y_
            << R"(,"vx":)" << data.linear_velocity_
            << R"(,"wz":)" << data.angular_velocity_
            << "}";
        return output.str();//取出最终组成的字符串
    }
    // 把一份电池快照转换为JSON。
    // 这里只处理业务数据，不调用ROS，也不发送TCP。
    std::string batteryToJson(const BatterySnapshot& battery)
    {
        // 暂定5秒；后续根据R1底层电池实际更新周期调整。
        constexpr double battery_timeout_sec = 5.0;
        constexpr int low_battery_threshold = 20;

        if (!battery.received_)
        {
            return R"({"received":false,"fresh":false,"age_sec":null,"percentage":null,"low_battery":null})";
        }

        const double age_sec =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now()
                - battery.last_update_time_)
                .count();

        const bool fresh = age_sec <= battery_timeout_sec;

        std::ostringstream out;

        // 保证小数点使用"."，布尔值输出true/false。
        out.imbue(std::locale::classic());
        out << std::boolalpha;
        out.setf(std::ios::fixed, std::ios::floatfield);
        out.precision(3);

        out << R"({"received":true,"fresh":)" << fresh
            << R"(,"age_sec":)" << age_sec
            << R"(,"percentage":)" << battery.percentage_
            << R"(,"low_battery":)";

        if (fresh)
        {
            out << (battery.percentage_ < low_battery_threshold);
        }
        else
        {
            // 数据过期，不能声称当前电量正常或当前仍然低电量。
            out << "null";
        }

        out << "}";

        return out.str();
    }
    std::string buildStateJson()
    {
        // 使用原来的逻辑生成完整里程计JSON。
        std::string response = buildOdomStateJson();

        if (response.empty() || response.back() != '}')
        {
            return R"({"ok":false,"error":"STATE_ENCODING_ERROR"})";
        }

        // 取得电池快照。复制完成后，缓存锁已释放。
        const auto battery = store_->batterySnapshot();

        // 去掉最外层JSON末尾的"}"，追加battery成员后再补回。
        response.pop_back();

        response += R"(,"battery":)";
        response += batteryToJson(battery);
        response += "}";

        return response;
    }

    std::shared_ptr<StateStore> store_;
    double timeout_sec_;
    muduo::net::TcpServer server_; 
};
}