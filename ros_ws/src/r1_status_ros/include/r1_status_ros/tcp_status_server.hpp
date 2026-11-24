/**
 * 接收查询、读取缓存、生成 JSON 响应
 */
#pragma once
#include "r1_status_ros/state_store.hpp"
#include "r1_status_ros/FrameCodec.h"
#include "r1_status_ros/FaultManager.hpp"

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
                    double timeout_sec,
                    std::shared_ptr<FaultManager> fault_manager = nullptr)
                    :store_(store),
                    fault_manager_(fault_manager),
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
    string batteryToJson(const BatterySnapshot& battery)
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
    // 把设备快照转换为JSON。
    // 与里程计、电池一样，在读取快照后再处理，不持有缓存锁。
    string deviceToJson(const DeviceSnapshot& device) const
    {
        // 暂定5秒，后续根据底层状态实际更新周期调整。
        constexpr double device_timeout_sec = 5.0;

        if (!device.received_)
        {
            return
                R"({"received":false,"fresh":false,"age_sec":null,)"
                R"("control_mode":null,"power_mode":null,)"
                R"("motor_mode":null,"e_stop":null})";
        }

        const double age_sec =
            std::chrono::duration<double>(
                SteadyClock::now() - device.last_update_time_)
                .count();

        const bool fresh = age_sec <= device_timeout_sec;

        std::ostringstream out;
        out.imbue(std::locale::classic());

        out << std::boolalpha
            << std::fixed
            << std::setprecision(3);

        out << R"({"received":true,"fresh":)" << fresh
            << R"(,"age_sec":)" << age_sec
            << R"(,"control_mode":)" << device.control_mode_
            << R"(,"power_mode":)" << device.power_mode_
            << R"(,"motor_mode":)" << device.motor_mode_
            << R"(,"e_stop":)" << device.e_stop_
            << "}";

        return out.str();
    }

    // 故障描述可能包含引号、反斜杠和换行。
    // 不能直接拼进JSON字符串。
    static string quoteJson(const std::string& text)
    {
        static const char hex[] = "0123456789abcdef";

        std::string result = "\"";

        for (unsigned char ch : text)
        {
            if (ch == '"')
            {
                result += "\\\"";
            }
            else if (ch == '\\')
            {
                result += "\\\\";
            }
            else if (ch < 0x20)
            {
                result += "\\u00";
                result += hex[(ch >> 4) & 0x0f];
                result += hex[ch & 0x0f];
            }
            else
            {
                result += static_cast<char>(ch);
            }
        }

        result += "\"";

        return result;
    }
    string faultsToJson() const
    {
        if (!fault_manager_)
        {
            return R"({"enabled":false})";
        }

        const auto data = fault_manager_->snapshot();

        if (!data.enabled_)
        {
            return R"({"enabled":false})";
        }

        std::ostringstream out;

        out << R"({"enabled":true,"active":[)";

        bool first = true;

        for (const auto& fault : data.active_)
        {
            if (!first)
            {
                out << ",";
            }

            first = false;

            const std::string severity =
                fault.severity_ == Severity::Error
                    ? "error"
                    : "warning";

            out << R"({"module":)" << quoteJson(fault.module_)
                << R"(,"code":)" << quoteJson(fault.code_)
                << R"(,"severity":)" << quoteJson(severity)
                << R"(,"detail":)" << quoteJson(fault.detail_)
                << "}";
        }

        out << R"(],"unknown_modules":[)";

        first = true;

        for (const auto& module : data.unknown_modules_)
        {
            if (!first)
            {
                out << ",";
            }

            first = false;
            out << quoteJson(module);
        }

        out << "]}";

        return out.str();
    }
    // 把一路机械臂反馈转换成JSON。
    // 关节错误码和系统错误码使用同一种格式。
    string armFeedbackToJson(const ArmFeedbackSnapshot& feedback) const
    {
        if (!feedback.received_)
        {
            return
                R"({"received":false,"fresh":false,)"
                R"("age_sec":null,"codes":null})";
        }

        const double age_sec =
            std::chrono::duration<double>(
                SteadyClock::now() - feedback.last_receive_time_)
                .count();

        const bool fresh = age_sec <= kArmTimeoutSec;

        std::ostringstream out;
        out.imbue(std::locale::classic());

        out << std::boolalpha
            << std::fixed
            << std::setprecision(3);

        out << R"({"received":true,"fresh":)" << fresh
            << R"(,"age_sec":)" << age_sec
            << R"(,"codes":[)";

        for (std::size_t i = 0; i < feedback.codes_.size(); ++i)
        {
            if (i != 0)
            {
                out << ",";
            }

            out << feedback.codes_[i];
        }

        out << "]}";
        return out.str();
    }

    // 把一侧机械臂的缓存转换成JSON。
    string armToJson(const ArmSnapshot& arm) const
    {
        std::ostringstream out;
        out.imbue(std::locale::classic());

        out << R"({"dof":)";

        if (arm.joint_.received_)
        {
            out << arm.dof_;
        }
        else
        {
            // 尚未收到关节反馈时，默认dof=0没有业务含义。
            out << "null";
        }

        out << R"(,"joint":)"
            << armFeedbackToJson(arm.joint_)
            << R"(,"system":)"
            << armFeedbackToJson(arm.system_)
            << "}";

        return out.str();
    }
    // 把一侧夹爪缓存转换为JSON。
    string gripperToJson(
        const GripperSnapshot& gripper) const
    {
        if (!gripper.received_)
        {
            return
                R"({"received":false,"fresh":false,)"
                R"("age_sec":null,"joint_name":null,)"
                R"("position_rad":null})";
        }

        const double age_sec =
            std::chrono::duration<double>(
                SteadyClock::now() - gripper.last_receive_time_)
                .count();

        const bool fresh = age_sec <= kGripperTimeoutSec;

        std::ostringstream out;
        out.imbue(std::locale::classic());

        out << std::boolalpha
            << std::fixed
            << std::setprecision(3);

        out << R"({"received":true,"fresh":)" << fresh
            << R"(,"age_sec":)" << age_sec
            << R"(,"joint_name":)" << quoteJson(gripper.joint_name_)
            << R"(,"position_rad":)" << gripper.position_rad_
            << "}";

        return out.str();
    }
    // 一路相机元数据转换为JSON。
    std::string cameraToJson(
        const CameraSnapshot& camera) const
    {
        std::ostringstream out;
        out.imbue(std::locale::classic());

        out << std::boolalpha
            << std::fixed
            << std::setprecision(3);

        out << R"({"topic":)" << quoteJson(camera.topic_);

        if (!camera.received_)
        {
            out
                << R"(,"received":false,"fresh":false,)"
                << R"("age_sec":null,"width":null,"height":null,)"
                << R"("encoding":null,"frames":0,"rx_fps":null})";

            return out.str();
        }

        const double age_sec =
            std::chrono::duration<double>(
                SteadyClock::now() - camera.last_receive_time_)
                .count();

        const bool fresh = age_sec <= kCameraTimeoutSec;

        out << R"(,"received":true,"fresh":)" << fresh
            << R"(,"age_sec":)" << age_sec
            << R"(,"width":)" << camera.width_
            << R"(,"height":)" << camera.height_
            << R"(,"encoding":)" << quoteJson(camera.encoding_)
            << R"(,"frames":)" << camera.frames_
            << R"(,"rx_fps":)";

        if (fresh && camera.fps_ready_)
        {
            out << camera.fps_;
        }
        else
        {
            // 尚在统计或数据过期时，不把旧帧率当成当前帧率。
            out << "null";
        }

        out << "}";

        return out.str();
    }

    // 多路相机使用同一套JSON生成逻辑。
    std::string camerasToJson(
        const CamerasSnapshot& cameras) const
    {
        std::string result = "{";
        bool first = true;

        for (const auto& item : cameras)
        {
            if (!first)
            {
                result += ",";
            }

            first = false;

            result += quoteJson(item.first);
            result += ":";
            result += cameraToJson(item.second);
        }

        result += "}";

        return result;
    }
    string buildStateJson()
    {
        // 继续使用原有里程计JSON。
        std::string response = buildOdomStateJson();

        if (response.empty() || response.back() != '}')
        {
            return R"({"ok":false,"error":"STATE_ENCODING_ERROR"})";
        }

        // 返回的是副本，后续拼接JSON时不持有缓存锁。
        const auto battery = store_->batterySnapshot();
        const auto device = store_->deviceSnapshot();
        const auto arms = store_->armsSnapshot();
        const auto grippers = store_->grippersSnapshot();
        const auto cameras = store_->camerasSnapshot();

        response.pop_back();

        response += R"(,"battery":)";
        response += batteryToJson(battery);

        response += R"(,"device":)";
        response += deviceToJson(device);

        // 新增左右机械臂状态。
        response += R"(,"arms":{"left":)";
        response += armToJson(arms.left_);

        response += R"(,"right":)";
        response += armToJson(arms.right_);
        response += "}";

        response += R"(,"grippers":{"left":)";
        response += gripperToJson(grippers.left_);

        response += R"(,"right":)";
        response += gripperToJson(grippers.right_);
        response += "}";

        response += R"(,"cameras":)";
        response += camerasToJson(cameras);

        // 原来的统一故障响应继续保留。
        response += R"(,"faults":)";
        response += faultsToJson();

        response += "}";

        return response;
    }
    std::shared_ptr<StateStore> store_;
    std::shared_ptr<FaultManager> fault_manager_;
    double timeout_sec_;
    muduo::net::TcpServer server_; 
};
}