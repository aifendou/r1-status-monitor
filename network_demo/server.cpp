/*
 * 项目：R1机器人状态采集与TCP远程监测
 * 阶段：第一步——验证TCP客户端与服务端的连接、接收和回复。
 *
 * 当前功能：
 * 1. 监听本机TCP端口。
 * 2. 记录客户端连接与断开。
 * 3. 打印收到的数据，并原样返回客户端。
 *
 * 后续扩展：
 * 通过ROS 2订阅R1状态，将数据保存到缓存；
 * 客户端查询时，通过TCP返回缓存中的机器人状态。
 *
 * 线程模型：
 * 1个主EventLoop线程：接收新连接、管理连接。
 * 3个额外I/O线程：处理已分配连接的网络事件和回调。
 *
 * 注意：
 * I/O线程不是独立的业务计算线程池。
 * 不要在网络回调中执行长时间阻塞操作。
 */

#include <muduo/net/TcpServer.h>
#include <muduo/net/TcpConnection.h>
#include <muduo/net/EventLoop.h>
#include <muduo/net/InetAddress.h>
#include <muduo/net/Buffer.h>
#include <muduo/base/Timestamp.h>

#include <iostream>
#include <functional>
#include <string>
#include <ctime>
#include <iomanip>

#include "FrameCodec.h"

using namespace std;
using namespace muduo;
using namespace muduo::net;
using namespace std::placeholders;

/*
 * R1监测服务端的应用层封装。
 *
 * 当前沿用课程中的ChatServer类名。
 * TcpServer负责网络连接和收发；
 * 本类负责定义连接变化、收到数据后的处理逻辑。
 */
class ChatServer
{
public:
    ChatServer(EventLoop *loop,               // 主事件循环，由main创建
               const InetAddress &listenAddr, // 服务端监听的IP和端口
               const string &nameArg)         // 服务名称，用于标识和日志
        : server_(loop, listenAddr, nameArg),
          loop_(loop)
    {
        /*
         * 注册连接状态回调。
         * 客户端连接建立或断开时，由Muduo调用onConnection。
         * 注册动作本身不会立即执行回调。
         */
        server_.setConnectionCallback(
            bind(&ChatServer::onConnection, this, _1));

        /*
         * 注册数据接收回调。
         * Muduo将收到的字节放入Buffer后，调用onMessage。
         *
         * 三个参数依次为：
         * 连接对象、接收缓冲区、接收时间戳。
         */
        server_.setMessageCallback(
            bind(&ChatServer::onMessage, this, _1, _2, _3));

        /*
         * 创建3个额外I/O线程。
         * 加上main所在的主EventLoop线程，共4个相关线程。
         * 新连接由主线程接收，再分配给I/O线程处理。
         */
        server_.setThreadNum(3);
    }

    /*
     * 启动服务器监听和配置的I/O线程。
     * 主线程的事件循环由main中的loop.loop()运行。
     */
    void start()
    {
        server_.start();
    }

private:
    /*
     * 客户端连接状态变化回调。
     *
     * 当前客户端是Ubuntu中的测试工具；
     * 后续可以是实验室电脑上的C++监测客户端。
     *
     * 这里的“连接成功”只表示TCP连接建立，
     * 不表示ROS 2数据或机器人硬件状态正常。
     */
    void onConnection(const TcpConnectionPtr &conn)
    {
        if (conn->connected())
        {
            // 打印客户端地址和服务端地址，确认连接来自哪里。
            cout << "[R1监测][TCP连接建立] "
                 << "客户端=" << conn->peerAddress().toIpPort()
                 << "，服务端=" << conn->localAddress().toIpPort()
                 << endl;
        }
        else
        {
            // 记录TCP断开；连接资源由Muduo管理和清理。
            cout << "[R1监测][TCP连接断开] "
                 << "客户端=" << conn->peerAddress().toIpPort()
                 << "，服务端=" << conn->localAddress().toIpPort()
                 << endl;
        }
    }

    /*
     * TCP数据接收回调
     * 增加消息拆包、GET_STATE请求解析和状态缓存查询。
     *
     * 注意：
     * TCP是字节流，一次回调不一定对应一条完整业务消息。
     * 当前Echo测试可以直接回传，业务协议阶段需要处理消息边界。
     */
    void onMessage(const TcpConnectionPtr &conn,
                   Buffer *buffer,
                   Timestamp time)
    {
        // 一个Buffer中可能有多条消息，所以使用循环
        while(true){
            string request;
            const auto result=r1_protocol::readFrame(buffer,request);
            if(result == r1_protocol::ReadResult::NeedMore){
                return;
            }
            if(result ==r1_protocol::ReadResult::Invalid){
                cerr
                << "[R1服务端][协议错误] 消息长度非法，关闭连接"
                << std::endl;
                conn->forceClose();
                return;
            }
            cout<< "[R1服务端][完整请求] "
                << request
                << std::endl;
            string response;
            if (request == "GET_STATE"){
                response = buildFixedState();
            }
            else{
                response =
                R"({"ok":false,"error":"UNKNOWN_COMMAND"})";
            }
            if (!r1_protocol::sendFrame(conn, response)){
                cerr
                << "[R1服务端] 响应长度不合法"
                << std::endl;
                conn->forceClose();
                return;
            }
        }
    }
    // 当前返回固定测试状态；后续再改为读取ROS缓存
    string buildFixedState() const
    {
        return R"({"ok":true,"robot_id":"R1","source":"mock",
        "odom_received":true,"odom_fresh":true,"x":1.0,"y":2.0,"vx":0.1,"wz":0.2})";
    }
    // Muduo服务端对象：负责监听、连接管理和网络收发。
    TcpServer server_;

    // 保存main创建的事件循环地址，不负责释放该对象。
    EventLoop *loop_;
};

int main()
{
    // 创建主线程的事件循环，负责监听连接及相关事件调度。
    EventLoop loop;
    InetAddress addr("127.0.0.1", 9090);
    // 创建R1监测服务端的应用对象。
    ChatServer server(&loop, addr, "R1MonitorServer");
    server.start();
    cout << "[R1监测][服务启动] "
         << "监听地址=127.0.0.1:9090"
         << "，额外I/O线程数=3"
         << "，当前模式=TCP回显测试"
         << endl;
    loop.loop();
    return 0;
}