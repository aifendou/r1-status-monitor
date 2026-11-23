/*
 * 项目：R1机器人状态采集与TCP远程监测
 *  
 * 功能：TCP 连接、定时查询、重连、接收响应
 * 1. 连接0.0.0.0:9090。
 * 2. 每两秒发送一次GET_STATE。
 * 3. 解析并显示完整状态响应。
 * 4. 断开后自动重连。
 * 5. 将客户端业务日志同时输出到终端和文件。
 */

#include <muduo/net/TcpClient.h>
#include <muduo/net/EventLoop.h>
#include <muduo/net/TcpConnection.h>
#include <muduo/net/InetAddress.h>
#include <muduo/net/Buffer.h>
#include <muduo/base/Timestamp.h>

#include <exception>
#include <functional>
#include <iostream>
#include <string>

#include "FrameCodec.h"
#include "FileLogger.h"
#include "StatusFormatter.h"

using namespace std;
using namespace muduo;
using namespace muduo::net;
using namespace std::placeholders;

class R1MonitorClient
{
public:
    R1MonitorClient(EventLoop *loop, const InetAddress &serverAddr, const string &nameArg,FileLogger& logger)
        : logger_(logger), client_(loop, serverAddr, nameArg), sequence_(0) // 测试消息序号	区分每一次发送
    {
        // 注册连接建立和断开的处理函数
        client_.setConnectionCallback(bind(&R1MonitorClient::onConnection, this, _1));
        // 注册收到服务端数据后的处理函数。
        client_.setMessageCallback(bind(&R1MonitorClient::onMessage, this, _1, _2, _3));
        // 连接断开后，允许Muduo自动重新连接。
        client_.enableRetry();
        /*
         * 定时器只注册一次，不在每次连接成功后重复注册。
         * 网络连接正常时发送测试文字，断开时跳过发送。
         */
        loop->runEvery(2.0, bind(&R1MonitorClient::sendTestMessage, this));
    }
    // 发起第一次连接
    void connect()
    {
        cout << "[R1客户端] 开始连接配置的服务端"
             << endl;
            client_.connect();
    }

private:
    // 处理连接建立和断开，回调函数绑定给Muduo
    void onConnection(const TcpConnectionPtr &conn)
    {
        if (conn->connected())
        {
            // 保存当前连接，供定时发送函数使用
            connection_ = conn;
            logger_.write(
                "CONNECTED",
                "本机=" + conn->localAddress().toIpPort()
                + "，服务端=" + conn->peerAddress().toIpPort());
        }
        else
        {
            // 清除失效连接，但不退出客户端事件循环
            connection_.reset();
            logger_.write(
                "DISCONNECTED",
                "TCP连接断开，客户端保持运行，等待自动重连");
        }
    }

    // 处理服务端发来的数据，回调函数绑定给Muduo
    void onMessage(const TcpConnectionPtr &conn,
                   Buffer *buffer,
                   Timestamp time)
    {
        while (true)
        {
            string response;

            const auto result =
                r1_protocol::readFrame(buffer, response);

            if (result == r1_protocol::ReadResult::NeedMore)
            {
                return;
            }

            if (result == r1_protocol::ReadResult::Invalid)
            {
                logger_.write(
                    "PROTOCOL_ERROR",
                    "响应长度非法，关闭当前连接");

                conn->forceClose();
                return;
            }

            // 保存完整JSON文本，目前不额外解析其中字段。
            handleStateResponse(response);
        }
    }
    // 当前沿用原函数名，功能已经改为周期查询状态
    void sendTestMessage(){
        // 断开期间不发送，也不缓存待发的测试数据。
        if(!connection_ || !connection_->connected()){
            return;
        }
        // 注意：正文是精确的GET_STATE，不带换行。
        const string request = "GET_STATE";//GET_STATE
        if(r1_protocol::sendFrame(connection_, request)){
            logger_.write(
                "QUERY",
                "sequence=" + to_string(++sequence_)
                + "，command=" + request);
        }
        else{
            logger_.write(
                "ERROR",
                "查询消息未通过发送长度检查");
        }
    }
    // 接收一条已经完成分帧的JSON响应，交给摘要模块处理。
    void handleStateResponse(const std::string& response)
    {
        const auto result =
            r1_monitor::formatStateResponse(response);

        // 完整保留Info、Warning、Error等级。
        // 无论什么等级，这里始终只写日常运行日志。
        logger_.writeRuntime(
            result.severity_,
            "STATE",
            result.text_);
    }
    FileLogger& logger_;          // 引用main中创建的日志对象，不复制日志文件流。
    TcpClient client_;            // 负责建立和管理TCP连接
    TcpConnectionPtr connection_; // 保存当前有效连接，供发送函数使用；断开时清空。
    unsigned int sequence_;       // 区分每一次发送
};
int main(int argc, char* argv[])
{   
    try
    {
        /*
         * 默认保存到当前工作目录下的logs。
         * 也允许通过第一个命令行参数指定日志目录。
         */
        const filesystem::path log_directory =
            argc > 1 ? argv[1] : "logs";
        // 先创建logger，使它比client更晚销毁。
        FileLogger logger(log_directory);
        logger.write(
            "START",
            "客户端启动，日志文件=" + logger.path());
        EventLoop loop;
        InetAddress serverAddr("192.168.11.11", 9090);
        R1MonitorClient client(&loop, serverAddr, "R1MonitorClient",logger);
        client.connect();
        loop.loop();
    }
    catch(const exception& error){
        cerr
        << "[客户端启动或运行错误] "
        << error.what()
        << endl;

        return 1;
    }

}
