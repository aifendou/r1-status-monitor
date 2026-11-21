#pragma once 
#include <muduo/net/Buffer.h>
#include <muduo/net/TcpConnection.h>
#include <cstddef>
#include <cstdint>
#include <string>
using namespace std;
using namespace muduo;
using namespace muduo::net;

//命名空间，可以不依托对象而调用FrameCodec内部的成员函数
namespace r1_protocol
{
// 当前协议限制单条正文最多4096字节
constexpr int32_t kMaxBodyLength =4096;
// 一次解码可能得到三种结果。
enum class ReadResult
{
    NeedMore,// 数据还不完整，继续等待
    Complete,// 已经取出一条完整消息
    Invalid  // 长度非法
};
// 给正文加上长度头，然后提交给Muduo发送
// 返回true只表示本函数通过了检查并调用了send，不表示对端已收到
inline bool sendFrame(
    const TcpConnectionPtr& conn,//通过哪条连接发送
    const string& body)//要发送的正文内容
    {
        if(body.empty()||body.size()>static_cast<size_t>(kMaxBodyLength)){
            return false;
        }
        Buffer output;
        // appendInt32内部会转换为网络字节序
        output.appendInt32(static_cast<int32_t>(body.size()));
        output.append(body.data(),body.size());
        conn->send(&output);
        return true;
    }
// 从接收Buffer尝试取出一条完整消息。
inline ReadResult readFrame(
    Buffer *buffer,
    string &body)
    {
        const size_t header_size =sizeof(int32_t);
        // 长度头本身都没收完整，暂不消费任何数据
        if(buffer->readableBytes()<header_size){
            return ReadResult::NeedMore;
        }
        // 查看长度，不移动读取位置
        // peekInt32内部会转换为主机字节序
        const int32_t length =buffer->peekInt32();
        if(length<=0||length>kMaxBodyLength){
            return ReadResult::Invalid;
        }
        const size_t frame_size = header_size + static_cast<size_t>(length);
        // 正文还没收完整，长度头和已有正文都保留下来。
        if(buffer->readableBytes()<frame_size){
            return ReadResult::NeedMore;
        }
        // 消费长度头，然后取出指定长度的正文。
        buffer->retrieve(header_size);
        body = buffer->retrieveAsString(static_cast<std::size_t>(length));
        return ReadResult::Complete;
    }
};