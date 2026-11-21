/*
 * 测试目标：
 * 直接控制Buffer中的字节，验证实际使用的readFrame()。
 *
 * 本程序不连接服务端，也不监听端口。
 */

#include "FrameCodec.h"

#include <cstdlib>
#include <iostream>
#include <string>

using muduo::net::Buffer;
using r1_protocol::ReadResult;
using r1_protocol::readFrame;

// 检查条件。不满足时打印原因，并以失败状态退出。
void check(bool condition, const char* description)
{
    if (!condition)
    {
        std::cerr << "[FAIL] " << description << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

// 手动构造已知正确的协议字节。
// 不调用sendFrame，避免收发两端同时写错却互相匹配。
std::string makeTestFrame()
{
    // 网络字节序下，整数9对应：00 00 00 09。
    const char header[4] = {0, 0, 0, 9};

    return std::string(header, 4) + "GET_STATE";
}

int main()
{
    const std::string frame = makeTestFrame();

    // 测试一：分批输入一条消息。
    {
        Buffer buffer;
        std::string body;

        // 第一次只收到长度头的前2字节。
        buffer.append(frame.data(), 2);

        check(
            readFrame(&buffer, body) == ReadResult::NeedMore,
            "Incomplete header must return NeedMore");

        check(
            buffer.readableBytes() == 2,
            "Incomplete header must remain in Buffer");

        std::cout << "[PASS] 长度头半包：等待且保留数据"
                  << std::endl;

        // 再加入5字节：
        // 剩余2字节长度头 + 正文前3字节GET。
        buffer.append(frame.data() + 2, 5);

        check(
            readFrame(&buffer, body) == ReadResult::NeedMore,
            "Incomplete body must return NeedMore");

        check(
            buffer.readableBytes() == 7,
            "Incomplete frame must remain in Buffer");

        std::cout << "[PASS] 正文半包：等待且保留数据"
                  << std::endl;

        // 补齐剩余6字节正文。
        buffer.append(frame.data() + 7, frame.size() - 7);

        check(
            readFrame(&buffer, body) == ReadResult::Complete,
            "Completed frame must return Complete");

        check(body == "GET_STATE", "Decoded body is incorrect");

        check(
            buffer.readableBytes() == 0,
            "Completed frame must be consumed exactly");

        std::cout << "[PASS] 补齐消息：正确取出GET_STATE"
                  << std::endl;
    }

    // 测试二：两条完整消息 + 第三条消息的前半段。
    {
        Buffer buffer;
        std::string body;

        const std::string input =
            frame + frame + frame.substr(0, 7);

        buffer.append(input.data(), input.size());

        int count = 0;

        // 与业务接收回调一样，连续尝试取出完整消息。
        while (true)
        {
            const auto result = readFrame(&buffer, body);

            if (result == ReadResult::NeedMore)
            {
                break;
            }

            check(
                result == ReadResult::Complete,
                "Valid input must not be rejected");

            check(body == "GET_STATE", "Decoded body is incorrect");

            ++count;

            // 同时防止解析器错误地不消费数据，导致无限循环。
            check(count <= 2, "Unexpected extra frame");
        }

        check(count == 2, "Must decode exactly two frames");

        check(
            buffer.readableBytes() == 7,
            "Partial third frame must remain in Buffer");

        // 将第三条消息补齐。
        buffer.append(frame.data() + 7, frame.size() - 7);

        check(
            readFrame(&buffer, body) == ReadResult::Complete,
            "Third frame must complete after remaining bytes arrive");

        check(body == "GET_STATE", "Third frame body is incorrect");

        check(
            buffer.readableBytes() == 0,
            "All three frames must be consumed exactly");

        std::cout
            << "[PASS] 多消息合并及尾部半包：正确取出并保留边界"
            << std::endl;
    }

    std::cout << "All frame boundary tests passed."
              << std::endl;

    return 0;
}