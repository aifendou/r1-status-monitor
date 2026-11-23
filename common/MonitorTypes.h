#pragma once

#include <cstdint>  // std::uint64_t：固定宽度的无符号整数。
#include <string>   // std::string：保存文字。
namespace r1_monitor
{
/*
 * 显示和日志等级。
 *
 * 它只表示这条信息需要多大程度的关注，
 * 不决定是否写入故障日志。
 *
 * 例如：
 * 电量19% → Warning → 日常日志标红。
 * 但低电量不属于本项目登记的设备故障，
 * 因此不会因为等级是Warning就写入故障日志。
 */
enum class Severity
{
    Info,       // 普通信息，当前方案中显示蓝色。
    Warning,    // 提醒或告警，当前方案中显示红色。
    Error       // 错误，当前方案中显示红色。
};
/*
 * 客户端摘要函数的统一返回结果。
 *
 * 里程计、电池、设备、故障摘要，
 * 都可以用同一种结构返回显示内容和等级。
 *
 * 这里只负责显示结果，不携带“必须写故障日志”的指令。
 */
struct FormatResult
{
    // 最终需要显示、保存的摘要文字。
    std::string text_;

    // 默认是普通信息。
    Severity severity_ = Severity::Info;
};

/*
 * 某一个“需要纳入故障管理的项目”的当前判断。
 *
 * Unknown不是Healthy：
 * 没收到有效数据、数据过期或编码含义不明确时，
 * 不能宣称故障已经解除。
 */
enum class FaultState
{
    Unknown,    // 当前无法确定。
    Healthy,    // 有效反馈确认该项故障当前未触发。
    Active      // 有效反馈确认该项故障当前触发。
};
/*
 * 提交给故障管理模块的一次观察结果。
 *
 * 只对“已经确定需要登记的故障项目”创建这个对象。
 *
 * 例如：
 * module_ = "chassis"
 * code_   = "motor_alarm"
 *
 */
struct FaultObservation
{
    // 故障来源模块，例如chassis、gripper。
    std::string module_;

    // 模块内的故障标识，例如motor_alarm。
    // module_和code_共同区分是哪一项故障。
    std::string code_;

    // 本次观察得到的状态，默认未知。
    FaultState state_ = FaultState::Unknown;

    // 故障信息的等级。
    // 注意：故障类别和显示等级是不同概念。
    Severity severity_ = Severity::Warning;

    // 给人阅读的说明，可包含厂家原始码。
    std::string detail_;

    /*
     * 故障内容的“比较标记”。
     *
     * 故障管理模块用它判断同一个故障是否发生实质变化。
     */
    std::string fingerprint_;
};
/*
 * 需要记录的故障事件种类。
 *
 * 区分“首次发现就存在”和“监测期间新出现”，
 * 避免把程序启动前的故障说成刚刚发生。
 */
enum class FaultEventKind
{
    InitialActive,  // 首次观察时，故障已经存在。
    Raised,         // 从已知未触发变为触发。
    Changed,        // 已触发故障的内容发生变化。
    Cleared         // 有效反馈确认故障解除。
};

/*
 * 故障管理模块生成的一次事件。
 *
 * FaultObservation是输入：
 *     “我现在看到了什么。”
 *
 * FaultEvent是管理模块判断变化后产生的记录：
 *     “这次观察值得记下一次出现、变化或解除。”
 */
struct FaultEvent
{
    // 在同一次服务端运行中递增的事件序号。
    // 跨进程重启时，还需要另外区分会话。
    std::uint64_t sequence_ = 0;

    std::string module_;
    std::string code_;

    FaultEventKind kind_ = FaultEventKind::InitialActive;
    Severity severity_ = Severity::Warning;

    // 监测程序观察到事件的时间文字。
    // 不保证等于硬件精确发生故障的时间。
    std::string observed_at_;

    // 本次事件的描述。
    std::string detail_;
};

}// namespace r1_monitor