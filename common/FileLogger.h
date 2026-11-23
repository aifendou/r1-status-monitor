#pragma once
#include "MonitorTypes.h"
#include <ctime>       // 系统时间、time_t、tm。
#include <filesystem>  // 文件路径、目录创建；需要C++17。
#include <fstream>     // 文件输出流ofstream。
#include <iomanip>     // 时间格式化工具put_time。
#include <iostream>    // 终端输出cout、错误输出cerr。
#include <mutex>
#include <sstream>     // 字符串输出流ostringstream。
#include <stdexcept>   // 标准异常runtime_error。
#include <string>      // 字符串与to_string

#include <sys/types.h> // Linux进程ID类型pid_t。
#include <unistd.h>    // Linux接口getpid()。

/**
 * 通用日志工具。
 * 
 * 两个独立出口：
 * writeRuntime()：日常运行日志，启动时打开文件。
 * writeFault()  ：故障事件日志，第一次调用时才打开文件。
 * 
 * 显示等级不决定日志去向。
 * 低电量调用writeRuntime(Warning, ...)，不会写入故障文件
 */


class FileLogger
{
public:
    explicit FileLogger(
        const std::filesystem::path& directory = "logs",
        const std::string& role = "client")
        : process_id_(::getpid())
    {
        if (role != "client" && role != "server")
        {
            throw std::invalid_argument(
                "日志角色必须是client或server");
        }

        const std::time_t start_time = std::time(nullptr);

        const std::string session_name =
            role + "_"
            + formatTime(start_time, "%Y%m%d_%H%M%S")
            + "_pid"
            + std::to_string(process_id_);

        const auto session_directory =
            std::filesystem::absolute(directory) / session_name;

        std::filesystem::create_directories(session_directory);

        const std::string date =
            formatTime(start_time, "%Y-%m-%d");

        runtime_path_ =
            session_directory
            / ("r1_" + role + "_" + date + ".log");

        fault_path_ =
            session_directory
            / ("r1_fault_" + date + ".log");

        // 日常日志立即打开。
        runtime_file_.open(
            runtime_path_,
            std::ios::out | std::ios::app);

        if (!runtime_file_.is_open())
        {
            throw std::runtime_error(
                "无法打开运行日志：" + runtime_path_.string());
        }

        // 注意：这里只计算故障日志路径。
        // 没有打开fault_file_，因此不会创建故障日志文件。
    }


    // 日常运行记录。
    // 可保存普通状态、低电量提醒、连接断开等信息。
    void writeRuntime(
        r1_monitor::Severity severity,
        const std::string& event,
        const std::string& message)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        const std::string line = makeLine(
            severity,
            event,
            message,
            formatTime(std::time(nullptr), "%Y-%m-%d %H:%M:%S"));

        printConsole(line, severity);

        appendToFile(
            runtime_file_,
            runtime_failed_,
            runtime_path_,
            line);
    }


    // 故障事件专用出口。
    //
    // 只由故障管理模块提交已确认需要记录的FaultEvent。
    // 不因为某条运行日志是Warning，就自动调用这里。
    void writeFault(const r1_monitor::FaultEvent& event)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        const auto severity =
            event.kind_ == r1_monitor::FaultEventKind::Cleared
                ? r1_monitor::Severity::Info
                : event.severity_;

        const std::string observed_at =
            event.observed_at_.empty()
                ? formatTime(
                    std::time(nullptr),
                    "%Y-%m-%d %H:%M:%S")
                : event.observed_at_;

        std::ostringstream message;

        message << "seq=" << event.sequence_
                << " module=" << event.module_
                << " code=" << event.code_
                << " " << event.detail_;

        const std::string line = makeLine(
            severity,
            faultEventName(event.kind_),
            message.str(),
            observed_at);

        printConsole(line, severity);

        if (fault_failed_)
        {
            return;
        }

        // 第一次真正记录故障事件时，才创建故障文件。
        if (!fault_file_.is_open())
        {
            fault_file_.open(
                fault_path_,
                std::ios::out | std::ios::app);

            if (!fault_file_.is_open())
            {
                fault_failed_ = true;

                std::cerr
                    << "[日志错误] 无法打开故障日志："
                    << fault_path_.string()
                    << std::endl;

                return;
            }
        }

        appendToFile(
            fault_file_,
            fault_failed_,
            fault_path_,
            line);
    }


    // 兼容原来的write("CONNECTED", "...")等调用。
    void write(
        const std::string& event,
        const std::string& message)
    {
        writeRuntime(
            r1_monitor::Severity::Info,
            event,
            message);
    }


    // 兼容原来的布尔颜色接口，后面可以逐步替换。
    void writeState(
        const std::string& event,
        const std::string& message,
        bool warning)
    {
        writeRuntime(
            warning
                ? r1_monitor::Severity::Warning
                : r1_monitor::Severity::Info,
            event,
            message);
    }


    // 返回日常日志路径，保留原来客户端的使用方式。
    std::string path() const
    {
        return runtime_path_.string();
    }


private:
    static std::string formatTime(
        std::time_t when,
        const char* pattern)
    {
        std::tm local_time{};

        if (::localtime_r(&when, &local_time) == nullptr)
        {
            throw std::runtime_error("本地时间转换失败");
        }

        std::ostringstream out;
        out << std::put_time(&local_time, pattern);

        return out.str();
    }


    static const char* severityName(
        r1_monitor::Severity severity)
    {
        switch (severity)
        {
        case r1_monitor::Severity::Info:
            return "INFO";

        case r1_monitor::Severity::Warning:
            return "WARN";

        case r1_monitor::Severity::Error:
            return "ERROR";
        }

        return "UNKNOWN";
    }


    static const char* faultEventName(
        r1_monitor::FaultEventKind kind)
    {
        switch (kind)
        {
        case r1_monitor::FaultEventKind::InitialActive:
            return "FAULT_INITIAL_ACTIVE";

        case r1_monitor::FaultEventKind::Raised:
            return "FAULT_RAISED";

        case r1_monitor::FaultEventKind::Changed:
            return "FAULT_CHANGED";

        case r1_monitor::FaultEventKind::Cleared:
            return "FAULT_CLEARED";
        }

        return "FAULT_UNKNOWN";
    }


    // 避免一条日志正文中的换行把记录拆成多行。
    static std::string oneLine(std::string text)
    {
        for (char& ch : text)
        {
            if (ch == '\n' || ch == '\r')
            {
                ch = ' ';
            }
        }

        return text;
    }


    std::string makeLine(
        r1_monitor::Severity severity,
        const std::string& event,
        const std::string& message,
        const std::string& observed_at) const
    {
        std::ostringstream out;

        out << "[" << oneLine(observed_at) << "] "
            << "[pid=" << process_id_ << "] "
            << "[" << severityName(severity) << "] "
            << "[" << oneLine(event) << "] "
            << oneLine(message);

        return out.str();
    }


    // ANSI颜色只输出到终端，不写入日志文件。
    static void printConsole(
        const std::string& line,
        r1_monitor::Severity severity)
    {
        const bool terminal =
            ::isatty(STDOUT_FILENO) != 0;

        if (terminal)
        {
            std::cout
                << (severity == r1_monitor::Severity::Info
                        ? "\033[34m"
                        : "\033[31m");
        }

        std::cout << line;

        if (terminal)
        {
            std::cout << "\033[0m";
        }

        std::cout << std::endl;
    }


    // 两种日志复用同一份文件写入逻辑。
    static void appendToFile(
        std::ofstream& file,
        bool& failed,
        const std::filesystem::path& path,
        const std::string& line)
    {
        if (failed)
        {
            return;
        }

        file << line << '\n';
        file.flush();

        if (!file)
        {
            failed = true;

            std::cerr
                << "[日志错误] 文件写入失败，"
                << "该文件后续停止写入："
                << path.string()
                << std::endl;
        }
    }


    const pid_t process_id_;

    std::filesystem::path runtime_path_;
    std::filesystem::path fault_path_;

    std::ofstream runtime_file_;
    std::ofstream fault_file_;

    bool runtime_failed_ = false;
    bool fault_failed_ = false;

    // 保护同一个日志对象的文件流和终端输出。
    // 不同进程仍然通过不同会话目录区分。
    std::mutex mutex_;
};