#pragma once

#include <ctime>       // 系统时间、time_t、tm。
#include <filesystem>  // 文件路径、目录创建；需要C++17。
#include <fstream>     // 文件输出流ofstream。
#include <iomanip>     // 时间格式化工具put_time。
#include <iostream>    // 终端输出cout、错误输出cerr。
#include <sstream>     // 字符串输出流ostringstream。
#include <stdexcept>   // 标准异常runtime_error。
#include <string>      // 字符串与to_string。

#include <sys/types.h> // Linux进程ID类型pid_t。
#include <unistd.h>    // Linux接口getpid()。
/*
 * 客户端业务日志工具。
 *
 * 功能：
 * 1. 根据启动时间和进程ID，创建独立的会话目录。
 * 2. 文件名使用：r1_client_年-月-日.log。
 * 3. 每条日志同时输出到终端和文件。
 * 4. 文件写入失败后提示错误，保留终端输出。
 *
 * 使用方式：
 *     FileLogger logger;
 *     logger.write("CONNECTED", "连接成功");
 *
 * 当前按单线程调用设计。
 * 不同客户端进程通过不同目录避免写入同一个文件；
 * 若以后同一进程中有多个线程同时调用write()，需要再加锁。
 */
using namespace std;
class FileLogger{
public:
    explicit FileLogger(const filesystem::path& directory = "logs")
                        :process_id_(::getpid())
    {
        // 只取一次启动时刻，保证目录和文件名使用同一日期
        const time_t start_time =time(nullptr);
        // 例如：client_20261121_100000_pid12345
        const string session_name =
                    "client_"
                    +formatTime(start_time,"%Y%m%d_%H%M%S")
                    +"_pid"
                    +to_string(process_id_);
        //absolute()把相对路径转换成绝对路径
        const filesystem::path session_directory =
                filesystem::absolute(directory)/session_name;
        // 自动创建本次会话目录，不清空已有目录
        filesystem::create_directories(session_directory);
        // 文件名只包含固定名称与日期
        file_path_=session_directory /
                ("r1_client_"
                + formatTime(start_time, "%Y-%m-%d")
                + ".log");

        file_.open(file_path_,ios::out | ios::app);
        if(!file_.is_open()){
            // 由main中的catch接收并打印错误,throw抛出异常
            throw runtime_error("无法打开日志文件：" + file_path_.string());
        }

    }
     /*
     * event：事件名称，例如START、CONNECTED、STATE。
     * message：具体描述或完整JSON响应。
     *
     * 示例输出：
     * [2026-11-21 10:00:00 +0800] [pid=12345]
     * [CONNECTED] 连接成功
     *
     * 实际写入时，每条记录占一行。
     */
    void write(const string& event,const string& message){
        // 修改副本，不修改调用者传来的原始字符串
        string one_line =message;
        // 避免正文中的实际换行将一条记录拆成多行
        for (char& ch : one_line)
        {
            if (ch == '\n' || ch == '\r')
            {
                ch = ' ';
            }
        }
        // 字符串流：使用<<组合内容，但不直接输出到终端
        ostringstream output;
        output
            << "["
            << formatTime(
                   std::time(nullptr),
                   "%Y-%m-%d %H:%M:%S")
            << "] "
            << "[pid=" << process_id_ << "] "
            << "[" << event << "] "
            << one_line;
        const string line = output.str();
        // 无论文件是否正常，终端都继续显示
        cout<<line<<endl;
        // 曾经发生文件写入失败后，不再反复尝试写入
        if (file_failed_)
        {
            return;
        }

        file_ << line << '\n';
        // 每条记录刷新一次C++文件流缓冲
        file_.flush();
        // 检查写入或刷新是否失败
        if(!file_){
            file_failed_ =true;
            cerr
            << "[日志错误] 文件写入失败，"
            << "后续日志仅输出到终端："
            << file_path_.string()
            << endl;
        }
    }
    /**
     * warning=false：正常状态，终端蓝色。
     * warning=true ：告警状态，终端红色。
     * 文件里保存[INFO]或[WARN]，不写入ANSI颜色控制字符。
     * 沿用现有write()的文件保存、日期切换和错误处理。
     * */
    void writeState(
        const std::string& event,
        const std::string& message,
        bool warning)
    {
        // 只有真正输出到终端时才使用颜色。
        // 重定向到文件时，不输出颜色控制字符。
        const bool terminal =
            ::isatty(STDOUT_FILENO) != 0;

        if (terminal)
        {
            std::cout
                << (warning ? "\033[31m" : "\033[34m");
        }

        try
        {
            // 原write()的第一个参数是事件标签。
            // 这里用INFO/WARN作为等级，业务事件放到正文中。
            write(
                warning ? "WARN" : "INFO",
                "[" + event + "] " + message);
        }
        catch (...)
        {
            if (terminal)
            {
                std::cout << "\033[0m" << std::flush;
            }

            throw;
        }

        if (terminal)
        {
            // 恢复终端颜色，避免后续提示符也被染色。
            std::cout << "\033[0m" << std::flush;
        }
    }
    // 返回完整文件路径，供客户端启动时打印
    string path() const{
        return file_path_.string();
    }
private:
    /*
     * 将指定系统时刻转换为Ubuntu本地时间字符串。
     *
     * when：要格式化的时间。
     * pattern：输出格式。
     *
     * %Y：年    %m：月    %d：日
     * %H：小时  %M：分钟  %S：秒
     * %z：时区偏移，例如+0800。
     *
     * static：不依赖某个FileLogger对象的成员。
     */
    static string formatTime(time_t when,const char* pattern){
        tm local_time{};
        // localtime_r将结果写入local_time，失败时返回空指针
        if(::localtime_r(&when,&local_time)==nullptr){
            throw runtime_error("本地时间转换失败");
        }
        ostringstream output;
        output << put_time(&local_time,pattern);
        return output.str();
    }
    // 当前进程ID；同机同时运行的不同进程具有不同PID
    const pid_t process_id_;
    // 本次会话的完整日志文件路径。
    filesystem::path file_path_;
    // 文件流正常销毁时会自动关闭文件。
    ofstream file_;
    // 只在首次写入失败时提示，之后保留终端输出
    bool file_failed_ =false;

};