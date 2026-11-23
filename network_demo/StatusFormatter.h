/** 
 * 功能：统一处理里程计、电池、设备、故障的显示摘要
 */
#pragma once

#include "MonitorTypes.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <exception>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>

namespace r1_monitor
{

using StatusJson = nlohmann::json;


// ============================================================
// 一、公共工具：各类摘要复用，避免重复写同样的检查。
// ============================================================

// 合并两个等级，保留更需要关注的等级。
inline Severity formatterHigherSeverity(
    Severity first,
    Severity second)
{
    if (first == Severity::Error
        || second == Severity::Error)
    {
        return Severity::Error;
    }

    if (first == Severity::Warning
        || second == Severity::Warning)
    {
        return Severity::Warning;
    }

    return Severity::Info;
}


// 读取布尔字段。
// 字段不存在或类型不正确时，JSON库会抛出异常。
// 后面的保护函数会把它转换成该模块的提示。
inline bool formatterReadBool(
    const StatusJson& object,
    const char* key)
{
    return object.at(key).get<bool>();
}


// 读取有限数值，防止把字符串、null等当作正常数据。
inline double formatterReadNumber(
    const StatusJson& object,
    const char* key)
{
    const auto& value = object.at(key);

    if (!value.is_number())
    {
        throw std::invalid_argument("Expected numeric field");
    }

    const double number = value.get<double>();

    if (!std::isfinite(number))
    {
        throw std::invalid_argument("Expected finite number");
    }

    return number;
}


// 读取指定范围内的整数。
// 先检查整数类型，再检查范围，避免把19.8直接截断成19。
inline int formatterReadInt(
    const StatusJson& object,
    const char* key,
    int minimum,
    int maximum)
{
    const auto& value = object.at(key);

    if (!value.is_number_integer())
    {
        throw std::invalid_argument("Expected integer field");
    }

    const double number = value.get<double>();

    if (number < minimum || number > maximum)
    {
        throw std::invalid_argument("Integer out of range");
    }

    return static_cast<int>(number);
}


// 向总摘要追加一个模块的结果。
inline void formatterAppend(
    FormatResult& total,
    const FormatResult& part)
{
    if (!total.text_.empty())
    {
        total.text_ += " | ";
    }

    total.text_ += part.text_;

    total.severity_ = formatterHigherSeverity(
        total.severity_,
        part.severity_);
}


// 对单个摘要函数提供异常保护。
// 某一部分字段错误，不影响其他模块继续显示。
template<typename Formatter>
inline FormatResult formatterProtect(
    const StatusJson& root,
    const char* module,
    Formatter formatter)
{
    try
    {
        return formatter(root);
    }
    catch (const std::exception&)
    {
        return {
            std::string(module)
                + "=响应字段缺失或无效，状态未知",
            Severity::Warning
        };
    }
}


// ============================================================
// 二、里程计摘要。
// ============================================================

inline FormatResult formatOdom(const StatusJson& root)
{
    const bool received =
        formatterReadBool(root, "odom_received");

    if (!received)
    {
        return {
            "odom=尚未收到",
            Severity::Warning
        };
    }

    const bool fresh =
        formatterReadBool(root, "odom_fresh");

    std::ostringstream text;
    text.imbue(std::locale::classic());

    text << std::fixed << std::setprecision(2)
         << "odom"
         << " x=" << formatterReadNumber(root, "x")
         << " y=" << formatterReadNumber(root, "y")
         << " vx=" << formatterReadNumber(root, "vx")
         << " wz=" << formatterReadNumber(root, "wz");

    if (!fresh)
    {
        text << " [数据过期]";

        return {
            text.str(),
            Severity::Warning
        };
    }

    return {
        text.str(),
        Severity::Info
    };
}


// ============================================================
// 三、电池摘要。
// 低电量只产生运行提醒，不生成FaultObservation或FaultEvent。
// ============================================================

inline FormatResult formatBattery(const StatusJson& root)
{
    const auto& battery = root.at("battery");

    const bool received =
        formatterReadBool(battery, "received");

    if (!received)
    {
        return {
            "battery=尚未收到",
            Severity::Warning
        };
    }

    const bool fresh =
        formatterReadBool(battery, "fresh");

    if (!fresh)
    {
        return {
            "battery=数据过期",
            Severity::Warning
        };
    }

    const int percentage =
        formatterReadInt(battery, "percentage", 0, 100);

    // 阈值判断仍由服务端完成，客户端读取结论。
    const bool low_battery =
        formatterReadBool(battery, "low_battery");

    std::string text =
        "battery=" + std::to_string(percentage) + "%";

    if (low_battery)
    {
        text += " [低电量提醒]";

        return {
            text,
            Severity::Warning
        };
    }

    text += " [正常]";

    return {
        text,
        Severity::Info
    };
}


// ============================================================
// 四、设备状态摘要。
// 原始编码含义未完全确认，先显示数值，不猜测设备故障。
// ============================================================

inline FormatResult formatDevice(const StatusJson& root)
{
    const auto& device = root.at("device");

    const bool received =
        formatterReadBool(device, "received");

    if (!received)
    {
        return {
            "device=尚未收到",
            Severity::Warning
        };
    }

    const bool fresh =
        formatterReadBool(device, "fresh");

    if (!fresh)
    {
        return {
            "device=数据过期",
            Severity::Warning
        };
    }

    const int control_mode =
        formatterReadInt(device, "control_mode", 0, 255);

    const int power_mode =
        formatterReadInt(device, "power_mode", 0, 255);

    const int motor_mode =
        formatterReadInt(device, "motor_mode", 0, 255);

    const int e_stop =
        formatterReadInt(device, "e_stop", 0, 255);

    std::ostringstream text;

    text << "device"
         << " ctrl=" << control_mode
         << " power=" << power_mode
         << " motor=" << motor_mode
         << " estop(raw)=" << e_stop;

    return {
        text.str(),
        Severity::Info
    };
}


// ============================================================
// 五、统一故障摘要。
// 读取任意模块的当前故障，不局限于底盘。
// 本函数只显示，不写入故障日志。
// ============================================================

inline FormatResult formatFaults(const StatusJson& root)
{
    if (!root.contains("faults"))
    {
        // 服务端尚未实现故障模块时，明确显示未接入。
        // 不把“尚未接入”说成“没有故障”。
        return {
            "faults=未接入",
            Severity::Info
        };
    }

    const auto& faults = root.at("faults");

    const bool enabled =
        formatterReadBool(faults, "enabled");

    if (!enabled)
    {
        return {
            "faults=未启用",
            Severity::Info
        };
    }

    const auto& active = faults.at("active");
    const auto& unknown = faults.at("unknown_modules");

    if (!active.is_array() || !unknown.is_array())
    {
        throw std::invalid_argument("Expected fault arrays");
    }

    if (active.empty() && unknown.empty())
    {
        return {
            "faults=已启用监测项未见故障",
            Severity::Info
        };
    }

    std::ostringstream text;
    Severity severity = Severity::Info;

    if (!active.empty())
    {
        text << "faults=故障[";

        bool first = true;

        for (const auto& fault : active)
        {
            const std::string module =
                fault.at("module").get<std::string>();

            const std::string code =
                fault.at("code").get<std::string>();

            const std::string level =
                fault.at("severity").get<std::string>();

            if (level != "warning" && level != "error")
            {
                throw std::invalid_argument(
                    "Unsupported fault severity");
            }

            if (!first)
            {
                text << "; ";
            }

            first = false;

            // 日常摘要只展示模块和故障标识，避免文字过长。
            // 详细描述由后续故障事件日志保存。
            text << module << "/" << code;

            const Severity fault_severity =
                level == "error"
                    ? Severity::Error
                    : Severity::Warning;

            severity = formatterHigherSeverity(
                severity,
                fault_severity);
        }

        text << "]";
    }
    else
    {
        text << "faults=状态未知";
    }

    if (!unknown.empty())
    {
        text << " [反馈未知:";

        bool first = true;

        for (const auto& module : unknown)
        {
            if (!first)
            {
                text << ",";
            }

            first = false;
            text << module.get<std::string>();
        }

        text << "]";

        severity = formatterHigherSeverity(
            severity,
            Severity::Warning);
    }

    return {
        text.str(),
        severity
    };
}


// ============================================================
// 六、客户端只调用这个总入口。
// ============================================================

inline FormatResult formatStateResponse(
    const std::string& response)
{
    try
    {
        const auto root = StatusJson::parse(response);

        if (!root.is_object())
        {
            return {
                "状态响应不是JSON对象",
                Severity::Error
            };
        }

        if (!formatterReadBool(root, "ok"))
        {
            return {
                "状态查询失败："
                    + root.value(
                        "error",
                        std::string("UNKNOWN_ERROR")),
                Severity::Error
            };
        }

        FormatResult total;

        formatterAppend(
            total,
            formatterProtect(root, "odom", formatOdom));

        formatterAppend(
            total,
            formatterProtect(root, "battery", formatBattery));

        formatterAppend(
            total,
            formatterProtect(root, "device", formatDevice));

        formatterAppend(
            total,
            formatterProtect(root, "faults", formatFaults));

        return total;
    }
    catch (const std::exception& error)
    {
        return {
            std::string("状态响应解析失败：") + error.what(),
            Severity::Error
        };
    }
}

} // namespace r1_monitor