/**
 * 功能：管理不同来源的报警、去重、识别变化与解除、保留事件历史；
 *      负责的是“异常的生命周期”
 *  首次观察到报警
        ↓
    记录出现事件
        ↓
    报警持续，状态没变化：不重复写出现事件
        ↓
    报警内容变化：记录变化
        ↓
    确认报警解除：记录解除事件
 */
#pragma once

#include "MonitorTypes.h"
#include "FileLogger.h"

#include <map>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace r1_monitor
{

class FaultManager
{
public:
    // TCP线程读取的一份故障状态副本。
    struct Snapshot
    {
        bool enabled_ = false;

        std::vector<FaultObservation> active_;
        std::vector<std::string> unknown_modules_;
    };

    explicit FaultManager(FileLogger& logger)
        : logger_(logger)
    {
    }

    // 只有实际启用监测的项目才注册。
    // 尚未接入的夹爪、摄像机暂时不要注册。
    void registerFault(
        const std::string& module,
        const std::string& code)
    {
        if (module.empty() || code.empty())
        {
            throw std::invalid_argument(
                "故障模块和标识不能为空");
        }

        std::lock_guard<std::mutex> lock(mutex_);

        const Key key{module, code};

        if (entries_.find(key) != entries_.end())
        {
            return;
        }

        Entry entry;
        entry.current_.module_ = module;
        entry.current_.code_ = code;

        // 新注册的项目默认为Unknown。
        // 收到有效反馈前，不宣称正常。
        entries_.emplace(key, entry);
    }

    // 更新接口当前由单一ROS执行线程调用。
    // TCP线程可以同时调用snapshot()读取。
    void observe(const FaultObservation& observation)
    {
        FaultEvent event;
        bool record_event = false;

        {
            std::lock_guard<std::mutex> lock(mutex_);

            const Key key{
                observation.module_,
                observation.code_
            };

            auto it = entries_.find(key);

            if (it == entries_.end())
            {
                throw std::invalid_argument(
                    "提交了尚未注册的故障项目");
            }

            Entry& entry = it->second;

            // Unknown不等于故障解除。
            if (observation.state_ == FaultState::Unknown)
            {
                entry.current_.state_ = FaultState::Unknown;
                return;
            }

            FaultObservation current = observation;

            // 已触发故障至少显示Warning，避免误显示为普通信息。
            if (current.state_ == FaultState::Active
                && current.severity_ == Severity::Info)
            {
                current.severity_ = Severity::Warning;
            }

            if (current.state_ == FaultState::Active)
            {
                if (entry.last_confirmed_ == FaultState::Unknown)
                {
                    event.kind_ = FaultEventKind::InitialActive;
                    record_event = true;
                }
                else if (
                    entry.last_confirmed_ == FaultState::Healthy)
                {
                    event.kind_ = FaultEventKind::Raised;
                    record_event = true;
                }
                else if (
                    current.fingerprint_ != entry.last_fingerprint_
                    || current.severity_ != entry.last_severity_)
                {
                    event.kind_ = FaultEventKind::Changed;
                    record_event = true;
                }
            }
            else if (
                entry.last_confirmed_ == FaultState::Active)
            {
                event.kind_ = FaultEventKind::Cleared;
                record_event = true;
            }

            if (record_event)
            {
                event.sequence_ = ++sequence_;
                event.module_ = current.module_;
                event.code_ = current.code_;

                event.severity_ =
                    event.kind_ == FaultEventKind::Cleared
                        ? Severity::Info
                        : current.severity_;

                if (event.kind_ == FaultEventKind::Changed
                    || event.kind_ == FaultEventKind::Cleared)
                {
                    event.detail_ =
                        entry.last_detail_
                        + " -> "
                        + current.detail_;
                }
                else
                {
                    event.detail_ = current.detail_;
                }

                // observed_at_留空，由日志工具在写入时填本地时间。
            }

            entry.current_ = current;
            entry.last_confirmed_ = current.state_;
            entry.last_fingerprint_ = current.fingerprint_;
            entry.last_severity_ = current.severity_;
            entry.last_detail_ = current.detail_;
        }

        // 已释放故障状态锁，再进行文件操作。
        // 文件写入不会阻塞TCP线程读取状态锁。
        if (record_event)
        {
            logger_.writeFault(event);
        }
    }

    // 某个已启用模块的数据失效。
    // 保留最后一次确认结果，但当前状态改为Unknown。
    // 不生成虚假的“故障解除”事件。
    void markModuleUnknown(const std::string& module)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        for (auto& item : entries_)
        {
            Entry& entry = item.second;

            if (entry.current_.module_ == module)
            {
                entry.current_.state_ = FaultState::Unknown;
            }
        }
    }

    Snapshot snapshot() const
    {
        std::lock_guard<std::mutex> lock(mutex_);

        Snapshot result;
        result.enabled_ = !entries_.empty();

        std::set<std::string> unknown_modules;

        for (const auto& item : entries_)
        {
            const auto& current = item.second.current_;

            if (current.state_ == FaultState::Active)
            {
                result.active_.push_back(current);
            }
            else if (current.state_ == FaultState::Unknown)
            {
                unknown_modules.insert(current.module_);
            }
        }

        result.unknown_modules_.assign(
            unknown_modules.begin(),
            unknown_modules.end());

        return result;
    }

private:
    using Key = std::pair<std::string, std::string>;

    struct Entry
    {
        FaultObservation current_;

        // 即使当前反馈变成Unknown，
        // 也保留最后一次有依据的判断。
        FaultState last_confirmed_ = FaultState::Unknown;

        std::string last_fingerprint_;
        Severity last_severity_ = Severity::Warning;
        std::string last_detail_;
    };

    FileLogger& logger_;

    mutable std::mutex mutex_;
    std::map<Key, Entry> entries_;

    std::uint64_t sequence_ = 0;
};

}