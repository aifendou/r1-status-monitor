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

    // 单项观察继续保留，方便以后接入只有一项反馈的模块。
    // 内部统一走批量处理，避免维护两套状态转换逻辑。
    void observe(const FaultObservation& observation)
    {
        observeBatch(
            std::vector<FaultObservation>{observation});
    }


    // 同一条反馈消息中的多个观察结果，一次提交。
    // 当前仍由单一ROS执行线程调用更新接口。
    void observeBatch(
        const std::vector<FaultObservation>& observations)
    {
        if (observations.empty())
        {
            return;
        }

        // 先收集本次产生的事件，稍后再写文件。
        std::vector<FaultEvent> events;
        events.reserve(observations.size());

        {
            // 这把锁覆盖本批次的全部状态更新。
            std::lock_guard<std::mutex> lock(mutex_);

            // 先检查整批项目，避免更新到一半才发现名称写错。
            std::set<Key> seen;

            for (const auto& observation : observations)
            {
                const Key key{
                    observation.module_,
                    observation.code_
                };

                if (entries_.find(key) == entries_.end())
                {
                    throw std::invalid_argument(
                        "批量提交中包含尚未注册的故障项目");
                }

                if (!seen.insert(key).second)
                {
                    throw std::invalid_argument(
                        "同一批次重复提交了同一个故障项目");
                }
            }

            // 验证通过后，在同一把锁保护下更新所有项目。
            for (const auto& observation : observations)
            {
                applyObservationLocked(observation, events);
            }
        }

        // 状态锁已释放。
        // 文件写入不会占用TCP读取快照需要的状态锁。
        for (const auto& event : events)
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
    /*
    * 处理一个观察结果。
    *
    * 调用者必须已经持有mutex_。
    * 本函数不再加锁，也不写文件，只更新状态、收集事件。
    */
    void applyObservationLocked(
        const FaultObservation& observation,
        std::vector<FaultEvent>& events)
    {
        const Key key{
            observation.module_,
            observation.code_
        };

        // observeBatch已经验证过该项目存在。
        Entry& entry = entries_.at(key);

        if (observation.state_ == FaultState::Unknown)
        {
            // 未知不代表解除。
            // 保留最后一次有依据的判断。
            entry.current_.state_ = FaultState::Unknown;
            return;
        }

        FaultObservation current = observation;

        if (current.state_ == FaultState::Active
            && current.severity_ == Severity::Info)
        {
            current.severity_ = Severity::Warning;
        }

        FaultEvent event;
        bool record_event = false;

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
            current.state_ == FaultState::Healthy
            && entry.last_confirmed_ == FaultState::Active)
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

            // observed_at_留空，由日志工具填写记录时间。
            events.push_back(std::move(event));
        }

        entry.current_ = current;
        entry.last_confirmed_ = current.state_;
        entry.last_fingerprint_ = current.fingerprint_;
        entry.last_severity_ = current.severity_;
        entry.last_detail_ = current.detail_;
    }
    FileLogger& logger_;

    mutable std::mutex mutex_;
    std::map<Key, Entry> entries_;

    std::uint64_t sequence_ = 0;
};

}