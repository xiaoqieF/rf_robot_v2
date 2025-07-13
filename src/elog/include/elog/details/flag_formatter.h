#pragma once

#include <chrono>
#include <ctime>

#include "elog/details/format_helper.h"
#include "elog/details/log_msg.h"

namespace elog {
namespace details {
class FlagFormatter {
public:
    virtual ~FlagFormatter() = default;
    virtual void format(const details::LogMsg& msg, const tm& tm_time, memory_buf_t& dest) = 0;
};

// '%n'
class LogNameFormatter final : public FlagFormatter {
public:
    void format(const details::LogMsg& msg, const tm&, memory_buf_t& dest) override {
        format_helper::appendStringView(msg.logger_name, dest);
    }
};

// '%l'
class LogLevelFormatter final : public FlagFormatter {
public:
    void format(const details::LogMsg& msg, const tm&, memory_buf_t& dest) override {
        msg.color_range_start = dest.size();
        const string_view_t level_name = getLevelName(msg.log_level);
        format_helper::appendStringView(level_name, dest);
        msg.color_range_end = dest.size();
    }
};

// '%L'
class SimpleLogLevelFormatter final : public FlagFormatter {
public:
    void format(const details::LogMsg& msg, const tm&, memory_buf_t& dest) override {
        msg.color_range_start = dest.size();
        const string_view_t level_name = SIMPLE_LEVEL_NAMES[static_cast<int>(msg.log_level)];
        format_helper::appendStringView(level_name, dest);
        msg.color_range_end = dest.size();
    }
};

// '%Y'
class YearFormatter final : public FlagFormatter {
public:
    void format(const details::LogMsg&, const tm& tm_time, memory_buf_t& dest) override {
        format_helper::appendInt(tm_time.tm_year + 1900, dest);
    }
};

// '%M'
class MonthFormatter final : public FlagFormatter {
public:
    void format(const details::LogMsg&, const tm& tm_time, memory_buf_t& dest) override {
        format_helper::pad2(tm_time.tm_mon + 1, dest);
    }
};

// '%D'
class DayFormatter final : public FlagFormatter {
public:
    void format(const details::LogMsg&, const tm& tm_time, memory_buf_t& dest) override {
        format_helper::pad2(tm_time.tm_mday, dest);
    }
};

// '%h'
class HourFormatter final : public FlagFormatter {
public:
    void format(const details::LogMsg&, const tm& tm_time, memory_buf_t& dest) override {
        format_helper::pad2(tm_time.tm_hour, dest);
    }
};

// '%m'
class MinFormatter final : public FlagFormatter {
public:
    void format(const details::LogMsg&, const tm& tm_time, memory_buf_t& dest) override {
        format_helper::pad2(tm_time.tm_min, dest);
    }
};

// '%s'
class SecondFormatter final : public FlagFormatter {
public:
    void format(const details::LogMsg&, const tm& tm_time, memory_buf_t& dest) override {
        format_helper::pad2(tm_time.tm_sec, dest);
    }
};

// '%e'
class MilliFormatter final : public FlagFormatter {
public:
    void format(const details::LogMsg& msg, const tm&, memory_buf_t& dest) override {
        auto millis = format_helper::timeFraction<std::chrono::milliseconds>(msg.time);
        format_helper::pad3(static_cast<uint32_t>(millis.count()), dest);
    }
};

// '%f'
class MicroFormatter final : public FlagFormatter {
public:
    void format(const details::LogMsg& msg, const tm&, memory_buf_t& dest) override {
        auto micro = format_helper::timeFraction<std::chrono::microseconds>(msg.time);
        format_helper::pad_uint(static_cast<uint32_t>(micro.count()), 6, dest);
    }
};

// '%t'
class TidFormatter final : public FlagFormatter {
public:
    void format(const details::LogMsg& msg, const tm&, memory_buf_t& dest) override {
        format_helper::appendInt(msg.thread_id, dest);
    }
};

// '%v'
class PayloadFormatter final : public FlagFormatter {
public:
    void format(const details::LogMsg& msg, const tm&, memory_buf_t& dest) override {
        format_helper::appendStringView(msg.payload, dest);
    }
};

// '%u'
class SourceLocFormatter final : public FlagFormatter {
public:
    static const char* basename(const char* filename) {
        // if the size is 2 (1 character + null terminator) we can use the more efficient strrchr
        // the branch will be elided by optimizations
        if (sizeof(os::FOLDER_SEP) == 2) {
            const char* rv = std::strrchr(filename, os::FOLDER_SEP[0]);
            return rv != nullptr ? rv + 1 : filename;
        } else {
            const std::reverse_iterator<const char*> begin(filename + std::strlen(filename));
            const std::reverse_iterator<const char*> end(filename);

            const auto it = std::find_first_of(begin, end, std::begin(os::FOLDER_SEP),
                                               std::end(os::FOLDER_SEP) - 1);
            return it != end ? it.base() : filename;
        }
    }

    void format(const details::LogMsg& msg, const tm&, memory_buf_t& dest) override {
        if (msg.source.empty()) {
            return;
        }
        const char* name = basename(msg.source.filename);
        format_helper::appendStringView(name, dest);
        dest.push_back(':');
        format_helper::appendInt(msg.source.line, dest);
    }
};

// '%w'
class FuncNameFormatter final : public FlagFormatter {
public:
    void format(const details::LogMsg& msg, const tm&, memory_buf_t& dest) override {
        if (msg.source.empty()) {
            return;
        }
        format_helper::appendStringView(msg.source.function_name, dest);
    }
};

// To display normal chars
class AggregateFormatter final : public FlagFormatter {
public:
    void addChar(char ch) { str_ += ch; }
    void format(const details::LogMsg&, const tm&, memory_buf_t& dest) {
        format_helper::appendStringView(str_, dest);
    }

private:
    std::string str_;
};

class FullFormatter final : public FlagFormatter {
public:
    void format(const details::LogMsg& msg, const tm& tm_time, memory_buf_t& dest) {
        using std::chrono::duration_cast;
        using std::chrono::microseconds;
        using std::chrono::seconds;

        auto duration = msg.time.time_since_epoch();
        auto secs = duration_cast<seconds>(duration);

        // cache time string within 1 second
        if (cached_seconds_ != secs || cached_time_str_.size() == 0) {
            cached_time_str_.clear();
            cached_time_str_.push_back('[');
            format_helper::appendInt(tm_time.tm_year + 1900, cached_time_str_);
            cached_time_str_.push_back('-');
            format_helper::pad2(tm_time.tm_mon + 1, cached_time_str_);
            cached_time_str_.push_back('-');
            format_helper::pad2(tm_time.tm_mday, cached_time_str_);
            cached_time_str_.push_back(' ');
            format_helper::pad2(tm_time.tm_hour, cached_time_str_);
            cached_time_str_.push_back(':');
            format_helper::pad2(tm_time.tm_min, cached_time_str_);
            cached_time_str_.push_back(':');
            format_helper::pad2(tm_time.tm_sec, cached_time_str_);
            cached_time_str_.push_back('.');
            cached_seconds_ = secs;
        }
        dest.append(cached_time_str_.begin(), cached_time_str_.end());

        auto micro = format_helper::timeFraction<microseconds>(msg.time);
        format_helper::pad_uint(static_cast<uint32_t>(micro.count()), 6, dest);
        dest.push_back(']');
        dest.push_back(' ');

        dest.push_back('[');
        msg.color_range_start = dest.size();
        format_helper::appendStringView(getLevelName(msg.log_level), dest);
        // format_helper::appendStringView(SIMPLE_LEVEL_NAMES[static_cast<int>(msg.log_level)],
        // dest);
        msg.color_range_end = dest.size();
        dest.push_back(']');
        dest.push_back(' ');

        format_helper::appendStringView(tid_prefix, dest);
        format_helper::appendInt(msg.thread_id, dest);
        dest.push_back(']');
        dest.push_back(' ');

        if (!msg.source.empty()) {
            dest.push_back('[');
            const char* short_filename = SourceLocFormatter::basename(msg.source.filename);
            format_helper::appendStringView(short_filename, dest);
            dest.push_back(':');
            format_helper::appendInt(msg.source.line, dest);
            dest.push_back(']');
            dest.push_back(' ');
        }
        format_helper::appendStringView(msg.payload, dest);
    }

private:
    std::chrono::seconds cached_seconds_{0};
    memory_buf_t cached_time_str_;

    const std::string tid_prefix{"[tid:"};
};

} // namespace details
} // namespace elog
