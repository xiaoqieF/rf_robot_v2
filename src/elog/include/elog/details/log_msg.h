#pragma once

#include "elog/common.h"
#include "elog/details/os.h"

namespace elog {
namespace details {
struct LogMsg {
    LogMsg() = default;
    LogMsg(LogClock::time_point log_time,
           SourceLocation loc,
           string_view_t logger_name,
           LogLevel level,
           string_view_t msg);
    LogMsg(SourceLocation loc, string_view_t logger_name, LogLevel level, string_view_t msg);
    LogMsg(string_view_t logger_name, LogLevel level, string_view_t msg);

    // copy or assign is ok
    LogMsg(const LogMsg&) = default;
    LogMsg& operator=(const LogMsg&) = default;

    string_view_t logger_name;
    LogLevel log_level = LogLevel::TRACE;
    LogClock::time_point time;
    size_t thread_id = 0;
    SourceLocation source;
    string_view_t payload;

    // be set at flat_formatter
    mutable size_t color_range_start = 0;
    mutable size_t color_range_end = 0;
};

inline LogMsg::LogMsg(LogClock::time_point log_time,
                      SourceLocation loc,
                      string_view_t logger_name,
                      LogLevel level,
                      string_view_t msg)
    : logger_name(logger_name),
      log_level(level),
      time(log_time),
      source(loc),
      payload(msg) {
    thread_id = os::getThreadId();
}

inline LogMsg::LogMsg(SourceLocation loc,
                      string_view_t logger_name,
                      LogLevel level,
                      string_view_t msg)
    : LogMsg(LogClock::now(), loc, logger_name, level, msg) {}

inline LogMsg::LogMsg(string_view_t logger_name, LogLevel level, string_view_t msg)
    : LogMsg(LogClock::now(), SourceLocation{}, logger_name, level, msg) {}

} // namespace details
} // namespace elog