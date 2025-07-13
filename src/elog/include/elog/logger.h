#pragma once

#include <string>
#include <vector>

#include "elog/common.h"
#include "elog/sinks/sink.h"

namespace elog {
class Logger {
public:
    Logger(std::string name, SinkPtr single_sink)
        : name_(std::move(name)),
          sinks_{single_sink} {}
    Logger(std::string name, std::initializer_list<SinkPtr> sinks)
        : name_(std::move(name)),
          sinks_(sinks) {}
    virtual ~Logger() = default;

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    void setLevel(LogLevel level) { level_ = level; }
    LogLevel level() const { return level_; }
    void setFlushLevel(LogLevel level) { flush_level_ = level; }
    void setPattern(std::string pattern) {
        for (auto& s : sinks_) {
            s->setPattern(pattern);
        }
    }
    void flush() {
        for (auto& sink : sinks_) {
            sink->flush();
        }
    }

    template <typename... Args>
    void log(SourceLocation loc, LogLevel level, format_string_t<Args...> fmt, Args&&... args) {
        log_(loc, level, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void log(LogLevel level, format_string_t<Args...> fmt, Args&&... args) {
        log_(SourceLocation{}, level, fmt, std::forward<Args>(args)...);
    }

    template <typename T>
    void log(LogLevel level, const T& msg) {
        log(SourceLocation{}, level, msg);
    }

    template <typename T>
    void log(SourceLocation loc, LogLevel level, const T& msg) {
        log(loc, level, "{}", msg);
    }

    template <typename... Args>
    void trace(format_string_t<Args...> fmt, Args&&... args) {
        log(LogLevel::TRACE, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void debug(format_string_t<Args...> fmt, Args&&... args) {
        log(LogLevel::DEBUG, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void info(format_string_t<Args...> fmt, Args&&... args) {
        log(LogLevel::INFO, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void warn(format_string_t<Args...> fmt, Args&&... args) {
        log(LogLevel::WARN, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void error(format_string_t<Args...> fmt, Args&&... args) {
        log(LogLevel::ERROR, fmt, std::forward<Args>(args)...);
    }

    template <typename T>
    void trace(const T& msg) {
        log(LogLevel::TRACE, msg);
    }

    template <typename T>
    void debug(const T& msg) {
        log(LogLevel::DEBUG, msg);
    }

    template <typename T>
    void info(const T& msg) {
        log(LogLevel::INFO, msg);
    }

    template <typename T>
    void warn(const T& msg) {
        log(LogLevel::WARN, msg);
    }

    template <typename T>
    void error(const T& msg) {
        log(LogLevel::ERROR, msg);
    }

protected:
    template <typename... Args>
    void log_(SourceLocation loc, LogLevel msg_level, string_view_t fmt, Args&&... args);
    // by default, sink msg synchronously
    virtual void sinkMsg(const details::LogMsg& log_msg) {
        for (auto& sink : sinks_) {
            if (sink->shouldLog(log_msg.log_level)) {
                sink->log(log_msg);
            }
            if (log_msg.log_level >= flush_level_) {
                sink->flush();
            }
        }
    }

    std::string name_;
    std::vector<SinkPtr> sinks_;
    // Msg only be logged when it's level >= level_
    LogLevel level_{LogLevel::TRACE};
    // When msg_level >= flush_level_, flush sinks immediately
    LogLevel flush_level_{LogLevel::WARN};
};

template <typename... Args>
void Logger::log_(SourceLocation loc, LogLevel msg_level, string_view_t fmt, Args&&... args) {
    if (msg_level < level_) {
        return;
    }
    // for synchronous logger, use local memory buf is ok, use it before buf is destroyed
    memory_buf_t dest;
    fmt::vformat_to(fmt::appender(dest), fmt, fmt::make_format_args(args...));
    details::LogMsg log_msg(loc, name_, msg_level, string_view_t(dest.data(), dest.size()));
    sinkMsg(log_msg);
}

} // namespace elog
