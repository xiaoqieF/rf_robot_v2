#pragma once

#include "elog/common.h"
#include "elog/details/log_msg.h"
#include "elog/formatter.h"

namespace elog {
namespace sinks {
class Sink {
public:
    virtual ~Sink() = default;
    virtual void log(const details::LogMsg&) = 0;
    virtual void flush() = 0;
    virtual void setPattern(const std::string& pattern) = 0;

    void setLevel(LogLevel level) { level_ = level; }
    LogLevel level() const { return level_; }
    bool shouldLog(LogLevel msg_level) const { return msg_level >= level_; }

protected:
    LogLevel level_{LogLevel::TRACE};
};

template <typename Mutex>
class BaseSink : public Sink {
public:
    virtual ~BaseSink() = default;
    BaseSink()
        : formatter_(make_unique<PatternFormatter>()) {}

    BaseSink(const BaseSink&) = delete;
    BaseSink& operator=(const BaseSink&) = delete;

    void setPattern(const std::string& pattern) {
        std::lock_guard<Mutex> lock(mutex_);
        formatter_ = make_unique<PatternFormatter>(pattern);
    }

    virtual void log(const details::LogMsg&) = 0;
    virtual void flush() = 0;

protected:
    std::unique_ptr<Formatter> formatter_;
    Mutex mutex_;
};

} // namespace sinks
} // namespace elog
