#pragma once

#include "elog/formatter.h"
#include "elog/sinks/console_mutex.h"
#include "elog/sinks/sink.h"

namespace elog {
namespace sinks {
template <typename ConsoleMutex>
class StdoutSink : public Sink {
public:
    using mutex_t = typename ConsoleMutex::mutex_t;
    explicit StdoutSink(FILE* file = stdout)
        : file_(file),
          formatter_(make_unique<PatternFormatter>()),
          mutex_(ConsoleMutex::mutex()) {}

    StdoutSink(const StdoutSink&) = delete;
    StdoutSink& operator=(const StdoutSink&) = delete;

    void log(const details::LogMsg& msg) override;
    void flush() override;
    void setPattern(const std::string&) override;

private:
    FILE* file_;
    std::unique_ptr<elog::Formatter> formatter_;
    mutex_t& mutex_;
};

template <typename ConsoleMutex>
inline void StdoutSink<ConsoleMutex>::log(const details::LogMsg& msg) {
    std::lock_guard<mutex_t> lock(mutex_);
    memory_buf_t formatted;
    formatter_->format(msg, formatted);
    ::fwrite(formatted.data(), sizeof(char), formatted.size(), file_);
    ::fflush(file_);
}

template <typename ConsoleMutex>
inline void StdoutSink<ConsoleMutex>::flush() {
    std::lock_guard<mutex_t> lock(mutex_);
    ::fflush(file_);
}

template <typename ConsoleMutex>
inline void StdoutSink<ConsoleMutex>::setPattern(const std::string& pattern) {
    std::lock_guard<mutex_t> lock(mutex_);
    formatter_ = make_unique<PatternFormatter>(pattern);
}

using StdoutSinkMutex = StdoutSink<details::ConsoleMutex>;
using StdoutSinkNullMutex = StdoutSink<details::ConsoleNullMutex>;

} // namespace sinks
} // namespace elog
