#pragma once

#include "elog/details/log_msg.h"
#include "elog/formatter.h"
#include "elog/sinks/console_mutex.h"
#include "elog/sinks/sink.h"

namespace elog {
namespace sinks {
template <typename ConsoleMutex>
class StdColorSink : public Sink {
public:
    const string_view_t black = "\033[30m";
    const string_view_t red = "\033[31m";
    const string_view_t green = "\033[32m";
    const string_view_t yellow = "\033[33m";
    const string_view_t blue = "\033[34m";
    const string_view_t magenta = "\033[35m";
    const string_view_t cyan = "\033[36m";
    const string_view_t white = "\033[37m";
    const string_view_t reset = "\033[m";

    using mutex_t = typename ConsoleMutex::mutex_t;
    explicit StdColorSink(FILE* fd = stdout);

    StdColorSink(const StdColorSink&) = delete;
    StdColorSink& operator=(const StdColorSink&) = delete;

    void log(const details::LogMsg&) override;
    void flush() override;
    void setPattern(const std::string& pattern) override;

private:
    void printRange(const memory_buf_t& formatted, size_t start, size_t end) {
        ::fwrite(formatted.data() + start, 1, end - start, fd_);
    }
    void printColorCode(const string_view_t& color_code) {
        ::fwrite(color_code.data(), 1, color_code.size(), fd_);
    }

    FILE* fd_;
    mutex_t& mutex_;
    std::unique_ptr<Formatter> formatter_;
    std::array<string_view_t, static_cast<size_t>(LogLevel::NUM_LEVELS)> colors_;
};

template <typename ConsoleMutex>
inline StdColorSink<ConsoleMutex>::StdColorSink(FILE* fd)
    : fd_(fd),
      mutex_(ConsoleMutex::mutex()),
      formatter_(make_unique<PatternFormatter>()),
      colors_{white, cyan, green, yellow, red} {}

template <typename ConsoleMutex>
inline void StdColorSink<ConsoleMutex>::log(const details::LogMsg& log_msg) {
    std::lock_guard<mutex_t> lock(mutex_);
    memory_buf_t formatted;
    formatter_->format(log_msg, formatted);
    if (os::isColorTerminal() && log_msg.color_range_end > log_msg.color_range_start) {
        printRange(formatted, 0, log_msg.color_range_start);
        printColorCode(colors_[static_cast<size_t>(log_msg.log_level)]);
        printRange(formatted, log_msg.color_range_start, log_msg.color_range_end);
        printColorCode(reset);
        printRange(formatted, log_msg.color_range_end, formatted.size());
    } else {
        printRange(formatted, 0, formatted.size());
    }
    ::fflush(fd_);
}

template <typename ConsoleMutex>
inline void StdColorSink<ConsoleMutex>::flush() {
    ::fflush(fd_);
}

template <typename ConsoleMutex>
inline void StdColorSink<ConsoleMutex>::setPattern(const std::string& pattern) {
    std::lock_guard<mutex_t> lock(mutex_);
    formatter_ = make_unique<PatternFormatter>(pattern);
}

using StdColorSinkMutex = StdColorSink<details::ConsoleMutex>;
using StdColorSinkNullMutex = StdColorSink<details::ConsoleNullMutex>;

} // namespace sinks
} // namespace elog
