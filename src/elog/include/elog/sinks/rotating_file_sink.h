#pragma once
#include "elog/details/file_helper.h"
#include "elog/details/format_helper.h"
#include "elog/sinks/null_mutex.h"
#include "elog/sinks/sink.h"

namespace elog {
namespace sinks {
template <typename Mutex>
class RotatingFileSink final : public BaseSink<Mutex> {
public:
    RotatingFileSink(std::string base_filename, size_t max_size = 20 * 1024 * 1024);

    RotatingFileSink(const RotatingFileSink&) = delete;
    RotatingFileSink& operator=(const RotatingFileSink&) = delete;

    // When Mutex is std::mutex, this two functions should be thread safe
    void log(const details::LogMsg&) override;
    void flush() override;

private:
    std::string getLogFilename();
    void rotateFile();
    std::string base_filename_;
    size_t max_size_;
    details::FileHelper file_helper_;
};

template <typename Mutex>
void RotatingFileSink<Mutex>::log(const details::LogMsg& log_msg) {
    std::lock_guard<Mutex> lock(BaseSink<Mutex>::mutex_);
    memory_buf_t formatted;
    BaseSink<Mutex>::formatter_->format(log_msg, formatted);
    if (file_helper_.written_bytes() + formatted.size() > max_size_) {
        rotateFile();
    }
    file_helper_.write(formatted);
}

template <typename Mutex>
void RotatingFileSink<Mutex>::flush() {
    std::lock_guard<Mutex> lock(BaseSink<Mutex>::mutex_);
    file_helper_.flush();
}

template <typename Mutex>
inline RotatingFileSink<Mutex>::RotatingFileSink(std::string base_filename, size_t max_size)
    : base_filename_(std::move(base_filename)),
      max_size_(max_size) {
    file_helper_.open(getLogFilename());
}

template <typename Mutex>
inline std::string RotatingFileSink<Mutex>::getLogFilename() {
    using std::chrono::milliseconds;

    std::string filename;
    auto pos = base_filename_.find_last_of('.');
    filename = (pos == std::string::npos ? base_filename_ : base_filename_.substr(0, pos));

    char time_buf[64];
    auto now = LogClock::now();
    time_t t = LogClock::to_time_t(now);
    tm tm = os::localtime(t);
    size_t size = strftime(time_buf, sizeof(time_buf), ".%Y%m%d-%H%M%S", &tm);
    auto milli = details::format_helper::timeFraction<milliseconds>(now);
    snprintf(time_buf + size, 64 - size, "%ld.log", milli.count());
    filename += time_buf;
    return filename;
}

template <typename Mutex>
inline void RotatingFileSink<Mutex>::rotateFile() {
    file_helper_.flush();
    file_helper_.reopen(getLogFilename());
}

using RotatingFileSinkMutex = RotatingFileSink<std::mutex>;
using RotatingFileSinkNullMutex = RotatingFileSink<details::NullMutex>;

} // namespace sinks
} // namespace elog
