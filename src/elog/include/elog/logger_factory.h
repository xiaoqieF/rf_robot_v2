#pragma once

#include "elog/async_logger.h"
#include "elog/common.h"
#include "elog/logger.h"
#include "elog/sinks/std_color_sink.h"

namespace elog {

namespace sync_factory {
template <typename Sink, typename... SinkArgs>
std::unique_ptr<Logger> create(std::string logger_name, SinkArgs &&...args) {
    auto sinker = std::make_shared<Sink>(std::forward<SinkArgs>(args)...);
    return make_unique<Logger>(std::move(logger_name), sinker);
}
}; // namespace sync_factory

namespace async_factory {
template <typename Sink, typename... SinkArgs>
std::unique_ptr<Logger> create(std::string logger_name, SinkArgs &&...args) {
    auto sinker = std::make_shared<Sink>(std::forward<SinkArgs>(args)...);
    return make_unique<AsyncLogger>(std::move(logger_name), sinker);
}
} // namespace async_factory

class LoggerFactory {
public:
    static LoggerFactory &instance() {
        static LoggerFactory loggerFactory;
        return loggerFactory;
    }

    void setDefaultLogger(std::unique_ptr<Logger> logger) { default_logger_ = std::move(logger); }
    Logger *defaultLogger() { return default_logger_.get(); }

private:
    LoggerFactory() { default_logger_ = sync_factory::create<sinks::StdColorSinkMutex>(""); };
    std::unique_ptr<Logger> default_logger_;
};

} // namespace elog
