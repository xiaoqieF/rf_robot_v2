#pragma once

#include "elog/async_logger_base.h"
#include "elog/details/threadpool.h"

namespace elog {
// use async logger with thread_pool of only 1 thread, logs in different threads may be out-of-order
// when thread_num > 1, logs in the same thread maybe out-of-order
class AsyncLogger : public AsyncLoggerBase {
public:
    AsyncLogger(std::string name, SinkPtr single_sink, int thread_num = 1)
        : AsyncLoggerBase(name, single_sink),
          pool_(this, thread_num) {}

    AsyncLogger(std::string name, std::initializer_list<SinkPtr> sinks, int thread_num = 1)
        : AsyncLoggerBase(std::move(name), sinks),
          pool_(this, thread_num) {}

protected:
    void enqueueMsg(const details::LogMsg& log_msg) override;
    void sinkMsgBackend(const details::LogMsg& log_msg) override;
    details::ThreadPool pool_;
};

inline void AsyncLogger::enqueueMsg(const details::LogMsg& log_msg) {
    pool_.postLog(log_msg, this);
}

inline void AsyncLogger::sinkMsgBackend(const details::LogMsg& log_msg) {
    for (auto& sink : sinks_) {
        if (sink->shouldLog(log_msg.log_level)) {
            sink->log(log_msg);
        }
        if (log_msg.log_level >= flush_level_) {
            sink->flush();
        }
    }
}

} // namespace elog
