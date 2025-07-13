#pragma once

#include "elog/logger.h"

namespace elog {
namespace details {
class ThreadPool;
}

class AsyncLoggerBase : public Logger {
    friend class details::ThreadPool;

public:
    AsyncLoggerBase(std::string name, SinkPtr single_sink)
        : Logger(std::move(name), single_sink) {}
    AsyncLoggerBase(std::string name, std::initializer_list<SinkPtr> sinks)
        : Logger(std::move(name), sinks) {}

protected:
    void sinkMsg(const details::LogMsg& log_msg) override { enqueueMsg(log_msg); }

    // write log into sink
    virtual void sinkMsgBackend(const details::LogMsg& log_msg) = 0;
    // post log to threadpool
    virtual void enqueueMsg(const details::LogMsg& log_msg) = 0;
};
} // namespace elog
