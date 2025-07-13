#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

#include "elog/async_logger_base.h"
#include "elog/details/async_log_msg.h"

namespace elog {
namespace details {
class ThreadPool {
public:
    explicit ThreadPool(Logger* own_logger, int thread_num = 1, int interval = 3)
        : own_logger_(own_logger),
          flush_interval_(interval) {
        running_ = true;
        threads_.reserve(thread_num);
        for (int i = 0; i < thread_num; ++i) {
            threads_.emplace_back(&ThreadPool::threadLoop, this);
        }
        flush_thread_ = std::thread(&ThreadPool::flushLoop, this);
    }

    ~ThreadPool() { stop(); }

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    void setFlushInterval(int second) { flush_interval_ = second; }

    void postLog(const details::LogMsg& msg, AsyncLoggerBase* logger) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        buffer_.emplace_back(msg, logger);
        queue_cv_.notify_one();
    }

    void stop() {
        running_ = false;
        queue_cv_.notify_all();
        flush_cv_.notify_all();
        for (auto& t : threads_) {
            t.join();
        }
        flush_thread_.join();
    }

private:
    void threadLoop();
    void flushLoop();
    void consumeAndFlush();
    std::mutex queue_mutex_;
    std::mutex flush_mutex_;
    std::condition_variable queue_cv_;
    std::condition_variable flush_cv_;
    std::vector<std::thread> threads_;
    std::thread flush_thread_;
    std::vector<details::AsyncMsg> buffer_;
    std::atomic_bool running_{false};
    Logger* own_logger_{nullptr};
    int flush_interval_;
};

inline void ThreadPool::threadLoop() {
    // when multi-consumer, log could be out-of-order
    details::AsyncMsg async_msg;
    while (running_) {
        std::vector<AsyncMsg> tmp;
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this]() { return !this->running_ || !this->buffer_.empty(); });
            if (!running_) {
                break;
            }
            buffer_.swap(tmp);
        }
        for (auto& async_msg : tmp) {
            async_msg.own_logger->sinkMsgBackend(async_msg.msg);
        }
    }
    // When program exit, ensure all logs be consumed
    consumeAndFlush();
}

// flush logs every flush_interval_ seconds
inline void ThreadPool::flushLoop() {
    while (running_) {
        std::unique_lock<std::mutex> lock(flush_mutex_);
        flush_cv_.wait_for(lock, std::chrono::seconds(flush_interval_),
                           [this]() { return !this->running_; });
        own_logger_->flush();
        if (!running_) {
            break;
        }
    }
}

inline void ThreadPool::consumeAndFlush() {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (buffer_.empty()) {
        return;
    }
    for (auto& async_msg : buffer_) {
        async_msg.own_logger->sinkMsgBackend(async_msg.msg);
    }
    own_logger_->flush();
    buffer_.clear();
}

} // namespace details
} // namespace elog
