#pragma once

#include <chrono>

namespace rf_util
{

class ExecutionTimer
{
public:
    ExecutionTimer() : start_time_(std::chrono::steady_clock::now()) {}

    void tick()
    {
        start_time_ = std::chrono::steady_clock::now();
    }

    void toc()
    {
        end_time_ = std::chrono::steady_clock::now();
    }

    std::chrono::nanoseconds elapsed() const
    {
        return end_time_ - start_time_;
    }

private:
    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point end_time_;
};

} // namespace rf_util