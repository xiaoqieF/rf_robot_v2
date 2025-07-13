#pragma once

#include <mutex>

#include "elog/sinks/null_mutex.h"

namespace elog {
namespace details {
// All console sinks shared singleton mutex
struct ConsoleMutex {
    using mutex_t = std::mutex;
    static std::mutex& mutex() {
        static std::mutex m;
        return m;
    }
};

struct ConsoleNullMutex {
    using mutex_t = NullMutex;
    static NullMutex& mutex() {
        static NullMutex m;
        return m;
    }
};

} // namespace details
} // namespace elog
