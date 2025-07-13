#pragma once

#define FMT_HEADER_ONLY // use fmtlib header only
// #define ELOG_NO_SOURCE_LOC

#include <chrono>
#include <memory>
#include <mutex>

#include "elog/fmt/core.h"

namespace elog {
using string_view_t = fmt::basic_string_view<char>;
using memory_buf_t = fmt::basic_memory_buffer<char, 250>;
using LogClock = std::chrono::system_clock;

namespace sinks {
class Sink;
}
using SinkPtr = std::shared_ptr<sinks::Sink>;

template <typename... Args>
using format_string_t = fmt::format_string<Args...>;

enum class LogLevel : int { TRACE = 0, DEBUG = 1, INFO = 2, WARN = 3, ERROR = 4, NUM_LEVELS = 5 };

#define ELOG_LEVEL_NAME_TRACE   elog::string_view_t("trace", 5)
#define ELOG_LEVEL_NAME_DEBUG   elog::string_view_t("debug", 5)
#define ELOG_LEVEL_NAME_INFO    elog::string_view_t("info", 4)
#define ELOG_LEVEL_NAME_WARNING elog::string_view_t("warning", 7)
#define ELOG_LEVEL_NAME_ERROR   elog::string_view_t("error", 5)

#if __cplusplus >= 201703L
constexpr
#endif
    static string_view_t level_string_views[]{ELOG_LEVEL_NAME_TRACE, ELOG_LEVEL_NAME_DEBUG,
                                              ELOG_LEVEL_NAME_INFO, ELOG_LEVEL_NAME_WARNING,
                                              ELOG_LEVEL_NAME_ERROR};
static const char* SIMPLE_LEVEL_NAMES[]{"T", "D", "I", "W", "E"};

inline const string_view_t getLevelName(LogLevel log_level) {
    return level_string_views[static_cast<int>(log_level)];
}

#if __cplusplus >= 201402L
using std::enable_if_t;
using std::make_unique;
#else
template <bool B, class T = void>
using enable_if_t = typename std::enable_if<B, T>::type;

template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
    static_assert(!std::is_array<T>::value, "Arrays not supported");
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}
#endif

struct SourceLocation {
    constexpr SourceLocation() = default;
    constexpr SourceLocation(const char* filename, int line, const char* func_name)
        : filename(filename),
          line(line),
          function_name(func_name) {}
    bool empty() const { return line == 0; }

    const char* filename = nullptr;
    int line = 0;
    const char* function_name = nullptr;
};

} // namespace elog