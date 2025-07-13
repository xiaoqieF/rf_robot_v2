#pragma once

#include "elog/logger_factory.h"

namespace elog {
inline Logger *defaultLogger() { return LoggerFactory::instance().defaultLogger(); }

inline void setDefaultLogger(std::unique_ptr<Logger> logger) {
    LoggerFactory::instance().setDefaultLogger(std::move(logger));
}

inline void setLogLevel(LogLevel level) { defaultLogger()->setLevel(level); }

inline void setPattern(const std::string &pattern) { defaultLogger()->setPattern(pattern); }

template <typename... Args>
void trace(format_string_t<Args...> fmt, Args &&...args) {
    defaultLogger()->trace(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
void debug(format_string_t<Args...> fmt, Args &&...args) {
    defaultLogger()->debug(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
void info(format_string_t<Args...> fmt, Args &&...args) {
    defaultLogger()->info(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
void warn(format_string_t<Args...> fmt, Args &&...args) {
    defaultLogger()->warn(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
void error(format_string_t<Args...> fmt, Args &&...args) {
    defaultLogger()->error(fmt, std::forward<Args>(args)...);
}

template <typename T>
void trace(const T &msg) {
    defaultLogger()->trace(msg);
}

template <typename T>
void debug(const T &msg) {
    defaultLogger()->debug(msg);
}

template <typename T>
void info(const T &msg) {
    defaultLogger()->info(msg);
}

template <typename T>
void warn(const T &msg) {
    defaultLogger()->warn(msg);
}

template <typename T>
void error(const T &msg) {
    defaultLogger()->error(msg);
}

#ifndef ELOG_NO_SOURCE_LOC
    #define ELOG_LOG(logger, level, ...) \
        (logger)->log(elog::SourceLocation{__FILE__, __LINE__, __FUNCTION__}, level, __VA_ARGS__)
#else
    #define ELOG_LOG(logger, level, ...) (logger)->log(elog::SourceLocation{}, level, __VA_ARGS__)
#endif

#define ELOG_TRACE(...) ELOG_LOG(elog::defaultLogger(), elog::LogLevel::TRACE, __VA_ARGS__)
#define ELOG_DEBUG(...) ELOG_LOG(elog::defaultLogger(), elog::LogLevel::DEBUG, __VA_ARGS__)
#define ELOG_INFO(...)  ELOG_LOG(elog::defaultLogger(), elog::LogLevel::INFO, __VA_ARGS__)
#define ELOG_WARN(...)  ELOG_LOG(elog::defaultLogger(), elog::LogLevel::WARN, __VA_ARGS__)
#define ELOG_ERROR(...) ELOG_LOG(elog::defaultLogger(), elog::LogLevel::ERROR, __VA_ARGS__)

} // namespace elog
