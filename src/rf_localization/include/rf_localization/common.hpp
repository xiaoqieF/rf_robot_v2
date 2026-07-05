#pragma once

#include "elog/elog.h"

namespace rf_localization
{

#define LOCALIZATION_INFO(fmt_str, ...) \
    do { \
        elog::info("[rf_localization] " fmt_str, ##__VA_ARGS__); \
    } while(0)

#define LOCALIZATION_WARN(fmt_str, ...) \
    do { \
        elog::warn("[rf_localization] " fmt_str, ##__VA_ARGS__); \
    } while(0)

#define LOCALIZATION_ERROR(fmt_str, ...) \
    do { \
        elog::error("[rf_localization] " fmt_str, ##__VA_ARGS__); \
    } while(0)

} // namespace rf_localization
