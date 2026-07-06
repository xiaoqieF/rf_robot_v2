#pragma once

#include "elog/elog.h"

namespace rf_local_planner
{

#define L_PLANNER_INFO(fmt_str, ...) \
    do { \
        elog::info("[rf_local_planner] " fmt_str, ##__VA_ARGS__); \
    } while(0)

#define L_PLANNER_WARN(fmt_str, ...) \
    do { \
        elog::warn("[rf_local_planner] " fmt_str, ##__VA_ARGS__); \
    } while(0)

#define L_PLANNER_ERROR(fmt_str, ...) \
    do { \
        elog::error("[rf_local_planner] " fmt_str, ##__VA_ARGS__); \
    } while(0)

} // namespace rf_local_planner
