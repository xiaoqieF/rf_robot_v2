#pragma once

#include "elog/elog.h"

namespace rf_global_planner
{

#define G_PLANNER_INFO(fmt_str, ...) \
    do { \
        elog::info("[rf_global_planner] " fmt_str, ##__VA_ARGS__); \
    } while(0)

#define G_PLANNER_WARN(fmt_str, ...) \
    do { \
        elog::warn("[rf_global_planner] " fmt_str, ##__VA_ARGS__); \
    } while(0)

#define G_PLANNER_ERROR(fmt_str, ...) \
    do { \
        elog::error("[rf_global_planner] " fmt_str, ##__VA_ARGS__); \
    } while(0)

}