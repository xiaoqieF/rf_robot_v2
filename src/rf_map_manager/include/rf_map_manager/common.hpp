#pragma once

#include "elog/elog.h"

#define MAP_MANAGER_INFO(fmt_str, ...) \
    do { \
        elog::info("[rf_map_manager] " fmt_str, ##__VA_ARGS__); \
    } while(0)

#define MAP_MANAGER_WARN(fmt_str, ...) \
    do { \
        elog::warn("[rf_map_manager] " fmt_str, ##__VA_ARGS__); \
    } while(0)
