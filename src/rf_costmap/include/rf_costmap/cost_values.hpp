#pragma once

#include <cstdint>
#include "elog/elog.h"

namespace rf_costmap
{
inline constexpr uint8_t NO_INFORMATION = 255;
inline constexpr uint8_t LETHAL_OBSTACLE = 254;
inline constexpr uint8_t FREE_SPACE = 0;

inline constexpr int8_t OCC_GRID_UNKNOWN = -1;
inline constexpr int8_t OCC_GRID_FREE = 0;
inline constexpr int8_t OCC_GRID_OCCUPIED = 100;

#define COSTMAP_INFO(fmt_str, ...) \
    do { \
        elog::info("[rf_costmap] " fmt_str, ##__VA_ARGS__); \
    } while(0)

#define COSTMAP_WARN(fmt_str, ...) \
    do { \
        elog::warn("[rf_costmap] " fmt_str, ##__VA_ARGS__); \
    } while(0)

} // namespace rf_costmap