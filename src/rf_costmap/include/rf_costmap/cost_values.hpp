#pragma once

#include <cstdint>

namespace rf_costmap
{
inline constexpr uint8_t NO_INFORMATION = 255;
inline constexpr uint8_t LETHAL_OBSTACLE = 254;
inline constexpr uint8_t FREE_SPACE = 0;

inline constexpr int8_t OCC_GRID_UNKNOWN = -1;
inline constexpr int8_t OCC_GRID_FREE = 0;
inline constexpr int8_t OCC_GRID_OCCUPIED = 100;

} // namespace rf_costmap