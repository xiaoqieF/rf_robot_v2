#pragma once

#include <string>
#include <vector>

namespace rf_costmap
{

struct CostmapConfig
{
    std::string map_name;
    // unit is meter
    double resolution{0.05};
    // used in local costmap
    unsigned int width{3};
    unsigned int height{3};
    bool rolling_window{false};
    unsigned int update_rate{10}; // Hz
    unsigned int publish_rate{5}; // Hz
    std::vector<std::string> layer_names;
};

} // namespace rf_costmap