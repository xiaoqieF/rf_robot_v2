#include "rf_costmap/inflation_layer.hpp"
#include "rf_costmap/cost_values.hpp"
#include "rf_costmap/master_costmap.hpp"
#include <cmath>
#include <cstdint>
#include <vector>

namespace rf_costmap
{

InflationLayer::InflationLayer()
    :cell_inflation_radius_(0), cache_length_(0)
{

}

void InflationLayer::onInitialize()
{
    seen_.clear();
    cached_distances_.clear();
    cached_costs_.clear();

    matchSize();
}

void InflationLayer::reset()
{
    matchSize();
}

void InflationLayer::matchSize()
{
    last_min_x_ = std::numeric_limits<double>::lowest();
    last_min_y_ = std::numeric_limits<double>::lowest();
    last_max_x_ = std::numeric_limits<double>::max();
    last_max_y_ = std::numeric_limits<double>::max();

    auto costmap = master_costmap_->getCostmap();
    cell_inflation_radius_ = static_cast<unsigned int>(
        inflation_radius / costmap->getResolution());
    seen_ = std::vector<bool>(costmap->getSizeX() * costmap->getSizeY(), false);
    computeCaches();
}

void InflationLayer::computeCaches()
{
    if (cell_inflation_radius_ == 0) {
        return; // No inflation radius, no need to compute caches
    }

    cache_length_ = cell_inflation_radius_ + 2;

    cached_costs_.resize(cache_length_ * cache_length_);
    cached_distances_.resize(cache_length_ * cache_length_);

    for (unsigned int i = 0; i < cache_length_; ++i) {
        for (unsigned int j = 0; j < cache_length_; ++j) {
            cached_distances_[i * cache_length_ + j] = std::hypot(i, j);
        }
    }

    for (unsigned int i = 0; i < cache_length_; ++i) {
        for (unsigned int j = 0; j < cache_length_; ++j) {
            double distance = cached_distances_[i * cache_length_ + j];
            cached_costs_[i * cache_length_ + j] = computeCost(distance);
        }
    }

    int max_level = computeLevelTable();
    inflation_cells_.clear();
    inflation_cells_.resize(max_level + 1);
}

uint8_t InflationLayer::computeCost(double distance) const
{
    auto resolution = master_costmap_->getCostmap()->getResolution();
    double world_distance = distance * resolution;
    uint8_t cost = 0;
    // distance in cell
    if (distance == 0) {
        cost = LETHAL_OBSTACLE;
    } else if (world_distance < inscribed_radius) {
        cost = INSCRIBED_INFLATED_OBSTACLE;
    } else {
        double factor = std::exp(-cost_scaling_factor * (world_distance - inscribed_radius));
        cost = static_cast<uint8_t>((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
    }
    return cost;
}

int InflationLayer::computeLevelTable()
{
    // Use level vector rather than priority queue to improve performance
    // precompute the level table
    const int r = cell_inflation_radius_ + 2;
    const int sz = r * 2 + 1;

    std::vector<std::pair<int, int>> points;
    points.reserve(sz * sz);

    for (int i = -r; i <= r; ++i) {
        for (int j = -r; j <= r; ++j) {
            points.emplace_back(i, j);
        }
    }

    std::sort(points.begin(), points.end(),
    [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
        return std::hypot(a.first, a.second) < std::hypot(b.first, b.second);
    });

    std::vector<std::vector<int>> level_table(sz, std::vector<int>(sz, 0));
    int level = 0;
    std::pair<int, int> last_point = {0, 0};

    for (const auto& point : points) {
        if (point.first * point.first + point.second * point.second !=
            last_point.first * last_point.first + last_point.second * last_point.second) {
            ++ level;
        }
        level_table[point.first + r][point.second + r] = level;
        last_point = point;
    }

    level_table_ = std::move(level_table);
    return level;
}

uint8_t InflationLayer::costLookup(unsigned int mx, unsigned int my,
    unsigned int src_x, unsigned int src_y) const
{
    unsigned dx = (mx >= src_x) ? (mx - src_x) : (src_x - mx);
    unsigned dy = (my >= src_y) ? (my - src_y) : (src_y - my);

    return cached_costs_[dx * cache_length_ + dy];
}

double InflationLayer::distanceLookup(unsigned int mx, unsigned int my,
    unsigned int src_x, unsigned int src_y) const
{
    unsigned dx = (mx >= src_x) ? (mx - src_x) : (src_x - mx);
    unsigned dy = (my >= src_y) ? (my - src_y) : (src_y - my);

    return cached_distances_[dx * cache_length_ + dy];
}

void InflationLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y, double* max_x, double* max_y)
{
    // Unused
    (void) robot_x;
    (void) robot_y;
    (void) robot_yaw;

    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;

    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;

    *min_x = std::min(*min_x, tmp_min_x) - inflation_radius;
    *min_y = std::min(*min_y, tmp_min_y) - inflation_radius;
    *max_x = std::max(*max_x, tmp_max_x) + inflation_radius;
    *max_y = std::max(*max_y, tmp_max_y) + inflation_radius;
}

void InflationLayer::updateCosts(Costmap2D& master_grid,
                    unsigned int min_i, unsigned int min_j,
                    unsigned int max_i, unsigned int max_j)
{
    if (cell_inflation_radius_ == 0) {
        return;
    }

    for (auto& dist : inflation_cells_) {
        if (!dist.empty()) {
            COSTMAP_WARN("Inflation cells are not empty, which should not happen.");
            dist.clear();
        }
    }

    auto costmap = master_costmap_->getCostmap();
    auto size_x = costmap->getSizeX();
    auto size_y = costmap->getSizeY();

    if (seen_.size() != size_x * size_y) {
        COSTMAP_WARN("Seen vector size does not match costmap size, resizing.");
        seen_.resize(size_x * size_y, false);
    }

    std::fill(seen_.begin(), seen_.end(), false);

    const auto base_min_i = min_i;
    const auto base_min_j = min_j;
    const auto base_max_i = max_i;
    const auto base_max_j = max_j;

    // Cells out of bounds can also influence the costs inside the bounds
    min_i = (min_i >= cell_inflation_radius_) ? (min_i - cell_inflation_radius_) : 0;
    min_j = (min_j >= cell_inflation_radius_) ? (min_j - cell_inflation_radius_) : 0;
    max_i = (max_i + cell_inflation_radius_ < size_x) ? (max_i + cell_inflation_radius_) : size_x - 1;
    max_j = (max_j + cell_inflation_radius_ < size_y) ? (max_j + cell_inflation_radius_) : size_y - 1;

    // To find all LETHAL_OBSTACLE cells as the source of inflation
    auto& obs_bin = inflation_cells_[0];
    obs_bin.reserve(200);
    for (auto j = min_j; j <= max_j; ++j) {
        for (auto i = min_i; i <= max_i; ++i) {
            auto cost = master_grid.getCost(i, j);
            if (cost == LETHAL_OBSTACLE) {
                obs_bin.emplace_back(i, j, i, j);
            }
        }
    }

    for (auto& dist_bin : inflation_cells_) {
        dist_bin.reserve(200);
        for (size_t i = 0; i < dist_bin.size(); ++i) {
            // Do not use iterators here, as we will emplace new cells
            // into the vector while iterating
            const CellData& cell = dist_bin[i];
            unsigned int mx = cell.x;
            unsigned int my = cell.y;
            unsigned int src_x = cell.src_x;
            unsigned int src_y = cell.src_y;
            unsigned int index = master_grid.getIndex(mx, my);

            if (seen_[index]) {
                continue; // Already visited
            }

            seen_[index] = true;

            uint8_t cost = costLookup(mx, my, src_x, src_y);
            uint8_t old_cost = master_grid.getCost(mx, my);

            if (mx >= base_min_i && mx <= base_max_i &&
                my >= base_min_j && my <= base_max_j) {
                if (old_cost == NO_INFORMATION && cost >= INSCRIBED_INFLATED_OBSTACLE) {
                    master_grid.setCost(mx, my, cost);
                } else {
                    master_grid.setCost(mx, my, std::max(old_cost, cost));
                }
            }

            if (mx > 0) {
                enqueue(index - 1, mx - 1, my, src_x, src_y);
            }
            if (my > 0) {
                enqueue(index - size_x, mx, my - 1, src_x, src_y);
            }
            if (mx < size_x - 1) {
                enqueue(index + 1, mx + 1, my, src_x, src_y);
            }
            if (my < size_y - 1) {
                enqueue(index + size_x, mx, my + 1, src_x, src_y);
            }
        }
        dist_bin.clear();
    }
}

void InflationLayer::enqueue(unsigned int index, unsigned int mx,
    unsigned int my, unsigned int src_x, unsigned int src_y)
{
    if (seen_[index]) {
        return; // Already visited
    }

    auto distance = distanceLookup(mx, my, src_x, src_y);
    if (distance > cell_inflation_radius_) {
        return; // Outside the inflation radius
    }

    const auto r = cell_inflation_radius_ + 2;
    const auto level = level_table_[mx - src_x + r][my - src_y + r];
    inflation_cells_[level].emplace_back(mx, my, src_x, src_y);
}

} // namespace rf_costmap