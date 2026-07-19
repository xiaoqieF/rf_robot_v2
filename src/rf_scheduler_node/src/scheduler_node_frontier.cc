#include "scheduler_node_internal.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <queue>
#include <tuple>
#include <utility>
#include <vector>

namespace rf_scheduler
{

std::vector<SchedulerNode::FrontierCandidate> SchedulerNode::computeFrontierCandidates(
    const OccupancyGridMsgT& map,
    const PoseStampedT* robot_pose,
    std::size_t* frontier_cluster_count_out,
    std::size_t* suppressed_cluster_count_out,
    std::size_t* rejected_cluster_count_out) const
{
    std::vector<FrontierCandidate> candidates;
    std::size_t frontier_cluster_count = 0;
    std::size_t suppressed_cluster_count = 0;
    std::size_t clusters_without_goal_count = 0;

    const auto width = static_cast<int>(map.info.width);
    const auto height = static_cast<int>(map.info.height);
    if (width <= 0 || height <= 0 || map.data.size() != static_cast<std::size_t>(width * height)) {
        return candidates;
    }

    const uint32_t min_cluster_size = detail::kDefaultMinFrontierClusterSize;
    const int search_radius_cells = std::max(
        1,
        static_cast<int>(std::ceil(detail::kGoalSearchRadiusMeters / map.info.resolution)));
    const int min_standoff_cells = std::max(
        1,
        static_cast<int>(std::ceil(detail::kGoalMinStandoffMeters / map.info.resolution)));
    const int clearance_radius_cells = std::max(
        1,
        static_cast<int>(std::ceil(detail::kGoalClearanceRadiusMeters / map.info.resolution)));
    const int known_radius_cells = std::max(
        1,
        static_cast<int>(std::round(detail::kGoalKnownRadiusMeters / map.info.resolution)));

    const auto index_of = [width](const int x, const int y) {
        return y * width + x;
    };

    const auto world_x_of = [&map](const int x) {
        return map.info.origin.position.x + (static_cast<double>(x) + 0.5) * map.info.resolution;
    };

    const auto world_y_of = [&map](const int y) {
        return map.info.origin.position.y + (static_cast<double>(y) + 0.5) * map.info.resolution;
    };

    const auto has_unknown_neighbor = [&](const int x, const int y) {
        static constexpr int kNeighborDx[4] = {1, -1, 0, 0};
        static constexpr int kNeighborDy[4] = {0, 0, 1, -1};
        for (int i = 0; i < 4; ++i) {
            const int nx = x + kNeighborDx[i];
            const int ny = y + kNeighborDy[i];
            if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
                continue;
            }
            if (detail::isUnknownCell(map.data[static_cast<std::size_t>(index_of(nx, ny))])) {
                return true;
            }
        }
        return false;
    };

    const auto is_frontier_cell = [&](const int x, const int y) {
        return detail::isFreeCell(map.data[static_cast<std::size_t>(index_of(x, y))]) && has_unknown_neighbor(x, y);
    };

    const auto has_goal_clearance = [&](const int x, const int y, const bool strict) {
        for (int dy = -clearance_radius_cells; dy <= clearance_radius_cells; ++dy) {
            for (int dx = -clearance_radius_cells; dx <= clearance_radius_cells; ++dx) {
                const int nx = x + dx;
                const int ny = y + dy;
                if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
                    return false;
                }

                if ((dx * dx + dy * dy) > clearance_radius_cells * clearance_radius_cells) {
                    continue;
                }

                const auto value = map.data[static_cast<std::size_t>(index_of(nx, ny))];
                const bool in_known_core = (dx * dx + dy * dy) <= known_radius_cells * known_radius_cells;

                if (strict) {
                    // Prefer goals with a ring of known-free space around them.
                    if (detail::isUnknownCell(value)) {
                        if (in_known_core) {
                            return false;
                        }
                        continue;
                    }

                    if (!detail::isFreeCell(value)) {
                        return false;
                    }
                    continue;
                }

                // Relaxed fallback: allow inflation-cost cells and unknown on the
                // outer rim, but still require a known-free core and reject hard
                // obstacles anywhere in the clearance disk.
                if (detail::isUnknownCell(value)) {
                    if (in_known_core) {
                        return false;
                    }
                    continue;
                }

                if (value > detail::kGoalOccupiedThreshold) {
                    return false;
                }
            }
        }
        return true;
    };

    const auto estimate_clearance_cells = [&](const int x, const int y, const bool strict) {
        int clearance = 0;
        while (clearance < search_radius_cells) {
            const int radius = clearance + 1;
            bool blocked = false;
            for (int dy = -radius; dy <= radius && !blocked; ++dy) {
                for (int dx = -radius; dx <= radius; ++dx) {
                    if (std::max(std::abs(dx), std::abs(dy)) != radius) {
                        continue;
                    }
                    const int nx = x + dx;
                    const int ny = y + dy;
                    if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
                        blocked = true;
                        break;
                    }

                    const auto value = map.data[static_cast<std::size_t>(index_of(nx, ny))];
                    if (strict
                        ? !detail::isFreeCell(value)
                        : detail::isUnknownCell(value) || value > detail::kGoalOccupiedThreshold) {
                        blocked = true;
                        break;
                    }
                }
            }

            if (blocked) {
                break;
            }
            ++clearance;
        }
        return clearance;
    };

    std::vector<bool> visited(static_cast<std::size_t>(width * height), false);
    static constexpr int kClusterDx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    static constexpr int kClusterDy[8] = {0, 1, 1, 1, 0, -1, -1, -1};

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const std::size_t seed_index = static_cast<std::size_t>(index_of(x, y));
            if (visited[seed_index] || !is_frontier_cell(x, y)) {
                continue;
            }

            std::vector<std::pair<int, int>> cluster_cells;
            cluster_cells.reserve(32);
            std::queue<std::pair<int, int>> bfs_queue;
            bfs_queue.emplace(x, y);
            visited[seed_index] = true;

            double sum_x = 0.0;
            double sum_y = 0.0;

            while (!bfs_queue.empty()) {
                const auto [cx, cy] = bfs_queue.front();
                bfs_queue.pop();

                cluster_cells.emplace_back(cx, cy);
                sum_x += static_cast<double>(cx);
                sum_y += static_cast<double>(cy);

                for (int i = 0; i < 8; ++i) {
                    const int nx = cx + kClusterDx[i];
                    const int ny = cy + kClusterDy[i];
                    if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
                        continue;
                    }

                    const std::size_t neighbor_index = static_cast<std::size_t>(index_of(nx, ny));
                    if (visited[neighbor_index] || !is_frontier_cell(nx, ny)) {
                        continue;
                    }

                    visited[neighbor_index] = true;
                    bfs_queue.emplace(nx, ny);
                }
            }

            if (cluster_cells.size() < min_cluster_size) {
                continue;
            }
            ++frontier_cluster_count;

            const double centroid_x = sum_x / static_cast<double>(cluster_cells.size());
            const double centroid_y = sum_y / static_cast<double>(cluster_cells.size());
            const double frontier_centroid_wx = world_x_of(static_cast<int>(std::round(centroid_x)));
            const double frontier_centroid_wy = world_y_of(static_cast<int>(std::round(centroid_y)));

            if (isFrontierSuppressed(frontier_centroid_wx, frontier_centroid_wy, true)) {
                ++suppressed_cluster_count;
                continue;
            }

            std::vector<bool> cluster_frontier_mask(static_cast<std::size_t>(width * height), false);
            for (const auto& cell : cluster_cells) {
                cluster_frontier_mask[static_cast<std::size_t>(index_of(cell.first, cell.second))] = true;
            }

            std::vector<bool> search_visited(static_cast<std::size_t>(width * height), false);
            std::queue<std::tuple<int, int, int>> search_queue;
            for (const auto& cell : cluster_cells) {
                for (int i = 0; i < 8; ++i) {
                    const int nx = cell.first + kClusterDx[i];
                    const int ny = cell.second + kClusterDy[i];
                    if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
                        continue;
                    }
                    const std::size_t neighbor_index = static_cast<std::size_t>(index_of(nx, ny));
                    if (cluster_frontier_mask[neighbor_index] || search_visited[neighbor_index]) {
                        continue;
                    }
                    if (!detail::isFreeCell(map.data[neighbor_index])) {
                        continue;
                    }

                    search_visited[neighbor_index] = true;
                    search_queue.emplace(nx, ny, 1);
                }
            }

            std::optional<std::pair<int, int>> best_goal_cell;
            double best_score = -std::numeric_limits<double>::infinity();
            double best_robot_distance = 0.0;

            auto evaluate_goal_cells = [&](const bool strict) {
                auto search_queue_copy = search_queue;
                auto search_visited_copy = search_visited;

                while (!search_queue_copy.empty()) {
                    const auto [cx, cy, depth] = search_queue_copy.front();
                    search_queue_copy.pop();

                    const double wx = world_x_of(cx);
                    const double wy = world_y_of(cy);

                    double robot_distance = 0.0;
                    if (robot_pose != nullptr) {
                        robot_distance = std::hypot(
                            wx - robot_pose->pose.position.x,
                            wy - robot_pose->pose.position.y);
                    }

                    if (depth >= min_standoff_cells &&
                        !isFrontierSuppressed(wx, wy) &&
                        has_goal_clearance(cx, cy, strict)) {
                        const int clearance_cells = estimate_clearance_cells(cx, cy, strict);
                        const double frontier_gain_m =
                            static_cast<double>(cluster_cells.size()) * map.info.resolution;
                        const double clearance_m =
                            static_cast<double>(clearance_cells) * map.info.resolution;
                        const double goal_to_frontier_distance_m =
                            static_cast<double>(depth) * map.info.resolution;
                        const double centroid_distance_m = std::hypot(
                            static_cast<double>(cx) - centroid_x,
                            static_cast<double>(cy) - centroid_y) * map.info.resolution;
                        const double standoff_penalty_m =
                            std::abs(goal_to_frontier_distance_m - detail::kGoalPreferredStandoffMeters);

                        double score =
                            frontier_gain_m * 3.0 +
                            clearance_m * (strict ? 2.0 : 1.0) +
                            goal_to_frontier_distance_m * 0.8 -
                            standoff_penalty_m * (strict ? 1.5 : 0.9) -
                            centroid_distance_m * 0.6;

                        if (robot_pose != nullptr) {
                            score -= robot_distance * 1.0;
                            if (robot_distance > std::numeric_limits<double>::epsilon()) {
                                const double candidate_yaw = std::atan2(
                                    wy - robot_pose->pose.position.y,
                                    wx - robot_pose->pose.position.x);
                                const double robot_yaw = detail::yawFromQuaternion(
                                    robot_pose->pose.orientation);
                                score += detail::kFrontierHeadingAlignmentWeight *
                                    std::cos(candidate_yaw - robot_yaw);
                            }
                        }

                        if (score > best_score) {
                            best_score = score;
                            best_goal_cell = std::make_pair(cx, cy);
                            best_robot_distance = robot_distance;
                        }
                    }

                    if (depth >= search_radius_cells) {
                        continue;
                    }

                    for (int i = 0; i < 8; ++i) {
                        const int nx = cx + kClusterDx[i];
                        const int ny = cy + kClusterDy[i];
                        if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
                            continue;
                        }

                        const std::size_t neighbor_index = static_cast<std::size_t>(index_of(nx, ny));
                        if (search_visited_copy[neighbor_index] || cluster_frontier_mask[neighbor_index]) {
                            continue;
                        }
                        if (!detail::isFreeCell(map.data[neighbor_index])) {
                            continue;
                        }

                        search_visited_copy[neighbor_index] = true;
                        search_queue_copy.emplace(nx, ny, depth + 1);
                    }
                }
            };

            evaluate_goal_cells(true);
            if (!best_goal_cell.has_value()) {
                evaluate_goal_cells(false);
            }

            if (!best_goal_cell.has_value()) {
                ++clusters_without_goal_count;
                continue;
            }

            FrontierCandidate candidate;
            candidate.cell_count = cluster_cells.size();
            candidate.distance = best_robot_distance;
            candidate.goal_pose.header.frame_id = map.header.frame_id.empty() ? "map" : map.header.frame_id;
            candidate.goal_pose.header.stamp = map.header.stamp;
            candidate.goal_pose.pose.position.x = world_x_of(best_goal_cell->first);
            candidate.goal_pose.pose.position.y = world_y_of(best_goal_cell->second);
            candidate.goal_pose.pose.position.z = 0.0;
            candidate.frontier_x = frontier_centroid_wx;
            candidate.frontier_y = frontier_centroid_wy;

            const double yaw = std::atan2(
                frontier_centroid_wy - candidate.goal_pose.pose.position.y,
                frontier_centroid_wx - candidate.goal_pose.pose.position.x);
            candidate.goal_pose.pose.orientation = detail::quaternionFromYaw(yaw);
            candidate.score = best_score;
            if (robot_pose == nullptr) {
                candidate.distance = 0.0;
            }

            candidates.push_back(candidate);
        }
    }

    std::sort(candidates.begin(), candidates.end(), [](const FrontierCandidate& lhs, const FrontierCandidate& rhs) {
        if (lhs.score == rhs.score) {
            return lhs.distance < rhs.distance;
        }
        return lhs.score > rhs.score;
    });

    if (candidates.empty() && frontier_cluster_count > 0) {
        SCHED_WARN(
            "Detected {} frontier clusters, but none produced a navigable goal (suppressed clusters: {}, rejected clusters: {}).",
            frontier_cluster_count,
            suppressed_cluster_count,
            clusters_without_goal_count);
    }

    if (frontier_cluster_count_out != nullptr) {
        *frontier_cluster_count_out = frontier_cluster_count;
    }
    if (suppressed_cluster_count_out != nullptr) {
        *suppressed_cluster_count_out = suppressed_cluster_count;
    }
    if (rejected_cluster_count_out != nullptr) {
        *rejected_cluster_count_out = clusters_without_goal_count;
    }

    return candidates;
}

void SchedulerNode::suppressFrontier(const FrontierCandidate& frontier_candidate, const double radius)
{
    std::lock_guard<std::mutex> lock(suppressed_frontiers_mutex_);
    suppressed_frontiers_.push_back(
        SuppressedFrontier{
            frontier_candidate.frontier_x,
            frontier_candidate.frontier_y,
            radius,
            true});
}

void SchedulerNode::suppressGoalPose(const PoseStampedT& goal_pose, const double radius)
{
    std::lock_guard<std::mutex> lock(suppressed_frontiers_mutex_);
    suppressed_frontiers_.push_back(
        SuppressedFrontier{
            goal_pose.pose.position.x,
            goal_pose.pose.position.y,
            radius,
            false});
}

bool SchedulerNode::isFrontierSuppressed(const double x, const double y, const bool cluster_only) const
{
    std::lock_guard<std::mutex> lock(suppressed_frontiers_mutex_);
    return std::any_of(
        suppressed_frontiers_.begin(),
        suppressed_frontiers_.end(),
        [x, y, cluster_only](const SuppressedFrontier& suppressed_frontier) {
            if (cluster_only && !suppressed_frontier.cluster_level) {
                return false;
            }
            const double radius_squared = suppressed_frontier.radius * suppressed_frontier.radius;
            return detail::squaredDistance(
                suppressed_frontier.x,
                suppressed_frontier.y,
                x,
                y) <= radius_squared;
        });
}

} // namespace rf_scheduler
