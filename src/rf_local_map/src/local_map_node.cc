#include "rf_local_map/local_map_node.hpp"
#include "elog/elog.h"

namespace rf_local_map
{

void LocalMapNode::init()
{
    rf_costmap::CostmapConfig config;
    config.map_name = "/local_costmap";
    config.resolution = 0.025; // 2.5 cm resolution
    config.width = 5; // 5 meters wide
    config.height = 5; // 5 meters tall
    config.rolling_window = true;
    config.update_rate = 10; // 10 Hz
    config.publish_rate = 5; // 5 Hz
    config.layer_names = {"obstacle_layer", "inflation_layer"};

    costmap_interface_ = std::make_unique<rf_costmap::CostmapInterface>(shared_from_this(), config);
    costmap_interface_->init();

    local_map_srv_ = this->create_service<ReqAckSrvT>(
        "/local_map_control",
        [this](const ReqAckSrvT::Request::SharedPtr request,
               const ReqAckSrvT::Response::SharedPtr response) {
            elog::info("[LocalMapNode] Received local map control request: {}",
                request->trigger ? "Start" : "Stop");
            if (request->trigger) {
                costmap_interface_->start();
            } else {
                costmap_interface_->stop();
            }
            response->ack = true; // Acknowledge the request
        });
}

} // namespace rf_local_map