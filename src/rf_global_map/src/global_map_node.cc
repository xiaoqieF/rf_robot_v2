#include "rf_global_map/global_map_node.hpp"
#include <elog/elog.h>

namespace rf_global_map
{

void GlobalMapNode::init()
{
    rf_costmap::CostmapConfig config;
    config.map_name = "/global_costmap";
    config.resolution = 0.05; // 5 cm resolution
    config.width = 0; // invalid, decide by the static layer
    config.height = 0; // invalid
    config.rolling_window = false;
    config.update_rate = 1; // 1 Hz
    config.publish_rate = 1; // 1 Hz
    config.layer_names = {"static_layer", "obstacle_layer","inflation_layer"};

    costmap_interface_ = std::make_unique<rf_costmap::CostmapInterface>(shared_from_this(), config);
    costmap_interface_->init();

    global_map_srv_ = this->create_service<ReqAckSrvT>(
        "/global_map_control",
        [this](const ReqAckSrvT::Request::SharedPtr request,
               const ReqAckSrvT::Response::SharedPtr response) {
            elog::info("[GlobalMapNode] Received global map control request: {}",
                request->trigger == request->START ? "Start" : "Stop");
            if (request->trigger == request->START) {
                startMapUpdate();
            } else {
                stopMapUpdate();
            }
            response->ack = response->OK; // Acknowledge the request
        });
}

void GlobalMapNode::startMapUpdate()
{
    costmap_interface_->start();
}

void GlobalMapNode::stopMapUpdate()
{
    costmap_interface_->stop();
}

} // namespace rf_global_map