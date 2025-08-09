#include "rf_map_manager/map_manager_node.hpp"

namespace rf_map_manager
{

void MapManagerNode::init()
{
    MAP_MANAGER_INFO("Initializing MapManagerNode...");

    manager_ = std::make_unique<MapManager>(shared_from_this(),
    std::string(std::getenv("HOME")) + "/.rf_robot/map");
    pub_map_service_ = this->create_service<ReqAckSrvT>(
        "/publish_static_map",
        [this](const ReqAckSrvT::Request::SharedPtr,
                const ReqAckSrvT::Response::SharedPtr response) {
            MAP_MANAGER_INFO("Received request to publish static map");
            manager_->publishMap();
            response->ack = response->OK; // Acknowledge the request
        });
}

} // namespace rf_map_manager
