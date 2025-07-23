#include "rf_map_manager/map_manager.hpp"
#include <cstdlib>
#include <memory>

namespace rf_map_manager
{
class MapManagerNode : public rclcpp::Node
{
public:
    MapManagerNode()
    : Node("map_manager_node")
    {
    }

    void init()
    {
        manager_ = std::make_unique<MapManager>(shared_from_this(),
        std::string(std::getenv("HOME")) + "/.rf_robot/map");
        manager_->publishMap();
    }


private:
    std::unique_ptr<MapManager> manager_;
};
} // namespace rf_map_manager


int main()
{
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<rf_map_manager::MapManagerNode>();
    node->init();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}