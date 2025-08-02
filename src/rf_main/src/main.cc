#include "rf_global_map/global_map_node.hpp"
#include "rf_local_map/local_map_node.hpp"
#include "elog/elog.h"
#include "rclcpp/rclcpp.hpp"

#include <thread>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto global_map_node = std::make_shared<rf_global_map::GlobalMapNode>();
    auto loacal_map_node = std::make_shared<rf_local_map::LocalMapNode>();

    global_map_node->init();
    loacal_map_node->init();

    auto map_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    map_executor->add_node(global_map_node);
    map_executor->add_node(loacal_map_node);

    std::thread spin_thread1([&]() {
        map_executor->spin();
    });

    spin_thread1.join();

    rclcpp::shutdown();
    elog::info("[rf_main] RF Robot shut down.");

    return 0;
}