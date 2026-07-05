#include "rf_global_map/global_map_node.hpp"
#include "rf_local_map/local_map_node.hpp"
#include "rf_localization/localization_node.hpp"
#include "rf_map_manager/map_manager_node.hpp"
#include "rf_global_planner/global_planner_node.hpp"
#include "rf_scheduler_node/scheduler_node.hpp"
#include "rf_map_builder/map_builder_node.hpp"
#include "elog/elog.h"
#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <thread>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto global_map_node = std::make_shared<rf_global_map::GlobalMapNode>();
    auto local_map_node = std::make_shared<rf_local_map::LocalMapNode>();
    auto localization_node = std::make_shared<rf_localization::LocalizationNode>();
    auto map_manager_node = std::make_shared<rf_map_manager::MapManagerNode>();
    auto global_planner_node = std::make_shared<rf_global_planner::GlobalPlannerNode>();
    auto sched_node = std::make_shared<rf_scheduler::SchedulerNode>();
    auto map_builder_node = std::make_shared<rf_map_builder::MapBuilderNode>();

    global_map_node->init();
    local_map_node->init();
    localization_node->init();
    map_manager_node->init();
    global_planner_node->init();
    sched_node->init();
    map_builder_node->init();

    auto map_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    map_executor->add_node(global_map_node);
    map_executor->add_node(local_map_node);
    map_executor->add_node(localization_node);
    map_executor->add_node(map_manager_node);

    std::thread spin_thread1([&]() {
        pthread_setname_np(pthread_self(), "spin_thread1");
        map_executor->spin();
    });

    auto scheduler_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    scheduler_executor->add_node(sched_node);
    scheduler_executor->add_node(global_planner_node);

    std::thread spin_thread2([&]() {
        pthread_setname_np(pthread_self(), "spin_thread2");
        scheduler_executor->spin();
    });

    auto map_builder_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    map_builder_executor->add_node(map_builder_node);

    std::thread spin_thread3([&]() {
        pthread_setname_np(pthread_self(), "map_thread");
        map_builder_executor->spin();
    });

    spin_thread1.join();
    spin_thread2.join();
    spin_thread3.join();

    rclcpp::shutdown();
    elog::info("[rf_main] RF Robot shut down.");

    return 0;
}
