#include "rf_global_map/global_map_node.hpp"
#include "rf_local_map/local_map_node.hpp"
#include "rf_map_manager/map_manager_node.hpp"
#include "rf_scheduler_node/scheduler_node.hpp"
#include "elog/elog.h"
#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <thread>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto global_map_node = std::make_shared<rf_global_map::GlobalMapNode>();
    auto local_map_node = std::make_shared<rf_local_map::LocalMapNode>();
    auto map_manager_node = std::make_shared<rf_map_manager::MapManagerNode>();
    auto sched_node = std::make_shared<rf_scheduler::SchedulerNode>();

    global_map_node->init();
    local_map_node->init();
    map_manager_node->init();
    sched_node->init();

    auto map_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    map_executor->add_node(global_map_node);
    map_executor->add_node(local_map_node);
    map_executor->add_node(map_manager_node);

    std::thread spin_thread1([&]() {
        pthread_setname_np(pthread_self(), "spin_thread1");
        map_executor->spin();
    });

    auto scheduler_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    scheduler_executor->add_node(sched_node);

    std::thread spin_thread2([&]() {
        pthread_setname_np(pthread_self(), "spin_thread2");
        scheduler_executor->spin();
    });

    spin_thread1.join();
    spin_thread2.join();

    rclcpp::shutdown();
    elog::info("[rf_main] RF Robot shut down.");

    return 0;
}