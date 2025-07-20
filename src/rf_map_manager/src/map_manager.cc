#include "rf_map_manager/map_manager.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "opencv2/opencv.hpp"
#include "yaml-cpp/yaml.h"
#include "elog/elog.h"

#include <cstdint>
#include <fstream>
#include <memory>
#include <opencv2/imgcodecs.hpp>
#include <string>
#include <vector>
#include <yaml-cpp/exceptions.h>

namespace rf_map_manager
{

const std::string occ_file_name = "occ_map";

MapManager::MapManager(rclcpp::Node::SharedPtr node,
    const std::string& map_dir)
    : node_(node),
      map_dir_(map_dir)
{
    map_publisher_ = node_->create_publisher<OccupancyGridMsgT>("map", rclcpp::QoS(1));
    get_map_service_ = node_->create_service<GetMapSrvT>(
        "get_map", std::bind(&MapManager::handleGetMapService, this, std::placeholders::_1, std::placeholders::_2));
    dump_map_service_ = node_->create_service<DumpMapSrvT>(
        "dump_map", std::bind(&MapManager::handleDumpMapService, this, std::placeholders::_1, std::placeholders::_2));
}

void MapManager::reloadMap()
{
    std::string meta_data_file = map_dir_ + "/" + occ_file_name + ".yaml";
    YAML::Node doc = YAML::LoadFile(map_dir_ + "/" + meta_data_file);

    double resolution = 0.0;
    std::string image_file_name;
    std::vector<double> origin;
    double free_thresh = 0.0;
    double occupied_thresh = 0.0;
    std::string mode;
    int negate = 0;

    try {
        image_file_name = doc["image"].as<std::string>();
        image_file_name = map_dir_ + "/" + image_file_name;
        resolution = doc["resolution"].as<double>();
        origin = doc["origin"].as<std::vector<double>>();
        if (origin.size() != 3) {
            throw YAML::Exception(
                doc["origin"].Mark(), "value of the 'origin' tag should have 3 elements, not " +
                std::to_string(origin.size()));
        }
        free_thresh = doc["free_thresh"].as<double>();
        occupied_thresh = doc["occupied_thresh"].as<double>();
        mode = doc["mode"].as<std::string>();
        negate = doc["negate"].as<int>();

        elog::info("Reload map yaml: {}, resolotion: {}, "
            "origin[0]: {}, origin[1]: {}, origin[2]: {}, fresh_thresh: {}, occupied_thresh: {}",
            meta_data_file, resolution, origin[0], origin[1], origin[2], free_thresh, occupied_thresh);
    } catch (YAML::Exception& e) {
        elog::error("Failed to parse YAML: {}", meta_data_file);
        return;
    }

    // Read map data
    cv::Mat img = cv::imread(image_file_name, cv::IMREAD_UNCHANGED);
    if (img.empty()) {
        elog::error("Failed to load map image: {}", image_file_name);
        return;
    }
    OccupancyGridMsgT grid;
    grid.info.width = img.cols;
    grid.info.height = img.rows;
    grid.info.resolution = resolution;
    grid.info.origin.position.x = origin[0];
    grid.info.origin.position.y = origin[1];
    grid.info.origin.position.z = 0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(grid.info.width * grid.info.height);
    for (int y = 0; y < img.rows; ++y) {
        for (int x = 0; x < img.cols; ++x) {
            int index = x + (img.rows - y - 1) * img.cols;  // flip Y
            uchar pixel = img.at<uchar>(y, x);
            double shade = negate ? pixel / 255.0 : 1 - pixel / 255.0;
            if (shade > occupied_thresh) {
                grid.data[index] = 100;
            } else if (shade < free_thresh) {
                grid.data[index] = 0;
            } else {
                grid.data[index] = -1;
            }
        }
    }

  cached_map_ = std::make_shared<OccupancyGridMsgT>(grid);
}

bool MapManager::dumpMap(const OccupancyGridMsgT& map)
{
    // Dump map as trinary mode
    static constexpr double occupied_thresh = 0.65;
    static constexpr double free_thresh = 0.196;

    // To dump map file
    std::string map_data_file = map_dir_ + "/" + occ_file_name + ".pgm";
    int free_thresh_int = std::rint(free_thresh * 100.0);
    int occupied_thresh_int = std::rint(occupied_thresh * 100.0);

    cv::Mat image(map.info.height, map.info.width, CV_8UC1);
    for (unsigned int y = 0; y < map.info.height; ++ y) {
        for (unsigned int x = 0; x < map.info.width; ++ x) {
            uint8_t cell = map.data[map.info.width * (map.info.height - y - 1) + x];
            uchar color = 0;
            if (cell == 0 || cell > 100) {
                color = 205;  // grey
            } else if (cell >= occupied_thresh_int) {
                color = 0;    // black
            } else if (cell <= free_thresh_int) {
                color = 254;  // white
            } else {
                color = 205;
            }
            image.at<uchar>(y, x) = color;
        }
    }

    cv::imwrite(map_data_file, image);
    elog::info("Write map_data into {}", map_data_file);

    // To dump param yaml
    std::string meta_data_file = map_dir_ + "/" + occ_file_name + ".yaml";
    std::ofstream yaml(meta_data_file);
    if (!yaml.is_open()) {
        elog::error("Cant not open file: {}", meta_data_file);
        return false;
    }
    geometry_msgs::msg::Quaternion orientation = map.info.origin.orientation;
    tf2::Matrix3x3 mat(tf2::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
    double yaw, pitch, roll;
    mat.getEulerYPR(yaw, pitch, roll);

    YAML::Emitter e;
    e << YAML::Precision(3);
    e << YAML::BeginMap;
    e << YAML::Key << "image" << YAML::Value << (occ_file_name + ".pgm");
    e << YAML::Key << "mode" << YAML::Value << "trinary";
    e << YAML::Key << "resolution" << YAML::Value << map.info.resolution;
    e << YAML::Key << "origin" << YAML::Flow << YAML::BeginSeq << map.info.origin.position.x
      << map.info.origin.position.y << yaw << YAML::EndSeq;
    e << YAML::Key << "negate" << YAML::Value << 0;
    e << YAML::Key << "occupied_thresh" << YAML::Value << occupied_thresh;
    e << YAML::Key << "free_thresh" << YAML::Value << free_thresh;

    if (!e.good()) {
        elog::error("Yaml writer failed with an error: {}", e.GetLastError());
        return false;
    }

    yaml << e.c_str();
    yaml.flush();
    yaml.close();
    elog::info("map meta data saved: {}", meta_data_file);
    return true;
}

void MapManager::publishMap()
{
    elog::info("publish occ map.");
    if (!cached_map_) {
        reloadMap();
    }
    if (!cached_map_) {
        elog::warn("No occ map yet.");
        return;
    }
    map_publisher_->publish(*cached_map_);
}

void MapManager::handleGetMapService(const std::shared_ptr<GetMapSrvT::Request>,
                            std::shared_ptr<GetMapSrvT::Response> response)
{
    elog::info("handleGetMap request.");
    if (!cached_map_) {
        reloadMap();
    }
    if (!cached_map_) {
        response->success = false;
        return;
    }
    response->success = true;
    response->map = *cached_map_;
}

void MapManager::handleDumpMapService(const std::shared_ptr<DumpMapSrvT::Request> request,
                            std::shared_ptr<DumpMapSrvT::Response> response)
{
    bool success = dumpMap(request->map);
    if (!success) {
        response->result = false;
        return;
    }
    response->result = true;
}

} // namespace rf_map_manager