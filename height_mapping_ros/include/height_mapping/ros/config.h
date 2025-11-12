#pragma once

#include <string>
#include <cstdlib>
#include <rclcpp/rclcpp.hpp>
#include "height_mapping/core/core.h"

namespace height_mapping_ros {

namespace height_mapper {

static inline height_mapping::HeightMapper::Config loadConfig(rclcpp::Node& node) {
  height_mapping::HeightMapper::Config cfg;
  cfg.estimator_type  = node.declare_parameter<std::string>("estimator_type", "StatMean");
  cfg.frame_id        = node.declare_parameter<std::string>("frame_id", "map");
  cfg.map_length_x    = node.declare_parameter<double>("map_length_x", 10.0);
  cfg.map_length_y    = node.declare_parameter<double>("map_length_y", 10.0);
  cfg.grid_resolution = node.declare_parameter<double>("grid_resolution", 0.1);
  cfg.min_height      = node.declare_parameter<double>("min_height_threshold", -0.2);
  cfg.max_height      = node.declare_parameter<double>("max_height_threshold", 1.5);
  return cfg;
}

// Overload conveniente si usas SharedPtr
static inline height_mapping::HeightMapper::Config
loadConfig(const rclcpp::Node::SharedPtr& node) {
  return loadConfig(*node);
}

} // namespace height_mapper

namespace global_mapper {

static inline height_mapping::GlobalMapper::Config loadConfig(rclcpp::Node& node) {
  height_mapping::GlobalMapper::Config cfg;
  cfg.estimator_type  = node.declare_parameter<std::string>("estimator_type", "StatMean");
  cfg.frame_id        = node.declare_parameter<std::string>("frame_id", "map");
  cfg.map_length_x    = node.declare_parameter<double>("map_length_x", 200.0);
  cfg.map_length_y    = node.declare_parameter<double>("map_length_y", 200.0);
  cfg.grid_resolution = node.declare_parameter<double>("grid_resolution", 0.1);
  cfg.min_height      = node.declare_parameter<double>("min_height_threshold", -0.2);
  cfg.max_height      = node.declare_parameter<double>("max_height_threshold", 1.5);

  const char* user = std::getenv("USER");
  const std::string default_dir = user ? (std::string("/home/") + user + "/Downloads") : std::string{};
  cfg.map_save_dir   = node.declare_parameter<std::string>("map_save_dir", default_dir);
  return cfg;
}

// Overload conveniente si usas SharedPtr
static inline height_mapping::GlobalMapper::Config
loadConfig(const rclcpp::Node::SharedPtr& node) {
  return loadConfig(*node);
}

} // namespace global_mapper

} // namespace height_mapping_ros
