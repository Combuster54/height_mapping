#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>

namespace frame_ids {

inline std::string ROBOT_BASE{};
inline std::string MAP{};

namespace sensor {
inline std::string LIDAR{};
inline std::string RGBD{};
} // namespace sensor

// Carga parámetros desde el nodo (ROS 2)
inline void loadFromConfig(rclcpp::Node & node) {
  auto get_or_declare = [&](const char * name, const char * default_value) -> std::string {
    if (!node.has_parameter(name)) {
      // Declara y devuelve el valor (permite overrides desde YAML/CLI)
      return node.declare_parameter<std::string>(name, default_value);
    }
    std::string v;
    if (node.get_parameter(name, v)) {
      return v;
    }
    // Fallback si estaba declarado con otro tipo o lectura fallida
    return std::string(default_value);
  };

  ROBOT_BASE    = get_or_declare("robot", "base_link");
  MAP           = get_or_declare("map", "map");
  sensor::LIDAR = get_or_declare("lidar_sensor", "");
  sensor::RGBD  = get_or_declare("rgbd_sensor", "");
}

} // namespace frame_ids
