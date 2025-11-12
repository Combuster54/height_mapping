#pragma once

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>  // ✅
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "common/ros/common.h"              // versión ROS2
#include "height_mapping/core/core.h"
#include "height_mapping/ros/config.h"      // loaders ROS2 (frame_ids, height_mapper)
#include "common/ros/TransformOps.h"

namespace height_mapping_ros {

class HeightMappingNode : public rclcpp::Node {
public:
  struct Config {
    std::string lidarcloud_topic;
    std::string rgbdcloud_topic;
    double      pose_update_rate;
    double      map_publish_rate;
    bool        remove_backward_points;
    bool        debug_mode;
  } cfg;

  HeightMappingNode();
  ~HeightMappingNode() override = default;

private:
  // init
  void initializePubSubs();
  void initializeTimers();   // crea timers al recibir primer scan
  void initializeServices(); // vacío (placeholder)

  // callbacks
  void lidarScanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void rgbdScanCallback (const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // core
  pcl::PointCloud<Laser>::Ptr
  processLidarScan(const pcl::PointCloud<Laser>::Ptr &cloud,
                   const geometry_msgs::msg::TransformStamped &lidar2base,
                   const geometry_msgs::msg::TransformStamped &base2map);

  pcl::PointCloud<Color>::Ptr
  processRGBDScan(const pcl::PointCloud<Color>::Ptr &cloud,
                  const geometry_msgs::msg::TransformStamped &camera2base,
                  const geometry_msgs::msg::TransformStamped &base2map);

  void publishRasterizedLidarScan(const pcl::PointCloud<Laser>::Ptr &scan);
  void publishRasterizedRGBDScan (const pcl::PointCloud<Color>::Ptr &scan);

  // timers
  void updateMapOrigin();  // pose_update_timer_
  void publishHeightMap(); // map_publish_timer_

  // pubs/subs
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidarscan_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_rgbdscan_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_lidarscan_rasterized_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_rgbdscan_rasterized_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr      pub_heightmap_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_debug_lidar_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_debug_rgbd_;

  // timers (se crean tras primer scan)
  rclcpp::TimerBase::SharedPtr pose_update_timer_;
  rclcpp::TimerBase::SharedPtr map_publish_timer_;

  // core objects
  std::unique_ptr<height_mapping::HeightMapper> mapper_;
  TransformOps tf_; // usa nodo interno; suficiente para tf2 en este util

  // estado
  bool lidarscan_received_{false};
  bool rgbdscan_received_{false};
};

} // namespace height_mapping_ros
