#pragma once

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "common/ros/common.h"
#include "common/ros/TransformOps.h"

#include "height_mapping/core/core.h"
#include "height_mapping/types/elevation_point.h"
#include "height_mapping/ros/config.h"
#include "height_mapping/srv/save_map.hpp"

namespace height_mapping_ros {

class GlobalMappingNode : public rclcpp::Node {
public:
  struct Config {
    std::string lidarcloud_topic;
    std::string rgbdcloud_topic;
    double      map_publish_rate;
    bool        remove_backward_points;
    bool        debug_mode;
    std::string map_save_dir;     // (opcional)
    std::string map_save_format;  // "pcd" | "bag"
  } cfg;

  GlobalMappingNode();
  ~GlobalMappingNode() override = default;

private:
  // init
  void initializePubSubs();
  void initializeServices();

  // callbacks
  void lidarScanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void rgbdScanCallback (const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void publishPointCloudMap();

  // Core
  pcl::PointCloud<Laser>::Ptr
  processLidarScan(const pcl::PointCloud<Laser>::Ptr &cloud,
                   const geometry_msgs::msg::TransformStamped &lidar2base,
                   const geometry_msgs::msg::TransformStamped &base2map);

  pcl::PointCloud<Color>::Ptr
  processRGBDScan(const pcl::PointCloud<Color>::Ptr &cloud,
                  const geometry_msgs::msg::TransformStamped &camera2base,
                  const geometry_msgs::msg::TransformStamped &base2map);

  // Services
  void saveMap(const std::shared_ptr<height_mapping::srv::SaveMap::Request> req,
               std::shared_ptr<height_mapping::srv::SaveMap::Response> res);
  void clearMap(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                std::shared_ptr<std_srvs::srv::Empty::Response>);

  // Helpers
  using HeightMap = height_mapping::HeightMap;

  bool toPclCloud(const HeightMap &map,
                  const std::unordered_set<grid_map::Index> &grid_indices,
                  pcl::PointCloud<height_mapping_types::ElevationPoint> &cloud);

  bool savePointCloud(const pcl::PointCloud<height_mapping_types::ElevationPoint> &cloud,
                      const std::string &filename);

  bool saveMapToBag(const HeightMap &map, const std::string &filename);

  void toPointCloud2(const HeightMap &map,
                     const std::vector<std::string> &layers,
                     const std::unordered_set<grid_map::Index> &grid_indices,
                     sensor_msgs::msg::PointCloud2 &cloud);

  void toMapRegion(const HeightMap &map, visualization_msgs::msg::Marker &region);

  // Subs/Pubs
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidarscan_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_rgbdscan_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_map_cloud_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr  pub_map_region_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_scan_rasterized_;

  // Services
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr         srv_clear_map_;
  rclcpp::Service<height_mapping::srv::SaveMap>::SharedPtr srv_save_map_;

  // Timer (se crea al recibir el primer scan)
  rclcpp::TimerBase::SharedPtr map_publish_timer_;

  // Core mapping
  std::unique_ptr<height_mapping::GlobalMapper> mapper_;
  TransformOps tf_;

  // State
  bool lidarscan_received_{false};
  bool rgbdscan_received_{false};
};

} // namespace height_mapping_ros
