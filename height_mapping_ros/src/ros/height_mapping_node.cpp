#include "height_mapping/ros/height_mapping_node.hpp"

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <pcl_conversions/pcl_conversions.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace height_mapping_ros {

HeightMappingNode::HeightMappingNode()
: rclcpp::Node("height_mapping_node")
{
  // Parámetros (equiv. a cfg_node en ROS1)
  cfg.lidarcloud_topic       = this->declare_parameter<std::string>("node.lidar_topic", "/velodyne_points");
  cfg.rgbdcloud_topic        = this->declare_parameter<std::string>("node.rgbd_topic",   "/camera/pointcloud/points");
  cfg.pose_update_rate       = this->declare_parameter<double>("node.pose_update_rate", 15.0);
  cfg.map_publish_rate       = this->declare_parameter<double>("node.map_publish_rate", 10.0);
  cfg.remove_backward_points = this->declare_parameter<bool>("node.remove_backward_points", false);
  cfg.debug_mode             = this->declare_parameter<bool>("node.debug_mode", false);

  // Frames (lee de parámetros "frame_id.*")
  frame_ids::loadFromConfig(*this);

  // HeightMapper (usa loader ROS2)
  auto hm_cfg = height_mapper::loadConfig(*this);
  mapper_ = std::make_unique<height_mapping::HeightMapper>(hm_cfg);

  // Pub/Sub y (timers/servicios se crean más tarde)
  initializePubSubs();
  initializeServices(); // vacío

  RCLCPP_INFO(this->get_logger(),
              "\033[1;33m[height_mapping_ros::HeightMappingNode]: Height mapping node initialized. Waiting for scan inputs...\033[0m");
}

void HeightMappingNode::initializePubSubs() {
  auto sensor_qos = rclcpp::SensorDataQoS();

  sub_lidarscan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cfg.lidarcloud_topic, sensor_qos,
      std::bind(&HeightMappingNode::lidarScanCallback, this, _1));

  sub_rgbdscan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cfg.rgbdcloud_topic, sensor_qos,
      std::bind(&HeightMappingNode::rgbdScanCallback, this, _1));

  pub_lidarscan_rasterized_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/height_mapping/local/scan_rasterized_lidar", 1);

  pub_rgbdscan_rasterized_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/height_mapping/local/scan_rasterized_rgbd", 1);

  pub_heightmap_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
      "/height_mapping/local/map", 1);

  if (cfg.debug_mode) {
    pub_debug_lidar_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/height_mapping/local/debug_lidar", 1);
    pub_debug_rgbd_  = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/height_mapping/local/debug_rgbd", 1);
  }
}

void HeightMappingNode::initializeTimers() {
  // En ROS2 los timers arrancan al crearse: los creamos al primer scan
  const auto pose_period = std::chrono::duration<double>(1.0 / std::max(1e-6, cfg.pose_update_rate));
  const auto map_period  = std::chrono::duration<double>(1.0 / std::max(1e-6, cfg.map_publish_rate));

  pose_update_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(pose_period),
      std::bind(&HeightMappingNode::updateMapOrigin, this));

  map_publish_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(map_period),
      std::bind(&HeightMappingNode::publishHeightMap, this));
}

void HeightMappingNode::initializeServices() {
  // Placeholder: no hay servicios en esta versión local
}

void HeightMappingNode::lidarScanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (!lidarscan_received_) {
    frame_ids::sensor::LIDAR = msg->header.frame_id;
    lidarscan_received_ = true;
    if (!pose_update_timer_ || !map_publish_timer_) initializeTimers();
    RCLCPP_INFO(this->get_logger(),
      "\033[1;32m[HeightMappingNode]: LiDAR pointcloud received — starting timers.\033[0m");
  }

  // 1) TF
  geometry_msgs::msg::TransformStamped lidar2base, base2map;
  if (!tf_.lookupTransform(frame_ids::ROBOT_BASE, frame_ids::sensor::LIDAR, lidar2base) ||
      !tf_.lookupTransform(frame_ids::MAP,         frame_ids::ROBOT_BASE,   base2map)) {
    return;
  }

  // 2) ROS2 -> PCL
  auto scan_raw = pcl::make_shared<pcl::PointCloud<Laser>>();
  pcl::moveFromROSMsg(*msg, *scan_raw);

  // 3) Preproceso
  auto scan_preprocessed = processLidarScan(scan_raw, lidar2base, base2map);
  if (!scan_preprocessed) return;

  // 4) Rasterizado / mapeo  (forzar plantilla)
  auto scan_rasterized = mapper_->heightMapping<Laser>(scan_preprocessed);

  // 5) Publica
  publishRasterizedLidarScan(scan_rasterized);

  // 6) Raycasting (carving)  (forzar plantilla)
  auto sensor2map = TransformOps::multiplyTransforms(lidar2base, base2map);
  Eigen::Vector3f sensorOrigin3D(sensor2map.transform.translation.x,
                                 sensor2map.transform.translation.y,
                                 sensor2map.transform.translation.z);
  mapper_->raycasting<Laser>(sensorOrigin3D, scan_preprocessed);

  // Debug
  if (cfg.debug_mode && pub_debug_lidar_) {
    sensor_msgs::msg::PointCloud2 msg_debug;
    pcl::toROSMsg(*scan_preprocessed, msg_debug);
    msg_debug.header.stamp = this->now();
    msg_debug.header.frame_id = frame_ids::MAP;
    pub_debug_lidar_->publish(msg_debug);
  }
}

void HeightMappingNode::rgbdScanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (!rgbdscan_received_) {
    rgbdscan_received_ = true;
    frame_ids::sensor::RGBD = msg->header.frame_id;
    if (!pose_update_timer_ || !map_publish_timer_) initializeTimers();
    RCLCPP_INFO(this->get_logger(),
      "\033[1;33m[HeightMappingNode]: RGB-D cloud received — starting timers.\033[0m");
  }

  // 1) TF
  geometry_msgs::msg::TransformStamped camera2base, base2map;
  if (!tf_.lookupTransform(frame_ids::ROBOT_BASE, frame_ids::sensor::RGBD, camera2base) ||
      !tf_.lookupTransform(frame_ids::MAP,         frame_ids::ROBOT_BASE,   base2map)) {
    return;
  }

  // 2) ROS2 -> PCL
  auto scan_raw = pcl::make_shared<pcl::PointCloud<Color>>();
  pcl::moveFromROSMsg(*msg, *scan_raw);

  // 3) Preproceso
  auto scan_processed = processRGBDScan(scan_raw, camera2base, base2map);
  if (!scan_processed) return;

  // 4) Rasterizado / mapeo (ya forzado a Color)
  auto cloud_mapped = mapper_->heightMapping<Color>(scan_processed);

  // 5) Publica
  publishRasterizedRGBDScan(cloud_mapped);

  // Debug
  if (cfg.debug_mode && pub_debug_rgbd_) {
    sensor_msgs::msg::PointCloud2 debugMsg;
    pcl::toROSMsg(*scan_processed, debugMsg);
    debugMsg.header.stamp = this->now();
    debugMsg.header.frame_id = frame_ids::MAP;
    pub_debug_rgbd_->publish(debugMsg);
  }
}

pcl::PointCloud<Laser>::Ptr
HeightMappingNode::processLidarScan(const pcl::PointCloud<Laser>::Ptr &cloud,
                                    const geometry_msgs::msg::TransformStamped &lidar2base,
                                    const geometry_msgs::msg::TransformStamped &base2map) {
  // 1) filtro en base
  auto cloud_base = PointCloudOps::applyTransform<Laser>(cloud, lidar2base);
  auto cloud_processed = pcl::make_shared<pcl::PointCloud<Laser>>();

  // Forzar plantilla
  mapper_->fastHeightFilter<Laser>(cloud_base, cloud_processed);

  // rango local del mapa
  auto range = mapper_->getHeightMap().getLength() / 2.0;
  cloud_processed = PointCloudOps::passThrough<Laser>(cloud_processed, "x", -range.x(), range.x());
  cloud_processed = PointCloudOps::passThrough<Laser>(cloud_processed, "y", -range.y(), range.y());

  // opcional: recorta ángulo
  if (cfg.remove_backward_points)
    cloud_processed = PointCloudOps::filterAngle2D<Laser>(cloud_processed, -135.0, 135.0);

  // 2) a frame del mapa
  cloud_processed = PointCloudOps::applyTransform<Laser>(cloud_processed, base2map);

  if (cloud_processed->empty()) return nullptr;
  return cloud_processed;
}

pcl::PointCloud<Color>::Ptr
HeightMappingNode::processRGBDScan(const pcl::PointCloud<Color>::Ptr &cloud,
                                   const geometry_msgs::msg::TransformStamped &camera2base,
                                   const geometry_msgs::msg::TransformStamped &base2map) {
  auto cloud_base = PointCloudOps::applyTransform<Color>(cloud, camera2base);
  auto cloud_processed = pcl::make_shared<pcl::PointCloud<Color>>();

  mapper_->fastHeightFilter<Color>(cloud_base, cloud_processed);

  auto range = mapper_->getHeightMap().getLength() / 2.0;
  cloud_processed = PointCloudOps::passThrough<Color>(cloud_processed, "x", -range.x(), range.x());
  cloud_processed = PointCloudOps::passThrough<Color>(cloud_processed, "y", -range.y(), range.y());

  if (cfg.remove_backward_points)
    cloud_processed = PointCloudOps::filterAngle2D<Color>(cloud_processed, -135.0, 135.0);

  cloud_processed = PointCloudOps::applyTransform<Color>(cloud_processed, base2map);

  if (cloud_processed->empty()) return nullptr;
  return cloud_processed;
}

// Timers
void HeightMappingNode::updateMapOrigin() {
  geometry_msgs::msg::TransformStamped base2map;
  if (!tf_.lookupTransform(frame_ids::MAP, frame_ids::ROBOT_BASE, base2map))
    return;

  const auto robot_pos = grid_map::Position(base2map.transform.translation.x,
                                            base2map.transform.translation.y);
  mapper_->moveMapOrigin(robot_pos);
}

void HeightMappingNode::publishHeightMap() {
  // API ROS2: devuelve unique_ptr
  auto msg_ptr = grid_map::GridMapRosConverter::toMessage(mapper_->getHeightMap());
  msg_ptr->header.stamp = this->now();
  msg_ptr->header.frame_id = mapper_->getHeightMap().getFrameId();
  pub_heightmap_->publish(*msg_ptr);
}

void HeightMappingNode::publishRasterizedLidarScan(
    const pcl::PointCloud<Laser>::Ptr &scan) {
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*scan, msg);
  msg.header.stamp = this->now();
  msg.header.frame_id = frame_ids::MAP;
  pub_lidarscan_rasterized_->publish(msg);
}

void HeightMappingNode::publishRasterizedRGBDScan(
    const pcl::PointCloud<Color>::Ptr &scan) {
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*scan, msg);
  msg.header.stamp = this->now();
  msg.header.frame_id = frame_ids::MAP;
  pub_rgbdscan_rasterized_->publish(msg);
}

} // namespace height_mapping_ros

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<height_mapping_ros::HeightMappingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
