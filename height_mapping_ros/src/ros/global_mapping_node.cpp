#include "height_mapping/ros/global_mapping_node.hpp"

#include <chrono>
#include <iomanip>
#include <sstream>
#include <cstdlib>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <grid_map_ros/GridMapRosConverter.hpp>
// opcional si usas conversión CV en otros sitios
#include <grid_map_cv/GridMapCvConverter.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_cpp/converter_options.hpp>

#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace {

// Directorio para guardar (por defecto, share/ del paquete)
std::string determineSaveDirectory(const std::string &requested_directory) {
  if (requested_directory.empty()) {
    try {
      const auto share = ament_index_cpp::get_package_share_directory("height_mapping");
      return share + std::string("/");
    } catch (...) {
      return std::string("./");
    }
  }

  std::string directory = requested_directory;
  if (!directory.empty() && directory.front() == '~') {
    if (const char *home = std::getenv("HOME")) {
      directory.replace(0, 1, home);
    }
  }
  if (!directory.empty() && directory.back() != '/')
    directory.push_back('/');
  return directory;
}

std::string createFilename(const std::string &directory,
                           const std::string &requested_filename) {
  if (requested_filename.empty()) {
    const auto now = std::chrono::system_clock::now();
    const auto now_c = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%Y-%m-%d-%H-%M-%S");
    return directory + "elevation_" + ss.str();
  }
  return directory + requested_filename;
}

} // namespace

namespace height_mapping_ros {

GlobalMappingNode::GlobalMappingNode()
: rclcpp::Node("global_mapping_node")
{
  // Parámetros del nodo
  cfg.lidarcloud_topic       = this->declare_parameter<std::string>("node.lidar_topic", "/velodyne_points");
  cfg.rgbdcloud_topic        = this->declare_parameter<std::string>("node.rgbd_topic",   "/camera/pointcloud/points");
  cfg.map_publish_rate       = this->declare_parameter<double>("node.map_publish_rate", 10.0);
  cfg.remove_backward_points = this->declare_parameter<bool>("node.remove_backward_points", false);
  cfg.debug_mode             = this->declare_parameter<bool>("node.debug_mode", false);
  cfg.map_save_format        = this->declare_parameter<std::string>("node.map_save_format", "bag");
  cfg.map_save_dir           = this->declare_parameter<std::string>("node.map_save_dir", "");

  // Frames desde parámetros
  frame_ids::loadFromConfig(*this);

  // Global Mapper
  auto gm_cfg = global_mapper::loadConfig(*this);
  mapper_ = std::make_unique<height_mapping::GlobalMapper>(gm_cfg);

  // Pub/Sub/Services
  initializePubSubs();
  initializeServices();

  RCLCPP_INFO(this->get_logger(),
              "\033[1;33m[GlobalMappingNode]: initialized. Waiting for scan inputs...\033[0m");
}

void GlobalMappingNode::initializePubSubs() {
  auto qos = rclcpp::SensorDataQoS();

  sub_lidarscan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cfg.lidarcloud_topic, qos,
      std::bind(&GlobalMappingNode::lidarScanCallback, this, _1));

  sub_rgbdscan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cfg.rgbdcloud_topic, qos,
      std::bind(&GlobalMappingNode::rgbdScanCallback, this, _1));

  pub_scan_rasterized_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/height_mapping/global/scan_rasterized", 1);

  pub_map_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/height_mapping/global/map_cloud", 1);

  pub_map_region_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/height_mapping/global/mapping_region", 1);
}

void GlobalMappingNode::initializeServices() {
  srv_save_map_ = this->create_service<height_mapping::srv::SaveMap>(
      "/height_mapping/global/save_map",
      std::bind(&GlobalMappingNode::saveMap, this, std::placeholders::_1, std::placeholders::_2));

  srv_clear_map_ = this->create_service<std_srvs::srv::Empty>(
      "/height_mapping/global/clear_map",
      std::bind(&GlobalMappingNode::clearMap, this, std::placeholders::_1, std::placeholders::_2));
}

void GlobalMappingNode::lidarScanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (!lidarscan_received_) {
    frame_ids::sensor::LIDAR = msg->header.frame_id;
    lidarscan_received_ = true;

    const auto period = std::chrono::duration<double>(1.0 / std::max(1e-6, cfg.map_publish_rate));
    map_publish_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&GlobalMappingNode::publishPointCloudMap, this));

    RCLCPP_INFO(this->get_logger(),
                "\033[1;32m[GlobalMappingNode]: LiDAR cloud received. Starting map publisher...\033[0m");
  }

  // TFs
  geometry_msgs::msg::TransformStamped lidar2base, base2map;
  if (!tf_.lookupTransform(frame_ids::ROBOT_BASE, frame_ids::sensor::LIDAR, lidar2base) ||
      !tf_.lookupTransform(frame_ids::MAP,         frame_ids::ROBOT_BASE,   base2map)) {
    return;
  }

  // ROS msg -> PCL
  auto scan_raw = pcl::make_shared<pcl::PointCloud<Laser>>();
  pcl::moveFromROSMsg(*msg, *scan_raw);

  auto scan_processed = processLidarScan(scan_raw, lidar2base, base2map);
  if (!scan_processed) return;

  // Forzar tipo de plantilla
  auto scan_rasterized = mapper_->heightMapping<Laser>(scan_processed);

  auto sensor2map = TransformOps::multiplyTransforms(lidar2base, base2map);
  Eigen::Vector3f sensorOrigin3D(sensor2map.transform.translation.x,
                                 sensor2map.transform.translation.y,
                                 sensor2map.transform.translation.z);

  mapper_->raycasting<Laser>(sensorOrigin3D, scan_processed);

  sensor_msgs::msg::PointCloud2 msg_cloud;
  pcl::toROSMsg(*scan_rasterized, msg_cloud);
  msg_cloud.header.stamp = this->now();
  msg_cloud.header.frame_id = frame_ids::MAP;
  pub_scan_rasterized_->publish(msg_cloud);
}

void GlobalMappingNode::rgbdScanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (!rgbdscan_received_) {
    rgbdscan_received_ = true;
    frame_ids::sensor::RGBD = msg->header.frame_id;

    if (!map_publish_timer_) {
      const auto period = std::chrono::duration<double>(1.0 / std::max(1e-6, cfg.map_publish_rate));
      map_publish_timer_ = this->create_wall_timer(
          std::chrono::duration_cast<std::chrono::milliseconds>(period),
          std::bind(&GlobalMappingNode::publishPointCloudMap, this));
    }

    RCLCPP_INFO(this->get_logger(),
                "\033[1;32m[GlobalMappingNode]: RGB-D cloud received. Starting map publisher...\033[0m");
  }

  // TFs
  geometry_msgs::msg::TransformStamped camera2base, base2map;
  if (!tf_.lookupTransform(frame_ids::ROBOT_BASE, frame_ids::sensor::RGBD, camera2base) ||
      !tf_.lookupTransform(frame_ids::MAP,         frame_ids::ROBOT_BASE,   base2map)) {
    return;
  }

  // ROS msg -> PCL
  auto scan_raw = pcl::make_shared<pcl::PointCloud<Color>>();
  pcl::moveFromROSMsg(*msg, *scan_raw);

  auto scan_processed = processRGBDScan(scan_raw, camera2base, base2map);
  if (!scan_processed) return;

  auto scan_rasterized = mapper_->heightMapping<Color>(scan_processed);

  auto sensor2map = TransformOps::multiplyTransforms(camera2base, base2map);
  Eigen::Vector3f sensorOrigin3D(sensor2map.transform.translation.x,
                                 sensor2map.transform.translation.y,
                                 sensor2map.transform.translation.z);

  mapper_->raycasting<Color>(sensorOrigin3D, scan_processed);

  sensor_msgs::msg::PointCloud2 msg_cloud;
  pcl::toROSMsg(*scan_rasterized, msg_cloud);
  msg_cloud.header.stamp = this->now();
  msg_cloud.header.frame_id = frame_ids::MAP;
  pub_scan_rasterized_->publish(msg_cloud);
}

pcl::PointCloud<Laser>::Ptr
GlobalMappingNode::processLidarScan(const pcl::PointCloud<Laser>::Ptr &cloud,
                                    const geometry_msgs::msg::TransformStamped &lidar2base,
                                    const geometry_msgs::msg::TransformStamped &base2map) {
  auto cloud_base = PointCloudOps::applyTransform<Laser>(cloud, lidar2base);
  auto cloud_processed = pcl::make_shared<pcl::PointCloud<Laser>>();

  mapper_->fastHeightFilter<Laser>(cloud_base, cloud_processed);

  // recorte (puedes cambiar límites por tamaño de mapa si quieres)
  cloud_processed = PointCloudOps::passThrough<Laser>(cloud_processed, "x", -10.0, 10.0);
  cloud_processed = PointCloudOps::passThrough<Laser>(cloud_processed, "y", -10.0, 10.0);

  if (cfg.remove_backward_points)
    cloud_processed = PointCloudOps::filterAngle2D<Laser>(cloud_processed, -135.0, 135.0);

  cloud_processed = PointCloudOps::applyTransform<Laser>(cloud_processed, base2map);
  if (cloud_processed->empty()) return nullptr;
  return cloud_processed;
}

pcl::PointCloud<Color>::Ptr
GlobalMappingNode::processRGBDScan(const pcl::PointCloud<Color>::Ptr &cloud,
                                   const geometry_msgs::msg::TransformStamped &camera2base,
                                   const geometry_msgs::msg::TransformStamped &base2map) {
  auto cloud_base = PointCloudOps::applyTransform<Color>(cloud, camera2base);
  auto cloud_processed = pcl::make_shared<pcl::PointCloud<Color>>();

  mapper_->fastHeightFilter<Color>(cloud_base, cloud_processed);

  cloud_processed = PointCloudOps::passThrough<Color>(cloud_processed, "x", -10.0, 10.0);
  cloud_processed = PointCloudOps::passThrough<Color>(cloud_processed, "y", -10.0, 10.0);

  if (cfg.remove_backward_points)
    cloud_processed = PointCloudOps::filterAngle2D<Color>(cloud_processed, -135.0, 135.0);

  cloud_processed = PointCloudOps::applyTransform<Color>(cloud_processed, base2map);
  if (cloud_processed->empty()) return nullptr;
  return cloud_processed;
}

void GlobalMappingNode::publishPointCloudMap() {
  if (!mapper_) return;

  std::vector<std::string> layers = {
      height_mapping::layers::Height::ELEVATION,
      height_mapping::layers::Height::ELEVATION_MAX,
      height_mapping::layers::Height::ELEVATION_MIN,
      height_mapping::layers::Height::ELEVATION_VARIANCE,
      height_mapping::layers::Height::N_MEASUREMENTS,
  };

  sensor_msgs::msg::PointCloud2 msg_map_cloud;
  toPointCloud2(mapper_->getHeightMap(),
                layers,
                mapper_->getMeasuredGridIndices(),
                msg_map_cloud);
  pub_map_cloud_->publish(msg_map_cloud);

  visualization_msgs::msg::Marker msg_map_region;
  toMapRegion(mapper_->getHeightMap(), msg_map_region);
  pub_map_region_->publish(msg_map_region);
}

void GlobalMappingNode::toPointCloud2(
    const HeightMap &map,
    const std::vector<std::string> &layers,
    const std::unordered_set<grid_map::Index> &measuredIndices,
    sensor_msgs::msg::PointCloud2 &cloud) {

  cloud.header.frame_id = map.getFrameId();
  const uint64_t t_ns = map.getTimestamp();
  builtin_interfaces::msg::Time stamp;
  stamp.sec     = static_cast<int32_t>(t_ns / 1000000000ULL);
  stamp.nanosec = static_cast<uint32_t>(t_ns % 1000000000ULL);
  cloud.header.stamp = stamp;
  
  cloud.is_dense = false;

  // Campos (x,y,z + capas)
  std::vector<std::string> fieldNames;
  fieldNames.reserve(layers.size() + 3);
  fieldNames.insert(fieldNames.end(), {"x", "y", "z"});
  for (const auto &layer : layers) {
    fieldNames.push_back(layer == "color" ? "rgb" : layer);
  }

  cloud.fields.clear();
  cloud.fields.reserve(fieldNames.size());
  int offset = 0;
  for (const auto &name : fieldNames) {
    sensor_msgs::msg::PointField f;
    f.name     = name;
    f.count    = 1;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.offset   = offset;
    cloud.fields.push_back(f);
    offset += sizeof(float);
  }

  const size_t num_points = measuredIndices.size();
  cloud.height = 1;
  cloud.width  = num_points;
  cloud.point_step = offset;
  cloud.row_step   = cloud.width * cloud.point_step;
  cloud.data.resize(cloud.height * cloud.row_step);

  // Iteradores
  std::unordered_map<std::string, sensor_msgs::PointCloud2Iterator<float>> iters;
  for (const auto &name : fieldNames) {
    iters.emplace(name, sensor_msgs::PointCloud2Iterator<float>(cloud, name));
  }

  size_t valid = 0;
  for (const auto &index : measuredIndices) {
    grid_map::Position3 pos;
    if (!map.getPosition3(height_mapping::layers::Height::ELEVATION, index, pos)) {
      continue;
    }

    for (auto &kv : iters) {
      auto &fname = kv.first;
      auto &it    = kv.second;
      if (fname == "x")         *it = static_cast<float>(pos.x());
      else if (fname == "y")    *it = static_cast<float>(pos.y());
      else if (fname == "z")    *it = static_cast<float>(pos.z());
      else if (fname == "rgb")  *it = static_cast<float>(map.at("color", index));
      else                      *it = static_cast<float>(map.at(fname, index));
      ++it;
    }
    ++valid;
  }

  cloud.width    = valid;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.data.resize(cloud.height * cloud.row_step);
}

void GlobalMappingNode::toMapRegion(const HeightMap &map,
                                    visualization_msgs::msg::Marker &marker) {
  marker.ns = "height_map";
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;

  marker.scale.x = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  marker.header.frame_id = map.getFrameId();
  marker.header.stamp    = this->now();
  marker.lifetime        = rclcpp::Duration(0,0);

  const float half_x = (map.getLength().x() - 0.5f * map.getResolution()) / 2.0f;
  const float half_y = (map.getLength().y() - 0.5f * map.getResolution()) / 2.0f;

  marker.points.resize(5);
  marker.points[0].x = map.getPosition().x() + half_x;
  marker.points[0].y = map.getPosition().y() + half_y;
  marker.points[0].z = 0;

  marker.points[1].x = map.getPosition().x() + half_x;
  marker.points[1].y = map.getPosition().y() - half_y;
  marker.points[1].z = 0;

  marker.points[2].x = map.getPosition().x() - half_x;
  marker.points[2].y = map.getPosition().y() - half_y;
  marker.points[2].z = 0;

  marker.points[3].x = map.getPosition().x() - half_x;
  marker.points[3].y = map.getPosition().y() + half_y;
  marker.points[3].z = 0;

  marker.points[4] = marker.points[0];
}

bool GlobalMappingNode::toPclCloud(
    const HeightMap &map,
    const std::unordered_set<grid_map::Index> &grid_indices,
    pcl::PointCloud<height_mapping_types::ElevationPoint> &cloud) {

  // Llena el header usando std_msgs::Header para evitar líos con stamp
  std_msgs::msg::Header h;
  h.frame_id = map.getFrameId();
  h.stamp    = this->now();
  pcl_conversions::toPCL(h, cloud.header);

  try {
    cloud.points.reserve(grid_indices.size());
    for (const auto &index : grid_indices) {
      grid_map::Position3 position;
      map.getPosition3(height_mapping::layers::Height::ELEVATION, index, position);

      height_mapping_types::ElevationPoint p;
      p.x = position.x();
      p.y = position.y();
      p.z = position.z();
      p.elevation           = map.at(height_mapping::layers::Height::ELEVATION, index);
      p.elevation_min       = map.at(height_mapping::layers::Height::ELEVATION_MIN, index);
      p.elevation_max       = map.at(height_mapping::layers::Height::ELEVATION_MAX, index);
      p.elevation_variance  = map.at(height_mapping::layers::Height::ELEVATION_VARIANCE, index);
      p.n_measurements      = map.at(height_mapping::layers::Height::N_MEASUREMENTS, index);

      cloud.push_back(p);
    }
    cloud.width    = cloud.points.size();
    cloud.height   = 1;
    cloud.is_dense = false;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Error converting height map to point cloud: %s", e.what());
    return false;
  }
  return true;
}

bool GlobalMappingNode::savePointCloud(
    const pcl::PointCloud<height_mapping_types::ElevationPoint> &cloud,
    const std::string &filename) {

  const std::string pcd_path = filename + ".pcd";
  if (pcl::io::savePCDFileASCII(pcd_path, cloud) == -1) {
    RCLCPP_WARN(this->get_logger(), "\033[33mFailed to save elevation cloud to %s\033[0m", pcd_path.c_str());
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "\033[1;32mElevation cloud saved to %s\033[0m", pcd_path.c_str());
  return true;
}

void GlobalMappingNode::clearMap(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                                 std::shared_ptr<std_srvs::srv::Empty::Response>) {
  mapper_->clearMap();
}

bool GlobalMappingNode::saveMapToBag(const HeightMap &map, const std::string &filename) {
  try {
    // 1) Convertir a mensaje (ROS2: devuelve unique_ptr)
    auto msg_ptr = grid_map::GridMapRosConverter::toMessage(map);

    // 2) Config rosbag2 (sqlite3 .db3)
    rosbag2_cpp::Writer writer;
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = filename;             // rosbag2 añadirá .db3
    storage_options.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format  = "cdr";
    converter_options.output_serialization_format = "cdr";

    writer.open(storage_options, converter_options);

    rosbag2_storage::TopicMetadata topic_md;
    topic_md.name = "/height_map_global";
    topic_md.type = "grid_map_msgs/msg/GridMap";
    topic_md.serialization_format = "cdr";
    writer.create_topic(topic_md);

    // 3) Escritura tipada (Writer serializa por ti)
    writer.write(*msg_ptr, topic_md.name, this->now());

    RCLCPP_INFO(this->get_logger(), "\033[1;32mSaved to %s.db3\033[0m", filename.c_str());
    return true;
  } catch (const std::exception &e) {
    RCLCPP_WARN(this->get_logger(), "\033[33mFailed to save height map to rosbag2: %s\033[0m", e.what());
    return false;
  }
}

void GlobalMappingNode::saveMap(
    const std::shared_ptr<height_mapping::srv::SaveMap::Request> req,
    std::shared_ptr<height_mapping::srv::SaveMap::Response> res) {

  RCLCPP_INFO(this->get_logger(), "\033[1;33mSaving map as %s...\033[0m", cfg.map_save_format.c_str());

  if (mapper_->getMeasuredGridIndices().empty()) {
    RCLCPP_ERROR(this->get_logger(), "\033[31mHeight map is empty. Failed to save.\033[0m");
    res->success = false;
    return;
  }

  const std::string directory = determineSaveDirectory(req->directory);
  const std::string filename  = createFilename(directory, req->filename);
  auto &map = mapper_->getHeightMap();

  if (cfg.map_save_format == "pcd") {
    pcl::PointCloud<height_mapping_types::ElevationPoint> cloud;
    const auto &indices = mapper_->getMeasuredGridIndices();
    if (!toPclCloud(map, indices, cloud)) { res->success = false; return; }
    res->success = savePointCloud(cloud, filename);
    return;
  } else if (cfg.map_save_format == "bag") {
    res->success = saveMapToBag(map, filename);
    return;
  } else {
    RCLCPP_ERROR(this->get_logger(), "\033[31mInvalid map save format: %s\033[0m", cfg.map_save_format.c_str());
    res->success = false;
    return;
  }
}

} // namespace height_mapping_ros

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<height_mapping_ros::GlobalMappingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
