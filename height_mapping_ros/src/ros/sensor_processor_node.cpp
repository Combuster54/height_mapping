#include "height_mapping/ros/sensor_processor_node.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace height_mapping_ros {

SensorProcessorNode::SensorProcessorNode()
: rclcpp::Node("sensor_processor_node")
{

  tf_ = std::make_unique<TransformOps>("transform_ops");  // arranca hilo + executor interno

  // Cargar parámetros
  loadConfig();

  // Cargar frame IDs desde parámetros (ROS 2)
  frame_ids::loadFromConfig(*this);

  // Pub/Sub
  initializePubSubs();

  RCLCPP_INFO(this->get_logger(),
    "\033[1;32m[SensorProcessorNode]: listo; esperando %zu nubes...\033[0m",
    cfg.inputcloud_topics.size());
}

void SensorProcessorNode::loadConfig() {
  cfg.inputcloud_topics = this->declare_parameter<std::vector<std::string>>(
      "node.input_cloud_topics",
      std::vector<std::string>{"/front/front_cam_depth_sensor/points", "/left/left_cam_depth_sensor/points", "/right/right_cam_depth_sensor/points"});

  cfg.outputcloud_topic     = this->declare_parameter<std::string>("node.outputcloud_topic", "/sensor_processor/points");
  cfg.cloud_publish_rate    = this->declare_parameter<double>("node.cloud_publish_rate", 10.0);
  cfg.downsample_resolution = this->declare_parameter<double>("node.downsample_resolution", 0.1);
  cfg.min_range_threshold   = this->declare_parameter<double>("node.min_range_threshold", 0.3);
  cfg.max_range_threshold   = this->declare_parameter<double>("node.max_range_threshold", 5.0);
}

void SensorProcessorNode::initializePubSubs() {
  auto qos = rclcpp::SensorDataQoS();
  const rmw_qos_profile_t qos_profile = qos.get_rmw_qos_profile();

  const int queue_size = 10;
  sub_clouds_.reserve(cfg.inputcloud_topics.size());

  // Usa el overload: Subscriber(NodeType* node, const std::string& topic, rmw_qos_profile_t qos)
  for (const auto &topic : cfg.inputcloud_topics) {
    sub_clouds_.push_back(std::make_shared<CloudSubscriber>(
        this,                    // <-- Node* (válido dentro del ctor)
        topic,
        qos_profile));
  }

  if (sub_clouds_.size() == 2) {
    sync2_ = std::make_shared<Synchronizer2>(
        SyncPolicy2(queue_size), *sub_clouds_[0], *sub_clouds_[1]);
    sync2_->registerCallback(std::bind(&SensorProcessorNode::syncCallback2, this, _1, _2));
  } else if (sub_clouds_.size() == 3) {
    sync3_ = std::make_shared<Synchronizer3>(
        SyncPolicy3(queue_size), *sub_clouds_[0], *sub_clouds_[1], *sub_clouds_[2]);
    sync3_->registerCallback(std::bind(&SensorProcessorNode::syncCallback3, this, _1, _2, _3));
  } else {
    RCLCPP_ERROR(this->get_logger(),
      "Se requieren 2 o 3 tópicos; configurados: %zu", sub_clouds_.size());
  }

  pub_cloud_processed_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          cfg.outputcloud_topic, rclcpp::SensorDataQoS());
}



static inline void ros2ToPcl(const sensor_msgs::msg::PointCloud2 &msg,
                             pcl::PointCloud<Color> &cloud_out)
{
  pcl::PCLPointCloud2 tmp;
  pcl_conversions::toPCL(msg, tmp);
  pcl::fromPCLPointCloud2(tmp, cloud_out);
}

void SensorProcessorNode::syncCallback2(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg1,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg2) {

  // ROS2 → PCL (vía toPCL + fromPCLPointCloud2)
  ros2ToPcl(*msg1, *cloud1_);
  ros2ToPcl(*msg2, *cloud2_);

  // Downsampling
  downsampled1_ = PointCloudOps::downsampleVoxel<Color>(cloud1_, cfg.downsample_resolution);
  downsampled2_ = PointCloudOps::downsampleVoxel<Color>(cloud2_, cfg.downsample_resolution);

  cloud1_->clear(); cloud2_->clear();

  // Transformar a base_link
  transformToBaselink(downsampled1_, transformed1_, msg1->header.frame_id);
  transformToBaselink(downsampled2_, transformed2_, msg2->header.frame_id);

  downsampled1_->clear(); downsampled2_->clear();

  // Filtro por rango 2D
  filtered1_ = PointCloudOps::filterRange2D<Color>(transformed1_, cfg.min_range_threshold, cfg.max_range_threshold);
  filtered2_ = PointCloudOps::filterRange2D<Color>(transformed2_, cfg.min_range_threshold, cfg.max_range_threshold);

  transformed1_->clear(); transformed2_->clear();

  // Merge
  *filtered1_ += *filtered2_;

  // PCL → ROS2
  sensor_msgs::msg::PointCloud2 out;
  pcl::toROSMsg(*filtered1_, out);
  out.header.frame_id = frame_ids::ROBOT_BASE;
  out.header.stamp    = msg1->header.stamp;
  pub_cloud_processed_->publish(out);

  filtered1_->clear(); filtered2_->clear();
}

void SensorProcessorNode::syncCallback3(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg1,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg2,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg3) {

  ros2ToPcl(*msg1, *cloud1_);
  ros2ToPcl(*msg2, *cloud2_);
  ros2ToPcl(*msg3, *cloud3_);

  downsampled1_ = PointCloudOps::downsampleVoxel<Color>(cloud1_, cfg.downsample_resolution);
  downsampled2_ = PointCloudOps::downsampleVoxel<Color>(cloud2_, cfg.downsample_resolution);
  downsampled3_ = PointCloudOps::downsampleVoxel<Color>(cloud3_, cfg.downsample_resolution);

  cloud1_->clear(); cloud2_->clear(); cloud3_->clear();

  transformToBaselink(downsampled1_, transformed1_, msg1->header.frame_id);
  transformToBaselink(downsampled2_, transformed2_, msg2->header.frame_id);
  transformToBaselink(downsampled3_, transformed3_, msg3->header.frame_id);

  downsampled1_->clear(); downsampled2_->clear(); downsampled3_->clear();

  filtered1_ = PointCloudOps::filterRange2D<Color>(transformed1_, cfg.min_range_threshold, cfg.max_range_threshold);
  filtered2_ = PointCloudOps::filterRange2D<Color>(transformed2_, cfg.min_range_threshold, cfg.max_range_threshold);
  filtered3_ = PointCloudOps::filterRange2D<Color>(transformed3_, cfg.min_range_threshold, cfg.max_range_threshold);

  transformed1_->clear(); transformed2_->clear(); transformed3_->clear();

  *filtered1_ += *filtered2_;
  *filtered1_ += *filtered3_;

  sensor_msgs::msg::PointCloud2 out;
  pcl::toROSMsg(*filtered1_, out);
  out.header.frame_id = frame_ids::ROBOT_BASE;
  out.header.stamp    = msg1->header.stamp;
  pub_cloud_processed_->publish(out);

  filtered1_->clear(); filtered2_->clear(); filtered3_->clear();
}

void SensorProcessorNode::transformToBaselink(
    const pcl::PointCloud<Color>::Ptr &cloud_in,
    pcl::PointCloud<Color>::Ptr &cloud_out,
    const std::string &sensor_frame) {

  geometry_msgs::msg::TransformStamped tf_s2b;
  if (!tf_->lookupTransform(frame_ids::ROBOT_BASE, sensor_frame, tf_s2b))
    return;

  cloud_out = PointCloudOps::applyTransform<Color>(cloud_in, tf_s2b);
}

} // namespace height_mapping_ros

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<height_mapping_ros::SensorProcessorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
