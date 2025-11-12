#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "common/ros/common.h"          // tu versión ROS 2 (incluye PointCloudOps)
#include "height_mapping/core/core.h"   // define tipos Laser/Color
#include "height_mapping/ros/config.h"  // frame_ids::loadFromConfig
#include "common/ros/TransformOps.h"

namespace height_mapping_ros {

class SensorProcessorNode : public rclcpp::Node {
public:
  struct Config {
    std::vector<std::string> inputcloud_topics;
    std::string outputcloud_topic;
    double cloud_publish_rate;
    double downsample_resolution;
    double min_range_threshold;
    double max_range_threshold;
  } cfg;

  SensorProcessorNode();
  ~SensorProcessorNode() override = default;

private:
  void loadConfig();
  void initializePubSubs();

  // Callbacks sincronizados
  void syncCallback2(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg1,
                     const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg2);
  void syncCallback3(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg1,
                     const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg2,
                     const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg3);

  void transformToBaselink(const pcl::PointCloud<Color>::Ptr &cloud_in,
                           pcl::PointCloud<Color>::Ptr &cloud_out,
                           const std::string &sensor_frame);

  // message_filters
  using CloudSubscriber = message_filters::Subscriber<sensor_msgs::msg::PointCloud2>;
  using SyncPolicy2 = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>;
  using SyncPolicy3 = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>;
  using Synchronizer2 = message_filters::Synchronizer<SyncPolicy2>;
  using Synchronizer3 = message_filters::Synchronizer<SyncPolicy3>;

  // Subs y sincronizadores
  std::vector<std::shared_ptr<CloudSubscriber>> sub_clouds_;
  std::shared_ptr<Synchronizer2> sync2_;
  std::shared_ptr<Synchronizer3> sync3_;

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_processed_;

  // TF helper
  // TransformOps tf_;
  std::unique_ptr<TransformOps> tf_;

  // PCL buffers
  pcl::PointCloud<Color>::Ptr cloud1_{new pcl::PointCloud<Color>};
  pcl::PointCloud<Color>::Ptr cloud2_{new pcl::PointCloud<Color>};
  pcl::PointCloud<Color>::Ptr cloud3_{new pcl::PointCloud<Color>};

  pcl::PointCloud<Color>::Ptr downsampled1_{new pcl::PointCloud<Color>};
  pcl::PointCloud<Color>::Ptr downsampled2_{new pcl::PointCloud<Color>};
  pcl::PointCloud<Color>::Ptr downsampled3_{new pcl::PointCloud<Color>};

  pcl::PointCloud<Color>::Ptr filtered1_{new pcl::PointCloud<Color>};
  pcl::PointCloud<Color>::Ptr filtered2_{new pcl::PointCloud<Color>};
  pcl::PointCloud<Color>::Ptr filtered3_{new pcl::PointCloud<Color>};

  pcl::PointCloud<Color>::Ptr transformed1_{new pcl::PointCloud<Color>};
  pcl::PointCloud<Color>::Ptr transformed2_{new pcl::PointCloud<Color>};
  pcl::PointCloud<Color>::Ptr transformed3_{new pcl::PointCloud<Color>};
};

} // namespace height_mapping_ros
