#pragma once

#include <cmath>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>

#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

class PointCloudOps {
public:
  template <typename T> using Cloud    = pcl::PointCloud<T>;
  template <typename T> using CloudPtr = typename Cloud<T>::Ptr;

  // Convierte geometry_msgs::Transform a una matriz 4x4 float (preferida por PCL)
  static Eigen::Matrix4f toMat4f(const geometry_msgs::msg::Transform &t) {
    Eigen::Quaterniond qd(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
    qd.normalize();
    Eigen::Isometry3d iso = Eigen::Translation3d(t.translation.x, t.translation.y, t.translation.z) * qd;
    return iso.matrix().cast<float>();
  }

  template <typename T>
  static CloudPtr<T> applyTransform(const CloudPtr<T> &input,
                                    const geometry_msgs::msg::TransformStamped &ts) {
    if (!input || input->empty()) return input;

    const Eigen::Matrix4f tf = toMat4f(ts.transform);

    auto output = pcl::make_shared<Cloud<T>>();
    pcl::transformPointCloud(*input, *output, tf);

    // Actualiza metadatos: conserva stamp y cambia frame al destino
    output->header = input->header;
    output->header.frame_id = ts.header.frame_id;
    output->is_dense = input->is_dense;
    return output;
  }

  template <typename T>
  static CloudPtr<T> passThrough(const CloudPtr<T> &input,
                                 const std::string &field,
                                 double minVal, double maxVal,
                                 bool invert = false) {
    if (!input || input->empty()) return input;
    auto output = pcl::make_shared<Cloud<T>>();
    pcl::PassThrough<T> pass;
    pass.setInputCloud(input);
    pass.setFilterFieldName(field);
    pass.setFilterLimits(static_cast<float>(minVal), static_cast<float>(maxVal));
    pass.setFilterLimitsNegative(invert);
    pass.filter(*output);

    output->header = input->header;
    // width/height los setea el filtro; reafirmamos is_dense
    output->is_dense = input->is_dense;
    return output;
  }

  template <typename T>
  static CloudPtr<T> filterRange2D(const CloudPtr<T> &input,
                                   double minRange, double maxRange) {
    if (!input || input->empty()) return input;
    auto output = pcl::make_shared<Cloud<T>>();
    output->header = input->header;
    output->points.reserve(input->points.size());

    const double rmin = minRange;
    const double rmax = maxRange;

    for (const auto &pt : input->points) {
      const double r = std::sqrt(pt.x * pt.x + pt.y * pt.y);
      if (r > rmin && r < rmax) output->points.push_back(pt);
    }
    output->width  = static_cast<uint32_t>(output->points.size());
    output->height = 1;
    output->is_dense = true; // o input->is_dense si prefieres heredar
    return output;
  }

  template <typename T>
  static CloudPtr<T> filterRange3D(const CloudPtr<T> &input,
                                   double minRange, double maxRange) {
    if (!input || input->empty()) return input;
    auto output = pcl::make_shared<Cloud<T>>();
    output->header = input->header;
    output->points.reserve(input->points.size());

    const double rmin = minRange;
    const double rmax = maxRange;

    for (const auto &pt : input->points) {
      const double r = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
      if (r > rmin && r < rmax) output->points.push_back(pt);
    }
    output->width  = static_cast<uint32_t>(output->points.size());
    output->height = 1;
    output->is_dense = true;
    return output;
  }

  template <typename T>
  static CloudPtr<T> filterAngle2D(const CloudPtr<T> &input,
                                   double startDeg, double endDeg,
                                   bool invert = false) {
    if (!input || input->empty()) return input;
    auto output = pcl::make_shared<Cloud<T>>();
    output->header = input->header;
    output->points.reserve(input->points.size());

    const double sRad = normalizeAngle(startDeg) * M_PI / 180.0;
    const double eRad = normalizeAngle(endDeg)   * M_PI / 180.0;

    for (const auto &p : input->points) {
      const double a = std::atan2(p.y, p.x);
      bool keep = (sRad <= eRad) ? (a >= sRad && a <= eRad) : (a >= sRad || a <= eRad);
      if (invert) keep = !keep;
      if (keep) output->points.push_back(p);
    }
    output->width  = static_cast<uint32_t>(output->points.size());
    output->height = 1;
    output->is_dense = true;
    return output;
  }

  template <typename T>
  static CloudPtr<T> downsampleVoxel(const CloudPtr<T> &input,
                                     double dx, double dy, double dz) {
    if (!input || input->empty()) return input;
    auto output = pcl::make_shared<Cloud<T>>();
    pcl::VoxelGrid<T> vg;
    vg.setInputCloud(input);
    vg.setLeafSize(static_cast<float>(dx),
                   static_cast<float>(dy),
                   static_cast<float>(dz));
    vg.filter(*output);

    output->header = input->header;
    output->is_dense = input->is_dense; // VoxelGrid suele devolver densa; mantenemos coherencia
    return output;
  }

  template <typename T>
  static CloudPtr<T> downsampleVoxel(const CloudPtr<T> &input, double leaf) {
    return downsampleVoxel<T>(input, leaf, leaf, leaf);
  }

private:
  static double normalizeAngle(double angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
  }

  PointCloudOps() = delete;
  ~PointCloudOps() = delete;
  PointCloudOps(const PointCloudOps &) = delete;
  PointCloudOps &operator=(const PointCloudOps &) = delete;
};
