#pragma once

#include "height_mapping/core/HeightMapper.h"
#include <unordered_set>

// hash y equal_to para grid_map::Index
namespace std {
template <> struct hash<grid_map::Index> {
  std::size_t operator()(const grid_map::Index &i) const {
    std::size_t h1 = std::hash<int>{}(i[0]);
    std::size_t h2 = std::hash<int>{}(i[1]);
    return h1 ^ (h2 << 1);
  }
};
template <> struct equal_to<grid_map::Index> {
  bool operator()(const grid_map::Index &a, const grid_map::Index &b) const {
    return (a[0] == b[0]) && (a[1] == b[1]);
  }
};
} // namespace std

namespace height_mapping {

class GlobalMapper : public HeightMapper {
public:
  struct Config : public HeightMapper::Config {
    std::string map_save_dir;
  } cfg;

  GlobalMapper(const Config &cfg);

  template <typename PointT>
  using Cloud = pcl::PointCloud<PointT>;
  template <typename PointT>
  using CloudPtr = typename Cloud<PointT>::Ptr;

  template <typename PointT>
  CloudPtr<PointT> heightMapping(const CloudPtr<PointT> &cloud);

  const std::unordered_set<grid_map::Index> &getMeasuredGridIndices() const {
    return measured_indices_;
  }

private:
  template <typename PointT>
  void recordMeasuredCells(const height_mapping::HeightMap &map,
                           const pcl::PointCloud<PointT> &cloud);

  std::unordered_set<grid_map::Index> measured_indices_;
};

} // namespace height_mapping
