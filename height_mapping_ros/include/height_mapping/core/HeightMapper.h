#pragma once

#include <memory>
#include <unordered_map>
#include <cmath>

// Height Map (trae GridMap y tu core)
#include <height_mapping_core/height_mapping_core.h>

namespace height_mapping {

class HeightMapper {
public:
  struct Config {
    std::string estimator_type;
    std::string frame_id;
    double map_length_x;
    double map_length_y;
    double grid_resolution;
    double min_height;
    double max_height;
  } cfg;

  HeightMapper(const Config &cfg);

  // Aliases cómodos
  template <typename PointT>
  using Cloud = pcl::PointCloud<PointT>;
  template <typename PointT>
  using CloudPtr = typename Cloud<PointT>::Ptr;

  /**
   * @brief Gridded height mapping using height estimator
   * @param cloud: input pointcloud
   * @return filtered pointcloud (rasterizada)
   */
  template <typename PointT>
  CloudPtr<PointT> heightMapping(const CloudPtr<PointT> &cloud);

  /*
   * @brief Corrige el height map con raycasting (sensor→punto)
   */
  template <typename PointT>
  void raycasting(const Eigen::Vector3f &sensorOrigin,
                  const CloudPtr<PointT> &cloud);

  /**
   * @brief Filtro rápido por altura
   */
  template <typename PointT>
  void fastHeightFilter(const CloudPtr<PointT> &cloud,
                        CloudPtr<PointT> &filtered_cloud);

  void moveMapOrigin(const grid_map::Position &position);

  const HeightMap &getHeightMap() const { return map_; }
  HeightMap &getHeightMap() { return map_; }

  void setMapPosition(const grid_map::Position &position) { map_.setPosition(position); }
  void clearMap() { map_.clearAll(); }

private:
  void initMap();
  void initHeightEstimator();

  template <typename PointT>
  CloudPtr<PointT> cloudRasterization(const CloudPtr<PointT> &cloud, float gridSize);

  template <typename PointT>
  CloudPtr<PointT> cloudRasterizationAlt(const CloudPtr<PointT> &cloud, float gridSize);

  struct pair_hash {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2> &p) const {
      auto h1 = std::hash<T1>{}(p.first);
      auto h2 = std::hash<T2>{}(p.second);
      return h1 ^ h2;
    }
  };

  // Members
  HeightMap map_;

  // Height mapping objects
  FastHeightFilter heightFilter_;
  HeightEstimatorBase::Ptr height_estimator_;
  HeightMapRaycaster raycaster_;
};

} // namespace height_mapping
