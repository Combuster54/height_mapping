#include "height_mapping/core/HeightMapper.h"

namespace height_mapping {

HeightMapper::HeightMapper(const Config &cfg)
    : cfg{cfg}, heightFilter_{cfg.min_height, cfg.max_height} {
  initMap();
  initHeightEstimator();
}

void HeightMapper::initMap() {
  if (cfg.grid_resolution <= 0) {
    throw std::invalid_argument(
        "[height_mapping::HeightMapper]: Grid resolution must be positive");
  }
  if (cfg.map_length_x <= 0 || cfg.map_length_y <= 0) {
    throw std::invalid_argument(
        "[height_mapping::HeightMapper]: Map dimensions must be positive");
  }

  map_.setFrameId(cfg.frame_id);
  map_.setGeometry(grid_map::Length(cfg.map_length_x, cfg.map_length_y),
                   cfg.grid_resolution);
}

void HeightMapper::initHeightEstimator() {
  if (cfg.estimator_type == "KalmanFilter") {
    height_estimator_ = std::make_unique<height_mapping::KalmanEstimator>();
    std::cout << "\033[1;33m[height_mapping::HeightMapper]: Height estimator "
                 "type --> KalmanFilter \033[0m\n";
  } else if (cfg.estimator_type == "MovingAverage") {
    height_estimator_ = std::make_unique<height_mapping::MovingAverageEstimator>();
    std::cout << "\033[1;33m[height_mapping::HeightMapper]: Height estimator "
                 "type --> MovingAverage \033[0m\n";
  } else if (cfg.estimator_type == "StatMean") {
    height_estimator_ = std::make_unique<height_mapping::StatMeanEstimator>();
    std::cout << "\033[1;33m[height_mapping::HeightMapper]: Height estimator "
                 "type --> StatisticalMeanEstimator \033[0m\n";
  } else {
    height_estimator_ = std::make_unique<height_mapping::StatMeanEstimator>();
    std::cout << "\033[1;33m[height_mapping::HeightMapper] Invalid height "
                 "estimator type. Set Default: StatMeanEstimator \033[0m\n";
  }
}

template <typename PointT>
typename HeightMapper::CloudPtr<PointT>
HeightMapper::heightMapping(const CloudPtr<PointT> &cloud) {
  // 1) Rasterizar
  auto cloud_rasterized = cloudRasterization<PointT>(cloud, cfg.grid_resolution);
  if (!cloud_rasterized || cloud_rasterized->empty()) {
    std::cout << "\033[1;31m[height_mapping::HeightMapper]: Height estimation failed. "
                 "Rasterized cloud is empty\033[0m\n";
    return nullptr;
  }
  // 2) Estimar/actualizar mapa de altura
  height_estimator_->estimate(map_, *cloud_rasterized);
  return cloud_rasterized;
}

template <typename PointT>
void HeightMapper::fastHeightFilter(const CloudPtr<PointT> &cloud,
                                    CloudPtr<PointT> &filtered_cloud) {
  heightFilter_.filter<PointT>(cloud, filtered_cloud);
}

// Rejilla XY: en cada celda, conserva el punto con mayor Z
template <typename PointT>
typename HeightMapper::CloudPtr<PointT>
HeightMapper::cloudRasterization(const CloudPtr<PointT> &cloud, float gridSize) {
  if (!cloud || cloud->empty()) return cloud;

  std::unordered_map<std::pair<int, int>, PointT, pair_hash> cell_max;
  for (const auto &pt : *cloud) {
    int xi = static_cast<int>(std::floor(pt.x / gridSize));
    int yi = static_cast<int>(std::floor(pt.y / gridSize));
    auto key = std::make_pair(xi, yi);
    auto [it, inserted] = cell_max.try_emplace(key, pt);
    if (!inserted && pt.z > it->second.z) it->second = pt;
  }

  auto cloud_down = pcl::make_shared<pcl::PointCloud<PointT>>();
  cloud_down->reserve(cell_max.size());
  for (const auto &kv : cell_max) cloud_down->points.emplace_back(kv.second);
  cloud_down->header = cloud->header;
  return cloud_down;
}

// Igual que arriba, pero garantiza centro de celda y respeta límites del mapa
template <typename PointT>
typename HeightMapper::CloudPtr<PointT>
HeightMapper::cloudRasterizationAlt(const CloudPtr<PointT> &cloud, float /*gridSize*/) {
  if (!cloud || cloud->empty()) return cloud;

  std::unordered_map<std::pair<int, int>, PointT, pair_hash> gridCells;
  grid_map::Position measuredPosition;
  grid_map::Index measuredIndex;

  for (const auto &pt : *cloud) {
    measuredPosition << pt.x, pt.y;
    if (!map_.getIndex(measuredPosition, measuredIndex)) continue;

    auto key = std::make_pair(measuredIndex.x(), measuredIndex.y());
    auto [it, inserted] = gridCells.try_emplace(key, pt);

    if (inserted || pt.z > it->second.z) {
      it->second = pt;
      map_.getPosition(measuredIndex, measuredPosition);
      it->second.x = measuredPosition.x();
      it->second.y = measuredPosition.y();
    }
  }

  auto cloudDown = pcl::make_shared<pcl::PointCloud<PointT>>();
  cloudDown->reserve(gridCells.size());
  for (const auto &kv : gridCells) cloudDown->points.emplace_back(kv.second);
  cloudDown->header = cloud->header;
  return cloudDown;
}

template <typename PointT>
void HeightMapper::raycasting(const Eigen::Vector3f &sensorOrigin,
                              const CloudPtr<PointT> &cloud) {
  if (!cloud || cloud->empty()) return;
  // Para cada punto: traza rayo sensor→punto y recorta celdas del mapa por donde pasa.
  raycaster_.correctHeight(map_, *cloud, sensorOrigin);
}

void HeightMapper::moveMapOrigin(const grid_map::Position &position) {
  map_.move(position);
}

/************** INSTANTIACIONES EXPLÍCITAS **************/
// Asegúrate de que Laser/Color estén definidos antes de estas líneas.
template HeightMapper::CloudPtr<Laser>
HeightMapper::heightMapping<Laser>(const HeightMapper::CloudPtr<Laser>&);
template HeightMapper::CloudPtr<Color>
HeightMapper::heightMapping<Color>(const HeightMapper::CloudPtr<Color>&);

template void HeightMapper::fastHeightFilter<Laser>(
  const HeightMapper::CloudPtr<Laser>&,
  HeightMapper::CloudPtr<Laser>&);
template void HeightMapper::fastHeightFilter<Color>(
  const HeightMapper::CloudPtr<Color>&,
  HeightMapper::CloudPtr<Color>&);

template HeightMapper::CloudPtr<Laser>
HeightMapper::cloudRasterization<Laser>(const HeightMapper::CloudPtr<Laser>&, float);
template HeightMapper::CloudPtr<Color>
HeightMapper::cloudRasterization<Color>(const HeightMapper::CloudPtr<Color>&, float);

template HeightMapper::CloudPtr<Laser>
HeightMapper::cloudRasterizationAlt<Laser>(const HeightMapper::CloudPtr<Laser>&, float);
template HeightMapper::CloudPtr<Color>
HeightMapper::cloudRasterizationAlt<Color>(const HeightMapper::CloudPtr<Color>&, float);

template void HeightMapper::raycasting<Laser>(const Eigen::Vector3f&,
                                              const HeightMapper::CloudPtr<Laser>&);
template void HeightMapper::raycasting<Color>(const Eigen::Vector3f&,
                                              const HeightMapper::CloudPtr<Color>&);

} // namespace height_mapping
