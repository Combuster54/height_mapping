#include "height_mapping/core/GlobalMapper.h"

namespace height_mapping {

GlobalMapper::GlobalMapper(const Config &cfg) : HeightMapper(cfg) {
  const auto &map = getHeightMap();
  measured_indices_.reserve(map.getSize().prod());
}

template <typename PointT>
typename GlobalMapper::CloudPtr<PointT>
GlobalMapper::heightMapping(const CloudPtr<PointT> &cloud) {
  auto cloud_rasterized = HeightMapper::heightMapping<PointT>(cloud);
  if (!cloud_rasterized) return nullptr;
  // Guardar celdas medidas
  recordMeasuredCells(getHeightMap(), *cloud_rasterized);
  return cloud_rasterized;
}

template <typename PointT>
void GlobalMapper::recordMeasuredCells(const height_mapping::HeightMap &map,
                                       const pcl::PointCloud<PointT> &cloud) {
  grid_map::Index idx;
  grid_map::Position pos;
  for (const auto &pt : cloud.points) {
    pos.x() = pt.x; pos.y() = pt.y;
    if (!map.getIndex(pos, idx)) continue;          // fuera del mapa
    if (map.isEmptyAt(idx))     continue;           // celda sin dato válido
    measured_indices_.insert(idx);
  }
}

/************ Instanciaciones explícitas ************/
template GlobalMapper::CloudPtr<Laser>
GlobalMapper::heightMapping<Laser>(const GlobalMapper::CloudPtr<Laser> &);
template void GlobalMapper::recordMeasuredCells<Laser>(
    const height_mapping::HeightMap&, const pcl::PointCloud<Laser>&);

template GlobalMapper::CloudPtr<Color>
GlobalMapper::heightMapping<Color>(const GlobalMapper::CloudPtr<Color> &);
template void GlobalMapper::recordMeasuredCells<Color>(
    const height_mapping::HeightMap&, const pcl::PointCloud<Color>&);

} // namespace height_mapping
