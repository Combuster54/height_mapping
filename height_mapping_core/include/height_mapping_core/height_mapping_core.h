#pragma once

// Reduce las alturas Z que son mayores al z_rayo + umbral, se detiene en obstaculos estaticos para no afectarlos
#include "height_mapping_core/height_correctors/HeightMapRaycaster.h"

// KalmanEstimator: fusiona cada escaneo con el HeightMap por celda (Kalman),
// corrige altura con incertidumbre (varianza) y actualiza min/max, confianza,
// e intensidad/color cuando existan.
#include "height_mapping_core/height_estimators/KalmanEstimator.h"

// MovingAverageEstimator: suaviza alturas (e intensidad/color) por celda con media móvil
// exponencial (α); actualiza min/max. Rápido, pero no modela incertidumbre.
#include "height_mapping_core/height_estimators/MovingAverageEstimator.h"

// StatMeanEstimator: acumula estadísticas por celda (Welford): media y varianza online,
// min/max y calidad (error estándar e intervalo de confianza); opcional intensidad/color.
#include "height_mapping_core/height_estimators/StatMeanEstimator.h"

// filtra la nube por altura Z, copiando solo puntos con z ∈ [minZ, maxZ] (admite ±∞ como límites).
#include "height_mapping_core/height_filters/FastHeightFilter.h"

// Tipos de pcl
#include "height_mapping_core/height_map/cloud_types.h"

// wrapper de grid_map para mapa 2.5D; gestiona capas de altura (elevación, min/max, varianza, # mediciones),
// valida celdas y ofrece estadísticas/vecinos. Incluye HeightMapMath para min/max por capa.
#include "height_mapping_core/height_map/HeightMap.h"

using namespace height_mapping;