### Infraestructura base para mapas de altura sobre grid_map + tipos de nube PCL:

Aliases de puntos PCL para sensores comunes.

HeightMap: envoltorio de grid_map::GridMap con capas estándar (elevación, varianza, conteo, etc.) y utilidades.

Definiciones de nombres de capas (Height/Scan/Confidence/Sensor).

Tipos de nube (cloud_types.h)
using Laser = pcl::PointXYZI; → LiDAR con intensidad.

using Color = pcl::PointXYZRGB; → RGB-D / cámara con color.

HeightMap (API esencial)
Extiende grid_map::GridMap con helpers de capas y acceso directo a matrices.

### Gestión de capas
addLayer(layer, default=NAN), addBasicLayer(layer), removeLayer(layer), hasLayer(layer)

Nota: get(layer) requiere que la capa exista (añádela antes).

### Buenas prácticas
Siempre añade la capa antes de get(layer).

Usa NaN para representar celdas vacías/no observadas.

Mantén mismo frame entre nube ↔ mapa.

Para rendimiento, accede por referencias a las matrices (evita copias).

### Casos típicos
Pre-filtro LiDAR en Z → guardar intensidad en lidar/intensity.

Fusión/estimación → leer/escribir elevation, variance, n_measured.

Confianza/diagnóstico → height/standard_error, height/confidence_interval, scan/*.