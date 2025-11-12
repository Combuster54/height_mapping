### sensor_processor_node

Pensado para unir varias cámaras RGB-D (o, en general, nubes PointXYZRGB) en una sola nube preprocesada.

### height_mapping_node

Nodo  recibe nubes LiDAR y RGB-D, las preprocesa y transforma al frame del mapa, actualiza un HeightMap (rasterización + estimación y, para LiDAR, raycasting), publica el GridMap y las nubes rasterizadas, y recentra el mapa periódicamente con la pose del robot.

### global_mapping_node

Es un nodo de mapeo global que suscribe LiDAR y/o RGB-D, preprocesa y transforma las nubes al frame del mapa, actualiza el GlobalMapper (rasterización + estimación y raycasting), publica periódicamente el mapa como PointCloud2 y el contorno de la región, y ofrece servicios para guardar (PCD/BAG) o limpiar el mapa.