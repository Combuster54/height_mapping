### GlobalMapper
```sh
┌══════════════════════════════════════════════════════════════════════════════┐
│                               GLOBALMAPPER                                   │
│ cfg{frame, Lx/Ly, res, zmin/zmax, estimator_type, map_save_dir}              │
│ state: measured_indices_ (unordered_set<Index>)                              │
├─────────────────────────────────────┬─────────────────────────────────────────┤
│                                     │                                         │
│                                 INIT                                          │
│  ┌──────────────────────────────────▼──────────────────────────────────┐       │
│  │                           HEIGHTMAPPER                              │       │
│  │  map_ (GridMap+capas) | heightFilter_ | estimator_ | raycaster_     │       │
│  └───────────────┬───────────────────────────────────────────────┬─────┘       │
│                  │                                               │             │
│                  │                                   (opcional)  │             │
│                  │                                               │             │
│        ┌─────────▼─────────┐                             ┌───────▼──────────┐  │
│        │  heightMapping()  │                             │   raycasting()   │  │
│        └─────────┬─────────┘                             └────────┬─────────┘  │
│                  │                                               │             │
│       ENTRADA: cloud_in (frame==cfg.frame ?)                     │ ENTRADA: sensorOrigin, cloud_in
│                  │                                               │
│                  ▼                                               │
│        ┌──────────────────────┐                                  │
│        │ FastHeightFilter Z   │  ← filtra [zmin, zmax]           │
│        └─────────┬────────────┘                                  │
│                  │ (cloud_z)                                     │
│                  ▼                                               │
│        ┌──────────────────────┐                                  │
│        │ (opt) Rasterización  │  ← 1 punto/celda (grid=res)      │
│        └─────────┬────────────┘                                  │
│                  │ (cloud_grid)                                  │
│                  ▼                                               │
│        ┌──────────────────────────────────────────────────────┐   │
│        │  ESTIMADOR por celda (cfg.estimator_type)            │   │
│        │   • Kalman  → μ,σ² (fusión bayesiana)                │   │
│        │   • EMA     → μ = α·z + (1−α)μ                       │   │
│        │   • Stats   → media/var + SE/IC                      │   │
│        └─────────┬────────────────────────────────────────────┘   │
│                  │                                                │
│                  ▼                                                │
│        ┌──────────────────────────────────────────────────────┐   │
│        │   ACTUALIZA MAPA (capas):                            │   │
│        │   elevation / min / max / (variance | SE/IC) / n     │   │
│        │   + (lidar/intensity, rgbd r|g|b|color si aplica)    │   │
│        └─────────┬────────────────────────────────────────────┘   │
│                  │                                                │
│                  ├─────────► GlobalMapper::recordMeasuredCells()  │
│                  │             └─ llena measured_indices_         │
│                  │                                                │
│                  │                                                ▼
│                  │                                     ┌──────────────────────┐
│                  │                                     │ HeightMapRaycaster   │
│                  │                                     │  • scan/scan_height  │
│                  │                                     │  • traza rayos       │
│                  │                                     │  • corta obstáculo   │
│                  │                                     │  • recorta altura    │
│                  │                                     │    (z_rayo+ε), ++var │
│                  │                                     │    n=1, scan/ray*    │
│                  │                                     └─────────┬────────────┘
│                  │                                               │
│                  ▼                                               │
│        ┌──────────────────────────────────────────────────────┐   │
│        │        map_ final (GridMap + todas las capas)        │◄──┘
│        └──────────────────────────────────────────────────────┘
│
├───────────────────────────────────────────────────────────────────────────────┤
│ SALIDAS                                                                       │
│  • map_ actualizado (alturas, min/max, var/SE/IC, n, scan/* si raycasting)    │
│  • measured_indices_ (celdas observadas en la pasada)                         │
│  • (opcional) nube filtrada/rasterizada para depuración                       │
└═══════════════════════════════════════════════════════════════════════════════┘
```