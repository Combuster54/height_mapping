Pequeño filtro “pass-through” en Z para nubes PCL. Elimina puntos fuera de [minZ, maxZ]. Genérico para cualquier PointT con campo .z.

¿Para qué sirve?
Recortar rápido por altura (suelo/techo/ruido).

Pre-filtrar antes de fusionar en el height map o estimadores.

Interfaz
Ctor: FastHeightFilter(double minZ, double maxZ)

filter:
template <typename PointT>
void filter(const pcl::PointCloud<PointT>::Ptr& cloud,
pcl::PointCloud<PointT>::Ptr& cloudFiltered);

Por defecto, si usas minZ = -inf y maxZ = +inf, no filtra nada.

Flujo (mini)
```
1) Recibe nube de entrada (Ptr) y salida (Ptr).
2) Itera puntos:
   - Si minZ_ ≤ p.z ≤ maxZ_  → mantener
   - Si no → descartar
3) Escribe la nube filtrada en cloudFiltered.
```
Complejidad

- Tiempo: O(N)

- Memoria: proporcional a nº de puntos que pasan el filtro.

Uso rápido
```sh
using PointT = pcl::PointXYZI;

auto in  = boost::make_shared<pcl::PointCloud<PointT>>();
auto out = boost::make_shared<pcl::PointCloud<PointT>>();

// Ejemplo: quedarnos con [-0.5 m, 2.0 m]
auto f = std::make_shared<height_mapping::FastHeightFilter>(-0.5, 2.0);
f->filter<PointT>(in, out);
```

Buenas prácticas
- Asegura que out es distinto a in (evita sobrescribir).

- Define minZ/maxZ en el mismo frame que la nube.

- Si tu fuente trae NaN/Inf en z, límpialos antes o trátalos como fuera de rango.

- Encadénalo antes de estimadores (Kalman/EMA/Stats) para ahorrar cómputo.

Casos límite

- minZ > maxZ: rango inválido (define correctamente).

- minZ == -inf y maxZ == +inf: sin efecto (pasa todo).

- Inclusión en bordes: incluye puntos con z == minZ o z == maxZ.

Similar al PassThrough de PCL en eje z, pero integrado y minimalista para height mapping.