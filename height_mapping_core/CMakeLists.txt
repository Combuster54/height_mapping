cmake_minimum_required(VERSION 3.16)
project(height_mapping_core)

# C++
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

# Deps
find_package(ament_cmake REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

# Lib
add_library(${PROJECT_NAME} SHARED
  src/HeightMap.cpp
  src/KalmanEstimator.cpp
  src/MovingAverageEstimator.cpp
  src/StatMeanEstimator.cpp
  src/FastHeightFilter.cpp
)

# Includes correctos para build/install
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
)

# (Opcional) defs de PCL
target_compile_definitions(${PROJECT_NAME} PRIVATE ${PCL_DEFINITIONS})

# ⬇️ Firma PLANA (sin PUBLIC/PRIVATE) para evitar el conflicto
target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
)

# ament deps (esta función internamente usa firma PLANA)
ament_target_dependencies(${PROJECT_NAME}
  grid_map_core
)

# Install & export
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(grid_map_core)

ament_package()
