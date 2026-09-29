cmake_minimum_required(VERSION 3.8)
project(handheld_bringup)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
  add_compile_options(-Wall -Wextra)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rog_map REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP)

# ---------------------------------------------------------------------------
# rog_map_node：独立运行 ROGMap（订阅 LIO 点云与里程计，发布 layer_type 等）
# rog_map 的编译宏为 PUBLIC 但未导出，消费方需要自行定义。
# ---------------------------------------------------------------------------
add_executable(rog_map_node src/rog_map_node.cpp)
target_compile_definitions(rog_map_node PRIVATE
  ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/"
  ORIGIN_AT_CORNER
  USE_ROS2
  QT_NO_VERSION_TAGGING
)
target_include_directories(rog_map_node PRIVATE ${PCL_INCLUDE_DIRS})
ament_target_dependencies(rog_map_node rclcpp tf2_ros rog_map)
target_link_libraries(rog_map_node Eigen3::Eigen ${PCL_LIBRARIES} yaml-cpp)
if(OpenMP_CXX_FOUND)
  target_link_libraries(rog_map_node OpenMP::OpenMP_CXX)
endif()

# ---------------------------------------------------------------------------
# tunnel_status_node：统计 ROI 内 layer_type，输出 PASSABLE/BLOCKED/UNKNOWN
# ---------------------------------------------------------------------------
add_executable(tunnel_status_node src/tunnel_status_node.cpp)
ament_target_dependencies(tunnel_status_node
  rclcpp std_msgs nav_msgs visualization_msgs geometry_msgs)

install(TARGETS rog_map_node tunnel_status_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/bag_to_tum.py
  scripts/resource_monitor.py
  scripts/tunnel_event_marker.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
