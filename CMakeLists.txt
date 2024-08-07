cmake_minimum_required(VERSION 3.8)
project(stereo_visual_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -march=native")

# find dependencies
set(library_name sv_lib)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(g2o REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(${library_name}
  src/stereo_visual_slam.cpp
  src/frontend.cpp
  src/map_point.cpp
  src/map.cpp
  src/frame.cpp
  src/feature.cpp
  src/config.cpp
  src/pinhole_camera.cpp
  src/viewer.cpp
)

target_link_libraries(${library_name}
  g2o_core
  g2o_stuff
  g2o_types_sba
  g2o_types_slam3d
  g2o_solver_dense
  g2o_solver_csparse
  Sophus::Sophus
)

ament_target_dependencies(${library_name}
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  cv_bridge
  OpenCV
  tf2_eigen
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${G2O_INCLUDE_DIRS}
)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(stereo_visual_slam_node
  executables/stereo_visual_slam_node.cpp
)
ament_target_dependencies(stereo_visual_slam_node
  rclcpp
)
target_link_libraries(stereo_visual_slam_node
  ${library_name}
)

install(TARGETS
  stereo_visual_slam_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
