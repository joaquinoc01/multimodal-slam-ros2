cmake_minimum_required(VERSION 3.8)
project(lidar_frontend)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(ROS2_DEPS
  rclcpp
  sensor_msgs
  tf2_ros
  pcl_conversions
)

find_package(ament_cmake REQUIRED)
foreach(pkg IN LISTS ROS2_DEPS)
  find_package(${pkg} REQUIRED)
endforeach()

find_package(PCL REQUIRED COMPONENTS common io)

add_executable(scan_node src/scan_subscriber.cpp)

ament_target_dependencies(scan_node ${ROS2_DEPS})

target_include_directories(scan_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(scan_node ${PCL_LIBRARIES})

install(TARGETS
  scan_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()