cmake_minimum_required(VERSION 3.8)
project(localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 dependencies
set(ROS2_DEPS
  rclcpp
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  geometry_msgs
  tf2_geometry_msgs
  pcl_conversions
)

find_package(ament_cmake REQUIRED)
foreach(pkg IN LISTS ROS2_DEPS)
  find_package(${pkg} REQUIRED)
endforeach()

# Find PCL with ICP components
find_package(PCL REQUIRED COMPONENTS common io registration)

add_executable(localization_node src/localization_node.cpp)

ament_target_dependencies(localization_node ${ROS2_DEPS})

target_include_directories(localization_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(localization_node
  ${PCL_LIBRARIES}
)

install(TARGETS localization_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
