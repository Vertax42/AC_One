cmake_minimum_required(VERSION 3.8)
project(pico_xr_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(arx5_arm_msg REQUIRED)
find_package(arm_control REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

# Add include directory
include_directories(include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)


# Add original pico_xr_node executable
add_executable(pico_xr_node 
  src/pico_xr_node.cpp
  src/log4z.cpp
  src/utils_common.cpp
)

# Add new teleop executable
add_executable(pico_xr_teleop_node 
  src/pico_xr_teleop_node.cpp
)

# Link libraries for pico_xr_node
target_link_libraries(pico_xr_node
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libPXREARobotSDK.so
  nlohmann_json::nlohmann_json
  Eigen3::Eigen
  Boost::filesystem
)

# Dependencies for pico_xr_node
ament_target_dependencies(pico_xr_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
)

# Dependencies for pico_xr_teleop_node
ament_target_dependencies(pico_xr_teleop_node
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  arx5_arm_msg
  arm_control
)

# Link Eigen3 to pico_xr_teleop_node
target_link_libraries(pico_xr_teleop_node
  Eigen3::Eigen
)


# Install targets
install(TARGETS
  pico_xr_node
  pico_xr_teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install shared library
install(FILES
  lib/libPXREARobotSDK.so
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
