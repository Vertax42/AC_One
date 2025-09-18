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
find_package(nlohmann_json CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

# Add include directory
include_directories(include)

# Add executable
add_executable(pico_xr_node 
  src/pico_xr_node.cpp
  src/log4z.cpp
  src/utils_common.cpp
)

# Link libraries
target_link_libraries(pico_xr_node
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libPXREARobotSDK.so
  nlohmann_json::nlohmann_json
  Eigen3::Eigen
  Boost::filesystem
)

ament_target_dependencies(pico_xr_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
)

# Install targets
install(TARGETS
  pico_xr_node
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