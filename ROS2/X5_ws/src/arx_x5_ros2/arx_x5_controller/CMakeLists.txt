cmake_minimum_required(VERSION 3.8)
project(arx_x5_controller)

# 抑制ROS2消息生成的警告
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wno-stringop-overflow)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(arm_control REQUIRED)  # 确保这个包存在
find_package(arx5_arm_msg REQUIRED)  # 确保这个包存在

include_directories(include)

# 如果目标是 ARM 架构
if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    message(STATUS "Target architecture is ARM")
    set(DEPEND_LIB ${CMAKE_CURRENT_SOURCE_DIR}/lib/arx_x5_src/libarx_x5_src-arm64.so)
else ()
    set(DEPEND_LIB ${CMAKE_CURRENT_SOURCE_DIR}/lib/arx_x5_src/libarx_x5_src.so)
endif ()

# 添加可执行文件 X5Controller
add_executable(X5Controller src/X5Controller.cpp src/utils.cpp)
target_include_directories(X5Controller PUBLIC lib/arx_hardware_interface/include lib/arx_x5_src/include)
target_link_libraries(X5Controller ${DEPEND_LIB})
ament_target_dependencies(X5Controller rclcpp ament_index_cpp std_msgs arx5_arm_msg arm_control)

# 添加可执行文件 KeyBoard
add_executable(KeyBoard src/KeyBoard.cpp)
ament_target_dependencies(KeyBoard rclcpp std_msgs arx5_arm_msg)

# 安装可执行文件
install(TARGETS
        X5Controller
        KeyBoard
        DESTINATION lib/${PROJECT_NAME})

# 安装依赖的库文件
install(FILES ${DEPEND_LIB}
        DESTINATION lib/
)

# 安装 launch 文件
install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
        FILES_MATCHING PATTERN "*.launch.py")

# 安装配置文件
install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config
        FILES_MATCHING PATTERN "*.yaml")

# 安装 URDF 文件
install(FILES x5.urdf
        DESTINATION share/${PROJECT_NAME}
)

install(FILES x5_master.urdf
        DESTINATION share/${PROJECT_NAME}
)

install(FILES x5_2025.urdf
        DESTINATION share/${PROJECT_NAME}
)

# 导出 ROS2 运行时依赖
ament_export_dependencies(rosidl_default_runtime)

ament_package()

