cmake_minimum_required(VERSION 3.8)
project(arm_control)

# 找到需要的依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 声明消息文件
rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/PosCmd.msg"
        "msg/JointControl.msg"
        "msg/ArxImu.msg"
        DEPENDENCIES
        std_msgs
        geometry_msgs
        builtin_interfaces
)

# 安装消息文件
install(DIRECTORY msg
        DESTINATION share/${PROJECT_NAME}/msg)

# 如果将来需要生成可执行文件或库，使用以下命令：
# add_executable(example_node src/example_node.cpp)
# ament_target_dependencies(example_node rclcpp std_msgs geometry_msgs)
# install(TARGETS example_node DESTINATION lib/${PROJECT_NAME})

# 添加接口依赖并导出
ament_export_dependencies(rosidl_default_runtime)

ament_package()

