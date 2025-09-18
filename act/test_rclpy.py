#!/usr/bin/env python3

import sys

print(f"Python版本: {sys.version}")
print(f"Python路径: {sys.executable}")
print("=" * 50)

try:
    import rclpy
    print("✅ rclpy导入成功")

    # 检查rclpy版本
    if hasattr(rclpy, '__version__'):
        print(f"✅ rclpy版本: {rclpy.__version__}")
    else:
        print("⚠️  无法获取rclpy版本信息")

    # 检查rclpy路径
    import rclpy
    print(f"✅ rclpy路径: {rclpy.__file__}")

    # 测试rclpy初始化
    try:
        rclpy.init(args=None)
        print("✅ rclpy.init() 成功")

        # 测试创建节点
        from rclpy.node import Node

        class TestNode(Node):
            def __init__(self):
                super().__init__('test_node')

        node = TestNode()
        print("✅ ROS2节点创建成功")
        print(f"✅ 节点名称: {node.get_name()}")

        # 清理
        node.destroy_node()
        rclpy.shutdown()
        print("✅ rclpy清理成功")

    except Exception as e:
        print(f"❌ rclpy功能测试失败: {e}")

    # 测试其他ROS2消息导入
    try:
        from geometry_msgs.msg import Twist
        from sensor_msgs.msg import Image, CompressedImage
        from std_msgs.msg import Int32MultiArray
        print("✅ 标准ROS2消息导入成功")

        # 显示消息类型
        print(f"✅ Twist: {Twist}")
        print(f"✅ Image: {Image}")
        print(f"✅ CompressedImage: {CompressedImage}")

    except ImportError as e:
        print(f"❌ ROS2消息导入失败: {e}")

    # 测试cv_bridge
    try:
        from cv_bridge import CvBridge
        bridge = CvBridge()
        print("✅ cv_bridge导入成功")
        print(f"✅ CvBridge: {CvBridge}")
    except ImportError as e:
        print(f"❌ cv_bridge导入失败: {e}")

except ImportError as e:
    print(f"❌ rclpy导入失败: {e}")
    print("请检查ROS2是否正确安装在ros_openpi环境中")

print("=" * 50)
print("ROS2环境测试完成")