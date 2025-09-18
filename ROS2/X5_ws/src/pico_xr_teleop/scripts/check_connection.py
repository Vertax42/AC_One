#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Joy
import time

class PicoXRConnectionChecker(Node):
    def __init__(self):
        super().__init__('pico_xr_connection_checker')
        
        # Subscribers
        self.left_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pico_xr/left_controller/pose',
            self.left_pose_callback,
            10
        )
        
        self.right_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pico_xr/right_controller/pose',
            self.right_pose_callback,
            10
        )
        
        self.headset_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pico_xr/headset/pose',
            self.headset_pose_callback,
            10
        )
        
        # Connection status
        self.left_connected = False
        self.right_connected = False
        self.headset_connected = False
        
        self.left_last_time = time.time()
        self.right_last_time = time.time()
        self.headset_last_time = time.time()
        
        # Timer for status check
        self.timer = self.create_timer(2.0, self.check_connection_status)
        
        self.get_logger().info("PICO XR Connection Checker started")
    
    def left_pose_callback(self, msg):
        self.left_connected = True
        self.left_last_time = time.time()
    
    def right_pose_callback(self, msg):
        self.right_connected = True
        self.right_last_time = time.time()
    
    def headset_pose_callback(self, msg):
        self.headset_connected = True
        self.headset_last_time = time.time()
    
    def check_connection_status(self):
        current_time = time.time()
        
        # Check for stale data (no data for 5 seconds)
        if current_time - self.left_last_time > 5.0:
            self.left_connected = False
        if current_time - self.right_last_time > 5.0:
            self.right_connected = False
        if current_time - self.headset_last_time > 5.0:
            self.headset_connected = False
        
        # Log status
        status_msg = (
            f"PICO XR Connection Status:\n"
            f"  Left Controller: {'✓' if self.left_connected else '✗'}\n"
            f"  Right Controller: {'✓' if self.right_connected else '✗'}\n"
            f"  Headset: {'✓' if self.headset_connected else '✗'}\n"
            f"  Last data times: L={current_time - self.left_last_time:.1f}s, "
            f"R={current_time - self.right_last_time:.1f}s, "
            f"H={current_time - self.headset_last_time:.1f}s"
        )
        
        if self.left_connected or self.right_connected or self.headset_connected:
            self.get_logger().info(status_msg)
        else:
            self.get_logger().warn(status_msg)

def main(args=None):
    rclpy.init(args=args)
    
    checker = PicoXRConnectionChecker()
    
    try:
        rclpy.spin(checker)
    except KeyboardInterrupt:
        pass
    finally:
        checker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
