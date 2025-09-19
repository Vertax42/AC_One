import os
import threading
import collections
import cv2

import numpy as np
import matplotlib.pyplot as plt

from collections import deque

from scipy.spatial.transform import Rotation as R  # eef:ZXY

import rclpy

from rclpy.node import Node

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage, Imu
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Int32MultiArray

from utils.controller import PIDController

import time


class Rate:
    def __init__(self, hz):
        self.period = 1.0 / hz
        self.last_time = time.time()

    def sleep(self):
        now = time.time()
        elapsed = now - self.last_time
        sleep_time = self.period - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)
        self.last_time = time.time()


class RosOperator(Node):
    def __init__(self, args, config, in_collect=False):
        super().__init__("robot_operator")

        # from arm_control.msg._pos_cmd import PosCmd
        # from arm_control.msg._joint_control import JointControl
        from arx5_arm_msg.msg._robot_cmd import RobotCmd
        from arx5_arm_msg.msg._robot_status import RobotStatus

        self.args = args
        self.config = config

        self.in_collect = in_collect

        self.ctrl_state = False
        self.ctrl_state_lock = threading.Lock()

        self.bridge = CvBridge()

        self.img_head_deque = deque()
        self.img_left_deque = deque()
        self.img_right_deque = deque()

        self.img_head_depth_deque = deque()
        self.img_left_depth_deque = deque()
        self.img_right_depth_deque = deque()

        self.controller_left_deque = deque()
        self.controller_right_deque = deque()
        self.feedback_left_arm_deque = deque()
        self.feedback_right_arm_deque = deque()

        self.follow_arm_publish_lock = threading.Lock()
        self.follow_arm_publish_lock.acquire()

        self.last_joy = [0, 0, 0, 0]
        self.triggered_joys = {}
        self.joy_lock = threading.Lock()

        # self.pos_cmd = PosCmd
        # self.joint_control = JointControl
        self.robot_cmd = RobotCmd
        self.robot_status = RobotStatus

        # 摄像头订阅
        img_topics = {
            "img_head": "img_head_topic",
            "img_left": "img_left_topic",
            "img_right": "img_right_topic",
        }
        for key, topic in img_topics.items():
            try:
                self.create_subscription(
                    CompressedImage,
                    self.config["camera_config"][topic],
                    getattr(self, f"{key}_callback"),
                    2,
                )
            except KeyError as e:
                self.get_logger().error(f"Topic config missing: {e}")
            except AttributeError as e:
                self.get_logger().error(f"Callback not found for key: {key} -> {e}")

        if self.args.use_depth_image:
            depth_img_topics = {
                "img_head_depth": "img_head_depth_topic",
                "img_left_depth": "img_left_depth_topic",
                "img_right_depth": "img_right_depth_topic",
            }
            for key, topic in depth_img_topics.items():
                try:
                    self.create_subscription(
                        CompressedImage,
                        self.config["camera_config"][topic],
                        getattr(self, f"{key}_callback"),
                        2,
                    )
                except KeyError as e:
                    self.get_logger().error(f"Topic config missing: {e}")
                except AttributeError as e:
                    self.get_logger().error(f"Callback not found for key: {key} -> {e}")

        # arm topics
        arm_topics = {
            "feedback_left": ("feedback_left_topic", self.robot_status),
            "feedback_right": ("feedback_right_topic", self.robot_status),
        }

        if self.in_collect:
            arm_topics.update(
                {
                    "controller_left": ("controller_left_topic", self.robot_status),
                    "controller_right": ("controller_right_topic", self.robot_status),
                }
            )

        for key, (topic_key, msg_type) in arm_topics.items():
            try:
                self.create_subscription(
                    msg_type,
                    self.config["arm_config"][topic_key],
                    getattr(self, f"{key}_callback"),
                    2,
                )
            except KeyError as e:
                self.get_logger().error(f"Topic config missing: {e}")
            except AttributeError as e:
                self.get_logger().error(f"Callback not found for key: {key} -> {e}")

        # joystick subscription
        self.create_subscription(
            Int32MultiArray,
            self.config["joy_config"]["joy_topic"],
            self.joy_callback,
            2,
        )

        # inference related publisher
        if not self.in_collect:
            self.controller_arm_left_publisher = self.create_publisher(
                self.robot_status,
                self.config["arm_config"]["controller_left_topic"],
                10,
            )
            self.controller_arm_right_publisher = self.create_publisher(
                self.robot_status,
                self.config["arm_config"]["controller_right_topic"],
                10,
            )

    # inference publish function
    def follow_arm_publish(self, left, right):
        if len(left) == 7 and len(right) == 7:
            joint_state_msg_left = self.robot_status()
            joint_state_msg_right = self.robot_status()
        else:
            print("\033[31mERROR action\033[0m")

            return

        joint_state_msg_left.joint_pos = left.astype(np.float64)
        self.controller_arm_left_publisher.publish(
            joint_state_msg_left
        )  # /left_joint_control

        if len(right) != 0:
            joint_state_msg_right.joint_pos = right.astype(np.float64)
            self.controller_arm_right_publisher.publish(
                joint_state_msg_right
            )  # /right_joint_control

    def visualize_pid_base(self, states, target, plot_path=None):
        STATE_NAMES = ["DX", "DY", "Yaw"]
        label1, label2 = "states", "target"
        states = np.array(states)
        target = np.array(target)

        num_ts, num_dim = states.shape
        fig, axs = plt.subplots(num_dim, 1, figsize=(8, 2 * num_dim))

        all_names = [f"{name}_left" for name in STATE_NAMES] + [
            f"{name}_right" for name in STATE_NAMES
        ]

        for dim_idx, ax in enumerate(axs):
            ax.plot(states[:, dim_idx], label=label1, color="orangered")
            ax.plot(target[:, dim_idx], label=label2)
            ax.set_title(f"Joint {dim_idx}: {all_names[dim_idx]}")
            ax.legend()

        plt.tight_layout()
        if plot_path:
            plt.savefig(plot_path)
            print(f"Saved pid control plot to: {plot_path}")
        else:
            plt.show()

        plt.close()

    def follow_arm_publish_continuous(
        self, left_target, right_target, arm_steps_length=None, max_steps=1000
    ):
        """
        连续发布机械臂控制命令，平滑移动到目标位置

        Args:
            left_target: 左臂目标位置
            right_target: 右臂目标位置
            arm_steps_length: 每关节步长，默认使用预设值
            max_steps: 最大步数，防止无限循环
        """
        if arm_steps_length is None:
            arm_steps_length = [0.01, 0.01, 0.005, 0.01, 0.01, 0.01, 0.02]

        # check data dim
        if len(left_target) != 7 or len(right_target) != 7:
            self.get_logger().error(
                f"Invalid joint length: left={len(left_target)}, right={len(right_target)}"
            )
            return False

        if len(arm_steps_length) != 7:
            self.get_logger().error(f"Invalid steps length: {len(arm_steps_length)}")
            return False

        left_arm = None
        right_arm = None
        rate = self.create_rate(self.args.frame_rate)

        while rclpy.ok():
            if len(self.feedback_left_arm_deque) != 0:
                left_arm = list(self.feedback_left_arm_deque[-1].joint_pos)

            if len(self.feedback_right_arm_deque) != 0:
                right_arm = list(self.feedback_right_arm_deque[-1].joint_pos)

            if left_arm is not None and right_arm is not None:
                break

        # direction flags
        left_symbol = np.sign(np.array(left_target) - np.array(left_arm))
        right_symbol = np.sign(np.array(right_target) - np.array(right_arm))

        # init ros2 message
        left_joint_state_msg = self.robot_status()
        right_joint_state_msg = self.robot_status()

        step = 0
        while rclpy.ok() and step < max_steps:
            left_done = self._update_arm_position(
                left_target, left_arm, left_symbol, arm_steps_length
            )
            right_done = self._update_arm_position(
                right_target, right_arm, right_symbol, arm_steps_length
            )

            # check if reach dest
            if left_done >= len(left_target) and right_done >= len(right_target):
                self.get_logger().info(f"Reached target positions in {step} steps")
                break

            # update message
            left_joint_state_msg.joint_pos = np.asarray(left_arm, dtype=np.float64)
            right_joint_state_msg.joint_pos = np.asarray(right_arm, dtype=np.float64)

            # publish control message
            self.controller_arm_left_publisher.publish(left_joint_state_msg)
            self.controller_arm_right_publisher.publish(right_joint_state_msg)

            step += 1

            # print debug info
            if step % 20 == 0:
                self.get_logger().info(
                    f"Step {step} - Left joints: {[f'{x:.3f}' for x in left_arm]}, Right joints: {[f'{x:.3f}' for x in right_arm]}"
                )

            rate.sleep()

        if step >= max_steps:
            self.get_logger().warn(
                f"Reached maximum steps ({max_steps}), movement may be incomplete"
            )

        return True

    def _extract_eef_data(self, eef):
        return [eef.x, eef.y, eef.z, eef.roll, eef.pitch, eef.yaw]

    def get_observation(self, ts=-1):  # get the robot observation
        img_data = {
            "head": None,
            "left_wrist": None,
            "right_wrist": None,
        }
        img_depth_data = {
            "head": None,
            "left_wrist": None,
            "right_wrist": None,
        }
        arm_data = {
            "left_arm": self.robot_status(),
            "right_arm": self.robot_status(),
        }

        # get image data
        for cam_name in self.args.camera_names:
            if cam_name in img_data:
                deque_map = {
                    "head": self.img_head_deque,
                    "left_wrist": self.img_left_deque,
                    "right_wrist": self.img_right_deque,
                }

                if len(deque_map[cam_name]) == 0:
                    print(f"there is no {cam_name}_deque")

                    return None

                # image process
                img_data[cam_name] = self.bridge.compressed_imgmsg_to_cv2(
                    deque_map[cam_name].pop(), "passthrough"
                )

            if self.args.use_depth_image:
                if cam_name in img_depth_data:
                    deque_map = {
                        "head_depth": self.img_head_depth_deque,
                        "left_wrist_depth": self.img_left_depth_deque,
                        "right_wrist_depth": self.img_right_depth_deque,
                    }

                    key = cam_name + "_depth"

                    if len(deque_map[key]) == 0:
                        print(f"there is no {key}_deque")

                        return None

                    img_depth_data[key] = self.bridge.imgmsg_to_cv2(
                        deque_map[key].pop(), "passthrough"
                    )

        # get robot arm states
        for arm_name in ["left_arm", "right_arm"]:
            deque_map = {
                "left_arm": self.feedback_left_arm_deque,
                "right_arm": self.feedback_right_arm_deque,
            }

            if len(deque_map[arm_name]) == 0:
                print(f"there is no {arm_name}_deque")

                return None

            arm_data[arm_name] = deque_map[arm_name].pop()

        obs_dict = collections.OrderedDict()  # orderedDict

        # save images
        obs_dict["images"] = {
            cam: img for cam, img in img_data.items() if cam in self.args.camera_names
        }

        if self.args.use_depth_image:
            obs_dict["images_depth"] = {
                cam: img_depth_data[cam]
                for cam in img_depth_data
                if cam in self.args.camera_names
            }

        # save arm states
        left_eef = np.concatenate(
            [
                arm_data["left_arm"].end_pos,
                [arm_data["left_arm"].joint_pos[-1]],
            ]
        )

        right_eef = np.concatenate(
            [arm_data["right_arm"].end_pos, [arm_data["right_arm"].joint_pos[-1]]]
        )

        obs_dict["eef"] = np.concatenate((left_eef, right_eef), axis=0)
        obs_dict["qpos"] = np.concatenate(
            (
                np.array(arm_data["left_arm"].joint_pos),
                np.array(arm_data["right_arm"].joint_pos),
            ),
            axis=0,
        )
        obs_dict["qvel"] = np.concatenate(
            (
                np.array(arm_data["left_arm"].joint_vel),
                np.array(arm_data["right_arm"].joint_vel),
            ),
            axis=0,
        )
        obs_dict["effort"] = np.concatenate(
            (
                np.array(arm_data["left_arm"].joint_cur),
                np.array(arm_data["right_arm"].joint_cur),
            ),
            axis=0,
        )

        return obs_dict

    def get_action(self):
        joints_dim = 7

        action_dict = collections.OrderedDict()

        deque_map = {
            "control_left_arm_deque": self.controller_left_deque,
            "control_right_arm_deque": self.controller_right_deque,
        }

        for name, deque in deque_map.items():
            if len(deque) == 0:
                print(f"there is no {name}")

                return None

        # get arm frame
        left_frame = deque_map["control_left_arm_deque"].pop()
        right_frame = deque_map["control_right_arm_deque"].pop()

        control_left_arm = left_frame.end_pos
        control_right_arm = right_frame.end_pos
        control_left_arm_gripper = left_frame.joint_pos[-1]
        control_right_arm_gripper = right_frame.joint_pos[-1]

        # concat arm eef pos
        control_left_arm_eef = np.concatenate(
            [control_left_arm, [control_left_arm_gripper]]
        )
        control_right_arm_eef = np.concatenate(
            [control_right_arm, [control_right_arm_gripper]]
        )

        # create action dict
        action_dict["action"] = np.zeros((joints_dim * 2,))
        action_dict["action_qvel"] = np.zeros((joints_dim * 2,))
        action_dict["action_eef"] = np.concatenate(
            (control_left_arm_eef, control_right_arm_eef), axis=0
        )

        return action_dict

    def img_head_callback(self, msg):
        if len(self.img_head_deque) >= 2000:
            self.img_head_deque.popleft()
        self.img_head_deque.append(msg)

    def img_left_callback(self, msg):
        if len(self.img_left_deque) >= 2000:
            self.img_left_deque.popleft()
        self.img_left_deque.append(msg)

    def img_right_callback(self, msg):
        if len(self.img_right_deque) >= 2000:
            self.img_right_deque.popleft()
        self.img_right_deque.append(msg)

    def img_head_depth_callback(self, msg):
        if len(self.img_head_depth_deque) >= 2000:
            self.img_head_depth_deque.popleft()
        self.img_head_depth_deque.append(msg)

    def img_left_depth_callback(self, msg):
        if len(self.img_left_depth_deque) >= 2000:
            self.img_left_depth_deque.popleft()
        self.img_left_depth_deque.append(msg)

    def img_right_depth_callback(self, msg):
        if len(self.img_right_depth_deque) >= 2000:
            self.img_right_depth_deque.popleft()
        self.img_right_depth_deque.append(msg)

    def controller_left_callback(self, msg):
        if len(self.controller_left_deque) >= 2000:
            self.controller_left_deque.popleft()
        self.controller_left_deque.append(msg)
        self.feedback_left_arm_deque.append(msg)

    def controller_right_callback(self, msg):
        if len(self.controller_right_deque) >= 2000:
            self.controller_right_deque.popleft()
        self.controller_right_deque.append(msg)
        self.feedback_right_arm_deque.append(msg)

    def feedback_left_callback(self, msg):
        if len(self.feedback_left_arm_deque) >= 2000:
            self.feedback_left_arm_deque.popleft()
        self.feedback_left_arm_deque.append(msg)

    def feedback_right_callback(self, msg):
        if len(self.feedback_right_arm_deque) >= 2000:
            self.feedback_right_arm_deque.popleft()
        self.feedback_right_arm_deque.append(msg)

    def joy_callback(self, msg):
        joy = list(msg.data)

        with self.joy_lock:
            for i in range(4):
                if self.last_joy[i] == 0 and joy[i] == 1:
                    self.triggered_joys[i] = joy.copy()

            self.last_joy = joy

    def _update_arm_position(self, target, arm, symbol, steps_length):
        diff = [abs(target[i] - arm[i]) for i in range(len(target))]
        done = 0
        for i in range(len(target)):
            if diff[i] < steps_length[i]:
                arm[i] = target[i]
                done += 1
            else:
                arm[i] += symbol[i] * steps_length[i]

        return done
