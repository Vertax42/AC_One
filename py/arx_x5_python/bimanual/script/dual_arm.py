from typing import List, Tuple, Union, Optional, Dict, Any
import numpy as np
from .single_arm import SingleArm


class BimanualArm:
    """
    Class for bimanual robot arm teleoperation. We assume the base/root link of the URDF model
    is in the center of shoulder (the midpoint between the bases of the two arms).
    x-axis pointing forward, y-axis pointing right, z-axis pointing down.

    Args:
        left_arm_config (Dict[str, Any]): Configuration dictionary for the left arm
        right_arm_config (Dict[str, Any]): Configuration dictionary for the right arm

    Attributes:
        left_arm (SingleArm): Left arm instance
        right_arm (SingleArm): Right arm instance
    """

    def __init__(
        self, left_arm_config: Dict[str, Any], right_arm_config: Dict[str, Any]
    ):
        self.left_arm = SingleArm(left_arm_config)
        self.right_arm = SingleArm(right_arm_config)

    def go_home(
        self, duration: float = 2.0, frequency: float = 500.0
    ) -> Dict[str, bool]:
        """
        Move both robot arms to pre-defined home poses with smooth interpolation.

        Args:
            duration (float): Duration for smooth movement in seconds (default: 3.0)
            frequency (float): Control frequency in Hz (default: 200.0)

        Returns:
            Dict[str, bool]: Success status for each arm
        """
        print(f"双臂平滑回到初始位置，持续时间: {duration}s")

        # 获取当前关节位置
        current_joint_positions = self.get_joint_positions()

        # 定义初始位置（所有关节为0）
        home_joint_positions = {
            "left": [0.0] * 7,  # 7个关节
            "right": [0.0] * 7,  # 7个关节
        }

        # 平滑插值到初始位置
        self._smooth_joint_movement(
            current_joint_positions, home_joint_positions, duration, frequency
        )

        # 调用原始go_home确保完全到达初始位置
        return {"left": self.left_arm.go_home(), "right": self.right_arm.go_home()}

    def _smooth_joint_movement(
        self,
        start_positions: Dict[str, List[float]],
        target_positions: Dict[str, List[float]],
        duration: float,
        frequency: float,
    ):
        """
        平滑关节运动插值

        Args:
            start_positions: 起始关节位置
            target_positions: 目标关节位置
            duration: 运动持续时间
            frequency: 控制频率
        """
        import time

        dt = 1.0 / frequency
        steps = int(duration * frequency)

        for step in range(steps + 1):
            t = min(step / steps, 1.0)  # 插值因子 0->1

            # 为每个机械臂计算插值位置
            interpolated_positions = {}
            for arm_name in ["left", "right"]:
                if arm_name in start_positions and arm_name in target_positions:
                    start_pos = np.array(start_positions[arm_name])
                    target_pos = np.array(target_positions[arm_name])

                    # 线性插值
                    interp_pos = start_pos + t * (target_pos - start_pos)
                    interpolated_positions[arm_name] = interp_pos.tolist()

            # 发送插值位置
            if interpolated_positions:
                self.set_joint_positions(interpolated_positions)

            # 等待下一个控制周期
            time.sleep(dt)

            # 打印进度
            if step % int(frequency / 10) == 0:  # 每0.1秒打印一次
                print(f"回到初始位置进度: {t*100:.1f}%")

        print("平滑回到初始位置完成!")

    def gravity_compensation(self) -> Dict[str, bool]:
        return {
            "left": self.left_arm.gravity_compensation(),
            "right": self.right_arm.gravity_compensation(),
        }

    def get_joint_names(self, arm: str = "both") -> Dict[str, List[str]]:
        """
        Get the names of all joints for the specified arm(s).

        Args:
            arm (str): Which arm to get joint names for ("left", "right", or "both")

        Returns:
            Dict[str, List[str]]: Dictionary containing joint names for the specified arm(s).
                                  Shape: Dict with keys 'left' and/or 'right',
                                         each with a list of joint names of shape (num_joints,)
        """
        if arm == "both":
            return {
                "left": self.left_arm.get_joint_names(),
                "right": self.right_arm.get_joint_names(),
            }
        elif arm == "left":
            return {"left": self.left_arm.get_joint_names()}
        elif arm == "right":
            return {"right": self.right_arm.get_joint_names()}
        else:
            raise ValueError("Invalid arm specified. Use 'left', 'right', or 'both'.")

    def set_joint_positions(
        self,
        positions: Dict[str, Union[float, List[float], np.ndarray]],
        joint_names: Optional[Dict[str, Union[str, List[str]]]] = None,
        **kwargs,
    ):
        """
        Set the target joint position(s) for the specified arm(s).

        Args:
            positions: Dictionary with keys 'left' and/or 'right', each containing desired joint position(s).
                        Shape of each arm's positions: (num_joints,)
            joint_names: Dictionary with keys 'left' and/or 'right', each containing name(s) of the joint(s) to set position for.
                         Shape of each arm's joint_names: (num_joints,) or single string
            **kwargs: Additional arguments

        """
        if "left" in positions:
            self.left_arm.set_joint_positions(
                positions["left"],
                **kwargs,
            )
        if "right" in positions:
            self.right_arm.set_joint_positions(
                positions["right"],
                **kwargs,
            )

    def get_joint_positions(
        self,
        arm: str = "both",
        joint_names: Optional[Dict[str, Union[str, List[str]]]] = None,
    ) -> Dict[str, np.ndarray]:
        """
        Get the current joint position(s) for the specified arm(s).

        Args:
            arm: Which arm(s) to get joint position(s) for ("left", "right", or "both")
            joint_names: Dictionary with keys 'left' and/or 'right', each containing name(s) of the joint(s) to get velocity for.
                         If None, returns all joint position(s) for the specified arm(s).

        Returns:
            Dict[str, np.ndarray]: Dictionary containing current joint velocity(ies) for the specified arm(s).
                                   Shape of each arm's position(s): (num_requested_joints,)
        """
        result = {}
        if arm in ["left", "both"]:
            result["left"] = self.left_arm.get_joint_positions(
                joint_names.get("left") if joint_names else None
            )
        if arm in ["right", "both"]:
            result["right"] = self.right_arm.get_joint_positions(
                joint_names.get("right") if joint_names else None
            )
        return result

    def get_joint_velocities(
        self,
        arm: str = "both",
        joint_names: Optional[Dict[str, Union[str, List[str]]]] = None,
    ) -> Dict[str, np.ndarray]:
        """
        Get the current joint velocity(ies) for the specified arm(s).

        Args:
            arm: Which arm(s) to get joint velocities for ("left", "right", or "both")
            joint_names: Dictionary with keys 'left' and/or 'right', each containing name(s) of the joint(s) to get velocity for.
                         If None, returns all joint velocities for the specified arm(s).

        Returns:
            Dict[str, np.ndarray]: Dictionary containing current joint velocity(ies) for the specified arm(s).
                                   Shape of each arm's velocities: (num_requested_joints,)
        """
        result = {}
        if arm in ["left", "both"]:
            result["left"] = self.left_arm.get_joint_velocities(
                joint_names.get("left") if joint_names else None
            )
        if arm in ["right", "both"]:
            result["right"] = self.right_arm.get_joint_velocities(
                joint_names.get("right") if joint_names else None
            )
        return result

    def set_ee_pose(self, poses: Dict[str, Tuple[np.ndarray, np.ndarray]]):
        """
        Set the end-effector poses for both arms.

        Args:
            poses: Dict with keys 'left' and 'right', each containing a tuple of (position, orientation)
        """
        if "left" in poses:
            position, orientation = poses["left"]
            self.left_arm.set_ee_pose(position, orientation)

        if "right" in poses:
            position, orientation = poses["right"]
            self.right_arm.set_ee_pose(position, orientation)

    def set_ee_pose_rpy(self, xyzrpy: Dict[str, Tuple[np.ndarray]], **kwargs) -> bool:
        """
        Move the end effector to the given pose.

        Args:
            xyzrpy: Desired position [x, y, z, rol, pitch, yaw]. Shape: (6,)
            **kwargs: Additional arguments

        """
        if "left" in xyzrpy:
            position, orientation = xyzrpy["left"]
            self.left_arm.set_ee_pose_rpy(position, orientation)

        if "right" in xyzrpy:
            position, orientation = xyzrpy["right"]
            self.right_arm.set_ee_pose_rpy(position, orientation)

    def get_ee_pose(
        self, arm: str = "both"
    ) -> Dict[str, Tuple[np.ndarray, np.ndarray]]:
        """
        Get the current end-effector pose(s) for the specified arm(s).

        Args:
            arm (str): Which arm(s) to get the end-effector pose for ("left", "right", or "both")

        Returns:
            Dict[str, Tuple[np.ndarray, np.ndarray]]: Dictionary containing current end-effector pose(s) for the specified arm(s).
                                                      Each pose is a tuple of (position, orientation).
                                                      Position shape: (3,)
                                                      Orientation shape: (4,) (quaternion [x, y, z, w])
        """
        result = {}
        if arm in ["left", "both"]:
            result["left"] = self.left_arm.get_ee_pose()
        if arm in ["right", "both"]:
            result["right"] = self.right_arm.get_ee_pose()
        return result

    def set_catch_pos(self, positions: Dict[str, float]):
        """
        Set the catch position for the specified arm(s).

        Args:
            positions: Dictionary with keys 'left' and/or 'right', each containing catch position (0.0-1.0)
        """
        if "left" in positions:
            self.left_arm.set_catch_pos(positions["left"])
        if "right" in positions:
            self.right_arm.set_catch_pos(positions["right"])

    def get_catch_pos(self, arm: str = "both") -> Dict[str, float]:
        """
        Get the current catch position for the specified arm(s).

        Args:
            arm (str): Which arm(s) to get catch position for ("left", "right", or "both")

        Returns:
            Dict[str, float]: Dictionary containing current catch positions for the specified arm(s).
        """
        result = {}
        if arm in ["left", "both"]:
            result["left"] = self.left_arm.get_catch_pos()
        if arm in ["right", "both"]:
            result["right"] = self.right_arm.get_catch_pos()
        return result
