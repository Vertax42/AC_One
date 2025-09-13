from bimanual import BimanualArm
import numpy as np
from typing import Dict, Any
import time


def test_dual_arm(dual_arm: BimanualArm, duration: float = 10.0, dt: float = 0.01):
    dual_arm.go_home()

    result = dual_arm.gravity_compensation()
    print(f"重力补偿设置结果: {result}")
    while 1:

        # # Example positions and orientations for the arms
        # left_position = np.array([0.0, 0.0, 0.1])  # Example position (x, y, z)
        # left_orientation = np.array(
        #     [1.0, 0.0, 0.0, 0.0]
        # )  # Example orientation as a quaternion
        right_position = dual_arm.right_arm.get_joint_positions()

        left_position = dual_arm.left_arm.get_joint_positions()
        # left_names = dual_arm.left_arm.get_joint_names()
        # right_names = dual_arm.right_arm.get_joint_names()
        print(f"右臂关节此时位置: {right_position}")
        # # print(f"右臂关节此时名称: {right_names}")
        print(f"左臂关节此时位置: {left_position}")
        # print(f"左臂关节此时名称: {left_names}")
        time.sleep(0.01)

        # right_position = np.array([0.0, 0.0, 0.1])  # Example position (x, y, z)
        # right_orientation = np.array(
        #     [1.0, 0.0, 0.0, 0.0]
        # )  # Example orientation as a quaternion

        # poses = {
        #     "left": (left_position, left_orientation),
        #     "right": (right_position, right_orientation),
        # }
        # dual_arm.set_ee_pose(poses)
        # while 1:
        #     print("testing ...")

        # # position = [0.0, 0.1, 0.1]  # x, y, z 位置
        # quaternion = [1.0, 0.0, 0.0, 0.0]  # 四元数表示方向

        # success = dual_arm.set_ee_pose(pos=position, quat=quaternion)

        # print(single_arm.get_ee_pose())
        # print(single_arm.get_joint_positions())

        # positions = [0.5, 1.0, -0.5]  # 指定每个关节的位置
        # joint_names = ["joint1", "joint2", "joint3"]  # 对应关节的名称

        # success = single_arm.set_joint_positions(positions=positions, joint_names=joint_names)


if __name__ == "__main__":
    left_arm_config: Dict[str, Any] = {
        "can_port": "can1",
        "type": 2,
        "dt": 0.005,
    }
    right_arm_config: Dict[str, Any] = {
        "can_port": "can3",
        "type": 2,
        "dt": 0.005,
    }

    try:
        # 创建双臂实例 - 这会初始化CAN通信
        print("正在初始化双臂...")
        bimanual_arm = BimanualArm(left_arm_config, right_arm_config)
        print("双臂初始化成功!")

        # # 测试CAN通信 - 读取状态而不发送控制命令
        # print("测试CAN通信...")
        # joint_positions = bimanual_arm.get_joint_positions()
        # print(f"左臂关节位置: {joint_positions.get('left', 'N/A')}")
        # print(f"右臂关节位置: {joint_positions.get('right', 'N/A')}")

        # # 可选：设置重力补偿模式测试
        # print("设置重力补偿模式...")
        # result = bimanual_arm.gravity_compensation()
        # print(f"重力补偿设置结果: {result}")

        # print("CAN通信测试完成!")

    except Exception as e:
        print(f"CAN通信测试失败: {e}")

    print("测试双臂重力补偿...")
    test_dual_arm(bimanual_arm)
    print("测试双臂完成!")
