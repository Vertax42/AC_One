from bimanual import SingleArm
from typing import Dict, Any
import numpy as np
import time


def test_gripper_control(single_arm: SingleArm):
    """测试夹爪控制功能"""
    print("=" * 50)
    print("开始夹爪控制测试")
    print("=" * 50)

    try:
        # 1. 测试夹爪打开
        print("1. 测试夹爪打开...")
        single_arm.set_catch_pos(0.0)  # 完全打开
        time.sleep(1.0)
        current_pos = single_arm.get_catch_pos()
        print(f"   当前夹爪位置: {current_pos}")

        # 2. 测试夹爪关闭
        print("2. 测试夹爪关闭...")
        single_arm.set_catch_pos(1.0)  # 完全关闭
        time.sleep(1.0)
        current_pos = single_arm.get_catch_pos()
        print(f"   当前夹爪位置: {current_pos}")

        # 3. 测试夹爪中间位置
        print("3. 测试夹爪中间位置...")
        single_arm.set_catch_pos(0.5)  # 中间位置
        time.sleep(1.0)
        current_pos = single_arm.get_catch_pos()
        print(f"   当前夹爪位置: {current_pos}")

        # 4. 测试夹爪渐进控制
        print("4. 测试夹爪渐进控制...")
        positions = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 0.8, 0.6, 0.4, 0.2, 0.0]
        for i, pos in enumerate(positions):
            print(f"   设置夹爪位置: {pos}")
            single_arm.set_catch_pos(pos)
            time.sleep(0.5)
            current_pos = single_arm.get_catch_pos()
            print(f"   实际夹爪位置: {current_pos}")

        print("夹爪控制测试完成!")

    except Exception as e:
        print(f"夹爪控制测试失败: {e}")


def test_single_arm(single_arm: SingleArm, duration: float = 10.0, dt: float = 0.01):
    """原始单臂测试功能"""
    print("=" * 50)
    print("开始单臂基础测试")
    print("=" * 50)

    try:
        # 测试关节位置控制
        print("测试关节位置控制...")
        positions = [0.5, 0.0, 0.0]  # 指定每个关节的位置
        joint_names = ["joint1", "joint2", "joint3"]  # 对应关节的名称

        success = single_arm.set_joint_positions(
            positions=positions, joint_names=joint_names
        )
        print(f"关节位置设置结果: {success}")

        # 获取当前状态
        print("获取当前状态...")
        ee_pose = single_arm.get_ee_pose()
        joint_positions = single_arm.get_joint_positions()
        catch_pos = single_arm.get_catch_pos()

        print(f"末端执行器位置: {ee_pose[0]}")
        print(f"末端执行器方向: {ee_pose[1]}")
        print(f"关节位置: {joint_positions}")
        print(f"夹爪位置: {catch_pos}")

    except Exception as e:
        print(f"单臂测试失败: {e}")


def main():
    """主测试函数"""
    print("单臂夹爪测试程序")
    print("按 Ctrl+C 退出")

    arm_config: Dict[str, Any] = {
        "can_port": "can1",
        "type": 2,
    }

    try:
        single_arm = SingleArm(arm_config)
        print("单臂初始化成功")

        # 先进行基础测试
        test_single_arm(single_arm)
        time.sleep(1)

        # 再进行夹爪测试
        test_gripper_control(single_arm)

        print("所有测试完成!")

    except Exception as e:
        print(f"测试失败: {e}")
    finally:
        print("程序结束")


if __name__ == "__main__":
    main()
