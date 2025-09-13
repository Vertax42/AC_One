from bimanual import BimanualArm
import numpy as np
from typing import Dict, Any
import time


def test_gripper_control(dual_arm: BimanualArm):
    """测试双臂夹爪控制功能"""
    print("=" * 50)
    print("开始双臂夹爪控制测试")
    print("=" * 50)

    # 根据实际测量值定义夹爪位置
    GRIPPER_OPEN = 0  # 完全张开
    GRIPPER_CLOSE = 1  # 完全关闭

    try:
        # 1. 测试夹爪打开
        print("1. 测试夹爪打开...")
        dual_arm.set_catch_pos({"left": GRIPPER_OPEN, "right": GRIPPER_OPEN})
        time.sleep(1.0)
        current_pos = dual_arm.get_catch_pos()
        print(f"   左臂夹爪位置: {current_pos['left']}")
        print(f"   右臂夹爪位置: {current_pos['right']}")

        # 2. 测试夹爪关闭
        print("2. 测试夹爪关闭...")
        dual_arm.set_catch_pos({"left": GRIPPER_CLOSE, "right": GRIPPER_CLOSE})
        time.sleep(1.0)
        current_pos = dual_arm.get_catch_pos()
        print(f"   左臂夹爪位置: {current_pos['left']}")
        print(f"   右臂夹爪位置: {current_pos['right']}")

        # 3. 测试夹爪中间位置
        print("3. 测试夹爪中间位置...")
        middle_pos = (GRIPPER_OPEN + GRIPPER_CLOSE) / 2  # 中间位置
        dual_arm.set_catch_pos({"left": middle_pos, "right": middle_pos})
        time.sleep(1.0)
        current_pos = dual_arm.get_catch_pos()
        print(f"   左臂夹爪位置: {current_pos['left']}")
        print(f"   右臂夹爪位置: {current_pos['right']}")

        # 4. 测试夹爪独立控制
        print("4. 测试夹爪独立控制...")
        dual_arm.set_catch_pos(
            {"left": GRIPPER_OPEN, "right": GRIPPER_CLOSE}
        )  # 左开右关
        time.sleep(1.0)
        current_pos = dual_arm.get_catch_pos()
        print(f"   左臂夹爪位置: {current_pos['left']}")
        print(f"   右臂夹爪位置: {current_pos['right']}")

        dual_arm.set_catch_pos(
            {"left": GRIPPER_CLOSE, "right": GRIPPER_OPEN}
        )  # 左关右开
        time.sleep(1.0)
        current_pos = dual_arm.get_catch_pos()
        print(f"   左臂夹爪位置: {current_pos['left']}")
        print(f"   右臂夹爪位置: {current_pos['right']}")

        # 5. 测试夹爪渐进控制
        print("5. 测试夹爪渐进控制...")
        # 在完全张开和完全关闭之间创建渐进位置
        positions = []
        for i in range(11):
            ratio = i / 10.0  # 0.0 到 1.0
            pos = GRIPPER_OPEN + ratio * (GRIPPER_CLOSE - GRIPPER_OPEN)
            positions.append(pos)

        for i, pos in enumerate(positions):
            print(f"   设置夹爪位置: {pos:.3f}")
            dual_arm.set_catch_pos({"left": pos, "right": pos})
            time.sleep(0.5)
            current_pos = dual_arm.get_catch_pos()
            print(
                f"   实际夹爪位置 - 左: {current_pos['left']:.3f}, 右: {current_pos['right']:.3f}"
            )

        print("双臂夹爪控制测试完成!")

    except Exception as e:
        print(f"夹爪控制测试失败: {e}")


def test_dual_arm(dual_arm: BimanualArm, duration: float = 10.0, dt: float = 0.01):
    """双臂基础测试"""
    print("=" * 50)
    print("开始双臂基础测试")
    print("=" * 50)

    try:
        # 回零和重力补偿
        print("执行回零...")
        dual_arm.go_home()
        # print("启用重力补偿...")
        # dual_arm.gravity_compensation()

        # 获取关节位置
        print("获取关节位置...")
        current_joint_positions = dual_arm.get_joint_positions()

        # 打印左臂关节位置
        print("左臂关节位置:")
        for i, pos in enumerate(current_joint_positions["left"]):
            print(f"  left_joint{i+1}: {pos}")

        # 打印右臂关节位置
        print("右臂关节位置:")
        for i, pos in enumerate(current_joint_positions["right"]):
            print(f"  right_joint{i+1}: {pos}")

        # 获取夹爪位置
        print("获取夹爪位置...")
        catch_positions = dual_arm.get_catch_pos()
        print(f"左臂夹爪位置: {catch_positions['left']}")
        print(f"右臂夹爪位置: {catch_positions['right']}")

        print("双臂基础测试完成!")

    except Exception as e:
        print(f"双臂测试失败: {e}")


def main():
    """主测试函数"""
    print("双臂夹爪测试程序")
    print("按 Ctrl+C 退出")

    # Define arm configurations
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
        # Create BimanualArm instance
        print("初始化双臂...")
        bimanual_arm = BimanualArm(left_arm_config, right_arm_config)
        print("双臂初始化成功")

        # 先进行基础测试
        test_dual_arm(bimanual_arm)
        time.sleep(1)
        # 再进行夹爪测试
        test_gripper_control(bimanual_arm)

        print("所有测试完成!")

    except Exception as e:
        print(f"测试失败: {e}")
    finally:
        print("程序结束")


if __name__ == "__main__":
    main()
