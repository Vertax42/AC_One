#!/usr/bin/env python3
"""
关节夹爪控制测试程序
尝试通过关节位置控制夹爪
"""

from bimanual import SingleArm
import time


def main():
    print("=== 关节夹爪控制测试程序 ===")

    # 机械臂配置
    arm_config = {"can_port": "can1", "type": 2, "dt": 0.005, "num_joints": 7}

    try:
        print("正在初始化机械臂...")
        arm = SingleArm(arm_config)
        print("✓ 机械臂初始化成功")

        # 显示初始状态
        print("\n=== 初始状态 ===")
        joint_pos = arm.get_joint_positions()
        print(f"当前关节位置: {[f'{p:.3f}' for p in joint_pos]}")
        if len(joint_pos) >= 7:
            print(f"当前夹爪位置: {joint_pos[6]:.3f}")

        print("\n开始关节夹爪测试...")
        print("注意: 请观察夹爪是否有动作")

        # 等待用户确认
        input("按回车键开始测试...")

        # 测试1: 通过关节位置控制夹爪
        print("\n=== 测试1: 关节位置控制夹爪 ===")

        # 获取当前关节位置
        current_joint_pos = arm.get_joint_positions()
        if len(current_joint_pos) < 7:
            print("无法获取完整的关节位置")
            return

        # 测试不同的夹爪位置
        test_catch_positions = [0.0, 0.1, 0.2, 0.0]

        for i, catch_pos in enumerate(test_catch_positions):
            print(f"\n--- 步骤 {i+1}: 设置夹爪关节位置 {catch_pos} ---")

            # 创建新的关节位置数组，只修改第7个关节（夹爪）
            new_joint_pos = current_joint_pos.copy()
            new_joint_pos[6] = catch_pos

            print(f"设置前夹爪位置: {current_joint_pos[6]:.3f}")
            print(f"设置前关节位置: {[f'{p:.3f}' for p in current_joint_pos]}")

            # 设置关节位置
            arm.set_joint_positions(new_joint_pos)
            arm.set_arm_status(5)  # POSITION_CONTROL状态

            time.sleep(2)  # 等待动作完成

            # 检查结果
            new_joint_pos_actual = arm.get_joint_positions()
            print(f"设置后夹爪位置: {new_joint_pos_actual[6]:.3f}")
            print(f"设置后关节位置: {[f'{p:.3f}' for p in new_joint_pos_actual]}")

            print("请观察夹爪是否移动，然后按回车继续...")
            input()

            # 更新当前位置
            current_joint_pos = new_joint_pos_actual

        # 测试2: 使用set_catch函数
        print("\n=== 测试2: 使用set_catch函数 ===")

        for i, catch_pos in enumerate(test_catch_positions):
            print(f"\n--- 步骤 {i+1}: 使用set_catch设置位置 {catch_pos} ---")

            print(f"设置前夹爪位置: {arm.get_catch_pos():.3f}")

            # 使用set_catch函数
            arm.set_catch_pos(catch_pos)

            time.sleep(2)  # 等待动作完成

            print(f"设置后夹爪位置: {arm.get_catch_pos():.3f}")

            print("请观察夹爪是否移动，然后按回车继续...")
            input()

        print("\n=== 测试完成 ===")
        print("如果夹爪仍然没有动作，可能的原因:")
        print("1. 夹爪硬件未连接")
        print("2. 夹爪控制需要特定的机械臂状态")
        print("3. 夹爪位置范围不正确")
        print("4. 夹爪控制方式不正确")

    except Exception as e:
        print(f"测试过程中出错: {e}")
        import traceback

        traceback.print_exc()


if __name__ == "__main__":
    main()
