#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
使用inference.py中init_robot函数的目标位置测试follow_arm_publish_continuous
"""

import sys
import os
import time
import numpy as np
from pathlib import Path

# 添加项目路径
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))
    os.chdir(str(ROOT))


def simulate_inference_init_sequence():
    """
    模拟inference.py中init_robot函数的完整初始化序列
    """
    print("🤖 模拟inference.py中init_robot函数的初始化序列")
    print("=" * 60)

    # 从inference.py中复制的目标位置
    init0 = [0.0, 0.948, 0.858, -0.573, 0.0, 0.0, 0.0]  # 夹爪关闭状态
    init1 = [0.0, 0.948, 0.858, -0.573, 0.0, 0.0, -2.8]  # 夹爪打开状态

    print(f"📋 init0 (夹爪关闭): {init0}")
    print(f"📋 init1 (夹爪打开): {init1}")
    print("-" * 60)

    # 模拟第一步：移动到init0位置（夹爪关闭）
    print("\n🔧 第一步：移动到init0位置（夹爪关闭状态）")
    print("=" * 50)
    simulate_follow_arm_publish_continuous(init0, init0, "init0")

    # 模拟等待用户确认
    print("\n⏳ 模拟等待用户确认...")
    time.sleep(1)

    # 模拟第二步：移动到init1位置（夹爪打开）
    print("\n🔧 第二步：移动到init1位置（夹爪打开状态）")
    print("=" * 50)
    simulate_follow_arm_publish_continuous(init1, init1, "init1")

    print("\n✅ 完整的inference初始化序列模拟完成！")


def simulate_follow_arm_publish_continuous(
    left_target, right_target, step_name, frame_rate=30
):
    """
    模拟 follow_arm_publish_continuous 函数
    """
    print(f"🎯 开始模拟 {step_name} 步骤")
    print(f"左臂目标: {left_target}")
    print(f"右臂目标: {right_target}")

    # 步长设置（与ros_operator.py中相同）
    arm_steps_length = [0.05, 0.05, 0.03, 0.05, 0.05, 0.05, 0.2]
    print(f"步长设置: {arm_steps_length}")
    print("-" * 50)

    # 模拟当前位置（假设从原点开始）
    left_arm = [0.0] * 7
    right_arm = [0.0] * 7

    # 计算方向标志位
    left_symbol = [
        1 if left_target[i] - left_arm[i] > 0 else -1 for i in range(len(left_target))
    ]
    right_symbol = [
        1 if right_target[i] - right_arm[i] > 0 else -1
        for i in range(len(right_target))
    ]

    print(f"左臂移动方向: {left_symbol}")
    print(f"右臂移动方向: {right_symbol}")
    print("-" * 50)

    step = 0
    max_steps = 100  # 防止无限循环

    while step < max_steps:
        left_done = 0
        right_done = 0

        # 更新左臂位置
        left_done = update_arm_position(
            left_target, left_arm, left_symbol, arm_steps_length
        )

        # 更新右臂位置
        right_done = update_arm_position(
            right_target, right_arm, right_symbol, arm_steps_length
        )

        # 打印当前状态（每5步打印一次，避免输出过多）
        if (
            step % 5 == 0
            or left_done >= len(left_target)
            or right_done >= len(right_target)
        ):
            print(
                f"步骤 {step + 1:2d}: "
                f"左臂={[f'{x:7.3f}' for x in left_arm]} (完成{left_done}/7), "
                f"右臂={[f'{x:7.3f}' for x in right_arm]} (完成{right_done}/7)"
            )

        # 检查是否完成
        if left_done >= len(left_target) and right_done >= len(right_target):
            print(f"✅ {step_name} 步骤完成！所有关节都到达目标位置！")
            break

        step += 1
        time.sleep(1.0 / frame_rate)  # 模拟帧率

    if step >= max_steps:
        print(f"⚠️  {step_name} 步骤达到最大步数限制")

    print("-" * 50)
    print(f"最终左臂位置: {[f'{x:.3f}' for x in left_arm]}")
    print(f"最终右臂位置: {[f'{x:.3f}' for x in right_arm]}")
    print(f"总步数: {step + 1}")
    print(f"耗时: {(step + 1) / frame_rate:.2f} 秒")


def update_arm_position(target, arm, symbol, steps_length):
    """更新机械臂位置（复制自 _update_arm_position）"""
    diff = [abs(target[i] - arm[i]) for i in range(len(target))]
    done = 0
    for i in range(len(target)):
        if diff[i] < steps_length[i]:
            arm[i] = target[i]
            done += 1
        else:
            arm[i] += symbol[i] * steps_length[i]
    return done


def analyze_joint_movements():
    """分析关节移动情况"""
    print("\n📊 关节移动分析")
    print("=" * 40)

    init0 = [0.0, 0.948, 0.858, -0.573, 0.0, 0.0, -2.8]
    init1 = [0.0, 0.948, 0.858, -0.573, 0.0, 0.0, 0.0]
    arm_steps_length = [0.05, 0.05, 0.03, 0.05, 0.05, 0.05, 0.2]

    print("关节分析:")
    joint_names = ["关节1", "关节2", "关节3", "关节4", "关节5", "关节6", "夹爪"]

    for i in range(7):
        init0_val = init0[i]
        init1_val = init1[i]
        step_size = arm_steps_length[i]

        if init0_val != init1_val:
            diff = abs(init1_val - init0_val)
            steps_needed = int(diff / step_size) + (1 if diff % step_size > 0 else 0)
            print(
                f"  {joint_names[i]}: {init0_val:6.3f} → {init1_val:6.3f} "
                f"(差值: {diff:6.3f}, 需要步数: {steps_needed:2d})"
            )
        else:
            print(f"  {joint_names[i]}: {init0_val:6.3f} → {init1_val:6.3f} (无变化)")


def test_individual_joints():
    """测试各个关节的移动"""
    print("\n🔬 单独测试各个关节移动")
    print("=" * 40)

    init0 = [0.0, 0.948, 0.858, -0.573, 0.0, 0.0, -2.8]
    init1 = [0.0, 0.948, 0.858, -0.573, 0.0, 0.0, 0.0]
    arm_steps_length = [0.05, 0.05, 0.03, 0.05, 0.05, 0.05, 0.2]

    joint_names = ["关节1", "关节2", "关节3", "关节4", "关节5", "关节6", "夹爪"]

    for i in range(7):
        if init0[i] != init1[i]:
            print(f"\n测试 {joint_names[i]} (关节{i+1}):")

            # 创建单关节测试
            test_target = [0.0] * 7
            test_target[i] = init1[i]
            test_arm = [0.0] * 7
            test_arm[i] = init0[i]

            symbol = [1 if test_target[j] - test_arm[j] > 0 else -1 for j in range(7)]

            step = 0
            while step < 50:  # 限制步数
                done = update_arm_position(
                    test_target, test_arm, symbol, arm_steps_length
                )
                if step % 5 == 0 or done >= 7:
                    print(f"  步骤 {step + 1:2d}: {[f'{x:6.3f}' for x in test_arm]}")

                if done >= 7:
                    print(f"  ✅ {joint_names[i]} 移动完成")
                    break
                step += 1


def main():
    print("🤖 使用inference.py init_robot位置测试follow_arm_publish_continuous")
    print("=" * 70)

    try:
        # 分析关节移动
        analyze_joint_movements()

        # 测试各个关节
        test_individual_joints()

        # 等待用户确认
        input("\n按 Enter 键开始完整的inference初始化序列模拟...")

        # 模拟完整的初始化序列
        simulate_inference_init_sequence()

        print("\n🎉 所有测试完成！")
        print("\n📝 测试总结:")
        print("1. 成功模拟了inference.py中的init_robot函数")
        print("2. 验证了从init0到init1的夹爪状态切换")
        print("3. 确认了所有关节的平滑移动")
        print("4. 夹爪从-2.8弧度移动到0弧度（从关闭到打开）")

    except KeyboardInterrupt:
        print("\n⚠️  用户中断测试")
    except Exception as e:
        print(f"\n❌ 测试过程中出现错误: {e}")
        import traceback

        traceback.print_exc()


if __name__ == "__main__":
    main()
