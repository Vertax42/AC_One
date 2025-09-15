#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
调试版 init_robot 测试脚本
添加详细的调试信息来诊断夹爪移动问题
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

import rclpy
from rclpy.executors import MultiThreadedExecutor
import threading
import yaml
import signal
import numpy as np

from utils.ros_operator import RosOperator
from utils.setup_loader import setup_loader


def debug_follow_arm_publish_continuous(
    ros_operator, left_target, right_target, step_name
):
    """调试版 follow_arm_publish_continuous 函数"""
    print(f"\n🔧 调试 {step_name} 步骤")
    print("=" * 60)

    # 步长设置
    arm_steps_length = [0.05, 0.05, 0.03, 0.05, 0.05, 0.05, 0.2]

    print(f"目标位置 - 左臂: {left_target}")
    print(f"目标位置 - 右臂: {right_target}")
    print(f"步长设置: {arm_steps_length}")

    # 获取当前位置
    print("\n📍 获取当前位置...")
    left_arm = None
    right_arm = None

    # 等待获取位置
    timeout = 5.0
    start_time = time.time()

    while time.time() - start_time < timeout:
        if len(ros_operator.feedback_left_arm_deque) > 0:
            left_arm = list(ros_operator.feedback_left_arm_deque[-1].joint_pos)
        if len(ros_operator.feedback_right_arm_deque) > 0:
            right_arm = list(ros_operator.feedback_right_arm_deque[-1].joint_pos)

        if left_arm is not None and right_arm is not None:
            break

        time.sleep(0.1)

    if left_arm is None or right_arm is None:
        print("❌ 无法获取当前位置")
        return False

    print(f"当前位置 - 左臂: {[f'{x:.3f}' for x in left_arm]}")
    print(f"当前位置 - 右臂: {[f'{x:.3f}' for x in right_arm]}")

    # 计算差值
    left_diff = [abs(left_target[i] - left_arm[i]) for i in range(len(left_target))]
    right_diff = [abs(right_target[i] - right_arm[i]) for i in range(len(right_target))]

    print(f"差值 - 左臂: {[f'{x:.3f}' for x in left_diff]}")
    print(f"差值 - 右臂: {[f'{x:.3f}' for x in right_diff]}")

    # 检查是否需要移动
    left_need_move = any(d > 0.001 for d in left_diff)
    right_need_move = any(d > 0.001 for d in right_diff)

    print(f"左臂需要移动: {left_need_move}")
    print(f"右臂需要移动: {right_need_move}")

    if not left_need_move and not right_need_move:
        print("⚠️  机械臂已经在目标位置，无需移动")
        return True

    # 计算方向
    left_symbol = [
        1 if left_target[i] - left_arm[i] > 0 else -1 for i in range(len(left_target))
    ]
    right_symbol = [
        1 if right_target[i] - right_arm[i] > 0 else -1
        for i in range(len(right_target))
    ]

    print(f"移动方向 - 左臂: {left_symbol}")
    print(f"移动方向 - 右臂: {right_symbol}")

    # 开始移动
    print("\n🔄 开始移动...")
    step = 0
    max_steps = 100

    # 创建rate对象
    rate = ros_operator.create_rate(30)

    while step < max_steps and rclpy.ok():
        left_done = 0
        right_done = 0

        # 更新左臂位置
        left_done = update_arm_position_debug(
            left_target, left_arm, left_symbol, arm_steps_length, "左臂"
        )

        # 更新右臂位置
        right_done = update_arm_position_debug(
            right_target, right_arm, right_symbol, arm_steps_length, "右臂"
        )

        # 每5步打印一次状态
        if (
            step % 5 == 0
            or left_done >= len(left_target)
            or right_done >= len(right_target)
        ):
            print(
                f"步骤 {step + 1:2d}: 左臂={[f'{x:6.3f}' for x in left_arm]} (完成{left_done}/7), "
                f"右臂={[f'{x:6.3f}' for x in right_arm]} (完成{right_done}/7)"
            )

        # 发布位置命令到机械臂
        if len(left_arm) == 7 and len(right_arm) == 7:
            left_joint_state_msg = ros_operator.robot_status()
            right_joint_state_msg = ros_operator.robot_status()

            left_joint_state_msg.joint_pos = np.asarray(left_arm, dtype=np.float64)
            right_joint_state_msg.joint_pos = np.asarray(right_arm, dtype=np.float64)

            ros_operator.controller_arm_left_publisher.publish(left_joint_state_msg)
            ros_operator.controller_arm_right_publisher.publish(right_joint_state_msg)

        # 检查是否完成
        if left_done >= len(left_target) and right_done >= len(right_target):
            print("✅ 所有关节都到达目标位置！")
            break

        step += 1
        rate.sleep()

    if step >= max_steps:
        print("⚠️  达到最大步数限制")

    print(f"最终位置 - 左臂: {[f'{x:.3f}' for x in left_arm]}")
    print(f"最终位置 - 右臂: {[f'{x:.3f}' for x in right_arm]}")
    print(f"总步数: {step + 1}")

    return True


def update_arm_position_debug(target, arm, symbol, steps_length, arm_name):
    """调试版位置更新函数"""
    diff = [abs(target[i] - arm[i]) for i in range(len(target))]
    done = 0

    for i in range(len(target)):
        if diff[i] < steps_length[i]:
            arm[i] = target[i]
            done += 1
        else:
            arm[i] += symbol[i] * steps_length[i]

    return done


def init_robot_debug_test(ros_operator, use_base=False):
    """调试版 init_robot 测试"""
    print("🤖 调试版 init_robot 测试")
    print("=" * 60)

    # 预定义的安全初始位置（关节角度）
    init0 = [0.0, 0.948, 0.858, -0.573, 0.0, 0.0, 0.0]  # 夹爪关闭状态
    init1 = [0.0, 0.948, 0.858, -0.573, 0.0, 0.0, -2.8]  # 夹爪打开状态

    print(f"📋 init0 (夹爪关闭): {init0}")
    print(f"📋 init1 (夹爪打开): {init1}")

    # 第一步：移动到init0位置
    print("\n" + "=" * 60)
    print("🔧 步骤1: 移动到 init0 位置（夹爪关闭）")
    start_time = time.time()

    success = debug_follow_arm_publish_continuous(ros_operator, init0, init0, "init0")

    step1_time = time.time() - start_time
    print(f"✅ 步骤1完成，耗时: {step1_time:.2f} 秒，成功: {success}")

    # 等待用户确认
    print("\n⏳ 等待用户确认...")
    input("按 Enter 键继续到步骤2（夹爪打开）...")

    # 第二步：移动到init1位置
    print("\n" + "=" * 60)
    print("🔧 步骤2: 移动到 init1 位置（夹爪打开）")
    start_time = time.time()

    success = debug_follow_arm_publish_continuous(ros_operator, init1, init1, "init1")

    step2_time = time.time() - start_time
    print(f"✅ 步骤2完成，耗时: {step2_time:.2f} 秒，成功: {success}")

    if use_base:
        ros_operator.start_base_control_thread()

    total_time = step1_time + step2_time
    print(f"\n🎉 init_robot 初始化完成！")
    print(f"⏱️  总耗时: {total_time:.2f} 秒")
    print(f"   步骤1: {step1_time:.2f} 秒")
    print(f"   步骤2: {step2_time:.2f} 秒")


def emergency_safety_return(ros_operator):
    """紧急安全回零功能"""
    try:
        print("\n🔄 开始紧急安全回零...")

        # 获取当前位置
        current_left = None
        current_right = None

        timeout = 2.0
        start_time = time.time()

        while time.time() - start_time < timeout:
            if len(ros_operator.feedback_left_arm_deque) > 0:
                current_left = list(ros_operator.feedback_left_arm_deque[-1].joint_pos)
            if len(ros_operator.feedback_right_arm_deque) > 0:
                current_right = list(
                    ros_operator.feedback_right_arm_deque[-1].joint_pos
                )

            if current_left is not None and current_right is not None:
                break

            time.sleep(0.1)

        if current_left is None or current_right is None:
            print("⚠️  无法获取当前位置，使用直接回零")
            # 直接发布零位命令
            zero_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            left_joint_state_msg = ros_operator.robot_status()
            right_joint_state_msg = ros_operator.robot_status()

            left_joint_state_msg.joint_pos = np.asarray(zero_position, dtype=np.float64)
            right_joint_state_msg.joint_pos = np.asarray(
                zero_position, dtype=np.float64
            )

            ros_operator.controller_arm_left_publisher.publish(left_joint_state_msg)
            ros_operator.controller_arm_right_publisher.publish(right_joint_state_msg)
            return

        print(f"📍 当前位置 - 左臂: {[f'{x:.3f}' for x in current_left]}")
        print(f"📍 当前位置 - 右臂: {[f'{x:.3f}' for x in current_right]}")

        # 零位目标
        zero_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        print(f"🎯 目标位置 - 零位: {zero_position}")

        # 插值回零（3秒内）
        duration = 3.0
        total_steps = int(duration * 30)

        for step in range(total_steps):
            progress = (step + 1) / total_steps

            # 线性插值
            current_left_pos = [
                current_left[i] + (zero_position[i] - current_left[i]) * progress
                for i in range(7)
            ]
            current_right_pos = [
                current_right[i] + (zero_position[i] - current_right[i]) * progress
                for i in range(7)
            ]

            # 发布位置命令
            left_joint_state_msg = ros_operator.robot_status()
            right_joint_state_msg = ros_operator.robot_status()

            left_joint_state_msg.joint_pos = np.asarray(
                current_left_pos, dtype=np.float64
            )
            right_joint_state_msg.joint_pos = np.asarray(
                current_right_pos, dtype=np.float64
            )

            ros_operator.controller_arm_left_publisher.publish(left_joint_state_msg)
            ros_operator.controller_arm_right_publisher.publish(right_joint_state_msg)

            if step % 10 == 0:
                print(
                    f"🔄 回零进度: {progress*100:.1f}% - 左臂: {[f'{x:.3f}' for x in current_left_pos]}"
                )

            time.sleep(1.0 / 30)

        print("✅ 紧急安全回零完成")

    except Exception as e:
        print(f"❌ 紧急安全回零失败: {e}")


def signal_handler(signum, frame, ros_operator):
    """信号处理器"""
    print("\n🚨 紧急停止！接收到 Ctrl+C 信号")
    print("🛡️  启动紧急安全保护：3秒内安全回到零位...")
    emergency_safety_return(ros_operator)
    sys.exit(0)


def main():
    print("🤖 init_robot 调试版实机测试")
    print("=" * 60)
    print("⚠️  警告：此脚本将控制真实机械臂！")
    print("⚠️  请确保机械臂周围安全！")
    print("🛡️  紧急安全保护：按 Ctrl+C 将在3秒内安全回到零位")
    print("=" * 60)

    # 安全确认
    confirm = input("确认开始测试？(y/N): ").strip().lower()
    if confirm != "y":
        print("❌ 测试取消")
        return

    print("\n🚀 初始化ROS环境...")
    setup_loader(ROOT)
    rclpy.init()

    ros_operator = None
    executor = None

    try:
        # 加载配置
        config_path = Path.joinpath(ROOT, "data/config.yaml")
        with open(config_path, "r", encoding="utf-8") as file:
            config = yaml.safe_load(file)

        # 创建模拟的args对象
        class MockArgs:
            def __init__(self):
                self.frame_rate = 30
                self.use_base = False
                self.camera_names = ["head", "left_wrist", "right_wrist"]
                self.use_depth_image = False

        args = MockArgs()

        # 创建RosOperator
        ros_operator = RosOperator(args, config, in_collect=False)

        # 创建执行器
        executor = MultiThreadedExecutor()
        executor.add_node(ros_operator)

        # 启动ROS线程
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        print("✅ ROS环境初始化完成")

        # 等待ROS系统稳定
        print("\n⏳ 等待ROS系统稳定...")
        time.sleep(3)

        # 设置信号处理器
        signal.signal(signal.SIGINT, lambda s, f: signal_handler(s, f, ros_operator))

        # 执行调试版 init_robot 测试
        init_robot_debug_test(ros_operator, args.use_base)

        print("\n🎉 调试测试完成！")

    except KeyboardInterrupt:
        print("\n⚠️  用户中断测试")
        if ros_operator:
            emergency_safety_return(ros_operator)
    except Exception as e:
        print(f"\n❌ 测试过程中出现错误: {e}")
        import traceback

        traceback.print_exc()
    finally:
        print("\n🧹 清理资源...")
        try:
            if executor:
                executor.shutdown()
            rclpy.shutdown()
        except:
            pass
        print("✅ 清理完成")


if __name__ == "__main__":
    main()
