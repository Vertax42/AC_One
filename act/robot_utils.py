import numpy as np
import time


def linear_func(t):
    """线形插值"""
    return t


def easeInOutQuad(t):
    """二次贝塞尔插值"""
    t *= 2
    if t < 1:
        return t * t / 2
    else:
        t -= 1
        return -(t * (t - 2) - 1) / 2


def easeInOutCubic(t):
    """三次贝塞尔缓动函数"""
    if t < 0.5:
        return 4 * t * t * t
    else:
        t = 2 * t - 2
        return (t * t * t + 2) / 2


def easeInSine(t):
    """正弦缓动函数 (ease-in)
    运动曲线：慢-快-慢，适合需要缓慢启动的场景, 如机械臂启动
    """
    return 1 - np.cos(t * np.pi / 2)


def easeOutSine(t):
    """正弦缓动函数 (ease-out)
    运动曲线：快-快-慢，适合需要平稳停止的场景, 如机械臂停止
    """
    return np.sin(t * np.pi / 2)


def easeInOutSine(t):
    """正弦缓动函数 (ease-in-out)
    运动曲线：慢-快-慢，适合需要平滑运动的场景
    """
    return -(np.cos(np.pi * t) - 1) / 2


def smooth_joint_interpolation(
    controller,
    target_poses,
    duration,
    control_dt,
    gripper_target=0.0,
    interpolation_func=easeInOutQuad,
    print_interval=100,
):
    """
    平滑关节插值函数

    参数:
    - controller: Arx5JointController 实例
    - target_poses: 目标关节位置 (numpy array, 6个关节)
    - duration: 插值持续时间 (秒)
    - control_dt: 控制周期 (秒)
    - gripper_target: 目标夹爪位置 (米)
    - interpolation_func: 插值函数 (默认 easeInOutQuad)
    - print_interval: 打印间隔 (步数)

    返回:
    - 最终关节状态
    """
    # 计算步数
    step_num = int(duration / control_dt)

    # 获取初始状态
    initial_state = controller.get_joint_state()
    initial_poses = initial_state.pos().copy()

    print(f"开始平滑插值: {step_num} 步, 持续时间: {duration:.3f}s")
    print(f"初始位置: {initial_poses[:3]}")
    print(f"目标位置: {target_poses[:3]}")

    # 执行插值
    for i in range(step_num):
        # 计算插值参数 (0 到 1)
        t = float(i) / (step_num - 1) if step_num > 1 else 0.0
        alpha = interpolation_func(t)

        # 计算当前目标位置
        cmd = arx5.JointState(controller.get_robot_config().joint_dof)
        cmd.pos()[:] = initial_poses + alpha * (target_poses - initial_poses)
        cmd.gripper_pos = initial_state.gripper_pos + alpha * (
            gripper_target - initial_state.gripper_pos
        )

        # 设置命令
        controller.set_joint_cmd(cmd)

        # 通信
        if not controller.get_controller_config().background_send_recv:
            controller.send_recv_once()
        else:
            time.sleep(control_dt)

        # 获取当前状态
        current_state = controller.get_joint_state()

        # 定期打印状态
        if i % print_interval == 0:
            print(
                f"步骤 {i:4d}: 目标={cmd.pos()[:3]}, 实际={current_state.pos()[:3]}, "
                f"速度={current_state.vel()[:3]}"
            )

    # 返回最终状态
    final_state = controller.get_joint_state()
    print(f"插值完成: 最终位置={final_state.pos()[:3]}")
    return final_state


def create_custom_interpolation(
    controller,
    target_poses,
    duration,
    control_dt,
    gripper_target=0.0,
    interpolation_func=easeInOutQuad,
    print_interval=100,
):
    """
    创建自定义插值函数的便捷方法

    示例用法:
    # 使用三次贝塞尔插值
    create_custom_interpolation(controller, poses, 3.0, dt, interpolation_func=easeInOutCubic)

    # 使用正弦插值
    create_custom_interpolation(controller, poses, 3.0, dt, interpolation_func=easeInOutSine)
    """
    return smooth_joint_interpolation(
        controller,
        target_poses,
        duration,
        control_dt,
        gripper_target,
        interpolation_func,
        print_interval,
    )
