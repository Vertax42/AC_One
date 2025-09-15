# -- coding: UTF-8
"""
机器人推理脚本 - 实时控制ARX双臂机器人
本脚本实现了完整的机器人控制流程：观测获取 -> 模型推理 -> 动作执行

主要功能模块：
1. 多进程架构：ROS进程负责数据通信，推理进程负责模型预测
2. 共享内存机制：实现进程间高效数据传递
3. 实时观测获取：从摄像头和关节状态传感器获取数据
4. 神经网络推理：使用训练好的ACT/CNN/Diffusion模型预测动作
5. 动作执行：将预测的动作发送给机器人执行
"""

import os
import sys

# 设置标准输出和标准错误的缓冲模式为行缓冲，确保实时输出
sys.stdout = open(sys.stdout.fileno(), mode="w", buffering=1)
sys.stderr = open(sys.stderr.fileno(), mode="w", buffering=1)

from pathlib import Path

# 设置项目根路径，确保模块导入正确
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))
    os.chdir(str(ROOT))

import argparse
import collections
import pickle
import yaml
from einops import rearrange
import rclpy
import torch
import threading

from rclpy.executors import MultiThreadedExecutor

# 导入策略模型（ACT、CNN-MLP、Diffusion Policy）
from utils.policy import ACTPolicy, CNNMLPPolicy, DiffusionPolicy
from utils.utils import set_seed  # 随机种子设置

# 导入ROS操作类和其他工具
from utils.ros_operator import RosOperator, Rate
from utils.setup_loader import setup_loader
from functools import partial
import signal
import sys

# 全局观测字典，用于存储当前观测数据
obs_dict = collections.OrderedDict()

import multiprocessing as mp
from multiprocessing.shared_memory import SharedMemory

import numpy as np

# 设置numpy输出格式，方便调试
np.set_printoptions(linewidth=200)  # 设置打印输出行宽
np.set_printoptions(suppress=True)  # 禁用科学计数法


def load_yaml(yaml_file):
    """
    加载YAML配置文件

    Args:
        yaml_file (str): YAML文件路径

    Returns:
        dict: 解析后的配置字典，出错时返回None
    """
    try:
        with open(yaml_file, "r", encoding="utf-8") as file:
            return yaml.safe_load(file)
    except FileNotFoundError:
        print(f"Error: File not found - {yaml_file}")

        return None
    except yaml.YAMLError as e:
        print(f"Error: Failed to parse YAML file - {e}")

        return None


def make_shm_name_dict(args, shapes):
    """
    创建共享内存名称字典
    为每个数据类型（图像、状态、动作）分配唯一的共享内存名称

    Args:
        args: 命令行参数，包含摄像头名称等
        shapes: 数据形状字典

    Returns:
        dict: 共享内存名称映射字典
    """
    shm_name_dict = {}
    # 为每个摄像头创建共享内存名称
    for cam in args.camera_names:
        shm_name_dict[cam] = f"shm_img_{cam}"
    # 为每个状态变量创建共享内存名称
    for state_key in shapes["states"]:
        shm_name_dict[state_key] = f"shm_state_{state_key}"
    # 为动作创建共享内存名称
    shm_name_dict["action"] = "shm_action"

    return shm_name_dict


def create_shm_dict(config, shm_name_dict, shapes, dtypes):
    """
    创建共享内存字典
    为图像数据、状态数据和动作数据分配共享内存空间

    Args:
        config: 模型配置
        shm_name_dict: 共享内存名称字典
        shapes: 数据形状字典
        dtypes: 数据类型字典

    Returns:
        dict: 共享内存对象字典，每个条目包含(SharedMemory对象, 形状, 数据类型)
    """
    shm_dict = {}

    # 为图像数据创建共享内存
    for cam, shape in shapes["images"].items():
        size = np.prod(shape) * np.dtype(dtypes[cam]).itemsize
        shm = SharedMemory(name=shm_name_dict[cam], create=True, size=size)
        shm_dict[cam] = (shm, shape, dtypes[cam])

    # 为状态数据创建共享内存
    for state_key, shape in shapes["states"].items():
        size = np.prod(shape) * np.dtype(np.float32).itemsize
        shm = SharedMemory(name=shm_name_dict[state_key], create=True, size=size)
        shm_dict[state_key] = (shm, shape, np.float32)

    # 为动作数据创建共享内存
    action_shape = config["policy_config"]["action_dim"]
    size = np.prod(action_shape) * np.dtype(np.float32).itemsize
    shm = SharedMemory(name=shm_name_dict["action"], create=True, size=size)
    shm_dict["action"] = (shm, action_shape, np.float32)

    return shm_dict


def connect_shm_dict(shm_name_dict, shapes, dtypes, config):
    """
    连接到已存在的共享内存
    推理进程使用此函数连接到ROS进程创建的共享内存

    Args:
        shm_name_dict: 共享内存名称字典
        shapes: 数据形状字典
        dtypes: 数据类型字典
        config: 模型配置

    Returns:
        dict: 共享内存对象字典
    """
    shm_dict = {}

    # 连接图像共享内存
    for cam, shape in shapes["images"].items():
        shm = SharedMemory(name=shm_name_dict[cam], create=False)
        shm_dict[cam] = (shm, shape, dtypes[cam])

    # 连接状态共享内存
    for state_key, shape in shapes["states"].items():
        shm = SharedMemory(name=shm_name_dict[state_key], create=False)
        shm_dict[state_key] = (shm, shape, np.float32)

    # 连接动作共享内存
    action_shape = (config["policy_config"]["action_dim"],)
    shm = SharedMemory(name=shm_name_dict["action"], create=False)
    shm_dict["action"] = (shm, action_shape, np.float32)

    return shm_dict


def robot_action(action, shm_dict):
    """
    将动作写入共享内存
    推理进程使用此函数将预测的动作传递给ROS进程执行

    Args:
        action: 预测的动作数组
        shm_dict: 共享内存字典
    """
    shm, shape, dtype = shm_dict["action"]
    np_array = np.ndarray(shape, dtype=dtype, buffer=shm.buf)
    np_array[:] = action


def get_model_config(args):
    """
    配置模型参数
    根据命令行参数创建完整的模型配置字典

    Args:
        args: 解析后的命令行参数

    Returns:
        dict: 包含模型、训练、推理所需的完整配置
    """
    set_seed(args.seed)

    # 基础配置，所有策略都会使用
    base_config = {
        "lr": args.lr,
        "lr_backbone": args.lr_backbone,
        "weight_decay": args.weight_decay,
        "loss_function": args.loss_function,
        "backbone": args.backbone,
        "chunk_size": args.chunk_size,
        "hidden_dim": args.hidden_dim,
        "camera_names": args.camera_names,
        "position_embedding": args.position_embedding,
        "masks": args.masks,
        "dilation": args.dilation,
        "use_base": args.use_base,
        "use_depth_image": args.use_depth_image,
    }

    # 根据策略类型配置特定参数
    if args.policy_class == "ACT":
        # ACT（Action Chunking with Transformers）策略配置
        policy_config = {
            **base_config,
            "enc_layers": args.enc_layers,  # 编码器层数
            "dec_layers": args.dec_layers,  # 解码器层数
            "nheads": args.nheads,  # 注意力头数
            "dropout": args.dropout,  # Dropout率
            "pre_norm": args.pre_norm,  # 预归一化
            "states_dim": 7,  # 基础状态维度（单臂7个关节）
            "action_dim": 7,  # 基础动作维度（单臂7个关节）
            "kl_weight": args.kl_weight,  # KL散度权重
            "dim_feedforward": args.dim_feedforward,  # 前馈网络维度
            "use_qvel": args.use_qvel,  # 是否使用关节速度
            "use_effort": args.use_effort,  # 是否使用关节力矩
            "use_eef_states": args.use_eef_states,  # 是否使用末端执行器状态
        }

        # 动态更新状态维度
        policy_config["states_dim"] += (
            policy_config["action_dim"] if args.use_qvel else 0
        )
        policy_config["states_dim"] += 1 if args.use_effort else 0
        policy_config["states_dim"] *= 2  # 双臂系统

        # 动态更新动作维度
        policy_config["action_dim"] *= 2  # 双臂预测（左臂+右臂）
        policy_config["action_dim"] += 10 if args.use_base else 0  # 移动底盘
        policy_config["action_dim"] *= 2  # 再次乘2（可能用于不同的动作表示）

        action_dim = policy_config["action_dim"]
        states_dim = policy_config["states_dim"]
        print(f"{action_dim=}", f"{states_dim=}")

    elif args.policy_class == "CNNMLP":
        # CNN-MLP策略配置（简单的卷积+多层感知机）
        policy_config = {
            **base_config,
            "action_dim": 14,  # 固定14维动作
            "states_dim": 14,  # 固定14维状态
        }
    elif args.policy_class == "Diffusion":
        # 扩散模型策略配置
        policy_config = {
            **base_config,
            "observation_horizon": args.observation_horizon,  # 观测历史长度
            "action_horizon": args.action_horizon,  # 动作预测长度
            "num_inference_timesteps": args.num_inference_timesteps,  # 推理时间步数
            "ema_power": args.ema_power,  # 指数移动平均功率
            "action_dim": 14,
            "states_dim": 14,
        }
    else:
        raise NotImplementedError

    # 完整的配置字典
    config = {
        "ckpt_dir": (
            args.ckpt_dir if sys.stdin.isatty() else Path.joinpath(ROOT, args.ckpt_dir)
        ),
        "ckpt_name": args.ckpt_name,  # 模型权重文件名
        "ckpt_stats_name": args.ckpt_stats_name,  # 数据统计文件名
        "episode_len": args.max_publish_step,  # 单回合最大步数
        "state_dim": policy_config["states_dim"],
        "policy_class": args.policy_class,
        "policy_config": policy_config,
        "temporal_agg": args.temporal_agg,  # 时序聚合
        "camera_names": args.camera_names,
    }

    return config


def make_policy(policy_class, policy_config):
    """
    创建策略模型实例

    Args:
        policy_class: 策略类型 ('ACT', 'CNNMLP', 'Diffusion')
        policy_config: 策略配置字典

    Returns:
        策略模型对象
    """
    if policy_class == "ACT":
        policy = ACTPolicy(policy_config)
    elif policy_class == "CNNMLP":
        policy = CNNMLPPolicy(policy_config)
    elif policy_class == "Diffusion":
        policy = DiffusionPolicy(policy_config)
    else:
        raise NotImplementedError

    return policy


def get_image(observation, camera_names):
    """
    处理图像观测数据
    将多个摄像头的图像数据转换为模型输入格式

    Args:
        observation: 观测数据字典
        camera_names: 摄像头名称列表

    Returns:
        torch.Tensor: 归一化后的图像张量 [1, C*N, H, W]
                     C=3为RGB通道数，N为摄像头数量
    """
    curr_images = []
    for cam_name in camera_names:
        # 将图像从 (H, W, C) 转换为 (C, H, W) 格式
        curr_image = rearrange(observation["images"][cam_name], "h w c -> c h w")
        curr_images.append(curr_image)

    # 堆叠所有摄像头图像：[N, C, H, W]
    curr_image = np.stack(curr_images, axis=0)
    # 归一化到[0,1]并转换为GPU张量，添加batch维度：[1, N*C, H, W]
    curr_image = torch.from_numpy(curr_image / 255.0).float().cuda().unsqueeze(0)

    return curr_image


def get_depth_image(observation, camera_names):
    """
    处理深度图像观测数据

    Args:
        observation: 观测数据字典
        camera_names: 摄像头名称列表

    Returns:
        torch.Tensor: 深度图像张量
    """
    curr_images = []
    for cam_name in camera_names:
        curr_images.append(observation["images_depth"][cam_name])
    curr_image = np.stack(curr_images, axis=0)
    curr_image = torch.from_numpy(curr_image / 255.0).float().cuda().unsqueeze(0)

    return curr_image


def apply_gripper_gate(action_value, gate):
    """
    应用夹爪阈值控制
    将连续的夹爪动作值转换为离散的开/关状态

    Args:
        action_value: 原始动作值
        gate: 阈值

    Returns:
        int: 离散化后的夹爪动作 (0=关闭, 5=打开)
    """
    min_gripper = 0
    max_gripper = 5

    return min_gripper if action_value < gate else max_gripper


def get_obervations(args, timestep, ros_operator):
    """
    获取单次观测数据
    从ROS系统获取当前时刻的传感器数据

    Args:
        args: 命令行参数
        timestep: 当前时间步
        ros_operator: ROS操作对象

    Returns:
        dict: 观测数据字典，包含图像和状态信息
    """
    global obs_dict

    rate = Rate(args.frame_rate)  # 设置循环频率
    while True and rclpy.ok():
        # 尝试获取同步的观测数据
        obs_dict = ros_operator.get_observation(ts=timestep)
        if not obs_dict:
            print("syn fail")  # 同步失败
            rate.sleep()
            continue

        return obs_dict


def init_robot(ros_operator, use_base, connected_event, start_event):
    """
    机器人初始化函数
    将机器人移动到安全的初始位置，准备开始任务执行

    Args:
        ros_operator: ROS操作对象
        use_base: 是否使用移动底盘
        connected_event: 连接完成事件
        start_event: 开始任务事件
    """
    # 预定义的安全初始位置（关节角度）
    init0 = [0.0, 0.948, 0.858, -0.573, 0.0, 0.0, -2.8]  # 夹爪打开状态
    init1 = [0.0, 0.948, 0.858, -0.573, 0.0, 0.0, 0.0]  # 夹爪关闭状态

    # 发布初始位置（关节空间姿态）- 双臂同时移动到初始位置
    ros_operator.follow_arm_publish_continuous(init0, init0)

    # 通知主进程机器人已连接并初始化完成
    connected_event.set()
    # 等待主进程确认开始任务
    start_event.wait()

    # 将夹爪设置为关闭状态，准备操作
    ros_operator.follow_arm_publish_continuous(init1, init1)
    if use_base:
        # 如果使用底盘，启动底盘控制线程
        ros_operator.start_base_control_thread()


def signal_handler(signal, frame, ros_operator):
    """
    信号处理函数，处理Ctrl+C中断
    安全关闭机器人控制

    Args:
        signal: 信号类型
        frame: 堆栈帧
        ros_operator: ROS操作对象
    """
    print("Caught Ctrl+C / SIGINT signal")

    # 安全关闭底盘控制
    ros_operator.base_enable = False
    ros_operator.robot_base_shutdown()
    ros_operator.base_control_thread.join()

    sys.exit(0)


def cleanup_shm(names):
    """
    清理共享内存
    删除可能残留的共享内存对象

    Args:
        names: 共享内存名称列表
    """
    for name in names:
        try:
            shm = SharedMemory(name=name)
            shm.close()
            shm.unlink()
        except FileNotFoundError:
            pass  # 共享内存不存在，忽略错误


def ros_process(
    args, config, meta_queue, connected_event, start_event, shm_ready_event
):
    """
    ROS进程主函数 - 观测获取和动作执行
    这是系统的核心进程，负责：
    1. 初始化ROS系统和机器人
    2. 建立共享内存通信
    3. 实时获取传感器观测数据
    4. 执行模型预测的动作

    Args:
        args: 命令行参数
        config: 模型配置
        meta_queue: 元数据队列，用于进程间通信
        connected_event: 连接事件，通知机器人初始化完成
        start_event: 开始事件，允许任务开始执行
        shm_ready_event: 共享内存就绪事件
    """

    def _ros_spin(executor):
        """ROS事件循环函数"""
        executor.spin()

    # 加载ROS环境和依赖
    setup_loader(ROOT)

    # 初始化ROS2系统
    rclpy.init()

    # 加载机器人配置文件
    data = load_yaml(args.data)
    ros_operator = RosOperator(args, data, in_collect=False)  # 推理模式

    # 创建多线程执行器，处理ROS回调
    executor = MultiThreadedExecutor()
    executor.add_node(ros_operator)

    # 在后台线程中运行ROS事件循环
    spin_thread = threading.Thread(target=_ros_spin, args=(executor,), daemon=True)
    spin_thread.start()

    # 设置信号处理（用于安全关闭底盘）
    if args.use_base:
        signal.signal(signal.SIGINT, partial(signal_handler, ros_operator=ros_operator))

    # 初始化机器人到安全位置
    init_robot(ros_operator, args.use_base, connected_event, start_event)

    # === 阶段1：获取数据形状信息 ===
    rate = Rate(args.frame_rate)
    while rclpy.ok():
        # 获取一次观测数据来确定数据形状
        obs = ros_operator.get_observation()
        if obs:
            # 构建数据形状字典，用于创建共享内存
            shapes = {"images": {}, "states": {}, "dtypes": {}}

            # 记录每个摄像头图像的形状和数据类型
            for cam in args.camera_names:
                img = obs["images"][cam]
                shapes["images"][cam] = img.shape
                shapes["dtypes"][cam] = img.dtype

            # 记录机器人状态数据的形状
            shapes["states"]["qpos"] = obs["qpos"].shape  # 关节位置
            shapes["states"]["qvel"] = obs["qvel"].shape  # 关节速度
            shapes["states"]["effort"] = obs["effort"].shape  # 关节力矩
            shapes["states"]["robot_base"] = obs["robot_base"].shape  # 底盘状态
            shapes["states"]["base_velocity"] = obs["base_velocity"].shape  # 底盘速度

            # 将形状信息发送给主进程
            meta_queue.put(shapes)
            break

        rate.sleep()

    # === 阶段2：建立共享内存通信 ===
    # 从主进程接收共享内存名称字典
    shm_name_dict = meta_queue.get()

    # 清理可能残留的共享内存
    cleanup_shm(shm_name_dict.values())
    # 创建新的共享内存空间
    shm_dict = create_shm_dict(config, shm_name_dict, shapes, shapes["dtypes"])
    # 通知主进程共享内存已就绪
    shm_ready_event.set()

    # === 阶段3：主循环 - 观测获取和动作执行 ===
    rate = Rate(args.frame_rate)
    while rclpy.ok():
        # 1. 获取最新观测数据
        obs = ros_operator.get_observation()
        if not obs:
            rate.sleep()
            continue

        # 2. 将观测数据写入共享内存（供推理进程使用）
        # 写入图像数据
        for cam in args.camera_names:
            shm, shape, dtype = shm_dict[cam]
            np_array = np.ndarray(shape, dtype=dtype, buffer=shm.buf)
            np_array[:] = obs["images"][cam]

        # 写入状态数据
        for state_key in shapes["states"]:
            shm, shape, dtype = shm_dict[state_key]
            np_array = np.ndarray(shape, dtype=dtype, buffer=shm.buf)
            np_array[:] = obs[state_key]

        # 3. 从共享内存读取动作并执行
        shm, shape, dtype = shm_dict["action"]
        action = np.ndarray(shape, dtype=dtype, buffer=shm.buf).copy()

        if np.any(action):  # 确保动作不全是0（即有有效的预测动作）
            gripper_gate = args.gripper_gate

            # 动作分解：双臂系统的关节索引
            gripper_idx = [6, 13]  # 左臂夹爪索引6，右臂夹爪索引13

            # 提取左臂动作（前7个关节：6个自由度+夹爪）
            left_action = action[: gripper_idx[0] + 1]
            if gripper_gate != -1:
                # 应用夹爪阈值控制
                left_action[gripper_idx[0]] = apply_gripper_gate(
                    left_action[gripper_idx[0]], gripper_gate
                )

            # 提取右臂动作（关节7-13：6个自由度+夹爪）
            right_action = action[gripper_idx[0] + 1 : gripper_idx[1] + 1]
            if gripper_gate != -1:
                # 注意：这里可能是原代码的错误，应该是right_action[gripper_idx[0]]
                right_action[gripper_idx[0]] = apply_gripper_gate(
                    left_action[gripper_idx[0]], gripper_gate
                )

            # 发布双臂动作到机器人
            ros_operator.follow_arm_publish(left_action, right_action)

            # 如果启用底盘，提取并发布底盘动作
            if args.use_base:
                action_base = action[
                    gripper_idx[1] + 1 : gripper_idx[1] + 1 + 10
                ]  # 10维底盘动作
                ros_operator.set_robot_base_target(action_base)

        rate.sleep()

    # 清理资源
    executor.shutdown()
    rclpy.shutdown()
    for shm, _, _ in shm_dict.values():
        shm.close()
        shm.unlink()


def inference_process(args, config, shm_dict, shapes, ros_proc):
    model = make_policy(config["policy_class"], config["policy_config"])
    ckpt_dir = (
        config["ckpt_dir"]
        if sys.stdin.isatty()
        else Path.joinpath(ROOT, config["ckpt_dir"])
    )
    ckpt_path = os.path.join(ckpt_dir, config["ckpt_name"])
    loading_status = model.load_state_dict(torch.load(ckpt_path, weights_only=True))
    print(loading_status)

    # 加载统计信息
    stats_path = os.path.join(config["ckpt_dir"], config["ckpt_stats_name"])
    with open(stats_path, "rb") as f:
        stats = pickle.load(f)

    chunk_size = config["policy_config"]["chunk_size"]
    hidden_dim = config["policy_config"]["hidden_dim"]
    action_dim = config["policy_config"]["action_dim"]

    use_qvel = config["policy_config"]["use_qvel"]
    use_effort = config["policy_config"]["use_effort"]
    use_eef_states = config["policy_config"]["use_eef_states"]

    use_robot_base = config["policy_config"]["use_base"]
    action = np.zeros((action_dim,))

    pre_left_states_process = (
        lambda s: (s - stats["left_states_mean"]) / stats["left_states_std"]
    )
    pre_right_states_process = (
        lambda s: (s - stats["right_states_mean"]) / stats["right_states_std"]
    )
    pre_robot_base_process = (
        lambda s: (s - stats["robot_base_mean"]) / stats["robot_base_std"]
    )
    pre_robot_head_process = (
        lambda s: (s - stats["robot_head_mean"]) / stats["robot_head_std"]
    )
    pre_base_velocity_process = (
        lambda s: (s - stats["base_velocity_mean"]) / stats["base_velocity_std"]
    )
    post_process = lambda a: a * stats["action_std"] + stats["action_mean"]

    model.cuda()
    model.eval()

    max_publish_step = config["episode_len"]

    # while ros_proc.is_alive():
    if config["temporal_agg"]:
        print(f"{config['state_dim']=}")

        # 计算内存需求
        memory_gb = (
            max_publish_step
            * (max_publish_step + chunk_size)
            * action_dim
            * 8
            / (1024**3)
        )
        print(f"📊 temporal_agg模式内存需求: {memory_gb:.2f} GB")
        print(
            f"   数组形状: ({max_publish_step}, {max_publish_step + chunk_size}, {action_dim})"
        )

        if memory_gb > 8:
            print(f"⚠️  警告：内存需求较大({memory_gb:.2f}GB)，请确保系统有足够内存")

        all_time_actions = np.zeros(
            (max_publish_step, max_publish_step + chunk_size, action_dim)
        )
        print(f"✅ 动作历史数组已创建")

    timestep = 0

    with torch.inference_mode():
        print(f"🚀 开始推理循环，最大步数: {args.max_publish_step}")
        print(f"📡 ROS进程状态: {'alive' if ros_proc.is_alive() else 'dead'}")

        while timestep < args.max_publish_step and ros_proc.is_alive():
            if timestep % 100 == 0:  # 每100步打印一次状态
                print(f"⏱️  推理步数: {timestep}/{args.max_publish_step}")
            obs_dict = {
                "images": {},
                "qpos": None,
                "qvel": None,
                "effort": None,
                "robot_base": None,
                "base_velocity": None,
            }

            # 从共享内存读取
            try:
                for cam in args.camera_names:
                    shm, shape, dtype = shm_dict[cam]
                    obs_dict["images"][cam] = np.ndarray(
                        shape, dtype=dtype, buffer=shm.buf
                    ).copy()
                for state_key in shapes["states"]:
                    shm, shape, dtype = shm_dict[state_key]
                    obs_dict[state_key] = np.ndarray(
                        shape, dtype=dtype, buffer=shm.buf
                    ).copy()
            except Exception as e:
                print(f"❌ 共享内存读取错误: {e}")
                print(f"   步数: {timestep}")
                break

            gripper_idx = [6, 13]

            left_qpos = (
                obs_dict["eef"][: gripper_idx[0] + 1]
                if use_eef_states
                else obs_dict["qpos"][: gripper_idx[0] + 1]
            )
            left_states = left_qpos

            right_qpos = (
                obs_dict["eef"][gripper_idx[0] + 1 : gripper_idx[1] + 1]
                if use_eef_states
                else obs_dict["qpos"][gripper_idx[0] + 1 : gripper_idx[1] + 1]
            )
            right_states = right_qpos

            left_states = (
                np.concatenate(
                    (left_states, obs_dict["qvel"][: gripper_idx[0] + 1]), axis=0
                )
                if use_qvel
                else left_states
            )
            left_states = (
                np.concatenate(
                    (
                        left_states,
                        obs_dict["effort"][gripper_idx[0] : gripper_idx[0] + 1],
                    ),
                    axis=0,
                )
                if use_effort
                else left_states
            )

            right_states = (
                np.concatenate(
                    (
                        right_states,
                        obs_dict["qvel"][gripper_idx[0] + 1 : gripper_idx[1] + 1],
                    ),
                    axis=0,
                )
                if use_qvel
                else right_states
            )  #
            right_states = (
                np.concatenate(
                    (
                        right_states,
                        obs_dict["effort"][gripper_idx[1] : gripper_idx[1] + 1],
                    ),
                    axis=0,
                )
                if use_effort
                else right_states
            )  #

            left_states = np.concatenate((left_states, right_states), axis=0)
            right_states = left_states

            robot_base = obs_dict["robot_base"][:3]

            robot_base = pre_robot_base_process(robot_base)
            robot_base = torch.from_numpy(robot_base).float().cuda().unsqueeze(0)

            robot_head = obs_dict["robot_base"][3:6]
            robot_head = pre_robot_head_process(robot_head)
            robot_head = torch.from_numpy(robot_head).float().cuda().unsqueeze(0)

            base_velocity = obs_dict["base_velocity"]
            base_velocity = pre_base_velocity_process(base_velocity)
            base_velocity = torch.from_numpy(base_velocity).float().cuda().unsqueeze(0)

            left_states = pre_left_states_process(left_states)
            left_states = torch.from_numpy(left_states).float().cuda().unsqueeze(0)

            right_states = pre_right_states_process(right_states)
            right_states = torch.from_numpy(right_states).float().cuda().unsqueeze(0)

            curr_image = get_image(obs_dict, config["camera_names"])
            curr_depth_image = None

            if args.use_depth_image:
                curr_depth_image = get_depth_image(obs_dict, config["camera_names"])

            if config["policy_class"] == "ACT":
                # ===== 步骤1: 模型推理 =====
                # 将当前观测数据输入ACT模型，获取未来chunk_size步的动作预测
                # all_actions形状: (1, chunk_size, action_dim)
                # 例如: (1, 30, 14) 表示预测未来30步的14维动作
                try:
                    if timestep % 100 == 0:
                        print(f"🧠 开始模型推理，步数: {timestep}")
                    all_actions = model(
                        curr_image,  # 当前图像观测 [1, C*N, H, W]
                        curr_depth_image,  # 当前深度图像 [1, C*N, H, W] (可选)
                        left_states,  # 左臂状态 [1, states_dim]
                        right_states,  # 右臂状态 [1, states_dim]
                        robot_base=robot_base,  # 机器人底盘位置 [1, 3]
                        robot_head=robot_head,  # 机器人头部姿态 [1, 3]
                        base_velocity=base_velocity,  # 底盘速度 [1, 4]
                    )
                    if timestep % 100 == 0:
                        print(f"✅ 模型推理完成，动作形状: {all_actions.shape}")
                except Exception as e:
                    print(f"❌ 模型推理错误: {e}")
                    print(f"   步数: {timestep}")
                    break

                # ===== 步骤2: 动作处理分支 =====
                if config["temporal_agg"]:
                    # ===== 分支A: 时序聚合模式 =====
                    # 用于提高动作的平滑性和稳定性

                    # 2.1 存储历史动作预测
                    # 将当前时间步的动作预测存储到全局动作矩阵中
                    # all_time_actions形状: (max_publish_step, max_publish_step + chunk_size, action_dim)
                    # 例如: (10000, 10030, 14) 用于存储所有时间步的动作预测
                    all_time_actions[[timestep], timestep : timestep + chunk_size] = (
                        all_actions.cpu().numpy()  # 转换为numpy数组并存储
                    )

                    # 2.2 提取当前时间步的所有历史预测
                    # 从所有历史预测中提取当前时间步的动作
                    # 形状: (timestep+1, action_dim) - 包含从开始到当前的所有预测
                    actions_for_curr_step = all_time_actions[:, timestep]

                    # 2.3 过滤有效预测
                    # 只保留非零的动作预测（排除未初始化的部分）
                    # actions_populated: 布尔数组，标记哪些时间步有有效预测
                    actions_populated = np.all(actions_for_curr_step != 0, axis=1)
                    actions_for_curr_step = actions_for_curr_step[actions_populated]

                    # 2.4 计算指数衰减权重
                    # 使用指数衰减函数给不同时间步的预测分配权重
                    # 越近期的预测权重越大，越早期的预测权重越小
                    k = 0.01  # 衰减系数，控制权重衰减速度
                    exp_weights = np.exp(-k * np.arange(len(actions_for_curr_step)))
                    exp_weights = exp_weights / exp_weights.sum()  # 归一化权重
                    exp_weights = exp_weights[:, np.newaxis]  # 添加维度用于广播

                    # 2.5 加权平均计算最终动作
                    # 将历史预测按权重进行加权平均，得到平滑的最终动作
                    # 形状: (1, action_dim)
                    raw_action = (actions_for_curr_step * exp_weights).sum(
                        axis=0, keepdims=True
                    )
                else:
                    # ===== 分支B: 直接模式 =====
                    # 不使用时序聚合，直接使用当前预测的动作

                    if args.pos_lookahead_step != 0:
                        # 2.1 位置前瞻模式
                        # 使用特定的前瞻步数来选择动作
                        # 用于处理位置相关的特殊控制需求
                        raw_action = all_actions[
                            :, timestep % args.model.inference.pos_lookahead_step
                        ]
                    else:
                        # 2.2 标准模式
                        # 使用当前时间步在chunk中的位置来选择动作
                        # timestep % chunk_size 确保在chunk范围内循环
                        raw_action = all_actions[:, timestep % chunk_size]

            else:
                # 其他策略类型暂未实现
                raise NotImplementedError

            # ===== 步骤3: 动作后处理 =====
            # 将模型输出的原始动作转换为实际可执行的动作
            # post_process函数会进行反归一化等操作
            # raw_action[0] 去除batch维度，得到 (action_dim,) 的动作向量
            action = post_process(raw_action[0])

            # ===== 步骤4: 动作执行 =====
            # 将处理后的动作写入共享内存，供ROS进程读取并执行
            robot_action(action, shm_dict)

            # ===== 步骤5: 时间步更新 =====
            timestep += 1

        # ===== 步骤6: 底盘动作清理 =====
        # 如果使用移动底盘，在推理结束时将某些底盘动作设为0
        # 这可能是为了安全停止或重置某些底盘功能
        if args.use_base:
            action[16] = 0  # 底盘动作索引16设为0
            action[17] = 0  # 底盘动作索引17设为0
            action[19] = 0  # 底盘动作索引19设为0

        # ===== 步骤7: 最终动作执行 =====
        # 执行清理后的最终动作
        robot_action(action, shm_dict)


def parse_args(known=False):
    parser = argparse.ArgumentParser()  # 解析命令行参数
    parser.add_argument(
        "--max_publish_step", type=int, default=3600, help="max publish step"
    )  # 单回合最大步数，默认3600步匹配数据集长度

    # 数据集和检查点设置
    parser.add_argument(
        "--ckpt_dir", type=str, default=Path.joinpath(ROOT, "weights"), help="ckpt dir"
    )
    parser.add_argument(
        "--ckpt_name", type=str, default="policy_best.ckpt", help="ckpt name"
    )
    parser.add_argument("--pretrain_ckpt", type=str, default="", help="pretrain ckpt")
    parser.add_argument(
        "--ckpt_stats_name",
        type=str,
        default="dataset_stats.pkl",
        help="ckpt stats name",
    )

    # 配置文件
    parser.add_argument(
        "--data",
        type=str,
        default=Path.joinpath(ROOT, "data/config.yaml"),
        help="config file",
    )

    # 推理设置
    parser.add_argument("--seed", type=int, default=0, help="seed")
    parser.add_argument("--lr", type=float, default=1e-5, help="learning rate")
    parser.add_argument(
        "--lr_backbone", type=float, default=1e-5, help="learning rate for backbone"
    )
    parser.add_argument(
        "--weight_decay", type=float, default=1e-4, help="weight decay rate"
    )
    parser.add_argument(
        "--loss_function",
        type=str,
        choices=["l1", "l2", "l1+l2"],
        default="l1",
        help="loss function",
    )
    parser.add_argument(
        "--pos_lookahead_step", type=int, default=0, help="position lookahead step"
    )

    # 模型结构设置
    parser.add_argument(
        "--policy_class",
        type=str,
        choices=["CNNMLP", "ACT", "Diffusion"],
        default="ACT",
        help="policy class selection",
    )
    parser.add_argument(
        "--backbone", type=str, default="resnet18", help="backbone model architecture"
    )
    parser.add_argument(
        "--chunk_size", type=int, default=30, help="chunk size for input data"
    )
    parser.add_argument(
        "--hidden_dim", type=int, default=512, help="hidden layer dimension size"
    )

    # 摄像头和位置嵌入设置
    parser.add_argument(
        "--camera_names",
        nargs="+",
        type=str,
        choices=[
            "head",
            "left_wrist",
            "right_wrist",
        ],
        default=["head", "left_wrist", "right_wrist"],
        help="camera names to use",
    )
    parser.add_argument(
        "--position_embedding",
        type=str,
        choices=("sine", "learned"),
        default="sine",
        help="type of positional embedding to use",
    )
    parser.add_argument(
        "--masks", action="store_true", help="train segmentation head if provided"
    )
    parser.add_argument(
        "--dilation",
        action="store_true",
        help="replace stride with dilation in the last convolutional block (DC5)",
    )

    # 机器人设置
    parser.add_argument("--use_base", action="store_true", help="use robot base")
    parser.add_argument(
        "--record",
        choices=["Distance", "Speed"],
        default="Distance",
        help="record data",
    )
    parser.add_argument("--frame_rate", type=int, default=60, help="frame rate")

    # ACT模型专用设置
    parser.add_argument(
        "--enc_layers", type=int, default=4, help="number of encoder layers"
    )
    parser.add_argument(
        "--dec_layers", type=int, default=7, help="number of decoder layers"
    )
    parser.add_argument(
        "--nheads", type=int, default=8, help="number of attention heads"
    )
    parser.add_argument(
        "--dropout", type=float, default=0.1, help="dropout rate in transformer layers"
    )
    parser.add_argument(
        "--pre_norm", action="store_true", help="use pre-normalization in transformer"
    )
    parser.add_argument(
        "--states_dim", type=int, default=14, help="state dimension size"
    )
    parser.add_argument(
        "--kl_weight", type=int, default=10, help="KL divergence weight"
    )
    parser.add_argument(
        "--dim_feedforward",
        type=int,
        default=3200,
        help="feedforward network dimension",
    )
    parser.add_argument(
        "--temporal_agg",
        type=bool,
        default=True,
        help="use temporal aggregation (uses more memory)",
    )

    # Diffusion模型专用设置
    parser.add_argument(
        "--observation_horizon", type=int, default=1, help="observation horizon length"
    )
    parser.add_argument(
        "--action_horizon", type=int, default=8, help="action horizon length"
    )
    parser.add_argument(
        "--num_inference_timesteps",
        type=int,
        default=10,
        help="number of inference timesteps",
    )
    parser.add_argument(
        "--ema_power", type=int, default=0.75, help="EMA power for diffusion process"
    )

    # 图像设置
    parser.add_argument(
        "--use_depth_image", action="store_true", help="use depth image"
    )

    # 状态和动作设置
    parser.add_argument(
        "--use_qvel", action="store_true", help="include qvel in state information"
    )
    parser.add_argument(
        "--use_effort", action="store_true", help="include effort data in state"
    )
    parser.add_argument(
        "--use_eef_states", action="store_true", help="use eef data in state"
    )

    parser.add_argument(
        "--gripper_gate", type=float, default=-1, help="gripper gate threshold"
    )

    return parser.parse_known_args()[0] if known else parser.parse_args()


def main(args):
    """
    1、多线程通信机制：队列用于数据交换，事件用于同步
    2、ROS进程+推理进程：ROS进程负责数据通信，推理进程负责模型预测
    3、共享内存机制：实现进程间高效数据传递
    """
    meta_queue = mp.Queue()  # 用于在主进程和推理进程之间传递meta信息

    connected_event = mp.Event()  # 用于在主进程和推理进程之间传递连接事件
    start_event = mp.Event()  # 用于在主进程和推理进程之间传递启动事件
    shm_ready_event = mp.Event()  # 用于在主进程和推理进程之间传递共享内存就绪事件

    # 获取模型config
    config = get_model_config(args)

    # *启动ROS进程
    ros_proc = mp.Process(
        target=ros_process,
        args=(args, config, meta_queue, connected_event, start_event, shm_ready_event),
    )
    ros_proc.start()

    connected_event.wait()
    print("🚀 系统已就绪，3秒后自动开始推理...")
    import time

    time.sleep(3)
    start_event.set()

    # 等待meta信息
    shapes = meta_queue.get()

    shm_name_dict = make_shm_name_dict(args, shapes)

    meta_queue.put(shm_name_dict)

    shm_ready_event.wait()

    shm_dict = connect_shm_dict(shm_name_dict, shapes, shapes["dtypes"], config)

    # 推理
    try:
        inference_process(args, config, shm_dict, shapes, ros_proc)
    except KeyboardInterrupt:
        pass
    finally:
        for shm, _, _ in shm_dict.values():
            shm.close()
            shm.unlink()
        ros_proc.terminate()
        ros_proc.join()


if __name__ == "__main__":
    args = parse_args()
    main(args)
