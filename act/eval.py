# -- coding: UTF-8
import os
import sys

sys.stdout = open(sys.stdout.fileno(), mode="w", buffering=1)
sys.stderr = open(sys.stderr.fileno(), mode="w", buffering=1)

from pathlib import Path

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))
    os.chdir(str(ROOT))

import yaml
import torch
import pickle
import argparse
import numpy as np
import h5py
import cv2
from tqdm import tqdm

# 设置环境变量避免警告
os.environ["ROBOMIMIC_PRIVATE_MACROS_PATH"] = os.path.join(
    os.path.dirname(__file__), "robomimic_macros.py"
)

from utils.utils import load_data, compute_dict_mean, set_seed, detach_dict
from utils.policy import ACTPolicy, CNNMLPPolicy, DiffusionPolicy

# 设置打印输出行宽
np.set_printoptions(linewidth=200)

# 禁用科学计数法
np.set_printoptions(suppress=True)


def initialize_policy_config(args):
    """初始化策略配置，参考train.py"""
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
        "use_depth_image": args.use_depth_image,
        "temporal_agg": args.temporal_agg,
        "max_predictions": args.max_predictions,
    }

    if args.policy_class == "ACT":
        act_config = {
            "policy_class": "ACT",
            "enc_layers": args.enc_layers,
            "dec_layers": args.dec_layers,
            "nheads": args.nheads,
            "dropout": args.dropout,
            "pre_norm": args.pre_norm,
            "states_dim": 7,
            "action_dim": 7,
            "kl_weight": args.kl_weight,
            "dim_feedforward": args.dim_feedforward,
            "use_qvel": args.use_qvel,
            "use_effort": args.use_effort,
            "use_eef_states": args.use_eef_states,
            "use_eef_action": args.use_eef_action,
        }

        # 更新 states_dim
        act_config["states_dim"] += act_config["action_dim"] if args.use_qvel else 0
        act_config["states_dim"] += 1 if args.use_effort else 0
        act_config["states_dim"] *= 2

        # 更新 action_dim - 修正为14维以匹配检查点
        act_config["action_dim"] = 14  # 直接设置为14维

        return {**base_config, **act_config}

    elif args.policy_class == "CNNMLP":
        cnnmlp_config = {
            "policy_class": "CNNMLP",
            "action_dim": 14,
            "states_dim": 14,
        }

        return {**base_config, **cnnmlp_config}

    elif args.policy_class == "Diffusion":
        diffusion_config = {
            "policy_class": "Diffusion",
            "observation_horizon": args.observation_horizon,
            "action_horizon": args.action_horizon,
            "num_inference_timesteps": args.num_inference_timesteps,
            "ema_power": args.ema_power,
            "action_dim": 14,
            "states_dim": 14,
        }

        return {**base_config, **diffusion_config}

    else:
        raise NotImplementedError("Unknown policy class")


def load_hdf5_data(dataset_path):
    """加载HDF5数据"""
    print(f"📁 加载数据: {dataset_path}")

    try:
        with h5py.File(dataset_path, "r") as f:
            # 读取数据
            qposes = f["observations/qpos"][:]
            eefs = f["observations/eef"][:]
            actions = f["action"][:]  # 修正：动作数据在根级别

            # 读取图像数据
            images = {}
            for camera_name in ["head", "left_wrist", "right_wrist"]:
                if f"observations/images/{camera_name}" in f:
                    images[camera_name] = f[f"observations/images/{camera_name}"][:]
                else:
                    print(f"⚠️  警告: 未找到相机 {camera_name} 的数据")

            print(f"✅ 数据加载成功:")
            print(f"  QPOS shape: {qposes.shape}")
            print(f"  EEF shape: {eefs.shape}")
            print(f"  Actions shape: {actions.shape}")
            print(f"  Images: {list(images.keys())}")
            for camera_name, data in images.items():
                print(f"    {camera_name}: {data.shape}")

            return {
                "qposes": qposes,
                "eefs": eefs,
                "actions": actions,
                "images": images,
            }
    except Exception as e:
        print(f"❌ 数据加载失败: {e}")
        return None


def load_stats(stats_path):
    """加载统计信息"""
    print(f"📊 加载统计信息: {stats_path}")

    try:
        with open(stats_path, "rb") as f:
            stats = pickle.load(f)
        print(f"✅ 统计信息加载成功")
        return stats
    except Exception as e:
        print(f"❌ 统计信息加载失败: {e}")
        return None


def evaluate_episode(model, dataset, stats, args, config):
    """评估单个episode"""
    print(f"\n🔍 开始评估 episode...")

    # 设置模型为评估模式
    model.eval()

    # 获取数据
    qposes = dataset["qposes"]
    eefs = dataset["eefs"]
    actions = dataset["actions"]
    images = dataset["images"]
    episode_len = len(actions)
    print(f"Episode 长度: {episode_len}")

    # 初始化temporal aggregation
    if config["temporal_agg"]:
        action_dim = config["action_dim"]
        chunk_size = config["chunk_size"]
        all_time_actions = torch.zeros(
            (episode_len, episode_len + chunk_size, action_dim), device="cuda"
        )

    predicted_actions = []
    ground_truth_actions = []

    with torch.inference_mode():
        for timestep in tqdm(range(episode_len), desc="评估进度"):
            # 获取当前观测
            qpos = qposes[timestep]
            eef = eefs[timestep]

            # 预处理状态 - 参考train.py中的处理方式
            if config["policy_class"] == "ACT":
                # 构建状态向量
                states = []

                # 左臂状态
                left_qpos = qpos[:7]
                left_eef = eef[:7]
                left_states = [left_qpos, left_eef]

                if config["use_qvel"]:
                    # 这里需要qvel数据，暂时跳过
                    pass
                if config["use_effort"]:
                    # 这里需要effort数据，暂时跳过
                    pass

                # 右臂状态
                right_qpos = qpos[7:]
                right_eef = eef[7:]
                right_states = [right_qpos, right_eef]

                if config["use_qvel"]:
                    # 这里需要qvel数据，暂时跳过
                    pass
                if config["use_effort"]:
                    # 这里需要effort数据，暂时跳过
                    pass

                # 合并状态
                left_states = np.concatenate(left_states)
                right_states = np.concatenate(right_states)

                # 转换为张量
                left_states_tensor = (
                    torch.from_numpy(left_states).float().cuda().unsqueeze(0)
                )
                right_states_tensor = (
                    torch.from_numpy(right_states).float().cuda().unsqueeze(0)
                )

                # 获取图像 - 参考inference.py的处理方式
                curr_images = []
                for camera_name in config["camera_names"]:
                    if camera_name in images:
                        # 解码JPEG图像
                        img_data = images[camera_name][timestep]
                        img = cv2.imdecode(img_data, cv2.IMREAD_COLOR)
                        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                        img = cv2.resize(img, (480, 480))
                        # 重新排列为 (c, h, w) 格式
                        img = np.transpose(img, (2, 0, 1))
                        curr_images.append(img)

                if curr_images:
                    curr_image = np.stack(curr_images, axis=0)  # 堆叠多相机图像
                    curr_image = (
                        torch.from_numpy(curr_image / 255.0).float().cuda().unsqueeze(0)
                    )
                else:
                    # 如果没有图像，创建零图像
                    curr_image = torch.zeros(
                        (1, len(config["camera_names"]), 3, 480, 480), device="cuda"
                    )

                # 模型推理
                all_actions = model(
                    curr_image, None, left_states_tensor, right_states_tensor
                )

                chunk_size = config["chunk_size"]

                if config["temporal_agg"]:
                    # Temporal aggregation
                    all_time_actions[[timestep], timestep : timestep + chunk_size] = (
                        all_actions
                    )
                    actions_for_curr_step = all_time_actions[:, timestep]
                    actions_populated = torch.all(actions_for_curr_step != 0, axis=1)
                    actions_for_curr_step = actions_for_curr_step[actions_populated]

                    # 限制有效预测数量
                    max_predictions = args.max_predictions
                    if len(actions_for_curr_step) > max_predictions:
                        actions_for_curr_step = actions_for_curr_step[-max_predictions:]

                    k = 0.1
                    exp_weights = torch.exp(
                        -k
                        * torch.arange(
                            len(actions_for_curr_step),
                            device=actions_for_curr_step.device,
                        )
                    )
                    exp_weights = exp_weights / exp_weights.sum()
                    exp_weights = exp_weights.unsqueeze(dim=1)
                    raw_action = (actions_for_curr_step * exp_weights).sum(
                        dim=0, keepdim=True
                    )
                else:
                    raw_action = all_actions[:, timestep % chunk_size]

            elif config["policy_class"] == "Diffusion":
                # 处理Diffusion模型
                states = np.concatenate([qpos, eef])
                states_tensor = torch.from_numpy(states).float().cuda().unsqueeze(0)

                # 获取图像
                curr_images = []
                for camera_name in config["camera_names"]:
                    if camera_name in images:
                        img_data = images[camera_name][timestep]
                        img = cv2.imdecode(img_data, cv2.IMREAD_COLOR)
                        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                        img = cv2.resize(img, (480, 480))
                        curr_images.append(img)

                if curr_images:
                    curr_image = np.concatenate(curr_images, axis=2)
                    curr_image = (
                        torch.from_numpy(curr_image / 255.0).float().cuda().unsqueeze(0)
                    )
                else:
                    curr_image = torch.zeros((1, 480, 480, 3), device="cuda")

                all_actions = model(states_tensor, curr_image)
                raw_action = all_actions[:, timestep % chunk_size]

            elif config["policy_class"] == "CNNMLP":
                # 处理CNNMLP模型
                states = np.concatenate([qpos, eef])
                states_tensor = torch.from_numpy(states).float().cuda().unsqueeze(0)

                # 获取图像
                curr_images = []
                for camera_name in config["camera_names"]:
                    if camera_name in images:
                        img_data = images[camera_name][timestep]
                        img = cv2.imdecode(img_data, cv2.IMREAD_COLOR)
                        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                        img = cv2.resize(img, (480, 480))
                        curr_images.append(img)

                if curr_images:
                    curr_image = np.concatenate(curr_images, axis=2)
                    curr_image = (
                        torch.from_numpy(curr_image / 255.0).float().cuda().unsqueeze(0)
                    )
                else:
                    curr_image = torch.zeros((1, 480, 480, 3), device="cuda")

                raw_action = model(states_tensor, curr_image)

            else:
                raise NotImplementedError(
                    f"Unknown policy class: {config['policy_class']}"
                )

            # 后处理动作
            raw_action_cpu = raw_action.cpu().numpy()
            predicted_action = postprocess_action(raw_action_cpu[0], stats)
            predicted_actions.append(predicted_action)
            ground_truth_actions.append(actions[timestep])

            # 打印对比信息
            if timestep % 100 == 0:  # 每100步打印一次
                print(f"\n📊 Step {timestep} 动作对比:")
                print(f"预测动作: {[f'{x:.4f}' for x in predicted_action]}")
                print(f"真值动作: {[f'{x:.4f}' for x in actions[timestep]]}")

                # 计算差异
                diff = np.abs(predicted_action - actions[timestep])
                print(f"绝对差异: {[f'{x:.4f}' for x in diff]}")
                print(f"平均差异: {np.mean(diff):.4f}")

                # 特别关注夹爪动作
                gripper_indices = [6, 13]  # 左臂和右臂夹爪
                print(f"夹爪动作对比:")
                for i, idx in enumerate(gripper_indices):
                    arm_name = "左臂" if i == 0 else "右臂"
                    print(
                        f"  {arm_name}夹爪 - 预测: {predicted_action[idx]:.4f}, 真值: {actions[timestep][idx]:.4f}, 差异: {diff[idx]:.4f}"
                    )

    predicted_actions = np.array(predicted_actions)
    ground_truth_actions = np.array(ground_truth_actions)

    return predicted_actions, ground_truth_actions


def postprocess_action(raw_action, stats):
    """后处理动作数据"""
    if stats is None:
        return raw_action

    # 使用统计信息进行后处理
    action_mean = stats.get("action_mean", 0)
    action_std = stats.get("action_std", 1)

    processed_action = raw_action * action_std + action_mean
    return processed_action


def calculate_metrics(predicted_actions, ground_truth_actions):
    """计算评估指标"""
    # 计算MSE
    mse = np.mean((predicted_actions - ground_truth_actions) ** 2)

    # 计算MAE
    mae = np.mean(np.abs(predicted_actions - ground_truth_actions))

    # 计算相关系数
    correlation = np.corrcoef(
        predicted_actions.flatten(), ground_truth_actions.flatten()
    )[0, 1]

    return {"mse": mse, "mae": mae, "correlation": correlation}


def main():
    parser = argparse.ArgumentParser(description="模型评估脚本")

    # 数据参数
    parser.add_argument("--data", type=str, required=True, help="数据配置文件路径")
    parser.add_argument("--ckpt_dir", type=str, required=True, help="检查点目录")
    parser.add_argument("--dataset_path", type=str, required=True, help="数据集路径")
    parser.add_argument(
        "--camera_names",
        nargs="+",
        default=["head", "left_wrist", "right_wrist"],
        help="相机名称",
    )

    # 模型参数
    parser.add_argument(
        "--policy_class",
        type=str,
        default="ACT",
        choices=["ACT", "CNNMLP", "Diffusion"],
        help="策略类别",
    )
    parser.add_argument("--chunk_size", type=int, default=50, help="chunk size")
    parser.add_argument("--hidden_dim", type=int, default=512, help="hidden dimension")
    parser.add_argument(
        "--dim_feedforward", type=int, default=3200, help="feedforward dimension"
    )
    parser.add_argument("--kl_weight", type=float, default=10, help="KL weight")
    parser.add_argument("--lr", type=float, default=1e-5, help="learning rate")

    # 训练参数
    parser.add_argument(
        "--lr_backbone", type=float, default=1e-5, help="backbone learning rate"
    )
    parser.add_argument("--weight_decay", type=float, default=1e-4, help="weight decay")
    parser.add_argument("--loss_function", type=str, default="l1", help="loss function")
    parser.add_argument("--backbone", type=str, default="resnet18", help="backbone")
    parser.add_argument(
        "--position_embedding",
        type=str,
        default="sine",
        choices=["sine", "learned"],
        help="position embedding",
    )
    parser.add_argument("--masks", action="store_true", help="use masks")
    parser.add_argument("--dilation", action="store_true", help="use dilation")
    parser.add_argument(
        "--use_depth_image", action="store_true", help="use depth image"
    )

    # ACT参数
    parser.add_argument("--enc_layers", type=int, default=4, help="encoder layers")
    parser.add_argument("--dec_layers", type=int, default=7, help="decoder layers")
    parser.add_argument("--nheads", type=int, default=8, help="number of heads")
    parser.add_argument("--dropout", type=float, default=0.1, help="dropout rate")
    parser.add_argument("--pre_norm", action="store_true", help="use pre norm")
    parser.add_argument("--use_qvel", action="store_true", help="use qvel")
    parser.add_argument("--use_effort", action="store_true", help="use effort")
    parser.add_argument("--use_eef_states", action="store_true", help="use eef states")
    parser.add_argument("--use_eef_action", action="store_true", help="use eef action")

    # Diffusion参数
    parser.add_argument(
        "--observation_horizon", type=int, default=1, help="observation horizon"
    )
    parser.add_argument("--action_horizon", type=int, default=8, help="action horizon")
    parser.add_argument(
        "--num_inference_timesteps", type=int, default=10, help="inference timesteps"
    )
    parser.add_argument("--ema_power", type=float, default=0.75, help="ema power")

    # Temporal aggregation参数
    parser.add_argument(
        "--temporal_agg", action="store_true", help="use temporal aggregation"
    )
    parser.add_argument(
        "--max_predictions",
        type=int,
        default=50,
        help="max predictions for temporal agg",
    )

    args = parser.parse_args()

    print("🚀 开始模型评估...")
    print(f"数据配置: {args.data}")
    print(f"检查点目录: {args.ckpt_dir}")
    print(f"数据集路径: {args.dataset_path}")

    # 加载数据
    dataset = load_hdf5_data(args.dataset_path)
    if dataset is None:
        return

    # 加载统计信息
    stats_path = os.path.join(args.ckpt_dir, "dataset_stats.pkl")
    stats = load_stats(stats_path)

    # 初始化策略配置
    config = initialize_policy_config(args)

    # 创建策略
    if args.policy_class == "ACT":
        policy = ACTPolicy(config)
    elif args.policy_class == "CNNMLP":
        policy = CNNMLPPolicy(config)
    elif args.policy_class == "Diffusion":
        policy = DiffusionPolicy(config)
    else:
        raise NotImplementedError(f"Unknown policy class: {args.policy_class}")

    # 加载检查点
    ckpt_path = os.path.join(args.ckpt_dir, "policy_best.ckpt")
    if os.path.exists(ckpt_path):
        print(f"✅ 加载检查点: {ckpt_path}")
        checkpoint = torch.load(ckpt_path, map_location="cuda")
        policy.load_state_dict(checkpoint)
        print(f"加载状态: {checkpoint.get('state_dict', 'No state_dict found')}")
    else:
        print(f"❌ 检查点不存在: {ckpt_path}")
        return

    # 评估
    predicted_actions, ground_truth_actions = evaluate_episode(
        policy, dataset, stats, args, config
    )

    # 计算指标
    metrics = calculate_metrics(predicted_actions, ground_truth_actions)

    print(f"\n📊 评估结果:")
    print(f"MSE: {metrics['mse']:.6f}")
    print(f"MAE: {metrics['mae']:.6f}")
    print(f"Correlation: {metrics['correlation']:.6f}")

    # 保存结果
    results = {
        "predicted_actions": predicted_actions,
        "ground_truth_actions": ground_truth_actions,
        "metrics": metrics,
    }

    output_path = "eval_results.pkl"
    with open(output_path, "wb") as f:
        pickle.dump(results, f)
    print(f"💾 结果已保存到: {output_path}")


if __name__ == "__main__":
    main()
