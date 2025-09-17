#!/usr/bin/env python3
"""
分析HDF5数据集中的action数据结构，特别是gripper维度的处理
"""
import h5py
import numpy as np
import os


def analyze_episode_dataset(dataset_path):
    """分析单个episode数据集"""
    print(f"分析数据集: {dataset_path}")
    print("=" * 60)

    with h5py.File(dataset_path, "r") as root:
        # 打印所有可用的数据集
        print("可用的数据集:")

        def print_structure(name, obj):
            if isinstance(obj, h5py.Dataset):
                print(f"  {name}: shape={obj.shape}, dtype={obj.dtype}")

        root.visititems(print_structure)
        print()

        # 分析action相关数据
        action_datasets = ["action", "action_eef", "action_base", "action_velocity"]

        for dataset_name in action_datasets:
            if dataset_name in root:
                data = root[dataset_name]
                print(f"{dataset_name}:")
                print(f"  shape: {data.shape}")
                print(f"  dtype: {data.dtype}")
                if len(data.shape) > 0:
                    print(f"  前5个样本:")
                    for i in range(min(5, data.shape[0])):
                        print(f"    [{i}]: {data[i]}")
                print()

        # 特别分析action数据
        if "action" in root:
            action_data = root["action"]
            print("Action数据分析:")
            print(
                f"  总维度: {action_data.shape[1] if len(action_data.shape) > 1 else 'N/A'}"
            )
            print(
                f"  时间步数: {action_data.shape[0] if len(action_data.shape) > 0 else 'N/A'}"
            )

            # 分析gripper维度（假设是最后几维）
            if len(action_data.shape) > 1:
                action_dim = action_data.shape[1]
                print(f"  Action维度分析:")

                # 假设是双臂机器人，每臂7个关节 + 1个gripper
                if action_dim == 14:  # 7+7 = 14 (双臂)
                    print(f"    左臂关节 (0-6): {action_data[0, :7]}")
                    print(f"    左臂gripper (6): {action_data[0, 6]}")
                    print(f"    右臂关节 (7-13): {action_data[0, 7:13]}")
                    print(f"    右臂gripper (13): {action_data[0, 13]}")
                elif action_dim == 16:  # 7+7+2 = 16 (双臂+base)
                    print(f"    左臂关节 (0-6): {action_data[0, :7]}")
                    print(f"    左臂gripper (6): {action_data[0, 6]}")
                    print(f"    右臂关节 (7-13): {action_data[0, 7:13]}")
                    print(f"    右臂gripper (13): {action_data[0, 13]}")
                    print(f"    Base (14-15): {action_data[0, 14:16]}")
                else:
                    print(f"    未知的action维度结构: {action_dim}")
                    print(f"    前几个值: {action_data[0, :min(10, action_dim)]}")

                # 分析gripper值的范围
                if action_dim >= 14:
                    left_gripper = action_data[:, 6]
                    right_gripper = action_data[:, 13]
                    print(
                        f"  左臂gripper范围: [{left_gripper.min():.3f}, {left_gripper.max():.3f}]"
                    )
                    print(
                        f"  右臂gripper范围: [{right_gripper.min():.3f}, {right_gripper.max():.3f}]"
                    )
                    print(f"  左臂gripper唯一值: {np.unique(left_gripper)}")
                    print(f"  右臂gripper唯一值: {np.unique(right_gripper)}")

        print()


def main():
    dataset_dir = "/home/Xense/AC_One/act/datasets"

    # 分析前几个episode
    for i in range(3):
        episode_path = os.path.join(dataset_dir, f"episode_{i}.hdf5")
        if os.path.exists(episode_path):
            analyze_episode_dataset(episode_path)
        else:
            print(f"Episode {i} 不存在")


if __name__ == "__main__":
    main()
