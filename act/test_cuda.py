#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys

# 设置CUDA设备
os.environ["CUDA_VISIBLE_DEVICES"] = "0"

import torch


def test_cuda():
    print("🔧 CUDA 测试")
    print("=" * 50)

    # 检查环境变量
    print(f"CUDA_VISIBLE_DEVICES: {os.environ.get('CUDA_VISIBLE_DEVICES', 'Not set')}")

    # 检查CUDA可用性
    print(f"CUDA 可用: {torch.cuda.is_available()}")

    if torch.cuda.is_available():
        print(f"GPU 数量: {torch.cuda.device_count()}")
        print(f"当前设备: {torch.cuda.current_device()}")
        print(f"设备名称: {torch.cuda.get_device_name(0)}")

        # 测试简单的CUDA操作
        try:
            x = torch.tensor([1.0, 2.0, 3.0]).cuda()
            y = x * 2
            print(f"CUDA 测试成功: {y.cpu().numpy()}")
        except Exception as e:
            print(f"CUDA 操作失败: {e}")
    else:
        print("❌ CUDA 不可用")
        print("可能的原因:")
        print("1. NVIDIA 驱动未安装或版本不兼容")
        print("2. CUDA 未安装或版本不兼容")
        print("3. PyTorch 版本不支持 CUDA")


if __name__ == "__main__":
    test_cuda()
