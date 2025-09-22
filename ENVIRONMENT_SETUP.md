  # ROS OpenPi Environment Setup Guide

本指南帮助您在新电脑上快速重建ros_openpi环境。

## 方法1：使用environment.yml文件（推荐）

### 创建环境
```bash
# 安装mamba（如果还没有）
curl -L -O https://github.com/conda-forge/miniforge/releases/latest/download/Mambaforge-Linux-x86_64.sh
bash Mambaforge-Linux-x86_64.sh

# 使用yml文件创建环境
mamba env create -f ros_openpi_environment.yml
```

### 激活环境
```bash
mamba activate ros_openpi
```

## 方法2：使用packages.txt文件

```bash
# 创建新环境
mamba create -n ros_openpi python=3.11

# 激活环境
mamba activate ros_openpi

# 安装包（这个方法可能需要手动解决依赖）
mamba install --file ros_openpi_packages.txt
```

## 验证环境

激活环境后，验证关键组件：

```bash
# 检查Python版本
python --version

# 检查ROS2
ros2 --version

# 检查PyTorch
python -c "import torch; print(f'PyTorch: {torch.__version__}, CUDA: {torch.cuda.is_available()}')"

# 检查其他关键包
python -c "import cv2, numpy, h5py; print('All packages imported successfully')"
```

## 文件说明

- `ros_openpi_environment.yml`: 完整环境配置（推荐使用）
- `ros_openpi_packages.txt`: 包列表文件（备用方案）

## 注意事项

1. 确保目标系统有足够的磁盘空间（至少5GB）
2. 如果使用GPU，确保安装了相应的CUDA驱动
3. ROS2相关包可能需要额外的系统依赖

## 故障排除

如果遇到问题，可以：
1. 更新mamba: `mamba update mamba`
2. 清理缓存: `mamba clean --all`
3. 手动安装关键包: `mamba install pytorch torchvision torchaudio -c pytorch`
