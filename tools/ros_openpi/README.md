# ROS OpenPi 环境启动脚本

这个目录包含了适配 `ros_openpi` 环境（使用 mamba 和 Python 3.11）的所有启动脚本。

## 环境要求

- **Mamba**: 已安装 mamba 包管理器
- **ros_openpi 环境**: 包含 ROS2 Humble 和 Python 3.11 的 mamba 环境
- **编译**: ROS2 工作空间必须在 ros_openpi 环境下编译

## 使用流程

### 1. 首次设置 - 编译工作空间

在使用任何其他脚本之前，必须先在 ros_openpi 环境下编译 ROS2 工作空间：

```bash
cd tools/ros_openpi
./build_ros_openpi.sh
```

这个脚本会：
- 检查 mamba 和 ros_openpi 环境
- 清理旧的编译文件
- 在 ros_openpi 环境下重新编译 ROS2 工作空间
- 确保 Python 版本一致性

### 2. 机械臂控制

#### 末端位置控制
```bash
./00_eef_control_ros_openpi.sh
```
启动末端位置控制模式，支持六自由度控制。

#### 关节空间控制
```bash
./00_joint_control_ros_openpi.sh
```
启动关节空间控制模式，支持每个关节独立控制。

### 3. 数据采集
```bash
./01_collect_ros_openpi.sh
```

启动完整的数据采集系统，包括：
- CAN 接口 (can1, can3, can6)
- ROS2 机械臂控制器
- 手柄控制
- RealSense 相机
- 数据采集程序

**操作说明**：
- 等待3秒倒计时后开始录制
- 按手柄按键0开始录制当前episode
- 按手柄按键2删除上一个episode
- 机械臂回到初始位置时自动停止录制
- 按 ESC 键退出数据采集系统

### 4. 训练
```bash
./02_train_ros_openpi.sh
```

在 ros_openpi 环境下启动模型训练。

### 5. 推理
```bash
./03_inference_ros_openpi.sh
```

启动推理系统，包括：
- CAN 接口
- ROS2 机械臂控制器 (关节控制模式)
- 手柄控制
- RealSense 相机
- 推理程序

## 脚本功能对比

| 脚本 | 功能 | 控制模式 | 用途 |
|------|------|----------|------|
| `build_ros_openpi.sh` | 编译工作空间 | - | 首次设置必需 |
| `00_eef_control_ros_openpi.sh` | 末端控制 | 末端位置控制 | 手动操作 |
| `00_joint_control_ros_openpi.sh` | 关节控制 | 关节空间控制 | 手动操作 |
| `01_collect_ros_openpi.sh` | 数据采集 | 主从控制 | 示教学习 |
| `02_train_ros_openpi.sh` | 模型训练 | - | 模型训练 |
| `03_inference_ros_openpi.sh` | 模型推理 | 关节控制 | 自动执行 |

## 环境信息

所有脚本都会显示以下环境信息：
- 使用环境: ros_openpi
- Python 版本: 3.11.x
- ROS2 版本: Humble
- Python 路径: mamba 环境中的 python

## 故障排除

### 1. 环境问题
如果遇到 Python 版本不匹配或导入错误：
```bash
# 重新编译工作空间
./build_ros_openpi.sh
```

### 2. 节点启动异常
检查日志输出，常见问题：
- CAN 接口未就绪
- ROS2 环境未正确 source
- Python 路径不正确

### 3. 相机问题
确保 RealSense 相机已连接且驱动正常：
```bash
# 检查相机设备
rs-enumerate-devices
```

### 4. 手柄问题
确保手柄已连接：
```bash
# 检查手柄设备
ls /dev/input/js*
```

## 注意事项

1. **环境一致性**: 确保编译和运行都在 ros_openpi 环境下
2. **权限**: 脚本需要执行权限
3. **硬件**: 需要连接 ARX 机械臂、RealSense 相机和手柄
4. **CAN 接口**: 需要管理员权限设置 CAN 接口

## 与原版脚本的区别

| 项目 | 原版脚本 | ros_openpi 版本 |
|------|----------|----------------|
| 环境管理器 | conda | mamba |
| Python 版本 | 3.10 | 3.11 |
| 环境名称 | act | ros_openpi |
| 激活方式 | `conda activate act` | `mamba activate ros_openpi` |
| 路径调整 | `../` | `../../` |
| 编译检查 | 无 | 强制编译检查 |