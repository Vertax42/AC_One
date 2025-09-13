# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a robotic manipulation system based on mobile-aloha and act-plus-plus, specifically adapted for ARX robotics. The system consists of three main phases: data collection, training, and inference for robotic manipulation tasks.

## Architecture

The repository is organized into several key components:

- **act/**: Core ACT (Action Chunking with Transformers) implementation
  - `collect.py`: Data collection from robot sensors and cameras
  - `train.py`: Neural network training for imitation learning
  - `inference.py`: Real-time robot control using trained models
  - `robomimic/`: Robotics learning framework integration
  - `detr/`: DETR transformer architecture for vision
  - `utils/`: Utilities for policy, ROS operations, and data handling

- **realsense/**: ROS2 camera integration for Intel RealSense cameras
  - Configured for 3 cameras (head, left, right) with specific serial numbers
  - Provides color and depth streams at 640x480@90fps

- **tools/**: Automation scripts for the complete workflow
  - `01_collect.sh`: Automated data collection setup
  - `02_train.sh`: Training pipeline execution
  - `03_inference.sh`: Inference/deployment setup

- **ARX_CAN/**: CAN bus communication for robot control
- **ROS2/**: ROS2 workspace for robot control systems
- **arx_joy/**: Joystick controller integration

## Dependencies and Environment

The project uses conda environments with Python dependencies listed in `tools/IL/requirements.txt`:
- PyTorch with CUDA support
- ROS2 integration (rclpy)
- Computer vision (opencv-python, torchvision)
- Robotics (mujoco, dm_control)
- Data handling (h5py, numpy==1.26)

## Common Development Commands

### Environment Setup
```bash
# Activate the conda environment
conda activate act

# Install dependencies
pip install -r tools/IL/requirements.txt
```

### Data Collection
```bash
# Full automated collection setup (starts CAN, ROS2, cameras, and collection)
./tools/01_collect.sh

# Manual collection
cd act
python collect.py --episode_idx -1 --num_episodes 20
```

### Training
```bash
# Automated training
./tools/02_train.sh

# Manual training
cd act
python train.py --num_episodes -1
```

### Inference/Deployment
```bash
# Automated inference setup
./tools/03_inference.sh

# Manual inference
cd act
python inference.py
```

### Camera Management
```bash
# Start all RealSense cameras
cd realsense
./realsense.sh
```

## Configuration

### Main Configuration
- `act/data/config.yaml`: Camera topics, arm controllers, and robot base configuration
- Camera topics follow pattern: `/camera/camera_{h,l,r}/color/image_rect_raw/compressed`
- Arm controllers: `/arm_master_{l,r}_status` and `/arm_slave_{l,r}_status`

### Code Style
- Flake8 configuration in `.flake8` with max line length of 120 characters
- Run flake8 for code style checking: `flake8 <file>`

## Key Processes

1. **Data Collection**: Multi-terminal setup launching CAN communication, ROS2 controllers, joystick input, cameras, and data collection simultaneously
2. **Training**: Single-process neural network training using collected demonstration data
3. **Inference**: Real-time robot control using trained models with camera input and robot feedback

## Important Notes

- The system expects specific hardware setup with ARX robots, RealSense cameras, and CAN bus communication
- All scripts use gnome-terminal for multi-process coordination
- Camera serial numbers are hardcoded in realsense.sh for specific hardware setup
- Training and inference require GPU support (CUDA)
- The codebase is primarily in Chinese with some English comments