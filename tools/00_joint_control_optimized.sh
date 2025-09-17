#!/bin/bash

# 00_joint_control_optimized.sh - 优化版本
# 启动双臂机械臂的关节空间控制器

workspace=$(pwd)
shell_type=${SHELL##*/}
shell_exec="exec $shell_type"

# 进程跟踪数组
declare -a PIDS=()

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 日志函数
log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}


# 检查CAN接口是否就绪
check_can_ready() {
    local can_interface=$1
    local max_attempts=10
    local attempt=0
    
    log_info "检查CAN接口 $can_interface 状态..."
    
    while [ $attempt -lt $max_attempts ]; do
        if ip link show "$can_interface" > /dev/null 2>&1; then
            if ip link show "$can_interface" | grep -q "UP"; then
                log_info "CAN接口 $can_interface 已就绪"
                return 0
            fi
        fi
        sleep 0.5
        ((attempt++))
    done
    log_error "CAN接口 $can_interface 启动超时"
    return 1
}

# 快速检查ROS2节点是否运行
check_ros2_node_fast() {
    local node_name=$1
    local max_attempts=3
    local attempt=0
    
    log_info "快速检查节点 $node_name..."
    
    while [ $attempt -lt $max_attempts ]; do
        if ros2 node list 2>/dev/null | grep -q "$node_name"; then
            log_info "节点 $node_name 已运行"
            return 0
        fi
        sleep 0.3
        ((attempt++))
    done
    
    log_warn "节点 $node_name 启动异常"
    return 1
}

# 检查ROS2话题是否有数据（保留作为备选）
check_ros2_topic() {
    local topic_name=$1
    local max_attempts=8
    local attempt=0
    
    log_info "检查话题 $topic_name 数据流..."
    
    while [ $attempt -lt $max_attempts ]; do
        # 使用timeout命令限制检查时间，减少到1秒
        if timeout 1 ros2 topic echo "$topic_name" --once > /dev/null 2>&1; then
            log_info "话题 $topic_name 数据流正常"
            return 0
        fi
        sleep 0.5
        ((attempt++))
    done
    
    log_warn "话题 $topic_name 数据流异常或无数据"
    return 1
}

# 检查ROS2节点是否运行（保留作为备用方法）
check_ros2_node() {
    local node_name=$1
    local max_attempts=6
    local attempt=0
    
    log_info "检查ROS2节点 $node_name 状态..."
    
    while [ $attempt -lt $max_attempts ]; do
        if ros2 node list 2>/dev/null | grep -q "$node_name"; then
            log_info "ROS2节点 $node_name 已运行"
            return 0
        fi
        sleep 0.5
        ((attempt++))
    done
    log_error "ROS2节点 $node_name 启动超时"
    return 1
}

# 主函数
main() {
    log_info "开始启动ROS2 Joint Controller..."
    
    # 检查工作目录
    if [ ! -d "../ARX_CAN" ] || [ ! -d "../ROS2" ] || [ ! -d "../arx_joy" ]; then
        log_error "工作目录不正确，请确保在正确的目录下运行此脚本"
        exit 1
    fi
    
    # 启动CAN接口
    log_info "启动CAN接口..."
    gnome-terminal -t "can1" -x bash -c "cd ${workspace}; cd ../ARX_CAN/arx_can; ./arx_can1.sh; exec bash;"
    sleep 0.3
    gnome-terminal -t "can3" -x bash -c "cd ${workspace}; cd ../ARX_CAN/arx_can; ./arx_can3.sh; exec bash;"
    sleep 0.3
    gnome-terminal -t "can6" -x bash -c "cd ${workspace}; cd ../ARX_CAN/arx_can; ./arx_can6.sh; exec bash;"
    sleep 0.3
    
    # 等待CAN接口就绪
    log_info "等待CAN接口就绪..."
    check_can_ready "can1" || { log_error "CAN1接口启动失败"; exit 1; }
    check_can_ready "can3" || { log_error "CAN3接口启动失败"; exit 1; }
    check_can_ready "can6" || { log_error "CAN6接口启动失败"; exit 1; }
    
    # 启动ROS2系统
    log_info "启动ROS2机械臂控制器..."
    gnome-terminal --title="ac_one" -x $shell_type -i -c "cd ${workspace}/../ROS2/X5_ws; source install/setup.bash; ros2 launch arx_x5_controller v2_joint_control.launch.py; $shell_exec"
    sleep 2
    
    # 检查机械臂控制器是否启动（快速检查）
    source ${workspace}/../ROS2/X5_ws/install/setup.bash
    check_ros2_node_fast "/arm_slave_l" || { log_warn "左臂Joint控制器节点启动异常"; }
    check_ros2_node_fast "/arm_slave_r" || { log_warn "右臂Joint控制器节点启动异常"; }
    
    # 启动手柄控制
    log_info "启动手柄控制..."
    gnome-terminal --title="joy" -x $shell_type -i -c "cd ${workspace}/../arx_joy; source install/setup.bash; ros2 run arx_joy arx_joy; $shell_exec"
    sleep 1
    
    # 检查手柄节点（快速检查）
    check_ros2_node_fast "arx_joy" || { log_warn "手柄节点启动异常"; }
    
    log_info "控制系统启动完成！"
    log_info "可用的ROS2节点："
    ros2 node list 2>/dev/null || log_warn "无法获取ROS2节点列表"
}

# 错误处理
set -e
trap 'log_error "脚本执行出错，行号: $LINENO"' ERR

# 执行主函数
main "$@"
