#!/bin/bash

# 01_collect_fixed.sh - 数据采集脚本优化版本
# 修复了路径问题、依赖关系问题和错误处理问题

workspace=$(pwd)
shell_type=${SHELL##*/}
shell_exec="exec $shell_type"

# ESC键退出标志
exit_flag=false

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
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

log_step() {
    echo -e "${BLUE}[STEP]${NC} $1"
}

# 键盘监听函数
keyboard_listener() {
    local key
    while true; do
        read -rsn1 key
        if [ "$key" = $'\e' ]; then
            # 检查是否是ESC键（不是ESC序列的一部分）
            read -rsn1 -t 0.1 next_key
            if [ -z "$next_key" ]; then
                echo ""
                log_warn "检测到ESC键，准备退出..."
                exit_flag=true
                break
            fi
        fi
    done
}

# 清理函数 - 关闭所有启动的终端
cleanup_terminals() {
    log_info "开始清理所有启动的终端..."
    
    # 1. 首先关闭数据采集进程（collect.py）
    log_info "关闭数据采集进程..."
    pkill -f "collect.py" 2>/dev/null || true
    sleep 1
    
    # 2. 关闭collect终端窗口
    log_info "关闭collect终端窗口..."
    wmctrl -l | grep "collect" | awk '{print $1}' | xargs -r wmctrl -i -c
    sleep 1
    
    # 3. 关闭其他ROS2相关进程
    log_info "关闭ROS2相关进程..."
    pkill -f "arx_x5_controller" 2>/dev/null || true
    pkill -f "arx_joy" 2>/dev/null || true
    pkill -f "realsense" 2>/dev/null || true
    sleep 1
    
    # 4. 关闭其他终端窗口
    local titles=("can1" "can3" "can6" "lift" "joy" "realsense")
    for title in "${titles[@]}"; do
        log_info "关闭终端窗口: $title"
        wmctrl -l | grep "$title" | awk '{print $1}' | xargs -r wmctrl -i -c
    done
    
    # 5. 最后关闭CAN相关进程
    log_info "关闭CAN相关进程..."
    pkill -f "arx_can" 2>/dev/null || true
    pkill -f "can1\|can3\|can6" 2>/dev/null || true
    
    # 6. 强制终止可能残留的进程
    log_info "清理残留进程..."
    pkill -f "ros2" 2>/dev/null || true
    
    log_info "清理完成"
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

# 检查ROS2话题是否有数据
check_ros2_topic() {
    local topic_name=$1
    local max_attempts=15
    local attempt=0
    
    log_info "检查话题 $topic_name 数据流..."
    
    while [ $attempt -lt $max_attempts ]; do
        # 使用timeout命令限制检查时间，避免长时间阻塞
        if timeout 2 ros2 topic echo "$topic_name" --once > /dev/null 2>&1; then
            log_info "话题 $topic_name 数据流正常"
            return 0
        fi
        sleep 1
        ((attempt++))
    done
    
    log_warn "话题 $topic_name 数据流异常或无数据"
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

# 检查conda环境
check_conda_env() {
    if ! command -v conda &> /dev/null; then
        log_error "Conda未安装或未在PATH中"
        return 1
    fi
    
    if ! conda info --envs | grep -q "act"; then
        log_error "Conda环境 'act' 不存在"
        return 1
    fi
    
    log_info "Conda环境检查通过"
    return 0
}

# 检查Python脚本
check_python_script() {
    local script_path="${workspace}/../act/collect.py"
    if [ ! -f "$script_path" ]; then
        log_error "Python脚本不存在: $script_path"
        return 1
    fi
    
    log_info "Python脚本检查通过"
    return 0
}

# 清理函数
cleanup() {
    log_info "开始清理资源..."
    
    # 终止所有相关进程
    pkill -f "arx_can" 2>/dev/null || true
    pkill -f "arx_x5_controller" 2>/dev/null || true
    pkill -f "arx_joy" 2>/dev/null || true
    pkill -f "realsense" 2>/dev/null || true
    pkill -f "collect.py" 2>/dev/null || true
    
    log_info "清理完成"
}

# 设置清理陷阱
trap cleanup EXIT

# 主函数
main() {
    log_info "开始启动数据采集系统..."
    
    # 检查工作目录
    if [ ! -d "../ARX_CAN" ] || [ ! -d "../ROS2" ] || [ ! -d "../arx_joy" ] || [ ! -d "../realsense" ] || [ ! -d "../act" ]; then
        log_error "工作目录不正确，请确保在正确的目录下运行此脚本"
        exit 1
    fi
    
    # 检查conda环境
    check_conda_env || exit 1
    
    # 检查Python脚本
    check_python_script || exit 1
    
    # 启动CAN接口
    log_step "启动CAN接口..."
    gnome-terminal -t "can1" -x bash -c "cd ${workspace}/../ARX_CAN/arx_can; ./arx_can1.sh; exec bash;"
    sleep 0.3
    gnome-terminal -t "can3" -x bash -c "cd ${workspace}/../ARX_CAN/arx_can; ./arx_can3.sh; exec bash;"
    sleep 0.3
    gnome-terminal -t "can6" -x bash -c "cd ${workspace}/../ARX_CAN/arx_can; ./arx_can6.sh; exec bash;"
    sleep 0.3
    
    # 等待CAN接口就绪
    log_info "等待CAN接口就绪..."
    check_can_ready "can1" || { log_error "CAN1接口启动失败"; exit 1; }
    check_can_ready "can3" || { log_error "CAN3接口启动失败"; exit 1; }
    check_can_ready "can6" || { log_error "CAN6接口启动失败"; exit 1; }
    
    # 启动ROS2系统
    log_step "启动ROS2机械臂控制器..."
    gnome-terminal --title="lift" -x $shell_type -i -c "cd ${workspace}/../ROS2/X5_ws; source install/setup.bash; ros2 launch arx_x5_controller v2_collect.launch.py; $shell_exec"
    sleep 2
    
    # 检查机械臂控制器是否启动（快速检查）
    source ${workspace}/../ROS2/X5_ws/install/setup.bash
    check_ros2_node_fast "/arm_master_l" || { log_warn "左臂控制器节点启动异常"; }
    check_ros2_node_fast "/arm_master_r" || { log_warn "右臂控制器节点启动异常"; }
    
    # 启动手柄控制
    log_step "启动手柄控制..."
    gnome-terminal --title="joy" -x $shell_type -i -c "cd ${workspace}/../arx_joy; source install/setup.bash; ros2 run arx_joy arx_joy; $shell_exec"
    sleep 1
    
    # 检查手柄节点（快速检查）
    check_ros2_node_fast "arx_joy" || { log_warn "手柄节点启动异常"; }
    
    # 启动RealSense相机
    log_step "启动RealSense相机..."
    gnome-terminal --title="realsense" -x $shell_type -i -c "cd ${workspace}/../realsense; ./realsense.sh; $shell_exec"
    sleep 3
    
    # 检查相机节点（快速检查）
    check_ros2_node_fast "/camera/camera_h" || { log_warn "头部相机节点启动异常"; }
    check_ros2_node_fast "/camera/camera_l" || { log_warn "左侧相机节点启动异常"; }
    check_ros2_node_fast "/camera/camera_r" || { log_warn "右侧相机节点启动异常"; }
    
    # 启动数据采集
    log_step "启动数据采集..."
    log_info "数据采集参数:"
    log_info "  - Episode索引: -1 (自动递增)"
    log_info "  - Episode数量: 20"
    log_info "  - 最大时间步: 1800"
    log_info "  - 帧率: 60Hz"
    
    gnome-terminal --title="collect" -x $shell_type -i -c "cd ${workspace}/../act; conda activate act; python collect.py --episode_idx -1 --num_episodes 20 --max_timesteps 1800 --frame_rate 60; $shell_exec"
    
    log_info "数据采集系统启动完成！"
    log_info "可用的ROS2节点："
    ros2 node list 2>/dev/null || log_warn "无法获取ROS2节点列表"
    
    log_info "数据采集说明："
    log_info "1. 等待3秒倒计时后开始录制"
    log_info "2. 按手柄按键0开始录制当前episode"
    log_info "3. 按手柄按键2删除上一个episode"
    log_info "4. 机械臂回到初始位置时自动停止录制"
    log_info "5. 数据将保存到 act/datasets/ 目录"
    log_info "6. 按 ESC 键退出数据采集系统"
    
    # 启动键盘监听
    log_info "启动键盘监听..."
    keyboard_listener &
    local listener_pid=$!
    
    # 等待ESC键退出
    echo ""
    log_info "系统运行中... 按 Ctrl+C 键退出"
    
    # 等待ESC键
    while true; do
        if [ "$exit_flag" = true ]; then
            log_info "正在退出数据采集系统..."
            kill $listener_pid 2>/dev/null || true
            cleanup_terminals
            log_info "数据采集系统已安全退出"
            break
        fi
        sleep 0.1
    done
}

# 错误处理
set -e
trap 'log_error "脚本执行出错，行号: $LINENO"' ERR

# 执行主函数
main "$@"
