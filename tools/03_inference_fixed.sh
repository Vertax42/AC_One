#!/bin/bash

# 优化的推理脚本 - 解决内存问题
# 作者: AI Assistant
# 日期: $(date +%Y-%m-%d)

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 日志函数
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

# 获取脚本目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(dirname "$SCRIPT_DIR")"

log_info "=========================================="
log_info "🤖 优化版推理脚本启动"
log_info "=========================================="
log_info "工作目录: $WORKSPACE"
log_info "脚本目录: $SCRIPT_DIR"

# 检查conda环境
check_conda_env() {
    log_info "检查conda环境..."
    if ! command -v conda &> /dev/null; then
        log_error "conda未安装或未在PATH中"
        return 1
    fi
    
    if ! conda info --envs | grep -q "act"; then
        log_error "act环境不存在"
        return 1
    fi
    
    log_success "conda环境检查通过"
    return 0
}

# 检查Python脚本
check_python_script() {
    local script_path="$WORKSPACE/act/inference.py"
    log_info "检查推理脚本..."
    
    if [ ! -f "$script_path" ]; then
        log_error "推理脚本不存在: $script_path"
        return 1
    fi
    
    log_success "推理脚本检查通过"
    return 0
}

# 检查权重文件
check_weights() {
    local weights_dir="$WORKSPACE/act/weights"
    log_info "检查权重文件..."
    
    if [ ! -d "$weights_dir" ]; then
        log_error "权重目录不存在: $weights_dir"
        return 1
    fi
    
    if [ ! -f "$weights_dir/policy_best.ckpt" ]; then
        log_error "默认权重文件不存在: policy_best.ckpt"
        return 1
    fi
    
    if [ ! -f "$weights_dir/dataset_stats.pkl" ]; then
        log_error "统计文件不存在: dataset_stats.pkl"
        return 1
    fi
    
    log_success "权重文件检查通过"
    return 0
}

# 检查系统内存
check_memory() {
    log_info "检查系统内存..."
    local total_mem=$(free -g | awk 'NR==2{print $2}')
    local available_mem=$(free -g | awk 'NR==2{print $7}')
    
    log_info "总内存: ${total_mem}GB, 可用内存: ${available_mem}GB"
    
    if [ "$available_mem" -lt 8 ]; then
        log_warn "可用内存不足8GB，推理可能不稳定"
        log_warn "建议关闭其他程序释放内存"
    fi
    
    return 0
}

# 清理函数
cleanup() {
    log_info "开始清理资源..."
    
    # 关闭推理进程
    pkill -f "inference.py" 2>/dev/null || true
    sleep 1
    
    # 关闭其他ROS2相关进程
    pkill -f "arx_x5_controller" 2>/dev/null || true
    pkill -f "realsense" 2>/dev/null || true
    sleep 1
    
    # 关闭终端窗口
    local titles=("can1" "can3" "lift" "realsense" "inference")
    for title in "${titles[@]}"; do
        wmctrl -l | grep "$title" | awk '{print $1}' | xargs -r wmctrl -i -c
    done
    
    # 关闭CAN相关进程
    pkill -f "arx_can" 2>/dev/null || true
    pkill -f "can1\|can3" 2>/dev/null || true
    
    log_success "清理完成"
}

# 设置信号处理
trap cleanup EXIT INT TERM

# 主函数
main() {
    log_info "开始系统检查..."
    
    # 检查依赖
    check_conda_env || exit 1
    check_python_script || exit 1
    check_weights || exit 1
    check_memory || exit 1
    
    log_success "所有检查通过，开始启动系统..."
    
    # 启动CAN接口
    log_info "启动CAN接口..."
    gnome-terminal -t "can1" -x bash -c "cd $WORKSPACE/ARX_CAN/arx_can; ./arx_can1.sh; exec bash;"
    sleep 0.5
    gnome-terminal -t "can3" -x bash -c "cd $WORKSPACE/ARX_CAN/arx_can; ./arx_can3.sh; exec bash;"
    sleep 0.5
    
    # 启动机械臂控制器
    log_info "启动机械臂控制器..."
    gnome-terminal --title="lift" -x bash -c "cd $WORKSPACE/ROS2/X5_ws; source install/setup.bash; ros2 launch arx_x5_controller x5_v2/v2_joint_control.launch.py; exec bash;"
    sleep 2
    
    # 启动RealSense相机
    log_info "启动RealSense相机..."
    gnome-terminal --title="realsense" -x bash -c "cd $WORKSPACE/realsense; ./realsense.sh; exec bash;"
    sleep 3
    
    # 启动推理程序（temporal_agg模式）
    log_info "启动推理程序..."
    log_info "使用temporal_agg模式: max_publish_step=3600 (约2.73GB内存)"
    log_warn "⚠️  监控内存使用情况，如出现内存不足请降低max_publish_step"
    gnome-terminal --title="inference" -x bash -c "cd $WORKSPACE/act; conda activate act; python inference.py; exec bash;"
    
    log_success "所有组件已启动"
    log_info "按 Ctrl+C 安全退出"
    
    # 等待用户中断
    while true; do
        sleep 1
    done
}

# 运行主函数
main "$@"
