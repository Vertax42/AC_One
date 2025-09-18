#!/bin/bash

# 02_train_ros_openpi.sh - 训练脚本 (ros_openpi环境版本)

workspace=$(pwd)
shell_type=${SHELL##*/}
shell_exec="exec $shell_type"

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

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

# 清理函数 - 关闭所有启动的终端
cleanup_terminals() {
    log_info "开始清理所有启动的终端..."

    # 1. 首先关闭训练进程
    log_info "关闭训练进程..."
    pkill -f "train.py" 2>/dev/null || true
    sleep 1

    # 2. 关闭终端窗口
    local titles=("train")
    for title in "${titles[@]}"; do
        log_info "关闭终端窗口: $title"
        wmctrl -l | grep "$title" | awk '{print $1}' | xargs -r wmctrl -i -c
    done

    log_info "清理完成"
}

# 手动清理函数
manual_cleanup() {
    log_info "执行手动清理..."
    cleanup_terminals
    log_info "手动清理完成"
}

check_environment() {
    log_info "检查ros_openpi环境..."

    if ! command -v mamba &> /dev/null; then
        log_error "Mamba未安装或未在PATH中"
        return 1
    fi

    if ! mamba info --envs | grep -q "ros_openpi"; then
        log_error "Mamba环境 'ros_openpi' 不存在"
        return 1
    fi

    # 检查ROS2工作空间是否已编译
    if [ ! -f "../../ROS2/X5_ws/install/setup.bash" ]; then
        log_error "ROS2工作空间未编译或编译不完整"
        log_error "请先运行: ./build_ros_openpi.sh"
        return 1
    fi

    # 检查是否是用正确Python版本编译的 - 修复检测逻辑
    if [ -f "../../ROS2/X5_ws/build/arx_x5_controller/CMakeCache.txt" ]; then
        if grep -q "python3.11" ../../ROS2/X5_ws/build/arx_x5_controller/CMakeCache.txt 2>/dev/null; then
            log_info "ROS2工作空间已用Python3.11编译"
        else
            log_warn "ROS2工作空间可能不是用Python3.11编译的"
            log_warn "建议运行: ./build_ros_openpi.sh 重新编译"
        fi
    else
        log_warn "无法检测ROS2工作空间编译版本"
    fi

    if [ ! -f "../../act/train.py" ]; then
        log_error "训练脚本不存在: ../../act/train.py"
        return 1
    fi

    log_info "环境检查通过"
    return 0
}

# 主函数
main() {
    # 检查命令行参数
    if [ "$1" = "--cleanup" ] || [ "$1" = "-c" ]; then
        manual_cleanup
        exit 0
    fi
    
    log_info "开始启动训练系统 (ros_openpi环境)..."

    # 检查工作目录
    if [ ! -d "../../act" ]; then
        log_error "act目录不存在"
        log_warn "继续执行，但可能遇到错误..."
    fi

    # 检查环境
    check_environment || log_warn "环境检查失败，但继续执行..."

    log_step "启动训练..."
    log_info "训练参数:"
    log_info "  - 环境: ros_openpi"
    log_info "  - Episode数量: -1 (全部)"
    log_info "  - 使用GPU加速"

    # 启动训练
    gnome-terminal --title="train" -x $shell_type -i -c "
        eval \"\$(mamba shell hook --shell bash)\";
        mamba activate ros_openpi;
        cd ${workspace}/../../act;
        echo \"激活环境: ros_openpi\";
        echo \"Python版本: \$(python --version)\";
        echo \"Python路径: \$(which python)\";
        echo \"开始训练...\";
        python train.py --num_episodes -1;
        echo \"训练完成！按任意键退出...\";
        read;
        $shell_exec"

    log_info "训练系统启动完成！"
    log_info "环境信息:"
    log_info "  - 使用环境: ros_openpi"
    log_info "  - Python版本: $(python --version)"
    
    # 确保环境激活 - 再次激活以确保ROS2命令可用
    eval "$(mamba shell hook --shell bash)"
    mamba activate ros_openpi
    source ../../ROS2/X5_ws/install/setup.bash
    
    # 检查ROS2版本 - 确保在正确环境中
    if command -v ros2 &> /dev/null; then
        if [ -n "$ROS_DISTRO" ]; then
            log_info "  - ROS2版本: $ROS_DISTRO"
        else
            log_info "  - ROS2状态: 可用 (版本未知)"
        fi
    else
        log_warn "  - ROS2状态: 不可用"
    fi
    
    log_info "训练说明："
    log_info "1. 训练窗口已打开，请查看训练进度"
    log_info "2. 训练完成后会自动保存模型"
    log_info "3. 训练日志会显示在终端中"
    log_info ""
    log_info "清理说明："
    log_info "  - 按 Enter 键将退出脚本并清理所有终端"
    log_info "  - 如需单独清理所有终端，请运行: $0 --cleanup"
    log_info "  - 或使用: $0 -c"
    
    # 保持脚本运行，等待用户输入
    log_info ""
    log_info "按 Enter 键退出脚本并清理所有终端..."
    read -r
    log_info "开始清理所有终端..."
    cleanup_terminals
    log_info "脚本退出，所有终端已清理"
}

# 错误处理 - 移除严格模式
# set -e  # 取消严格模式，允许脚本在遇到错误时继续运行
# trap 'log_error "脚本执行出错，行号: $LINENO"' ERR  # 取消错误陷阱

# 执行主函数
main "$@"