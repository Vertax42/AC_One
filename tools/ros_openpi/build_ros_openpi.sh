#!/bin/bash

# build_ros_openpi.sh - 在ros_openpi环境下重新编译ROS2工作空间
# 确保Python版本一致性

workspace=$(pwd)
shell_type=${SHELL##*/}

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

# 检查mamba环境
check_mamba_env() {
    if ! command -v mamba &> /dev/null; then
        log_error "Mamba未安装或未在PATH中"
        return 1
    fi

    if ! mamba info --envs | grep -q "ros_openpi"; then
        log_error "Mamba环境 'ros_openpi' 不存在"
        return 1
    fi

    log_info "Mamba环境检查通过"
    return 0
}

# 激活环境并检查Python版本
check_python_version() {
    log_info "检查ros_openpi环境中的Python版本..."

    # 临时激活环境检查Python版本
    eval "$(mamba shell hook --shell bash)"
    mamba activate ros_openpi

    local python_version=$(python --version 2>&1)
    log_info "当前Python版本: $python_version"

    if [[ $python_version =~ "Python 3.11" ]]; then
        log_info "Python版本检查通过"
        return 0
    else
        log_warn "Python版本不是3.11，但继续编译"
        return 0
    fi
}

# 清理旧的编译文件
clean_workspace() {
    log_step "清理旧的编译文件..."

    cd ../../ROS2/X5_ws

    if [ -d "build" ]; then
        log_info "删除build目录..."
        rm -rf build/
    fi

    if [ -d "install" ]; then
        log_info "删除install目录..."
        rm -rf install/
    fi

    if [ -d "log" ]; then
        log_info "删除log目录..."
        rm -rf log/
    fi

    log_info "清理完成"
}

# 在ros_openpi环境下编译
build_workspace() {
    log_step "在ros_openpi环境下编译ROS2工作空间..."

    cd ../../ROS2/X5_ws

    # 激活mamba环境
    # eval "$(mamba shell.bash hook)"
    eval "$(mamba shell hook --shell bash)"
    log_info "激活ros_openpi环境..."
    mamba activate ros_openpi

    # 检查ROS2环境
    if ! command -v colcon &> /dev/null; then
        log_error "colcon未找到，请确保ros_openpi环境包含ROS2开发工具"
        return 1
    fi

    # 编译
    log_info "开始编译..."
    log_info "使用的Python: $(which python)"
    log_info "Python版本: $(python --version)"

    if colcon build --symlink-install; then
        log_info "编译成功！"
        return 0
    else
        log_error "编译失败！"
        return 1
    fi
}

# 验证编译结果
verify_build() {
    log_step "验证编译结果..."

    cd ../../ROS2/X5_ws

    # 激活环境
    eval "$(mamba shell hook --shell bash)"
    mamba activate ros_openpi

    # source setup
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
        log_info "成功source setup.bash"
    else
        log_error "setup.bash不存在！"
        return 1
    fi

    # 检查包是否可用
    if ros2 pkg list | grep -q "arx_x5_controller"; then
        log_info "arx_x5_controller包可用"
    else
        log_warn "arx_x5_controller包未找到"
    fi

    # 检查Python路径
    log_info "检查编译后的Python路径..."
    grep -r "python3.11" install/ | head -3 || log_warn "未找到Python3.11相关路径"

    log_info "验证完成"
}

# 主函数
main() {
    log_info "开始在ros_openpi环境下重新编译ROS2工作空间..."

    # 检查工作目录
    if [ ! -d "../../ROS2/X5_ws" ]; then
        log_error "ROS2工作空间不存在: ../../ROS2/X5_ws"
        exit 1
    fi

    # 检查环境
    check_mamba_env || exit 1
    check_python_version || exit 1

    # 询问是否清理
    echo ""
    log_warn "注意：这将删除现有的build、install和log目录"
    read -p "是否继续？(y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        log_info "操作已取消"
        exit 0
    fi

    # 执行编译流程
    clean_workspace || exit 1
    build_workspace || exit 1
    verify_build || exit 1

    log_info ""
    log_info "============================"
    log_info "编译完成！"
    log_info "现在可以使用ros_openpi环境运行ROS2节点了"
    log_info "============================"
    log_info ""
    log_info "下一步："
    log_info "1. 使用 ./01_collect_ros_openpi.sh 进行数据采集"
    log_info "2. 使用 ./02_train_ros_openpi.sh 进行训练"
    log_info "3. 使用 ./03_inference_ros_openpi.sh 进行推理"
}

# 错误处理
set -e
trap 'log_error "脚本执行出错，行号: $LINENO"' ERR

# 执行主函数
main "$@"
