#!/bin/bash

# 01_collect_ros_openpi.sh - 数据采集脚本 (ros_openpi环境版本)
# 适配robostack ros_openpi环境，确保Python版本一致性

workspace=$(pwd)
shell_type=${SHELL##*/}
shell_exec="exec $shell_type"

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


# 清理函数 - 关闭所有启动的终端
cleanup_terminals() {
    log_info "开始清理所有启动的终端..."

    # 1. 首先关闭数据采集进程
    log_info "关闭数据采集进程..."
    pkill -f "collect.py" 2>/dev/null || true
    sleep 1

    # 2. 关闭终端窗口
    local titles=("collect" "can1" "can3" "can6" "lift" "joy" "realsense")
    for title in "${titles[@]}"; do
        log_info "关闭终端窗口: $title"
        if command -v wmctrl &> /dev/null; then
            # 获取窗口ID并检查是否存在
            local window_ids=$(wmctrl -l 2>/dev/null | grep "$title" | awk '{print $1}')
            if [ -n "$window_ids" ]; then
                echo "$window_ids" | while read -r window_id; do
                    if [ -n "$window_id" ]; then
                        wmctrl -i -c "$window_id" 2>/dev/null || true
                    fi
                done
            else
                log_info "  窗口 $title 不存在或已关闭"
            fi
        else
            log_warn "wmctrl命令不可用，无法自动关闭终端窗口"
            log_warn "请手动关闭终端窗口"
        fi
    done

    # 3. 关闭ROS2相关进程
    log_info "关闭ROS2相关进程..."
    pkill -f "arx_x5_controller" 2>/dev/null || true
    pkill -f "arx_joy" 2>/dev/null || true
    pkill -f "realsense" 2>/dev/null || true
    sleep 1

    # 4. 关闭CAN相关进程
    log_info "关闭CAN相关进程..."
    pkill -f "arx_can" 2>/dev/null || true
    pkill -f "can1\|can3\|can6" 2>/dev/null || true

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

    if [ ! -f "../../act/collect.py" ]; then
        log_error "数据采集脚本不存在: ../../act/collect.py"
        return 1
    fi

    log_info "环境检查通过"
    return 0
}

# 检查Python脚本
check_python_script() {
    local script_path="${workspace}/../../act/collect.py"
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

    pkill -f "arx_can" 2>/dev/null || true
    pkill -f "arx_x5_controller" 2>/dev/null || true
    pkill -f "arx_joy" 2>/dev/null || true
    pkill -f "realsense" 2>/dev/null || true
    pkill -f "collect.py" 2>/dev/null || true

    log_info "清理完成"
}

# 设置清理陷阱 - 注释掉自动清理，避免脚本退出时关闭所有终端
# trap cleanup EXIT

# 手动清理函数
manual_cleanup() {
    log_info "执行手动清理..."
    cleanup_terminals
    log_info "手动清理完成"
}

# 主函数
main() {
    # 检查命令行参数
    if [ "$1" = "--cleanup" ] || [ "$1" = "-c" ]; then
        manual_cleanup
        exit 0
    fi
    
    log_info "开始启动数据采集系统 (ros_openpi环境)..."

    # 检查工作目录
    if [ ! -d "../../ARX_CAN" ] || [ ! -d "../../ROS2" ] || [ ! -d "../../arx_joy" ] || [ ! -d "../../realsense" ] || [ ! -d "../../act" ]; then
        log_error "工作目录不正确，请确保在正确的目录下运行此脚本"
        log_warn "继续执行，但可能遇到错误..."
    fi

    # 检查环境和编译状态
    check_environment || log_warn "环境检查失败，但继续执行..."
    check_python_script || log_warn "Python脚本检查失败，但继续执行..."

    # 启动CAN接口
    log_step "启动CAN接口..."
    gnome-terminal -t "can1" -x bash -c "cd ${workspace}/../../ARX_CAN/arx_can; ./arx_can1.sh; exec bash;"
    sleep 0.3
    gnome-terminal -t "can3" -x bash -c "cd ${workspace}/../../ARX_CAN/arx_can; ./arx_can3.sh; exec bash;"
    sleep 0.3
    gnome-terminal -t "can6" -x bash -c "cd ${workspace}/../../ARX_CAN/arx_can; ./arx_can6.sh; exec bash;"
    sleep 0.3

    # 等待CAN接口就绪
    log_info "等待CAN接口就绪..."
    check_can_ready "can1" || log_warn "CAN1接口启动异常，但继续执行..."
    check_can_ready "can3" || log_warn "CAN3接口启动异常，但继续执行..."
    check_can_ready "can6" || log_warn "CAN6接口启动异常，但继续执行..."

    # 启动ROS2系统 (使用ros_openpi环境)
    log_step "启动ROS2机械臂控制器 (ros_openpi环境)..."
    gnome-terminal --title="lift" -x $shell_type -i -c "
        eval \"\$(mamba shell hook --shell bash)\";
        mamba activate ros_openpi;
        cd ${workspace}/../../ROS2/X5_ws;
        source install/setup.bash;
        echo \"Python版本: \$(python --version)\";
        echo \"ROS2版本: ros2 humble\";
        ros2 launch arx_x5_controller v2_collect.launch.py;
        $shell_exec"
    sleep 3

    # 启动手柄控制 (使用ros_openpi环境)
    log_step "启动手柄控制 (ros_openpi环境)..."
    gnome-terminal --title="joy" -x $shell_type -i -c "
        eval \"\$(mamba shell hook --shell bash)\";
        mamba activate ros_openpi;
        cd ${workspace}/../../arx_joy;
        source install/setup.bash;
        ros2 run arx_joy arx_joy;
        $shell_exec"
    sleep 2

    # 启动RealSense相机 (使用ros_openpi环境)
    log_step "启动RealSense相机 (ros_openpi环境)..."
    gnome-terminal --title="realsense" -x $shell_type -i -c "
        eval \"\$(mamba shell hook --shell bash)\";
        mamba activate ros_openpi;
        cd ${workspace}/../../realsense;
        ./realsense.sh;
        $shell_exec"
    sleep 4

    # 临时激活环境进行检查
    eval "$(mamba shell hook --shell bash)"
    mamba activate ros_openpi
    source ../../ROS2/X5_ws/install/setup.bash

    # 检查节点状态
    check_ros2_node_fast "/arm_master_l" || { log_warn "左臂控制器节点启动异常"; }
    check_ros2_node_fast "/arm_master_r" || { log_warn "右臂控制器节点启动异常"; }
    check_ros2_node_fast "arx_joy" || { log_warn "手柄节点启动异常"; }
    check_ros2_node_fast "/camera/camera_h" || { log_warn "头部相机节点启动异常"; }

    # 启动数据采集 (使用ros_openpi环境)
    log_step "启动数据采集 (ros_openpi环境)..."
    log_info "数据采集参数:"
    log_info "  - Episode索引: -1 (自动递增)"
    log_info "  - Episode数量: 50"
    log_info "  - 最大时间步: 3600"
    log_info "  - 帧率: 60Hz"

    gnome-terminal --title="collect" -x $shell_type -i -c "
        eval \"\$(mamba shell hook --shell bash)\";
        mamba activate ros_openpi;
        cd ${workspace}/../../act;
        echo \"激活环境: ros_openpi\";
        echo \"Python版本: \$(python --version)\";
        echo \"Python路径: \$(which python)\";
        python collect.py --episode_idx -1 --num_episodes 50 --max_timesteps 3600 --frame_rate 60;
        $shell_exec"

    log_info "数据采集系统启动完成！"
    log_info "环境信息:"
    log_info "  - 使用环境: ros_openpi"
    log_info "  - Python版本: $(python --version)"
    
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

    # 检查ROS2节点列表
    log_info "可用的ROS2节点："
    if command -v ros2 &> /dev/null; then
        local node_list=$(ros2 node list 2>/dev/null)
        if [ -n "$node_list" ]; then
            echo "$node_list" | while read -r node; do
                log_info "  - $node"
            done
        else
            log_warn "  - 暂无ROS2节点运行"
        fi
    else
        log_warn "  - ROS2命令不可用，无法获取节点列表"
    fi

    log_info "数据采集说明："
    log_info "1. 等待3秒倒计时后开始录制"
    log_info "2. 按手柄按键0开始录制当前episode"
    log_info "3. 按手柄按键1回到设置的Home位置"
    log_info "4. 按手柄按键2删除上一个episode"
    log_info "5. 机械臂回到初始位置时自动停止录制"
    log_info "6. 数据将保存到 act/datasets/ 目录"
    log_info "7. 按 Ctrl+C 键退出数据采集系统"
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