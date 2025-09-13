from bimanual import BimanualArm
import numpy as np
from typing import Dict, Any, Tuple
import time
import threading
import signal
import sys
import logging
from dataclasses import dataclass
from enum import Enum


class SafetyLevel(Enum):
    """安全等级枚举"""

    LOW = 1  # 低风险操作
    MEDIUM = 2  # 中等风险操作
    HIGH = 3  # 高风险操作


@dataclass
class SafetyConfig:
    """安全配置类"""

    # 工作空间限制
    workspace_limits = {
        "x": (-0.8, 0.8),
        "y": (-0.8, 0.8),
        "z": (-0.2, 0.8),
    }

    # 安全参数 - 基于你的成功经验优化
    min_height = -0.05
    max_reach = 0.7
    max_speed = 0.5  # 保持你测试成功的速度
    max_acceleration = 0.8  # 稍微提高加速度

    # 控制参数 - 优化控制频率
    control_frequency = 500.0  # 提高控制频率以获得更平滑的运动
    min_duration = 0.3  # 减少最小运动时间
    max_duration = 15.0  # 增加最大运动时间

    # 安全监控
    position_tolerance = 0.01
    velocity_tolerance = 0.05
    max_position_error = 0.05


class SafetyMonitor:
    """安全监控器"""

    def __init__(self, config: SafetyConfig):
        self.config = config
        self.emergency_stop = False
        self.motion_active = False
        self.go_home_requested = False  # 添加回零请求标志
        self.last_positions = {}
        self.last_velocities = {}
        self.error_count = 0
        self.max_errors = 5

        # 设置日志
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)

        # 注册信号处理
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        """信号处理函数 - 支持Ctrl+C回零"""
        if signum == signal.SIGINT:  # Ctrl+C
            self.logger.warning("接收到Ctrl+C信号，准备安全回零...")
            self.emergency_stop = True
            self.motion_active = False
            self.go_home_requested = True  # 请求回零
        else:  # 其他信号
            self.logger.warning(f"接收到信号 {signum}，激活紧急停止!")
            self.emergency_stop = True
            self.motion_active = False
            sys.exit(0)

    def check_emergency_stop(self) -> bool:
        """检查紧急停止状态"""
        return self.emergency_stop

    def check_go_home_request(self) -> bool:
        """检查回零请求"""
        return self.go_home_requested

    def request_go_home(self):
        """请求回零"""
        self.go_home_requested = True
        self.motion_active = False  # 停止当前运动
        self.logger.info("用户请求回零，正在终止当前运动...")

    def clear_go_home_request(self):
        """清除回零请求"""
        self.go_home_requested = False

    def validate_workspace_limits(self, position: np.ndarray, arm_name: str) -> bool:
        """验证工作空间限制"""
        x, y, z = position

        # 检查基本坐标限制
        if not (
            self.config.workspace_limits["x"][0]
            <= x
            <= self.config.workspace_limits["x"][1]
        ):
            self.logger.error(f"{arm_name} X坐标超限: {x:.3f}")
            return False

        if not (
            self.config.workspace_limits["y"][0]
            <= y
            <= self.config.workspace_limits["y"][1]
        ):
            self.logger.error(f"{arm_name} Y坐标超限: {y:.3f}")
            return False

        if not (
            self.config.workspace_limits["z"][0]
            <= z
            <= self.config.workspace_limits["z"][1]
        ):
            self.logger.error(f"{arm_name} Z坐标超限: {z:.3f}")
            return False

        # 检查最低高度
        if z < self.config.min_height:
            self.logger.error(f"{arm_name} 高度过低: {z:.3f}m")
            return False

        # 检查最大伸展距离
        reach = np.sqrt(x**2 + y**2 + z**2)
        if reach > self.config.max_reach:
            self.logger.error(f"{arm_name} 伸展距离过大: {reach:.3f}m")
            return False

        return True

    def validate_motion_speed(
        self, start_pos: np.ndarray, target_pos: np.ndarray, duration: float
    ) -> bool:
        """验证运动速度"""
        if duration <= 0:
            self.logger.error("运动时间必须大于0")
            return False

        if duration < self.config.min_duration:
            self.logger.warning(
                f"运动时间过短: {duration:.2f}s，建议至少 {self.config.min_duration}s"
            )

        if duration > self.config.max_duration:
            self.logger.error(
                f"运动时间过长: {duration:.2f}s，最大 {self.config.max_duration}s"
            )
            return False

        distance = np.linalg.norm(target_pos - start_pos)
        speed = distance / duration

        if speed > self.config.max_speed:
            self.logger.error(
                f"运动速度过快: {speed:.3f}m/s (最大: {self.config.max_speed}m/s)"
            )
            return False

        # 检查加速度
        acceleration = 2 * distance / (duration**2)
        if acceleration > self.config.max_acceleration:
            self.logger.error(
                f"加速度过大: {acceleration:.3f}m/s² (最大: {self.config.max_acceleration}m/s²)"
            )
            return False

        return True

    def validate_quaternion(self, quat: np.ndarray) -> bool:
        """验证四元数"""
        if len(quat) != 4:
            self.logger.error("四元数必须包含4个元素")
            return False

        norm = np.linalg.norm(quat)
        if abs(norm - 1.0) > 0.1:
            self.logger.error(f"四元数无效，模长: {norm:.3f}")
            return False

        return True

    def monitor_motion_safety(self, dual_arm: BimanualArm) -> bool:
        """监控运动安全性"""
        try:
            current_poses = dual_arm.get_ee_pose()

            for arm_name, ee_pose in current_poses.items():
                if ee_pose is None:
                    continue

                current_pos = np.array(ee_pose[:3])

                # 检查工作空间限制
                if not self.validate_workspace_limits(current_pos, arm_name):
                    self.logger.error(f"{arm_name} 超出安全工作空间!")
                    return False

                # 检查位置变化率（速度）
                if arm_name in self.last_positions:
                    dt = 1.0 / self.config.control_frequency
                    velocity = (current_pos - self.last_positions[arm_name]) / dt
                    speed = np.linalg.norm(velocity)

                    if speed > self.config.max_speed:
                        self.logger.error(f"{arm_name} 速度过快: {speed:.3f}m/s")
                        return False

                self.last_positions[arm_name] = current_pos.copy()

            return True

        except Exception as e:
            self.logger.error(f"安全监控异常: {e}")
            self.error_count += 1
            return self.error_count < self.max_errors

    def emergency_stop_monitor(self):
        """紧急停止监控线程"""
        self.logger.info(
            "紧急停止监控已启动 - 按 'q' + Enter 紧急停止，按 'r' + Enter 回零"
        )

        while not self.emergency_stop:
            try:
                user_input = input().strip().lower()
                if user_input == "q":
                    self.logger.warning("用户触发紧急停止!")
                    self.emergency_stop = True
                    self.motion_active = False
                    break
                elif user_input == "r":
                    self.logger.info("用户请求回零!")
                    self.request_go_home()
            except Exception:
                break


class OptimizedDualArmController:
    """优化的双臂控制器"""

    def __init__(self, left_config: Dict[str, Any], right_config: Dict[str, Any]):
        self.safety_config = SafetyConfig()
        self.safety_monitor = SafetyMonitor(self.safety_config)
        self.dual_arm = None
        self.motion_thread = None
        self.ctrl_c_pressed = False  # Ctrl+C标志

        # 初始化机械臂
        self._initialize_arms(left_config, right_config)

        # 设置全局Ctrl+C处理
        self._setup_global_signal_handlers()

    def _setup_global_signal_handlers(self):
        """设置全局信号处理器"""

        def global_signal_handler(signum, frame):
            if signum == signal.SIGINT:  # Ctrl+C
                self.safety_monitor.logger.warning("检测到Ctrl+C，准备安全回零...")
                self.ctrl_c_pressed = True
                self.safety_monitor.emergency_stop = True
                self.safety_monitor.motion_active = False
                self.safety_monitor.go_home_requested = True
            else:
                self.safety_monitor.logger.warning(f"接收到信号 {signum}，紧急停止!")
                self.safety_monitor.emergency_stop = True
                self.safety_monitor.motion_active = False
                sys.exit(0)

        # 注册全局信号处理
        signal.signal(signal.SIGINT, global_signal_handler)
        signal.signal(signal.SIGTERM, global_signal_handler)

    def _initialize_arms(
        self, left_config: Dict[str, Any], right_config: Dict[str, Any]
    ):
        """初始化机械臂"""
        try:
            self.safety_monitor.logger.info("正在初始化双臂...")
            self.dual_arm = BimanualArm(left_config, right_config)
            self.safety_monitor.logger.info("双臂初始化成功!")
        except Exception as e:
            self.safety_monitor.logger.error(f"双臂初始化失败: {e}")
            raise

    def quaternion_slerp(self, q1: np.ndarray, q2: np.ndarray, t: float) -> np.ndarray:
        """四元数球面线性插值"""
        q1 = q1 / np.linalg.norm(q1)
        q2 = q2 / np.linalg.norm(q2)

        dot = np.dot(q1, q2)
        if dot < 0.0:
            q2 = -q2
            dot = -dot

        if dot > 0.9995:
            result = q1 + t * (q2 - q1)
            return result / np.linalg.norm(result)

        theta_0 = np.arccos(np.abs(dot))
        theta = theta_0 * t
        sin_theta = np.sin(theta)
        sin_theta_0 = np.sin(theta_0)

        s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0

        return s0 * q1 + s1 * q2

    def set_ee_pose_smooth(
        self,
        target_poses: Dict[str, Tuple[np.ndarray, np.ndarray]],
        duration: float = 2.0,
        safety_level: SafetyLevel = SafetyLevel.LOW,
    ) -> bool:
        """
        平滑地设置末端执行器位姿 - 优化版本

        Args:
            target_poses: 目标位姿字典
            duration: 运动持续时间
            safety_level: 安全等级
        """
        # 检查紧急停止
        if self.safety_monitor.check_emergency_stop():
            self.safety_monitor.logger.error("紧急停止激活，取消运动")
            return False

        # 验证输入参数
        if not self._validate_target_poses(target_poses):
            return False

        # 验证运动参数
        if not self._validate_motion_parameters(target_poses, duration, safety_level):
            return False

        # 执行平滑运动
        return self._execute_smooth_motion(target_poses, duration)

    def _validate_target_poses(
        self, target_poses: Dict[str, Tuple[np.ndarray, np.ndarray]]
    ) -> bool:
        """验证目标位姿"""
        for arm_name, (position, quaternion) in target_poses.items():
            # 验证位置
            if not self.safety_monitor.validate_workspace_limits(position, arm_name):
                return False

            # 验证四元数
            if not self.safety_monitor.validate_quaternion(quaternion):
                return False

        return True

    def _validate_motion_parameters(
        self,
        target_poses: Dict[str, Tuple[np.ndarray, np.ndarray]],
        duration: float,
        safety_level: SafetyLevel,
    ) -> bool:
        """验证运动参数"""
        # 根据安全等级调整参数
        if safety_level == SafetyLevel.HIGH:
            self.safety_config.max_speed = 10.0
            self.safety_config.max_acceleration = 0.2
        elif safety_level == SafetyLevel.MEDIUM:
            self.safety_config.max_speed = 50.0
            self.safety_config.max_acceleration = 0.5
        else:  # LOW
            self.safety_config.max_speed = 100.0
            self.safety_config.max_acceleration = 1.0

        # 获取当前位姿并验证运动
        current_poses = self.dual_arm.get_ee_pose()
        for arm_name, (target_pos, target_quat) in target_poses.items():
            if arm_name in current_poses:
                current_pose = current_poses[arm_name]
                current_pos = np.array(current_pose[:3])

                if not self.safety_monitor.validate_motion_speed(
                    current_pos, target_pos, duration
                ):
                    return False

        return True

    def _execute_smooth_motion(
        self, target_poses: Dict[str, Tuple[np.ndarray, np.ndarray]], duration: float
    ) -> bool:
        """执行平滑运动 - 优化版本"""
        try:
            self.safety_monitor.motion_active = True
            self.safety_monitor.logger.info(f"开始平滑运动，持续时间: {duration}s")

            # 获取当前位姿
            current_poses = self.dual_arm.get_ee_pose()
            start_poses = {}

            for arm_name in target_poses.keys():
                if arm_name in current_poses:
                    ee_pose = current_poses[arm_name]
                    current_pos = np.array(ee_pose[:3])
                    current_quat = np.array(ee_pose[3:])
                    start_poses[arm_name] = (current_pos, current_quat)

            # 执行插值运动
            dt = 1.0 / self.safety_config.control_frequency
            steps = int(duration * self.safety_config.control_frequency)

            start_time = time.time()

            for step in range(steps + 1):
                # 检查Ctrl+C信号
                if self.ctrl_c_pressed:
                    self.safety_monitor.logger.warning(
                        "检测到Ctrl+C，立即执行安全回零..."
                    )
                    self.safety_monitor.motion_active = False
                    # 立即执行回零
                    try:
                        self.dual_arm.go_home(duration=1.0)
                        self.safety_monitor.logger.info("Ctrl+C回零完成")
                    except Exception as e:
                        self.safety_monitor.logger.error(f"Ctrl+C回零失败: {e}")
                    return False

                # 检查紧急停止
                if self.safety_monitor.check_emergency_stop():
                    self.safety_monitor.logger.warning("运动被紧急停止")
                    return False

                # 检查回零请求
                if self.safety_monitor.check_go_home_request():
                    self.safety_monitor.logger.warning("检测到回零请求，终止当前运动")
                    self.safety_monitor.motion_active = False
                    return False

                # 计算插值因子 - 使用平滑曲线
                t = min(step / steps, 1.0)
                # 使用sigmoid函数获得更平滑的加速和减速
                smooth_t = 1 / (1 + np.exp(-10 * (t - 0.5)))

                # 计算插值位姿
                interpolated_poses = {}
                for arm_name, (target_pos, target_quat) in target_poses.items():
                    if arm_name in start_poses:
                        start_pos, start_quat = start_poses[arm_name]

                        # 位置线性插值
                        interp_pos = start_pos + smooth_t * (target_pos - start_pos)

                        # 四元数球面插值
                        interp_quat = self.quaternion_slerp(
                            start_quat, target_quat, smooth_t
                        )

                        interpolated_poses[arm_name] = (interp_pos, interp_quat)

                # 发送位姿命令
                if interpolated_poses:
                    self.dual_arm.set_ee_pose(interpolated_poses)

                # 安全监控 - 降低监控频率以提高性能
                if (
                    step % int(self.safety_config.control_frequency / 5) == 0
                ):  # 每0.2秒监控一次
                    if not self.safety_monitor.monitor_motion_safety(self.dual_arm):
                        self.safety_monitor.logger.error("安全监控失败，停止运动")
                        return False

                # 时间控制
                elapsed = time.time() - start_time
                target_time = step * dt
                sleep_time = target_time - elapsed

                if sleep_time > 0:
                    time.sleep(sleep_time)

            self.safety_monitor.motion_active = False
            self.safety_monitor.logger.info("平滑运动完成")
            return True

        except Exception as e:
            self.safety_monitor.logger.error(f"运动执行异常: {e}")
            self.safety_monitor.motion_active = False
            return False

    def go_home_safe(self, duration: float = 2.0) -> bool:
        """安全回零"""
        self.safety_monitor.logger.info("开始安全回零...")
        try:
            self.dual_arm.go_home(duration)
            time.sleep(0.5)  # 等待回零完成
            self.safety_monitor.logger.info("回零完成")
            return True
        except Exception as e:
            self.safety_monitor.logger.error(f"回零失败: {e}")
            return False

    def handle_go_home_request(self) -> bool:
        """处理回零请求"""
        if self.safety_monitor.check_go_home_request():
            self.safety_monitor.logger.info("执行用户回零请求...")
            success = self.go_home_safe()
            if success:
                self.safety_monitor.clear_go_home_request()
                self.safety_monitor.logger.info("回零请求处理完成")
            return success
        return True

    def start_emergency_monitor(self):
        """启动紧急监控线程"""
        monitor_thread = threading.Thread(
            target=self.safety_monitor.emergency_stop_monitor
        )
        monitor_thread.daemon = True
        monitor_thread.start()

    def set_catch_positions(self, positions: Dict[str, float]):
        """设置夹爪位置"""
        try:
            self.dual_arm.set_catch_pos(positions)
            self.safety_monitor.logger.info(f"夹爪位置设置: {positions}")
        except Exception as e:
            self.safety_monitor.logger.error(f"夹爪位置设置失败: {e}")

    def open_grippers(self):
        """打开夹爪"""
        self.set_catch_positions({"left": 0.0, "right": 0.0})
        self.safety_monitor.logger.info("夹爪已打开")

    def close_grippers(self):
        """关闭夹爪"""
        self.set_catch_positions({"left": 1.0, "right": 1.0})
        self.safety_monitor.logger.info("夹爪已关闭")

    def get_status(self) -> Dict[str, Any]:
        """获取当前状态"""
        try:
            poses = self.dual_arm.get_ee_pose()
            joint_positions = self.dual_arm.get_joint_positions()
            catch_positions = self.dual_arm.get_catch_pos()
            return {
                "emergency_stop": self.safety_monitor.emergency_stop,
                "motion_active": self.safety_monitor.motion_active,
                "poses": poses,
                "joint_positions": joint_positions,
                "catch_positions": catch_positions,
                "error_count": self.safety_monitor.error_count,
            }
        except Exception as e:
            self.safety_monitor.logger.error(f"获取状态失败: {e}")
            return {"error": str(e)}

    def test_multiple_positions(self):
        """测试多个位置的运动 - 增强版"""
        positions = [
            # 1. 初始位置 - 双臂对称
            {
                "left": (np.array([0.1, 0.2, 0.1]), np.array([1.0, 0.0, 0.0, 0.0])),
                "right": (np.array([0.1, -0.2, 0.1]), np.array([1.0, 0.0, 0.0, 0.0])),
            },
            # 2. 向前伸展 - 保持水平
            {
                "left": (np.array([0.3, 0.15, 0.1]), np.array([1.0, 0.0, 0.0, 0.0])),
                "right": (np.array([0.3, -0.15, 0.1]), np.array([1.0, 0.0, 0.0, 0.0])),
            },
            # 3. 向上抬起 - 添加俯仰旋转（降低角度）
            {
                "left": (
                    np.array([0.25, 0.1, 0.25]),
                    np.array([0.924, 0.0, 0.383, 0.0]),
                ),  # 绕Y轴旋转45度
                "right": (
                    np.array([0.25, -0.1, 0.25]),
                    np.array([0.924, 0.0, -0.383, 0.0]),
                ),  # 绕Y轴旋转-45度
            },
            # 4. 侧向移动 - 添加偏航旋转（降低角度）
            {
                "left": (
                    np.array([0.2, 0.3, 0.2]),
                    np.array([0.924, 0.0, 0.0, 0.383]),
                ),  # 绕Z轴旋转45度
                "right": (
                    np.array([0.2, -0.3, 0.2]),
                    np.array([0.924, 0.0, 0.0, -0.383]),
                ),  # 绕Z轴旋转-45度
            },
            # 5. 对角线运动 - 复合旋转（降低角度）
            {
                "left": (
                    np.array([0.35, 0.2, 0.3]),
                    np.array([0.8, 0.2, 0.2, 0.2]),
                ),  # 复合旋转（降低）
                "right": (
                    np.array([0.35, -0.2, 0.3]),
                    np.array([0.8, -0.2, -0.2, 0.2]),
                ),  # 复合旋转（降低）
            },
            # 6. 圆形轨迹 - 保持高度变化（降低角度）
            {
                "left": (
                    np.array([0.2, 0.25, 0.15]),
                    np.array([0.966, 0.0, 0.0, 0.259]),
                ),  # 绕Z轴旋转30度
                "right": (
                    np.array([0.2, -0.25, 0.15]),
                    np.array([0.966, 0.0, 0.0, -0.259]),
                ),  # 绕Z轴旋转-30度
            },
            # 7. 低位置 - 向下俯视（降低角度）
            {
                "left": (
                    np.array([0.15, 0.1, 0.05]),
                    np.array([0.924, 0.383, 0.0, 0.0]),
                ),  # 绕X轴旋转45度
                "right": (
                    np.array([0.15, -0.1, 0.05]),
                    np.array([0.924, -0.383, 0.0, 0.0]),
                ),  # 绕X轴旋转-45度
            },
            # 8. 高位置 - 向上仰视（降低角度）
            {
                "left": (
                    np.array([0.25, 0.05, 0.35]),
                    np.array([0.924, -0.383, 0.0, 0.0]),
                ),  # 绕X轴旋转-45度
                "right": (
                    np.array([0.25, -0.05, 0.35]),
                    np.array([0.924, 0.383, 0.0, 0.0]),
                ),  # 绕X轴旋转45度
            },
            # 9. 螺旋运动 - 复杂旋转（降低角度）
            {
                "left": (
                    np.array([0.3, 0.0, 0.2]),
                    np.array([0.7, 0.3, 0.3, 0.3]),
                ),  # 复合旋转（降低）
                "right": (
                    np.array([0.3, 0.0, 0.2]),
                    np.array([0.7, -0.3, -0.3, 0.3]),
                ),  # 复合旋转（降低）
            },
            # 10. 回到中心 - 准备下一轮
            {
                "left": (np.array([0.1, 0.0, 0.1]), np.array([1.0, 0.0, 0.0, 0.0])),
                "right": (np.array([0.1, 0.0, 0.1]), np.array([1.0, 0.0, 0.0, 0.0])),
            },
        ]

        for i, target_poses in enumerate(positions):
            # 检查Ctrl+C信号
            if self.ctrl_c_pressed:
                print("检测到Ctrl+C，停止多位置测试并回零...")
                try:
                    self.dual_arm.go_home(duration=1.0)
                    print("Ctrl+C回零完成")
                except Exception as e:
                    print(f"Ctrl+C回零失败: {e}")
                return

            print(f"执行第 {i+1} 个位置...")

            # 根据位置类型调整运动参数
            duration = self._get_duration_for_position(i, target_poses)
            safety_level = self._get_safety_level_for_position(i, target_poses)

            success = self.set_ee_pose_smooth(
                target_poses, duration=duration, safety_level=safety_level
            )

            if success:
                print(
                    f"第 {i+1} 个位置运动完成! (持续时间: {duration}s, 安全等级: {safety_level.name})"
                )

                # 在特定位置添加夹爪控制
                if i in [2, 5, 8]:  # 在关键位置停留并控制夹爪
                    print("在关键位置停留并控制夹爪...")
                    self.open_grippers()
                    time.sleep(0.3)
                    self.close_grippers()
                    time.sleep(0.3)
            else:
                print(f"第 {i+1} 个位置运动失败!")
                # 检查是否是因为回零请求导致的失败
                if self.safety_monitor.check_go_home_request() or self.ctrl_c_pressed:
                    print("检测到回零请求或Ctrl+C，执行回零...")
                    self.handle_go_home_request()
                break

            # 检查回零请求
            if self.safety_monitor.check_go_home_request() or self.ctrl_c_pressed:
                print("检测到回零请求或Ctrl+C，执行回零...")
                self.handle_go_home_request()

            time.sleep(0.2)  # 位置间暂停（降低）

    def _get_duration_for_position(
        self,
        position_index: int,
        target_poses: Dict[str, Tuple[np.ndarray, np.ndarray]],
    ) -> float:
        """根据位置索引和目标位姿动态调整运动持续时间"""
        # 基础持续时间
        base_duration = 1.5

        # 根据位置索引调整
        if position_index in [0, 9]:  # 初始和结束位置
            return base_duration
        elif position_index in [2, 5, 8]:  # 关键位置（旋转较多）
            return base_duration * 1.5  # 更慢，更安全
        elif position_index in [3, 6, 7]:  # 侧向和高低位置
            return base_duration * 1.2
        else:
            return base_duration

    def _get_safety_level_for_position(
        self,
        position_index: int,
        target_poses: Dict[str, Tuple[np.ndarray, np.ndarray]],
    ) -> SafetyLevel:
        """根据位置索引和目标位姿动态调整安全等级"""
        # 检查是否有复杂旋转
        has_complex_rotation = False
        for arm_name, (pos, quat) in target_poses.items():
            # 检查四元数是否接近单位四元数（简单旋转）
            quat_norm = np.linalg.norm(quat)
            if abs(quat_norm - 1.0) > 0.1:
                has_complex_rotation = True
                break

            # 检查是否有非零的旋转分量
            if not np.allclose(quat, [1.0, 0.0, 0.0, 0.0], atol=0.1):
                has_complex_rotation = True
                break

        # 检查高度变化
        height_variation = 0
        for arm_name, (pos, quat) in target_poses.items():
            height_variation = max(
                height_variation, abs(pos[2] - 0.1)
            )  # 相对于基准高度0.1的变化

        # 根据复杂度和高度调整安全等级
        if has_complex_rotation or height_variation > 0.2:
            return SafetyLevel.LOW
        elif position_index in [2, 5, 8]:  # 关键位置
            return SafetyLevel.LOW
        else:
            return SafetyLevel.LOW

    def test_circular_motion(
        self, radius: float = 0.15, height: float = 0.2, num_points: int = 8
    ):
        """测试圆形运动轨迹"""
        print(
            f"开始圆形运动测试 (半径: {radius}m, 高度: {height}m, 点数: {num_points})"
        )

        positions = []
        for i in range(num_points):
            angle = 2 * np.pi * i / num_points

            # 左臂圆形轨迹
            left_x = radius * np.cos(angle) + 0.2
            left_y = radius * np.sin(angle) + 0.1
            left_pos = np.array([left_x, left_y, height])

            # 右臂圆形轨迹（相反方向）
            right_x = radius * np.cos(angle + np.pi) + 0.2
            right_y = radius * np.sin(angle + np.pi) - 0.1
            right_pos = np.array([right_x, right_y, height])

            # 保持末端执行器旋转角度不变，避免奇异点
            # 使用固定的四元数，不随角度变化
            left_quat = np.array([1.0, 0.0, 0.0, 0.0])  # 无旋转
            right_quat = np.array([1.0, 0.0, 0.0, 0.0])  # 无旋转

            positions.append(
                {"left": (left_pos, left_quat), "right": (right_pos, right_quat)}
            )

        # 执行圆形运动
        for i, target_poses in enumerate(positions):
            # 检查Ctrl+C信号
            if self.ctrl_c_pressed:
                print("检测到Ctrl+C，停止圆形运动并回零...")
                try:
                    self.dual_arm.go_home(duration=1.0)
                    print("Ctrl+C回零完成")
                except Exception as e:
                    print(f"Ctrl+C回零失败: {e}")
                return

            print(f"圆形运动点 {i+1}/{num_points}")
            success = self.set_ee_pose_smooth(
                target_poses, duration=1.0, safety_level=SafetyLevel.MEDIUM
            )

            if not success:
                print(f"圆形运动在点 {i+1} 失败")
                break

            # 在特定点控制夹爪
            if i % 3 == 0:  # 每3个点控制一次夹爪
                if i % 6 == 0:
                    self.open_grippers()
                else:
                    self.close_grippers()
                time.sleep(0.2)

            time.sleep(0.1)  # 圆形运动暂停（降低）

    def _euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        """将欧拉角转换为四元数"""
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return np.array([w, x, y, z])

    def test_spiral_motion(
        self,
        radius: float = 0.1,
        height_start: float = 0.1,
        height_end: float = 0.3,
        num_turns: float = 2.0,
        num_points: int = 16,
    ):
        """测试单轴旋转运动轨迹"""
        print("开始单轴旋转测试 (X轴→Y轴→Z轴旋转)")

        # 定义初始位置
        initial_pos_left = np.array([0.15, 0.0, 0.1])
        initial_pos_right = np.array([0.15, -0.0, 0.1])

        positions = []

        # 1. X轴旋转测试：-90度 → +90度 → 回到初始
        print("X轴旋转测试...")
        for angle in [-90, 90, 0]:  # 度
            roll = np.radians(angle)
            pitch = 0.0
            yaw = 0.0
            left_quat = self._euler_to_quaternion(roll, pitch, yaw)
            right_quat = self._euler_to_quaternion(roll, pitch, yaw)

            positions.append(
                {
                    "left": (initial_pos_left.copy(), left_quat),
                    "right": (initial_pos_right.copy(), right_quat),
                }
            )

        # 2. Y轴旋转测试：-20度 → +20度 → 回到初始
        print("Y轴旋转测试...")
        for angle in [-20, 20, 0]:  # 度
            roll = 0.0
            pitch = np.radians(angle)
            yaw = 0.0
            left_quat = self._euler_to_quaternion(roll, pitch, yaw)
            right_quat = self._euler_to_quaternion(roll, pitch, yaw)

            positions.append(
                {
                    "left": (initial_pos_left.copy(), left_quat),
                    "right": (initial_pos_right.copy(), right_quat),
                }
            )

        # 3. Z轴旋转测试：+45度 → -45度 → 回到初始
        print("Z轴旋转测试...")
        for angle in [45, -45, 0]:  # 度
            roll = 0.0
            pitch = 0.0
            yaw = np.radians(angle)
            left_quat = self._euler_to_quaternion(roll, pitch, yaw)
            right_quat = self._euler_to_quaternion(roll, pitch, yaw)

            positions.append(
                {
                    "left": (initial_pos_left.copy(), left_quat),
                    "right": (initial_pos_right.copy(), right_quat),
                }
            )

        # 执行单轴旋转运动
        for i, target_poses in enumerate(positions):
            # 检查Ctrl+C信号
            if self.ctrl_c_pressed:
                print("检测到Ctrl+C，停止单轴旋转测试并回零...")
                try:
                    self.dual_arm.go_home(duration=1.0)
                    print("Ctrl+C回零完成")
                except Exception as e:
                    print(f"Ctrl+C回零失败: {e}")
                return

            # 确定当前测试的轴
            if i < 3:
                axis_name = "X轴"
                angle_deg = [-90, 90, 0][i]
            elif i < 6:
                axis_name = "Y轴"
                angle_deg = [-30, 30, 0][i - 3]
            else:
                axis_name = "Z轴"
                angle_deg = [90, -90, 0][i - 6]

            print(f"{axis_name}旋转测试 {i+1}/9 (角度: {angle_deg}度)")
            success = self.set_ee_pose_smooth(
                target_poses, duration=1.5, safety_level=SafetyLevel.MEDIUM
            )

            if not success:
                print(f"{axis_name}旋转测试在角度 {angle_deg}度 失败")
                break

            time.sleep(0.3)  # 单轴旋转暂停（降低）

    def test_wave_motion(
        self,
        amplitude: float = 0.1,
        frequency: float = 2.0,
        duration: float = 4.0,
        num_points: int = 20,
    ):
        """测试波浪运动轨迹"""
        print(
            f"开始波浪运动测试 (振幅: {amplitude}m, 频率: {frequency}Hz, 持续时间: {duration}s)"
        )

        positions = []
        for i in range(num_points):
            t = i / (num_points - 1) * duration
            wave_offset = amplitude * np.sin(2 * np.pi * frequency * t)

            # 左臂波浪轨迹
            left_x = 0.2 + wave_offset * 0.5
            left_y = 0.1 + wave_offset
            left_z = 0.15 + wave_offset * 0.3
            left_pos = np.array([left_x, left_y, left_z])

            # 右臂波浪轨迹（相位差π）
            right_x = 0.2 - wave_offset * 0.5
            right_y = -0.1 - wave_offset
            right_z = 0.15 - wave_offset * 0.3
            right_pos = np.array([right_x, right_y, right_z])

            # 保持末端执行器旋转角度基本不变，避免奇异点
            # 使用极小的旋转变化，主要保持方向稳定
            roll = wave_offset * 0.1  # 极小幅度
            pitch = wave_offset * 0.1  # 极小幅度
            yaw = wave_offset * 0.2  # 极小幅度

            # 转换为四元数
            left_quat = self._euler_to_quaternion(roll, pitch, yaw)
            right_quat = self._euler_to_quaternion(-roll, -pitch, -yaw)

            positions.append(
                {"left": (left_pos, left_quat), "right": (right_pos, right_quat)}
            )

        # 执行波浪运动
        for i, target_poses in enumerate(positions):
            # 检查Ctrl+C信号
            if self.ctrl_c_pressed:
                print("检测到Ctrl+C，停止波浪运动并回零...")
                try:
                    self.dual_arm.go_home(duration=1.0)
                    print("Ctrl+C回零完成")
                except Exception as e:
                    print(f"Ctrl+C回零失败: {e}")
                return

            print(f"波浪运动点 {i+1}/{num_points}")
            success = self.set_ee_pose_smooth(
                target_poses, duration=0.8, safety_level=SafetyLevel.LOW
            )

            if not success:
                print(f"波浪运动在点 {i+1} 失败")
                break

            time.sleep(0.05)  # 波浪运动暂停（降低）


def test_optimized_dual_arm():
    """优化测试函数"""
    # 检查ROS2环境
    import os

    if not os.environ.get("AMENT_PREFIX_PATH"):
        print("错误: ROS2环境未设置!")
        print("请使用以下方法之一运行脚本:")
        print("1. 使用提供的脚本: ./run_dual_arm_test.sh")
        print("2. 手动设置环境:")
        print("   source /opt/ros/humble/setup.bash")
        print("   source /home/vertax/ROS2_AC-one_Play/ROS2/X5_ws/install/setup.bash")
        print("   python3 test_dual_arm_eef_pos_optimized.py")
        return

    # 配置
    left_arm_config = {
        "can_port": "can1",
        "type": 2,
        "dt": 0.002,
    }
    right_arm_config = {
        "can_port": "can3",
        "type": 2,
        "dt": 0.002,
    }

    try:
        # 创建优化控制器
        controller = OptimizedDualArmController(left_arm_config, right_arm_config)

        # 启动紧急监控
        controller.start_emergency_monitor()

        print("=" * 60)
        print("双臂运动测试开始!")
        print("安全提示:")
        print("- 按 Ctrl+C 可以随时安全回零并退出")
        print("- 按 'q' + Enter 可以紧急停止")
        print("- 按 'r' + Enter 可以回零")
        print("=" * 60)

        # 安全回零
        if not controller.go_home_safe(duration=1.0):
            print("回零失败，退出测试")
            return

        # 初始化夹爪（打开状态）
        print("初始化夹爪...")
        controller.open_grippers()
        time.sleep(0.5)

        print("开始多位置测试...")
        controller.test_multiple_positions()
        print("多位置测试完成!")
        time.sleep(1)

        print("开始圆形运动测试...")
        controller.test_circular_motion(radius=0.12, height=0.2, num_points=12)
        print("圆形运动测试完成!")
        time.sleep(1)

        print("开始单轴旋转测试...")
        controller.test_spiral_motion(
            radius=0.08, height_start=0.1, height_end=0.25, num_turns=0.5, num_points=12
        )
        print("单轴旋转测试完成!")
        time.sleep(1)

        print("开始波浪运动测试...")
        controller.test_wave_motion(
            amplitude=0.08, frequency=1.5, duration=3.0, num_points=15
        )
        print("波浪运动测试完成!")
        time.sleep(1)

        print("进入回零模式...")
        controller.dual_arm.go_home()

        # 监控状态
        # print("进入安全监控模式...")
        # print("提示: 按 'r' + Enter 可以立即回零，按 'q' + Enter 可以紧急停止")
        # while not controller.safety_monitor.check_emergency_stop():
        #     # 检查回零请求
        #     if controller.safety_monitor.check_go_home_request():
        #         print("检测到回零请求，执行回零...")
        #         controller.handle_go_home_request()

        #     print("监控状态...")
        #     status = controller.get_status()
        #     print(f"状态: {status}")
        #     time.sleep(0.01)

    except Exception as e:
        print(f"测试失败: {e}")


if __name__ == "__main__":
    test_optimized_dual_arm()
