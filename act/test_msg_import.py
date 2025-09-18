#!/usr/bin/env python3

import sys
import os
from pathlib import Path

# 模拟act环境的setup
ROOT = Path(__file__).parent
sys.path.append(str(ROOT))

# 导入setup_loader
from utils.setup_loader import setup_loader

print(f"Python版本: {sys.version}")
print(f"Python路径: {sys.executable}")

# 设置msg加载器
try:
    setup_loader(ROOT)
    print("✅ setup_loader 成功")

    # 测试导入msg文件
    try:
        from arm_control.msg._pos_cmd import PosCmd
        from arm_control.msg._joint_control import JointControl
        from arx5_arm_msg.msg._robot_cmd import RobotCmd
        from arx5_arm_msg.msg._robot_status import RobotStatus
        print("✅ 所有msg文件导入成功")

        # 显示Python版本信息
        py = sys.version_info
        msg_dir = f"msg/{py.major}.{py.minor}"
        print(f"✅ 使用的msg目录: {msg_dir}")

        # 检查类是否正确加载
        print(f"✅ PosCmd类: {PosCmd}")
        print(f"✅ JointControl类: {JointControl}")
        print(f"✅ RobotCmd类: {RobotCmd}")
        print(f"✅ RobotStatus类: {RobotStatus}")

    except ImportError as e:
        print(f"❌ msg文件导入失败: {e}")

except Exception as e:
    print(f"❌ setup_loader失败: {e}")