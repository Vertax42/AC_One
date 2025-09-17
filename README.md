本项目由[mobile-aloha](https://github.com/MarkFzp/mobile-aloha)和[act-plus-plus](https://github.com/MarkFzp/act-plus-plus)修改而来，感谢[Zipeng Fu](https://zipengfu.github.io/)，[Tony Z. Zhao](https://tonyzhaozh.github.io/)，[Chelsea Finn](https://ai.stanford.edu/~cbfinn/)的开源工作

使用前请先阅读[ARX_all_in_one_readme](https://github.com/ARXroboticsX/ARX_all_in_one_readme)中的手册


# command cli
ros2 topic pub /arm_master_l_status arx5_arm_msg/msg/RobotStatus "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
end_pos: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
joint_pos: [0.0, 0.948, 0.858, -0.573, 0.0, 0.0, 0.5]
joint_vel: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
joint_cur: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
" --once

# 打开夹爪 (gripper=1.0 对应 -3.4)
ros2 topic pub /left_eef_cmd arm_control/msg/PosCmd "
x: 0.3
y: 0.0
z: 0.2
roll: 0.0
pitch: 0.0
yaw: 0.0
gripper: 1.0
" --once
