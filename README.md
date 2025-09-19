# librealsense install
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
mkdir build
cd build
cmake ../ -DBUILD_EXAMPLES=true
sudo make install -j$(nproc)

sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

sudo apt install -y ros-$ROS_DISTRO-diagnostic-updater or 
mamba install -y ros-$ROS_DISTRO-diagnostic-updater

sudo apt install -y ros-$ROS_DISTRO-image-transport-plugins or
mamba install -y ros-$ROS_DISTRO-iamge-transport-plugins

cd realsense
colcon build
./search.sh

# *Dangerous* joint position controller ros2 topic command cli

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

# *Dangerous* eef controller ros2 topic command cli
# gripper pos[0, 1] -> actual =[0, -3.4]
ros2 topic pub /left_eef_cmd arm_control/msg/PosCmd "
x: 0.2
y: 0.0
z: 0.2
roll: 0.0
pitch: 0.0
yaw: 0.0
gripper: 1.0
" --once

# init mamba environment
eval "$(mamba shell hook --shell bash)"
mamba activate ros_openpi

# check ros2 package python version
cd ${ros2_ws}
grep "PYTHON_EXECUTABLE" ./build/*/CMakeCache.txt

# pyttsx3 dependency problem
sudo cp -rf /usr/share/alse ~/miniconda3/envs/ros_openpi/share
sudo cp -rf /usr/lib/x86_64-linux-gnu/alsa-lib ~/miniconda3/envs/ros_openpi/lib
