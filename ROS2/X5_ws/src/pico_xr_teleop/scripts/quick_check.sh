#!/bin/bash

# PICO XR Quick Connection Check Script

echo "=== PICO XR Connection Status Check ==="
echo

# Check if ROS2 is running
if ! pgrep -f "ros2" > /dev/null; then
    echo "❌ ROS2 is not running"
    exit 1
fi

# Check if pico_xr_node is running
if ! pgrep -f "pico_xr_node" > /dev/null; then
    echo "❌ pico_xr_node is not running"
    echo "   Start it with: ros2 launch pico_xr_teleop pico_xr_teleop.launch.py"
    exit 1
fi

echo "✅ pico_xr_node is running"

# Check topics
echo
echo "=== Available Topics ==="
ros2 topic list | grep pico_xr

echo
echo "=== Topic Data Check ==="

# Check left controller
echo -n "Left Controller: "
if timeout 2s ros2 topic echo /pico_xr/left_controller/pose --once > /dev/null 2>&1; then
    echo "✅ Receiving data"
else
    echo "❌ No data or timeout"
fi

# Check right controller
echo -n "Right Controller: "
if timeout 2s ros2 topic echo /pico_xr/right_controller/pose --once > /dev/null 2>&1; then
    echo "✅ Receiving data"
else
    echo "❌ No data or timeout"
fi

# Check headset
echo -n "Headset: "
if timeout 2s ros2 topic echo /pico_xr/headset/pose --once > /dev/null 2>&1; then
    echo "✅ Receiving data"
else
    echo "❌ No data or timeout"
fi

echo
echo "=== Recent Log Messages ==="
# Show recent log messages from pico_xr_node
journalctl --user -u ros2* -n 20 2>/dev/null | grep -i "pico\|xr" | tail -5 || echo "No recent logs found"

echo
echo "=== Quick Data Sample ==="
echo "Left Controller Pose (last 3 seconds):"
timeout 3s ros2 topic echo /pico_xr/left_controller/pose --once 2>/dev/null || echo "No data received"

echo
echo "=== Connection Check Complete ==="
