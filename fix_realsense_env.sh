#!/bin/bash
# RealSense ROS2 환경 설정 스크립트
# 컨테이너 내부에서 실행: source fix_realsense_env.sh

echo "Setting up ROS2 environment for RealSense..."

# ROS2 Jazzy 환경 소스
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    echo "✓ ROS2 Jazzy environment loaded"
else
    echo "ERROR: /opt/ros/jazzy/setup.bash not found!"
    exit 1
fi

# 워크스페이스 환경 소스 (있는 경우)
if [ -f /workspace/OmniNav/install/setup.bash ]; then
    source /workspace/OmniNav/install/setup.bash
    echo "✓ Workspace environment loaded"
fi

if [ -f /when2reason_ws/install/setup.bash ]; then
    source /when2reason_ws/install/setup.bash
    echo "✓ Scout Mini workspace environment loaded"
fi

# 환경변수 설정
export RS2_AC_DISABLE_CONDITIONS_CHECK=1
export ROS_DISTRO=jazzy

# 패키지 확인
echo ""
echo "Checking realsense2_camera package..."
if ros2 pkg list | grep -q "realsense2_camera"; then
    echo "✓ realsense2_camera package found"
    ros2 pkg list | grep realsense2_camera
else
    echo "✗ realsense2_camera package NOT found"
    echo ""
    echo "Trying to install..."
    apt-get update && apt-get install -y ros-jazzy-realsense2-camera
    source /opt/ros/jazzy/setup.bash
fi

echo ""
echo "Environment setup complete!"
echo "You can now run: ros2 run realsense2_camera realsense2_camera_node"

