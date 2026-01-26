#!/bin/bash
# Safe RealSense Camera Launch Script - FOR USE INSIDE CONTAINER
# 컨테이너 내부에서 실행하세요: ./safe_camera_launch_inside.sh

set -e

echo "=========================================="
echo "Safe RealSense Camera Launcher (Container)"
echo "=========================================="

# 1. 기존 RealSense 프로세스 정리
echo "[1/4] Cleaning up existing RealSense processes..."
pkill -9 -f "realsense2_camera" 2>/dev/null || true
pkill -9 -f "rs-" 2>/dev/null || true

# video 디바이스 사용 중인 프로세스 종료
for dev in /dev/video*; do
    if [ -e "$dev" ]; then
        fuser -k "$dev" 2>/dev/null || true
    fi
done
sleep 2

# 2. 카메라 연결 확인
echo "[2/4] Verifying camera connection..."
if ! rs-enumerate-devices --compact 2>&1 | grep -q 'Intel RealSense'; then
    echo "ERROR: RealSense camera not detected!"
    echo "Please check:"
    echo "  - USB cable connection"
    echo "  - Container was started with --privileged and device access"
    echo "  - Try: docker run with -v /dev:/dev --privileged"
    exit 1
fi
echo "  Camera detected successfully"
rs-enumerate-devices --compact 2>&1 | grep "Intel RealSense" | head -1

# 3. video 디바이스 확인
echo "[3/4] Checking video devices..."
if ! ls /dev/video0 >/dev/null 2>&1; then
    echo "ERROR: /dev/video0 not found!"
    echo "Make sure container has access to /dev/video* devices"
    exit 1
fi
echo "  Video devices:"
ls -la /dev/video* 2>/dev/null | head -6

# 4. 카메라 노드 및 compressed republisher 시작
echo "[4/5] Starting RealSense camera node..."
echo ""
echo "Press Ctrl+C to stop the camera"
echo ""

# trap으로 안전한 종료 보장
cleanup() {
    echo ""
    echo "Stopping camera gracefully..."
    pkill -2 -f "realsense2_camera" 2>/dev/null || true
    pkill -2 -f "image_transport" 2>/dev/null || true
    sleep 2
    pkill -9 -f "realsense2_camera" 2>/dev/null || true
    pkill -9 -f "image_transport" 2>/dev/null || true
    echo "Camera stopped."
    exit 0
}
trap cleanup SIGINT SIGTERM

# ROS2 환경 설정
source /opt/ros/jazzy/setup.bash
source /workspace/OmniNav/install/setup.bash 2>/dev/null || true

# 환경변수 설정
export RS2_AC_DISABLE_CONDITIONS_CHECK=1

# 카메라 시작 (백그라운드)
echo "[5/5] Starting camera and compressed image republisher..."
ros2 run realsense2_camera realsense2_camera_node --ros-args \
    -p enable_color:=true \
    -p color_width:=640 \
    -p color_height:=480 \
    -p color_fps:=5 \
    -p depth_module.enable_depth:=false \
    -p enable_infra1:=false \
    -p enable_infra2:=false \
    -p enable_gyro:=false \
    -p enable_accel:=false \
    -p initial_reset:=true \
    -p reconnect_timeout:=5.0 \
    -p wait_for_device_timeout:=10.0 &

# 카메라가 시작될 때까지 대기
sleep 3

# compressed 이미지 republisher 시작 (raw -> compressed)
ros2 run image_transport republish raw compressed \
    --ros-args \
    -r in:=/camera/camera/color/image_raw \
    -r out:=/camera/camera/color/image_raw/compressed &

echo ""
echo "Camera and compressed republisher started!"
echo "Topics available:"
echo "  - /camera/camera/color/image_raw (raw)"
echo "  - /camera/camera/color/image_raw/compressed (compressed)"
echo ""
echo "Waiting for processes..."

# 모든 프로세스가 종료될 때까지 대기
wait

