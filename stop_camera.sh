#!/bin/bash
# Safe RealSense Camera Stop Script
# 카메라를 안전하게 종료하고 리소스를 해제합니다.

CONTAINER_NAME="omninav_container"

echo "Stopping RealSense camera..."

docker exec $CONTAINER_NAME bash -c '
    # 1. SIGINT로 graceful 종료 시도
    pkill -2 -f "realsense2_camera" 2>/dev/null
    sleep 2
    
    # 2. 아직 살아있으면 SIGKILL
    pkill -9 -f "realsense2_camera" 2>/dev/null || true
    pkill -9 -f "rs-" 2>/dev/null || true
    
    # 3. video 디바이스 해제
    for dev in /dev/video*; do
        if [ -e "$dev" ]; then
            fuser -k "$dev" 2>/dev/null || true
        fi
    done
    
    sleep 1
    echo "Camera processes terminated."
' 2>/dev/null

echo "Done. Camera resources released."
echo ""
echo "If you still have issues, try:"
echo "  1. Unplug and replug the USB cable"
echo "  2. Run: sudo modprobe -r uvcvideo && sudo modprobe uvcvideo"

