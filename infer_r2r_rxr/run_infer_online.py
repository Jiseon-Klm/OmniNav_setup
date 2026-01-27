#!/usr/bin/env python3
"""
OmniNav Real-time Inference with RealSense Camera
Subscribes to RealSense compressed image, runs inference, publishes waypoints to /action
"""
# HPC-X/UCC library conflict prevention (NGC container issue)
import os
import sys

_LD_PRELOAD_LIBS = "/opt/hpcx/ucx/lib/libucs.so.0:/opt/hpcx/ucx/lib/libucp.so.0:/opt/hpcx/ucx/lib/libucm.so.0"
_REEXEC_VAR = "_OMNINAV_REEXEC"

if os.environ.get(_REEXEC_VAR) != "1" and os.path.exists("/opt/hpcx/ucx/lib/libucs.so.0"):
    os.environ["LD_PRELOAD"] = _LD_PRELOAD_LIBS
    os.environ[_REEXEC_VAR] = "1"
    os.execv(sys.executable, [sys.executable] + sys.argv)

import numpy as np
import argparse
import torch
import json
import time
import threading
import cv2
from datetime import datetime


# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import math

from agent.waypoint_agent import Waypoint_Agent


def load_rgb_image_from_array(img_bgr: np.ndarray) -> np.ndarray:
    """Load RGB image from BGR array (identical to run_infer_iphone.py's load_rgb_image)
    
    Args:
        img_bgr: BGR image array from cv2.imdecode()
        
    Returns:
        RGB image array
    """
    if img_bgr is None:
        raise ValueError("Failed to decode image: img_bgr is None")
    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    return img_rgb

import math  # 상단 import에 math가 없으면 추가해주세요

def draw_waypoint_arrows_fpv(
    img: np.ndarray,
    waypoints: list,
    arrow_color: tuple = (0, 255, 0),
    arrow_thickness: int = 3,
    tipLength: float = 0.3,
    stop_color: tuple = (255, 0, 0),
    stop_radius: int = 10,
    scale_factor: float = 0.3,     # [설정] 네트워크 출력값 스케일링
    vis_scale: float = 150.0       # [설정] 미터 -> 픽셀 변환 비율 (화면 크기에 맞춰 조절)
) -> np.ndarray:
    """
    [수정됨] Dead Reckoning 로직 적용 + OpenCV 네이티브 드로잉
    """
    out = img.copy()
    h, w = out.shape[:2]
    
    # 시작점: 화면 하단 중앙
    start_pixel = (w // 2, h - 30)
    
    # 로컬 누적 좌표 초기화 (항상 0,0,0에서 시작)
    Gx, Gy, Gtheta = 0.0, 0.0, 0.0
    prev_pixel = start_pixel

    for i, wp in enumerate(waypoints):
        dx_net = wp.get('dx', 0.0)
        dy_net = wp.get('dy', 0.0)
        dtheta_deg = wp.get('dtheta', 0.0)
        arrive = wp.get('arrive', 0)

        # 1. Body Frame 변환 (x:전방, y:좌측)
        dx_net_scaled = dx_net * scale_factor
        dy_net_scaled = dy_net * scale_factor
        
        x_body = dy_net_scaled
        y_body = -dx_net_scaled

        # 2. 로컬 적분 (Dead Reckoning)
        delta_Gx = x_body * math.cos(Gtheta) - y_body * math.sin(Gtheta)
        delta_Gy = x_body * math.sin(Gtheta) + y_body * math.cos(Gtheta)

        Gx += delta_Gx
        Gy += delta_Gy
        Gtheta += math.radians(dtheta_deg)

        # 3. 화면 좌표 매핑 (전방->위쪽, 좌측->왼쪽)
        screen_x = int(start_pixel[0] - (Gy * vis_scale))
        screen_y = int(start_pixel[1] - (Gx * vis_scale))
        
        # 화면 밖으로 나가는 것 방지
        screen_x = np.clip(screen_x, 0, w - 1)
        screen_y = np.clip(screen_y, 0, h - 1)
        curr_pixel = (screen_x, screen_y)

        if arrive > 0:
            cv2.circle(out, curr_pixel, stop_radius, stop_color, -1)
            cv2.putText(out, "STOP", (curr_pixel[0]-20, curr_pixel[1]-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            break 
        else:
            # 이동량이 너무 작으면 화살표 그리기 생략 (에러 방지)
            if np.linalg.norm(np.array(curr_pixel) - np.array(prev_pixel)) > 2:
                cv2.arrowedLine(out, prev_pixel, curr_pixel, arrow_color, arrow_thickness, 
                                tipLength=tipLength, line_type=cv2.LINE_AA)
            
        prev_pixel = curr_pixel

    return out

class OmniNavOnlineInference:
    """Real-time OmniNav inference with RealSense camera"""
    
    def __init__(self, model_path: str, instruction: str, result_path: str = "./data/result_online"):
        """
        Args:
            model_path: Path to OmniNav model
            instruction: Navigation instruction text
            result_path: Path to save results
        """
        self.instruction = instruction
        self.result_path = result_path
        
        # Image buffer (stores only the latest image)
        self.latest_image = None
        self.image_lock = threading.Lock()
        self.image_timestamp = None
        
        # Frame counter
        self.frame_count = 0
        
        # Visualization storage
        self.vis_frame_list = []  # Store visualized frames for video
        self.save_video = True  # Save video on shutdown
        
        # Initialize ROS2
        rclpy.init()
        self.ros_node = rclpy.create_node('omninav_inference')
        
        # QoS profile for RealSense image subscription
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1  # Keep only latest image
        )
        
        # Image subscriber
        self.image_sub = self.ros_node.create_subscription(
            CompressedImage,
            '/camera/camera/color/image_raw/compressed',
            self._image_callback,
            qos_profile
        )
        
        # Action publisher (waypoints as JSON string)
        self.action_pub = self.ros_node.create_publisher(
            String,
            '/action',
            qos_profile
        )
        
        print("=" * 60)
        print("[OmniNav Online] ROS2 Node Ready")
        print(f"[OmniNav Online] Subscribing to: /camera/camera/color/image_raw/compressed")
        print(f"[OmniNav Online] Publishing to: /action")
        print(f"[OmniNav Online] Instruction: {instruction}")
        print("=" * 60)
        
        # Initialize OmniNav agent
        # Use system temp directory for agent (it needs result_path but we don't use its outputs)
        # We only save video to result_path, not agent's internal folders
        import tempfile
        temp_agent_path = tempfile.mkdtemp(prefix="omninav_agent_")
        
        print("[OmniNav Online] Loading model...")
        self.agent = Waypoint_Agent(model_path, temp_agent_path, require_map=False)
        self.agent.reset()
        self.agent.episode_id = "online_session"
        print("[OmniNav Online] Model loaded successfully")
        
        # Start ROS2 spin thread
        self.spin_thread = threading.Thread(target=self._spin_ros, daemon=True)
        self.spin_thread.start()
    
    def _image_callback(self, msg: CompressedImage):
        """Callback for compressed image - stores only the latest image
        
        Identical image processing to run_infer_iphone.py's load_rgb_image()
        """
        try:
            # Decode compressed image (same as cv2.imread() in run_infer_iphone.py)
            np_arr = np.frombuffer(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if img_bgr is not None and img_bgr.size > 0:
                # Convert BGR to RGB (identical to run_infer_iphone.py's load_rgb_image)
                img_rgb = load_rgb_image_from_array(img_bgr)
                
                # Store latest image (thread-safe)
                with self.image_lock:
                    self.latest_image = img_rgb
                    self.image_timestamp = time.time()
            else:
                print(f"[OmniNav Online] Warning: Decoded image is None or empty")
        except Exception as e:
            print(f"[OmniNav Online] Image decode error: {e}")
            import traceback
            traceback.print_exc()
    
    def _spin_ros(self):
        """ROS2 spin in background thread"""
        rclpy.spin(self.ros_node)
    
    def _create_observations(self, rgb_image: np.ndarray) -> dict:
        """Create observations dictionary mimicking simulator format
        
        Identical to run_infer_iphone.py's create_fake_observations()
        """
        # Default pose (no odometry in online mode)
        # Same structure as run_infer_iphone.py
        default_pose = {
            'position': [0.0, 0.0, 0.0],
            'rotation': [1.0, 0.0, 0.0, 0.0]
        }
        
        # Identical structure to run_infer_iphone.py's create_fake_observations()
        observations = {
            'front': rgb_image,
            'left': rgb_image.copy(),  # Copy from front for now (same as run_infer_iphone.py)
            'right': rgb_image.copy(),  # Copy from front for now (same as run_infer_iphone.py)
            'rgb': rgb_image,
            'instruction': {'text': self.instruction},
            'pose': default_pose
        }
        
        return observations
    
    def _create_info(self) -> dict:
        """Create info dictionary (minimal, no topdown map in online mode)"""
        return {
            'top_down_map_vlnce': None,
            'gt_map': None,
            'pred_map': None,
        }
    
    def get_latest_image(self) -> tuple:
        """Get the latest image (thread-safe)
        
        Returns:
            (image, timestamp) or (None, None) if no image available
        """
        with self.image_lock:
            if self.latest_image is not None:
                return self.latest_image.copy(), self.image_timestamp
            return None, None
    
    def run_inference(self, rgb_image: np.ndarray) -> dict:
        """Run OmniNav inference on a single image
        
        This is the core inference loop, identical to run_infer_iphone.py
        
        Args:
            rgb_image: RGB image (H, W, 3)
            
        Returns:
            action dictionary with waypoints
        """
        # Create observations (identical to run_infer_iphone.py)
        obs = self._create_observations(rgb_image)
        
        # Create info (minimal for online mode)
        info = self._create_info()
        
        # Run inference (identical to run_infer_iphone.py)
        start_time = time.time()
        with torch.no_grad():
            action = self.agent.act(obs, info, "online_session")
        infer_time = time.time() - start_time
        
        self.frame_count += 1
        
        return action, infer_time
    
    def publish_action(self, action: dict):
        """
        [수정됨] 첫 번째 웨이포인트만 추출하여 /action 토픽으로 발행
        """
        if 'arrive_pred' not in action or 'action' not in action or 'recover_angle' not in action:
            print("[OmniNav Online] Invalid action format, skipping publish")
            return None
        
        arrive = int(action['arrive_pred'])
        waypoints = action['action']  # shape (5, 2)
        recover_angles = action['recover_angle']  # shape (5,)
        
        # Flatten if needed
        if isinstance(waypoints, np.ndarray) and waypoints.ndim > 1:
            waypoints = waypoints.reshape(-1, 2)
        if isinstance(recover_angles, np.ndarray) and recover_angles.ndim > 1:
            recover_angles = recover_angles.flatten()
        
        # 1. 전체 웨이포인트 리스트 생성 (시각화용)
        full_waypoint_list = []
        
        # 디버깅: waypoints 정보 출력
        print(f"[Frame {self.frame_count:04d}] DEBUG: waypoints shape={waypoints.shape if isinstance(waypoints, np.ndarray) else type(waypoints)}, len={len(waypoints) if hasattr(waypoints, '__len__') else 'N/A'}")
        
        for i in range(min(5, len(waypoints))):
            dx = float(waypoints[i][0])
            dy = float(waypoints[i][1])
            dtheta = float(np.degrees(recover_angles[i])) if i < len(recover_angles) else 0.0
            
            full_waypoint_list.append({
                'dx': dx,
                'dy': dy,
                'dtheta': dtheta,
                'arrive': arrive
            })
        
        # full_waypoint_list는 항상 5개여야 함
        if len(full_waypoint_list) != 5:
            print(f"[Frame {self.frame_count:04d}] WARNING: full_waypoint_list has {len(full_waypoint_list)} items, expected 5!")
        
        # 2. [핵심 수정] 제어용으로는 '첫 번째' 웨이포인트만 리스트에 담음
        if len(full_waypoint_list) > 0:
            control_waypoint_list = [full_waypoint_list[0]]
        else:
            control_waypoint_list = []
            print(f"[Frame {self.frame_count:04d}] ERROR: full_waypoint_list is empty!")

        # Create JSON message (제어용 리스트 전송)
        msg_data = {
            'waypoints': control_waypoint_list,  # 여기에는 1개만 들어감
            'arrive_pred': arrive,
            'timestamp': time.time(),
            'frame_count': self.frame_count
        }
        
        msg = String()
        msg.data = json.dumps(msg_data)
        
        # Check subscription count before publishing
        subscription_count = self.action_pub.get_subscription_count()
        if subscription_count == 0:
            print(f"[OmniNav Online] WARNING: No subscribers for /action topic! (frame {self.frame_count})")
        else:
            print(f"[OmniNav Online] Publishing to {subscription_count} subscriber(s)")
        
        self.action_pub.publish(msg)
        
        # Log
        if control_waypoint_list:
            wp = control_waypoint_list[0]
            print(f"[Frame {self.frame_count:04d}] Published 1st Action: arrive={arrive}, "
                  f"dx={wp['dx']:.3f}, dy={wp['dy']:.3f}, dtheta={wp['dtheta']:.1f}°")
        
        # 리턴은 전체 리스트를 반환해서 화면에는 5개 화살표가 다 보이게 함 (디버깅용)
        return full_waypoint_list
        
    def run_loop(self, inference_interval: float = 1.0):
        """Main inference loop - Step-by-Step Mode"""
        
        # [설정] 로봇이 실제로 움직일 시간 (제어 노드와 맞춰야 함)
        # 이 시간이 지나야 다음 이미지를 찍습니다.
        ROBOT_MOVE_DURATION = 1.0  
        
        print(f"\n[OmniNav Online] Starting Step-by-Step Loop (Move Duration: {ROBOT_MOVE_DURATION}s)")
        print("[OmniNav Online] Waiting for first image...")
        
        # Wait for first image
        while True:
            image, timestamp = self.get_latest_image()
            if image is not None:
                print(f"[OmniNav Online] First image received! Shape: {image.shape}")
                break
            time.sleep(0.1)
        
        print("[OmniNav Online] Running inference loop. Press Ctrl+C to stop.")
        print("=" * 60)
        
        try:
            while True:
                # ---------------------------------------------------------
                # 1. 이미지 획득 (항상 최신 이미지를 가져와야 함)
                # ---------------------------------------------------------
                # 로봇이 멈춘 직후의 깨끗한 이미지를 얻기 위해 잠시 대기할 수도 있음
                # time.sleep(0.1) 
                
                image, timestamp = self.get_latest_image()
                
                if image is None:
                    print("[OmniNav Online] No image available, waiting...")
                    time.sleep(0.1)
                    continue
                
                # ---------------------------------------------------------
                # 2. 추론 (Inference)
                # ---------------------------------------------------------
                action, infer_time = self.run_inference(image)
                print(f"[Frame {self.frame_count:04d}] Inference time: {infer_time:.3f}s")
                
                # ---------------------------------------------------------
                # 3. 액션 퍼블리시 (로봇 출발 신호)
                # ---------------------------------------------------------
                waypoint_list = self.publish_action(action)
                
                # ---------------------------------------------------------
                # 4. 시각화 (Visualization) - [수정됨]
                # ---------------------------------------------------------
                if waypoint_list is not None and len(waypoint_list) > 0:
                    # 위에서 만든 함수를 호출해 화살표가 그려진 이미지를 받음
                    vis_image = draw_waypoint_arrows_fpv(image, waypoint_list)
                    self.vis_frame_list.append(vis_image)  # <-- 중요: 그려진 이미지를 저장
                else:
                    self.vis_frame_list.append(image.copy()) # 데이터 없으면 원본 저장     
                # ---------------------------------------------------------
                # 5. [핵심] 로봇 이동 대기 (Sleep)
                # ---------------------------------------------------------
                # 도착(Stop) 상태가 아니라면, 로봇이 움직일 시간을 줍니다.
                if action.get('arrive_pred', 0) == 0:
                    print(f"  >> Moving robot for {ROBOT_MOVE_DURATION}s...")
                    # 로봇이 실제로 움직이는 시간동안 대기 (이 동안은 다음 이미지 안 찍음)
                    time.sleep(ROBOT_MOVE_DURATION)
                    
                else:
                    print("\n" + "=" * 60)
                    print("[OmniNav Online] ARRIVED! Navigation complete.")
                    print("=" * 60)
                    
        except KeyboardInterrupt:
            print("\n[OmniNav Online] Stopping...")
        finally:
            self.shutdown()


    def shutdown(self):
        """Clean up resources and save video"""
        print("[OmniNav Online] Shutting down...")
        
        # Save video if frames were collected
        if self.save_video and len(self.vis_frame_list) > 0:
            self._save_video()
        
        self.agent.reset()
        self.ros_node.destroy_node()
        rclpy.shutdown()
        print("[OmniNav Online] Shutdown complete")
    
    def _save_video(self):
        """Save visualized frames as MP4 video"""
        if len(self.vis_frame_list) == 0:
            print("[OmniNav Online] No frames to save")
            return
        
        # Create video output directory (only create if needed)
        os.makedirs(self.result_path, exist_ok=True)
        
        # Save video directly to result_path (no subdirectories)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        video_path = os.path.join(self.result_path, f"omninav_online_{timestamp}.mp4")
        
        print(f"\n[OmniNav Online] Saving {len(self.vis_frame_list)} frames to {video_path}...")
        
        # Get frame dimensions from first frame
        h, w = self.vis_frame_list[0].shape[:2]
        
        # Define codec and create VideoWriter
        # Use 'mp4v' codec (works on most systems)
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        fps = 1.0  # 1 frame per second (matching inference interval)
        video_writer = cv2.VideoWriter(video_path, fourcc, fps, (w, h))
        
        if not video_writer.isOpened():
            print(f"[OmniNav Online] Warning: Failed to open video writer, trying alternative codec...")
            # Try alternative codec
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            video_writer = cv2.VideoWriter(video_path, fourcc, fps, (w, h))
        
        if not video_writer.isOpened():
            print(f"[OmniNav Online] Error: Failed to create video file")
            return
        
        # Write frames
        for frame in self.vis_frame_list:
            # [수정됨] RGB(Matplotlib/PIL 기준) -> BGR(OpenCV 비디오 기준) 변환
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            video_writer.write(frame_bgr)
            
        
        video_writer.release()
        print(f"[OmniNav Online] Video saved: {video_path}")
        print(f"[OmniNav Online] Video info: {len(self.vis_frame_list)} frames, {w}x{h}, {fps} fps")


def main():
    parser = argparse.ArgumentParser(description='OmniNav Real-time Inference')
    
    parser.add_argument(
        "--model-path",
        type=str,
        required=True,
        help="OmniNav model path"
    )
    
    parser.add_argument(
        "--instruction",
        type=str,
        required=True,
        help="Navigation instruction text"
    )
    
    parser.add_argument(
        "--result-path",
        type=str,
        default="./results",
        help="Result save path"
    )
    
    parser.add_argument(
        "--inference-interval",
        type=float,
        default=1.0,
        help="Time between inferences in seconds (default: 1.0)"
    )
    
    parser.add_argument(
        "--save-video",
        action="store_true",
        default=True,
        help="Save visualization as MP4 video (default: True)"
    )
    
    parser.add_argument(
        "--no-save-video",
        action="store_true",
        help="Disable video saving"
    )
    
    args = parser.parse_args()
    
    # Handle video saving flag
    save_video = args.save_video and not args.no_save_video
    
    # Create inference instance
    inference = OmniNavOnlineInference(
        model_path=args.model_path,
        instruction=args.instruction,
        result_path=args.result_path
    )
    
    # Set video saving flag
    inference.save_video = save_video
    if save_video:
        print("[OmniNav Online] Video saving enabled")
    else:
        print("[OmniNav Online] Video saving disabled")
    
    # Run inference loop
    inference.run_loop(inference_interval=args.inference_interval)


if __name__ == "__main__":
    main()
