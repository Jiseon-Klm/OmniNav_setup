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
import csv
import time
import threading
import cv2
from datetime import datetime
import math

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

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
        
        # CSV records for waypoint data
        self.csv_records = []
        self.total_infer_time = 0.0
        
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
    
    def get_latest_image(self) -> tuple:
        """Get the latest image (thread-safe)
        
        Returns:
            (image, timestamp) or (None, None) if no image available
        """
        with self.image_lock:
            if self.latest_image is not None:
                return self.latest_image.copy(), self.image_timestamp
            return None, None
    
    def publish_action(self, action: dict):
        """Publish first waypoint to /action topic, return full waypoint list for visualization"""
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
        
        # Create full waypoint list (for visualization)
        full_waypoint_list = []
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
        
        # Control: publish only first waypoint
        if len(full_waypoint_list) > 0:
            control_waypoint_list = [full_waypoint_list[0]]
        else:
            print(f"[Frame {self.frame_count:04d}] ERROR: full_waypoint_list is empty!")
            return None

        # Create JSON message
        msg_data = {
            'waypoints': control_waypoint_list,
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
        
        self.action_pub.publish(msg)
        
        # Return full list for visualization
        return full_waypoint_list
        
    def run_loop(self, inference_interval: float = 1.0):
        """Main inference loop - Step-by-Step Mode"""
        
        # [설정] 로봇이 실제로 움직일 시간 (제어 노드와 맞춰야 함)
        ROBOT_MOVE_DURATION = 1.0
        
        # PREDICT_SCALE (same as run_infer_iphone.py)
        PREDICT_SCALE = 0.3
        
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
                # 1. 이미지 획득
                image, timestamp = self.get_latest_image()
                
                if image is None:
                    print("[OmniNav Online] No image available, waiting...")
                    time.sleep(0.1)
                    continue
                
                # 2. Create observations (inline, same as run_infer_iphone.py)
                default_pose = {
                    'position': [0.0, 0.0, 0.0],
                    'rotation': [1.0, 0.0, 0.0, 0.0]
                }
                obs = {
                    'front': image,
                    'left': image.copy(),
                    'right': image.copy(),
                    'rgb': image,
                    'instruction': {'text': self.instruction},
                    'pose': default_pose
                }
                
                # 3. Create info (minimal for online mode)
                info = {
                    'top_down_map_vlnce': None,
                    'gt_map': None,
                    'pred_map': None,
                }
                
                # 4. Run inference (direct agent.act call, same as run_infer_iphone.py)
                start_time = time.time()
                with torch.no_grad():
                    action = self.agent.act(obs, info, "online_session")
                infer_time = time.time() - start_time
                self.total_infer_time += infer_time
                self.frame_count += 1
                
                print(f"[Frame {self.frame_count:04d}] Inference time: {infer_time:.3f}s")
                
                # 5. Save CSV records (all 5 waypoints, same format as run_infer_iphone.py)
                if 'arrive_pred' in action and 'action' in action and 'recover_angle' in action:
                    arrive = int(action['arrive_pred'])
                    waypoints = action['action']  # shape (5, 2), scale already applied
                    recover_angles = action['recover_angle']  # shape (5,)
                    
                    # Flatten if needed
                    if isinstance(waypoints, np.ndarray) and waypoints.ndim > 1:
                        waypoints = waypoints.reshape(-1, 2)
                    if isinstance(recover_angles, np.ndarray) and recover_angles.ndim > 1:
                        recover_angles = recover_angles.flatten()
                    
                    # Save each of 5 waypoints as CSV record
                    for subframe_idx in range(min(5, len(waypoints))):
                        # Restore to original scale (/ PREDICT_SCALE)
                        dx = waypoints[subframe_idx][0] / PREDICT_SCALE
                        dy = waypoints[subframe_idx][1] / PREDICT_SCALE
                        dtheta = np.degrees(recover_angles[subframe_idx]) if subframe_idx < len(recover_angles) else 0.0
                        
                        self.csv_records.append({
                            'frame_idx': self.frame_count,
                            'subframe_idx': subframe_idx,
                            'dx': float(dx),
                            'dy': float(dy),
                            'dtheta': float(dtheta),
                            'arrive': arrive,
                            'infer_time_s': float(infer_time) if subframe_idx == 0 else 0.0
                        })
                    
                    # Simple log output (first waypoint only)
                    wp = waypoints[0]
                    dtheta0 = np.degrees(recover_angles[0]) if len(recover_angles) > 0 else 0.0
                    print(f"  -> arrive={arrive}, dtheta={dtheta0:.2f}, wp[0]=({wp[0]:.4f}, {wp[1]:.4f})")
                
                # 6. 액션 퍼블리시 (로봇 출발 신호)
                waypoint_list = self.publish_action(action)
                
                # 7. 시각화
                if waypoint_list is not None and len(waypoint_list) > 0:
                    vis_image = draw_waypoint_arrows_fpv(image, waypoint_list)
                    self.vis_frame_list.append(vis_image)
                else:
                    self.vis_frame_list.append(image.copy())
                
                # 8. 로봇 이동 대기
                if action.get('arrive_pred', 0) == 0:
                    print(f"  >> Moving robot for {ROBOT_MOVE_DURATION}s...")
                    time.sleep(ROBOT_MOVE_DURATION)
                else:
                    print("\n" + "=" * 60)
                    print("[OmniNav Online] ARRIVED! Navigation complete.")
                    print("=" * 60)
                    break
                    
        except KeyboardInterrupt:
            print("\n[OmniNav Online] Stopping...")
        finally:
            self.shutdown()


    def shutdown(self):
        """Clean up resources and save video/CSV"""
        print("[OmniNav Online] Shutting down...")
        
        # Save CSV results (same format as run_infer_iphone.py)
        if len(self.csv_records) > 0:
            os.makedirs(self.result_path, exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            csv_file = os.path.join(self.result_path, f"waypoint_data_online_{timestamp}.csv")
            
            with open(csv_file, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=['frame_idx', 'subframe_idx', 'dx', 'dy', 'dtheta', 'arrive', 'infer_time_s'])
                writer.writeheader()
                writer.writerows(self.csv_records)
            
            print(f"[OmniNav Online] CSV saved: {csv_file} ({len(self.csv_records)} records, {len(self.csv_records)//5} frames)")
        
        # Print inference time statistics
        if self.frame_count > 0:
            avg_infer_time = self.total_infer_time / self.frame_count
            print(f"\n{'='*60}")
            print(f"[TIMING] Total inference time: {self.total_infer_time:.3f}s")
            print(f"[TIMING] Average inference time per frame: {avg_infer_time:.3f}s")
            print(f"[TIMING] FPS: {1.0/avg_infer_time:.2f}" if avg_infer_time > 0 else "[TIMING] FPS: N/A")
            print(f"{'='*60}\n")
        
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
