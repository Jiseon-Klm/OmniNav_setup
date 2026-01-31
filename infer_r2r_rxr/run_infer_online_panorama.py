#!/usr/bin/env python3
"""
OmniNav Real-time Inference with Front/Left/Right Cameras
Subscribes to /cam_front/color/image_raw, /cam_left/color/image_raw, /cam_right/color/image_raw,
runs inference, publishes waypoints to /action.
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
from pathlib import Path
import math

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
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


def get_rgb_frame_folders(rgb_dir):
    """Get RGB frame folder list (sorted). Same structure as run_infer_iphone_panorama.
    Returns list of frame folder paths. Each folder contains frame_XXXX_front.jpg, _left.jpg, _right.jpg
    """
    rgb_path = Path(rgb_dir)
    frame_folders = sorted([d for d in rgb_path.iterdir() if d.is_dir() and d.name.startswith('frame_')])
    return [str(p) for p in frame_folders]


def load_rgb_image(image_path):
    """Load RGB image from path (BGR -> RGB). Same as run_infer_iphone_panorama.load_rgb_image."""
    img_bgr = cv2.imread(image_path)
    if img_bgr is None:
        raise ValueError(f"Failed to load image: {image_path}")
    return cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)


def load_observations_from_frame_folder(frame_folder_path, instruction):
    """Load front, left, right from one frame folder. Same paths as run_infer_iphone_panorama."""
    frame_folder = Path(frame_folder_path)
    frame_name = frame_folder.name  # e.g. "frame_0000"
    front_path = frame_folder / f"{frame_name}_front.jpg"
    left_path = frame_folder / f"{frame_name}_left.jpg"
    right_path = frame_folder / f"{frame_name}_right.jpg"
    front_rgb = load_rgb_image(str(front_path))
    left_rgb = load_rgb_image(str(left_path))
    right_rgb = load_rgb_image(str(right_path))
    default_pose = {'position': [0.0, 0.0, 0.0], 'rotation': [1.0, 0.0, 0.0, 0.0]}
    return {
        'front': front_rgb,
        'left': left_rgb,
        'right': right_rgb,
        'rgb': front_rgb,
        'instruction': {'text': instruction},
        'pose': default_pose,
    }


# def draw_waypoint_arrows_fpv(
#     img: np.ndarray,
#     waypoints: list,
#     arrow_color: tuple = (0, 255, 0),
#     arrow_thickness: int = 3,
#     tipLength: float = 0.3,
#     stop_color: tuple = (255, 0, 0),
#     stop_radius: int = 10,
#     scale_factor: float = 0.3,     # [설정] 네트워크 출력값 스케일링
#     vis_scale: float = 150.0       # [설정] 미터 -> 픽셀 변환 비율 (화면 크기에 맞춰 조절)
# ) -> np.ndarray:
#     """
#     [수정됨] Dead Reckoning 로직 적용 + OpenCV 네이티브 드로잉
#     """
#     out = img.copy()
#     h, w = out.shape[:2]
    
#     # 시작점: 화면 하단 중앙
#     start_pixel = (w // 2, h - 30)
    
#     # 로컬 누적 좌표 초기화 (항상 0,0,0에서 시작)
#     Gx, Gy, Gtheta = 0.0, 0.0, 0.0
#     prev_pixel = start_pixel

#     for i, wp in enumerate(waypoints):
#         dx_net = wp.get('dx', 0.0)
#         dy_net = wp.get('dy', 0.0)
#         dtheta_deg = wp.get('dtheta', 0.0)
#         arrive = wp.get('arrive', 0)

#         # 1. Body Frame 변환 (x:전방, y:좌측)
#         dx_net_scaled = dx_net * scale_factor
#         dy_net_scaled = dy_net * scale_factor
        
#         x_body = dy_net_scaled
#         y_body = -dx_net_scaled

#         # 2. 로컬 적분 (Dead Reckoning)
#         delta_Gx = x_body * math.cos(Gtheta) - y_body * math.sin(Gtheta)
#         delta_Gy = x_body * math.sin(Gtheta) + y_body * math.cos(Gtheta)

#         Gx += delta_Gx
#         Gy += delta_Gy
#         Gtheta += math.radians(dtheta_deg)

#         # 3. 화면 좌표 매핑 (전방->위쪽, 좌측->왼쪽)
#         screen_x = int(start_pixel[0] - (Gy * vis_scale))
#         screen_y = int(start_pixel[1] - (Gx * vis_scale))
        
#         # 화면 밖으로 나가는 것 방지
#         screen_x = np.clip(screen_x, 0, w - 1)
#         screen_y = np.clip(screen_y, 0, h - 1)
#         curr_pixel = (screen_x, screen_y)

#         if arrive > 0:
#             cv2.circle(out, curr_pixel, stop_radius, stop_color, -1)
#             cv2.putText(out, "STOP", (curr_pixel[0]-20, curr_pixel[1]-10), 
#                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
#             break 
#         else:
#             # 이동량이 너무 작으면 화살표 그리기 생략 (에러 방지)
#             if np.linalg.norm(np.array(curr_pixel) - np.array(prev_pixel)) > 2:
#                 cv2.arrowedLine(out, prev_pixel, curr_pixel, arrow_color, arrow_thickness, 
#                                 tipLength=tipLength, line_type=cv2.LINE_AA)
            
#         prev_pixel = curr_pixel

#     return out

class OmniNavOnlineInference:
    """Real-time OmniNav inference with RealSense camera or panorama data-dir (front/left/right per frame)."""
    
    def __init__(self, model_path: str, instruction: str, result_path: str = "./data/result_online", data_dir: str = None):
        """
        Args:
            model_path: Path to OmniNav model
            instruction: Navigation instruction text (ignored if data_dir is set; loaded from data_dir/instruction.txt)
            result_path: Path to save results
            data_dir: If set, use panorama structure: data_dir/rgb/frame_XXXX/ with _front.jpg, _left.jpg, _right.jpg per frame. Front/left/right are loaded from disk per frame.
        """
        self.result_path = result_path
        self.data_dir = Path(data_dir) if data_dir else None
        self.frame_folders = []
        self.use_panorama = self.data_dir is not None and self.data_dir.is_dir()
        
        if self.use_panorama:
            instruction_path = self.data_dir / 'instruction.txt'
            self.instruction = instruction_path.read_text(encoding='utf-8').strip()
            rgb_dir = self.data_dir / 'rgb'
            self.frame_folders = get_rgb_frame_folders(str(rgb_dir))
            if not self.frame_folders:
                raise FileNotFoundError(f"No frame_* folders in {rgb_dir}")
        else:
            self.instruction = instruction
        
        # Image buffers for front/left/right (only used when not use_panorama)
        self.latest_front = None
        self.latest_left = None
        self.latest_right = None
        self.image_lock = threading.Lock()
        self.image_timestamp = None
        
        # Frame counter
        self.frame_count = 0
        
        # Visualization storage
        self.vis_frame_list = []
        self.save_video = True
        
        # CSV records for waypoint data
        self.csv_records = []
        self.total_infer_time = 0.0
        
        # Initialize ROS2
        rclpy.init()
        self.ros_node = rclpy.create_node('omninav_inference')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Image subscribers (front, left, right) only when not using panorama
        if not self.use_panorama:
            self.ros_node.create_subscription(
                Image,
                '/cam_front/color/image_raw',
                self._image_front_callback,
                qos_profile
            )
            self.ros_node.create_subscription(
                Image,
                '/cam_left/color/image_raw',
                self._image_left_callback,
                qos_profile
            )
            self.ros_node.create_subscription(
                Image,
                '/cam_right/color/image_raw',
                self._image_right_callback,
                qos_profile
            )
        else:
            pass
        
        # Action publisher (waypoints as JSON string)
        self.action_pub = self.ros_node.create_publisher(String, '/action', qos_profile)
        
        print("=" * 60)
        print("[OmniNav Online] ROS2 Node Ready")
        if self.use_panorama:
            print(f"[OmniNav Online] Data-dir (panorama): {self.data_dir} | {len(self.frame_folders)} frames")
        else:
            print(f"[OmniNav Online] Subscribing to: /cam_front/color/image_raw, /cam_left/color/image_raw, /cam_right/color/image_raw")
        print(f"[OmniNav Online] Publishing to: /action")
        print(f"[OmniNav Online] Instruction: {self.instruction[:80]}...")
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
    
    def _image_msg_to_rgb(self, msg: Image) -> np.ndarray:
        """Decode sensor_msgs/Image to RGB numpy array. Supports rgb8, bgr8, rgba8, bgra8."""
        try:
            # ROS2 may expose msg.data as list or bytes; use np.array for compatibility
            data = np.array(msg.data, dtype=np.uint8)
            if msg.encoding in ('bgr8', 'bgra8'):
                channels = 4 if msg.encoding == 'bgra8' else 3
                shape = (msg.height, msg.width, channels)
                img = data.reshape(shape)
                if channels == 4:
                    img = img[:, :, :3]
                img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            elif msg.encoding in ('rgb8', 'rgba8'):
                channels = 4 if msg.encoding == 'rgba8' else 3
                shape = (msg.height, msg.width, channels)
                img = data.reshape(shape)
                if channels == 4:
                    img_rgb = img[:, :, :3].copy()
                else:
                    img_rgb = img
            else:
                # mono8 or unknown: treat as grayscale and duplicate to RGB
                img = data.reshape((msg.height, msg.width))
                img_rgb = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
            return img_rgb
        except Exception as e:
            print(f"[OmniNav Online] _image_msg_to_rgb error (encoding={getattr(msg, 'encoding', '?')}): {e}")
            return None

    def _image_front_callback(self, msg: Image):
        """Store latest front camera image."""
        img_rgb = self._image_msg_to_rgb(msg)
        if img_rgb is not None:
            with self.image_lock:
                self.latest_front = img_rgb
                self.image_timestamp = time.time()

    def _image_left_callback(self, msg: Image):
        """Store latest left camera image."""
        img_rgb = self._image_msg_to_rgb(msg)
        if img_rgb is not None:
            with self.image_lock:
                self.latest_left = img_rgb
                self.image_timestamp = time.time()

    def _image_right_callback(self, msg: Image):
        """Store latest right camera image."""
        img_rgb = self._image_msg_to_rgb(msg)
        if img_rgb is not None:
            with self.image_lock:
                self.latest_right = img_rgb
                self.image_timestamp = time.time()
    
    def _spin_ros(self):
        """ROS2 spin in background thread"""
        rclpy.spin(self.ros_node)
    
    def get_latest_images(self) -> tuple:
        """Get the latest front, left, right images (thread-safe).
        
        Returns:
            (front_rgb, left_rgb, right_rgb, timestamp) or (None, None, None, None) if any is missing
        """
        with self.image_lock:
            if self.latest_front is not None and self.latest_left is not None and self.latest_right is not None:
                return (
                    self.latest_front.copy(),
                    self.latest_left.copy(),
                    self.latest_right.copy(),
                    self.image_timestamp,
                )
            return None, None, None, None
    
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
        """Main inference loop - Step-by-Step Mode. Uses panorama front/left/right from disk if data_dir was set."""
        move_duration = inference_interval  # time to wait between inferences (robot move duration)
        PREDICT_SCALE = 0.3
        info = {'top_down_map_vlnce': None, 'gt_map': None, 'pred_map': None}
        
        if self.use_panorama:
            self._run_loop_panorama(move_duration, PREDICT_SCALE, info)
        else:
            self._run_loop_ros(move_duration, PREDICT_SCALE, info)
    
    def _run_loop_panorama(self, move_duration, PREDICT_SCALE, info):
        """Iterate frame folders; load front/left/right from disk (same structure as run_infer_iphone_panorama)."""
        print(f"\n[OmniNav Online] Starting Panorama Loop ({len(self.frame_folders)} frames, Move Duration: {move_duration}s)")
        print("[OmniNav Online] Running inference loop. Press Ctrl+C to stop.")
        print("=" * 60)
        try:
            for i in range(len(self.frame_folders)):
                frame_folder = self.frame_folders[i]
                obs = load_observations_from_frame_folder(frame_folder, self.instruction)
                front_rgb = obs['front']
                
                start_time = time.time()
                with torch.no_grad():
                    action = self.agent.act(obs, info, "online_session")
                infer_time = time.time() - start_time
                self.total_infer_time += infer_time
                self.frame_count += 1
                
                print(f"[Frame {self.frame_count:04d}] Inference time: {infer_time:.3f}s")
                
                if 'arrive_pred' in action and 'action' in action and 'recover_angle' in action:
                    arrive = int(action['arrive_pred'])
                    waypoints = action['action']
                    recover_angles = action['recover_angle']
                    if isinstance(waypoints, np.ndarray) and waypoints.ndim > 1:
                        waypoints = waypoints.reshape(-1, 2)
                    if isinstance(recover_angles, np.ndarray) and recover_angles.ndim > 1:
                        recover_angles = recover_angles.flatten()
                    for subframe_idx in range(min(5, len(waypoints))):
                        dx = waypoints[subframe_idx][0] / PREDICT_SCALE
                        dy = waypoints[subframe_idx][1] / PREDICT_SCALE
                        dtheta = np.degrees(recover_angles[subframe_idx]) if subframe_idx < len(recover_angles) else 0.0
                        self.csv_records.append({
                            'frame_idx': self.frame_count,
                            'subframe_idx': subframe_idx,
                            'dx': float(dx), 'dy': float(dy), 'dtheta': float(dtheta),
                            'arrive': arrive,
                            'infer_time_s': float(infer_time) if subframe_idx == 0 else 0.0
                        })
                    wp, dtheta0 = waypoints[0], np.degrees(recover_angles[0]) if len(recover_angles) > 0 else 0.0
                    print(f"  -> arrive={arrive}, dtheta={dtheta0:.2f}, wp[0]=({wp[0]:.4f}, {wp[1]:.4f})")
                
                waypoint_list = self.publish_action(action)
                # Visualization
                # if waypoint_list and len(waypoint_list) > 0:
                #     self.vis_frame_list.append(draw_waypoint_arrows_fpv(front_rgb, waypoint_list))
                # else:
                #     self.vis_frame_list.append(front_rgb.copy())
                
                if action.get('arrive_pred', 0) != 0:
                    print("\n" + "=" * 60)
                    print("[OmniNav Online] ARRIVED! Navigation complete.")
                    print("=" * 60)
                    break
                print(f"  >> Moving robot for {move_duration}s...")
                time.sleep(move_duration)
        except KeyboardInterrupt:
            print("\n[OmniNav Online] Stopping...")
        finally:
            self.shutdown()
    
    def _run_loop_ros(self, move_duration, PREDICT_SCALE, info):
        """Use subscribed front/left/right images from /cam_front/color/image_raw, /cam_left/color/image_raw, /cam_right/color/image_raw."""
        print(f"\n[OmniNav Online] Starting Step-by-Step Loop (Move Duration: {move_duration}s)")
        print("[OmniNav Online] Waiting for first images (front, left, right)...")
        while True:
            front, left, right, _ = self.get_latest_images()
            if front is not None and left is not None and right is not None:
                print(f"[OmniNav Online] First images received! front={front.shape}, left={left.shape}, right={right.shape}")
                break
            time.sleep(0.1)
        print("[OmniNav Online] Running inference loop. Press Ctrl+C to stop.")
        print("=" * 60)
        try:
            while True:
                front, left, right, _ = self.get_latest_images()
                if front is None or left is None or right is None:
                    print("[OmniNav Online] Waiting for front/left/right images...")
                    time.sleep(0.1)
                    continue
                default_pose = {'position': [0.0, 0.0, 0.0], 'rotation': [1.0, 0.0, 0.0, 0.0]}
                obs = {
                    'front': front,
                    'left': left,
                    'right': right,
                    'rgb': front,
                    'instruction': {'text': self.instruction},
                    'pose': default_pose
                }
                start_time = time.time()
                with torch.no_grad():
                    action = self.agent.act(obs, info, "online_session")
                infer_time = time.time() - start_time
                self.total_infer_time += infer_time
                self.frame_count += 1
                print(f"[Frame {self.frame_count:04d}] Inference time: {infer_time:.3f}s")
                
                if 'arrive_pred' in action and 'action' in action and 'recover_angle' in action:
                    arrive = int(action['arrive_pred'])
                    waypoints = action['action']
                    recover_angles = action['recover_angle']
                    if isinstance(waypoints, np.ndarray) and waypoints.ndim > 1:
                        waypoints = waypoints.reshape(-1, 2)
                    if isinstance(recover_angles, np.ndarray) and recover_angles.ndim > 1:
                        recover_angles = recover_angles.flatten()
                    for subframe_idx in range(min(5, len(waypoints))):
                        dx = waypoints[subframe_idx][0] / PREDICT_SCALE
                        dy = waypoints[subframe_idx][1] / PREDICT_SCALE
                        dtheta = np.degrees(recover_angles[subframe_idx]) if subframe_idx < len(recover_angles) else 0.0
                        self.csv_records.append({
                            'frame_idx': self.frame_count,
                            'subframe_idx': subframe_idx,
                            'dx': float(dx), 'dy': float(dy), 'dtheta': float(dtheta),
                            'arrive': arrive,
                            'infer_time_s': float(infer_time) if subframe_idx == 0 else 0.0
                        })
                    wp, dtheta0 = waypoints[0], np.degrees(recover_angles[0]) if len(recover_angles) > 0 else 0.0
                    print(f"  -> arrive={arrive}, dtheta={dtheta0:.2f}, wp[0]=({wp[0]:.4f}, {wp[1]:.4f})")
                
                waypoint_list = self.publish_action(action)
                # Visualization: save front frame (draw_waypoint_arrows_fpv is commented out above)
                self.vis_frame_list.append(front.copy())
                
                if action.get('arrive_pred', 0) != 0:
                    print("\n" + "=" * 60)
                    print("[OmniNav Online] ARRIVED! Navigation complete.")
                    print("=" * 60)
                    break
                print(f"  >> Moving robot for {move_duration}s...")
                time.sleep(move_duration)
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
        
        # # Save video if frames were collected
        # if self.save_video and len(self.vis_frame_list) > 0:
        #     self._save_video()
        
        self.agent.reset()
        self.ros_node.destroy_node()
        rclpy.shutdown()
        print("[OmniNav Online] Shutdown complete")
    
    # def _save_video(self):
        # """Save visualized frames as MP4 video"""
        # if len(self.vis_frame_list) == 0:
        #     print("[OmniNav Online] No frames to save")
        #     return
        
        # Create video output directory (only create if needed)
        # os.makedirs(self.result_path, exist_ok=True)
        
        # Save video directly to result_path (no subdirectories)
        # timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        # video_path = os.path.join(self.result_path, f"omninav_online_{timestamp}.mp4")
        
        # print(f"\n[OmniNav Online] Saving {len(self.vis_frame_list)} frames to {video_path}...")
        
        # Get frame dimensions from first frame
        # h, w = self.vis_frame_list[0].shape[:2]
        
        # Define codec and create VideoWriter
        # Use 'mp4v' codec (works on most systems)
        # fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        # fps = 1.0  # 1 frame per second (matching inference interval)
        # video_writer = cv2.VideoWriter(video_path, fourcc, fps, (w, h))
        
        # if not video_writer.isOpened():
        #     print(f"[OmniNav Online] Warning: Failed to open video writer, trying alternative codec...")
        #     # Try alternative codec
        #     fourcc = cv2.VideoWriter_fourcc(*'XVID')
        #     video_writer = cv2.VideoWriter(video_path, fourcc, fps, (w, h))
        
        # if not video_writer.isOpened():
        #     print(f"[OmniNav Online] Error: Failed to create video file")
        #     return
        
        # # Write frames
        # for frame in self.vis_frame_list:
        #     # [수정됨] RGB(Matplotlib/PIL 기준) -> BGR(OpenCV 비디오 기준) 변환
        #     frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        #     video_writer.write(frame_bgr)
            
        
        # video_writer.release()
        # print(f"[OmniNav Online] Video saved: {video_path}")
        # print(f"[OmniNav Online] Video info: {len(self.vis_frame_list)} frames, {w}x{h}, {fps} fps")


def main():
    parser = argparse.ArgumentParser(description='OmniNav Real-time Inference')
    
    parser.add_argument("--model-path", type=str, required=True, help="OmniNav model path")
    parser.add_argument("--instruction", type=str, default="", help="Navigation instruction (ignored if --data-dir is set)")
    parser.add_argument("--data-dir", type=str, default=None, help="Data dir with panorama structure: data_dir/instruction.txt, data_dir/rgb/frame_XXXX/frame_XXXX_front.jpg, _left.jpg, _right.jpg (same as run_infer_iphone_panorama). If set, front/left/right are loaded from disk per frame.")
    parser.add_argument("--result-path", type=str, default="./results", help="Result save path")
    parser.add_argument("--inference-interval", type=float, default=1.0, help="Time between inferences in seconds (default: 1.0)")
    # parser.add_argument("--save-video", action="store_true", default=True, help="Save visualization as MP4 video (default: True)")
    parser.add_argument("--no-save-video", action="store_true", help="Disable video saving")
    
    args = parser.parse_args()
    
    if not args.data_dir and not args.instruction:
        parser.error("Either --instruction or --data-dir is required")
    
    # save_video = args.save_video and not args.no_save_video
    
    inference = OmniNavOnlineInference(
        model_path=args.model_path,
        instruction=args.instruction or "dummy",
        result_path=args.result_path,
        data_dir=args.data_dir
    )
    # inference.save_video = save_video
    # if save_video:
    #     print("[OmniNav Online] Video saving enabled")
    # else:
    #     print("[OmniNav Online] Video saving disabled")
    
    inference.run_loop(inference_interval=args.inference_interval)


if __name__ == "__main__":
    main()
