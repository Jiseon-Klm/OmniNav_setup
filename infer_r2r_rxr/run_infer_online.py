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
    arrow_len: int = 30,
    arrow_gap: int = 5,
    arrow_color: tuple = (0, 255, 0),
    arrow_thickness: int = 3,
    tipLength: float = 0.3,
    stop_color: tuple = (0, 0, 255),
    stop_radius: int = 8,
    base_y_ratio: float = 0.95
) -> np.ndarray:
    """
    Draw waypoint arrows on first-person view image
    
    Args:
        img: RGB image (H, W, 3)
        waypoints: List of waypoint dicts with 'dx', 'dy', 'dtheta', 'arrive'
        arrow_len: Length of arrow in pixels
        arrow_gap: Gap between arrows in pixels
        arrow_color: Arrow color (R, G, B)
        arrow_thickness: Arrow line thickness
        tipLength: Arrow tip length ratio
        stop_color: Stop indicator color
        stop_radius: Stop indicator radius
        base_y_ratio: Base Y position ratio (0.95 = 95% from top, bottom of image)
        
    Returns:
        RGB image with arrows drawn
    """
    out = img.copy()
    h, w = out.shape[:2]
    
    # Base position: center bottom of image
    base_x = w // 2
    base_y = int(h * base_y_ratio)
    
    # Draw arrows from bottom to top (first waypoint at bottom, last at top)
    for i, wp in enumerate(waypoints):
        dx = wp.get('dx', 0.0)
        dy = wp.get('dy', 0.0)
        dtheta = wp.get('dtheta', 0.0)  # degrees
        arrive = wp.get('arrive', 0)
        
        # Calculate arrow position (stacked from bottom to top)
        y_pos = int(base_y - i * (arrow_len + arrow_gap))
        start = (base_x, y_pos)
        
        # Check if arrived (stop indicator)
        if arrive > 0:
            # Draw stop circle
            cv2.circle(out, start, stop_radius, stop_color, -1)
            # Draw text "STOP"
            cv2.putText(out, "STOP", (start[0] - 20, start[1] + 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        else:
            # Calculate arrow direction from waypoint (dx, dy)
            # 
            # Robot coordinate system (waypoint):
            # - dx: forward direction (X-axis, + = forward)
            # - dy: left direction (Y-axis, + = left)
            # - This is a standard robot frame: X=forward, Y=left
            # 
            # Image coordinate system:
            # - x: rightward (+)
            # - y: downward (+)
            # - Image directions:
            #   * Up (forward) = -π/2
            #   * Right = 0
            #   * Down (backward) = π/2
            #   * Left = π (or -π)
            # 
            # Coordinate transformation:
            # - Robot forward (X+, Y=0) → Image up (-π/2)
            # - Robot left (X=0, Y+) → Image left (π)
            # - Robot right (X=0, Y-) → Image right (0)
            # - Robot backward (X-, Y=0) → Image down (π/2)
            
            # Calculate angle in robot coordinate system
            # atan2(dy, dx) = atan2(Y, X): angle from X-axis (forward)
            # - dx > 0, dy = 0: forward (0 radians)
            # - dx = 0, dy > 0: left (π/2 radians)
            # - dx = 0, dy < 0: right (-π/2 radians)
            # - dx < 0, dy = 0: backward (π radians)
            # - Any combination: full 360 degrees supported
            robot_angle = np.arctan2(dy, dx)
            
            # Convert to image coordinate system
            # Formula: img_angle = -robot_angle - π/2
            # This maps:
            # - robot 0 (forward) → image -π/2 (up) ✓
            # - robot π/2 (left) → image -π (left) ✓
            # - robot -π/2 (right) → image 0 (right) ✓
            # - robot π (backward) → image -3π/2 = π/2 (down) ✓
            img_angle = -robot_angle - np.pi / 2
            
            # Normalize angle to [-π, π] range for consistency
            img_angle = np.arctan2(np.sin(img_angle), np.cos(img_angle))
            
            # Calculate end point using image coordinate system
            # x increases rightward, y increases downward
            end_x = int(start[0] + arrow_len * np.cos(img_angle))
            end_y = int(start[1] + arrow_len * np.sin(img_angle))
            end = (end_x, end_y)
            
            # Draw arrow
            cv2.arrowedLine(out, start, end, arrow_color, arrow_thickness, 
                          tipLength=tipLength, line_type=cv2.LINE_AA)
            
            # Draw waypoint index (optional, for debugging)
            cv2.putText(out, f"{i+1}", (start[0] + 15, start[1] + 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, arrow_color, 1)
    
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
        """Publish action to /action topic as JSON string
        
        Args:
            action: Action dictionary from inference
        """
        if 'arrive_pred' not in action or 'action' not in action or 'recover_angle' not in action:
            print("[OmniNav Online] Invalid action format, skipping publish")
            return None
        
        arrive = int(action['arrive_pred'])
        waypoints = action['action']  # shape (5, 2), already in meters (scale applied by waypoint_agent)
        recover_angles = action['recover_angle']  # shape (5,)
        
        # Flatten if needed
        if isinstance(waypoints, np.ndarray) and waypoints.ndim > 1:
            waypoints = waypoints.reshape(-1, 2)
        if isinstance(recover_angles, np.ndarray) and recover_angles.ndim > 1:
            recover_angles = recover_angles.flatten()
        
        # Build waypoint list (waypoints are already in meters)
        waypoint_list = []
        for i in range(min(5, len(waypoints))):
            dx = float(waypoints[i][0])  # Already in meters
            dy = float(waypoints[i][1])  # Already in meters
            dtheta = float(np.degrees(recover_angles[i])) if i < len(recover_angles) else 0.0
            
            waypoint_list.append({
                'dx': dx,
                'dy': dy,
                'dtheta': dtheta,
                'arrive': arrive
            })
        
        # Create JSON message
        msg_data = {
            'waypoints': waypoint_list,
            'arrive_pred': arrive,
            'timestamp': time.time(),
            'frame_count': self.frame_count
        }
        
        msg = String()
        msg.data = json.dumps(msg_data)
        self.action_pub.publish(msg)
        
        # Log
        wp = waypoint_list[0]
        print(f"[Frame {self.frame_count:04d}] Published: arrive={arrive}, "
              f"wp[0]=(dx={wp['dx']:.3f}, dy={wp['dy']:.3f}, dtheta={wp['dtheta']:.1f}°)")
        
        return waypoint_list  # Return for visualization
    
    def run_loop(self, inference_interval: float = 1.0):
        """Main inference loop
        
        Args:
            inference_interval: Time between inferences in seconds (default 1.0s)
        """
        print(f"\n[OmniNav Online] Starting inference loop (interval: {inference_interval}s)")
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
                loop_start = time.time()
                
                # Get latest image
                image, timestamp = self.get_latest_image()
                
                if image is None:
                    print("[OmniNav Online] No image available, waiting...")
                    time.sleep(0.1)
                    continue
                
                # Run inference
                action, infer_time = self.run_inference(image)
                
                print(f"[Frame {self.frame_count:04d}] Inference time: {infer_time:.3f}s")
                
                # Publish action and get waypoint list for visualization
                waypoint_list = self.publish_action(action)
                
                # Visualize waypoints on image
                if waypoint_list is not None:
                    vis_image = draw_waypoint_arrows_fpv(image, waypoint_list)
                    # Store visualized frame (make a copy to avoid reference issues)
                    self.vis_frame_list.append(vis_image.copy())
                
                # Check if arrived
                if action.get('arrive_pred', 0) > 0:
                    print("\n" + "=" * 60)
                    print("[OmniNav Online] ARRIVED! Navigation complete.")
                    print("=" * 60)
                    # Continue running but log arrival
                
                # Wait for next inference cycle
                elapsed = time.time() - loop_start
                sleep_time = max(0, inference_interval - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
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
        
        # Write frames (convert RGB to BGR for OpenCV)
        for frame in self.vis_frame_list:
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

