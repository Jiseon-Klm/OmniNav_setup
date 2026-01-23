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


class OmniNavOnlineInference:
    """Real-time OmniNav inference with RealSense camera"""
    
    def __init__(self, model_path: str, instruction: str, result_path: str = "./data/result_online",
                 image_topic: str = "/camera/color/image_raw/compressed"):
        """
        Args:
            model_path: Path to OmniNav model
            instruction: Navigation instruction text
            result_path: Path to save results
            image_topic: ROS2 topic for compressed image
        """
        self.instruction = instruction
        self.result_path = result_path
        self.image_topic = image_topic
        
        # Image buffer (stores only the latest image)
        self.latest_image = None
        self.image_lock = threading.Lock()
        self.image_timestamp = None
        
        # Frame counter
        self.frame_count = 0
        
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
        model_name = '/'.join(model_path.split('/')[-3:])
        result_path_full = os.path.join(result_path, model_name)
        os.makedirs(result_path_full, exist_ok=True)
        
        print("[OmniNav Online] Loading model...")
        self.agent = Waypoint_Agent(model_path, result_path_full, require_map=False)
        self.agent.reset()
        self.agent.episode_id = "online_session"
        print("[OmniNav Online] Model loaded successfully")
        
        # Start ROS2 spin thread
        self.spin_thread = threading.Thread(target=self._spin_ros, daemon=True)
        self.spin_thread.start()
    
    def _image_callback(self, msg: CompressedImage):
        """Callback for compressed image - stores only the latest image"""
        try:
            # Decode compressed image
            np_arr = np.frombuffer(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if img_bgr is not None:
                # Convert BGR to RGB
                img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
                
                # Store latest image (thread-safe)
                with self.image_lock:
                    self.latest_image = img_rgb
                    self.image_timestamp = time.time()
        except Exception as e:
            print(f"[OmniNav Online] Image decode error: {e}")
    
    def _spin_ros(self):
        """ROS2 spin in background thread"""
        rclpy.spin(self.ros_node)
    
    def _create_observations(self, rgb_image: np.ndarray) -> dict:
        """Create observations dictionary mimicking simulator format
        
        Identical to run_infer_iphone.py's create_fake_observations
        """
        # Default pose (no odometry in online mode)
        default_pose = {
            'position': [0.0, 0.0, 0.0],
            'rotation': [1.0, 0.0, 0.0, 0.0]
        }
        
        observations = {
            'front': rgb_image,
            'left': rgb_image.copy(),
            'right': rgb_image.copy(),
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
        PREDICT_SCALE = 0.3
        
        if 'arrive_pred' not in action or 'action' not in action or 'recover_angle' not in action:
            print("[OmniNav Online] Invalid action format, skipping publish")
            return
        
        arrive = int(action['arrive_pred'])
        waypoints = action['action']  # shape (5, 2), scale already applied
        recover_angles = action['recover_angle']  # shape (5,)
        
        # Flatten if needed
        if isinstance(waypoints, np.ndarray) and waypoints.ndim > 1:
            waypoints = waypoints.reshape(-1, 2)
        if isinstance(recover_angles, np.ndarray) and recover_angles.ndim > 1:
            recover_angles = recover_angles.flatten()
        
        # Build waypoint list (restore to original scale)
        waypoint_list = []
        for i in range(min(5, len(waypoints))):
            dx = float(waypoints[i][0] / PREDICT_SCALE)
            dy = float(waypoints[i][1] / PREDICT_SCALE)
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
                
                # Publish action
                self.publish_action(action)
                
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
        """Clean up resources"""
        print("[OmniNav Online] Shutting down...")
        self.agent.reset()
        self.ros_node.destroy_node()
        rclpy.shutdown()
        print("[OmniNav Online] Shutdown complete")


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
        default="./data/result_online",
        help="Result save path"
    )
    
    parser.add_argument(
        "--inference-interval",
        type=float,
        default=1.0,
        help="Time between inferences in seconds (default: 1.0)"
    )
    
    parser.add_argument(
        "--image-topic",
        type=str,
        default="/camera/color/image_raw/compressed",
        help="ROS2 topic for compressed image (default: /camera/color/image_raw/compressed)"
    )
    
    args = parser.parse_args()
    
    # Create inference instance
    inference = OmniNavOnlineInference(
        model_path=args.model_path,
        instruction=args.instruction,
        result_path=args.result_path,
        image_topic=args.image_topic
    )
    
    # Run inference loop
    inference.run_loop(inference_interval=args.inference_interval)


if __name__ == "__main__":
    main()

