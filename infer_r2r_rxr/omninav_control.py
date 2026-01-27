#!/usr/bin/env python3
"""
OmniNav Robot Controller
Subscribes to /action topic (waypoints from run_infer_online.py)
Converts waypoints to cmd_vel and publishes to /scout_mini_base_controller/cmd_vel

Waypoint coordinate system:
- dx: Forward direction (meters, positive = forward)
- dy: Left direction (meters, positive = left)
- dtheta: Heading angle at waypoint (degrees, from recover_angle)

Control strategy:
- Follow predicted waypoints as accurately as possible
- Use Pure Pursuit-style control for smooth trajectory
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
import math
import json
import threading
import numpy as np


class OmniNavController(Node):
    """
    OmniNav Robot Controller for Scout Mini
    
    Converts waypoints (dx, dy, dtheta) to cmd_vel commands.
    Executes 5 waypoints sequentially, each for 0.2 seconds.
    """
    
    # Robot parameters (Scout Mini)
    MAX_LINEAR_VEL = 0.7      # m/s (최대 추론속도 반영)
    MAX_ANGULAR_VEL = 1.5708  # rad/s (90 deg/s)
    ACTION_DURATION = 1     # seconds per waypoint
    CONTROL_RATE = 5          # Hz (control loop frequency)
    
    # Control tuning
    MIN_DISTANCE_THRESHOLD = 0.005  # 5mm 미만은 이동 안함
    
    def __init__(self):
        super().__init__('omninav_controller')
        
        self.callback_group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        
        # QoS profile (Best Effort, depth=1 for latest message only)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe to /action topic (waypoints from run_infer_online.py)
        self.action_sub = self.create_subscription(
            String,
            '/action',
            self._action_callback,
            qos_profile,
            callback_group=self.callback_group
        )
        
        # Publish to /scout_mini_base_controller/cmd_vel
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            '/scout_mini_base_controller/cmd_vel',
            qos_profile
        )
        
        # Control loop timer (5 Hz = 0.2s)
        self.timer = self.create_timer(
            1.0 / self.CONTROL_RATE,
            self._control_loop,
            callback_group=self.callback_group
        )
        
        # State variables
        self.waypoint_queue = []         # Current waypoint queue (5 waypoints)
        self.current_waypoint_idx = -1   # Current waypoint index (-1 = idle)
        self.waypoint_start_time = 0.0   # Start time of current waypoint
        self.is_executing = False        # Whether executing waypoints
        self.new_waypoints_received = False
        self.arrive_flag = False         # Arrival flag from inference
        self.current_target_waypoint = None  # Initialize to None (safety)
        
        # Current velocity command
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        
        # Statistics
        self.total_waypoints_executed = 0
        self.callback_count = 0  # Debug: count callbacks received
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("OmniNav Controller Ready")
        self.get_logger().info(f"  Subscribing to: /action")
        self.get_logger().info(f"  Publishing to: /scout_mini_base_controller/cmd_vel")
        self.get_logger().info(f"  Max linear vel: {self.MAX_LINEAR_VEL} m/s")
        self.get_logger().info(f"  Max angular vel: {self.MAX_ANGULAR_VEL} rad/s ({math.degrees(self.MAX_ANGULAR_VEL):.1f} deg/s)")
        self.get_logger().info(f"  Action duration: {self.ACTION_DURATION}s per waypoint")
        self.get_logger().info(f"  Control rate: {self.CONTROL_RATE} Hz")
        self.get_logger().info("=" * 60)
        
        # Verify subscription is active (check for publishers)
        import time
        time.sleep(0.5)  # Wait for subscription to be established
        publisher_count = self.action_sub.get_publisher_count()
        if publisher_count > 0:
            self.get_logger().info(f"[DEBUG] Subscription active: {publisher_count} publisher(s) found")
        else:
            self.get_logger().warn("[DEBUG] WARNING: No publishers found for /action topic!")
    
    def _action_callback(self, msg: String):
        """
        Callback for /action topic
        [수정됨] 큐 방식 제거 -> 최신 목표점 갱신 방식으로 변경
        """
        try:
            self.callback_count += 1
            self.get_logger().info(f"[DEBUG] Action callback received! (count: {self.callback_count})")
            
            data = json.loads(msg.data)
            waypoints = data.get('waypoints', [])
            arrive_pred = data.get('arrive_pred', 0)
            frame_count = data.get('frame_count', 0)
            
            self.get_logger().info(f"[DEBUG] Parsed data: {len(waypoints)} waypoints, arrive_pred={arrive_pred}, frame={frame_count}")
            
            if not waypoints:
                self.get_logger().warn("[DEBUG] Empty waypoints list, ignoring")
                return
            
            with self.lock:
                # 큐를 쌓지 않고, 가장 최신의 '첫번째' 목표만 저장합니다.
                # 추론 노드에서 이미 1개만 보내주지만, 혹시 몰라 0번 인덱스만 취합니다.
                self.current_target_waypoint = waypoints[0]
                self.arrive_flag = (arrive_pred > 0)
                
                # [중요] 새로운 명령이 왔으므로 타이머를 리셋하여 즉시 ACTION_DURATION 동안 동작하게 함
                self.waypoint_start_time = self.get_clock().now().nanoseconds / 1e9
                self.is_executing = True
                
                # 로그: 갱신 확인
                wp = self.current_target_waypoint
                self.get_logger().info(f"[Frame {frame_count}] Action Updated: dx={wp.get('dx', 0):.4f}, dy={wp.get('dy', 0):.4f}, dtheta={wp.get('dtheta', 0):.2f}°, arrive={arrive_pred}")
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON decode error: {e}")
            self.get_logger().error(f"Message data: {msg.data[:200] if len(msg.data) > 200 else msg.data}")
        except Exception as e:
            self.get_logger().error(f"Action callback error: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

    def _waypoint_to_cmd_vel(self, waypoint: dict) -> tuple:
        """
        [수정됨] 좌표계 변환 적용 (Network -> Robot Body)
        Network Output: dx (Right+), dy (Forward+)
        Robot Frame:    x (Forward+), y (Left+)
        """
        dx_net = waypoint.get('dx', 0.0)
        dy_net = waypoint.get('dy', 0.0)
        arrive = waypoint.get('arrive', 0)
        
        if arrive > 0:
            return 0.0, 0.0

        # -----------------------------------------------------------
        # [CRITICAL FIX] 좌표계 변환
        # -----------------------------------------------------------
        target_x = dy_net        # Net Forward -> Robot X (Forward)
        target_y = -dx_net       # Net Right -> Robot -Y (Left)
        # -----------------------------------------------------------
        
        distance = math.sqrt(target_x**2 + target_y**2)
        
        # 노이즈 필터
        if distance < self.MIN_DISTANCE_THRESHOLD:
             return 0.0, 0.0

        # Pure Pursuit Angle
        target_angle = math.atan2(target_y, target_x)
        
        # 속도 계산 (0.2초 내 도달 목표)
        linear_vel = distance / self.ACTION_DURATION
        angular_vel = target_angle / self.ACTION_DURATION
        
        # 속도 제한 (Clipping)
        if abs(linear_vel) > self.MAX_LINEAR_VEL:
            scale = self.MAX_LINEAR_VEL / abs(linear_vel)
            linear_vel *= scale
            angular_vel *= scale
            
        angular_vel = self._clip(angular_vel, -self.MAX_ANGULAR_VEL, self.MAX_ANGULAR_VEL)             
        return linear_vel, angular_vel
    
    def _clip(self, value: float, min_val: float, max_val: float) -> float:
        """Clip value to range [min_val, max_val]"""
        return max(min_val, min(max_val, value))
     
    def _control_loop(self):
        """
        [수정됨] 단일 액션 실행 루프
        - 새로운 명령이 들어온 시점부터 정확히 ACTION_DURATION 동안만 움직이고
        - 다음 명령이 안 오면 멈춤 (Safety)
        """
        current_time = self.get_clock().now().nanoseconds / 1e9
        linear_vel = 0.0
        angular_vel = 0.0
        
        with self.lock:
            # [CRITICAL FIX] current_target_waypoint가 None이 아니고, is_executing이 True일 때만 움직임
            if self.is_executing and self.current_target_waypoint is not None:
                elapsed = current_time - self.waypoint_start_time
                
                # ACTION_DURATION 이내라면 계속 움직임
                if elapsed < self.ACTION_DURATION:
                    if self.arrive_flag:
                        linear_vel, angular_vel = 0.0, 0.0
                    else:
                        linear_vel, angular_vel = self._waypoint_to_cmd_vel(self.current_target_waypoint)
                else:
                    # ACTION_DURATION이 지났는데 아직 새 명령이 안 왔으면 정지 (Safety)
                    self.is_executing = False
                    linear_vel = 0.0
                    angular_vel = 0.0
                    self.get_logger().debug(f"Action duration ({self.ACTION_DURATION}s) elapsed, stopping")
            else:
                # 명령이 없으면 항상 정지 (Safety)
                linear_vel = 0.0
                angular_vel = 0.0
                if self.is_executing:
                    # 이전에 실행 중이었는데 갑자기 waypoint가 None이 되면 로그
                    self.get_logger().warn("[DEBUG] is_executing=True but current_target_waypoint is None, stopping")
                    self.is_executing = False
            
            # 상태 저장
            self.current_linear_vel = linear_vel
            self.current_angular_vel = angular_vel
        
        # Publish (항상 0.0이어도 publish하여 명시적으로 정지 신호 전송)
        self._publish_cmd_vel(linear_vel, angular_vel)   
    
    def _publish_cmd_vel(self, linear_vel: float, angular_vel: float):
        """
        Publish cmd_vel to /scout_mini_base_controller/cmd_vel
        """
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = linear_vel
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = angular_vel
        
        self.cmd_vel_pub.publish(msg)
    
    def stop_robot(self):
        """Emergency stop"""
        self.get_logger().warn("Emergency stop!")
        with self.lock:
            self.is_executing = False
            self.current_waypoint_idx = -1
            self.current_target_waypoint = None
            self.current_linear_vel = 0.0
            self.current_angular_vel = 0.0
            self.waypoint_queue = []
        
        self._publish_cmd_vel(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    
    node = OmniNavController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()