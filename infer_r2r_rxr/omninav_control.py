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
    ACTION_DURATION = 0.2     # seconds per waypoint
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
        
        # Current velocity command
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        
        # Statistics
        self.total_waypoints_executed = 0
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("OmniNav Controller Ready")
        self.get_logger().info(f"  Subscribing to: /action")
        self.get_logger().info(f"  Publishing to: /scout_mini_base_controller/cmd_vel")
        self.get_logger().info(f"  Max linear vel: {self.MAX_LINEAR_VEL} m/s")
        self.get_logger().info(f"  Max angular vel: {self.MAX_ANGULAR_VEL} rad/s ({math.degrees(self.MAX_ANGULAR_VEL):.1f} deg/s)")
        self.get_logger().info(f"  Action duration: {self.ACTION_DURATION}s per waypoint")
        self.get_logger().info("=" * 60)
    
    def _action_callback(self, msg: String):
        """
        Callback for /action topic
        Receives waypoints from run_infer_online.py and queues them for execution
        """
        try:
            data = json.loads(msg.data)
            waypoints = data.get('waypoints', [])
            arrive_pred = data.get('arrive_pred', 0)
            frame_count = data.get('frame_count', 0)
            
            if not waypoints:
                self.get_logger().warn("Received empty waypoints")
                return
            
            with self.lock:
                # Update waypoint queue (replace previous queue)
                # 새로운 waypoint가 들어오면 현재 실행 중인 것을 중단하고 새 것으로 시작
                self.waypoint_queue = waypoints
                self.arrive_flag = (arrive_pred > 0)
                self.new_waypoints_received = True
                # 현재 실행 중인 waypoint 중단 (새로운 추론 결과가 우선)
                if self.is_executing:
                    self.get_logger().info(f"  >> Interrupting current waypoint execution for new inference")
                
            self.get_logger().info(f"[Frame {frame_count}] Received {len(waypoints)} waypoints, arrive={arrive_pred}")
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON decode error: {e}")
        except Exception as e:
            self.get_logger().error(f"Action callback error: {e}")
    
    def _waypoint_to_cmd_vel(self, waypoint: dict) -> tuple:
        """
        Convert waypoint (dx, dy, dtheta) to cmd_vel (linear_vel, angular_vel)
        
        모델이 예측한 waypoint를 최대한 정확하게 따라가도록 변환
        
        좌표계:
        - dx: 전방 방향 (m, + = 전진)
        - dy: 좌측 방향 (m, + = 왼쪽)
        - dtheta: waypoint에서의 heading angle (도)
        
        변환 전략 (Pure Pursuit 스타일):
        1. 목표 거리 계산: distance = sqrt(dx² + dy²)
        2. 목표 방향 계산: target_angle = atan2(dy, dx)
        3. 선형 속도: distance / ACTION_DURATION (0.2초에 해당 거리 이동)
        4. 각속도: target_angle / ACTION_DURATION (0.2초에 해당 각도 회전)
        
        Args:
            waypoint: Dict with 'dx', 'dy', 'dtheta', 'arrive'
            
        Returns:
            (linear_vel, angular_vel) tuple
        """
        dx = waypoint.get('dx', 0.0)
        dy = waypoint.get('dy', 0.0)
        dtheta = waypoint.get('dtheta', 0.0)  # degrees (recover_angle)
        arrive = waypoint.get('arrive', 0)
        
        # 도착 예측이면 정지
        if arrive > 0:
            return 0.0, 0.0
        
        # 1. 목표까지 거리 계산
        distance = math.sqrt(dx * dx + dy * dy)
        
        # 너무 가까우면 이동 안함 (노이즈 방지)
        if distance < self.MIN_DISTANCE_THRESHOLD:
            # 거리가 작으면 heading 조정만 수행
            dtheta_rad = math.radians(dtheta)
            if abs(dtheta_rad) > 0.01:  # 0.5도 이상일 때만 회전
                angular_vel = self._clip(dtheta_rad / self.ACTION_DURATION, 
                                         -self.MAX_ANGULAR_VEL, self.MAX_ANGULAR_VEL)
                return 0.0, angular_vel
            return 0.0, 0.0
        
        # 2. 목표 방향 계산 (로컬 좌표계: dx=전방, dy=왼쪽)
        # atan2(dy, dx): dy가 양수면 왼쪽으로, 음수면 오른쪽으로
        target_angle = math.atan2(dy, dx)  # 라디안
        
        # 3. 선형 속도 계산
        # 0.2초 안에 distance만큼 이동하려면 필요한 속도
        linear_vel = distance / self.ACTION_DURATION
        
        # 4. 각속도 계산
        # 목표 방향으로 회전하면서 전진 (아크 이동)
        angular_vel = target_angle / self.ACTION_DURATION
        
        # 5. 속도 제한 적용
        # 선형 속도가 너무 크면 스케일링 (비율 유지)
        if abs(linear_vel) > self.MAX_LINEAR_VEL:
            scale = self.MAX_LINEAR_VEL / abs(linear_vel)
            linear_vel = linear_vel * scale
            # 각속도도 같이 스케일링하여 경로 형태 유지
            angular_vel = angular_vel * scale
        
        # 각속도 최종 제한
        angular_vel = self._clip(angular_vel, -self.MAX_ANGULAR_VEL, self.MAX_ANGULAR_VEL)
        
        # 선형 속도는 항상 양수 (전진만, 후진 없음)
        # 단, dx가 음수면 후진해야 하는 상황
        if dx < 0 and abs(dx) > abs(dy):
            # 후진보다는 제자리 회전 후 전진 권장
            # 여기서는 간단히 후진 허용
            linear_vel = self._clip(linear_vel, -self.MAX_LINEAR_VEL, self.MAX_LINEAR_VEL)
        else:
            linear_vel = self._clip(linear_vel, 0, self.MAX_LINEAR_VEL)
        
        return linear_vel, angular_vel
    
    def _clip(self, value: float, min_val: float, max_val: float) -> float:
        """Clip value to range [min_val, max_val]"""
        return max(min_val, min(max_val, value))
    
    def _control_loop(self):
        """
        Control loop (runs at CONTROL_RATE Hz)
        Executes waypoints sequentially, each for ACTION_DURATION seconds
        """
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        with self.lock:
            # Check if new waypoints received
            if self.new_waypoints_received:
                # Start executing from first waypoint
                self.current_waypoint_idx = 0
                self.waypoint_start_time = current_time
                self.is_executing = True
                self.new_waypoints_received = False
                
                # Log first waypoint
                if self.waypoint_queue:
                    wp = self.waypoint_queue[0]
                    dist = math.sqrt(wp['dx']**2 + wp['dy']**2)
                    self.get_logger().info(
                        f"  >> wp[0]: dx={wp['dx']:.4f}m, dy={wp['dy']:.4f}m, "
                        f"dist={dist:.4f}m, dtheta={wp['dtheta']:.1f}°"
                    )
            
            # Check if we should move to next waypoint
            if self.is_executing and self.current_waypoint_idx >= 0:
                elapsed = current_time - self.waypoint_start_time
                
                if elapsed >= self.ACTION_DURATION:
                    # Move to next waypoint
                    self.current_waypoint_idx += 1
                    self.waypoint_start_time = current_time
                    self.total_waypoints_executed += 1
                    
                    if self.current_waypoint_idx >= len(self.waypoint_queue):
                        # All waypoints executed
                        self.is_executing = False
                        self.current_waypoint_idx = -1
                        self.current_linear_vel = 0.0
                        self.current_angular_vel = 0.0
                        self.get_logger().info("  >> All 5 waypoints done, waiting...")
                    else:
                        # Log current waypoint
                        wp = self.waypoint_queue[self.current_waypoint_idx]
                        dist = math.sqrt(wp['dx']**2 + wp['dy']**2)
                        self.get_logger().info(
                            f"  >> wp[{self.current_waypoint_idx}]: dx={wp['dx']:.4f}m, dy={wp['dy']:.4f}m, "
                            f"dist={dist:.4f}m, dtheta={wp['dtheta']:.1f}°"
                        )
            
            # Calculate velocity for current waypoint
            if self.is_executing and 0 <= self.current_waypoint_idx < len(self.waypoint_queue):
                waypoint = self.waypoint_queue[self.current_waypoint_idx]
                self.current_linear_vel, self.current_angular_vel = self._waypoint_to_cmd_vel(waypoint)
            else:
                # Idle - stop
                self.current_linear_vel = 0.0
                self.current_angular_vel = 0.0
            
            # Check arrival flag - stop if arrived
            if self.arrive_flag and not self.is_executing:
                self.current_linear_vel = 0.0
                self.current_angular_vel = 0.0
            
            # Copy values for publishing
            linear_vel = self.current_linear_vel
            angular_vel = self.current_angular_vel
        
        # Publish cmd_vel (outside lock to avoid blocking)
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
