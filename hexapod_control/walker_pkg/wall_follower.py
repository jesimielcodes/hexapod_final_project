#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import math


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        # Publishers
        self.vel_pub = self.create_publisher(Twist, '/hexapod/body_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 20)
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        self.waypoint_sub = self.create_subscription(
            Float32MultiArray, '/planned_waypoints', self.waypoint_callback, 1)
        
        # Parameters
        self.declare_parameter('linear_speed', 0.30)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('wall_follow_distance', 0.4)
        self.declare_parameter('waypoint_threshold', 0.25)  # 25cm to reach waypoint
        
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.wall_dist = self.get_parameter('wall_follow_distance').value
        self.waypoint_threshold = self.get_parameter('waypoint_threshold').value
        
        # State
        self.waypoints = []
        self.current_wp_idx = 0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        self.stuck_counter = 0
        self.max_stuck_count = 50  # ~2.5 seconds at 20Hz
        
        self.get_logger().info("Waypoint Follower initialized")
        self.get_logger().info(f"Linear speed: {self.linear_speed}, Wall distance: {self.wall_dist}")

    def waypoint_callback(self, msg):
        """Receive waypoint list from A* planner"""
        waypoints = msg.data
        self.waypoints = [(waypoints[i], waypoints[i+1]) 
                         for i in range(0, len(waypoints), 2)]
        self.current_wp_idx = 0
        self.stuck_counter = 0
        
        self.get_logger().info(
            f"Received {len(self.waypoints)} waypoints"
        )

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def _normalize_angle(self, angle):
        """Keep angle between -pi and pi"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def scan_callback(self, msg):
        ranges = list(msg.ranges)
        front = min(self.get_min_range(ranges, 330, 360), 
                    self.get_min_range(ranges, 0, 30))
        front_left = self.get_min_range(ranges, 30, 60)
        front_right = self.get_min_range(ranges, 300, 330)
        left = self.get_min_range(ranges, 60, 120)

        cmd = Twist()

        # Primary behaviour: steer toward waypoint
        if self.waypoints:
            target_x, target_y = self.waypoints[self.current_wp_idx]
            dx = target_x - self.robot_x
            dy = target_y - self.robot_y
            dist = math.sqrt(dx*dx + dy*dy)

            if dist < self.waypoint_threshold:
                self.current_wp_idx = min(
                    self.current_wp_idx + 1,
                    len(self.waypoints) - 1
                )

            target_yaw = math.atan2(dy, dx)
            angle_error = self._normalize_angle(target_yaw - self.robot_yaw)
            cmd.angular.z = angle_error * 1.0
            cmd.linear.x = self.linear_speed * max(0.3, 1.0 - abs(angle_error))

        else:
            # No waypoints yet — wall follow to explore and build the map
            if front < 0.5:
                cmd.linear.x = 0.0
                cmd.angular.z = -self.angular_speed  # turn right from wall ahead
            elif left > self.wall_dist * 2.0:
                cmd.linear.x = self.linear_speed * 0.5
                cmd.angular.z = self.angular_speed * 0.5   # turn left to find wall
            elif left < self.wall_dist * 0.6:
                cmd.linear.x = self.linear_speed * 0.5
                cmd.angular.z = -self.angular_speed * 0.3  # too close, nudge right
            else:
                cmd.linear.x = self.linear_speed
                cmd.angular.z = 0.0  # wall is good, go straight

        # Override: emergency obstacle avoidance
        if front < 0.8:
            cmd.linear.x = 0.0
            cmd.angular.z = -self.angular_speed  # turn right
            self.stuck_counter += 1
        elif front_left < 0.6:
            cmd.angular.z -= 0.3  # nudge right
            self.stuck_counter = max(0, self.stuck_counter - 1)
        elif front_right < 0.6:
            cmd.angular.z += 0.4  # nudge left
            self.stuck_counter = max(0, self.stuck_counter - 1)
        else:
            self.stuck_counter = max(0, self.stuck_counter - 1)

        # Stuck recovery
        if self.stuck_counter > self.max_stuck_count:
            self.stuck_counter = 0
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed

        self.vel_pub.publish(cmd)

    def get_min_range(self, ranges, start_idx, end_idx):
        if end_idx <= start_idx:
            sector = ranges[start_idx:] + ranges[:end_idx]
        else:
            sector = ranges[start_idx:end_idx]
        
        valid = [r for r in sector 
                if not math.isnan(r) and r > 0.05]  # remove nan but keep inf check below
        
        # treat inf as max range, not as free — cap it
        capped = [min(r, 3.0) for r in valid]
        return min(capped) if capped else 3.0  # default to 3m not inf


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopping follower...')
        cmd = Twist()
        node.vel_pub.publish(cmd)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()