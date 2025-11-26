#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import termios
import sys
import tty
import select
import time
import tf_transformations


class MotionPatterns(Node):

    def __init__(self):
        super().__init__('motion_patterns')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.speed = 1.5       # m/s
        self.turn_speed = 0.8  # rad/s

        # Start automatically in Circle mode
        self.current_mode = 'c'
        self.start_time = time.time()

        self.current_pose = None
        self.home_pose = (0.0, 0.0, 0.0)

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.timer = self.create_timer(0.05, self.update_motion)

        self.get_logger().info(
            "\nMotion Pattern Controller Active\n"
            "[l] Line  |  [c] Circle  |  [s] Square\n"
            "[8] Figure-8  |  [h] Return Home  |  [x] Stop\n"
        )

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def get_key(self):
        dr, _, _ = select.select([sys.stdin], [], [], 0)
        return sys.stdin.read(1) if dr else None

    def update_motion(self):
        key = self.get_key()
        if key:
            self.current_mode = key
            self.start_time = time.time()
            self.get_logger().info(f"➡ Mode: {key}")

        cmd = Twist()
        t = time.time() - self.start_time

        # Straight Motion
        if self.current_mode == 'l':
            cmd.linear.x = self.speed

        # Parametric Circle
        elif self.current_mode == 'c':
            R = 1.2   # radius (m)
            w = 0.6   # rad/s

            cmd.linear.x = R * w
            cmd.angular.z = w

        # Square Path (2 sec edges)
        elif self.current_mode == 's':
            phase = int(t) % 4
            if phase < 2:
                cmd.linear.x = self.speed
            else:
                cmd.angular.z = self.turn_speed

        # Lemniscate of Gerono (Figure-8)
        elif self.current_mode == '8':
            A = 1.2
            w = 0.6
            theta = w * t

            dx = A * w * math.cos(theta)
            dy = A * w * math.sin(theta) * math.cos(theta)
            ddx = -A * w**2 * math.sin(theta)
            ddy = A * w**2 * (math.cos(2 * theta) - math.sin(theta)**2)

            v = min(math.sqrt(dx**2 + dy**2), 0.8)
            omega = (dx * ddy - dy * ddx) / (v**2 + 1e-6)

            cmd.linear.x = v
            cmd.angular.z = omega

        # Return Home
        elif self.current_mode == 'h' and self.current_pose:
            hx, hy, _ = self.home_pose
            x = self.current_pose.position.x
            y = self.current_pose.position.y

            q = self.current_pose.orientation
            _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

            dx = hx - x
            dy = hy - y
            dist = math.sqrt(dx**2 + dy**2)

            ang_to_goal = math.atan2(dy, dx)
            head_err = math.atan2(math.sin(ang_to_goal - yaw),
                                  math.cos(ang_to_goal - yaw))

            cmd.linear.x = min(0.7 * dist, 1.0)
            cmd.angular.z = 1.2 * head_err

            if dist < 0.25:
                self.get_logger().info("🏁 Returned Home!")
                self.current_mode = 'x'

        # Stop
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        # Safety Boundary (stay inside 45m radius)
        if self.current_pose and self.current_mode != 'h':
            x = self.current_pose.position.x
            y = self.current_pose.position.y
            radius = math.sqrt(x**2 + y**2)

            if radius > 45.0:
                cmd.linear.x = 0.0
                cmd.angular.z = self.turn_speed
                self.get_logger().warn("⚠ Boundary reached → turning inward!")

        self.pub.publish(cmd)


def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    rclpy.init(args=args)
    node = MotionPatterns()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()