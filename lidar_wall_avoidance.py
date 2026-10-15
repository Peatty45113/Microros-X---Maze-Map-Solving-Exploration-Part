#!/usr/bin/env python3
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class WallAvoider:
    def __init__(self, cell=0.30, front_gap=0.02, robot_half_width=0.08, robot_front=0.10,
                 safe_margin=0.02, k_avoid=6.0, safe_speed=0.03, turn_speed=0.5,
                 ignore_below=0.08, lidar_front_deg=0.0, lidar_ccw=True, logger=None):
        self.half_w = robot_half_width
        self.robot_front = robot_front
        self.safe_margin = safe_margin
        self.k_avoid = k_avoid
        self.safe_speed = safe_speed
        self.w_turn = turn_speed
        self.ignore_below = ignore_below
        self.front_stop = cell / 2.0 + front_gap
        self.lidar_front = lidar_front_deg
        self.lidar_sign = 1.0 if lidar_ccw else -1.0
        self.log = logger
        self.avoiding = False
        self.scan_r = self.scan_a = self.scan_ok = None

    def update_scan(self, msg):
        r = np.asarray(msg.ranges, dtype=float)
        ang = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        ang = np.degrees(ang) * self.lidar_sign - self.lidar_front
        ang = (ang + 180.0) % 360.0 - 180.0
        ok = np.isfinite(r) & (r > max(msg.range_min, 0.02)) & (r < msg.range_max)
        self.scan_r, self.scan_a, self.scan_ok = r, ang, ok

    def filter(self, v, w):
        if self.scan_r is None:
            return 0.0, 0.0

        a = np.radians(self.scan_a)
        x = self.scan_r * np.cos(a)
        y = self.scan_r * np.sin(a)

        box_w = self.half_w + self.safe_margin
        box_x = self.front_stop - 0.01

        hit = (self.scan_ok & (self.scan_r >= self.ignore_below) & (x > 0.0)
               & (x <= box_x) & (np.abs(y) <= box_w))

        if not hit.any():
            self.avoiding = False
            return v, w

        depth = float(box_w - np.min(np.abs(y[hit])))
        if not self.avoiding and self.log:
            self.log(f'Wall in the path ({depth * 100:.1f} cm) -> steering away')
        self.avoiding = True

        away = -1.0 if float(np.mean(y[hit])) > 0 else 1.0

        w2 = clamp(away * (0.2 + self.k_avoid * depth), -self.w_turn, self.w_turn)

        v2 = 0.0 if float(np.min(x[hit])) < self.robot_front + 0.01 else min(v, self.safe_speed)
        return v2, w2


class WallAvoidanceDemo(Node):
    def __init__(self):
        super().__init__('lidar_wall_avoidance')
        p = self.declare_parameter
        p('cell_size', 0.30)
        p('linear_speed', 0.08)
        p('robot_half_width', 0.08)
        p('robot_front', 0.10)
        p('safe_margin', 0.02)
        p('k_avoid', 6.0)
        p('safe_speed', 0.03)
        p('lidar_front_deg', 0.0)
        p('lidar_ccw', True)
        g = lambda n: self.get_parameter(n).value
        self.v = float(g('linear_speed'))
        self.avoider = WallAvoider(
            cell=float(g('cell_size')), robot_half_width=float(g('robot_half_width')),
            robot_front=float(g('robot_front')), safe_margin=float(g('safe_margin')),
            k_avoid=float(g('k_avoid')), safe_speed=float(g('safe_speed')),
            lidar_front_deg=float(g('lidar_front_deg')), lidar_ccw=bool(g('lidar_ccw')),
            logger=self.get_logger().info)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.avoider.update_scan, qos_profile_sensor_data)
        self.create_timer(0.05, self.loop)
        self.get_logger().info('Wall avoidance demo started (Ctrl+C to stop).')

    def front_distance(self):
        a = self.avoider
        m = a.scan_ok & (np.abs(a.scan_a) <= 15)
        return float(np.percentile(a.scan_r[m], 10)) if m.any() else math.inf

    def loop(self):
        if self.avoider.scan_r is None:
            return
        v, w = self.v, 0.0
        if self.front_distance() < self.avoider.front_stop:
            v = 0.0
        v, w = self.avoider.filter(v, w)
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.pub.publish(msg)

    def stop(self):
        for _ in range(5):
            self.pub.publish(Twist())


def main():
    rclpy.init()
    node = WallAvoidanceDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
