#!/usr/bin/env python3
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


def norm_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


FOLLOW = 'FOLLOW'
ADVANCE = 'ADVANCE'
DECIDE = 'DECIDE'
TURN = 'TURN'
ALIGN = 'ALIGN'
EXIT = 'EXIT'
DIR_NAME = {'R': 'RIGHT', 'L': 'LEFT', 'F': 'FORWARD', 'B': 'BACK'}
JUNCTION_NAME = {
    1: '1-WAY (dead end)',
    2: '2-WAY (corner / corridor)',
    3: '3-WAY JUNCTION (T-junction)',
    4: '4-WAY JUNCTION (crossroad)',
}


class RightWallFollower(Node):

    def __init__(self):
        super().__init__('right_wall_follower')
        p = self.declare_parameter
        p('scan_topic', '/scan')
        p('cmd_vel_topic', '/cmd_vel')
        p('odom_topic', '/odom')
        p('control_rate', 20.0)
        p('cell_size', 0.0)
        p('linear_speed', 0.08)
        p('turn_speed', 0.5)
        p('kp', 2.5)
        p('heading_kp', 1.5)
        p('max_correction', 0.4)
        p('max_push', 0.10)
        p('wall_near_ratio', 0.75)
        p('front_gap', 0.02)
        p('open_confirm_ticks', 3)
        p('turn_tolerance_deg', 3.0)
        p('use_lidar_center', True)
        p('lidar_offset_x', 0.0)
        p('center_tolerance', 0.015)
        p('approach_kp', 0.6)
        p('approach_speed', 0.03)
        p('advance_timeout', 6.0)
        p('sensor_delay', 0.2)
        p('settle_time', 0.3)
        p('decide_samples', 10)
        p('use_lidar_angle', True)
        p('wall_min_pts', 10)
        p('wall_bin', 0.01)
        p('wall_search_deg', 30.0)
        p('wall_range_cells', 1.5)
        p('ignore_below', 0.08)
        p('grid_corr_gain', 0.05)
        p('align_tolerance_deg', 1.5)
        p('align_kp', 2.0)
        p('align_min_w', 0.12)
        p('align_timeout', 3.0)
        p('use_safety', True)
        p('robot_half_width', 0.08)
        p('robot_front', 0.10)
        p('safe_margin', 0.02)
        p('k_avoid', 6.0)
        p('safe_speed', 0.03)
        p('lidar_front_deg', 0.0)
        p('lidar_ccw', True)
        g = lambda n: self.get_parameter(n).value
        self.rate = float(g('control_rate'))
        self.v = float(g('linear_speed'))
        self.w_turn = float(g('turn_speed'))
        self.kp = float(g('kp'))
        self.kh = float(g('heading_kp'))
        self.max_corr = float(g('max_correction'))
        self.max_push = float(g('max_push'))
        self.wall_near_ratio = float(g('wall_near_ratio'))
        self.front_gap = float(g('front_gap'))
        self.confirm_n = int(g('open_confirm_ticks'))
        self.turn_tol = math.radians(float(g('turn_tolerance_deg')))
        self.use_lidar_center = bool(g('use_lidar_center'))
        self.lidar_offset_x = float(g('lidar_offset_x'))
        self.center_tol = float(g('center_tolerance'))
        self.approach_kp = float(g('approach_kp'))
        self.approach_speed = float(g('approach_speed'))
        self.adv_ticks_max = int(float(g('advance_timeout')) * self.rate)
        self.sensor_delay = float(g('sensor_delay'))
        self.settle_ticks = int(float(g('settle_time')) * self.rate)
        self.decide_samples = int(g('decide_samples'))
        self.use_lidar_angle = bool(g('use_lidar_angle'))
        self.wall_min_pts = int(g('wall_min_pts'))
        self.wall_bin = float(g('wall_bin'))
        self.wall_search = math.radians(float(g('wall_search_deg')))
        self.wall_range_cells = float(g('wall_range_cells'))
        self.ignore_below = float(g('ignore_below'))
        self.grid_corr_gain = float(g('grid_corr_gain'))
        self.align_tol = math.radians(float(g('align_tolerance_deg')))
        self.align_kp = float(g('align_kp'))
        self.align_min_w = float(g('align_min_w'))
        self.align_ticks_max = int(float(g('align_timeout')) * self.rate)
        self.use_safety = bool(g('use_safety'))
        self.half_w = float(g('robot_half_width'))
        self.robot_front = float(g('robot_front'))
        self.safe_margin = float(g('safe_margin'))
        self.k_avoid = float(g('k_avoid'))
        self.safe_speed = float(g('safe_speed'))
        self.lidar_front = float(g('lidar_front_deg'))
        self.lidar_sign = 1.0 if bool(g('lidar_ccw')) else -1.0
        self.cell = None
        self.cell_samples = []
        if float(g('cell_size')) > 0:
            self.apply_cell(float(g('cell_size')))
        self.pub = self.create_publisher(Twist, g('cmd_vel_topic'), 10)
        self.create_subscription(LaserScan, g('scan_topic'), self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, g('odom_topic'), self.odom_cb, 10)
        self.timer = self.create_timer(1.0 / self.rate, self.loop)
        self.scan_r = None
        self.scan_a = None
        self.scan_ok = None
        self.x = self.y = self.yaw = 0.0
        self.last_odom_time = 0.0
        self.last_cmd = (0.0, 0.0)
        self.last_loop_time = time.time()
        self.state = ALIGN
        self.align_next = FOLLOW
        self.state_ticks = 0
        self.seg_x = self.seg_y = 0.0
        self.heading_ref = None
        self.left_cnt = 0
        self.right_cnt = 0
        self.turn_goal = 0.0
        self.turn_target = 0.0
        self.turn_step = 0.0
        self.turn_steps = 0
        self.grid_off = 0.0
        self.wall_ang = None
        self.wall_peak = 0
        self.decide_aligned = False
        self.settle_start = 0
        self.avoiding = False
        self.adv_dist = 0.0
        self.ahead_cnt = 0
        self.junc_rem = None
        self.slow_ticks = 0
        self.open_votes = {'R': 0, 'F': 0, 'L': 0}
        self.vote_count = 0
        self.junction_no = 0
        self.last_status = 0.0
        self.start_time = time.time()
        self.get_logger().info('Right wall follower started. Press Ctrl+C to stop.')

    def apply_cell(self, cell):
        self.cell = cell
        self.open_dist = cell
        self.side_target = cell / 2.0
        self.front_stop = cell / 2.0 + self.front_gap - self.lidar_offset_x
        self.center_offset = cell / 2.0
        self.exit_dist = cell
        self.wall_near = self.wall_near_ratio * cell
        self.get_logger().info(f'Cell size {cell:.2f} m -> open>{self.open_dist:.2f} '
                               f'stop<{self.front_stop:.2f} side={self.side_target:.2f}')

    def scan_cb(self, msg: LaserScan):
        r = np.asarray(msg.ranges, dtype=float)
        ang = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        ang = np.degrees(ang) * self.lidar_sign - self.lidar_front
        ang = (ang + 180.0) % 360.0 - 180.0
        ok = np.isfinite(r) & (r > max(msg.range_min, 0.02)) & (r < msg.range_max)
        self.scan_r, self.scan_a, self.scan_ok = r, ang, ok

    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.last_odom_time = time.time()

    def sector(self, lo, hi, stat='median'):
        m = self.scan_ok & (self.scan_a >= lo) & (self.scan_a <= hi)
        vals = self.scan_r[m]
        if vals.size == 0:
            return float('inf')
        if stat == 'min':
            return float(np.min(vals))
        if stat == 'p10':
            return float(np.percentile(vals, 10))
        return float(np.median(vals))

    def measure(self):
        return {
            'front': self.sector(-15, 15, 'p10'),
            'left': self.sector(80, 100, 'median'),
            'right': self.sector(-100, -80, 'median'),
            'left_wall': self.sector(70, 110, 'p10'),
            'right_wall': self.sector(-110, -70, 'p10'),
        }

    def wall_score(self, x, y, th):
        c, s = math.cos(th), math.sin(th)
        u = x * c + y * s
        v = -x * s + y * c
        hu = np.bincount(np.floor((u - u.min()) / self.wall_bin).astype(int))
        hv = np.bincount(np.floor((v - v.min()) / self.wall_bin).astype(int))
        score = float(np.sum(hu.astype(float) ** 2) + np.sum(hv.astype(float) ** 2))
        return score, int(max(hu.max(), hv.max()))

    def lidar_heading_error(self):
        sel = (self.scan_ok & (self.scan_r >= self.ignore_below)
               & (self.scan_r <= self.wall_range_cells * self.cell))
        a = np.radians(self.scan_a[sel])
        x = self.scan_r[sel] * np.cos(a)
        y = self.scan_r[sel] * np.sin(a)
        if len(x) < self.wall_min_pts:
            self.wall_peak = 0
            return None
        best, best_score = 0.0, -1.0
        for th in np.arange(-self.wall_search, self.wall_search + 1e-9, math.radians(1.0)):
            sc = self.wall_score(x, y, th)[0]
            if sc > best_score:
                best, best_score = th, sc
        coarse = best
        for th in np.arange(coarse - math.radians(1.0), coarse + math.radians(1.0) + 1e-9, math.radians(0.1)):
            sc = self.wall_score(x, y, th)[0]
            if sc > best_score:
                best, best_score = th, sc
        self.wall_peak = self.wall_score(x, y, best)[1]
        if self.wall_peak < self.wall_min_pts:
            return None
        return float(best)

    def grid_heading(self):
        q = math.pi / 2
        return norm_angle(round(norm_angle(self.yaw - self.grid_off) / q) * q + self.grid_off)

    def update_grid(self, gain=None):
        if not self.use_lidar_angle or self.wall_ang is None:
            return
        k = self.grid_corr_gain if gain is None else gain
        q = math.pi / 2
        g = self.yaw + self.wall_ang
        off = norm_angle(g - round(g / q) * q)
        self.grid_off = norm_angle(self.grid_off + k * norm_angle(off - self.grid_off))

    def align_w(self):
        if self.wall_ang is None:
            return None
        if abs(self.wall_ang) < self.align_tol:
            return 0.0
        w = clamp(self.align_kp * self.wall_ang, -0.6 * self.w_turn, 0.6 * self.w_turn)
        if abs(w) < self.align_min_w:
            w = math.copysign(self.align_min_w, w)
        return w

    def side_gap_center(self, xs):
        if xs.size < 3:
            return None
        xs = np.sort(xs)
        min_gap = 0.6 * self.cell
        d = np.diff(xs)
        for i in np.nonzero(d >= min_gap)[0]:
            a, b = xs[i], xs[i + 1]
            if b > 0.0:
                if b - a <= 1.4 * self.cell:
                    return 0.5 * (a + b)
                return a + 0.5 * self.cell
        if xs[-1] < 1.5 * self.cell - min_gap and xs[-1] > -0.5 * self.cell:
            return xs[-1] + 0.5 * self.cell
        return None

    def junction_remaining(self):
        th = self.wall_ang if self.wall_ang is not None else 0.0
        a = np.radians(self.scan_a)
        x = self.scan_r * np.cos(a)
        y = self.scan_r * np.sin(a)
        u = x * math.cos(th) + y * math.sin(th)
        v = -x * math.sin(th) + y * math.cos(th)
        ok = (self.scan_ok & (self.scan_r >= self.ignore_below)
              & (u > -1.0 * self.cell) & (u < 1.5 * self.cell))
        lo, hi = 0.3 * self.cell, self.wall_near
        found = []
        for side in (1.0, -1.0):
            band = ok & (side * v >= lo) & (side * v <= hi)
            c = self.side_gap_center(u[band])
            if c is not None:
                found.append(c)
        if not found:
            return None
        c = min(found)
        return c + self.lidar_offset_x

    def side_clear(self, side):
        th = self.wall_ang if self.wall_ang is not None else 0.0
        a = np.radians(self.scan_a)
        x = self.scan_r * np.cos(a)
        y = self.scan_r * np.sin(a)
        u = x * math.cos(th) + y * math.sin(th)
        v = -x * math.sin(th) + y * math.cos(th)
        wall = (self.scan_ok & (self.scan_r >= self.ignore_below)
                & (side * v >= 0.3 * self.cell) & (side * v <= self.wall_near)
                & (np.abs(u + self.lidar_offset_x) <= 0.25 * self.cell))
        return int(np.count_nonzero(wall)) < 3

    def safe_cmd(self, v, w, m):
        if not self.use_safety:
            return v, w
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
        if not self.avoiding:
            self.get_logger().info(f'Wall in the path ({depth * 100:.1f} cm) -> steering away')
        self.avoiding = True
        away = -1.0 if float(np.mean(y[hit])) > 0 else 1.0
        w2 = clamp(away * (0.2 + self.k_avoid * depth), -self.w_turn, self.w_turn)
        v2 = 0.0 if float(np.min(x[hit])) < self.robot_front + 0.01 else min(v, self.safe_speed)
        return v2, w2

    def publish(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub.publish(msg)
        self.last_cmd = (v, w)

    def stop_robot(self):
        for _ in range(5):
            self.publish(0.0, 0.0)
            time.sleep(0.05)

    def traveled(self):
        return math.hypot(self.x - self.seg_x, self.y - self.seg_y)

    def start_segment(self):
        self.seg_x, self.seg_y = self.x, self.y

    def heading_hold(self):
        if self.use_lidar_angle and self.wall_ang is not None:
            self.heading_ref = self.grid_heading()
            return clamp(self.kh * self.wall_ang, -self.max_corr, self.max_corr)
        if self.heading_ref is None:
            self.heading_ref = self.grid_heading()
        return clamp(self.kh * norm_angle(self.heading_ref - self.yaw), -self.max_corr, self.max_corr)

    def wall_follow_w(self, m):
        lw = m['left_wall'] < self.wall_near
        rw = m['right_wall'] < self.wall_near
        if self.heading_ref is None:
            self.heading_ref = self.grid_heading()
        heading_err = norm_angle(self.heading_ref - self.yaw)
        if self.use_lidar_angle and self.wall_ang is not None:
            heading_err = self.wall_ang
            self.heading_ref = self.grid_heading()
        if lw and rw:
            push = self.kp * (m['left_wall'] - m['right_wall'])
        elif rw:
            push = -self.kp * (m['right_wall'] - self.side_target)
        elif lw:
            push = self.kp * (m['left_wall'] - self.side_target)
        else:
            return self.heading_hold()
        push = clamp(push, -self.max_push, self.max_push)
        w = push + self.kh * heading_err
        return clamp(w, -self.max_corr, self.max_corr)

    def set_state(self, s):
        self.state = s
        self.state_ticks = 0
        self.junc_rem = None
        self.slow_ticks = 0
        if s == DECIDE:
            self.open_votes = {'R': 0, 'F': 0, 'L': 0}
            self.vote_count = 0
            self.decide_aligned = False

    def start_turn(self, direction):
        self.turn_goal = {'L': math.pi / 2, 'R': -math.pi / 2, 'B': -math.pi}[direction]
        self.turn_step = math.pi / 2 if direction == 'L' else -math.pi / 2
        self.turn_steps = 2 if direction == 'B' else 1
        self.turn_target = norm_angle(self.grid_heading() + self.turn_step)
        self.set_state(TURN)

    def execute(self, direction, options):
        self.junction_no += 1
        self.get_logger().info(f'#{self.junction_no} open={"".join(options) or "-"} -> {DIR_NAME[direction]}')
        if direction == 'F':
            self.heading_ref = self.grid_heading()
            self.start_segment()
            self.set_state(EXIT)
        else:
            self.start_turn(direction)

    def measure_cell(self, m):
        if m['left_wall'] < 0.6 and m['right_wall'] < 0.6:
            self.cell_samples.append(m['left_wall'] + m['right_wall'])
        if len(self.cell_samples) >= 10:
            width = float(np.median(self.cell_samples))
            self.get_logger().info(f'Measured corridor width {width:.3f} m')
            self.apply_cell(clamp(width, 0.25, 0.60))
        elif self.state_ticks > 2 * self.rate:
            self.get_logger().info('Cannot see both walls -> using cell_size 0.30 m')
            self.apply_cell(0.30)

    def loop(self):
        now = time.time()
        dt = now - self.last_loop_time
        self.last_loop_time = now
        if now - self.last_odom_time > 0.5:
            v, w = self.last_cmd
            self.yaw = norm_angle(self.yaw + w * dt)
            self.x += v * math.cos(self.yaw) * dt
            self.y += v * math.sin(self.yaw) * dt
        if self.scan_r is None:
            return
        m = self.measure()
        self.state_ticks += 1
        if self.cell is None:
            self.publish(0.0, 0.0)
            self.measure_cell(m)
            return
        self.wall_ang = self.lidar_heading_error() if self.use_lidar_angle else None
        if self.state in (FOLLOW, EXIT):
            self.update_grid()
        self.print_status(m, now)
        if self.state == FOLLOW:
            if m['front'] - self.last_cmd[0] * self.sensor_delay < self.front_stop:
                self.publish(0.0, 0.0)
                self.set_state(DECIDE)
                return
            self.left_cnt = self.left_cnt + 1 if m['left'] > self.open_dist else 0
            self.right_cnt = self.right_cnt + 1 if m['right'] > self.open_dist else 0
            self.junc_rem = self.junction_remaining() if self.use_lidar_center else None
            self.ahead_cnt = (self.ahead_cnt + 1 if self.junc_rem is not None
                              and self.junc_rem < 0.75 * self.cell else 0)
            if (self.left_cnt >= self.confirm_n or self.right_cnt >= self.confirm_n
                    or self.ahead_cnt >= self.confirm_n):
                self.get_logger().info('Side opening detected -> moving to junction centre')
                self.heading_ref = self.grid_heading()
                self.start_segment()
                self.adv_dist = self.junc_rem if self.junc_rem is not None else self.center_offset
                self.ahead_cnt = 0
                self.set_state(ADVANCE)
                return
            v = clamp(self.approach_kp * (m['front'] - self.front_stop), self.approach_speed, self.v)
            if self.junc_rem is not None and self.junc_rem < self.cell:
                v = min(v, max(self.approach_speed, self.approach_kp * self.junc_rem))
            self.publish(*self.safe_cmd(v, self.wall_follow_w(m), m))
        elif self.state == ADVANCE:
            if m['front'] < self.front_stop:
                self.publish(0.0, 0.0)
                self.set_state(DECIDE)
                return
            rem = self.junction_remaining() if self.use_lidar_center else None
            if rem is None:
                rem = self.adv_dist - self.traveled()
            rem -= self.last_cmd[0] * self.sensor_delay
            self.junc_rem = rem
            self.slow_ticks = self.slow_ticks + 1 if abs(self.last_cmd[0]) <= self.approach_speed + 1e-3 else 0
            slow_enough = self.slow_ticks > self.sensor_delay * self.rate
            if ((abs(rem) <= self.center_tol and slow_enough)
                    or self.state_ticks > self.adv_ticks_max):
                self.publish(0.0, 0.0)
                self.get_logger().info(f'At junction centre (error {rem * 100:+.1f} cm)')
                self.set_state(DECIDE)
                return
            v = clamp(self.approach_kp * rem, -self.v, self.v)
            if abs(v) < self.approach_speed:
                v = math.copysign(self.approach_speed, v)
            if v > 0:
                self.publish(*self.safe_cmd(v, self.heading_hold(), m))
            else:
                self.publish(v, self.heading_hold())
        elif self.state == DECIDE:
            if not self.decide_aligned:
                w = self.align_w()
                if w and self.state_ticks <= self.align_ticks_max:
                    self.publish(0.0, w)
                    return
                self.publish(0.0, 0.0)
                if w == 0.0:
                    self.update_grid(gain=1.0)
                self.decide_aligned = True
                self.settle_start = self.state_ticks
                return
            self.publish(0.0, 0.0)
            if self.state_ticks - self.settle_start <= self.settle_ticks:
                return
            self.vote_count += 1
            self.open_votes['R'] += m['right'] > self.open_dist and self.side_clear(-1.0)
            self.open_votes['F'] += m['front'] > self.open_dist
            self.open_votes['L'] += m['left'] > self.open_dist and self.side_clear(1.0)
            if self.vote_count < self.decide_samples:
                return
            options = [d for d in ('R', 'F', 'L')
                       if self.open_votes[d] * 2 > self.vote_count]
            ways = len(options) + 1
            self.get_logger().info(
                f'{JUNCTION_NAME[ways]}  open={"".join(options) or "-"}  '
                f'votes R{self.open_votes["R"]} F{self.open_votes["F"]} '
                f'L{self.open_votes["L"]} of {self.vote_count}')
            self.execute(options[0] if options else 'B', options)
        elif self.state == TURN:
            remaining = norm_angle(self.turn_target - self.yaw)
            if abs(remaining) < self.turn_tol:
                self.publish(0.0, 0.0)
                self.align_next = EXIT
                self.set_state(ALIGN)
                return
            w = clamp(2.0 * remaining, -self.w_turn, self.w_turn)
            if abs(w) < 0.2:
                w = math.copysign(0.2, w)
            self.publish(0.0, w)
        elif self.state == ALIGN:
            w = self.align_w()
            if w and self.state_ticks <= self.align_ticks_max:
                self.publish(0.0, w)
                return
            self.publish(0.0, 0.0)
            if w == 0.0:
                self.update_grid(gain=1.0)
            self.turn_steps -= 1
            if self.turn_steps > 0:
                self.turn_target = norm_angle(self.grid_heading() + self.turn_step)
                self.set_state(TURN)
                return
            self.heading_ref = self.grid_heading()
            self.start_segment()
            self.set_state(self.align_next)
        elif self.state == EXIT:
            if m['front'] < self.front_stop:
                self.publish(0.0, 0.0)
                self.set_state(DECIDE)
                return
            if self.traveled() >= self.exit_dist:
                self.left_cnt = self.right_cnt = 0
                if m['left'] > self.open_dist or m['right'] > self.open_dist:
                    self.start_segment()
                    self.adv_dist = 0.0
                    self.set_state(ADVANCE)
                else:
                    self.set_state(FOLLOW)
                return
            self.publish(*self.safe_cmd(self.v, self.wall_follow_w(m), m))

    def print_status(self, m, now):
        if now - self.last_status < 1.0:
            return
        self.last_status = now
        f = lambda d: ' inf' if math.isinf(d) else f'{d:4.2f}'
        ang = '  -- ' if self.wall_ang is None else f'{math.degrees(self.wall_ang):+5.1f}'
        jr = ' --' if self.junc_rem is None else f'{self.junc_rem * 100:+.0f}cm'
        self.get_logger().info(
            f'[{self.state:7s}] F={f(m["front"])} L={f(m["left"])} R={f(m["right"])} '
            f'ang={ang} pts={self.wall_peak} junc={jr}')


def main():
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()
    node = RightWallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nCtrl+C -> stopping robot.')
    except Exception as e:
        print(f'\nStopped: {e}')
    finally:
        try:
            node.stop_robot()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
