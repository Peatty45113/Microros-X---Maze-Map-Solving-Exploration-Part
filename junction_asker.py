#!/usr/bin/env python3
import queue
import sys
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

DIR_NAME = {'R': 'RIGHT', 'L': 'LEFT', 'F': 'FORWARD', 'B': 'BACK'}
KEY_OF = {'F': 'w', 'L': 'a', 'B': 's', 'R': 'd'}
JUNCTION_NAME = {
    1: '1-WAY (dead end)',
    2: '2-WAY (corner / corridor)',
    3: '3-WAY JUNCTION (T-junction)',
    4: '4-WAY JUNCTION (crossroad)',
}
INPUT_MAP = {
    'w': 'F', 'forward': 'F', 'straight': 'F', 'ตรง': 'F',
    'a': 'L', 'left': 'L', 'ซ้าย': 'L',
    'd': 'R', 'right': 'R', 'ขวา': 'R',
    's': 'B', 'back': 'B', 'กลับ': 'B',
}


class JunctionAsker:
    def __init__(self, open_dist=0.30, decide_samples=10, ask_min_ways=3, start_keyboard=True):
        self.open_dist = open_dist
        self.decide_samples = decide_samples
        self.ask_min_ways = ask_min_ways
        self.junction_no = 0
        self.ask_options, self.ask_ways = [], 0
        self.waiting = False
        self.reset_votes()
        self.input_q = queue.Queue()
        if start_keyboard:
            threading.Thread(target=self._stdin_reader, daemon=True).start()

    def _stdin_reader(self):
        while True:
            line = sys.stdin.readline()
            if line == '':
                return
            self.input_q.put(line.strip().lower())

    def reset_votes(self):
        self.open_votes = {'R': 0, 'F': 0, 'L': 0}
        self.vote_count = 0

    def add_vote(self, left, front, right, left_clear=True, right_clear=True):
        self.vote_count += 1
        self.open_votes['R'] += right > self.open_dist and right_clear
        self.open_votes['F'] += front > self.open_dist
        self.open_votes['L'] += left > self.open_dist and left_clear

    def votes_done(self):
        return self.vote_count >= self.decide_samples

    def result(self):
        options = [d for d in ('R', 'F', 'L') if self.open_votes[d] * 2 > self.vote_count]
        return len(options) + 1, options

    def auto_choice(self, options):
        return options[0] if options else 'B'

    @staticmethod
    def junction_picture(opts):
        top = '[w] FORWARD' if 'F' in opts else '  (wall)   '
        stem = '|' if 'F' in opts else '='
        left = '[a] LEFT ---' if 'L' in opts else '    (wall) |'
        right = '--- [d] RIGHT' if 'R' in opts else '| (wall)'
        return [
            '              ' + top,
            '                   ' + stem,
            '  ' + left + ' ROBOT ' + right,
            '                   |',
            '               (came from)',
        ]

    def ask(self, options, ways):
        self.ask_options, self.ask_ways, self.waiting = options, ways, True
        while not self.input_q.empty():
            self.input_q.get_nowait()
        self.prompt()

    def prompt(self):
        print('\n' + '=' * 60)
        print(f'  {JUNCTION_NAME.get(self.ask_ways, "JUNCTION")}   '
              f'#{self.junction_no + 1}   - robot stopped')
        print('=' * 60)
        for line in self.junction_picture(self.ask_options):
            print(line)
        order = [d for d in ('L', 'R', 'F') if d in self.ask_options] + ['B']
        print('  Choose: ' + ' / '.join(f'[{KEY_OF[d]}] {DIR_NAME[d]}' for d in order))
        print(f'  (just Enter = {DIR_NAME[self.ask_options[0]]}, right-hand rule)')
        print('> ', end='', flush=True)

    def poll(self):
        if not self.waiting:
            return None
        try:
            ans = self.input_q.get_nowait()
        except queue.Empty:
            return None
        if ans == '':
            choice = self.ask_options[0]
        else:
            choice = INPUT_MAP.get(ans)
        if choice is None:
            print(f'  "{ans}" not understood.')
            self.prompt()
            return None
        if choice not in self.ask_options and choice != 'B':
            print(f'  {DIR_NAME[choice]} looks blocked by the LiDAR - going anyway.')
        self.waiting = False
        self.junction_no += 1
        return choice


class JunctionAskDemo(Node):
    def __init__(self):
        super().__init__('junction_asker')
        self.declare_parameter('cell_size', 0.30)
        cell = float(self.get_parameter('cell_size').value)
        self.asker = JunctionAsker(open_dist=cell)
        self.r = self.a = self.ok = None
        self.last = None
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.create_timer(0.05, self.loop)
        self.get_logger().info('Junction asker demo: put the robot at a junction (it will not move).')

    def scan_cb(self, msg):
        r = np.asarray(msg.ranges, dtype=float)
        ang = np.degrees(msg.angle_min + np.arange(len(r)) * msg.angle_increment)
        self.a = (ang + 180.0) % 360.0 - 180.0
        self.ok = np.isfinite(r) & (r > max(msg.range_min, 0.02)) & (r < msg.range_max)
        self.r = r

    def sector(self, lo, hi, stat):
        m = self.ok & (self.a >= lo) & (self.a <= hi)
        if not m.any():
            return float('inf')
        return float(np.percentile(self.r[m], 10)) if stat == 'p10' else float(np.median(self.r[m]))

    def loop(self):
        if self.r is None:
            return
        A = self.asker
        if A.waiting:
            choice = A.poll()
            if choice:
                self.get_logger().info(f'-> {DIR_NAME[choice]} (your choice). Voting again ...')
                A.reset_votes()
            return
        A.add_vote(left=self.sector(80, 100, 'median'),
                   front=self.sector(-15, 15, 'p10'),
                   right=self.sector(-100, -80, 'median'))
        if not A.votes_done():
            return
        ways, options = A.result()
        if ways >= A.ask_min_ways and options:
            self.get_logger().info(f'{JUNCTION_NAME[ways]}  open={"".join(options)}')
            A.ask(options, ways)
            self.last = None
        else:
            if options != self.last:
                self.get_logger().info(f'{JUNCTION_NAME[ways]}  open={"".join(options) or "-"}  '
                                       f'-> {DIR_NAME[A.auto_choice(options)]} (auto, no question)')
                self.last = options
            A.reset_votes()


def main():
    rclpy.init()
    node = JunctionAskDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
