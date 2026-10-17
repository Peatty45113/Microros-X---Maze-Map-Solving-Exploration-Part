#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ↑ บรรทัดแรก: บอก Linux ให้ใช้ python3 รันไฟล์นี้ | บรรทัดสอง: ไฟล์นี้เป็น UTF-8 (เขียนภาษาไทยได้)
# ==============================================================================
#  maze_explorer.py  (เวอร์ชัน 4: LiDAR คุมให้วิ่งขนานกำแพงตลอด)
#
#  สำรวจเขาวงกตที่ไม่รู้จักแบบ "เดินเกาะผนังขวา" (Right Wall Follower)
#  เขาวงกต 10 x 10 ช่อง ทางเดินกว้าง 30 หรือ 40 cm (วัดเองอัตโนมัติตอนเริ่มได้)
#  หุ่นยนต์ : Yahboom MicroROS + LiDAR, ROS 2 + Gmapping
# ==============================================================================
#
#  โปรแกรมนี้ทำอะไร
#  -----------------
#   1. เดินตามทางเดิน รักษาระยะกลางทาง (ตัวควบคุม P, Kp = 2.5)
#   2. ทางโค้ง (2 ทาง) / ทางตัน (1 ทาง) -> เลี้ยว / กลับหลังเอง
#   3. สามแยก / สี่แยก -> หยุด วาดรูปทางแยก แล้วถามเราใน Terminal (w a s d)
#   4. Gmapping (Terminal อื่น) สร้างแผนที่ระหว่างที่หุ่นยนต์เดิน
#   5. ถึงเส้นชัยกด Ctrl+C -> หยุดหุ่นยนต์ + บันทึกแผนที่อัตโนมัติ (เหมือนสไลด์ 8 P7-7)
#
#  ★★★ LiDAR ทำให้หุ่นยนต์วิ่งตรงขนานกำแพงได้อย่างไร (ส่วนสำคัญของเวอร์ชันนี้) ★★★
#  ในโค้ด ทุกบรรทัดที่เกี่ยวข้องมีเครื่องหมาย ★ นำหน้าคำอธิบาย
#   ก) wall_score() : หมุนจุด LiDAR รอบตัวไปเป็นมุมทดลอง แล้วนับว่าจุดตกลงใน
#                   "แถว" เดียวกัน (กว้าง 1 cm) มากแค่ไหน กำแพงในเขาวงกตเป็นมุมฉากทั้งหมด
#                   ที่มุมที่ถูกต้อง จุดของกำแพงทุกด้าน (ซ้าย ขวา หน้า หลัง) จะเรียงเป็นแถวพร้อมกัน
#   ข) lidar_heading_error() : ลองมุม -30 ถึง +30 องศา หามุมที่จุดเรียงแถวดีที่สุด
#                   -> self.wall_ang = หุ่นยนต์เอียงจากกำแพงกี่องศา
#                   0 องศา = ขนานกำแพงพอดี, บวก = ต้องหมุนซ้าย, ลบ = ต้องหมุนขวา
#                   (มุมห้อง ปลายกำแพง ไม้ต่อ ไม่ทำให้วัดผิด เพราะใช้กำแพงทุกด้านพร้อมกัน)
#   ค) wall_follow_w() : ขณะเดิน ใช้ self.wall_ang แก้มุมทุก 0.05 วินาที
#                   หุ่นยนต์จึงขนานกำแพงตลอด แม้ odom จะเพี้ยน
#      และจำกัดแรงดึงเข้ากลางทาง (max_push) ไว้ไม่ให้ชนะแรงคุมมุม หุ่นยนต์จึงไม่เดินเฉียง
#      กลับเข้ากลางทาง แต่ค่อย ๆ ไหลกลับขณะที่ตัวยังขนานกำแพง
#   ง) สถานะ ALIGN : หลังเลี้ยว / กลับหลัง ทุกครั้ง หุ่นยนต์หมุนอยู่กับที่ต่อ
#                   จนกว่า LiDAR บอกว่าตรงกับกำแพง (คลาดไม่เกิน 1.5 องศา)
#                   แก้ปัญหาล้อลื่นตอนหมุน (odom บอก 90 องศา แต่หมุนจริงได้น้อยกว่า)
#                   กลับหลัง = หมุน 90 องศา 2 ครั้ง จัดตัวด้วย LiDAR ทุกครั้ง (ล้อลื่นแค่ไหนก็ไม่หลงทิศ)
#   จ) DECIDE ขั้นที่ 1 : ก่อนตัดสินใจที่ทางแยก จัดตัวให้ตรงด้วย LiDAR ก่อนเสมอ
#   ฉ) update_grid() : ใช้มุมกำแพงแก้ทิศของตารางเขาวงกต การเลี้ยวครั้งต่อไปจึงแม่น
#   ช) safe_cmd() : ถ้ามีจุด LiDAR อยู่ใน "ทางที่ตัวหุ่นยนต์กำลังจะวิ่งผ่าน" (กล่องกว้างเท่าตัว
#                   + 2 cm ด้านหน้า) แปลว่ากำลังวิ่งเฉียงเข้าหากำแพง -> ช้าลงและหมุนหนี
#                   กำแพงที่ขนานอยู่ข้างตัวไม่อยู่ในกล่องนี้ จึงไม่ทำให้หุ่นยนต์ส่ายโดยไม่จำเป็น
#
#  วิธีรัน (ใช้แทน P7-6 คีย์บอร์ด และ P7-7 บันทึกแผนที่ ในสไลด์ 8)
#   T1: sh start_agent_computer.sh                                  (P7-2)
#   T2: ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py (P7-3)
#   T3: ros2 launch yahboomcar_nav display_launch.py                (P7-4)
#   T4: ros2 launch yahboomcar_nav map_gmapping_launch.py           (P7-5)
#   T5: python3 maze_explorer.py                          (วัดความกว้างทางเดินเอง)
#       python3 maze_explorer.py --ros-args -p cell_size:=0.40   (บอกเองว่ากว้าง 40 cm)
#
#  วางหุ่นยนต์ที่จุดเริ่ม: ให้อยู่ระหว่างกำแพงสองข้าง หันตามทางเดินโดยประมาณ
#  (เอียงได้ ~20 องศา โปรแกรมจะจัดตัวให้ตรงด้วย LiDAR ก่อนออกเดิน)
#
#  INPUT  : /scan (LiDAR), /odom (ตำแหน่ง), คีย์บอร์ด (w/a/s/d/Enter), Ctrl+C
#  OUTPUT : /cmd_vel (linear.x = เดินหน้า m/s, angular.z = หมุน rad/s; + ซ้าย, - ขวา)
#           /beep (ไม่บังคับ), ข้อความใน Terminal, ไฟล์แผนที่ .pgm/.yaml + บันทึกเส้นทาง
#
#  ทิศทางในไฟล์นี้ (มองจากตัวหุ่นยนต์):  0 องศา = หน้า, +90 = ซ้าย, -90 = ขวา, 180 = หลัง
#  พิกัดจุด LiDAR: x = ไปข้างหน้า, y = ไปทางซ้าย (หน่วยเมตร)
# ==============================================================================

import math                      # คณิตศาสตร์: sin, cos, atan2, pi, hypot
import os                        # จัดการไฟล์และโฟลเดอร์
import queue                     # คิวส่งคำตอบจากคีย์บอร์ดข้าม thread อย่างปลอดภัย
import subprocess                # สั่งรันคำสั่ง Terminal (ตอนบันทึกแผนที่)
import sys                       # อ่านคีย์บอร์ดผ่าน sys.stdin
import threading                 # ให้การอ่านคีย์บอร์ดทำงานคู่ขนานกับการคุมหุ่นยนต์
import time                      # เวลาปัจจุบัน และ sleep
from datetime import datetime    # วันเวลา ใช้ตั้งชื่อไฟล์แผนที่

import numpy as np                               # คำนวณ array ของจุด LiDAR ได้เร็ว
import rclpy                                     # ไลบรารี ROS 2 สำหรับ Python
from rclpy.node import Node                      # คลาสแม่ของโหนด ROS 2
from rclpy.qos import qos_profile_sensor_data    # QoS แบบ best effort สำหรับเซนเซอร์
from geometry_msgs.msg import Twist              # ข้อความคำสั่งความเร็ว
from std_msgs.msg import UInt16                  # ข้อความสั่งบี๊บ
from sensor_msgs.msg import LaserScan            # ข้อความ LiDAR
from nav_msgs.msg import Odometry                # ข้อความตำแหน่งหุ่นยนต์


# ==============================================================================
#  ฟังก์ชันช่วยเล็ก ๆ
# ==============================================================================
def norm_angle(a):                               # บีบมุม a (เรเดียน) ให้อยู่ในช่วง -pi ถึง +pi
    return math.atan2(math.sin(a), math.cos(a))  # sin/cos แล้ว atan2 กลับ = มุมเดิมในช่วง [-pi, pi]


def clamp(v, lo, hi):                            # จำกัดค่า v ให้อยู่ระหว่าง lo กับ hi
    return max(lo, min(hi, v))                   # ตัดส่วนที่เกิน hi และต่ำกว่า lo ทิ้ง


# ==============================================================================
#  STATE MACHINE: หุ่นยนต์อยู่ในสถานะเดียวเสมอ
#
#   (เริ่ม) ALIGN -> FOLLOW
#   FOLLOW --(กำแพงข้างหน้า)----------------------> DECIDE
#   FOLLOW --(เห็นทางเปิดซ้าย/ขวา)--> ADVANCE --> DECIDE
#   DECIDE --(1 ทาง: ทางตัน)--------------------> TURN (กลับหลัง)
#   DECIDE --(2 ทาง: ทางโค้ง)--------------------> TURN หรือ EXIT
#   DECIDE --(สามแยก / สี่แยก)---------> ASK --> TURN หรือ EXIT
#   TURN   --(odom บอกว่าหมุนครบ)----------------> ALIGN ★ (LiDAR จัดตัวให้ตรง)
#   ALIGN  --(ตรงกับกำแพงแล้ว)--------------------> EXIT
#   EXIT   --(เดินออกครบ 1 ช่อง)------------------> FOLLOW หรือ DECIDE
# ==============================================================================
FOLLOW = 'FOLLOW'     # เดินตามทางเดิน
ADVANCE = 'ADVANCE'   # เห็นทางเปิดด้านข้าง เดินต่อให้ตัวหุ่นยนต์อยู่กลางทางแยก
DECIDE = 'DECIDE'     # หยุด จัดตัวตรง ดูว่าทางไหนเปิด แล้วตัดสินใจ
ASK = 'ASK'           # หยุด รอคำตอบจากเราใน Terminal
TURN = 'TURN'         # หมุนอยู่กับที่ 90 หรือ 180 องศา (ตาม odom)
ALIGN = 'ALIGN'       # ★ หมุนอยู่กับที่ทีละนิด จนกว่า LiDAR บอกว่าขนานกำแพง
EXIT = 'EXIT'         # เดินออกจากทางแยก 1 ช่อง โดยไม่สนทางเปิดระหว่างนั้น

DIR_NAME = {'R': 'RIGHT', 'L': 'LEFT', 'F': 'FORWARD', 'B': 'BACK'}   # รหัสย่อ -> ชื่อที่แสดง
KEY_OF = {'F': 'w', 'L': 'a', 'B': 's', 'R': 'd'}                     # รหัสย่อ -> ปุ่มคีย์บอร์ด
JUNCTION_NAME = {                          # จำนวนทาง (รวมทางที่เดินมา) -> ชื่อทางแยก
    1: '1-WAY (dead end)',                 # ทางตัน
    2: '2-WAY (corner / corridor)',        # ทางโค้ง / ทางตรง
    3: '3-WAY JUNCTION (T-junction)',      # สามแยก
    4: '4-WAY JUNCTION (crossroad)',       # สี่แยก
}                                          # ปิด dict
INPUT_MAP = {                                                  # สิ่งที่พิมพ์ได้ -> รหัสทิศทาง
    'w': 'F', 'forward': 'F', 'straight': 'F', 'ตรง': 'F',    # W = ตรงไป
    'a': 'L', 'left': 'L', 'ซ้าย': 'L',                         # A = ซ้าย
    'd': 'R', 'right': 'R', 'ขวา': 'R',                         # D = ขวา
    's': 'B', 'back': 'B', 'กลับ': 'B',                         # S = กลับหลัง
}                                                              # ปิด dict


# ==============================================================================
#  โหนด ROS 2 หลัก
# ==============================================================================
class MazeExplorer(Node):                    # โหนด ROS 2 (สืบทอดจาก Node)

    def __init__(self):                      # ทำงานครั้งเดียวตอนเริ่มโปรแกรม
        super().__init__('maze_explorer')    # ตั้งชื่อโหนดว่า maze_explorer

        # ------------------------------------------------------------------
        # 1) พารามิเตอร์ (เปลี่ยนตอนรันได้ด้วย --ros-args -p ชื่อ:=ค่า)
        # ------------------------------------------------------------------
        p = self.declare_parameter   # ชื่อสั้นของฟังก์ชันประกาศพารามิเตอร์

        p('scan_topic', '/scan')          # topic LiDAR (input)
        p('cmd_vel_topic', '/cmd_vel')    # topic คำสั่งความเร็ว (output)
        p('odom_topic', '/odom')          # topic odometry (input)
        p('control_rate', 20.0)           # loop() ทำงาน 20 ครั้งต่อวินาที

        p('cell_size', 0.0)           # ความกว้างทางเดิน/ขนาดช่อง [m]: 0.30 หรือ 0.40; 0 = วัดเองตอนเริ่ม
        p('linear_speed', 0.15)       # ความเร็วเดินหน้า [m/s] (เดิม 0.08 ช้าไป; สไลด์ 8 แนะนำราว 0.1)
        p('turn_speed', 0.5)          # ความเร็วหมุนสูงสุด [rad/s] (เดิม 0.5; สไลด์ 8 แนะนำราว 0.5)
        p('kp', 2.5)                  # ค่า P คุมระยะกลางทางเดิน
        p('heading_kp', 1.5)          # ★ ค่า P คุมมุม (ให้ขนานกำแพง)
        p('max_correction', 0.4)      # ขณะเดิน หมุนแก้ได้มากสุด [rad/s]
        p('max_push', 0.10)           # ★ แรงดึงเข้ากลางทางสูงสุด [rad/s] (น้อยกว่าแรงคุมมุม -> ตัวยังขนานกำแพง)
        p('wall_near_ratio', 0.75)    # ★ กำแพงข้างตัวต้องใกล้กว่า 0.75 x ความกว้างทาง จึงใช้คุมกลางทาง
        p('front_gap', 0.02)          # หยุดเมื่อกำแพงหน้าอยู่ห่าง (ครึ่งช่อง + ค่านี้) [m]
        p('open_confirm_ticks', 3)    # ต้องเห็นทางเปิดติดกัน 3 รอบจึงเชื่อ
        p('turn_tolerance_deg', 3.0)  # TURN ถือว่าเสร็จเมื่อเหลือน้อยกว่า 3 องศา (ตาม odom)

        p('settle_time', 0.3)         # จัดตัวตรงแล้ว รอนิ่งอีก 0.3 วินาที ก่อนโหวตทางแยก
        p('decide_samples', 10)       # โหวตทางแยก 10 รอบ (0.5 วินาที)
        p('ask_min_ways', 3)          # ถามเราเมื่อทางแยก >= 3 ทาง (2 = ถามทางโค้งด้วย, 5 = ไม่ถาม)

        p('use_lidar_angle', True)    # ★ ใช้ LiDAR วัดมุมกำแพง (หัวใจของการวิ่งตรง)
        p('wall_min_pts', 10)         # ★ กำแพงที่ดีที่สุดต้องมีจุด LiDAR เรียงแถวอย่างน้อย 10 จุด
        p('wall_bin', 0.01)           # ★ ความกว้างของ "แถว" ที่ใช้นับจุด [m] (1 cm)
        p('wall_search_deg', 30.0)    # ★ ลองมุมตั้งแต่ -30 ถึง +30 องศา
        p('wall_range_cells', 1.5)    # ★ ใช้จุดที่อยู่ไม่เกิน 1.5 ช่องรอบตัว
        p('ignore_below', 0.08)       # ★ ไม่ใช้จุดที่ใกล้กว่า 8 cm (อาจเป็นเสาอากาศ/ตัวหุ่นยนต์เอง)
        p('grid_corr_gain', 0.05)     # ★ ขณะเดิน ใช้มุมกำแพงแก้ทิศตารางทีละ 5% ต่อรอบ
        p('align_tolerance_deg', 1.5) # ★ ALIGN เสร็จเมื่อเอียงจากกำแพงไม่เกิน 1.5 องศา
        p('align_kp', 2.0)            # ★ ความแรงในการหมุนจัดตัว (rad/s ต่อ rad ที่เอียง)
        p('align_min_w', 0.20)        # ★ หมุนจัดตัวอย่างน้อย 0.20 rad/s (0.12 มอเตอร์ไม่ขยับ)
        p('align_timeout', 3.0)       # ★ จัดตัวนานสุด 3 วินาที แล้วไปต่อ (กันค้าง)

        p('use_safety', True)         # เปิดระบบหลบกำแพงเมื่อเข้าใกล้เกินไป
        p('robot_half_width', 0.08)   # ครึ่งความกว้างตัวหุ่นยนต์ วัดจากจุดกลาง LiDAR [m] (ตัวกว้าง ~16 cm)
        p('robot_front', 0.10)        # ระยะจากจุดกลาง LiDAR ถึงหน้าสุดของตัวหุ่นยนต์ [m]
        p('safe_margin', 0.02)        # ระยะเผื่อด้านข้างตัว [m]
        p('k_avoid', 6.0)             # แรงหมุนหนี (rad/s ต่อเมตรที่กำแพงล้ำเข้ามาในกล่อง)
        p('safe_speed', 0.05)         # ความเร็วเดินหน้าขณะหลบ [m/s] (เดิม 0.03)

        p('beep_on_ask', False)       # บี๊บตอนถามหรือไม่
        p('beep_topic', '/beep')      # topic เสียงบี๊บ (ดู test_beep.py)
        p('beep_ms', 100)             # ความยาวเสียงบี๊บ
        p('lidar_front_deg', 0.0)     # มุม LiDAR ที่ชี้ไปหน้าหุ่นยนต์ (ถ้าผิดลอง 180, 90, -90)
        p('lidar_ccw', True)          # True = มุม LiDAR เพิ่มทวนเข็มนาฬิกา (มาตรฐาน ROS)
        p('save_method', 'both')      # บันทึกแผนที่: 'launch' = สไลด์ 8, 'cli' = สำรอง, 'both' = ทั้งคู่
        p('map_dir', os.path.expanduser('~/yahboomcar_ws/src/yahboomcar_nav/maps'))  # โฟลเดอร์เก็บแผนที่

        g = lambda n: self.get_parameter(n).value          # g(ชื่อ) = อ่านค่าพารามิเตอร์
        self.rate = float(g('control_rate'))               # ความถี่ loop
        self.v = float(g('linear_speed'))                  # ความเร็วเดินหน้า
        self.w_turn = float(g('turn_speed'))               # ความเร็วหมุนสูงสุด
        self.kp = float(g('kp'))                           # ค่า P ระยะ
        self.kh = float(g('heading_kp'))                   # ★ ค่า P มุม
        self.max_corr = float(g('max_correction'))         # หมุนแก้สูงสุดขณะเดิน
        self.max_push = float(g('max_push'))               # ★ แรงดึงเข้ากลางทางสูงสุด
        self.wall_near_ratio = float(g('wall_near_ratio')) # ★ เกณฑ์ว่ากำแพงข้างตัวอยู่ใกล้พอ
        self.front_gap = float(g('front_gap'))             # ระยะเผื่อหน้ากำแพง
        self.confirm_n = int(g('open_confirm_ticks'))      # รอบยืนยันทางเปิด
        self.turn_tol = math.radians(float(g('turn_tolerance_deg')))   # แปลงองศา -> เรเดียน
        self.settle_ticks = int(float(g('settle_time')) * self.rate)   # แปลงวินาที -> จำนวนรอบ loop
        self.decide_samples = int(g('decide_samples'))     # จำนวนรอบโหวต
        self.ask_min_ways = int(g('ask_min_ways'))         # จำนวนทางขั้นต่ำที่จะถาม
        self.use_lidar_angle = bool(g('use_lidar_angle'))  # ★ เปิด/ปิดการใช้มุมกำแพง
        self.wall_min_pts = int(g('wall_min_pts'))         # ★ จุดขั้นต่ำบนกำแพงที่ดีที่สุด
        self.wall_bin = float(g('wall_bin'))               # ★ ความกว้างแถว
        self.wall_search = math.radians(float(g('wall_search_deg')))   # ★ ช่วงมุมที่ลอง (เรเดียน)
        self.wall_range_cells = float(g('wall_range_cells'))           # ★ ระยะจุดที่ใช้ (หน่วย: ช่อง)
        self.ignore_below = float(g('ignore_below'))       # ★ ระยะใกล้สุดที่ใช้
        self.grid_corr_gain = float(g('grid_corr_gain'))   # ★ ความเร็วแก้ทิศตาราง
        self.align_tol = math.radians(float(g('align_tolerance_deg')))   # ★ ค่ายอมรับของ ALIGN (เรเดียน)
        self.align_kp = float(g('align_kp'))               # ★ ความแรงหมุนจัดตัว
        self.align_min_w = float(g('align_min_w'))         # ★ ความเร็วหมุนจัดตัวขั้นต่ำ
        self.align_ticks_max = int(float(g('align_timeout')) * self.rate)   # ★ เวลาจัดตัวสูงสุด (รอบ)
        self.use_safety = bool(g('use_safety'))            # เปิด/ปิดระบบหลบกำแพง
        self.half_w = float(g('robot_half_width'))         # ครึ่งความกว้างตัว
        self.robot_front = float(g('robot_front'))         # ระยะถึงหน้าตัวหุ่นยนต์
        self.safe_margin = float(g('safe_margin'))         # ระยะเผื่อ
        self.k_avoid = float(g('k_avoid'))                 # แรงหมุนหนี
        self.safe_speed = float(g('safe_speed'))           # ความเร็วขณะหลบ
        self.beep_on_ask = bool(g('beep_on_ask'))          # บี๊บหรือไม่
        self.beep_ms = int(g('beep_ms'))                   # ความยาวบี๊บ
        self.lidar_front = float(g('lidar_front_deg'))     # มุมหน้าของ LiDAR
        self.lidar_sign = 1.0 if bool(g('lidar_ccw')) else -1.0     # +1 ทวนเข็ม, -1 ตามเข็ม
        self.save_method = str(g('save_method'))           # วิธีบันทึกแผนที่
        self.map_dir = str(g('map_dir'))                   # โฟลเดอร์แผนที่

        self.cell = None                                   # ขนาดช่อง [m] (None = ยังไม่รู้ ต้องวัดก่อน)
        self.cell_samples = []                             # ค่าความกว้างทางเดินที่วัดได้ตอนเริ่ม
        if float(g('cell_size')) > 0:                      # ถ้าผู้ใช้บอกขนาดช่องมาเอง
            self.apply_cell(float(g('cell_size')))         # ... คำนวณระยะต่าง ๆ จากขนาดนั้นเลย

        # ------------------------------------------------------------------
        # 2) ROS INPUT / OUTPUT
        # ------------------------------------------------------------------
        self.pub = self.create_publisher(Twist, g('cmd_vel_topic'), 10)   # OUTPUT: ส่งความเร็วไป /cmd_vel
        self.create_subscription(LaserScan, g('scan_topic'), self.scan_cb, qos_profile_sensor_data)  # INPUT: /scan -> scan_cb
        self.create_subscription(Odometry, g('odom_topic'), self.odom_cb, qos_profile_sensor_data)   # INPUT: /odom -> odom_cb (best effort รับได้ทั้งสองแบบ)
        self.timer = self.create_timer(1.0 / self.rate, self.loop)              # เรียก loop() ทุก 0.05 วินาที
        self.beep_pub = (self.create_publisher(UInt16, g('beep_topic'), 10)     # OUTPUT (ไม่บังคับ): เสียงบี๊บ
                         if self.beep_on_ask else None)                         # ... สร้างเฉพาะเมื่อเปิดใช้

        # ------------------------------------------------------------------
        # 3) ตัวแปรภายใน (หน่วยความจำของหุ่นยนต์)
        # ------------------------------------------------------------------
        self.scan_r = None     # ระยะของแต่ละลำแสง LiDAR [m]
        self.scan_a = None     # มุมของแต่ละลำแสง เทียบกับตัวหุ่นยนต์ [องศา]
        self.scan_ok = None    # ลำแสงนี้ใช้ได้หรือไม่ (True/False)

        self.x = self.y = self.yaw = 0.0     # ตำแหน่ง [m] และมุมหัน [rad] จาก /odom
        self.last_odom_time = 0.0            # เวลาที่ได้ /odom ครั้งล่าสุด
        self.last_cmd = (0.0, 0.0)           # คำสั่ง (v, w) ล่าสุด
        self.last_loop_time = time.time()    # เวลาที่ loop() ทำงานครั้งล่าสุด

        self.state = ALIGN           # ★ เริ่มด้วยการจัดตัวให้ตรงกับทางเดินก่อน
        self.align_next = FOLLOW     # ★ จัดตัวเสร็จแล้วไปสถานะนี้
        self.state_ticks = 0         # อยู่ในสถานะนี้มากี่รอบแล้ว
        self.seg_x = self.seg_y = 0.0  # จุดเริ่มวัดระยะที่เดิน
        self.heading_ref = None      # ทิศ (odom) ที่ต้องรักษาตอนไม่เห็นกำแพง
        self.left_cnt = 0            # จำนวนรอบติดกันที่ซ้ายเปิด
        self.right_cnt = 0           # จำนวนรอบติดกันที่ขวาเปิด
        self.turn_goal = 0.0         # มุมที่ต้องหมุน [rad]
        self.turn_target = 0.0       # ทิศเป้าหมายหลังหมุน [rad]
        self.turn_step = 0.0         # ★ หมุนครั้งละเท่าไร (+90 / -90 องศา)
        self.turn_steps = 0          # ★ ยังต้องหมุนอีกกี่ครั้ง (กลับหลัง = 2 ครั้ง)
        self.grid_off = 0.0          # ★ ทิศของตารางเขาวงกตใน odom (LiDAR แก้ให้) [rad]
        self.wall_ang = None         # ★ หุ่นยนต์เอียงจากกำแพงกี่ rad (None = ไม่เห็นกำแพงตรง)
        self.wall_peak = 0           # ★ จำนวนจุดบนกำแพงที่เรียงแถวดีที่สุด (มากยิ่งเชื่อได้)
        self.decide_aligned = False  # ★ DECIDE: จัดตัวตรงแล้วหรือยัง
        self.settle_start = 0        # DECIDE: รอบที่เริ่มรอนิ่ง
        self.avoiding = False        # กำลังหลบกำแพงอยู่หรือไม่
        self.ask_options = []        # ทิศที่เปิดที่ทางแยกที่กำลังถาม
        self.ask_ways = 0            # ชนิดทางแยกที่กำลังถาม (3 หรือ 4)
        self.open_votes = {'R': 0, 'F': 0, 'L': 0}   # คะแนนโหวตทางเปิด
        self.vote_count = 0                          # จำนวนรอบที่โหวตแล้ว
        self.junction_no = 0         # จำนวนครั้งที่ตัดสินใจ
        self.route_log = []          # บันทึกเส้นทาง
        self.last_status = 0.0       # เวลาที่พิมพ์สถานะครั้งล่าสุด
        self.start_time = time.time()  # เวลาเริ่มโปรแกรม

        self.input_q = queue.Queue()                                       # คิวเก็บคำตอบจากคีย์บอร์ด
        threading.Thread(target=self._stdin_reader, daemon=True).start()   # thread อ่านคีย์บอร์ด (ไม่ขวาง loop)

        self.get_logger().info('Maze explorer v4 started. Press Ctrl+C at the goal to stop and save the map.')  # บอกว่าเริ่มแล้ว

    def apply_cell(self, cell):                      # ตั้งขนาดช่อง แล้วคำนวณระยะทั้งหมดจากขนาดนี้
        self.cell = cell                             # ขนาดช่อง = ความกว้างทางเดิน [m] (0.30 หรือ 0.40)
        self.open_dist = cell                        # ทิศไหนว่างไกลกว่า 1 ช่อง = "ทางเปิด"
        self.side_target = cell / 2.0                # มีกำแพงข้างเดียว -> ห่างกำแพงครึ่งช่อง (= กลางทาง)
        self.front_stop = cell / 2.0 + self.front_gap   # หยุดเมื่อกำแพงหน้าห่างครึ่งช่อง + 2 cm (ตัวอยู่กลางช่อง)
        self.center_offset = cell / 2.0              # เห็นทางเปิดแล้วเดินต่อครึ่งช่อง (ถึงกลางทางแยก)
        self.exit_dist = cell                        # ออกจากทางแยก 1 ช่อง
        self.wall_near = self.wall_near_ratio * cell # ★ กำแพงข้างตัวใกล้กว่านี้ = กำแพงทางเดิน (0.225 / 0.30 m)
        self.get_logger().info(f'Cell size {cell:.2f} m -> open>{self.open_dist:.2f} '   # พิมพ์ค่าที่ใช้
                               f'stop<{self.front_stop:.2f} side={self.side_target:.2f}')  # ... ให้ตรวจสอบได้

    # ==========================================================================
    #  CALLBACKS
    # ==========================================================================
    def scan_cb(self, msg: LaserScan):       # ถูกเรียกเมื่อมี LiDAR scan ใหม่
        r = np.asarray(msg.ranges, dtype=float)                        # ระยะทุกลำแสง -> numpy array
        ang = msg.angle_min + np.arange(len(r)) * msg.angle_increment  # มุมแต่ละลำแสง [rad]
        ang = np.degrees(ang) * self.lidar_sign - self.lidar_front     # แปลงเป็นองศาเทียบหน้าหุ่นยนต์
        ang = (ang + 180.0) % 360.0 - 180.0                            # บีบให้อยู่ใน -180..180 องศา
        ok = np.isfinite(r) & (r > 0.01) & (r < msg.range_max)  # ลำแสงที่ใช้ได้ (เก็บค่าที่ใกล้มากไว้ด้วย เพราะนั่นคือ "กำลังจะชน")
        self.scan_r, self.scan_a, self.scan_ok = r, ang, ok            # เก็บไว้ใช้ใน loop()

    def odom_cb(self, msg: Odometry):        # ถูกเรียกเมื่อมี /odom ใหม่
        self.x = msg.pose.pose.position.x    # ตำแหน่ง x [m]
        self.y = msg.pose.pose.position.y    # ตำแหน่ง y [m]
        q = msg.pose.pose.orientation        # ทิศ (quaternion)
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))  # quaternion -> yaw [rad]
        self.last_odom_time = time.time()    # จำเวลาที่ได้

    def _stdin_reader(self):                 # thread แยก: อ่านคีย์บอร์ดตลอดเวลา
        while True:                          # วนไม่รู้จบ
            line = sys.stdin.readline()      # รอจนกด Enter
            if line == '':                   # stdin ถูกปิด
                return                       # ... จบ thread
            self.input_q.put(line.strip().lower())   # ตัดช่องว่าง ทำตัวเล็ก ใส่คิว

    # ==========================================================================
    #  LiDAR: ระยะ
    # ==========================================================================
    def sector(self, lo, hi, stat='median', empty=float('inf')):   # สรุประยะในช่วงมุม lo..hi องศา ให้เหลือค่าเดียว
        m = self.scan_ok & (self.scan_a >= lo) & (self.scan_a <= hi)   # เลือกลำแสงที่ใช้ได้ในช่วงมุม
        vals = self.scan_r[m]                    # ระยะของลำแสงเหล่านั้น
        if vals.size == 0:                       # ไม่มีลำแสงที่ใช้ได้เลย
            return empty                         # ... ด้านข้าง: inf (ทางเปิด) / ด้านหน้า: 0 (ชิดจน LiDAR วัดไม่ได้ = ตัน)
        if stat == 'min':                        # แบบใกล้สุด
            return float(np.min(vals))           # ... จุดที่ใกล้ที่สุด
        if stat == 'p10':                        # แบบเปอร์เซ็นไทล์ 10
            return float(np.percentile(vals, 10))   # ... ใกล้ แต่ไม่สนจุดรบกวน 1-2 จุด
        return float(np.median(vals))            # แบบค่ากลาง

    def measure(self):                           # วัดระยะที่ต้องใช้ทั้งหมดจาก scan ล่าสุด
        return {                                         # คืนค่าเป็น dict [m]
            'front': self.sector(-15, 15, 'p10', empty=0.0),   # กำแพงข้างหน้า (ไม่มีข้อมูล = ถือว่าตัน กันดันกำแพง)
            'left': self.sector(80, 100, 'median'),      # ที่ว่างทางซ้ายตรง ๆ (ดูทางเปิด)
            'right': self.sector(-100, -80, 'median'),   # ที่ว่างทางขวาตรง ๆ (ดูทางเปิด)
            'left_wall': self.sector(70, 110, 'p10'),    # ระยะถึงกำแพงซ้าย (คุมกลางทาง)
            'right_wall': self.sector(-110, -70, 'p10'), # ระยะถึงกำแพงขวา (คุมกลางทาง)
        }                                                # ปิด dict

    # ==========================================================================
    #  ★★★ LiDAR: วัดมุมระหว่างหุ่นยนต์กับกำแพง (ทำให้วิ่งตรงขนานกำแพง) ★★★
    # ==========================================================================
    def wall_score(self, x, y, th):              # ★ ให้คะแนนว่าที่มุม th จุด LiDAR เรียงเป็นแถว (กำแพง) ดีแค่ไหน
        c, s = math.cos(th), math.sin(th)        # ★ cos, sin ของมุมทดลอง
        u = x * c + y * s                        # ★ หมุนจุด: พิกัดตามแกนที่เอียง th (แถวของกำแพงหน้า/หลัง)
        v = -x * s + y * c                       # ★ หมุนจุด: พิกัดตามแกนตั้งฉาก (แถวของกำแพงซ้าย/ขวา)
        hu = np.bincount(np.floor((u - u.min()) / self.wall_bin).astype(int))   # ★ นับจุดในแต่ละแถว (กว้าง 1 cm) ตามแกน u
        hv = np.bincount(np.floor((v - v.min()) / self.wall_bin).astype(int))   # ★ นับจุดในแต่ละแถว ตามแกน v
        score = float(np.sum(hu.astype(float) ** 2) + np.sum(hv.astype(float) ** 2))   # ★ ผลรวมกำลังสอง: จุดรวมกันในแถวเดียวยิ่งได้คะแนนมาก
        return score, int(max(hu.max(), hv.max()))   # ★ คืน (คะแนน, จำนวนจุดในแถวที่แน่นที่สุด)

    def lidar_heading_error(self):               # ★ หาว่าหุ่นยนต์เอียงจากกำแพงกี่องศา (ใช้จุด LiDAR ทุกด้านพร้อมกัน)
        sel = (self.scan_ok & (self.scan_r >= self.ignore_below)            # ★ เลือกจุดที่ใช้ได้ ไม่ใกล้เกิน 8 cm
               & (self.scan_r <= self.wall_range_cells * self.cell))       # ★ ... และไม่ไกลเกิน 1.5 ช่อง
        a = np.radians(self.scan_a[sel])         # ★ มุมของจุด (เรเดียน)
        x = self.scan_r[sel] * np.cos(a)         # ★ จุด -> พิกัด x (ไปข้างหน้า) [m]
        y = self.scan_r[sel] * np.sin(a)         # ★ จุด -> พิกัด y (ไปทางซ้าย) [m]
        if len(x) < self.wall_min_pts:           # ★ จุดน้อยเกินไป
            self.wall_peak = 0                   # ★ ... ไม่มีกำแพงให้วัด
            return None                          # ★ ... วัดไม่ได้
        best, best_score = 0.0, -1.0             # ★ มุมที่ดีที่สุดที่เจอ และคะแนน
        for th in np.arange(-self.wall_search, self.wall_search + 1e-9, math.radians(1.0)):   # ★ รอบหยาบ: ลองทุก 1 องศา
            sc = self.wall_score(x, y, th)[0]    # ★ ... คะแนนของมุมนี้
            if sc > best_score:                  # ★ ... ดีกว่าที่เคยเจอ
                best, best_score = th, sc        # ★ ...... จำไว้
        coarse = best                            # ★ มุมที่ดีที่สุดจากรอบหยาบ
        for th in np.arange(coarse - math.radians(1.0), coarse + math.radians(1.0) + 1e-9, math.radians(0.1)):   # ★ รอบละเอียด: ทุก 0.1 องศา รอบ ๆ มุมนั้น
            sc = self.wall_score(x, y, th)[0]    # ★ ... คะแนน
            if sc > best_score:                  # ★ ... ดีกว่า
                best, best_score = th, sc        # ★ ...... จำไว้
        self.wall_peak = self.wall_score(x, y, best)[1]   # ★ จำนวนจุดบนกำแพงที่เรียงแถวดีที่สุด
        if self.wall_peak < self.wall_min_pts:   # ★ ไม่มีกำแพงไหนเรียงแถวชัดพอ
            return None                          # ★ ... ไม่เชื่อผลนี้
        return float(best)                       # ★ มุมที่กำแพงเรียงตรงที่สุด = หุ่นยนต์เอียงเท่านี้ (+ หมุนซ้าย, - หมุนขวา)

    def grid_heading(self):                      # ทิศของตารางเขาวงกต (ใน odom) ที่ใกล้ทิศหุ่นยนต์ที่สุด
        q = math.pi / 2                          # 90 องศา
        return norm_angle(round(norm_angle(self.yaw - self.grid_off) / q) * q + self.grid_off)   # ปัดเป็นทวีคูณ 90 องศา (หัก/บวก grid_off)

    def update_grid(self, gain=None):            # ★ ใช้มุมกำแพงแก้ทิศตาราง (grid_off) ให้ตรงกับเขาวงกตจริง
        if not self.use_lidar_angle or self.wall_ang is None:   # ★ ไม่เห็นกำแพงตรง
            return                               # ★ ... ไม่แก้
        k = self.grid_corr_gain if gain is None else gain   # ★ แก้ทีละ 5% (ขณะเดิน) หรือ 100% (ตอนจัดตัวเสร็จ)
        q = math.pi / 2                          # ★ 90 องศา
        g = self.yaw + self.wall_ang             # ★ ทิศของกำแพงใน odom = ทิศตารางจริง
        off = norm_angle(g - round(g / q) * q)   # ★ เบี่ยงจากทวีคูณ 90 องศาเท่าไร
        self.grid_off = norm_angle(self.grid_off + k * norm_angle(off - self.grid_off))   # ★ ขยับ grid_off เข้าหาค่าจริง

    def align_w(self):                           # ★ คำนวณความเร็วหมุนเพื่อจัดตัวให้ขนานกำแพง
        if self.wall_ang is None:                # ★ ไม่เห็นกำแพงตรงเลย
            return None                          # ★ ... จัดตัวไม่ได้
        if abs(self.wall_ang) < self.align_tol:  # ★ เอียงไม่เกิน 1.5 องศาแล้ว
            return 0.0                           # ★ ... ตรงแล้ว
        w = clamp(self.align_kp * self.wall_ang, -0.6 * self.w_turn, 0.6 * self.w_turn)   # ★ หมุนแก้ตามมุมที่เอียง
        if abs(w) < self.align_min_w:            # ★ ช้าเกินไป มอเตอร์อาจไม่หมุน
            w = math.copysign(self.align_min_w, w)   # ★ ... ใช้ความเร็วขั้นต่ำ
        return w                                 # ★ คืนความเร็วหมุน [rad/s]

    def safe_cmd(self, v, w, m):                 # ชั้นความปลอดภัย: หมุนหนีเมื่อกำลังวิ่งเฉียงเข้าหากำแพง
        if not self.use_safety:                  # ปิดระบบนี้อยู่
            return v, w                          # ... ส่งคำสั่งเดิม
        a = np.radians(self.scan_a)              # มุมทุกลำแสง (เรเดียน)
        x = self.scan_r * np.cos(a)              # จุด -> x (ไปข้างหน้า) [m]
        y = self.scan_r * np.sin(a)              # จุด -> y (ไปทางซ้าย) [m]
        box_w = self.half_w + self.safe_margin   # ครึ่งความกว้างของ "ทางที่ตัวจะวิ่งผ่าน" (ตัว + 2 cm)
        box_x = self.front_stop - 0.01           # ความยาวกล่องด้านหน้า (สั้นกว่าระยะหยุดหน้ากำแพง)
        hit = (self.scan_ok & (self.scan_r >= self.ignore_below) & (x > 0.0)   # จุดที่ใช้ได้ อยู่ด้านหน้า
               & (x <= box_x) & (np.abs(y) <= box_w))                          # ... และอยู่ในกล่องหน้าตัว
        if not hit.any():                        # ไม่มีกำแพงล้ำเข้ามาในทางวิ่ง
            self.avoiding = False                # ... ไม่ได้หลบ
            return v, w                          # ... ส่งคำสั่งเดิม
        depth = float(box_w - np.min(np.abs(y[hit])))   # กำแพงล้ำเข้ามาในกล่องลึกกี่เมตร
        if not self.avoiding:                    # เพิ่งเริ่มหลบ
            self.get_logger().info(f'Wall in the path ({depth * 100:.1f} cm) -> steering away')   # ... บอกครั้งเดียว
        self.avoiding = True                     # กำลังหลบ
        away = -1.0 if float(np.mean(y[hit])) > 0 else 1.0   # กำแพงอยู่ทางซ้าย -> หมุนขวา, ทางขวา -> หมุนซ้าย
        w2 = clamp(away * (0.2 + self.k_avoid * depth), -self.w_turn, self.w_turn)   # แรงหมุนหนี (ยิ่งล้ำลึกยิ่งแรง)
        v2 = 0.0 if float(np.min(x[hit])) < self.robot_front + 0.01 else min(v, self.safe_speed)   # ชิดหน้าตัวมาก -> หยุดเดินหน้า
        return v2, w2                            # คำสั่งใหม่

    # ==========================================================================
    #  การเคลื่อนที่
    # ==========================================================================
    def publish(self, v, w):                 # ส่งคำสั่งความเร็ว (เหมือน test_cmd_vel.py)
        msg = Twist()                        # ข้อความเปล่า
        msg.linear.x = float(v)              # ความเร็วเดินหน้า [m/s]
        msg.angular.z = float(w)             # ความเร็วหมุน [rad/s]
        self.pub.publish(msg)                # ส่งไป /cmd_vel
        self.last_cmd = (v, w)               # จำไว้

    def stop_robot(self):                    # หยุดหุ่นยนต์อย่างปลอดภัย
        for _ in range(5):                   # ส่ง 5 ครั้ง กันข้อความหาย
            self.publish(0.0, 0.0)           # ความเร็ว 0
            time.sleep(0.05)                 # รอ 0.05 วินาที

    def traveled(self):                      # ระยะที่เดินไปแล้วนับจาก start_segment()
        return math.hypot(self.x - self.seg_x, self.y - self.seg_y)   # ระยะเส้นตรง (พีทาโกรัส)

    def start_segment(self):                 # จำตำแหน่งปัจจุบันเป็นจุดเริ่มวัดระยะ
        self.seg_x, self.seg_y = self.x, self.y   # เก็บ x, y

    def heading_hold(self):                  # เดินตรง (ใช้ตอนไม่มีกำแพงข้างตัว เช่น กลางทางแยก)
        if self.use_lidar_angle and self.wall_ang is not None:   # ★ LiDAR ยังเห็นกำแพงด้านอื่น (หน้า/หลัง/ไกลออกไป)
            self.heading_ref = self.grid_heading()               # ★ ... ให้ทิศสำรองตรงกับตาราง
            return clamp(self.kh * self.wall_ang, -self.max_corr, self.max_corr)   # ★ ... หมุนแก้ตามมุมกำแพงจริง
        if self.heading_ref is None:         # ไม่เห็นกำแพงเลย และยังไม่มีทิศเป้าหมาย
            self.heading_ref = self.grid_heading()   # ... ใช้ทิศตารางที่ใกล้ที่สุด
        return clamp(self.kh * norm_angle(self.heading_ref - self.yaw), -self.max_corr, self.max_corr)   # ใช้ทิศจาก odom แทน

    def wall_follow_w(self, m):              # ตัวควบคุมเดินตามทางเดิน: w = ระยะ + ★ มุม
        lw = m['left_wall'] < self.wall_near      # ★ มีกำแพงซ้ายของทางเดิน (ใกล้กว่า 0.75 ช่อง)
        rw = m['right_wall'] < self.wall_near     # ★ มีกำแพงขวาของทางเดิน (ขอบทางแยกที่ไกลกว่านี้ไม่นับ)
        if self.heading_ref is None:              # ยังไม่มีทิศเป้าหมาย
            self.heading_ref = self.grid_heading()   # ... ใช้ทิศตาราง
        heading_err = norm_angle(self.heading_ref - self.yaw)   # มุมคลาดเคลื่อนจาก odom (ใช้สำรอง)
        if self.use_lidar_angle and self.wall_ang is not None:  # ★ LiDAR เห็นกำแพงตรง
            heading_err = self.wall_ang                         # ★ ใช้มุมที่วัดจากกำแพงจริงแทน (ไม่เพี้ยนตาม odom)
            self.heading_ref = self.grid_heading()              # ★ ให้ทิศสำรองตรงกับตารางที่แก้แล้ว
        if lw and rw:                             # กำแพงสองข้าง
            push = self.kp * (m['left_wall'] - m['right_wall'])     # แรงดึงเข้ากลางทาง (ใกล้ขวา -> บวก -> ไปซ้าย)
        elif rw:                                  # กำแพงขวาข้างเดียว
            push = -self.kp * (m['right_wall'] - self.side_target)  # แรงดึงให้ห่างกำแพงขวาครึ่งช่อง
        elif lw:                                  # กำแพงซ้ายข้างเดียว
            push = self.kp * (m['left_wall'] - self.side_target)    # แรงดึงให้ห่างกำแพงซ้ายครึ่งช่อง
        else:                                     # ไม่มีกำแพงข้างตัว
            return self.heading_hold()            # ... เดินตรงตามทิศ odom
        push = clamp(push, -self.max_push, self.max_push)   # ★ จำกัดแรงดึงไม่เกิน 0.10 rad/s (เอียงเข้ากลางทางได้ไม่เกิน ~4 องศา)
        w = push + self.kh * heading_err          # ★ รวม: ดึงเข้ากลางทาง + คุมให้ขนานกำแพง
        return clamp(w, -self.max_corr, self.max_corr)   # จำกัดไม่เกิน +/- 0.4 rad/s

    def set_state(self, s):                  # เปลี่ยนสถานะ
        self.state = s                       # สถานะใหม่
        self.state_ticks = 0                 # เริ่มนับรอบใหม่
        if s == DECIDE:                      # เข้า DECIDE
            self.open_votes = {'R': 0, 'F': 0, 'L': 0}   # ... ล้างคะแนนโหวต
            self.vote_count = 0                          # ... ล้างจำนวนรอบโหวต
            self.decide_aligned = False                  # ★ ... ต้องจัดตัวตรงก่อน

    def beep(self):                          # บี๊บ (ถ้าเปิดใช้)
        if self.beep_pub is not None:        # มี publisher เสียง
            msg = UInt16()                   # ข้อความ
            msg.data = self.beep_ms          # ความยาวเสียง
            self.beep_pub.publish(msg)       # ส่ง

    def start_turn(self, direction):         # เตรียมหมุน: 'L' = +90, 'R' = -90, 'B' = 180 องศา
        self.turn_goal = {'L': math.pi / 2, 'R': -math.pi / 2, 'B': -math.pi}[direction]   # มุมรวมที่ต้องหมุน
        self.turn_step = math.pi / 2 if direction == 'L' else -math.pi / 2   # ★ หมุนครั้งละ 90 องศา
        self.turn_steps = 2 if direction == 'B' else 1   # ★ กลับหลัง = หมุนขวา 90 สองครั้ง (จัดตัวด้วย LiDAR ระหว่างกลาง)
        self.turn_target = norm_angle(self.grid_heading() + self.turn_step)   # ทิศเป้าหมายของครั้งนี้ = ทิศตาราง + 90
        self.set_state(TURN)                 # เข้าสถานะ TURN

    def execute(self, direction, auto, options):   # ทำตามการตัดสินใจ และจดบันทึกเส้นทาง
        self.junction_no += 1                      # นับการตัดสินใจ
        t = time.time() - self.start_time          # เวลาตั้งแต่เริ่ม
        self.route_log.append(                     # จดบันทึก 1 บรรทัด:
            f'#{self.junction_no:02d}  t={t:6.1f}s  pos=({self.x:+.2f},{self.y:+.2f})  '   # ลำดับ เวลา ตำแหน่ง
            f'open={"".join(options) or "-"}  choice={DIR_NAME[direction]}  '              # ทางที่เปิด ทางที่เลือก
            f'{"(auto)" if auto else "(user)"}')                                            # ใครเลือก
        self.get_logger().info(f'-> {DIR_NAME[direction]} {"(auto)" if auto else "(your choice)"}')   # พิมพ์บอก
        if direction == 'F':                       # ตรงไป
            self.heading_ref = self.grid_heading() # ... ทิศที่จะเดิน
            self.start_segment()                   # ... เริ่มวัดระยะ
            self.set_state(EXIT)                   # ... เดินออกจากทางแยก
        else:                                      # ซ้าย ขวา กลับหลัง
            self.start_turn(direction)             # ... หมุนก่อน

    def measure_cell(self, m):               # วัดความกว้างทางเดินตอนเริ่ม (เมื่อ cell_size = 0)
        if m['left_wall'] < 0.6 and m['right_wall'] < 0.6:   # เห็นกำแพงทั้งสองข้าง
            self.cell_samples.append(m['left_wall'] + m['right_wall'])   # ... ความกว้าง = ซ้าย + ขวา
        if len(self.cell_samples) >= 10:     # ได้ 10 ค่าแล้ว
            width = float(np.median(self.cell_samples))   # ... ใช้ค่ากลาง
            self.get_logger().info(f'Measured corridor width {width:.3f} m')   # ... บอกค่าที่วัดได้
            self.apply_cell(clamp(width, 0.25, 0.60))     # ... ตั้งขนาดช่อง
        elif self.state_ticks > 2 * self.rate:   # 2 วินาทีแล้วยังไม่เห็นกำแพงสองข้าง
            self.get_logger().info('Cannot see both walls -> using cell_size 0.30 m')   # ... เตือน
            self.apply_cell(0.30)            # ... ใช้ 0.30 m

    # ==========================================================================
    #  LOOP หลัก ทำงาน 20 ครั้งต่อวินาที
    # ==========================================================================
    def loop(self):                               # "สมอง" ของหุ่นยนต์
        now = time.time()                         # เวลาปัจจุบัน
        dt = now - self.last_loop_time            # เวลาที่ผ่านไปจากรอบก่อน (~0.05 s)
        self.last_loop_time = now                 # จำไว้

        if now - self.last_odom_time > 0.5:       # /odom หายไปเกิน 0.5 วินาที
            v, w = self.last_cmd                  # ... ประมาณตำแหน่งจากคำสั่งที่ส่งไป
            self.yaw = norm_angle(self.yaw + w * dt)      # ... yaw ใหม่
            self.x += v * math.cos(self.yaw) * dt         # ... x ใหม่
            self.y += v * math.sin(self.yaw) * dt         # ... y ใหม่

        if self.scan_r is None:                   # ยังไม่มีข้อมูล LiDAR
            return                                # ... รอ
        m = self.measure()                        # วัดระยะทุกทิศ
        self.state_ticks += 1                     # นับรอบในสถานะนี้
        if self.cell is None:                     # ยังไม่รู้ขนาดช่อง
            self.publish(0.0, 0.0)                # ... หยุดนิ่ง
            self.measure_cell(m)                  # ... วัดความกว้างทางเดิน
            return                                # ... รอบหน้าค่อยทำงานต่อ
        self.wall_ang = self.lidar_heading_error() if self.use_lidar_angle else None   # ★ วัดว่าหุ่นยนต์เอียงจากกำแพงกี่องศา
        if self.state in (FOLLOW, EXIT):          # ★ กำลังเดินในทางเดิน
            self.update_grid()                    # ★ ... ใช้มุมกำแพงแก้ทิศตารางทีละนิด
        self.print_status(m, now)                 # พิมพ์สถานะวินาทีละครั้ง

        # ---------------- FOLLOW: เดินตามทางเดิน ----------------
        if self.state == FOLLOW:                  # สถานะ FOLLOW
            if m['front'] < self.front_stop:      # กำแพงข้างหน้าใกล้ (ถึงกลางช่องสุดท้ายแล้ว)
                self.publish(0.0, 0.0)            # ... หยุด
                self.set_state(DECIDE)            # ... ไปตัดสินใจ
                return                            # ... จบรอบนี้
            self.left_cnt = self.left_cnt + 1 if m['left'] > self.open_dist else 0     # นับว่าซ้ายเปิดติดกันกี่รอบ
            self.right_cnt = self.right_cnt + 1 if m['right'] > self.open_dist else 0  # นับว่าขวาเปิดติดกันกี่รอบ
            if self.left_cnt >= self.confirm_n or self.right_cnt >= self.confirm_n:    # เปิดครบ 3 รอบ = เจอทางแยก
                self.get_logger().info('Side opening detected -> moving to junction centre')  # ... บอก
                self.heading_ref = self.grid_heading()   # ... ทิศที่จะเดินต่อ
                self.start_segment()              # ... เริ่มวัดระยะ
                self.set_state(ADVANCE)           # ... เดินเข้ากลางทางแยก
                return                            # ... จบรอบนี้
            self.publish(*self.safe_cmd(self.v, self.wall_follow_w(m), m))   # เดินหน้า + ★ คุมให้ขนานกำแพง + หลบถ้าใกล้

        # ---------------- ADVANCE: เดินเข้ากลางทางแยก ----------------
        elif self.state == ADVANCE:               # สถานะ ADVANCE
            if m['front'] < self.front_stop or self.traveled() >= self.center_offset:   # ถึงกลางทางแยก หรือกำแพงหน้าใกล้
                self.publish(0.0, 0.0)            # ... หยุด
                self.set_state(DECIDE)            # ... ไปตัดสินใจ
                return                            # ... จบรอบนี้
            self.publish(*self.safe_cmd(self.v, self.heading_hold(), m))   # เดินตรงตามทิศตาราง

        # ---------------- DECIDE: จัดตัวตรง -> รอนิ่ง -> โหวต -> ตัดสินใจ ----------------
        elif self.state == DECIDE:                # สถานะ DECIDE
            if not self.decide_aligned:           # ★ ขั้น 1: ยังไม่ได้จัดตัวตรง
                w = self.align_w()                # ★ ... LiDAR บอกว่าต้องหมุนเท่าไร
                if w and self.state_ticks <= self.align_ticks_max:   # ★ ... ยังเอียง และยังไม่หมดเวลา
                    self.publish(0.0, w)          # ★ ...... หมุนอยู่กับที่จัดตัว
                    return                        # ★ ...... รอบหน้าวัดใหม่
                self.publish(0.0, 0.0)            # ★ ... ตรงแล้ว (หรือหมดเวลา) -> หยุด
                if w == 0.0:                      # ★ ... ถ้าตรงจริง
                    self.update_grid(gain=1.0)    # ★ ...... ตั้งทิศตารางใหม่ให้ตรงกับกำแพงเต็ม ๆ
                self.decide_aligned = True        # ★ ... จำว่าจัดตัวแล้ว
                self.settle_start = self.state_ticks   # ... เริ่มนับเวลารอนิ่ง
                return                            # ... จบรอบนี้
            self.publish(0.0, 0.0)                # หยุดนิ่ง
            if self.state_ticks - self.settle_start <= self.settle_ticks:   # ขั้น 2: รอนิ่ง 0.3 วินาที
                return                            # ... ยังไม่ครบ
            self.vote_count += 1                  # ขั้น 3: โหวตรอบนี้
            self.open_votes['R'] += m['right'] > self.open_dist   # ขวาเปิด +1
            self.open_votes['F'] += m['front'] > self.open_dist   # หน้าเปิด +1
            self.open_votes['L'] += m['left'] > self.open_dist    # ซ้ายเปิด +1
            if self.vote_count < self.decide_samples:             # ยังโหวตไม่ครบ 10 รอบ
                return                                            # ... รอบหน้า
            options = [d for d in ('R', 'F', 'L')                 # ทิศที่เปิด (เรียงตามกฎมือขวา)
                       if self.open_votes[d] * 2 > self.vote_count]   # ... ได้คะแนนเกินครึ่ง
            ways = len(options) + 1               # จำนวนทาง = ทางเปิด + ทางที่มา
            self.get_logger().info(               # บอกผล
                f'{JUNCTION_NAME[ways]}  open={"".join(options) or "-"}  '        # ชนิดทางแยก ทางเปิด
                f'votes R{self.open_votes["R"]} F{self.open_votes["F"]} '          # คะแนน
                f'L{self.open_votes["L"]} of {self.vote_count}')                   # ... ต่อ
            if ways >= self.ask_min_ways and options:     # สามแยก / สี่แยก
                self.ask_options = options        # ... จำทางเปิด
                self.ask_ways = ways              # ... จำชนิด
                while not self.input_q.empty():   # ... ล้างปุ่มที่กดค้าง
                    self.input_q.get_nowait()     # ...... ดึงทิ้ง
                self.beep()                       # ... บี๊บ
                self.prompt()                     # ... ถาม
                self.set_state(ASK)               # ... รอคำตอบ
            elif not options:                     # ทางตัน
                self.execute('B', True, options)  # ... กลับหลังเอง
            else:                                 # ทางโค้ง
                self.execute(options[0], True, options)   # ... ไปทางเดียวที่เปิด

        # ---------------- ASK: รอคำตอบ ----------------
        elif self.state == ASK:                   # สถานะ ASK
            self.publish(0.0, 0.0)                # หยุดนิ่ง
            try:                                  # ลองดึงคำตอบ
                ans = self.input_q.get_nowait()   # ... ไม่รอ
            except queue.Empty:                   # ยังไม่มีคำตอบ
                return                            # ... รอบหน้าเช็กใหม่
            if ans == '':                         # กด Enter เฉย ๆ
                choice = self.ask_options[0]      # ... ตามกฎมือขวา
            else:                                 # พิมพ์มา
                choice = INPUT_MAP.get(ans)       # ... แปลงเป็นรหัสทิศ
            if choice is None:                    # พิมพ์ไม่ถูก
                print(f'  "{ans}" not understood.')   # ... บอก
                self.prompt()                     # ... ถามใหม่
                return                            # ... จบรอบ
            if choice not in self.ask_options and choice != 'B':   # เลือกทางที่ LiDAR เห็นว่าตัน
                print(f'  {DIR_NAME[choice]} looks blocked by the LiDAR - going anyway.')   # ... เตือน
            self.execute(choice, False, self.ask_options)   # ทำตามที่เลือก

        # ---------------- TURN: หมุนตาม odom ----------------
        elif self.state == TURN:                  # สถานะ TURN
            remaining = norm_angle(self.turn_target - self.yaw)   # เหลือต้องหมุนอีกเท่าไร (ตาม odom)
            if abs(remaining) < self.turn_tol:    # เหลือน้อยกว่า 3 องศา
                self.publish(0.0, 0.0)            # ... หยุดหมุน
                self.align_next = EXIT            # ★ ... จัดตัวด้วย LiDAR เสร็จแล้วค่อยเดินออก
                self.set_state(ALIGN)             # ★ ... ไปสถานะ ALIGN (odom อาจคลาดเพราะล้อลื่น)
                return                            # ... จบรอบ
            w = clamp(2.0 * remaining, -self.w_turn, self.w_turn)   # หมุน = 2 x มุมที่เหลือ (ใกล้ถึงช้าลง)
            if abs(w) < 0.2:                      # ช้าเกิน
                w = math.copysign(0.2, w)         # ... อย่างน้อย 0.2 rad/s
            self.publish(0.0, w)                  # หมุนอยู่กับที่

        # ---------------- ★ ALIGN: LiDAR จัดตัวให้ขนานกำแพง ----------------
        elif self.state == ALIGN:                 # ★ สถานะ ALIGN
            w = self.align_w()                    # ★ LiDAR บอกว่าต้องหมุนแก้เท่าไร
            if w and self.state_ticks <= self.align_ticks_max:   # ★ ยังเอียงเกิน 1.5 องศา และยังไม่หมดเวลา
                self.publish(0.0, w)              # ★ ... หมุนแก้อยู่กับที่
                return                            # ★ ... รอบหน้าวัดใหม่
            self.publish(0.0, 0.0)                # ★ ตรงแล้ว (หรือหมดเวลา / ไม่เห็นกำแพง) -> หยุด
            if w == 0.0:                          # ★ ตรงกับกำแพงจริง
                self.update_grid(gain=1.0)        # ★ ... ตั้งทิศตารางให้ตรงกับทิศนี้เต็ม ๆ
            self.turn_steps -= 1                  # ★ หมุนเสร็จไป 1 ครั้ง
            if self.turn_steps > 0:               # ★ ยังต้องหมุนอีก (กำลังกลับหลัง)
                self.turn_target = norm_angle(self.grid_heading() + self.turn_step)   # ★ ... ทิศเป้าหมายครั้งถัดไป
                self.set_state(TURN)              # ★ ... หมุนอีก 90 องศา
                return                            # ★ ... จบรอบ
            self.heading_ref = self.grid_heading()   # ทิศที่จะเดินต่อ
            self.start_segment()                  # เริ่มวัดระยะ
            self.set_state(self.align_next)       # ไปสถานะถัดไป (FOLLOW ตอนเริ่ม, EXIT หลังเลี้ยว)

        # ---------------- EXIT: เดินออกจากทางแยก 1 ช่อง ----------------
        elif self.state == EXIT:                  # สถานะ EXIT
            if m['front'] < self.front_stop:      # กำแพงหน้าใกล้
                self.publish(0.0, 0.0)            # ... หยุด
                self.set_state(DECIDE)            # ... ตัดสินใจใหม่
                return                            # ... จบรอบ
            if self.traveled() >= self.exit_dist: # ออกมาครบ 1 ช่อง
                self.left_cnt = self.right_cnt = 0     # ... รีเซ็ตตัวนับ
                if m['left'] > self.open_dist or m['right'] > self.open_dist:   # ... ข้างยังเปิด = ทางแยกอีก
                    self.publish(0.0, 0.0)        # ...... หยุด
                    self.set_state(DECIDE)        # ...... ตัดสินใจ
                else:                             # ... ข้างเป็นกำแพง
                    self.set_state(FOLLOW)        # ...... เดินตามทางต่อ
                return                            # ... จบรอบ
            self.publish(*self.safe_cmd(self.v, self.wall_follow_w(m), m))   # เดินหน้า + ★ ขนานกำแพง + หลบถ้าใกล้

    # ==========================================================================
    #  ข้อความใน Terminal
    # ==========================================================================
    @staticmethod                                 # ไม่ใช้ข้อมูลของโหนด
    def junction_picture(opts):                   # วาดรูปทางแยก (ทิศที่ปิดเขียนว่า wall)
        top = '[w] FORWARD' if 'F' in opts else '  (wall)   '    # ด้านหน้า
        stem = '|' if 'F' in opts else '='                       # เส้นด้านบน
        left = '[a] LEFT ---' if 'L' in opts else '    (wall) |'  # ด้านซ้าย
        right = '--- [d] RIGHT' if 'R' in opts else '| (wall)'    # ด้านขวา
        return [                                                 # 5 บรรทัด
            '            ' + top,                                # หน้า
            '                 ' + stem,                          # เส้น
            '  ' + left + ' ROBOT ' + right,                     # ซ้าย-หุ่นยนต์-ขวา
            '                 |',                                # เส้น
            '             (came from)',                          # ทางที่มา
        ]                                                        # ปิด list

    def prompt(self):                             # พิมพ์คำถามที่ทางแยก
        print('\n' + '=' * 60)                    # เส้นคั่น
        print(f'  {JUNCTION_NAME.get(self.ask_ways, "JUNCTION")}   '   # ชนิดทางแยก
              f'#{self.junction_no + 1}   - robot stopped')            # ลำดับ
        print('=' * 60)                           # เส้นคั่น
        for line in self.junction_picture(self.ask_options):   # รูปทางแยก
            print(line)                           # ... ทีละบรรทัด
        order = [d for d in ('L', 'R', 'F') if d in self.ask_options] + ['B']   # ตัวเลือก
        print('  Choose: ' + ' / '.join(f'[{KEY_OF[d]}] {DIR_NAME[d]}' for d in order))   # พิมพ์ตัวเลือก
        print(f'  (just Enter = {DIR_NAME[self.ask_options[0]]}, right-hand rule)')    # Enter = กฎมือขวา
        print('> ', end='', flush=True)           # ที่ให้พิมพ์คำตอบ

    def print_status(self, m, now):               # พิมพ์สถานะวินาทีละครั้ง
        if now - self.last_status < 1.0 or self.state == ASK:   # ยังไม่ครบ 1 วินาที หรือกำลังถาม
            return                                # ... ไม่พิมพ์
        self.last_status = now                    # จำเวลา
        f = lambda d: ' inf' if math.isinf(d) else f'{d:4.2f}'  # ระยะ -> ข้อความ
        ang = '  -- ' if self.wall_ang is None else f'{math.degrees(self.wall_ang):+5.1f}'   # ★ มุมเอียงจากกำแพง (องศา)
        self.get_logger().info(                   # บรรทัดสถานะ เช่น [FOLLOW ] F=0.45 L=0.15 R=0.16 ang= +0.8 pts=31
            f'[{self.state:7s}] F={f(m["front"])} L={f(m["left"])} R={f(m["right"])} '   # ... สถานะ + ระยะ
            f'ang={ang} pts={self.wall_peak}')    # ... ★ มุมเอียง + จำนวนจุดบนกำแพงหลัก

    # ==========================================================================
    #  บันทึกแผนที่ (สไลด์ 8, P7-7) หลังกด Ctrl+C
    # ==========================================================================
    def save_map(self):                           # บันทึกแผนที่และบันทึกเส้นทาง
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')    # วันเวลา เช่น 20261010_171500
        map_dir = self.map_dir                           # โฟลเดอร์แผนที่
        if not os.path.isdir(os.path.dirname(map_dir.rstrip('/'))):   # ไม่มีโฟลเดอร์แม่
            map_dir = os.path.expanduser('~/maze_maps')               # ... ใช้ ~/maze_maps แทน
        os.makedirs(map_dir, exist_ok=True)              # สร้างโฟลเดอร์

        print('\nSaving map ... (do NOT press Ctrl+C again)', flush=True)   # อย่ากด Ctrl+C ซ้ำ

        def run(cmd):                                    # รันคำสั่ง Terminal 1 คำสั่ง
            print('  $ ' + ' '.join(cmd), flush=True)    # แสดงคำสั่ง
            try:                                         # ลองรัน
                r = subprocess.run(cmd, timeout=60, start_new_session=True)   # รอเสร็จ (สูงสุด 60 s), Ctrl+C ซ้ำไม่หยุด
                print('  ok' if r.returncode == 0 else f'  exit code {r.returncode}', flush=True)   # ผล
            except FileNotFoundError:                    # หา ros2 ไม่เจอ
                print('  command not found (is ROS 2 sourced in this terminal?)')   # ... source ROS 2 หรือยัง
            except subprocess.TimeoutExpired:            # เกิน 60 วินาที
                print('  timed out - is map_gmapping_launch.py still running?')     # ... Gmapping ยังรันอยู่ไหม

        if self.save_method in ('launch', 'both'):       # คำสั่งสไลด์ 8
            run(['ros2', 'launch', 'yahboomcar_nav', 'save_map_launch.py'])   # ... yahboom_map.pgm/.yaml
        if self.save_method in ('cli', 'both'):          # สำรอง
            run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli',          # ... ตัวบันทึกของ Nav2
                 '-f', os.path.join(map_dir, f'maze_{ts}')])                 # ... maze_<วันเวลา>

        log_path = os.path.join(map_dir, f'maze_{ts}_route.txt')   # ไฟล์บันทึกเส้นทาง
        with open(log_path, 'w', encoding='utf-8') as fh:          # เปิดเขียน
            fh.write(f'Maze run {ts}\n')                           # หัวไฟล์
            fh.write('\n'.join(self.route_log) + '\n')             # ทุกการตัดสินใจ
        print(f'Route log: {log_path}')                  # บอกที่อยู่ไฟล์
        print(f'Maps: {map_dir}', flush=True)            # บอกโฟลเดอร์


# ==============================================================================
#  จุดเริ่มโปรแกรม
# ==============================================================================
def main():                                       # ฟังก์ชันแรกที่ทำงาน
    try:                                          # เริ่ม ROS 2 แบบจัดการ Ctrl+C เอง
        from rclpy.signals import SignalHandlerOptions             # ตัวเลือกจัดการสัญญาณ (ROS 2 รุ่นใหม่)
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO) # Ctrl+C แล้วยังส่งคำสั่งหยุดและบันทึกแผนที่ได้
    except (ImportError, TypeError):              # ROS 2 รุ่นเก่า
        rclpy.init()                              # ... เริ่มแบบปกติ

    node = MazeExplorer()                         # สร้างโหนด
    try:                                          # ทำงานหลัก
        rclpy.spin(node)                          # วนทำงานจนกด Ctrl+C
    except KeyboardInterrupt:                     # กด Ctrl+C
        print('\nCtrl+C -> goal reached, stopping robot.')   # ... ถึงเส้นชัย
    except Exception as e:  # noqa: BLE001        # error อื่น
        print(f'\nStopped: {e}')                  # ... พิมพ์ error
    finally:                                      # ทำเสมอ
        try:                                      # ลองหยุดหุ่นยนต์
            node.stop_robot()                     # ... 1) หยุด
        except Exception:  # noqa: BLE001         # หยุดไม่ได้
            pass                                  # ... ข้าม
        try:                                      # ลองบันทึกแผนที่
            node.save_map()                       # ... 2) บันทึกแผนที่
        except Exception as e:  # noqa: BLE001    # บันทึกไม่สำเร็จ
            print(f'Map save failed: {e}')        # ... บอก error แต่ยังปิดโหนดต่อได้
        node.destroy_node()                       # 3) ปิดโหนด
        if rclpy.ok():                            # ROS 2 ยังเปิดอยู่
            rclpy.shutdown()                      # ... ปิด


if __name__ == '__main__':                        # รันไฟล์นี้ตรง ๆ
    main()                                        # ... เริ่มโปรแกรม
