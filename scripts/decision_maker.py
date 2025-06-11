#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Range, NavSatFix, Imu
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from pid_controller import PIDController

class DecisionMaker(Node):
    def __init__(self):
        super().__init__('decision_maker')
        self.bridge = CvBridge()

        self.image = None
        self.lidar_data = None
        self.sonar_data = {}
        self.imu_data = None
        self.gps_data = None

        # Subscribers
        self.create_subscription(Image, '/front_camera_prius_clean', self.image_cb, 1)
        self.create_subscription(LaserScan, '/center_lidar_clean', self.lidar_cb, 1)
        sonar_topics = ['back_left_middle_sonar_clean', 'back_right_middle_sonar_clean',
                        'front_left_middle_sonar_clean', 'front_right_middle_sonar_clean']
        for topic in sonar_topics:
            self.create_subscription(Range, f'/{topic}', lambda msg, t=topic: self.sonar_cb(msg, t), 1)
        self.create_subscription(NavSatFix, '/gps_sensor_clean', self.gps_cb, 1)
        self.create_subscription(Imu, '/imu_clean', self.imu_cb, 1)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # PID controllers
        self.steer_pid = PIDController(kp=0.8, ki=0.0, kd=0.1, min_out=-1.0, max_out=1.0)
        self.speed_pid = PIDController(kp=0.5, ki=0.0, kd=0.05, min_out=0.0, max_out=0.5)

        self.current_offset = 0.0
        self.target_speed = 0.3  # m/s
        self.prev_time = self.get_clock().now()

        # Timer
        self.create_timer(0.1, self.control_loop)

    def image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)

        h, w = edges.shape
        mask = np.zeros_like(edges)
        polygon = np.array([[(0, h), (w, h), (w, int(h * 0.6)), (0, int(h * 0.6))]])
        cv2.fillPoly(mask, polygon, 255)
        cropped = cv2.bitwise_and(edges, mask)

        lines = cv2.HoughLinesP(cropped, 1, np.pi / 180, threshold=50, minLineLength=40, maxLineGap=100)
        if lines is None:
            self.current_offset = 0.0
            return

        left_x, right_x = [], []
        for x1, y1, x2, y2 in lines[:, 0]:
            slope = (y2 - y1) / (x2 - x1 + 1e-6)
            if abs(slope) < 0.5:
                continue
            if slope < 0:
                left_x.append((x1 + x2) / 2)
            else:
                right_x.append((x1 + x2) / 2)

        if left_x and right_x:
            lane_center = (np.mean(left_x) + np.mean(right_x)) / 2
            self.current_offset = (lane_center - w / 2) / (w / 2)
        else:
            self.current_offset = 0.0

    def lidar_cb(self, msg: LaserScan):
        self.lidar_data = msg

    def sonar_cb(self, msg: Range, topic: str):
        self.sonar_data[topic] = msg.range

    def gps_cb(self, msg: NavSatFix):
        self.gps_data = msg

    def imu_cb(self, msg: Imu):
        self.imu_data = msg

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        self.prev_time = now

        if self.image is None:
            return

        # Steering
        steer = self.steer_pid.calculate(0.0, self.current_offset, dt)

        # Speed (örnek: lidar'dan 0.5 metreden yakın engel varsa dur)
        obstacle_close = False
        if self.lidar_data:
            min_distance = min(self.lidar_data.ranges)
            obstacle_close = min_distance < 0.5

        speed = self.speed_pid.calculate(self.target_speed, 0.0, dt) if not obstacle_close else 0.0

        # Yayınla
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = steer
        self.cmd_pub.publish(twist)

    def run(self):
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    node = DecisionMaker()
    node.run()
    rclpy.shutdown()

