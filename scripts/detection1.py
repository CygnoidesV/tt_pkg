#!/usr/bin/env python3
import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tt_pkg.msg import DetectInfo
from tt_pkg.config import config, settings_BL
from tt_pkg.detection import detect_QR, detect_BL

def exp(points):
    n = len(points)
    ave_x = ave_y = 0
    for point in points:
        ave_x += point[0] / n
        ave_y += point[1] / n
    var_x = var_y = 0
    for point in points:
        var_x += ((point[0] - ave_x) ** 2) / n 
        var_y += ((point[1] - ave_y) ** 2) / n 
    return [ave_x, ave_y], [var_x, var_y]

class Detection1(Node):
    def __init__(self):
        self.cap = cv2.VideoCapture(settings_BL["camera_id"])
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FPS, settings_BL["fps"])
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, settings_BL["frame_width"])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, settings_BL["frame_height"])
        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, settings_BL["brightness"])
        # self.cap.set(cv2.CAP_PROP_HUE, 0)  # 固定色调
        # self.cap.set(cv2.CAP_PROP_SATURATION, 75)  # 设置饱和度
        # self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)  # 禁用自动曝光
        # self.cap.set(cv2.CAP_PROP_EXPOSURE, settings_BL["exposure"])

        self.task_sequence = String()
        self.task_sequence.data = ""
        self.bridge = CvBridge()
        self.rst_r = []
        self.rst_g = []
        self.rst_b = []

        super().__init__("detection1_node")
        self.pub1_ = self.create_publisher(String, "task_sequence", 10)
        self.pub2_ = self.create_publisher(DetectInfo, "stuff_info", 10)
        self.pub3_ = self.create_publisher(Image, "detection1_image", 1)
        self.timer_ = self.create_timer(1.0/30, self.timer_callback)
        self.get_logger().info("detection1_node is started successfully.")

    def timer_callback(self):
        _, ori_img = self.cap.read()
        if ori_img is None:
            self.get_logger().info("Frame is None.")
            return

        current_time = rclpy.clock.Clock().now()  # 使用ROS 2的时钟来获取当前时间

        if self.task_sequence.data == "":
            try:
                codeinfo, points, straight_qrcode = detect_QR(ori_img)
                if codeinfo:
                    self.task_sequence.data = codeinfo
                    self.pub1_.publish(self.task_sequence)
                    # self.get_logger().info("Publishing: %s" % msg.data)
            except:
                pass

        result_r, result_g, result_b = detect_BL(ori_img)

        # if result_r[1]:
        #     cv2.circle(ori_img, result_r[1], 5, (0, 0, 255), 2)

        # if result_g[1]:
        #     cv2.circle(ori_img, result_g[1], 5, (0, 255, 0), 2)

        # if result_b[1]:
        #     cv2.circle(ori_img, result_b[1], 5, (255, 0, 0), 2)
        # print(result_r, result_g, result_b)

        # msg_img = self.bridge.cv2_to_imgmsg(ori_img)
        # self.pub3_.publish(msg_img)

        if result_r[1]:
            self.rst_r.append(result_r[1])
        elif len(self.rst_r) > 0:
            self.rst_r.pop(0)
        if len(self.rst_r) > config.get("frame_buff"):
            self.rst_r.pop(0)
        if result_g[1]:
            self.rst_g.append(result_g[1])
        elif len(self.rst_g) > 0:
            self.rst_g.pop(0)
        if len(self.rst_g) > config.get("frame_buff"):
            self.rst_g.pop(0)
        if result_b[1]:
            self.rst_b.append(result_b[1])
        elif len(self.rst_b) > 0:
            self.rst_b.pop(0)
        if len(self.rst_b) > config.get("frame_buff"):
            self.rst_b.pop(0)
        msg = DetectInfo()
        msg.header.stamp = current_time.to_msg()
        if len(self.rst_r) == config.get("frame_buff"):
            ave, var = exp(self.rst_r)
            if var[0] <= config.get("max_var") and var[1] <= config.get("max_var"):
                msg.r_f = 1
                msg.r_x = int(ave[0])
                msg.r_y = int(ave[1])
        if len(self.rst_g) == config.get("frame_buff"):
            ave, var = exp(self.rst_g)
            if var[0] <= config.get("max_var") and var[1] <= config.get("max_var"):
                msg.g_f = 2
                msg.g_x = int(ave[0])
                msg.g_y = int(ave[1])
        if len(self.rst_b) == config.get("frame_buff"):
            ave, var = exp(self.rst_b)
            if var[0] <= config.get("max_var") and var[1] <= config.get("max_var"):
                msg.b_f = 3
                msg.b_x = int(ave[0])
                msg.b_y = int(ave[1])
        self.pub2_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Detection1()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
