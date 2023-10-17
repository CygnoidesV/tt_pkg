#!/usr/bin/env python3
import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tt_pkg.msg import DetectInfo
from tt_pkg.config import settings_BL
from tt_pkg.detection import detect_QR, detect_BL


def cul_k(x1, x2, x3, x4):  # 计算斜率差
    k1 = (x2[1] - x1[1]) / (float(x2[0] - x1[0]))
    k2 = (x4[1] - x3[1]) / (float(x4[0] - x3[0]))
    return (k1 - k2) ** 2


def cul_diff(points):
    p1 = points[0][0]
    p2 = points[0][1]
    p3 = points[0][2]
    p4 = points[0][3]

    return cul_k(p1, p2, p4, p3), cul_k(p1, p4, p2, p3)


class Detection1(Node):
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_HUE, 0)  # 固定色调
        self.cap.set(cv2.CAP_PROP_SATURATION, 75)  # 设置饱和度
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)  # 禁用自动曝光
        self.cap.set(cv2.CAP_PROP_EXPOSURE, settings_BL["exposure"])
        self.cap.set(6, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))

        self.task_sequence = String()
        self.task_sequence.data = ""
        self.bridge = CvBridge()

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
                    a, b = cul_diff(points)
                    if a <= 0.1 or b <= 0.1:  # 斜率差足够小（平行）
                        # 绘制识别框
                        cv2.drawContours(
                            ori_img, [np.int32(points)], 0, (0, 0, 255), 2)
                    # self.get_logger().info("Publishing: %s" % msg.data)
            except:
                pass

        result_r, result_g, result_b = detect_BL(ori_img)

        # if result_r[1]:
        #     cv2.circle(ori_img, result_r[1], 5, (0, 255, 0), 2)

        # if result_g[1]:
        #     cv2.circle(ori_img, result_g[1], 5, (0, 255, 0), 2)

        # if result_b[1]:
        #     cv2.circle(ori_img, result_b[1], 5, (0, 255, 0), 2)

        # ori_img = cv2.resize(ori_img, (640, 480))
        # img_msg = self.bridge.cv2_to_imgmsg(ori_img, 'bgr8')
        # self.pub3_.publish(img_msg)
        # cv2.imshow("result1", ori_img)
        # key = cv2.waitKey(1)
        # if key == 27:
        #     return

        if result_r[1]:
            msg = DetectInfo()
            msg.header.stamp = current_time.to_msg()
            msg.color = 1
            msg.x_pixel, msg.y_pixel = result_r[1]
            self.pub2_.publish(msg)
            # self.get_logger().info("Publishing: Red")

        if result_g[1]:
            msg = DetectInfo()
            msg.header.stamp = current_time.to_msg()
            msg.color = 2
            msg.x_pixel, msg.y_pixel = result_g[1]
            self.pub2_.publish(msg)
            # self.get_logger().info("Publishing: Green")

        if result_b[1]:
            msg = DetectInfo()
            msg.header.stamp = current_time.to_msg()
            msg.color = 3
            msg.x_pixel, msg.y_pixel = result_b[1]
            self.pub2_.publish(msg)
            # self.get_logger().info("Publishing: Blue")


def main(args=None):
    rclpy.init(args=args)
    node = Detection1()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
