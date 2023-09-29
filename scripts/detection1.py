#!/usr/bin/env python3
import rclpy
import cv2
from rclpy.node import Node
from std_msgs.msg import String
from tt_pkg.msg import DetectInfo
from tt_pkg.config import settings_BL
from tt_pkg.detection import detect_QR, detect_BL

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

        super().__init__("detection1_node")
        self.pub1_ = self.create_publisher(String, "task_sequence", 10)
        self.pub2_ = self.create_publisher(DetectInfo, "stuff_info", 10)
        self.timer_ = self.create_timer(0.04, self.timer_callback)
        self.get_logger().info("detection1_node is started successfully.")

    def timer_callback(self):

        _, ori_img = self.cap.read()
        if ori_img is None:
            self.get_logger().info("Frame is None.")
            return

        cv2.imshow("result1", ori_img)
        key = cv2.waitKey(1)
        if key == 27:
            return

        current_time = rclpy.clock.Clock().now()  # 使用ROS 2的时钟来获取当前时间

        codeinfo, points, straight_qrcode = detect_QR(ori_img)
        result_r, result_g, result_b = detect_BL(ori_img)

        if codeinfo:
            msg = String()
            msg.data = codeinfo
            self.pub1_.publish(msg)
            # self.get_logger().info("Publishing: %s" % msg.data)

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

def main(args = None):
    rclpy.init(args = args)
    node = Detection1()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()