#!/usr/bin/env python3
import rclpy
import cv2
from rclpy.node import Node
from tt_pkg.config import settings_PU

class Detection2(Node):
    def __init__(self):
        self.cap = cv2.VideoCapture(2)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_HUE, 0)  # 固定色调
        self.cap.set(cv2.CAP_PROP_SATURATION, 75)  # 设置饱和度
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)  # 禁用自动曝光
        self.cap.set(cv2.CAP_PROP_EXPOSURE, settings_PU["exposure"])
        self.cap.set(6, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))

        super().__init__("detection2_node")
        self.timer_ = self.create_timer(0.04, self.timer_callback)
        self.get_logger().info("detection2_node is started successfully.")

    def timer_callback(self):

        _, ori_img = self.cap.read()
        if ori_img is None:
            self.get_logger().info("Frame is None.")
            return

        cv2.imshow("result2", ori_img)
        key = cv2.waitKey(1)
        if key == 27:
            return

def main(args = None):
    rclpy.init(args = args)
    node = Detection2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()