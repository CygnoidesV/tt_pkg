#!/usr/bin/env python3
import rclpy
import cv2
from rclpy.node import Node
from tt_pkg.msg import DetectInfo
from tt_pkg.config import settings_PU
from tt_pkg.config import config
from tt_pkg.detection import detect_PU

from tt_pkg.config import config, settings_BL


class Detection2(Node):
    def __init__(self):
        # self.cap = cv2.VideoCapture(2)
        # self.cap.set(cv2.CAP_PROP_FPS, 30)
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        # self.cap.set(cv2.CAP_PROP_HUE, 0)  # 固定色调
        # self.cap.set(cv2.CAP_PROP_SATURATION, 80)  # 设置饱和度
        # self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)  # 禁用自动曝光
        # self.cap.set(cv2.CAP_PROP_EXPOSURE, settings_PU["exposure"])
        # self.cap.set(6, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))

        self.cap = cv2.VideoCapture(settings_BL["camera_id"])
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FPS, settings_BL["fps"])
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, settings_BL["frame_width"])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, settings_BL["frame_height"])
        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, settings_BL["brightness"])

        super().__init__("detection2_node")
        self.pub1_ = self.create_publisher(DetectInfo, "target_info", 10)
        self.timer_ = self.create_timer(0.04, self.timer_callback)
        self.get_logger().info("detection2_node is started successfully.")

    def timer_callback(self):

        _, ori_img = self.cap.read()
        if ori_img is None:
            self.get_logger().info("Frame is None. Please make sure the camera ID is 2.")
            return
        size = ori_img.shape
        area = size[0]*size[1]

        result = detect_PU(ori_img, area)

        if result is not None:
            current_time = rclpy.clock.Clock().now()  # 使用ROS 2的时钟来获取当前时间
            msg = DetectInfo()
            for i in range(len(result)):
                msg.header.stamp = current_time.to_msg()
                msg.color = result[i][0]
                msg.x_pixel, msg.y_pixel = result[i][1]
                self.pub1_.publish(msg)

        #         for point in result:
        #             cv2.circle(ori_img, point[1], 2, (0, 255, 0), 2)

        # operate_pixel2 = config.get("operate_pixel2")
        # cv2.circle(ori_img, operate_pixel2, 2, (255, 0, 0), 2)
        # cv2.imshow("result2", ori_img)
        # key = cv2.waitKey(1)
        # if key == 27:
        #     return


def main(args=None):
    rclpy.init(args=args)
    node = Detection2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
