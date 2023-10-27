#!/usr/bin/env python3

import rclpy
import curses
import math
from rclpy.node import Node
from tt_pkg.msg import MoveCmd, MoveGoal, ArmCmd, PositionInfo, DetectInfo
from tt_pkg.config import config

ARM_RST = 0x01
ARM_TO_CODE = 0x02
ARM_TO_STUFF = 0x03
ARM_GRAB_MATERIAL = 0x04
ARM_PLACE_GROUND = 0x05
ARM_GRAB_GROUND = 0x06
ARM_PLACE_STUFF = 0x07


def get_time_diff(stamp1, stamp2):
    # 获取时间戳并将其转换为秒
    timestamp1 = stamp1.sec + stamp1.nanosec / 1e9
    timestamp2 = stamp2.sec + stamp2.nanosec / 1e9

    # 计算时间差
    time_difference = abs(timestamp1 - timestamp2)
    return time_difference

class Keyboard(Node):

    def __init__(self, stdscr):
        self.position_info = PositionInfo()
        super().__init__('keyboard_node')
        self.sub1_ = self.create_subscription(
            PositionInfo, "position_info", self.sub1_callback, 10)
        self.sub2_ = self.create_subscription(
            DetectInfo, "target_info", self.sub2_callback, 10)
        self.pub1_ = self.create_publisher(MoveCmd, "move_cmd", 10)
        self.pub2_ = self.create_publisher(MoveGoal, "move_goal", 10)
        self.pub3_ = self.create_publisher(ArmCmd, "arm_cmd", 10)
        self.timer_ = self.create_timer(0.04, self.timer_callback)
        
        self.target_info = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        self.cnt = 0
        self.stdscr = stdscr
        self.move_goal_msg = MoveGoal()

    def reach_target(self, color):
        current_time = rclpy.clock.Clock().now()  # 使用ROS 2的时钟来获取当前时间
        start_time = current_time
        operate_pixel2 = config.get("operate_pixel2")
        pixel_error = config.get("pixel_error")
        target_info = self.target_info[color - 1]
        while(True):
            current_time = rclpy.clock.Clock().now()  # 使用ROS 2的时钟来获取当前时间
            if get_time_diff(current_time.to_msg(), start_time.to_msg()) > config.get("target_timeout"):
                break
            target_info = self.target_info[color - 1]
            if target_info[0] == 0:
                self.move_stop()
                continue
            elif abs(target_info[1] - operate_pixel2[0]) > pixel_error or abs(target_info[2] - operate_pixel2[1]) > pixel_error:
                w = math.atan2(operate_pixel2[0] - target_info[1], operate_pixel2[1] - target_info[2])
                w = w *180.0 / math.pi
                self.move_cam(w)
            else:
                self.move_stop()
                break

        msg = ArmCmd()
        msg.act_id = ARM_PLACE_GROUND
        for i in range(10):
            self.pub3_.publish(msg) 

    def move_cam(self, w):
        val = config.get("val_camera_to_ground")
        angle = self.position_info.angle_abs * math.pi / 180.0
        w = w * math.pi / 180.0
        msg = MoveCmd()
        msg.vx = self.position_info.x_abs + val * math.sin(w + angle)
        msg.vy = self.position_info.y_abs - val * math.cos(w + angle)
        msg.vw = self.position_info.angle_abs
        for i in range(10):
            self.pub1_.publish(msg)

    def move_stop(self):
        msg = MoveCmd()
        msg.vx = self.position_info.x_abs
        msg.vy = self.position_info.y_abs
        msg.vw = self.position_info.angle_abs
        for i in range(10):
            self.pub1_.publish(msg)

    def sub1_callback(self, msg):
        self.position_info = msg

    def sub2_callback(self, msg):
        self.target_info[0] = [msg.r_f, msg.r_x, msg.r_y]
        self.target_info[1] = [msg.g_f, msg.g_x, msg.g_y]
        self.target_info[2] = [msg.b_f, msg.b_x, msg.b_y]

    def timer_callback(self):
        # Reset the velocities

        # Check if a key is pressed
        key = self.stdscr.getch()
        if key != curses.ERR: 
            if key == ord('='):
                pose = config.get("end_pose")
                self.move_goal_msg.x_abs, self.move_goal_msg.y_abs, self.move_goal_msg.angle_abs = pose
                self.pub2_.publish(self.move_goal_msg)
            if key == ord('-'):
                pose = config.get("start_pose")
                self.move_goal_msg.x_abs, self.move_goal_msg.y_abs, self.move_goal_msg.angle_abs = pose
                self.pub2_.publish(self.move_goal_msg)
            if key == ord('1'):
                pose = config.get("qr_code_pose")
                self.move_goal_msg.x_abs, self.move_goal_msg.y_abs, self.move_goal_msg.angle_abs = pose
                self.pub2_.publish(self.move_goal_msg)
            if key == ord('2'):
                pose = config.get("material_pose")
                self.move_goal_msg.x_abs, self.move_goal_msg.y_abs, self.move_goal_msg.angle_abs = pose
                self.pub2_.publish(self.move_goal_msg)
            if key == ord('3'):
                pose = config.get("machining_pose")
                self.move_goal_msg.x_abs, self.move_goal_msg.y_abs, self.move_goal_msg.angle_abs = pose
                self.pub2_.publish(self.move_goal_msg)
            if key == ord('4'):
                pose = config.get("machining_red_pose")
                self.move_goal_msg.x_abs, self.move_goal_msg.y_abs, self.move_goal_msg.angle_abs = pose
                self.pub2_.publish(self.move_goal_msg)
            if key == ord('5'):
                pose = config.get("machining_green_pose")
                self.move_goal_msg.x_abs, self.move_goal_msg.y_abs, self.move_goal_msg.angle_abs = pose
                self.pub2_.publish(self.move_goal_msg)
            if key == ord('6'):
                pose = config.get("machining_blue_pose")
                self.move_goal_msg.x_abs, self.move_goal_msg.y_abs, self.move_goal_msg.angle_abs = pose
                self.pub2_.publish(self.move_goal_msg)
            if key == ord('7'):
                pose = config.get("staging_pose")
                self.move_goal_msg.x_abs, self.move_goal_msg.y_abs, self.move_goal_msg.angle_abs = pose
                self.pub2_.publish(self.move_goal_msg)
            if key == ord('8'):
                pose = config.get("staging_red_pose")
                self.move_goal_msg.x_abs, self.move_goal_msg.y_abs, self.move_goal_msg.angle_abs = pose
                self.pub2_.publish(self.move_goal_msg)
            if key == ord('9'):
                pose = config.get("staging_green_pose")
                self.move_goal_msg.x_abs, self.move_goal_msg.y_abs, self.move_goal_msg.angle_abs = pose
                self.pub2_.publish(self.move_goal_msg)
            if key == ord('0'):
                pose = config.get("staging_blue_pose")
                self.move_goal_msg.x_abs, self.move_goal_msg.y_abs, self.move_goal_msg.angle_abs = pose
                self.pub2_.publish(self.move_goal_msg)
            if key == ord(' '):
                msg = MoveCmd()
                msg.vx = self.position_info.x_abs
                msg.vy = self.position_info.y_abs
                msg.vw = self.position_info.angle_abs
                self.pub1_.publish(msg)
            if key == ord('w'):
                self.move_cam(0)
            if key == ord('a'):
                self.move_cam(90)
            if key == ord('s'):
                self.move_cam(180)
            if key == ord('d'):
                self.move_cam(-90)
            if key == ord('q'): 
                val = config.get("val_camera_to_ground")
                msg = MoveCmd()
                msg.vx = self.position_info.x_abs
                msg.vy = self.position_info.y_abs
                msg.vw = self.position_info.angle_abs + 90
                self.pub1_.publish(msg)
            if key == ord('e'):
                val = config.get("val_camera_to_ground")
                msg = MoveCmd()
                msg.vx = self.position_info.x_abs
                msg.vy = self.position_info.y_abs
                msg.vw = self.position_info.angle_abs - 90
            if key == ord('r'):
                msg = ArmCmd()
                msg.act_id = ARM_RST
                for i in range(10):
                    self.pub3_.publish(msg)
            if key == ord('t'):
                msg = ArmCmd()
                msg.act_id = ARM_TO_CODE
                for i in range(10):
                    self.pub3_.publish(msg)
            if key == ord('y'):
                msg = ArmCmd()
                msg.act_id = ARM_TO_STUFF
                for i in range(10):
                    self.pub3_.publish(msg)
            if key == ord('u'):
                msg = ArmCmd()
                msg.act_id = ARM_GRAB_MATERIAL
                for i in range(10):
                    self.pub3_.publish(msg)
            if key == ord('i'):
                msg = ArmCmd()
                msg.act_id = ARM_PLACE_GROUND
                for i in range(10):
                    self.pub3_.publish(msg)
            if key == ord('o'):
                msg = ArmCmd()
                msg.act_id = ARM_GRAB_GROUND
                for i in range(10):
                    self.pub3_.publish(msg)
            if key == ord('p'):
                msg = ArmCmd()
                msg.act_id = ARM_PLACE_STUFF
                for i in range(10):
                    self.pub3_.publish(msg)
            if key == ord('z'):
                self.reach_target(1)
            if key == ord('x'):
                self.reach_target(2)
            if key == ord('c'):
                self.reach_target(3)
            # Publish the MoveCmd message
        # self.pub1_.publish(self.move_cmd_msg)


def main(stdscr):
    rclpy.init()
    node = Keyboard(stdscr)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    curses.wrapper(main)
