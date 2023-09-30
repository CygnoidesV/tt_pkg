#!/usr/bin/env python3

import rclpy
import curses
from rclpy.node import Node
from tt_pkg.msg import MoveCmd, MoveGoal
from tt_pkg.config import config

class Keyboard(Node):

    def __init__(self, stdscr):
        super().__init__('keyboard_node')
        self.pub1_ = self.create_publisher(MoveCmd, "move_cmd", 10)
        self.pub2_ = self.create_publisher(MoveGoal, "move_goal", 10)
        self.timer_ = self.create_timer(0.04, self.timer_callback)
        self.cnt = 0
        self.stdscr = stdscr

        # Initialize MoveCmd message
        self.move_cmd_msg = MoveCmd()
        self.move_goal_msg = MoveGoal()

    def timer_callback(self):
        # Reset the velocities

        # Check if a key is pressed
        key = self.stdscr.getch()
        if key != curses.ERR:
            if key == ord('0'):
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
