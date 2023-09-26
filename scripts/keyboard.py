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
        self.move_cmd_msg.vx = 0.0
        self.move_cmd_msg.vy = 0.0
        self.move_cmd_msg.vw = 0.0

        # Check if a key is pressed
        key = self.stdscr.getch()
        if key != curses.ERR:
            if key == ord('w'):
                self.move_cmd_msg.vy = 1000.0  # Forward motion
                self.move_goal_msg.y_abs = self.move_goal_msg.y_abs + config.get("position_error")
            elif key == ord('s'):
                self.move_cmd_msg.vy = -1000.0  # Backward motion
                self.move_goal_msg.y_abs = self.move_goal_msg.y_abs - config.get("position_error")

            if key == ord('a'):
                self.move_cmd_msg.vx = -1000.0  # Turn left
                self.move_goal_msg.x_abs = self.move_goal_msg.x_abs - config.get("position_error")
            elif key == ord('d'):
                self.move_cmd_msg.vx = 1000.0  # Turn right
                self.move_goal_msg.x_abs = self.move_goal_msg.x_abs + config.get("position_error")

            if key == ord('q'):
                self.move_cmd_msg.vw = 90.0  # Turn left
                self.move_goal_msg.angle_abs = self.move_goal_msg.angle_abs + config.get("position_error")
            elif key == ord('e'):
                self.move_cmd_msg.vw = -90.0  # Turn right
                self.move_goal_msg.angle_abs = self.move_goal_msg.angle_abs - config.get("position_error")

            if key == ord('t'):
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
