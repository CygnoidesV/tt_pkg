#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tt_pkg.msg import PositionInfo, MoveCmd, MoveGoal, ArmCmd, StuffInfo
from tt_pkg.config import config

ARM1_GRAP1 = b'0x01'
ARM2_GRAP1 = b'0x02'
ARM2_PLACE1 = b'0x03'
ARM2_PLACE2 = b'0x04'

def get_time_diff(stamp1, stamp2):
    # 获取时间戳并将其转换为秒
    timestamp1 = stamp1.sec + stamp1.nanosec / 1e9
    timestamp2 = stamp2.sec + stamp2.nanosec / 1e9

    # 计算时间差
    time_difference = abs(timestamp1 - timestamp2)
    return time_difference

class Policy(Node):
    def __init__(self):
        self.task_pipeline = []
        self.task_sequence = []
        self.task_index = 0
        self.arm_cmd_flag = 0
        self.stuff_red = []
        self.stuff_green = []
        self.stuff_blue = []
        self.position_info = PositionInfo()

        super().__init__("policy_node")
        self.sub1_ = self.create_subscription(String, "task_sequence", self.sub1_callback, 10)
        self.sub2_ = self.create_subscription(StuffInfo, "stuff_info", self.sub2_callback, 10)
        self.sub3_ = self.create_subscription(PositionInfo, "position_info", self.sub3_callback, 10)
        self.pub1_ = self.create_publisher(MoveGoal, "move_goal", 10)
        self.pub2_ = self.create_publisher(MoveCmd, "move_cmd", 10)
        self.pub3_ = self.create_publisher(ArmCmd, "arm_cmd", 10)
        self.timer_ = self.create_timer(0.04, self.timer_callback)

    def sub1_callback(self, msg):
        if self.task_sequence:
            return
        if len(msg.data) == 7:
            self.task_sequence = [int(msg.data[0]), int(msg.data[1]), int(msg.data[2]), int(msg.data[4]), int(msg.data[5]), int(msg.data[6])]
            print(self.task_sequence)

    def sub2_callback(self, msg):
        if msg.color == 1:
            self.stuff_red.append(msg)
        if msg.color == 2:
            self.stuff_green.append(msg)
        if msg.color == 3:
            self.stuff_blue.append(msg)

    def sub3_callback(self, msg):
        if self.position_info.stuff_num != msg.stuff_num:
            self.arm_cmd_flag = 0
        if self.position_info.stuff_num < msg.stuff_num:
            self.task_index = self.task_index + 1
        if self.position_info.stuff_num > msg.stuff_num:
            self.task_index = self.task_index - 1
        self.position_info = msg

    def timer_callback(self):

        self.stuff_info_update()

        if len(self.task_pipeline) == 0:
            return

        if self.task_pipeline[0] == "ready":
            if len(self.position_info.stuff_num) >= 0 and len(self.position_info.stuff_num) <= 3:
                msg = MoveGoal()
                msg.x_abs, msg.y_abs, msg.angle_abs = config.get("qr_code_pose")
                self.pub1_.publish(msg)
                self.task_pipeline.pop(0)

        if self.task_pipeline[0] == "waiting_for_task_sequence":
            if len(self.task_sequence) == 6:
                msg = MoveGoal()
                msg.x_abs, msg.y_abs, msg.angle_abs = config.get("material_pose")
                self.pub1_.publish(msg)
                self.task_pipeline.pop(0)

        if self.task_pipeline[0] == "arm1_grapping1":
            if self.position_info.stuff_num == 3:
                msg = MoveGoal()
                msg.x_abs, msg.y_abs, msg.angle_abs = config.get("rough_machining_pose")
                self.pub1_.publish(msg)
                self.task_pipeline.pop(0)

            color = self.task_sequence[self.task_index]
            if self.arm_cmd_flag == 0 and ((color == 1 and len(self.stuff_red) >= 10) or (color == 2 and len(self.stuff_green) >= 10) or (color == 3 and len(self.stuff_blue) >= 10))
                msg = ArmCmd()
                msg.act_id = ARM1_GRAP1
                self.pub3_.publish(msg)
                self.arm_cmd_flag = 1

        
    
    def stuff_info_update(self):
        current_time = rclpy.clock.Clock().now()  # 使用ROS 2的时钟来获取当前时间

        if len(self.stuff_red) > 0:
            while get_time_diff(current_time.to_msg(), self.stuff_red[0].header.stamp) > config.get("max_time_diff") or len(self.stuff_red) > config.get("frame_buff"):
                print(get_time_diff(current_time.to_msg(), self.stuff_red[0].header.stamp))
                self.stuff_red.pop(0)
                if len(self.stuff_red) == 0:
                    break 

        if len(self.stuff_green) > 0:
            while get_time_diff(current_time.to_msg(), self.stuff_green[0].header.stamp) > config.get("max_time_diff") or len(self.stuff_green) > config.get("frame_buff"):
                self.stuff_green.pop(0)
                if len(self.stuff_green) == 0:
                    break 

        if len(self.stuff_blue) > 0:
            while get_time_diff(current_time.to_msg(), self.stuff_blue[0].header.stamp) > config.get("max_time_diff") or len(self.stuff_blue) > config.get("frame_buff"):
                self.stuff_blue.pop(0)
                if len(self.stuff_blue) == 0:
                    break 

def main(args = None):
    rclpy.init(args = args)
    node = Policy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
