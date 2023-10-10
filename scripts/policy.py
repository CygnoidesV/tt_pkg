#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tt_pkg.msg import PositionInfo, MoveCmd, MoveGoal, ArmCmd, DetectInfo
from tt_pkg.config import config
from tt_pkg.PID import pid_c

ARM1_GRAP1 = 0x01
ARM2_GRAP1 = 0x02
ARM2_PLACE1 = 0x03
ARM2_PLACE2 = 0x04


def get_distance(point1, point2):
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)


def get_time_diff(stamp1, stamp2):
    # 获取时间戳并将其转换为秒
    timestamp1 = stamp1.sec + stamp1.nanosec / 1e9
    timestamp2 = stamp2.sec + stamp2.nanosec / 1e9

    # 计算时间差
    time_difference = abs(timestamp1 - timestamp2)
    return time_difference


def check_info(info):
    if len(info) != config.get("frame_buff"):
        return []

    current_time = rclpy.clock.Clock().now()  # 使用ROS 2的时钟来获取当前时间
    if get_time_diff(current_time.to_msg(), info[0].header.stamp) > config.get("max_time_diff"):
        return []

    ave_x = ave_y = 0
    for msg in info:
        ave_x = ave_x + msg.x_pixel / config.get("frame_buff")
        ave_y = ave_y + msg.y_pixel / config.get("frame_buff")

    var_x = var_y = 0
    for msg in info:
        var_x = var_x + (msg.x_pixel - ave_x)**2 / config.get("frame_buff")
        var_y = var_y + (msg.y_pixel - ave_y)**2 / config.get("frame_buff")

    if var_x > config.get("max_var") or var_y > config.get("max_var"):
        # print("Var: ", var_x, var_y)
        return []

    return [ave_x, ave_y]


class Policy(Node):
    def __init__(self):
        self.task_pipeline = [
            "ready",
            "wait_for_task_sequence",
            "arm1_grap1",
            "arm2_place1",
            # "arm2_grap1",
            # "arm2_place2",
            # "arm1_grap1",
            # "arm2_place1",
            # "arm2_grap1",
            # "arm2_place2"
        ]
        self.task_sequence = []
        self.task_index = -1
        self.arm_cmd_flag = 0
        self.stuff_red = []
        self.stuff_green = []
        self.stuff_blue = []
        self.position_info = PositionInfo()
        self.position_info.stuff_num = 100
        self.target_red = []
        self.target_green = []
        self.target_blue = []

        super().__init__("policy_node")
        self.sub1_ = self.create_subscription(
            String, "task_sequence", self.sub1_callback, 10)
        self.sub2_ = self.create_subscription(
            DetectInfo, "stuff_info", self.sub2_callback, 10)
        self.sub3_ = self.create_subscription(
            DetectInfo, "target_info", self.sub3_callback, 10)
        self.sub4_ = self.create_subscription(
            PositionInfo, "position_info", self.sub4_callback, 10)
        self.pub1_ = self.create_publisher(MoveGoal, "move_goal", 10)
        self.pub2_ = self.create_publisher(MoveCmd, "move_cmd", 10)
        self.pub3_ = self.create_publisher(ArmCmd, "arm_cmd", 10)
        self.timer_ = self.create_timer(0.04, self.timer_callback)

    def move_stop(self):
        msg = MoveCmd()
        msg.vx = 0.0
        msg.vy = 0.0
        msg.vw = 0.0
        for i in range(10):
            self.pub1_.publish(msg)

    def sub1_callback(self, msg):
        if self.task_sequence:
            return
        if len(msg.data) == 7:
            self.task_sequence = [int(msg.data[0]), int(msg.data[1]), int(
                msg.data[2]), int(msg.data[4]), int(msg.data[5]), int(msg.data[6])]
            print(self.task_sequence)

    def sub2_callback(self, msg):
        if msg.color == 1:
            self.stuff_red.append(msg)
            if len(self.stuff_red) > config.get("frame_buff"):
                self.stuff_red.pop(0)
        if msg.color == 2:
            self.stuff_green.append(msg)
            if len(self.stuff_green) > config.get("frame_buff"):
                self.stuff_green.pop(0)
        if msg.color == 3:
            self.stuff_blue.append(msg)
            if len(self.stuff_blue) > config.get("frame_buff"):
                self.stuff_blue.pop(0)

    def sub3_callback(self, msg):
        color = msg.color
        if color == 1:
            self.target_red.append(msg)
            if len(self.target_red) > config.get("frame_buff"):
                self.target_red.pop(0)
        if color == 2:
            self.target_green.append(msg)
            if len(self.target_green) > config.get("frame_buff"):
                self.target_green.pop(0)
        if color == 3:
            self.target_blue.append(msg)
            if len(self.target_blue) > config.get("frame_buff"):
                self.target_blue.pop(0)

    def sub4_callback(self, msg):
        if self.position_info.stuff_num != msg.stuff_num:
            self.arm_cmd_flag = 0
            self.task_index = self.task_index + 1
        self.position_info = msg

    def timer_callback(self):
        if len(self.task_pipeline) == 0:
            return

        # print(self.task_pipeline)

        if self.task_pipeline[0] == "ready":
            if self.position_info.stuff_num >= 0 and self.position_info.stuff_num <= 3:
                msg = MoveGoal()
                msg.x_abs, msg.y_abs, msg.angle_abs = config.get(
                    "qr_code_pose")
                self.pub1_.publish(msg)
                print(self.task_pipeline)
                self.task_pipeline.pop(0)
                return

        if self.task_pipeline[0] == "wait_for_task_sequence":
            if len(self.task_sequence) == 6:
                msg = MoveGoal()
                msg.x_abs, msg.y_abs, msg.angle_abs = config.get(
                    "material_pose")
                self.pub1_.publish(msg)
                print(self.task_pipeline)
                self.task_pipeline.pop(0)
                return

        if self.task_pipeline[0] == "arm1_grap1":
            if self.position_info.stuff_num == 3:
                msg = MoveGoal()
                msg.x_abs, msg.y_abs, msg.angle_abs = config.get(
                    "machining_pose")
                self.pub1_.publish(msg)
                print(self.task_pipeline)
                self.task_pipeline.pop(0)
                self.task_index = self.task_index - 3
                return

            material_pose = config.get("material_pose")
            position_err = config.get("position_error")
            operate_pixel1 = config.get("operate_pixel1")
            if get_distance([material_pose[0], material_pose[1]], [self.position_info.x_abs, self.position_info.y_abs]) > position_err * 2 or self.arm_cmd_flag == 1:
                return

            color = self.task_sequence[self.task_index]
            if color == 1:
                ave = check_info(self.stuff_red)
                if len(ave) == 0:
                    return
                if (ave[0] - operate_pixel1[0]) ** 2 < config.get("operate_err") ** 2:
                    msg = ArmCmd()
                    msg.act_id = ARM1_GRAP1
                    for i in range(10):
                        self.pub3_.publish(msg)
                    self.arm_cmd_flag = 1
            if color == 2:
                ave = check_info(self.stuff_green)
                if len(ave) == 0:
                    return
                if (ave[0] - operate_pixel1[0]) ** 2 < config.get("operate_err") ** 2:
                    msg = ArmCmd()
                    msg.act_id = ARM1_GRAP1
                    for i in range(10):
                        self.pub3_.publish(msg)
                    self.arm_cmd_flag = 1
            if color == 3:
                ave = check_info(self.stuff_blue)
                if len(ave) == 0:
                    return
                if (ave[0] - operate_pixel1[0]) ** 2 < config.get("operate_err") ** 2:
                    msg = ArmCmd()
                    msg.act_id = ARM1_GRAP1
                    for i in range(10):
                        self.pub3_.publish(msg)
                    self.arm_cmd_flag = 1

        if self.task_pipeline[0] == "arm2_place1":
            if self.position_info.stuff_num == 0:
                msg = MoveGoal()
                msg.x_abs, msg.y_abs, msg.angle_abs = config.get(
                    "machining_pose")
                self.pub1_.publish(msg)
                print(self.task_pipeline)
                self.task_pipeline.pop(0)
                self.task_index = self.task_index - 3
                return

            color = self.task_sequence[self.task_index]
            target_position = []
            target_ave = []

            if color == 1:
                target_position = config.get("machining_red_pose")
                target_ave = check_info(self.target_red)
            if color == 2:
                target_position = config.get("machining_green_pose")
                target_ave = check_info(self.target_green)
            if color == 3:
                target_position = config.get("machining_blue_pose")
                target_ave = check_info(self.target_blue)

            if get_distance([target_position[0], target_position[1]], [self.position_info.x_abs, self.position_info.y_abs]) > config.get("position_error"):
                msg = MoveGoal()
                msg.x_abs, msg.y_abs, msg.angle_abs = target_position
                self.pub1_.publish(msg)
                return

            if len(target_ave) == 0:
                return

            operate_pixel2 = config.get("operate_pixel2")
            if get_distance(operate_pixel2, target_ave) > config.get("pixel_error"):
                vx = pid_c.update(target_ave[0], operate_pixel2[0])
                vy = pid_c.update(target_ave[1], operate_pixel2[1])
                msg = MoveCmd()
                msg.vx = vx
                msg.vy = -vy
                msg.vw = 0.0
                self.pub2_.publish(msg)
            elif self.arm_cmd_flag == 0:
                self.move_stop()
                msg = ArmCmd()
                msg.act_id = ARM2_PLACE1
                for i in range(10):
                    self.pub3_.publish(msg)
                self.arm_cmd_flag = 1

        if self.task_pipeline[0] == "arm2_grap1":
            if self.position_info.stuff_num == 3:
                msg = MoveGoal()
                msg.x_abs, msg.y_abs, msg.angle_abs = config.get(
                    "staging_pose")
                self.pub1_.publish(msg)
                print(self.task_pipeline)
                self.task_pipeline.pop(0)
                self.task_index = self.task_index - 3
                return

            color = self.task_sequence[self.task_index]
            target_position = []

            if color == 1:
                target_position = config.get("machining_red_pose")
            if color == 2:
                target_position = config.get("machining_green_pose")
            if color == 3:
                target_position = config.get("machining_blue_pose")

            if get_distance([target_position[0], target_position[1]], [self.position_info.x_abs, self.position_info.y_abs]) > config.get("position_error"):
                msg = MoveGoal()
                msg.x_abs, msg.y_abs, msg.angle_abs = target_position
                self.pub1_.publish(msg)
            elif self.arm_cmd_flag == 0:
                msg = ArmCmd()
                msg.act_id = ARM2_PLACE1
                for i in range(10):
                    self.pub3_.publish(msg)
                self.arm_cmd_flag = 1

        if self.task_pipeline[0] == "arm2_place2":
            if self.position_info.stuff_num == 0:
                msg = MoveGoal()
                if self.task_index == 3:
                    msg.x_abs, msg.y_abs, msg.angle_abs = config.get(
                        "material_pose")
                    self.task_index = self.task_index + 3
                else:
                    msg.x_abs, msg.y_abs, msg.angle_abs = config.get(
                        "start_pose")
                    self.task_index == self.task_index - 3
                self.pub1_.publish(msg)
                print(self.task_pipeline)
                self.task_pipeline.pop(0)
                self.task_index = self.task_index - 3
                return

            color = self.task_sequence[self.task_index]
            target_position = []
            target_ave = []

            if color == 1:
                target_position = config.get("staging_red_pose")
                target_ave = check_info(self.target_red)
            if color == 2:
                target_position = config.get("staging_green_pose")
                target_ave = check_info(self.target_green)
            if color == 3:
                target_position = config.get("staging_blue_pose")
                target_ave = check_info(self.target_blue)

            if get_distance([target_position[0], target_position[1]], [self.position_info.x_abs, self.position_info.y_abs]) > config.get("position_error"):
                msg = MoveGoal()
                msg.x_abs, msg.y_abs, msg.angle_abs = target_position
                self.pub1_.publish(msg)
                return

            if len(target_ave) == 0:
                return

            operate_pixel2 = config.get("operate_pixel2")
            if get_distance(operate_pixel2, target_ave) > config.get("pixel_error"):
                vx = pid_c.update(target_ave[0], operate_pixel2[0])
                vy = pid_c.update(target_ave[1], operate_pixel2[1])
                msg = MoveCmd()
                msg.vx = vx
                msg.vy = -vy
                msg.vw = 0.0
                self.pub2_.publish(msg)
            elif self.arm_cmd_flag == 0:
                self.move_stop()
                msg = ArmCmd()
                if self.task_index < 3:
                    msg.act_id = ARM2_PLACE1
                else:
                    msg.act_id = ARM2_PLACE2
                for i in range(10):
                    self.pub3_.publish(msg)
                self.arm_cmd_flag = 1


def main(args=None):
    rclpy.init(args=args)
    node = Policy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
