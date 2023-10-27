#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tt_pkg.msg import PositionInfo, MoveCmd, MoveGoal, ArmCmd, DetectInfo
from tt_pkg.config import config
from tt_pkg.PID import pid_c

ARM_RST = 0x01
ARM_TO_CODE = 0x02
ARM_TO_STUFF = 0x03
ARM_GRAB_MATERIAL = 0x04
ARM_PLACE_GROUND = 0x05
ARM_GRAB_GROUND = 0x06
ARM_PLACE_STUFF = 0x07


def get_distance(point1, point2):
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)


def get_time_diff(stamp1, stamp2):
    # 获取时间戳并将其转换为秒
    timestamp1 = stamp1.sec + stamp1.nanosec / 1e9
    timestamp2 = stamp2.sec + stamp2.nanosec / 1e9

    # 计算时间差
    time_difference = abs(timestamp1 - timestamp2)
    return time_difference


class Policy(Node):
    def __init__(self):
        self.task_pipeline = [
            "ready",
            "wait_for_task_sequence",
            "arm_grab_material",
            "arm_place_machining",
            "arm_grab_machining",
            "arm_place_staging",
            "arm_grab_material",
            "arm_place_machining",
            "arm_grab_machining",
            "arm_place_staging"
        ]
        self.task_sequence = []
        self.task_index = -1
        self.arm_cmd_flag = 0
        self.arm_cmd_flag_last = 0
        self.stuff_info = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        self.target_info = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        self.position_info = PositionInfo()
        self.position_info.stuff_num = 100

        self.machining_poses = [config.get("machining_red_pose"), config.get("machining_green_pose"), config.get("machining_blue_pose")]
        self.stagine_poses = [config.get("staging_red_pose"), config.get("staging_green_pose"), config.get("staging_blue_pose")]

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
        self.get_logger().info("policy_node is started successfully.")

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
        if self.arm_cmd_flag_last == 0:
            self.stuff_info[0] = [msg.r_f, msg.r_x, msg.r_y]
            self.stuff_info[1] = [msg.g_f, msg.g_x, msg.g_y]
            self.stuff_info[2] = [msg.b_f, msg.b_x, msg.b_y]
        else:
            self.stuff_info = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]

    def sub3_callback(self, msg):
        if self.arm_cmd_flag_last == 0:
            self.target_info[0] = [msg.r_f, msg.r_x, msg.r_y]
            self.target_info[1] = [msg.g_f, msg.g_x, msg.g_y]
            self.target_info[2] = [msg.b_f, msg.b_x, msg.b_y]
        else:
            self.target_info = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]

    def sub4_callback(self, msg):
        stuff_num_last = self.position_info.stuff_num
        self.position_info = msg
        if stuff_num_last != msg.stuff_num:
            self.task_index = self.task_index + 1
            time.sleep(0.2)
            self.arm_cmd_flag = 0

    def timer_callback(self):
        if len(self.task_pipeline) == 0:
            return

        # print(self.task_pipeline[0])
        # print("Position_info: ", self.position_info.x_abs, self.position_info.y_abs, self.position_info.angle_abs)
        # if len(self.task_sequence) == 6:
        #     print("Color_index: ", self.task_sequence[self.task_index])
        # print("Stuff_info:\nRed: ", self.stuff_info[0], "\nGreen: ", self.stuff_info[1], "\nBlue: ", self.stuff_info[2])
        # print("Target_info:\nRed: ", self.target_info[0], "\nGreen: ", self.target_info[1], "\nBlue: ", self.target_info[2])

        if self.task_pipeline[0] == "ready":
            if self.position_info.stuff_num >= 0 and self.position_info.stuff_num <= 3:
                msg = MoveGoal()
                msg.x_abs, msg.y_abs, msg.angle_abs = config.get(
                    "qr_code_pose")
                self.pub1_.publish(msg)
                self.task_pipeline.pop(0)

                time.sleep(1)
                msg = ArmCmd()
                msg.act_id = ARM_TO_CODE
                for i in range(10):
                    self.pub3_.publish(msg)

                return

        if self.task_pipeline[0] == "wait_for_task_sequence":
            if len(self.task_sequence) == 6:
                msg = MoveGoal()
                msg.x_abs, msg.y_abs, msg.angle_abs = config.get(
                    "material_pose")
                self.pub1_.publish(msg)
                print(self.task_pipeline)
                time.sleep(0.5)
                msg = ArmCmd()
                msg.act_id = ARM_TO_STUFF
                for i in range(10):
                    self.pub3_.publish(msg)
                self.task_pipeline.pop(0)
                return
        
        if self.task_pipeline[0] == "arm_grab_material":
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
            if get_distance([material_pose[0], material_pose[1]], [self.position_info.x_abs, self.position_info.y_abs]) > position_err:
                msg = MoveGoal()
                msg.x_abs, msg.y_abs, msg.angle_abs = material_pose
                self.pub1_.publish(msg)
                return
            
            if self.arm_cmd_flag_last == 0:
                operate_pixel1 = config.get("operate_pixel1")
                operate_err = config.get("operate_err")
                color = self.task_sequence[self.task_index]
                if self.stuff_info[color - 1][0] != 0 and (self.stuff_info[color - 1][1] - operate_pixel1[0]) ** 2 < operate_err ** 2:
                    msg = ArmCmd()
                    msg.act_id = ARM_GRAB_MATERIAL
                    for i in range(10):
                        self.pub3_.publish(msg)
                    self.arm_cmd_flag = 1

        if self.task_pipeline[0] == "arm_place_machining":
            if self.position_info.stuff_num == 0:
                # msg = MoveGoal()
                # msg.x_abs, msg.y_abs, msg.angle_abs = config.get("machining_pose")
                # self.pub1_.publish(msg)
                print(self.task_pipeline)
                self.task_pipeline.pop(0)
                self.task_index = self.task_index - 3
                return
            
            color = self.task_sequence[self.task_index]
            target_position = self.machining_poses[color - 1]

            if get_distance([target_position[0], target_position[1]], [self.position_info.x_abs, self.position_info.y_abs]) > config.get("position_error"):
                msg = MoveGoal()
                msg.x_abs, msg.y_abs, msg.angle_abs = target_position
                self.pub1_.publish(msg)
                return
            
            if self.arm_cmd_flag_last == 0:
                # operate_pixel2 = config.get("operate_pixel2")
                # pixel_err = config.get("pixel_error")
                # if self.target_info[color - 1][0] != 0 and get_distance(operate_pixel2, [self.target_info[color - 1][1], self.target_info[color - 1][2]]) > pixel_err:
                #     vx = pid_c.update(self.target_info[color - 1][1], operate_pixel2[0])
                #     vy = pid_c.update(self.target_info[color - 1][2], operate_pixel2[1])
                #     msg = MoveCmd()
                #     msg.vx = vx
                #     msg.vy = -vy
                #     msg.vw = 0.0
                #     self.pub2_.publish(msg)
                # else:
                #     self.move_stop()
                self.machining_poses[color - 1] = [self.position_info.x_abs, self.position_info.y_abs, self.position_info.angle_abs]
                msg = ArmCmd()
                msg.act_id = ARM_PLACE_GROUND
                for i in range(10):
                    self.pub3_.publish(msg)
                self.arm_cmd_flag = 1

        if self.task_pipeline[0] == "arm_grab_machining":
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
            target_position = self.machining_poses[color - 1]

            if get_distance([target_position[0], target_position[1]], [self.position_info.x_abs, self.position_info.y_abs]) > config.get("position_error"):
                msg = MoveGoal()
                msg.x_abs, msg.y_abs, msg.angle_abs = target_position
                self.pub1_.publish(msg)
                return
            
            if self.arm_cmd_flag_last == 0:
                msg = ArmCmd()
                msg.act_id = ARM_GRAB_GROUND
                for i in range(10):
                    self.pub3_.publish(msg)
                self.arm_cmd_flag = 1

        if self.task_pipeline[0] == "arm_place_staging":
            if self.position_info.stuff_num == 0:
                if self.task_index == 3:
                    msg = MoveGoal()
                    msg.x_abs, msg.y_abs, msg.angle_abs = config.get(
                        "material_pose")
                    # self.task_index = self.task_index + 3
                    self.pub1_.publish(msg)
                else:
                    msg = MoveGoal()
                    msg.x_abs, msg.y_abs, msg.angle_abs = config.get(
                        "end_pose")
                    self.pub1_.publish(msg)
                    time.sleep(1)
                    msg_arm = ArmCmd()
                    msg_arm.act_id = ARM_RST
                    for i in range(10):
                        self.pub3_.publish(msg_arm)
                print(self.task_pipeline)
                self.task_pipeline.pop(0)
                self.task_index = self.task_index - 3
                return
            
            color = self.task_sequence[self.task_index]
            target_position = self.stagine_poses[color - 1]

            if get_distance([target_position[0], target_position[1]], [self.position_info.x_abs, self.position_info.y_abs]) > config.get("position_error"):
                msg = MoveGoal()
                msg.x_abs, msg.y_abs, msg.angle_abs = target_position
                self.pub1_.publish(msg)
                return

            # if self.task_index < 3:
            #     operate_pixel2 = config.get("operate_pixel2")
            #     pixel_err = config.get("pixel_error")
            #     if self.target_info[color - 1][0] != 0 and get_distance(operate_pixel2, [self.target_info[color - 1][1], self.target_info[color - 1][2]]) > pixel_err:
            #         vx = pid_c.update(self.target_info[color - 1][1], operate_pixel2[0])
            #         vy = pid_c.update(self.target_info[color - 1][2], operate_pixel2[1])
            #         msg = MoveCmd()
            #         msg.vx = vx
            #         msg.vy = -vy
            #         msg.vw = 0.0
            #         self.pub2_.publish(msg)
            #         return
                
            if self.arm_cmd_flag_last == 0:
                # self.move_stop()
                msg = ArmCmd()
                if self.task_index < 3:
                    self.stagine_poses[color - 1] = [self.position_info.x_abs, self.position_info.y_abs, self.position_info.angle_abs]
                    msg.act_id = ARM_PLACE_GROUND
                else:
                    msg.act_id = ARM_PLACE_STUFF
                for i in range(10):
                    self.pub3_.publish(msg)
                self.arm_cmd_flag = 1
        
        self.arm_cmd_flag_last = self.arm_cmd_flag


def main(args=None):
    rclpy.init(args=args)
    node = Policy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
