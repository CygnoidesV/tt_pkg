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

ARM_GRAB_GROUND1 = 0x09
ARM_GRAB_GROUND2 = 0x0A
ARM_GRAB_GROUND3 = 0x0B
ARM_PLACE_GROUND1 = 0x0C
ARM_PLACE_GROUND2 = 0x0D
ARM_PLACE_GROUND3 = 0x0E
ARM_GRAB_GROUND4 = 0x0F
ARM_GRAB_GROUND5 = 0x10
ARM_GRAB_GROUND6 = 0x11
ARM_PLACE_MATERIAL1 = 0x12
ARM_PLACE_MATERIAL2 = 0x13
ARM_PLACE_MATERIAL3 = 0x14
ARM_TO_TARGET = 0x15


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
            "arm_grab_staging",
            "arm_grab_staging1",
            "arm_grab_staging2",
            "arm_grab_staging3",
            "arm_place_machining",
            "arm_place_machining1",
            "arm_place_machining2",
            "arm_place_machining3",
            "arm_grab_machining1",
            "arm_grab_machining2",
            "arm_grab_machining3",
            "arm_place_material",
            "arm_place_material1",
            "arm_place_material2",
            "arm_place_material3",
            "arm_grab_staging",
            "arm_grab_staging1",
            "arm_grab_staging2",
            "arm_grab_staging3",
            "arm_place_machining",
            "arm_place_machining1",
            "arm_place_machining2",
            "arm_place_machining3",
            "arm_grab_machining1",
            "arm_grab_machining2",
            "arm_grab_machining3",
            "arm_place_material",
            "arm_place_material1",
            "arm_place_material2",
            "arm_place_material3",
            "come_to_end"
        ]
        self.task_sequence = [1, 2, 3, 3, 2, 1]
        self.task_index = -1
        self.arm_cmd_flag = 0
        self.arm_cmd_flag_last = 0
        self.stuff_info = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        self.target_info = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        self.position_info = PositionInfo()
        self.position_info.stuff_num = 100
        self.cur_time = rclpy.clock.Clock().now()
        self.last_time = self.cur_time

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
        if len(msg.data) == 7:
            self.task_sequence = [int(msg.data[0]), int(msg.data[1]), int(
                msg.data[2]), int(msg.data[4]), int(msg.data[5]), int(msg.data[6])]
        print("Task_sequence: ", self.task_sequence)

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
        self.position_info = msg

    def timer_callback(self):
        if len(self.task_pipeline) == 0:
            return
        # print("Task_sequence: ", self.task_sequence)

        # print(self.task_pipeline[0])
        # print("Position_info: ", self.position_info.x_abs, self.position_info.y_abs, self.position_info.angle_abs)
        # if len(self.task_sequence) == 6:
        #     print("Color_index: ", self.task_sequence[self.task_index])
        # print("Stuff_info:\nRed: ", self.stuff_info[0], "\nGreen: ", self.stuff_info[1], "\nBlue: ", self.stuff_info[2])
        # print("Target_info:\nRed: ", self.target_info[0], "\nGreen: ", self.target_info[1], "\nBlue: ", self.target_info[2])
        position_err = config.get("position_error")

        if self.task_pipeline[0] == "ready":
            if self.position_info.stuff_num >= 0 and self.position_info.stuff_num <= 3:
                msg = MoveGoal()
                msg.x_abs, msg.y_abs, msg.angle_abs = config.get(
                    "staging_pose")
                self.pub1_.publish(msg)
                self.task_pipeline.pop(0)

                time.sleep(2)
                msg = ArmCmd()
                msg.act_id = ARM_TO_TARGET
                for i in range(10):
                    self.pub3_.publish(msg)

                return

        if self.task_pipeline[0] == "arm_grab_staging":
            goal_pose = config.get("staging_pose")
            if get_distance([goal_pose[0], goal_pose[1]], [self.position_info.x_abs, self.position_info.y_abs]) > position_err:
                msg = MoveGoal()
                msg.x_abs, msg.y_abs, msg.angle_abs = goal_pose
                self.pub1_.publish(msg)
                return
            else:
                self.task_pipeline.pop(0)
        
        if self.task_pipeline[0] == "arm_grab_staging1":
            if self.position_info.stuff_num == 1:
                self.task_pipeline.pop(0)
                return

            last_time = rclpy.clock.Clock().now()
            goal_pose = config.get("staging_red_pose")
            while get_distance([goal_pose[0], goal_pose[1]], [self.position_info.x_abs, self.position_info.y_abs]) > position_err:
                current_time = rclpy.clock.Clock().now()
                if get_time_diff(current_time.to_msg(), last_time.to_msg()) > config.get("running_timeout"):
                    break
                msg = MoveGoal()
                msg.x_abs, msg.y_abs, msg.angle_abs = goal_pose
                self.pub1_.publish(msg)

            time.sleep(0.5)   
            msg = ArmCmd()
            msg.act_id = ARM_GRAB_GROUND1
            self.pub3_.publish(msg)
            return
        
        if self.task_pipeline[0] == "arm_grab_staging2":
            if self.position_info.stuff_num == 2:
                self.task_pipeline.pop(0)
                return

            last_time = rclpy.clock.Clock().now()
            goal_pose = config.get("staging_green_pose")
            while get_distance([goal_pose[0], goal_pose[1]], [self.position_info.x_abs, self.position_info.y_abs]) > position_err:
                current_time = rclpy.clock.Clock().now()
                if get_time_diff(current_time.to_msg(), last_time.to_msg()) > config.get("running_timeout"):
                    break
                msg = MoveGoal()
                msg.x_abs, msg.y_abs, msg.angle_abs = goal_pose
                self.pub1_.publish(msg)
            
            time.sleep(0.5)   
            msg = ArmCmd()
            msg.act_id = ARM_GRAB_GROUND2
            self.pub3_.publish(msg)
            return
        
        if self.task_pipeline[0] == "arm_grab_staging3":
            if self.position_info.stuff_num == 3:
                self.task_pipeline.pop(0)
                return

            last_time = rclpy.clock.Clock().now()
            goal_pose = config.get("staging_blue_pose")
            while get_distance([goal_pose[0], goal_pose[1]], [self.position_info.x_abs, self.position_info.y_abs]) > position_err:
                current_time = rclpy.clock.Clock().now()
                if get_time_diff(current_time.to_msg(), last_time.to_msg()) > config.get("running_timeout"):
                    break
                msg = MoveGoal()
                msg.x_abs, msg.y_abs, msg.angle_abs = goal_pose
                self.pub1_.publish(msg)
            
            time.sleep(0.5)   
            msg = ArmCmd()
            msg.act_id = ARM_GRAB_GROUND3
            self.pub3_.publish(msg)
            return
        
        if self.task_pipeline[0] == "arm_place_machining":
            goal_pose = config.get("machining_pose")
            if get_distance([goal_pose[0], goal_pose[1]], [self.position_info.x_abs, self.position_info.y_abs]) > position_err:
                msg = MoveGoal()
                msg.x_abs, msg.y_abs, msg.angle_abs = goal_pose
                self.pub1_.publish(msg)
                return
            else:
                self.task_pipeline.pop(0)
        
        if self.task_pipeline[0] == "arm_place_machining1":
            if self.position_info.stuff_num == 2:
                self.task_index = self.task_index + 1
                self.task_pipeline.pop(0)
                return

            last_time = rclpy.clock.Clock().now()
            color = self.task_sequence[self.task_index]
            goal_pose = self.machining_poses[color - 1]
            while get_distance([goal_pose[0], goal_pose[1]], [self.position_info.x_abs, self.position_info.y_abs]) > position_err:
                current_time = rclpy.clock.Clock().now()
                if get_time_diff(current_time.to_msg(), last_time.to_msg()) > config.get("running_timeout"):
                    break
                msg = MoveGoal()
                msg.x_abs, msg.y_abs, msg.angle_abs = goal_pose
                self.pub1_.publish(msg)
            
            time.sleep(0.5)   
            self.machining_poses[color - 1] = [self.position_info.x_abs, self.position_info.y_abs, self.position_info.angle_abs]
            msg = ArmCmd()
            msg.act_id = ARM_PLACE_GROUND1
            self.pub3_.publish(msg)
            return
        
        if self.task_pipeline[0] == "arm_place_machining2":
            if self.position_info.stuff_num == 1:
                self.task_index = self.task_index + 1
                self.task_pipeline.pop(0)
                return

            last_time = rclpy.clock.Clock().now()
            color = self.task_sequence[self.task_index]
            goal_pose = self.machining_poses[color - 1]
            while get_distance([goal_pose[0], goal_pose[1]], [self.position_info.x_abs, self.position_info.y_abs]) > position_err:
                current_time = rclpy.clock.Clock().now()
                if get_time_diff(current_time.to_msg(), last_time.to_msg()) > config.get("running_timeout"):
                    break
                msg = MoveGoal()
                msg.x_abs, msg.y_abs, msg.angle_abs = goal_pose
                self.pub1_.publish(msg)
            
            time.sleep(0.5)   
            self.machining_poses[color - 1] = [self.position_info.x_abs, self.position_info.y_abs, self.position_info.angle_abs]
            msg = ArmCmd()
            msg.act_id = ARM_PLACE_GROUND2
            self.pub3_.publish(msg)
            return
        
        if self.task_pipeline[0] == "arm_place_machining3":
            if self.position_info.stuff_num == 0:
                self.task_index = self.task_index - 2
                self.task_pipeline.pop(0)
                return

            last_time = rclpy.clock.Clock().now()
            color = self.task_sequence[self.task_index]
            goal_pose = self.machining_poses[color - 1]
            while get_distance([goal_pose[0], goal_pose[1]], [self.position_info.x_abs, self.position_info.y_abs]) > position_err:
                current_time = rclpy.clock.Clock().now()
                if get_time_diff(current_time.to_msg(), last_time.to_msg()) > config.get("running_timeout"):
                    break
                msg = MoveGoal()
                msg.x_abs, msg.y_abs, msg.angle_abs = goal_pose
                self.pub1_.publish(msg)
            
            time.sleep(0.5)   
            self.machining_poses[color - 1] = [self.position_info.x_abs, self.position_info.y_abs, self.position_info.angle_abs]
            msg = ArmCmd()
            msg.act_id = ARM_PLACE_GROUND3
            self.pub3_.publish(msg)
            return

        if self.task_pipeline[0] == "arm_grab_machining1":
            if self.position_info.stuff_num == 1:
                self.task_index = self.task_index + 1
                self.task_pipeline.pop(0)
                return

            last_time = rclpy.clock.Clock().now()
            color = self.task_sequence[self.task_index]
            goal_pose = self.machining_poses[color - 1]
            while get_distance([goal_pose[0], goal_pose[1]], [self.position_info.x_abs, self.position_info.y_abs]) > position_err:
                current_time = rclpy.clock.Clock().now()
                if get_time_diff(current_time.to_msg(), last_time.to_msg()) > config.get("running_timeout"):
                    break
                msg = MoveGoal()
                msg.x_abs, msg.y_abs, msg.angle_abs = goal_pose
                self.pub1_.publish(msg)
            
            
            time.sleep(0.5)   
            msg = ArmCmd()
            msg.act_id = ARM_GRAB_GROUND4
            self.pub3_.publish(msg)
            return
        
        if self.task_pipeline[0] == "arm_grab_machining2":
            if self.position_info.stuff_num == 2:
                self.task_index = self.task_index + 1
                self.task_pipeline.pop(0)
                return

            last_time = rclpy.clock.Clock().now()
            color = self.task_sequence[self.task_index]
            goal_pose = self.machining_poses[color - 1]
            while get_distance([goal_pose[0], goal_pose[1]], [self.position_info.x_abs, self.position_info.y_abs]) > position_err:
                current_time = rclpy.clock.Clock().now()
                if get_time_diff(current_time.to_msg(), last_time.to_msg()) > config.get("running_timeout"):
                    break
                msg = MoveGoal()
                msg.x_abs, msg.y_abs, msg.angle_abs = goal_pose
                self.pub1_.publish(msg)
            
            
            time.sleep(0.5)   
            msg = ArmCmd()
            msg.act_id = ARM_GRAB_GROUND5
            self.pub3_.publish(msg)
            return
        
        if self.task_pipeline[0] == "arm_grab_machining3":
            if self.position_info.stuff_num == 3:
                self.task_index = self.task_index - 2
                self.task_pipeline.pop(0)
                return

            last_time = rclpy.clock.Clock().now()
            color = self.task_sequence[self.task_index]
            goal_pose = self.machining_poses[color - 1]
            while get_distance([goal_pose[0], goal_pose[1]], [self.position_info.x_abs, self.position_info.y_abs]) > position_err:
                current_time = rclpy.clock.Clock().now()
                if get_time_diff(current_time.to_msg(), last_time.to_msg()) > config.get("running_timeout"):
                    break
                msg = MoveGoal()
                msg.x_abs, msg.y_abs, msg.angle_abs = goal_pose
                self.pub1_.publish(msg)
            
            time.sleep(0.5)
            msg = ArmCmd()
            msg.act_id = ARM_GRAB_GROUND6
            self.pub3_.publish(msg)
            return
        
        if self.task_pipeline[0] == "arm_place_material":
            goal_pose = config.get("material_pose")
            if get_distance([goal_pose[0], goal_pose[1]], [self.position_info.x_abs, self.position_info.y_abs]) > position_err:
                msg = MoveGoal()
                msg.x_abs, msg.y_abs, msg.angle_abs = goal_pose
                self.pub1_.publish(msg)
                return
            else:
                self.task_pipeline.pop(0)

        if self.task_pipeline[0] == "arm_place_material1":
            if self.position_info.stuff_num == 2:
                self.task_index = self.task_index + 1
                self.task_pipeline.pop(0)
            color = self.task_sequence[self.task_index]
            goal_pose = config.get("material_pose")
            operate_pixel1 = config.get("operate_pixel1")
            operate_err = config.get("operate_err")
            if self.stuff_info[0] != 0 and get_distance([self.stuff_info[1], operate_pixel1[0]], [self.stuff_info[2], operate_pixel1[1]]) < operate_err:
                msg = ArmCmd()
                msg.act_id = ARM_PLACE_MATERIAL1
                self.pub3_.publish(msg)
        
        if self.task_pipeline[0] == "arm_place_material2":
            if self.position_info.stuff_num == 1:
                self.task_index = self.task_index + 1
                self.task_pipeline.pop(0)
            color = self.task_sequence[self.task_index]
            goal_pose = config.get("material_pose")
            operate_pixel1 = config.get("operate_pixel1")
            operate_err = config.get("operate_err")
            if self.stuff_info[0] != 0 and get_distance([self.stuff_info[1], operate_pixel1[0]], [self.stuff_info[2], operate_pixel1[1]]) < operate_err:
                msg = ArmCmd()
                msg.act_id = ARM_PLACE_MATERIAL2
                self.pub3_.publish(msg)
        
        if self.task_pipeline[0] == "arm_place_material3":
            if self.position_info.stuff_num == 0:
                self.task_index = self.task_index + 1
                self.task_pipeline.pop(0)
            color = self.task_sequence[self.task_index]
            goal_pose = config.get("material_pose")
            operate_pixel1 = config.get("operate_pixel1")
            operate_err = config.get("operate_err")
            if self.stuff_info[0] != 0 and get_distance([self.stuff_info[1], operate_pixel1[0]], [self.stuff_info[2], operate_pixel1[1]]) < operate_err:
                msg = ArmCmd()
                msg.act_id = ARM_PLACE_MATERIAL3
                self.pub3_.publish(msg)

        if self.task_pipeline[0] == "come_to_end":
            goal_pose = config.get("start_pose")
            if get_distance([goal_pose[0], goal_pose[1]], [self.position_info.x_abs, self.position_info.y_abs]) > position_err:
                msg = MoveGoal()
                msg.x_abs, msg.y_abs, msg.angle_abs = goal_pose
                self.pub1_.publish(msg)
                return
            else:
                self.task_pipeline.pop(0)


def main(args=None):
    rclpy.init(args=args)
    node = Policy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
