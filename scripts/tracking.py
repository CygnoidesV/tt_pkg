#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from tt_pkg.msg import PositionInfo, MoveCmd, MoveGoal
from tt_pkg.config import config
from tt_pkg.PID import pid_v, pid_w


def get_time_diff(stamp1, stamp2):
    # 获取时间戳并将其转换为秒
    timestamp1 = stamp1.sec + stamp1.nanosec / 1e9
    timestamp2 = stamp2.sec + stamp2.nanosec / 1e9

    # 计算时间差
    time_difference = abs(timestamp1 - timestamp2)
    return time_difference

def get_distance(point1, point2):
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)


def get_index(point, queue):
    dist = 0
    index = None
    i = 0
    while i < len(queue):
        current_dist = get_distance(
            point, queue[i-1]) + get_distance(point, queue[i])
        if index is None or current_dist < dist:
            dist = current_dist
            index = i
        i = i + 1

    return index


def get_queue(current_pose, goal_pose, road_points):
    roads = []
    for point in road_points:
        roads.append([point[0], point[1]])
    current_point = [current_pose[0], current_pose[1]]
    goal_point = [goal_pose[0], goal_pose[1]]
    start_index = get_index(current_point, roads)
    end_index = get_index(goal_point, roads)
    # print("Current_point: ",current_point)
    # print("Goal_point: ",goal_point)
    # print("Start_index: ", start_index)
    # print("End_index: ", end_index)
    queue1 = [current_point]
    index = start_index
    while True:
        if index == end_index:
            queue1.append(goal_point)
            break
        queue1.append(roads[index])
        index = index + 1
        if index == len(roads):
            index = 0

    queue2 = [current_point]
    index = start_index
    while True:
        if index == end_index:
            queue2.append(goal_point)
            break
        index = index - 1
        if index < 0:
            index = len(roads) - 1
        queue2.append(roads[index])

    # print("Roads: ", roads)
    # print("Queue1: ", queue1)
    # print("Queue2: ", queue2)
    dist1 = dist2 = 0
    index = 1
    while index < len(queue1):
        dist1 = dist1 + get_distance(queue1[index-1], queue1[index])
        index = index + 1

    index = 1
    while index < len(queue2):
        dist2 = dist2 + get_distance(queue2[index-1], queue2[index])
        index = index + 1

    if dist1 < dist2:
        for i in range(len(queue1)):
            queue1[i].append(goal_pose[2])
        # queue1.append(goal_pose)
        return queue1
    else:
        for i in range(len(queue2)):
            queue2[i].append(goal_pose[2])
        # queue2.append(goal_pose)
        return queue2


def get_target(err):
    v_max = config.get("v_max")
    position_error = config.get("position_error")

    if err > 0:
        v_target = v_max
    else:
        v_target = -v_max

    return v_target


def adjust_angle(set_value, actual_value):
    if (set_value - actual_value) ** 2 > 180.0 ** 2:
        return 360.0
    else:
        return 0.0


class Tracking(Node):
    def __init__(self):
        super().__init__("tracking_node")
        # print(config.get("road_points"), config.get("start_point"))
        self.road_points = config.get("road_points")
        self.position_info = PositionInfo()
        self.cmd_queue = []

        self.sub1_ = self.create_subscription(PositionInfo, "position_info", self.sub1_callback, 10)
        self.sub2_ = self.create_subscription(MoveGoal, "move_goal", self.sub2_callback, 10)
        self.pub1_ = self.create_publisher(MoveCmd, "move_cmd", 10)
        self.timer_ = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info("tracking_node is started successfully.")

    def move_stop(self):
        msg = MoveCmd()
        msg.vx = 0.0
        msg.vy = 0.0
        msg.vw = 0.0
        for i in range(10):
            self.pub1_.publish(msg)

    def sub1_callback(self, msg):
        self.position_info = msg

    def sub2_callback(self, msg):
        self.cmd_queue = get_queue([self.position_info.x_abs, self.position_info.y_abs,self.position_info.angle_abs], 
                                   [msg.x_abs, msg.y_abs, msg.angle_abs], self.road_points)
        # print("Cmd_queue: ", self.cmd_queue)

    def timer_callback(self):
        current_time = rclpy.clock.Clock().now()  # 使用ROS 2的时钟来获取当前时间
        if len(self.cmd_queue) == 0 or get_time_diff(current_time, self.position_info.header.stamp) > config.get("max_time_diff"):
            return

        # print("Cmd_queue: ", self.cmd_queue)
        vx = pid_v.update(self.cmd_queue[0][0], self.position_info.x_abs)
        vy = pid_v.update(self.cmd_queue[0][1], self.position_info.y_abs)
        vw = pid_w.update(self.cmd_queue[0][2], self.position_info.angle_abs)
        # print("Goal: ", self.cmd_queue[0],"Position_info: ", self.position_info_list[-1].x_abs, self.position_info_list[-1].y_abs, self.position_info_list[-1].angle_abs)
        position_error = config.get("position_error")
        angle_error = config.get("angle_error")

        if get_distance([self.cmd_queue[0][0], self.cmd_queue[0][1]], [self.position_info.x_abs, self.position_info.y_abs]) < position_error and (self.cmd_queue[0][2]-self.position_info.angle_abs)**2 < angle_error**2:
            self.cmd_queue.pop(0)
            if len(self.cmd_queue) == 0:
                self.move_stop()
        else:
            angle = self.position_info.angle_abs * math.pi / 180
            msg = MoveCmd()
            msg.vx = vx * math.cos(angle) + vy * math.sin(angle)
            msg.vy = vy * math.cos(angle) - vx * math.sin(angle)
            msg.vw = vw
            self.pub1_.publish(msg)
            # print("V: ", msg.vx, msg.vy, msg.vw)


def main(args=None):
    rclpy.init(args=args)
    node = Tracking()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
