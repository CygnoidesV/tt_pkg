#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
import time
import threading

# expression_list = ["(♥◠‿◠)ﾉ", "(๑•̀ㅂ•́)✧", "(ง •̀_•́)ง", "(๑•ั็ω•็ั๑)", "(*￣∇￣*)", "(｀･д･′)", "(.•˘_˘•.)", "( ′◔ ‸◔`)", "( ◔ ‸◔？)","（ﾉ′д｀）", "(つ﹏⊂)","(╥╯^╰╥)", "(╥﹏╥)"]
expression_list = ["WORKING"]


class Display(Node):
    def __init__(self):
        super().__init__("display_node")
        self.sub1_ = self.create_subscription(
            String, "task_sequence", self.sub1_callback, 10)
        self.get_logger().info("display_node is started successfully.")

        self.task_sequence = None

    def sub1_callback(self, msg):
        self.task_sequence = msg.data


def gui_part(display_node):
    # 创建GUI窗口
    root = tk.Tk()
    root.title("Task Sequence Display")

    # 控制窗口大小和位置
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    window_width = int(screen_width * 0.9)
    window_height = int(screen_height * 0.9)
    x = (screen_width - window_width) // 2
    y = (screen_height - window_height) // 2
    root.geometry(f"{window_width}x{window_height}+{x}+{y}")

    # 创建标签用于显示任务序列
    label = tk.Label(root, text=expression_list[0], font=(
        "Arial", int(window_height * 0.3)))
    # 将标签放置在窗口中央
    label.pack(expand=True, anchor='center')
    cnt = 0

    while True:
        if display_node.task_sequence is None:
            if cnt // 50 < len(expression_list):
                label.config(text=expression_list[cnt//50])
                cnt = cnt + 1
        else:
            label.config(text=display_node.task_sequence)

        root.update()
        # Add a small delay to control update frequency
        time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = Display()
    # 启动一个线程来更新GUI
    gui_thread = threading.Thread(target=gui_part, args=(node,), daemon=True)
    gui_thread.start()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
