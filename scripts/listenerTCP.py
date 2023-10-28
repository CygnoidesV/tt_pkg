#! /usr/bin/env python

import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class NetworkPortListener(Node):

    def __init__(self):
        super().__init__('network_port_listener')
        self.pub1_ = self.create_publisher(String, "task_sequence", 10)  # 将发布者正确赋值给self.pub1_

        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.bind(('0.0.0.0', 9050))
        self.tcp_socket.listen(5)
        self.get_logger().info("network_port_listener is started successfully.")
        self.accept_connections()


    def accept_connections(self):
        self.get_logger().info("Listening for incoming connections...")
        while rclpy.ok():
            client_socket, addr = self.tcp_socket.accept()
            # self.get_logger().info(f"Accepted connection from {addr[0]}:{addr[1]}")
            self.handle_client(client_socket)

    def handle_client(self, client_socket):
        with client_socket:
            while rclpy.ok():
                data = client_socket.recv(1024)
                if not data:
                    break  # 连接已关闭

                msg = String()
                msg.data = data.decode('utf-8')
                self.pub1_.publish(msg)
                # self.get_logger().info("Publishing: %s" % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = NetworkPortListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
