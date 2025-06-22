# udp_node.py

import rclpy
from rclpy.node import Node
import socket
import threading
import struct
import select
from std_msgs.msg import String

class UDPNode(Node):
    def __init__(self, node_name='udp_node'):
        super().__init__(node_name)
        self.left_speed = 0
        self.right_speed = 0

        # CONFIG
        self.udp_listen_ip = '0.0.0.0'     # Terima dari semua IP
        self.udp_listen_port = 12345       # Port ESP32
        self.udp_target_ip = '172.20.10.2'  # IP ESP32
        self.udp_target_port = 12346

        # Socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_listen_ip, self.udp_listen_port))
        self.sock.setblocking(False)

        # Logging
        self.get_logger().info(f"UDP Node started. Listening on {self.udp_listen_ip}:{self.udp_listen_port}")

        # Thread receive
        self.running = True
        self.thread = threading.Thread(target=self.receive_loop)

        self.thread.start()
        
        self.subscription = self.create_subscription(
            String,
            'esp32_data',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        try:
            parts = msg.data.strip().split()

            self.right_speed = int(parts[0])
            self.left_speed = int(parts[1])


        except Exception as e:
            self.get_logger().error(f"Failed to parse/publish: {e}")


    def receive_loop(self):
        while self.running:
            try:
                ready, _, _ = select.select([self.sock], [], [], 0.001)
                if ready:
                    data, addr = self.sock.recvfrom(1024)
                    if len(data) >= 8:
                        angka, speed = struct.unpack('if', data[:8])
                        self.get_logger().info(f"Received from {addr}: angka={angka}, speed={speed:.4f}\n")

                        self.send_data(self.right_speed, self.left_speed)
            except Exception as e:
                self.get_logger().warn(f"UDP recv error: {e}")

    def send_data(self, angka: float, speed: float):
        try:
            packed_data = struct.pack('if', angka, speed)
            self.sock.sendto(packed_data, (self.udp_target_ip, self.udp_target_port))
            self.get_logger().info(f"Sent to {self.udp_target_ip}:{self.udp_target_port} → angka={angka}, speed={speed:.4f}")
        except Exception as e:
            self.get_logger().error(f"UDP send error: {e}")

    def destroy_node(self):
        self.running = False
        self.thread.join()
        self.sock.close()
        super().destroy_node()