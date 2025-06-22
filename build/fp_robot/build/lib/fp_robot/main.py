# main.py

import rclpy
from fp_robot.udp_node import UDPNode

def main(args=None):
    rclpy.init(args=args)

    node = UDPNode()

    try:
        rclpy.spin(node)  # Tunggu dan proses callback dari subscriber
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
