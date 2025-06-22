# main.py

import rclpy
from fp_robot.udp_node import UDPNode
from fp_robot.brain import MazeSolverNode

def main(args=None):
    rclpy.init(args=args)

    udp_node = UDPNode()
    # maze_node = MazeSolverNode()

    try:
        # Jalankan dua node sekaligus
        rclpy.spin(udp_node, executor=rclpy.executors.MultiThreadedExecutor())
        # rclpy.spin(maze_node)
    except KeyboardInterrupt:
        pass

    udp_node.destroy_node()
    # maze_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
