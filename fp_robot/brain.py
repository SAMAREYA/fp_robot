import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from fp_robot.pymaze import *
import fp_robot.astar as at

class MazeSolverNode(Node):
    def __init__(self):
        super().__init__('maze_solver_node')
        self.publisher_ = self.create_publisher(String, 'maze_commands', 10)
        self.subscriber_ = self.create_subscription(String, 'udp_data', self.obstacle_callback, 10)
        
        self.maze_file = '/home/henray/RoMaze_ws/src/fp_robot/fp_robot/Maze_Loop50.csv'
        self.goal = [(2, 7), (10, 4), (10, 11)]  
        self.goal_target = 0
        self.visited_path = [False] * len(self.goal)
        self.current_pos = (1, 1)

        self.command_list = []
        self.path = {}
        self.step_index = 0
        self.executing = False
        self.obstacle_detected = False

        self.get_logger().info("MazeSolverNode started.")
        self.calculate_path()
        self.timer = self.create_timer(2.0, self.timer_callback)  # Setiap 1 detik

    def timer_callback(self):
        if self.executing:
            self.step_once()


    def calculate_path(self):
        target = self.goal[self.goal_target]

        if self.current_pos == target:
            self.visited_path[self.goal_target] = True
            self.get_logger().info(f"✅ Reached goal {self.goal_target+1} at {target}")

            # Cari goal berikutnya
            self.goal_target += 1
            if self.goal_target >= len(self.goal):
                self.get_logger().info("🏁 All goals reached.")
                return
            else:
                target = self.goal[self.goal_target]
                self.get_logger().info(f"🎯 Moving to next goal: {target}")

        m = maze(10, 11)
        m.CreateMaze(x=target[0], y=target[1], loadMaze=self.maze_file)

        searchPath, _, fwdPath = at.aStar(m, start=self.current_pos, goal=target)

        if not fwdPath:
            self.get_logger().error("❌ No path found!")
            return

        self.path = fwdPath
        self.command_list = [self.command(k, v) for k, v in fwdPath.items()]
        self.command_list.reverse()
        self.step_index = 0

        self.get_logger().info(f"📌 Calculated path to goal {self.goal_target+1}: {self.command_list}")

    def command(self, curr, next_):
        x1, y1 = curr
        x2, y2 = next_
        if x1 == x2 and y2 == y1+1:
            return 'R'
        elif x1 == x2 and y2 == y1-1:
            return 'L'
        elif x2 == x1+1 and y1 == y2:
            return 'D'
        elif x2 == x1-1 and y1 == y2:
            return 'U'

    def obstacle_callback(self, msg: Bool):
        self.executing = False
        self.obstacle_detected = msg.data

        if msg.data == 'True':
            self.get_logger().warn("🚧 Obstacle detected. Recalculating path...")
            self.calculate_path()
        else:
            self.executing = True

    def step_once(self):
        if self.step_index < len(self.command_list):
            cmd = self.command_list[self.step_index]
            self.publisher_.publish(String(data=cmd))
            self.get_logger().info(f"🚀 Sent: {cmd}")
            self.update_position(cmd)
            self.step_index += 1
        else:
            self.calculate_path()

    def update_position(self, cmd):
        x, y = self.current_pos
        if cmd == 'U':
            x -= 1
        elif cmd == 'D':
            x += 1
        elif cmd == 'L':
            y -= 1
        elif cmd == 'R':
            y += 1
        self.current_pos = (x, y)
        self.get_logger().info(f"📍 Updated position: {self.current_pos}")

def main(args=None):
    rclpy.init(args=args)
    node = MazeSolverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()