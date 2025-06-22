import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import threading
import queue
import fp_robot.kinematic as kc

class DirectionPublisher(Node):
    def __init__(self):
        super().__init__('direction_publisher')
        self.publisher_ = self.create_publisher(String, 'esp32_data', 10)

        self.directions = []
        self.index = 0
        self.start_time = time.time()

        self.instruction_queue = queue.Queue()
        self.worker_thread = threading.Thread(target=self.worker_loop)
        self.worker_thread.daemon = True
        self.worker_thread.start()
        # self.obstacle = False
 
        self.timer = self.create_timer(0.001, self.enqueue_next_direction)

        self.subscription = self.create_subscription(
            String,
            'maze_commands',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        parts = msg.data.strip().split()
        self.get_logger().info(f"Received: {parts}")
        self.directions = parts
        self.index = 0
        self.start_time = time.time()
        self.get_logger().info(f"Enqueuing {len(self.directions)} directions...")
        # self.obstacle = parts[0]



    def enqueue_next_direction(self):
        if self.index < len(self.directions):
            dir_char = self.directions[self.index]
            self.instruction_queue.put((self.index, dir_char, time.time()))
            self.index += 1
        else:
            total_time = time.time() - self.start_time
            # self.get_logger().info(f"✅ All directions enqueued in {total_time:.2f} seconds.")
            # self.destroy_timer(self.timer)

    def worker_loop(self):
        while True:
            index, dir_char, enqueue_time = self.instruction_queue.get()

            def publisher_fn(msg):
                ros_msg = String()
                ros_msg.data = msg
                self.publisher_.publish(ros_msg)
                self.get_logger().info(f"[{index + 1}] Sent: {msg}")

            
            start = time.time()
            kc.process_data(dir_char, publisher_fn)
            end = time.time()   

            self.get_logger().info(
                f"🔄 Direction {index + 1}/{len(self.directions)} '{dir_char}' processed in {end - start:.3f}s "
                f"(Waited {start - enqueue_time:.3f}s in queue), (there is an obstacle)"
            )


def main(args=None):
    rclpy.init(args=args)
    node = DirectionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
1
if __name__ == '__main__':
    main()
