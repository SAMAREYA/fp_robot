import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import threading
import queue
import kinematic as kc

class DirectionPublisher(Node):
    def __init__(self):
        super().__init__('direction_publisher')
        self.publisher_ = self.create_publisher(String, 'esp32_data', 10)

        self.directions = ['D', 'R', 'U', 'R', 'R', 'D', 'D', 'R', 'D', 'D', 'R', 'D', 'L', 'D', 'D', 'R', 'R', 'R', 'R', 'U', 'U', 'L', 'L', 'U', 'R', 'R', 'R', 'U', 'U', 'U', 'L', 'U', 'L', 'D', 'D', 'R', 'D', 'L', 'L', 'L', 'U', 'U', 'R', 'L', 'D', 'D', 'R', 'R', 'R', 'U', 'L', 'U', 'U', 'R', 'D', 'R', 'D', 'D', 'D', 'L', 'L', 'L', 'D', 'R', 'R', 'D', 'D', 'L', 'L', 'L', 'L', 'U', 'U', 'R', 'U', 'L', 'U', 'U', 'L', 'L', 'L', 'L', 'D', 'D', 'R', 'D', 'L', 'D', 'D', 'D', 'R', 'U', 'U', 'R', 'D', 'D', 'D', 'R', 'D', 'D', 'R', 'D', 'L', 'D', 'R', 'R', 'U', 'U', 'R', 'R', 'R', 'R', 'D', 'D', 'L', 'D', 'R', 'D', 'D', 'R', 'R', 'R', 'R', 'U', 'U', 'L', 'U', 'R', 'U', 'U', 'U', 'L', 'D', 'D', 'L', 'U', 'U', 'U', 'L']
        self.index = 0
        self.start_time = time.time()

        self.instruction_queue = queue.Queue()
        self.worker_thread = threading.Thread(target=self.worker_loop)
        self.worker_thread.daemon = True
        self.worker_thread.start()
 
        self.timer = self.create_timer(0.001, self.enqueue_next_direction)

    def enqueue_next_direction(self):
        if self.index < len(self.directions):
            dir_char = self.directions[self.index]
            self.instruction_queue.put((self.index, dir_char, time.time()))
            self.index += 1
        else:
            total_time = time.time() - self.start_time
            self.get_logger().info(f"✅ All directions enqueued in {total_time:.2f} seconds.")
            self.destroy_timer(self.timer)

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
                f"(Waited {start - enqueue_time:.3f}s in queue)"
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
