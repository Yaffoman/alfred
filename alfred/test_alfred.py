#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class AlfredTester(Node):
    """
    Alfred Tester node that tests the alfred body node.

    Responsibilities:
    - Publishes commands to the alfred/command topic
    - Cycles through a list of commands
    - Publishes the response to the alfred/response topic
    - No need for camera or voice model, just tests the body node.
    """
    def __init__(self):
        super().__init__('alfred_tester')
        
        # 1. Create Publisher
        # This talks to the SAME topic Alfred is listening to
        self.publisher_ = self.create_publisher(String, '/alfred/command', 10)
        
        # 2. Timer to send commands automatically
        timer_period = 3.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
        # List of commands to cycle through
        self.commands = ["bring me tea", "go to the kitchen", "sleep", "unknown command"]

    def timer_callback(self):
        msg = String()
        # Cycle through the list of commands
        command_text = self.commands[self.i % len(self.commands)]
        msg.data = command_text
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    tester = AlfredTester()
    rclpy.spin(tester)
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()