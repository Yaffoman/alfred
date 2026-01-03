#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String, Float64MultiArray

# MoveIt Action Messages
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint

from alfred.shared_consts import JOINT_NAMES, NAMED_POSES, GROUP_NAME


class Body(Node):
    def __init__(self):
        super().__init__('alfred_body')

        # 1. Subscribe to Commands
        # Listens for "up", "home", etc.
        self._send_goal_future = None
        self._get_result_future = None
        self.subscription = self.create_subscription(
            String,
            '/alfred/command',
            self.command_callback,
            10)
        self.joints_subscription = self.create_subscription(
            Float64MultiArray,
            '/alfred/joints',
            self.joints_callback,
            10)
        # 2. Setup Action Client for MoveIt
        self._action_client = ActionClient(self, MoveGroup, '/move_action')

        self.get_logger().info("Alfred Body waiting for MoveIt Server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Connected to MoveIt! Ready for commands.")

    def command_callback(self, msg):
        command = msg.data.lower().strip()
        self.get_logger().info(f"Received command: '{command}'")

        if command in NAMED_POSES:
            joint_values = NAMED_POSES[command]
            self.move_to_target(joint_values)
        else:
            self.get_logger().warning(f"Unknown pose '{command}'. Available: {list(NAMED_POSES.keys())}")

    def joints_callback(self, joints: Float64MultiArray):
        self.get_logger().info(f"Received joints: '{joints}'")
        self.move_to_target(joints.data)

    def move_to_target(self, joint_values: list[float]):

        # 1. Build the Goal Message
        goal_msg = MoveGroup.Goal()
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.replan = True
        goal_msg.request.group_name = GROUP_NAME
        goal_msg.request.max_velocity_scaling_factor = 1.0
        goal_msg.request.max_acceleration_scaling_factor = 1.0

        # 2. Create Constraints (The "Target")
        constraints = Constraints()
        for i, angle in enumerate(joint_values):
            jc = JointConstraint()
            jc.joint_name = JOINT_NAMES[i]
            jc.position = float(angle)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal_msg.request.goal_constraints.append(constraints)

        # 3. Send to Server
        self.get_logger().info(f"Sending goal '{joint_values}' to MoveIt...")
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by MoveIt :(')
            return

        self.get_logger().info('Goal accepted! Executing...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code.val == 1: # 1 = SUCCESS
            self.get_logger().info('Movement finished successfully.')
        else:
            self.get_logger().error(f'Movement failed with error code: {result.error_code.val}')

def main(args=None):
    rclpy.init(args=args)
    node = Body()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
