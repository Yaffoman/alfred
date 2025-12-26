#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from moveit_msgs.msg import MoveItErrorCodes

# Try to import MoveIt Python bindings
# For ROS 2, MoveIt Python API may vary by version
MOVEIT_AVAILABLE = False
MOVEIT_STYLE = None

try:
    # Try ROS 2 native moveit_py (newer API)
    from moveit.planning import MoveItPy
    MOVEIT_AVAILABLE = True
    MOVEIT_STYLE = 'moveit_py'
except ImportError:
    try:
        # Try moveit_commander (works in some ROS 2 setups with compatibility layer)
        import moveit_commander
        MOVEIT_AVAILABLE = True
        MOVEIT_STYLE = 'commander'
    except ImportError:
        try:
            # Try direct Python bindings (if available)
            from moveit_py_bindings import move_group_interface
            MOVEIT_AVAILABLE = True
            MOVEIT_STYLE = 'bindings'
        except ImportError:
            MOVEIT_AVAILABLE = False

class AlfredButler(Node):
    """
    Alfred Butler node that listens for commands from the brain and executes them.

    Responsibilities:
    - Subscribes to the alfred/command topic
    - Executes the command (robot controller)
    - Converts natural language commands to robot control commands
    - Publishes cmd_vel for base movement and TargetAngle for arm joints
    """
    def __init__(self):
        super().__init__('alfred_butler')
        self.get_logger().info('Alfred is online. Awaiting your instructions, sir.')
        
        # Subscribe to natural language commands from the brain
        self.command_subscription = self.create_subscription(
            String,
            '/alfred/command',
            self.command_callback,
            10)
        
        # Publish velocity commands for base movement
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        
        # Publish arm joint angles
        # Note: Assuming ArmJoint is a custom message type
        # If it doesn't exist, you may need to create it or use std_msgs/Float64MultiArray
        # For now, using String as placeholder - you'll need to import the actual ArmJoint message
        # from your_custom_msgs.msg import ArmJoint
        self.arm_publisher = self.create_publisher(
            String,  # TODO: Replace with actual ArmJoint message type
            '/TargetAngle',
            10)
        
        # Timer and state for salute sequence
        self.salute_timer = None
        self.salute_sequence = []
        self.salute_index = 0
        
        # Initialize MoveIt (lazy initialization)
        self.move_group = None
        self.moveit_initialized = False
        

    def command_callback(self, msg):
        """
        Process natural language command and convert to robot control commands.
        """
        command = msg.data.lower()
        self.get_logger().info(f'Received command: "{command}"')
        
        # Parse command and generate control commands
        self.parse_and_execute(command)

    def parse_and_execute(self, command):
        """
        Parse natural language command and execute appropriate robot actions.
        """
        # Example command parsing logic
        if "move forward" in command or "go forward" in command:
            self.move_base(linear_x=0.5, angular_z=0.0)
        elif "move backward" in command or "go back" in command:
            self.move_base(linear_x=-0.5, angular_z=0.0)
        elif "turn left" in command:
            self.move_base(linear_x=0.0, angular_z=0.5)
        elif "turn right" in command:
            self.move_base(linear_x=0.0, angular_z=-0.5)
        elif "stop" in command or "halt" in command:
            self.move_base(linear_x=0.0, angular_z=0.0)
        elif "home" in command:
            self.move_arm_to_named("home")
        elif "sleep" in command:
            self.move_arm_to_named("sleep")
        elif "up" in command:
            self.move_arm_to_named("up")
        elif "down" in command:
            self.move_arm_to_named("down")
        elif "left" in command:
            self.move_arm_to_named("left")
        elif "right" in command:
            self.move_arm_to_named("right")
        elif "crouch" in command:
            self.move_arm_to_named("crouch")
        elif "to attention" in command or "attention" in command:
            self.move_arm_to_named("up")
        elif "get ready" in command:
            self.move_arm_to_named("crouch")
        elif "salute" in command:
            self.execute_salute()
        elif "open" in command and "gripper" in command:
            self.control_gripper("open")
        elif "close" in command and "gripper" in command:
            self.control_gripper("close")
        elif "tea" in command:
            self.get_logger().info("Preparing the arm to serve tea (Simulation).")
            # Move arm to a position suitable for serving tea
            self.move_arm_to_named("up")
        else:
            self.get_logger().warn(f'Unknown command: "{command}"')
            self.get_logger().info("I am afraid I do not know how to do that yet.")

    def move_base(self, linear_x=0.0, linear_y=0.0, linear_z=0.0, 
                  angular_x=0.0, angular_y=0.0, angular_z=0.0):
        """
        Publish velocity command for base movement.
        """
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.linear.y = float(linear_y)
        twist.linear.z = float(linear_z)
        twist.angular.x = float(angular_x)
        twist.angular.y = float(angular_y)
        twist.angular.z = float(angular_z)
        
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info(f'Published cmd_vel: linear=({linear_x}, {linear_y}, {linear_z}), '
                              f'angular=({angular_x}, {angular_y}, {angular_z})')

    def _initialize_moveit(self):
        """
        Initialize MoveIt MoveGroupInterface.
        Based on random_move.cpp initialization pattern (lines 19-24).
        """
        if self.moveit_initialized:
            return True
            
        if not MOVEIT_AVAILABLE:
            self.get_logger().warn('MoveIt not available. Using placeholder for arm movements.')
            return False
        
        try:
            # Initialize MoveGroupInterface with planning group "arm_group"
            # Following the pattern from random_move.cpp line 19
            if MOVEIT_STYLE == 'commander':
                # Using moveit_commander (works in some ROS 2 setups)
                import moveit_commander
                moveit_commander.roscpp_initialize([])
                self.move_group = moveit_commander.MoveGroupCommander("arm_group")
            elif MOVEIT_STYLE == 'moveit_py':
                # Using moveit_py (ROS 2 native, newer API)
                # Note: API may vary by version - adjust as needed
                from moveit.planning import MoveItPy
                self.moveit = MoveItPy(node_name="alfred_butler")
                # Get planning component for arm_group
                # API may be: get_planning_component or similar
                self.move_group = self.moveit.get_planning_component("arm_group")
            elif MOVEIT_STYLE == 'bindings':
                # Direct Python bindings
                from moveit_py_bindings import move_group_interface
                self.move_group = move_group_interface.MoveGroupInterface(self, "arm_group")
            else:
                self.get_logger().error('Unknown MoveIt style')
                return False
            
            # Set planning parameters (matching random_move.cpp lines 21-24)
            # setNumPlanningAttempts(10)
            if hasattr(self.move_group, 'set_num_planning_attempts'):
                self.move_group.set_num_planning_attempts(10)
            elif hasattr(self.move_group, 'setNumPlanningAttempts'):
                self.move_group.setNumPlanningAttempts(10)
            
            # setMaxVelocityScalingFactor(1)
            if hasattr(self.move_group, 'set_max_velocity_scaling_factor'):
                self.move_group.set_max_velocity_scaling_factor(1.0)
            elif hasattr(self.move_group, 'setMaxVelocityScalingFactor'):
                self.move_group.setMaxVelocityScalingFactor(1.0)
            
            # setMaxAccelerationScalingFactor(1)
            if hasattr(self.move_group, 'set_max_acceleration_scaling_factor'):
                self.move_group.set_max_acceleration_scaling_factor(1.0)
            elif hasattr(self.move_group, 'setMaxAccelerationScalingFactor'):
                self.move_group.setMaxAccelerationScalingFactor(1.0)
            
            # setPlanningTime(5.0)
            if hasattr(self.move_group, 'set_planning_time'):
                self.move_group.set_planning_time(5.0)
            elif hasattr(self.move_group, 'setPlanningTime'):
                self.move_group.setPlanningTime(5.0)
            
            self.moveit_initialized = True
            self.get_logger().info('MoveIt initialized successfully')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to initialize MoveIt: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return False

    def move_arm_to_named(self, target_name):
        """
        Move arm to a named position (e.g., "home", "sleep", "up", "down", etc.).
        Based on random_move.cpp pattern (lines 28-35): setNamedTarget -> plan -> execute
        """
        # Initialize MoveIt if not already done
        if not self._initialize_moveit():
            # Fallback to placeholder if MoveIt not available
            arm_msg = String()
            arm_msg.data = f"move_arm_to_{target_name}"
            self.arm_publisher.publish(arm_msg)
            self.get_logger().warn(f'MoveIt not available, using placeholder for: {target_name}')
            return
        
        try:
            # Set the named target (matching random_move.cpp line 28: setNamedTarget("up"))
            if hasattr(self.move_group, 'set_named_target'):
                self.move_group.set_named_target(target_name)
            elif hasattr(self.move_group, 'setNamedTarget'):
                self.move_group.setNamedTarget(target_name)
            else:
                self.get_logger().error('MoveGroupInterface does not support set_named_target')
                return
            
            # Plan the movement (matching random_move.cpp lines 26, 29)
            # moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            # bool success = (move_group_interface_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            plan_result = None
            if hasattr(self.move_group, 'plan'):
                plan_result = self.move_group.plan()
            elif hasattr(self.move_group, 'create_plan'):
                plan_result = self.move_group.create_plan()
            elif hasattr(self.move_group, 'go'):
                # Some APIs combine plan and execute in 'go'
                success = self.move_group.go(wait=True)
                if success:
                    self.get_logger().info(f'Successfully moved arm to: {target_name}')
                else:
                    self.get_logger().error(f'Movement failed for target: {target_name}')
                return
            else:
                self.get_logger().error('MoveGroupInterface does not support planning')
                return
            
            # Check if planning was successful (matching random_move.cpp line 29)
            success = False
            if plan_result is None:
                success = False
            elif isinstance(plan_result, bool):
                success = plan_result
            elif hasattr(plan_result, 'error_code'):
                # Check error code (matching MoveItErrorCode::SUCCESS)
                error_code_val = plan_result.error_code.val if hasattr(plan_result.error_code, 'val') else plan_result.error_code
                success = (error_code_val == MoveItErrorCodes.SUCCESS)
            elif hasattr(plan_result, 'trajectory'):
                # If plan has a trajectory, assume success
                success = plan_result.trajectory is not None and len(plan_result.trajectory.joint_trajectory.points) > 0
            elif hasattr(plan_result, 'planning_result'):
                # Some APIs wrap the result
                success = (plan_result.planning_result == MoveItErrorCodes.SUCCESS)
            
            if success:
                # Execute the plan (matching random_move.cpp line 34: move_group_interface_->execute(my_plan))
                if hasattr(self.move_group, 'execute'):
                    if hasattr(self.move_group.execute, '__call__'):
                        # execute(plan, wait=True) or execute(plan_result)
                        self.move_group.execute(plan_result, wait=True)
                    else:
                        self.move_group.execute()
                elif hasattr(self.move_group, 'async_execute'):
                    self.move_group.async_execute(plan_result)
                
                self.get_logger().info(f'Successfully moved arm to: {target_name}')
            else:
                self.get_logger().error(f'Planning failed for target: {target_name}')
                # Log error code if available
                if hasattr(plan_result, 'error_code'):
                    error_code = plan_result.error_code.val if hasattr(plan_result.error_code, 'val') else plan_result.error_code
                    self.get_logger().error(f'Error code: {error_code}')
                
        except Exception as e:
            self.get_logger().error(f'Error executing arm movement to {target_name}: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            # Fallback to placeholder on error
            arm_msg = String()
            arm_msg.data = f"move_arm_to_{target_name}"
            self.arm_publisher.publish(arm_msg)

    def control_gripper(self, action):
        """
        Control gripper (open/close).
        """
        # TODO: Replace with actual ArmJoint message for gripper control
        gripper_msg = String()
        gripper_msg.data = f"gripper_{action}"
        self.arm_publisher.publish(gripper_msg)
        self.get_logger().info(f'Published gripper command: {action}')

    def execute_salute(self):
        """
        Execute salute sequence: left -> right -> left -> right
        """
        # Cancel any existing salute timer
        if self.salute_timer is not None:
            self.salute_timer.cancel()
        
        # Initialize salute sequence: left -> right -> left -> right
        self.salute_sequence = ["left", "right", "left", "right"]
        self.salute_index = 0
        
        # Start the sequence
        self._salute_step()
        
        self.get_logger().info('Starting salute sequence')

    def _salute_step(self):
        """
        Execute the next step in the salute sequence.
        """
        if self.salute_index < len(self.salute_sequence):
            target = self.salute_sequence[self.salute_index]
            self.move_arm_to_named(target)
            self.salute_index += 1
            
            # Schedule next step after a delay (1 second) if there are more steps
            # In a real implementation, you might want to wait for movement completion
            if self.salute_index < len(self.salute_sequence):
                # Cancel existing timer if any
                if self.salute_timer is not None:
                    self.salute_timer.cancel()
                # Create a one-shot timer by canceling after first callback
                self.salute_timer = self.create_timer(1.0, self._salute_timer_callback)
            else:
                # Sequence complete
                self.get_logger().info('Salute sequence complete')
                if self.salute_timer is not None:
                    self.salute_timer.cancel()
                    self.salute_timer = None
        else:
            # Sequence complete
            self.get_logger().info('Salute sequence complete')
            if self.salute_timer is not None:
                self.salute_timer.cancel()
                self.salute_timer = None

    def _salute_timer_callback(self):
        """
        Timer callback for salute sequence.
        """
        # Cancel timer immediately to make it one-shot
        if self.salute_timer is not None:
            self.salute_timer.cancel()
            self.salute_timer = None
        # Execute next step
        self._salute_step()

def main(args=None):
    rclpy.init(args=args)
    butler = AlfredButler()
    rclpy.spin(butler)
    butler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

