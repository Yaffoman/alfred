#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from thefuzz import fuzz, process

class Brain(Node):
    """
    Brain node that listens for images + text, sends to LLM, and returns response.
    For now, skips the LLM and just logs the received data.

    Responsibilities:
    - Subscribes to the camera/image_raw topic and the voice/text topic
    - Sends the data to the LLM (gemini)
    - Publishes the response to the alfred/command topic
    """
    def __init__(self):
        super().__init__('brain')
        
        # Define available commands that the butler understands
        # Based on robot_controller.py parse_and_execute method
        self.available_commands = [
            "move forward",
            "go forward",
            "move backward",
            "go back",
            "turn left",
            "turn right",
            "stop",
            "halt",
            "home",
            "sleep",
            "up",
            "down",
            "left",
            "right",
            "crouch",
            "to attention",
            "attention",
            "get ready",
            "salute",
            "open gripper",
            "close gripper",
            "tea"
        ]
        
        # Minimum similarity threshold for fuzzy matching (0-100)
        self.similarity_threshold = self.declare_parameter('similarity_threshold', 60).value
        
        # Subscribe to camera images
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        # Subscribe to text input (e.g., from voice model or other sources)
        self.text_subscription = self.create_subscription(
            String,
            '/voice/text',
            self.text_callback,
            10)
        
        # Publish LLM responses to butler
        self.command_publisher = self.create_publisher(
            String,
            '/alfred/command',
            10)
        
        # Store latest image and text for processing
        self.latest_image = None
        self.latest_text = None
        
        self.get_logger().info('Brain node started. Waiting for images and text...')
        self.get_logger().info(f'Fuzzy matching threshold: {self.similarity_threshold}%')

    def image_callback(self, msg):
        """
        Callback for incoming camera images.
        """
        self.latest_image = msg
        self.get_logger().info(f'Received image: {msg.width}x{msg.height}, encoding: {msg.encoding}')
        
        # If we have both image and text, process them
        if self.latest_text is not None:
            self.process_multimodal_input()

    def text_callback(self, msg):
        """
        Callback for incoming text (e.g., from voice model).
        """
        self.latest_text = msg.data
        self.get_logger().info(f'Received text: "{msg.data}"')
        
        # Process text-only commands immediately (no need to wait for image)
        matched_command = self.match_command(self.latest_text)
        if matched_command:
            self.publish_command(matched_command)
            self.latest_text = None  # Reset after processing
        
        # If we have both image and text, process them for multimodal LLM
        if self.latest_image is not None:
            self.process_multimodal_input()

    def match_command(self, text):
        """
        Use fuzzy matching to find the best matching command from available commands.
        
        Args:
            text: Input text to match against available commands
            
        Returns:
            Matched command string if similarity >= threshold, None otherwise
        """
        if not text or not text.strip():
            return None
        
        # Use thefuzz to find the best match
        # process.extractOne returns (matched_string, score, index)
        result = process.extractOne(
            text.lower().strip(),
            self.available_commands,
            scorer=fuzz.ratio
        )
        
        if result:
            matched_command, similarity_score = result
            self.get_logger().info(
                f'Fuzzy match: "{text}" -> "{matched_command}" '
                f'(similarity: {similarity_score}%)'
            )
            
            if similarity_score >= self.similarity_threshold:
                return matched_command
            else:
                self.get_logger().warn(
                    f'Match below threshold ({similarity_score}% < {self.similarity_threshold}%)'
                )
                return None
        
        return None

    def publish_command(self, command):
        """
        Publish a matched command to the butler.
        
        Args:
            command: Command string to publish
        """
        command_msg = String()
        command_msg.data = command
        self.command_publisher.publish(command_msg)
        self.get_logger().info(f'Published command to butler: "{command}"')

    def process_multimodal_input(self):
        """
        Process image + text through LLM.
        For now, uses fuzzy matching on the text portion.
        In the future, this will send image + text to LLM.
        """
        if self.latest_image is None or self.latest_text is None:
            return
        
        self.get_logger().info('Processing multimodal input (image + text)...')
        
        # In the future: send image + text to LLM
        # response = self.llm_process(self.latest_image, self.latest_text)
        # For now, use fuzzy matching on the text
        matched_command = self.match_command(self.latest_text)
        
        if matched_command:
            self.publish_command(matched_command)
        else:
            self.get_logger().warn(
                f'Could not match multimodal input text: "{self.latest_text}"'
            )
        
        # Reset for next multimodal input
        self.latest_image = None
        self.latest_text = None

def main(args=None):
    rclpy.init(args=args)
    brain = Brain()
    rclpy.spin(brain)
    brain.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

