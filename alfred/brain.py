import sys
import os
import threading
import time

sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))

import json
import logging
from typing import List, Optional
from pydantic import BaseModel, field_validator, model_validator
import google.genai as genai
from google.genai import types
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import Image
from thefuzz import fuzz, process
from shared_consts import NAMED_POSES

client = genai.Client(api_key=os.getenv("GOOGLE_API_KEY"))


class JointModel(BaseModel):
    joints: Optional[List[float]] = None
    joints_sequence: Optional[List[List[float]]] = None

    @model_validator(mode='after')
    def validate_either_or(self):
        # 1. Check strict existence: At least one must be provided
        if self.joints is None and self.joints_sequence is None:
            raise ValueError("Response must contain either 'joints' or 'joints_sequence'")

        # 2. Validate 'joints' if it exists
        if self.joints is not None:
            if len(self.joints) != 5:
                raise ValueError(f"Field 'joints' must have exactly 5 items, got {len(self.joints)}")

        # 3. Validate 'joints_sequence' if it exists
        if self.joints_sequence is not None:
            if len(self.joints_sequence) == 0:
                raise ValueError("Field 'joints_sequence' cannot be empty")

            # Check every sub-list in the sequence
            for i, pose in enumerate(self.joints_sequence):
                if len(pose) != 5:
                    raise ValueError(f"Item {i} in 'joints_sequence' must have 5 joints, got {len(pose)}")

        return self


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

        # Define available commands that the body understands
        # Based on body_old.py parse_and_execute method
        self.available_commands = list(NAMED_POSES.keys())

        # Minimum similarity threshold for fuzzy matching (0-100)
        self.similarity_threshold = self.declare_parameter('similarity_threshold', 80).value

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

        # Publish LLM responses to body
        self.command_publisher = self.create_publisher(
            String,
            '/alfred/command',
            10)
        # Publish LLM responses to body
        self.joints_publisher = self.create_publisher(
            Float64MultiArray,
            '/alfred/joints',
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
        else:
            joint_model = process_text(self.latest_text, self.get_logger())
            if joint_model.joints:
                self.publish_joints(Float64MultiArray(data=joint_model.joints))
            else:
                self.publish_sequence_threaded(joint_model.joints_sequence)

        self.latest_text = None  # Reset after processing

    def publish_sequence_threaded(self, joints_sequence):
        """
        Runs the sequence in a background thread so we don't block ROS.
        """

        def run_seq():
            self.get_logger().info(f"Starting sequence of {len(joints_sequence)} moves...")

            for i, joints in enumerate(joints_sequence):
                # 1. Publish the current step
                self.publish_joints(Float64MultiArray(data=joints))
                self.get_logger().info(f"Step {i + 1}/{len(joints_sequence)} sent.")

                # 2. Sleep safely (this only blocks this background thread)
                time.sleep(5.0)

            self.get_logger().info("Sequence complete.")

        # Create and start the thread
        seq_thread = threading.Thread(target=run_seq, daemon=True)
        seq_thread.start()

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
                self.get_logger().warning(
                    f'Match below threshold ({similarity_score}% < {self.similarity_threshold}%)'
                )
                return None

        return None

    def publish_command(self, command):
        """
        Publish a matched command to the body.
        
        Args:
            command: Command string to publish
        """
        command_msg = String()
        command_msg.data = command
        self.command_publisher.publish(command_msg)
        self.get_logger().info(f'Published command to body: "{command}"')

    def publish_joints(self, joints):
        """
        Publish a matched joints to the body.

        Args:
            joints: Command string to publish
        """
        self.joints_publisher.publish(joints)
        self.get_logger().info(f'Published joints to body: "{joints}"')

    def process_multimodal_input(self):
        """
        Process image + text through LLM.
        For now, uses fuzzy matching on the text portion.
        In the future, this will send image + text to LLM.
        """
        if self.latest_image is None or self.latest_text is None:
            return

        self.get_logger().info('Processing multimodal input (image + text)...')

        matched_command = self.match_command(self.latest_text)

        if matched_command:
            self.publish_command(matched_command)
        else:
            self.get_logger().warning(
                f'Could not match multimodal input text: "{self.latest_text}"'
            )

        # Reset for next multimodal input
        self.latest_image = None
        self.latest_text = None


def process_text(text, logger=None) -> JointModel:
    """
    Translates text into a list of 5 joint positions using Gemini.
    Falls back to 'crouch' if interpretation fails.
    """
    if logger is None:
        logger = logging.getLogger()

    # 2. Define the Fallback (Safety)
    fallback_pose = JointModel(joints=NAMED_POSES['crouch'])

    if not text:
        logger.warning("No text command found. Defaulting to crouch.")
        return fallback_pose

    system_context = f"""
        You are the motion controller for a 5-Degree-of-Freedom (5-DOF) robotic arm. 
Your job is to output a specific array of 5 numbers (in radians) that controls the robot's shape.

### **THE ROBOT'S BODY (How the joints work)**
The arm is controlled by an array of 5 floats: `[Base, Shoulder, Elbow, Wrist_Pitch, Wrist_Roll]`.
All angles are in RADIANS. 

1. **Joint 0: THE BASE (Yaw / Rotation)**
   - **0.0** = Facing straight forward.
   - **+1.57** = Rotates the whole arm 90° to the LEFT.
   - **-1.57** = Rotates the whole arm 90° to the RIGHT.
   - *Use this to turn the robot towards an object.*

2. **Joint 1: THE SHOULDER (Pitch / Lift)**
   - **0.0** = Straight up (Vertical).
   - **-1.57** = Lowered forward (Horizontal).
   - **+1.57** = Leaning backward.
   - *Use this to reach out (negative) or stand tall (zero).*

3. **Joint 2: THE ELBOW (Pitch / Reach)**
   - **0.0** = Straight (arm is fully extended like a stick).
   - **-1.57** = Bent 90° downwards (like a crane hook).
   - **+1.57** = Bent 90° upwards.
   - *Use this to adjust the height of the gripper.*

4. **Joint 3: THE WRIST PITCH (Nodding)**
   - **0.0** = Straight (aligned with the forearm).
   - **-1.57** = Tilted down (gripper points at the floor).
   - **+1.57** = Tilted up (gripper points at the ceiling).
   - *Use this to orient the gripper for grabbing.*

5. **Joint 4: THE WRIST ROLL (Spinning)**
   - **0.0** = Neutral (Gripper fingers are horizontal).
   - **+1.57** = Rotated 90° clockwise.
   - *Use this only if you need to twist the object.*

### **PRE-CALCULATED POSES (Reference)**
Use these exact values as a baseline. Modify them slightly if the user asks to "move a little bit."
{json.dumps(NAMED_POSES, indent=2)}


### **YOUR TASK**
1. Analyze the user's natural language command.
2. Determine which joints need to move based on the descriptions above.
3. Return the final `joints` array or a `joints_sequence` (list of arrays) if multiple steps are needed.
"""
    # 3. Construct the Prompt with Strict Schema
    prompt = f"""
        {system_context}
        USER COMMAND: "{text}"
    
        INSTRUCTIONS:
        1. If the command matches a named pose (e.g., "look left"), return those exact values.
        2. Otherwise, generate a joint array that when executed would result in the commanded pose.
        3. If the command warrants multiple poses in a row, (e.g. wave, nod) then return joints_sequence and leave joints as null.

        Try your best to create a pose or poses that satisfy the users conditions. If the input command doesn't make sense then you can just return [0,0,0,0,0]
                
        RESPONSE FORMAT:
        You must return ONLY a raw JSON object with no markdown formatting.
        Example: {{"joints": [0.0, -1.57, 0.0, 0.0, 0.0]}}
        """

    try:
        # 4. Call Gemini
        response = client.models.generate_content(
            model='gemma-3-27b-it',
            contents=types.Part.from_text(text=prompt),
            config=types.GenerateContentConfig(
                temperature=0,
                response_schema=JointModel,
            ),
        )
        joint_model: JointModel = response.parsed
        if joint_model:
            logger.info(f"Parsed joints: {joint_model.joints}")
            if joint_model.joints_sequence:
                log_str = "Parsed sequence:\n"
                for joint in joint_model.joints_sequence:
                    log_str += f"\t{joint}\n"
                logger.info(log_str)

            return joint_model
        else:
            # Fallback: Try to parse the model manually from the raw text
            logger.warning(f"Automatic parsing failed. Attempting manual parse of raw text: {response.text}")

            try:
                # 1. Clean the text: Remove markdown code fences if present
                clean_text = response.text.strip()
                if clean_text.startswith("```"):
                    clean_text = clean_text.replace("```json", "").replace("```", "").strip()

                # 2. Convert string to Dict
                data = json.loads(clean_text)

                # 3. Validate using the Pydantic model explicitly
                manual_model = JointModel(**data)

                logger.info(f"Manual parse successful: {manual_model}")
                return manual_model

            except json.JSONDecodeError as e:
                logger.error(f"JSON Decode Error: {e}. Raw text was: {response.text}")
                raise ValueError("Failed to decode JSON from LLM response")
            except Exception as e:
                logger.error(f"Validation Error during manual parse: {e}")
                raise ValueError("LLM response did not match JointModel schema")

    except Exception as e:
        logger.error(f"LLM processing failed: {e}. Falling back to CROUCH.")
        return fallback_pose


def main(args=None):
    rclpy.init(args=args)
    brain = Brain()
    rclpy.spin(brain)
    brain.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
