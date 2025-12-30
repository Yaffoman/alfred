#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import whisper
import threading

class Ear(Node):
    def __init__(self):
        super().__init__('ear')
        
        # 1. Setup Publisher
        # Publishes the raw text to the brain
        self.publisher_ = self.create_publisher(String, '/voice/text', 10)
        
        # 2. Load Whisper Model (Small or Base is good for Jetson)
        self.get_logger().info("Loading Whisper model... (this may take a moment)")
        self.model = whisper.load_model("base") 
        self.get_logger().info("Whisper model loaded.")

        # 3. Setup microphone and recognizer
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        
        # Adjust for ambient noise once at startup
        with self.microphone as source:
            self.get_logger().info("Adjusting for ambient noise... Please be silent.")
            self.recognizer.adjust_for_ambient_noise(source, duration=2)
            self.get_logger().info("Ear ready. Waiting for 'listen' command...")

        # Lock to prevent multiple simultaneous listens
        self.listening_lock = threading.Lock()
        self.is_listening = False

        # Subscribe to command topic to trigger listening
        self.command_subscription = self.create_subscription(
            String,
            '/ear/command',
            self.command_callback,
            10)

    def command_callback(self, msg):
        """Handle listen commands."""
        command = msg.data.lower().strip()
        
        if command == "listen":
            # Check if already listening
            if self.is_listening:
                self.get_logger().warning("Already listening, ignoring command")
                return
            
            # Start listening in a background thread
            self.get_logger().info("Received 'listen' command. Starting to listen...")
            listen_thread = threading.Thread(target=self.listen_once)
            listen_thread.daemon = True
            listen_thread.start()
        else:
            self.get_logger().debug(f"Received command: '{command}' (ignored, only 'listen' is supported)")

    def listen_once(self):
        """Listen for audio once and transcribe it."""
        with self.listening_lock:
            if self.is_listening:
                return
            self.is_listening = True
        
        try:
            with self.microphone as source:
                self.get_logger().info("Listening... (speak now)")
                # 'listen' blocks until it hears silence
                audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=10)
            
            self.get_logger().info("Processing audio...")
            
            # Convert audio data to format Whisper expects
            wav_data = audio.get_wav_data()
            
            # Transcribe
            # We use a temp file logic or direct buffer. 
            # Simplest reliable way with Whisper python lib is writing temp:
            with open("temp_command.wav", "wb") as f:
                f.write(wav_data)
            
            result = self.model.transcribe("temp_command.wav", fp16=False)
            text = result['text'].strip()

            if text:
                self.get_logger().info(f"Heard: '{text}'")
                msg = String()
                msg.data = text
                self.publisher_.publish(msg)
            else:
                self.get_logger().warning("No text transcribed from audio")

        except sr.WaitTimeoutError:
            self.get_logger().warning("Listening timeout - no audio detected")
        except Exception as e:
            self.get_logger().error(f"Error in listener: {e}")
        finally:
            with self.listening_lock:
                self.is_listening = False
            self.get_logger().info("Ready for next 'listen' command")

def main(args=None):
    rclpy.init(args=args)
    node = Ear()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
