#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file to start the Alfred system nodes:
    - ear: Listens for audio and converts to text
    - brain: Processes text and images through LLM
    - butler: Executes commands from the brain
    - MoveIt: Motion planning framework
    - RViz2: Visualization tool
    
    Note: To prevent DDS conflicts with already-running rviz2/moveit, ensure:
    1. All processes use the same ROS_DOMAIN_ID (set via environment variable)
    2. All processes use the same RMW_IMPLEMENTATION (set via environment variable)
    
    Example: Before launching, set:
        export ROS_DOMAIN_ID=0
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp  # or rmw_cyclonedx_cpp
    """
    
    # Voice model node - converts audio to text
    ear_node = Node(
        package='alfred',
        executable='ear',
        name='ear',
        output='screen',
        parameters=[{
            'whisper_model': 'base'  # Options: 'tiny', 'base', 'small', 'medium', 'large'
        }]
    )
    
    # Brain node - processes multimodal input (text + images) through LLM
    brain_node = Node(
        package='alfred',
        executable='brain',
        name='brain',
        output='screen'
    )
    
    # body node - executes commands from the brain
    body_node = Node(
        package='alfred',
        executable='body',
        name='body',
        output='screen'
    )
    
    # Add a small delay before starting nodes to let DDS stabilize
    # This helps prevent type registration conflicts with already-running nodes
    # The delay gives the DDS middleware time to properly handle type registration
    delayed_nodes = TimerAction(
        period=2.0,  # 2 second delay to let DDS stabilize
        actions=[
            ear_node,
            brain_node,
            body_node,
        ]
    )
    
    return LaunchDescription([
        delayed_nodes,
    ])

