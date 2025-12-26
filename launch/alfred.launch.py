#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launch file to start the Alfred system nodes:
    - ear: Listens for audio and converts to text
    - brain: Processes text and images through LLM
    - butler: Executes commands from the brain
    - MoveIt: Motion planning framework
    - RViz2: Visualization tool
    """
    
    # Get the path to the test_moveit_config package
    test_moveit_config_path = get_package_share_directory('test_moveit_config')
    
    # Include robot state publisher launch (publishes URDF)
    rsp_launch_file = os.path.join(
        test_moveit_config_path,
        'launch',
        'rsp.launch.py'
    )
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rsp_launch_file)
    )
    
    # Include MoveIt and RViz2 launch file
    moveit_rviz_launch_file = os.path.join(
        test_moveit_config_path,
        'launch',
        'moveit_rviz.launch.py'
    )
    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_rviz_launch_file)
    )
    
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
    
    # Butler node - executes commands from the brain
    butler_node = Node(
        package='alfred',
        executable='butler',
        name='alfred_butler',
        output='screen'
    )
    
    return LaunchDescription([
        rsp_launch,  # Robot state publisher (must come before MoveIt/RViz)
        moveit_rviz_launch,
        ear_node,
        brain_node,
        butler_node,
    ])

