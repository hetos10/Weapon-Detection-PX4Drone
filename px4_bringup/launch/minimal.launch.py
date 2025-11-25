#!/usr/bin/env python3

"""
PX4 Minimal Launch File
Launches PX4 SITL and MicroXRCE Agent
"""

from launch import LaunchDescription 
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    
    # Define PX4 directory path
    px4_dir = os.path.expanduser('~/PX4-Autopilot')
    qgc_path = os.path.expanduser('~/Downloads/QGroundControl-x86_64.AppImage')
    launch_dir = os.path.expanduser('~/drone_ws/src/ROS2-PX4_Drone_Teleoperation_Using_Joystick/px4_bringup/launch')
    
    return LaunchDescription([
        # 1. Launch PX4 SITL with Gazebo in a new terminal
        ExecuteProcess(
            cmd=[
                'gnome-terminal',
                '--title=PX4 SITL',
                '--',
                'bash', '-c',
                f'cd {px4_dir} && make px4_sitl gz_x500_mono_cam; exec bash'
            ],
            output='screen',
            name='px4_sitl'
        ),
        
        # 2. Launch MicroXRCE Agent after 3 seconds
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'gnome-terminal',
                        '--title=MicroXRCE Agent',
                        '--',
                        'bash', '-c',
                        'MicroXRCEAgent udp4 -p 8888; exec bash'
                    ],
                    output='screen',
                    name='microxrce_agent'
                )
            ]
        ),

       
    
    ])