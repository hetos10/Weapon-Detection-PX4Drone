#!/usr/bin/env python3
# yolo_detect_launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('image_topic',
                                       default_value='/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image',
                                       description='Input image topic'))

    ld.add_action(DeclareLaunchArgument('output_image_topic',
                                       default_value='/drone1/camera/image_detected',
                                       description='Annotated output image topic'))

    ld.add_action(DeclareLaunchArgument('detections_topic',
                                       default_value='/drone1/camera/detections_json',
                                       description='Detections JSON topic'))

    ld.add_action(DeclareLaunchArgument('model_path',
                                       default_value='/home/het/models/best.pt',
                                       description='Path to custom YOLO .pt model'))

    ld.add_action(DeclareLaunchArgument('conf_threshold',
                                       default_value='0.25',
                                       description='Confidence threshold'))

    ld.add_action(DeclareLaunchArgument('imgsz',
                                       default_value='640',
                                       description='Inference image size'))

    ld.add_action(DeclareLaunchArgument('device',
                                       default_value='cpu',
                                       description="Device for inference: 'cpu' or '0' for GPU"))

    ld.add_action(DeclareLaunchArgument('class_names',
                                       default_value="['Automatic Rifle','Bazooka','Grenade Launcher','Handgun','Knife','Shotgun','SMG','Sniper','Sword']",
                                       description='Fallback class names list'))

    node = Node(
        package='your_yolo_pkg',           # CHANGE: set to your package name
        executable='yolo_ros2_node.py',    # if you used console_scripts entrypoint, use that instead
        name='yolo_ros2_node',
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'output_image_topic': LaunchConfiguration('output_image_topic'),
            'detections_topic': LaunchConfiguration('detections_topic'),
            'model_path': LaunchConfiguration('model_path'),
            'conf_threshold': LaunchConfiguration('conf_threshold'),
            'imgsz': LaunchConfiguration('imgsz'),
            'device': LaunchConfiguration('device'),
            'class_names': LaunchConfiguration('class_names')
        }],
        # optionally remap topics
        remappings=[
            # ('/input/image', LaunchConfiguration('image_topic'))  # not needed; using param-based subscription
        ]
    )

    ld.add_action(node)
    return ld
