# bridge_camera_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_camera_bridge',
            arguments=[
                '/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
            ],
            output='screen'
        )
    ])
