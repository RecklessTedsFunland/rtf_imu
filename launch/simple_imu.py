import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

world2imu = "--x {x} --y {y} --z {z} ".format(x=0.0, y=0.0, z=0.2)
world2imu += "--yaw {yaw} --pitch {pitch} --roll {roll} ".format(yaw=0*pi/180, pitch=0*pi/180, roll=0*pi/180)
world2imu += "--child-frame-id {child} --frame-id {parent}".format(child="imu", parent="world")
world2imu = world2imu.split(' ')

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rtf_sensors'),
        'config',
        'default.yaml'
    )

    return LaunchDescription([
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            parameters=[
                {"port": 8765},
                {"address": "0.0.0.0"},
                {"tls": False},
                {"topic_whitelist": [".*"]},
                {"send_buffer_limit": 10000000},
                {"use_sim_time": False},
                {"num_threads": 0}
            ]
        ),
        Node(
            package = "tf2_ros",
            executable = "static_transform_publisher",
            arguments = world2imu
        ),
        Node(
            package="rtf_sensors",
            executable="imu_node",
            # namespace='turtlesim2',
            name='imu',
            parameters=[config]
        )
    ])