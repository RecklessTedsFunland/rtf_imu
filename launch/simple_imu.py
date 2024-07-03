import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   config = os.path.join(
      get_package_share_directory('rtf_sensors'),
      'config',
      'default.yaml'
      )

   return LaunchDescription([
      Node(
         package="rtf_sensors",
         executable="",
         namespace='turtlesim2',
         name='sim',
         parameters=[config]
      )
   ])