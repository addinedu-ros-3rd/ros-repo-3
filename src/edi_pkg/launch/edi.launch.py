from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=["ros2", "run", "edi_gui", "gui"], output="screen"
            ),

            ExecuteProcess(
                cmd=["ros2", "run", "edi_db", "db"], output="screen"
            ),

            ExecuteProcess(
                cmd=["ros2", "run", "edi_navigation", "navigation"], output="screen"
            ),

        ]
    )