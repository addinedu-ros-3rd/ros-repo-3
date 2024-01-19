from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=["ros2", "run", "edi_pkg", "gui"], output="screen"
            ),

            ExecuteProcess(
                cmd=["ros2", "run", "edi_pkg", "db"], output="screen"
            ),

            ExecuteProcess(
                cmd=["ros2", "run", "edi_pkg", "navigation"], output="screen"
            ),

            ExecuteProcess(
                cmd=["ros2", "run", "edi_pkg", "avoid"], output="screen"
            ),

            ExecuteProcess(
                cmd=["ros2", "run", "edi_pkg", "detect"], output="screen"
            ),
        ]
    )