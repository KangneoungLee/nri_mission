from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
             package='nri_mission_runner',
             namespace='',
             executable='nri_mission_runner_node',
             name='nri_mission_runner_node'
            )
        ]
        )
        
