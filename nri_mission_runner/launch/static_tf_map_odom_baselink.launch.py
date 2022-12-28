from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
             package='tf2_ros',
             namespace='',
             executable='static_transform_publisher',
             arguments =["0","0","0","0","0","0","map","odom"],
             name ='map_to_odom'
            ),
        Node(
             package='tf2_ros',
             namespace='',
             executable='static_transform_publisher',
             arguments =["0","0","0","0","0","0","odom","base_link"],
             name ='odom_to_base_link'
            ),
        ]
        )
        
