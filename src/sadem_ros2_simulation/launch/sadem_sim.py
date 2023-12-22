from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sadem_ros2_simulation',
            namespace='',
            executable='motors',
            name='motors'
        ),
        Node(
            package='sadem_ros2_simulation',
            namespace='',
            executable='path_to_goal',
            name='path_to_goal'
        ),
        DeclareLaunchArgument('map_number',default_value='1',description='map_number'),
        Node(
            package='sadem_ros2_simulation',
            namespace='',
            executable='a_star',
            name='a_star',
            parameters=[{'map_number': LaunchConfiguration('map_number')}],
        ),
        
])