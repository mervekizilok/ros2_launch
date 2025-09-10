from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            prefix='gnome-terminal -- ',
            output='screen'
        ),
        Node(
            package='my_turtlesim',
            executable='draw_shapes',
            name='draw_shapes',
            prefix='gnome-terminal -- ',
            output='screen'
        ),
        Node(
            package='my_pub_sub',
            executable='random_five_publisher',
            name='random_five_publisher',
            prefix='gnome-terminal -- ',
            output='screen'
        ),
        Node(
            package='my_pub_sub',
            executable='random_five_subscriber',
            name='random_five_subscriber',
            prefix='gnome-terminal -- ',
            output='screen'
        ),
        Node(
            package='my_service',
            executable='server',
            name='add_server',
            prefix='gnome-terminal -- ',
            output='screen'
        ),
        Node(
            package='my_service',
            executable='client',
            name='add_client',
            prefix='gnome-terminal -- ',
            output='screen'
        ),
    ])
