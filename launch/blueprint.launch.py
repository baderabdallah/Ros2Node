from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    run_dir_arg = DeclareLaunchArgument('run_dir')

    node = Node(
        package='ros_node_blueprint',
        executable='blueprint_node',
        name='blueprint_node',
        parameters=[{
            'run_dir': LaunchConfiguration('run_dir')
        }]
    )

    return LaunchDescription([
        run_dir_arg,
        node,
    ])
