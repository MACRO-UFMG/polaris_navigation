from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    package_name = 'polaris_control'

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value='pioneer_params.yaml',
        description='Controller params YAML filename under polaris_control/config/',
    )

    param_config_file = PathJoinSubstitution([
        FindPackageShare(package_name),
        'config',
        LaunchConfiguration('params_file'),
    ])

    feedback_node = Node(
        package=package_name,
        executable='follower_control.py',
        name='follower_control',
        output='screen',
        parameters=[param_config_file],
    )

    return LaunchDescription([
        declare_params_file,
        feedback_node,
    ])
