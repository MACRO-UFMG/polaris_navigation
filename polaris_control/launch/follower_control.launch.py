import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    """Evaluate launch args in context so float gains are typed correctly."""
    package_name = 'polaris_control'

    params_file = LaunchConfiguration('params_file').perform(context)
    kp_pos = float(LaunchConfiguration('kp_pos').perform(context))
    kp_orient = float(LaunchConfiguration('kp_orient').perform(context))

    pkg_share = FindPackageShare(package_name).perform(context)
    param_config_file = os.path.join(pkg_share, 'config', params_file)

    # YAML file sets all defaults; the dict overrides only the two gains so
    # docker-compose / CI can tune them without touching the params file.
    feedback_node = Node(
        package=package_name,
        executable='follower_control.py',
        name='follower_control',
        output='screen',
        parameters=[
            param_config_file,
            {'kp_pos': kp_pos, 'kp_orient': kp_orient},
        ],
    )

    return [feedback_node]


def generate_launch_description():

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value='pioneer_params.yaml',
        description='Controller params YAML filename under polaris_control/config/',
    )

    # Proportional gain for linear velocity in CONTROL_POSITION (feedback-linearization)
    declare_kp_pos = DeclareLaunchArgument(
        'kp_pos',
        default_value='1.0',
        description='Proportional gain for linear velocity in CONTROL_POSITION state',
    )

    # Proportional gain for angular velocity in ALIGN_YAW (P controller)
    declare_kp_orient = DeclareLaunchArgument(
        'kp_orient',
        default_value='1.0',
        description='Proportional gain for angular velocity in ALIGN_YAW state',
    )

    return LaunchDescription([
        declare_params_file,
        declare_kp_pos,
        declare_kp_orient,
        OpaqueFunction(function=_launch_setup),
    ])
