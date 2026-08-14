import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_PARAMS_FILE = "pioneer_params.yaml"


def _resolve_params_file(params_file_arg, package_share, source_config_dirs=None):
    if os.path.isabs(params_file_arg) and os.path.isfile(params_file_arg):
        return params_file_arg
    search_dirs = [os.path.join(package_share, 'config')]
    if source_config_dirs:
        search_dirs.extend(source_config_dirs)
    for directory in search_dirs:
        candidate = os.path.join(directory, params_file_arg)
        if os.path.isfile(candidate):
            return candidate
    return os.path.join(package_share, 'config', DEFAULT_PARAMS_FILE)


def _launch_setup(context, *args, **kwargs):
    params_file_arg = LaunchConfiguration('params_file').perform(context)
    pkg_share = get_package_share_directory('polaris_control')
    source_config = ['/workspace/src/polaris_navigation/polaris_control/config']
    param_config_file = _resolve_params_file(
        params_file_arg, pkg_share, source_config
    )

    feedback_node = Node(
        package='polaris_control',
        executable='feedback_linearization.py',
        name='feedback_linearization',
        output='screen',
        parameters=[param_config_file],
    )
    return [feedback_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=DEFAULT_PARAMS_FILE,
            description='Params YAML basename under polaris_control/config, or absolute path.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
