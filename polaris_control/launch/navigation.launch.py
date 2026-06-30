import logging
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_logger = logging.getLogger('navigation.launch')


def _launch_setup(context, *_args, **_kwargs):
    package_name = 'polaris_control'

    params_file = LaunchConfiguration('params_file').perform(context)
    tf_robot_pose = LaunchConfiguration('tf_robot_pose').perform(context)
    tf_reference_frame = LaunchConfiguration('tf_reference_frame').perform(context)

    pkg_share = FindPackageShare(package_name).perform(context)
    param_config_file = os.path.join(pkg_share, 'config', params_file)

    # YAML file sets all defaults; build an optional override dict for frame
    # names so docker-compose / CI can tune them without touching the YAML.
    overrides = {}

    if tf_robot_pose:
        overrides['tf_robot_pose'] = tf_robot_pose
    else:
        _logger.warning(
            '⚠️  tf_robot_pose not set via launch arg — '
            'falling back to value in %s. Pass tf_robot_pose:=<frame> to override.',
            params_file,
        )

    if tf_reference_frame:
        overrides['tf_reference_frame'] = tf_reference_frame
    else:
        _logger.warning(
            '⚠️  tf_reference_frame not set via launch arg — '
            'falling back to value in %s. Pass tf_reference_frame:=<frame> to override.',
            params_file,
        )

    parameters = [param_config_file]
    if overrides:
        parameters.append(overrides)

    controller_node = Node(
        package=package_name,
        executable='vector_field_controller',
        name='controller',
        output='screen',
        parameters=parameters,
    )

    planner_node = Node(
        package='polaris_planning',
        executable='path_from_points',
        name='planner',
        output='screen',
        parameters=parameters,
    )

    static_tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    static_tf_body_to_livox_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_body_to_livox_frame_publisher',
        # livox is 32 cm above the body frame
        arguments=['0', '0', '0.32', '0', '0', '0', 'body', 'livox_frame'],
    )

    return [
        controller_node,
        planner_node,
        static_tf_map_to_odom,
        static_tf_body_to_livox_frame,
    ]


def generate_launch_description():

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value='pioneer_params.yaml',
        description='Controller params YAML filename under polaris_control/config/',
    )

    # TF child frame identifying the robot body (e.g. pioneer, scout_mini).
    # Empty string means: use whatever is set in the params YAML.
    declare_tf_robot_pose = DeclareLaunchArgument(
        'tf_robot_pose',
        default_value='',
        description=(
            'TF child frame for the robot body (e.g. pioneer, scout_mini). '
            'If empty, the value from params_file YAML is used.'
        ),
    )

    # TF parent (world/reference) frame used by the controller and planner.
    # Empty string means: use whatever is set in the params YAML.
    declare_tf_reference_frame = DeclareLaunchArgument(
        'tf_reference_frame',
        default_value='',
        description=(
            'TF parent (world/reference) frame (e.g. sim_world). '
            'If empty, the value from params_file YAML is used.'
        ),
    )

    return LaunchDescription([
        declare_params_file,
        declare_tf_robot_pose,
        declare_tf_reference_frame,
        OpaqueFunction(function=_launch_setup),
    ])
