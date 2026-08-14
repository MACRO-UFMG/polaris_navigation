import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_PARAMS_FILE = "espeleo_params.yaml"


def _rpy_to_quaternion(roll, pitch, yaw):
    """Convert RPY (radians) to quaternion (x, y, z, w)."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def _resolve_params_file(params_file_arg, package_share, source_config_dirs=None):
    """Accept absolute path or basename under package config/ (install or source)."""
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


def _resolve_planning_config(filename, package_share, source_config_dirs=None):
    search_dirs = [os.path.join(package_share, 'config')]
    if source_config_dirs:
        search_dirs.extend(source_config_dirs)
    for directory in search_dirs:
        candidate = os.path.join(directory, filename)
        if os.path.isfile(candidate):
            return candidate
    return os.path.join(package_share, 'config', filename)


def _launch_setup(context, *args, **kwargs):
    use_rviz = LaunchConfiguration('use_rviz')
    publish_map_to_camera_init = LaunchConfiguration('publish_map_to_camera_init')
    publish_odom_to_camera_init = LaunchConfiguration('publish_odom_to_camera_init')
    publish_map_to_chassis_init = LaunchConfiguration('publish_map_to_chassis_init')
    publish_world_to_map = LaunchConfiguration('publish_world_to_map')

    params_file_arg = LaunchConfiguration('params_file').perform(context)
    planner_type = LaunchConfiguration('planner_type').perform(context).strip().lower()
    path_number = int(LaunchConfiguration('path_number').perform(context))

    odom_cam_qx = LaunchConfiguration('odom_cam_qx').perform(context)
    odom_cam_qy = LaunchConfiguration('odom_cam_qy').perform(context)
    odom_cam_qz = LaunchConfiguration('odom_cam_qz').perform(context)
    odom_cam_qw = LaunchConfiguration('odom_cam_qw').perform(context)

    livox_tf_x = LaunchConfiguration('livox_tf_x').perform(context)
    livox_tf_y = LaunchConfiguration('livox_tf_y').perform(context)
    livox_tf_z = LaunchConfiguration('livox_tf_z').perform(context)
    livox_tf_yaw = LaunchConfiguration('livox_tf_yaw').perform(context)
    livox_tf_pitch = LaunchConfiguration('livox_tf_pitch').perform(context)
    livox_tf_roll = LaunchConfiguration('livox_tf_roll').perform(context)

    world_yaw = float(LaunchConfiguration('world_to_map_yaw_radians').perform(context))
    world_pitch = float(LaunchConfiguration('world_to_map_pitch_radians').perform(context))
    world_roll = float(LaunchConfiguration('world_to_map_roll_radians').perform(context))
    world_qx, world_qy, world_qz, world_qw = _rpy_to_quaternion(
        world_roll, world_pitch, world_yaw
    )

    polaris_control_share = get_package_share_directory('polaris_control')
    polaris_planning_share = get_package_share_directory('polaris_planning')
    # Prefer install share; fall back to workspace source before rebuild.
    control_source_config = [
        '/workspace/src/polaris_navigation/polaris_control/config',
    ]
    planning_source_config = [
        '/workspace/src/polaris_navigation/polaris_planning/config',
    ]

    rviz_config_file = os.path.join(
        polaris_control_share, 'config', 'demo_rviz.rviz'
    )
    detector_params_file = os.path.join(
        polaris_control_share, 'config', 'closest_obstacle_detector_params.yaml'
    )
    param_controller_file = _resolve_params_file(
        params_file_arg, polaris_control_share, control_source_config
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(use_rviz),
    )

    controller_node = Node(
        package='polaris_control',
        executable='vector_field_controller',
        name='controller',
        output='screen',
        parameters=[param_controller_file],
    )

    if planner_type == 'equation':
        planner_params_file = _resolve_planning_config(
            'path_from_equation_lemniscate.yaml',
            polaris_planning_share,
            planning_source_config,
        )
        planner_node = Node(
            package='polaris_planning',
            executable='path_from_equation',
            name='planner',
            output='screen',
            parameters=[planner_params_file, {'path_number': path_number}],
        )
    else:
        planner_params_file = _resolve_planning_config(
            'path_from_points.yaml',
            polaris_planning_share,
            planning_source_config,
        )
        planner_node = Node(
            package='polaris_planning',
            executable='path_from_points',
            name='planner',
            output='screen',
            parameters=[planner_params_file],
        )

    closest_obstacle_detector_node = Node(
        package='polaris_control',
        executable='closest_obstacle_detector',
        name='closest_obstacle_detector',
        output='screen',
        parameters=[detector_params_file],
    )

    # Mode B: static map -> camera_init (no AMCL).
    static_tf_map_to_camera_init = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_camera_init_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_init'],
        condition=IfCondition(publish_map_to_camera_init),
    )

    # Mode A: static odom -> camera_init (axis alignment for AMCL).
    # Defaults match former dev map->camera_init quaternion.
    static_tf_odom_to_camera_init = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_odom_to_camera_init_publisher',
        arguments=[
            '0', '0', '0',
            odom_cam_qx, odom_cam_qy, odom_cam_qz, odom_cam_qw,
            'odom', 'camera_init',
        ],
        condition=IfCondition(publish_odom_to_camera_init),
    )

    # Link wheel-odom island under map (shared origin with mocap / LIO).
    static_tf_map_to_chassis_init = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_chassis_init_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'chassis_init'],
        condition=IfCondition(publish_map_to_chassis_init),
    )

    # Optional RViz-only world -> map (does not affect localization).
    static_tf_world_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_world_to_map_publisher',
        arguments=[
            '0', '0', '0',
            str(world_qx), str(world_qy), str(world_qz), str(world_qw),
            'world', 'map',
        ],
        condition=IfCondition(publish_world_to_map),
    )

    static_tf_body_to_livox_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_body_to_livox_frame_publisher',
        arguments=[
            livox_tf_x, livox_tf_y, livox_tf_z,
            livox_tf_yaw, livox_tf_pitch, livox_tf_roll,
            'body', 'livox_frame',
        ],
    )

    return [
        rviz_node,
        controller_node,
        planner_node,
        closest_obstacle_detector_node,
        static_tf_map_to_camera_init,
        static_tf_odom_to_camera_init,
        static_tf_map_to_chassis_init,
        static_tf_world_to_map,
        static_tf_body_to_livox_frame,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=DEFAULT_PARAMS_FILE,
            description='Polaris params YAML basename under polaris_control/config, or absolute path.',
        ),
        DeclareLaunchArgument(
            'planner_type',
            default_value='points',
            description='Planner executable: points (path_from_points) or equation (path_from_equation).',
        ),
        DeclareLaunchArgument(
            'path_number',
            default_value='5',
            description='Equation planner curve number (5 = lemniscate). Ignored for points.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch RViz with the Polaris demo config.',
        ),
        DeclareLaunchArgument(
            'publish_map_to_camera_init',
            default_value='true',
            description='Publish static map -> camera_init (Mode B / no AMCL).',
        ),
        DeclareLaunchArgument(
            'publish_odom_to_camera_init',
            default_value='false',
            description='Publish static odom -> camera_init (Mode A / AMCL axis align).',
        ),
        DeclareLaunchArgument(
            'publish_map_to_chassis_init',
            default_value='false',
            description='Publish static map -> chassis_init (link wheel odom under map).',
        ),
        DeclareLaunchArgument(
            'odom_cam_qx',
            default_value='0.5',
            description='Quaternion x for odom -> camera_init.',
        ),
        DeclareLaunchArgument(
            'odom_cam_qy',
            default_value='-0.5',
            description='Quaternion y for odom -> camera_init.',
        ),
        DeclareLaunchArgument(
            'odom_cam_qz',
            default_value='0.5',
            description='Quaternion z for odom -> camera_init.',
        ),
        DeclareLaunchArgument(
            'odom_cam_qw',
            default_value='-0.5',
            description='Quaternion w for odom -> camera_init.',
        ),
        DeclareLaunchArgument(
            'publish_world_to_map',
            default_value='false',
            description='Publish optional static world -> map (RViz only).',
        ),
        DeclareLaunchArgument(
            'world_to_map_yaw_radians',
            default_value='0.0',
            description='Yaw (rad) for world -> map.',
        ),
        DeclareLaunchArgument(
            'world_to_map_pitch_radians',
            default_value='0.0',
            description='Pitch (rad) for world -> map.',
        ),
        DeclareLaunchArgument(
            'world_to_map_roll_radians',
            default_value='0.0',
            description='Roll (rad) for world -> map.',
        ),
        DeclareLaunchArgument(
            'livox_tf_x',
            default_value='-0.011',
            description='Translation x (m) for body -> livox_frame (MID-360 factory IMU->LiDAR).',
        ),
        DeclareLaunchArgument(
            'livox_tf_y',
            default_value='-0.02329',
            description='Translation y (m) for body -> livox_frame.',
        ),
        DeclareLaunchArgument(
            'livox_tf_z',
            default_value='0.04412',
            description='Translation z (m) for body -> livox_frame.',
        ),
        DeclareLaunchArgument(
            'livox_tf_yaw',
            default_value='0.0',
            description='Yaw (rad) for body -> livox_frame.',
        ),
        DeclareLaunchArgument(
            'livox_tf_pitch',
            default_value='0.0',
            description='Pitch (rad) for body -> livox_frame.',
        ),
        DeclareLaunchArgument(
            'livox_tf_roll',
            default_value='0.0',
            description='Roll (rad) for body -> livox_frame.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
