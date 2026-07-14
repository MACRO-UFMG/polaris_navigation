import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

params_file = "espeleo_params.yaml"

def generate_launch_description():

    # ===================================================================
    # Obter caminhos de pacotes e arquivos de configuração
    # ===================================================================
    
    # $(find-pkg-share polaris_control)
    polaris_control_share = get_package_share_directory('polaris_control')
    
    
    # Caminho para o arquivo de parâmetros do detector de obstáculos
    # $(find-pkg-share polaris_control)/config/closest_obstacle_detector_params.yaml
    detector_params_file = os.path.join(
        polaris_control_share, 'config', 'closest_obstacle_detector_params.yaml'
    )

    param_controller_file = os.path.join(polaris_control_share, 'config', params_file)

    # ===================================================================
    # Definições dos Nós
    # ===================================================================

    # --- Nó do Controlador (Vector Field) ---
    # <node pkg="polaris_control" exec="vector_field_controller" ...>
    controller_node = Node(
        package='polaris_control',
        executable='vector_field_controller',
        name='controller',
        output='screen',
        parameters=[param_controller_file]
    )

    # --- Nó do Planejador ---
    # <node pkg="polaris_planning" exec="path_from_points" ...>
    planner_node = Node(
        package='polaris_planning',
        executable='path_from_equation',
        name='planner',
        output='screen'
    )

    # --- PERCEPTION - LaserScan Obstacle Detector ---
    # <node pkg="polaris_control" exec="closest_obstacle_detector" ...>
    #     <param from=".../closest_obstacle_detector_params.yaml"/>
    # </node>
    closest_obstacle_detector_node = Node(
        package='polaris_control',
        executable='closest_obstacle_detector',
        name='closest_obstacle_detector',
        output='screen',
        # É assim que se carrega um arquivo de parâmetros YAML em Python
        parameters=[detector_params_file] 
    )

    # --- Nó do Simulador (Comentado) ---
    # <node pkg="polaris_control" exec="robot_simulator.py" ...>
    # robot_sim_node = Node(
    #     package='polaris_control',
    #     executable='robot_simulator.py',
    #     name='robot_sim',
    #     output='screen'
    # )

    # --- Publicador de TF Estática 1 (scout_mini/base_link -> fast_lio/base_link) ---
    # <node pkg="tf2_ros" exec="static_transform_publisher" ...>
    static_tf_map_to_base_init = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom_publisher',
        # Note que 'args' no XML é uma string única, 
        # mas 'arguments' no Python é uma lista de strings
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_init']
    )

    static_tf_base_init_to_chassis = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_init_to_chassis_publisher',
        # Note que 'args' no XML é uma string única, 
        # mas 'arguments' no Python é uma lista de strings
        arguments=['0', '0', '0', '0', '0', '0', 'base_init', 'chassis_init']
    )

    # static_tf_map_to_camera_init = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_map_to_camera_init_publisher',
    #     # Note que 'args' no XML é uma string única, 
    #     # mas 'arguments' no Python é uma lista de strings
    #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera init']
    # )

    static_tf_map_to_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_world_publisher',
        # Note que 'args' no XML é uma string única, 
        # mas 'arguments' no Python é uma lista de strings
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'world']
    )

    static_tf_map_to_camera_init = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_camera_init_publisher',
        arguments=[
            '0', '0', '0',             # Translação (x, y, z)
            '0.5', '-0.5', '0.5', '-0.5', # Rotação (qx, qy, qz, qw)
            'map',                     # Frame de origem
            'camera_init'              # Frame de destino
        ]
    )

    static_tf_test = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_test',
        arguments=[
            '0', '0', '0',             # Translação (x, y, z)
            '0.5', '-0.5', '0.5', '0.5', # Rotação (qx, qy, qz, qw)
            'body',                     # Frame de origem
            'robo'              # Frame de destino
        ]
    )

    static_robo_to_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_robo_to_velodyne',
        arguments=[
            '0', '0', '0.27',             # Translação (x, y, z)
            '0.0', '0.0', '0.0', '1.0', # Rotação (qx, qy, qz, qw)
            'robo',                     # Frame de origem
            'velodyne'              # Frame de destino
        ]
    )


    # ===================================================================
    # Retorna a Descrição do Launch
    # ===================================================================
    return LaunchDescription([
        controller_node,
        planner_node,
        closest_obstacle_detector_node,
        static_tf_map_to_base_init,
        static_tf_base_init_to_chassis,
        static_tf_map_to_camera_init,
        static_tf_test,
        static_robo_to_velodyne,
        static_tf_map_to_world,

        
        # Descomente as linhas abaixo se quiser adicionar os nós comentados
        # robot_sim_node,
        # static_tf_world_to_map,
        # static_tf_base_to_fastlio,
        # vector_follower_node,
    ])