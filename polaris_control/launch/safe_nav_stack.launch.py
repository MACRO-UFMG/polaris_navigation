import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

params_file = "scout_params.yaml"

def generate_launch_description():

    # ===================================================================
    # Obter caminhos de pacotes e arquivos de configuração
    # ===================================================================
    
    # $(find-pkg-share polaris_control)
    polaris_control_share = get_package_share_directory('polaris_control')
    
    # Caminho para o arquivo RViz
    rviz_config_file = os.path.join(
        polaris_control_share, 'config', 'demo_rviz.rviz'
    )
    
    # Caminho para o arquivo de parâmetros do detector de obstáculos
    # $(find-pkg-share polaris_control)/config/closest_obstacle_detector_params.yaml
    detector_params_file = os.path.join(
        polaris_control_share, 'config', 'closest_obstacle_detector_params.yaml'
    )

    param_controller_file = os.path.join(polaris_control_share, 'config', params_file)

    # ===================================================================
    # Definições dos Nós
    # ===================================================================

    # --- Nó do RViz ---
    # <node pkg="rviz2" exec="rviz2" name="rviz" ...>
    #     <param name="use_sim_time" value="false"/>
    # </node>
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}]
    )

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
        executable='path_from_points',
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
    static_tf_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_scout_to_fastlio', # Nome precisa ser único
        arguments=['0', '0', '0', '0', '0', '0', 'scout_mini/base_link', 'fast_lio/base_link']
    )

    # --- Publicador de TF Estática 2 (scout_mini/base_link -> base_link) ---
    # <node pkg="tf2_ros" exec="static_transform_publisher" ...>
    static_tf_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_scout_to_base', # Nome precisa ser único
        arguments=['0', '0', '0', '0', '0', '0', 'scout_mini/base_link', 'base_link']
    )

    # --- Outros TFs Estáticos (Comentados) ---
    # <node pkg="tf2_ros" ... args="0 0 0 0 0 0 world map" />
    # static_tf_world_to_map = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_tf_world_to_map',
    #     arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
    # )
    
    # <node pkg="tf2_ros" ... args="0 0 0 0 0 0 base_link fast_lio/base_link" />
    # static_tf_base_to_fastlio = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_tf_base_to_fastlio',
    #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'fast_lio/base_link']
    # )

    # --- Nó 'vector_follower' (Comentado) ---
    # <node pkg="polaris_control" exec="vector_follower_node" ...>
    # vector_follower_node = Node(
    #     package='polaris_control',
    #     executable='vector_follower_node',
    #     name='vector_follower',
    #     output='screen'
    # )

    # ===================================================================
    # Retorna a Descrição do Launch
    # ===================================================================
    return LaunchDescription([
        rviz_node,
        controller_node,
        planner_node,
        closest_obstacle_detector_node,
        static_tf_1,
        static_tf_2,
        
        # Descomente as linhas abaixo se quiser adicionar os nós comentados
        # robot_sim_node,
        # static_tf_world_to_map,
        # static_tf_base_to_fastlio,
        # vector_follower_node,
    ])