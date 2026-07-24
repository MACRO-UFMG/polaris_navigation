import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

params_file = "scout_params.yaml"


def generate_launch_description():

    # Encontra o caminho para o pacote 'polaris_control'
    polaris_control_share = get_package_share_directory('polaris_control')
    polaris_planning_share = get_package_share_directory('polaris_planning')
    
    # Define o caminho completo para o arquivo de configuração do RViz
    rviz_config_file = os.path.join(polaris_control_share, 'config', 'demo_rviz.rviz')

    param_controller_file = os.path.join(polaris_control_share, 'config', params_file)
    planner_params_file = os.path.join(polaris_planning_share, 'config', 'path_from_points.yaml')

    # --- Nó do RViz ---
    # <node pkg="rviz2" exec="rviz2" name="rviz" output="screen" args="-d $(find-pkg-share polaris_control)/config/demo_rviz.rviz">
    #     <param name="use_sim_time" value="false"/>
    # </node>
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', rviz_config_file], # 'args' vira 'arguments' (uma lista)
        parameters=[{'use_sim_time': False}] # 'param' vira 'parameters' (lista de dicts)
    )

    # --- Nó do Controlador (Vector Field) ---
    # <node pkg="polaris_control" exec="vector_field_controller" name="controller" output="screen">
    # </node>
    controller_node = Node(
        package='polaris_control',
        executable='vector_field_controller',
        name='controller',
        output='screen',
        parameters=[param_controller_file]
    )

    # --- Nó do Planejador ---
    # <node pkg="polaris_planning" exec="path_from_points" name="planner" output="screen">
    # </node>
    planner_node = Node(
        package='polaris_planning',
        executable='path_from_points',
        name='planner',
        output='screen',
        parameters=[planner_params_file]
    )

    # --- Nós comentados (exemplo) ---
    # <node pkg="polaris_control" exec="robot_simulator.py" name="robot_sim" output="screen">
    # </node>
    # robot_sim_node = Node(
    #     package='polaris_control',
    #     executable='robot_simulator.py',
    #     name='robot_sim',
    #     output='screen'
    # )

    # --- Nó do Publicador de TF Estática (map -> odom) ---
    # <node pkg="tf2_ros"
    #     exec="static_transform_publisher"
    #     name="static_map_to_odom_publisher"
    #     args="0 0 0 0 0 0 map odom" />
    static_tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom_publisher',
        # Note que 'args' no XML é uma string única, 
        # mas 'arguments' no Python é uma lista de strings
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # static_tf_map_to_camera_init = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_map_to_camera_init_publisher',
    #     # Note que 'args' no XML é uma string única, 
    #     # mas 'arguments' no Python é uma lista de strings
    #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_init']
    # )

    #tf from body to livox_frame
    static_tf_body_to_livox_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_body_to_livox_frame_publisher',
        # Note que 'args' no XML é uma string única, 
        # mas 'arguments' no Python é uma lista de strings
        #livox is 37cm above the body (37cm in z axis)
        arguments=['0', '0', '0.32', '0', '0', '0', 'body', 'livox_frame']
    )
    
    # --- Outro TF estático comentado ---
    # <node pkg="tf2_ros"
    #     exec="static_transform_publisher"
    #     name="static_map_to_odom_publisher"
    #     args="0 0 0 0 0 0 base_link fast_lio/base_link" />
    # static_tf_base_link = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_map_to_odom_publisher_2', # Nome precisa ser único
    #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'fast_lio/base_link']
    # )

    # --- Nó 'vector_follower' comentado ---
    # <node pkg="polaris_control" exec="vector_follower_node" name="vector_follower" output="screen">
    # </node>
    # vector_follower_node = Node(
    #     package='polaris_control',
    #     executable='vector_follower_node',
    #     name='vector_follower',
    #     output='screen'
    # )


    # --- Retorna a Descrição do Launch ---
    # Lista de todas as ações (nós, argumentos, etc.) que você quer executar
    return LaunchDescription([
        rviz_node,
        controller_node,
        planner_node,
        static_tf_map_to_odom,
        #static_tf_map_to_camera_init,
        static_tf_body_to_livox_frame,
        # Descomente as linhas abaixo se quiser adicionar os nós comentados
        # robot_sim_node,
        # static_tf_base_link,
        # vector_follower_node,
    ])
