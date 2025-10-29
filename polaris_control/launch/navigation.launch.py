import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Encontra o caminho para o pacote 'polaris_control'
    polaris_control_share = get_package_share_directory('polaris_control')
    
    # Define o caminho completo para o arquivo de configuração do RViz
    rviz_config_file = os.path.join(polaris_control_share, 'config', 'demo_rviz.rviz')

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
        output='screen'
    )

    # --- Nó do Planejador ---
    # <node pkg="polaris_planning" exec="path_from_points" name="planner" output="screen">
    # </node>
    planner_node = Node(
        package='polaris_planning',
        executable='path_from_points',
        name='planner',
        output='screen'
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
        
        # Descomente as linhas abaixo se quiser adicionar os nós comentados
        # robot_sim_node,
        # static_tf_base_link,
        # vector_follower_node,
    ])