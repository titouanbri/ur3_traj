import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Chemins
    ur_sim_pkg_share = get_package_share_directory('ur_simulation_gazebo')
    
    # 2. Simulation (Lance Gazebo + MoveIt + Le "Mauvais" RViz)
    # On enlève l'argument 'launch_rviz' car il n'existe pas et créait peut-être des erreurs silencieuses
    ur_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur_sim_pkg_share, 'launch', 'ur_sim_moveit.launch.py')
        ),
        launch_arguments={
            'ur_type': 'ur3',
        }.items()
    )

    # 3. TON RViz (Le "Bon" RViz)
    # AJOUT CAPITAL : parameters=[{'use_sim_time': True}]
    # Sans ça, RViz ne comprend pas les mouvements du robot venant de Gazebo
    # 4. Tes nœuds
    # Eux aussi doivent être synchronisés avec le temps de simulation !
    scene_object_node = Node(
        package='ur3_traj',
        executable='scene_object',
        name='scene_object',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    visu_vector_node = Node(
        package='ur3_traj',
        executable='visu_3D_point', 
        name='vector_visualizer',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        ur_simulation_launch,
        scene_object_node,
        visu_vector_node
    ])