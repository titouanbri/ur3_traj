import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. Définition des chemins des packages ---
    ur_driver_pkg = get_package_share_directory('ur_robot_driver')
    ur_moveit_pkg = get_package_share_directory('ur_moveit_config')

    # --- 2. Lancement du Driver (Connexion au robot réel) ---
    # ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3 robot_ip:=192.168.1.102 launch_rviz:=false
    ur_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur_driver_pkg, 'launch', 'ur_control.launch.py')
        ),
        launch_arguments={
            'ur_type': 'ur3',
            'robot_ip': '192.168.1.102',
            'launch_rviz': 'false', # On désactive RViz ici car on le lance avec MoveIt après
        }.items()
    )

    # --- 3. Lancement de MoveIt + RViz ---
    # ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3 launch_rviz:=true
    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur_moveit_pkg, 'launch', 'ur_moveit.launch.py')
        ),
        launch_arguments={
            'ur_type': 'ur3',
            'launch_rviz': 'true',
        }.items()
    )

    # --- 4. Lancement de votre script (scene_object) ---
    # ros2 run ur3_traj scene_object

    scene_object_node = Node(
        package='ur3_traj',
        executable='scene_object',
        name='scene_object',
        output='screen'
    )

    # --- ASTUCE PRO ---
    # On ajoute un délai pour le script 'scene_object'.
    # Cela laisse le temps à MoveIt et au Driver de s'initialiser avant que votre code n'essaie de bouger le robot.
    delayed_scene_object = TimerAction(
        period=5.0, # Attendre 10 secondes
        actions=[scene_object_node]
    )

    return LaunchDescription([
        ur_driver_launch,
        ur_moveit_launch,
        delayed_scene_object # On lance la version retardée
    ])