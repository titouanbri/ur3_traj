#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
import copy
import math
from geometry_msgs.msg import Pose, Point, Quaternion

# Imports pour TF (Transformations)
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

# Messages ROS2 / MoveIt
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume, RobotState
from moveit_msgs.srv import GetCartesianPath
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState

class UR3MoveItActionClient(Node):
    def __init__(self):
        super().__init__('pose_robot')
        
        self._move_action_client = ActionClient(self, MoveGroup, 'move_action')
        self._execute_action_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self._compute_cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self._joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self._latest_joint_state = None

        # --- Initialisation de TF pour écouter la pose ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Gestion d'état
        self.movement_done = False
        self.success = False

        # Note: Si vous voulez juste lire la pose sans lancer MoveIt, 
        # vous pouvez commenter les wait_for_server ci-dessous pour gagner du temps au démarrage.
        print("Attente des serveurs MoveIt...")
        self._move_action_client.wait_for_server()
        self._execute_action_client.wait_for_server()
        self._compute_cartesian_client.wait_for_service()
        print("Connecté !")
        
        self.target_pose = Pose(
            position=Point(x=0.3, y=-0.2, z=0.2),
            orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0))

    def joint_state_callback(self, msg):
        self._latest_joint_state = msg

    def get_current_pose(self):
        """Récupère la pose de tool0 par rapport à base_link"""
        try:
            # Time(seconds=0) demande la transformation la plus récente disponible
            # C'est souvent plus robuste que rclpy.time.Time() qui demande l'instant "t" exact
            now = rclpy.time.Time() 
            
            trans = self.tf_buffer.lookup_transform(
                'base_link', 
                'tool0', 
                now) # Vous pouvez aussi utiliser rclpy.time.Time() si vos horloges sont bien synchro
            
            current_pose = Pose()
            current_pose.position.x = trans.transform.translation.x
            current_pose.position.y = trans.transform.translation.y
            current_pose.position.z = trans.transform.translation.z
            current_pose.orientation = trans.transform.rotation
            
            return current_pose
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            # On ne spamme pas les erreurs dans le terminal, on retourne juste None
            return None

    # Les méthodes de mouvement sont conservées dans la classe mais inutilisées dans le main
    def wait_for_completion(self):
        while not self.movement_done:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.success

    def send_joint_goal(self, joint_values):
        pass # (Code masqué pour clarté, identique à l'original)

    def send_cartesian_path(self, waypoints):
        pass # (Code masqué pour clarté, identique à l'original)
    
    def _cb_cartesian_computed(self, future):
        pass 

    def send_pose_goal(self, target_pose, tolerance=0.01):
        pass 

    def goal_response_callback(self, future):
        pass

    def get_result_callback(self, future):
        pass
    
    # ... autres méthodes utilitaires ...

def main(args=None):
    rclpy.init(args=args)
    ur3_client = UR3MoveItActionClient()
    
    print("--- Démarrage du monitoring de la pose (CTRL+C pour quitter) ---")
    
    try:
        while rclpy.ok():
            # 1. IMPORTANT : On fait tourner le noeud pour mettre à jour le buffer TF
            rclpy.spin_once(ur3_client, timeout_sec=0.1)
            
            # 2. Lecture de la pose
            pose = ur3_client.get_current_pose()
            
            if pose:
                # Affichage propre avec 4 décimales
                pos_str = f"X={pose.position.x:.4f}, Y={pose.position.y:.4f}, Z={pose.position.z:.4f}"
                rot_str = f"QX={pose.orientation.x:.3f}, QY={pose.orientation.y:.3f}, QZ={pose.orientation.z:.3f}, QW={pose.orientation.w:.3f}"
                print(f"\r[Pose Actuelle] {pos_str} | {rot_str}", end="")
            else:
                print("\r[Pose Actuelle] En attente des données TF...", end="")
            
            # Petite pause pour ne pas surcharger le CPU
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nArrêt du script.")
    except Exception as e:
        print(f"\nErreur critique : {e}")
    finally:
        ur3_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()