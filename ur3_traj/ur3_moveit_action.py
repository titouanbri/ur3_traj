#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time
import time
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from scipy.interpolate import interp1d

# Messages ROS2 / MoveIt
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import RobotState, DisplayTrajectory
from moveit_msgs.srv import GetCartesianPath
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import copy

class UR3MoveItActionClient(Node):
    def __init__(self):
        super().__init__('ur3_moveit_client')
        
        # Clients d'action et services
        self._move_action_client = ActionClient(self, MoveGroup, 'move_action')
        self._execute_action_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self._compute_cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        
        self._joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self._latest_joint_state = None
        self._display_path_pub = self.create_publisher(DisplayTrajectory, '/move_group/display_planned_path', 10)

        self.movement_done = False
        self.success = False

        print("Attente des serveurs MoveIt...")
        self._move_action_client.wait_for_server()
        self._execute_action_client.wait_for_server()
        self._compute_cartesian_client.wait_for_service()
        print("Connecté !")
        
        # Paramètres de vitesse globale
        self.velocity_factor = 0.2 

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def joint_state_callback(self, msg):
        self._latest_joint_state = msg

    def wait_for_completion(self):
        print("   -> En attente de la fin du processus...")
        while not self.movement_done:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.success
    
    def get_current_pose(self, reference_frame="base_link", target_frame="tool0"):
        try:
            # Utilisation de Time() vide pour obtenir la dernière transformation disponible
            trans = self.tf_buffer.lookup_transform(
                reference_frame,
                target_frame,
                Time()) 

            current_pose = Pose()
            current_pose.position.x = trans.transform.translation.x
            current_pose.position.y = trans.transform.translation.y
            current_pose.position.z = trans.transform.translation.z
            current_pose.orientation = trans.transform.rotation
            return current_pose

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF pas prêt : {e}")
            return None

    def visualize_and_confirm(self, trajectory):
        print("\n>>> TRAJECTOIRE CALCULÉE !")
        display_msg = DisplayTrajectory()
        display_msg.trajectory_start = RobotState()
        if self._latest_joint_state:
            display_msg.trajectory_start.joint_state = self._latest_joint_state
        display_msg.trajectory.append(trajectory)
        self._display_path_pub.publish(display_msg)
        print(">>> Trajectoire publiée sur RViz.")
        
        # Note: L'input bloque le thread, donc _latest_joint_state ne se met plus à jour ici.
        # C'est pourquoi la correction dans _cb_cartesian_computed est CRITIQUE.
        # try:
        #     user_input = input(f">>> Vitesse: {self.velocity_factor*100}%. [ENTRÉE] pour exécuter, 'n' pour annuler : ")
        # except EOFError:
        #     user_input = 'n'
            
        # if user_input.lower() == 'n':
        #     print("Annulation.")
        #     return False
        return True

    # ---------------------------------------------------------
    # [NOUVEAU] Helper pour aligner l'ordre des joints
    # ---------------------------------------------------------
    def align_joint_order(self, current_state, target_joint_names):
        """
        Réordonne les positions actuelles (current_state) pour qu'elles correspondent
        à l'ordre des noms de joints de la trajectoire MoveIt (target_joint_names).
        """
        current_map = dict(zip(current_state.name, current_state.position))
        reordered_positions = []
        
        try:
            for name in target_joint_names:
                reordered_positions.append(current_map[name])
            return reordered_positions
        except KeyError as e:
            self.get_logger().error(f"Joint manquant dans l'état actuel : {e}")
            return None

    # ---------------------------------------------------------
    # RETIMING
    # ---------------------------------------------------------
    def retimer_trajectoire(self, robot_traj):
        joint_traj = robot_traj.joint_trajectory
        if not joint_traj.points:
            return robot_traj

        MAX_JOINT_VEL = 3.0 
        target_vel = MAX_JOINT_VEL * self.velocity_factor

        joint_traj.points[0].time_from_start = Duration(seconds=0).to_msg()
        current_time = 0.0

        for i in range(1, len(joint_traj.points)):
            p_prev = joint_traj.points[i-1]
            p_curr = joint_traj.points[i]

            diffs = [abs(q2 - q1) for q1, q2 in zip(p_prev.positions, p_curr.positions)]
            max_diff = max(diffs)

            dt = max_diff / (target_vel + 1e-6)
            if dt < 0.01: dt = 0.01

            current_time += dt
            
            sec = int(current_time)
            nanosec = int((current_time - sec) * 1e9)
            p_curr.time_from_start.sec = sec
            p_curr.time_from_start.nanosec = nanosec
            
            p_curr.velocities = []
            p_curr.accelerations = []

        robot_traj.joint_trajectory = joint_traj
        return robot_traj

    # ---------------------------------------------------------
    # EXECUTION CARTESIENNE
    # ---------------------------------------------------------
    def send_cartesian_path(self, waypoints):
        self.movement_done = False
        self.success = False
        
        if self._latest_joint_state is None:
            self.get_logger().error("Pas de joint_states !")
            self.movement_done = True
            return

        print(f"\n--- Calcul Trajectoire Cartésienne ({len(waypoints)} pts) ---")
        req = GetCartesianPath.Request()
        req.header.frame_id = "base_link"
        req.group_name = "ur_manipulator"
        req.link_name = "tool0"
        req.waypoints = waypoints
        req.max_step = 0.01
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        req.start_state = RobotState()
        req.start_state.joint_state = self._latest_joint_state
        
        future = self._compute_cartesian_client.call_async(req)
        future.add_done_callback(self._cb_cartesian_computed)

    def _cb_cartesian_computed(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'Service failed: {e}')
            self.movement_done = True
            return

        if response.fraction < 0.90:
            self.get_logger().warn(f"Trajectoire incomplète ({response.fraction*100:.1f}%). Abandon.")
            self.movement_done = True
            return

        trajectory_to_exec = self.retimer_trajectoire(response.solution)

        # Visualisation (Bloquant)
        if not self.visualize_and_confirm(trajectory_to_exec):
            self.movement_done = True
            return

        print("Préparation de l'exécution...")

        # --- [CORRECTION CRITIQUE POUR ERREUR -4] ---
        if self._latest_joint_state:
            # 1. Récupérer les noms des joints attendus par MoveIt
            traj_joint_names = trajectory_to_exec.joint_trajectory.joint_names
            
            # 2. Réordonner les positions actuelles (robot) pour matcher l'ordre MoveIt
            current_positions_sorted = self.align_joint_order(self._latest_joint_state, traj_joint_names)
            
            if current_positions_sorted:
                # 3. Écraser le point de départ avec la position RÉELLE actuelle
                # Cela corrige le décalage temporel causé par le "input()"
                trajectory_to_exec.joint_trajectory.points[0].positions = current_positions_sorted
                
                # 4. S'assurer que vélocité/accel sont à 0 au départ pour éviter les sauts
                trajectory_to_exec.joint_trajectory.points[0].velocities = [0.0] * len(traj_joint_names)
                trajectory_to_exec.joint_trajectory.points[0].accelerations = [0.0] * len(traj_joint_names)
                
                # 5. Reset du temps à 0
                trajectory_to_exec.joint_trajectory.points[0].time_from_start.sec = 0
                trajectory_to_exec.joint_trajectory.points[0].time_from_start.nanosec = 0
            else:
                self.get_logger().error("Impossible d'aligner les joints. Annulation.")
                self.movement_done = True
                return

        # 6. Mise à jour du Header Stamp (Indispensable)
        trajectory_to_exec.joint_trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory_to_exec.joint_trajectory.header.frame_id = "base_link"

        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = trajectory_to_exec
        self._send_exec_future = self._execute_action_client.send_goal_async(goal_msg)
        self._send_exec_future.add_done_callback(self.goal_execute_response_callback)
            

    def goal_execute_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("Exécution rejetée par le serveur.")
            self.movement_done = True
            return
        self._get_exec_result_future = goal_handle.get_result_async()
        self._get_exec_result_future.add_done_callback(self.get_execute_result_callback)

    def get_execute_result_callback(self, future):
        result = future.result().result
        if result.error_code.val == 1:
            print(">>> SUCCÈS MOUVEMENT !")
            self.success = True
        else:
            print(f">>> ÉCHEC MOUVEMENT. Code: {result.error_code.val}")
            self.success = False
        self.movement_done = True

    def interpoler_poses(self, liste_poses, nombre_points_total):
        numeric_poses = []
        for p in liste_poses:
            valeurs = [p.position.x, p.position.y, p.position.z,
                       p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
            numeric_poses.append(valeurs)
        poses = np.array(numeric_poses)
        positions = poses[:, :3]
        quaternions = poses[:, 3:]
        temps_cles = np.arange(len(poses))
        temps_cibles = np.linspace(0, len(poses) - 1, nombre_points_total)
        interpolateur_pos = interp1d(temps_cles, positions, axis=0, kind='linear')
        nouvelles_positions = interpolateur_pos(temps_cibles)
        rotations = R.from_quat(quaternions)
        slerp = Slerp(temps_cles, rotations)
        nouvelles_rotations = slerp(temps_cibles)
        nouveaux_quats = nouvelles_rotations.as_quat()
        resultat_final = []
        for i in range(len(nouvelles_positions)):
            pos = nouvelles_positions[i]
            quat = nouveaux_quats[i]
            p = Pose()
            p.position.x, p.position.y, p.position.z = pos
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat
            resultat_final.append(p)
        return resultat_final


    def create_points_for_traj(self, start_point, start_euler):
        # start_point : geometry_msgs/Pose
        # start_euler : [pitch, yaw, roll] en degrés
        
        # 1. On crée une COPIE INDÉPENDANTE du point de départ pour travailler
        curr_point = copy.deepcopy(start_point)
        # On copie aussi la liste des angles pour ne pas modifier l'originale
        curr_euler = list(start_euler) 

        # Liste finale
        final_points = []
        
        # On ajoute le point de départ (une copie pour être sûr)
        final_points.append(copy.deepcopy(curr_point))
        
        # --- Point Bas ---
        # On crée une copie pour modifier Z sans toucher au reste
        point_bas = copy.deepcopy(curr_point)
        point_bas.position.z -= 0.025
        final_points.append(point_bas)
        
        # On met à jour curr_point pour repartir de la position basse
        curr_point = copy.deepcopy(point_bas)

        # --- Boucle ---
        for i in range(4):
            curr_euler[0] += 20
            q = R.from_euler('xyz', curr_euler, degrees=True).as_quat()

            # Modification relative
            curr_point.position.x += 0.06
            curr_point.position.z -= 0.03
            curr_point.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            
            # CRITIQUE : On ajoute une COPIE (deepcopy) de l'état actuel dans la liste
            final_points.append(copy.deepcopy(curr_point))
            
        return final_points        

def main(args=None):
    rclpy.init(args=args)
    ur3_client = UR3MoveItActionClient()
    
    # Attente de la première pose
    pose_actuelle = None
    print("Attente des données du robot...")
    while pose_actuelle is None or ur3_client._latest_joint_state is None:
        rclpy.spin_once(ur3_client, timeout_sec=0.1)
        pose_actuelle = ur3_client.get_current_pose()
    
    print("Données reçues. Démarrage de la séquence.")

    try:
        # --- Point 1 ---
        print("\n--- Go initial pose ---")
        q = R.from_euler('xyz', [-210, 0, 90], degrees=True).as_quat() 
        target_pose = Pose(
            position=Point(x=0.45, y=0.06, z=0.45), 
            orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        )

        trajectoire_init = [ur3_client.get_current_pose(), target_pose]
        traj_interpolee_init = ur3_client.interpoler_poses(trajectoire_init, 20)
        ur3_client.send_cartesian_path(traj_interpolee_init)
        ur3_client.wait_for_completion()

        # --- Point 2 ---
        # input("\n[ENTRÉE] pour le point suivant (Z-0.025)...")
        # for _ in range(5): rclpy.spin_once(ur3_client, timeout_sec=0.01)

        traj=ur3_client.create_points_for_traj(ur3_client.get_current_pose(), [-210,0,90])
        traj_interpolee=ur3_client.interpoler_poses(traj, 20)
        ur3_client.send_cartesian_path(traj_interpolee)
        ur3_client.wait_for_completion()

        # point_bas = Pose()
        # point_bas.position.x = target_pose.position.x
        # point_bas.position.y = target_pose.position.y
        # point_bas.position.z = target_pose.position.z - 0.025
        # point_bas.orientation = target_pose.orientation
        
        # traj_bas=[ur3_client.get_current_pose(), point_bas]
        # traj_interpolee_bas=ur3_client.interpoler_poses(traj_bas, 20)
        # ur3_client.send_cartesian_path(traj_interpolee_bas)
        # ur3_client.wait_for_completion()

        # # --- Point 3 & 4 ---
        # input("\n[ENTRÉE] pour la suite de la trajectoire...")
        # for _ in range(5): rclpy.spin_once(ur3_client, timeout_sec=0.01)

        # q2=R.from_euler('xyz', [-160, 0, 90], degrees=True).as_quat()
        # point2=Pose(position=Point(x=-0.26, y=0.06, z=0.43), orientation=Quaternion(x=q2[0], y=q2[1], z=q2[2], w=q2[3]))

        # q3=R.from_euler('xyz', [-140, 0, 90], degrees=True).as_quat()
        # point3=Pose(position=Point(x=-0.20, y=0.06, z=0.40), orientation=Quaternion(x=q3[0], y=q3[1], z=q3[2], w=q3[3]))

        # # Note: on part du point actuel réel pour éviter les sauts
        # trajectoire = [ur3_client.get_current_pose(), point2, point3]
        # traj_interpolee = ur3_client.interpoler_poses(trajectoire, 20)
        # ur3_client.send_cartesian_path(traj_interpolee)
        # ur3_client.wait_for_completion()


    except KeyboardInterrupt:
        print("Arrêt par l'utilisateur.")
    except Exception as e:
        print(f"Erreur critique : {e}")
    finally:
        ur3_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()