#!/usr/bin/env python3
from copy import deepcopy
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
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
from ur_msgs.srv import SetIO


class UR3MoveItActionClient(Node):
    def __init__(self):
        super().__init__('ur3_moveit_client')

        # gripper
        self.service_name = '/io_and_status_controller/set_io'
        self.io_client = self.create_client(SetIO, self.service_name)
        while not self.io_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service {self.service_name} non disponible, attente...')



        
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
        print("Connecté")
        
        
        self.velocity_factor = 0.15  # % de la vitesse max du robot

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
                # [CORRECTION] Utiliser Time() vide = "Dernière transfo disponible"
                # C'est beaucoup plus robuste que rclpy.time.Time()
                from rclpy.time import Time
                
                trans = self.tf_buffer.lookup_transform(
                    reference_frame,
                    target_frame,
                    Time()) # <--- Changement ici : on demande la dernière dispo

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
        
        # try:
        #     user_input = input(f">>> Vitesse: {self.velocity_factor*100}%. [ENTRÉE] pour exécuter, 'n' pour annuler : ")
        # except EOFError:
        #     user_input = 'n'
            
        # if user_input.lower() == 'n':
        #     print("Annulation.")
        #     return False
        return True

    def retimer_trajectoire(self, robot_traj):
        
        joint_traj = robot_traj.joint_trajectory
        if not joint_traj.points:
            return robot_traj

        # Vitesse max approximative d'un joint UR (rad/s) utilisée comme référence
        MAX_JOINT_VEL = 3.0 
        target_vel = MAX_JOINT_VEL * self.velocity_factor

        # Le premier point est à t=0
        joint_traj.points[0].time_from_start = Duration(seconds=0).to_msg()
        
        current_time = 0.0

        for i in range(1, len(joint_traj.points)):
            p_prev = joint_traj.points[i-1]
            p_curr = joint_traj.points[i]

            # Calculer la plus grande distance parcourue par un joint entre ces deux points
            # On prend le max pour que le joint le plus lent dicte le temps (synchro)
            diffs = [abs(q2 - q1) for q1, q2 in zip(p_prev.positions, p_curr.positions)]
            max_diff = max(diffs)

            # Temps = Distance / Vitesse
            # On ajoute un petit epsilon pour éviter la division par zéro
            dt = max_diff / (target_vel + 1e-6)
            
            # Si dt est trop petit (points très proches), on force un minimum
            if dt < 0.01: 
                dt = 0.01

            current_time += dt
            
            # Conversion float -> ROS Duration Msg
            sec = int(current_time)
            nanosec = int((current_time - sec) * 1e9)
            p_curr.time_from_start.sec = sec
            p_curr.time_from_start.nanosec = nanosec
            
            # On nettoie les vélocités/accélérations calculées par le solveur IK
            # pour laisser le contrôleur du robot gérer l'interpolation propre
            p_curr.velocities = []
            p_curr.accelerations = []

        robot_traj.joint_trajectory = joint_traj
        return robot_traj

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

        if not self.visualize_and_confirm(trajectory_to_exec):
            self.movement_done = True
            return

        print("Exécution...")
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = trajectory_to_exec
        self._send_exec_future = self._execute_action_client.send_goal_async(goal_msg)
        self._send_exec_future.add_done_callback(self.goal_execute_response_callback)
            

    def goal_execute_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("Exécution rejetée.")
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
        # (Fonction inchangée)
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
    
    def create_point(self,start_point):
        #start_point : geometry_msgs/Pose
        current_point=deepcopy(start_point)
        q = [start_point.orientation.x,start_point.orientation.y,start_point.orientation.z,start_point.orientation.w]
        euler=R.from_quat(q).as_euler('xyz', degrees=True)
        final_points=[]
        for i in range(4):
            current_point.position.z-=0.02
            current_point.position.x+=0.05
            euler[1]+=-15
            q=R.from_euler('xyz', euler, degrees=True).as_quat()
            current_point.orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            final_points.append(deepcopy(current_point))
        return final_points
    
    def send_command(self,point):
        pose_point=Pose(
            position=Point(x=point[0], y=point[1], z=point[2]), 
            orientation=Quaternion(x=point[3], y=point[4], z=point[5], w=point[6])
        )
        trajectoire = [self.get_current_pose(),pose_point]
        traj_interpolee = self.interpoler_poses(trajectoire, 20)
        self.send_cartesian_path(traj_interpolee)
        self.wait_for_completion()


    def _set_io_state(self, pin, state):
        """
        Méthode générique pour appeler le service SetIO.
        fun=1 (Digital Output), pin=ton pin, state=état désiré
        """
        req = SetIO.Request()
        req.fun = 1  # 1 correspond généralement aux Digital Outputs sur l'UR
        req.pin = pin
        req.state = state

        # Appel asynchrone
        future = self.io_client.call_async(req)
        
        # Dans un script séquentiel simple, on peut bloquer jusqu'à la réponse
        # Note : Dans une architecture complexe, évite de bloquer le thread principal ainsi.
        rclpy.spin_until_future_complete(self, future)
        
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'IO Set avec succès : Pin {pin} -> {state}')
            else:
                self.get_logger().warn(f'Echec de la commande IO sur le Pin {pin}')
        except Exception as e:
            self.get_logger().error(f'Appel de service échoué : {e}')

    def open_gripper(self):
        """Ouvre le gripper (Logique basée sur ton code ROS 1 : Pin 1 à 1.0)"""
        self.get_logger().info("Ouverture du gripper...")
        # Ton code ROS1 envoyait (1, 1, 1) pour ouvrir
        self._set_io_state(pin=1, state=0.0) 

    def close_gripper(self):
        """Ferme le gripper (Logique basée sur ton code ROS 1 : Pin 1 à 0.0)"""
        self.get_logger().info("Fermeture du gripper...")
        # Ton code ROS1 envoyait (1, 1, 0) pour fermer
        self._set_io_state(pin=1, state=1.0)





    

def main(args=None):
    rclpy.init(args=args)
    ur3_client = UR3MoveItActionClient()
    pose_actuelle = None
    while pose_actuelle is None or ur3_client._latest_joint_state is None:
        rclpy.spin_once(ur3_client, timeout_sec=0.1)
        pose_actuelle = ur3_client.get_current_pose()
    print("Synchronisation joint_states...")
    while ur3_client._latest_joint_state is None:
        rclpy.spin_once(ur3_client, timeout_sec=0.1)


    try:
        print("\n--- Go initial pose ---")
        ur3_client.open_gripper()
        
        point1=[-0.18, -0.05, 0.59, -0.089, -0.790, -0.037, 0.606]
        ur3_client.send_command(point1)

        point_bas=[-0.2995, -0.0257, 0.5469, 0.048, 0.941, 0.060, -0.331]
        ur3_client.send_command(point_bas)

        ur3_client.close_gripper()




        # input("\enter to continue")
        time.sleep(1)

        point3=[-0.1616, -0.0565, 0.3140, 0.085, 0.987, 0.052, -0.126]
        ur3_client.send_command(point3)

        # point4=[-0.0809, 0.2720, 0.3698, 0.756, 0.548, 0.242, -0.264]
        # point4=[-0.1325, 0.0639, 0.3333, 0.370, 0.900, 0.127, -0.192]

        # ur3_client.send_command(point4)

        ur3_client.open_gripper()










    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Erreur critique : {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()