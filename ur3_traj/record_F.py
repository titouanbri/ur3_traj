#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState  # ### NOUVEAU JOINT_STATES : Import du type de message
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import csv
import os
from datetime import datetime
import cv2  # ### NOUVEAU : Import OpenCV

class ForceTorqueCamRecorder(Node):
    def __init__(self):
        super().__init__('ft_cam_recorder_node')

        # --- Paramètres de configuration ---
        self.record_duration = 13  # secondes
        self.topic_name = '/force_torque_sensor_broadcaster/wrench'
        self.joint_state_topic = '/joint_states' # ### NOUVEAU JOINT_STATES : Nom du topic
        self.base_frame = 'base_link'   
        self.target_frame = 'tool0'
        
        # Paramètres Vidéo
        self.camera_index = 2  # 0 est généralement la webcam par défaut ou USB. Essayez 2 ou 4 si vous avez une cam intégrée.
        self.fps = 30.0        # Images par seconde souhaitées

        # --- Gestion des fichiers ---
        self.output_dir = '/home/titouan/ros2_ws/src/ur3_traj/CSV'
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            
        timestamp = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
        self.csv_filename = os.path.join(self.output_dir, f'ur3_data_{timestamp}.csv')
        self.video_filename = os.path.join(self.output_dir, f'ur3_video_{timestamp}.mp4')

        # --- Variables d'état ---
        self.start_time = None
        self.data_buffer = []
        self.is_recording = True
        self.latest_joint_velocities = [0.0] * 6 # ### NOUVEAU JOINT_STATES : Init à 0 pour les 6 axes de l'UR3

        # --- Initialisation TF ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Initialisation Caméra (OpenCV) ---
        self.cap = cv2.VideoCapture(self.camera_index)
        
        # Vérification caméra
        if not self.cap.isOpened():
            self.get_logger().error(f"Impossible d'ouvrir la caméra (Index {self.camera_index}) !")
        else:
            width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self.get_logger().info(f"Caméra initialisée : {width}x{height} @ {self.fps} FPS")

            fourcc = cv2.VideoWriter_fourcc(*'mp4v') 
            self.video_writer = cv2.VideoWriter(self.video_filename, fourcc, self.fps, (width, height))

        # --- Subscriber Force/Torque ---
        self.subscription = self.create_subscription(
            WrenchStamped,
            self.topic_name,
            self.listener_callback,
            10)

        # --- Subscriber Joint States ### NOUVEAU JOINT_STATES ---
        self.joint_sub = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            10)
        
        # --- Timer Vidéo ---
        self.timer = self.create_timer(1.0/self.fps, self.video_timer_callback)
        
        self.get_logger().info(f"Prêt. En attente de données FT pour lancer l'enregistrement...")

    def joint_state_callback(self, msg):
        """ ### NOUVEAU JOINT_STATES : Mise à jour continue des vitesses articulaires """
        if not self.is_recording:
            return
        
        # Le driver UR envoie les infos des 6 axes. On s'assure que la liste n'est pas vide.
        if msg.velocity:
            self.latest_joint_velocities = list(msg.velocity)

    def video_timer_callback(self):
        if not self.is_recording:
            return

        if self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                if self.start_time is not None:
                    self.video_writer.write(frame)

    def listener_callback(self, msg):
        if not self.is_recording:
            return

        # 1. Gestion du temps (Déclencheur global)
        if self.start_time is None:
            self.start_time = self.get_clock().now()
            self.get_logger().info(f"DÉBUT ENREGISTREMENT ({self.record_duration}s) - CSV, Vidéo et Joints synchros.")

        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9

        if elapsed_time > self.record_duration:
            self.stop_recording()
            return

        # 2. Récupération TF 
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame, self.target_frame, rclpy.time.Time())
            
            pos_x = t.transform.translation.x
            pos_y = t.transform.translation.y
            pos_z = t.transform.translation.z
            quat_x = t.transform.rotation.x
            quat_y = t.transform.rotation.y
            quat_z = t.transform.rotation.z
            quat_w = t.transform.rotation.w
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        # 3. Buffer Data (Incluant les vitesses articulaires)
        row = [
            elapsed_time,
            msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
            msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z,
            pos_x, pos_y, pos_z,
            quat_x, quat_y, quat_z, quat_w
        ]
        
        # ### NOUVEAU JOINT_STATES : Ajout des 6 vitesses à la fin de la ligne
        row.extend(self.latest_joint_velocities) 
        
        self.data_buffer.append(row)

    def stop_recording(self):
        self.is_recording = False
        self.get_logger().info("Temps écoulé. Finalisation...")
        
        # Sauvegarde CSV
        self.save_to_csv()
        
        # Fermeture Vidéo 
        if self.cap.isOpened():
            self.cap.release()
            self.video_writer.release()
            self.get_logger().info(f"Vidéo sauvegardée : {self.video_filename}")
        
        raise SystemExit

    def save_to_csv(self):
        try:
            with open(self.csv_filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                # ### NOUVEAU JOINT_STATES : Ajout des en-têtes Vj1 à Vj6
                header = ['Time', 'Fx', 'Fy', 'Fz', 'Tx', 'Ty', 'Tz', 
                          'Px', 'Py', 'Pz', 'Qx', 'Qy', 'Qz', 'Qw',
                          'Vj1', 'Vj2', 'Vj3', 'Vj4', 'Vj5', 'Vj6']
                writer.writerow(header)
                writer.writerows(self.data_buffer)
            self.get_logger().info(f"CSV sauvegardé : {self.csv_filename} ({len(self.data_buffer)} lignes)")
        except Exception as e:
            self.get_logger().error(f"Erreur CSV : {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ForceTorqueCamRecorder()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'cap') and node.cap.isOpened():
            node.cap.release()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()