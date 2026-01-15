#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import csv
import os
from datetime import datetime

class ForceTorqueRecorder(Node):
    def __init__(self):
        super().__init__('ft_recorder_node')

        # --- Paramètres de configuration ---
        self.record_duration = 3.0  # secondes
        self.topic_name = '/force_torque_sensor_broadcaster/wrench'
        
        # Définition du chemin absolu
        self.output_dir = '/home/titouan/ros2_ws/src/ur3_traj/CSV'
        
        # --- MODIFICATION ICI : Ajout de la date et l'heure au nom du fichier ---
        # Format : ur3_ft_data_2023_10_27_14_30_05.csv
        timestamp = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
        self.filename = os.path.join(self.output_dir, f'ur3_ft_data_{timestamp}.csv')
        
        # --- Variables d'état ---
        self.start_time = None
        self.data_buffer = []
        self.is_recording = True

        # --- Subscriber ---
        self.subscription = self.create_subscription(
            WrenchStamped,
            self.topic_name,
            self.listener_callback,
            10)
        
        self.get_logger().info(f"En attente de données sur {self.topic_name}...")
        self.get_logger().info(f"Le fichier sera enregistré sous : {self.filename}")

    def listener_callback(self, msg):
        if not self.is_recording:
            return

        # Initialisation du temps au premier message reçu
        if self.start_time is None:
            self.start_time = self.get_clock().now()
            self.get_logger().info(f"Début de l'enregistrement pour {self.record_duration} secondes...")

        # Calcul du temps écoulé
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9

        # Vérification de la durée
        if elapsed_time > self.record_duration:
            self.stop_recording()
            return

        # Extraction des données
        row = [
            elapsed_time,
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z,
            msg.wrench.torque.x,
            msg.wrench.torque.y,
            msg.wrench.torque.z
        ]
        self.data_buffer.append(row)

    def stop_recording(self):
        self.is_recording = False
        self.get_logger().info("Temps écoulé. Sauvegarde du fichier...")
        self.save_to_csv()
        # On lève SystemExit pour briser la boucle rclpy.spin() proprement
        raise SystemExit

    def save_to_csv(self):
        try:
            # Vérification et création du dossier si nécessaire
            if not os.path.exists(self.output_dir):
                self.get_logger().warn(f"Le dossier {self.output_dir} n'existe pas. Création en cours...")
                os.makedirs(self.output_dir)

            with open(self.filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                header = ['Time (s)', 'Force X', 'Force Y', 'Force Z', 'Torque X', 'Torque Y', 'Torque Z']
                writer.writerow(header)
                writer.writerows(self.data_buffer)
            
            self.get_logger().info(f"Succès ! {len(self.data_buffer)} échantillons enregistrés dans :")
            self.get_logger().info(f"{self.filename}")
            
        except Exception as e:
            self.get_logger().error(f"Erreur lors de l'écriture du fichier : {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ForceTorqueRecorder()
    
    try:
        rclpy.spin(node)
    except SystemExit:
        # Gestion propre de la fin de l'enregistrement
        rclpy.logging.get_logger("Node").info("Fermeture du script.")
    except KeyboardInterrupt:
        pass
    finally:
        # Nettoyage final
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()