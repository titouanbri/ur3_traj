#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial.transform import Rotation as R

class VectorVisualizer(Node):

    def __init__(self):
        super().__init__('vector_visualizer_node')
        
        self.publisher_ = self.create_publisher(MarkerArray, '/visualisation_vecteurs', 10)
        self.timer = self.create_timer(0.1, self.publish_vectors)
        self.get_logger().info("Nœud de visualisation (Aligné Robot) démarré.")

    def create_vector_marker(self, marker_id, pos_x, pos_y, pos_z, roll_deg, pitch_deg, yaw_deg, length=0.2):
        """
        Crée une flèche alignée avec l'axe Z du robot (Tool0).
        Les angles sont en DEGRÉS (pour correspondre à ur3_moveit_action.py).
        """
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = marker_id
        marker.ns = "points_et_vecteurs"
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # --- 1. Position ---
        marker.pose.position.x = float(pos_x)
        marker.pose.position.y = float(pos_y)
        marker.pose.position.z = float(pos_z)
        
        # --- 2. Orientation (Correction X -> Z) ---
        # A. On crée la rotation cible (celle du robot)
        r_robot = R.from_euler('xyz', [roll_deg, pitch_deg, yaw_deg], degrees=True)
        
        # B. On crée une rotation de correction :
        # La flèche pointe vers X, le robot vers Z.
        # Une rotation de -90° sur Y transforme l'axe X en axe Z.
        r_correction = R.from_euler('y', -90, degrees=True)
        
        # C. On combine : Rotation Robot * Correction
        r_final = r_robot * r_correction
        q = r_final.as_quat()

        marker.pose.orientation.x = float(q[0])
        marker.pose.orientation.y = float(q[1])
        marker.pose.orientation.z = float(q[2])
        marker.pose.orientation.w = float(q[3])
        
        marker.points = []
        
        # --- 3. Échelle ---
        marker.scale.x = float(length) # Longueur de la flèche
        marker.scale.y = 0.015          # Diamètre tige
        marker.scale.z = 0.015        # Diamètre tête
        
        # --- 4. Couleur ---
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        return marker

    def publish_vectors(self):
        marker_array = MarkerArray()
    
        # UTILISATION :
        # Copie simplement les valeurs de "self.target_pose" ou "R.from_euler"
        # depuis ton script ur3_moveit_action.py.
        # Ici : [-90, -90, 0]
        
        marker_1 = self.create_vector_marker(
            marker_id=1,
            pos_x=-0.1, pos_y=0.45, pos_z=0.40,
            roll_deg=-90,   
            pitch_deg=0,  
            yaw_deg=0,      
            length=0.1
        )
        marker_2 = self.create_vector_marker(
            marker_id=2,
            pos_x=-0.1, pos_y=0.40, pos_z=0.35,
            roll_deg=-70,   
            pitch_deg=0,  
            yaw_deg=0,      
            length=0.1
        )

        marker_3 = self.create_vector_marker(
            marker_id=3,
            pos_x=-0.1, pos_y=0.35, pos_z=0.30,
            roll_deg=-45,   
            pitch_deg=0,  
            yaw_deg=0,      
            length=0.1
        )



        marker_array.markers.append(marker_1)
        marker_array.markers.append(marker_2)
        marker_array.markers.append(marker_3)
        self.publisher_.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = VectorVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()