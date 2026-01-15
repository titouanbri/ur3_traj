import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

class SceneCreator(Node):
    def __init__(self):
        super().__init__('scene_creator')
        self.publisher = self.create_publisher(CollisionObject, '/collision_object', 10)
        self.timer = self.create_timer(1.0, self.add_scene_objects)

    def add_scene_objects(self):
        self.get_logger().info('Ajout de la scène : Sol + 3 Murs (Arrière, Droit, Avant)...')

        # 1. Le Sol
        self.publish_box("sol", [2.0, 2.0, 0.01], [0.0, 0.0, -0.005])

        # 2. Mur Arrière (reste inchangé)
        # self.publish_box(
        #     name="mur_arriere",
        #     dimensions=[0.02, 1.0, 1.0], 
        #     position=[-0.61, 0.0, 0.5] 
        # )

        # # 3. Mur Droit (reste inchangé)
        # self.publish_box(
        #     name="mur_droit",
        #     dimensions=[1.0, 0.02, 1.0],
        #     position=[0.0, -0.31, 0.5]
        # )

        # # 4. Troisième mur -> CHANGÉ pour être "Mur Avant"
        # # Situé à +30cm devant le robot (axe X Positif)
        # self.publish_box(
        #     name="mur_avant",
        #     dimensions=[0.02, 1.0, 1.0], # [épaisseur, largeur, hauteur]
        #     position=[0.31, 0.0, 0.5]    # x = +0.30 + 0.01
        # )

        self.timer.cancel()

    def publish_box(self, name, dimensions, position, orientation_w=1.0):
        msg = CollisionObject()
        msg.header.frame_id = "world"
        msg.id = name
        msg.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = dimensions 

        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]
        pose.pose.orientation.w = orientation_w

        msg.primitives.append(primitive)
        msg.primitive_poses.append(pose.pose)

        self.publisher.publish(msg)
        self.get_logger().info(f'Objet "{name}" publié.')

def main(args=None):
    rclpy.init(args=args)
    node = SceneCreator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()