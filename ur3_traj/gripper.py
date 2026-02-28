import rclpy
from rclpy.node import Node
from ur_msgs.srv import SetIO

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper')
        
        self.service_name = '/io_and_status_controller/set_io'
        
        self.io_client = self.create_client(SetIO, self.service_name)
        
        # Attendre que le service soit disponible
        while not self.io_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service {self.service_name} non disponible, attente...')

    def _set_io_state(self, pin, state):
        """
        Méthode générique pour appeler le service SetIO.
        fun=1 (Digital Output), pin=ton pin, state=état désiré
        """
        req = SetIO.Request()
        req.fun = 1  # 1 = Digital Output
        req.pin = pin
        req.state = state

        # Appel asynchrone
        future = self.io_client.call_async(req)
        
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
        self.get_logger().info("Ouverture du gripper...")
        self._set_io_state(pin=1, state=0.0) 

    def close_gripper(self):
        self.get_logger().info("Fermeture du gripper...")
        self._set_io_state(pin=1, state=1.0)

def main(args=None):
    rclpy.init(args=args)
    gripper = GripperController()
    
    # Test simple
    gripper.open_gripper()
    import time
    time.sleep(2)
    gripper.close_gripper()
    
    gripper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()