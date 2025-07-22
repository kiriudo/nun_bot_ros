import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class CommandBridge(Node):
    def __init__(self):
        super().__init__('command_bridge')

        # Port série vers l'Arduino (à adapter selon ton cas)
        self.serial_port = serial.Serial('/dev/ttyACM0', 57600, timeout=1)

        # Souscription au topic ROS 2
        self.subscription = self.create_subscription(
            String,
            'cmd_direction',  # Topic ROS 2
            self.listener_callback,
            10
        )
        self.get_logger().info("Bridge actif, prêt à recevoir les commandes (forward, back...)")

        # Dictionnaire de traduction
        self.command_map = {
            'forward': '1',
            'right': '2',
            'backward': '3',
            'left': '4',
            'stop': '5'
        }

    def listener_callback(self, msg):
        command_str = msg.data.strip().lower()

        if command_str in self.command_map:
            arduino_cmd = self.command_map[command_str]
            self.serial_port.write(arduino_cmd.encode())
            self.get_logger().info(f"Commande reçue: {command_str} → envoyée: {arduino_cmd}")
        else:
            self.get_logger().warn(f"Commande inconnue: '{command_str}'")

def main(args=None):
    rclpy.init(args=args)
    bridge = CommandBridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()
