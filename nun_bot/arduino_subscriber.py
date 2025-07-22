import rclpy
from rclpy.node import Node
from nun_bot.srv import SendCommand
import serial

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_listener')

        # Port série (adapter si besoin)
        self.serial_port = serial.Serial('/dev/ttyACM0', 57600, timeout=1)

        # Déclaration du service
        self.srv = self.create_service(SendCommand, 'send_command', self.handle_command)
        self.get_logger().info("Service 'send_command' prêt à recevoir des commandes")

        # Dictionnaire de mappage
        self.command_map = {
            'forward': '1',
            'right': '2',
            'backward': '3',
            'left': '4',
            'stop': '5',
            'readsensor': '6'
        }

    def handle_command(self, request, response):
        command_str = request.command.strip().lower()

        if command_str not in self.command_map:
            self.get_logger().warn(f"Commande inconnue : {command_str}")
            response.response = "Commande invalide"
            return response

        arduino_cmd = self.command_map[command_str]
        self.serial_port.write(arduino_cmd.encode())
        self.get_logger().info(f"Commande envoyée à l'Arduino : {command_str} → {arduino_cmd}")

        # Si on demande les capteurs, lire la réponse
        if command_str == "readsensor":
            arduino_response = self.serial_port.readline().decode().strip()
            self.get_logger().info(f"Réponse capteurs : {arduino_response}")
            response.response = arduino_response
        else:
            response.response = "1"  # simple ACK

        return response

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
