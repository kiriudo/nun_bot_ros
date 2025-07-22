import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import tty
import termios
import select

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher_ = self.create_publisher(String, 'cmd_direction', 10)
        self.get_logger().info("Utilisez les flèches directionnelles pour contrôler le robot (ou Espace pour STOP). Ctrl+C pour quitter.")

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(3)  # lecture de séquences d'échappement pour les flèches
            else:
                key = ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        return key

    def run(self):
        key_map = {
            '\x1b[A': 'forward',   # Flèche haut
            '\x1b[B': 'backward',  # Flèche bas
            '\x1b[C': 'right',     # Flèche droite
            '\x1b[D': 'left',      # Flèche gauche
            ' ': 'stop'            # Espace = stop
        }

        try:
            while rclpy.ok():
                key = self.get_key()
                if key in key_map:
                    msg = String()
                    msg.data = key_map[key]
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Commande envoyée : {msg.data}")
        except KeyboardInterrupt:
            self.get_logger().info("Arrêt du contrôle clavier")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPublisher()
    node.run()
    node.destroy_node()
    rclpy.shutdown()
