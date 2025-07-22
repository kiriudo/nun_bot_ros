import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import tty
import termios

class KeyboardCommandPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_command_publisher')
        self.publisher_ = self.create_publisher(String, 'cmd_direction', 10)
        self.get_logger().info("Contrôle avec les flèches. [ESPACE] = stop, [0] = readsensor, [CTRL+C] pour quitter. test")
        self.run()

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
            if ch == '\x1b':
                ch += sys.stdin.read(2)  # read next two characters
            return ch
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def run(self):
        key_map = {
            '\x1b[A': 'forward',    # flèche haut
            '\x1b[B': 'backward',   # flèche bas
            '\x1b[C': 'right',      # flèche droite
            '\x1b[D': 'left',       # flèche gauche
            ' ': 'stop',            # espace
            '0': 'readsensor'       # touche 0
        }

        try:
            while rclpy.ok():
                key = self.get_key()
                print(f"Touche détectée: {repr(key)}")
                if key in key_map:
                    msg = String()
                    msg.data = key_map[key]
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Touche: {repr(key)} → Commande: {msg.data}")
        except KeyboardInterrupt:
            self.get_logger().info("Arrêt du contrôleur clavier.")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardCommandPublisher()
    rclpy.shutdown()
