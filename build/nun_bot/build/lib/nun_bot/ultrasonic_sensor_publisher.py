import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial
import time
import RPi.GPIO as GPIO


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'urm04_distance', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # --- Init UART et GPIO ---
        self.serial_port = '/dev/ttyUSB0'
        self.baudrate = 19200
        self.sensor_id = 0x11  # Adresse capteur URM04
        self.trigger_pin = 17

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trigger_pin, GPIO.OUT)
        GPIO.output(self.trigger_pin, GPIO.LOW)

        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=0.1)
            self.get_logger().info("✅ Initialisation RS485 OK")
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Erreur ouverture port série: {e}")
            exit(1)

    def build_command(self, cmd_type):
        header = [0x55, 0xAA, self.sensor_id, 0x00, cmd_type]
        checksum = sum(header) & 0xFF
        return bytes(header + [checksum])

    def send_command(self, cmd_bytes):
        GPIO.output(self.trigger_pin, GPIO.HIGH)
        time.sleep(0.001)
        self.ser.write(cmd_bytes)
        time.sleep(0.003)
        GPIO.output(self.trigger_pin, GPIO.LOW)

    def read_response(self):
        response = self.ser.read(8)
        return list(response)

    def analyze_response(self, data):
        if len(data) != 8:
            self.get_logger().warn("Réponse incomplète")
            return None

        if sum(data[:7]) & 0xFF != data[7]:
            self.get_logger().warn("Checksum invalide")
            return None

        if data[5] == 0xFF and data[6] == 0xFF:
            self.get_logger().warn("Mesure hors plage")
            return None

        distance_cm = data[5] * 256 + data[6]
        return distance_cm

    def get_sensor_value(self):
        # Étape 1 : envoyer commande de déclenchement
        self.send_command(self.build_command(0x01))
        time.sleep(0.03)  # Attendre 30 ms comme spécifié dans la datasheet

        # Étape 2 : envoyer commande de lecture
        self.send_command(self.build_command(0x02))
        time.sleep(0.01)  # Légère pause pour laisser le capteur répondre

        # Étape 3 : lire et analyser
        response = self.read_response()
        return self.analyze_response(response)

    def timer_callback(self):
        distance = self.get_sensor_value()
        msg = String()

        if distance is not None:
            msg.data = f"{distance}"
        else:
            msg.data = "NaN"  # Pour traitement en aval

        self.publisher_.publish(msg)
        self.get_logger().info(f'📡 Distance publiée: "{msg.data}"')

    def destroy_node(self):
        self.ser.close()
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
