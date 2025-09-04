#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist
import serial


class SerialROSNode(Node):
    def __init__(self):
        super().__init__('serial_shoulder_node')

        # Parámetros
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('epsilon', 1e-6)     # umbral para considerar "hay comando"
        self.declare_parameter('ser_timeout', 0.01) # s, lectura no bloqueante

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.epsilon = self.get_parameter('epsilon').get_parameter_value().double_value
        ser_timeout = self.get_parameter('ser_timeout').get_parameter_value().double_value

        # Serial no bloqueante (timeout pequeño)
        self.ser = serial.Serial(port, baud, timeout=ser_timeout)

        # Publishers
        self.pub_angle = self.create_publisher(Float32, 'shoulder/angle', 10)
        self.pub_text  = self.create_publisher(String,  'shoulder/raw_text', 10)  # opcional, debug

        # Subscriber
        self.sub_cmd = self.create_subscription(String, 'shoulder/cmd_vel', self.send_cmd, 10)

        self.get_logger().info("Listo: sub 'shoulder/cmd_vel' → envía 'v' o 'r' y publica 'shoulder/angle'.")

    def _try_publish_line(self, line: str):
        """Intenta publicar ángulo si 'line' es número; si no, publica texto crudo (debug)."""
        if not line:
            return
        try:
            value = float(line.strip())
            self.pub_angle.publish(Float32(data=value))
        except ValueError:
            # No era número; puedes comentar si no quieres publicar texto crudo
            self.pub_text.publish(String(data=line.strip()))

    def send_cmd(self, msg: Twist):
        """TODO EN EL CALLBACK:
        - Si hay comando (|ang.x|>ε): envia 'v <ang.x>\n' una vez.
        - Si no hay comando: envia 'r\n' y publica respuesta si llega.
        - Luego intenta leer 1 línea (no bloqueante) y publicarla (ángulo si es número).
        """
        cmd = String(msg.data)

        try:
            if cmd.startswith("v"): ## Esto debe ser el problema
                # Enviar comando una sola vez
                cmd_l = cmd.strip().split()
                if len(cmd_l) != 2:
                    self.get_logger().warn(f"Comando inválido: {cmd}") ## Revisar que hace
                    return
                try:
                    angx = float(cmd_l[1])
                except ValueError:
                    self.get_logger().warn(f"Comando inválido: {cmd}")
                    return
                if abs(angx) <= self.epsilon:
                    cmd_serial = f"s\n".encode('utf-8')
                else:
                    cmd_serial = f"v {cmd_l[1]}\n".encode('utf-8')
                self.ser.write(cmd_serial) ## Esto factorizar si es que en los demás if sean así
                self.get_logger().debug(f"TX: {cmd_serial.decode().strip()}")
            elif cmd == "s":
                cmd_serial = b"s\n".encode('utf-8')
                self.ser.write(cmd_serial)
                self.get_logger().debug(f"TX: {cmd_serial.decode().strip()}")
            elif cmd == "r":
                # Modo “sin comando”: pedir lectura 'r'
                self.ser.write(b"r\n")
                self.get_logger().debug("TX: r")

            # Intentar leer una sola línea (no bloqueante por timeout corto)
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                self._try_publish_line(line)

        except Exception as e:
            self.get_logger().warn(f"Error serial en send_cmd: {e}")


def main():
    rclpy.init()
    node = SerialROSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()