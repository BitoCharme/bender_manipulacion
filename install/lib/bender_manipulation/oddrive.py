import odrive
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math

class ODriveController:
    def __init__(self):
        print("Buscando ODrive...")
        self.odrv = odrive.find_any()
        print(f"ODrive encontrado: {self.odrv}")
        self.velocity = 0
        self.pos = 0

    def set_velocity(self, velocity):
        self.velocity = velocity

        reduction = 200
        velocity_rad_s = velocity
        velocity_rps = velocity_rad_s / (2 * math.pi)
        motor_rps = velocity_rps * reduction
        print(f"Estableciendo velocidad en {velocity_rps} rps")
        self.odrv.axis0.controller.input_vel = motor_rps

    def get_pos(self):
        return self.pos
    
    def go_pos(self, target_pos, velocity=0.2, tolerance=0.01):
        print(f"Moviendo hacia {target_pos:.3f} rad")
        direction = 1 if target_pos > self.pos else -1
        self.set_velocity(direction * abs(velocity))

        while abs(target_pos - self.get_pos()) > tolerance:
            print(f"Pos actual: {self.pos:.3f} rad")
            time.sleep(0.05)
            self.pos += self.velocity*0.05

        self.set_velocity(0.0)
        print("Posición alcanzada.")

            

class ODriveRosNode(Node):
    def __init__(self):
        super().__init__('odrive_ros_node')

        # Inicializamos el controlador ODrive
        self.odrive_controller = ODriveController()

        # Suscripción al tópico /set_velocity
        self.subscription = self.create_subscription(
            Float64,
            'set_velocity',
            self.listener_callback,
            10)

        self.get_logger().info("Nodo ODrive ROS listo y escuchando /set_velocity")

    def listener_callback(self, msg):
        pos = msg.data
        self.odrive_controller.go_pos(pos)
        

def main(args=None):
    rclpy.init(args=args)
    node = ODriveController()
    while True:
        vel = float(input("vel:"))
        node.set_velocity(vel)
if __name__ == '__main__':
    main()
