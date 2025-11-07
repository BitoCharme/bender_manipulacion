#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from moveit_msgs.msg import DisplayTrajectory
from std_msgs.msg import Float64MultiArray


class LocalPlanPublisher(Node):
    def __init__(self):
        super().__init__('local_plan_publisher')

        # Suscripción al tópico de trayectorias planificadas de MoveIt2
        self.subscriber = self.create_subscription(
            DisplayTrajectory,
            '/display_planned_path',
            self.display_traj_callback,
            10
        )

        # Publicador del nuevo tópico con las configuraciones q
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/goal_pos',  # tópico destino
            10
        )

        self.get_logger().info('Nodo local_plan_publisher inicializado')

    def display_traj_callback(self, msg: DisplayTrajectory):
        """
        Callback que recibe el mensaje DisplayTrajectory desde MoveIt2.
        Extrae los puntos de joint_trajectory y publica cada vector de posiciones
        completado a 8 elementos (las 2 últimas en cero para el gripper).
        """
        if not msg.trajectory:
            self.get_logger().warn('Mensaje DisplayTrajectory vacío')
            return

        joint_traj = msg.trajectory[0].joint_trajectory
        num_points = len(joint_traj.points)
        num_joints = len(joint_traj.joint_names)
        self.get_logger().info(
            f'Recibida trayectoria con {num_points} puntos y {num_joints} articulaciones.'
        )

        # Publicar cada configuración q (completando a 8 valores)
        for i, point in enumerate(joint_traj.points):
            q_values = list(point.positions)

            # Si hay menos de 8 articulaciones, completar con ceros
            if len(q_values) < 8:
                q_values.extend([0.0] * (8 - len(q_values)))

            q_array = Float64MultiArray()
            q_array.data = q_values

            self.publisher.publish(q_array)
            self.get_logger().debug(f'[{i+1}/{num_points}] q = {q_array.data}')

        self.get_logger().info('Trayectoria publicada en /goal_pos')


def main(args=None):
    rclpy.init(args=args)
    node = LocalPlanPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
