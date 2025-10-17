#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseStamped

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint, WorkspaceParameters
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Point

from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose


class MoveRightArm(Node):
    def __init__(self):
        super().__init__("move_right_arm_node")

        # Cliente de acción
        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        self._executing = False

        # Suscripción al tópico /target_pose
        self.subscription = self.create_subscription(
            PoseStamped,
            '/target_pose_right',
            self.pose_callback,
            10
        )

        self.set_parameters([rclpy.parameter.Parameter(
            "use_sim_time",
            rclpy.Parameter.Type.BOOL,
            True
        )])

        # Publicador de markers
        self.marker_pub = self.create_publisher(Marker, '/debug_markers_right', 10)

        # TF2 buffer y listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_pose = None

        self.get_logger().info("Nodo listo, publica geometry_msgs/PoseStamped en /target_pose_right")

    def publish_marker(self, pose: Pose, marker_id: int, color=(0.0, 1.0, 0.0)):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "right_arm_debug"
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = pose
        marker.scale.x = 0.1
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

    def pose_callback(self, msg: PoseStamped):

        if self._executing:
            self.get_logger().warn("Ya ejecutando una trayectoria — ignorando nuevo target")
            return
        
        self.get_logger().info(f"Recibida target pose (frame={msg.header.frame_id}) : {msg.pose}")

        # markers (convertir PoseStamped.pose a Pose)
        self.publish_marker(msg.pose, 0, (0.0,1.0,0.0))
        self.publish_marker(msg.pose, 1, (0.0,0.0,1.0))

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server /move_action no disponible")
            return

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "right_arm"
        # IMPORTANTE: rellenar trajectory_start si lo quieres consistente con robot
        # goal_msg.request.start_state.is_diff = True
        goal_msg.request.start_state.is_diff = True  # opcional: prueba con False y con True
        # si añades start_state, usa get_planning_scene/current state (más abajo)

        # Constraints
        constraints = Constraints()
        constraints.name = "l6r_1_goal"

        # Position constraint
        pos = PositionConstraint()
        pos.header.frame_id = msg.header.frame_id   # usa el frame del pose recibido
        pos.header.stamp = self.get_clock().now().to_msg()
        pos.link_name = "l6r_1"  # <-- cambia por el link correcto de tu URDF
        pos.weight = 1.0

        # target point offset: 0,0,0 (usar el origen del link)
        pos.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)

        # define a small box centered at la pose objetivo (pose interpreted in frame_id)
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.05, 0.05, 0.05]

        pos.constraint_region.primitives.append(box)

        # primitive_poses: el pose de la caja en el frame 'frame_id'
        # usa msg.pose directamente (es geometry_msgs/Pose)
        pos.constraint_region.primitive_poses.append(msg.pose)

        constraints.position_constraints.append(pos)

        # Orientation constraint (opcional) — si no la necesitas, QUÍTALA
        orient = OrientationConstraint()
        orient.header.frame_id = msg.header.frame_id
        orient.header.stamp = self.get_clock().now().to_msg()
        orient.link_name = "l6r_1"  # mismo link
        orient.orientation = msg.pose.orientation
        orient.weight = 1.0
        # PON tolerancias razonables si quieres fijar la orientación:
        orient.absolute_x_axis_tolerance = 0.1
        orient.absolute_y_axis_tolerance = 0.1
        orient.absolute_z_axis_tolerance = 0.5

        constraints.orientation_constraints.append(orient)

        goal_msg.request.goal_constraints.append(constraints)

        # Enviar goal
        self._executing = True
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rechazado por MoveGroup")
            self._executing = False
            return
        self.get_logger().info("Goal aceptado, esperando resultado...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info(f"Movimiento completado: {result.error_code}")
        except Exception as e:
            self.get_logger().error(f"Error al obtener resultado: {e}")
        finally:
            # liberar flag para permitir nuevos goals
            self._executing = False


def main():
    rclpy.init()
    node = MoveRightArm()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
