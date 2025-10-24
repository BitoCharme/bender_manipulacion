from .publish_target_pose import TargetPosePublisher
from .move_grippers import GripperActionCommander
import rclpy
import numpy as np

def euler_to_quaternion(yaw, pitch, roll):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

PUBLISHER: TargetPosePublisher | None = None
GRIPPER: GripperActionCommander | None = None


def move_arm(side, sleep_time, x, y, z, yaw, pitch, roll):
    global PUBLISHER
    qx, qy, qz, qw = euler_to_quaternion(yaw, pitch, roll)
    side_l = str(side).strip().lower()
    PUBLISHER.publish_target_pose(side_l, sleep_time, x, y, z, qx, qy, qz, qw)
    return

def move_gripper(side, action):
    global GRIPPER
    GRIPPER.send_goal(side, action)
    return

def pick(side, x, y, z, yaw=0, pitch=-1.57079632679, roll=0):
    if PUBLISHER:
        PUBLISHER.get_logger().info(
            f"Picking object at position: x={x}, y={y}, z={z} with orientation (yaw={yaw}, pitch={pitch}, roll={roll})"
        )
    move_arm(side, 4, x - 0.15, y, z + 0.15, yaw, pitch, roll)  # Se mueve 15 centìmetros arriba del objeto, 15 centímetros atrás.
    move_arm(side, 4, x - 0.15, y, z, yaw, pitch, roll)          # Se mueve al objeto
    move_gripper(side, "close")              # Cierra el gripper para recoger
    move_arm(side, 4, x - 0.15, y, z + 0.15, yaw, pitch, roll)  # Se mueve hacia arriba
    return

def place(side, x, y, z, yaw=0, pitch=-1.57079632679, roll=0):
    if PUBLISHER:
        PUBLISHER.get_logger().info(
            f"Placing object at position: x={x}, y={y}, z={z} with orientation (yaw={yaw}, pitch={pitch}, roll={roll})"
        )
    move_arm(side, 4, x - 0.15, y, z + 0.15, yaw, pitch, roll)  # Se mueve arriba de la posición de colocación
    move_arm(side, 4, x - 0.15, y, z, yaw, pitch, roll)        # Se mueve a la posición de colocación
    move_gripper(side, "open")              # Se abre el gripper para soltar
    move_arm(side, 4, x - 0.15, y, z + 0.15, yaw, pitch, roll)  # Se mueve hacia arriba
    return

def return_home(side):
    if side == "right":
        move_arm(side, 4, 0, -0.3, 0.5, 0, 0, 0)  # Posición inicial del brazo
    else:
        move_arm(side, 4, 0, 0.3, 0.5, 0, 0, 0)  # Posición inicial del brazo
    if PUBLISHER:
        PUBLISHER.get_logger().info("Returning to home position")
    return
#0 -1.57079632679 0
def pick_and_place(side, pick_x, pick_y, pick_z, place_x, place_y, place_z,
                   place_yaw=0, place_pitch=-1.57079632679, place_roll=0, pick_yaw=0, pick_pitch=-1.57079632679, pick_roll=0):
    return_home(side)
    move_gripper(side, "open")
    i = 0
    while i < 2:
        if side == "right":
            move_arm(side, 5, 0.3, -0.3, 0.9, 0, -1.57079632679, 0)  # Se mueve arriba.
        else:
            move_arm(side, 5, 0.3, 0.3, 0.9, 0, -1.57079632679, 0)  # Se mueve arriba.
        i += 1
    pick(side, pick_x, pick_y, pick_z, pick_yaw, pick_pitch, pick_roll)
    place(side, place_x, place_y, place_z, place_yaw, place_pitch, place_roll)
    move_gripper(side, "close")
    return_home(side)
    return

def main():
    rclpy.init()
    # Inicializar nodos UNA sola vez y reutilizarlos
    global PUBLISHER, GRIPPER
    PUBLISHER = TargetPosePublisher()
    GRIPPER = GripperActionCommander()

    side = input("Escribe el lado del brazo a utilizar (right/left): ")
    input_values = input("Escribe los valores donde está el objeto en x, y, z separados por espacios: ")
    pick_x, pick_y, pick_z = map(float, input_values.split())
    input_values = input("Escribe los valores donde quieres dejar el objeto en x, y, z separados por espacios: ")
    place_x, place_y, place_z = map(float, input_values.split())
    # Ejemplo de uso de la función pick_and_place
    try:
        pick_and_place(
            side=side,
            pick_x=pick_x, pick_y=pick_y, pick_z=pick_z,
            place_x=place_x, place_y=place_y, place_z=place_z
        )
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
