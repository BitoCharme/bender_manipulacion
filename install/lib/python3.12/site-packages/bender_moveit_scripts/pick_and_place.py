def move_arm(x, y, z, yaw=0, pitch=0, roll=0):
    #rclpy.init()
    #node = TargetPosePublisher()
    #qx, qy, qz, qw = euler_to_quaternion(yaw, pitch, roll)
    #node.publish_target_pose(x, y, z, qx, qy, qz, qw)
    print(f"Moving arm to position: x={x}, y={y}, z={z} with orientation (yaw={yaw}, pitch={pitch}, roll={roll})")
    return

def open_gripper():
    print(f"Opening gripper")
    return

def close_gripper():
    print(f"Closing gripper")
    return

def pick(x, y, z, yaw=0, pitch=0, roll=0):
    print(f"Picking object at position: x={x}, y={y}, z={z} with orientation (yaw={yaw}, pitch={pitch}, roll={roll})")
    move_arm(x, y, z + 0.1, yaw, pitch, roll)  # Se mueve 10 centìmetros arriba del objeto
    move_arm(x, y, z, yaw, pitch, roll)          # Se mueve al objeto
    close_gripper()                             # Cierra el gripper para recoger
    move_arm(x, y, z + 0.1, yaw, pitch, roll)  # Se mueve hacia arriba
    return

def place(x, y, z, yaw=0, pitch=0, roll=0):
    print(f"Placing object at position: x={x}, y={y}, z={z} with orientation (yaw={yaw}, pitch={pitch}, roll={roll})")
    move_arm(x, y, z + 0.1, yaw, pitch, roll)  # Se mueve arriba de la posición de colocación
    move_arm(x, y, z, yaw, pitch, roll)          # Se mueve a la posición de colocación
    open_gripper()                              # Se abre el gripper para soltar
    move_arm(x, y, z + 0.1, yaw, pitch, roll)  # Se mueve hacia arriba
    return

def return_home():
    move_arm(0.4, 0.0, 0.8, 0, 0, 0)  # Posición inicial del brazo
    print("Returning to home position")
    return


def pick_and_place(pick_x, pick_y, pick_z, place_x, place_y, place_z,
                   place_yaw=0, place_pitch=0, place_roll=0, pick_yaw=0, pick_pitch=0, pick_roll=0):
    open_gripper()
    pick(pick_x, pick_y, pick_z, pick_yaw, pick_pitch, pick_roll)
    place(place_x, place_y, place_z, place_yaw, place_pitch, place_roll)
    close_gripper()
    return_home()
    return

def main():

    input_values = input("Escribe los valores donde està el objeto en x, y, z separados por espacios: ")
    pick_x, pick_y, pick_z = map(float, input_values.split())
    input_values = input("Escribe los valores donde quieres dejar el objeto en x, y, z separados por espacios: ")
    place_x, place_y, place_z = map(float, input_values.split())
    # Ejemplo de uso de la función pick_and_place
    pick_and_place(
        pick_x=pick_x, pick_y=pick_y, pick_z=pick_z,
        place_x=place_x, place_y=place_y, place_z=place_z
    )

if __name__ == "__main__":
    main()
