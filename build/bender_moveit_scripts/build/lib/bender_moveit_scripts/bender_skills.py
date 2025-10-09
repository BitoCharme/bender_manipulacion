def move_left_arm(x,y,z,x2,y2,z2,w2):
    #solo debe ser un script que publique
    ros2 topic pub /target_pose geometry_msgs/msg/Pose "{position:{x: 0.0, y: 0.0, z: 0.0}, orientation:{x: 0.5, y: 0.0, z: 0.0, w: 0.0}}"
    #con las coordenadas que quieras
    #y que te diga que no se puede si la pose es inalcanzable o inválida