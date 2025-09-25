#!/usr/bin/env python3
"""
Global Motion Planner usando MoveIt Python API
Equivalente a usar planning_scene_interface.h pero en Python
"""
import rclpy
from rclpy.node import Node
from moveit_py import MoveItPy
from moveit_py.planning import PlanRequestParameters
from geometry_msgs.msg import PoseStamped
import time

class GlobalMotionPlannerPy(Node):
    def __init__(self):
        super().__init__('global_motion_planner_py')
        
        # Inicializar MoveIt Python
        self.moveit = MoveItPy(node_name="moveit_py")
        self.robot_model = self.moveit.get_robot_model()
        
        # Obtener el grupo de planificación (ajusta el nombre según tu configuración)
        self.planning_group = "manipulator"  # o "arm", "bender_arm", etc.
        
        # Planning scene para gestionar obstáculos
        self.planning_scene_monitor = self.moveit.get_planning_scene_monitor()
        
        self.get_logger().info("Global Motion Planner Python inicializado")
        
        # Timer para ejecutar planificación cada 10 segundos
        self.timer = self.create_timer(10.0, self.plan_and_execute)
        
    def plan_and_execute(self):
        """Planifica y ejecuta movimiento hacia una pose objetivo"""
        try:
            # Crear pose objetivo
            pose_goal = PoseStamped()
            pose_goal.header.frame_id = "base_link"
            pose_goal.header.stamp = self.get_clock().now().to_msg()
            
            # Definir posición y orientación objetivo
            pose_goal.pose.position.x = 0.5
            pose_goal.pose.position.y = 0.3
            pose_goal.pose.position.z = 0.8
            
            pose_goal.pose.orientation.x = 0.0
            pose_goal.pose.orientation.y = 0.0
            pose_goal.pose.orientation.z = 0.0
            pose_goal.pose.orientation.w = 1.0
            
            # Configurar parámetros de planificación
            plan_request_params = PlanRequestParameters(
                planner_id="RRTConnectkConfigDefault",  # Planner a usar
                planning_time=10.0,  # Tiempo máximo de planificación
                max_velocity_scaling_factor=0.1,  # Factor de velocidad (más lento)
                max_acceleration_scaling_factor=0.1  # Factor de aceleración
            )
            
            self.get_logger().info(f"Planificando movimiento hacia: x={pose_goal.pose.position.x}, y={pose_goal.pose.position.y}, z={pose_goal.pose.position.z}")
            
            # Planificar movimiento
            robot_trajectory = self.moveit.plan_kinematic_path(
                planning_component=self.planning_group,
                robot_state=self.moveit.get_robot_state(),
                multi_plan_request_parameters=plan_request_params,
                pose_goal=pose_goal
            )
            
            if robot_trajectory:
                self.get_logger().info("✅ Plan encontrado! Ejecutando movimiento...")
                
                # Ejecutar el plan
                self.moveit.execute(
                    robot_trajectory=robot_trajectory,
                    controllers=[]  # Usar controladores por defecto
                )
                
                self.get_logger().info("✅ Movimiento completado!")
            else:
                self.get_logger().warn("❌ No se pudo encontrar un plan válido")
                
        except Exception as e:
            self.get_logger().error(f"Error en planificación: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        planner = GlobalMotionPlannerPy()
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
