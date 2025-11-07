#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from dynamixel_sdk import *  
import time
import yaml
import math
import os

class DynamixelCommander():
    def __init__(self,config_path="config/params.yaml"):

        with open(config_path, "r") as file:
            self.config = yaml.safe_load(file)

        # Configuración general
        self.PROTOCOL_VERSION = self.config["protocol_version"]
        self.BAUDRATE = self.config["baudrate"]
        self.DEVICE_NAME = self.config["device_name"]
        self.MOTORS = self.config["motors"]

        self.DXL_IDS = [motor["id"] for motor in self.MOTORS]
        self.offsets = {motor["id"]: motor["offset"] for motor in self.MOTORS}
        self.limits = {motor["id"]: motor["limits"] for motor in self.MOTORS}

        print(self.limits)
        # Direcciones de los RX-28, RX-64 y MX-106 
        self.ADDR_PRESENT_POSITION = 36
        self.ADDR_PRESENT_SPEED = 38
        self.ADDR_TORQUE_ENABLE = 24
        self.ADDR_GOAL_POSITION = 30
        self.ADDR_GOAL_SPEED = 32

        self.ADDR_CW_ANGLE_LIMIT = 6
        self.ADDR_CCW_ANGLE_LIMIT = 8

        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0

        # Inicializa puerto y handler
        self.portHandler = PortHandler(self.DEVICE_NAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        if not self.portHandler.openPort():
            print("[ERROR] No se pudo abrir el puerto")
            exit()

        if not self.portHandler.setBaudRate(self.BAUDRATE):
            print("[ERROR] No se pudo establecer la velocidad de baudios")
            exit()

        # Habilitar torque y activar modo wheel
        for dxl_id in self.DXL_IDS:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)


        print("[INFO] DynamixelCommander inicializado en modo wheel.")

    def get_joints_data(self):
        positions = []
        velocities = []
        for dxl_id in self.DXL_IDS:
            data, result, error = self.packetHandler.readTxRx(
                self.portHandler, dxl_id, self.ADDR_PRESENT_POSITION, 4)

            if result != COMM_SUCCESS or error != 0 or data is None or len(data) < 4:
                print(f"[WARN] Fallo al leer motor {dxl_id}")
                continue

            pos_raw = data[0] + (data[1] << 8)
            vel_raw = data[2] + (data[3] << 8)

            positions.append(pos_raw-self.offsets[dxl_id])
            velocities.append(vel_raw)

        return positions, velocities

    def set_joints(self, joints, vel=0.5):
        t = time.time()
        if len(joints) != 7:
            print("[WARN] Se esperaban exactamente 7 posiciones (en radianes)")
            return

        for i, dxl_id in enumerate(self.DXL_IDS):
            # Obtener tipo de motor
            motor_type = self.MOTORS[i]["type"]
            if motor_type == "MX-106":
                resolution = 4095
                max_radians = 2 * math.pi
                resolution_per_unit = 0.114
            else:  # RX-28
                resolution = 1023
                max_radians = math.radians(300)
                resolution_per_unit = 0.111

            # Velocidad deseada
            rpm = vel * 9.55
            raw_speed = int(abs(rpm) / resolution_per_unit)
            raw_speed = min(raw_speed, 1023)

            # Setear velocidad máxima
            result, error = self.packetHandler.write2ByteTxRx(
                self.portHandler, dxl_id, self.ADDR_GOAL_SPEED, raw_speed)
            if result != COMM_SUCCESS or error != 0:
                print(f"[WARN] Error al enviar velocidad al motor {dxl_id}")

            # Posición deseada
            joint_pos = joints[i]   
            pos_raw = int((joint_pos / max_radians) * resolution)
            pos_raw += self.offsets[dxl_id]
            pos_raw = max(0, min(resolution, pos_raw))
            # Enviar posición al motor
            result, error = self.packetHandler.write2ByteTxRx(
                self.portHandler, dxl_id, self.ADDR_GOAL_POSITION, pos_raw)
            if result != COMM_SUCCESS or error != 0:
                print(f"[WARN] Error al mover motor {dxl_id}")

        print(time.time()-t)
    def shutdown(self):
        for dxl_id in self.DXL_IDS:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        self.portHandler.closePort()
        print("[INFO] Puerto cerrado y torque desactivado.")




