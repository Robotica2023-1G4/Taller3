#!/usr/bin/env python3

import rclpy
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import serial

#Nombres entrada del puerto para los motores
pserial = serial.Serial('/dev/ttyACM0', 9600)

#Se crea la clase que se encarga de recibir los mensajes de velocidad
class RobotManipulatorController(Node):

    def __init__(self):
        super().__init__('robot_manipulator_controller')
        self.subscription = self.create_subscription(
            Twist,
            'robot_manipulator_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        x_speed = int(msg.linear.x * 100) # convertimos la velocidad lineal en un entero entre -100 y 100
        y_speed = int(msg.linear.y * 100) # convertimos la velocidad lineal en un entero entre -100 y 100
        g_speed = int(msg.linear.z * 100) # convertimos la velocidad lineal en un entero entre -100 y 100
        message = f"{x_speed},{y_speed},{g_speed}\n" # creamos el mensaje con el formato requerido por Arduino
        self.serial_port.write(message.encode()) # enviamos el mensaje a través del puerto serial
        self.get_logger().info(f'Message sent: {message}')

def main(args=None):
    rclpy.init(args=args)
    robot_manipulator_controller = RobotManipulatorController()
    rclpy.spin(robot_manipulator_controller)
    robot_manipulator_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()