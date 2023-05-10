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
        velRot = int(msg.linear.x) 
        velj1 = int(msg.linear.y) 
        velj2 = int(msg.linear.z)
        velj3 = int(msg.angular.y)
        velg = int(msg.angular.x)
        velrotg = int(msg.angular.z) 
        message = f"{velRot},{velj1},{velj2},{velj3},{velg},{velrotg}\n" # creamos el mensaje con el formato requerido por Arduino
        pserial.write(message.encode()) # enviamos el mensaje a través del puerto serial
        

def main(args=None):
    rclpy.init(args=args)
    robot_manipulator_controller = RobotManipulatorController()
    rclpy.spin(robot_manipulator_controller)
    robot_manipulator_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
