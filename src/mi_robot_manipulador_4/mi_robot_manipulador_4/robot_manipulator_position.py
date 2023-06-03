import rclpy
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import serial

#Definir dimensiones del brazo robotico
l1 = 13 #Longitud del primer eslabon (cm)
l2 = 26 #Longitud del segundo eslabon (cm)
h = 31 #Altura del efector final (cm)

#Definir el puerto de comunicacion serial
pserial = serial.Serial('/dev/ttyACM0', 9600)

#Definir variables globales de posicion deseada del efector final
global desx,desy,desz
desx=0
desy=0
desz=0
#Definir grados
global gradoRot,gradoj1,gradoj2
gradoRot = 0
gradoj1 = 0
gradoj2 = 0
#Definir pòsicion actual
global x,y,z
x=0
y=0
z=0

#Definir la clase que publique la posicion del efector final
class RobotManipulatorPosition(Node):
    def __init__(self):
        super().__init__('robot_manipulator_position')
        self.pubpos = self.create_publisher(Twist, 'robot_manipulator_pos', 10)
        self.pubgra = self.create_publisher(Twist, 'robot_manipulator_gra',10)
        self.pubvel = self.create_publisher(Twist, 'robot_manipulator_vel', 10)
        self.msg = Twist()
        self.msggrados = Twist()
        global desx, desy, desz, gradoRot, gradoj1, gradoj2, x, y, z
        while True:
            #Obtener angulos de los motores a partir del puerto serial
            grados = pserial.readline().decode().rstrip().split(',')
            print(grados)
            if len(grados) >= 4:
                gradoRot, gradoj1, gradoj2, gradog = int(grados[0]), int(grados[1]), int(grados[2]), int(grados[3])
                x,y,z = self.calcularPosicion(gradoRot, gradoj1, gradoj2)
                #Publicar posicion del end effector
                print(x,y,z)
                self.msg.linear.x = x
                self.msg.linear.y = y
                self.msg.linear.z = z
                self.msggrados.linear.x = float(gradoRot)
                self.msggrados.linear.y = float(gradoj1)
                self.msggrados.linear.z = float(gradoj2)
                self.pubpos.publish(self.msg)
                self.pubgra.publish(self.msggrados)
            

    def calcularPosicion(self, gradoRot, gradoj1, gradoj2):
        #Convierter grados a radianes
        radRot = math.radians(gradoRot)
        radj1 = math.radians(gradoj1)
        radj2 = math.radians(gradoj2)

        #Calcular posicion del end effector
        x = l1*math.cos(radj1)*math.cos(radRot) + l2*math.cos(radj1 + radj2)*math.cos(radRot)
        y = l1*math.cos(radj1)*math.sin(radRot) + l2*math.cos(radj1 + radj2)*math.sin(radRot)
        z = h + l1*math.sin(radj1) + l2*math.sin(radj1 + radj2)

        return x,y,z

    
def main(args=None):
    rclpy.init(args=args)
    robot_manipulator_position = RobotManipulatorPosition()
    rclpy.spin(robot_manipulator_position)
    robot_manipulator_position.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


        



