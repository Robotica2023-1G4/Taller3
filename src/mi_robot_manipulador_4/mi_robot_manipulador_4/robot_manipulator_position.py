import rclpy
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import serial

#Definir dimensiones del brazo robotico
l1 = 12 #Longitud del primer eslabon (cm)
l2 = 24 #Longitud del segundo eslabon (cm)
h = 25.5 #Altura del efector final (cm)

#Definir el puerto de comunicacion serial
pserial = serial.Serial('/dev/ttyACM0', 9600)

#Definir variables globales de posicion deseada del efector final
global desx,desy,desz
desx=0
desy=0
desz=0
global goal #Variable que indica si se ha llegado a la posicion deseada
goal = False
global llamado #Variable que indica si se ha solicitado una nueva posicion
llamado = False

#Definir la clase que publique la posicion del efector final
class RobotManipulatorPosition(Node):
    def __init__(self):
        super().__init__('robot_manipulator_position')
        self.pubpos = self.create_publisher(Twist, 'robot_manipulator_pos', 10)
        self.pubvel = self.create_publisher(Twist, 'robot_manipulator_vel', 10)
        self.subgoal = self.create_subscription(Twist, 'robot_manipulator_goal', self.listener_callback, 10)
        self.msg = Twist()
        global goal,llamado, desx, desy, desz
        while True:
            #Obtener angulos de los motores a partir del puerto serial
            grados = pserial.readline().decode().rstrip().split(',')

            if len(grados) >= 4:
                gradoRot, gradoj1, gradoj2, gradog = int(grados[0]), int(grados[1]), int(grados[2]), int(grados[3])
                x,y,z = self.calcularPosicion(gradoRot, gradoj1, gradoj2)
                #Publicar posicion del end effector
                self.msg.linear.x = x
                self.msg.linear.y = y
                self.msg.linear.z = z
                self.pubpos.publish(self.msg)

                #Verificar si se ha llegado a la posicion deseada
                if goal == False and llamado == True:
                    print(desx)
                    if abs(x-desx) < 0.1 and abs(y-desy) < 0.1 and abs(z-desz) < 0.1:
                        goal = True
                        llamado = False
                        self.get_logger().info('Posicion deseada alcanzada')
                    else:
                        #Recibir grados cinematica inversa
                        gRot,gj1,gj2 = self.calcularCinematicaInversa()
                        #Calcular cambio en grados de los motores
                        dRot = gRot - gradoRot
                        dj1 = gj1 - gradoj1
                        dj2 = gj2 - gradoj2
                        #Publicar cambio en grados de los motores
                        self.msg.linear.x = dRot
                        self.msg.linear.y = dj1
                        self.msg.linear.z = dj2
                        self.msg.linear.x = 0
                        self.pubvel.publish(self.msg)
               
    def listener_callback(self, msg):
        global x,y,z
        global goal
        global llamado
        desx = msg.linear.x
        desy = msg.linear.y
        desz = msg.linear.z
        goal = False
        llamado = True
        self.get_logger().info('Posicion solicitada en x: "%f"' % desx + ' y: "%f"' % desy + ' z: "%f"' % desz)

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
    
    def calcularCinematicaInversa(self):

        #Calcular cinematica inversa
        # Resto del código de cinemática inversa
        distance = (((-desx)**2 + (desy)**2 + (desz-h)**2) - l1**2 - l2**2) / (2 * l1 * l2)
        print(distance)
        theta1 = math.atan2(desy, desx)
        theta3 = math.atan2((-math.sqrt(1 - distance**2)), distance)
        theta2 = math.atan2(desz - h,math.sqrt(desx**2+desy**2)) - math.atan2((l2 * (-math.sqrt(1-distance**2))), (l1 + l2 * distance))
    
        #Convertir angulos a valores entre -pi y pi
        if theta1 > math.pi:
            while theta1 > math.pi:
                theta1 = theta1 - math.pi
        elif theta1 < -math.pi:
            while theta1 < -math.pi:
                theta1 = theta1 + math.pi

        if theta2 > math.pi:
            while theta2 > math.pi:
                theta2 = theta2 - math.pi
        elif theta2 < -math.pi:
            while theta2 < -math.pi:
                theta2 = theta2 + math.pi

        if theta3 > math.pi:
            while theta3 > math.pi:
                theta3 = theta3 - math.pi
        elif theta3 < -math.pi:
            while theta3 < -math.pi:
                theta3 = theta3 + math.pi

        #Calcular nuevos grados como valores enteros
        gRot = int(math.degrees(theta1))
        gj1 = int(math.degrees(theta2))
        gj2 = int(math.degrees(theta3))

        #Retornar grados
        return gRot,gj1,gj2

    
def main(args=None):
    rclpy.init(args=args)
    robot_manipulator_position = RobotManipulatorPosition()
    rclpy.spin(robot_manipulator_position)
    robot_manipulator_position.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


        



