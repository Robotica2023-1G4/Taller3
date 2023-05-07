import rclpy
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import serial

#Definir dimensiones del brazo robotico
l1 = 0.135 #Longitud del primer eslabon (cm)
l2 = 0.175 #Longitud del segundo eslabon (cm)
h = 0.218 #Altura del efector final (cm)

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

        while True:
            #Obtener angulos de los motores a partir del puerto serial
            grados = pserial.readline().decode().rstrip().split(',')

            if len(grados) >= 4:
                gradoRot, gradoj1, gradoj2, gradog = int(grados[0]), int(grados[1]), int(grados[2]), int(grados[3])
                #Calcular posicion del end effector
                x,y,z = self.calcularPosicion(gradoRot, gradoj1, gradoj2)
                #Publicar posicion del end effector
                self.msg.linear.x = x
                self.msg.linear.y = y
                self.msg.linear.z = z
                self.pubpos.publish(self.msg)

                #Verificar si se ha llegado a la posicion deseada
                if goal == False and llamado == True:
                    if abs(x-desx) < 0.01 and abs(y-desy) < 0.01 and abs(z-desz) < 0.01:
                        goal = True
                        llamado = False
                        self.get_logger().info('Posicion deseada alcanzada')
                    else:
                        self.calcularCinematicaInversa(x,y,z,gradoRot, gradoj1, gradoj2, gradog)
               
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
    
    def calcularCinematicaInversa(self, x, y, z, gradoRot, gradoj1, gradoj2, gradog):

        radRot = math.radians(gradoRot)
        radj1 = math.radians(gradoj1)
        radj2 = math.radians(gradoj2)

        #Calcular errores de posicion
        ex = desx - x
        ey = desy - y
        ez = desz - z

        #Vector de errores
        e = np.array([ex, ey, ez])

        theta = np.zeros([radRot, radj1, radj2])  # Vector columna de radRot, radj1, radj2
        a = np.array([l1, l2, h])  # Vector fila de l1, l2, h

        #Matriz jacoviana
        # Calcular las entradas de la matriz jacobiana
        J = np.zeros((3,3))
        J[0,0] = -a[0]*math.cos(theta[1])*math.sin(theta[0]) - a[1]*math.cos(theta[1] + theta[2])*math.sin(theta[0])
        J[0,1] = -a[0]*math.cos(theta[0])*math.sin(theta[1]) - a[1]*math.sin(theta[1] + theta[2])*math.cos(theta[0])
        J[0,2] = -a[1]*math.sin(theta[1] + theta[2])*math.cos(theta[0])
        J[1,0] = a[0]*math.cos(theta[0])*math.cos(theta[1]) + a[1]*math.cos(theta[0])*math.cos(theta[1] + theta[2])
        J[1,1] = -a[0]*math.sin(theta[0])*math.sin(theta[1]) - a[1]*math.sin(theta[0])*math.sin(theta[1] + theta[2])
        J[1,2] = -a[1]*math.sin(theta[0])*math.sin(theta[1] + theta[2])
        J[2,0] = 0
        J[2,1] = a[0]*math.cos(theta[1]) + a[1]*math.cos(theta[1] + theta[2])
        J[2,2] = a[1]*math.cos(theta[1] + theta[2])

        #Calcular la matriz inversa de la matriz jacobiana
        Jinv = np.linalg.inv(J)

        #Calcular Matriz de ganancias
        K = np.array([[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]])

        #Calcular ley de control (v = Jinv*K*e)
        v = np.dot(np.dot(Jinv, K), e)

        #Calcular cambio en grados
        dtheta = v*0.1

        #Calcular nuevos grados
        theta = theta + dtheta

        #Calcular nuevos grados como valores enteros
        gRot = int(math.degrees(theta[0]))
        gj1 = int(math.degrees(theta[1]))
        gj2 = int(math.degrees(theta[2]))

        #Publicar cambio en grados
        self.msg.linear.x = gRot
        self.msg.linear.y = gj1
        self.msg.linear.z = gj2
        self.msg.angular.x = gradog
        self.pubvel.publish(self.msg)

    
def main(args=None):
    rclpy.init(args=args)
    robot_manipulator_position = RobotManipulatorPosition()
    rclpy.spin(robot_manipulator_position)
    robot_manipulator_position.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


        



