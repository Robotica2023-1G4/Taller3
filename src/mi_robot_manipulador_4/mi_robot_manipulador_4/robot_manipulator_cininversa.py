import rclpy
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


#Definir dimensiones del brazo robotico
l1 = 12 #Longitud del primer eslabon (cm)
l2 = 24 #Longitud del segundo eslabon (cm)
h = 25.5 #Altura del efector final (cm)

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


#Definir la clase que publique la posicion del efector final
class RobotManipulatorCininversa(Node):
    def __init__(self):
        super().__init__('robot_manipulator_cininversa')
        self.pubvel = self.create_publisher(Twist, 'robot_manipulator_vel', 10)
        self.subgra = self.create_subscription(Twist,'robot_manipulator_gra', self.listener_callback1,10)
        self.subgoal = self.create_subscription(Twist,'robot_manipulator_goal', self.listener_callback2,10)
        self.msg = Twist()
        self.pubvel.publish(self.msg)
        self.msggrados = Twist()

    def listener_callback1(self, msggrados):
        global gradoRot,gradoj1,gradoj2
        print('adios')
        gradoRot = msggrados.linear.x
        gradoj1 = msggrados.linear.y
        gradoj2 = msggrados.linear.z


    def listener_callback2(self, msg):
        print('hola')
        global desx,desy,desz,x,y,z,gradoRot,gradoj1,gradoj2
        desx = msg.linear.x
        desy = msg.linear.y
        desz = msg.linear.z
        self.get_logger().info('Posicion solicitada en x: "%f"' % desx + ' y: "%f"' % desy + ' z: "%f"' % desz)

        gradoRot=int(gradoRot)
        gradoj1=int(gradoj1)
        gradoj2=int(gradoj2)

        
        #Recibir grados cinematica inversa
        gRot,gj1,gj2 = self.calcularCinematicaInversa()
        #Calcular cambio en grados de los motores
        print(gRot,gj1,gj2)
        print(gradoRot)
        dRot = gRot - gradoRot
        print(dRot)
        dj1 = gj1 - gradoj1
        dj2 = gj2 - gradoj2
        #Publicar cambio en grados de los motores
        self.msg.linear.x = float(dRot)
        self.msg.linear.y = float(dj1)
        self.msg.linear.z = float(dj2)
        self.msg.angular.x = 0.0
        self.pubvel.publish(self.msg)
    
    def calcularCinematicaInversa(self):

        #Calcular cinematica inversa
        # Resto del código de cinemática inversa
        distance = (((-desx)**2 + (desy)**2 + (desz-h)**2) - l1**2 - l2**2) / (2 * l1 * l2)
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
    robot_manipulator_cininversa = RobotManipulatorCininversa()
    rclpy.spin(robot_manipulator_cininversa)
    robot_manipulator_cininversa.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


        



