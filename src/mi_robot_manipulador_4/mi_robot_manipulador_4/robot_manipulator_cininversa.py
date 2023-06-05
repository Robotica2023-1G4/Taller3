import rclpy
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time


#Definir dimensiones del brazo robotico
l1 = 12 #Longitud del primer eslabon (cm)
l2 = 12 #Longitud del segundo eslabon (cm)
h = 30 #Altura del efector final (cm)

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
#Definir posiciones de la zona de interes
global gradoRotZone,gradoj1Zone,gradoj2Zone
gradoRotZone = 0
gradoj1Zone = -30
gradoj2Zone = 15


#Definir la clase que publique la posicion del efector final
class RobotManipulatorCininversa(Node):
    def __init__(self):
        super().__init__('robot_manipulator_cininversa')
        self.pubvel = self.create_publisher(Twist, 'robot_manipulator_vel', 10)
        self.subgra = self.create_subscription(Twist,'robot_manipulator_gra', self.listener_callback1,10)
        self.subgoal = self.create_subscription(Twist,'robot_manipulator_goal', self.listener_callback2,10)
        self.subzone = self.create_subscription(Twist,'robot_manipulator_zone', self.listener_callback3,10)
        self.msg = Twist()
        self.pubvel.publish(self.msg)
        self.msggrados = Twist()

    def listener_callback1(self, msggrados):
        global gradoRot,gradoj1,gradoj2
        gradoRot = msggrados.linear.x
        gradoj1 = msggrados.linear.y
        gradoj2 = msggrados.linear.z


    def listener_callback2(self, msg):
        global desx,desy,desz,gradoRot,gradoj1,gradoj2
        desx = msg.linear.x
        desy = msg.linear.y
        desz = msg.linear.z

        gradoRot=int(gradoRot)
        gradoj1=int(gradoj1)
        gradoj2=int(gradoj2)
        
        #Recibir grados cinematica inversa
        gRot,gj1,gj2 = self.calcularCinematicaInversa()
        #Calcular cambio en grados de los motores
        dRot = gRot - gradoRot
        dj1 = -(gj1 - gradoj1)
        dj2 = gj2 - gradoj2
        #Publicar cambio en grados de los motores
        self.msg.linear.x = float(dRot)
        self.msg.linear.z = float(dj1)
        self.msg.angular.y = float(dj2)
        self.msg.angular.x = 0.0
        self.pubvel.publish(self.msg)

    def listener_callback3(self, msg):
        global gradoRotZone,gradoj1Zone,gradoj2Zone,gradoRot,gradoj1,gradoj2
        
        gradoRot=int(gradoRot)
        gradoj1=int(gradoj1)
        gradoj2=int(gradoj2)

        #Calcular cambio en grados de los motores
        dRot = gradoRotZone - gradoRot
        dj1 = -(30 - gradoj1)
        dj2 = gradoj2Zone - gradoj2
        #Publicar cambio en grados de los motores
        self.msg.linear.x = float(dRot)
        self.msg.linear.z = float(dj1)
        self.msg.angular.y = float(dj2)
        self.msg.angular.z = -80.0
        self.pubvel.publish(self.msg)

        #Esperar 2 segundos
        time.sleep(2)

        #Calcular cambio en grados de los motores
        twistmsg = Twist()
        #Publicar cambio en grados de los motores
        twistmsg.linear.x = 0.0
        twistmsg.linear.z = 60.0
        twistmsg.angular.y = 0.0
        twistmsg.angular.z = 0.0
        self.pubvel.publish(twistmsg)

        #Esperar 2 segundos
        time.sleep(2)
        
        #Cierro la garra
        twistmsg = Twist()
        twistmsg.angular.z = 80.0 
        self.pubvel.publish(twistmsg)

        #Esperar 2 segundos
        time.sleep(2)

        #Elevo el brazo
        twistmsg = Twist()
        twistmsg.angular.y = 60.0
        self.pubvel.publish(twistmsg)

        #Esperar 5 segundos 
        time.sleep(5)

        #Bajar el brazo
        twistmsg = Twist()
        twistmsg.angular.y = -60.0
        twistmsg.angular.z = -80.0
        self.pubvel.publish(twistmsg)


    
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


        



