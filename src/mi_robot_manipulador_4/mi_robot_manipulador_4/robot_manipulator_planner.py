import rclpy
import time
import sys, select, tty, termios
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from pynput import keyboard
from pynput.keyboard import Key, Listener

#Declarar variables globales de interes
global posx, posy, posz, zona, opcion
posx = 0
posy = 0
posz = 0
zona = 0
opcion = 0

#Preguntar si quiere mover el robot a un punto o localizar un objeto con la garra (1 o 2)
print("¿Que desea hacer?")
print("1. Mover el robot a un punto")
print("2. Agarrar objeto con la garra")
opcion = input("Ingrese el numero de la opcion: ")

opcion = int(opcion)

if opcion == 1:
    #Preguntar por las posiciones deseadas
    posx = input("Ingrese la posicion en x: ")
    posy = input("Ingrese la posicion en y: ")
    posz = input("Ingrese la posicion en z: ")
elif opcion == 2:
    #Preguntar en que zona de la base se encuentra el objeto
    zona = opcion


posx = float(posx)
posy = float(posy)
posz = float(posz)

class RobotManipulatorPlanner(Node):
    def __init__(self):
        super().__init__('robot_manipulator_planner')
        self.pubcmd = self.create_publisher(Twist, 'robot_manipulator_goal', 10)
        self.pubzone = self.create_publisher(String, 'robot_manipulator_zone', 10)
        self.msg = Twist()
        self.msgzone = String()

        #Llamado a variables globales de interes
        global posx, posy, posz, zona, opcion

        #Revisa cual opcion fue seleccionada
        if opcion == 1:
            #Publicar posiciones deseadas
            self.msg.linear.x = posx
            self.msg.linear.y = posy
            self.msg.linear.z = posz
            self.pubcmd.publish(self.msg)
        elif opcion == 2:
            #Publicar zona de interes
            self.msgzone.data = str(zona)
            self.pubzone.publish(self.msgzone)

def main(args=None):
    rclpy.init(args=args)
    robot_manipulator_planner = RobotManipulatorPlanner()
    rclpy.spin(robot_manipulator_planner)
    robot_manipulator_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()