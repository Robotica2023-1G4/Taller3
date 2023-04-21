import rclpy
import time
import sys, select, tty, termios
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard
from pynput.keyboard import Key, Listener

#Solicitar posicion en x,y y angulo de apertura de la garra
posx = input("Posicion en x (Valor entre 0 y 1): ")
posy = input("Posicion en y (Valor entre 0 y 1): ")
posg = input("Angulo de apertura de la garra (Valor entre 0 y 180): ")

posx = float(posx)
posy = float(posy)
posg = float(posg)

class RobotManipulatorPlanner(Node):
    def __init__(self):
        super().__init__('robot_manipulator_teleop')
        self.pubcmd = self.create_publisher(Twist, 'robot_manipulator_vel', 10)
        self.msg = Twist()

        #Publicar posiciones deseadas
        self.msg.linear.x = posx
        self.msg.linear.y = posy
        self.msg.linear.z = posg
        self.pubcmd.publish(self.msg)