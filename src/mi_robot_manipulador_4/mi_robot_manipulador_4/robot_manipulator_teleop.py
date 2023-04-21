import rclpy
import time
import sys, select, tty, termios
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard
from pynput.keyboard import Key, Listener


#Solicitamos las variables de velocidad 
velx = input("Velocidad en x (Valor entre 1 y 10): ")
vely = input("Velocidad en y (Valor entre 1 y 10): ")
velg = input("Velocidad garra (Valor entre 1 y 10): ")

velx = float(velx)
vely = float(vely)

