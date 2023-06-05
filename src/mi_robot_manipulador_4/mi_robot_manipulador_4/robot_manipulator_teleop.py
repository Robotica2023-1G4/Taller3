import rclpy
import time
import sys, select, tty, termios
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard
from pynput.keyboard import Key, Listener


#Solicitamos las variables de velocidad 
velrot = input("Velocidad rotación sobre su eje (Valor entre 1 y 10): ")
velj1 = input("Velocidad juntura 1 (Valor entre 1 y 10): ")
velj2 = input("Velocidad juntura 2 (Valor entre 1 y 10): ")
#velj3 = input("Velocidad juntura 3 (Valor entre 1 y 10): ")
velRotg = input("Velocidad de rotacion de garra (Valor entre 1 y 10): ")
velg = input("Velocidad garra (Valor entre 1 y 10): ")

velrot = float(velrot)
velj1 = float(velj1)
velj2 = float(velj2)
velj3 = float(velj1)
velRotg = float(velRotg)
velg = float(velg)

#Interfaz que indica las teclas a oprmir para mover el robot
print("Para mover el robot, oprima las teclas indicadas a continuacion: ")
print("q: Subir juntura 1")
print("a: Bajar juntura 1")
print("o: Subir juntura 2")
print("i: Bajar juntura 2")
print("z: Rotar sobre su eje en sentido horario")
print("x: Rotar sobre su eje en sentido antihorario")
print("e: Abrir garra")
print("d: Cerrar garra")
print("k: Rotar garra sobre su eje en sentido horario")
print("l: Rotar garra sobre su eje en sentido antihorario")
print("p: Salir")

#Se crea la clase que se encarga de publicar los mensajes de velocidad
class RobotManipulatorTeleop(Node):
    def __init__(self):
        super().__init__('robot_manipulator_teleop')
        self.pubcmd = self.create_publisher(Twist, 'robot_manipulator_vel', 10)
        self.msg = Twist()
        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        with keyboard.Listener(
                on_press=self.on_press,
                on_release=self.on_release) as listener:
             listener.join()

    #Se crea la funcion que se encarga de recibir las teclas presionadas
    def on_press(self,key):

        try:
            #Hacia arriba
            if key.char == "w":
                self.app_twist(0.0,velj1,0.0,0.0,0.0,0.0)
            #Hacia abajo
            elif key.char == "s":
                self.app_twist(0.0,-velj1,0.0,0.0,0.0,0.0)
            #Hacia arriba servo2
            elif key.char == "q":
                self.app_twist(0.0,0.0,velj2,0.0,0.0,0.0)
            #Hacia abajo servo2
            elif key.char == "a":
                self.app_twist(0.0,0.0,-velj2,0.0,0.0,0.0)
            #Rotar en sentido horario
            elif key.char == "z":
                self.app_twist(velrot,0.0,0.0,0.0,0.0,0.0)
            #Rotar en sentido antihorario
            elif key.char == "x":
                self.app_twist(-velrot,0.0,0.0,0.0,0.0,0.0)
            #Hacia arriba servo3
            elif key.char == "e":
                self.app_twist(0.0,0.0,0.0,0.0,velj3,0.0)
            #Hacia abajo servo3
            elif key.char == "d":
                self.app_twist(0.0,0.0,0.0,0.0,-velj3,0.0)
            #Abrir garra
            elif key.char == "i":
                self.app_twist(0.0,0.0,0.0,-velg,0.0,0.0)
            #Cerrar garra
            elif key.char == "o":
                self.app_twist(0.0,0.0,0.0,velg,0.0,0.0)
            #Girar garra horario
            elif key.char == "k":
                self.app_twist(0.0,0.0,0.0,0.0,0.0,velRotg)
            #Girar garra antihorario
            elif key.char == "l":
                self.app_twist(0.0,0.0,0.0,0.0,0.0,-velRotg)
            #Salir
            elif key.char == "p":
                self.app_twist(0.0,0.0,0.0,0.0,0.0,0.0)
                exit()
            else:
                self.app_twist(0.0,0.0,0.0,0.0,0.0,0.0)
        except AttributeError:
            print("Tecla no valida")

    #Se crea la funcion que se encarga de recibir las teclas soltadas
    def on_release(self,key):
        self.app_twist(0.0,0.0,0.0,0.0,0.0,0.0)

    #Se crea la funcion que se encarga de publicar los mensajes de velocidad
    def app_twist(self, rot, j1, j2, j3, grot, g):
        self.msg.linear.x = rot
        self.msg.linear.y = j1
        self.msg.linear.z = j2
        self.msg.angular.x = g
        self.msg.angular.y = j3
        self.msg.angular.z = grot
        self.pubcmd.publish(self.msg)

#Se crea la funcion main
def main(args=None):
    rclpy.init(args=args)
    robot_manipulator_teleop = RobotManipulatorTeleop()
    rclpy.spin(robot_manipulator_teleop)
    robot_manipulator_teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
        

