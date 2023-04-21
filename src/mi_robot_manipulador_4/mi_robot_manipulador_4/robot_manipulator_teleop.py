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

#Interfaz que indica las teclas a oprmir para mover el robot
print("Para mover el robot, oprima las teclas indicadas a continuacion: ")
print("w: Subir")
print("s: Bajar")
print("a: Izquierda")
print("d: Derecha")
print("i: Abrir garra")
print("o: Cerrar garra")
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
                self.app_twist(velx,0.0,0.0) 
            #Hacia abajo
            elif key.char == "s":
                self.app_twist(-velx,0.0,0.0)
            #Hacia la izquierda
            elif key.char == "a":
                self.app_twist(0.0,vely,0.0)
            #Hacia la derecha
            elif key.char == "d":
                self.app_twist(0.0,-vely,0.0)
            #Abrir garra
            elif key.char == "i":
                self.app_twist(0.0,0.0,velg)
            #Cerrar garra
            elif key.char == "o":
                self.app_twist(0.0,0.0,-velg)
            #Salir
            elif key.char == "p":
                self.app_twist(0.0,0.0,0.0)
                exit()
            else:
                self.app_twist(0.0,0.0,0.0)
        except AttributeError:
            print("Tecla no valida")

    #Se crea la funcion que se encarga de recibir las teclas soltadas
    def on_release(self,key):
        self.app_twist(0.0,0.0,0.0)

    #Se crea la funcion que se encarga de publicar los mensajes de velocidad
    def app_twist(self, x, y, z):
        self.msg.linear.x = x
        self.msg.linear.y = y
        self.msg.linear.z = z
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
        
        

