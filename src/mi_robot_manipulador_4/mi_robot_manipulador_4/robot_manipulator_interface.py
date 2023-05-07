import rclpy
import threading
from geometry_msgs.msg import Twist
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import tkinter as tk
from tkinter import font as tkFont
from tkinter import *
import tkinter.messagebox as messagebox
from tkinter.filedialog import *

global x,y,z,k
x = []
y = []
z = []
k = 0

class RobotManipulatorInterface(Node):

    def __init__(self,interfaz):
        super().__init__('robot_manipulator_interface')
        self.subscription = self.create_subscription(
            Twist,
            'robot_manipulator_pos',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.interfaz = interfaz

    def listener_callback(self, msg):
        xact = int(msg.linear.x) 
        yact = int(msg.linear.y) 
        zact = int(msg.linear.z) 

        k = k+1
        x.append(xact)
        y.append(yact)
        z.append(zact)

        # Si alguna de las variables cambio con respecto al valor anterior, se envia el mensaje
        if (x[k-1] != x[k]) or (y[k-1] != y[k]) or (z[k-1] != z[k]):
            self.interfaz.drawpos(x[k-1],y[k-1],x[k],y[k],z[k-1],z[k])
            
        
      

class MyInterfaz:

    def __init__(self):

        self.tk = tk.Tk()
        self.frame = tk.Frame(self, bg="#6E6E6E")

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlim(-100, 100)
        self.ax.set_ylim(-100, 100)
        self.ax.set_zlim(-100, 100)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('Posición del brazo robótico')
        self.ax.grid(True)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.frame)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

        self.frame.pack(fill=tk.BOTH, expand=1) 

        self.botones = tk.Frame(self)
        self.btn_save = tk.Button(self.botones, font=('Courier',16), text="Guardar", command=self.guardar)

        self.btn_save.pack(side="bottom")

        self.anim1 = None

        self.framegraf.pack(fill=tk.BOTH, expand=1)
        self.botones.pack(fill="x")

    def guardar(self):
        if self.btn_save["text"] == "Guardar Figura":          
            input_path = asksaveasfilename()
            plt.savefig(input_path)
        else:
            self.btn_guardar.configure(text="Guardado")
    
    def drawpos(self, x0, y0, x1, y1, z0, z1):
        xs = [x0, x1]
        ys = [y0, y1]
        zs = [z0, z1]
        if self.anim1:
            self.anim1.event_source.stop()
        self.ax.plot(xs, ys, zs, color='red')
        self.canvas.draw()
    
    def start(self):
        self.master.mainloop()


class MyThread(threading.Thread):
    def __init__(self, node, gui):
        threading.Thread.__init__(self)
        self.node = node
        self.gui = gui

    def run(self):
        while True:
            rclpy.spin_once(self.node)

def main(args=None):
    rclpy.init()
    my_gui = MyInterfaz()
    my_node = RobotManipulatorInterface(my_gui)
    my_thread = MyThread(my_node, my_gui)
    my_thread.start()
    my_gui.start()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()

         
            



