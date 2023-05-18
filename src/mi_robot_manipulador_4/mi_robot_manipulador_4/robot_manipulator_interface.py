import tkinter as tk
import threading
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tkinter import filedialog
from PIL import Image, ImageDraw

global ka
global xs   
global ys  
global i   

ka = 0
xs = [250]
ys = [250]

class Robot_interface(Node):

    def __init__(self, interfaz):

        global leer

        super().__init__('robot_interface')
        self.subscription = self.create_subscription(Twist,'robot_manipulator_pos', self.listener_callback,10)
        self.subscription

        self.interfaz = interfaz

    
    def listener_callback(self, msg):
        
        global ka
        global xs
        global ys

        ka += 1
        xs.append(250 + int(msg.linear.x)*2)
        ys.append(250 + int(msg.linear.y)*(-2))
        if xs[ka-1] != xs[ka] or ys[ka-1] != ys[ka]:
            print(xs[ka])
            print(ys[ka])
            self.interfaz.draw_line(xs[ka-1],ys[ka-1],xs[ka],ys[ka])
            #self.interfaz.paint_pixel(int(250+int((msg.linear.x)*100)), (int((msg.linear.y)*(-100))+250))
    	



class MyInterfaz:

    def __init__(self):
    
        self.master = tk.Tk()

        #self.master.withdraw()

        self.canvas = tk.Canvas(self.master, width=1000, height=1000, background='white')
        self.canvas.pack()

        # definir dimensiones de la cuadrícula
        grid_size = 14
        canvas_width = 1000
        canvas_height = 1000

        # crear las líneas de la cuadrícula
        for i in range(grid_size + 1):
            x = i * (canvas_width / grid_size)
            self.canvas.create_line(x, 0, x, canvas_height, fill='yellow')
            y = i * (canvas_height / grid_size)
            self.canvas.create_line(0, y, canvas_width, y, fill='yellow')


        self.name_entry = tk.Entry(self.master)
        self.name_entry.pack()

        self.save_button = tk.Button(self.master, text="Save Image", command=self.save_image)
        self.save_button.pack()


    def preguntarRecorrido(self):
        
        global leer
        global lecturaTxt
        global escribir
        global escrituraTxt

        root = tk.Tk()
        root.withdraw()

    def paint_pixel(self,posx, posy):

        self.canvas.create_oval(posx, posy, posx+3, posy+3, fill='black')
        
    def draw_line(self,posx, posy,posx1, posy1):

        self.canvas.create_line(posx, posy, posx1, posy1, width=5, fill='black')
    
    def save_image(self):

        filename = filedialog.asksaveasfilename(initialfile = str(self.name_entry.get()) + '.png',defaultextension=".png")

        if filename:
            x = self.canvas.winfo_rootx() + self.canvas.winfo_x()
            y = self.canvas.winfo_rooty() + self.canvas.winfo_y()
            x1 = x + self.canvas.winfo_width()
            y1 = y + self.canvas.winfo_height()
            image = Image.new("RGB", (self.canvas.winfo_width(), self.canvas.winfo_height()), "white")
            draw = ImageDraw.Draw(image)

            for item in self.canvas.find_all():
                coords = self.canvas.coords(item)
                x0 = coords[0]
                y0 = coords[1]
                x1 = coords[2]
                y1 = coords[3]
                draw.rectangle([x0, y0, x1, y1], fill="black")

            image.save(filename)
        	       
      

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
    my_node = Robot_interface(my_gui)
    my_thread = MyThread(my_node, my_gui)
    my_thread.start()
    my_gui.start()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
         
            



