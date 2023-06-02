import math
import numpy as np
import time

#Definir dimensiones del brazo robotico
l1 = 12 #Longitud del primer eslabon (cm)
l2 = 24 #Longitud del segundo eslabon (cm)
h = 25.5 #Altura del efector final (cm)


#Definir variables globales de posicion deseada del efector final
global desx,desy,desz
desx=0
desy=36
desz=25.5
global goal #Variable que indica si se ha llegado a la posicion deseada
goal = False
global llamado #Variable que indica si se ha solicitado una nueva posicion
global grados
grados = [90,45,0,0]
llamado = True

#Definir la clase que publique la posicion del efector final
class RobotManipulatorPosition:
    def __init__(self):
        while True:
            #Obtener valores de prueba de los angulos de los motores
            global grados, goal, llamado

            if len(grados) >= 4:
                gradoRot, gradoj1, gradoj2, gradog = int(grados[0]), int(grados[1]), int(grados[2]), int(grados[3])
                #Calcular posicion del end effector
                x,y,z = self.calcularPosicion(gradoRot, gradoj1, gradoj2)
                print(x,y,z)
                #Publicar posicion del end effector

                #Verificar si se ha llegado a la posicion deseada
                if goal == False and llamado == True:
                    if abs(x-desx) < 0.01 and abs(y-desy) < 0.01 and abs(z-desz) < 0.01:
                        goal = True
                        llamado = False
                        self.get_logger().info('Posicion deseada alcanzada')
                    else:
                        self.calcularCinematicaInversa(x,y,z,gradoRot, gradoj1, gradoj2, gradog)
                        print('Cinematica inversa calculada')
                
                #Esperar 1 segundo
                time.sleep(1)


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

        global grados 
        global desx, desy, desz
        #Convierter grados a radianes
        radRot = math.radians(gradoRot)
        radj1 = math.radians(gradoj1)
        radj2 = math.radians(gradoj2)

        #Calcular errores de posicion
        ex = desx - x
        ey = desy - y
        ez = desz - z

        #Vector de errores
        e = np.array([ex, ey, ez])
        print(e)

        theta = np.array([radRot, radj1, radj2])  # Vector columna de radRot, radj1, radj2
        a = np.array([l1, l2, h])  # Vector fila de l1, l2, h
        print(theta)
        print(a)

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

        #Impresion de la matriz jacobiana
        print(J)

        #Calcular determinante de la matriz jacobiana
        detJ = np.linalg.det(J)

        #Imprimir determinante de la matriz jacobiana
        print(detJ)

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

        #Guardar grados en la lista
        grados[0] = gRot
        grados[1] = gj1
        grados[2] = gj2
        grados[3] = gradog


    
def main(args=None):
    robot_position = RobotManipulatorPosition()

if __name__ == '__main__':
    main()