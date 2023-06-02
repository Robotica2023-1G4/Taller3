import math

# Dimensiones del brazo
l1 = 64 # Longitud del primer eslabón (cm)
l2 = 80 # Longitud del segundo eslabón (cm)
h = 80 # Altura del efector final (cm)

# Posición deseada del efector final
desx = 0
desy = 144
desz = 80

# Resto del código de cinemática inversa
distance = (((-desx)**2 + (desy)**2 + (desz-h)**2) - l1**2 - l2**2) / (2 * l1 * l2)
print(distance)
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

# Imprimir resultados
print("Ángulos de articulación:")
print("Theta1:", math.degrees(theta1))
print("Theta2:", math.degrees(theta2))
print("Theta3:", math.degrees(theta3))


#Convierter grados a radianes
radRot = theta1
radj1 = theta2 
radj2 = theta3

#Calcular posicion del end effector
x = l1*math.cos(radj1)*math.cos(radRot) + l2*math.cos(radj1 + radj2)*math.cos(radRot)
y = l1*math.cos(radj1)*math.sin(radRot) + l2*math.cos(radj1 + radj2)*math.sin(radRot)
z = h + l1*math.sin(radj1) + l2*math.sin(radj1 + radj2)
#H1 = [[math.cos(radRot), -math.sin(radRot)*math.cos(math.pi/2), math.sin(radRot)*math.sin(math.pi/2), 0],[math.sin(radRot), math.cos(radRot)*math.cos(math.pi/2), -math.cos(radRot)*math.sin(math.pi/2), 0], [0, math.sin(math.pi/2), math.cos(math.pi/2), l1*math.sin(math.pi/2)], [0, 0, 0, 1]]

#redondeado_x = round(x, 1)
#redondeado_y = round(y, 1)


print("Posición:")
print("x =" + str(x))
print("y =" + str(y))
print("z =" + str(z))

