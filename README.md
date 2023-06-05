# Taller3

Robot Manipulador - Límites de Trabajo

Este archivo README proporciona información sobre los límites de trabajo de un robot manipulador. A continuación se detallan los rangos de grados y las restricciones de posición en los ejes x, y y z.

1. Servo de Rotación:
   - Rango de Grados: -90° a 80°.

2. Primer Servo de Juntura:
   - Rango de Grados: -70° a 110°.

3. Segundo Servo de Juntura:
   - Rango de Grados: -35° a 145° (referido a 0° del primer servo de juntura).

4. Restricciones de Posición:
   - Eje x:
     - Rango de Posición: 0 cm a 24 cm.
     - Restricción Adicional: La suma de las junturas no debe exceder los 24 metros.
     - Los valores del eje x y y se relacionan entre ellos, por lo que deben cumplir la relacion de no exceder el circulo definido por el radio de las junturas entre ellos

   - Eje y:
     - Rango de Posición: 0 cm a 24 cm.
     - Restricción Adicional: La suma de las junturas no debe exceder los 24 metros.
     - Los valores del eje x y y se relacionan entre ellos, por lo que deben cumplir la relacion de no exceder el circulo definido por el radio de las junturas entre ellos

   - Eje z:
     - Altura Inicial del Marco de Referencia: 30 cm.
     - Restricción Adicional: La altura no debe ser inferior a 30 cm para evitar dañar el robot. Además, tanto el valor en el eje x como en el eje y no pueden ser inferiores a 5 cm, ya que esto también podría representar un riesgo para el robot.
     - 
