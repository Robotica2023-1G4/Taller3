#include <Servo.h>

const int servoPin1 = 2;
const int servoPin2 = 3;
const int servoPin3 = 5;
const int servoPin4 = 4;

const int servoPin5 = 6;
const int servoPin6 = 7;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;

void setup() {
  // Configurar los pines de los servos
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);
  servo4.attach(servoPin4);
  servo5.attach(servoPin5);
  servo6.attach(servoPin6);
  // Incializa angulos
  servo1.write(100);
  servo2.write(0);
  servo3.write(0);
  servo4.write(90);
  servo5.write(20);
  servo6.write(170);


  // Inicializar comunicación serial
  Serial.begin(9600);
}

void loop() {
  
  // Esperar a recibir datos por el puerto serial
  if (Serial.available() > 0) {
    /*int movRot = Serial.parseInt();
    int movj1 = Serial.parseInt();
    int movj2 = Serial.parseInt();
    int movg = Serial.parseInt();*/

    // Leer la cadena de texto recibida por el puerto serial (Si algo descomentar esto y comentar lo de atras)
    String message = Serial.readStringUntil('\n');
    
    int movRot, movj1, movj2, movj3, movrotg, movg;
    // Extraer valores de la cadena de texto
    sscanf(message.c_str(), "%d,%d,%d,%d,%d,%d", &movRot, &movj1, &movj2, &movj3, &movrotg, &movg);
    Serial.println(movrotg);
    // mueve el servo 1
    if (movRot != 0) {
      int rotGrados = constrain(movRot, -90, 90); // limita el valor del servo1_pos al rango de -90 a 90 grados
      int newRot = servo1.read() + rotGrados; // lee la posición actual del servo y le suma los grados de movimiento
      newRot = constrain(newRot, 0, 180); // limita el nuevo valor al rango de 0 a 180 grados
      servo1.write(newRot); // mueve el servo a la nueva posición  
    }
    
    // mueve el servo 2
    if (movj1 != 0) {
      int j1grados = constrain(movj1, -90, 90);
      int newJ1 = servo2.read() + j1grados;
      newJ1 = constrain(newJ1, 0, 180);
      servo2.write(newJ1);  
    }
    
    
    // mueve el servo 3
    if (movj2 != 0) {
      int j2grados = constrain(movj2, -90, 90);
      int newJ2 = servo3.read() + j2grados;
      newJ2 = constrain(newJ2, 0, 180);
      servo3.write(newJ2);
    }

    // mueve el servo 4
    if (movj3 != 0) {
      int j3grados = constrain(movj3, -90, 90);
      int newJ3 = servo4.read() + j3grados;
      newJ3 = constrain(newJ3, 0, 180);
      servo4.write(newJ3);
    }

    // mueve el servo de rotacion de la garra
    if (movrotg != 0) {
      
      int rotgrados = constrain(movrotg, -90, 90);
      int newrotgrados = servo5.read() + rotgrados;
      newrotgrados = constrain(newrotgrados, 0, 180);
      
      servo5.write(newrotgrados);
    }
    
    // mueve el servo de la garra
    if (movg != 0) {
      
      int ggrados = constrain(movg, -90, 90);
      int newG = servo6.read() + ggrados;
      newG = constrain(newG, 0, 180);
      servo6.write(newG);  
    }
    

    //Convertir a grados negativos y positivos
    /*newRot = newRot - 90;
    newJ1 = newJ1 - 90;
    newJ2 = newJ2 - 90;
    newG = newG - 90;*/
    /*// Imprimir los valores de los servos*/
    /*Serial.print(newRot);
    Serial.print(",");
    Serial.print(newJ1);
    Serial.print(",");
    Serial.print(newJ2);
    Serial.print(",");
    Serial.println(newG);
    */
  }              
}
