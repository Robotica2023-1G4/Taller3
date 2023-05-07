#include <Servo.h>

const int servoPin1 = 9;
const int servoPin2 = 10;
const int servoPin3 = 11;
const int servoPin4 = 12;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

void setup() {
  // Configurar los pines de los servos
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);
  servo4.attach(servoPin4);
  // Establecer posición inicial de los servos
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);
  // Inicializar comunicación serial
  Serial.begin(9600);
}

void loop() {
  // Esperar a recibir datos por el puerto serial
  if (Serial.available() > 0) {
    int movRot = Serial.parseInt();
    int movj1 = Serial.parseInt();
    int movj2 = Serial.parseInt();
    int movg = Serial.parseInt();

    // Leer la cadena de texto recibida por el puerto serial (Si algo descomentar esto y comentar lo de atras)
    //String message = Serial.readStringUntil('\n');
    //int velRot, velj1, velj2, velg;
    // Extraer valores de la cadena de texto
    //sscanf(message.c_str(), "%d,%d,%d,%d", &velRot, &velj1, &velj2, &velg);

    // mueve el servo 1
    int rotGrados = constrain(movRot, -90, 90); // limita el valor del servo1_pos al rango de -90 a 90 grados
    int newRot = servo1.read() + rotGrados; // lee la posición actual del servo y le suma los grados de movimiento
    newRot = constrain(newRot, 0, 180); // limita el nuevo valor al rango de 0 a 180 grados
    servo1.write(newRot); // mueve el servo a la nueva posición
    // mueve el servo 2
    int j1grados = constrain(movj1, -90, 90);
    int newJ1 = servo2.read() + j1grados;
    newJ1 = constrain(newJ1, 0, 180);
    servo2.write(newJ1);
    // mueve el servo 3
    int j2grados = constrain(movj2, -90, 90);
    int newJ2 = servo3.read() + j2grados;
    newJ2 = constrain(newJ2, 0, 180);
    servo3.write(newJ2)
    // mueve el servo de la garra
    int ggrados = constrain(movg, -90, 90);
    int newG = servo4.read() + ggrados;
    newG = constrain(newG, 0, 180);
    servo4.write(newG);

    //Convertir a grados negativos y positivos
    newRot = newRot - 90;
    newJ1 = newJ1 - 90;
    newJ2 = newJ2 - 90;
    newG = newG - 90;
    // Imprimir los valores de los servos
    Serial.print(newRot);
    Serial.print(",");
    Serial.print(newJ1);
    Serial.print(",");
    Serial.print(newJ2);
    Serial.print(",");
    Serial.println(newG);

  }               
}