#include <Servo.h>

const int servoPin1 = 9;
const int servoPin2 = 10;
const int servoPin3 = 11;

Servo servo1;
Servo servo2;
Servo servo3;

void setup() {
  // Configurar los pines de los servos
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);
  // Establecer posición inicial de los servos
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  // Inicializar comunicación serial
  Serial.begin(9600);
}

void loop() {
  // Esperar a recibir datos por el puerto serial
  if (Serial.available() > 0) {
    int x_speed = Serial.parseInt(); // lee el valor de x_speed
    int y_speed = Serial.parseInt(); // lee el valor de y_speed
    int g_speed = Serial.parseInt(); // lee el valor de g_speed
    // mueve el servo 1
    int servo1_pos = constrain(x_speed, -90, 90); // limita el valor del servo1_pos al rango de -90 a 90 grados
    servo1.write(servo1_pos + 90); // agrega 90 grados al valor de servo1_pos para ajustar el rango a 0 a 180 grados y mueve el servo
    // mueve el servo 2
    int servo2_pos = constrain(y_speed, -90, 90); // limita el valor del servo2_pos al rango de -90 a 90 grados
    servo2.write(servo2_pos + 90); // agrega 90 grados al valor de servo2_pos para ajustar el rango a 0 a 180 grados y mueve el servo
    // mueve el servo de la garra
    int g_pos = constrain(g_speed, -90, 90); // limita el valor de g_pos al rango de -90 a 90 grados
    servo_g.write(g_pos + 90); // agrega 90 grados al valor de g_pos para ajustar el rango a 0 a 180 grados y mueve el servo de la garra
  }
}