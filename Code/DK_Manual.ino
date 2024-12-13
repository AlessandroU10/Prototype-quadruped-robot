#include <Servo.h>
#include <Math.h>

Servo servo1; // q1
Servo servo2; // q2 (fémur)
Servo servo3; // q3 (tibia)

const int servo1Pin = 9;
const int servo2Pin = 10;
const int servo3Pin = 11;

float q1_deg = 0.0;  // q1 = 0° modelo
float q2_deg = 0.0;  // Modelo: -90 a +90, 0 = 90° servo
float q3_deg = 0.0;  // Modelo: -90 a +90, 0 = 90° servo

void setup() {
  Serial.begin(9600);
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  servo3.attach(servo3Pin);

  moveServos(q1_deg, q2_deg, q3_deg);

  float x, y, z;
  forwardKinematics(q1_deg, q2_deg, q3_deg, x, y, z);

  // Imprimir cabezales (opcional, se pueden omitir)
  Serial.println("x,y,z");
  // Imprimir posición inicial
  Serial.print(x); Serial.print(","); Serial.print(y); Serial.print(","); Serial.println(z);
  // Indicar al usuario cómo ingresar datos:
  Serial.println("Ingrese q2,q3 en [-90,90], ejemplo: 0,0");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    int commaIndex = input.indexOf(',');

    if (commaIndex > 0) {
      String q2String = input.substring(0, commaIndex);
      String q3String = input.substring(commaIndex + 1);

      float q2Input = q2String.toFloat();
      float q3Input = q3String.toFloat();

      if (q2Input >= -90 && q2Input <= 90 && q3Input >= -90 && q3Input <= 90) {
        q2_deg = q2Input;
        q3_deg = q3Input;

        moveServos(q1_deg, q2_deg, q3_deg);

        float x, y, z;
        forwardKinematics(q1_deg, q2_deg, q3_deg, x, y, z);
        // Imprimir sólo valores x,y,z
        Serial.print(x); Serial.print(","); Serial.print(y); Serial.print(","); Serial.println(z);
      } else {
        Serial.println("Error: Angulos fuera de rango");
      }
    } else {
      Serial.println("Error: Formato incorrecto. Use q2,q3 (ej: 0,0)");
    }
  }
}

void forwardKinematics(float q1_deg, float q2_deg, float q3_deg, float &x, float &y, float &z) {
  float q1 = radians(q1_deg);
  float q2 = radians(q2_deg);
  float q3 = radians(q3_deg);

  float s1 = sin(q1);
  float c1 = cos(q1);
  float s2 = sin(q2);
  float c2 = cos(q2);
  float s23 = sin(q2 - q3);
  float c23 = cos(q2 - q3);

  x = 7.96*s1*s23 + 64.49*s1*c2 + 70.54*s1*c23 + 22.93*s1 + 25.06*c1;
  y = 25.06*s1 - 7.96*s23*c1 - 64.49*c1*c2 - 70.54*c1*c23 - 22.93*c1;
  z = -64.49*s2 - 70.54*s23 + 7.96*c23 + 24.25;
}

void moveServos(float q1_model, float q2_model, float q3_model) {
  // Mapear q2 y q3 al servo
  float servo2_angle = q2_model + 90.0;     // q2: 0° modelo→90° servo, +90° modelo→180°, -90°→0°
  float servo3_angle = 90.0 - q3_model;     // q3: 0° modelo→90° servo, +90°→0°, -90°→180°
  float servo1_angle = q1_model;            // q1 sin ajuste

  servo1_angle = constrain(servo1_angle, 0, 180);
  servo2_angle = constrain(servo2_angle, 0, 180);
  servo3_angle = constrain(servo3_angle, 0, 180);

  servo1.write(servo1_angle);
  servo2.write(servo2_angle);
  servo3.write(servo3_angle);
}
