#include <Servo.h>
#include <Math.h>

// Definir Servos
Servo servo1; 
Servo servo2; 
Servo servo3; 

const int servo1Pin = 9;
const int servo2Pin = 10;
const int servo3Pin = 11;

float q1_deg = 0.0;  // q1 fijo en 0° para este ejemplo

// forwardKinematics: Calcula x,y,z dados q1_deg, q2_deg, q3_deg (en grados)
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

// moveServos: Ajusta servos según q2, q3 en el modelo
void moveServos(float q1_model, float q2_model, float q3_model) {
  float servo1_angle = q1_model;
  float servo2_angle = q2_model + 90.0;    
  float servo3_angle = 90.0 - q3_model;

  servo1_angle = constrain(servo1_angle, 0, 180);
  servo2_angle = constrain(servo2_angle, 0, 180);
  servo3_angle = constrain(servo3_angle, 0, 180);

  servo1.write(servo1_angle);
  servo2.write(servo2_angle);
  servo3.write(servo3_angle);
}

// Secuencia con incrementos de 45°
float sequence[][3] = {
    {0, -90, 90},
    {0, -90, 45},
    {0, -90, 0},
    {0, -90, -45},
    {0, -90, -90},
    {0, -45, 90},
    {0, -45, 45},
    {0, -45, 0},
    {0, -45, -45},
    {0, -45, -90},
    {0, 0, 90},
    {0, 0, 45},
    {0, 0, 0},
    {0, 0, -45},
    {0, 0, -90},
    {0, 45, 90},
    {0, 45, 45},
    {0, 45, 0},
    {0, 45, -45},
    {0, 45, -90},
    {0, 90, 90},
    {0, 90, 45},
    {0, 90, 0},
    {0, 90, -45},
    {0, 90, -90}
};

int num_steps = sizeof(sequence)/sizeof(sequence[0]);

void setup() {
  Serial.begin(9600);
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  servo3.attach(servo3Pin);

  Serial.println("x,y,z"); // Encabezado opcional

  // Posición inicial
  float x, y, z;
  q1_deg = 0; // q1 fijo, en este caso
  float q2_deg = sequence[0][1];
  float q3_deg = sequence[0][2];
  moveServos(q1_deg, q2_deg, q3_deg);
  forwardKinematics(q1_deg, q2_deg, q3_deg, x, y, z);
  Serial.print(x); Serial.print(","); Serial.print(y); Serial.print(","); Serial.println(z);
}

void loop() {
  // Iterar sobre la secuencia
  for (int i = 0; i < num_steps; i++) {
    float q1_deg = sequence[i][0];
    float q2_deg = sequence[i][1];
    float q3_deg = sequence[i][2];

    // Mover servos a la posición
    moveServos(q1_deg, q2_deg, q3_deg);

    // Calcular cinemática directa
    float x, y, z;
    forwardKinematics(q1_deg, q2_deg, q3_deg, x, y, z);

    // Imprimir x,y,z
    Serial.print(x); Serial.print(","); Serial.print(y); Serial.print(","); Serial.println(z);

    delay(1000); // Esperar 1s entre cada posición para que se vea el cambio en el plot
  }

}
