/*  Line follower: QTR8 + TB6612 + PID
*/

#include <QTRSensors.h>
#include <SparkFun_TB6612.h>

#define AIN1 5
#define BIN1 7
#define AIN2 4
#define BIN2 8
#define PWMA 3
#define PWMB 9
#define STBY 6

const int offsetA = 1;
const int offsetB = 1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

const int max_speed = 225;   // max wheel command
int L = 0;
int R = 0;

float Kp = 0.0728; 
float Ki = 0.0090;  
float Kd = 0.6;

float P = 0, I = 0, D = 0;
float lastError = 0;
float adj = 0.0;

uint16_t position;

void setup() {
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  brake(motor1, motor2);

  Serial.begin(9600);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A0,A1,A2,A3,A4,A5,A6,A7 }, SensorCount);
  qtr.setEmitterPin(2);

  // Calibrate: move sensor across line during calibration if possible
  for (uint16_t i = 0; i < 200; i++) {
    qtr.calibrate();
    delay(5); // small delay helps calibration readings vary
  }

  Serial.println("Done");
}

void loop() {
  PID_control();
  delay(10); // keep loop timing stable (~100 Hz). Adjust if needed.
}

void PID_control() {
  position = qtr.readLineWhite(sensorValues);
  Serial.print("QTR::");
  Serial.println(position);

  // If line is lost (very left or very right), do recovery
  if (position < 100 || position > 6900) {
    // simple recovery: rotate toward last error sign
    if (lastError >= 0) { // last error positive => robot was right of center, turn left
      sharp_left();
      delay(80); // short turn
    } else {
      sharp_right();
      delay(80);
    }
    brake(motor1, motor2);
    // small reset to avoid large integrator build-up
    I = 0;
    return;
  }

  float error = 3500.0 - (float)position; // center = 3500 for 8 sensors

  P = error;
  I += error;
  I = constrain(I, -1000.0, 1000.0); // anti-windup

  D = error - lastError;
  lastError = error;

  adj = (Kp * P) + (Ki * I) + (Kd * D);

  float Lf = (float)max_speed + adj;
  float Rf = (float)max_speed - adj;

  L = (int)constrain(round(Lf), 0, max_speed);
  R = (int)constrain(round(Rf), 0, max_speed);

  forward(L, R);
}

void forward(int L, int R) {
  motor1.drive(L);
  motor2.drive(R);
}
void sharp_right() {
  motor1.drive(-200);
  motor2.drive(200);
}
void sharp_left() {
  motor1.drive(200);
  motor2.drive(-200);
}
