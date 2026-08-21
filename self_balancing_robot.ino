#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define ENA 5
#define IN1 6
#define IN2 7
#define IN3 8
#define IN4 9
#define ENB 10

double Kp = 75;
double Ki = 300.0;
double Kd = 2.5;

double setpoint = 2.9; 

#define MOTOR_MIN_SPEED 50
#define MOTOR_MAX_SPEED 255
#define FALL_ANGLE 30.0

MPU6050 mpu;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];
float m_amp=1.0;
volatile bool mpuInterrupt = false;

void dmpDataReady() {
  mpuInterrupt = true;
}

double input, output;
double lastError = 0;
double integral = 0;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  stopMotors();

  Wire.begin();
  TWBR = 24;

  mpu.initialize();

  if (!mpu.testConnection()) {
    while (1);
  }

  devStatus = mpu.dmpInitialize();

  mpu.setXAccelOffset(751);
  mpu.setYAccelOffset(-461);
  mpu.setZAccelOffset(1170);
  mpu.setXGyroOffset(87);
  mpu.setYGyroOffset(63);
  mpu.setZGyroOffset(-5);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    while (1);
  }

  lastTime = millis();
}

void loop() {
  if (!dmpReady) return;

  while (!mpuInterrupt && fifoCount < packetSize) {}

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    return;
  }

  if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) {
      fifoCount = mpu.getFIFOCount();
    }

    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    input = ypr[1] * 180.0 / M_PI;

    if (abs(input - setpoint) > FALL_ANGLE) {
      stopMotors();
      integral = 0;
      return;
    }

    output = computePID(input);
    driveMotors(output);
  }
}

double computePID(double currentAngle) {
  unsigned long now = millis();
  double dt = (now - lastTime) / 1000.0;

  if (dt <= 0) dt = 0.001;

  lastTime = now;

  double error = currentAngle - setpoint;
 Serial.println(currentAngle);

  double P = Kp * error;

  integral += error * dt;
  integral = constrain(integral, -30.0, 30.0);

  double I = Ki * integral;

  double D = Kd * (error - lastError) / dt;
  lastError = error;

  return P + I + D;
}

void driveMotors(double pid) {
  int speed = abs(pid);

  if (speed < MOTOR_MIN_SPEED) {
    stopMotors();
    return;
  }

  speed = constrain(speed, MOTOR_MIN_SPEED, MOTOR_MAX_SPEED);

  if (pid > 0) {
    motorBackward(speed);
  } else {
    motorForward(speed);
  }
}

void motorForward(int speed) {
  float speed1=(m_amp*speed);
  speed1= constrain(speed1,-255,255);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed1);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speed);
}

void motorBackward(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed*m_amp);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}
