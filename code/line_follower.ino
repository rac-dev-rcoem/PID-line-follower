#include <QTRSensors.h>      // Include the QTRSensors library for handling the line sensors
#include <SparkFun_TB6612.h> // Include the SparkFun TB6612 motor driver library

QTRSensors qtr;                      // Create a QTRSensors object
const uint8_t SensorCount = 10;      // Number of sensors in the QTR sensor array
uint16_t sensorValues[SensorCount];  // Array to hold sensor readings

// PID constants
float Kp = 0.35; // Proportional constant
float Ki = 0;    // Integral constant
float Kd = 3.5;  // Derivative constant

// PID variables
int PL;             // Proportional term
int IL = 0;         // Integral term, initialized to 0
int DL;             // Derivative term
int lastError = 0;  // Last error, initialized to 0

// Speed constants
const uint8_t maxspeeda = 85;  // Maximum speed for motor A
const uint8_t maxspeedb = 85;  // Maximum speed for motor B
const uint8_t basespeeda = 105; // Base speed for motor A
const uint8_t basespeedb = 105; // Base speed for motor B

// Motor pins
#define AIN1 27 // Control pin 1 for motor A
#define BIN2 33 // Control pin 2 for motor B
#define AIN2 14 // Control pin 2 for motor A
#define BIN1 25 // Control pin 1 for motor B
#define PWMA 13 // PWM control pin for motor A
#define PWMB 32 // PWM control pin for motor B
#define STBY 26 // Standby pin for motor driver

const int offsetA = 1; // Offset for motor A (can be 1 or -1)
const int offsetB = 1; // Offset for motor B (can be 1 or -1)

// Motor objects
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY); // Motor A
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY); // Motor B

// Function to set motor speed
void mspeed(int posa, int posb) {
  motor1.drive(posa); // Set speed for motor A
  motor2.drive(posb); // Set speed for motor B
}

// PID control function
void PID_control() {
  uint16_t position = qtr.readLineBlack(sensorValues); // Read the line position from the sensor array
  int error = 4500 - position; // Calculate the error (4500 is the middle position for a 9-sensor array)

  // Calculate PID terms
  PL = error;              // Proportional term
  IL += error;             // Integral term (sum of errors)
  DL = error - lastError;  // Derivative term (change in error)
  lastError = error;       // Update last error

  int motorspeed = PL * Kp + IL * Ki + DL * Kd; // Calculate the overall motor speed adjustment

  // Calculate individual motor speeds
  int motorspeeda = basespeeda + motorspeed; // Speed for motor A
  int motorspeedb = basespeedb - motorspeed; // Speed for motor B

  // Clamping motor speeds to within allowed range
  if (motorspeeda > maxspeeda) {
    motorspeeda = maxspeeda; // Cap motor A speed at maxspeeda
  }
  if (motorspeedb > maxspeedb) {
    motorspeedb = maxspeedb; // Cap motor B speed at maxspeedb
  }
  if (motorspeeda < 0) {
    motorspeeda = 0; // Ensure motor A speed is not negative
  }
  if (motorspeedb < 0) {
    motorspeedb = 0; // Ensure motor B speed is not negative
  }

  mspeed(motorspeeda, motorspeedb); // Set the motor speeds
}

void setup() {
  qtr.setTypeRC(); // Set the QTR sensor type to RC (reflectance)
  qtr.setSensorPins((const uint8_t[]){23, 22, 21, 19, 18, 17, 5, 16, 4, 15}, SensorCount); // Define sensor pins

  // Initializing motor pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(35, INPUT); // Define pin 35 as input for switch 1
  pinMode(34, INPUT); // Define pin 34 as input for switch 2

  delay(300); // Small delay to allow the system to stabilize

  // Sensor calibration
  for (int i = 0; i <= 100; i++) {
    motor1.drive(80); // Drive motors in one direction
    motor2.drive(-80);
    qtr.calibrate(); // Calibrate the sensors
  }

  for (int i = 0; i <= 100; i++) {
    motor1.drive(-80); // Drive motors in the opposite direction
    motor2.drive(80);
    qtr.calibrate(); // Calibrate the sensors
  }

  motor1.brake(); // Stop motor A
  motor2.brake(); // Stop motor B
  delay(2); // Small delay

  // Waiting for switches to be in the correct state
  while (digitalRead(35) == HIGH); // Wait until switch 1 is pressed
  while (digitalRead(34) == LOW);  // Wait until switch 2 is released
}

void loop() {
  PID_control(); // Call the PID control function in the loop
}
