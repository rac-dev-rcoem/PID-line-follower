
#include <QTRSensors.h> //Make sure to install the library

/*************************************************************************
* Sensor Array object initialisation 
*************************************************************************/
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

/*************************************************************************
* PID control system variables 
*************************************************************************/
float Kp = 0.15; //related to the proportional control term; 
              //change the value by trial-and-error (ex: 0.07).
              // Sweet spot: 0.06
float Ki = 0.01; //related to the integral control term; 
              //change the value by trial-and-error (ex: 0.0008).
float Kd = 0.8; //related to the derivative control term; 
              //change the value by trial-and-error (ex: 0.6).
int P;
int I;
int D;

/*************************************************************************
* Global variables
*************************************************************************/
int lastError = 0;
boolean onoff = false;

/*************************************************************************
* Motor speed variables (choose between 0 - no speed, and 255 - maximum speed)
*************************************************************************/
const uint8_t maxspeeda = 250;
const uint8_t maxspeedb = 250;
const uint8_t basespeeda = 240;
const uint8_t basespeedb = 240;

/*************************************************************************
* DRV8835 GPIO pins declaration
*************************************************************************/
// int mode = 8;
int aphase = 6;
int aenbl = 9;
int bphase = 3;
int benbl = 5;

/*************************************************************************
* Buttons pins declaration
*************************************************************************/
int buttoncalibrate = A6; //or pin A3
int buttonstart = A7;
uint16_t position;

void rotate_right();
void rotate_left();
void test_emmiter();

int arr[SensorCount];
int thresh = 500;
void toBinaryArray() {
  for (int i = 0; i < SensorCount; i++) {
    arr[i] = sensorValues[i] > thresh;
  }
}

/*************************************************************************
* Function Name: setup
**************************************************************************
* Summary:
* This is the setup function for the Arduino board. It first sets up the 
* pins for the sensor array and the motor driver. Then the user needs to 
* slide the sensors across the line for 10 seconds as they need to be 
* calibrated. 
* 
* Parameters:
*  none
* 
* Returns:
*  none
*************************************************************************/
void setup() {
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){8, A0, A1, A2, A3, A4, A5, 7}, SensorCount);
  Serial.begin(9600);
  // qtr.setEmitterPin(7);//LEDON PIN

  // pinMode(mode, OUTPUT);
  pinMode(aphase, OUTPUT);
  pinMode(aenbl, OUTPUT);
  pinMode(bphase, OUTPUT);
  pinMode(benbl, OUTPUT);

  pinMode(buttoncalibrate, INPUT);
  pinMode(buttonstart, INPUT); 
  // digitalWrite(mode, HIGH); //one of the two control interfaces 
                            //(simplified drive/brake operation)
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("Calibrating...");

  // calibration();

  boolean Ok = false;
  while (Ok == false) { // the main function won't start until the robot is calibrated
    if(analogRead(buttoncalibrate) > 700) {
      Serial.println("Started Calibrating");
      calibration(); //calibrate the robot for 10 seconds
      Ok = true;
    }
  }
  // test_emmiter();
  forward_brake(0, 0); //stop the motors
  Serial.println("Calibrated");

  while(analogRead(buttonstart) < 600) {}

  char mode;
}

/*************************************************************************
* Function Name: calibration
**************************************************************************
* Summary:
* This is the calibration function for the QTR-8RC Reflectance Sensor Array. 
* The function calls the method 'qtr.calibrate()' offered by the imported 
* library. For approx. 10 seconds, each of the 8 sensors will calibrate with
* readings from the track. 
* 
* Parameters:
*  none
* 
* Returns:
*  none
*************************************************************************/
void calibration() {
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 170; i++)
  {
    qtr.calibrate();
    if (i < 20 ||  i > 50 && i < 80 || i  > 110 && i < 140) {
      rotate_left();
    } else {
      rotate_right();
    }
  }
  digitalWrite(LED_BUILTIN, LOW);
}

// Code to rotate bot during calibration
int rotation_speed = 110;
void rotate_right() {
  analogWrite(aphase, rotation_speed + 40);
  analogWrite(bphase, 0);
  analogWrite(aenbl, 0);
  analogWrite(benbl, rotation_speed);
}

void rotate_left() {
  analogWrite(aphase, 0);
  analogWrite(bphase, rotation_speed);
  analogWrite(aenbl, rotation_speed + 40);
  analogWrite(benbl, 0);
}

/*************************************************************************
* Function Name: loop
**************************************************************************
* Summary:
* This is the main function of this application. When the start button is
* pressed, the robot will toggle between following the track and stopping.
* When following the track, the function calls the PID control method. 
* 
* Parameters:
*  none
* 
* Returns:
*  none
*************************************************************************/

void loop() {
  PID_control();
  // delay(500);

  // qtr.read(sensorValues);
  
  while (position == 0) {
    delay(200);
    position = qtr.readLineBlack(sensorValues);
    forward_brake(250, -250);
  } 
  while (position == 7000) {
    delay(200);
    position = qtr.readLineBlack(sensorValues);
    forward_brake(-250, 250);
  }

  

  // print the sensor values as numbers from 0 to 2500, where 0 means maximum
  // reflectance and 2500 means minimum reflectance
  // for debugging purpose
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('_');
    Serial.print(arr[i]);
    Serial.print('\t');
  }
  Serial.println();

  // Code to sense for start
  if(analogRead(buttonstart) > 700) {
    onoff =! onoff;
    if(onoff = true) {
      delay(1000);//a delay when the robot starts
    }
    else {
      delay(50);
    }
  }
  if (onoff == true) {
    PID_control();
  }
  else {
    forward_brake(0,0); //stop the motors
  }
}

/*************************************************************************
* Function Name: forward_brake
**************************************************************************
* Summary:
* This is the control interface function of the motor driver. As shown in
* the Pololu's documentation of the DRV8835 motor driver, when the MODE is 
* equal to 1 (the pin is set to output HIGH), the robot will go forward at
* the given speed specified by the parameters. The phase pins control the
* direction of the spin, and the enbl pins control the speed of the motor.
* 
* A warning though, depending on the wiring, you might need to change the 
* aphase and bphase from LOW to HIGH, in order for the robot to spin forward. 
* 
* Parameters:
*  int posa: int value from 0 to 255; controls the speed of the motor A.
*  int posb: int value from 0 to 255; controls the speed of the motor B.
* 
* Returns:
*  none
*************************************************************************/

void forward_brake(int posa, int posb) {
  //set the appropriate values for aphase and bphase so that the robot goes straight
  if (posa > 0) {
  digitalWrite(aphase, LOW);
  analogWrite(aenbl, posa);
  } else {
  digitalWrite(aenbl, LOW);
  analogWrite(aphase, -posa);
  }

  if (posb > 0) {
  digitalWrite(bphase, LOW);
  analogWrite(benbl, posb);
  } else {
  digitalWrite(benbl, LOW);
  analogWrite(bphase, -posb);
  }
}

/*************************************************************************
* Function Name: PID_control
**************************************************************************
* Summary: 
* This is the function of the PID control system. The distinguishing 
* feature of the PID controller is the ability to use the three control 
* terms of proportional, integral and derivative influence on the controller 
* output to apply accurate and optimal control. This correction is applied to
* the speed of the motors, which should be in range of the interval [0, max_speed],
* max_speed <= 255. 
* 
* Parameters:
* none
* 
* Returns:
*  none
*************************************************************************/

void PID_control() {
  position = qtr.readLineBlack(sensorValues); //read the current position
  int error = 3500 - position; //3500 is the ideal position (the centre)
  Serial.println(position);

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P*Kp + I*Ki + D*Kd; //calculate the correction
                                       //needed to be applied to the speed
  
  int motorspeeda = basespeeda + motorspeed;
  int motorspeedb = basespeedb - motorspeed;
  
  if (motorspeeda > maxspeeda) {
    motorspeeda = maxspeeda;
  }
  if (motorspeedb > maxspeedb) {
    motorspeedb = maxspeedb;
  }
  if (motorspeeda < 0) {
    motorspeeda = 0;
  }
  if (motorspeedb < 0) {
    motorspeedb = 0;
  } 
  forward_brake(motorspeeda, motorspeedb);
}