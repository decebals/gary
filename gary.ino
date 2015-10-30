
// see http://letsmakerobots.com/blog/enigmerald/pid-tutorials-line-following
// see http://letsmakerobots.com/node/39972
#include <QTRSensors.h>

// set to 1 to print to serial monitor, 0 to disable
#define DEBUG 1

// experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kp 2 
// experiment to determine this, slowly increase the speeds and adjust this value. (Note: Kp < Kd) 
#define Kd 0.1

// max speed of the robot
#define rightMaxSpeed 250
// max speed of the robot
#define leftMaxSpeed 250
// this is the speed at which the motors should spin when the robot is perfectly on the line
#define rightBaseSpeed 240
// this is the speed at which the motors should spin when the robot is perfectly on the line 
#define leftBaseSpeed 240 

// number of sensors used
#define NUM_SENSORS  3

// waits for 2500 us for sensor outputs to go low
//#define TIMEOUT 2500
// emitter is controlled by digital pin 2
//#define EMITTER_PIN 2

// motors pins
#define leftMotor1 7
#define leftMotor2 6
#define leftMotorPWM 5
#define rightMotor1 4
#define rightMotor2 2
#define rightMotorPWM 3

int lastError = 0;

// sensor connected through digital pins 9-11
QTRSensorsRC qtrrc((unsigned char[]) { 9, 10, 11}, NUM_SENSORS);

unsigned int sensorValues[NUM_SENSORS];

void setup() {
  Serial.begin(9600);
      
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);

//  manualCalibration();
  autoCalibration();
  
  // wait for 2s to position the bot before entering the main loop 
  delay(2000);
    
  if (DEBUG) {
    debugCalibration();
  }
}

void loop() {
  // get calibrated readings along with the line position
  // refer to the QTR Sensors Arduino Library for more details on line position.
  int position = qtrrc.readLine(sensorValues);
  
  int error = position - 1000;

  int pid = Kp * error + Kd * (error - lastError);
  lastError = error;

  int rightMotorSpeed = rightBaseSpeed + pid;
  int leftMotorSpeed = leftBaseSpeed - pid;

  // prevent the motor from going beyond max speed
  if (rightMotorSpeed > rightMaxSpeed ) {
    rightMotorSpeed = rightMaxSpeed;
  }

  // prevent the motor from going beyond max speed
  if (leftMotorSpeed > leftMaxSpeed ) {
    leftMotorSpeed = leftMaxSpeed;
  }

  // keep the motor speed positive
  if (rightMotorSpeed < 0) {
    rightMotorSpeed = 0; 
  }

  // keep the motor speed positive
  if (leftMotorSpeed < 0) {
    leftMotorSpeed = 0;
  }
  
  // adjust left motor speed
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  analogWrite(leftMotorPWM, leftMotorSpeed);

  // adjust right motor speed
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(rightMotorPWM, rightMotorSpeed);
}

// calibrate for sometime by sliding the sensors across the line
void manualCalibration() { 
  for (int i = 0; i < 100; i++) {
    qtrrc.calibrate();
    delay(20);
  }
}

void autoCalibration() {
  for (int i = 0; i < 40; i++) {
    // turn to the left and right to expose the sensors to the brightest and darkest readings that may be encountered  
    if (i  < 10 || i >= 30) {
       turnRight();  
    } else {
       turnLeft();
    }
    qtrrc.calibrate();
    delay(20);  
  }

  stopMotors();
}

void debugCalibration() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();

  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
}

void turnLeft() {
  // move left motor backward
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  analogWrite(leftMotorPWM, 80);

  // move right motor forward
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, HIGH);
  analogWrite(rightMotorPWM, 75);
}

void turnRight() {
  // move left motor forward
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, HIGH);
  analogWrite(leftMotorPWM, 92);

  // move right motor backward
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(rightMotorPWM, 80);  
}

void stopMotors() {
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, LOW);
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, LOW);  
}

