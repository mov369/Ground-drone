#include "Wire.h"              // for I2C
#include "sensorbar.h"         // needs SparkFun library
#include <HCSR04.h>

const byte triggerPin = 13;
const byte echoPin = 12;
const byte btriggerPin = 9;
const byte bechoPin = 8;

UltraSonicDistanceSensor distanceSensor(triggerPin, echoPin);

//Uncomment one of the four lines to match your SX1509's address
//pin selects. SX1509 breakout defaults to [0:0] (0x3E).
//const uint8_t SX1509_ADDRESS = 0x3E;  // SX1509 I2C address (00)
//const byte SX1509_ADDRESS = 0x3F;  // SX1509 I2C address (01)
//const byte SX1509_ADDRESS = 0x70;  // SX1509 I2C address (10)
//const byte SX1509_ADDRESS = 0x71;  // SX1509 I2C address (11)
//SensorBar mySensorBar(SX1509_ADDRESS);
// based on SparkFun MostBasicFollower code
// Define the states that the decision making machines uses:
#define IDLE_STATE 0
#define READ_LINE 1
#define GO_FORWARD 2
#define GO_LEFT 3
#define GO_RIGHT 4
#define GO_LEFT_2 5
#define GO_RIGHT_2 6

//STUFF FROM ARDUMOTO SETUP
// Depending on how you wired your motors, you may need to swap.
#define FORWARD  0
#define REVERSE 1

// Motor definitions to make life easier:
#define MOTOR_L 0
#define MOTOR_R 1
// Pin Assignments //
//Default pins, L and R Motors when looking from the back:
#define DIRL 2 // Direction control for motor A // was int Ldir = 4;
#define PWML 3  // PWM control (speed) for motor A // was int Lmotor = 5;
#define DIRR 4 // Direction control for motor B // was int Rdir = 7;
#define PWMR 11 // PWM control (speed) for motor B // was int Rmotor = 6;
uint8_t state;

void setup() {
  setupArdumoto();
  Serial.begin(9600);  // start serial for output
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(btriggerPin, OUTPUT);
  pinMode(bechoPin, INPUT);
  //pinMode(LED, OUTPUT);
  //pinMode(LED2, OUTPUT);
}

void loop() {
  //Serial.println(distanceSensor.measureDistanceCm());
  //delay(1500);
  //float distance = distanceSensor.measureDistanceCm();
  //Serial.print(distance);
  //Serial.println();

  int duration, distance;
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10000);
  digitalWrite(triggerPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;

  int bduration, bdistance;
  digitalWrite(btriggerPin, HIGH);
  delayMicroseconds(10000);
  digitalWrite(btriggerPin, LOW);
  bduration = pulseIn(bechoPin, HIGH);
  bdistance = (bduration/2) / 29.1;

  Serial.print("Front Ultrasound: ");
  Serial.print(distance);
  Serial.println(" cm");

  Serial.print("Side Ultrasound: ");
  Serial.print(bdistance);
  Serial.println(" cm");

  //digitalWrite(triggerPin,LOW);
  //delayMicroseconds(2);
  //digitalWrite(btriggerPin,HIGH);
  //delayMicroseconds(2);
  //digitalWrite(triggerPin,LOW);

  //long timedelay = pulseIn(echoPin,HIGH);
  //int distance1 = 0.0343 * (timedelay/2);
  //Serial.print("Sensor 1 : ");
  //Serial.println(distance1);

  //delayMicroseconds(2);

  //digitalWrite(btriggerPin,LOW);
  //delayMicroseconds(2);
  //digitalWrite(btriggerPin,HIGH);
  //delayMicroseconds(2);
  //digitalWrite(btriggerPin,LOW);

  //long td = pulseIn(bechoPin,HIGH);
  //int distance2 = 0.0343 * (td/2);

  //Serial.print("Sensor 2 : ");
  //Serial.println(distance2);
  

  uint8_t nextState = state;
  switch (state) {
  case IDLE_STATE:
    stopArdumoto(MOTOR_L);  // STOP motor A 
    stopArdumoto(MOTOR_R);  // STOP motor B
    nextState = READ_LINE;
    break;
  case READ_LINE:
    if(distance == 0){
    nextState = IDLE_STATE;
    break;
    }
    if(distance < 10){
      nextState = GO_FORWARD;
      delay(1000);
      //if( mySensorBar.getPosition() < -50 && mySensorBar.getPosition()> -97)
      //  {
      //  nextState = IDLE_STATE;//GO_LEFT;
      //  }
      //if( mySensorBar.getPosition() < -110)
      //  {
      //  nextState = IDLE_STATE;//GO_LEFT_2;
      //  }
      //if( mySensorBar.getPosition() > 50 && mySensorBar.getPosition() < 97 )
      //  {
      //  nextState = IDLE_STATE;//GO_RIGHT;
      //  }
      //if( mySensorBar.getPosition() > 110 )
      //  {
      //  nextState = IDLE_STATE;//GO_RIGHT_2;
      //  }
      }
      else{
        nextState = IDLE_STATE;
        }
      break;
    

  case GO_FORWARD:
    driveArdumoto(MOTOR_L, FORWARD, 150);  // Motor A at max speed.
    driveArdumoto(MOTOR_R, FORWARD, 150);  // Motor B at max speed.
    nextState = READ_LINE;
    break;
  case GO_LEFT:
    driveArdumoto(MOTOR_L, FORWARD, 0);  // Motor A at max speed.
    driveArdumoto(MOTOR_R, FORWARD, 150);  // Motor B at max speed.
    nextState = READ_LINE;
    break;
  case GO_LEFT_2:
    driveArdumoto(MOTOR_L, REVERSE, 150);  // Motor A at max speed.
    driveArdumoto(MOTOR_R, FORWARD, 150);  // Motor B at max speed.
    nextState = READ_LINE;
    break;
  case GO_RIGHT:
    driveArdumoto(MOTOR_R, FORWARD, 0);  // Motor B at max speed.
    driveArdumoto(MOTOR_L, FORWARD, 150);  // Motor A at max speed.
    
    nextState = READ_LINE;
    break;
  case GO_RIGHT_2:
    driveArdumoto(MOTOR_R, REVERSE, 150);  // Motor B at max speed.
    driveArdumoto(MOTOR_L, FORWARD, 150);  // Motor A at max speed.
    
    nextState = READ_LINE;
    break;
  default:
    stopArdumoto(MOTOR_L);  // STOP motor A 
    stopArdumoto(MOTOR_R);        // Stops both motors
    break;
  }
  state = nextState;
  }

/*else {
    driveArdumoto(MOTOR_R, REVERSE, 200);  // Motor B at max speed.
    driveArdumoto(MOTOR_L, FORWARD, 200);  // Motor A at max speed.
    delay(500);
    driveArdumoto(MOTOR_L, FORWARD, 200);  // Motor A at max speed.
    driveArdumoto(MOTOR_R, FORWARD, 200);  // Motor B at max speed.
    delay(1000);
    driveArdumoto(MOTOR_L, REVERSE, 200);  // Motor A at max speed.
    driveArdumoto(MOTOR_R, FORWARD, 200);  // Motor B at max speed.
    delay(500);
    driveArdumoto(MOTOR_L, FORWARD, 200);  // Motor A at max speed.
    driveArdumoto(MOTOR_R, FORWARD, 200);  // Motor B at max speed.
    delay(500);
    driveArdumoto(MOTOR_L, FORWARD, 0);  // Motor A at max speed.
    driveArdumoto(MOTOR_R, FORWARD, 200);  // Motor B at max speed.
    delay(500);
    driveArdumoto(MOTOR_L, FORWARD, 200);  // Motor A at max speed.
    driveArdumoto(MOTOR_R, FORWARD, 200);  // Motor B at max speed.
*/



// STUFF FROM THE ARDUMOTO DRIVER SETUP
// driveArdumoto([motor], [direction], [speed]) -- Drive [motor] 
// (0 for Left, 1 for Right) in [direction] (0 or 1) at a [speed] between 0 and 255. 
// It will spin until told to stop.
// driveArdumoto drives 'motor' in 'dir' direction at 'spd' speed
void driveArdumoto(byte motor, byte dir, byte spd){
  if (motor == MOTOR_L)
  {
    digitalWrite(DIRL, dir);
    analogWrite(PWML, spd);
  }
  else if (motor == MOTOR_R)
  {
    digitalWrite(DIRR, dir);
    analogWrite(PWMR, spd);
  }  
}

// stopArdumoto makes a motor stop
void stopArdumoto(byte motor){
  driveArdumoto(motor, 0, 0);
}

//  Setup the Ardumoto Shield pins.
void setupArdumoto(){
  //All pins should be setup as outputs:
  pinMode(PWML, OUTPUT);
  pinMode(PWMR, OUTPUT);
  pinMode(DIRL, OUTPUT);
  pinMode(DIRR, OUTPUT);

  //Initialize all pins as low:
  digitalWrite(PWML, LOW);
  digitalWrite(PWMR, LOW);
  digitalWrite(DIRL, LOW);
  digitalWrite(DIRR, LOW);
}