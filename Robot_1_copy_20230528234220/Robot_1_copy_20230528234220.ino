//Robot_1 - Line Following, Obstacle Avoiding Robot with PID Controller
/* ABLB
 * ActoBitty Line Bot 
 *
 * base robot Actobotics ActoBitty:
 * https://www.servocity.com/html/actobitty_2_wheel_robot_kit.html#.VthCuvkrI-U
 * 
 * microcontroller board DFRobot Romeo 2 (has built in motor controller):
 * http://www.dfrobot.com/wiki/index.php/Romeo_V2-All_in_one_Controller_(R3)_(SKU:DFR0225)
 *
 * mount panel for board:
 * http://www.thingiverse.com/thing:1377159
 * 
 * line follower array from Sparkfun:
 * https://github.com/sparkfun/Line_Follower_Array
 *  
 */

#include "Wire.h"              // for I2C
#include "sensorbar.h"         // needs SparkFun library

// Uncomment one of the four lines to match your SX1509's address
//  pin selects. SX1509 breakout defaults to [0:0] (0x3E).
const uint8_t SX1509_ADDRESS = 0x3E;  // SX1509 I2C address (00)
//const byte SX1509_ADDRESS = 0x3F;  // SX1509 I2C address (01)
//const byte SX1509_ADDRESS = 0x70;  // SX1509 I2C address (10)
//const byte SX1509_ADDRESS = 0x71;  // SX1509 I2C address (11)

SensorBar mySensorBar(SX1509_ADDRESS);

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

void setup() 
{
  Serial.begin(9600);  // start serial for output
  Serial.println("Program started.");
  Serial.println();

 //Default: the IR will only be turned on during reads.
  mySensorBar.setBarStrobe();
  //Other option: Command to run all the time
  //mySensorBar.clearBarStrobe();

  //Default: dark on light
  //mySensorBar.clearInvertBits();
  //Other option: light line on dark
  mySensorBar.setInvertBits();
  
  //Don't forget to call .begin() to get the bar ready.  This configures HW.
  uint8_t returnStatus = mySensorBar.begin();
 
  if(returnStatus)
  {
	  Serial.println("sx1509 IC communication OK");
  }
  else
  {
	  Serial.println("sx1509 IC communication FAILED!");
  }
  Serial.println();
  setupArdumoto(); //Set all pins as outputs and intialise as LOW 
  mySensorBar.begin();
}

void loop() {

  uint8_t nextState = state;
  switch (state) {
  case IDLE_STATE:
    stopArdumoto(MOTOR_L);  // STOP motor A 
    stopArdumoto(MOTOR_R);  // STOP motor B
    nextState = READ_LINE;
    break;
  case READ_LINE:
    //if( mySensorBar.getDensity() == 0)
    //{
    //  nextState = IDLE_STATE;
    //   break;
    //}
    if( mySensorBar.getDensity() > 0 )
    {
      nextState = GO_FORWARD;
      if( mySensorBar.getPosition() < -50 && mySensorBar.getPosition()> -111)
      {
        nextState = GO_LEFT;
      }
      if( mySensorBar.getPosition() < -111)
      {
        nextState = GO_LEFT_2;
      }
      if( mySensorBar.getPosition() > 50 && mySensorBar.getPosition() < 111 )
      {
        nextState = GO_RIGHT;
      }
      if( mySensorBar.getPosition() > 111 )
      {
        nextState = GO_RIGHT_2;
      }
    }
    else
    {
      nextState = IDLE_STATE;
    }
    break;
  case GO_FORWARD:
    driveArdumoto(MOTOR_L, FORWARD, 255);  // Motor A at max speed.
    driveArdumoto(MOTOR_R, FORWARD, 255);  // Motor B at max speed.
    nextState = READ_LINE;
    break;
  case GO_LEFT:
    driveArdumoto(MOTOR_L, FORWARD, 0);  // Motor A at max speed.
    driveArdumoto(MOTOR_R, FORWARD, 255);  // Motor B at max speed.
    nextState = READ_LINE;
    break;
  case GO_LEFT_2:
    driveArdumoto(MOTOR_L, REVERSE, 255);  // Motor A at max speed.
    driveArdumoto(MOTOR_R, FORWARD, 255);  // Motor B at max speed.
    nextState = READ_LINE;
    break;
  case GO_RIGHT:
    driveArdumoto(MOTOR_R, FORWARD, 0);  // Motor B at max speed.
    driveArdumoto(MOTOR_L, FORWARD, 255);  // Motor A at max speed.
    
    nextState = READ_LINE;
    break;
  case GO_RIGHT_2:
    driveArdumoto(MOTOR_R, REVERSE, 255);  // Motor B at max speed.
    driveArdumoto(MOTOR_L, FORWARD, 255);  // Motor A at max speed.
    
    nextState = READ_LINE;
    break;
  default:
    stopArdumoto(MOTOR_L);  // STOP motor A 
    stopArdumoto(MOTOR_R);        // Stops both motors
    break;
  }
  state = nextState;
  //delay(100);


}
// STUFF FROM THE ARDUMOTO DRIVER SETUP

// driveArdumoto([motor], [direction], [speed]) -- Drive [motor] 
// (0 for Left, 1 for Right) in [direction] (0 or 1) at a [speed] between 0 and 255. 
// It will spin until told to stop.
// driveArdumoto drives 'motor' in 'dir' direction at 'spd' speed
void driveArdumoto(byte motor, byte dir, byte spd)
{
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
void stopArdumoto(byte motor)
{
  driveArdumoto(motor, 0, 0);
}

void setupArdumoto()
{
   //  Setup the Ardumoto Shield pins.
   //  All pins should be setup as outputs:
  pinMode(PWML, OUTPUT);
  pinMode(PWMR, OUTPUT);
  pinMode(DIRL, OUTPUT);
  pinMode(DIRR, OUTPUT);

  // Initialize all pins as low:
  digitalWrite(PWML, LOW);
  digitalWrite(PWMR, LOW);
  digitalWrite(DIRL, LOW);
  digitalWrite(DIRR, LOW);
}
