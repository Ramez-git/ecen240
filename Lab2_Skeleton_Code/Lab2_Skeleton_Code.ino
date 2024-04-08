/********************************************************************
  ECEN 240/301 Lab Code
  Light-Following Robot

  The approach of this code is to use an architectured that employs
  three different processes:
    Perception
    Planning
    Action

  By separating these processes, this allows one to focus on the
  individual elements needed to do these tasks that are general
  to most robotics.


  Version History
  1.1.3       11 January 2023   Creation by Dr. Mazzeo and TAs from 2022 version

 ********************************************************************/

/* These initial includes allow you to use necessary libraries for
your sensors and servos. */
#include "Arduino.h"

//
// Compiler defines: the compiler replaces each name with its assignment
// (These make your code so much more readable.)
//

/***********************************************************/
// Hardware pin definitions
// Replace the pin numbers with those you connect to your robot

// Button pins. These will be replaced with the photodiode variables in lab 5
#define BUTTON_1  A0     // Far left Button - Servo Up
#define BUTTON_2  A2     // Left middle button - Left Motor
#define BUTTON_3  A4     // Middle Button - Collision
#define BUTTON_4  A6     // Right middle button - Right Motor
#define BUTTON_5  A7     // Far right button - Servo Down
#include "Arduino.h"
#include <CapacitiveSensor.h>
#define CAP_SENSOR_SEND     9
#define CAP_SENSOR_RECEIVE  A1
#define CAP_SENSOR_SAMPLES 40
#define CAP_SENSOR_TAU_THRESHOLD 25


// LED pins (note that digital pins do not need "D" in front of them)
#define LED_1   6       // Far Left LED - Servo Up
#define H_BRIDGE_ENA   5       // Left Middle LED  - Left Motor
#define LED_3   4       // Middle LED - Collision
#define H_BRIDGE_ENB   3       // Right Middle LED - Right Motor
#define LED_5   2       // Far Right LED - Servo Down


// Motor enable pins - Lab 3
// These will replace LEDs 2 and 4

// Photodiode pins - Lab 5
// These will replace buttons 1, 2, 4, 5

// Capacitive sensor pins - Lab 4


// Ultrasonic sensor pin - Lab 6
// This will replace button 3 and LED 3 will no longer be needed

// Servo pin - Lab 6
// This will replace LEDs 1 and 5

/***********************************************************/
// Configuration parameter definitions
// Replace the parameters with those that are appropriate for your robot

// Voltage at which a button is considered to be pressed
#define BUTTON_THRESHOLD 2.5

// Voltage at which a photodiode voltage is considered to be present - Lab 5


// Number of samples that the capacitor sensor will use in a measurement - Lab 4


// Parameters for servo control as well as instantiation - Lab 6


// Parameters for ultrasonic sensor and instantiation - Lab 6


// Parameter to define when the ultrasonic sensor detects a collision - Lab 6



/***********************************************************/
// Defintions that allow one to set states
// Sensor state definitions
#define DETECTION_NO    0
#define DETECTION_YES   1

// Motor speed definitions - Lab 4

// Collision definitions
#define COLLISION_ON   1
#define COLLISION_OFF  0

// Driving direction definitions
#define DRIVE_STOP      0
#define DRIVE_LEFT      1
#define DRIVE_RIGHT     2
#define DRIVE_STRAIGHT  3

// Servo movement definitions
#define SERVO_MOVE_STOP 0
#define SERVO_MOVE_UP   1
#define SERVO_MOVE_DOWN 2


/***********************************************************/
// Global variables that define PERCEPTION and initialization

// Collision (using Definitions)
int SensedCollision = DETECTION_NO;

// Photodiode inputs (using Definitions) - The button represent the photodiodes for lab 2
int SensedLightRight = DETECTION_NO;
int SensedLightLeft = DETECTION_NO;
int SensedLightUp = DETECTION_NO;
int SensedLightDown = DETECTION_NO;

// Capacitive sensor input (using Definitions) - Lab 4
//int SensedCapacitiveTouch = DETECTION_NO;


/***********************************************************/
// Global variables that define ACTION and initialization

// Collision Actions (using Definitions)
int ActionCollision = COLLISION_OFF;

// Main motors Action (using Definitions)
int ActionRobotDrive = DRIVE_STOP;
// Add speed action in Lab 4

// Servo Action (using Definitions)
int ActionServoMove =  SERVO_MOVE_STOP;

/********************************************************************
  SETUP function - this gets executed at power up, or after a reset
 ********************************************************************/
void setup() {
  //Set up serial connection at 9600 Baud
  Serial.begin(9600);
  
  //Set up output pins
  pinMode(LED_1, OUTPUT);
  pinMode(H_BRIDGE_ENA, OUTPUT);
  pinMode(LED_3, OUTPUT);
  pinMode(H_BRIDGE_ENB, OUTPUT);
  pinMode(LED_5, OUTPUT);
  //Set up input pins
  pinMode(BUTTON_1, INPUT);
  pinMode(BUTTON_2, INPUT);
  pinMode(BUTTON_3, INPUT);
  pinMode(BUTTON_4, INPUT);
  pinMode(BUTTON_5, INPUT);

//set up leds for battary stats, resistors used are 120 and 150
pinMode(A0,INPUT);
pinMode(12,OUTPUT);
pinMode(11,OUTPUT);
pinMode(10,OUTPUT);
  pinMode(CAP_SENSOR_RECEIVE, INPUT);
  pinMode(CAP_SENSOR_SEND, OUTPUT);  
  //Set up servo - Lab 6

}

/********************************************************************
  Main LOOP function - this gets executed in an infinite loop until
  power off or reset. - Notice: PERCEPTION, PLANNING, ACTION
 ********************************************************************/
 float Motorspeed =0;
void loop() {
  // This DebugStateOutput flag can be used to easily turn on the
  // serial debugging to know what the robot is perceiving and what
  // actions the robot wants to take.
 static CapacitiveSensor sensor = CapacitiveSensor(CAP_SENSOR_SEND, CAP_SENSOR_RECEIVE);
 long tau =  sensor.capacitiveSensor(CAP_SENSOR_SAMPLES);
 if(tau>1500){
 if(Motorspeed ==0){
  Motorspeed =255*.33;
  delay(500);
 }
 else if(Motorspeed == (255*.33)){
  Motorspeed = 255*.66;
  delay(500);
 }
 else if(Motorspeed == (255*.66)){
  Motorspeed = 255;
  delay(500);
 }
 else if(Motorspeed ==255){
  Motorspeed = 0;
  delay(500);
 }}
 Serial.print("motorspeed is:");
 Serial.println(Motorspeed);
  getbattarystat();
  int DebugStateOutput = true; // Change false to true to debug
  RobotPerception(); // PERCEPTION
  if (DebugStateOutput) {
    Serial.print("Perception:");
    Serial.print(SensedLightUp);
    Serial.print(SensedLightLeft);
    Serial.print(SensedCollision);
    Serial.print(SensedLightRight); 
    Serial.print(SensedLightDown);
//    Serial.print(SensedCapacitiveTouch); - Lab 4
  }
  
  RobotPlanning(); // PLANNING
  if (DebugStateOutput) {
    Serial.print(" Action:");
    Serial.print(ActionCollision);
    Serial.print(ActionRobotDrive); 
//    Serial.print(ActionRobotSpeed); - Lab 4
    Serial.println(ActionServoMove);
  }
  RobotAction(); // ACTION
}

/**********************************************************************************************************
  Robot PERCEPTION - all of the sensing
 ********************************************************************/
void RobotPerception() {
  // This function polls all of the sensors and then assigns sensor outputs
  // that can be used by the robot in subsequent stages


  
  // Photodiode Sensing
  //Serial.println(getPinVoltage(BUTTON_2)); //uncomment for debugging
  
  if (is_LIGHT(BUTTON_2) &&is_LIGHT(BUTTON_4)){
    SensedLightLeft = DETECTION_YES;
    SensedLightRight=DETECTION_YES;
  }
  else{
  if (is_LIGHT(BUTTON_2)){
    SensedLightLeft = DETECTION_YES;
  } else {
    SensedLightLeft = DETECTION_NO;
  }
  // Remember, you can find the buttons and which one goes to what towards the top of the file
  if (is_LIGHT(BUTTON_4)) { 
    SensedLightRight = DETECTION_YES;
  } else {
    SensedLightRight = DETECTION_NO;
  }}


   if (is_LIGHT(BUTTON_1)){
    SensedLightUp = DETECTION_YES;
  } else {
    SensedLightUp = DETECTION_NO;
  }
  if (is_LIGHT(BUTTON_5)) { 
    SensedLightDown = DETECTION_YES;
  } else {
    SensedLightDown = DETECTION_NO;
  }

  

   // Capacitive Sensor
   /*Add code in lab 4*/

   // Collision Sensor
   if (isCollision()) {   // Add code in isCollision() function for lab 2 milestone 1
    SensedCollision = DETECTION_YES;
   } else {
    SensedCollision = DETECTION_NO;
   }
}


////////////////////////////////////////////////////////////////////
// Function to read pin voltage
////////////////////////////////////////////////////////////////////
float getPinVoltage(int pin) {
  //This function can be used for many different tasks in the labs
  //Study this line of code to understand what is going on!!
  //What does analogRead(pin) do?
  //Why is (float) needed?
  //Why divide by 1024?
  //Why multiply by 5?
  return 5 * (float)analogRead(pin) / 1024;
}

////////////////////////////////////////////////////////////////////
// Function to determine if a button is pushed or not
////////////////////////////////////////////////////////////////////
bool isButtonPushed(int button_pin) {
  //This function can be used to determine if a said button is pushed.
  //Remember that when the voltage is 0, it's only close to zero.
  //Hint: Call the getPinVoltage function and if that value is greater
  // than the BUTTON_THRESHOLD variable toward the top of the file, return true.
  if (analogRead(button_pin)<900){
    return true;
  } else {
    return false;
  }
}


////////////////////////////////////////////////////////////////////
// Function that detects if there is an obstacle in front of robot
////////////////////////////////////////////////////////////////////
bool isCollision() {
  //This is where you add code that tests if the collision button 
  // was pushed (BUTTON_3)
  //In lab 6 you will add a sonar sensor to detect collision and
  // the code for the sonar sensor will go in this function.
  // Until then we will use a button to model the sensor.
  if (analogRead(BUTTON_3)<900) {
    return true;
  } else {
    return false;
  }
}

////////////////////////////////////////////////////////////////////
// Function that detects if the capacitive sensor is being touched
////////////////////////////////////////////////////////////////////
bool isCapacitiveSensorTouched() {
  //In lab 4 you will add a capacitive sensor, and
  // you will need to modify this function accordingly.
}


/**********************************************************************************************************
  Robot PLANNING - using the sensing to make decisions
 **********************************************************************************************************/
void RobotPlanning(void) {
  // The planning FSMs that are used by the robot to assign actions
  // based on the sensing from the Perception stage.
  fsmCollisionDetection(); // Milestone 1
  fsmMoveServoUpAndDown(); // Milestone 3
  // Add Speed Control State Machine in lab 4
}

////////////////////////////////////////////////////////////////////
// State machine for detecting collisions, and stopping the robot
// if necessary.
////////////////////////////////////////////////////////////////////
void fsmCollisionDetection() {
  static int collisionDetectionState = COLLISION_OFF;
  //Serial.println(collisionDetectionState); //uncomment for debugging
  
  switch (collisionDetectionState) {
    case 1: //collision detected
      //There is an obstacle, stop the robot
      ActionCollision = COLLISION_ON; // Sets the action to turn on the collision LED
      /* Add code in milestone 2 to stop the robot's wheels - Hint: ActionRobotDrive = ________ */
      ActionRobotDrive=DRIVE_STOP;
      
      //State transition logic
      if ( SensedCollision == DETECTION_NO) {
        collisionDetectionState = COLLISION_OFF; //if no collision, go to no collision state
      }
      break;
    
    case 0: //no collision
      //There is no obstacle, drive the robot
      ActionCollision = COLLISION_OFF; // Sets action to turn off the collision LED

      fsmSteerRobot(); // Milestone 2
      
      //State transition logic
      if (SensedCollision == DETECTION_YES) {
        collisionDetectionState = COLLISION_ON; //if collision, go to collision state
      }
      break;

    default: // error handling
      {
        collisionDetectionState = COLLISION_OFF;
      }
      break;
  }
}
void doTurnLedOff(int ledpin){
  digitalWrite(ledpin,LOW);
}
void doTurnLedOn(int ledpin){
  digitalWrite(ledpin,HIGH);
}

////////////////////////////////////////////////////////////////////
// State machine for detecting if light is to the right or left,
// and steering the robot accordingly.
////////////////////////////////////////////////////////////////////
void fsmSteerRobot() {
  static int steerRobotState = DRIVE_STOP;
  //Serial.println(steerRobotState); //uncomment for debugging

  switch (steerRobotState) {
    case 0: //light is not detected
      ActionRobotDrive = DRIVE_STOP;
      
      //State transition logic
      if ( SensedLightLeft == DETECTION_YES && SensedLightRight != DETECTION_YES ) {
        steerRobotState = 1; //if light on left of robot, go to left state
      } else if ( SensedLightRight == DETECTION_YES && SensedLightLeft != DETECTION_YES ) {
        steerRobotState = 2; //if light on right of robot, go to right state
      }
      else if( SensedLightRight == DETECTION_YES && SensedLightLeft == DETECTION_YES ){
        steerRobotState =3;
      }
        
      break;
    
    case 1: //light is to the left of robot
      //The light is on the left, turn left
      ActionRobotDrive =DRIVE_LEFT;  //Add appropriate variable to set the action to turn left
      if ( SensedLightRight == DETECTION_YES && SensedLightLeft != DETECTION_YES ) {
        steerRobotState = 2; //if light on right of robot, go to right state
      }
      else if( SensedLightRight == DETECTION_YES && SensedLightLeft == DETECTION_YES ){
        steerRobotState =3;
      }
      else if ( SensedLightLeft != DETECTION_YES && SensedLightRight != DETECTION_YES ){
        steerRobotState =0;
      }
      break;
    
    case 2: //light is to the right of robot
      ActionRobotDrive =DRIVE_RIGHT; 
      
      if ( SensedLightRight != DETECTION_YES && SensedLightLeft == DETECTION_YES ) {
        steerRobotState = 1; //if light on right of robot, go to right state
      }
      else if( SensedLightRight == DETECTION_YES && SensedLightLeft == DETECTION_YES ){
        steerRobotState =3;
      }
      else if ( SensedLightLeft != DETECTION_YES && SensedLightRight != DETECTION_YES ){
        steerRobotState =0;
      }

      break;
      
    // light is on both right and left
    case 3:
    ActionRobotDrive=DRIVE_STRAIGHT;
  if ( SensedLightLeft != DETECTION_YES && SensedLightRight != DETECTION_YES ){
        steerRobotState =0;
      }
  else if ( SensedLightRight != DETECTION_YES && SensedLightLeft == DETECTION_YES ) {
        steerRobotState = 1; //if light on right of robot, go to right state
      }
  else if ( SensedLightRight == DETECTION_YES && SensedLightLeft != DETECTION_YES ) {
        steerRobotState = 2; //if light on right of robot, go to right state
      }  
      break;
      
    default: // error handling
    {
      steerRobotState =DRIVE_STOP;
    }
  }
}

////////////////////////////////////////////////////////////////////
// State machine for detecting if light is above or below center,
// and moving the servo accordingly.
////////////////////////////////////////////////////////////////////
void fsmMoveServoUpAndDown() {
  static int moveServoState = 0;
  //Serial.println(moveServoState); //uncomment for debugging
  switch(moveServoState){
    case 0://no light detected
    ActionServoMove=SERVO_MOVE_STOP;
    if(SensedLightUp && !SensedLightDown){
      moveServoState = 1;
    }
    else if(SensedLightDown && !SensedLightUp){
      moveServoState = 2;
      
    }
    break;
    case 1://light is up
    ActionServoMove=SERVO_MOVE_UP;
    if(SensedLightDown && !SensedLightUp){
      moveServoState = 2;
      
    }
    else if((!SensedLightDown && !SensedLightUp)||(SensedLightDown && SensedLightUp)){
      moveServoState = 0;
    }
    break;
    case 2://light is down
    ActionServoMove=SERVO_MOVE_DOWN;
    if(!SensedLightDown && SensedLightUp){
      moveServoState = 1;
    }
    else if((!SensedLightDown && !SensedLightUp)||(SensedLightDown && SensedLightUp)){
      moveServoState = 0;
    }
    break;
    default:
    moveServoState = 0;
    break;
  }
  
  // Milestone 3
  //Create a state machine modeled after the ones in milestones 1 and 2
  // to plan the servo action based off of the perception of the robot
  //Remember no light or light in front = servo doesn't move
  //Light above = servo moves up
  //Light below = servo moves down
  
}

////////////////////////////////////////////////////////////////////
// State machine for detecting when the capacitive sensor is
// touched, and changing the robot's speed.
////////////////////////////////////////////////////////////////////
void fsmCapacitiveSensorSpeedControl() {
  /*Implement in lab 4*/
}

////////////////////////////////////////////////////////////////////
// State machine for cycling through the robot's speeds.
////////////////////////////////////////////////////////////////////
void fsmChangeSpeed() {
  /*Implement in lab 4*/
}


/**********************************************************************************************************
  Robot ACTION - implementing the decisions from planning to specific actions
 ********************************************************************/
void RobotAction() {
  // Here the results of planning are implented so the robot does something
  
  // This turns the collision LED on and off
  switch(ActionCollision) {
    case COLLISION_OFF:
      doTurnLedOff(LED_3); //Collision LED off
                           
      break;
    case COLLISION_ON:
      doTurnLedOn(LED_3); 
      break;
  }
  
  // This drives the main motors on the robot
  switch(ActionRobotDrive) {
    case DRIVE_STOP:
      doTurnLedOff(H_BRIDGE_ENA);
      doTurnLedOff(H_BRIDGE_ENB);
      break;
    case DRIVE_LEFT:
      doTurnLedOff(H_BRIDGE_ENB);
      analogWrite(H_BRIDGE_ENA,Motorspeed);
      break;
    case DRIVE_RIGHT:
      analogWrite(H_BRIDGE_ENB,Motorspeed);
      doTurnLedOff(H_BRIDGE_ENA);
      break;
    case DRIVE_STRAIGHT:
    analogWrite(H_BRIDGE_ENB,Motorspeed);
    analogWrite(H_BRIDGE_ENA,Motorspeed);
      break;
  }
  
  // This calls a function to move the servo
    MoveServo();       
}


////////////////////////////////////////////////////////////////////
// Function that causes the servo to move up or down.
////////////////////////////////////////////////////////////////////
void MoveServo() {
  // Note that there needs to be some logic in the action of moving
  // the servo so that it does not exceed its range
  /* Add CurrentServoAngle in lab 6 */
  switch(ActionServoMove) {
    case SERVO_MOVE_STOP:
      doTurnLedOff(LED_5);
      doTurnLedOff(LED_1);
      break;
    case SERVO_MOVE_UP:
      doTurnLedOff(LED_5);
      doTurnLedOn(LED_1);
      break;
    case SERVO_MOVE_DOWN:
      doTurnLedOff(LED_1);
      doTurnLedOn(LED_5);
      break;
  }
}
void getbattarystat(){
double x = analogRead(A0)/204.6;
Serial.print("Here is the voltage:");
Serial.println(x);
if(x>4.5){
  digitalWrite(12,HIGH);
}
else{
  digitalWrite(12,LOW);
}
if(x>4){
  digitalWrite(11,HIGH);
}
else{
  digitalWrite(11,LOW);
}
if(x>3.5){
  digitalWrite(10,HIGH);
}
else{
  digitalWrite(10,LOW);
}
}

float computeTau(float measuredTime, float beta)
{
  float tau = 1;
  return(tau);
}

float pinVoltage(int pin)
{
  float v_pin = 5.0 * ( (float) analogRead(pin) / 1024.0 );
  return(v_pin);
}
float is_LIGHT(int pin){
if((analogRead(pin)*0.004887585532746823069403714565)>3.2){
  return true;
}
else{
  return false;
}
}
