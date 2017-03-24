/**
 * MIT License
 * 
 * Copyright (c) 2017 Jeferson Lima & Andressa Andrade
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
  
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */
/**
 * 
 * BotLib
 * BotLib is a all-in-one Arduino sketch that is inteed.
 *
 * To use this sketch install these libraries first:
 *
 *   * IRRemote
 *   * TimerOne
 *   * DigitalIO
 *
 * To install these libraries open Sketch -> Include Library -> Manage Libraries
 * select show all, and then search for the libraries, after install it, you
 * can use the code.
 *
 * P.S.: These sketch has several documented configuration that need to be changed
 * for each specific case, please don't forget this before open an issue or submit a
 * PL, if you have difficult and adapt this code for your reality feel free to send
 * us an e-mail.
 * 
 * Warning!: Don't forget to install the libraries used first
 *
 * modified 21 Mar 2017
 * by Jeferson Lima(@jefersonla)<jefersonlimaa@dcc.ufba.br> 
 * and Andressa Andrade(@dsandrade)<emaildedessa@hotmail.com>
 */

/* Libraries Used */
#include <math.h>
#include <stdint.h>
#include <PID_v1.h>
#include <DigitalIO.h>
#include <TimerOne.h>
#include <SoftwareSerial.h>

/* Robot setup routine */
#include "robot_setup.h"

/* System Utils */
#include "system_utils.h"

/***** USER CONFIG *****/

/* Enable control over serial */
#define SERIAL_CONTROL_ENABLED

/* Enable command using Ir Signal */
//#define IR_CONTROL_ENABLED

/* Enable serial log if you are using serial communication */
#define ENABLE_LOG

/* Enable log on software serial, the default is on main Serial */
//#define SOFTWARE_SERIAL_LOG

/* Enable display of lazy debug. Please don't use this on normal conditions */
//#define LAZY_DEBUG

/* Enable use of old and deprecated functions */
//#define USE_OBSOLETE

/* Enable this macro if you have a L298P with mirroed motor outputs */
#define MIRROED_MOTORS

/***********************/

/* Log Utilities */

#ifdef ENABLE_LOG

/* Start of a log */
#define LOG_LINE_START    "[log] >> "

/* Start of a debug message */
#define DEBUG_LINE_START  "[debug] >> "

/* Log serial object */
#ifndef SOFTWARE_SERIAL_LOG
#define LOG_OBJECT        Serial
#else
/* Create the object */
SoftwareSerial            serialLog(SOFTWARE_SERIAL_TX_PIN, SOFTWARE_SERIAL_RX_PIN);
#define LOG_OBJECT        serialLog
#endif

/* Print Log */
#define printLog(MSG)     LOG_OBJECT.print(F(LOG_LINE_START MSG))
#define printLogn(MSG)    printLog(MSG "\n")
#define printLogVar(VAR)  LOG_OBJECT.print(VAR)
#define printLogVarn(VAR) LOG_OBJECT.println(VAR)

/* Print Debug */
#define printDebug(MSG)   LOG_OBJECT.print(F(DEBUG_LINE_START MSG))
#define printDebugn(MSG)  printDebug(MSG "\n")

/* Print Progmem */
#define printMem(MSG)     LOG_OBJECT.print(F(MSG))
#define printMemn(MSG)    LOG_OBJECT.println(F(MSG))

#endif

/* IRremote Control */

#ifdef IR_CONTROL_ENABLED
#include <IRremote.h>

/* IR Receiver */
IRrecv ir_receiver(RECEIVER_PIN);

/* IR Receiver Decoder */
decode_results ir_result;

#endif

/* Button State */
unsigned long button_state;

/* Actual rotation */
int actual_rotation;

/* Step Angle */
int step_angle;

/* Step Length */
int step_length;

/* Rotation length */
int rotation_length;

/* Distance wanted */
volatile unsigned int distance_wanted_left;
volatile unsigned int distance_wanted_right;

/* Motor speed in steps per interval */
int motor_speed_steps;

/* Pwm speed for each motor */
double pwm_motor_speed_left;
double pwm_motor_speed_right;

/* Distance read by each motor */
volatile int steps_reach_motor_left;
volatile int steps_reach_motor_right;

/* Number of steps completed */
volatile uint8_t steps_motor_left;
volatile uint8_t steps_motor_right;

/* Number of rotations completed */
volatile unsigned long rotations_motor_left;
volatile unsigned long rotations_motor_right;

/* Number of ticks on each motor, used to tune adjust */
double input_steps_motor_left;
double input_steps_motor_right;

/* Routine Flag Lock */
mutex routine_flag_mutex_lock_left;
mutex routine_flag_mutex_lock_right;

/* PID Constants */
volatile double pid_kp = DEFAULT_KP;
volatile double pid_ki = DEFAULT_KI;
volatile double pid_kd = DEFAULT_KD;

/* PID Speed wanted */
double setpoint_steps_speed_left = 0;
double setpoint_steps_speed_right = 0;

/* PID Object */
PID motor_left_pid(&input_steps_motor_left, &pwm_motor_speed_left, &setpoint_steps_speed_left, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DIRECT);
PID motor_right_pid(&input_steps_motor_right, &pwm_motor_speed_right, &setpoint_steps_speed_right, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DIRECT);

/* ::::::::::::::::::: Configure the robot ::::::::::::::::::: */

void setup(){
  #ifdef ENABLE_LOG
  /* Initialize serial communication */
  LOG_OBJECT.begin(SERIAL_SPEED);
  #endif

  #ifdef ENABLE_LOG
  /* Print Hello Message */
  printMemn("...::: Hi, I'm Armadillomon :) ! :::...\n"
            "By Jeferson Lima & Andressa Andrade\n"
            "Starting System...\n" );
  #endif

  /* Print which user configurations are enabled */
  #ifdef ENABLE_LOG
  printMemn("Checking user settings:");
  
  /* Use of Serial Control */
  #ifdef SERIAL_CONTROL_ENABLED
  printMemn("SERIAL_CONTROL = ENABLED");
  #else
  printMemn("SERIAL_CONTROL = DISABLED");
  #endif
  
  /* Use of Control by IR */
  #ifdef IR_CONTROL_ENABLED
  printMemn("IR_CONTROL = ENABLED");
  #else
  printMemn("IR_CONTROL = DISABLED");
  #endif
  
  /* Log on Software Serial */
  #ifdef SOFTWARE_SERIAL_LOG
  printMemn("SOFTWARE_SERIAL_LOG = ENABLED");
  #else
  printMemn("SOFTWARE_SERIAL_LOG = DISABLED");
  #endif
  
  /* Lazy Debug */
  #ifdef LAZY_DEBUG
  printMemn("LAZY_DEBUG = ENABLED");
  #else
  printMemn("LAZY_DEBUG = DISABLED");
  #endif
  
  /* Use of obsolete or deprecated functions */
  #ifdef USE_OBSOLETE
  printMemn("USE_OBSOLETE = ENABLED");
  #else
  printMemn("USE_OBSOLETE = DISABLED");
  #endif
  
  /* Mirroed output motors pins */ 
  #ifdef MIRROED_MOTORS
  printMemn("MIRROED_MOTORS = ENABLED");
  #else
  printMemn("MIRROED_MOTORS = DISABLED");
  #endif

  printMemn("");
    
  #endif
  
  /* Configure all motors related pins as output */
  fastPinMode(IN1_MOTOR_LEFT, OUTPUT);
  fastPinMode(IN2_MOTOR_LEFT, OUTPUT);
  fastPinMode(IN1_MOTOR_RIGHT, OUTPUT);
  fastPinMode(IN2_MOTOR_RIGHT, OUTPUT);
  fastPinMode(SPEED_MOTOR_LEFT, OUTPUT);
  fastPinMode(SPEED_MOTOR_RIGHT, OUTPUT);

  /* Configure Interruption pins as INPUT with internal PULLUP ressitors */
  pinMode(ENCODER_LEFT_PIN, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_PIN, INPUT_PULLUP);

  #ifdef ENABLE_LOG
  /* Configurated all the pins */
  printLogn("Configurated all pin modes");
  #endif

  #ifdef IR_CONTROL_ENABLED
  /* Enable IR Receiver */
  ir_receiver.enableIRIn();
  #endif

  /* Initialize actual cm/s virtual speed */
  motor_speed_steps = 0;

  /* Initialize distance wanted by user */
  distance_wanted_left = 0;
  distance_wanted_right = 0;
  
  /* Initialize distance rech by each motor */
  steps_reach_motor_left = 0;
  steps_reach_motor_right = 0;

  /* Initialize step counters */
  steps_motor_left = 0;
  steps_motor_right = 0;

  /* Initiliaze rotation counters */
  rotations_motor_left = 0;
  rotations_motor_right = 0;

  /* Initialize actual rotation */
  actual_rotation = 0;

  /* Initialize pwm speed */
  pwm_motor_speed_left = 0;
  pwm_motor_speed_right = 0;

  /* Initialize counter of steps per motor adjust */
  input_steps_motor_left = 0;
  input_steps_motor_right = 0;

  /* Initialize speed wanted in each motor */
  setpoint_steps_speed_left = 0;
  setpoint_steps_speed_right = 0;

  /* Blocking mutex */
  routine_flag_mutex_lock_left = false;
  routine_flag_mutex_lock_right = false;
  
  #ifdef ENABLE_LOG
  /* Initialized all variables */
  printLogn("Initialized all variables");
  #endif

  /* Initialize step angle */
  step_angle = ceil(360.0 / ENCODER_STEPS);

  #ifdef ENABLE_LOG
  /* Calculated step angle */
  printLogn("Calculated step angle");
  #endif

  /* Initialize step length */
  step_length = ceil((step_angle * PI * (WHEEL_DIAMETER_MM / 2.0)) / 180.0);

  #ifdef ENABLE_LOG
  /* Calculated step length */
  printLogn("Calculated step length");
  #endif

  /* Initialize Rotation Length */
  rotation_length = ceil(PI * WHEEL_DIAMETER_MM);

  #ifdef ENABLE_LOG
  /* Calculated rotation length*/
  printLogn("Calculated rotation length");
  #endif

  /* Attach interrupts of the encoders */
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), stepCounterMotorLeft, ENCODER_INTERRUPTION_TYPE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), stepCounterMotorRight, ENCODER_INTERRUPTION_TYPE);
  
  #ifdef ENABLE_LOG
  /* Attached the encoder interrupts */
  printLogn("Attached the encoder interrupts");
  #endif

  /* Set motor default state */
  brake();

  #ifdef ENABLE_LOG
  /* Brake motor */
  printLogn("Brake motor");
  #endif
  
  /* Start timer to adjust Motors in 1/{ADJUST_MOTOR_INTERVAL}S  */
  Timer1.initialize(TIME_1S_US / ADJUST_MOTOR_INTERVAL);
  Timer1.attachInterrupt(adjustMotors); 

  #ifdef ENABLE_LOG
  /* Start adjust of motors to desired virtual cm/s speed */
  printLogn("Motors adjust timer started");
  #endif

  /* Configure PID */
  motor_left_pid.SetMode(AUTOMATIC);
  motor_right_pid.SetMode(AUTOMATIC);

  #ifdef ENABLE_LOG
  /* Start PID Controller for each motor on automatic mode */
  printLogn("PID Mode setted on automatic");
  #endif

  #ifdef ENABLE_LOG
  /* Print end of setup */
  printMemn("\nSystem Started Successfully!\n");
  #endif
}

/* ::::::::::::::::::: Execute System code ::::::::::::::::::: */

void loop(){

#ifdef IR_CONTROL_ENABLED
  /* Read IR Signal */
  readIRSignal();
#endif

#ifdef SERIAL_CONTROL_ENABLED
  /* Read Serial commands */
  readSerialData();
#endif

  /* Move 1000mm - 100cm - 1m */
  moveAhead(1000);
  //while(true);
  //delay(30000);
  //unsigned int distance_wanted = ceil((double) abs(1000) / step_length);
  //distance_wanted_left = distance_wanted;
  //distance_wanted_right = distance_wanted;
  //turnForward();
  //acelerateMotors(80);
  delay(10000);
  brake();
  delay(5000);
}

/* :::::::::::::::::::       Helpers       ::::::::::::::::::: */

/* Increment counter of the motor left */
void stepCounterMotorLeft(){
  /* Number of steps performed by motor left */
  steps_motor_left++;  
  /* Number of steps as input for PID */
  input_steps_motor_left += 1;
  /* Total number of steps reached by motor left */
  steps_reach_motor_left++;
  
  /* Brake motors if we had reached the distance wanted */
  if(steps_reach_motor_left >= distance_wanted_left){
    /* Brake this motor */
    brakeMotor(MOTOR_LEFT);
    /* Zero speed of pwm output */
    zeroPwmSpeedSide(left);
    /* And set the speed wanted to 0 (fading stop) */
    setpoint_steps_speed_left = 0;
    /* Zero number of steps reach by this motor */
    steps_reach_motor_left = 0;
    /* Remove mutex */
    routine_flag_mutex_lock_left = false;
    /* Zero distance Wanted */
    distance_wanted_left = 0;

    #ifdef ENABLE_LOG
    printDebugn("Stopped Motor Left");
    #endif
  }

    /* Increase rotation if we had completed one */
  if(steps_motor_left == ENCODER_STEPS){
    steps_motor_left = 0;
    rotations_motor_left++;
  }

  /* Show that a interruption has occured on lazy debug */
  #if defined(ENABLE_LOG) && defined(LAZY_DEBUG)
  printDebug("Interruption Ocurred - Left Side - #");
  printLogVar(rotations_motor_left);
  printMem(" $");
  printLogVarn(steps_motor_left);
  #endif
}

/* Increment counter of the motor right */
void stepCounterMotorRight(){
  /* Number of steps performed by motor right */
  steps_motor_right++;
  /* Number of steps as input for PID */
  input_steps_motor_right += 1;
  /* Total number of steps reached by motor right */
  steps_reach_motor_right++;

  /* Brake motor right if we had reached the distance wanted */
  if(steps_reach_motor_right >= distance_wanted_right){  
    /* Brake this motor */
    brakeMotor(MOTOR_RIGHT);
    /* Zero speed of pwm output */
    zeroPwmSpeedSide(right);
    /* And set the speed wanted to 0 (fading stop) */
    setpoint_steps_speed_right = 0;
    /* Zero number of steps reach by this motor */
    steps_reach_motor_right = 0;
    /* Remove mutex */
    routine_flag_mutex_lock_right = false;
    /* Zero distance Wanted */
    distance_wanted_right = 0;
    
    #ifdef ENABLE_LOG
    printDebugn("Stopped Motor Right");
    #endif
  }
  
  /* Increase rotation if we had completed one */
  if(steps_motor_right == ENCODER_STEPS){
    steps_motor_right = 0;
    rotations_motor_right++;
  }
  
  /* Show that a interruption has occured on lazy debug */
  #if defined(ENABLE_LOG) && defined(LAZY_DEBUG)
  printDebug("Interruption Ocurred - Right Side - #");
  printLogVar(rotations_motor_right);
  printMem(" $");
  printLogVarn(steps_motor_right);
  #endif
}

/* Adjust desired speed to pwm */
void adjustMotors(){
  /* Compute new PID values */
  motor_left_pid.Compute();
  motor_right_pid.Compute();

  /* Adjust pwm output of each motor */
  analogWrite(SPEED_MOTOR_LEFT, pwm_motor_speed_left);
  analogWrite(SPEED_MOTOR_RIGHT, pwm_motor_speed_right);

  /* Clean values for next interation */
  input_steps_motor_left = 0;
  input_steps_motor_right = 0;

  #ifdef ENABLE_LOG
  printLog("PWM (L|R) (");
  printLogVar(ceil(pwm_motor_speed_left));
  printMem(" | ");
  printLogVar(ceil(pwm_motor_speed_right));
  printMemn(" )");
  #endif
}

/* Move robot to an specific angle */
void rotateRobot(int16_t desired_angle){
  
  /* Always move in clockwise direction if the direction wanted is positive otherwise move in anticlockwise */
  if(desired_angle > 0){
    turnRight(); 
  }
  else{
    turnLeft();
  }
  unsigned int distance_wanted;

  /* While we wanted a rotation greater than 360, we increase distance wanted by number of steps */
  while(desired_angle >= 180){
    distance_wanted += ceil(rotation_length/(step_length*2));
    desired_angle -= 180;
  }
 
  /* Now we calc the wanted distance, and to ensure precision distance is defined in steps */
  distance_wanted += ceil((desired_angle * PI * (ROBOT_WIDTH_MM / 2.0)) / 180.0);
  distance_wanted_left = distance_wanted;
  distance_wanted_right = distance_wanted;

  /* Start motor parallel aceleration */
  parallelAceleration();
}

/* Move ahead for some distance, if distance is negative move reverse */
void moveAhead(int distance){

  /* Move forward if distance is greater than 0 and backward otherwise */
  if(distance > 0){
    turnForward();
  }
  else{
    turnReverse();
  }
  
  /* Now we calc the wanted distance, and to ensure precision distance is defined in steps */
  unsigned int distance_wanted = ceil((double) abs(distance) / step_length);
  distance_wanted_left = distance_wanted;
  distance_wanted_right = distance_wanted;
  
  /* Lock Mutex */
  routine_flag_mutex_lock_left = true;
  routine_flag_mutex_lock_right = true;

  /* Start motor parallel aceleration */
  parallelAceleration();

  /* Wait until it finishes the thread */
  await(routine_flag_mutex_lock_left || routine_flag_mutex_lock_right);
}

/* Parallel aceleratio of wheels */
void parallelAcelerationMotor(int motor_num){

  /* Define setpoint speed for each motor based on number of steps by second divided by motor adjust interval */
  switch(motor_num){
    case MOTOR_LEFT:
      setpoint_steps_speed_left = ceil(STEP_SPEED / ADJUST_MOTOR_INTERVAL);
      break;
    case MOTOR_RIGHT:
      setpoint_steps_speed_right = ceil(STEP_SPEED / ADJUST_MOTOR_INTERVAL);
      break;
    case BOTH_MOTORS:
      setpoint_steps_speed_left = ceil(STEP_SPEED / ADJUST_MOTOR_INTERVAL);
      setpoint_steps_speed_right = ceil(STEP_SPEED / ADJUST_MOTOR_INTERVAL);
      break;
  }
}

/* Check Obstacles */
void checkObstacles(){
  // TODO
}

/* Read Serial data */
#ifdef SERIAL_CONTROL_ENABLED
void readSerialData(){

  /* If there are serial incoming data */
  if(Serial.available() > 0){
    /* Read serial and execute command */
    char serial_data = Serial.read();

    /* If we have special commands */
    if((serial_data == STEP_COMMAND || serial_data == TURN_COMMAND)){
      char serial_data2 = Serial.read();
      switch(serial_data2){
        case LEFT_DIRECTION:
          serial_data = (serial_data == STEP_COMMAND) ? '1' : '3';
        case RIGHT_DIRECTION:
          serial_data = (serial_data == STEP_COMMAND) ? '2' : '4';
        default:
          #ifdef ENABLE_LOG
          printLog("Invalid char read : ");
          printLogVarn(serial_data);
          #endif
          serial_data = '0';
      }
    }

    /* Execute the command received, if it's not an special character */
    if(serial_data != '\r' && serial_data != '\n' && serial_data != '\t'){
      executeCommand((unsigned long) serial_data);
    }
  }
}
#endif

#ifdef IR_CONTROL_ENABLED
/* Read Ir signal */
void readIRSignal(){

  /* Verify if there are Ir signal */
  if(ir_receiver.decode(&ir_result)){

    /* Get the button pressed */
    button_state = ir_result.value;

    /* Execut the input command */
    executeCommand(button_state);

    /* Flush the ir receiver buffer */
    ir_receiver.resume();
  }
}
#endif

/* Change Motor State */
void executeCommand(unsigned long motor_command){
  
  /* Check the command and change the direction state */
  switch(motor_command){
    case ACELERATE_COMMAND:
      acelerateMotors(PWM_MEDIUM_SPEED);
      #ifdef ENABLE_LOG
      printLogn("Pressed acelerator");
      #endif
      break;
    case FORWARD_BUTTON_IR:
      acelerateMotors(PWM_MEDIUM_SPEED);    
    case FORWARD_COMMAND:
      turnForward();
      #ifdef ENABLE_LOG
      printLogn("Pressed Forward");
      #endif
      break;
    case REVERSE_BUTTON_IR:
      acelerateMotors(PWM_MEDIUM_SPEED);
    case REVERSE_COMMAND:
      turnReverse();
      #ifdef ENABLE_LOG
      printLogn("Pressed Reverse");
      #endif
      break;
    case LEFT_BUTTON_IR:
    case STEP_LEFT_COMMAND:
      stepLeft();
      #ifdef ENABLE_LOG
      printLogn("Pressed Step Left");
      #endif
      break;
    case RIGHT_BUTTON_IR:
    case STEP_RIGHT_COMMAND:
      stepRight();
      #ifdef ENABLE_LOG
      printLogn("Pressed Step Right");
      #endif
      break;
    case TURN_LEFT_COMMAND:
      turnLeft();
      #ifdef ENABLE_LOG
      printLogn("Pressed Turn Left");
      #endif
      break;
    case TURN_RIGHT_COMMAND:
      turnRight();
      #ifdef ENABLE_LOG
      printLogn("Pressed Turn Right");
      #endif
      break;
    case BRAKE_BUTTON_IR:
    case BRAKE_COMMAND:
      brake();
      #ifdef ENABLE_LOG
      printLogn("Pressed Brake");
      #endif
      break;
    case DUMP_COMMAND:
      #ifdef ENABLE_LOG
      dumpInfo();
      #endif
      break;
    default:
      #ifdef ENABLE_LOG
      printLogn("INVALID COMMAND!");
      printLog("Command Received: ");
      if(motor_command < 255){
        LOG_OBJECT.print((char)motor_command);
        LOG_OBJECT.print('\n');
      }
      else{
        printLogVarn(motor_command);
      }
      #endif
  }
}

/* Acelerate a given motor */
void acelerateMotor(int motor_num, int pwm_motor_speed){
  switch(motor_num){
    case MOTOR_LEFT:
      pwm_motor_speed_left = pwm_motor_speed;
      analogWrite(SPEED_MOTOR_LEFT, pwm_motor_speed_left);
      break;
    case MOTOR_RIGHT:
      pwm_motor_speed_right = pwm_motor_speed;
      analogWrite(SPEED_MOTOR_RIGHT, pwm_motor_speed_right);
      break;
    case BOTH_MOTORS:
      pwm_motor_speed_left = pwm_motor_speed;
      pwm_motor_speed_right = pwm_motor_speed;
      analogWrite(SPEED_MOTOR_LEFT, pwm_motor_speed_left);
      analogWrite(SPEED_MOTOR_RIGHT, pwm_motor_speed_right);
      break;
  }
}

/* Change state of enable pin */
void changeMotorState(int motor_num, bool in1_motor_state, bool in2_motor_state){
  switch(motor_num){
    case MOTOR_LEFT:
      fastDigitalWrite(IN1_MOTOR_LEFT, in1_motor_state);
      fastDigitalWrite(IN2_MOTOR_LEFT, in2_motor_state);
      break;
    case MOTOR_RIGHT:
      #ifdef MIRROED_MOTORS
      fastDigitalWrite(IN1_MOTOR_RIGHT, in2_motor_state);
      fastDigitalWrite(IN2_MOTOR_RIGHT, in1_motor_state);
      #else
      fastDigitalWrite(IN1_MOTOR_RIGHT, in1_motor_state);
      fastDigitalWrite(IN2_MOTOR_RIGHT, in2_motor_state);
      #endif
      break;
    case BOTH_MOTORS:
      fastDigitalWrite(IN1_MOTOR_LEFT, in1_motor_state);
      fastDigitalWrite(IN2_MOTOR_LEFT, in2_motor_state);
      #ifdef MIRROED_MOTORS
      fastDigitalWrite(IN1_MOTOR_RIGHT, in2_motor_state);
      fastDigitalWrite(IN2_MOTOR_RIGHT, in1_motor_state);
      #else
      fastDigitalWrite(IN1_MOTOR_RIGHT, in1_motor_state);
      fastDigitalWrite(IN2_MOTOR_RIGHT, in2_motor_state);
      #endif
      break;
  }
}

/* Dump variables in log object */
#ifdef ENABLE_LOG
void dumpInfo(){
  printLogn("Dumping system variables...");
  printMem("Button State = ");
  printLogVarn(button_state);
  printMem("Motor Speed Virtual Cms = ");
  printLogVarn(motor_speed_steps);
  printMem("Pwm Speed Motor Left = ");
  printLogVarn((int)ceil(pwm_motor_speed_left));
  printMem("Pwm Speed Motor Right = ");
  printLogVarn((int)ceil(pwm_motor_speed_left));
  printMem("Actual Rotation = ");
  printLogVarn(actual_rotation);
  printMem("Step Angle = ");
  printLogVarn(step_angle);
  printMem("Step Length = ");
  printLogVarn(step_length);
  printMem("Rotation Length = ");
  printLogVarn(rotation_length);
  printMem("Distance Wanted = ");
  printLogVar(distance_wanted_left);
  printMem(" ");
  printLogVarn(distance_wanted_right);
  printMem("Distance Reach Motor Left = ");
  printLogVarn(steps_reach_motor_left);
  printMem("Distance Reach Motor Right = ");
  printLogVarn(steps_reach_motor_right);
  printMem("Steps Motor Left = ");
  printLogVarn(steps_motor_left);
  printMem("Steps Motor Right = ");
  printLogVarn(steps_motor_right);
  printMem("Rotations Motor Left = ");
  printLogVarn(rotations_motor_left);
  printMem("Rotations Motor Right = ");
  printLogVarn(rotations_motor_right);
  printMem("Input Ticks Motor Left = ");
  printLogVarn(input_steps_motor_left);
  printMem("Input Ticks Motor Right = ");
  printLogVarn(input_steps_motor_right);
  printMem("Routine Flag Mutex Lock = ");
  printLogVar(routine_flag_mutex_lock_left);
  printMem(" ");
  printLogVarn(routine_flag_mutex_lock_right);
}
#endif
