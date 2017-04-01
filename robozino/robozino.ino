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
 * and Andressa Andrade(@AndressAndrade)<dsandrade@dcc.ufba.br>
 */

/* Libraries Used */
#include "pid.c"
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

/* User Code */
extern void userSetup();
extern void userLoop();
#include "user.cpp"

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
//volatile double pwm_motor_speed_left;
//volatile double pwm_motor_speed_right;
volatile int pwm_motor_speed_left;
volatile int pwm_motor_speed_right;

/* Distance read by each motor */
volatile unsigned int steps_reach_motor_left;
volatile unsigned int steps_reach_motor_right;

/* Enable movement of each motor */
volatile bool motor_left_enabled;
volatile bool motor_right_enabled;

/* Number of ticks on each motor, used to tune adjust */
//volatile double input_steps_motor_left;
//volatile double input_steps_motor_right;
volatile int steps_motor_left;
volatile int steps_motor_right;

/* Routine Flag Lock */
mutex routine_flag_mutex_lock_left;
mutex routine_flag_mutex_lock_right;

/* PID Constants */
volatile double pid_kp_left = DEFAULT_KP;
volatile double pid_ki_left = DEFAULT_KI;
volatile double pid_kd_left = DEFAULT_KD;
volatile double pid_kp_right = DEFAULT_KP;
volatile double pid_ki_right = DEFAULT_KI;
volatile double pid_kd_right = DEFAULT_KD;
//volatile uint16_t pid_kp = DEFAULT_KP;
//volatile uint16_t pid_ki = DEFAULT_KI;
//volatile uint16_t pid_kd = DEFAULT_KD;

/* PID Speed wanted */
//volatile double setpoint_steps_speed_left = 0;
//volatile double setpoint_steps_speed_right = 0;
volatile int setpoint_steps_speed_left;
volatile int setpoint_steps_speed_right;

/* PID Object */
//PID motor_left_pid(&input_steps_motor_left, &pwm_motor_speed_left, &setpoint_steps_speed_left, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DIRECT);
//PID motor_right_pid(&input_steps_motor_right, &pwm_motor_speed_right, &setpoint_steps_speed_right, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DIRECT);
//volatile struct PID_DATA motor_left_pid;
//volatile struct PID_DATA motor_right_pid;

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
  fastPinMode(REGISTER_LOCK_MOTOR_LEFT, OUTPUT);
  fastPinMode(REGISTER_LOCK_MOTOR_RIGHT, OUTPUT);

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

  /* Initialize actual rotation */
  actual_rotation = 0;

  /* Disable the two motors */
  motor_left_enabled = false;
  motor_right_enabled = false;

  /* Initialize pwm speed */
  pwm_motor_speed_left = 0;
  pwm_motor_speed_right = 0;

  /* Initialize counter of steps per motor adjust */
  //input_steps_motor_left = 0;
  //input_steps_motor_right = 0;
  steps_motor_left = 0;
  steps_motor_right = 0;

  /* Initialize speed wanted in each motor */
  setpoint_steps_speed_left = 0;
  setpoint_steps_speed_right = 0;

  /* Blocking mutex */
  routine_flag_mutex_lock_left = false;
  fastDigitalWrite(REGISTER_LOCK_MOTOR_LEFT, LOW);
  routine_flag_mutex_lock_right = false;
  fastDigitalWrite(REGISTER_LOCK_MOTOR_RIGHT, LOW);
  
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
  //motor_left_pid.SetMode(AUTOMATIC);
  //motor_right_pid.SetMode(AUTOMATIC);
  //pid_Init(pid_kp, pid_ki, pid_kd, &motor_left_pid);
  //pid_Init(pid_kp, pid_ki, pid_kd, &motor_right_pid);

  #ifdef ENABLE_LOG
  /* Start PID Controller for each motor on automatic mode */
  printLogn("PID Mode setted on automatic");
  #endif

  /* Call user setup routine */
  userSetup();

  #ifdef ENABLE_LOG
  /* Start PID Controller for each motor on automatic mode */
  printLogn("Perfomed user setup functions");
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

  /* Call user loop routine */
  userLoop();

  /* Move 1000mm - 100cm - 1m */
  //moveAhead(400);
  //delay(10000);
  //rotateRobot(90);
  //delay(5000);
}

/* :::::::::::::::::::       Helpers       ::::::::::::::::::: */

/* Increment counter of the motor left */
void stepCounterMotorLeft(){
  /* Steps cycle of motor right */
  steps_motor_left++;
  /* Total number of steps reached by motor left */
  steps_reach_motor_left++;
  
  /* Brake motors if we had reached the distance wanted */
  if(steps_reach_motor_left >= distance_wanted_left){
    /* Remove mutex */
    routine_flag_mutex_lock_left = false;
    //fastDigitalWrite(REGISTER_LOCK_MOTOR_LEFT, LOW);
    PORTB &= ~(1 << PORTB4);
    /* Disable motor left */
    motor_left_enabled = false;
    /* Brake this motor */
    brakeMotor(MOTOR_LEFT);
    //turnReverseMotor(MOTOR_LEFT);
    //acelerateMotor(MOTOR_LEFT, REGISTER_LOCK_MOTOR_RIGHT);
    /* Zero number of steps reach by this motor */
    steps_reach_motor_left = 0;
    /* Zero distance Wanted */
    distance_wanted_left = 0;
    /* Zero PWM Speed */
    //pwm_motor_speed_left = 0;
    setpoint_steps_speed_left = 0;

    #ifdef ENABLE_LOG
    //printDebugn("Stopped Motor Left");
    #endif
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
  /* Steps cycle of motor right */
  steps_motor_right++;
  /* Total number of steps reached by motor right */
  steps_reach_motor_right++;

  /* Brake motor right if we had reached the distance wanted */
  if(steps_reach_motor_right >= distance_wanted_right){  
    /* Remove mutex */
    routine_flag_mutex_lock_right = false;
    //fastDigitalWrite(REGISTER_LOCK_MOTOR_RIGHT, LOW);
    PORTB &= ~(1 << PORTB5);
    /* Disable motor right */
    motor_right_enabled = false;
    /* Brake this motor */
    brakeMotor(MOTOR_RIGHT);
    //turnReverseMotor(MOTOR_RIGHT);
    //acelerateMotor(MOTOR_RIGHT, REGISTER_LOCK_MOTOR_RIGHT);
    /* Zero number of steps reach by this motor */
    steps_reach_motor_right = 0;
    /* Zero distance Wanted */
    distance_wanted_right = 0;
    /* Zero PWM Speed */
    //pwm_motor_speed_right = 0;
    setpoint_steps_speed_left = 0;
    
    #ifdef ENABLE_LOG
    //printDebugn("Stopped Motor Right");
    #endif
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
  /* Change inputs of PID controller */
  //input_steps_motor_left = steps_reach_motor_left;
  //input_steps_motor_right = steps_reach_motor_right;
  
  /* Compute new PID values */
  //motor_left_pid.Compute();
  //motor_right_pid.Compute();
  //pwm_motor_speed_left = pid_Controller(setpoint_steps_speed_left, steps_motor_left, &motor_left_pid);
  //pwm_motor_speed_right = pid_Controller(setpoint_steps_speed_right, steps_motor_right, &motor_left_pid);

  if(steps_motor_left > steps_motor_right){
    pid_kp_left *= (steps_motor_right / steps_motor_left);
    pid_kp_right *= (steps_motor_left / steps_motor_right);
  }
  if(steps_motor_right > steps_motor_left ){
    pid_kp_left *= (steps_motor_right / steps_motor_left);
    pid_kp_right *= (steps_motor_left / steps_motor_right);
  }

  /* Adjust pwm output of each motor */
  if(motor_left_enabled){
    pwm_motor_speed_left += pid_kp_left * (setpoint_steps_speed_left - steps_motor_left);
  }
  if(motor_right_enabled){
    pwm_motor_speed_right += pid_kp_right * (setpoint_steps_speed_right - steps_motor_right);
  }

  /* Adjust difference of the two motors */
  if(steps_motor_left > steps_motor_right){
    pwm_motor_speed_right += pid_ki_right * (steps_motor_left - steps_motor_right);
    pwm_motor_speed_left -= pid_kd_left * (steps_motor_left - steps_motor_right);
  }
  else{
    pwm_motor_speed_left += pid_ki_left * (steps_motor_right - steps_motor_left);
    pwm_motor_speed_right -= pid_kd_right * (steps_motor_right - steps_motor_left);
  }
  
  analogWrite(SPEED_MOTOR_RIGHT, pwm_motor_speed_right);
  analogWrite(SPEED_MOTOR_LEFT, pwm_motor_speed_left);

  /* Clean values for next interation */
  steps_motor_left = 0;
  steps_motor_right = 0;

  #ifdef ENABLE_LOG
  /*
  printLog("PWM (L|R) (");
  printLogVar(ceil(pwm_motor_speed_left));
  printMem(" | ");
  printLogVar(ceil(pwm_motor_speed_right));
  printMemn(" )");
  */
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
  unsigned int distance_wanted = 0;

  /* While we wanted a rotation greater than 360, we increase distance wanted by number of steps */
  while(desired_angle >= 180){
    distance_wanted += ceil(rotation_length/(step_length*2));
    desired_angle -= 180;
  }
 
  /* Now we calc the wanted distance, and to ensure precision distance is defined in steps */
  distance_wanted += ceil(ceil((desired_angle * PI * (ROBOT_WIDTH_MM / 2.0)) / 180.0) / step_length);
  distance_wanted_left = distance_wanted;
  distance_wanted_right = distance_wanted;

  /* Start motor parallel aceleration */
  parallelAceleration();
  
  /* Lock Mutex */
  routine_flag_mutex_lock_left = true;
  digitalWrite(REGISTER_LOCK_MOTOR_LEFT, HIGH);
  routine_flag_mutex_lock_right = true;
  digitalWrite(REGISTER_LOCK_MOTOR_RIGHT, HIGH);
  delay(50);
  
  /* Wait until it finishes the thread */
  //await(routine_flag_mutex_lock_left || routine_flag_mutex_lock_right);
  Serial.println("STARTOU");
  //await(fastDigitalRead(REGISTER_LOCK_MOTOR_LEFT) || fastDigitalRead(REGISTER_LOCK_MOTOR_RIGHT));
  //while(digitalRead(REGISTER_LOCK_MOTOR_LEFT) || digitalRead(REGISTER_LOCK_MOTOR_RIGHT)){
  Serial.println(PINB, BIN);
  while((((PINB & B00110000) >> 4) ^ B00000011) != B00000011){
    delay(50);
  }
  delay(1000);
  Serial.println("FINALIZOU GG");
  brake();
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
  digitalWrite(REGISTER_LOCK_MOTOR_LEFT, HIGH);
  routine_flag_mutex_lock_right = true;
  digitalWrite(REGISTER_LOCK_MOTOR_RIGHT, HIGH);
  delay(50);

  /* Start motor parallel aceleration */
  parallelAceleration();

  Serial.println("COMECOU 1");

  /* Wait until it finishes the thread */
  //await(routine_flag_mutex_lock_left || routine_flag_mutex_lock_right);
  //while(routine_flag_mutex_lock_left || routine_flag_mutex_lock_right){
  while((((PINB & B00110000) >> 4) ^ B00000011) != B00000011){
    delay(50);
  }
  delay(1000);
  Serial.println("FINALIZOU 3");
  brake();
}

/* Parallel aceleratio of wheels */
void parallelAcelerationMotor(int motor_num){

  /* Define setpoint speed for each motor based on number of steps by second divided by motor adjust interval */
  switch(motor_num){
    case MOTOR_LEFT:
      motor_left_enabled = true;
      setpoint_steps_speed_left = ceil(STEP_SPEED / ADJUST_MOTOR_INTERVAL);
      break;
    case MOTOR_RIGHT:
      motor_right_enabled = true;
      setpoint_steps_speed_right = ceil(STEP_SPEED / ADJUST_MOTOR_INTERVAL);
      break;
    case BOTH_MOTORS:
      motor_left_enabled = true;
      motor_right_enabled = true;
      setpoint_steps_speed_left = ceil(STEP_SPEED / ADJUST_MOTOR_INTERVAL);
      setpoint_steps_speed_right = ceil(STEP_SPEED / ADJUST_MOTOR_INTERVAL);
      break;
  }
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
  printMem("Input Ticks Motor Left = ");
  //printLogVarn(input_steps_motor_left);
  printLogVarn(steps_motor_left);
  printMem("Input Ticks Motor Right = ");
  //printLogVarn(input_steps_motor_right);
  printLogVarn(steps_motor_right);
  printMem("Routine Flag Mutex Lock = ");
  printLogVar(routine_flag_mutex_lock_left);
  printMem(" ");
  printLogVarn(routine_flag_mutex_lock_right);
}
#endif

//////////////////////////

/* Define your libraries and variables here */
#define n 3
#define a   25//(n * REGISTER_LOCK_MOTOR_RIGHT)
#define m   (n + 2)

static int grid[m][m];
static int heuristic[n][n];

/* MAKE IT STOP */
static int fim = 0;           
static int counter = 0;       

/* Define Goal */
static int GoalRow = 1;
static int GoalCol = 3;

/* Define Actual Point */
static int ActualRow;
static int ActualCol;

/* Define Next Point */
static int NextRow;
static int NextCol;


/* Define StartPoint */
static int StartRow = 3;
static int StartCol = 1;

/* Function g(n) - Cost */
static int ort = 10;      // Define ortogonal's(up, down, left, right) cost as 10
static int obstacle = 1000; //Define obstacle

/* Variables */
static int i;
static int j;
static int z;//Indice utilizado nos vetores path
static int p;
static int pathRow[a];
static int pathCol[a];
static int finalRow[m];
static int finalCol[m];

void intervencao(){
  for(i = 0; i < m; i++){
      for(j = 0; j < m; j++){
         Serial.println(grid[i][j]);
         fim = 1;
      }
   }
}

/* Define your code here */
void printpath(){
  Serial.println("Este eh ao caminho percorrido ate aqui:");
  for(i = 0; i < z; i++){
    Serial.print(pathRow[i]);
    Serial.print(" ");
    Serial.println(pathCol[i]);
  }
}

void printheuristic(){
  Serial.println("Este eh a heuristica do labirinto, resolvido por Manhanttan:");
  for(i = 0; i < n; i++){
    for(j = 0; j < n; j++){
      Serial.print(heuristic[i][j]);
      Serial.print("  ");
    }
    Serial.print("\n");
  }  
}

void printgrid(){
  Serial.println("Este eh o grid do labirinto, resolvido por A*:");
  for(i = 0; i < m; i++){
    for(j = 0; j < m; j++){
      Serial.print(grid[i][j]);
      Serial.print("  ");
    }
    Serial.print("\n");
  }  
}

boolean isFim(){
  Serial.println("Verificando se eh o fim...");
  //This funcion will see if we reach the goal
  //if ActualPoint is equal GoalPoint
  if(ActualRow == GoalRow && ActualCol == GoalCol){
    fim = 1;
    return true; 
  }
  else{
    return false;
  }
}

/* PD Functions - Make the Robot drive straight*/

/* Obstacles Detector Functions */

boolean isObstacle(int x, int y){
  Serial.println("Verificando obstaculos...");
  //This funcion will see if there is a object on a point in the matrix that is a obstacles
  if (x == 0 or y == 0 or x == 4 or y == 4){
    return true;
  }
  else{
    return false;
  }
}

/* Robot Moviments Functions */

//This follows moviments has as function, besides move the robot aroud the grid, make the robo orientation be always to the North, this can make it easier to move the robot 
void goForward(){
  Serial.println("Movendo para frente...");
  moveAhead(400);
  //Move robot forward
}

void goBackward(){
  Serial.println("Movendo para tras...");
  moveAhead(-400);
  //Move robot backward
}

void turn_Left(){
  Serial.println("Movendo para esquerda...");
  rotateRobot(-90);
  moveAhead(-400);
  //Turn robot left *doesn't mean change matriz position, move robot 90º on his atual position
  //goForward()     *go to other position on the grid
  //Turn robot right
}

void turn_Right(){
  Serial.println("Movendo para direita...");
  rotateRobot(90);
  //Turn robot right 
  //goForward()     
  //Turn robot left
}

//Just change the robot orientation
void turnAround(){
  Serial.println("Girando...");
  rotateRobot(90);
  //Turn robot right
}


/* A Star Functions */

int aux1;
int aux2;

void setGrid(int x, int y){
  Serial.println("Calculando heuristica mais custo...");
  int q = x;
  int r = y;
  if (isObstacle(q, r)){
    if (q >= 0 && r >= 0){
      grid[x][y] = obstacle;
      return;
    }
  }
  if (grid[x][y] == 2000){
      return;
  }
  else{
      grid[x][y] = 0;
      grid[x][y] = 10 + heuristic[x-1][y-1];
      // NAO
      //int w = z;
      //while(pathRow[w] != StartRow && pathCol[w] != StartCol){
     //   grid[x][y] += heuristic[pathRow[w-1]][pathCol[w-1]];
      //  w -= 1;
     // }
     // NAO
     Serial.print("Este eh meu indice Z: ");
     Serial.println(z);
     p = z-1;
     int lock2 = 1;
     printpath();
     while(lock2){
      if(pathRow[p] == StartRow && pathCol[p] == StartCol){
        lock2 = 0;  
      }
      else{  
        /*Serial.print(pathRow[p]);
        Serial.print("  ");
        Serial.println(pathCol[p]);*/
        grid[x][y] += heuristic[pathRow[p]-1][pathCol[p]-1];
        p-=1;
      }  
    }
  }
}

/* Function h(n) - Define Heuristic using Manhattan Distance*/
void heuristicFunction(){
  Serial.println("Calculando matriz heuristica...");
  for(i = 0; i < n; i++){
    for(j = 0; j < n; j++){
      aux1 = abs(i - (GoalRow-1));
      aux2 = abs(j - (GoalCol-1));
      aux1 = aux1 + aux2;
      heuristic[i][j] = aux1;
      /*
      Serial.print("heuristic[");
      Serial.print(i);
      Serial.print("][");
      Serial.print(j);
      Serial.print("] = ");
      Serial.println(heuristic[i][j]);
      */
    }
  }  
}

/* Function f(n) = g(n) + h(n) */
void a_star(){
  Serial.println("Aplicando A* em meus vizinhos...");
  /*
  int StartRow = 3;
  int StartCol = 1;
  I need o check all around, but skip previous calculate...
  */
  //Seta o grid como 2000 para evitar loop infinito
  grid[ActualRow][ActualCol] = 2000;
   
  //Fist test forward (forward is -1 of the ActualPosition Row)
  setGrid(ActualRow-1,ActualCol);
  //Then test right (right is +1 of the ActualPosition Col)
  turnAround();
  setGrid(ActualRow,ActualCol+1);
  //Then test backward (backward is +1 of the ActualPosition Row)
  turnAround();
  setGrid(ActualRow+1,ActualCol);
  //Then test left (left is -1 of the ActualPosition Col),
  turnAround();
  setGrid(ActualRow,ActualCol-1);
  //Finally turn back to Origin orientation
  turnAround();
}

void findPath(){
  Serial.println("Definindo rota...");
  p = z - 1;
  int padrao = 1000;
  for(i = 1; i < m; i++){
    for(j = 1; j < m; j++){
       if(grid[i][j] < padrao){
        NextRow = i;
        NextCol = j;
        //Serial.print(NextRow);
        //Serial.println(NextCol);
        padrao = grid[i][j];
       }
    }
  }
}

void goAhead(){
  Serial.println("Indo para proxima posicao...");
  Serial.print("Linha: ");
  Serial.print(NextRow);
  Serial.print(" Coluna: ");
  Serial.println(NextCol);
  /*É necessario se posicionar neste novo grid */
  //Verifico se próxima posição é um vizinho
  if(ActualRow == NextRow){  //Se for,  eles estão na mesma linha, devo left ou right
    while(ActualCol != NextCol){   
      if(ActualCol < NextCol){
        turn_Right();
        ActualRow = NextRow;
        ActualCol +=1;
      }
      else{
        turn_Left(); 
        ActualRow = NextRow;
        ActualCol -= 1;
      }
      pathRow[z] = ActualRow;
      pathCol[z] = ActualCol;
      z += 1;
    }  
    return;
  }
  if(ActualCol == NextCol){  //Se for, eles estão na mesma coluna, devo forward ou backward
     while(ActualRow != NextRow){ 
      if(ActualRow < NextRow){// turn right
        goBackward();
        ActualRow += 1;
        ActualCol = NextCol;
      }
      else{
        goForward();
        ActualRow -= 1;
        ActualCol = NextCol;
      }
      pathRow[z] = ActualRow;
      pathCol[z] = ActualCol;
      z += 1;
    } 
    return;
  }
  //Caso não seja vizinho, devo regredir no grid, retrocendendo na lista percorrida até alcançar um vizinho,
  Serial.println("Nao e vizinho.............................");
  p -= 1;
  if(ActualRow == pathRow[p]){ //Se for,  eles estão na mesma linha, devo left ou right
     if(ActualCol < pathCol[p]){
      turn_Right();
     }
     else{
      turn_Left(); 
     }
     ActualRow = pathRow[p];
     ActualCol = pathCol[p];
     pathRow[z] = ActualRow;
     pathCol[z] = ActualCol;
     z += 1;
     goAhead();
  }
  if(ActualCol == pathCol[p]){  //Se for, eles estão na mesma coluna, devo forward ou backward
     if(ActualRow < pathRow[p]){// turn right
      goBackward();
     }
     else{
      goForward();
     }
     ActualRow = pathRow[p];
     ActualCol = pathCol[p];
     pathRow[z] = ActualRow;
     pathCol[z] = ActualCol;
     z += 1;
     goAhead();
  }
  goAhead(); //?
}

void defineWay(){
  p = z;
  int lock = 1;
  Serial.println("Imprimindo caminho final...");
  while(lock){
    if(pathRow[p] == StartRow && pathCol[p] == StartCol){
      lock = 0;  
    }
    else{  
      counter++;
      p--;
    }  
  }
  for(i = 0; i < counter; i++){
    finalRow[i] = pathRow[p];
    finalCol[i] = pathCol[p];
    Serial.print("Passo ");
    Serial.print(i+1);
    Serial.print(" ");
    Serial.print(pathRow[p]);
    Serial.print(" ");
    Serial.println(pathCol[p]);
    p++;
  }
}

/********************************/
/* Don't remove these functions */
/********************************/

void userSetup(){
  /* Define your own setup routine here */ 
  heuristicFunction();
  ActualRow = StartRow;
  ActualCol = StartCol;

  pathRow[0] = StartRow;
  pathCol[0] = StartCol;
  z = 1;
  
  for(i = 0; i < m; i++){
    for(j = 0; j < m; j++){
      grid[i][j] = 1000;
    }
  }
}

void userLoop(){
  /* Define your own loop routine here */ 
  delay(1000);
  if(!fim){
    a_star();
    delay(1000);
    findPath();
    delay(1000);
    goAhead();
    delay(1000);
    printgrid();
    printheuristic();
    delay(1000);
    if(isFim()){
       defineWay();
       delay(10000);
    }
    delay(2000);

  }

  Serial.println("Fim do loop!");
}
