#ifndef ROBOT_SETUP
#define ROBOT_SETUP

/* Optical Encoders */
#define ENCODER_LEFT_PIN  2
#define ENCODER_RIGHT_PIN 3

/* IR Receiver pin */
#define RECEIVER_PIN      4

/* Speed of each motor */
/* Change speed of each motor */
#define SPEED_MOTOR_LEFT  5
#define SPEED_MOTOR_RIGHT 6

/* Enable of the Motor Left */
/* Enable change direction or stop this motor */
#define IN1_MOTOR_LEFT    7
#define IN2_MOTOR_LEFT    8

/* Enable of the Motor Right */
/* Enable change direction or stop this motor */
#define IN1_MOTOR_RIGHT   9
#define IN2_MOTOR_RIGHT   10

/* Software Serial Pins */
#define SOFTWARE_SERIAL_TX_PIN  11
#define SOFTWARE_SERIAL_RX_PIN  12

/* Define the pins of the Ultrasonic Sensors */
#define TRIGGER_PIN       14
#define ECHO_PIN_LEFT     15
#define ECHO_PIN_RIGHT    16

/* Robot Configs */

/* Robot Width */
#define ROBOT_WIDTH_MM    156

/* Robot Length */
#define ROBOT_LENGTH_MM   210

/* Center of wheels */
#define WHEEL_CENTER_X    78
#define WHEEL_CENTER_Y    135

/* Wheel width */
#define WHEEL_WIDTH_MM    26

/* Wheel size */
#define WHEEL_DIAMETER_MM 76

/* Robot distance between wheels */
#define WHEEL_DISTANCE_MM 100

/* Number of slices of optical encoder */
/* WARNING: MULTIPLY NUMBER OF SPACES IN ENCODER BY TWO IF INTERRUPTION IS CHANGE! */
#define ENCODER_STEPS     40

/* Encoder type of interruption */
/* NOTE: USING HIGH/LOW/RISING/FALLING WILL ONLY GIVE ONE INTERRUPTION AT EACH 2 SPACES
         (ONE CLOSED AND OTHER OPEN, USING CHANGE WILL GIVE YOU ONE INTERRUPTION AT EACH SPACE
         IMPROVING YOUR PRECISION*/
#define ENCODER_INTERRUPTION_TYPE CHANGE

/* Command buttons */

/* Forward Button */
#define FORWARD_BUTTON_IR   0x8076A05F
#define FORWARD_COMMAND     'F'

/* Reverse Button */
#define REVERSE_BUTTON_IR   0x807620DF
#define REVERSE_COMMAND     'R'

/* Step Left Button */
#define LEFT_BUTTON_IR      0x8076F807
#define STEP_LEFT_COMMAND    '1'

/* Step Right Button */
#define RIGHT_BUTTON_IR     0x80767887
#define STEP_RIGHT_COMMAND  '2'

/* Turn Left Command */
#define TURN_LEFT_COMMAND   '3'

/* Turn Right Command */
#define TURN_RIGHT_COMMAND  '4'

/* Step Command */
#define STEP_COMMAND        'S'
#define TURN_COMMAND        'T'

/* Directions possible */
#define LEFT_DIRECTION      'L'
#define RIGHT_DIRECTION     'R'

/* Acelerate Command */
#define ACELERATE_COMMAND   'A'

/* Brake Button */
#define BRAKE_BUTTON_IR     0x8076708F
#define BRAKE_COMMAND       'B'

/* Dump data */
#define DUMP_COMMAND        'D'

/* PID Constants */
#define DEFAULT_KP 30
#define DEFAULT_KI 20
#define DEFAULT_KD 10

/* Helpers */

/* Concatenate variable name */
#define concatenateVarName(VAR, NAME) VAR ## NAME

/* Acelerate both motors */
#define acelerateMotors(motor_speed) acelerateMotor(BOTH_MOTORS, motor_speed)

/* Turn Forward a given motor */
#define turnForwardMotor(motor_num) changeMotorState(motor_num, HIGH, LOW)

/* Turn Reverse a given motor */
#define turnReverseMotor(motor_num) changeMotorState(motor_num, LOW, HIGH)

/* Brake a given motor */
#define brakeMotor(motor_num) changeMotorState(motor_num, LOW, LOW)

/* Parallel aceleration both motors */
#define parallelAceleration() parallelAcelerationMotor(BOTH_MOTORS)

/* Turn both motors foward */
#define turnForward()   turnForwardMotor(BOTH_MOTORS)

/* Turn both motors reverse */
#define turnReverse()   turnReverseMotor(BOTH_MOTORS)

/* Zero PWM Speed */
#define zeroPwmSpeedSide(SIDE) concatenateVarName(pwm_motor_speed_, SIDE) = 0

/* Remove PWM adjust from both motors */
#define zeroPwmSpeed()  do { zeroPwmSpeedSide(left); zeroPwmSpeedSide(right); } while(false)

/* Zero distance reach */
#define zeroDistanceReach() do { distance_wanted = 0; distance_reach_motor_left = 0; distance_reach_motor_right = 0; } while(false)

/* Brake both motors */
#define brake()         do { brakeMotor(BOTH_MOTORS); acelerateMotors(PWM_STOP_SPEED); zeroPwmSpeed(); } while(false)

/* Step Right */
/* Stop one of the motors and move the other in opossite direction */
#define stepLeft()      do { brakeMotor(MOTOR_RIGHT); turnForwardMotor(MOTOR_LEFT); } while(false)

/* Step Left */
/* Stop one of the motors and move the other in opossite direction */
#define stepRight()     do { brakeMotor(MOTOR_LEFT);  turnForwardMotor(MOTOR_RIGHT); } while(false)

/* Turn Left */
/* Move both motors in opossite directions */
#define turnLeft()      do { turnForwardMotor(MOTOR_LEFT); turnReverseMotor(MOTOR_RIGHT); } while(false)

/* Turn Right */
/* Move both motors in opossite directions */
#define turnRight()     do { turnForwardMotor(MOTOR_RIGHT); turnReverseMotor(MOTOR_LEFT); } while(false)

#endif
