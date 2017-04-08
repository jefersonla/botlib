#ifndef BOTLIB_H
#define BOTLIB_H

#include <stdint.h>

/* Change state of enable pin */
void changeMotorState(int motor_num, bool in1_motor_state, bool in2_motor_state);

/* Acelerate a given motor */
void acelerateMotor(int motor_num, int pwm_motor_speed);

/* Change Motor State */
void executeCommand(unsigned long motor_command);

/* Read Ir signal */
void readIRSignal();

/* Read Serial data */
void readSerialData();

/* Parallel aceleratio of wheels */
void parallelAcelerationMotor(int motor_num);

/* Move ahead for some distance, if distance is negative move reverse */
void moveAhead(int distance);

/* Move robot to an specific angle */
void rotateRobot(int16_t desired_angle);

#endif
