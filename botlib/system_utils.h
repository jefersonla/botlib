#ifndef SYSTEM_SETUP
#define SYSTEM_SETUP

/* Serial Speed */
#define SERIAL_SPEED      115200

/* Motor Types */

/* Define the types of each motor */
#define MOTOR_LEFT        1
#define MOTOR_RIGHT       2
#define BOTH_MOTORS       3

/* Motor Speed and Adjust interval */

/* Step speed */
#define STEP_SPEED 40

/* Step speed interval by seconds */
#define ADJUST_MOTOR_INTERVAL 4

/* 
 *                   !!!!! WARNING !!!!!
 *  TAKE CARE WHILE CHOOSING THIS SINCE A STEP SPEED OF 120, 
 *   RESULT IN 4 COMPLETE ROTATIONS OF EACH WHEEL BY SECOND
 */

/* Default speeds */

/* Stop speed */
#define PWM_STOP_SPEED    0

/* Low speed - 30% */
#define PWM_LOW_SPEED     80

/* Medium speed - 60% */
#define PWM_MEDIUM_SPEED  150

/* High Speed - 90% */
#define PWM_HIGH_SPEED    230

/* Maximum Speed - 100% */
#define PWM_MAXIMUM_SPEED 255

/* Max pwm increment */
#define MAX_PWM_INCREMENT 20

/* MAX pwm decrement */
#define MAX_PWM_DECREMENT 20

/* Timer Utilities */

/* Time constants */
#define TIME_1S_MS        1000
#define TIME_2S_MS        2000
#define TIME_0_1S_US      100000
#define TIME_0_25S_US     250000
#define TIME_0_5S_US      500000
#define TIME_1S_US        1000000
#define TIME_2S_US        1000000

/* Parallel operations */

/* Mutex type */
typedef volatile bool mutex;

/* Wait while some mutex are locked */
#define await(MUTEX_NAME) while(MUTEX_NAME)

/* 
 *                  !!!!! WARNING !!!!!
 *     DON'T FORGET TO DEFINE MUTEX AS 'volatile bool'! 
 *       DESPITE OF THAT YOU CAN USE MUTEX TYPE TOO
 */

/* Grid Movimentation  */

/* Grid Size */
#define GRID_SIZE         20

/* 
 *                   !!!!! WARNING !!!!!
 *           GRIDS SHOULDN'T BE GREATER THAN 35! 
 *  A 35x35 CONSUMES 1225 BYTES AND MIGHT BREAK EVERYTHING!
 */

#endif
