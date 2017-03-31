#ifndef OBSOLETE_CODE
#define OBSOLETE_CODE

/* ::::::::::::::::::: Obsolete  Functions ::::::::::::::::::: */

/* Just here for history purpose */
#ifdef USE_OBSOLETE

/* Warning the compiler about use of Deprecated functions */
#pragma GCC warning "CAUTION! USE OF DEPRECATED FUNCTIONS ENABLED!"

/* Chaos correction test */
#define WITH_MORE_CHAOS_MORE_ORDER 5
#define MAX_CHAOS_ALLOWED 50
#define MIN_CHAOS_ALLOWED -MAX_CHAOS_ALLOWED

/* Increment counter of the motor left */
void oldStepCounterMotorLeft(){
  steps_motor_left++;

  /* Reset chaos if it's too great */
  if(pwm_adjust_motor_left > MAX_CHAOS_ALLOWED){
    pwm_adjust_motor_left = 0;
  }

  /* Adjust motor if it's starting move oposite */
  if((steps_motor_left - steps_motor_right) > 3){
    steps_motor_right -= 2;
    pwm_adjust_motor_left -= (pwm_adjust_motor_left > MIN_CHAOS_ALLOWED) ? (pwm_adjust_motor_left) : WITH_MORE_CHAOS_MORE_ORDER;
    pwm_adjust_motor_right += (pwm_adjust_motor_right > MAX_CHAOS_ALLOWED) ? (-pwm_adjust_motor_right) : WITH_MORE_CHAOS_MORE_ORDER;
  }

  /* Increase one rotation if we had completed */
  if(steps_motor_left == ENCODER_STEPS){
    steps_motor_left = 0;
    rotations_motor_left++;
  }
}

/* Increment counter of the motor right */
void oldStepCounterMotorRight(){
  steps_motor_right++;

  /* Adjust motor if it's starting move oposite */
  if((steps_motor_right - steps_motor_left) > 3){
    steps_motor_left -= 2;
    pwm_adjust_motor_right -= (pwm_adjust_motor_right > MIN_CHAOS_ALLOWED) ? (pwm_adjust_motor_right) : WITH_MORE_CHAOS_MORE_ORDER;
    pwm_adjust_motor_left += (pwm_adjust_motor_left > MAX_CHAOS_ALLOWED) ? (-pwm_adjust_motor_left) : WITH_MORE_CHAOS_MORE_ORDER;
  }

  /* Increase one rotation if we had completed */
  if(steps_motor_right == ENCODER_STEPS){
    steps_motor_right = 0;
    rotations_motor_right++;
  }
}

/* Motor aceleration test routine */
void oldAcelerationTestRoutine(){

  /* Stop and dettach this routine after execution */
  Timer1.stop();
  Timer1.detachInterrupt();

  /* Brake the motor */
  brake();

  /* Unlock mutex lock */
  routine_flag_mutex_lock = false;
}

/* Aceleration Test */
void oldAcelerationTest(){

  #ifdef ENABLE_LOG
  /* Doing aceleration test */
  printLogn("Doing aceleration test");
  #endif

  /* Clean counters variables */
  steps_motor_left = 0;
  rotations_motor_left = 0;
  steps_motor_right = 0;
  rotations_motor_right = 0;
  
  /* Lock a mutex and acelerate motors with a high speed */
  routine_flag_mutex_lock = true;
  turnForward();
  acelerateMotors(PWM_HIGH_SPEED);

  delay(TIME_1S_MS);
  
  /* Reconfigurate timer */
  Timer1.stop();
  Timer1.attachInterrupt(oldAcelerationTestRoutine);
  Timer1.start();

  /* Wait until it finishes the thread */
  await(routine_flag_mutex_lock);
}

/* Adjust motors to run in the same speed */
void oldAdjustMotors(){

  /* First initialize Timer1 with a routine of 1 second */
  Timer1.initialize(TIME_2S_US);

  int motors_steps_difference = 0;

  /* Compute the difference between these two motors, until it finishes */
  do{

    #ifdef ENABLE_LOG
    /* Start adjust of motors */
    printLogn("Adjusting motors...");
    #endif

    /* Do aceleration test */
    oldAcelerationTest();
 
    /* Total number of steps perfomed by each motor */
    int motor_left_total_steps = (rotations_motor_left * ENCODER_STEPS) + steps_motor_left;
    int motor_right_total_steps = (rotations_motor_right * ENCODER_STEPS) + steps_motor_right;
    
    /* Check the difference between the two motors */
    int motors_rotation_difference = abs(rotations_motor_left - rotations_motor_right);
    motors_steps_difference = abs(motor_left_total_steps - motor_right_total_steps);

    #ifdef ENABLE_LOG
    /* Start adjust of motors */
    printLog("Motor left total steps = ");
    printLogVarn(motor_left_total_steps);
    printLog("Motor right total steps = ");
    printLogVarn(motor_right_total_steps);
    printLog("Motors rotation difference = ");
    printLogVarn(motors_rotation_difference);
    printLog("Motors steps difference = ");
    printLogVarn(motors_steps_difference);
    printLog("Motors pwm adjust motor left = ");
    printLogVarn(pwm_adjust_motor_left);
    printLog("Motors pwm adjust motor right = ");
    printLogVarn(pwm_adjust_motor_right);
    #endif

    /* Until the error between the two motors are close enough */
    if(motors_steps_difference > MAX_STEPS_ERROR){
      
      /* Check which motor should be corrected to the other */
      if(motor_left_total_steps > motor_right_total_steps){
        /* Adjust strength of motor right */
        if(pwm_adjust_motor_right < MAX_PWM_INCREMENT){
          pwm_adjust_motor_right++;
        }
        /* Adjust strength of motor left */
        else if(pwm_adjust_motor_left > MAX_PWM_DECREMENT){
          pwm_adjust_motor_left--;
        }
      }
      else if(motor_right_total_steps > motor_left_total_steps){
        /* Adjust strength of motor right */
        if(pwm_adjust_motor_left < MAX_PWM_INCREMENT){
          pwm_adjust_motor_left++;
        }
        /* Adjust strength of motor left */
        else if(pwm_adjust_motor_right > MAX_PWM_DECREMENT){
          pwm_adjust_motor_right--;
        }
      }
    }
    
    #ifdef ENABLE_LOG
    /* Print Check between error allowed and speed */
    printLog("(motors_steps_difference = ");
    printLogVar(motors_steps_difference);
    printMem(") <= (MAX_STEPS_ERROR = ");
    printLogVar(MAX_STEPS_ERROR);
    printMemn(")");
    #endif

    /* Wait 2S to perform new adjustment */
    delay(TIME_2S_MS);
    
  } while(motors_steps_difference > MAX_STEPS_ERROR);

  /* Stop pwm to speed */
  acelerateMotors(PWM_STOP_SPEED);
}

/* Synchronize motors */
void oldSyncMotors(){
  analogWrite(SPEED_MOTOR_LEFT, actual_pwm_motor_speed + pwm_adjust_motor_left);
  analogWrite(SPEED_MOTOR_RIGHT, actual_pwm_motor_speed + pwm_adjust_motor_right);
}

#endif

#endif
