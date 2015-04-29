/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#include <xc.h>            /* XC8 General Include File */
#include <stdint.h>        /* For uint8_t definition */
#include <stdbool.h>       /* For true/false definition */
#include <stdio.h>
#include <stdlib.h>
#include <plib.h>
#include <usart.h>
#include "system.h"        /* System funct/params, like osc/peripheral config */
#include "user.h"          /* User funct/params, such as InitApp */
#include <timers.h>
#include <p18f8722.h>

/******************************************************************************/
/* Definitions                                                                */
/******************************************************************************/
#define loop_forever while (1)

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

unsigned char robotDirection;
unsigned int speed, pidOutput;
char str[15];
unsigned int setpoint;

/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/

void main(void)
{
    unsigned char stop, no_line;
    int angle = 0;

    local_global_var_init();
    ConfigPorts();
    config_sensor_analog();
    ConfigMotors();
    ConfigServo();
    OpenTmr0();
    ConfigInterrupts();

    StartCapture(); //capture compare interrupt for encoder
  
    enableMotors;
    SetDCPWM5(800);
    loop_forever{
      senseRequest = 1;
      if (senseRequest) {
          senseRequest = 0;
          read_sensor_analog(&no_line, &angle);
          direction_pid(angle);
          if (no_line){
              stop = check_stop();
          }
          else
              stop = 0;
      }
      
      if (stop){
          disableMotors;
      }
      else {
        enableMotors;
        if(CaptureInterruptFlag){
            CaptureInterruptFlag = 0;
            LATDbits.LATD0 = !LATDbits.LATD0; //toggle IO line to show interrupt
            speed = calculateSpeed();
            setpoint = calculateSetpt(angle);
            pidOutput = calculateSpeedPID(speed, setpoint);
            SetDCPWM5(pidOutput); //duty cycle can range from 0 to 400 (0-100%)
        }
      }
   }
}

void interrupt isr(void)
{
  if(ReceiveFlag1){ // USART receive interrupt
      //not using this yet
  }

  if(CCP4IE && CCP4IF){ //Encoder Capture interrupt
      CCP4IF = 0;
      capture_cnt ++;
  }

  if(TMR3IE && TMR3IF){ //Timer 0 interrupt
      TMR3IF = 0;  // Clear flag
      LATDbits.LATD0 = !LATDbits.LATD0; //toggle IO line to show interrupt
      timerRolloverCount++;
      if (timerRolloverCount >= 10)
        CaptureInterruptFlag = 1;
   }

  if (TMR0IE && TMR0IF) {
      TMR0IF = 0;
      senseRequest = 1;
  }

  if (TMR1IE && TMR1IF) {
      TMR1IF = 0;
      if (pwm_state) {
          pwm_state = 0;
          SERVO_OP = 1;
          TMR1 = 65535 - (PULSE_PERIOD - pulse_ontime);
      }
      else {
          pwm_state = 1;
          SERVO_OP = 0;
          TMR1 = 65535 - (pulse_ontime);
      }
  }
}




