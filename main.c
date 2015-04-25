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

extern volatile char RB0InterruptFlag,Timer0InterruptFlag,Timer1InterruptFlag,CaptureInterruptFlag,senseRequest;
unsigned int  encoderCount  = 0;
unsigned char robotDirection;
unsigned int speed, pidOutput;
char str[15];

unsigned int setpoint = 200; // Setpoint for the motors in RPM

/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/

void main(void)
{
    unsigned char stop, no_line;
    int angle;

    local_global_var_init();
    ConfigPorts();
    ConfigMotors();
    ConfigServo();
    OpenTmr0();
    ConfigInterrupts();

    StartCapture(); //capture compare interrupt for encoder
    OpnUSART();
  
    enableMotors;
    //disableMotors;
    SetDCPWM1(950);

    loop_forever{
      if (senseRequest) {
          senseRequest = 0;
          read_angle(&no_line, &angle);
          direction_pid(angle);
          if (no_line)
              stop = check_stop();
          else
              stop = 0;
      }
      if (stop) {
          disableMotors;
      }
      else {
        if(CaptureInterruptFlag){
            CaptureInterruptFlag = 0;
            LATDbits.LATD0 = !LATDbits.LATD0; //toggle IO line to show interrupt
            speed = calculateSpeed();
            pidOutput = calculateSpeedPID(speed, setpoint);
            SetDCPWM1(pidOutput); //duty cycle can range from 0 to 400 (0-100%)
        }
//        Displaying speed through UART
//
//        sprintf(str, "%d", speed);
//        putrs1USART("   Current Speed: ");
//        putrs1USART(str);
//        putrs1USART("\r\n");
//        sprintf(str, "%d", pidOutput);
//        putrs1USART("   PID Output: ");
//        putrs1USART(str);
//        putrs1USART("\r\n");
      }
   }
}





