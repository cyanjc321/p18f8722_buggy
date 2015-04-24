/******************************************************************************/
/*Files to Include                                                            */
/******************************************************************************/
#include <xc.h>
#include "user.h"
#include "system.h"

/******************************************************************************/
/* Global Variables                                                           */
/******************************************************************************/
volatile char  RB0InterruptFlag, Timer0InterruptFlag,Timer1InterruptFlag,CaptureInterruptFlag;
volatile unsigned char received[125]; //Enough to hold UART receive values
volatile unsigned int timerRolloverCount;
volatile unsigned char senseRequest;
volatile unsigned char pwm_state;
volatile unsigned int pulse_ontime;

/******************************************************************************/
/* Interrupt Routines                                                         */
/******************************************************************************/

void interrupt isr(void)
{
  if(ReceiveFlag1){ // USART receive interrupt
      //not using this yet
  }

  if(CCP2IE && CCP2IF){ //Encoder Capture interrupt
      CCP2IF = 0;
      CaptureInterruptFlag = 1;
  }
  
  if(TMR1IE && TMR1IF){ //Timer 0 interrupt
      TMR1IF = 0;  // Clear flag
      timerRolloverCount++;
      LATDbits.LATD0 = !LATDbits.LATD0; //toggle IO line to show interrupt
   }

  if (TMR0IE && TMR0IF) {
      TMR0IF = 0;
      senseRequest = 1;
  }

  if (TMR3IE && TMR3IF) {
      TMR3IF = 0;
      if (pwm_state) {
          pwm_state = 0;
          SERVO_OP = 1;
          TMR3 = 65535 - (PULSE_PERIOD - pulse_ontime);
      }
      else {
          pwm_state = 1;
          SERVO_OP = 0;
          TMR3 = 65535 - (pulse_ontime);
      }
  }
}

