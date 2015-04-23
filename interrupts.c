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

/******************************************************************************/
/* Interrupt Routines                                                         */
/******************************************************************************/

void interrupt isr(void)
{
  if(ReceiveFlag1){ // USART receive interrupt
      //not using this yet
  }

  if(CCP4IE & CCP4IF){ //Encoder Capture interrupt
      CCP4IF = 0;
      CaptureInterruptFlag = 1;
  }
  
  if(TMR0IE && TMR0IF){ //Timer 0 interrupt
      TMR0IF = 0;  // Clear flag
      timerRolloverCount++;
      LATDbits.LATD0 = !LATDbits.LATD0; //toggle IO line to show interrupt
   }
//  if (TMR1IE && TMR1IF){ //Timer 1 interrupt
//    TMR1IF=0;
//    TMR1 = Timer1Period; //set back up for 100Hz
//    Timer1InterruptFlag = 1;
//  }
//
//  if(INTCONbits.INT0IF == 1) //RB0 interrupt
//    {
//        INTCONbits.INT0IF = 0;
//        RB0InterruptFlag = 1;
//    }
  
}
