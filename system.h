/******************************************************************************/
/* System Level #define Macros                                                */
/******************************************************************************/

/* TODO Change all these values for your project */

/* Microcontroller MIPs (FCY) */
#define p18f8722         //either write p18f4520 or p18f8722 or your own if added.
#define SYS_FREQ         40000000L  //ONLY 10MHz confirmed working.. have a go
#define _XTAL_FREQ       10000000
#define FCY              SYS_FREQ/4

#define Timer0Period     64558 //10Hz period
 /*40535 comes from, time per instruction = TOSC = 256*4/FOSC
 so for 10MHz FOSC and 256 prescaler time per instruction = 102.4uS.
 So if we want 10Hz that is 976.6 instructions for the delay, 65535-976.6 = 64558*/
#define Timer1Period     40535 //100Hz period
 /*40535 comes from, time per instruction = TOSC = 4/FOSC
 so for 10MHz FSOC and 1 prescaler time per instruction = 400nS.
 So if we want 100Hz that is 25000 instructions for the delay, 65535-25000 = 40535*/

#define BAUD_RATE        9600 //

//Motor board defitions//
#define uniPolar       {LATJbits.LATJ0 = 0;}
#define biPolar        {LATJbits.LATJ0 = 1;}
#define robotForwards  {LATJbits.LATJ1 = 0;}
#define robotBackwards {LATJbits.LATJ1 = 1;}
#define enableMotors   {LATJbits.LATJ4 = 1;}
#define disableMotors  {LATJbits.LATJ4 = 0;}


#ifdef p18f8722
  #define busyUsart        Busy1USART() //For 18F8722 this is Busy1USART()
  #define TransmitBuffer   TXREG1 //For 18F8722 this is TXREG1
  #define ReceiveBuffer    RCREG1 //For 18F8722 this is RXREG1
  #define ReceiveFlag1     PIR1bits.RCIF
  #define TransmitFlag1    PIR1bits.TXIF
  #define Timer0Flag       INTCONbits.TMR0IF
  #define Timer1Flag       PIR1bits.TMR1IF
  #define RB0Flag          INTCONbits.RBIF
#endif
/******************************************************************************/
/* System Function Prototypes                                                 */
/******************************************************************************/

void ConfigInterrupts(void);

void ConfigPorts(void);

void ConfigMotors(void);

void StartCapture(void);

void interrupt isr(void);

void ClsUSART(void);
void OpnUSART(void);

void OpenTmr0(void);
void OpenTmr1(void);

unsigned int calculateSpeed(void);
unsigned int calculateSpeedPID(unsigned int, unsigned int);

