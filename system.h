/******************************************************************************/
/* System Level #define Macros                                                */
/******************************************************************************/

/* TODO Change all these values for your project */

/* Microcontroller MIPs (FCY) */
#define p18f8722         //either write p18f4520 or p18f8722 or your own if added.
#define SYS_FREQ         40000000L  //ONLY 10MHz confirmed working.. have a go
#define _XTAL_FREQ       10000000
#define FCY              SYS_FREQ/4

/* time per instruction = TOSC = 256*4/FOSC */
/* timer overflow period in ms*/
#define Timer0Period_ms     10

#define STOP_ERROR_MIN     10

#define BAUD_RATE        9600 //

const char sensor_weight[] = {-3, -2, -1, 1, 2, 3};  //adjust these value according to relative spacing between sensors

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

unsigned int calculateSpeed(void);
unsigned int calculateSpeedPID(unsigned int, unsigned int);

void poll_angle(unsigned char* no_line, int* angle);
unsigned char check_stop(int angle);

unsigned int TMRPeriod_ms_to_instr(unsigned int ms, unsigned int prescaler, unsigned char resolution);
