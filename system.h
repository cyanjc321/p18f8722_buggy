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
#define Timer0Period_ms     1

#define STOP_ERROR_MIN     8

#define BAUD_RATE        9600 //

//const char sensor_weight[] = {-43, -26, -7, 6, 27, 43};  //adjust these value according to relative spacing between sensors
/* sensor config: pair 6 5 4 3 2 1 */
const signed char sw_d[] = {43, 27, 6, -7, -26, -43}; //weighted according to distance
const signed char sw_a[] = {0};

#define SPEED_SETPT 400; // Setpoint for the motors in RPM
#define SPEED_MIN_PER 50;
#define SPEED_KP    1.0
#define SPEED_KI    1.0
#define SPEED_KD    0.0

#define SERVO_KP    0.70
#define SERVO_KI    0.04
#define SERVO_KD    0.0

//servo output pin
#define SERVO_OP       LATGbits.LATG0

/* pulse period = 8 * 4 / FOSC * 5500 = 4400 us */
#define PULSE_PERIOD    5500
#define PULSE_MIN       1250    //-60 degree
#define PULSE_MAX       2250    //60 degree
#define PULSE_MID       1700    //0 degree
#define PULSE_ANGLE_RATIO 10.416 //timer cnt per degree


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
/* Global Variables                                                           */
/******************************************************************************/
volatile char CaptureInterruptFlag;
volatile unsigned char received[125]; //Enough to hold UART receive values
volatile unsigned int timerRolloverCount, capture_cnt;
volatile unsigned char senseRequest;
volatile unsigned char pwm_state;
volatile unsigned int pulse_ontime;
int angle_derivative;

/******************************************************************************/
/* System Function Prototypes                                                 */
/******************************************************************************/

//config
void ConfigInterrupts(void);
void ConfigPorts(void);
void ConfigMotors(void);
void ConfigServo(void);
void StartCapture(void);
void OpenTmr0(void);
void config_sensor_digital(void);
void config_sensor_analog(void);

//io
void read_sensor_digital(unsigned char* no_line, int* angle_error);
void read_sensor_analog(unsigned char* no_line, int* angle_error);

//pid
void direction_pid(int angle);
unsigned int calculateSpeedPID(unsigned int, unsigned int);

//mis
unsigned int calculateSpeed(void);
unsigned int calculateSetpt(int angle);
unsigned char check_stop(void);
unsigned int TMRPeriod_ms_to_instr(unsigned int ms, unsigned int prescaler, unsigned char resolution);
void local_global_var_init(void);