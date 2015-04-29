/******************************************************************************/
/*Files to Include                                                            */
/******************************************************************************/

#include <xc.h>         /* XC8 General Include File */
#include <stdint.h>         /* For uint8_t definition */
#include <stdbool.h>        /* For true/false definition */
#include <plib.h>
#include <p18f8722.h>

#include "system.h"

static int last_angle;
static int angle_integral;
static unsigned char sensor_reading[6];


//PLEASE NOTE THESE FUNCTIONS ONLY WORK FOR 18F4520
void OpnUSART(void)
{
/******************************************************************************/
/* This just sets up the relevant parameters for the UART                     */
/******************************************************************************/
  //-----SPBRG needs to be changed depending upon oscillator frequency-------
  unsigned long spbrg = 0;

  spbrg = SYS_FREQ/BAUD_RATE; // define these in system.h
  spbrg /= 64;
  spbrg -= 1;
  ClsUSART();    //Incase USART already opened

  #ifdef p18f8722
      TXSTA1 = 0;           // Reset USART registers to POR state
      RCSTA1 = 0;

      TRISCbits.TRISC6 = 0; // Sets transmit pin as output
      TRISCbits.TRISC7 = 1; // Sets recieve pin as input

      TXSTA1bits.SYNC = 0;  // Async mode
      TXSTA1bits.TX9 = 0;   // 8-bit mode
      TXSTA1bits.SENDB = 0; // Sync break option (not sure)
      TXSTA1bits.BRGH = 0;  // Low speed

      RCSTA1bits.CREN = 1;  //Continuous receive
      RCSTA1bits.RX9 = 0;   //8-bit reception

      PIE1bits.RCIE = 1;   // Interrupt on receive
      PIE1bits.TXIE = 0;   // Interrupt on transmit

      BAUDCON1bits.BRG16 = 0; // 8-bit Baud register
      SPBRG = spbrg;         // Write baudrate to SPBRG1

      TXSTA1bits.TXEN = 1;  // Enable transmitter
      RCSTA1bits.SPEN = 1;  // Enable receiver
  #endif
}

void ClsUSART(void)
{
  #ifdef p18f4520
    RCSTA &= 0b01001111;  // Disable the receiver
    TXSTAbits.TXEN = 0;   //  and transmitter
    PIE1 &= 0b11001111;   // Disable both interrupts
  #endif
  #ifdef p18f8722
    RCSTA1 &= 0b01001111;  // Disable the receiver
    TXSTA1bits.TXEN = 0;   //  and transmitter
    PIE1 &= 0b11001111;   // Disable both interrupts
  #endif
}

void OpenTmr0(void)
{
    T0CONbits.TMR0ON = 0;   // timer off
    T0CONbits.T08BIT = 0;   // 16-bit operation
    T0CONbits.T0CS = 0;     //Internal clock
    T0CONbits.PSA = 256;    // prescaler 1:256

    TMR0 = TMRPeriod_ms_to_instr(Timer0Period_ms, 256, 16);
    
    INTCONbits.TMR0IF = 0;  // Clear Timer0 overflow flag
    INTCONbits.TMR0IE = 1;  // Enable Timer0 overflow interrupt
    T0CONbits.TMR0ON = 1;
}


void ConfigInterrupts(void)
{
    INTCON2bits.NOT_RBPU = 1;
    INTCONbits.INT0IE = 1; //enable Interrupt 0 (RB0 as interrupt)
    INTCON2bits.INTEDG0 = 1; //cause interrupt at rising edge
    INTCONbits.INT0IF = 0; //reset interrupt flag
    INTCONbits.PEIE = 1;
    ei();
}

void ConfigPorts(void)
{
/******************************************************************************/
/* General port configuration, a lot of this is duplicated in other functions */
/******************************************************************************/
    TRISBbits.RB0 = 1; //set RB0 as Input
    TRISJbits.TRISJ2 = 0; //set RJ2 as output
    TRISDbits.TRISD0 = 0; //set RD0 as output
    TRISGbits.TRISG3 = 1; //input for encoder capture
    TRISGbits.RG0 = 0;
}

void read_adc(void) {
    int i;
    static const char sensor_ch[] = {ADC_CH0, ADC_CH12, ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4};
    for (i = 0; i < 6; i++) {
        SetChanADC(sensor_ch[i]);
        ConvertADC();
        while (BusyADC());
        sensor_reading[i] = ReadADC() >> 2;
    }
}

void StartCapture(void)
{
/******************************************************************************/
/* This just configs the encoder to fire an interrupt on every 16'th rising   */
/* edge of pin RG3                                                            */
/******************************************************************************/
    TRISGbits.TRISG3 = 1; //input for encoder capture
    OpenTimer3(TIMER_INT_ON & T3_16BIT_RW & T3_SOURCE_INT & T3_PS_1_8
            & T3_SYNC_EXT_OFF & T12_CCP12_T34_CCP345);
    OpenCapture4(CAPTURE_INT_ON & C4_EVERY_16_RISE_EDGE);
}

void ConfigMotors(void)
/******************************************************************************/
/* This just sets up the relevant stuff for the motors                        */
/******************************************************************************/
{
  //Initial config

  TRISJ = 0x00; //All motor config pins are outputs
  uniPolar;
  robotForwards;
  disableMotors;

  TRISCbits.TRISC2 = 0; //Sets CCP1 as output MAIN MOTOR
  //TRISEbits.TRISE7 = 0; //Sets CCP2 as output
  //TRISGbits.TRISG3 = 0; //Sets CCP4 as output pin 8
  //TRISGbits.TRISG4 = 0; //Sets CCP5 as output pin 10
  OpenTimer2(TIMER_INT_OFF & T2_PS_1_1 & T2_POST_1_1 & T12_CCP12_T34_CCP345);
  OpenPWM1(254); //40KHz PWM CCP1 RC2
}

void ConfigServo(void) {
    OpenTimer1(TIMER_INT_ON & T1_16BIT_RW & T1_PS_1_8 & T1_SOURCE_INT & T1_SYNC_EXT_OFF & T1_OSC1EN_OFF & T12_CCP12_T34_CCP345);
    pulse_ontime = PULSE_MID;
    pwm_state = 1;
    SERVO_OP = 0;
    TMR1 = 65535 - (pulse_ontime);
}

unsigned int calculateSpeed(void)
{
/* "time" is the time it took to do 1.33 revolutions of the wheel
 * because we are running at PLL (40MHz) and 256 PS, TMR0 rolls over every 1.67s
 * with a time per click time of 25.6uS. 1/25.6uS = 39062.5
 * 0.75 because 12 holes in encoder, interrupt every 16 clicks, 16/12 = 1.33
 * 1/1.33 = 0.75 so if we multiply time by that we get time per revolution */
    unsigned int speed;
    static unsigned long time_long;
    float time;

    time = (float)(1.67 * timerRolloverCount)+(TMR3/39062.5);
    time = 1/(14*(0.75 * time)); //rps remember 14:1 gearing
    speed = (unsigned int)(60*time); //RPM

    //clear rollover count and timer
    timerRolloverCount = 0;
    TMR3 = 0;
    return speed;
}

unsigned int calculateSpeedPID(unsigned int currentValue, unsigned int setpoint)
{
    static float integral   = 0;
    static float derivative = 0;
    static float lastError = 0;

    static float pid = 0;
    unsigned int controlOutput = 0;
    static int error;
    
    error = (int)setpoint - currentValue;

    integral   = integral + error;
    integral   = 0.6 * integral;    //damping integral
    derivative = error - lastError;

    pid =  (SPEED_KP * error);
    pid += (SPEED_KD * derivative);
    pid += (SPEED_KI * integral);

    lastError = error;

    if(pid > 1023){pid = 1023;}
    else if (pid < 0){pid = 0;}

    controlOutput = (unsigned int)pid;
    controlOutput = 1023-controlOutput;
    return controlOutput;
}

unsigned int TMRPeriod_ms_to_instr(unsigned int ms, unsigned int prescaler, unsigned char resolution) {
    unsigned long instr;
    unsigned long ret;
    instr = (unsigned long)ms * (SYS_FREQ / 1000) / prescaler / 4;
    ret = 1 << resolution - 1;
    ret = ret - instr;
    return (unsigned int)ret;
}

void config_sensor_digital(void) {
    ADCON1 = 0xff;
    TRISH = 0xff;
    TRISA = 0xff;
}

void config_sensor_analog(void) {
    OpenADC(ADC_FOSC_4 & ADC_RIGHT_JUST & ADC_20_TAD,
            ADC_CH0 & ADC_INT_OFF & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS,
            0);
}
void read_sensor_digital(unsigned char* no_line, int* angle_error){
    unsigned char reading_a, reading_h, reading, i;
    reading_a = PORTA & 0b00101111;
    reading_h = PORTH & 0b00010000;
    reading = (reading_a & 0b00001110) << 1;
    reading = reading | (reading_a & 0b00100001);
    reading = reading | (reading_h >> 3);
    reading = ~reading;
    reading = reading & 0b00111111;
    *angle_error = 0;
    *no_line = !reading;
    if (*no_line)
        *angle_error = last_angle;
    else
        for (i = 0; i < 6; i++)
            if ((reading >> i) & 0x1)
                *angle_error += ((int)sw_d[i]);
}

void read_sensor_analog(unsigned char* no_line, int* angle_error) {
}

unsigned char check_stop(){
    return (angle_integral < STOP_ERROR_MIN) ? 1 : 0;
}

void local_global_var_init() {
    angle_integral = 0;
    last_angle = 0;
}

void direction_pid(int angle_error){
    int angle_derivative;
    float servo_angle;

    angle_integral += angle_error;
    angle_derivative = angle_error - last_angle;

    servo_angle = SERVO_KP * angle_error + SERVO_KI * angle_integral + SERVO_KD * angle_derivative;
    if (servo_angle < -60)     servo_angle = -60;
    else
        if (servo_angle > 60)  servo_angle = 60;
    pulse_ontime = (unsigned int)(PULSE_MID - PULSE_ANGLE_RATIO * servo_angle);

    last_angle = angle_error;
}