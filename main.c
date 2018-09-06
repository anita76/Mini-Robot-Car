/*
* Mini Robot Car Code adapted from MSP-EXP430G2-LaunchPad User Experience Application
* for UART control
* main.c
*/
#include "msp430.h"
#define LED1 BIT0
#define LED2 BIT6
#define BUTTON BIT3
#define PreAppMode 0
#define RunningMode 1
// Servo motor definitions
#define MCU_CLOCK 1100000
#define PWM_FREQUENCY 50 // In Hertz
#define SERVO_STEPS 180 // Maximum amount of steps
#define SERVO_MIN 700 // The minimum duty cycle
#define SERVO_MAX 2000 // The maximum duty cycle
volatile unsigned int Mode;
unsigned int PWM_Period = (MCU_CLOCK / PWM_FREQUENCY);
unsigned int PWM_Duty = 0;
unsigned int delayTime = 400; //delay for the windshield wiper
void InitializeButton(void);
void PreApplicationMode(void);
void main(void)
{
float distance;
int count=0;
unsigned int servo_stepval, servo_stepnow;
unsigned int servo_lut[ SERVO_STEPS+1 ];
servo_stepval = ( (SERVO_MAX - SERVO_MIN) / SERVO_STEPS );
servo_stepnow = SERVO_MIN;
unsigned int i;
WDTCTL = WDTPW + WDTHOLD; // Stop WDT
/* next three lines to use internal calibrated 1MHz clock: */
BCSCTL1 = CALBC1_1MHZ; // Set range
DCOCTL = CALDCO_1MHZ;
BCSCTL2 &= ~(DIVS_3); // SMCLK = DCO = 1MHz
InitializeButton();
// setup port for leds:
P1DIR |= LED1 + LED2;
P1OUT &= ~(LED1 + LED2);
P1DIR |= BIT2;
P1OUT |= BIT2;
Mode = PreAppMode;
PreApplicationMode(); // Blinks LEDs, waits for button press
__enable_interrupt(); // Enable interrupts.
// Fill up the LUT
for (i = 0; i < SERVO_STEPS; i++) {
servo_stepnow += servo_stepval;
servo_lut[i] = servo_stepnow;
}
// TimerA.0 setup for PWM to use with the windshield wiper
TA0CCTL1 = OUTMOD_7; // TACCR1 reset/set
TA0CTL = TASSEL_2 + MC_1; // SMCLK, upmode
TA0CCR0 = PWM_Period-1; // PWM Period
TA0CCR1 = PWM_Duty;
//setup the ADC
ADC10CTL0 = ADC10SHT_2 + ADC10ON;
ADC10CTL1 = INCH_1;
ADC10AE0 |= 0x02; //p1.1 is ADC input read for rain sensor
P1DIR |= BIT2; // P1.2 is servo motor output
P1DIR &= ~BIT1; // P1.1 is rain sensor input
P1SEL |= BIT2; // P1.2 = TA1 output
P1DIR |= BIT0;
P1DIR |= BIT6;
P1OUT &= ~BIT0; //turns off when wiper is on
P1OUT &= ~BIT6;
// Timer A.1 setup for using counter
TA1CTL = TASSEL_2 | MC_2 | ID_0;
// Setup P2 pins to send signal to H-Bridge and run the motor
// pin 2 is used to determine whether it is automatic (low) or remote (high)
P2DIR = 0b00011011;
P2OUT = 0x0;
P1DIR |= BIT4;
P1DIR &= ~BIT5;
P1DIR &= ~BIT7;
/* Main Application Loop */
while(1)
{
if(!(P2IN & BIT2)){//automatic mode
//ADC reading
ADC10CTL0 |= ENC + ADC10SC;
while (ADC10CTL1 &ADC10BUSY);
// compute the time using ultrasonic sensor data
delayTime = 400*(((float)ADC10MEM)/((float)0x3a0))*(((float)ADC10MEM)/((float)0x3a0));
if(ADC10MEM<0x3a0){ //if sensed rain
P1OUT &= ~BIT0;
// move the wiper forward
for (i = 0; i < SERVO_STEPS; i++) {
TA0CCR1 = servo_lut[i];
__delay_cycles(2000);
}
// Move wiper backward
for (i = SERVO_STEPS; i > 0; i--) {
TA0CCR1 = servo_lut[i];
__delay_cycles(2000);
}
//delay
for(i =0; i<delayTime; i++){
__delay_cycles(5000);
}
}else{ //didn't sense rain
P1OUT|=BIT0;
}
// send signal to the ultrasonic sensor
P1OUT|=BIT4;
__delay_cycles(10);
P1OUT&= ~BIT4;
// measure time of signal sent by sensor
while(!(P1IN&BIT5));
TA1R =0;
while((P1IN&BIT5));
distance=((float) TA1R)/((float)58.0);
if(distance <=30 && distance >10){//turn right
P2OUT = 0b00000010;
}else if(distance <=15){//reverse
P2OUT = 0b00001001;
}else{//move forward
P2OUT = 0b00010010;
}
__delay_cycles(60000);
}else{//remote control mode
while(P1IN&BIT7);//IR signal not received
count = (count +1)%3;
switch (count) {
case 1:
P2OUT = 0b00010010;//move forward
break;
case 2:
P2OUT = 0b00000010;//turn
break;
case 0:
P2OUT = 0b00000000;//stop
break;
}
for(int i=0; i<100; i++){
__delay_cycles(20000);
}
}
}
}
void PreApplicationMode(void)
{
P1DIR |= LED1 + LED2;
P1OUT |= LED1; // To enable the LED toggling effect
P1OUT &= ~LED2;
/* these next two lines configure the ACLK signal to come from
a secondary oscillator source, called VLO */
BCSCTL1 |= DIVA_1; // ACLK is half the speed of the source (VLO)
BCSCTL3 |= LFXT1S_2; // ACLK = VLO
/* here we're setting up a timer to fire an interrupt periodically.
When the timer 1 hits its limit, the interrupt will toggle the lights
We're using ACLK as the timer source, since it lets us go into LPM3
(where SMCLK and MCLK are turned off). */
TACCR0 = 1200; // period
TACTL = TASSEL_1 | MC_1; // TACLK = ACLK, Up mode.
TACCTL1 = CCIE + OUTMOD_3; // TACCTL1 Capture Compare
TACCR1 = 600; // duty cycle
__bis_SR_register(LPM3_bits + GIE); // LPM3 with interrupts enabled
// in LPM3, MCLCK and SMCLK are off, but ACLK is on.
}
// this gets used in pre-application mode only to toggle the lights:
#if defined(__TI_COMPILER_VERSION__)
#pragma vector=TIMER0_A1_VECTOR
__interrupt void ta1_isr (void)
#else
void __attribute__ ((interrupt(TIMER0_A1_VECTOR))) ta1_isr (void)
#endif
{
TACCTL1 &= ~CCIFG; // reset the interrupt flag
if (Mode == PreAppMode){
P1OUT ^= (LED1 + LED2); // toggle the two lights.
}
else{
TACCTL1 = 0; // no more interrupts.
__bic_SR_register_on_exit(LPM3_bits); // Restart the cpu
}
}
void InitializeButton(void) // Configure Push Button
{
P1DIR &= ~BUTTON;
P1OUT |= BUTTON;
P1REN |= BUTTON;
P1IES |= BUTTON;
P1IFG &= ~BUTTON;
P1IE |= BUTTON;
}
/* *************************************************************
* Port Interrupt for Button Press
* 1. During standby mode: to enter application mode
*
* *********************************************************** */
#if defined(__TI_COMPILER_VERSION__)
#pragma vector=PORT1_VECTOR
__interrupt void port1_isr(void)
#else
void __attribute__ ((interrupt(PORT1_VECTOR))) port1_isr (void)
#endif
{
/* this disables port1 interrupts for a little while so that 
we don't try to respond to two consecutive button pushes right together.
The watchdog timer interrupt will re-enable port1 interrupts
This whole watchdog thing is completely unnecessary here, but its useful
to see how it is done.
*/
P1IFG = 0; // clear out interrupt flag
P1IE &= ~BUTTON; // Disable port 1 interrupts
WDTCTL = WDT_ADLY_250; // set up watchdog timer duration
IFG1 &= ~WDTIFG; // clear interrupt flag
IE1 |= WDTIE; // enable watchdog interrupts
TACCTL1 = 0; // turn off timer 1 interrupts
P1OUT &= ~(LED1+LED2); // turn off the leds
Mode = RunningMode;
__bic_SR_register_on_exit(LPM3_bits); // take us out of low power mode
}
// WDT Interrupt Service Routine used to de-bounce button press
#if defined(__TI_COMPILER_VERSION__)
#pragma vector=WDT_VECTOR
__interrupt void wdt_isr(void)
#else
void __attribute__ ((interrupt(WDT_VECTOR))) wdt_isr (void)
#endif
{
IE1 &= ~WDTIE; /* disable watchdog interrupt */
IFG1 &= ~WDTIFG; /* clear interrupt flag */
WDTCTL = WDTPW + WDTHOLD; /* put WDT back in hold state */
P1IE |= BUTTON; /* Debouncing complete - reenable port 1 interrupts*/
}