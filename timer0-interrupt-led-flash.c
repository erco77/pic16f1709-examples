/*
 * File:   main.c
 * Author: erco
 *
 * Test interrupts on PIC16f1709: blinks LED at 1Hz on output pin B6.
 * This code specifically builds for MPLABX with XC8 version 2.x.
 * Note XC8 v2.x changed the syntax of interrupt function declarations:
 *
 *    OLD: void interrupt isr(void) {      // nice old syntax..
 *    NEW: void __interrupt() isr(void) {  // weird new syntax in XC8 v2.x
 * 
 * Created on January 17, 2019, 1:49 AM
 */
// This must be #defined before #includes
#define _XTAL_FREQ 500000UL       // system oscillator speed in HZ (__delay_ms() needs this)
#define FOSC4 (_XTAL_FREQ/4)      // instruction clock is 1/4 XTAL speed

// --- The following section copy/pasted from MPLAB X menu: Production -> Set Configuration Bits -> Generate Source..
// CONFIG1
#pragma config FOSC     = INTOSC  // USE INTERNAL OSCILLATOR: Oscillator Selection Bits (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE     = OFF     // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE    = OFF     // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE    = ON      // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP       = OFF     // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN    = OFF     // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = OFF     // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO     = OFF     // Internal/External Switchover Mode (Internal/External Switchover Mode is disabled)
#pragma config FCMEN    = OFF     // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT     = OFF      // Flash Memory Self-Write Protection (Write protection off)
#pragma config PPS1WAY = ON       // Peripheral Pin Select one-way control (The PPSLOCK bit cannot be cleared once it is set by software)
#pragma config ZCDDIS  = ON       // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR)
#pragma config PLLEN   = OFF      // Phase Lock Loop enable (4x PLL is enabled when software sets the SPLLEN bit)
#pragma config STVREN  = ON       // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV    = LO       // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR   = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP     = ON       // Low-Voltage Programming Enable (Low-voltage programming enabled)
// --- end section

#include <xc.h>
#include <pic16f1709.h>

int G_icount = 0;                 // timer0 interrupt counter
int G_intspersec;                 // #ints per second (recalculated by GetIntsPerSec())

// Interrupt service routine
void __interrupt() isr(void) {
    if ( INTCONbits.TMR0IF ) {    // int timer overflow?
        INTCONbits.TMR0IF = 0;    // clear bit for next overflow
        if ( G_icount++ == G_intspersec ) {
            G_icount = 0;
            // DO INTERRUPT STUFF HERE
            LATBbits.LATB6 ^= 1;  // blink LED at 1Hz rate
        }
    }
}

// Return # ints per second, given current clock freq and prescaler
//    Assumes internal xtal w/instruction clock
//
int GetIntsPerSec(int prescaler) {
    return FOSC4 / prescaler / 256;
}

// See p.76 of PIC16F1709 data sheet for other values for OSCCON IRCF<3:0> -erco
#define CPU_CLK_16MHZ  0b1111
#define CPU_CLK_4MHZ   0b1101
#define CPU_CLK_1MHZ   0b1011
#define CPU_CLK_500KHZ 0b0111
//                        \\\\_ IRCF0   \    Together these are
//                         \\\_ IRCF1    |-- the IRCF bits of the
//                          \\_ IRCF2    |    OSCCON register
//                           \_ IRCF3   /

// See p.xx of PIC16F1709 data sheet for other values for PS -erco
#define PS_256  0b111
#define PS_128  0b110
#define PS_4    0b001
#define PS_2    0b000
//                 \\\_ PS0 \    Together these are
//                  \\_ PS1  |-- the PS bits of the
//                   \_ PS2 /    OPTION_REG.
void Init() {
    OSCCONbits.IRCF = CPU_CLK_500KHZ;  // Sets cpu xtal clock speed. If changed, update _XTAL_FREQ @top of file!
    // Initialize for use of TMR0 interrupts
    INTCONbits.GIE        = 1;          // Global Interrupt Enable (GIE)
    INTCONbits.PEIE       = 1;          // PEripheral Interrupt Enable (PEIE)
    INTCONbits.TMR0IE     = 1;          // timer 0 Interrupt Enable (IE)
    INTCONbits.TMR0IF     = 0;          // timer 0 Interrupt Flag (IF)
    // Configure timer
    OPTION_REGbits.TMR0CS = 0;          // set timer 0 Clock Source (CS) to the internal instruction clock
    OPTION_REGbits.TMR0SE = 0;          // Select Edge (SE) to be rising (0=rising edge, 1=falling edge)
    OPTION_REGbits.PSA    = 0;          // PreScaler Assignment (PSA) (0=assigned to timer0, 1=not assigned to timer0)
    // Set timer0 prescaler speed, and set global ints-per-second accordingly
    OPTION_REGbits.PS     = PS_256;     // PreScaler value. If changed, update GetIntsPerSec() argument! (below)
    G_intspersec = GetIntsPerSec(256);
    // Configure digital I/O for all outputs
    TRISA  = 0x0;
    TRISB  = 0x0;
    TRISC  = 0x0;
    ADCON0 = 0x0; // disable analog i/o
}

void main(void) {
    Init();       // initialize
    ei();         // enable ints last
    while(1) { }  // do nothin loop: let ints handle blinking led
}
