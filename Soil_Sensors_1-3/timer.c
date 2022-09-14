/*
 * timer.c
 *
 *  Created on: Jun 9, 2021
 *      Author: Adam Nygard
 */
#include "timer.h"

void init_timer(void) {
    P2SEL1 |= BIT6 | BIT7;                  // P2.6~P2.7: crystal pins
    do
    {
        CSCTL7 &= ~(XT1OFFG | DCOFFG);      // Clear XT1 and DCO fault flag
        SFRIFG1 &= ~OFIFG;
    } while (SFRIFG1 & OFIFG);

    // RTC count re-load compare value at 32.
    // 1024/32768 * 19200 = 600 sec -> 10 minutes.
    RTCMOD = 19200;
    //RTCMOD = 100;

    // Source = 32kHz crystal, divided by 1024
    RTCCTL = RTCSS__XT1CLK | RTCSR | RTCPS__1024 | RTCIE;
}

void sleep_pins(void) {

    P1DIR |= 0xf3; // don't touch SCL or SDA
    P2DIR = 0x3f; // don't touch the crystal pins
    P3DIR = 0xff;
    P4DIR = 0xff;
    P5DIR = 0xff;
    P6DIR = 0xff;

    P1OUT &= ~0x00;
    P2OUT &= ~0x00;
    P3OUT &= ~0x50;
    P4OUT &= ~0x20;
    P5OUT &= ~0x04;
    P6OUT &= ~0x00;

    UCB0CTLW0 |= UCSWRST; // put euscib0 into reset
    UCA0CTLW0 |= UCSWRST; // put euscia0 into reset
}

void sleep_10_min(void) {

    sleep_pins();

    init_timer();

    __bis_SR_register(LPM4_bits | GIE);     // Enter LPM3, enable interrupt
}

void init_timer_B_open(void) {
    TB0CTL = TBCLR; // reset the peripheral
    TB0CTL |= TBSSEL__ACLK | MC__UP;
    TB0CCR0 = 713; // 21.75ms --> (1/32768) * 713
    TB0CCR1 = 57; // 1.75ms --> (1/32768) * 57
    TB0CCTL0 |= CCIE;
    TB0CCTL1 |= CCIE;
    __enable_interrupt();
    TB0CCTL0 &= ~CCIFG;
    TB0CCTL1 &= ~CCIFG;
}

void init_timer_B_close(void) {
    TB0CTL = TBCLR; // reset the peripheral
    TB0CTL |= TBSSEL__ACLK | MC__UP;
    TB0CCR0 = 680; // 20.75ms --> (1/32768) * 680
    TB0CCR1 = 24; // .75ms --> (1/32768) * 25
    TB0CCTL0 |= CCIE;
    TB0CCTL1 |= CCIE;
    __enable_interrupt();
    TB0CCTL0 &= ~CCIFG;
    TB0CCTL1 &= ~CCIFG;
}

void i2c_timeout(void) {
    // Timer1_B3 setup
    TB1CCTL0 = CCIE;                              // TBCCR0 interrupt enabled
    TB1CCR0 = 65000;
    TB1CTL = TBSSEL_1 | MC_1;                     // ACLK, up mode
}

void cancel_i2c_timeout(void) {
    TB1CTL = 0;
}

