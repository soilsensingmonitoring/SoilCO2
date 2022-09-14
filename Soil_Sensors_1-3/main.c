#include "main.h"

uint8_t wireless_buf[35];
uint8_t seesaw_buf[26];
bool i2c_succeed;

#define MODULE_ID 3 // change between each flash!

volatile uint8_t pulses;

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer

    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;

    initClockTo1MHz(); // set SMCLK and CLK up

    uint8_t iteration = 0; // loop variables for servo motors
    wireless_buf[34] = 0;
    seesaw_buf[25] = 0;

    while(1) {

        // power pins for sensors
        P5DIR |= BIT3; //3V3_SENSORS_EN
        P5DIR |= BIT2;
        P4DIR |= BIT5;
        P3DIR |= BIT4; //3V3_LORA_EN
        P3DIR |= BIT6; //SERVO_GPIO_EN

        P5OUT &= ~BIT3;
        P3OUT &= ~BIT4;
        P3OUT &= ~BIT6;
        P5OUT &= ~BIT2;
        P4OUT &= ~BIT5;

        // PWM for servo motor
        P3DIR |= BIT1;
        P3OUT &= ~BIT1;

        // turn on all sensors
        P5OUT |= BIT2;
        P5OUT |= BIT3; //3V3_SENSORS_EN
        //P3OUT |= BIT4; //3V3_LORA_EN
        P3OUT |= BIT6; //SERVO_GPIO_EN
        P4OUT |= BIT5;

        // start up the SPI and I2C bus
        initGPIO_SPI();
        initSPI();
        initGPIO_I2C();
        initI2C();

        __delay_cycles(1000);

        // take moisture reading
        i2c_succeed = false;
        read_temp_seesaw();
        P1OUT &= ~BIT0;
        P6OUT &= ~BIT6;
        i2c_succeed = false;
        read_cap_seesaw();
        P1OUT &= ~BIT0;
        P6OUT &= ~BIT6;

        // take pressure sensor reading
        i2c_succeed = false;
        while (!i2c_succeed)
            init_pressure();
        i2c_succeed = false;
        while (!i2c_succeed)
            read_pressure();

        // take co2 reading
        i2c_succeed = false;
        while (!i2c_succeed)
            init_co2();
        
        
        i2c_succeed = false;
        while (!i2c_succeed)
            read_data_co2();

        // turn off sensors
        P5OUT &= ~BIT2;
        P5OUT &= ~BIT3; //3V3_SENSORS_EN
        P3OUT &= ~BIT4; //3V3_LORA_EN
        P3OUT &= ~BIT6; //SERVO_GPIO_EN
        P4OUT &= ~BIT5;

        // set the module IDs
        wireless_buf[33] = MODULE_ID;
        seesaw_buf[24] = MODULE_ID;

        // send wireless data
        P3OUT |= BIT4; //3V3_LORA_EN
        __delay_cycles(10000);
        while(!init_wireless());
        // must send in two bursts, we have too much data to send all at once
        wireless_send(wireless_buf, 35); // send pressure and CO2 data
        __delay_cycles(500000); // give it some time to send
        wireless_send(seesaw_buf, 26); // send moisture data
        __delay_cycles(100000); // give it some time to send
        P3OUT &= ~BIT4; //3V3_LORA_EN

        pulses = 0;
        if (iteration == 6) // open for 1/2 hour, closed for 1/2 hour
            iteration = 0;

        if (iteration == 0 || iteration == 3) { // only activate the servo if we need to, the disk will give if we turn it on even for a tick
            P5OUT |= BIT3;
            if (iteration == 0) {
                init_timer_B_close();
                wireless_buf[34] = 0;
                seesaw_buf[25] = 0;
            } else { // iteration must be equal to close value
                init_timer_B_open();
                wireless_buf[34] = 1;
                seesaw_buf[25] = 1;
            }
            __bis_SR_register(LPM3_bits);
        }
        iteration++;

        // go to sleep for 10 minutes, turn off all sensors
        sleep_10_min();

        __delay_cycles(10000); // let the device ramp up
    }
}

// RTC interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=RTC_VECTOR
__interrupt void RTC_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(RTC_VECTOR))) RTC_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(RTCIV,RTCIV_RTCIF))
    {
        case  RTCIV_NONE:   break;          // No interrupt
        case  RTCIV_RTCIF:                  // RTC Overflow
            RTCCTL |= RTCSR; // turn off the timer
            __bic_SR_register_on_exit(LPM4_bits);
            break;
        default: break;
    }
}

#pragma vector=TIMER0_B0_VECTOR
__interrupt void ISR_TB0_CCR0(void)
{
    if (pulses == 75) {
        TB0CTL = TBCLR; // reset the peripheral
        __bic_SR_register_on_exit(LPM3_bits);
        P5OUT |= BIT3;
    }
    P3OUT |= BIT1;
    TB0CCTL0 &= ~CCIFG;
    pulses++;
}

#pragma vector=TIMER0_B1_VECTOR
__interrupt void ISR_TB0_CCR1(void)
{
    P3OUT &= ~BIT1;
    TB0CCTL1 &= ~CCIFG;
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER1_B0_VECTOR
__interrupt void Timer1_B0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER1_B0_VECTOR))) Timer1_B0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    __bic_SR_register_on_exit(LPM0_bits);
    MasterMode = TIMEOUT_MODE;
    initGPIO_I2C();
    initI2C();
    TB1CTL = 0; // reset the timer, we'll set it again later
}
