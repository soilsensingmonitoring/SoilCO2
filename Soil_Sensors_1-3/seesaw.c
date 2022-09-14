/*
 * seesaw.c
 *
 *  Created on: Jun 2, 2021
 *      Author: Dragyn
 */

#include "main.h"

//changed to one sensor
#define NUM_SEESAWS 1

void read_temp_seesaw() {

    uint8_t buf[4];
    uint8_t i = 0;
    for (i = 0; i < NUM_SEESAWS; i++) {
        if (i == 0) {
            P1OUT |= BIT0;
            P6OUT &= ~BIT6;
        } else if (i == 1) {
            P1OUT &= ~BIT0;
            P6OUT |= BIT6;
        } else if (i == 2) {
            P1OUT |= BIT0;
            P6OUT |= BIT6;
        }

        if (I2C_Master_ReadReg(SOIL_SENSOR_ADDR + i, SEESAW_STATUS_TEMP, true, 4) == IDLE_MODE) {

            CopyArray(ReceiveBuffer, buf, 4);

            if (buf[0] == 0) { // if we read inaccurately, try again
                i2c_succeed = true;
            }
        }

        if (i2c_succeed) {
            seesaw_buf[0 + (4 * i)] = buf[0];
            seesaw_buf[1 + (4 * i)] = buf[1];
            seesaw_buf[2 + (4 * i)] = buf[2];
            seesaw_buf[3 + (4 * i)] = buf[3];
            i2c_succeed = false;
        } else { // reset the i2c bus
            initGPIO_I2C();
            initI2C();
            __delay_cycles(1000);
            i--;
        }
    }

}

void read_cap_seesaw() {

    uint8_t buf[2];
    uint8_t i = 0;
    for (i = 0; i < NUM_SEESAWS; i++) {
        if (i == 0) {
            P1OUT |= BIT0;
            P6OUT &= ~BIT6;
        } else if (i == 1) {
            P1OUT &= ~BIT0;
            P6OUT |= BIT6;
        } else if (i == 2) {
            P1OUT |= BIT0;
            P6OUT |= BIT6;
        }

        if (I2C_Master_WriteReg(SOIL_SENSOR_ADDR + i, SEESAW_TOUCH_CHANNEL, true, 0, 0) != IDLE_MODE) {

            __delay_cycles(5000);

            if(I2C_Master_ReadIntoBuffer(SOIL_SENSOR_ADDR + i, 2) == IDLE_MODE) {
                CopyArray(ReceiveBuffer, buf, 2);

                uint16_t moisture = (((uint16_t)buf[0]) << 8) | (uint16_t)buf[1];

                if (moisture > 200 && moisture < 2000) // once we get our first accurate measurement, break out
                    i2c_succeed = true;
            }
        }
        if (i2c_succeed) {
            seesaw_buf[16 + (2 * i)] = buf[0];
            seesaw_buf[17 + (2 * i)] = buf[1];
            i2c_succeed = false;
        } else {
            initGPIO_I2C();
            initI2C();
            __delay_cycles(1000);
            i--;
        }
    }
}

