/*
 * pressure.c
 *
 *  Created on: Jun 4, 2021
 *      Author: Adam Nygard
 */
#include "main.h"

/*
 * Reads the coefficient registers associated with the pressure reading
 * Initializes the sensor for reading
 */
void init_pressure() {

    uint8_t buf[1] = {0x00};

    __delay_cycles(1000); // wait 1 ms - give the sensor some time to initialize

    while(!(buf[0] & 0x40)) { //  wait until sensor is ready
        if (I2C_Master_ReadReg(PRESSURE_I2C_ADDR, MEAS_CFG, false, 1) == IDLE_MODE)
            CopyArray(ReceiveBuffer, buf, 1);
        else {
            initGPIO_I2C();
            initI2C();
            __delay_cycles(1000);
            return;
        }
    }

/*
    I2C_Master_ReadReg(PRESSURE_I2C_ADDR, 0x0C, false, 1); // copy existing bits
    CopyArray(ReceiveBuffer, buf, 1);
    buf[0] = buf[0] | 0x09; // generate software reset
    I2C_Master_WriteReg(PRESSURE_I2C_ADDR, 0x0C, false, buf, 1);
*/

    while(!(buf[0] & 0x80)) { //  wait until coefficients are ready
        while(I2C_Master_ReadReg(PRESSURE_I2C_ADDR, MEAS_CFG, false, 1) != IDLE_MODE){}
        CopyArray(ReceiveBuffer, buf, 1);
    }

    // start reading coefficients
    while(I2C_Master_ReadReg(PRESSURE_I2C_ADDR, 0x13, false, 1) != IDLE_MODE){}
    CopyArray(ReceiveBuffer, buf, 1);
    wireless_buf[0] = buf[0]; // c00_mmsb

    while(I2C_Master_ReadReg(PRESSURE_I2C_ADDR, 0x14, false, 1) != IDLE_MODE){}
    CopyArray(ReceiveBuffer, buf, 1);
    wireless_buf[1] = buf[0]; // c00_mid

    while(I2C_Master_ReadReg(PRESSURE_I2C_ADDR, 0x15, false, 1) != IDLE_MODE){}
    CopyArray(ReceiveBuffer, buf, 1);
    wireless_buf[2] = buf[0]; // c00 and c10 bits

    while(I2C_Master_ReadReg(PRESSURE_I2C_ADDR, 0x16, false, 1) != IDLE_MODE){}
    CopyArray(ReceiveBuffer, buf, 1);
    wireless_buf[3] = buf[0]; // c10_mid

    while(I2C_Master_ReadReg(PRESSURE_I2C_ADDR, 0x17, false, 1) != IDLE_MODE){}
    CopyArray(ReceiveBuffer, buf, 1);
    wireless_buf[4] = buf[0]; // c10 lsb

    while(I2C_Master_ReadReg(PRESSURE_I2C_ADDR, 0x18, false, 1) != IDLE_MODE){}
    CopyArray(ReceiveBuffer, buf, 1);
    wireless_buf[5] = buf[0]; // c01_msb

    while(I2C_Master_ReadReg(PRESSURE_I2C_ADDR, 0x19, false, 1) != IDLE_MODE){}
    CopyArray(ReceiveBuffer, buf, 1);
    wireless_buf[6] = buf[0]; // c01_lsb

    while(I2C_Master_ReadReg(PRESSURE_I2C_ADDR, 0x1A, false, 1) != IDLE_MODE){}
    CopyArray(ReceiveBuffer, buf, 1);
    wireless_buf[7] = buf[0]; // c11_msb

    while(I2C_Master_ReadReg(PRESSURE_I2C_ADDR, 0x1B, false, 1) != IDLE_MODE){}
    CopyArray(ReceiveBuffer, buf, 1);
    wireless_buf[8] = buf[0]; // c11_lsb

    while(I2C_Master_ReadReg(PRESSURE_I2C_ADDR, 0x1C, false, 1) != IDLE_MODE){}
    CopyArray(ReceiveBuffer, buf, 1);
    wireless_buf[9] = buf[0]; // c20_msb

    while(I2C_Master_ReadReg(PRESSURE_I2C_ADDR, 0x1D, false, 1) != IDLE_MODE){}
    CopyArray(ReceiveBuffer, buf, 1);
    wireless_buf[10] = buf[0]; // c20_lsb

    while(I2C_Master_ReadReg(PRESSURE_I2C_ADDR, 0x1E, false, 1) != IDLE_MODE){}
    CopyArray(ReceiveBuffer, buf, 1);
    wireless_buf[11] = buf[0]; //c21_msb

    while(I2C_Master_ReadReg(PRESSURE_I2C_ADDR, 0x1F, false, 1) != IDLE_MODE){}
    CopyArray(ReceiveBuffer, buf, 1);
    wireless_buf[12] = buf[0]; // c21_lsb

    while(I2C_Master_ReadReg(PRESSURE_I2C_ADDR, 0x20, false, 1) != IDLE_MODE){}
    CopyArray(ReceiveBuffer, buf, 1);
    wireless_buf[13] = buf[0]; // c30_msb

    while(I2C_Master_ReadReg(PRESSURE_I2C_ADDR, 0x21, false, 1) != IDLE_MODE){}
    CopyArray(ReceiveBuffer, buf, 1);
    wireless_buf[14] = buf[0]; // c30_lsb

    while(I2C_Master_ReadReg(PRESSURE_I2C_ADDR, PRS_CFG, false, 1) != IDLE_MODE){}
    CopyArray(ReceiveBuffer, buf, 1);
    buf[0] = (buf[0] & 0x80) | 0x01; // clear everything but the reserved bit before writing
    // 1b0 - reserved, 3b000 - reserved for background mode only, 4b0001 - 2x oversampling sample rate (low power)
    while(I2C_Master_WriteReg(PRESSURE_I2C_ADDR, PRS_CFG, false, buf, 1) != IDLE_MODE){} // datasheet recommends reading 0x10 to 0x20 on start

    while(I2C_Master_ReadReg(PRESSURE_I2C_ADDR, 0x28, false, 1) != IDLE_MODE){} // copy the coefficient source for the temperature calibration
    CopyArray(ReceiveBuffer, buf, 1);
    buf[0] &= 0x80; // clear everything but the coefficient bit
    while(I2C_Master_WriteReg(PRESSURE_I2C_ADDR, TMP_CFG, false, buf, 1) != IDLE_MODE){}

    buf[0] = 0x00; // put the device in idle mode, disable interrupts
    while(I2C_Master_WriteReg(PRESSURE_I2C_ADDR, CFG_REG, false, buf, 1) != IDLE_MODE){}

    while(I2C_Master_ReadReg(PRESSURE_I2C_ADDR, MEAS_CFG, false, 1) != IDLE_MODE){}
    CopyArray(ReceiveBuffer, buf, 1);
    buf[0] &= 0x08; // clear everything but the reserved bit
    while(I2C_Master_WriteReg(PRESSURE_I2C_ADDR, MEAS_CFG, false, buf, 1) != IDLE_MODE){}

    i2c_succeed = true;
}

void read_pressure() {

    uint8_t buf[1];

    if (I2C_Master_ReadReg(PRESSURE_I2C_ADDR, MEAS_CFG, false, 1) == IDLE_MODE) {
        CopyArray(ReceiveBuffer, buf, 1);
    } else {
        initGPIO_I2C();
        initI2C();
        __delay_cycles(1000);
        return;
    }
    buf[0] |= 0x01; // take a pressure measurement
    while(I2C_Master_WriteReg(PRESSURE_I2C_ADDR, MEAS_CFG, false, buf, 1) != IDLE_MODE){}

    // check if a pressure measurement is ready
    while(!(buf[0] & 0x10)) {
        I2C_Master_ReadReg(PRESSURE_I2C_ADDR, MEAS_CFG, false, 1);
        CopyArray(ReceiveBuffer, buf, 1);
    }

    while(I2C_Master_ReadReg(PRESSURE_I2C_ADDR, PSR_B2, false, 1) != IDLE_MODE){}
    CopyArray(ReceiveBuffer, buf, 1);
    wireless_buf[15] = buf[0];

    while(I2C_Master_ReadReg(PRESSURE_I2C_ADDR, PSR_B1, false, 1) != IDLE_MODE){}
    CopyArray(ReceiveBuffer, buf, 1);
    wireless_buf[16] = buf[0];

    while(I2C_Master_ReadReg(PRESSURE_I2C_ADDR, PSR_B0, false, 1) != IDLE_MODE){}
    CopyArray(ReceiveBuffer, buf, 1);
    wireless_buf[17] = buf[0];

    while(I2C_Master_ReadReg(PRESSURE_I2C_ADDR, MEAS_CFG, false, 1) != IDLE_MODE){}
    CopyArray(ReceiveBuffer, buf, 1);
    buf[0] |= 0x02; // take a temp measurement
    while(I2C_Master_WriteReg(PRESSURE_I2C_ADDR, MEAS_CFG, false, buf, 1) != IDLE_MODE){}

    // check if a temp measurement is ready
    while(!(buf[0] & 0x20)) {
        I2C_Master_ReadReg(PRESSURE_I2C_ADDR, MEAS_CFG, false, 1);
        CopyArray(ReceiveBuffer, buf, 1);
    }

    while(I2C_Master_ReadReg(PRESSURE_I2C_ADDR, TMP_B2, false, 1) != IDLE_MODE){}
    CopyArray(ReceiveBuffer, buf, 1);
    wireless_buf[18] = buf[0];

    while(I2C_Master_ReadReg(PRESSURE_I2C_ADDR, TMP_B1, false, 1) != IDLE_MODE){}
    CopyArray(ReceiveBuffer, buf, 1);
    wireless_buf[19] = buf[0];

    while(I2C_Master_ReadReg(PRESSURE_I2C_ADDR, TMP_B0, false, 1) != IDLE_MODE){}
    CopyArray(ReceiveBuffer, buf, 1);
    wireless_buf[20] = buf[0];

    buf[0] = 0x00; // put in idle mode
    while(I2C_Master_WriteReg(PRESSURE_I2C_ADDR, MEAS_CFG, false, buf, 1) != IDLE_MODE){}

    i2c_succeed = true;
}

