/*
 * co2.c
 *
 *  Created on: Jun 3, 2021
 *      Author: Dragyn
 */
#include "main.h"

uint8_t CO2_buf[18];

/*
 * checksum calculation for a given set of data
 */
static uint8_t crc8(const uint8_t *data, int len) {
    const uint8_t POLYNOMIAL = 0x31;
    uint8_t crc = 0xFF;

    int i, j;

    for (j = len; j; j--) {
        crc ^= *data++;
        for (i = 8; i; i--) {
            crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
        }
    }
    return crc;
}

/*
 * Initializes the CO2 sensor with continuous measurement at a 15 second measuring interval
 */
void init_co2() {

    uint8_t buf[3];
    uint8_t toWrite = 3;

    buf[0] = 0x00;
    buf[1] = 0x00; // no ambient pressure compensation
    buf[2] = crc8(buf, 2);

    if (I2C_Master_WriteReg(CO2_ADDR, SCD30_CMD_CONT_MEASUREMENT, true, buf, toWrite) != IDLE_MODE) { // trigger continuous measurement
        initGPIO_I2C();
        initI2C();
        __delay_cycles(1000);
        return;
    }

    buf[0] = 0x00;
    buf[1] = 0x02; // set a 2 second measuring interval - this is the minimum
    buf[2] = crc8(buf, 2); // CRC calculation

    while(I2C_Master_WriteReg(CO2_ADDR, SCD30_CMD_SET_MEASUREMENT_INTERVAL, true, buf, toWrite) != IDLE_MODE){}

    i2c_succeed = true;
}

/*
 * Returns true or false if new data is available in the CO2 sensor
 */
int read_data_ready_co2() {
    uint8_t buf[3]; // buf[2] is the CRC
    while(1) {
        __delay_cycles(100000); // read every .1 seconds - give the sensor some time to do other things while we wait

        if (I2C_Master_WriteReg(CO2_ADDR, SCD30_CMD_GET_DATA_READY, true, 0, 0) == IDLE_MODE){
            __delay_cycles(1000); // delay 1 ms - any longer and the transaction gets overwritten

            if(I2C_Master_ReadIntoBuffer(CO2_ADDR, 3) == IDLE_MODE) {
                CopyArray(ReceiveBuffer, buf, 3);
                break;
            }
        } else {
            initGPIO_I2C();
            initI2C();
            __delay_cycles(1000);
            return 0;
        }
    }

    return (buf[0] == 0) & (buf[1] == 1); // return 0 on not ready, 1 if ready
}

/*
 * Returns the temp, humidity, and CO2 levels, writes the data to CO2_buf
 */
void read_data_co2() {
    int i;
    for (i = 0; i < 5; i++) {
        while (!read_data_ready_co2()); // wait until the data is ready

        while(I2C_Master_WriteReg(CO2_ADDR, SCD30_CMD_READ_MEASUREMENT, true, 0, 0) != IDLE_MODE){}

        __delay_cycles(3000); // delay 3 ms - specified by datasheet

        if(I2C_Master_ReadIntoBuffer(CO2_ADDR, 18) == IDLE_MODE) {
            if (i == 4)
                CopyArray(ReceiveBuffer, CO2_buf, 18);
        }
    }

    while(I2C_Master_WriteReg(CO2_ADDR, SCD30_STOP_MEASURING, true, 0, 0) != IDLE_MODE){} // stop measuring

    i2c_succeed = true;

    wireless_buf[21] = CO2_buf[0];
    wireless_buf[22] = CO2_buf[1];
    wireless_buf[23] = CO2_buf[3];
    wireless_buf[24] = CO2_buf[4];
    wireless_buf[25] = CO2_buf[6];
    wireless_buf[26] = CO2_buf[7];
    wireless_buf[27] = CO2_buf[9];
    wireless_buf[28] = CO2_buf[10];
    wireless_buf[29] = CO2_buf[12];
    wireless_buf[30] = CO2_buf[13];
    wireless_buf[31] = CO2_buf[15];
    wireless_buf[32] = CO2_buf[16];
}
