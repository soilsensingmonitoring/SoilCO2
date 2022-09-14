/*
 * co2.h
 *
 *  Created on: Jun 3, 2021
 *      Author: Dragyn
 */

#ifndef CO2_H_
#define CO2_H_

#include "i2c.h"

#define CO2_ADDR 0x61

#define SCD30_CMD_SOFT_RESET 0xD304
#define SCD30_CMD_CONT_MEASUREMENT 0x0010
#define SCD30_STOP_MEASURING 0x0104
#define SCD30_CMD_READ_MEASUREMENT 0x0300
#define SCD30_CMD_GET_DATA_READY 0x0202
#define SCD30_CMD_SET_MEASUREMENT_INTERVAL 0x4600

uint8_t CO2_buf[18];

void init_co2();
void read_data_co2();
int read_data_ready_co2();

#endif /* CO2_H_ */
