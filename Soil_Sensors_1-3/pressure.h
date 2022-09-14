/*
 * pressure.h
 *
 *  Created on: Jun 4, 2021
 *      Author: Dragyn
 */

#ifndef PRESSURE_H_
#define PRESSURE_H_

#include "i2c.h"

#define PRESSURE_I2C_ADDR 0x77

#define PSR_B2 0x00
#define PSR_B1 0x01
#define PSR_B0 0x02

#define TMP_B2 0x03
#define TMP_B1 0x04
#define TMP_B0 0x05

#define PRS_CFG 0x06
#define TMP_CFG 0x07

#define MEAS_CFG 0x08

#define CFG_REG 0x09

void init_pressure();
void read_pressure();

#endif /* PRESSURE_H_ */
