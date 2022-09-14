/*
 * seesaw.h
 *
 *  Created on: Jun 2, 2021
 *      Author: Dragyn
 */

#ifndef SEESAW_H_
#define SEESAW_H_

#include "i2c.h"

#define SOIL_SENSOR_ADDR 0x36

#define SEESAW_RESET 0x007F
#define SEESAW_STATUS_TEMP 0x0004
#define SEESAW_TOUCH_CHANNEL 0x0F10

void init_seesaw();

void read_temp_seesaw();

void read_cap_seesaw();

#endif /* SEESAW_H_ */
