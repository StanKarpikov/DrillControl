/*
 * soft_i2c.h
 *
 * Copyright 2015 Edward V. Emelianoff <eddy@sao.ru, edward.emelianoff@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */

#pragma once
#ifndef __I2C_H__
#define __I2C_H__

#include "stm8s.h"
#include "stm8s_gpio.h"
#include <intrinsics.h>
#include <iostm8.h>
// macro for using in port constructions like PORT(LED_PORT, ODR) = xx
#define CONCAT(a, b)	a ## _ ## b
#define PORT(a, b)		CONCAT(a , b)

//soft I2C
#define I2C_SDA_PORT	PA
#define I2C_SDA_PIN	(1<<4)
#define I2C_SCL_PORT	PA
#define I2C_SCL_PIN	(1<<6)
void delayX(volatile uint32_t count);
// flags for i2c_state
typedef enum{
	I2C_OK = 0,
	I2C_LINEBUSY,
	I2C_TMOUT,
	I2C_NOADDR,
	I2C_NACK,
	I2C_HWPROBLEM,
} i2c_status;

void i2c_setup();
void i2c_set_addr7(u8 addr);
i2c_status i2c_7bit_send_onebyte(u8 data, u8 stop);
i2c_status i2c_7bit_send(u8 *data, u8 datalen, u8 stop);
i2c_status i2c_7bit_sendR(u8 reg,u8 *data, u8 datalen, u8 stop);
i2c_status i2c_7bit_receive_onebyte(u8 *data, u8 wait);
i2c_status i2c_7bit_receive_twobyte(u8 *data, u8 wait);

#endif // __I2C_H__
