/*
 * Copyright (c) 2022 Mauro Medina Serangeli
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdint.h>

#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>
#include <drivers/i2c.h>
#include <drivers/gpio.h>
#include <sys/util.h>
#include <sys/printk.h>
#include <zephyr/logging/log.h>

#include "nrf52833dk_bsp.h"
#include "Drv_I2C.h"

#define LOG_LEVEL LOG_LEVEL_DBG

LOG_MODULE_REGISTER(main);

//Register Map
typedef enum
{
	NAU7802_PU_CTRL      = 0x00,
	NAU7802_CTRL1,
	NAU7802_CTRL2,
	NAU7802_OCAL1_B2,
	NAU7802_OCAL1_B1,
	NAU7802_OCAL1_B0,
	NAU7802_GCAL1_B3,
	NAU7802_GCAL1_B2,
	NAU7802_GCAL1_B1,
	NAU7802_GCAL1_B0,
	NAU7802_OCAL2_B2,
	NAU7802_OCAL2_B1,
	NAU7802_OCAL2_B0,
	NAU7802_GCAL2_B3,
	NAU7802_GCAL2_B2,
	NAU7802_GCAL2_B1,
	NAU7802_GCAL2_B0,
	NAU7802_I2C_CONTROL,
	NAU7802_ADCO_B2,
	NAU7802_ADCO_B1,
	NAU7802_ADCO_B0,
	NAU7802_ADC          = 0x15,	//Shared ADC and OTP 32:24
	NAU7802_OTP_B1,					//OTP 23:16 or 7:0?
	NAU7802_OTP_B0,					//OTP 15:8
	NAU7802_PGA          = 0x1B,
	NAU7802_PGA_PWR      = 0x1C,
	NAU7802_DEVICE_REV   = 0x1F,
} Scale_Registers;

//Bits within the PU_CTRL register
typedef enum
{
	NAU7802_PU_CTRL_RR = 0,
	NAU7802_PU_CTRL_PUD,
	NAU7802_PU_CTRL_PUA,
	NAU7802_PU_CTRL_PUR,
	NAU7802_PU_CTRL_CS,
	NAU7802_PU_CTRL_CR,
	NAU7802_PU_CTRL_OSCS,
	NAU7802_PU_CTRL_AVDDS,
} PU_CTRL_Bits;

/**
 * @file Sample app using the Fujitsu MB85RC256V FRAM through I2C.
 */

#define NAU7802_I2C_ADDR	0x2A

static volatile bool btn_flg = false;
static int nau7802_read_id(void);

/* Button interrupt callbacks. */
void button0_pressed(const struct device *dev, struct gpio_callback *cb,
		    			uint32_t pins)
{
	bsp_led0_toggle();
}

void button1_pressed(const struct device *dev, struct gpio_callback *cb,
		    			uint32_t pins)
{
	bsp_led1_toggle();
}

void button3_pressed(const struct device *dev, struct gpio_callback *cb,
		    			uint32_t pins)
{
	NVIC_SystemReset();
}

void button2_pressed(const struct device *dev, struct gpio_callback *cb,
		    			uint32_t pins)
{
	btn_flg = true;
}

//Send a given value to be written to given address
//Return true if successful
static int32_t nau7802_setRegister(uint8_t registerAddress, uint8_t value)
{
	return I2C_byte_transfer(registerAddress, value);
}

//Get contents of a register
static int32_t nau7802_getRegister(uint8_t registerAddress, uint8_t *data)
{
	if(I2C_byte_read(registerAddress, data) == 0){
		
		return 0;
	}
	
	return (-EIO);		//Error
}

//Mask & set a given bit within a register
static int32_t nau7802_setBit(uint8_t bitNumber, uint8_t registerAddress)
{
	uint8_t value;
	int32_t ret = nau7802_getRegister(registerAddress, &value);

	if(ret != 0){return ret;}

	value |= (1 << bitNumber);		//Set this bit
	return (nau7802_setRegister(registerAddress, value));
}

//Mask & clear a given bit within a register
static int32_t nau7802_clearBit(uint8_t bitNumber, uint8_t registerAddress)
{
	uint8_t value;
	int32_t ret = nau7802_getRegister(registerAddress, &value);
	
	if(ret != 0){return ret;}

	value &= ~(1 << bitNumber);		//Set this bit
	return (nau7802_setRegister(registerAddress, value));
}

//Return a given bit within a register
static int32_t nau7802_getBit(uint8_t bitNumber, uint8_t registerAddress, uint8_t *data)
{
	uint8_t value;
	int32_t ret = nau7802_getRegister(registerAddress, &value);

	if(ret != 0){return ret;}

	value &= (1 << bitNumber);		//Clear all but this bit

	*data = value;

	return ret;
}

//Power up digital and analog sections of scale
static int32_t nau7802_powerUp(void)
{
	uint8_t data = 1;

	nau7802_setBit(NAU7802_PU_CTRL_PUD, NAU7802_PU_CTRL);
	nau7802_setBit(NAU7802_PU_CTRL_PUA, NAU7802_PU_CTRL);

	//Wait for Power Up bit to be set - takes approximately 200us
	uint8_t counter = 0;
	
	while (1)
	{
		nau7802_getBit(NAU7802_PU_CTRL_PUR, NAU7802_PU_CTRL, &data);

		if (data == 1)
		{
			break;				//Good to go
		}
		
		k_msleep(1);
		
		if (counter++ > 100)
		{
			return (false);		//Error
		}
	}
	
	return (true);
}

static int nau7802_read_id(void)
{
	uint8_t data = 0;
	uint8_t reg_addr = 0x1F;
	int32_t ret = 0;

	//Read CHIP-ID
	ret = I2C_byte_read(reg_addr, &data);
	if (ret) {
		printk("Error reading from NAU7802! error code (%d)\n", ret);
		return ret;
	} else {
		printk("Read 0x%X from address 0x%X.\n", data, reg_addr);
	}
	return ret;
} 

void main(void)
{
	int i, ret;

	bsp_init(button0_pressed, button1_pressed, button2_pressed, button3_pressed);

	I2C_init(NAU7802_I2C_ADDR);

	nau7802_powerUp();

	do
	{
		ret = nau7802_read_id();

		while(!btn_flg) k_msleep(100);

		btn_flg = false;
		// k_msleep(500);
	} while (true);

	(void) i;
}
