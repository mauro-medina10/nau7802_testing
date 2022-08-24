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
#include "nau7802.h"	

#define LOG_LEVEL LOG_LEVEL_DBG

LOG_MODULE_REGISTER(main);

/**
 * @file Sample app using the Fujitsu MB85RC256V FRAM through I2C.
 */

#define NAU7802_I2C_ADDR	0x2A

static volatile bool btn_flg = false;
static int nau7802_read_id(const struct device *dev);

const struct device *nau7802;

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

static int nau7802_read_id(const struct device *dev)
{
	uint8_t data = 0;
	uint8_t reg_addr = 0x1F;
	int32_t ret = 0;

	//Read CHIP-ID
	ret = nau7802_getRegister(nau7802, reg_addr, &data);
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
	int32_t data;

	bsp_init(button0_pressed, button1_pressed, button2_pressed, button3_pressed);

	// nau7802_powerUp();

	// do
	// {
	// 	ret = nau7802_read_id();

	// 	while(!btn_flg) k_msleep(100);

	// 	btn_flg = false;
	// 	// k_msleep(500);
	// } while (true);

	ret = nau7802_begin(nau7802);

	while (true)
	{
		while(nau7802_data_available(nau7802) == 0);

		ret = nau7802_getReading(nau7802, &data);

		if(ret == 0)
		{
			printk("ADC value: %d.\n", data);
		}else
		{
			printk("ADC error: %d.\n", ret);
		}
		
		k_msleep(500);
	}
	(void) i;
}
