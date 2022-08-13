/*
 * Copyright (c) 2015 Intel Corporation
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

#define LOG_LEVEL 4

LOG_MODULE_REGISTER(main);

/**
 * @file Sample app using the Fujitsu MB85RC256V FRAM through I2C.
 */

#define NAU7802_I2C_ADDR	0x2A

static const struct device *i2c_dev;
static volatile bool btn_flg = false;
static int nau7802_read_id(const struct device *i2c_device);

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

// static int write_bytes(const struct device *i2c_dev, uint8_t addr,
// 		       uint8_t *data, uint32_t num_bytes)
// {
// 	uint8_t wr_addr[2];
// 	struct i2c_msg msgs[2];

// 	/* FRAM address */
// 	wr_addr[0] = addr;
// 	wr_addr[1] = NAU7802_I2C_ADDR;

// 	/* Setup I2C messages */

// 	/* Send the address to write to */
// 	msgs[0].buf = wr_addr;
// 	msgs[0].len = 2U;
// 	msgs[0].flags = I2C_MSG_WRITE;

// 	/* Data to be written, and STOP after this. */
// 	msgs[1].buf = data;
// 	msgs[1].len = num_bytes;
// 	msgs[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

// 	return i2c_transfer(i2c_dev, &msgs[0], 2, NAU7802_I2C_ADDR);
// }
static struct i2c_msg msgs[2];
uint8_t wr_addr;

static int read_bytes(const struct device *i2c_dev, uint8_t addr,
		      uint8_t *data, uint32_t num_bytes)
{

	/* Now try to read back from FRAM */

	/* FRAM address */
	wr_addr = addr;

	/* Setup I2C messages */

	/* Send the address to read from */
	msgs[0].buf = &wr_addr;
	msgs[0].len = 1U;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	/* Read from device. STOP after this. */
	msgs[1].buf = data;
	msgs[1].len = num_bytes;
	msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

	//return i2c_write_read(i2c_dev, NAU7802_I2C_ADDR, wr_addr, msgs[0].len, data, num_bytes);//i2c_transfer(i2c_dev, &msgs[0], 2, NAU7802_I2C_ADDR);
	return i2c_transfer(i2c_dev, msgs, 2U, NAU7802_I2C_ADDR);
	//return i2c_reg_read_byte(i2c_dev, NAU7802_I2C_ADDR, addr, data);
}	

static int nau7802_read_id(const struct device *i2c_device)
{
	uint8_t data[2];
	uint8_t reg_addr = 0x1F;
	int32_t ret = 0;

	//Read CHIP-ID
	data[0] = 0x00;
	ret = read_bytes(i2c_device, reg_addr, &data[0], 1);
	if (ret) {
		printk("Error reading from NAU7802! error code (%d)\n", ret);
		return ret;
	} else {
		printk("Read 0x%X from address 0x%X.\n", data[0], reg_addr);
	}
	return ret;
} 

void main(void)
{
	i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
	if (!device_is_ready(i2c_dev)) {
		printk("I2C: Device is not ready.\n");
		return;
	}

	//uint8_t cmp_data[16];
	// uint8_t data[16];
	// uint8_t reg_addr = 0;
	int i, ret;

	bsp_init(button0_pressed, button1_pressed, button2_pressed, button3_pressed);

	do
	{
		nau7802_read_id(i2c_dev);

		while(!btn_flg) k_msleep(100);

		btn_flg = false;
		// k_msleep(500);
	} while (true);

	(void) i;
	/* Do one-byte read/write */

	// data[0] = 0xAE;
	// ret = write_bytes(i2c_dev, 0x00, &data[0], 1);
	// if (ret) {
	// 	printk("Error writing to FRAM! error code (%d)\n", ret);
	// 	return;
	// } else {
	// 	printk("Wrote 0xAE to address 0x00.\n");
	// }

	// data[0] = 0x86;
	// ret = write_bytes(i2c_dev, 0x01, &data[0], 1);
	// if (ret) {
	// 	printk("Error writing to FRAM! error code (%d)\n", ret);
	// 	return;
	// } else {
	// 	printk("Wrote 0x86 to address 0x01.\n");
	// }

	// data[0] = 0x00;
	// ret = read_bytes(i2c_dev, 0x00, &data[0], 1);
	// if (ret) {
	// 	printk("Error reading from FRAM! error code (%d)\n", ret);
	// 	return;
	// } else {
	// 	printk("Read 0x%X from address 0x00.\n", data[0]);
	// }

	// data[1] = 0x00;
	// ret = read_bytes(i2c_dev, 0x01, &data[0], 1);
	// if (ret) {
	// 	printk("Error reading from FRAM! error code (%d)\n", ret);
	// 	return;
	// } else {
	// 	printk("Read 0x%X from address 0x01.\n", data[0]);
	// }

	// /* Do multi-byte read/write */

	// /* get some random data, and clear out data[] */
	// for (i = 0; i < sizeof(cmp_data); i++) {
	// 	cmp_data[i] = k_cycle_get_32() & 0xFF;
	// 	data[i] = 0x00;
	// }

	// /* write them to the FRAM */
	// ret = write_bytes(i2c_dev, 0x00, cmp_data, sizeof(cmp_data));
	// if (ret) {
	// 	printk("Error writing to FRAM! error code (%d)\n", ret);
	// 	return;
	// } else {
	// 	printk("Wrote %zu bytes to address 0x00.\n", sizeof(cmp_data));
	// }

	// ret = read_bytes(i2c_dev, 0x00, data, sizeof(data));
	// if (ret) {
	// 	printk("Error reading from FRAM! error code (%d)\n", ret);
	// 	return;
	// } else {
	// 	printk("Read %zu bytes from address 0x00.\n", sizeof(data));
	// }

	// ret = 0;
	// for (i = 0; i < sizeof(cmp_data); i++) {
	// 	/* uncomment below if you want to see all the bytes */
	// 	/* printk("0x%X ?= 0x%X\n", cmp_data[i], data[i]); */
	// 	if (cmp_data[i] != data[i]) {
	// 		printk("Data comparison failed @ %d.\n", i);
	// 		ret = -EIO;
	// 	}
	// }
	// if (ret == 0) {
	// 	printk("Data comparison successful.\n");
	// }
}
