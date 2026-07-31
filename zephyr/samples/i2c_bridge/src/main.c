/*
 * Copyright (c) 2026 Open Device Partnership and Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Minimal smoke test for the Pico de Gallo I2C controller. Reads the WHO_AM_I
 * register of an MPU6050 at address 0x68 through the generic Zephyr I2C API.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

#define MPU6050_ADDR 0x68U
#define REG_WHO_AM_I 0x75U

int main(void)
{
	const struct device *bus = DEVICE_DT_GET(DT_NODELABEL(pdg_i2c0));
	uint8_t reg = REG_WHO_AM_I;
	uint8_t id = 0U;
	int ret;

	if (!device_is_ready(bus)) {
		printk("Pico de Gallo I2C bus not ready\n");
		return 0;
	}

	ret = i2c_write_read(bus, MPU6050_ADDR, &reg, sizeof(reg), &id, sizeof(id));
	if (ret < 0) {
		printk("i2c_write_read failed: %d\n", ret);
		return 0;
	}

	printk("WHO_AM_I = 0x%02x\n", id);

	return 0;
}
