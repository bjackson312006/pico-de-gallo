/*
 * Copyright (c) 2026 Open Device Partnership and Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Zephyr I2C controller driver for the Pico de Gallo USB bridge.
 *
 * This file runs in the embedded/Zephyr context. Note that
 * "embedded" here just means the embedded part of `native-sim`,
 * not something that actually gets flashed to hardware or anything.
 * Anyway, this file translates Zephyr I2C API transactions into the small 
 * host-context shim declared in pdg_i2c_bottom.h, which forwards them to the Pico de Gallo C FFI.
 */

#define DT_DRV_COMPAT odp_pico_de_gallo_i2c

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "pdg_i2c_bottom.h"

LOG_MODULE_REGISTER(i2c_pico_de_gallo, CONFIG_I2C_LOG_LEVEL);

/* Firmware single-transfer limit (pico_de_gallo_internal::MAX_TRANSFER_SIZE). */
#define PDG_I2C_MAX_XFER 4096U

struct pdg_i2c_config {
	const char *serial;
	uint32_t clock_frequency;
};

struct pdg_i2c_data {
	void *ctx;
	struct k_mutex lock;
	uint32_t dev_config;
};

// helper to map a Zephyr I2C speed (see the zephyr I2C_SPEED_... macros) into a pico de gallo speed code (see gallo_i2c_set_config() in pico_de_gallo.h)
// 
// `speed` is the Zephyr I2C speed (meaning you will probably pass a I2C_SPEED_... macro into the parameter). The returned value is one of the possible pico-de-gallo
// speed codes accepted by the FFI `gallo_i2c_set_config()` function via the `frequency` parameter:
// 0 = Standard (100 kHz), 1 = Fast (400 kHz), 2 = Fast+ (1 MHz).
//
// note: in the future it seems like it could be nice for the pico de gallo FFI to just have an enum for these values instead of accepting a uint8_t. However, maybe
//       there's a reason why there isn't.
static uint8_t speed_to_code_(uint32_t speed)
{
	// the three pico de gallo I2C speed settings
	static const uint8_t Gallo_Standard = 0U; // (100 kHz)
	static const uint8_t Gallo_Fast = 1U; 	  // (400 kHz)
	static const uint8_t Gallo_FastPlus = 2U; // (1 MHz)

	// also here's each of the possible Zephyr I2C speed macros according to their API docs:
	// I2C_SPEED_STANDARD  (100 kHz)
	// I2C_SPEED_FAST 	   (400 kHz)
	// I2C_SPEED_FAST_PLUS (1 MHz)
	// I2C_SPEED_HIGH 	   (3.4 MHz)
	// I2C_SPEED_ULTRA     (5 MHz) 

	switch (speed) {
		case I2C_SPEED_STANDARD:  return Gallo_Standard;
		case I2C_SPEED_FAST:      return Gallo_Fast;
		case I2C_SPEED_FAST_PLUS: return Gallo_FastPlus;
		case I2C_SPEED_HIGH: 	  return Gallo_FastPlus; // pico de gallo has no speed to match this
		case I2C_SPEED_ULTRA: 	  return Gallo_FastPlus; // pico de gallo has no speed to match this
		default: 				  return Gallo_Standard; // default to standard if something else gets passed in 
	}
}

// helper to map a `clock_frequency` in Hz to a Zephyr I2C speed macro
static uint32_t freq_to_speed_(uint32_t clock_frequency)
{
	if 		(clock_frequency <= 100000U)  { return I2C_SPEED_STANDARD; }  // less than or equal to 100 kHz, use I2C_SPEED_STANDARD
	else if (clock_frequency <= 400000U)  { return I2C_SPEED_FAST; } 	  // between 100 kHz..=400 kHz, use I2C_SPEED_FAST
	else if (clock_frequency <= 1000000U) { return I2C_SPEED_FAST_PLUS; } // between 400 kHz..=1 MHz, use I2C_SPEED_FAST_PLUS
	else if (clock_frequency <= 3400000U) { return I2C_SPEED_HIGH; }	  // between 1 MHz..=3.4 MHz, use I2C_SPEED_HIGH
	return I2C_SPEED_ULTRA; 										  	  // otherwise (greater than 3.4 MHz) use I2C_SPEED_ULTRA
}

static int pdg_i2c_configure(const struct device *dev, uint32_t dev_config)
{
	struct pdg_i2c_data *data = dev->data;
	int ret;

	if ((dev_config & I2C_ADDR_10_BITS) != 0U) {
		return -ENOTSUP;
	}

	if ((dev_config & I2C_MODE_CONTROLLER) == 0U) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = pdg_i2c_bottom_set_config(data->ctx, speed_to_code_(I2C_SPEED_GET(dev_config)));
	if (ret == 0) {
		data->dev_config = dev_config;
	}
	k_mutex_unlock(&data->lock);

	return ret;
}

static int pdg_i2c_get_config(const struct device *dev, uint32_t *dev_config)
{
	struct pdg_i2c_data *data = dev->data;

	*dev_config = data->dev_config;

	return 0;
}

static int pdg_i2c_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs, uint16_t addr)
{
	struct pdg_i2c_data *data = dev->data;
	int ret;

	if (addr > 0x7fU) {
		return -EINVAL;
	}

	for (uint8_t i = 0U; i < num_msgs; i++) {
		if ((msgs[i].flags & I2C_MSG_ADDR_10_BITS) != 0U) {
			return -ENOTSUP;
		}

		if (msgs[i].len > PDG_I2C_MAX_XFER) {
			return -EINVAL;
		}
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	if (num_msgs == 1U) {
		if ((msgs[0].flags & I2C_MSG_READ) != 0U) {
			ret = pdg_i2c_bottom_read(data->ctx, addr, msgs[0].buf,
						  msgs[0].len);
		} else {
			ret = pdg_i2c_bottom_write(data->ctx, addr, msgs[0].buf,
						   msgs[0].len);
		}
	} else if ((num_msgs == 2U) &&
		   ((msgs[0].flags & I2C_MSG_READ) == 0U) &&
		   ((msgs[1].flags & I2C_MSG_READ) != 0U)) {
		ret = pdg_i2c_bottom_write_read(data->ctx, addr,
						msgs[0].buf, msgs[0].len,
						msgs[1].buf, msgs[1].len);
	} else {
		/* The bridge cannot express arbitrary scatter/gather
		 * transactions with repeated starts. Supported forms are a
		 * single write, a single read, and a write followed by a read.
		 */
		ret = -ENOTSUP;
	}

	k_mutex_unlock(&data->lock);

	return ret;
}

static DEVICE_API(i2c, pdg_i2c_api) = {
	.configure = pdg_i2c_configure,
	.get_config = pdg_i2c_get_config,
	.transfer = pdg_i2c_transfer,
};

static int pdg_i2c_init(const struct device *dev)
{
	const struct pdg_i2c_config *config = dev->config;
	struct pdg_i2c_data *data = dev->data;
	uint32_t speed = freq_to_speed_(config->clock_frequency);

	k_mutex_init(&data->lock);

	data->ctx = pdg_i2c_bottom_open(config->serial);
	if (data->ctx == NULL) {
		LOG_ERR("Failed to open Pico de Gallo bridge (device connected?)");
		return -ENODEV;
	}

	data->dev_config = I2C_MODE_CONTROLLER | I2C_SPEED_SET(speed);

	return pdg_i2c_bottom_set_config(data->ctx, speed_to_code_(speed));
}

#define PDG_I2C_INIT(inst)							\
	static struct pdg_i2c_data pdg_i2c_data_##inst;				\
										\
	static const struct pdg_i2c_config pdg_i2c_config_##inst = {		\
		.serial = DT_INST_PROP_OR(inst, serial_number, NULL),		\
		.clock_frequency = DT_INST_PROP_OR(inst, clock_frequency,	\
						   I2C_BITRATE_STANDARD),	\
	};									\
										\
	I2C_DEVICE_DT_INST_DEFINE(inst, pdg_i2c_init, NULL,			\
				  &pdg_i2c_data_##inst,				\
				  &pdg_i2c_config_##inst, POST_KERNEL,		\
				  CONFIG_I2C_INIT_PRIORITY, &pdg_i2c_api);

DT_INST_FOREACH_STATUS_OKAY(PDG_I2C_INIT)
