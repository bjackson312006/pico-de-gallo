/*
 * Copyright (c) 2026 Open Device Partnership and Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Zephyr SPI controller driver for the Pico de Gallo USB bridge.
 *
 * This file runs in the embedded/Zephyr context. Note that
 * "embedded" here just means the embedded part of `native-sim`,
 * not something that actually gets flashed to hardware or anything.
 * Anyway, this file translates Zephyr SPI API transactions into the small 
 * host-context shim declared in pdg_spi_bottom.h, which forwards them to the Pico de Gallo C FFI.
 */

#define DT_DRV_COMPAT odp_pico_de_gallo_spi

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <string.h>

#include "pdg_spi_bottom.h"

LOG_MODULE_REGISTER(spi_pico_de_gallo, CONFIG_SPI_LOG_LEVEL);

/* Firmware single-transfer limit (pico_de_gallo_internal::MAX_TRANSFER_SIZE). */
#define PDG_SPI_MAX_BUFFER 4096U

struct pdg_spi_config {
    const char *serial;
};

struct pdg_spi_data {
	void *ctx;
	struct k_mutex lock;
};

// helper to calculate the total byte length of every buffer in a `spi_buf_set`
// used when flattening Zephyr `spi_but_set`s into a normal contiguous buffer to pass into the pico-de-gallo ffi
static int bufset_len_(const struct spi_buf_set *bufs, size_t *total_len)
{
    *total_len = 0U;

    if (bufs == NULL) {
        return 0;
    }
    if (bufs->count != 0U && bufs->buffers == NULL) {
        return -EINVAL;
    }

    for (size_t i = 0U; i < bufs->count; ++i) {
        if (bufs->buffers[i].len > PDG_SPI_MAX_BUFFER - *total_len) {
            return -EMSGSIZE;
        }
        *total_len += bufs->buffers[i].len;
    }

    return 0;
}

// helper that flattens a set of `spi_buf_set`s into a single buffer
static void flatten_tx_(const struct spi_buf_set *tx_bufs, uint8_t *flat, size_t flat_len)
{
    size_t offset = 0U;

    memset(flat, 0, flat_len);
    if (tx_bufs == NULL) {
        return;
    }

    for (size_t i = 0U; i < tx_bufs->count; ++i) {
        const struct spi_buf *buf = &tx_bufs->buffers[i];

        if (buf->buf != NULL) {
            memcpy(flat + offset, buf->buf, buf->len);
        }
        offset += buf->len;
    }
}

// helper that takes in a normal flat buffer from pico-de-gallo and organizes it into the multiple
// buffer sets provided by zephyr
static void unflatten_rx_(const struct spi_buf_set *rx_bufs, const uint8_t *flat)
{
    size_t offset = 0U;

    if (rx_bufs == NULL) {
        return;
    }

    for (size_t i = 0U; i < rx_bufs->count; ++i) {
        const struct spi_buf *buf = &rx_bufs->buffers[i];

        if (buf->buf != NULL) {
            memcpy(buf->buf, flat + offset, buf->len);
        }
        offset += buf->len;
    }
}

static int pdg_spi_transceive(const struct device *dev, const struct spi_config *config, const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs)
{
    struct pdg_spi_data *data = dev->data;
    struct pdg_spi_batch_op op = {0};
    uint8_t *tx_flat = NULL;
    uint8_t *rx_flat = NULL;
    size_t tx_len;
    size_t rx_len;
    size_t clock_len;
    size_t out_len = 0U;
    int ret;

    if (config == NULL 
		|| SPI_OP_MODE_GET(config->operation) != SPI_OP_MODE_MASTER 
		|| SPI_WORD_SIZE_GET(config->operation) != 8U
		|| (config->operation & (SPI_TRANSFER_LSB | SPI_MODE_LOOP | SPI_HALF_DUPLEX | SPI_HOLD_ON_CS | SPI_LOCK_ON | SPI_CS_ACTIVE_HIGH)) != 0U 
		|| config->slave > 3U) {
        return -ENOTSUP;
    }

    ret = bufset_len_(tx_bufs, &tx_len);
    if (ret != 0) {
        return ret;
    }
    ret = bufset_len_(rx_bufs, &rx_len);
    if (ret != 0) {
        return ret;
    }

    clock_len = MAX(tx_len, rx_len);
    if (clock_len == 0U) {
        return 0;
    }

    if (tx_len != 0U) {
        tx_flat = k_malloc(clock_len);
        if (tx_flat == NULL) {
            return -ENOMEM;
        }
        flatten_tx_(tx_bufs, tx_flat, clock_len);
    }
    if (rx_len != 0U) {
        rx_flat = k_malloc(clock_len);
        if (rx_flat == NULL) {
            k_free(tx_flat);
            return -ENOMEM;
        }
    }

    if (tx_len == 0U) {
        op.tag = PDG_SPI_BATCH_READ;
        op.read_len = (uint16_t)clock_len;
    } else if (rx_len == 0U) {
        op.tag = PDG_SPI_BATCH_WRITE;
        op.data = tx_flat;
        op.data_len = clock_len;
    } else {
        op.tag = PDG_SPI_BATCH_TRANSFER;
        op.data = tx_flat;
        op.data_len = clock_len;
    }

    k_mutex_lock(&data->lock, K_FOREVER);
    ret = pdg_spi_bottom_set_config(data->ctx, config->frequency, (config->operation & SPI_MODE_CPHA) != 0U, (config->operation & SPI_MODE_CPOL) != 0U);
    if (ret == 0) {
        ret = pdg_spi_bottom_batch(data->ctx, (uint8_t)config->slave, &op, 1U, rx_flat, rx_len == 0U ? 0U : clock_len, &out_len, NULL);
    }
    k_mutex_unlock(&data->lock);

    if (ret == 0 && rx_len != 0U) {
        if (out_len != clock_len) {
            ret = -EPROTO;
        } else {
            unflatten_rx_(rx_bufs, rx_flat);
        }
    }

    k_free(rx_flat);
    k_free(tx_flat);
    return ret;
}

static DEVICE_API(spi, pdg_spi_api) = {
    .transceive = pdg_spi_transceive,
};

static int pdg_spi_init(const struct device *dev)
{
	const struct pdg_spi_config *config = dev->config;
	struct pdg_spi_data *data = dev->data;

	k_mutex_init(&data->lock);

	data->ctx = pdg_spi_bottom_open(config->serial);
	if (data->ctx == NULL) {
		return -ENODEV;
	}

	return 0;
}

#define PDG_SPI_INIT(inst)                                                \
	static struct pdg_spi_data pdg_spi_data_##inst;                     \
	                                                                    \
	static const struct pdg_spi_config pdg_spi_config_##inst = {        \
		.serial = DT_INST_PROP_OR(inst, serial_number, NULL),          \
	};                                                                  \
	                                                                    \
	SPI_DEVICE_DT_INST_DEFINE(inst, pdg_spi_init, NULL,                  \
				  &pdg_spi_data_##inst, &pdg_spi_config_##inst,   \
				  POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,          \
				  &pdg_spi_api);

DT_INST_FOREACH_STATUS_OKAY(PDG_SPI_INIT)
