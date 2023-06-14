// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
/*
 * Analog Devices MAX14001 ADC driver
 *
 * Copyright 2023 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/bitrev.h>
#include <linux/device.h>
#include <linux/iio/iio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/types.h>

#include <asm/unaligned.h>

/* MAX14001 Registers Address */
#define MAX14001_ADC			0x00
#define MAX14001_FADC			0x01
#define MAX14001_FLAGS			0x02
#define MAX14001_FLTEN			0x03
#define MAX14001_THL			0x04
#define MAX14001_THU			0x05
#define MAX14001_INRR			0x06
#define MAX14001_INRT			0x07
#define MAX14001_INRP			0x08
#define MAX14001_CFG			0x09
#define MAX14001_ENBL			0x0A
#define MAX14001_ACT			0x0B
#define MAX14001_WEN			0x0C

#define MAX14001_VERIFICATION_REG(x)	((x) + 0x10)

#define MAX14001_CFG_EXRF		BIT(5)

#define MAX14001_ADDR_MASK		GENMASK(15, 11)
#define MAX14001_DATA_MASK		GENMASK(9, 0)
#define MAX14001_FILTER_MASK		GENMASK(3, 2)

#define MAX14001_SET_WRITE_BIT		BIT(10)
#define MAX14001_WRITE_WEN		0x294

struct max14001_state {
	struct spi_device	*spi;
	/*
	 * lock protect against multiple concurrent accesses, RMW sequence, and
	 * SPI transfer
	 */
	struct mutex		lock;
	int			vref_mv;
	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	__be16			spi_tx_buffer __aligned(IIO_DMA_MINALIGN);
	__be16			spi_rx_buffer;
};

static int max14001_read(void *context, unsigned int reg_addr, unsigned int *data)
{
	struct max14001_state *st = context;
	int ret;

	struct spi_transfer xfers[] = {
		{
			.tx_buf = &st->spi_tx_buffer,
			.len = sizeof(st->spi_tx_buffer),
			.cs_change = 1,
		}, {
			.rx_buf = &st->spi_rx_buffer,
			.len = sizeof(st->spi_rx_buffer),
		},
	};

	st->spi_tx_buffer = bitrev16(cpu_to_be16(FIELD_PREP(MAX14001_ADDR_MASK,
								reg_addr)));

	ret = spi_sync_transfer(st->spi, xfers, ARRAY_SIZE(xfers));
	if (ret)
		return ret;

	*data = bitrev16(be16_to_cpu(st->spi_rx_buffer)) & MAX14001_DATA_MASK;

	return 0;
}

static int max14001_write(void *context, unsigned int reg_addr, unsigned int data)
{
	struct max14001_state *st = context;

	st->spi_tx_buffer = bitrev16(cpu_to_be16(
			FIELD_PREP(MAX14001_ADDR_MASK, reg_addr) |
			FIELD_PREP(MAX14001_SET_WRITE_BIT, 1) |
			FIELD_PREP(MAX14001_DATA_MASK, data)));

	return spi_write(st->spi, &st->spi_tx_buffer, sizeof(st->spi_tx_buffer));
}

static int max14001_write_verification_reg(struct max14001_state *st,
						unsigned int reg_addr)
{
	unsigned int reg_data;
	int ret;

	ret = max14001_read(st, reg_addr, &reg_data);
	if (ret)
		return ret;

	return max14001_write(st, MAX14001_VERIFICATION_REG(reg_addr), reg_data);
}

static int max14001_reg_update(struct max14001_state *st,
				unsigned int reg_addr,
				unsigned int mask,
				unsigned int val)
{
	int ret;
	unsigned int reg_data;

	/* Enable SPI Registers Write */
	ret = max14001_write(st, MAX14001_WEN, MAX14001_WRITE_WEN);
	if (ret)
		return ret;

	ret = max14001_read(st, reg_addr, &reg_data);
	if (ret)
		return ret;

	reg_data = FIELD_PREP(mask, val);

	ret = max14001_write(st, reg_addr, reg_data);
	if (ret)
		return ret;

	/* Write Verification Register */
	ret = max14001_write_verification_reg(st, reg_addr);
	if (ret)
		return ret;

	/* Disable SPI Registers Write */
	return max14001_write(st, MAX14001_WEN, 0);
}

static int max14001_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int *val, int *val2, long mask)
{
	struct max14001_state *st = iio_priv(indio_dev);
	unsigned int data;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&st->lock);
		ret = max14001_read(st, MAX14001_ADC, &data);
		mutex_unlock(&st->lock);
		if (ret < 0)
			return ret;

		*val = data;

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = st->vref_mv;
		*val2 = 10;

		return IIO_VAL_FRACTIONAL_LOG2;

	default:
		return -EINVAL;
	}
}

static const struct iio_info max14001_info = {
	.read_raw = max14001_read_raw,
};

static const struct iio_chan_spec max14001_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
	}
};

static void max14001_regulator_disable(void *data)
{
	struct regulator *reg = data;

	regulator_disable(reg);
}

static int max14001_init(struct max14001_state *st)
{
	int ret;

	/* Enable SPI Registers Write */
	ret = max14001_write(st, MAX14001_WEN, MAX14001_WRITE_WEN);
	if (ret)
		return ret;

	/*
	 * Reads all registers and writes the values back to their appropriate
	 * verification registers to clear the Memory Validation fault.
	 */
	ret = max14001_write_verification_reg(st, MAX14001_FLTEN);
	if (ret)
		return ret;

	ret = max14001_write_verification_reg(st, MAX14001_THL);
	if (ret)
		return ret;

	ret = max14001_write_verification_reg(st, MAX14001_THU);
	if (ret)
		return ret;

	ret = max14001_write_verification_reg(st, MAX14001_INRR);
	if (ret)
		return ret;

	ret = max14001_write_verification_reg(st, MAX14001_INRT);
	if (ret)
		return ret;

	ret = max14001_write_verification_reg(st, MAX14001_INRP);
	if (ret)
		return ret;

	ret = max14001_write_verification_reg(st, MAX14001_CFG);
	if (ret)
		return ret;

	ret = max14001_write_verification_reg(st, MAX14001_ENBL);
	if (ret)
		return ret;

	/* Disable SPI Registers Write */
	return max14001_write(st, MAX14001_WEN, 0);
}

static int max14001_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct max14001_state *st;
	struct regulator *vref;
	struct device *dev = &spi->dev;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;

	indio_dev->name = "max14001";
	indio_dev->info = &max14001_info;
	indio_dev->channels = max14001_channels;
	indio_dev->num_channels = ARRAY_SIZE(max14001_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = max14001_init(st);
	if (ret)
		return ret;

	vref = devm_regulator_get_optional(dev, "vref");
	if (IS_ERR(vref)) {
		if (PTR_ERR(vref) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(vref),
					     "Failed to get vref regulator");

		/* internal reference */
		st->vref_mv = 1250;
	} else {
		ret = regulator_enable(vref);
		if (ret)
			return dev_err_probe(dev, ret,
					"Failed to enable vref regulators\n");

		ret = devm_add_action_or_reset(dev, max14001_regulator_disable,
					       vref);
		if (ret)
			return ret;

		ret = regulator_get_voltage(vref);
		if (ret < 0)
			return dev_err_probe(dev, ret,
					     "Failed to get vref\n");

		st->vref_mv = ret / 1000;

		/* select external voltage reference source for the ADC */
		ret = max14001_reg_update(st, MAX14001_CFG,
					  MAX14001_CFG_EXRF, 1);

		if (ret < 0)
			return ret;
	}

	mutex_init(&st->lock);

	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id max14001_of_match[] = {
	{ .compatible = "adi,max14001" },
	{ }
};
MODULE_DEVICE_TABLE(of, max14001_of_match);

static struct spi_driver max14001_driver = {
	.driver = {
		.name = "max14001",
		.of_match_table = max14001_of_match,
	},
	.probe = max14001_probe,
};
module_spi_driver(max14001_driver);

MODULE_AUTHOR("Kim Seer Paller");
MODULE_DESCRIPTION("MAX14001 ADC driver");
MODULE_LICENSE("GPL");
