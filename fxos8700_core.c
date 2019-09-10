/*
 *  Copyright (C) 2012-2013 Freescale Semiconductor, Inc. All Rights Reserved. TODO
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/of_irq.h>
#include <linux/mutex.h>

#include <linux/iio/iio.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/buffer.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>

#include "fxos8700.h"

/* Regmap info */
static const struct regmap_range read_range[] = {
	{
		.range_min = 0x00,
		.range_max = 0x18,
	}, {
		.range_min = 0x1d,
		.range_max = 0x78,
	},
};
static const struct regmap_range write_range[] = {
	{
		.range_min = 0x09,
		.range_max = 0x0a,
	}, {
		.range_min = 0x0e,
		.range_max = 0x0f,
	}, {
		.range_min = 0x11,
		.range_max = 0x15,
	}, {
		.range_min = 0x17,
		.range_max = 0x1d,
	}, {
		.range_min = 0x1f,
		.range_max = 0x21,
	}, {
		.range_min = 0x23,
		.range_max = 0x31,
	}, {
		.range_min = 0x3f,
		.range_max = 0x44,
	}, {
		.range_min = 0x52,
		.range_max = 0x52,
	}, {
		.range_min = 0x54,
		.range_max = 0x5d,
	}, {
		.range_min = 0x5f,
		.range_max = 0x78,
	},
};

struct regmap_access_table driver_read_table = {
	.yes_ranges =   read_range,
	.n_yes_ranges = ARRAY_SIZE(write_read_range),
};

struct regmap_access_table driver_write_table = {
	.yes_ranges =   write_range,
	.n_yes_ranges = ARRAY_SIZE(write_read_range),
};

const struct regmap_config fxos8700_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = FXOS8700_NVM_DATA_BNK0,
	.rd_table = driver_read_table;
	.wr_table = driver_write_table;
};
EXPORT_SYMBOL(fxos8700_regmap_config);

#define FXOS8700_CHANNEL(_type, _axis, _index) {		\
	.type = _type,						\
	.modified = 1,						\
	.channel2 = IIO_MOD_##_axis,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |  \
		BIT(IIO_CHAN_INFO_SAMP_FREQ),			\
	.scan_index = _index,					\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 16,					\
		.storagebits = 16,				\
		.endianness = IIO_LE,				\
	},							\
}

/* scan indexes follow DATA register order */
enum fxos8700_scan_axis {
	FXOS8700_SCAN_ACCEL_X = 0,
	FXOS8700_SCAN_ACCEL_Y,
	FXOS8700_SCAN_ACCEL_Z,
	FXOS8700_SCAN_MAGN_X,
	FXOS8700_SCAN_MAGN_Y,
	FXOS8700_SCAN_MAGN_Z,
	FXOS8700_SCAN_RHALL,
	FXOS8700_SCAN_TIMESTAMP,
};

enum fxos8700_sensor {
	FXOS8700_ACCEL	= 0,
	FXOS8700_MAGN,
	FXOS8700_NUM_SENSORS /* must be last */
};

enum fxos8700_int_pin {
	FXOS8700_PIN_INT1,
	FXOS8700_PIN_INT2
};

struct fxos8700_scale {
	u8 bits;
	int uscale;
};

struct fxos8700_odr {
	u8 bits;
	int odr;
	int uodr;
};

/* See  section 8.1 */
static const struct fxos8700_scale fxos8700_accel_scale[] = {
	{ MODE_2G, 244},
	{ MODE_4G, 488},
	{ MODE_8G, 976},
};

/* accell and magn have the same ODR options, set in the CTRL_REG1 reg
 * ODR is halved when using both sensors at once in hybrid mode
 */
static const struct fxos8700_odr fxos8700_odr[] = {
	{0x00, 800, 0},
	{0x01, 400, 0},
	{0x02, 200, 0},
	{0x03, 100, 0},
	{0x04, 50, 0},
	{0x05, 12, 500000},
	{0x06, 6, 250000},
	{0x07, 1, 562500},
};

static const struct iio_chan_spec fxos8700_channels[] = { //TODO: see if macro needs to change
	FXOS8700_CHANNEL(IIO_ACCEL, X, FXOS8700_SCAN_ACCEL_X),
	FXOS8700_CHANNEL(IIO_ACCEL, Y, FXOS8700_SCAN_ACCEL_Y),
	FXOS8700_CHANNEL(IIO_ACCEL, Z, FXOS8700_SCAN_ACCEL_Z),
	FXOS8700_CHANNEL(IIO_MAGN, X, FXOS8700_SCAN_MAGN_X),
	FXOS8700_CHANNEL(IIO_MAGN, Y, FXOS8700_SCAN_MAGN_Y),
	FXOS8700_CHANNEL(IIO_MAGN, Z, FXOS8700_SCAN_MAGN_Z),
	IIO_CHAN_SOFT_TIMESTAMP(FXOS8700_SCAN_TIMESTAMP),
};

static enum fxos8700_sensor fxos8700_to_sensor(enum iio_chan_type iio_type) {
	switch (iio_type) {
	case IIO_ACCEL:
		return FXOS8700_ACCEL;
	case IIO_ANGL_VEL:
		return FXOS8700_MAGN;
	default:
		return -EINVAL;
	}
}

static
int fxos8700_set_active_mode(struct fxos8700_data *data,
			 enum fxos8700_sensor t, bool mode)
{
	int ret;

	ret = regmap_write(data->regmap, FXOS8700_CTRL_REG1, mode);
	if (ret)
		return ret;

	usleep_range(FXOS8700_ACTIVE_MIN_USLEEP,
		FXOS8700_ACTIVE_MIN_USLEEP + 1000);

	return 0;
}

static
int fxos8700_set_scale(struct fxos8700_data *data, enum fxos8700_sensor t,
			 int uscale)
{
	int i;
	static const int scale_num = ARRAY_SIZE(fxos8700_accel_scale);

	if (t == FXOS8700_MAGN) {
		printk("Magnetometer scale is locked at 1200uT");
		return -EINVAL;
	}

	for (i = 0; i < scale_num; i++)
		if (fxos8700_accel_scale[i].uscale == uscale)
			break;

	if (i == scale_num)
		return -EINVAL;

	return regmap_write(data->regmap, FXOS8700_XYZ_DATA_CFG,
				fxos8700_accel_scale[i].bits);
}

static
int fxos8700_get_scale(struct fxos8700_data *data, enum fxos8700_sensor t,
			 int *uscale)
{
	int i, ret, val;
	static const int scale_num = ARRAY_SIZE(fxos8700_accel_scale);

	if (t == FXOS8700_MAGN) {
		*uscale = 1200; // Magnetometer is locked at 1200uT
		return 0;
	}

	ret = regmap_read(data->regmap, FXOS8700_XYZ_DATA_CFG, &val);
	if (ret)
		return ret;


	for (i = 0; i < scale_num; i++)
		if (fxos8700_accel_scale[i].bits == val & 0x3) {
			*uscale = fxos8700_accel_scale[i].uscale;
			return 0;
		}

	return -EINVAL;
}

static int fxos8700_get_data(struct fxos8700_data *data, int chan_type,
			   int axis, int *val)
{
	u8 reg;
	int ret;
	__le16 sample;
	enum fxos8700_sensor t = fxos8700_to_sensor(chan_type);

	reg = fxos8700_regs[t].data + (axis - IIO_MOD_X) * sizeof(sample);

	ret = regmap_bulk_read(data->regmap, reg, &sample, sizeof(sample));
	if (ret)
		return ret;

	*val = sign_extend32(le16_to_cpu(sample), 15);

	return 0;
}

static
int fxos8700_set_odr(struct fxos8700_data *data, enum fxos8700_sensor t,
		   int odr, int uodr)
{
	int i, val;
	bool active_mode;
	static const int odr_num = ARRAY_SIZE(fxos8700_odr);


	ret = regmap_read(data->regmap, FXOS8700_CTRL_REG1, &val);
	if (ret)
		return ret;

	active_mode = val & FXOS8700_ACTIVE;


	if (active_mode) {
		/* The device must be in standby mode to change any of the other fields
		 * within CTRL_REG1
		 */
		ret = regmap_write(data->regmap, FXOS8700_CTRL_REG1,
				   val & ~FXOS8700_ACTIVE);
		if (ret)
			return ret;
	}

	for (i = 0; i < odr_num; i++)
		if (fxos8700_odr[i].odr == odr &&
			fxos8700_odr[i].uodr == uodr)
			break;

	if (i >= odr_num)
		return -EINVAL;

	return regmap_update_bits(data->regmap,
				  FXOS8700_CTRL_REG1,
				  FXOS8700_CTRL_ODR_MSK + FXOS8700_ACTIVE,
				  fxos8700_odr[i].bits << 3 | active_mode);
}

static int fxos8700_get_odr(struct fxos8700_data *data, enum fxos8700_sensor t,
			  int *odr, int *uodr)
{
	int i, val, ret;
	static const int odr_num = ARRAY_SIZE(fxos8700_odr);

	ret = regmap_read(data->regmap, FXOS8700_CTRL_REG1, &val);
	if (ret)
		return ret;

	val &= FXOS8700_CTRL_ODR_MSK;

	for (i = 0; i < odr_num; i++)
		if (val == fxos8700_odr.bits)
			break;

	if (i >= odr_num)
		return -EINVAL;

	*odr = fxos8700_odr[i].odr;
	*uodr = fxos8700_odr[i].uodr;

	return 0;
}

static irqreturn_t fxos8700_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct fxos8700_data *data = iio_priv(indio_dev);
	__le16 buf[16];
	/* 3 sens x 3 axis x __le16 + 3 x __le16 pad + 4 x __le16 tstamp */
	int i, ret, j = 0, base = FXOS8700_OUT_X_MSB;
	__le16 sample;

	for_each_set_bit(i, indio_dev->active_scan_mask,
			 indio_dev->masklength) {
		ret = regmap_bulk_read(data->regmap, base + i * sizeof(sample),
					   &sample, sizeof(sample));
		if (ret)
			goto done;
		buf[j++] = sample;
	}

	iio_push_to_buffers_with_timestamp(indio_dev, buf, pf->timestamp);
done:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

static int fxos8700_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	int ret;
	struct fxos8700_data *data = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = fxos8700_get_data(data, chan->type, chan->channel2, val);
		if (ret)
			return ret;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		ret = fxos8700_get_scale(data, fxos8700_to_sensor(chan->type),
					 val2);
		return ret ? ret : IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = fxos8700_get_odr(data, fxos8700_to_sensor(chan->type),
					 val, val2);
		return ret ? ret : IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}

	return 0;
}

static int fxos8700_write_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int val, int val2, long mask)
{
	struct fxos8700_data *data = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		return fxos8700_set_scale(data,
					fxos8700_to_sensor(chan->type), val2);
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		return fxos8700_set_odr(data, fxos8700_to_sensor(chan->type),
					  val, val2);
	default:
		return -EINVAL;
	}

	return 0;
}

static
IIO_CONST_ATTR(in_accel_sampling_frequency_available,
		   "1.5625 6.25 12.5 50 100 200 400 800");
static
IIO_CONST_ATTR(in_magn_sampling_frequency_available,
		   "1.5625 6.25 12.5 50 100 200 400 800");
static
IIO_CONST_ATTR(in_accel_scale_available,
		   "0.000244 0.000488 0.000976");
static
IIO_CONST_ATTR(in_magn_scale_available,
		   "0.000001200");

static struct attribute *fxos8700_attrs[] = {
	&iio_const_attr_in_accel_sampling_frequency_available.dev_attr.attr,
	&iio_const_attr_in_magn_sampling_frequency_available.dev_attr.attr,
	&iio_const_attr_in_accel_scale_available.dev_attr.attr,
	&iio_const_attr_in_magn_scale_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group fxos8700_attrs_group = {
	.attrs = fxos8700_attrs,
};

static const struct iio_info fxos8700_info = {
	.read_raw = fxos8700_read_raw,
	.write_raw = fxos8700_write_raw,
	.attrs = &fxos8700_attrs_group,
};

static const char *fxos8700_match_acpi_device(struct device *dev)
{
	const struct acpi_device_id *id;

	id = acpi_match_device(dev->driver->acpi_match_table, dev);
	if (!id)
		return NULL;

	return dev_name(dev);
}

static int fxos8700_config_pin(struct regmap *regmap, enum fxos8700_int_pin pin,
				 bool open_drain, u8 irq_mask,
				 unsigned long write_usleep)
{
	int ret;
	struct device *dev = regmap_get_device(regmap);
	u8 int_out_ctrl_shift;
	u8 int_latch_mask;
	u8 int_map_mask;
	u8 int_out_ctrl_mask;
	u8 int_out_ctrl_bits;
	const char *pin_name;

	switch (pin) {
	case FXOS8700_PIN_INT1:
		int_out_ctrl_shift = FXOS8700_INT1_OUT_CTRL_SHIFT;
		int_latch_mask = FXOS8700_INT1_LATCH_MASK;
		int_map_mask = FXOS8700_INT1_MAP_DRDY_EN;
		break;
	case FXOS8700_PIN_INT2:
		int_out_ctrl_shift = FXOS8700_INT2_OUT_CTRL_SHIFT;
		int_latch_mask = FXOS8700_INT2_LATCH_MASK;
		int_map_mask = FXOS8700_INT2_MAP_DRDY_EN;
		break;
	}
	int_out_ctrl_mask = FXOS8700_INT_OUT_CTRL_MASK << int_out_ctrl_shift;

	/*
	 * Enable the requested pin with the right settings:
	 * - Push-pull/open-drain
	 * - Active low/high
	 * - Edge/level triggered
	 */
	int_out_ctrl_bits = FXOS8700_OUTPUT_EN;
	if (open_drain)
		/* Default is push-pull. */
		int_out_ctrl_bits |= FXOS8700_OPEN_DRAIN;
	int_out_ctrl_bits |= irq_mask;
	int_out_ctrl_bits <<= int_out_ctrl_shift;

	ret = regmap_update_bits(regmap, FXOS8700_REG_INT_OUT_CTRL,
				 int_out_ctrl_mask, int_out_ctrl_bits);
	if (ret)
		return ret;

	/* Set the pin to input mode with no latching. */
	ret = regmap_update_bits(regmap, FXOS8700_REG_INT_LATCH,
				 int_latch_mask, int_latch_mask);
	if (ret)
		return ret;

	/* Map interrupts to the requested pin. */
	ret = regmap_update_bits(regmap, FXOS8700_REG_INT_MAP,
				 int_map_mask, int_map_mask);
	if (ret) {
		switch (pin) {
		case FXOS8700_PIN_INT1:
			pin_name = "INT1";
			break;
		case FXOS8700_PIN_INT2:
			pin_name = "INT2";
			break;
		}
		dev_err(dev, "Failed to configure %s IRQ pin", pin_name);
	}

	return ret;
}

int fxos8700_enable_irq(struct regmap *regmap, bool enable)
{
	unsigned int enable_bit = 0;

	if (enable)
		enable_bit = FXOS8700_DRDY_INT_EN;

	return regmap_update_bits(regmap, FXOS8700_REG_INT_EN,
				  FXOS8700_DRDY_INT_EN, enable_bit);
}
EXPORT_SYMBOL(fxos8700_enable_irq);

static int fxos8700_get_irq(struct device_node *of_node, enum fxos8700_int_pin *pin)
{
	int irq;

	/* Use INT1 if possible, otherwise fall back to INT2. */
	irq = of_irq_get_byname(of_node, "INT1");
	if (irq > 0) {
		*pin = FXOS8700_PIN_INT1;
		return irq;
	}

	irq = of_irq_get_byname(of_node, "INT2");
	if (irq > 0)
		*pin = FXOS8700_PIN_INT2;

	return irq;
}

static int fxos8700_config_device_irq(struct iio_dev *indio_dev, int irq_type,
					enum fxos8700_int_pin pin)
{
	bool open_drain;
	u8 irq_mask;
	struct fxos8700_data *data = iio_priv(indio_dev);
	struct device *dev = regmap_get_device(data->regmap);

	/* Level-triggered, active-low is the default if we set all zeroes. */
	if (irq_type == IRQF_TRIGGER_RISING)
		irq_mask = FXOS8700_ACTIVE_HIGH | FXOS8700_EDGE_TRIGGERED;
	else if (irq_type == IRQF_TRIGGER_FALLING)
		irq_mask = FXOS8700_EDGE_TRIGGERED;
	else if (irq_type == IRQF_TRIGGER_HIGH)
		irq_mask = FXOS8700_ACTIVE_HIGH;
	else if (irq_type == IRQF_TRIGGER_LOW)
		irq_mask = 0;
	else {
		dev_err(&indio_dev->dev,
			"Invalid interrupt type 0x%x specified\n", irq_type);
		return -EINVAL;
	}

	open_drain = of_property_read_bool(dev->of_node, "drive-open-drain");

	return fxos8700_config_pin(data->regmap, pin, open_drain, irq_mask,
				 FXOS8700_NORMAL_WRITE_USLEEP);
}

static int fxos8700_setup_irq(struct iio_dev *indio_dev, int irq,
				enum fxos8700_int_pin pin)
{
	struct irq_data *desc;
	u32 irq_type;
	int ret;

	desc = irq_get_irq_data(irq);
	if (!desc) {
		dev_err(&indio_dev->dev, "Could not find IRQ %d\n", irq);
		return -EINVAL;
	}

	irq_type = irqd_get_trigger_type(desc);

	ret = fxos8700_config_device_irq(indio_dev, irq_type, pin);
	if (ret)
		return ret;

	return fxos8700_probe_trigger(indio_dev, irq, irq_type);
}

static int fxos8700_chip_init(struct fxos8700_data *data, bool use_spi)
{
	int ret;
	unsigned int val;
	struct device *dev = regmap_get_device(data->regmap);

	// ret = regmap_write(data->regmap, FXOS8700_REG_CMD, FXOS8700_CMD_SOFTRESET);
	// if (ret)
	// 	return ret;
	// usleep_range(FXOS8700_SOFTRESET_USLEEP, FXOS8700_SOFTRESET_USLEEP + 1);
	/*
	 * CS rising edge is needed before starting SPI, so do a dummy read
	 * See Section 3.2.1, page 86 of the datasheet
	 */
	// if (use_spi) {
	// 	ret = regmap_read(data->regmap, FXOS8700_REG_DUMMY, &val);
	// 	if (ret)
	// 		return ret;
	// }

	ret = regmap_read(data->regmap, FXOS8700_WHO_AM_I, &val);
	if (ret) {
		dev_err(dev, "Error reading chip id\n");
		return ret;
	}
	if (val != FXOS8700_DEVICE_ID && val != FXOS8700_PRE_DEVICE_ID) {
		dev_err(dev, "Wrong chip id, got %x expected %x or %x \n",
			val, FXOS8700_DEVICE_ID, FXOS8700_PRE_DEVICE_ID);
		return -ENODEV;
	}

	ret = fxos8700_set_active_mode(data, FXOS8700_ACCEL, true);
	if (ret)
		return ret;

	ret = fxos8700_set_active_mode(data, FXOS8700_MAGN, true);
	if (ret)
		return ret;





	/* TODO: set interrupt pin as open-drain */
	// if (of_get_property(np, "interrupt-open-drain", NULL)) {
	// 	result = i2c_smbus_write_byte_data(client, FXOS8700_CTRL_REG3, 0x01);
	// 	if (result < 0)
	// 		goto out;
	// }

	/* The device must be in standby mode to change any of the other fields
	 * within CTRL_REG1
	 */
	ret = regmap_write(data->regmap, FXOS8700_CTRL_REG1, 0x00);
	if (ret)
		return ret;
	// Set max oversample ratio (OSR) and both devices active
	ret = regmap_write(data->regmap, FXOS8700_M_CTRL_REG1, 0x1F);
	if (ret)
		return ret;
	// Disable and rst min/max measurements & threshold, enable hybrid autoinc
	ret = regmap_write(data->regmap, FXOS8700_M_CTRL_REG2, 0x3c);
	if (ret)
		return ret;
	// Min ODR (1.56Hz individual or 0.78Hz hybrid), leave in standby mode
	ret = regmap_write(data->regmap, FXOS8700_CTRL_REG1, 0x03 << 3);
	if (ret)
		return ret;
	// Set for max full-scale range (+/-8G)
	ret = regmap_write(data->regmap, FXOS8700_XYZ_DATA_CFG, MODE_8G);
	if (ret)
		return ret;

	atomic_set(&pdata->acc_active, FXOS8700_STANDBY);
	atomic_set(&pdata->mag_active, FXOS8700_STANDBY);
	atomic_set(&pdata->position, FXOS8700_POSITION_DEFAULT);
	atomic_set(&pdata->range, MODE_8G);




	printk("fxos8700 device driver probe successfully");
	return 0;
out:
	dev_err(&client->dev, "Error when init fxos8700 device:(%d)", result);
	return result;
}

static int fxos8700_data_rdy_trigger_set_state(struct iio_trigger *trig,
						 bool enable)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct fxos8700_data *data = iio_priv(indio_dev);

	return fxos8700_enable_irq(data->regmap, enable);
}

static const struct iio_trigger_ops fxos8700_trigger_ops = {
	.set_trigger_state = &fxos8700_data_rdy_trigger_set_state,
};

int fxos8700_probe_trigger(struct iio_dev *indio_dev, int irq, u32 irq_type)
{
	struct fxos8700_data *data = iio_priv(indio_dev);
	int ret;

	data->trig = devm_iio_trigger_alloc(&indio_dev->dev, "%s-dev%d",
						indio_dev->name, indio_dev->id);

	if (data->trig == NULL)
		return -ENOMEM;

	ret = devm_request_irq(&indio_dev->dev, irq,
				   &iio_trigger_generic_data_rdy_poll,
				   irq_type, "fxos8700", data->trig);
	if (ret)
		return ret;

	data->trig->dev.parent = regmap_get_device(data->regmap);
	data->trig->ops = &fxos8700_trigger_ops;
	iio_trigger_set_drvdata(data->trig, indio_dev);

	ret = devm_iio_trigger_register(&indio_dev->dev, data->trig);
	if (ret)
		return ret;

	indio_dev->trig = iio_trigger_get(data->trig);

	return 0;
}

int fxos8700_core_probe(struct device *dev, struct regmap *regmap,
			  const char *name, bool use_spi)
{
	struct iio_dev *indio_dev;
	struct fxos8700_data *data;
	int irq;
	enum fxos8700_int_pin int_pin;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	dev_set_drvdata(dev, indio_dev);
	data->regmap = regmap;

	ret = fxos8700_chip_init(data, use_spi);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, fxos8700_chip_uninit, data);
	if (ret)
		return ret;

	if (!name && ACPI_HANDLE(dev))
		name = fxos8700_match_acpi_device(dev);

	indio_dev->dev.parent = dev;
	indio_dev->channels = fxos8700_channels;
	indio_dev->num_channels = ARRAY_SIZE(fxos8700_channels);
	indio_dev->name = name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &fxos8700_info;

	ret = devm_iio_triggered_buffer_setup(dev, indio_dev,
						  iio_pollfunc_store_time,
						  fxos8700_trigger_handler, NULL);
	if (ret)
		return ret;

	irq = fxos8700_get_irq(dev->of_node, &int_pin);
	if (irq > 0) {
		ret = fxos8700_setup_irq(indio_dev, irq, int_pin);
		if (ret)
			dev_err(&indio_dev->dev, "Failed to setup IRQ %d\n",
				irq);
	} else {
		dev_info(&indio_dev->dev, "Not setting up IRQ trigger\n");
	}

	return devm_iio_device_register(dev, indio_dev);
}
EXPORT_SYMBOL_GPL(fxos8700_core_probe);

MODULE_AUTHOR("Gateworks, Inc.");
MODULE_DESCRIPTION("FXOS8700 6-Axis Acc and Mag Combo Sensor driver");
MODULE_LICENSE("GPL");