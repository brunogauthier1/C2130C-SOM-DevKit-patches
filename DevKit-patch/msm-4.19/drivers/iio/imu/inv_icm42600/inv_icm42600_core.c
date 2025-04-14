// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2020 Invensense, Inc.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/iio/iio.h>

#include "inv_icm42600.h"
#include "inv_icm42600_buffer.h"
#include "inv_icm42600_timestamp.h"

#define USE_VDD 0

static const struct regmap_range_cfg inv_icm42600_regmap_ranges[] = {
	{
		.name = "user banks",
		.range_min = 0x0000,
		.range_max = 0x4FFF,
		.selector_reg = INV_ICM42600_REG_BANK_SEL,
		.selector_mask = INV_ICM42600_BANK_SEL_MASK,
		.selector_shift = 0,
		.window_start = 0,
		.window_len = 0x1000,
	},
};

const struct regmap_config inv_icm42600_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x4FFF,
	.ranges = inv_icm42600_regmap_ranges,
	.num_ranges = ARRAY_SIZE(inv_icm42600_regmap_ranges),
};
EXPORT_SYMBOL_GPL(inv_icm42600_regmap_config);

struct inv_icm42600_hw {
	uint8_t whoami;
	const char *name;
	const struct inv_icm42600_conf *conf;
};

/* chip initial default configuration */
static const struct inv_icm42600_conf inv_icm42600_default_conf = {
	.gyro = {
		.mode = INV_ICM42600_SENSOR_MODE_OFF,
		.fs = INV_ICM42600_GYRO_FS_2000DPS,
		.odr = INV_ICM42600_ODR_50HZ,
		.filter = INV_ICM42600_FILTER_BW_ODR_DIV_2,
	},
	.accel = {
		.mode = INV_ICM42600_SENSOR_MODE_OFF,
		.fs = INV_ICM42600_ACCEL_FS_16G,
		.odr = INV_ICM42600_ODR_50HZ,
		.filter = INV_ICM42600_FILTER_BW_ODR_DIV_2,
	},
	.temp_en = false,
};

static const struct inv_icm42600_hw inv_icm42600_hw[INV_CHIP_NB] = {
	[INV_CHIP_ICM42600] = {
		.whoami = INV_ICM42600_WHOAMI_ICM42600,
		.name = "icm42600",
		.conf = &inv_icm42600_default_conf,
	},
	[INV_CHIP_ICM42602] = {
		.whoami = INV_ICM42600_WHOAMI_ICM42602,
		.name = "icm42602",
		.conf = &inv_icm42600_default_conf,
	},
	[INV_CHIP_ICM42605] = {
		.whoami = INV_ICM42600_WHOAMI_ICM42605,
		.name = "icm42605",
		.conf = &inv_icm42600_default_conf,
	},
	[INV_CHIP_ICM42622] = {
		.whoami = INV_ICM42600_WHOAMI_ICM42622,
		.name = "icm42622",
		.conf = &inv_icm42600_default_conf,
	},
};

const struct iio_mount_matrix *
inv_icm42600_get_mount_matrix(const struct iio_dev *indio_dev,
			      const struct iio_chan_spec *chan)
{
	const struct inv_icm42600_state *st = iio_device_get_drvdata((struct iio_dev *) indio_dev);

	return &st->orientation;
}

uint32_t inv_icm42600_odr_to_period(enum inv_icm42600_odr odr)
{
	static uint32_t odr_periods[INV_ICM42600_ODR_NB] = {
		/* reserved values */
		0, 0, 0,
		/* 8kHz */
		125000,
		/* 4kHz */
		250000,
		/* 2kHz */
		500000,
		/* 1kHz */
		1000000,
		/* 200Hz */
		5000000,
		/* 100Hz */
		10000000,
		/* 50Hz */
		20000000,
		/* 25Hz */
		40000000,
		/* 12.5Hz */
		80000000,
		/* 6.25Hz */
		160000000,
		/* 3.125Hz */
		320000000,
		/* 1.5625Hz */
		640000000,
		/* 500Hz */
		2000000,
	};

	return odr_periods[odr];
}

static int inv_icm42600_set_pwr_mgmt0(struct inv_icm42600_state *st,
				      enum inv_icm42600_sensor_mode gyro,
				      enum inv_icm42600_sensor_mode accel,
				      bool temp, unsigned int *sleep_ms)
{
	enum inv_icm42600_sensor_mode oldgyro = st->conf.gyro.mode;
	enum inv_icm42600_sensor_mode oldaccel = st->conf.accel.mode;
	bool oldtemp = st->conf.temp_en;
	unsigned int sleepval;
	unsigned int val;
	int ret;

    printk("%s:%d ACHEUL, in", __FUNCTION__, __LINE__);

	/* if nothing changed, exit */
	if (gyro == oldgyro && accel == oldaccel && temp == oldtemp){
        printk("%s:%d ACHEUL, out", __FUNCTION__, __LINE__);
		return 0;
    }

	val = INV_ICM42600_PWR_MGMT0_GYRO(gyro) |
	      INV_ICM42600_PWR_MGMT0_ACCEL(accel);
	if (!temp)
		val |= INV_ICM42600_PWR_MGMT0_TEMP_DIS;
	ret = regmap_write(st->map, INV_ICM42600_REG_PWR_MGMT0, val);
	if (ret)
		return ret;

	st->conf.gyro.mode = gyro;
	st->conf.accel.mode = accel;
	st->conf.temp_en = temp;

	/* compute required wait time for sensors to stabilize */
	sleepval = 0;
	/* temperature stabilization time */
	if (temp && !oldtemp) {
		if (sleepval < INV_ICM42600_TEMP_STARTUP_TIME_MS)
			sleepval = INV_ICM42600_TEMP_STARTUP_TIME_MS;
	}
	/* accel startup time */
	if (accel != oldaccel && oldaccel == INV_ICM42600_SENSOR_MODE_OFF) {
		/* block any register write for at least 200 µs */
		usleep_range(200, 300);
		if (sleepval < INV_ICM42600_ACCEL_STARTUP_TIME_MS)
			sleepval = INV_ICM42600_ACCEL_STARTUP_TIME_MS;
	}
	if (gyro != oldgyro) {
		/* gyro startup time */
		if (oldgyro == INV_ICM42600_SENSOR_MODE_OFF) {
			/* block any register write for at least 200 µs */
			usleep_range(200, 300);
			if (sleepval < INV_ICM42600_GYRO_STARTUP_TIME_MS)
				sleepval = INV_ICM42600_GYRO_STARTUP_TIME_MS;
		/* gyro stop time */
		} else if (gyro == INV_ICM42600_SENSOR_MODE_OFF) {
			if (sleepval < INV_ICM42600_GYRO_STOP_TIME_MS)
				sleepval =  INV_ICM42600_GYRO_STOP_TIME_MS;
		}
	}

	/* deferred sleep value if sleep pointer is provided or direct sleep */
	if (sleep_ms)
		*sleep_ms = sleepval;
	else if (sleepval)
		msleep(sleepval);

    printk("%s:%d ACHEUL, out", __FUNCTION__, __LINE__);
	return 0;
}

int inv_icm42600_set_accel_conf(struct inv_icm42600_state *st,
				struct inv_icm42600_sensor_conf *conf,
				unsigned int *sleep_ms)
{
	struct inv_icm42600_sensor_conf *oldconf = &st->conf.accel;
	unsigned int val;
	int ret;

	/* Sanitize missing values with current values */
	if (conf->mode < 0)
		conf->mode = oldconf->mode;
	if (conf->fs < 0)
		conf->fs = oldconf->fs;
	if (conf->odr < 0)
		conf->odr = oldconf->odr;
	if (conf->filter < 0)
		conf->filter = oldconf->filter;

	/* set ACCEL_CONFIG0 register (accel fullscale & odr) */
	if (conf->fs != oldconf->fs || conf->odr != oldconf->odr) {
		val = INV_ICM42600_ACCEL_CONFIG0_FS(conf->fs) |
		      INV_ICM42600_ACCEL_CONFIG0_ODR(conf->odr);
		ret = regmap_write(st->map, INV_ICM42600_REG_ACCEL_CONFIG0, val);
		if (ret)
			return ret;
		oldconf->fs = conf->fs;
		oldconf->odr = conf->odr;
	}

	/* set GYRO_ACCEL_CONFIG0 register (accel filter) */
	if (conf->filter != oldconf->filter) {
		val = INV_ICM42600_GYRO_ACCEL_CONFIG0_ACCEL_FILT(conf->filter) |
		      INV_ICM42600_GYRO_ACCEL_CONFIG0_GYRO_FILT(st->conf.gyro.filter);
		ret = regmap_write(st->map, INV_ICM42600_REG_GYRO_ACCEL_CONFIG0, val);
		if (ret)
			return ret;
		oldconf->filter = conf->filter;
	}

	/* set PWR_MGMT0 register (accel sensor mode) */
	return inv_icm42600_set_pwr_mgmt0(st, st->conf.gyro.mode, conf->mode,
					  st->conf.temp_en, sleep_ms);
}

int inv_icm42600_set_gyro_conf(struct inv_icm42600_state *st,
			       struct inv_icm42600_sensor_conf *conf,
			       unsigned int *sleep_ms)
{
	struct inv_icm42600_sensor_conf *oldconf = &st->conf.gyro;
	unsigned int val;
	int ret;

	/* sanitize missing values with current values */
	if (conf->mode < 0)
		conf->mode = oldconf->mode;
	if (conf->fs < 0)
		conf->fs = oldconf->fs;
	if (conf->odr < 0)
		conf->odr = oldconf->odr;
	if (conf->filter < 0)
		conf->filter = oldconf->filter;

	/* set GYRO_CONFIG0 register (gyro fullscale & odr) */
	if (conf->fs != oldconf->fs || conf->odr != oldconf->odr) {
		val = INV_ICM42600_GYRO_CONFIG0_FS(conf->fs) |
		      INV_ICM42600_GYRO_CONFIG0_ODR(conf->odr);
		ret = regmap_write(st->map, INV_ICM42600_REG_GYRO_CONFIG0, val);
		if (ret)
			return ret;
		oldconf->fs = conf->fs;
		oldconf->odr = conf->odr;
	}

	/* set GYRO_ACCEL_CONFIG0 register (gyro filter) */
	if (conf->filter != oldconf->filter) {
		val = INV_ICM42600_GYRO_ACCEL_CONFIG0_ACCEL_FILT(st->conf.accel.filter) |
		      INV_ICM42600_GYRO_ACCEL_CONFIG0_GYRO_FILT(conf->filter);
		ret = regmap_write(st->map, INV_ICM42600_REG_GYRO_ACCEL_CONFIG0, val);
		if (ret)
			return ret;
		oldconf->filter = conf->filter;
	}

	/* set PWR_MGMT0 register (gyro sensor mode) */
	return inv_icm42600_set_pwr_mgmt0(st, conf->mode, st->conf.accel.mode,
					  st->conf.temp_en, sleep_ms);

	return 0;
}

int inv_icm42600_set_temp_conf(struct inv_icm42600_state *st, bool enable,
			       unsigned int *sleep_ms)
{
	return inv_icm42600_set_pwr_mgmt0(st, st->conf.gyro.mode,
					  st->conf.accel.mode, enable,
					  sleep_ms);
}

int inv_icm42600_debugfs_reg(struct iio_dev *indio_dev, unsigned int reg,
			     unsigned int writeval, unsigned int *readval)
{
	struct inv_icm42600_state *st = iio_device_get_drvdata(indio_dev);
	int ret;

	mutex_lock(&st->lock);

	if (readval)
		ret = regmap_read(st->map, reg, readval);
	else
		ret = regmap_write(st->map, reg, writeval);

	mutex_unlock(&st->lock);

	return ret;
}

static int inv_icm42600_set_conf(struct inv_icm42600_state *st,
				 const struct inv_icm42600_conf *conf)
{
	unsigned int val;
	int ret;

    printk("%s:%d, ACHEUL in", __FUNCTION__, __LINE__);

	/* set PWR_MGMT0 register (gyro & accel sensor mode, temp enabled) */
	val = INV_ICM42600_PWR_MGMT0_GYRO(conf->gyro.mode) |
	      INV_ICM42600_PWR_MGMT0_ACCEL(conf->accel.mode);
	if (!conf->temp_en)
		val |= INV_ICM42600_PWR_MGMT0_TEMP_DIS;
	ret = regmap_write(st->map, INV_ICM42600_REG_PWR_MGMT0, val);
	if (ret){
        printk("%s:%d, ACHEUL, error: %d", __FUNCTION__, __LINE__, ret);
		return ret;
    }

	/* set GYRO_CONFIG0 register (gyro fullscale & odr) */
	val = INV_ICM42600_GYRO_CONFIG0_FS(conf->gyro.fs) |
	      INV_ICM42600_GYRO_CONFIG0_ODR(conf->gyro.odr);
	ret = regmap_write(st->map, INV_ICM42600_REG_GYRO_CONFIG0, val);
	if (ret){
        printk("%s:%d, ACHEUL, error: %d", __FUNCTION__, __LINE__, ret);
		return ret;
    }

	/* set ACCEL_CONFIG0 register (accel fullscale & odr) */
	val = INV_ICM42600_ACCEL_CONFIG0_FS(conf->accel.fs) |
	      INV_ICM42600_ACCEL_CONFIG0_ODR(conf->accel.odr);
	ret = regmap_write(st->map, INV_ICM42600_REG_ACCEL_CONFIG0, val);
	if (ret){
        printk("%s:%d, ACHEUL, error: %d", __FUNCTION__, __LINE__, ret);
		return ret;
    }

	/* set GYRO_ACCEL_CONFIG0 register (gyro & accel filters) */
	val = INV_ICM42600_GYRO_ACCEL_CONFIG0_ACCEL_FILT(conf->accel.filter) |
	      INV_ICM42600_GYRO_ACCEL_CONFIG0_GYRO_FILT(conf->gyro.filter);
	ret = regmap_write(st->map, INV_ICM42600_REG_GYRO_ACCEL_CONFIG0, val);
	if (ret){
        printk("%s:%d, ACHEUL, error: %d", __FUNCTION__, __LINE__, ret);
		return ret;
    }

	/* update internal conf */
	st->conf = *conf;

    printk("%s:%d, ACHEUL out", __FUNCTION__, __LINE__);
	return 0;
}

/**
 *  inv_icm42600_setup() - check and setup chip
 *  @st:	driver internal state
 *  @bus_setup:	callback for setting up bus specific registers
 *
 *  Returns 0 on success, a negative error code otherwise.
 */
static int inv_icm42600_setup(struct inv_icm42600_state *st, inv_icm42600_bus_setup bus_setup)
{
	const struct inv_icm42600_hw *hw = &inv_icm42600_hw[st->chip];
	const struct device *dev = regmap_get_device(st->map);
	unsigned int val;
	int ret;

    printk("%s:%d, ACHEUL in", __FUNCTION__, __LINE__);

	/* check chip self-identification value */
	ret = regmap_read(st->map, INV_ICM42600_REG_WHOAMI, &val);
	if (ret){
        printk("%s:%d, ACHEUL, failed read regmap: %d", __FUNCTION__, __LINE__, ret);
		return ret;
    }

    printk("%s:%d, ACHEUL val: %d, whoami: %d, name: %s", __FUNCTION__, __LINE__, val, hw->whoami, hw->name);

	if (val != hw->whoami) {
		dev_err(dev, "invalid whoami %#02x expected %#02x (%s)\n", val, hw->whoami, hw->name);
        printk("%s:%d, invalid whoami %#02x expected %#02x (%s)\n", __FUNCTION__, __LINE__, val, hw->whoami, hw->name);
		return -ENODEV;
	}
	st->name = hw->name;

    printk("%s:%d, ACHEUL", __FUNCTION__, __LINE__);
	/* reset to make sure previous state are not there */
	ret = regmap_write(st->map, INV_ICM42600_REG_DEVICE_CONFIG, INV_ICM42600_DEVICE_CONFIG_SOFT_RESET);
	if (ret){
        printk("%s:%d, ACHEUL, failed write regmap: %d", __FUNCTION__, __LINE__, ret);
		return ret;
    }
	msleep(INV_ICM42600_RESET_TIME_MS);

    printk("%s:%d, ACHEUL", __FUNCTION__, __LINE__);
	ret = regmap_read(st->map, INV_ICM42600_REG_INT_STATUS, &val);
	if (ret){
        printk("%s:%d, ACHEUL, failed read regmap: %d", __FUNCTION__, __LINE__, ret);
		return ret;
    }

    printk("%s:%d, ACHEUL", __FUNCTION__, __LINE__);
	if (!(val & INV_ICM42600_INT_STATUS_RESET_DONE)) {
		dev_err(dev, "reset error, reset done bit not set\n");
        printk("%s:%d, ACHUEL, reset problem", __FUNCTION__, __LINE__);
		return -ENODEV;
	}

    printk("%s:%d, ACHEUL", __FUNCTION__, __LINE__);
	/* set chip bus configuration */
	ret = bus_setup(st);
	if (ret) {
        printk("%s:%d, ACHEUL, failed to bus setup: %d", __FUNCTION__, __LINE__, ret);
		return ret;
    }

    printk("%s:%d, ACHEUL", __FUNCTION__, __LINE__);
	/* sensor data in big-endian (default) */
	ret = regmap_update_bits(st->map, INV_ICM42600_REG_INTF_CONFIG0,
				 INV_ICM42600_INTF_CONFIG0_SENSOR_DATA_ENDIAN,
				 INV_ICM42600_INTF_CONFIG0_SENSOR_DATA_ENDIAN);
	if (ret) {
        printk("%s:%d, ACHEUL, failed to regmap_update_bits ret: %d", __FUNCTION__, __LINE__, ret);
		return ret;
    }

    printk("%s:%d, ACHEUL", __FUNCTION__, __LINE__);
	ret = inv_icm42600_set_conf(st, hw->conf);

    printk("%s:%d, ACHEUL out: ret: %d", __FUNCTION__, __LINE__, ret);
    return ret;
}

static irqreturn_t inv_icm42600_irq_timestamp(int irq, void *_data)
{
	struct inv_icm42600_state *st = _data;

	st->timestamp.gyro = iio_get_time_ns(st->indio_gyro);
	st->timestamp.accel = iio_get_time_ns(st->indio_accel);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t inv_icm42600_irq_handler(int irq, void *_data)
{
	struct inv_icm42600_state *st = _data;
	struct device *dev = regmap_get_device(st->map);
	unsigned int status;
	int ret;

	mutex_lock(&st->lock);

	ret = regmap_read(st->map, INV_ICM42600_REG_INT_STATUS, &status);
	if (ret)
		goto out_unlock;

	/* FIFO full */
	if (status & INV_ICM42600_INT_STATUS_FIFO_FULL)
		dev_warn(dev, "FIFO full data lost!\n");

	/* FIFO threshold reached */
	if (status & INV_ICM42600_INT_STATUS_FIFO_THS) {
		ret = inv_icm42600_buffer_fifo_read(st, 0);
		if (ret) {
			dev_err(dev, "FIFO read error %d\n", ret);
			goto out_unlock;
		}
		ret = inv_icm42600_buffer_fifo_parse(st);
		if (ret)
			dev_err(dev, "FIFO parsing error %d\n", ret);
	}

out_unlock:
	mutex_unlock(&st->lock);
	return IRQ_HANDLED;
}

/**
 * inv_icm42600_irq_init() - initialize int pin and interrupt handler
 * @st:		driver internal state
 * @irq:	irq number
 * @irq_type:	irq trigger type
 * @open_drain:	true if irq is open drain, false for push-pull
 *
 * Returns 0 on success, a negative error code otherwise.
 */
static int inv_icm42600_irq_init(struct inv_icm42600_state *st, int irq, int irq_type, bool open_drain)
{
	struct device *dev = regmap_get_device(st->map);
	unsigned int val;
	int ret;

    printk("%s:%d, ACHEUL, in", __FUNCTION__, __LINE__);

	/* configure INT1 interrupt: default is active low on edge */
	switch (irq_type) {
	case IRQF_TRIGGER_RISING:
	case IRQF_TRIGGER_HIGH:
		val = INV_ICM42600_INT_CONFIG_INT1_ACTIVE_HIGH;
		break;
	default:
		val = INV_ICM42600_INT_CONFIG_INT1_ACTIVE_LOW;
		break;
	}

	switch (irq_type) {
	case IRQF_TRIGGER_LOW:
	case IRQF_TRIGGER_HIGH:
		val |= INV_ICM42600_INT_CONFIG_INT1_LATCHED;
		break;
	default:
		break;
	}

	if (!open_drain)
		val |= INV_ICM42600_INT_CONFIG_INT1_PUSH_PULL;

	ret = regmap_write(st->map, INV_ICM42600_REG_INT_CONFIG, val);
	if (ret) {
        printk("%s:%d, ACHEUL, error: %d", __FUNCTION__, __LINE__, ret);
		return ret;
    }

	/* Deassert async reset for proper INT pin operation (cf datasheet) */
	ret = regmap_update_bits(st->map, INV_ICM42600_REG_INT_CONFIG1,
				 INV_ICM42600_INT_CONFIG1_ASYNC_RESET, 0);
	if (ret){
        printk("%s:%d, ACHEUL, error: %d", __FUNCTION__, __LINE__, ret);
		return ret;
    }

	ret = devm_request_threaded_irq(dev, irq, inv_icm42600_irq_timestamp, inv_icm42600_irq_handler, irq_type, "inv_icm42600", st);

    printk("%s:%d, ACHEUL, out: ret: %d", __FUNCTION__, __LINE__, ret);
    return ret;
}

static int inv_icm42600_enable_regulator_vddio(struct inv_icm42600_state *st)
{

#if USE_VDD
	int ret;
    printk("%s:%d ACHEUL, VDD management is enabled", __FUNCTION__, __LINE__);

	ret = regulator_enable(st->vddio_supply);
	if (ret)
		return ret;

	/* wait a little for supply ramp */
	usleep_range(3000, 4000);
#else
    printk("%s:%d ACHEUL, FIXME! VDD management is disabled", __FUNCTION__, __LINE__);
#endif // USE_VDD

	return 0;
}

#if USE_VDD
static void inv_icm42600_disable_vdd_reg(void *_data)
{
	struct inv_icm42600_state *st = _data;
	const struct device *dev = regmap_get_device(st->map);
	int ret;

	ret = regulator_disable(st->vdd_supply);
	if (ret)
		dev_err(dev, "failed to disable vdd error %d\n", ret);
}
#endif // USE_VDD

#if USE_VDD
static void inv_icm42600_disable_vddio_reg(void *_data)
{
	struct inv_icm42600_state *st = _data;
	const struct device *dev = regmap_get_device(st->map);
	int ret;

	ret = regulator_disable(st->vddio_supply);
	if (ret)
		dev_err(dev, "failed to disable vddio error %d\n", ret);
}
#endif // USE_VDD

static void inv_icm42600_disable_pm(void *_data)
{
	struct device *dev = _data;

	pm_runtime_put_sync(dev);
	pm_runtime_disable(dev);
}

int inv_icm42600_core_probe(struct regmap *regmap, int chip, int irq, inv_icm42600_bus_setup bus_setup)
{
	struct device *dev = regmap_get_device(regmap);
	struct inv_icm42600_state *st;
	struct irq_data *irq_desc;
	int irq_type;
	bool open_drain;
	int ret;

    printk("%s:%d, ACHEUL, IN", __FUNCTION__, __LINE__);

	if (chip <= INV_CHIP_INVALID || chip >= INV_CHIP_NB) {
		dev_err(dev, "ACHEUL, invalid chip = %d\n", chip);
		return -ENODEV;
	}

    printk("%s:%d, ACHEUL, chip: %d", __FUNCTION__, __LINE__, chip);

	/* get irq properties, set trigger falling by default */
	irq_desc = irq_get_irq_data(irq);
	if (!irq_desc) {
		dev_err(dev, "ACHEUL, could not find IRQ: %d\n", irq);
		return -EINVAL;
	}

	irq_type = irqd_get_trigger_type(irq_desc);
	if (!irq_type)
		irq_type = IRQF_TRIGGER_FALLING;

	open_drain = device_property_read_bool(dev, "drive-open-drain");

	st = devm_kzalloc(dev, sizeof(*st), GFP_KERNEL);
	if (!st) {
		dev_err(dev, "ACHEUL, could allocate mem\n");
		return -ENOMEM;
    }

	dev_set_drvdata(dev, st);
	mutex_init(&st->lock);
	st->chip = chip;
	st->map = regmap;

	ret = of_iio_read_mount_matrix(dev, "mount-matrix", &st->orientation);
	if (ret) {
		dev_err(dev, "ACHEUL, FIXME! failed to retrieve mounting matrix %d\n", ret);
		return ret;
	}

// ===============================================
// ACHEUL, FIXME VDD/VDDIO not supported yet
#if USE_VDD
    printk("%s:%d ACHEUL, VDD management is enabled", __FUNCTION__, __LINE__);

	st->vdd_supply = devm_regulator_get(dev, "vdd");
	if (IS_ERR(st->vdd_supply)) {
		dev_err(dev, "failed, no vdd supply\n");
		//return PTR_ERR(st->vdd_supply);
    }

	st->vddio_supply = devm_regulator_get(dev, "vddio");
	if (IS_ERR(st->vddio_supply)) {
		dev_err(dev, "failed, no vddio supply\n");
		//return PTR_ERR(st->vddio_supply);
    }

	ret = regulator_enable(st->vdd_supply);
	if (ret) {
		dev_err(dev, "failed to enable regulator\n");
		return ret;
    }
	msleep(INV_ICM42600_POWER_UP_TIME_MS);

	ret = devm_add_action_or_reset(dev, inv_icm42600_disable_vdd_reg, st);
	if (ret) {
		dev_err(dev, "failed to add action reset\n");
		return ret;
    }

	ret = inv_icm42600_enable_regulator_vddio(st);
	if (ret){
		dev_err(dev, "failed to enable vddio\n");
		return ret;
    }

	ret = devm_add_action_or_reset(dev, inv_icm42600_disable_vddio_reg, st);
	if (ret){
		dev_err(dev, "failed to add action reset\n");
		return ret;
    }
#endif // 0
// ===============================================

	/* setup chip registers */
	ret = inv_icm42600_setup(st, bus_setup);
	if (ret) {
		dev_err(dev, "ACHEUL, failed to setup, error: %d\n", ret);
		return ret;
    }

	ret = inv_icm42600_timestamp_setup(st);
	if (ret) {
		dev_err(dev, "ACHEUL, failed to timestamp setup, error: %d\n", ret);
		return ret;
    }

	ret = inv_icm42600_buffer_init(st);
	if (ret) {
		dev_err(dev, "ACHEUL, failed, to buffer init, error: %d\n", ret);
		return ret;
    }

	st->indio_gyro = inv_icm42600_gyro_init(st);
	if (IS_ERR(st->indio_gyro)) {
		return PTR_ERR(st->indio_gyro);
    }

	st->indio_accel = inv_icm42600_accel_init(st);
	if (IS_ERR(st->indio_accel)) {
		dev_err(dev, "ACHEUL, failed, accel init\n");
		return PTR_ERR(st->indio_accel);
    }

	ret = inv_icm42600_irq_init(st, irq, irq_type, open_drain);
	if (ret) {
		dev_err(dev, "ACHEUL, failed, irq initi, %d\n", ret);
		return ret;
    }

	/* setup runtime power management */
	ret = pm_runtime_set_active(dev);
	if (ret) {
		dev_err(dev, "ACHEUL, failed, pm_runtime_set_active, %d\n", ret);
		return ret;
    }

	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, INV_ICM42600_SUSPEND_DELAY_MS);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_put(dev);

	return devm_add_action_or_reset(dev, inv_icm42600_disable_pm, dev);
}
EXPORT_SYMBOL_GPL(inv_icm42600_core_probe);

/*
 * Suspend saves sensors state and turns everything off.
 * Check first if runtime suspend has not already done the job.
 */
static int __maybe_unused inv_icm42600_suspend(struct device *dev)
{

#if USE_VDD
	struct inv_icm42600_state *st = dev_get_drvdata(dev);
	int ret;

    printk("%s:%d ACHEUL, in", __FUNCTION__, __LINE__);

	mutex_lock(&st->lock);

	st->suspended.gyro = st->conf.gyro.mode;
	st->suspended.accel = st->conf.accel.mode;
	st->suspended.temp = st->conf.temp_en;
	if (pm_runtime_suspended(dev)) {
		ret = 0;
		goto out_unlock;
	}

	/* disable FIFO data streaming */
	if (st->fifo.on) {
		ret = regmap_write(st->map, INV_ICM42600_REG_FIFO_CONFIG,
				   INV_ICM42600_FIFO_CONFIG_BYPASS);
		if (ret)
			goto out_unlock;
	}

	ret = inv_icm42600_set_pwr_mgmt0(st, INV_ICM42600_SENSOR_MODE_OFF,
					 INV_ICM42600_SENSOR_MODE_OFF, false,
					 NULL);
	if (ret)
		goto out_unlock;

	regulator_disable(st->vddio_supply);

out_unlock:
	mutex_unlock(&st->lock);
	return ret;
#else
    printk("%s:%d ACHEUL, FIXME! VDD management is disabled", __FUNCTION__, __LINE__);
    return 0;
#endif // USE_VDD

}

/*
 * System resume gets the system back on and restores the sensors state.
 * Manually put runtime power management in system active state.
 */
static int __maybe_unused inv_icm42600_resume(struct device *dev)
{
	struct inv_icm42600_state *st = dev_get_drvdata(dev);
	int ret;

    printk("%s:%d ACHEUL, in", __FUNCTION__, __LINE__);

	mutex_lock(&st->lock);

	ret = inv_icm42600_enable_regulator_vddio(st);
	if (ret)
		goto out_unlock;

	pm_runtime_disable(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	/* restore sensors state */
	ret = inv_icm42600_set_pwr_mgmt0(st, st->suspended.gyro,
					 st->suspended.accel,
					 st->suspended.temp, NULL);
	if (ret)
		goto out_unlock;

	/* restore FIFO data streaming */
	if (st->fifo.on)
		ret = regmap_write(st->map, INV_ICM42600_REG_FIFO_CONFIG,
				   INV_ICM42600_FIFO_CONFIG_STREAM);

out_unlock:
	mutex_unlock(&st->lock);
    printk("%s:%d ACHEUL, out: %d", __FUNCTION__, __LINE__, ret);
	return ret;
}

/* Runtime suspend will turn off sensors that are enabled by iio devices. */
static int __maybe_unused inv_icm42600_runtime_suspend(struct device *dev)
{
	struct inv_icm42600_state *st = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&st->lock);

	/* disable all sensors */
	ret = inv_icm42600_set_pwr_mgmt0(st, INV_ICM42600_SENSOR_MODE_OFF,
					 INV_ICM42600_SENSOR_MODE_OFF, false,
					 NULL);
	if (ret)
		goto error_unlock;

#if USE_VDD
    printk("%s:%d ACHEUL, VDD management is enabled", __FUNCTION__, __LINE__);
	regulator_disable(st->vddio_supply);
#else
    printk("%s:%d ACHEUL, VDD management is disabled", __FUNCTION__, __LINE__);
#endif // USE_VDD

error_unlock:
	mutex_unlock(&st->lock);
	return ret;
}

/* Sensors are enabled by iio devices, no need to turn them back on here. */
static int __maybe_unused inv_icm42600_runtime_resume(struct device *dev)
{
	struct inv_icm42600_state *st = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&st->lock);

#if USE_VDD
    printk("%s:%d ACHEUL, VDD management is enabled", __FUNCTION__, __LINE__);
	ret = inv_icm42600_enable_regulator_vddio(st);
#else
    printk("%s:%d ACHEUL, VDD management is disabled", __FUNCTION__, __LINE__);
#endif // USE_VDD

	mutex_unlock(&st->lock);
	return ret;
}

const struct dev_pm_ops inv_icm42600_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(inv_icm42600_suspend, inv_icm42600_resume)
	SET_RUNTIME_PM_OPS(inv_icm42600_runtime_suspend,
			   inv_icm42600_runtime_resume, NULL)
};
EXPORT_SYMBOL_GPL(inv_icm42600_pm_ops);

MODULE_AUTHOR("ACHEUL & InvenSense, Inc.");
MODULE_DESCRIPTION("InvenSense ICM-426xx device driver");
MODULE_LICENSE("GPL");
