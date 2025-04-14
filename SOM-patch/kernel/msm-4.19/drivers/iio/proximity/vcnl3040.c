// SPDX-License-Identifier: GPL-2.0-only
/*
 * Support for Vishay VCNL3040 proximity sensor on i2c bus.
 * Based on Vishay VCNL4000 driver code.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/interrupt.h>

#include <linux/iio/iio.h>
#include <linux/iio/events.h>

#define VCNL3040_PROD_ID	0x0186
#define LED_MASK_VALUE      0xF8FF

enum VCNL3040_REGS {
    PS_CONF1_2   = 0x03,
    PS_CONF3_MS  = 0x04,
    PS_CANCEL    = 0x05,
    PS_LOW_THDL  = 0x06,
    PS_HIGH_THDL = 0x07,
    PS_DATA      = 0x08,
    INT_FLAGS    = 0x0B,
    DEVICE_ID    = 0x0C
};

enum VCNL3040_PARAM
{
    ///LED current settings
    LED_CURRENT_50mA = 0x00,
    LED_CURRENT_75mA = 0x01,
    LED_CURRENT_120mA = 0x02,
    LED_CURRENT_140mA = 0x03,
    LED_CURRENT_160mA = 0x04,
    LED_CURRENT_180mA = 0x05,
    LED_CURRENT_200mA = 0x07,

    ///Duty cycle for sensors LED
    PS_DUTY_40 = 0x00,
    PS_DUTY_80 = 0x01,
    PS_DUTY_160 = 0x02,
    PS_DUTY_320 = 0x03,

    ///Measurement integration time 1T0 - IT = 1 -  125us (application note page 5 )
    PS_IT_1T0 = 0x00,
    PS_IT_1T5 = 0x01,
    PS_IT_2T0 = 0x02,
    PS_IT_2T5 = 0x03,
    PS_IT_3T0 = 0x04,
    PS_IT_3T5 = 0x05,
    PS_IT_4T0 = 0x06,
    PS_IT_8T0 = 0x07,

    ///Interupt persistance setting - how many consecutive measruements
    ///have to be made in interupt range to trigger interupt (datasheet page 11)
    PS_PERS_1 = 0x00,
    PS_PERS_2 = 0x01,
    PS_PERS_3 = 0x02,
    PS_PERS_4 = 0x03,

    ///Iterupt modes
    PS_INT_DISABLED = 0x00,
    PS_INT_CLOSING = 0x01,
    PS_INT_AWAY = 0x02,
    PS_INT_CLOSING_AWAY = 0x03,
};


#define VCNL_COMMAND		0x80 /* Command register */
#define VCNL_PROD_REV		0x81 /* Product ID and Revision ID */
#define VCNL_PROXIMITY_RATE	0x82 /* Rate of Proximity Measurement */
#define VCNL_LED_CURRENT	0x83 /* IR LED current for proximity mode */
#define VCNL_PS_RESULT_HI	0x87 /* Proximity result register, MSB */
#define VCNL_PS_RESULT_LO	0x88 /* Proximity result register, LSB */
#define VCNL_PS_ICR		    0x89 /* Interrupt Control Register */
#define VCNL_PS_LO_THR_HI	0x8a /* High byte of low threshold value */
#define VCNL_PS_LO_THR_LO	0x8b /* Low byte of low threshold value */
#define VCNL_PS_HI_THR_HI	0x8c /* High byte of high threshold value */
#define VCNL_PS_HI_THR_LO	0x8d /* Low byte of high threshold value */
#define VCNL_ISR		    0x8e /* Interrupt Status Register */
#define VCNL_PS_MOD_ADJ		0x8f /* Proximity Modulator Timing Adjustment */

/* Bit masks for COMMAND register */
#define VCNL_PS_RDY		BIT(5) /* proximity data ready? */
#define VCNL_PS_OD		BIT(3) /* start on-demand proximity
					* measurement
					*/

/* Enables periodic proximity measurement */
#define VCNL_PS_EN		BIT(1)

/* Enables state machine and LP oscillator for self timed  measurements */
#define VCNL_PS_SELFTIMED_EN	BIT(0)

/* Bit masks for ICR */

/* Enable interrupts on low or high thresholds */
#define  VCNL_ICR_THRES_EN	BIT(1)

/* Bit masks for ISR */
#define VCNL_INT_TH_HI		BIT(0)	/* High threshold hit */
#define VCNL_INT_TH_LOW		BIT(1)	/* Low threshold hit */

#define VCNL_ON_DEMAND_TIMEOUT_US	100000
#define VCNL_POLL_US			20000

static const int vcnl3040_prox_sampling_frequency[][2] = {
	{1, 950000},
	{3, 906250},
	{7, 812500},
	{16, 625000},
	{31, 250000},
	{62, 500000},
	{125, 0},
	{250, 0},
};

/**
 * struct vcnl3040_data - vcnl3040 specific data.
 * @regmap:	device register map.
 * @dev:	vcnl3040 device.
 * @rev:	revision id.
 * @lock:	lock for protecting access to device hardware registers.
 * @buf:	DMA safe __be16 buffer.
 */
struct vcnl3040_data {
	struct regmap *regmap;
	struct device *dev;
    struct i2c_client *client;
	u8 rev;
	struct mutex lock;
	__be16 buf ____cacheline_aligned;
};

/**
 * struct vcnl3040_property - vcnl3040 property.
 * @name:	property name.
 * @reg:	i2c register offset.
 * @conversion_func:	conversion function.
 */
struct vcnl3040_property {
	const char *name;
	u32 reg;
	u32 (*conversion_func)(u32 *val);
};

uint16_t vcnl3040_read_register(struct vcnl3040_data *data, uint8_t reg){

    uint16_t ret;

	ret = i2c_smbus_read_word_data(data->client, reg);

    //printk("%s:%d, ACHEUL reading vcnl3040 reg[0x%02X]: 0x%04X", __FUNCTION__, __LINE__, reg, ret);

    return ret;
}// vcnl3040_read_register

uint16_t vcnl3040_write_register(struct vcnl3040_data *data, uint8_t reg, uint16_t value){

    uint16_t ret;

	ret = i2c_smbus_write_word_data(data->client, reg, value);

    //printk("%s:%d, ACHEUL writing vcnl3040 reg[0x%02X]: 0x%04X, ret: %d", __FUNCTION__, __LINE__, reg, value, ret);

    return ret;
}// vcnl3040_write_register

uint16_t vcnl3040_read_sensor(struct vcnl3040_data *data) {

    uint16_t sensor_value = vcnl3040_read_register(data, PS_DATA);

    printk("%s:%d, ACHEUL, sensor value 0x%04X", __FUNCTION__, __LINE__, sensor_value);

    return sensor_value;
}// vcnl3040_read_sensor

static int vcnl3040_start(struct vcnl3040_data *data) {
    uint16_t current_settings = vcnl3040_read_register(data, PS_CONF1_2);
    uint16_t new_settings     = current_settings & 0xFFFE;

    printk("%s:%d, ACHEUL, starting proximity sensor from: 0x%04X to 0x%04X", __FUNCTION__, __LINE__, current_settings, new_settings);

    vcnl3040_write_register(data, PS_CONF1_2, new_settings);

    return 0;
}// vcnl3040_start

#if 0
static int vcnl3040_stop(struct vcnl3040_data *data) {
    uint16_t current_settings = vcnl3040_read_register(data, PS_CONF3_MS);
    uint16_t new_settings     = current_settings | 0x0001;

    printk("%s:%d, ACHEUL, stopping proximity sensor from: 0x%04X to 0x%04X", __FUNCTION__, __LINE__, current_settings, new_settings);

    vcnl3040_write_register(data, PS_CONF3_MS, new_settings);

    return 0;
}// vcnl3040_stop
#endif // 0

static int vcnl3040_show_config(struct vcnl3040_data *data) {
    uint16_t config;
    uint8_t  i;

    printk("%s:%d, ACHEUL, =============================================", __FUNCTION__, __LINE__);
    for (i = PS_CONF1_2; i <= PS_HIGH_THDL; i++) {

        config = vcnl3040_read_register(data, i);
        printk("%s:%d, ACHEUL, config: 0x%02X: 0x%04X", __FUNCTION__, __LINE__, i, config);
    }
    printk("%s:%d, ACHEUL, =============================================", __FUNCTION__, __LINE__);

    return 0;

}// vcnl3040_show_config

static int vcnl3040_set_led(struct vcnl3040_data *data, uint8_t led_setting) {
    uint16_t current_led_setting = vcnl3040_read_register(data, PS_CONF3_MS) | LED_MASK_VALUE;
    uint16_t new_led_setting     = current_led_setting | (led_setting << 8);

    printk("%s:%d, ACHEUL, setting led from: 0x%04X to 0x%04X", __FUNCTION__, __LINE__, current_led_setting, new_led_setting);

    vcnl3040_write_register(data, PS_CONF3_MS, new_led_setting);

    return 0;
}// vcnl3040_set_led

static int vcnl3040_init(struct vcnl3040_data *data) {
	uint16_t reg;

    printk("%s:%d, ACHEUL", __FUNCTION__, __LINE__);

    reg = vcnl3040_read_register(data, DEVICE_ID);

    if (reg != VCNL3040_PROD_ID) {
        printk("%s:%d, ACHEUL invalid product id: 0x%04X, looking for: 0x%04X", __FUNCTION__, __LINE__, reg, VCNL3040_PROD_ID);
		return -ENODEV;
    }

    vcnl3040_write_register(data, PS_CONF1_2  , 0x0003); // PS shut down 0x0001, PS_IT = (0 : 0 : 1) = 1.5T 0x0002
    vcnl3040_write_register(data, PS_CONF3_MS , 0x0000);
    vcnl3040_write_register(data, PS_CANCEL   , 0x0000);
    vcnl3040_write_register(data, PS_LOW_THDL , 0x0000);
    vcnl3040_write_register(data, PS_HIGH_THDL, 0x0000);

    vcnl3040_set_led(data, LED_CURRENT_200mA);
    vcnl3040_start  (data);

    vcnl3040_show_config(data);
    return 0;
} // vcnl3040_init

static int vcnl3040_read_proxy_samp_freq(struct vcnl3040_data *data, int *val, int *val2)
{
	int rc;
	unsigned int prox_rate;

    printk("%s:%d, ACHEUL", __FUNCTION__, __LINE__);
	rc = regmap_read(data->regmap, VCNL_PROXIMITY_RATE, &prox_rate);
	if (rc)
		return rc;

	if (prox_rate >= ARRAY_SIZE(vcnl3040_prox_sampling_frequency))
		return -EINVAL;

	*val = vcnl3040_prox_sampling_frequency[prox_rate][0];
	*val2 = vcnl3040_prox_sampling_frequency[prox_rate][1];

	return 0;
}

static bool vcnl3040_is_thr_enabled(struct vcnl3040_data *data)
{
	int rc;
	unsigned int icr;

    printk("%s:%d, ACHEUL", __FUNCTION__, __LINE__);
	rc = regmap_read(data->regmap, VCNL_PS_ICR, &icr);
	if (rc) {
		dev_err(data->dev,
			"Error (%d) reading ICR register\n", rc);
		return false;
	}

	return !!(icr & VCNL_ICR_THRES_EN);
}

static int vcnl3040_read_event(struct iio_dev *indio_dev,
			       const struct iio_chan_spec *chan,
			       enum iio_event_type type,
			       enum iio_event_direction dir,
			       enum iio_event_info info,
			       int *val, int *val2)
{
	int rc;
	struct vcnl3040_data *data = iio_priv(indio_dev);

    printk("%s:%d, ACHEUL", __FUNCTION__, __LINE__);
	switch (info) {
	case IIO_EV_INFO_VALUE:
		switch (dir) {
		case IIO_EV_DIR_RISING:
			rc = regmap_bulk_read(data->regmap, VCNL_PS_HI_THR_HI,
					      &data->buf, sizeof(data->buf));
			if (rc < 0)
				return rc;
			*val = be16_to_cpu(data->buf);
			return IIO_VAL_INT;
		case IIO_EV_DIR_FALLING:
			rc = regmap_bulk_read(data->regmap, VCNL_PS_LO_THR_HI,
					      &data->buf, sizeof(data->buf));
			if (rc < 0)
				return rc;
			*val = be16_to_cpu(data->buf);
			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}
	default:
		return -EINVAL;
	}
}

static int vcnl3040_write_event(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan,
				enum iio_event_type type,
				enum iio_event_direction dir,
				enum iio_event_info info,
				int val, int val2)
{
	int rc;
	struct vcnl3040_data *data = iio_priv(indio_dev);

    printk("%s:%d, ACHEUL", __FUNCTION__, __LINE__);
	mutex_lock(&data->lock);

	switch (info) {
	case IIO_EV_INFO_VALUE:
		switch (dir) {
		case IIO_EV_DIR_RISING:
			/* 16 bit word/ low * high */
			data->buf = cpu_to_be16(val);
			rc = regmap_bulk_write(data->regmap, VCNL_PS_HI_THR_HI,
					       &data->buf, sizeof(data->buf));
			if (rc < 0)
				goto err_unlock;
			rc = IIO_VAL_INT;
			goto err_unlock;
		case IIO_EV_DIR_FALLING:
			data->buf = cpu_to_be16(val);
			rc = regmap_bulk_write(data->regmap, VCNL_PS_LO_THR_HI,
					       &data->buf, sizeof(data->buf));
			if (rc < 0)
				goto err_unlock;
			rc = IIO_VAL_INT;
			goto err_unlock;
		default:
			rc = -EINVAL;
			goto err_unlock;
		}
	default:
		rc = -EINVAL;
		goto err_unlock;
	}
err_unlock:
	mutex_unlock(&data->lock);

	return rc;
}

static int vcnl3040_enable_periodic(struct iio_dev *indio_dev,
				    struct vcnl3040_data *data)
{
	int rc;
	int cmd;

    printk("%s:%d, ACHEUL", __FUNCTION__, __LINE__);

	mutex_lock(&data->lock);

	/* Enable periodic measurement of proximity data. */
	cmd = VCNL_PS_EN | VCNL_PS_SELFTIMED_EN;

	rc = regmap_write(data->regmap, VCNL_COMMAND, cmd);
	if (rc) {
		dev_err(data->dev,
			"Error (%d) writing command register\n", rc);
		goto err_unlock;
	}

	/*
	 * Enable interrupts on threshold, for proximity data by
	 * default.
	 */
	rc = regmap_write(data->regmap, VCNL_PS_ICR, VCNL_ICR_THRES_EN);
	if (rc)
		dev_err(data->dev,
			"Error (%d) reading ICR register\n", rc);

err_unlock:
	mutex_unlock(&data->lock);

	return rc;
}

static int vcnl3040_disable_periodic(struct iio_dev *indio_dev,
				     struct vcnl3040_data *data)
{
	int rc;

    printk("%s:%d, ACHEUL", __FUNCTION__, __LINE__);

	mutex_lock(&data->lock);

	rc = regmap_write(data->regmap, VCNL_COMMAND, 0);
	if (rc) {
		dev_err(data->dev,
			"Error (%d) writing command register\n", rc);
		goto err_unlock;
	}

	rc = regmap_write(data->regmap, VCNL_PS_ICR, 0);
	if (rc) {
		dev_err(data->dev,
			"Error (%d) writing ICR register\n", rc);
		goto err_unlock;
	}

	/* Clear interrupt flag bit */
	rc = regmap_write(data->regmap, VCNL_ISR, 0);
	if (rc)
		dev_err(data->dev,
			"Error (%d) writing ISR register\n", rc);

err_unlock:
	mutex_unlock(&data->lock);

	return rc;
}

static int vcnl3040_config_threshold(struct iio_dev *indio_dev, bool state)
{
	struct vcnl3040_data *data = iio_priv(indio_dev);


    printk("%s:%d, ACHEUL", __FUNCTION__, __LINE__);

	if (state) {
		return vcnl3040_enable_periodic(indio_dev, data);
	} else {
		if (!vcnl3040_is_thr_enabled(data))
			return 0;
		return vcnl3040_disable_periodic(indio_dev, data);
	}
}

static int vcnl3040_write_event_config(struct iio_dev *indio_dev,
				       const struct iio_chan_spec *chan,
				       enum iio_event_type type,
				       enum iio_event_direction dir,
				       int state)
{

    printk("%s:%d, ACHEUL", __FUNCTION__, __LINE__);

	switch (chan->type) {
	case IIO_PROXIMITY:
		return vcnl3040_config_threshold(indio_dev, state);
	default:
		return -EINVAL;
	}
}

static int vcnl3040_read_event_config(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan,
				      enum iio_event_type type,
				      enum iio_event_direction dir)
{
	struct vcnl3040_data *data = iio_priv(indio_dev);

	switch (chan->type) {
    printk("%s:%d, ACHEUL", __FUNCTION__, __LINE__);

	case IIO_PROXIMITY:
		return vcnl3040_is_thr_enabled(data);
	default:
		return -EINVAL;
	}
}

static const struct iio_event_spec vcnl3040_event_spec[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE),
	}, {
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE),
	}, {
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_EITHER,
		.mask_separate = BIT(IIO_EV_INFO_ENABLE),
	},
};

static const struct iio_chan_spec vcnl3040_channels[] = {
	{
		.type                         = IIO_PROXIMITY,
		.info_mask_separate           = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.info_mask_separate_available = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.event_spec                   = vcnl3040_event_spec,
		.num_event_specs              = ARRAY_SIZE(vcnl3040_event_spec),
	},
};

static int vcnl3040_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int *val, int *val2, long mask)
{
	int rc;
	struct vcnl3040_data *data = iio_priv(indio_dev);

    printk("%s:%d, ACHEUL", __FUNCTION__, __LINE__);

	switch (mask) {
	    case IIO_CHAN_INFO_RAW:
            *val = (int) vcnl3040_read_sensor(data);
		    return IIO_VAL_INT;
	    case IIO_CHAN_INFO_SAMP_FREQ:
		    rc = vcnl3040_read_proxy_samp_freq(data, val, val2);
		    if (rc < 0)
			    return rc;
		    return IIO_VAL_INT_PLUS_MICRO;
	    default:
		    return -EINVAL;
	}
}// vcnl3040_read_raw

static int vcnl3040_write_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int val1, int val2, long mask)
{
	struct vcnl3040_data *data = iio_priv(indio_dev);
    int                   ret  = 0;

    printk("%s:%d, ACHEUL, val1: 0x%02X, val2: 0x%02X", __FUNCTION__, __LINE__, val1, val2);

    if ((val1 < PS_CONF1_2) || (val1 > PS_HIGH_THDL)){
        printk("%s:%d, ACHEUL, error, invalid register: 0x%02X, supported values: [0x%02X..0x%02X]", __FUNCTION__, __LINE__, val1, PS_CONF1_2, PS_HIGH_THDL);
		return -EINVAL;
    }

    ret = vcnl3040_write_register(data, (uint8_t) val1, (uint16_t) val2);

    vcnl3040_show_config(data);
    return ret;
}// vcnl3040_write_raw

static int vcnl3040_read_avail(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       const int **vals, int *type, int *length,
			       long mask)
{
    printk("%s:%d, ACHEUL", __FUNCTION__, __LINE__);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*vals = (int *)vcnl3040_prox_sampling_frequency;
		*type = IIO_VAL_INT_PLUS_MICRO;
		*length = 2 * ARRAY_SIZE(vcnl3040_prox_sampling_frequency);
		return IIO_AVAIL_LIST;
	default:
		return -EINVAL;
	}
}

static const struct iio_info vcnl3040_info = {
	.read_raw           = vcnl3040_read_raw,
	.write_raw          = vcnl3040_write_raw,
	.read_avail         = vcnl3040_read_avail,
	.read_event_value   = vcnl3040_read_event,
	.write_event_value  = vcnl3040_write_event,
	.read_event_config  = vcnl3040_read_event_config,
	.write_event_config = vcnl3040_write_event_config,
};

static const struct regmap_config vcnl3040_regmap_config = {
	.reg_bits     = 8,
	.val_bits     = 16,
	.max_register = DEVICE_ID
};

static irqreturn_t vcnl3040_handle_irq_thread(int irq, void *p)
{
    struct iio_dev*       indio_dev       = p;
    struct vcnl3040_data* data            = iio_priv(indio_dev);
    uint16_t              int_flags       = vcnl3040_read_register(data, INT_FLAGS);
    bool                  close_proximity = int_flags & 0x0200 ? true : false;
    bool                  away_proximity  = int_flags & 0x0100 ? true : false;

    // FIXME! remove this
    printk("%s:%d, ACHEUL, close_proximity: %d, away_proximity: %d", __FUNCTION__, __LINE__, close_proximity, away_proximity);

    if (close_proximity) {
        iio_push_event(indio_dev, IIO_UNMOD_EVENT_CODE(IIO_PROXIMITY, 1, IIO_EV_TYPE_THRESH, IIO_EV_DIR_RISING), iio_get_time_ns(indio_dev));
    }

    if (away_proximity) {
        iio_push_event(indio_dev, IIO_UNMOD_EVENT_CODE(IIO_PROXIMITY, 1, IIO_EV_TYPE_THRESH, IIO_EV_DIR_FALLING), iio_get_time_ns(indio_dev));
    }

    return IRQ_HANDLED;
}// vcnl3040_handle_irq_thread

static int vcnl3040_probe(struct i2c_client *client)
{
	struct vcnl3040_data *data;
	struct iio_dev *indio_dev;
	struct regmap *regmap;
	int rc;

    printk("%s:%d, ACHEUL", __FUNCTION__, __LINE__);

	regmap = devm_regmap_init_i2c(client, &vcnl3040_regmap_config);
	if (IS_ERR(regmap)) {
        printk("%s:%d, ACHEUL, error: reg map init failed", __FUNCTION__, __LINE__);
		return PTR_ERR(regmap);
	}

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev) {
        printk("%s:%d, ACHEUL, error: failed to allocate", __FUNCTION__, __LINE__);
		return -ENOMEM;
    }

	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->regmap = regmap;
	data->dev    = &client->dev;
    data->client = client;

	rc = vcnl3040_init(data);
	if (rc){
        printk("%s:%d, ACHEUL, error: failed to initialize", __FUNCTION__, __LINE__);
		return rc;
    }

	indio_dev->info         = &vcnl3040_info;
	indio_dev->channels     = vcnl3040_channels;
	indio_dev->num_channels = ARRAY_SIZE(vcnl3040_channels);
	indio_dev->name         = "vcnl3040";
	indio_dev->modes        = INDIO_DIRECT_MODE;

    if (client->irq) {
        rc = devm_request_threaded_irq(&client->dev, client->irq, NULL, vcnl3040_handle_irq_thread, IRQF_ONESHOT, indio_dev->name, indio_dev);
        if (rc) {
            printk("%s:%d, ACHEUL, error: failed to initialize IRQ", __FUNCTION__, __LINE__);
            return rc;
        }
    }

    printk("%s:%d, ACHEUL", __FUNCTION__, __LINE__);
    rc = devm_iio_device_register(&client->dev, indio_dev);
    if (rc) {
        printk("%s:%d, ACHEUL, error: failed to register iio device, error: %d", __FUNCTION__, __LINE__, rc);
        return rc;
    }

    return rc;
}// vcnl3040_probe

static const struct of_device_id vcnl3040_of_match[] = {
	{
		.compatible = "vishay,vcnl3040",
	},
	{}
};

MODULE_DEVICE_TABLE(of, vcnl3040_of_match);

static struct i2c_driver vcnl3040_driver = {
	.driver = {
		.name           = "vishay,vcnl3040",
		.of_match_table = vcnl3040_of_match,
	},
	.probe_new  = vcnl3040_probe,
};
module_i2c_driver(vcnl3040_driver);

MODULE_AUTHOR("Bruno Bauthier <bruno.gauth@gmail.com>");
MODULE_DESCRIPTION("Vishay VCNL3040 proximity sensor driver");
MODULE_LICENSE("GPL");
