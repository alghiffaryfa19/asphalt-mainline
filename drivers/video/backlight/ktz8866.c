// SPDX-License-Identifier: GPL-2.0-only
/*
 * Backlight driver for the Kinetic KTZ8866
 *
 * Copyright (C) 2022, 2023 Jianhua Lu <lujianhua000@gmail.com>
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#define DEFAULT_BRIGHTNESS 1500
#define MAX_BRIGHTNESS 2047
#define REG_MAX 0x15

/* reg */
#define DEVICE_ID 0x01
#define BL_CFG1 0x02
#define BL_CFG2 0x03
#define BL_BRT_LSB 0x04
#define BL_BRT_MSB 0x05
#define BL_EN 0x08
#define LCD_BIAS_CFG1 0x09
#define LCD_BIAS_CFG2 0x0A
#define LCD_BIAS_CFG3 0x0B
#define LCD_BOOST_CFG 0x0C
#define OUTP_CFG 0x0D
#define OUTN_CFG 0x0E
#define FLAG 0x0F
#define BL_OPTION1 0x10
#define BL_OPTION2 0x11
#define PWM2DIG_LSBs 0x12
#define PWM2DIG_MSBs 0x13
#define BL_DIMMING 0x14
#define PWM_RAMP_TIME 0x15

/* definition */
#define BL_EN_BIT BIT(6)
#define LCD_BIAS_EN 0x9F
#define PWM_HYST 0x5
#define DEVICE_ID_VENDOR_MASK GENMASK(5, 3)
#define DEVICE_ID_VENDOR_KINETIC BIT(5)

struct ktz8866_chip_data {
	const struct reg_sequence *init_sequence;
	size_t init_sequence_size;
	unsigned int default_brightness;
};

struct ktz8866 {
	struct i2c_client *client;
	const struct ktz8866_chip_data *chip_data;
	struct regmap *regmap;
	struct regulator *vddpos;
	struct regulator *vddneg;
	bool led_on;
	bool powered;
	struct gpio_desc *enable_gpio;
};

static const struct regmap_config ktz8866_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = REG_MAX,
};

static const struct reg_sequence y700_init_sequence[] = {
	/* Keep the LEDs off while replacing their POR brightness of 2047. */
	{ BL_EN, 0x0f },
	{ BL_BRT_LSB, 0x00 },
	{ BL_BRT_MSB, 0x00 },
	{ BL_CFG1, 0x5a },
	{ BL_CFG2, 0xcd },
	{ BL_OPTION2, 0xb7 },
	{ PWM_RAMP_TIME, 0xa8 },
	{ LCD_BOOST_CFG, 0x26 },
	{ OUTP_CFG, 0x1e },
	{ OUTN_CFG, 0x1e },
	{ LCD_BIAS_CFG1, 0x9f },
};

static const struct ktz8866_chip_data y700_chip_data = {
	.init_sequence = y700_init_sequence,
	.init_sequence_size = ARRAY_SIZE(y700_init_sequence),
	.default_brightness = 0,
};

static int ktz8866_write(struct ktz8866 *ktz, unsigned int reg,
			 unsigned int val)
{
	return regmap_write(ktz->regmap, reg, val);
}

static int ktz8866_update_bits(struct ktz8866 *ktz, unsigned int reg,
			       unsigned int mask, unsigned int val)
{
	return regmap_update_bits(ktz->regmap, reg, mask, val);
}

static int ktz8866_backlight_update_status(struct backlight_device *backlight_dev)
{
	struct ktz8866 *ktz = bl_get_data(backlight_dev);
	unsigned int brightness = backlight_get_brightness(backlight_dev);
	int ret;

	if (brightness == 0) {
		ret = ktz8866_update_bits(ktz, BL_EN, BL_EN_BIT, 0);
		if (ret)
			return ret;

		ktz->led_on = false;
	} else {
		/* Replace the old brightness before allowing current to flow. */
		ret = ktz8866_write(ktz, BL_BRT_LSB, brightness & 0x7);
		if (ret)
			return ret;
		ret = ktz8866_write(ktz, BL_BRT_MSB, (brightness >> 3) & 0xff);
		if (ret)
			return ret;
		ret = ktz8866_update_bits(ktz, BL_EN, BL_EN_BIT, BL_EN_BIT);
		if (ret)
			return ret;

		ktz->led_on = true;
	}

	if (brightness == 0) {
		ret = ktz8866_write(ktz, BL_BRT_LSB, 0);
		if (ret)
			return ret;
		ret = ktz8866_write(ktz, BL_BRT_MSB, 0);
		if (ret)
			return ret;
	}

	return 0;
}

static const struct backlight_ops ktz8866_backlight_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = ktz8866_backlight_update_status,
};

static int ktz8866_init(struct ktz8866 *ktz)
{
	unsigned int val = 0;
	int ret;

	if (ktz->chip_data) {
		ret = regmap_read(ktz->regmap, DEVICE_ID, &val);
		if (ret)
			return ret;
		if ((val & DEVICE_ID_VENDOR_MASK) != DEVICE_ID_VENDOR_KINETIC)
			return -ENODEV;

		return regmap_multi_reg_write(ktz->regmap,
					      ktz->chip_data->init_sequence,
					      ktz->chip_data->init_sequence_size);
	}

	if (!of_property_read_u32(ktz->client->dev.of_node, "current-num-sinks", &val))
		ret = ktz8866_write(ktz, BL_EN, BIT(val) - 1);
	else
		/* Enable all 6 current sinks if the number of current sinks isn't specified. */
		ret = ktz8866_write(ktz, BL_EN, BIT(6) - 1);
	if (ret)
		return ret;

	if (!of_property_read_u32(ktz->client->dev.of_node, "kinetic,current-ramp-delay-ms", &val)) {
		if (val <= 128)
			ret = ktz8866_write(ktz, BL_CFG2,
					    BIT(7) | (ilog2(val) << 3) | PWM_HYST);
		else
			ret = ktz8866_write(ktz, BL_CFG2,
					    BIT(7) | ((5 + val / 64) << 3) | PWM_HYST);
		if (ret)
			return ret;
	}

	if (!of_property_read_u32(ktz->client->dev.of_node, "kinetic,led-enable-ramp-delay-ms", &val)) {
		if (val == 0)
			ret = ktz8866_write(ktz, BL_DIMMING, 0);
		else {
			unsigned int ramp_off_time = ilog2(val) + 1;
			unsigned int ramp_on_time = ramp_off_time << 4;
			ret = ktz8866_write(ktz, BL_DIMMING,
					    ramp_on_time | ramp_off_time);
		}
		if (ret)
			return ret;
	}

	if (of_property_read_bool(ktz->client->dev.of_node, "kinetic,enable-lcd-bias")) {
		ret = ktz8866_write(ktz, LCD_BIAS_CFG1, LCD_BIAS_EN);
		if (ret)
			return ret;
	}

	return 0;
}

static void ktz8866_y700_power_off(struct ktz8866 *ktz)
{
	if (!ktz->powered)
		return;

	/* Best effort: hardware disable below remains authoritative on I2C failure. */
	ktz8866_update_bits(ktz, BL_EN, BL_EN_BIT, 0);
	ktz8866_write(ktz, BL_BRT_LSB, 0);
	ktz8866_write(ktz, BL_BRT_MSB, 0);
	ktz8866_write(ktz, LCD_BIAS_CFG1, 0);
	regulator_disable(ktz->vddneg);
	regulator_disable(ktz->vddpos);
	gpiod_set_value_cansleep(ktz->enable_gpio, 0);
	ktz->led_on = false;
	ktz->powered = false;
}

static int ktz8866_y700_power_on(struct ktz8866 *ktz)
{
	int ret;

	gpiod_set_value_cansleep(ktz->enable_gpio, 1);
	usleep_range(3500, 4000);

	ret = regulator_enable(ktz->vddpos);
	if (ret)
		goto disable_hwen;

	ret = regulator_enable(ktz->vddneg);
	if (ret)
		goto disable_vddpos;

	ktz->powered = true;
	return 0;

disable_vddpos:
	regulator_disable(ktz->vddpos);
disable_hwen:
	gpiod_set_value_cansleep(ktz->enable_gpio, 0);
	return ret;
}

static int ktz8866_probe(struct i2c_client *client)
{
	struct backlight_device *backlight_dev;
	struct backlight_properties props;
	struct ktz8866 *ktz;
	int ret = 0;

	ktz = devm_kzalloc(&client->dev, sizeof(*ktz), GFP_KERNEL);
	if (!ktz)
		return -ENOMEM;

	ktz->client = client;
	ktz->chip_data = device_get_match_data(&client->dev);
	ktz->regmap = devm_regmap_init_i2c(client, &ktz8866_regmap_config);
	if (IS_ERR(ktz->regmap))
		return dev_err_probe(&client->dev, PTR_ERR(ktz->regmap), "failed to init regmap\n");

	if (ktz->chip_data) {
		ktz->enable_gpio = devm_gpiod_get(&client->dev, "enable", GPIOD_OUT_LOW);
		if (IS_ERR(ktz->enable_gpio))
			return PTR_ERR(ktz->enable_gpio);

		ktz->vddpos = devm_regulator_get(&client->dev, "vddpos");
		if (IS_ERR(ktz->vddpos))
			return dev_err_probe(&client->dev, PTR_ERR(ktz->vddpos),
					     "get regulator vddpos failed\n");
		ktz->vddneg = devm_regulator_get(&client->dev, "vddneg");
		if (IS_ERR(ktz->vddneg))
			return dev_err_probe(&client->dev, PTR_ERR(ktz->vddneg),
					     "get regulator vddneg failed\n");

		ret = ktz8866_y700_power_on(ktz);
		if (ret)
			return dev_err_probe(&client->dev, ret,
					     "failed to power on Y700 backlight\n");
	} else {
		ret = devm_regulator_get_enable(&client->dev, "vddpos");
		if (ret)
			return dev_err_probe(&client->dev, ret,
					     "get regulator vddpos failed\n");
		ret = devm_regulator_get_enable(&client->dev, "vddneg");
		if (ret)
			return dev_err_probe(&client->dev, ret,
					     "get regulator vddneg failed\n");

		ktz->enable_gpio = devm_gpiod_get_optional(&client->dev, "enable",
						       GPIOD_OUT_HIGH);
		if (IS_ERR(ktz->enable_gpio))
			return PTR_ERR(ktz->enable_gpio);
	}

	ret = ktz8866_init(ktz);
	if (ret) {
		dev_err_probe(&client->dev, ret, "failed to initialize backlight\n");
		goto err_power_off;
	}

	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = MAX_BRIGHTNESS;
	props.brightness = ktz->chip_data ? ktz->chip_data->default_brightness :
					       DEFAULT_BRIGHTNESS;
	props.scale = BACKLIGHT_SCALE_LINEAR;

	backlight_dev = devm_backlight_device_register(&client->dev, "ktz8866-backlight",
					&client->dev, ktz, &ktz8866_backlight_ops, &props);
	if (IS_ERR(backlight_dev))
		goto err_register;

	i2c_set_clientdata(client, backlight_dev);
	ret = backlight_update_status(backlight_dev);
	if (ret) {
		dev_err_probe(&client->dev, ret, "failed to set initial brightness\n");
		goto err_power_off;
	}

	return 0;

err_register:
	ret = dev_err_probe(&client->dev, PTR_ERR(backlight_dev),
			    "failed to register backlight device\n");
err_power_off:
	if (ktz->chip_data)
		ktz8866_y700_power_off(ktz);
	return ret;
}

static void ktz8866_remove(struct i2c_client *client)
{
	struct backlight_device *backlight_dev = i2c_get_clientdata(client);
	struct ktz8866 *ktz = bl_get_data(backlight_dev);

	backlight_dev->props.brightness = 0;
	backlight_update_status(backlight_dev);
	if (ktz->chip_data)
		ktz8866_y700_power_off(ktz);
}

static void ktz8866_shutdown(struct i2c_client *client)
{
	ktz8866_remove(client);
}

static const struct i2c_device_id ktz8866_ids[] = {
	{ .name = "ktz8866" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ktz8866_ids);

static const struct of_device_id ktz8866_match_table[] = {
	{
		.compatible = "lenovo,y700-ktz8866",
		.data = &y700_chip_data,
	},
	{
		.compatible = "kinetic,ktz8866",
	},
	{},
};
MODULE_DEVICE_TABLE(of, ktz8866_match_table);

static struct i2c_driver ktz8866_driver = {
	.driver = {
		.name = "ktz8866",
		.of_match_table = ktz8866_match_table,
	},
	.probe = ktz8866_probe,
	.remove = ktz8866_remove,
	.shutdown = ktz8866_shutdown,
	.id_table = ktz8866_ids,
};

module_i2c_driver(ktz8866_driver);

MODULE_DESCRIPTION("Kinetic KTZ8866 Backlight Driver");
MODULE_AUTHOR("Jianhua Lu <lujianhua000@gmail.com>");
MODULE_LICENSE("GPL");
