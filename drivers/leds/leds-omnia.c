/*
 * Copyright 2016 CZ.NIC, z.s.p.o.
 *
 * Author: Tomas Hlavacek <tmshlvck@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation. 
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/of.h>

#define MAX_LEDS 13
#define ALL_LEDS_INDEX 12

#define LED_AUTONOMOUS_ADDR 3
#define LED_ONOFF_ADDR 4



struct omnia_platform_data {
        struct led_platform_data leds;
};

static const struct i2c_device_id omnia_id[] = {
	{ "omnia", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, omnia_id);

struct omnia_led_mcu {
	struct mutex mutex;
	struct i2c_client *client;
	struct omnia_led *leds;
};

struct omnia_led {
	struct omnia_led_mcu *chip;
	struct led_classdev led_cdev;
	int led_num; /* 0 .. 11 + 12=ALL */
	char name[32];
	u8 autonomous;
};

static int omnia_led_brightness_set(struct omnia_led *led,
				enum led_brightness brightness)
{
	int ret;

	mutex_lock(&led->chip->mutex);

	ret = i2c_smbus_write_byte_data(led->chip->client, LED_ONOFF_ADDR,
			(led->led_num | ((brightness != LED_OFF)<<4)));

	mutex_unlock(&led->chip->mutex);
	return ret;
}

static int omnia_led_autonomous_set(struct omnia_led *led, int autonomous)
{
	int ret, i;

	mutex_lock(&led->chip->mutex);

	if (led->autonomous == (autonomous != 0)) {
		mutex_unlock(&led->chip->mutex);
		return 0;
	}

	led->autonomous = (autonomous != 0);

	if (led->led_num == ALL_LEDS_INDEX) {
		for (i=0; i<(MAX_LEDS-1); i++)
			led->chip->leds[i].autonomous = led->autonomous;
	}

	ret = i2c_smbus_write_byte_data(led->chip->client, LED_AUTONOMOUS_ADDR,
			(u8)(led->led_num | ((!led->autonomous) << 4)));

	mutex_unlock(&led->chip->mutex);
	return ret;
}

static int omnia_led_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	struct omnia_led *led;
	int ret;

	led = container_of(led_cdev, struct omnia_led, led_cdev);

	if (led->autonomous) {
		ret = omnia_led_autonomous_set(led, 0);
		if (ret < 0)
			return ret;
	}

	return omnia_led_brightness_set(led, value);
}

static struct omnia_platform_data *
omnia_dt_init(struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node, *child;
	struct omnia_platform_data *pdata;
	struct led_info *leds;
	int count;

	count = of_get_child_count(np);
	if (!count || count > MAX_LEDS)
		return ERR_PTR(-ENODEV);

	leds = devm_kzalloc(&client->dev,
			sizeof(struct led_info) * MAX_LEDS, GFP_KERNEL);
	if (!leds)
		return ERR_PTR(-ENOMEM);

	for_each_child_of_node(np, child) {
		u32 reg;
		int res;

		res = of_property_read_u32(child, "reg", &reg);
		if ((res != 0) || (reg >= MAX_LEDS))
			continue;
		leds[reg].name =
			of_get_property(child, "label", NULL) ? : child->name;
		leds[reg].default_trigger =
			of_get_property(child, "linux,default-trigger", NULL);
	}
	pdata = devm_kzalloc(&client->dev,
			     sizeof(struct omnia_platform_data), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->leds.leds = leds;
	pdata->leds.num_leds = MAX_LEDS;

	return pdata;
}

static const struct of_device_id of_omnia_match[] = {
	{ .compatible = "turris-leds,omnia", },
	{},
};
MODULE_DEVICE_TABLE(of, of_omnia_match);

static int omnia_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct omnia_led_mcu *chip;
	struct omnia_led *leds;
	struct omnia_platform_data *pdata;
	int i, err;

	pdata = dev_get_platdata(&client->dev);

	if (!pdata) {
		pdata = omnia_dt_init(client);
		if (IS_ERR(pdata)) {
			dev_warn(&client->dev, "could not parse configuration\n");
			pdata = NULL;
		}
	}

	chip = devm_kzalloc(&client->dev, sizeof(*chip),
				GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	leds = devm_kzalloc(&client->dev, MAX_LEDS * sizeof(*leds),
				GFP_KERNEL);
	if (!leds)
		return -ENOMEM;

	i2c_set_clientdata(client, chip);

	mutex_init(&chip->mutex);
	chip->client = client;
	chip->leds = leds;

	for (i = 0; i < MAX_LEDS; i++) {
		leds[i].led_num = i;
		leds[i].chip = chip;

		/* Platform data can specify LED names and default triggers */
		if (pdata && i < pdata->leds.num_leds) {
			if (pdata->leds.leds[i].name)
				snprintf(leds[i].name,
					 sizeof(leds[i].name), "omnia-led:%s",
					 pdata->leds.leds[i].name);
			if (pdata->leds.leds[i].default_trigger)
				leds[i].led_cdev.default_trigger =
					pdata->leds.leds[i].default_trigger;
		}
		if (!pdata || i >= pdata->leds.num_leds ||
						!pdata->leds.leds[i].name)
			snprintf(leds[i].name, sizeof(leds[i].name),
				 "omnia-led:%d", i);

		leds[i].led_cdev.name = leds[i].name;
		leds[i].led_cdev.brightness_set_blocking = omnia_led_set;

		err = led_classdev_register(&client->dev, &leds[i].led_cdev);
		if (err < 0)
			goto exit;

		/* Set AUTO for all LEDs by default */
		leds[i].autonomous = 0;
		omnia_led_autonomous_set(&leds[i], 1);
	}

	return 0;

exit:
	while (i--) {
		led_classdev_unregister(&leds[i].led_cdev);
	}

	return err;
}

static int omnia_remove(struct i2c_client *client)
{
	struct omnia_led_mcu *chip = i2c_get_clientdata(client);
	int i;

	for (i = 0; i < MAX_LEDS; i++) {
		led_classdev_unregister(&chip->leds[i].led_cdev);

		/* Set AUTO for the LED */
		omnia_led_autonomous_set(&chip->leds[i], 1);
	}

	return 0;
}

static struct i2c_driver omnia_driver = {
	.driver = {
		.name	= "leds-omnia",
		.of_match_table = of_match_ptr(of_omnia_match),
	},
	.probe	= omnia_probe,
	.remove	= omnia_remove,
	.id_table = omnia_id,
};

module_i2c_driver(omnia_driver);

MODULE_AUTHOR("Tomas Hlavacek <tmshlvck@gmail.com>");
MODULE_DESCRIPTION("Turris Omnia LED driver");
MODULE_LICENSE("GPL v2");

