/*
 * Copyright 2016 CZ.NIC, z.s.p.o.
 *
 * Author: Tomas Hlavacek <tmshlvck@gmail.com>
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 *
 * LED driver for the PCA9633 I2C LED driver (7-bit slave address 0x62)
 * LED driver for the PCA9634/5 I2C LED driver (7-bit slave address set by hw.)
 *
 * Note that hardware blinking violates the leds infrastructure driver
 * interface since the hardware only supports blinking all LEDs with the
 * same delay_on/delay_off rates.  That is, only the LEDs that are set to
 * blink will actually blink but all LEDs that are set to blink will blink
 * in identical fashion.  The delay_on/delay_off values of the last LED
 * that is set to blink will be used for all of the blinking LEDs.
 * Hardware blinking is disabled by default but can be enabled by setting
 * the 'blink_type' member in the platform_data struct to 'PCA963X_HW_BLINK'
 * or by adding the 'nxp,hw-blink' property to the DTS.
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

#define N_LEDS 12

struct omnia_led {
	struct omnia_led_mcu *chip;
	struct led_classdev led_cdev;
	int led_num; /* 0 .. 11 */
	char name[32];
	enum led_brightness last_brightness;
	u8 autonomous;
	u8 r;
	u8 g;
	u8 b;
};


#define LED_AUTONOMOUS_ADDR 3
#define LED_ONOFF_ADDR 4
#define LED_COLOR_ADDR 5
#define GLOB_BRIGHTNESS_READ 8
#define GLOB_BRIGHTNESS_WRITE 7

static int omnia_led_brightness_set(struct omnia_led *led,
				enum led_brightness brightness)
{
	int ret;

	mutex_lock(&led->chip->mutex);

	if (led->last_brightness == brightness) {
		mutex_unlock(&led->chip->mutex);
		return 0;
	}

	led->last_brightness = brightness;

	ret = i2c_smbus_write_byte_data(led->chip->client, LED_ONOFF_ADDR,
			(led->led_num | ((brightness != LED_OFF)<<4)));

	mutex_unlock(&led->chip->mutex);
	return ret;
}

static int omnia_led_autonomous_set(struct omnia_led *led, int autonomous)
{
	int ret;

	mutex_lock(&led->chip->mutex);

	if (led->autonomous == (autonomous != 0)) {
		mutex_unlock(&led->chip->mutex);
		return 0;
	}

	led->autonomous = (autonomous != 0);

	ret = i2c_smbus_write_byte_data(led->chip->client, LED_AUTONOMOUS_ADDR,
			(u8)(led->led_num | ((!led->autonomous) << 4)));

	mutex_unlock(&led->chip->mutex);
	return ret;
}

static int omnia_glob_brightness_set(struct omnia_led_mcu *chip,
					int glob_brightness)
{
	int ret;

	mutex_lock(&chip->mutex);

	ret = i2c_smbus_write_byte_data(chip->client, GLOB_BRIGHTNESS_WRITE,
						(u8)glob_brightness);

	mutex_unlock(&chip->mutex);
	return ret;
}

static int omnia_glob_brightness_get(struct omnia_led_mcu *chip)
{
	int ret;

	mutex_lock(&chip->mutex);

	ret = i2c_smbus_read_byte_data(chip->client, GLOB_BRIGHTNESS_READ);

	mutex_unlock(&chip->mutex);
	return ret;
}

static int omnia_led_color_set(struct omnia_led *led, u8 r, u8 g, u8 b)
{
	int ret;
	u8 buf[5];

	buf[0] = LED_COLOR_ADDR;
	buf[1] = led->led_num;
	buf[2] = r;
	buf[3] = g;
	buf[4] = b;

	mutex_lock(&led->chip->mutex);

	ret = i2c_master_send(led->chip->client, buf, 5);

	mutex_unlock(&led->chip->mutex);
	return -(ret<=0);
}




static int omnia_led_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	struct omnia_led *led;

	led = container_of(led_cdev, struct omnia_led, led_cdev);

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
	if (!count || count > N_LEDS)
		return ERR_PTR(-ENODEV);

	leds = devm_kzalloc(&client->dev,
			sizeof(struct led_info) * N_LEDS, GFP_KERNEL);
	if (!leds)
		return ERR_PTR(-ENOMEM);

	for_each_child_of_node(np, child) {
		u32 reg;
		int res;

		res = of_property_read_u32(child, "reg", &reg);
		if ((res != 0) || (reg >= N_LEDS))
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
	pdata->leds.num_leds = N_LEDS;

	return pdata;
}

static ssize_t global_brightness_show(struct device *d,
                struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(d);
	struct omnia_led_mcu *chip = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
				omnia_glob_brightness_get(chip));
}

static ssize_t global_brightness_store(struct device *d,
                struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(d);
        struct omnia_led_mcu *chip = i2c_get_clientdata(client);
	int ret;
	int global_brightness;

	if ((sscanf(buf, "%i", &global_brightness)) != 1)
		return -EINVAL;

	ret = omnia_glob_brightness_set(chip, global_brightness);
	if (ret < 0)
		return ret;

	return count;
}
static DEVICE_ATTR_RW(global_brightness);

static ssize_t autonomous_show(struct device *d,
                struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(d);
	struct omnia_led *led =
			container_of(led_cdev, struct omnia_led, led_cdev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", led->autonomous);
}

static ssize_t autonomous_store(struct device *d,
                struct device_attribute *attr, const char *buf, size_t count)
{
	int ret, autonomous;
	struct led_classdev *led_cdev = dev_get_drvdata(d);
	struct omnia_led *led =
			container_of(led_cdev, struct omnia_led, led_cdev);

	if ((sscanf(buf, "%i", &autonomous)) != 1)
		return -EINVAL;

	ret = omnia_led_autonomous_set(led, autonomous);
	if (ret < 0)
		return ret;

	led->autonomous = autonomous;
	return count;
}
static DEVICE_ATTR_RW(autonomous);

static ssize_t color_show(struct device *d,
                struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(d);
	struct omnia_led *led =
			container_of(led_cdev, struct omnia_led, led_cdev);

	return scnprintf(buf, PAGE_SIZE, "%d %d %d\n", led->r, led->g, led->b);
}

static ssize_t color_store(struct device *d,
                struct device_attribute *attr, const char *buf, size_t count)
{
	int ret, r, g, b;
	struct led_classdev *led_cdev = dev_get_drvdata(d);
	struct omnia_led *led =
			container_of(led_cdev, struct omnia_led, led_cdev);

	if ((sscanf(buf, "%i %i %i", &r, &g, &b)) != 3)
		return -EINVAL;

	ret = omnia_led_color_set(led, r, g, b);
	if (ret < 0)
		return ret;

	led->r = r;
	led->g = g;
	led->b = b;
	return count;
}
static DEVICE_ATTR_RW(color);


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
	leds = devm_kzalloc(&client->dev, N_LEDS * sizeof(*leds),
				GFP_KERNEL);
	if (!leds)
		return -ENOMEM;

	i2c_set_clientdata(client, chip);

	mutex_init(&chip->mutex);
	chip->client = client;
	chip->leds = leds;

	for (i = 0; i < N_LEDS; i++) {
		leds[i].r = 255;
		leds[i].g = 255;
		leds[i].b = 255;
		leds[i].autonomous = 0;
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

		err = device_create_file(leds[i].led_cdev.dev,
						&dev_attr_autonomous);
		if (err < 0) {
			dev_err(leds[i].led_cdev.dev,
				"failed to create attribute autonomous\n");
			goto exit;
		}

		err = device_create_file(leds[i].led_cdev.dev,
						&dev_attr_color);
		if (err < 0) {
			dev_err(leds[i].led_cdev.dev,
				"failed to create attribute color\n");
			goto exit;
		}



		/* Set AUTO for all LEDs by default*/
		omnia_led_autonomous_set(&leds[i], 1);
	}

	err = device_create_file(&client->dev, &dev_attr_global_brightness);
	if (err < 0) {
		dev_err(&client->dev,
			"failed to create attribute global_brightness\n");
		goto exit;
	}

	return 0;

exit:
	device_remove_file(&client->dev, &dev_attr_global_brightness);
	while (i--) {
		device_remove_file(chip->leds[i].led_cdev.dev,
			&dev_attr_color);
		device_remove_file(chip->leds[i].led_cdev.dev,
			&dev_attr_autonomous);

		led_classdev_unregister(&leds[i].led_cdev);
	}

	return err;
}

static int omnia_remove(struct i2c_client *client)
{
	struct omnia_led_mcu *chip = i2c_get_clientdata(client);
	int i;

	device_remove_file(&client->dev, &dev_attr_global_brightness);

	for (i = 0; i < N_LEDS; i++) {
		device_remove_file(chip->leds[i].led_cdev.dev,
			&dev_attr_color);
		device_remove_file(chip->leds[i].led_cdev.dev,
			&dev_attr_autonomous);

		led_classdev_unregister(&chip->leds[i].led_cdev);
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

