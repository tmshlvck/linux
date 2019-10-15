/*
 * Digital I/O driver for Turris Omnia MCU-based GPIOs.
 *
 * Copyright (C) 2019 CZ.NIC,z.s.p.o.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether expressed or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License version 2 for more details.
 */

#include <linux/gpio/driver.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/regmap.h>

#define DEFAULT_PIN_NUMBER	2

struct omnia_gpio_priv {
	struct gpio_chip gpio_chip;
	struct i2c_client *client;
	struct mutex mutex;
	unsigned int state;
};

static int omnia_gpio_get_direction(struct gpio_chip *chip,
				     unsigned int offset)
{
//	struct omnia_gpio_priv *priv = gpiochip_get_data(chip);
	return GPIOF_DIR_OUT;
}

static int omnia_gpio_direction_input(struct gpio_chip *chip,
				       unsigned int offset)
{
//	struct omnia_gpio_priv *priv = gpiochip_get_data(chip);

//	return -ENODEV;
	printk("%s called with offset %d\n", __func__, offset);
	return 0;
}

static int omnia_gpio_direction_output(struct gpio_chip *chip,
					unsigned int offset, int value)
{
//	struct omnia_gpio_priv *priv = gpiochip_get_data(chip);
	printk("%s called with offset %d\n", __func__, offset);
	return 0;
}

static int omnia_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct omnia_gpio_priv *priv = gpiochip_get_data(chip);

	printk("%s called with offset %d returning %d\n", __func__, offset, ((priv->state & (1<<offset))!=0));

	return ((priv->state & (1<<offset))!=0);
}

enum mcu_commands {
        CMD_GET_STATUS_WORD     = 0x01,
        CMD_GENERAL_CONTROL     = 0x02,
        CMD_GET_RESET           = 0x09,
        CMD_WATCHDOG_STATE      = 0x0b,
};

enum general_control_bits {
        USB30_PWRON             = 0x08,
        USB31_PWRON             = 0x10,
};

static void omnia_gpio_set_low(struct gpio_chip *chip, unsigned int offset,
			    int value)
{

        int ret;
        u16 buf;

	printk("%s called with offset %d\n", __func__, offset);

	struct omnia_gpio_priv *priv = gpiochip_get_data(chip);

	buf = ((offset?USB31_PWRON:USB30_PWRON)<<8);
	if (value)
		buf |= (offset?USB31_PWRON:USB30_PWRON);

        printk("omnia-mcu-gpio setting pin %d to value %d by writing 0x%x to to CMD_GENERAL_CONTROL\n", offset, value, buf);
	mutex_lock(&priv->mutex);
	ret = i2c_smbus_write_word_data(priv->client,
				     CMD_GENERAL_CONTROL, buf);
	priv->state = ((priv->state & (~(1<<offset))) | (value<<offset));
	mutex_unlock(&priv->mutex);
        if (ret)
                printk("omnia-mcu-gpio I2C write failed: %i\n", ret);
}

static void omnia_gpio_set(struct gpio_chip *chip, unsigned int offset,
			    int value)
{
	omnia_gpio_set_low(chip, offset, 0);
	mdelay(500);
	omnia_gpio_set_low(chip, offset, value);
}


static const struct gpio_chip omnia_chip = {
	.label			= "omnia-gpio",
	.ngpio			= DEFAULT_PIN_NUMBER,
	.owner			= THIS_MODULE,
	.get_direction		= omnia_gpio_get_direction,
	.direction_input	= omnia_gpio_direction_input,
	.direction_output	= omnia_gpio_direction_output,
	.get			= omnia_gpio_get,
	.set			= omnia_gpio_set,
	.base			= -1,
	.can_sleep		= true,
};

static const struct of_device_id omnia_gpio_of_match_table[] = {
	{ .compatible = "turris-gpio,omnia", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, omnia_gpio_of_match_table);

static int omnia_gpio_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct omnia_gpio_priv *priv;
	int ret;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->gpio_chip = omnia_chip;
	priv->gpio_chip.parent = &client->dev;
	priv->client = client;
	priv->state = (1 | 2); // both enabled by default

	ret = devm_gpiochip_add_data(&client->dev, &priv->gpio_chip, priv);
	if (ret < 0) {
		dev_err(&client->dev, "Unable to register gpiochip\n");
		return ret;
	}

	mutex_init(&priv->mutex);

	i2c_set_clientdata(client, priv);

	return 0;
}

static const struct i2c_device_id omnia_gpio_id_table[] = {
	{ "omnia-gpio", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, omnia_gpio_id_table);

static struct i2c_driver omnia_gpio_driver = {
	.driver = {
		.name = "omnia-gpio",
		.of_match_table = omnia_gpio_of_match_table,
	},
	.probe = omnia_gpio_probe,
	.id_table = omnia_gpio_id_table,
};
module_i2c_driver(omnia_gpio_driver);

MODULE_AUTHOR("CZ.NIC, z.s.p.o.");
MODULE_DESCRIPTION("GPIO interface for Turris Omnia MCU");
MODULE_LICENSE("GPL");
