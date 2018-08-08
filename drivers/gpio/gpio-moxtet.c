// SPDX-License-Identifier: GPL-2.0
/*
 *  Turris Mox Moxtet GPIO expander
 *
 *  Copyright (C) 2018 Marek Behun <marek.behun@nic.cz>
 */

#include <linux/gpio/consumer.h>
#include <linux/gpio.h>
#include <linux/mfd/moxtet.h>
#include <linux/module.h>
#include <linux/of_gpio.h>

struct moxtet_gpio_chip {
	struct device		*dev;
	struct gpio_chip	gpio_chip;
	u8			in_mask;
	u8			out_mask;
};

static int moxtet_gpio_dir_mask(struct gpio_chip *gc, unsigned int offset,
				int *dir, u8 *mask)
{
	struct moxtet_gpio_chip *chip = gpiochip_get_data(gc);
	int i;

	*dir = 0;
	for (i = 0; i < 4; ++i) {
		*mask = 1 << i;
		if (*mask & chip->in_mask) {
			if (offset == 0)
				return 0;
			--offset;
		}
	}

	*dir = 1;
	for (i = 0; i < 8; ++i) {
		*mask = 1 << i;
		if (*mask & chip->out_mask) {
			if (offset == 0)
				return 0;
		}
	}

	return -EINVAL;
}

static int moxtet_gpio_get_value(struct gpio_chip *gc, unsigned int offset)
{
	struct moxtet_gpio_chip *chip = gpiochip_get_data(gc);
	int ret, dir;
	u8 mask;

	if (moxtet_gpio_dir_mask(gc, offset, &dir, &mask) < 0)
		return -EINVAL;

	if (dir)
		ret = moxtet_device_written(chip->dev);
	else
		ret = moxtet_device_read(chip->dev);

	if (ret < 0)
		return ret;

	return (ret & mask) ? 1 : 0;
}

static void moxtet_gpio_set_value(struct gpio_chip *gc, unsigned int offset,
				  int val)
{
	struct moxtet_gpio_chip *chip = gpiochip_get_data(gc);
	int state, dir;
	u8 mask;

	if (moxtet_gpio_dir_mask(gc, offset, &dir, &mask) < 0)
		return;

	/* cannot change input GPIO */
	if (!dir)
		return;

	state = moxtet_device_written(chip->dev);
	if (state < 0)
		return;

	if (val)
		state |= mask;
	else
		state &= ~mask;

	moxtet_device_write(chip->dev, state);
}

static int moxtet_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	int dir;
	u8 mask;

	if (moxtet_gpio_dir_mask(gc, offset, &dir, &mask) < 0)
		return -EINVAL;

	return dir;
}

static int moxtet_gpio_direction_input(struct gpio_chip *gc,
				       unsigned int offset)
{
	int dir;
	u8 mask;

	if (moxtet_gpio_dir_mask(gc, offset, &dir, &mask) < 0)
		return -EINVAL;

	if (dir)
		return -EINVAL;

	return 0;
}

static int moxtet_gpio_direction_output(struct gpio_chip *gc,
					unsigned int offset, int val)
{
	int dir;
	u8 mask;

	if (moxtet_gpio_dir_mask(gc, offset, &dir, &mask) < 0)
		return -EINVAL;

	if (!dir)
		return -EINVAL;

	moxtet_gpio_set_value(gc, offset, val);
	return 0;
}

static int moxtet_gpio_probe(struct device *dev)
{
	struct moxtet_gpio_chip *chip;
	struct device_node *nc = dev->of_node;
	int ret;
	u32 val;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = dev;
	chip->gpio_chip.parent = dev;

	ret = of_property_read_u32(nc, "moxtet,input-mask", &val);
	if (ret < 0 || val > 0xf) {
		dev_err(dev,
			"%pOF has no valid 'moxtet,input-mask' property\n", nc);
		return ret < 0 ? ret : -ERANGE;
	}
	chip->in_mask = val;

	ret = of_property_read_u32(nc, "moxtet,output-mask", &val);
	if (ret < 0 || val > 0xff) {
		dev_err(dev,
			"%pOF has no valid 'moxtet,output-mask' property\n",
			nc);
		return ret < 0 ? ret : -ERANGE;
	}
	chip->out_mask = val;

	if (!chip->in_mask && !chip->out_mask) {
		dev_err(dev, "%pOF has zero GPIOs defined\n", nc);
		return -EINVAL;
	}

	dev_set_drvdata(dev, chip);

	chip->gpio_chip.label = dev_name(dev);
	chip->gpio_chip.get_direction = moxtet_gpio_get_direction;
	chip->gpio_chip.direction_input = moxtet_gpio_direction_input;
	chip->gpio_chip.direction_output = moxtet_gpio_direction_output;
	chip->gpio_chip.get = moxtet_gpio_get_value;
	chip->gpio_chip.set = moxtet_gpio_set_value;
	chip->gpio_chip.base = -1;

	chip->gpio_chip.ngpio = hweight8(chip->in_mask) +
				hweight8(chip->out_mask);

	chip->gpio_chip.can_sleep = true;
	chip->gpio_chip.owner = THIS_MODULE;

	return devm_gpiochip_add_data(dev, &chip->gpio_chip, chip);
}

static const struct of_device_id moxtet_gpio_dt_ids[] = {
	{ .compatible = "cznic,moxtet-gpio" },
	{},
};
MODULE_DEVICE_TABLE(of, moxtet_gpio_dt_ids);

static struct moxtet_driver moxtet_gpio_driver = {
	.driver = {
		.name		= "moxtet-gpio",
		.of_match_table	= moxtet_gpio_dt_ids,
		.probe		= moxtet_gpio_probe,
	},
};
module_moxtet_driver(moxtet_gpio_driver);

MODULE_AUTHOR("Marek Behun <marek.behun@nic.cz>");
MODULE_DESCRIPTION("Turris Mox Moxtet GPIO expander");
MODULE_LICENSE("GPL v2");
