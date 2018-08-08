// SPDX-License-Identifier: GPL-2.0
/*
 * Turris Mox module configuration bus driver
 *
 * Copyright (C) 2018 Marek Behun <marek.behun@nic.cz>
 */

#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>
#include <linux/mfd/moxtet.h>

static ssize_t
module_id_show(struct device *dev, struct device_attribute *a, char *buf)
{
	struct moxtet_device *mdev = to_moxtet_device(dev);

	return sprintf(buf, "0x%x\n", mdev->id);
}
static DEVICE_ATTR_RO(module_id);

static ssize_t
module_name_show(struct device *dev, struct device_attribute *a, char *buf)
{
	struct moxtet_device *mdev = to_moxtet_device(dev);

	return sprintf(buf, "%s\n", turris_mox_module_name(mdev->id));
}
static DEVICE_ATTR_RO(module_name);

static ssize_t
input_value_show(struct device *dev, struct device_attribute *a, char *buf)
{
	int ret;

	ret = moxtet_device_read(dev);
	if (ret < 0)
		return ret;

	return sprintf(buf, "0x%x\n", ret);
}
static DEVICE_ATTR_RO(input_value);

static ssize_t
output_value_show(struct device *dev, struct device_attribute *a, char *buf)
{
	int ret;

	ret = moxtet_device_written(dev);
	if (ret < 0)
		return ret;

	return sprintf(buf, "0x%x\n", ret);
}

static ssize_t
output_value_store(struct device *dev, struct device_attribute *a,
		   const char *buf, size_t count)
{
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val > 0xff)
		return -ERANGE;

	ret = moxtet_device_write(dev, val);
	if (ret < 0)
		return ret;

	return count;
}
static DEVICE_ATTR_RW(output_value);

static struct attribute *moxtet_dev_attrs[] = {
	&dev_attr_module_id.attr,
	&dev_attr_module_name.attr,
	&dev_attr_input_value.attr,
	&dev_attr_output_value.attr,
	NULL,
};

static const struct attribute_group moxtet_dev_group = {
	.attrs = moxtet_dev_attrs,
};

static const struct attribute_group *moxtet_dev_groups[] = {
	&moxtet_dev_group,
	NULL,
};

static int moxtet_match(struct device *dev, struct device_driver *drv)
{
	struct moxtet_device *mdev = to_moxtet_device(dev);
	struct moxtet_driver *tdrv = to_moxtet_driver(drv);
	const enum turris_mox_module_id *t;

	if (of_driver_match_device(dev, drv))
		return 1;

	if (!tdrv->id_table)
		return 0;

	for (t = tdrv->id_table; *t; ++t)
		if (*t == mdev->id)
			return 1;

	return 0;
}

struct bus_type moxtet_bus_type = {
	.name		= "moxtet",
	.dev_groups	= moxtet_dev_groups,
	.match		= moxtet_match,
};
EXPORT_SYMBOL_GPL(moxtet_bus_type);

int __moxtet_register_driver(struct module *owner,
			     struct moxtet_driver *mdrv)
{
	mdrv->driver.owner = owner;
	mdrv->driver.bus = &moxtet_bus_type;
	return driver_register(&mdrv->driver);
}
EXPORT_SYMBOL_GPL(__moxtet_register_driver);

static int moxtet_dev_check(struct device *dev, void *data)
{
	struct moxtet_device *mdev = to_moxtet_device(dev);
	struct moxtet_device *new_dev = data;

	if (mdev->moxtet == new_dev->moxtet && mdev->id == new_dev->id &&
	    mdev->idx == new_dev->idx)
		return -EBUSY;
	return 0;
}

static void moxtet_dev_release(struct device *dev)
{
	struct moxtet_device *mdev = to_moxtet_device(dev);

	put_device(mdev->moxtet->dev);
	kfree(mdev);
}

static struct moxtet_device *
moxtet_alloc_device(struct moxtet *moxtet)
{
	struct moxtet_device *dev;

	if (!get_device(moxtet->dev))
		return NULL;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		put_device(moxtet->dev);
		return NULL;
	}

	dev->moxtet = moxtet;
	dev->dev.parent = moxtet->dev;
	dev->dev.bus = &moxtet_bus_type;
	dev->dev.release = moxtet_dev_release;

	device_initialize(&dev->dev);

	return dev;
}

static int moxtet_add_device(struct moxtet_device *dev)
{
	static DEFINE_MUTEX(add_mutex);
	int ret;

	if (dev->idx >= TURRIS_MOX_MAX_MODULES || dev->id > 0xf)
		return -EINVAL;

	dev_set_name(&dev->dev, "moxtet-%s.%u",
		     turris_mox_module_name(dev->id), dev->idx);

	mutex_lock(&add_mutex);

	ret = bus_for_each_dev(&moxtet_bus_type, NULL, dev,
			       moxtet_dev_check);
	if (ret)
		goto done;

	ret = device_add(&dev->dev);
	if (ret < 0)
		dev_err(dev->moxtet->dev, "can't add %s, status %d\n",
			dev_name(dev->moxtet->dev), ret);

done:
	mutex_unlock(&add_mutex);
	return ret;
}

static int __unregister(struct device *dev, void *null)
{
	if (dev->of_node) {
		of_node_clear_flag(dev->of_node, OF_POPULATED);
		of_node_put(dev->of_node);
	}

	device_unregister(dev);

	return 0;
}

static struct moxtet_device *
of_register_moxtet_device(struct moxtet *moxtet, struct device_node *nc)
{
	struct moxtet_device *dev;
	u32 val;
	int ret;

	dev = moxtet_alloc_device(moxtet);
	if (!dev) {
		dev_err(moxtet->dev,
			"Moxtet device alloc error for %pOF\n", nc);
		return ERR_PTR(-ENOMEM);
	}

	ret = of_property_read_u32(nc, "reg", &val);
	if (ret || val >= TURRIS_MOX_MAX_MODULES) {
		dev_err(moxtet->dev, "%pOF has no valid 'reg' property (%d)\n",
			nc, ret);
		goto err_put;
	}
	dev->idx = val;

	ret = of_property_read_u32(nc, "moxtet,id", &val);
	if (ret || val > 0xf) {
		dev_err(moxtet->dev,
			"%pOF has no valid 'moxtet,id' property (%d)\n", nc,
			ret);
		goto err_put;
	}
	dev->id = val;

	if (moxtet->modules[dev->idx] != dev->id) {
		dev_err(moxtet->dev,
			"%pOF requested Moxtet device ID 0x%x, 0x%x found\n",
			nc, dev->id, moxtet->modules[dev->idx]);
		goto err_put;
	}

	of_node_get(nc);
	dev->dev.of_node = nc;

	ret = moxtet_add_device(dev);
	if (ret) {
		dev_err(moxtet->dev,
			"Moxtet device register error for %pOF\n", nc);
		of_node_put(nc);
		goto err_put;
	}

	return dev;

err_put:
	put_device(&dev->dev);
	return ERR_PTR(ret);
}

static void of_register_moxtet_devices(struct moxtet *moxtet)
{
	struct moxtet_device *dev;
	struct device_node *nc;

	if (!moxtet->dev->of_node)
		return;

	for_each_available_child_of_node(moxtet->dev->of_node, nc) {
		if (of_node_test_and_set_flag(nc, OF_POPULATED))
			continue;
		dev = of_register_moxtet_device(moxtet, nc);
		if (IS_ERR(dev)) {
			dev_warn(moxtet->dev,
				 "Failed to create Moxtet device for %pOF\n",
				 nc);
			of_node_clear_flag(nc, OF_POPULATED);
		}
	}
}

static void
moxtet_register_devices_from_topology(struct moxtet *moxtet)
{
	struct moxtet_device *dev;
	int i, ret;

	for (i = 0; i < moxtet->count; ++i) {
		dev = moxtet_alloc_device(moxtet);
		if (!dev) {
			dev_err(moxtet->dev, "Moxtet device %u alloc error\n",
				i);
			continue;
		}

		dev->idx = i;
		dev->id = moxtet->modules[i];

		ret = moxtet_add_device(dev);
		if (ret && ret != -EBUSY) {
			put_device(&dev->dev);
			dev_err(moxtet->dev,
				"Moxtet device %u register error: %i\n", i,
				ret);
		}
	}
}

static int moxtet_find_topology(struct moxtet *moxtet)
{
	u8 buf[TURRIS_MOX_MAX_MODULES];
	int i, ret;

	ret = spi_read(to_spi_device(moxtet->dev), buf, TURRIS_MOX_MAX_MODULES);
	if (ret < 0)
		return ret;

	if (buf[0] == TURRIS_MOX_CPU_ID_EMMC) {
		dev_info(moxtet->dev, "Found eMMC Turris Mox CPU module\n");
	} else if (buf[0] == TURRIS_MOX_CPU_ID_SD) {
		dev_info(moxtet->dev, "Found SD Turris Mox CPU module\n");
	} else {
		dev_err(moxtet->dev, "Invalid Turris Mox CPU module 0x%02x\n",
			buf[0]);
		return -ENODEV;
	}

	moxtet->count = 0;

	for (i = 1; i < TURRIS_MOX_MAX_MODULES; ++i) {
		int module_id;

		if (buf[i] == 0xff)
			break;

		module_id = buf[i] & 0xf;

		moxtet->modules[i-1] = module_id;
		++moxtet->count;

		switch (module_id) {
		case TURRIS_MOX_MODULE_SFP:
			dev_info(moxtet->dev, "SFP module found\n");
			break;
		case TURRIS_MOX_MODULE_PCI:
			dev_info(moxtet->dev, "PCIe module found\n");
			break;
		case TURRIS_MOX_MODULE_TOPAZ:
			dev_info(moxtet->dev, "Topaz Switch module found\n");
			break;
		case TURRIS_MOX_MODULE_PERIDOT:
			dev_info(moxtet->dev, "Peridot Switch module found\n");
			break;
		case TURRIS_MOX_MODULE_USB3:
			dev_info(moxtet->dev, "USB 3.0 module found\n");
			break;
		default:
			dev_info(moxtet->dev,
				 "Unknown Moxtet module found (ID 0x%02x)\n",
				 module_id);
		}
	}

	return 0;
}

int moxtet_device_read(struct device *dev)
{
	struct moxtet_device *mdev = to_moxtet_device(dev);
	struct moxtet *moxtet = mdev->moxtet;
	u8 buf[TURRIS_MOX_MAX_MODULES];
	struct spi_transfer xfer = {
		.rx_buf = buf,
		.tx_buf = moxtet->tx,
		.len = moxtet->count + 1
	};
	int ret;

	if (mdev->idx >= moxtet->count)
		return -EINVAL;

	mutex_lock(&moxtet->lock);

	ret = spi_sync_transfer(to_spi_device(moxtet->dev), &xfer, 1);

	mutex_unlock(&moxtet->lock);

	if (ret < 0)
		return ret;

	return buf[mdev->idx + 1] >> 4;
}
EXPORT_SYMBOL_GPL(moxtet_device_read);

int moxtet_device_write(struct device *dev, u8 val)
{
	struct moxtet_device *mdev = to_moxtet_device(dev);
	struct moxtet *moxtet = mdev->moxtet;
	int ret;

	if (mdev->idx >= moxtet->count)
		return -EINVAL;

	mutex_lock(&moxtet->lock);

	moxtet->tx[moxtet->count - mdev->idx] = val;

	ret = spi_write(to_spi_device(moxtet->dev), moxtet->tx,
			moxtet->count + 1);

	mutex_unlock(&moxtet->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(moxtet_device_write);

int moxtet_device_written(struct device *dev)
{
	struct moxtet_device *mdev = to_moxtet_device(dev);
	struct moxtet *moxtet = mdev->moxtet;

	if (mdev->idx >= moxtet->count)
		return -EINVAL;

	return moxtet->tx[moxtet->count - mdev->idx];
}
EXPORT_SYMBOL_GPL(moxtet_device_written);

static int moxtet_probe(struct spi_device *spi)
{
	struct moxtet *moxtet;
	int ret;

	ret = spi_setup(spi);
	if (ret < 0)
		return ret;

	moxtet = devm_kzalloc(&spi->dev, sizeof(struct moxtet),
			      GFP_KERNEL);
	if (!moxtet)
		return -ENOMEM;

	moxtet->dev = &spi->dev;
	spi_set_drvdata(spi, moxtet);

	mutex_init(&moxtet->lock);

	ret = moxtet_find_topology(moxtet);
	if (ret < 0)
		return ret;

	of_register_moxtet_devices(moxtet);
	moxtet_register_devices_from_topology(moxtet);

	return 0;
}

static int moxtet_remove(struct spi_device *spi)
{
	struct moxtet *moxtet = spi_get_drvdata(spi);
	int dummy;

	dummy = device_for_each_child(moxtet->dev, NULL, __unregister);

	mutex_destroy(&moxtet->lock);

	return 0;
}

static const struct of_device_id moxtet_dt_ids[] = {
	{ .compatible = "cznic,moxtet" },
	{},
};
MODULE_DEVICE_TABLE(of, moxtet_dt_ids);

static struct spi_driver moxtet_driver = {
	.driver = {
		.name		= "moxtet",
		.of_match_table = moxtet_dt_ids,
	},
	.probe		= moxtet_probe,
	.remove		= moxtet_remove,
};
module_spi_driver(moxtet_driver);

static int __init moxtet_init(void)
{
	return bus_register(&moxtet_bus_type);
}

postcore_initcall(moxtet_init);

MODULE_AUTHOR("Marek Behun <marek.behun@nic.cz>");
MODULE_DESCRIPTION("CZ.NIC's Turris Mox module configuration bus");
MODULE_LICENSE("GPL v2");
