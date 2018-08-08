/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Turris Mox module configuration bus driver
 *
 * Copyright (C) 2018 Marek Behun <marek.behun@nic.cz>
 */

#ifndef __LINUX_MFD_MOXTET_H
#define __LINUX_MFD_MOXTET_H

#include <linux/device.h>
#include <linux/mutex.h>

#define TURRIS_MOX_MAX_MODULES	10

enum turris_mox_cpu_module_id {
	TURRIS_MOX_CPU_ID_EMMC	= 0x00,
	TURRIS_MOX_CPU_ID_SD	= 0x10,
};

enum turris_mox_module_id {
	TURRIS_MOX_MODULE_SFP		= 0x01,
	TURRIS_MOX_MODULE_PCI		= 0x02,
	TURRIS_MOX_MODULE_TOPAZ		= 0x03,
	TURRIS_MOX_MODULE_PERIDOT	= 0x04,
	TURRIS_MOX_MODULE_USB3		= 0x05,
};

static inline const char *turris_mox_module_name(unsigned int id)
{
	switch (id) {
	case TURRIS_MOX_MODULE_SFP:
		return "sfp";
	case TURRIS_MOX_MODULE_PCI:
		return "pci";
	case TURRIS_MOX_MODULE_TOPAZ:
		return "topaz";
	case TURRIS_MOX_MODULE_PERIDOT:
		return "peridot";
	case TURRIS_MOX_MODULE_USB3:
		return "usb3";
	default:
		return "unknown";
	}
}

extern struct bus_type moxtet_type;

struct moxtet {
	struct device	*dev;
	struct mutex	lock;
	u8		modules[TURRIS_MOX_MAX_MODULES];
	int		count;
	u8		tx[TURRIS_MOX_MAX_MODULES];
	char		module_topology[128];
};

struct moxtet_driver {
	const enum turris_mox_module_id	*id_table;
	struct device_driver		driver;
};

static inline struct moxtet_driver *
to_moxtet_driver(struct device_driver *drv)
{
	if (!drv)
		return NULL;
	return container_of(drv, struct moxtet_driver, driver);
}

extern int __moxtet_register_driver(struct module *owner,
				    struct moxtet_driver *mdrv);

static inline void moxtet_unregister_driver(struct moxtet_driver *mdrv)
{
	if (mdrv)
		driver_unregister(&mdrv->driver);
}

#define moxtet_register_driver(driver) \
	__moxtet_register_driver(THIS_MODULE, driver)

#define module_moxtet_driver(__moxtet_driver) \
	module_driver(__moxtet_driver, moxtet_register_driver, \
			moxtet_unregister_driver)

struct moxtet_device {
	struct device			dev;
	struct moxtet			*moxtet;
	enum turris_mox_module_id	id;
	unsigned int			idx;
};

extern int moxtet_device_read(struct device *dev);
extern int moxtet_device_write(struct device *dev, u8 val);
extern int moxtet_device_written(struct device *dev);

static inline struct moxtet_device *
to_moxtet_device(struct device *dev)
{
	if (!dev)
		return NULL;
	return container_of(dev, struct moxtet_device, dev);
}

#endif /* __LINUX_MFD_MOXTET_H */
