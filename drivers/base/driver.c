// SPDX-License-Identifier: GPL-2.0
/*
 * driver.c - centralized device driver management
 *
 * Copyright (c) 2002-3 Patrick Mochel
 * Copyright (c) 2002-3 Open Source Development Labs
 * Copyright (c) 2007 Greg Kroah-Hartman <gregkh@suse.de>
 * Copyright (c) 2007 Novell Inc.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include "base.h"

static struct device *next_device(struct klist_iter *i)
{
	struct klist_node *n = klist_next(i);
	struct device *dev = NULL;
	struct device_private *dev_prv;

	if (n) {
		dev_prv = to_device_private_driver(n);
		dev = dev_prv->device;
	}
	return dev;
}

/**
 * driver_for_each_device - Iterator for devices bound to a driver.
 * @drv: Driver we're iterating.
 * @start: Device to begin with
 * @data: Data to pass to the callback.
 * @fn: Function to call for each device.
 *
 * Iterate over the @drv's list of devices calling @fn for each one.
 */
int driver_for_each_device(struct device_driver *drv, struct device *start,
			   void *data, int (*fn)(struct device *, void *))
{
	struct klist_iter i;
	struct device *dev;
	int error = 0;

	if (!drv)
		return -EINVAL;

	klist_iter_init_node(&drv->p->klist_devices, &i,
			     start ? &start->p->knode_driver : NULL);
	while (!error && (dev = next_device(&i)))
		error = fn(dev, data);
	klist_iter_exit(&i);
	return error;
}
EXPORT_SYMBOL_GPL(driver_for_each_device);

/**
 * driver_find_device - device iterator for locating a particular device.
 * @drv: The device's driver
 * @start: Device to begin with
 * @data: Data to pass to match function
 * @match: Callback function to check device
 *
 * This is similar to the driver_for_each_device() function above, but
 * it returns a reference to a device that is 'found' for later use, as
 * determined by the @match callback.
 *
 * The callback should return 0 if the device doesn't match and non-zero
 * if it does.  If the callback returns non-zero, this function will
 * return to the caller and not iterate over any more devices.
 */
struct device *driver_find_device(struct device_driver *drv,
				  struct device *start, const void *data,
				  int (*match)(struct device *dev, const void *data))
{
	struct klist_iter i;
	struct device *dev;

	if (!drv || !drv->p)
		return NULL;

	klist_iter_init_node(&drv->p->klist_devices, &i,
			     (start ? &start->p->knode_driver : NULL));
	while ((dev = next_device(&i)))
		if (match(dev, data) && get_device(dev))
			break;
	klist_iter_exit(&i);
	return dev;
}
EXPORT_SYMBOL_GPL(driver_find_device);

/**
 * driver_create_file - create sysfs file for driver.
 * @drv: driver.
 * @attr: driver attribute descriptor.
 */
/**
* driver_create_file -为驱动程序创建sysfs文件。
* @drv:司机。
* @attr:驱动程序属性描述符。	
*/
int driver_create_file(struct device_driver *drv,
		       const struct driver_attribute *attr)
{
	int error;

	if (drv)
		error = sysfs_create_file(&drv->p->kobj, &attr->attr);
	else
		error = -EINVAL;
	return error;
}
EXPORT_SYMBOL_GPL(driver_create_file);

/**
 * driver_remove_file - remove sysfs file for driver.
 * @drv: driver.
 * @attr: driver attribute descriptor.
 */
void driver_remove_file(struct device_driver *drv,
			const struct driver_attribute *attr)
{
	if (drv)
		sysfs_remove_file(&drv->p->kobj, &attr->attr);
}
EXPORT_SYMBOL_GPL(driver_remove_file);

// 如果grop不为空的话，将在驱动文件夹下创建以group名字的子文件夹，然后在子文件夹下添加group的属性文件
int driver_add_groups(struct device_driver *drv,
		      const struct attribute_group **groups)
{
	return sysfs_create_groups(&drv->p->kobj, groups);
}

void driver_remove_groups(struct device_driver *drv,
			  const struct attribute_group **groups)
{
	sysfs_remove_groups(&drv->p->kobj, groups);
}

/**
 * driver_register - register driver with bus
 * @drv: driver to register
 *
 * We pass off most of the work to the bus_add_driver() call,
 * since most of the things we have to do deal with the bus
 * structures.
 */
/**
* driver_register—使用总线注册驱动程序
* @drv:需要注册的驱动程序
*
* 我们将大部分工作传递给bus_add_driver()调用，因为我们必须处理总线结构的大部分事情。
*/
int driver_register(struct device_driver *drv)
{
	int ret;
	struct device_driver *other;

	// 检查总线上的私有数据结构是否分配，没有就退出。
	if (!drv->bus->p) {
		pr_err("Driver '%s' was unable to register with bus_type '%s' because the bus was not initialized.\n",
			   drv->name, drv->bus->name);
		return -EINVAL;
	}

	if ((drv->bus->probe && drv->probe) ||
	    (drv->bus->remove && drv->remove) ||
	    (drv->bus->shutdown && drv->shutdown))
		printk(KERN_WARNING "Driver '%s' needs updating - please use "
			"bus_type methods\n", drv->name);
	// 通过它的名字来定位总线上的驱动程序。如果定位到，说明本驱动已经注册，就退出。
	other = driver_find(drv->name, drv->bus);
	if (other) {
		printk(KERN_ERR "Error: Driver '%s' is already registered, "
			"aborting...\n", drv->name);
		return -EBUSY;
	}

	// 将本驱动添加到总线，添加失败，就退出。
	ret = bus_add_driver(drv);
	if (ret)
		return ret;
	// 如果grop不为空的话，将在驱动文件夹下创建以group名字的子文件夹，然后在子文件夹下添加group的属性文件
	ret = driver_add_groups(drv, drv->groups);
	if (ret) {
		bus_remove_driver(drv);
		return ret;
	}
	// 通过发送一个uevent来通知用户空间
	kobject_uevent(&drv->p->kobj, KOBJ_ADD);

	return ret;
}
EXPORT_SYMBOL_GPL(driver_register);

/**
 * driver_unregister - remove driver from system.
 * @drv: driver.
 *
 * Again, we pass off most of the work to the bus-level call.
 */
void driver_unregister(struct device_driver *drv)
{
	if (!drv || !drv->p) {
		WARN(1, "Unexpected driver unregister!\n");
		return;
	}
	driver_remove_groups(drv, drv->groups);
	bus_remove_driver(drv);
}
EXPORT_SYMBOL_GPL(driver_unregister);

/**
 * driver_find - locate driver on a bus by its name.
 * @name: name of the driver.
 * @bus: bus to scan for the driver.
 *
 * Call kset_find_obj() to iterate over list of drivers on
 * a bus to find driver by name. Return driver if found.
 *
 * This routine provides no locking to prevent the driver it returns
 * from being unregistered or unloaded while the caller is using it.
 * The caller is responsible for preventing this.
 */
/**
* driver_find -通过它的名字来定位总线上的驱动程序。
* @name:司机的名字。
* @bus:扫描司机的总线。
*
* 调用kset_find_obj()遍历总线上的驱动程序列表，按名称查找驱动程序。如果找到返回驱动程序。
* 这个例程没有提供锁来防止它返回的驱动程序在调用者使用它时被取消注册或卸载。调用者负责防止这种情况发生。
*/
struct device_driver *driver_find(const char *name, struct bus_type *bus)
{
	struct kobject *k = kset_find_obj(bus->p->drivers_kset, name);
	struct driver_private *priv;

	if (k) {
		/* Drop reference added by kset_find_obj() */
		kobject_put(k);
		priv = to_driver(k);
		return priv->driver;
	}
	return NULL;
}
EXPORT_SYMBOL_GPL(driver_find);
