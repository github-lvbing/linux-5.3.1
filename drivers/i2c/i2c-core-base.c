// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Linux I2C core
 *
 * Copyright (C) 1995-99 Simon G. Vogl
 *   With some changes from Ky枚sti M盲lkki <kmalkki@cc.hut.fi>
 *   Mux support by Rodolfo Giometti <giometti@enneenne.com> and
 *   Michael Lawnick <michael.lawnick.ext@nsn.com>
 *
 * Copyright (C) 2013-2017 Wolfram Sang <wsa@the-dreams.de>
 */

#define pr_fmt(fmt) "i2c-core: " fmt

#include <dt-bindings/i2c/i2c.h>
#include <linux/acpi.h>
#include <linux/clk/clk-conf.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/i2c-smbus.h>
#include <linux/idr.h>
#include <linux/init.h>
#include <linux/irqflags.h>
#include <linux/jump_label.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/pm_wakeirq.h>
#include <linux/property.h>
#include <linux/rwsem.h>
#include <linux/slab.h>

#include "i2c-core.h"

#define CREATE_TRACE_POINTS
#include <trace/events/i2c.h>

#define I2C_ADDR_OFFSET_TEN_BIT	0xa000
#define I2C_ADDR_OFFSET_SLAVE	0x1000

#define I2C_ADDR_7BITS_MAX	0x77
#define I2C_ADDR_7BITS_COUNT	(I2C_ADDR_7BITS_MAX + 1)

#define I2C_ADDR_DEVICE_ID	0x7c

/*
 * core_lock protects i2c_adapter_idr, and guarantees that device detection,
 * deletion of detected devices are serialized
 */
/*
 *core_lock保护i2c_adapter_idr，保证设备检测、被检测设备的删除被序列化
 */
static DEFINE_MUTEX(core_lock);
static DEFINE_IDR(i2c_adapter_idr);

static int i2c_detect(struct i2c_adapter *adapter, struct i2c_driver *driver);

static DEFINE_STATIC_KEY_FALSE(i2c_trace_msg_key);

// 标记本子系统是否注册
static bool is_registered;

// 追溯
int i2c_transfer_trace_reg(void)
{
	static_branch_inc(&i2c_trace_msg_key);
	return 0;
}

void i2c_transfer_trace_unreg(void)
{
	static_branch_dec(&i2c_trace_msg_key);
}

// 在i2c_device_id中查找指定的i2c_client，匹配上就返回 i2c_device_id 指针,否则返回NULL。
// 匹配原则是  i2c_client->name ?=  i2c_device_id->name
const struct i2c_device_id *i2c_match_id(const struct i2c_device_id *id,
						const struct i2c_client *client)
{
	// must  id and client is not NULL
	if (!(id && client))
		return NULL;
	// this why is {}. at "struct i2c_device_id"。循环遍历
	while (id->name[0]) {
		// i2c_client->name ?=  i2c_device_id->name
		if (strcmp(client->name, id->name) == 0)
			return id;
		id++;
	}
	return NULL;
}
EXPORT_SYMBOL_GPL(i2c_match_id);

// i2c设备（struct i2c_client）的设备模型(struct device) 与设备驱动模型(struct device_driver)匹配 ,匹配上返回1，否则返回0
// 匹配的优先顺序是：设备树-->ACPI表-->驱动列表
// 被指定到对象（struct bus_type i2c_bus_type）的一个i2c设备与i2c驱动的匹配方法。 
// 匹配上返回1,否则返回0.
static int i2c_device_match(struct device *dev, struct device_driver *drv)
{
    //  校验struct device 是个i2c设备
	struct i2c_client	*client = i2c_verify_client(dev);
	struct i2c_driver	*driver;


	/* Attempt an OF style match */  // 尝试设备树样式匹配
	if (i2c_of_match_device(drv->of_match_table, client)) // ==> drivers\i2c\i2c-core-of.c
		return 1;

	/* Then ACPI style match */     // 尝试 ACPI样式匹配
	if (acpi_driver_match_device(dev, drv))
		return 1;

	driver = to_i2c_driver(drv);

	/* Finally an I2C match */       // 最后是I2C匹配(关注设备驱动的struct i2c_device_id表name成员)
	if (i2c_match_id(driver->id_table, client))
		return 1;

	return 0;
}

static int i2c_device_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	struct i2c_client *client = to_i2c_client(dev);
	int rc;

	rc = of_device_uevent_modalias(dev, env);
	if (rc != -ENODEV)
		return rc;

	rc = acpi_device_uevent_modalias(dev, env);
	if (rc != -ENODEV)
		return rc;

	return add_uevent_var(env, "MODALIAS=%s%s", I2C_MODULE_PREFIX, client->name);
}

/* i2c bus recovery routines */
static int get_scl_gpio_value(struct i2c_adapter *adap)
{
	return gpiod_get_value_cansleep(adap->bus_recovery_info->scl_gpiod);
}

static void set_scl_gpio_value(struct i2c_adapter *adap, int val)
{
	gpiod_set_value_cansleep(adap->bus_recovery_info->scl_gpiod, val);
}

static int get_sda_gpio_value(struct i2c_adapter *adap)
{
	return gpiod_get_value_cansleep(adap->bus_recovery_info->sda_gpiod);
}

static void set_sda_gpio_value(struct i2c_adapter *adap, int val)
{
	gpiod_set_value_cansleep(adap->bus_recovery_info->sda_gpiod, val);
}

static int i2c_generic_bus_free(struct i2c_adapter *adap)
{
	struct i2c_bus_recovery_info *bri = adap->bus_recovery_info;
	int ret = -EOPNOTSUPP;

	if (bri->get_bus_free)
		ret = bri->get_bus_free(adap);
	else if (bri->get_sda)
		ret = bri->get_sda(adap);

	if (ret < 0)
		return ret;

	return ret ? 0 : -EBUSY;
}

/*
 * We are generating clock pulses. ndelay() determines durating of clk pulses.
 * We will generate clock with rate 100 KHz and so duration of both clock levels
 * is: delay in ns = (10^6 / 100) / 2
 */
	
/*
我们正在产生时钟脉冲。ndelay()决定了clk脉冲的持续时间。
*我们将生成速率为100khz的时钟，因此两个时钟级别的持续时间为:ns = (10^6 / 100) / 2
*/

#define RECOVERY_NDELAY		5000
#define RECOVERY_CLK_CNT	9

// 恢复 i2c 时钟
int i2c_generic_scl_recovery(struct i2c_adapter *adap)
{
	struct i2c_bus_recovery_info *bri = adap->bus_recovery_info;
	int i = 0, scl = 1, ret = 0;

	if (bri->prepare_recovery)
		bri->prepare_recovery(adap);

	/*
	 * If we can set SDA, we will always create a STOP to ensure additional
	 * pulses will do no harm. This is achieved by letting SDA follow SCL
	 * half a cycle later. Check the 'incomplete_write_byte' fault injector
	 * for details.
	 */
	bri->set_scl(adap, scl);
	ndelay(RECOVERY_NDELAY / 2);
	if (bri->set_sda)
		bri->set_sda(adap, scl);
	ndelay(RECOVERY_NDELAY / 2);

	/*
	 * By this time SCL is high, as we need to give 9 falling-rising edges
	 */
	while (i++ < RECOVERY_CLK_CNT * 2) {
		if (scl) {
			/* SCL shouldn't be low here */
			if (!bri->get_scl(adap)) {
				dev_err(&adap->dev,
					"SCL is stuck low, exit recovery\n");
				ret = -EBUSY;
				break;
			}
		}

		scl = !scl;
		bri->set_scl(adap, scl);
		/* Creating STOP again, see above */
		ndelay(RECOVERY_NDELAY / 2);
		if (bri->set_sda)
			bri->set_sda(adap, scl);
		ndelay(RECOVERY_NDELAY / 2);

		if (scl) {
			ret = i2c_generic_bus_free(adap);
			if (ret == 0)
				break;
		}
	}

	/* If we can't check bus status, assume recovery worked */
	if (ret == -EOPNOTSUPP)
		ret = 0;

	if (bri->unprepare_recovery)
		bri->unprepare_recovery(adap);

	return ret;
}
EXPORT_SYMBOL_GPL(i2c_generic_scl_recovery);

//  i2c 总线恢复
int i2c_recover_bus(struct i2c_adapter *adap)
{
	if (!adap->bus_recovery_info)
		return -EOPNOTSUPP;

	dev_dbg(&adap->dev, "Trying i2c bus recovery\n");
	return adap->bus_recovery_info->recover_bus(adap);
}
EXPORT_SYMBOL_GPL(i2c_recover_bus);

static void i2c_init_recovery(struct i2c_adapter *adap)
{
	struct i2c_bus_recovery_info *bri = adap->bus_recovery_info;
	char *err_str;

	if (!bri)
		return;

	if (!bri->recover_bus) {
		err_str = "no recover_bus() found";
		goto err;
	}

	if (bri->scl_gpiod && bri->recover_bus == i2c_generic_scl_recovery) {
		bri->get_scl = get_scl_gpio_value;
		bri->set_scl = set_scl_gpio_value;
		if (bri->sda_gpiod) {
			bri->get_sda = get_sda_gpio_value;
			/* FIXME: add proper flag instead of '0' once available */
			if (gpiod_get_direction(bri->sda_gpiod) == 0)
				bri->set_sda = set_sda_gpio_value;
		}
		return;
	}

	if (bri->recover_bus == i2c_generic_scl_recovery) {
		/* Generic SCL recovery */
		if (!bri->set_scl || !bri->get_scl) {
			err_str = "no {get|set}_scl() found";
			goto err;
		}
		if (!bri->set_sda && !bri->get_sda) {
			err_str = "either get_sda() or set_sda() needed";
			goto err;
		}
	}

	return;
 err:
	dev_err(&adap->dev, "Not using recovery: %s\n", err_str);
	adap->bus_recovery_info = NULL;
}

static int i2c_smbus_host_notify_to_irq(const struct i2c_client *client)
{
	struct i2c_adapter *adap = client->adapter;
	unsigned int irq;

	if (!adap->host_notify_domain)
		return -ENXIO;

	if (client->flags & I2C_CLIENT_TEN)
		return -EINVAL;

	irq = irq_create_mapping(adap->host_notify_domain, client->addr);

	return irq > 0 ? irq : -ENXIO;
}

// 被指定到对象（struct bus_type i2c_bus_type）的一个i2c设备与i2c驱动的匹配后前期预处理方法。
static int i2c_device_probe(struct device *dev)
{
	struct i2c_client	*client = i2c_verify_client(dev);
	struct i2c_driver	*driver;
	int status;

	if (!client)
		return 0;

	driver = to_i2c_driver(dev->driver);

	client->irq = client->init_irq;

	if (!client->irq && !driver->disable_i2c_core_irq_mapping) {
		int irq = -ENOENT;

		if (client->flags & I2C_CLIENT_HOST_NOTIFY) {
			dev_dbg(dev, "Using Host Notify IRQ\n");
			/* Keep adapter active when Host Notify is required */
			pm_runtime_get_sync(&client->adapter->dev);
			irq = i2c_smbus_host_notify_to_irq(client);
		} else if (dev->of_node) {
			irq = of_irq_get_byname(dev->of_node, "irq");
			if (irq == -EINVAL || irq == -ENODATA)
				irq = of_irq_get(dev->of_node, 0);
		} else if (ACPI_COMPANION(dev)) {
			irq = i2c_acpi_get_irq(client);
		}
		if (irq == -EPROBE_DEFER)
			return irq;

		if (irq < 0)
			irq = 0;

		client->irq = irq;
	}

	/*
	 * An I2C ID table is not mandatory, if and only if, a suitable OF
	 * or ACPI ID table is supplied for the probing device.
	 */
	if (!driver->id_table &&
	    !i2c_acpi_match_device(dev->driver->acpi_match_table, client) &&
	    !i2c_of_match_device(dev->driver->of_match_table, client))
		return -ENODEV;

	if (client->flags & I2C_CLIENT_WAKE) {
		int wakeirq = -ENOENT;

		if (dev->of_node) {
			wakeirq = of_irq_get_byname(dev->of_node, "wakeup");
			if (wakeirq == -EPROBE_DEFER)
				return wakeirq;
		}

		device_init_wakeup(&client->dev, true);

		if (wakeirq > 0 && wakeirq != client->irq)
			status = dev_pm_set_dedicated_wake_irq(dev, wakeirq);
		else if (client->irq > 0)
			status = dev_pm_set_wake_irq(dev, client->irq);
		else
			status = 0;

		if (status)
			dev_warn(&client->dev, "failed to set up wakeup irq\n");
	}

	dev_dbg(dev, "probe\n");

	status = of_clk_set_defaults(dev->of_node, false);
	if (status < 0)
		goto err_clear_wakeup_irq;

	status = dev_pm_domain_attach(&client->dev, true);
	if (status)
		goto err_clear_wakeup_irq;

	/*
	 * When there are no more users of probe(),
	 * rename probe_new to probe.
	 */
	if (driver->probe_new)
		status = driver->probe_new(client);
	else if (driver->probe)
		status = driver->probe(client,
				       i2c_match_id(driver->id_table, client));
	else
		status = -EINVAL;

	if (status)
		goto err_detach_pm_domain;

	return 0;

err_detach_pm_domain:
	dev_pm_domain_detach(&client->dev, true);
err_clear_wakeup_irq:
	dev_pm_clear_wake_irq(&client->dev);
	device_init_wakeup(&client->dev, false);
	return status;
}

// 被指定到对象（struct bus_type i2c_bus_type）的一个i2c设备与i2c驱动的解除匹配后后期处理方法。
static int i2c_device_remove(struct device *dev)
{
	struct i2c_client	*client = i2c_verify_client(dev);
	struct i2c_driver	*driver;
	int status = 0;

	if (!client || !dev->driver)
		return 0;

	driver = to_i2c_driver(dev->driver);
	if (driver->remove) {
		dev_dbg(dev, "remove\n");
		status = driver->remove(client);
	}

	dev_pm_domain_detach(&client->dev, true);

	dev_pm_clear_wake_irq(&client->dev);
	device_init_wakeup(&client->dev, false);

	client->irq = 0;
	if (client->flags & I2C_CLIENT_HOST_NOTIFY)
		pm_runtime_put(&client->adapter->dev);

	return status;
}

// 被指定到对象（struct bus_type i2c_bus_type）的一个i2c驱动关闭i2c设备的关闭方法。
static void i2c_device_shutdown(struct device *dev)
{
	struct i2c_client *client = i2c_verify_client(dev);
	struct i2c_driver *driver;

	if (!client || !dev->driver)
		return;
	driver = to_i2c_driver(dev->driver);
	if (driver->shutdown)
		driver->shutdown(client);
}

static void i2c_client_dev_release(struct device *dev)
{
	kfree(to_i2c_client(dev));
}

static ssize_t
show_name(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", dev->type == &i2c_client_type ?
		       to_i2c_client(dev)->name : to_i2c_adapter(dev)->name);
}
static DEVICE_ATTR(name, S_IRUGO, show_name, NULL);

static ssize_t
show_modalias(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int len;

	len = of_device_modalias(dev, buf, PAGE_SIZE);
	if (len != -ENODEV)
		return len;

	len = acpi_device_modalias(dev, buf, PAGE_SIZE -1);
	if (len != -ENODEV)
		return len;

	return sprintf(buf, "%s%s\n", I2C_MODULE_PREFIX, client->name);
}
static DEVICE_ATTR(modalias, S_IRUGO, show_modalias, NULL);

static struct attribute *i2c_dev_attrs[] = {
	&dev_attr_name.attr,
	/* modalias helps coldplug:  modprobe $(cat .../modalias) */
	&dev_attr_modalias.attr,
	NULL
};
ATTRIBUTE_GROUPS(i2c_dev);

// i2c设备总线类型对象
struct bus_type i2c_bus_type = {
	.name		= "i2c",
	.match		= i2c_device_match,
	.probe		= i2c_device_probe,
	.remove		= i2c_device_remove,
	.shutdown	= i2c_device_shutdown,
};
EXPORT_SYMBOL_GPL(i2c_bus_type);

// i2c设备类型对象
struct device_type i2c_client_type = {
	.groups		= i2c_dev_groups,
	.uevent		= i2c_device_uevent,
	.release	= i2c_client_dev_release,
};
EXPORT_SYMBOL_GPL(i2c_client_type);


/**
 * i2c_verify_client - return parameter as i2c_client, or NULL
 * @dev: device, probably from some driver model iterator
 *
 * When traversing the driver model tree, perhaps using driver model
 * iterators like @device_for_each_child(), you can't assume very much
 * about the nodes you find.  Use this function to avoid oopses caused
 * by wrongly treating some non-I2C device as an i2c_client.
 */
 /*
 * i2c_verify_client—返回参数i2c_client，或NULL
 *  @dev: device，可能是在遍历驱动程序模型树时从某个驱动程序模型迭代器中获得的，
 *                也可能使用像@device_for_each_child()这样的驱动程序模型迭代器，您不能对找到的节点做太多假设。
 *                使用此函数可以避免由于错误地将某些非i2c设备视为i2c_client而导致的oopses。
 *
 */
struct i2c_client *i2c_verify_client(struct device *dev)
{
	return (dev->type == &i2c_client_type)
			? to_i2c_client(dev)
			: NULL;
}
EXPORT_SYMBOL(i2c_verify_client);


/* Return a unique address which takes the flags of the client into account */
// 返回i2c设备的的硬件地址
static unsigned short i2c_encode_flags_to_addr(struct i2c_client *client)
{
	unsigned short addr = client->addr;

	/* For some client flags, add an arbitrary offset to avoid collisions */
	if (client->flags & I2C_CLIENT_TEN)
		addr |= I2C_ADDR_OFFSET_TEN_BIT;

	if (client->flags & I2C_CLIENT_SLAVE)
		addr |= I2C_ADDR_OFFSET_SLAVE;

	return addr;
}

/* This is a permissive address validity check, I2C address map constraints
 * are purposely not enforced, except for the general call address. */
 // 这是一个许可的地址有效性检查，除了一般的调用地址,I2C地址 map的约束是故意不执行的。
 // 有效返回0.
static int i2c_check_addr_validity(unsigned int addr, unsigned short flags)
{
	if (flags & I2C_CLIENT_TEN) {
		/* 10-bit address, all values are valid */
		if (addr > 0x3ff)
			return -EINVAL;
	} else {
		/* 7-bit address, reject the general call address */
		if (addr == 0x00 || addr > 0x7f)
			return -EINVAL;
	}
	return 0;
}

/* And this is a strict address validity check, used when probing. If a
 * device uses a reserved address, then it shouldn't be probed. 7-bit
 * addressing is assumed, 10-bit address devices are rare and should be
 * explicitly enumerated. */
 // 这是一个严格的地址有效性检查，用于探测。
 // 如果一个设备使用一个保留地址，那么它不应该被探测。假设是7位地址，10位地址设备很少，应该显式枚举。
int i2c_check_7bit_addr_validity_strict(unsigned short addr)
{
	/*
	 * Reserved addresses per I2C specification:  *按I2C规格预留地址:
	 *  0x00       General call address / START byte   一般调用地址/开始字节
	 *  0x01       CBUS address                       CBUS地址
	 *  0x02       Reserved for different bus format  预留给不同的总线格式
	 *  0x03       Reserved for future purposes      保留作将来使用
	 *  0x04-0x07  Hs-mode master code               Hs-mode主代码
	 *  0x78-0x7b  10-bit slave addressing          10位从地址
	 *  0x7c-0x7f  Reserved for future purposes     保留作将来使用
	 */
	if (addr < 0x08 || addr > 0x77)
		return -EINVAL;
	return 0;
}

// struct device 是个i2c设备，且它的硬件地址是否是addrp，有则返回-EBUSY。说明已经被使用，忙。
static int __i2c_check_addr_busy(struct device *dev, void *addrp)
{
	struct i2c_client	*client = i2c_verify_client(dev);
	int			addr = *(int *)addrp;

	if (client && i2c_encode_flags_to_addr(client) == addr)
		return -EBUSY;
	return 0;
}

/* walk up mux tree */
// 递归查找struct i2c_adapter的父亲。判断父亲上是否有“addrp”这个i2c设备。有返回-EBUSY。无返回0。
static int i2c_check_mux_parents(struct i2c_adapter *adapter, int addr)
{
	//  获得适配器的父设备地址。是适配器设备返回地址，不是返回NULL。
	struct i2c_adapter *parent = i2c_parent_is_2c_adapter(adapter);
	int result;
	// 通过迭代，在适配器上查找其上硬件设备地址为addr的设备。有这个设备返回-EBUSY，无返回0。
	result = device_for_each_child(&adapter->dev, &addr,
					__i2c_check_addr_busy);

	// 适配器adapter的父设备不是一个适配器，或者父设备（适配器adapter）上不存在addr这个设备孩子，则返回0。否则继续递归继续查找父父设备。
	if (!result && parent)
		result = i2c_check_mux_parents(parent, addr);

	return result;
}

/* recurse down mux tree */ 
// 迭代器：递归查找struct device的孩子。判断孩子上是否有“addrp”这个i2c设备。有返回-EBUSY。无返回0。
static int i2c_check_mux_children(struct device *dev, void *addrp)
{
	int result;
	// 若设备是一个适配器设备，则遍历它的孩子。直到它的孩子遍历完或者找到了一个i2c设备。会触发返回。
	if (dev->type == &i2c_adapter_type)
		result = device_for_each_child(dev, addrp,
						i2c_check_mux_children);
	// 若设备是不是一个适配器设备，若 struct device是否是i2c设备，且它的硬件地址是addrp，是说明有这个设备。返回-EBUSY。
	else
		result = __i2c_check_addr_busy(dev, addrp);

	return result;
}

// 查找判断在适配器上是否有地址为“addr”的i2c设备。已经占用就返回-EBUSY，无返回0。
static int i2c_check_addr_busy(struct i2c_adapter *adapter, int addr)
{
	struct i2c_adapter *parent = i2c_parent_is_i2c_adapter(adapter);
	int result = 0;
	// 若adapter的父设备也是一个适配器，递归查找它的父亲。
	// 判断父亲上是否有“addr”这个i2c设备。有返回-EBUSY= result。无返回0=result.
	if (parent)
		result = i2c_check_mux_parents(parent, addr);
	// 若adapter的父设备不是一个适配器，或者父设备不包含“addr”这个i2c设备。就递归查找struct device的孩子。判断孩子上是否有地址为“addrp”这个i2c设备。有返回-EBUSY。无返回0。
	if (!result)
		result = device_for_each_child(&adapter->dev, &addr,
						i2c_check_mux_children);

	return result;
}

/**
 * i2c_adapter_lock_bus - Get exclusive access to an I2C bus segment
 * @adapter: Target I2C bus segment
 * @flags: I2C_LOCK_ROOT_ADAPTER locks the root i2c adapter, I2C_LOCK_SEGMENT
 *	locks only this branch in the adapter tree
 */
 /**
 * i2c_adapter_lock_bus -获得对I2C总线段的独占访问
 * @adapter:目标I2C总线段
 * @flags: I2C_LOCK_ROOT_ADAPTER 锁定根i2c适配器，
 *         I2C_LOCK_SEGMENT 仅锁定适配器树中的这个分支
 */
static void i2c_adapter_lock_bus(struct i2c_adapter *adapter,
				 unsigned int flags)
{
	rt_mutex_lock_nested(&adapter->bus_lock, i2c_adapter_depth(adapter));
}

/**
 * i2c_adapter_trylock_bus - Try to get exclusive access to an I2C bus segment
 * @adapter: Target I2C bus segment
 * @flags: I2C_LOCK_ROOT_ADAPTER trylocks the root i2c adapter, I2C_LOCK_SEGMENT
 *	trylocks only this branch in the adapter tree
 */
 /**
 * i2c_adapter_lock_bus -尝试获得对I2C总线段的独占访问
 * @adapter:目标I2C总线段
 * @flags: I2C_LOCK_ROOT_ADAPTER 锁定根i2c适配器，
 *		   I2C_LOCK_SEGMENT 仅锁定适配器树中的这个分支
 */

static int i2c_adapter_trylock_bus(struct i2c_adapter *adapter,
				   unsigned int flags)
{
	return rt_mutex_trylock(&adapter->bus_lock);
}

/**
 * i2c_adapter_unlock_bus - Release exclusive access to an I2C bus segment
 * @adapter: Target I2C bus segment
 * @flags: I2C_LOCK_ROOT_ADAPTER unlocks the root i2c adapter, I2C_LOCK_SEGMENT
 *	unlocks only this branch in the adapter tree
 */
/**
 * i2c_adapter_unlock_bus -释放对I2C总线段的独占访问
 * @adapter:目标I2C总线段
 * @flags: I2C_LOCK_ROOT_ADAPTER解锁根i2c适配器，I2C_LOCK_SEGMENT仅解锁适配器树中的这个分支
 */
static void i2c_adapter_unlock_bus(struct i2c_adapter *adapter,
				   unsigned int flags)
{
	rt_mutex_unlock(&adapter->bus_lock);
}

// 给i2c设备client 设置名字（根据struct i2c_adapter 或者struct i2c_board_info）
static void i2c_dev_set_name(struct i2c_adapter *adap,
			     struct i2c_client *client,
			     struct i2c_board_info const *info)
{
	struct acpi_device *adev = ACPI_COMPANION(&client->dev);
	// 优先使用 struct i2c_board_info  信息
	if (info && info->dev_name) {
		dev_set_name(&client->dev, "i2c-%s", info->dev_name);
		return;
	}
	// 其次使用 struct i2c_adapter 信息
	if (adev) {
		dev_set_name(&client->dev, "i2c-%s", acpi_dev_name(adev));
		return;
	}
	// 最后使用 struct i2c_adapter 信息和 client地址信息
	dev_set_name(&client->dev, "%d-%04x", i2c_adapter_id(adap),
		     i2c_encode_flags_to_addr(client));
}

// 在资源表 struct resource中查找中断资源，并返回中断号，找不到返回0.
int i2c_dev_irq_from_resources(const struct resource *resources,
			       unsigned int num_resources)
{
	struct irq_data *irqd;
	int i;

	for (i = 0; i < num_resources; i++) {
		const struct resource *r = &resources[i];
        // 若不是IO的中断资源，就下一个
		if (resource_type(r) != IORESOURCE_IRQ)
			continue;
        // 若是中断资源资源，且有具体的定义。就查找中断树，以获得struct irq_data。
		if (r->flags & IORESOURCE_BITS) {
			irqd = irq_get_irq_data(r->start);
			// 无法获得 irq_data 函数立刻返回0.
			if (!irqd)
				break;
            // 若获得 irq_data ，更新irq芯片功能的状态信息。
			irqd_set_trigger_type(irqd, r->flags & IORESOURCE_BITS);
		}

		return r->start;
	}

	return 0;
}

/**
 * i2c_new_client_device - instantiate an i2c device
 * @adap: the adapter managing the device
 * @info: describes one I2C device; bus_num is ignored
 * Context: can sleep
 *
 * Create an i2c device. Binding is handled through driver model
 * probe()/remove() methods.  A driver may be bound to this device when we
 * return from this function, or any later moment (e.g. maybe hotplugging will
 * load the driver module).  This call is not appropriate for use by mainboard
 * initialization logic, which usually runs during an arch_initcall() long
 * before any i2c_adapter could exist.
 *
 * This returns the new i2c client, which may be saved for later use with
 * i2c_unregister_device(); or an ERR_PTR to describe the error.
 */
 /*
  * i2c_new_client_device -实例化一个i2c设备，依附适配器adap后并注册(device_register接口)到i2c总线。
  * @adap:管理设备的适配器
  * @info:描述一个I2C设备;bus_num被忽略上下文:可以休眠
  * 创建一个i2c设备。绑定是通过驱动程序模型probe()/remove()方法来处理的。
  * 当我们从这个函数返回时，驱动程序可能被绑定到这个设备，或者稍后的任何时刻(例如，可能热插拔将加载驱动程序模块)。
  * 主板初始化逻辑不适合使用这个调用，因为它通常在arch_initcall()期间运行，比任何i2c_adapter都要早得多。
  * 这将返回新的i2c客户端，该客户端可能被保存起来，以便稍后与i2c_unregister_device()一起使用;或者使用ERR_PTR来描述错误。
  */
struct i2c_client *
i2c_new_client_device(struct i2c_adapter *adap, struct i2c_board_info const *info)
{
	struct i2c_client	*client;
	int			status;
    // 创建一个i2c设备对象 struct i2c_client
	client = kzalloc(sizeof *client, GFP_KERNEL);
	if (!client)
		return ERR_PTR(-ENOMEM);
	// 指定i2c设备的适配器为 adap
	client->adapter = adap;
    // 将info的平台数据、flags、addr、irq指定为i2c设备的平台数据、flags、addr、和init_irq。
	client->dev.platform_data = info->platform_data;
	client->flags = info->flags;
	client->addr = info->addr;

	client->init_irq = info->irq;
	// 如果 info->irq=0，无中断信息，就在i2c_board_info的资源表 struct resource中查找中断资源，并返回中断号，找不到返回0.
	if (!client->init_irq)
		client->init_irq = i2c_dev_irq_from_resources(info->resources,
							 info->num_resources);
	// 将info的芯片类型标识，作为 i2c_client 的name。
	strlcpy(client->name, info->type, sizeof(client->name));

	// 校验i2c_client.addr 的有效性。无效的话，就err后退出。
	status = i2c_check_addr_validity(client->addr, client->flags);
	if (status) {
		dev_err(&adap->dev, "Invalid %d-bit I2C address 0x%02hx\n",
			client->flags & I2C_CLIENT_TEN ? 10 : 7, client->addr);
		goto out_err_silent;
	}

	/* Check for address business  */ 
	// 查询地址是否已经被占用,若已经占用就err后退出。
	status = i2c_check_addr_busy(adap, i2c_encode_flags_to_addr(client));
	if (status)
		goto out_err;
	// i2c设备的父设备指定为适配器；总线为 i2c_bus_type；设备类型为 i2c_client_type。
	client->dev.parent = &client->adapter->dev;
	client->dev.bus = &i2c_bus_type;
	client->dev.type = &i2c_client_type;
	// ??
	client->dev.of_node = of_node_get(info->of_node);
	client->dev.fwnode = info->fwnode;

	// 为i2c设备对象设置名字。
	i2c_dev_set_name(adap, client, info);

	// 若存在设备的附加属性，向设备对象添加属性集合
	if (info->properties) {
		status = device_add_properties(&client->dev, info->properties);
		if (status) {
			dev_err(&adap->dev,
				"Failed to add properties to client %s: %d\n",
				client->name, status);
			goto out_err_put_of_node;
		}
	}

	// 将i2c设备对象 注册到i2c总线中。
	status = device_register(&client->dev);
	if (status)
		goto out_free_props;

	dev_dbg(&adap->dev, "client [%s] registered with bus id %s\n",
		client->name, dev_name(&client->dev));

	return client;

out_free_props:
	if (info->properties)
		device_remove_properties(&client->dev);
out_err_put_of_node:
	of_node_put(info->of_node);
out_err:
	dev_err(&adap->dev,
		"Failed to register i2c client %s at 0x%02x (%d)\n",
		client->name, client->addr, status);
out_err_silent:
	kfree(client);
	return ERR_PTR(status);
}
EXPORT_SYMBOL_GPL(i2c_new_client_device);

/**
 * i2c_new_device - instantiate an i2c device
 * @adap: the adapter managing the device
 * @info: describes one I2C device; bus_num is ignored
 * Context: can sleep
 *
 * This deprecated function has the same functionality as
 * @i2c_new_client_device, it just returns NULL instead of an ERR_PTR in case of
 * an error for compatibility with current I2C API. It will be removed once all
 * users are converted.
 *
 * This returns the new i2c client, which may be saved for later use with
 * i2c_unregister_device(); or NULL to indicate an error.
 */
/**
 * i2c_new_device——通过 i2c_board_info 实例化一个i2c设备，依附适配器adap后并注册到i2c总线。
 * @adap:管理设备的适配器
 * @info:描述一个I2C设备;bus_num被忽略上下文:可以休眠
 * 这个被弃用的函数与@i2c_new_client_device具有相同的功能，它只是返回NULL，而不是ERR_PTR，以防与当前I2C API兼容出现错误。
 * 一旦所有用户都进行了转换，它将被删除。
 * 这将返回新的i2c客户端，该客户端可能被保存起来，以便稍后与i2c_unregister_device()一起使用;或NULL表示错误。
 */
struct i2c_client *
i2c_new_device(struct i2c_adapter *adap, struct i2c_board_info const *info)
{
	struct i2c_client *ret;

	ret = i2c_new_client_device(adap, info);
	return IS_ERR(ret) ? NULL : ret;
}
EXPORT_SYMBOL_GPL(i2c_new_device);


/**
 * i2c_unregister_device - reverse effect of i2c_new_device()
 * @client: value returned from i2c_new_device()
 * Context: can sleep
 */
/**
 * i2c_unregister_device - i2c_new_device()的反向效果;在i2c总线上注销i2c设备
 * @client:从i2c_new_device()上下文返回的值:可以休眠
 */
void i2c_unregister_device(struct i2c_client *client)
{
	if (IS_ERR_OR_NULL(client))
		return;

	if (client->dev.of_node) {
		of_node_clear_flag(client->dev.of_node, OF_POPULATED);
		of_node_put(client->dev.of_node);
	}

	if (ACPI_COMPANION(&client->dev))
		acpi_device_clear_enumerated(ACPI_COMPANION(&client->dev));
	// 从系统中注销设备。	
	device_unregister(&client->dev);
}
EXPORT_SYMBOL_GPL(i2c_unregister_device);


static const struct i2c_device_id dummy_id[] = {
	{ "dummy", 0 },
	{ },
};

static int dummy_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	return 0;
}

static int dummy_remove(struct i2c_client *client)
{
	return 0;
}

// 一个仿真的 i2c 设备驱动。查考i2c子系统初始化： static int __init i2c_init(void)。
static struct i2c_driver dummy_driver = {
	.driver.name	= "dummy",
	.probe		= dummy_probe,
	.remove		= dummy_remove,
	.id_table	= dummy_id,
};

/**
 * i2c_new_dummy_device - return a new i2c device bound to a dummy driver
 * @adapter: the adapter managing the device
 * @address: seven bit address to be used
 * Context: can sleep
 *
 * This returns an I2C client bound to the "dummy" driver, intended for use
 * with devices that consume multiple addresses.  Examples of such chips
 * include various EEPROMS (like 24c04 and 24c08 models).
 *
 * These dummy devices have two main uses.  First, most I2C and SMBus calls
 * except i2c_transfer() need a client handle; the dummy will be that handle.
 * And second, this prevents the specified address from being bound to a
 * different driver.
 *
 * This returns the new i2c client, which should be saved for later use with
 * i2c_unregister_device(); or an ERR_PTR to describe the error.
 */
 /*
 * i2c_new_dummy_device—实例化一个设备地址为addr的从设备，并注册到总线，返回绑定到虚拟驱动程序的新i2c设备
 * @adapter:管理设备的适配器
 * @address:使用的7位地址
 * Context:可以睡觉
 * 这将返回一个绑定到“虚拟”驱动程序的I2C客户端，用于使用多个地址的设备。这类芯片的例子包括各种EEPROMS(如24c04和24c08型号)。
 * 这些虚拟设备有两个主要用途。首先，除了i2c_transfer()之外，大多数I2C和SMBus调用都需要一个客户端句柄;假人就是那个把手。
 * 其次，这可以防止将指定的地址绑定到不同的驱动程序。
 * 这将返回新的i2c客户端，该客户端应该保存起来，以便稍后与i2c_unregister_device()一起使用;或者使用ERR_PTR来描述错误。
 */
struct i2c_client *i2c_new_dummy_device(struct i2c_adapter *adapter, u16 address)
{
	struct i2c_board_info info = {
		I2C_BOARD_INFO("dummy", address),
	};

	return i2c_new_client_device(adapter, &info);
}
EXPORT_SYMBOL_GPL(i2c_new_dummy_device);

/**
 * i2c_new_dummy - return a new i2c device bound to a dummy driver
 * @adapter: the adapter managing the device
 * @address: seven bit address to be used
 * Context: can sleep
 *
 * This deprecated function has the same functionality as @i2c_new_dummy_device,
 * it just returns NULL instead of an ERR_PTR in case of an error for
 * compatibility with current I2C API. It will be removed once all users are
 * converted.
 *
 * This returns the new i2c client, which should be saved for later use with
 * i2c_unregister_device(); or NULL to indicate an error.
 */
 /*
 * i2c_new_dummy—实例化一个设备地址为addr的从设备，依附adapter后，并注册到总线，返回绑定到虚拟驱动程序的新i2c设备
 * @adapter:管理设备的适配器
 * @address:使用的7位地址
 * 上下文:可以睡觉
 * 这个废弃的函数与@i2c_new_dummy_device具有相同的功能，
 * 如果与当前I2C API兼容出现错误，它只返回NULL，而不是ERR_PTR。一旦所有用户都进行了转换，它将被删除。
 * 这将返回新的i2c客户端，该客户端应该保存起来，以便稍后与i2c_unregister_device()一起使用;或NULL表示错误。
 */
struct i2c_client *i2c_new_dummy(struct i2c_adapter *adapter, u16 address)
{
	struct i2c_client *ret;

	ret = i2c_new_dummy_device(adapter, address);
	return IS_ERR(ret) ? NULL : ret;
}
EXPORT_SYMBOL_GPL(i2c_new_dummy);

struct i2c_dummy_devres {
	struct i2c_client *client;
};

static void devm_i2c_release_dummy(struct device *dev, void *res)
{
	struct i2c_dummy_devres *this = res;

	i2c_unregister_device(this->client);
}

/**
 * devm_i2c_new_dummy_device - return a new i2c device bound to a dummy driver
 * @dev: device the managed resource is bound to
 * @adapter: the adapter managing the device
 * @address: seven bit address to be used
 * Context: can sleep
 *
 * This is the device-managed version of @i2c_new_dummy_device. It returns the
 * new i2c client or an ERR_PTR in case of an error.
 */
/**
 * devm_i2c_new_dummy_device—返回绑定到虚拟驱动程序的新i2c设备
 * @dev:被管理资源绑定到的设备
 * @adapter:管理设备的适配器
 * @address:使用的7位地址
 * 上下文:可以睡觉
 * 这是@i2c_new_dummy_device的设备管理版本。如果出现错误，它将返回新的i2c客户机或ERR_PTR。
 */
struct i2c_client *devm_i2c_new_dummy_device(struct device *dev,
					     struct i2c_adapter *adapter,
					     u16 address)
{
	struct i2c_dummy_devres *dr;
	struct i2c_client *client;

	dr = devres_alloc(devm_i2c_release_dummy, sizeof(*dr), GFP_KERNEL);
	if (!dr)
		return ERR_PTR(-ENOMEM);

	client = i2c_new_dummy_device(adapter, address);
	if (IS_ERR(client)) {
		devres_free(dr);
	} else {
		dr->client = client;
		devres_add(dev, dr);
	}

	return client;
}
EXPORT_SYMBOL_GPL(devm_i2c_new_dummy_device);

/**
 * i2c_new_secondary_device - Helper to get the instantiated secondary address
 * and create the associated device
 * @client: Handle to the primary client
 * @name: Handle to specify which secondary address to get
 * @default_addr: Used as a fallback if no secondary address was specified
 * Context: can sleep
 *
 * I2C clients can be composed of multiple I2C slaves bound together in a single
 * component. The I2C client driver then binds to the master I2C slave and needs
 * to create I2C dummy clients to communicate with all the other slaves.
 *
 * This function creates and returns an I2C dummy client whose I2C address is
 * retrieved from the platform firmware based on the given slave name. If no
 * address is specified by the firmware default_addr is used.
 *
 * On DT-based platforms the address is retrieved from the "reg" property entry
 * cell whose "reg-names" value matches the slave name.
 *
 * This returns the new i2c client, which should be saved for later use with
 * i2c_unregister_device(); or NULL to indicate an error.
 */
/**
* i2c_new_secondary_device -帮助获取实例化的辅助地址
* 并创建关联设备
* @client:主客户端的句柄
* @name:指定获取哪个二级地址的句柄
* @default_addr:如果没有指定辅助地址，则用作回退
* 上下文:可以睡觉
*
* I2C客户端可以由多个绑定在单个组件中的I2C从服务器组成。然后，I2C客户端驱动程序绑定到主I2C从设备，并需要创建I2C虚拟客户端来与所有其他从设备通信。
*
* 这个函数创建并返回一个I2C虚拟客户端，它的I2C地址是根据给定的从属名从平台固件中获取的。如果固件没有指定地址，则使用default_addr。
*
* 在基于dd的平台上，地址是从“reg”属性条目单元格中检索的，该单元格的“reg-names”值与从属名称匹配。
*
* 这将返回新的i2c客户端，该客户端应该保存起来，以便稍后与i2c_unregister_device()一起使用;或NULL表示错误。
*/
struct i2c_client *i2c_new_secondary_device(struct i2c_client *client,
						const char *name,
						u16 default_addr)
{
	struct device_node *np = client->dev.of_node;
	u32 addr = default_addr;
	int i;

	if (np) {
		i = of_property_match_string(np, "reg-names", name);
		if (i >= 0)
			of_property_read_u32_index(np, "reg", i, &addr);
	}

	dev_dbg(&client->adapter->dev, "Address for %s : 0x%x\n", name, addr);
	return i2c_new_dummy(client->adapter, addr);
}
EXPORT_SYMBOL_GPL(i2c_new_secondary_device);

/* ------------------------------------------------------------------------- */

/* I2C bus adapters -- one roots each I2C or SMBUS segment */
// I2C总线适配器——每个I2C或SMBUS段都有一个根

// 告诉另一个线程工作已经完成。
static void i2c_adapter_dev_release(struct device *dev)
{
	struct i2c_adapter *adap = to_i2c_adapter(dev);
	complete(&adap->dev_released);
}

// 查看这是适配器的深度
unsigned int i2c_adapter_depth(struct i2c_adapter *adapter)
{
	unsigned int depth = 0;
	
	while ((adapter = i2c_parent_is_i2c_adapter(adapter)))
		depth++;

	WARN_ONCE(depth >= MAX_LOCKDEP_SUBCLASSES,
		  "adapter depth exceeds lockdep subclass limit\n");

	return depth;
}
EXPORT_SYMBOL_GPL(i2c_adapter_depth);

/*
 * Let users instantiate I2C devices through sysfs. This can be used when
 * platform initialization code doesn't contain the proper data for
 * whatever reason. Also useful for drivers that do device detection and
 * detection fails, either because the device uses an unexpected address,
 * or this is a compatible device with different ID register values.
 *
 * Parameter checking may look overzealous, but we really don't want
 * the user to provide incorrect parameters.
 */
/*	
* 让用户通过sysfs实例化I2C设备。当平台初始化代码由于某种原因不包含适当的数据时，可以使用这种方法。
* 对于设备检测和检测失败的驱动程序也很有用，因为设备使用了一个意外的地址，或者这是一个具有不同ID寄存器值的兼容设备。	
* 参数检查可能看起来过于热心，但我们真的不希望用户提供不正确的参数。	
*/
static ssize_t
i2c_sysfs_new_device(struct device *dev, struct device_attribute *attr,
		     const char *buf, size_t count)
{
	struct i2c_adapter *adap = to_i2c_adapter(dev);
	struct i2c_board_info info;
	struct i2c_client *client;
	char *blank, end;
	int res;
	// 构建 struct i2c_board_info对象。
	memset(&info, 0, sizeof(struct i2c_board_info));

	blank = strchr(buf, ' ');
	if (!blank) {
		dev_err(dev, "%s: Missing parameters\n", "new_device");
		return -EINVAL;
	}
	if (blank - buf > I2C_NAME_SIZE - 1) {
		dev_err(dev, "%s: Invalid device name\n", "new_device");
		return -EINVAL;
	}
	memcpy(info.type, buf, blank - buf);

	/* Parse remaining parameters, reject extra parameters */
	// 解析剩余的参数，拒绝额外的参数
	res = sscanf(++blank, "%hi%c", &info.addr, &end);
	if (res < 1) {
		dev_err(dev, "%s: Can't parse I2C address\n", "new_device");
		return -EINVAL;
	}
	if (res > 1  && end != '\n') {
		dev_err(dev, "%s: Extra parameters\n", "new_device");
		return -EINVAL;
	}

	if ((info.addr & I2C_ADDR_OFFSET_TEN_BIT) == I2C_ADDR_OFFSET_TEN_BIT) {
		info.addr &= ~I2C_ADDR_OFFSET_TEN_BIT;
		info.flags |= I2C_CLIENT_TEN;
	}

	if (info.addr & I2C_ADDR_OFFSET_SLAVE) {
		info.addr &= ~I2C_ADDR_OFFSET_SLAVE;
		info.flags |= I2C_CLIENT_SLAVE;
	}

	// 实例化一个i2c设备，依附适配器adap后并注册(device_register接口)到i2c总线。
	client = i2c_new_client_device(adap, &info);
	if (IS_ERR(client))
		return PTR_ERR(client);

	/* Keep track of the added device */
	mutex_lock(&adap->userspace_clients_lock);
	list_add_tail(&client->detected, &adap->userspace_clients);
	mutex_unlock(&adap->userspace_clients_lock);
	dev_info(dev, "%s: Instantiated device %s at 0x%02hx\n", "new_device",
		 info.type, info.addr);

	return count;
}
static DEVICE_ATTR(new_device, S_IWUSR, NULL, i2c_sysfs_new_device);

/*
 * And of course let the users delete the devices they instantiated, if
 * they got it wrong. This interface can only be used to delete devices
 * instantiated by i2c_sysfs_new_device above. This guarantees that we
 * don't delete devices to which some kernel code still has references.
 *
 * Parameter checking may look overzealous, but we really don't want
 * the user to delete the wrong device.
 */
 /*
 * 当然也允许用户删除他们实例化的设备，如果他们弄错了的话。
 * 这个接口只能用于删除上面i2c_sysfs_new_device实例化的设备。这保证了我们不会删除某些内核代码仍然有引用的设备。
 * 参数检查可能看起来过于热心，但我们真的不希望用户删除错误的设备。
 */
static ssize_t
i2c_sysfs_delete_device(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct i2c_adapter *adap = to_i2c_adapter(dev);
	struct i2c_client *client, *next;
	unsigned short addr;
	char end;
	int res;

	/* Parse parameters, reject extra parameters */
	// 解析参数，拒绝额外的参数
	res = sscanf(buf, "%hi%c", &addr, &end);
	if (res < 1) {
		dev_err(dev, "%s: Can't parse I2C address\n", "delete_device");
		return -EINVAL;
	}
	if (res > 1  && end != '\n') {
		dev_err(dev, "%s: Extra parameters\n", "delete_device");
		return -EINVAL;
	}

	/* Make sure the device was added through sysfs */
	// 确保设备是通过sysfs添加的
	res = -ENOENT;
	mutex_lock_nested(&adap->userspace_clients_lock,
			  i2c_adapter_depth(adap));
	list_for_each_entry_safe(client, next, &adap->userspace_clients,
				 detected) {
		if (i2c_encode_flags_to_addr(client) == addr) {
			dev_info(dev, "%s: Deleting device %s at 0x%02hx\n",
				 "delete_device", client->name, client->addr);
			// 删除适配器对他的探测关联
			list_del(&client->detected);
			// 在i2c总线上注销i2c设备
			i2c_unregister_device(client);
			res = count;
			break;
		}
	}
	mutex_unlock(&adap->userspace_clients_lock);

	if (res < 0)
		dev_err(dev, "%s: Can't find device in list\n",
			"delete_device");
	return res;
}

/*
等同于: 定义 用于导出设备属性的接口实例对象。
struct device_attribute dev_attr_delete_device ={
	.attr = {
		.name = __stringify(delete_device), 
			.mode = S_IWUSR,
			.ignore_lockdep = true 
			},
	.show		= NULL,
	.store		= i2c_sysfs_delete_device, // 允许用户删除他们实例化的设备
	};
*/
static DEVICE_ATTR_IGNORE_LOCKDEP(delete_device, S_IWUSR, NULL,
				   i2c_sysfs_delete_device);

static struct attribute *i2c_adapter_attrs[] = {
	&dev_attr_name.attr,
	&dev_attr_new_device.attr,
	&dev_attr_delete_device.attr,
	NULL
};
/*
等同于:
static const struct attribute_group i2c_adapter_group = {
	.attrs = i2c_adapter_attrs,
};
static const struct attribute_group *i2c_adapter_groups[] = {
	&i2c_adapter_group,	
	NULL,
}
note: i2c_register_adapter()接口中会使用。
*/
ATTRIBUTE_GROUPS(i2c_adapter);

struct device_type i2c_adapter_type = {
	.groups		= i2c_adapter_groups,
	.release	= i2c_adapter_dev_release,
};
EXPORT_SYMBOL_GPL(i2c_adapter_type);

/**
 * i2c_verify_adapter - return parameter as i2c_adapter or NULL
 * @dev: device, probably from some driver model iterator
 *
 * When traversing the driver model tree, perhaps using driver model
 * iterators like @device_for_each_child(), you can't assume very much
 * about the nodes you find.  Use this function to avoid oopses caused
 * by wrongly treating some non-I2C device as an i2c_adapter.
 */
/**
* i2c_verify_adapter—返回参数i2c_adapter或NULL。用于验证struct device是否是一个适配器（struct i2c_adapter）设备。
* @dev:设备，可能来自某个驱动程序模型迭代器
*
* 在遍历驱动程序模型树时，可能使用像@device_for_each_child()这样的驱动程序模型迭代器，您不能对找到的节点做太多假设。
* 使用此函数可以避免由于错误地将某些非i2c设备视为i2c_adapter而导致的oopses。
*/
struct i2c_adapter *i2c_verify_adapter(struct device *dev)
{
	return (dev->type == &i2c_adapter_type)
			? to_i2c_adapter(dev)
			: NULL;
}
EXPORT_SYMBOL(i2c_verify_adapter);

// 兼容性
#ifdef CONFIG_I2C_COMPAT
static struct class_compat *i2c_adapter_compat_class;
#endif

// （静态适配器号）将 全局链表__i2c_board_list 上的可依附到adapter上的i2c_board_info实例化并注册到其 i2c总线。
static void i2c_scan_static_board_info(struct i2c_adapter *adapter)
{
	struct i2c_devinfo	*devinfo;

	down_read(&__i2c_board_lock);
	// 遍历全局链表__i2c_board_list，
	list_for_each_entry(devinfo, &__i2c_board_list, list) {
		// 对每个可依附到adapter上的i2c_board_info实例化并注册到其 i2c总线。
		if (devinfo->busnum == adapter->nr
				&& !i2c_new_device(adapter,
						&devinfo->board_info))
			dev_err(&adapter->dev,
				"Can't create device at 0x%02x\n",
				devinfo->board_info.addr);
	}
	up_read(&__i2c_board_lock);
}


// 根据设备驱动 i2c_driver->address_list 提供的设备地址列表，在适配器和设备驱动同类的情况下，
// 完成实例化并实现将设备的到i2c总线的注册，和其与驱动driver的绑定。
static int i2c_do_add_adapter(struct i2c_driver *driver,
			      struct i2c_adapter *adap)
{
	/* Detect supported devices on that bus, and instantiate them */
	// 检测总线上受支持的设备，并实例化它们.
	i2c_detect(adap, driver);

	return 0;
}

// 同 i2c_do_add_adapter。
static int __process_new_adapter(struct device_driver *d, void *data)
{
	return i2c_do_add_adapter(to_i2c_driver(d), data);
}

static const struct i2c_lock_operations i2c_adapter_lock_ops = {
	.lock_bus =    i2c_adapter_lock_bus,
	.trylock_bus = i2c_adapter_trylock_bus,
	.unlock_bus =  i2c_adapter_unlock_bus,
};

static void i2c_host_notify_irq_teardown(struct i2c_adapter *adap)
{
	struct irq_domain *domain = adap->host_notify_domain;
	irq_hw_number_t hwirq;

	if (!domain)
		return;

	for (hwirq = 0 ; hwirq < I2C_ADDR_7BITS_COUNT ; hwirq++)
		irq_dispose_mapping(irq_find_mapping(domain, hwirq));

	irq_domain_remove(domain);
	adap->host_notify_domain = NULL;
}

// 创建或更新虚拟irq号和hw irq号之间的映射。
static int i2c_host_notify_irq_map(struct irq_domain *h,
					  unsigned int virq,
					  irq_hw_number_t hw_irq_num)
{
	irq_set_chip_and_handler(virq, &dummy_irq_chip, handle_simple_irq);

	return 0;
}

// irq_domain对象的方法
static const struct irq_domain_ops i2c_host_notify_irq_ops = {
	.map = i2c_host_notify_irq_map,
};

// 设置i2c主机通知irq域
static int i2c_setup_host_notify_irq_domain(struct i2c_adapter *adap)
{
	struct irq_domain *domain;
	// 如果i2c_adapter不支持I2C_FUNC_SMBUS_HOST_NOTIFY，返回。
	if (!i2c_check_functionality(adap, I2C_FUNC_SMBUS_HOST_NOTIFY))
		return 0;
	// 分配一个新的irq_domain数据结构.控制器支持的最大中断数=I2C_ADDR_7BITS_COUNT.
	domain = irq_domain_create_linear(adap->dev.fwnode,
					  I2C_ADDR_7BITS_COUNT,
					  &i2c_host_notify_irq_ops, adap);
	if (!domain)
		return -ENOMEM;

	adap->host_notify_domain = domain;

	return 0;
}

/**
 * i2c_handle_smbus_host_notify - Forward a Host Notify event to the correct
 * I2C client.
 * @adap: the adapter
 * @addr: the I2C address of the notifying device
 * Context: can't sleep
 *
 * Helper function to be called from an I2C bus driver's interrupt
 * handler. It will schedule the Host Notify IRQ.
 */
int i2c_handle_smbus_host_notify(struct i2c_adapter *adap, unsigned short addr)
{
	int irq;

	if (!adap)
		return -EINVAL;

	irq = irq_find_mapping(adap->host_notify_domain, addr);
	if (irq <= 0)
		return -ENXIO;

	generic_handle_irq(irq);

	return 0;
}
EXPORT_SYMBOL_GPL(i2c_handle_smbus_host_notify);

// 完成i2c_adapter到系统的注册，和相关设备实例化和到总线的注册，以及设备与依附适配驱动的绑定。
// 相关设备指：
// 1-适配器上预先声明的未实例化的设备节点。
// 2-全局链表__i2c_board_list 上的可依附到adapter上的i2c_board_info实例化并注册到其 i2c总线.
// 3-遍历总线 i2c_bus_type 上所有与适配器adap同类的设备驱动device_driver，成员 i2c_driver->address_list 提供的设备地址。
static int i2c_register_adapter(struct i2c_adapter *adap)
{
	int res = -EINVAL;

	/* Can't register until after driver model init */
	// 不能注册,直到驱动程序模型init后.
	if (WARN_ON(!is_registered)) {
		res = -EAGAIN;
		goto out_list;
	}

	/* Sanity checks */
	// 合理性检查
	if (WARN(!adap->name[0], "i2c adapter has no name"))
		goto out_list;

	if (!adap->algo) {
		pr_err("adapter '%s': no algo supplied!\n", adap->name);
		goto out_list;
	}

	if (!adap->lock_ops)
		adap->lock_ops = &i2c_adapter_lock_ops;

	adap->locked_flags = 0;
	rt_mutex_init(&adap->bus_lock);
	rt_mutex_init(&adap->mux_lock);
	mutex_init(&adap->userspace_clients_lock);
	INIT_LIST_HEAD(&adap->userspace_clients);

	/* Set default timeout to 1 second if not already set */
	// 如果尚未设置，则将默认超时设置为1秒.
	if (adap->timeout == 0)
		adap->timeout = HZ;

	/* register soft irqs for Host Notify */
	// 注册主机通知的软irq
	res = i2c_setup_host_notify_irq_domain(adap);
	if (res) {
		pr_err("adapter '%s': can't create Host Notify IRQs (%d)\n",
		       adap->name, res);
		goto out_list;
	}

	// 设置设备名称/总线类型/设备类型。
	dev_set_name(&adap->dev, "i2c-%d", adap->nr);
	adap->dev.bus = &i2c_bus_type;
	adap->dev.type = &i2c_adapter_type;
	// 将适配器注册到系统。成功返回0
	res = device_register(&adap->dev);
	if (res) {
		pr_err("adapter '%s': can't register device (%d)\n", adap->name, res);
		goto out_list;
	}

	// i2c 设置smbus警报
	res = of_i2c_setup_smbus_alert(adap);
	if (res)
		goto out_reg;

	dev_dbg(&adap->dev, "adapter [%s] registered\n", adap->name);

	pm_runtime_no_callbacks(&adap->dev);
	pm_suspend_ignore_children(&adap->dev, true);
	pm_runtime_enable(&adap->dev);

#ifdef CONFIG_I2C_COMPAT
	// 创建一个到总线设备的兼容类设备链接
	res = class_compat_create_link(i2c_adapter_compat_class, &adap->dev,
				       adap->dev.parent);
	if (res)
		dev_warn(&adap->dev,
			 "Failed to create compatibility class link\n");
#endif

	//I2C总线恢复信息:init       struct i2c_bus_recovery_info.
	i2c_init_recovery(adap);

	/* create pre-declared device nodes */
	// 创建本适配器上预先声明的未实例化的设备节点。实例化为i2c设备(i2c_client)并注册。
	of_i2c_register_devices(adap);
	// 枚举适配器上的I2C从设备
	i2c_acpi_register_devices(adap);
	i2c_acpi_install_space_handler(adap);

	// 如果本适配器的总线号<__i2c_first_dynamic_bus_num.即是一个静态分配的适配器。
	// 就将 全局链表__i2c_board_list 上的可依附到adapter上的i2c_board_info实例化并注册到其 i2c总线。
	if (adap->nr < __i2c_first_dynamic_bus_num)
		i2c_scan_static_board_info(adap);

	/* Notify drivers */
	// 通知驱动程序
	mutex_lock(&core_lock);
	// 遍历总线 i2c_bus_type 上所有的驱动device_driver，将 adap 作为参数并回调(*__process_new_adapter)处理。
	// 将adap上所有驱动成员 i2c_driver->address_list 提供的设备地址列表，在适配器和设备驱动同类的情况下，
	// 完成实例化并实现将设备的到i2c总线的注册，和其与本驱动data(driver)的绑定
	bus_for_each_drv(&i2c_bus_type, NULL, adap, __process_new_adapter);
	mutex_unlock(&core_lock);

	return 0;

out_reg:
	init_completion(&adap->dev_released);
	device_unregister(&adap->dev);
	wait_for_completion(&adap->dev_released);
out_list:
	mutex_lock(&core_lock);
	idr_remove(&i2c_adapter_idr, adap->nr);
	mutex_unlock(&core_lock);
	return res;
}

/**
 * __i2c_add_numbered_adapter - i2c_add_numbered_adapter where nr is never -1
 * @adap: the adapter to register (with adap->nr initialized)
 * Context: can sleep
 *
 * See i2c_add_numbered_adapter() for details.
 */
/**	
 * i2c_add_numbered_adapter - i2c_add_numbered_adapter，其中nr从不为-1	
 * @adap:需要注册的适配器(初始化adap->nr)	
 * 上下文:可以睡觉	 
 * 详见i2c_add_numbered_adapter()。
 */
// 给适配器授权一个id（静态分配）.完成i2c_adapter到系统的注册，和相关设备实例化和到总线的注册，以及设备与依附适配驱动的绑定.
static int __i2c_add_numbered_adapter(struct i2c_adapter *adap)
{
	int id;

	mutex_lock(&core_lock);
	// 给适配授权一个id.
	id = idr_alloc(&i2c_adapter_idr, adap, adap->nr, adap->nr + 1, GFP_KERNEL);
	mutex_unlock(&core_lock);
	if (WARN(id < 0, "couldn't get idr"))
		return id == -ENOSPC ? -EBUSY : id;

	// 完成i2c_adapter到系统的注册，和相关设备实例化和到总线的注册，以及设备与依附适配驱动的绑定。
	return i2c_register_adapter(adap);
}

/**
 * i2c_add_adapter - declare i2c adapter, use dynamic bus number
 * @adapter: the adapter to add
 * Context: can sleep
 *
 * This routine is used to declare an I2C adapter when its bus number
 * doesn't matter or when its bus number is specified by an dt alias.
 * Examples of bases when the bus number doesn't matter: I2C adapters
 * dynamically added by USB links or PCI plugin cards.
 *
 * When this returns zero, a new bus number was allocated and stored
 * in adap->nr, and the specified adapter became available for clients.
 * Otherwise, a negative errno value is returned.
 */
/**
 * i2c_add_adapter -声明i2c适配器，使用动态总线号
 * @adapter:要添加的适配器
 * 上下文:可以睡觉
 *
 * 当I2C适配器的总线号无关紧要或其总线号由dt别名指定时，此例程用于声明I2C适配器。
 * 当总线号码不重要的时候的基础的例子:I2C适配器动态地被USB连接或一种总线标准插件卡增加。
 *
 * 当返回0时，一个新的总线号被分配并存储在adap->nr中，并且指定的适配器对客户端可用。
 * 否则，将返回一个负的errno值。
 */
// 给适配分配一个id（动态分配），完成i2c_adapter到系统的注册，和相关设备实例化和到总线的注册，以及设备与依附适配驱动的绑定.
// 所分得的总线号肯会不会小于__i2c_first_dynamic_bus_num
int i2c_add_adapter(struct i2c_adapter *adapter)
{
	struct device *dev = &adapter->dev;
	int id;

	// 如果存在dev->of_node，且获取给定device_node的别名id >= 0。
	// 给适配授权一个id..后
	// 完成i2c_adapter到系统的注册，和相关设备实例化和到总线的注册，以及设备与依附适配驱动的绑定.
	if (dev->of_node) {
		// 获取给定device_node的别名id，
		id = of_alias_get_id(dev->of_node, "i2c");
		if (id >= 0) {
			adapter->nr = id;
			return __i2c_add_numbered_adapter(adapter);
		}
	}
	
	// 如果不存在dev->of_node，或者无法获取给定device_node的别名id。
	// 动态分配一个id.后。
	// 完成i2c_adapter到系统的注册，和相关设备实例化和到总线的注册，以及设备与依附适配驱动的绑定.
	mutex_lock(&core_lock);
	id = idr_alloc(&i2c_adapter_idr, adapter,
		       __i2c_first_dynamic_bus_num, 0, GFP_KERNEL);
	mutex_unlock(&core_lock);
	if (WARN(id < 0, "couldn't get idr"))
		return id;

	adapter->nr = id;
	return i2c_register_adapter(adapter);
}
EXPORT_SYMBOL(i2c_add_adapter);

/**
 * i2c_add_numbered_adapter - declare i2c adapter, use static bus number
 * @adap: the adapter to register (with adap->nr initialized)
 * Context: can sleep
 *
 * This routine is used to declare an I2C adapter when its bus number
 * matters.  For example, use it for I2C adapters from system-on-chip CPUs,
 * or otherwise built in to the system's mainboard, and where i2c_board_info
 * is used to properly configure I2C devices.
 *
 * If the requested bus number is set to -1, then this function will behave
 * identically to i2c_add_adapter, and will dynamically assign a bus number.
 *
 * If no devices have pre-been declared for this bus, then be sure to
 * register the adapter before any dynamically allocated ones.  Otherwise
 * the required bus ID may not be available.
 *
 * When this returns zero, the specified adapter became available for
 * clients using the bus number provided in adap->nr.  Also, the table
 * of I2C devices pre-declared using i2c_register_board_info() is scanned,
 * and the appropriate driver model device nodes are created.  Otherwise, a
 * negative errno value is returned.
 */
/**	
* i2c_add_numbered_adapter——声明i2c适配器，使用静态总线号	
* @adap:需要注册的适配器(初始化adap->nr)
* 上下文:可以睡觉	
* 这个例程用于在I2C适配器的总线号重要时声明它。
* 例如，将它用于来自系统片上cpu的I2C适配器，或内置于系统主板的其他适配器，并在其中使用i2c_board_info正确配置I2C设备。
* 如果请求的总线号设置为-1，那么这个函数的行为将与i2c_add_adapter相同，并将动态分配一个总线号。
* 如果没有预先为该总线声明设备，那么确保在动态分配设备之前注册适配器。否则，所需的总线ID可能不可用。
* 当返回0时，使用adap->nr中提供的总线号为客户端提供指定的适配器。
* 另外，使用i2c_register_board_info()预先声明的I2C设备表将被扫描，并创建适当的驱动程序模型设备节点。
* 否则，将返回一个负的errno值。	
*/
// 自动（静态或者动态）给适配分配一个id，完成i2c_adapter到系统的注册，和相关设备实例化和到总线的注册，以及设备与依附适配驱动的绑定.
int i2c_add_numbered_adapter(struct i2c_adapter *adap)
{
	if (adap->nr == -1) /* -1 means dynamically assign bus id */  // -1表示动态分配总线id
		return i2c_add_adapter(adap);

	// 否则静态分配
	return __i2c_add_numbered_adapter(adap);
}
EXPORT_SYMBOL_GPL(i2c_add_numbered_adapter);

// 从系统中注销i2c_driver中依附于adapter的client。
static void i2c_do_del_adapter(struct i2c_driver *driver,
			      struct i2c_adapter *adapter)
{
	struct i2c_client *client, *_n;

	/* Remove the devices we created ourselves as the result of hardware
	 * probing (using a driver's detect method) */
	// 删除我们自己创建的硬件探测设备(使用驱动程序的检测方法)
	// 遍历设备驱动上的依附于adapter的 client。
	list_for_each_entry_safe(client, _n, &driver->clients, detected) {
		if (client->adapter == adapter) {
			dev_dbg(&adapter->dev, "Removing %s at 0x%x\n",
				client->name, client->addr);
			// 删除设备与驱动的绑定。
			list_del(&client->detected);
			// 从系统中注销client
			i2c_unregister_device(client);
		}
	}
}

// 在系统中注销名dummy为的设备dev。（"dummy" 设备不可注销。）
static int __unregister_client(struct device *dev, void *dummy)
{
	struct i2c_client *client = i2c_verify_client(dev);
	// "dummy" 设备不可注销。
	if (client && strcmp(client->name, "dummy"))
		i2c_unregister_device(client);
	return 0;
}

// 在系统中注销名dummy为的设备dev。（"dummy" 设备可注销。）
static int __unregister_dummy(struct device *dev, void *dummy)
{
	struct i2c_client *client = i2c_verify_client(dev);
	i2c_unregister_device(client);
	return 0;
}

// 从系统中注销i2c_driver中依附于data(adapter)的所有client。
static int __process_removed_adapter(struct device_driver *d, void *data)
{
	i2c_do_del_adapter(to_i2c_driver(d), data);
	return 0;
}

/**
 * i2c_del_adapter - unregister I2C adapter
 * @adap: the adapter being unregistered
 * Context: can sleep
 *
 * This unregisters an I2C adapter which was previously registered
 * by @i2c_add_adapter or @i2c_add_numbered_adapter.
 */
/**
* i2c_del_adapter -取消注册I2C适配器	
* @adap:适配器未注册
* 上下文:可以睡觉
*
* 这将注销之前由@i2c_add_adapter或@i2c_add_numbered_adapter注册的I2C适配器。
*/
void i2c_del_adapter(struct i2c_adapter *adap)
{
	struct i2c_adapter *found;
	struct i2c_client *client, *next;

	/* First make sure that this adapter was ever added */
	// 首先，确保已经添加了这个适配器
	mutex_lock(&core_lock);
	found = idr_find(&i2c_adapter_idr, adap->nr);
	mutex_unlock(&core_lock);
	if (found != adap) {
		pr_debug("attempting to delete unregistered adapter [%s]\n", adap->name);
		return;
	}

	i2c_acpi_remove_space_handler(adap);
	/* Tell drivers about this removal */
	// 告诉adap上每个drivers这个迁移.从系统中注销每个i2c_driver中依附于data(adapter)的所有client。
	mutex_lock(&core_lock);
	bus_for_each_drv(&i2c_bus_type, NULL, adap,
			       __process_removed_adapter);
	mutex_unlock(&core_lock);

	/* Remove devices instantiated from sysfs */
	// 从sysfs中删除实例化的设备
	mutex_lock_nested(&adap->userspace_clients_lock,
			  i2c_adapter_depth(adap));
	list_for_each_entry_safe(client, next, &adap->userspace_clients,
				 detected) {
		dev_dbg(&adap->dev, "Removing %s at 0x%x\n", client->name,
			client->addr);
		list_del(&client->detected);
		i2c_unregister_device(client);
	}
	mutex_unlock(&adap->userspace_clients_lock);

	/* Detach any active clients. This can't fail, thus we do not
	 * check the returned value. This is a two-pass process, because
	 * we can't remove the dummy devices during the first pass: they
	 * could have been instantiated by real devices wishing to clean
	 * them up properly, so we give them a chance to do that first. */
	device_for_each_child(&adap->dev, NULL, __unregister_client);
	device_for_each_child(&adap->dev, NULL, __unregister_dummy);

#ifdef CONFIG_I2C_COMPAT
	class_compat_remove_link(i2c_adapter_compat_class, &adap->dev,
				 adap->dev.parent);
#endif

	/* device name is gone after device_unregister */
    // 设备名在device_unregister之后消失
	dev_dbg(&adap->dev, "adapter [%s] unregistered\n", adap->name);

	pm_runtime_disable(&adap->dev);

	i2c_host_notify_irq_teardown(adap);

	/* wait until all references to the device are gone
	 *
	 * FIXME: This is old code and should ideally be replaced by an
	 * alternative which results in decoupling the lifetime of the struct
	 * device from the i2c_adapter, like spi or netdev do. Any solution
	 * should be thoroughly tested with DEBUG_KOBJECT_RELEASE enabled!
	 */
	// 等待，直到对设备的所有引用都消失
	init_completion(&adap->dev_released);
	device_unregister(&adap->dev);
	wait_for_completion(&adap->dev_released);

	/* free bus id */
	// 注销 bus id
	mutex_lock(&core_lock);
	idr_remove(&i2c_adapter_idr, adap->nr);
	mutex_unlock(&core_lock);

	/* Clear the device structure in case this adapter is ever going to be
	   added again */
	// 清除设备结构，以防再次添加此适配器
	memset(&adap->dev, 0, sizeof(adap->dev));
}
EXPORT_SYMBOL(i2c_del_adapter);

/**
 * i2c_parse_fw_timings - get I2C related timing parameters from firmware
 * @dev: The device to scan for I2C timing properties
 * @t: the i2c_timings struct to be filled with values
 * @use_defaults: bool to use sane defaults derived from the I2C specification
 *		  when properties are not found, otherwise use 0
 *
 * Scan the device for the generic I2C properties describing timing parameters
 * for the signal and fill the given struct with the results. If a property was
 * not found and use_defaults was true, then maximum timings are assumed which
 * are derived from the I2C specification. If use_defaults is not used, the
 * results will be 0, so drivers can apply their own defaults later. The latter
 * is mainly intended for avoiding regressions of existing drivers which want
 * to switch to this function. New drivers almost always should use the defaults.
 */
/**
* i2c_parse_fw_timings—从固件获取I2C相关的定时参数
* @dev:扫描I2C定时属性的设备
* @t:要填充值的i2c_timings结构
* @use_defaults: bool在没有找到属性时使用来自I2C规范的sane默认值，否则使用0	
* 扫描设备，查找描述信号定时参数的通用I2C属性，并将结果填充到给定结构中。
* 如果没有找到属性，并且use_defaults为真，那么就会假定I2C规范中规定的最大时间。
* 如果没有使用use_defaults，那么结果将是0，因此驱动程序可以在以后应用它们自己的默认值。
* 后者主要是为了避免现有驱动程序想要切换到这个功能的回归。新驱动几乎总是应该使用默认。	
*/
void i2c_parse_fw_timings(struct device *dev, struct i2c_timings *t, bool use_defaults)
{
	int ret;

	memset(t, 0, sizeof(*t));

	ret = device_property_read_u32(dev, "clock-frequency", &t->bus_freq_hz);
	if (ret && use_defaults)
		t->bus_freq_hz = 100000;

	ret = device_property_read_u32(dev, "i2c-scl-rising-time-ns", &t->scl_rise_ns);
	if (ret && use_defaults) {
		if (t->bus_freq_hz <= 100000)
			t->scl_rise_ns = 1000;
		else if (t->bus_freq_hz <= 400000)
			t->scl_rise_ns = 300;
		else
			t->scl_rise_ns = 120;
	}

	ret = device_property_read_u32(dev, "i2c-scl-falling-time-ns", &t->scl_fall_ns);
	if (ret && use_defaults) {
		if (t->bus_freq_hz <= 400000)
			t->scl_fall_ns = 300;
		else
			t->scl_fall_ns = 120;
	}

	device_property_read_u32(dev, "i2c-scl-internal-delay-ns", &t->scl_int_delay_ns);

	ret = device_property_read_u32(dev, "i2c-sda-falling-time-ns", &t->sda_fall_ns);
	if (ret && use_defaults)
		t->sda_fall_ns = t->scl_fall_ns;

	device_property_read_u32(dev, "i2c-sda-hold-time-ns", &t->sda_hold_ns);
}
EXPORT_SYMBOL_GPL(i2c_parse_fw_timings);

/* ------------------------------------------------------------------------- */

// 遍历i2c总线上所有的设备 device，将data作为参数回调fn。
int i2c_for_each_dev(void *data, int (*fn)(struct device *dev, void *data))
{
	int res;

	mutex_lock(&core_lock);
	res = bus_for_each_dev(&i2c_bus_type, NULL, data, fn);
	mutex_unlock(&core_lock);

	return res;
}
EXPORT_SYMBOL_GPL(i2c_for_each_dev);


// 若dev是个适配器的话：
// 根据设备驱动 i2c_driver->address_list 提供的设备地址列表，在适配器和设备驱动同类的情况下，
// 完成驱动地址列表设备实例化与适配器的依附，并实现将设备的到i2c总线的注册，和其与驱动driver(data)的绑定。
static int __process_new_driver(struct device *dev, void *data)
{
	if (dev->type != &i2c_adapter_type)  // I2C_ADAPTER_VGADDC
		return 0;
	return i2c_do_add_adapter(data, to_i2c_adapter(dev));
}

/*
 * An i2c_driver is used with one or more i2c_client (device) nodes to access
 * i2c slave chips, on a bus instance associated with some i2c_adapter.
 */
/*
 * i2c_driver与一个或多个i2c_client(设备)节点一起使用，在与某个i2c_adapter关联的总线实例上访问i2c从芯片。	
 */
//实例化driver地址列表中的从设备，注册到与本driver同类型的总线上所有适配器后，并完成与本driver绑定。
int i2c_register_driver(struct module *owner, struct i2c_driver *driver)
{
	int res;

	/* Can't register until after driver model init */
	// 检查i2c总线是否已经注册，没有就退出。
	if (WARN_ON(!is_registered))
		return -EAGAIN;

	/* add the driver to the list of i2c drivers in the driver core */
	// 将驱动程序添加到驱动程序核心的i2c驱动程序列表中
	driver->driver.owner = owner;
	driver->driver.bus = &i2c_bus_type;
	INIT_LIST_HEAD(&driver->clients);

	/* When registration returns, the driver core
	 * will have called probe() for all matching-but-unbound devices.
	 */
	// 当注册返回时，驱动核心为本驱动所在的适配器上所有匹配但未绑定的设备调用probe()。完成设备与本驱动的绑定。
	res = driver_register(&driver->driver);
	if (res)
		return res;

	pr_debug("driver [%s] registered\n", driver->driver.name);

	/* Walk the adapters that are already present */
	// 遍历总线上的已经存在的适配器（一个总线上可能有多个适配器）,若与本驱动同类。
	// 完成设备驱动driver中地址列表设备实例化与此适配器的依附，并实现将从设备的到i2c总线的注册，和其与驱动driver的绑定。
	i2c_for_each_dev(driver, __process_new_driver);

	return 0;
}
EXPORT_SYMBOL(i2c_register_driver);

static int __process_removed_driver(struct device *dev, void *data)
{
	if (dev->type == &i2c_adapter_type)
		i2c_do_del_adapter(data, to_i2c_adapter(dev));
	return 0;
}

/**
 * i2c_del_driver - unregister I2C driver
 * @driver: the driver being unregistered
 * Context: can sleep
 */
void i2c_del_driver(struct i2c_driver *driver)
{
	i2c_for_each_dev(driver, __process_removed_driver);

	driver_unregister(&driver->driver);
	pr_debug("driver [%s] unregistered\n", driver->driver.name);
}
EXPORT_SYMBOL(i2c_del_driver);

/* ------------------------------------------------------------------------- */

/**
 * i2c_use_client - increments the reference count of the i2c client structure
 * @client: the client being referenced
 *
 * Each live reference to a client should be refcounted. The driver model does
 * that automatically as part of driver binding, so that most drivers don't
 * need to do this explicitly: they hold a reference until they're unbound
 * from the device.
 *
 * A pointer to the client with the incremented reference counter is returned.
 */
/**
* i2c_use_client—增加i2c客户端结构的引用计数
* @client:被引用的客户端
* 应重新计数对客户端的每个活引用。驱动程序模型自动将其作为驱动程序绑定的一部分，
* 因此大多数驱动程序不需要显式地这样做:它们持有一个引用，直到它们从设备上解除绑定。
* 返回一个指向带有递增引用计数器的客户端的指针。
*/
struct i2c_client *i2c_use_client(struct i2c_client *client)
{
	if (client && get_device(&client->dev))
		return client;
	return NULL;
}
EXPORT_SYMBOL(i2c_use_client);

/**
 * i2c_release_client - release a use of the i2c client structure
 * @client: the client being no longer referenced
 *
 * Must be called when a user of a client is finished with it.
 */
/**	
* i2c_release_client -释放i2c客户端结构
* @client:不再被引用的客户端	
* 必须在客户端用户使用完毕时调用。
*/
void i2c_release_client(struct i2c_client *client)
{
	if (client)
		put_device(&client->dev);
}
EXPORT_SYMBOL(i2c_release_client);

struct i2c_cmd_arg {
	unsigned	cmd;
	void		*arg;
};

//i2c从设备绑定的驱动执行设备的特定功能
static int i2c_cmd(struct device *dev, void *_arg)
{
	struct i2c_client	*client = i2c_verify_client(dev);
	struct i2c_cmd_arg	*arg = _arg;
	struct i2c_driver	*driver;

	if (!client || !client->dev.driver)
		return 0;

	driver = to_i2c_driver(client->dev.driver);
	if (driver->command)
		driver->command(client, arg->cmd, arg->arg);
	return 0;
}

// 让本适配器上的所有从设备均绑定的驱动执行设备的特定功能
void i2c_clients_command(struct i2c_adapter *adap, unsigned int cmd, void *arg)
{
	struct i2c_cmd_arg	cmd_arg;

	cmd_arg.cmd = cmd;
	cmd_arg.arg = arg;
	// 遍历本适配器的上孩子（从设备），i2c从设备绑定的驱动执行设备的特定功能
	device_for_each_child(&adap->dev, &cmd_arg, i2c_cmd);
}
EXPORT_SYMBOL(i2c_clients_command);

// i2c 子系统初始化
static int __init i2c_init(void)
{
	int retval;

	// 获得 aliases节点中的别名属性的最大id值
	retval = of_alias_get_highest_id("i2c");

	// 操作写信号量
	down_write(&__i2c_board_lock);
	// 更新   __i2c_first_dynamic_bus_num
	if (retval >= __i2c_first_dynamic_bus_num)
		__i2c_first_dynamic_bus_num = retval + 1;
	up_write(&__i2c_board_lock);

	// i2c 总线注册。
	retval = bus_register(&i2c_bus_type);
	if (retval)
		return retval;

	is_registered = true;

#ifdef CONFIG_I2C_COMPAT
	i2c_adapter_compat_class = class_compat_register("i2c-adapter");
	if (!i2c_adapter_compat_class) {
		retval = -ENOMEM;
		goto bus_err;
	}
#endif
	// 注册一个虚拟的i2c（逻辑）设备驱动。
	retval = i2c_add_driver(&dummy_driver);
	if (retval)
		goto class_err;

	if (IS_ENABLED(CONFIG_OF_DYNAMIC))
		WARN_ON(of_reconfig_notifier_register(&i2c_of_notifier));
	if (IS_ENABLED(CONFIG_ACPI))
		WARN_ON(acpi_reconfig_notifier_register(&i2c_acpi_notifier));

	return 0;

class_err:
#ifdef CONFIG_I2C_COMPAT
	class_compat_unregister(i2c_adapter_compat_class);
bus_err:
#endif
	is_registered = false;
	bus_unregister(&i2c_bus_type);
	return retval;
}

static void __exit i2c_exit(void)
{
	if (IS_ENABLED(CONFIG_ACPI))
		WARN_ON(acpi_reconfig_notifier_unregister(&i2c_acpi_notifier));
	if (IS_ENABLED(CONFIG_OF_DYNAMIC))
		WARN_ON(of_reconfig_notifier_unregister(&i2c_of_notifier));
	i2c_del_driver(&dummy_driver);
#ifdef CONFIG_I2C_COMPAT
	class_compat_unregister(i2c_adapter_compat_class);
#endif
	bus_unregister(&i2c_bus_type);
	tracepoint_synchronize_unregister();
}

/* We must initialize early, because some subsystems register i2c drivers
 * in subsys_initcall() code, but are linked (and initialized) before i2c.
 */
/*
* 我们必须尽早初始化，因为一些子系统在 subsys_initcall() 代码中注册了i2c驱动程序，
* 但是在i2c之前就被链接(并初始化)了。
*/
postcore_initcall(i2c_init);
module_exit(i2c_exit);

/* ----------------------------------------------------
 * the functional interface to the i2c busses.  i2c总线的功能接口。
 * ----------------------------------------------------
 */

/* Check if val is exceeding the quirk IFF quirk is non 0 */
// 检查val是否超过了quirk，如果quirk非0
#define i2c_quirk_exceeded(val, quirk) ((quirk) && ((val) > (quirk)))

static int i2c_quirk_error(struct i2c_adapter *adap, struct i2c_msg *msg, char *err_msg)
{
	dev_err_ratelimited(&adap->dev, "adapter quirk: %s (addr 0x%04x, size %u, %s)\n",
			    err_msg, msg->addr, msg->len,
			    msg->flags & I2C_M_RD ? "read" : "write");
	return -EOPNOTSUPP;
}

// i2c 总线对发送消息的缺陷性检查。
static int i2c_check_for_quirks(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	const struct i2c_adapter_quirks *q = adap->quirks;
	int max_num = q->max_num_msgs, i;
	bool do_len_check = true;

	if (q->flags & I2C_AQ_COMB) {
		max_num = 2;

		/* special checks for combined messages */
        // 合并消息的特殊检查
		if (num == 2) {
			if (q->flags & I2C_AQ_COMB_WRITE_FIRST && msgs[0].flags & I2C_M_RD)
				return i2c_quirk_error(adap, &msgs[0], "1st comb msg must be write");

			if (q->flags & I2C_AQ_COMB_READ_SECOND && !(msgs[1].flags & I2C_M_RD))
				return i2c_quirk_error(adap, &msgs[1], "2nd comb msg must be read");

			if (q->flags & I2C_AQ_COMB_SAME_ADDR && msgs[0].addr != msgs[1].addr)
				return i2c_quirk_error(adap, &msgs[0], "comb msg only to same addr");

			if (i2c_quirk_exceeded(msgs[0].len, q->max_comb_1st_msg_len))
				return i2c_quirk_error(adap, &msgs[0], "msg too long");

			if (i2c_quirk_exceeded(msgs[1].len, q->max_comb_2nd_msg_len))
				return i2c_quirk_error(adap, &msgs[1], "msg too long");

			do_len_check = false;
		}
	}

	if (i2c_quirk_exceeded(num, max_num))
		return i2c_quirk_error(adap, &msgs[0], "too many messages");

	for (i = 0; i < num; i++) {
		u16 len = msgs[i].len;

		if (msgs[i].flags & I2C_M_RD) {
			if (do_len_check && i2c_quirk_exceeded(len, q->max_read_len))
				return i2c_quirk_error(adap, &msgs[i], "msg too long");

			if (q->flags & I2C_AQ_NO_ZERO_LEN_READ && len == 0)
				return i2c_quirk_error(adap, &msgs[i], "no zero length");
		} else {
			if (do_len_check && i2c_quirk_exceeded(len, q->max_write_len))
				return i2c_quirk_error(adap, &msgs[i], "msg too long");

			if (q->flags & I2C_AQ_NO_ZERO_LEN_WRITE && len == 0)
				return i2c_quirk_error(adap, &msgs[i], "no zero length");
		}
	}

	return 0;
}

/**
 * __i2c_transfer - unlocked flavor of i2c_transfer
 * @adap: Handle to I2C bus
 * @msgs: One or more messages to execute before STOP is issued to
 *	terminate the operation; each message begins with a START.
 * @num: Number of messages to be executed.
 *
 * Returns negative errno, else the number of messages executed.
 *
 * Adapter lock must be held when calling this function. No debug logging
 * takes place. adap->algo->master_xfer existence isn't checked.
 */
/**
* i2c_transfer -  被i2c_transfer（）调用。
* @adap: I2C总线的句柄
* @msgs:在发出终止操作的STOP之前，需要执行一条或多条消息;每条消息都以开头开头。
* @num:要执行的消息数量。
*
* 返回负的errno，否则执行的消息数。
*	
* 调用此函数时必须持有适配器锁。不进行调试日志记录。adap->算法->master_xfer存在性没有检查。
*/
// 在适配器上发送num个消息
int __i2c_transfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	unsigned long orig_jiffies;
	int ret, try;

	if (WARN_ON(!msgs || num < 1))
		return -EINVAL;
	// 检查总线是否 suspend,是退出。
	ret = __i2c_check_suspended(adap);
	if (ret)
		return ret;

	if (adap->quirks && i2c_check_for_quirks(adap, msgs, num))
		return -EOPNOTSUPP;

	/*
	 * i2c_trace_msg_key gets enabled when tracepoint i2c_transfer gets
	 * enabled.  This is an efficient way of keeping the for-loop from
	 * being executed when not needed.
	 */
	//  i2c_trace_msg_key在tracepoint i2c_transfer启用时启用。这是一种在不需要时避免执行for循环的有效方法。
	if (static_branch_unlikely(&i2c_trace_msg_key)) {
		int i;
		for (i = 0; i < num; i++)
			if (msgs[i].flags & I2C_M_RD)
				trace_i2c_read(adap, &msgs[i], i);
			else
				trace_i2c_write(adap, &msgs[i], i);
	}

	/* Retry automatically on arbitration loss */
	// 仲裁损失则自动重试
	orig_jiffies = jiffies;
	for (ret = 0, try = 0; try <= adap->retries; try++) {
		// 回调总线指定的传输方法。
		if (i2c_in_atomic_xfer_mode() && adap->algo->master_xfer_atomic)
			ret = adap->algo->master_xfer_atomic(adap, msgs, num);
		else
			ret = adap->algo->master_xfer(adap, msgs, num);

		if (ret != -EAGAIN)
			break;
		if (time_after(jiffies, orig_jiffies + adap->timeout))
			break;
	}

	if (static_branch_unlikely(&i2c_trace_msg_key)) {
		int i;
		for (i = 0; i < ret; i++)
			if (msgs[i].flags & I2C_M_RD)
				trace_i2c_reply(adap, &msgs[i], i);
		trace_i2c_result(adap, num, ret);
	}

	return ret;
}
EXPORT_SYMBOL(__i2c_transfer);

/**
 * i2c_transfer - execute a single or combined I2C message
 * @adap: Handle to I2C bus
 * @msgs: One or more messages to execute before STOP is issued to
 *	terminate the operation; each message begins with a START.
 * @num: Number of messages to be executed.
 *
 * Returns negative errno, else the number of messages executed.
 *
 * Note that there is no requirement that each message be sent to
 * the same slave address, although that is the most common model.
 */
/**
* i2c_transfer—执行单个或组合的I2C消息
* @adap: I2C总线的句柄
* @msgs:在发出终止操作的STOP之前，需要执行一条或多条消息;每条消息都以开头开头。
* @num:要执行的消息数量。
*
* 返回负的errno，否则执行的消息数。
*
* 请注意，没有要求每个消息都发送到相同的从属地址，尽管这是最常见的模型。
*/
int i2c_transfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	int ret;

	if (!adap->algo->master_xfer) {
		dev_dbg(&adap->dev, "I2C level transfers not supported\n");
		return -EOPNOTSUPP;
	}

	/* REVISIT the fault reporting model here is weak:
	 *
	 *  - When we get an error after receiving N bytes from a slave,
	 *    there is no way to report "N".
	 *
	 *  - When we get a NAK after transmitting N bytes to a slave,
	 *    there is no way to report "N" ... or to let the master
	 *    continue executing the rest of this combined message, if
	 *    that's the appropriate response.
	 *
	 *  - When for example "num" is two and we successfully complete
	 *    the first message but get an error part way through the
	 *    second, it's unclear whether that should be reported as
	 *    one (discarding status on the second message) or errno
	 *    (discarding status on the first one).
	 */
/*这里的故障报告模型比较弱:
* -当我们收到一个错误后，从一个slave的N字节，没有办法报告“N”。
* -当我们得到一个NAK后，传输N个字节给一个slave，没有办法报告“N”…或者
* 让主进程继续执行组合消息的其余部分(如果这是适当的响应)。
* -例如，当“num”为2时，我们成功地完成了第一个消息，但在第二个消息中出现了部分错误，
* 不清楚应该将其报告为1(第二个消息的丢弃状态)还是errno(第一个消息的丢弃状态)。
*/
	ret = __i2c_lock_bus_helper(adap);
	if (ret)
		return ret;

	ret = __i2c_transfer(adap, msgs, num);
	i2c_unlock_bus(adap, I2C_LOCK_SEGMENT);

	return ret;
}
EXPORT_SYMBOL(i2c_transfer);

/**
 * i2c_transfer_buffer_flags - issue a single I2C message transferring data
 *			       to/from a buffer
 * @client: Handle to slave device
 * @buf: Where the data is stored
 * @count: How many bytes to transfer, must be less than 64k since msg.len is u16
 * @flags: The flags to be used for the message, e.g. I2C_M_RD for reads
 *
 * Returns negative errno, or else the number of bytes transferred.
 */
/**
* i2c_transfer_buffer_flags—发出一条I2C消息，将数据传输到缓冲区或从缓冲区返回
* @client:从设备的句柄
* @buf:数据存储的地方
* @count:要传输多少字节，必须小于64k，因为msg。len是u16
* @flags:用于消息的标志，例如 I2C_M_RD 用于读取
*
*返回负的errno，或传输的字节数。
*/
// f根据lags，读写从设备的一个数据消息
int i2c_transfer_buffer_flags(const struct i2c_client *client, char *buf,
			      int count, u16 flags)
{
	int ret;
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = flags | (client->flags & I2C_M_TEN),
		.len = count,
		.buf = buf,
	};

	ret = i2c_transfer(client->adapter, &msg, 1);

	/*
	 * If everything went ok (i.e. 1 msg transferred), return #bytes
	 * transferred, else error code.
	 */
	// 如果一切正常(即1 msg传输)，返回#字节传输，否则错误代码。
	return (ret == 1) ? count : ret;
}
EXPORT_SYMBOL(i2c_transfer_buffer_flags);

/**
 * i2c_get_device_id - get manufacturer, part id and die revision of a device
 * @client: The device to query
 * @id: The queried information
 *
 * Returns negative errno on error, zero on success.
 */
/**	
* i2c_get_device_id -获取设备的制造商、部件id和模具修订	
* @client:要查询的设备	
* @id:查询的信息	
*	
* 错误返回负的errno，成功返回0。	
*/
int i2c_get_device_id(const struct i2c_client *client,
		      struct i2c_device_identity *id)
{
	struct i2c_adapter *adap = client->adapter;
	union i2c_smbus_data raw_id;
	int ret;
	// 适配器支持我们需要的 I2C_FUNC_SMBUS_READ_I2C_BLOCK,若不支持退出。
	if (!i2c_check_functionality(adap, I2C_FUNC_SMBUS_READ_I2C_BLOCK))
		return -EOPNOTSUPP;

	raw_id.block[0] = 3;
	// 执行SMBus协议操作
	ret = i2c_smbus_xfer(adap, I2C_ADDR_DEVICE_ID, 0,
			     I2C_SMBUS_READ, client->addr << 1,
			     I2C_SMBUS_I2C_BLOCK_DATA, &raw_id);
	if (ret)
		return ret;

	id->manufacturer_id = (raw_id.block[1] << 4) | (raw_id.block[2] >> 4);
	id->part_id = ((raw_id.block[2] & 0xf) << 5) | (raw_id.block[3] >> 3);
	id->die_revision = raw_id.block[3] & 0x7;
	return 0;
}
EXPORT_SYMBOL_GPL(i2c_get_device_id);

/* ----------------------------------------------------
 * the i2c address scanning function
 * Will not work for 10-bit addresses!
 * ----------------------------------------------------
 */

/*
 * Legacy default probe function, mostly relevant for SMBus. The default
 * probe method is a quick write, but it is known to corrupt the 24RF08
 * EEPROMs due to a state machine bug, and could also irreversibly
 * write-protect some EEPROMs, so for address ranges 0x30-0x37 and 0x50-0x5f,
 * we use a short byte read instead. Also, some bus drivers don't implement
 * quick write, so we fallback to a byte read in that case too.
 * On x86, there is another special case for FSC hardware monitoring chips,
 * which want regular byte reads (address 0x73.) Fortunately, these are the
 * only known chips using this I2C address on PC hardware.
 * Returns 1 if probe succeeded, 0 if not.
 */
/*
* 遗留的默认探测功能，主要与SMBus相关。
* 默认的探测方法是快速写操作，但是由于一个状态机错误，它会破坏24RF08 eeprom，并且还可能不可逆转地写保护一些eeprom，
* 所以对于地址范围0x30-0x37和0x50-0x5f，我们使用一个短字节读取。
* 另外，一些总线驱动程序没有实现快速写入，所以在这种情况下我们也会回退到字节读取。
* 在x86上，FSC硬件监控芯片还有另一种特殊情况，需要定期读取字节(地址0x73)。
* 幸运的是，这些是唯一已知的在PC硬件上使用这个I2C地址的芯片。
* 如果探测成功，返回1;如果失败，返回0。
*/
static int i2c_default_probe(struct i2c_adapter *adap, unsigned short addr)
{
	int err;
	union i2c_smbus_data dummy;

#ifdef CONFIG_X86
	if (addr == 0x73 && (adap->class & I2C_CLASS_HWMON)
	 && i2c_check_functionality(adap, I2C_FUNC_SMBUS_READ_BYTE_DATA))
		err = i2c_smbus_xfer(adap, addr, 0, I2C_SMBUS_READ, 0,
				     I2C_SMBUS_BYTE_DATA, &dummy);
	else
#endif
	if (!((addr & ~0x07) == 0x30 || (addr & ~0x0f) == 0x50)
	 && i2c_check_functionality(adap, I2C_FUNC_SMBUS_QUICK))
		err = i2c_smbus_xfer(adap, addr, 0, I2C_SMBUS_WRITE, 0,
				     I2C_SMBUS_QUICK, NULL);
	else if (i2c_check_functionality(adap, I2C_FUNC_SMBUS_READ_BYTE))
		err = i2c_smbus_xfer(adap, addr, 0, I2C_SMBUS_READ, 0,
				     I2C_SMBUS_BYTE, &dummy);
	else {
		dev_warn(&adap->dev, "No suitable probing method supported for address 0x%02X\n",
			 addr);
		err = -EOPNOTSUPP;
	}

	return err >= 0;
}

// 根据i2c_client设备的adapter和addr成员信息完成设备的到i2c总线的注册，和与驱动driver的绑定。
static int i2c_detect_address(struct i2c_client *temp_client,
			      struct i2c_driver *driver)
{
	struct i2c_board_info info;
	struct i2c_adapter *adapter = temp_client->adapter;
	int addr = temp_client->addr;
	int err;

	/* Make sure the address is valid */
	// 确保地址是有效的
	err = i2c_check_7bit_addr_validity_strict(addr);
	if (err) {
		dev_warn(&adapter->dev, "Invalid probe address 0x%02x\n",
			 addr);
		return err;
	}

	/* Skip if already in use (7 bit, no need to encode flags) */
	// 如果已经使用则退出(7位，不需要编码标记)
	if (i2c_check_addr_busy(adapter, addr))
		return 0;

	/* Make sure there is something at this address */
	// 这个地址一定有什么东西.确实挂接了可以正常工作的一个设备。
	if (!i2c_default_probe(adapter, addr))
		return 0;

	/* Finally call the custom detection function */
	// 最后调用自定义检测函数
	memset(&info, 0, sizeof(struct i2c_board_info));
	info.addr = addr;
	//回调设备驱动实现的探测方法。告知驱动已经探测到的设备地址。
	err = driver->detect(temp_client, &info);
	if (err) {
		/* -ENODEV is returned if the detection fails. We catch it
		   here as this isn't an error. */
		return err == -ENODEV ? 0 : err;
	}

	/* Consistency check */
	// 一致性检验
	if (info.type[0] == '\0') {
		// 检测失败。
		dev_err(&adapter->dev,
			"%s detection function provided no name for 0x%x\n",
			driver->driver.name, addr);
	} else {
		struct i2c_client *client;

		/* Detection succeeded, instantiate the device */
		// 检测成功，实例化设备
		if (adapter->class & I2C_CLASS_DEPRECATED)
			dev_warn(&adapter->dev,
				"This adapter will soon drop class based instantiation of devices. "
				"Please make sure client 0x%02x gets instantiated by other means. "
				"Check 'Documentation/i2c/instantiating-devices' for details.\n",
				info.addr);

		dev_dbg(&adapter->dev, "Creating %s at 0x%02x\n",
			info.type, info.addr);
		// 通过 i2c_board_info 实例化一个i2c设备，依附适配器adap后并注册到i2c总线。
		client = i2c_new_device(adapter, &info);
		if (client)
			// 完成设备到驱动的绑定。
			list_add_tail(&client->detected, &driver->clients);
		else
			dev_err(&adapter->dev, "Failed creating %s at 0x%02x\n",
				info.type, info.addr);
	}
	return 0;
}

// 根据设备驱动 i2c_driver->address_list 提供的设备地址列表，在适配器和设备驱动同类的情况下，
// 完成实例化并实现将设备的到i2c总线的注册，和其与驱动driver的绑定。
static int i2c_detect(struct i2c_adapter *adapter, struct i2c_driver *driver)
{
	const unsigned short *address_list;
	struct i2c_client *temp_client;
	int i, err = 0;
	// 获得特定适配器的适配器号
	int adap_id = i2c_adapter_id(adapter);

	address_list = driver->address_list;
	if (!driver->detect || !address_list)
		return 0;

	/* Warn that the adapter lost class based instantiation */
	// 警告适配器丢失了基于类的实例化
	if (adapter->class == I2C_CLASS_DEPRECATED) {
		dev_dbg(&adapter->dev,
			"This adapter dropped support for I2C classes and won't auto-detect %s devices anymore. "
			"If you need it, check 'Documentation/i2c/instantiating-devices' for alternatives.\n",
			driver->driver.name);
		return 0;
	}

	/* Stop here if the classes do not match */
	// 如果类不匹配，请在此停止
	if (!(adapter->class & driver->class))
		return 0;

	/* Set up a temporary client to help detect callback */
	// 设置一个临时客户端来帮助检测回调.
	temp_client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (!temp_client)
		return -ENOMEM;
	temp_client->adapter = adapter;

	for (i = 0; address_list[i] != I2C_CLIENT_END; i += 1) {
		dev_dbg(&adapter->dev,
			"found normal entry for adapter %d, addr 0x%02x\n",
			adap_id, address_list[i]);
		temp_client->addr = address_list[i];
		// 根据i2c_client设备的adapter和addr成员信息完成设备的到i2c总线的注册，和与驱动driver的绑定。
		err = i2c_detect_address(temp_client, driver);
		if (unlikely(err))
			break;
	}

	kfree(temp_client);
	return err;
}

int i2c_probe_func_quick_read(struct i2c_adapter *adap, unsigned short addr)
{
	return i2c_smbus_xfer(adap, addr, 0, I2C_SMBUS_READ, 0,
			      I2C_SMBUS_QUICK, NULL) >= 0;
}
EXPORT_SYMBOL_GPL(i2c_probe_func_quick_read);

/*
* 如果你不知道I2C设备的确切地址，可以使用这个变体，它可以探测可能的地址列表中的设备是否存在。
* “探针”回调函数是可选的。如果提供了它，则在探测成功时必须返回1，否则返回0。
* 如果没有提供，则使用默认探测方法。
*/
struct i2c_client *
i2c_new_probed_device(struct i2c_adapter *adap,
		      struct i2c_board_info *info,
		      unsigned short const *addr_list,
		      int (*probe)(struct i2c_adapter *adap, unsigned short addr))
{
	int i;

	if (!probe)
		probe = i2c_default_probe;

	for (i = 0; addr_list[i] != I2C_CLIENT_END; i++) {
		/* Check address validity */
		if (i2c_check_7bit_addr_validity_strict(addr_list[i]) < 0) {
			dev_warn(&adap->dev, "Invalid 7-bit address 0x%02x\n",
				 addr_list[i]);
			continue;
		}

		/* Check address availability (7 bit, no need to encode flags) */
		// 检查地址可用性(7位，不需要编码标志)
		if (i2c_check_addr_busy(adap, addr_list[i])) {
			dev_dbg(&adap->dev,
				"Address 0x%02x already in use, not probing\n",
				addr_list[i]);
			continue;
		}

		/* Test address responsiveness */
		// 测试地址响应能力,如果探测到哟响应的设备，就break.
		if (probe(adap, addr_list[i]))
			break;
	}

	if (addr_list[i] == I2C_CLIENT_END) {
		dev_dbg(&adap->dev, "Probing failed, no device found\n");
		return NULL;
	}

	info->addr = addr_list[i];
	// 通过 i2c_board_info 实例化一个i2c设备，依附适配器adap后并注册到i2c总线。
	return i2c_new_device(adap, info);
}
EXPORT_SYMBOL_GPL(i2c_new_probed_device);

// 通过总线（适配器）的id获得i2c_adapter对象。
struct i2c_adapter *i2c_get_adapter(int nr)
{
	struct i2c_adapter *adapter;

	mutex_lock(&core_lock);
	adapter = idr_find(&i2c_adapter_idr, nr);
	if (!adapter)
		goto exit;

	if (try_module_get(adapter->owner))
		get_device(&adapter->dev);
	else
		adapter = NULL;

 exit:
	mutex_unlock(&core_lock);
	return adapter;
}
EXPORT_SYMBOL(i2c_get_adapter);

// 减少i2c_adapter和模型的引用计数。
void i2c_put_adapter(struct i2c_adapter *adap)
{
	if (!adap)
		return;

	put_device(&adap->dev);
	module_put(adap->owner);
}
EXPORT_SYMBOL(i2c_put_adapter);

/**
 * i2c_get_dma_safe_msg_buf() - get a DMA safe buffer for the given i2c_msg
 * @msg: the message to be checked
 * @threshold: the minimum number of bytes for which using DMA makes sense.
 *	       Should at least be 1.
 *
 * Return: NULL if a DMA safe buffer was not obtained. Use msg->buf with PIO.
 *	   Or a valid pointer to be used with DMA. After use, release it by
 *	   calling i2c_put_dma_safe_msg_buf().
 *
 * This function must only be called from process context!
 */
/**	
* i2c_get_dma_safe_msg_buf()——为给定的i2c_msg获取一个DMA安全缓冲区	
* @msg:要检查的消息	
* @threshold:使用DMA有意义的最小字节数。至少应该是1。
* 返回:NULL如果没有获得DMA安全缓冲区。与PIO一起使用msg->buf。
* 或与DMA一起使用的有效指针。使用之后，通过调用i2c_put_dma_safe_msg_buf()来释放它。
* 此函数只能从进程上下文调用!	
*/
u8 *i2c_get_dma_safe_msg_buf(struct i2c_msg *msg, unsigned int threshold)
{
	/* also skip 0-length msgs for bogus thresholds of 0 */
	if (!threshold)
		pr_debug("DMA buffer for addr=0x%02x with length 0 is bogus\n",
			 msg->addr);
	if (msg->len < threshold || msg->len == 0)
		return NULL;

	if (msg->flags & I2C_M_DMA_SAFE)
		return msg->buf;

	pr_debug("using bounce buffer for addr=0x%02x, len=%d\n",
		 msg->addr, msg->len);

	if (msg->flags & I2C_M_RD)
		return kzalloc(msg->len, GFP_KERNEL);
	else
		return kmemdup(msg->buf, msg->len, GFP_KERNEL);
}
EXPORT_SYMBOL_GPL(i2c_get_dma_safe_msg_buf);

/**
 * i2c_put_dma_safe_msg_buf - release DMA safe buffer and sync with i2c_msg
 * @buf: the buffer obtained from i2c_get_dma_safe_msg_buf(). May be NULL.
 * @msg: the message which the buffer corresponds to
 * @xferred: bool saying if the message was transferred
 */
/**	
* i2c_put_dma_safe_msg_buf -释放DMA安全缓冲区并与i2c_msg同步
* @buf:从i2c_get_dma_safe_msg_buf()获得的缓冲区。可能是NULL。	
* @msg:缓冲区对应的消息
* @xferred: bool表示消息是否已被传输
*/
void i2c_put_dma_safe_msg_buf(u8 *buf, struct i2c_msg *msg, bool xferred)
{
	if (!buf || buf == msg->buf)
		return;

	if (xferred && msg->flags & I2C_M_RD)
		memcpy(msg->buf, buf, msg->len);

	kfree(buf);
}
EXPORT_SYMBOL_GPL(i2c_put_dma_safe_msg_buf);

MODULE_AUTHOR("Simon G. Vogl <simon@tk.uni-linz.ac.at>");
MODULE_DESCRIPTION("I2C-Bus main module");
MODULE_LICENSE("GPL");
