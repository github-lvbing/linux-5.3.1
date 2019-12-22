// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Linux I2C core
 *
 * Copyright (C) 1995-99 Simon G. Vogl
 *   With some changes from Kyösti Mälkki <kmalkki@cc.hut.fi>
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
 *core_lock����i2c_adapter_idr����֤�豸��⡢������豸��ɾ�������л�
 */
static DEFINE_MUTEX(core_lock);
static DEFINE_IDR(i2c_adapter_idr);

static int i2c_detect(struct i2c_adapter *adapter, struct i2c_driver *driver);

static DEFINE_STATIC_KEY_FALSE(i2c_trace_msg_key);

// ��Ǳ���ϵͳ�Ƿ�ע��
static bool is_registered;

// ׷��
int i2c_transfer_trace_reg(void)
{
	static_branch_inc(&i2c_trace_msg_key);
	return 0;
}

void i2c_transfer_trace_unreg(void)
{
	static_branch_dec(&i2c_trace_msg_key);
}

// ��i2c_device_id�в���ָ����i2c_client��ƥ���Ͼͷ��� i2c_device_id ָ��,���򷵻�NULL��
// ƥ��ԭ����  i2c_client->name ?=  i2c_device_id->name
const struct i2c_device_id *i2c_match_id(const struct i2c_device_id *id,
						const struct i2c_client *client)
{
	// must  id and client is not NULL
	if (!(id && client))
		return NULL;
	// this why is {}. at "struct i2c_device_id"��ѭ������
	while (id->name[0]) {
		// i2c_client->name ?=  i2c_device_id->name
		if (strcmp(client->name, id->name) == 0)
			return id;
		id++;
	}
	return NULL;
}
EXPORT_SYMBOL_GPL(i2c_match_id);

// i2c�豸��struct i2c_client�����豸ģ��(struct device) ���豸����ģ��(struct device_driver)ƥ�� ,ƥ���Ϸ���1�����򷵻�0
// ƥ�������˳���ǣ��豸��-->ACPI��-->�����б�
// ��ָ��������struct bus_type i2c_bus_type����һ��i2c�豸��i2c������ƥ�䷽���� 
// ƥ���Ϸ���1,���򷵻�0.
static int i2c_device_match(struct device *dev, struct device_driver *drv)
{
    //  У��struct device �Ǹ�i2c�豸
	struct i2c_client	*client = i2c_verify_client(dev);
	struct i2c_driver	*driver;


	/* Attempt an OF style match */  // �����豸����ʽƥ��
	if (i2c_of_match_device(drv->of_match_table, client)) // ==> drivers\i2c\i2c-core-of.c
		return 1;

	/* Then ACPI style match */     // ���� ACPI��ʽƥ��
	if (acpi_driver_match_device(dev, drv))
		return 1;

	driver = to_i2c_driver(drv);

	/* Finally an I2C match */       // �����I2Cƥ��(��ע�豸������struct i2c_device_id��name��Ա)
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
�������ڲ���ʱ�����塣ndelay()������clk����ĳ���ʱ�䡣
*���ǽ���������Ϊ100khz��ʱ�ӣ��������ʱ�Ӽ���ĳ���ʱ��Ϊ:ns = (10^6 / 100) / 2
*/

#define RECOVERY_NDELAY		5000
#define RECOVERY_CLK_CNT	9

// �ָ� i2c ʱ��
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

//  i2c ���߻ָ�
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

// ��ָ��������struct bus_type i2c_bus_type����һ��i2c�豸��i2c������ƥ���ǰ��Ԥ��������
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

// ��ָ��������struct bus_type i2c_bus_type����һ��i2c�豸��i2c�����Ľ��ƥ�����ڴ�������
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

// ��ָ��������struct bus_type i2c_bus_type����һ��i2c�����ر�i2c�豸�Ĺرշ�����
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

// i2c�豸�������Ͷ���
struct bus_type i2c_bus_type = {
	.name		= "i2c",
	.match		= i2c_device_match,
	.probe		= i2c_device_probe,
	.remove		= i2c_device_remove,
	.shutdown	= i2c_device_shutdown,
};
EXPORT_SYMBOL_GPL(i2c_bus_type);

// i2c�豸���Ͷ���
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
 * i2c_verify_client�����ز���i2c_client����NULL
 *  @dev: device���������ڱ�����������ģ����ʱ��ĳ����������ģ�͵������л�õģ�
 *                Ҳ����ʹ����@device_for_each_child()��������������ģ�͵������������ܶ��ҵ��Ľڵ���̫����衣
 *                ʹ�ô˺������Ա������ڴ���ؽ�ĳЩ��i2c�豸��Ϊi2c_client�����µ�oopses��
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
// ����i2c�豸�ĵ�Ӳ����ַ
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
 // ����һ����ɵĵ�ַ��Ч�Լ�飬����һ��ĵ��õ�ַ,I2C��ַ map��Լ���ǹ��ⲻִ�еġ�
 // ��Ч����0.
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
 // ����һ���ϸ�ĵ�ַ��Ч�Լ�飬����̽�⡣
 // ���һ���豸ʹ��һ��������ַ����ô����Ӧ�ñ�̽�⡣������7λ��ַ��10λ��ַ�豸���٣�Ӧ����ʽö�١�
int i2c_check_7bit_addr_validity_strict(unsigned short addr)
{
	/*
	 * Reserved addresses per I2C specification:  *��I2C���Ԥ����ַ:
	 *  0x00       General call address / START byte   һ����õ�ַ/��ʼ�ֽ�
	 *  0x01       CBUS address                       CBUS��ַ
	 *  0x02       Reserved for different bus format  Ԥ������ͬ�����߸�ʽ
	 *  0x03       Reserved for future purposes      ����������ʹ��
	 *  0x04-0x07  Hs-mode master code               Hs-mode������
	 *  0x78-0x7b  10-bit slave addressing          10λ�ӵ�ַ
	 *  0x7c-0x7f  Reserved for future purposes     ����������ʹ��
	 */
	if (addr < 0x08 || addr > 0x77)
		return -EINVAL;
	return 0;
}

// struct device �Ǹ�i2c�豸��������Ӳ����ַ�Ƿ���addrp�����򷵻�-EBUSY��˵���Ѿ���ʹ�ã�æ��
static int __i2c_check_addr_busy(struct device *dev, void *addrp)
{
	struct i2c_client	*client = i2c_verify_client(dev);
	int			addr = *(int *)addrp;

	if (client && i2c_encode_flags_to_addr(client) == addr)
		return -EBUSY;
	return 0;
}

/* walk up mux tree */
// �ݹ����struct i2c_adapter�ĸ��ס��жϸ������Ƿ��С�addrp�����i2c�豸���з���-EBUSY���޷���0��
static int i2c_check_mux_parents(struct i2c_adapter *adapter, int addr)
{
	//  ����������ĸ��豸��ַ�����������豸���ص�ַ�����Ƿ���NULL��
	struct i2c_adapter *parent = i2c_parent_is_2c_adapter(adapter);
	int result;
	// ͨ�����������������ϲ�������Ӳ���豸��ַΪaddr���豸��������豸����-EBUSY���޷���0��
	result = device_for_each_child(&adapter->dev, &addr,
					__i2c_check_addr_busy);

	// ������adapter�ĸ��豸����һ�������������߸��豸��������adapter���ϲ�����addr����豸���ӣ��򷵻�0����������ݹ�������Ҹ����豸��
	if (!result && parent)
		result = i2c_check_mux_parents(parent, addr);

	return result;
}

/* recurse down mux tree */ 
// ���������ݹ����struct device�ĺ��ӡ��жϺ������Ƿ��С�addrp�����i2c�豸���з���-EBUSY���޷���0��
static int i2c_check_mux_children(struct device *dev, void *addrp)
{
	int result;
	// ���豸��һ���������豸����������ĺ��ӡ�ֱ�����ĺ��ӱ���������ҵ���һ��i2c�豸���ᴥ�����ء�
	if (dev->type == &i2c_adapter_type)
		result = device_for_each_child(dev, addrp,
						i2c_check_mux_children);
	// ���豸�ǲ���һ���������豸���� struct device�Ƿ���i2c�豸��������Ӳ����ַ��addrp����˵��������豸������-EBUSY��
	else
		result = __i2c_check_addr_busy(dev, addrp);

	return result;
}

// �����ж������������Ƿ��е�ַΪ��addr����i2c�豸���Ѿ�ռ�þͷ���-EBUSY���޷���0��
static int i2c_check_addr_busy(struct i2c_adapter *adapter, int addr)
{
	struct i2c_adapter *parent = i2c_parent_is_i2c_adapter(adapter);
	int result = 0;
	// ��adapter�ĸ��豸Ҳ��һ�����������ݹ�������ĸ��ס�
	// �жϸ������Ƿ��С�addr�����i2c�豸���з���-EBUSY= result���޷���0=result.
	if (parent)
		result = i2c_check_mux_parents(parent, addr);
	// ��adapter�ĸ��豸����һ�������������߸��豸��������addr�����i2c�豸���͵ݹ����struct device�ĺ��ӡ��жϺ������Ƿ��е�ַΪ��addrp�����i2c�豸���з���-EBUSY���޷���0��
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
 * i2c_adapter_lock_bus -��ö�I2C���߶εĶ�ռ����
 * @adapter:Ŀ��I2C���߶�
 * @flags: I2C_LOCK_ROOT_ADAPTER ������i2c��������
 *         I2C_LOCK_SEGMENT ���������������е������֧
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
 * i2c_adapter_lock_bus -���Ի�ö�I2C���߶εĶ�ռ����
 * @adapter:Ŀ��I2C���߶�
 * @flags: I2C_LOCK_ROOT_ADAPTER ������i2c��������
 *		   I2C_LOCK_SEGMENT ���������������е������֧
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
 * i2c_adapter_unlock_bus -�ͷŶ�I2C���߶εĶ�ռ����
 * @adapter:Ŀ��I2C���߶�
 * @flags: I2C_LOCK_ROOT_ADAPTER������i2c��������I2C_LOCK_SEGMENT���������������е������֧
 */
static void i2c_adapter_unlock_bus(struct i2c_adapter *adapter,
				   unsigned int flags)
{
	rt_mutex_unlock(&adapter->bus_lock);
}

// ��i2c�豸client �������֣�����struct i2c_adapter ����struct i2c_board_info��
static void i2c_dev_set_name(struct i2c_adapter *adap,
			     struct i2c_client *client,
			     struct i2c_board_info const *info)
{
	struct acpi_device *adev = ACPI_COMPANION(&client->dev);
	// ����ʹ�� struct i2c_board_info  ��Ϣ
	if (info && info->dev_name) {
		dev_set_name(&client->dev, "i2c-%s", info->dev_name);
		return;
	}
	// ���ʹ�� struct i2c_adapter ��Ϣ
	if (adev) {
		dev_set_name(&client->dev, "i2c-%s", acpi_dev_name(adev));
		return;
	}
	// ���ʹ�� struct i2c_adapter ��Ϣ�� client��ַ��Ϣ
	dev_set_name(&client->dev, "%d-%04x", i2c_adapter_id(adap),
		     i2c_encode_flags_to_addr(client));
}

// ����Դ�� struct resource�в����ж���Դ���������жϺţ��Ҳ�������0.
int i2c_dev_irq_from_resources(const struct resource *resources,
			       unsigned int num_resources)
{
	struct irq_data *irqd;
	int i;

	for (i = 0; i < num_resources; i++) {
		const struct resource *r = &resources[i];
        // ������IO���ж���Դ������һ��
		if (resource_type(r) != IORESOURCE_IRQ)
			continue;
        // �����ж���Դ��Դ�����о���Ķ��塣�Ͳ����ж������Ի��struct irq_data��
		if (r->flags & IORESOURCE_BITS) {
			irqd = irq_get_irq_data(r->start);
			// �޷���� irq_data �������̷���0.
			if (!irqd)
				break;
            // ����� irq_data ������irqоƬ���ܵ�״̬��Ϣ��
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
  * i2c_new_client_device -ʵ����һ��i2c�豸������������adap��ע��(device_register�ӿ�)��i2c���ߡ�
  * @adap:�����豸��������
  * @info:����һ��I2C�豸;bus_num������������:��������
  * ����һ��i2c�豸������ͨ����������ģ��probe()/remove()����������ġ�
  * �����Ǵ������������ʱ������������ܱ��󶨵�����豸�������Ժ���κ�ʱ��(���磬�����Ȳ�ν�������������ģ��)��
  * �����ʼ���߼����ʺ�ʹ��������ã���Ϊ��ͨ����arch_initcall()�ڼ����У����κ�i2c_adapter��Ҫ��öࡣ
  * �⽫�����µ�i2c�ͻ��ˣ��ÿͻ��˿��ܱ������������Ա��Ժ���i2c_unregister_device()һ��ʹ��;����ʹ��ERR_PTR����������
  */
struct i2c_client *
i2c_new_client_device(struct i2c_adapter *adap, struct i2c_board_info const *info)
{
	struct i2c_client	*client;
	int			status;
    // ����һ��i2c�豸���� struct i2c_client
	client = kzalloc(sizeof *client, GFP_KERNEL);
	if (!client)
		return ERR_PTR(-ENOMEM);
	// ָ��i2c�豸��������Ϊ adap
	client->adapter = adap;
    // ��info��ƽ̨���ݡ�flags��addr��irqָ��Ϊi2c�豸��ƽ̨���ݡ�flags��addr����init_irq��
	client->dev.platform_data = info->platform_data;
	client->flags = info->flags;
	client->addr = info->addr;

	client->init_irq = info->irq;
	// ��� info->irq=0�����ж���Ϣ������i2c_board_info����Դ�� struct resource�в����ж���Դ���������жϺţ��Ҳ�������0.
	if (!client->init_irq)
		client->init_irq = i2c_dev_irq_from_resources(info->resources,
							 info->num_resources);
	// ��info��оƬ���ͱ�ʶ����Ϊ i2c_client ��name��
	strlcpy(client->name, info->type, sizeof(client->name));

	// У��i2c_client.addr ����Ч�ԡ���Ч�Ļ�����err���˳���
	status = i2c_check_addr_validity(client->addr, client->flags);
	if (status) {
		dev_err(&adap->dev, "Invalid %d-bit I2C address 0x%02hx\n",
			client->flags & I2C_CLIENT_TEN ? 10 : 7, client->addr);
		goto out_err_silent;
	}

	/* Check for address business  */ 
	// ��ѯ��ַ�Ƿ��Ѿ���ռ��,���Ѿ�ռ�þ�err���˳���
	status = i2c_check_addr_busy(adap, i2c_encode_flags_to_addr(client));
	if (status)
		goto out_err;
	// i2c�豸�ĸ��豸ָ��Ϊ������������Ϊ i2c_bus_type���豸����Ϊ i2c_client_type��
	client->dev.parent = &client->adapter->dev;
	client->dev.bus = &i2c_bus_type;
	client->dev.type = &i2c_client_type;
	// ??
	client->dev.of_node = of_node_get(info->of_node);
	client->dev.fwnode = info->fwnode;

	// Ϊi2c�豸�����������֡�
	i2c_dev_set_name(adap, client, info);

	// �������豸�ĸ������ԣ����豸����������Լ���
	if (info->properties) {
		status = device_add_properties(&client->dev, info->properties);
		if (status) {
			dev_err(&adap->dev,
				"Failed to add properties to client %s: %d\n",
				client->name, status);
			goto out_err_put_of_node;
		}
	}

	// ��i2c�豸���� ע�ᵽi2c�����С�
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
 * i2c_new_device����ͨ�� i2c_board_info ʵ����һ��i2c�豸������������adap��ע�ᵽi2c���ߡ�
 * @adap:�����豸��������
 * @info:����һ��I2C�豸;bus_num������������:��������
 * ��������õĺ�����@i2c_new_client_device������ͬ�Ĺ��ܣ���ֻ�Ƿ���NULL��������ERR_PTR���Է��뵱ǰI2C API���ݳ��ִ���
 * һ�������û���������ת����������ɾ����
 * �⽫�����µ�i2c�ͻ��ˣ��ÿͻ��˿��ܱ������������Ա��Ժ���i2c_unregister_device()һ��ʹ��;��NULL��ʾ����
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
 * i2c_unregister_device - i2c_new_device()�ķ���Ч��;��i2c������ע��i2c�豸
 * @client:��i2c_new_device()�����ķ��ص�ֵ:��������
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
	// ��ϵͳ��ע���豸��	
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

// һ������� i2c �豸�������鿼i2c��ϵͳ��ʼ���� static int __init i2c_init(void)��
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
 * i2c_new_dummy_device��ʵ����һ���豸��ַΪaddr�Ĵ��豸����ע�ᵽ���ߣ����ذ󶨵����������������i2c�豸
 * @adapter:�����豸��������
 * @address:ʹ�õ�7λ��ַ
 * Context:����˯��
 * �⽫����һ���󶨵������⡱���������I2C�ͻ��ˣ�����ʹ�ö����ַ���豸������оƬ�����Ӱ�������EEPROMS(��24c04��24c08�ͺ�)��
 * ��Щ�����豸��������Ҫ��;�����ȣ�����i2c_transfer()֮�⣬�����I2C��SMBus���ö���Ҫһ���ͻ��˾��;���˾����Ǹ����֡�
 * ��Σ�����Է�ֹ��ָ���ĵ�ַ�󶨵���ͬ����������
 * �⽫�����µ�i2c�ͻ��ˣ��ÿͻ���Ӧ�ñ����������Ա��Ժ���i2c_unregister_device()һ��ʹ��;����ʹ��ERR_PTR����������
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
 * i2c_new_dummy��ʵ����һ���豸��ַΪaddr�Ĵ��豸������adapter�󣬲�ע�ᵽ���ߣ����ذ󶨵����������������i2c�豸
 * @adapter:�����豸��������
 * @address:ʹ�õ�7λ��ַ
 * ������:����˯��
 * ��������ĺ�����@i2c_new_dummy_device������ͬ�Ĺ��ܣ�
 * ����뵱ǰI2C API���ݳ��ִ�����ֻ����NULL��������ERR_PTR��һ�������û���������ת����������ɾ����
 * �⽫�����µ�i2c�ͻ��ˣ��ÿͻ���Ӧ�ñ����������Ա��Ժ���i2c_unregister_device()һ��ʹ��;��NULL��ʾ����
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
 * devm_i2c_new_dummy_device�����ذ󶨵����������������i2c�豸
 * @dev:��������Դ�󶨵����豸
 * @adapter:�����豸��������
 * @address:ʹ�õ�7λ��ַ
 * ������:����˯��
 * ����@i2c_new_dummy_device���豸����汾��������ִ������������µ�i2c�ͻ�����ERR_PTR��
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
* i2c_new_secondary_device -������ȡʵ�����ĸ�����ַ
* �����������豸
* @client:���ͻ��˵ľ��
* @name:ָ����ȡ�ĸ�������ַ�ľ��
* @default_addr:���û��ָ��������ַ������������
* ������:����˯��
*
* I2C�ͻ��˿����ɶ�����ڵ�������е�I2C�ӷ�������ɡ�Ȼ��I2C�ͻ�����������󶨵���I2C���豸������Ҫ����I2C����ͻ������������������豸ͨ�š�
*
* �����������������һ��I2C����ͻ��ˣ�����I2C��ַ�Ǹ��ݸ����Ĵ�������ƽ̨�̼��л�ȡ�ġ�����̼�û��ָ����ַ����ʹ��default_addr��
*
* �ڻ���dd��ƽ̨�ϣ���ַ�Ǵӡ�reg��������Ŀ��Ԫ���м����ģ��õ�Ԫ��ġ�reg-names��ֵ���������ƥ�䡣
*
* �⽫�����µ�i2c�ͻ��ˣ��ÿͻ���Ӧ�ñ����������Ա��Ժ���i2c_unregister_device()һ��ʹ��;��NULL��ʾ����
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
// I2C��������������ÿ��I2C��SMBUS�ζ���һ����

// ������һ���̹߳����Ѿ���ɡ�
static void i2c_adapter_dev_release(struct device *dev)
{
	struct i2c_adapter *adap = to_i2c_adapter(dev);
	complete(&adap->dev_released);
}

// �鿴���������������
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
* ���û�ͨ��sysfsʵ����I2C�豸����ƽ̨��ʼ����������ĳ��ԭ�򲻰����ʵ�������ʱ������ʹ�����ַ�����
* �����豸���ͼ��ʧ�ܵ���������Ҳ�����ã���Ϊ�豸ʹ����һ������ĵ�ַ����������һ�����в�ͬID�Ĵ���ֵ�ļ����豸��	
* ���������ܿ������������ģ���������Ĳ�ϣ���û��ṩ����ȷ�Ĳ�����	
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
	// ���� struct i2c_board_info����
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
	// ����ʣ��Ĳ������ܾ�����Ĳ���
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

	// ʵ����һ��i2c�豸������������adap��ע��(device_register�ӿ�)��i2c���ߡ�
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
 * ��ȻҲ�����û�ɾ������ʵ�������豸���������Ū���˵Ļ���
 * ����ӿ�ֻ������ɾ������i2c_sysfs_new_deviceʵ�������豸���Ᵽ֤�����ǲ���ɾ��ĳЩ�ں˴�����Ȼ�����õ��豸��
 * ���������ܿ������������ģ���������Ĳ�ϣ���û�ɾ��������豸��
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
	// �����������ܾ�����Ĳ���
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
	// ȷ���豸��ͨ��sysfs��ӵ�
	res = -ENOENT;
	mutex_lock_nested(&adap->userspace_clients_lock,
			  i2c_adapter_depth(adap));
	list_for_each_entry_safe(client, next, &adap->userspace_clients,
				 detected) {
		if (i2c_encode_flags_to_addr(client) == addr) {
			dev_info(dev, "%s: Deleting device %s at 0x%02hx\n",
				 "delete_device", client->name, client->addr);
			// ɾ��������������̽�����
			list_del(&client->detected);
			// ��i2c������ע��i2c�豸
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
��ͬ��: ���� ���ڵ����豸���ԵĽӿ�ʵ������
struct device_attribute dev_attr_delete_device ={
	.attr = {
		.name = __stringify(delete_device), 
			.mode = S_IWUSR,
			.ignore_lockdep = true 
			},
	.show		= NULL,
	.store		= i2c_sysfs_delete_device, // �����û�ɾ������ʵ�������豸
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
��ͬ��:
static const struct attribute_group i2c_adapter_group = {
	.attrs = i2c_adapter_attrs,
};
static const struct attribute_group *i2c_adapter_groups[] = {
	&i2c_adapter_group,	
	NULL,
}
note: i2c_register_adapter()�ӿ��л�ʹ�á�
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
* i2c_verify_adapter�����ز���i2c_adapter��NULL��������֤struct device�Ƿ���һ����������struct i2c_adapter���豸��
* @dev:�豸����������ĳ����������ģ�͵�����
*
* �ڱ�����������ģ����ʱ������ʹ����@device_for_each_child()��������������ģ�͵������������ܶ��ҵ��Ľڵ���̫����衣
* ʹ�ô˺������Ա������ڴ���ؽ�ĳЩ��i2c�豸��Ϊi2c_adapter�����µ�oopses��
*/
struct i2c_adapter *i2c_verify_adapter(struct device *dev)
{
	return (dev->type == &i2c_adapter_type)
			? to_i2c_adapter(dev)
			: NULL;
}
EXPORT_SYMBOL(i2c_verify_adapter);

// ������
#ifdef CONFIG_I2C_COMPAT
static struct class_compat *i2c_adapter_compat_class;
#endif

// ����̬�������ţ��� ȫ������__i2c_board_list �ϵĿ�������adapter�ϵ�i2c_board_infoʵ������ע�ᵽ�� i2c���ߡ�
static void i2c_scan_static_board_info(struct i2c_adapter *adapter)
{
	struct i2c_devinfo	*devinfo;

	down_read(&__i2c_board_lock);
	// ����ȫ������__i2c_board_list��
	list_for_each_entry(devinfo, &__i2c_board_list, list) {
		// ��ÿ����������adapter�ϵ�i2c_board_infoʵ������ע�ᵽ�� i2c���ߡ�
		if (devinfo->busnum == adapter->nr
				&& !i2c_new_device(adapter,
						&devinfo->board_info))
			dev_err(&adapter->dev,
				"Can't create device at 0x%02x\n",
				devinfo->board_info.addr);
	}
	up_read(&__i2c_board_lock);
}


// �����豸���� i2c_driver->address_list �ṩ���豸��ַ�б������������豸����ͬ�������£�
// ���ʵ������ʵ�ֽ��豸�ĵ�i2c���ߵ�ע�ᣬ����������driver�İ󶨡�
static int i2c_do_add_adapter(struct i2c_driver *driver,
			      struct i2c_adapter *adap)
{
	/* Detect supported devices on that bus, and instantiate them */
	// �����������֧�ֵ��豸����ʵ��������.
	i2c_detect(adap, driver);

	return 0;
}

// ͬ i2c_do_add_adapter��
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

// �������������irq�ź�hw irq��֮���ӳ�䡣
static int i2c_host_notify_irq_map(struct irq_domain *h,
					  unsigned int virq,
					  irq_hw_number_t hw_irq_num)
{
	irq_set_chip_and_handler(virq, &dummy_irq_chip, handle_simple_irq);

	return 0;
}

// irq_domain����ķ���
static const struct irq_domain_ops i2c_host_notify_irq_ops = {
	.map = i2c_host_notify_irq_map,
};

// ����i2c����֪ͨirq��
static int i2c_setup_host_notify_irq_domain(struct i2c_adapter *adap)
{
	struct irq_domain *domain;
	// ���i2c_adapter��֧��I2C_FUNC_SMBUS_HOST_NOTIFY�����ء�
	if (!i2c_check_functionality(adap, I2C_FUNC_SMBUS_HOST_NOTIFY))
		return 0;
	// ����һ���µ�irq_domain���ݽṹ.������֧�ֵ�����ж���=I2C_ADDR_7BITS_COUNT.
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

// ���i2c_adapter��ϵͳ��ע�ᣬ������豸ʵ�����͵����ߵ�ע�ᣬ�Լ��豸���������������İ󶨡�
// ����豸ָ��
// 1-��������Ԥ��������δʵ�������豸�ڵ㡣
// 2-ȫ������__i2c_board_list �ϵĿ�������adapter�ϵ�i2c_board_infoʵ������ע�ᵽ�� i2c����.
// 3-�������� i2c_bus_type ��������������adapͬ����豸����device_driver����Ա i2c_driver->address_list �ṩ���豸��ַ��
static int i2c_register_adapter(struct i2c_adapter *adap)
{
	int res = -EINVAL;

	/* Can't register until after driver model init */
	// ����ע��,ֱ����������ģ��init��.
	if (WARN_ON(!is_registered)) {
		res = -EAGAIN;
		goto out_list;
	}

	/* Sanity checks */
	// �����Լ��
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
	// �����δ���ã���Ĭ�ϳ�ʱ����Ϊ1��.
	if (adap->timeout == 0)
		adap->timeout = HZ;

	/* register soft irqs for Host Notify */
	// ע������֪ͨ����irq
	res = i2c_setup_host_notify_irq_domain(adap);
	if (res) {
		pr_err("adapter '%s': can't create Host Notify IRQs (%d)\n",
		       adap->name, res);
		goto out_list;
	}

	// �����豸����/��������/�豸���͡�
	dev_set_name(&adap->dev, "i2c-%d", adap->nr);
	adap->dev.bus = &i2c_bus_type;
	adap->dev.type = &i2c_adapter_type;
	// ��������ע�ᵽϵͳ���ɹ�����0
	res = device_register(&adap->dev);
	if (res) {
		pr_err("adapter '%s': can't register device (%d)\n", adap->name, res);
		goto out_list;
	}

	// i2c ����smbus����
	res = of_i2c_setup_smbus_alert(adap);
	if (res)
		goto out_reg;

	dev_dbg(&adap->dev, "adapter [%s] registered\n", adap->name);

	pm_runtime_no_callbacks(&adap->dev);
	pm_suspend_ignore_children(&adap->dev, true);
	pm_runtime_enable(&adap->dev);

#ifdef CONFIG_I2C_COMPAT
	// ����һ���������豸�ļ������豸����
	res = class_compat_create_link(i2c_adapter_compat_class, &adap->dev,
				       adap->dev.parent);
	if (res)
		dev_warn(&adap->dev,
			 "Failed to create compatibility class link\n");
#endif

	//I2C���߻ָ���Ϣ:init       struct i2c_bus_recovery_info.
	i2c_init_recovery(adap);

	/* create pre-declared device nodes */
	// ��������������Ԥ��������δʵ�������豸�ڵ㡣ʵ����Ϊi2c�豸(i2c_client)��ע�ᡣ
	of_i2c_register_devices(adap);
	// ö���������ϵ�I2C���豸
	i2c_acpi_register_devices(adap);
	i2c_acpi_install_space_handler(adap);

	// ����������������ߺ�<__i2c_first_dynamic_bus_num.����һ����̬�������������
	// �ͽ� ȫ������__i2c_board_list �ϵĿ�������adapter�ϵ�i2c_board_infoʵ������ע�ᵽ�� i2c���ߡ�
	if (adap->nr < __i2c_first_dynamic_bus_num)
		i2c_scan_static_board_info(adap);

	/* Notify drivers */
	// ֪ͨ��������
	mutex_lock(&core_lock);
	// �������� i2c_bus_type �����е�����device_driver���� adap ��Ϊ�������ص�(*__process_new_adapter)����
	// ��adap������������Ա i2c_driver->address_list �ṩ���豸��ַ�б������������豸����ͬ�������£�
	// ���ʵ������ʵ�ֽ��豸�ĵ�i2c���ߵ�ע�ᣬ�����뱾����data(driver)�İ�
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
 * i2c_add_numbered_adapter - i2c_add_numbered_adapter������nr�Ӳ�Ϊ-1	
 * @adap:��Ҫע���������(��ʼ��adap->nr)	
 * ������:����˯��	 
 * ���i2c_add_numbered_adapter()��
 */
// ����������Ȩһ��id����̬���䣩.���i2c_adapter��ϵͳ��ע�ᣬ������豸ʵ�����͵����ߵ�ע�ᣬ�Լ��豸���������������İ�.
static int __i2c_add_numbered_adapter(struct i2c_adapter *adap)
{
	int id;

	mutex_lock(&core_lock);
	// ��������Ȩһ��id.
	id = idr_alloc(&i2c_adapter_idr, adap, adap->nr, adap->nr + 1, GFP_KERNEL);
	mutex_unlock(&core_lock);
	if (WARN(id < 0, "couldn't get idr"))
		return id == -ENOSPC ? -EBUSY : id;

	// ���i2c_adapter��ϵͳ��ע�ᣬ������豸ʵ�����͵����ߵ�ע�ᣬ�Լ��豸���������������İ󶨡�
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
 * i2c_add_adapter -����i2c��������ʹ�ö�̬���ߺ�
 * @adapter:Ҫ��ӵ�������
 * ������:����˯��
 *
 * ��I2C�����������ߺ��޹ؽ�Ҫ�������ߺ���dt����ָ��ʱ����������������I2C��������
 * �����ߺ��벻��Ҫ��ʱ��Ļ���������:I2C��������̬�ر�USB���ӻ�һ�����߱�׼��������ӡ�
 *
 * ������0ʱ��һ���µ����ߺű����䲢�洢��adap->nr�У�����ָ�����������Կͻ��˿��á�
 * ���򣬽�����һ������errnoֵ��
 */
// ���������һ��id����̬���䣩�����i2c_adapter��ϵͳ��ע�ᣬ������豸ʵ�����͵����ߵ�ע�ᣬ�Լ��豸���������������İ�.
// ���ֵõ����ߺſϻ᲻��С��__i2c_first_dynamic_bus_num
int i2c_add_adapter(struct i2c_adapter *adapter)
{
	struct device *dev = &adapter->dev;
	int id;

	// �������dev->of_node���һ�ȡ����device_node�ı���id >= 0��
	// ��������Ȩһ��id..��
	// ���i2c_adapter��ϵͳ��ע�ᣬ������豸ʵ�����͵����ߵ�ע�ᣬ�Լ��豸���������������İ�.
	if (dev->of_node) {
		// ��ȡ����device_node�ı���id��
		id = of_alias_get_id(dev->of_node, "i2c");
		if (id >= 0) {
			adapter->nr = id;
			return __i2c_add_numbered_adapter(adapter);
		}
	}
	
	// ���������dev->of_node�������޷���ȡ����device_node�ı���id��
	// ��̬����һ��id.��
	// ���i2c_adapter��ϵͳ��ע�ᣬ������豸ʵ�����͵����ߵ�ע�ᣬ�Լ��豸���������������İ�.
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
* i2c_add_numbered_adapter��������i2c��������ʹ�þ�̬���ߺ�	
* @adap:��Ҫע���������(��ʼ��adap->nr)
* ������:����˯��	
* �������������I2C�����������ߺ���Ҫʱ��������
* ���磬������������ϵͳƬ��cpu��I2C����������������ϵͳ�������������������������ʹ��i2c_board_info��ȷ����I2C�豸��
* �����������ߺ�����Ϊ-1����ô�����������Ϊ����i2c_add_adapter��ͬ��������̬����һ�����ߺš�
* ���û��Ԥ��Ϊ�����������豸����ôȷ���ڶ�̬�����豸֮ǰע�����������������������ID���ܲ����á�
* ������0ʱ��ʹ��adap->nr���ṩ�����ߺ�Ϊ�ͻ����ṩָ������������
* ���⣬ʹ��i2c_register_board_info()Ԥ��������I2C�豸����ɨ�裬�������ʵ�����������ģ���豸�ڵ㡣
* ���򣬽�����һ������errnoֵ��	
*/
// �Զ�����̬���߶�̬�����������һ��id�����i2c_adapter��ϵͳ��ע�ᣬ������豸ʵ�����͵����ߵ�ע�ᣬ�Լ��豸���������������İ�.
int i2c_add_numbered_adapter(struct i2c_adapter *adap)
{
	if (adap->nr == -1) /* -1 means dynamically assign bus id */  // -1��ʾ��̬��������id
		return i2c_add_adapter(adap);

	// ����̬����
	return __i2c_add_numbered_adapter(adap);
}
EXPORT_SYMBOL_GPL(i2c_add_numbered_adapter);

// ��ϵͳ��ע��i2c_driver��������adapter��client��
static void i2c_do_del_adapter(struct i2c_driver *driver,
			      struct i2c_adapter *adapter)
{
	struct i2c_client *client, *_n;

	/* Remove the devices we created ourselves as the result of hardware
	 * probing (using a driver's detect method) */
	// ɾ�������Լ�������Ӳ��̽���豸(ʹ����������ļ�ⷽ��)
	// �����豸�����ϵ�������adapter�� client��
	list_for_each_entry_safe(client, _n, &driver->clients, detected) {
		if (client->adapter == adapter) {
			dev_dbg(&adapter->dev, "Removing %s at 0x%x\n",
				client->name, client->addr);
			// ɾ���豸�������İ󶨡�
			list_del(&client->detected);
			// ��ϵͳ��ע��client
			i2c_unregister_device(client);
		}
	}
}

// ��ϵͳ��ע����dummyΪ���豸dev����"dummy" �豸����ע������
static int __unregister_client(struct device *dev, void *dummy)
{
	struct i2c_client *client = i2c_verify_client(dev);
	// "dummy" �豸����ע����
	if (client && strcmp(client->name, "dummy"))
		i2c_unregister_device(client);
	return 0;
}

// ��ϵͳ��ע����dummyΪ���豸dev����"dummy" �豸��ע������
static int __unregister_dummy(struct device *dev, void *dummy)
{
	struct i2c_client *client = i2c_verify_client(dev);
	i2c_unregister_device(client);
	return 0;
}

// ��ϵͳ��ע��i2c_driver��������data(adapter)������client��
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
* i2c_del_adapter -ȡ��ע��I2C������	
* @adap:������δע��
* ������:����˯��
*
* �⽫ע��֮ǰ��@i2c_add_adapter��@i2c_add_numbered_adapterע���I2C��������
*/
void i2c_del_adapter(struct i2c_adapter *adap)
{
	struct i2c_adapter *found;
	struct i2c_client *client, *next;

	/* First make sure that this adapter was ever added */
	// ���ȣ�ȷ���Ѿ���������������
	mutex_lock(&core_lock);
	found = idr_find(&i2c_adapter_idr, adap->nr);
	mutex_unlock(&core_lock);
	if (found != adap) {
		pr_debug("attempting to delete unregistered adapter [%s]\n", adap->name);
		return;
	}

	i2c_acpi_remove_space_handler(adap);
	/* Tell drivers about this removal */
	// ����adap��ÿ��drivers���Ǩ��.��ϵͳ��ע��ÿ��i2c_driver��������data(adapter)������client��
	mutex_lock(&core_lock);
	bus_for_each_drv(&i2c_bus_type, NULL, adap,
			       __process_removed_adapter);
	mutex_unlock(&core_lock);

	/* Remove devices instantiated from sysfs */
	// ��sysfs��ɾ��ʵ�������豸
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
    // �豸����device_unregister֮����ʧ
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
	// �ȴ���ֱ�����豸���������ö���ʧ
	init_completion(&adap->dev_released);
	device_unregister(&adap->dev);
	wait_for_completion(&adap->dev_released);

	/* free bus id */
	// ע�� bus id
	mutex_lock(&core_lock);
	idr_remove(&i2c_adapter_idr, adap->nr);
	mutex_unlock(&core_lock);

	/* Clear the device structure in case this adapter is ever going to be
	   added again */
	// ����豸�ṹ���Է��ٴ���Ӵ�������
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
* i2c_parse_fw_timings���ӹ̼���ȡI2C��صĶ�ʱ����
* @dev:ɨ��I2C��ʱ���Ե��豸
* @t:Ҫ���ֵ��i2c_timings�ṹ
* @use_defaults: bool��û���ҵ�����ʱʹ������I2C�淶��saneĬ��ֵ������ʹ��0	
* ɨ���豸�����������źŶ�ʱ������ͨ��I2C���ԣ����������䵽�����ṹ�С�
* ���û���ҵ����ԣ�����use_defaultsΪ�棬��ô�ͻ�ٶ�I2C�淶�й涨�����ʱ�䡣
* ���û��ʹ��use_defaults����ô�������0�������������������Ժ�Ӧ�������Լ���Ĭ��ֵ��
* ������Ҫ��Ϊ�˱�����������������Ҫ�л���������ܵĻع顣��������������Ӧ��ʹ��Ĭ�ϡ�	
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

// ����i2c���������е��豸 device����data��Ϊ�����ص�fn��
int i2c_for_each_dev(void *data, int (*fn)(struct device *dev, void *data))
{
	int res;

	mutex_lock(&core_lock);
	res = bus_for_each_dev(&i2c_bus_type, NULL, data, fn);
	mutex_unlock(&core_lock);

	return res;
}
EXPORT_SYMBOL_GPL(i2c_for_each_dev);


// ��dev�Ǹ��������Ļ���
// �����豸���� i2c_driver->address_list �ṩ���豸��ַ�б������������豸����ͬ�������£�
// ���������ַ�б��豸ʵ����������������������ʵ�ֽ��豸�ĵ�i2c���ߵ�ע�ᣬ����������driver(data)�İ󶨡�
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
 * i2c_driver��һ������i2c_client(�豸)�ڵ�һ��ʹ�ã�����ĳ��i2c_adapter����������ʵ���Ϸ���i2c��оƬ��	
 */
//ʵ����driver��ַ�б��еĴ��豸��ע�ᵽ�뱾driverͬ���͵������������������󣬲�����뱾driver�󶨡�
int i2c_register_driver(struct module *owner, struct i2c_driver *driver)
{
	int res;

	/* Can't register until after driver model init */
	// ���i2c�����Ƿ��Ѿ�ע�ᣬû�о��˳���
	if (WARN_ON(!is_registered))
		return -EAGAIN;

	/* add the driver to the list of i2c drivers in the driver core */
	// ������������ӵ�����������ĵ�i2c���������б���
	driver->driver.owner = owner;
	driver->driver.bus = &i2c_bus_type;
	INIT_LIST_HEAD(&driver->clients);

	/* When registration returns, the driver core
	 * will have called probe() for all matching-but-unbound devices.
	 */
	// ��ע�᷵��ʱ����������Ϊ���������ڵ�������������ƥ�䵫δ�󶨵��豸����probe()������豸�뱾�����İ󶨡�
	res = driver_register(&driver->driver);
	if (res)
		return res;

	pr_debug("driver [%s] registered\n", driver->driver.name);

	/* Walk the adapters that are already present */
	// ���������ϵ��Ѿ����ڵ���������һ�������Ͽ����ж����������,���뱾����ͬ�ࡣ
	// ����豸����driver�е�ַ�б��豸ʵ�����������������������ʵ�ֽ����豸�ĵ�i2c���ߵ�ע�ᣬ����������driver�İ󶨡�
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
* i2c_use_client������i2c�ͻ��˽ṹ�����ü���
* @client:�����õĿͻ���
* Ӧ���¼����Կͻ��˵�ÿ�������á���������ģ���Զ�������Ϊ��������󶨵�һ���֣�
* ��˴��������������Ҫ��ʽ��������:���ǳ���һ�����ã�ֱ�����Ǵ��豸�Ͻ���󶨡�
* ����һ��ָ����е������ü������Ŀͻ��˵�ָ�롣
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
* i2c_release_client -�ͷ�i2c�ͻ��˽ṹ
* @client:���ٱ����õĿͻ���	
* �����ڿͻ����û�ʹ�����ʱ���á�
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

//i2c���豸�󶨵�����ִ���豸���ض�����
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

// �ñ��������ϵ����д��豸���󶨵�����ִ���豸���ض�����
void i2c_clients_command(struct i2c_adapter *adap, unsigned int cmd, void *arg)
{
	struct i2c_cmd_arg	cmd_arg;

	cmd_arg.cmd = cmd;
	cmd_arg.arg = arg;
	// ���������������Ϻ��ӣ����豸����i2c���豸�󶨵�����ִ���豸���ض�����
	device_for_each_child(&adap->dev, &cmd_arg, i2c_cmd);
}
EXPORT_SYMBOL(i2c_clients_command);

// i2c ��ϵͳ��ʼ��
static int __init i2c_init(void)
{
	int retval;

	// ��� aliases�ڵ��еı������Ե����idֵ
	retval = of_alias_get_highest_id("i2c");

	// ����д�ź���
	down_write(&__i2c_board_lock);
	// ����   __i2c_first_dynamic_bus_num
	if (retval >= __i2c_first_dynamic_bus_num)
		__i2c_first_dynamic_bus_num = retval + 1;
	up_write(&__i2c_board_lock);

	// i2c ����ע�ᡣ
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
	// ע��һ�������i2c���߼����豸������
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
* ���Ǳ��뾡���ʼ������ΪһЩ��ϵͳ�� subsys_initcall() ������ע����i2c��������
* ������i2c֮ǰ�ͱ�����(����ʼ��)�ˡ�
*/
postcore_initcall(i2c_init);
module_exit(i2c_exit);

/* ----------------------------------------------------
 * the functional interface to the i2c busses.  i2c���ߵĹ��ܽӿڡ�
 * ----------------------------------------------------
 */

/* Check if val is exceeding the quirk IFF quirk is non 0 */
// ���val�Ƿ񳬹���quirk�����quirk��0
#define i2c_quirk_exceeded(val, quirk) ((quirk) && ((val) > (quirk)))

static int i2c_quirk_error(struct i2c_adapter *adap, struct i2c_msg *msg, char *err_msg)
{
	dev_err_ratelimited(&adap->dev, "adapter quirk: %s (addr 0x%04x, size %u, %s)\n",
			    err_msg, msg->addr, msg->len,
			    msg->flags & I2C_M_RD ? "read" : "write");
	return -EOPNOTSUPP;
}

// i2c ���߶Է�����Ϣ��ȱ���Լ�顣
static int i2c_check_for_quirks(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	const struct i2c_adapter_quirks *q = adap->quirks;
	int max_num = q->max_num_msgs, i;
	bool do_len_check = true;

	if (q->flags & I2C_AQ_COMB) {
		max_num = 2;

		/* special checks for combined messages */
        // �ϲ���Ϣ��������
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
* i2c_transfer -  ��i2c_transfer�������á�
* @adap: I2C���ߵľ��
* @msgs:�ڷ�����ֹ������STOP֮ǰ����Ҫִ��һ���������Ϣ;ÿ����Ϣ���Կ�ͷ��ͷ��
* @num:Ҫִ�е���Ϣ������
*
* ���ظ���errno������ִ�е���Ϣ����
*	
* ���ô˺���ʱ����������������������е�����־��¼��adap->�㷨->master_xfer������û�м�顣
*/
// ���������Ϸ���num����Ϣ
int __i2c_transfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	unsigned long orig_jiffies;
	int ret, try;

	if (WARN_ON(!msgs || num < 1))
		return -EINVAL;
	// ��������Ƿ� suspend,���˳���
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
	//  i2c_trace_msg_key��tracepoint i2c_transfer����ʱ���á�����һ���ڲ���Ҫʱ����ִ��forѭ������Ч������
	if (static_branch_unlikely(&i2c_trace_msg_key)) {
		int i;
		for (i = 0; i < num; i++)
			if (msgs[i].flags & I2C_M_RD)
				trace_i2c_read(adap, &msgs[i], i);
			else
				trace_i2c_write(adap, &msgs[i], i);
	}

	/* Retry automatically on arbitration loss */
	// �ٲ���ʧ���Զ�����
	orig_jiffies = jiffies;
	for (ret = 0, try = 0; try <= adap->retries; try++) {
		// �ص�����ָ���Ĵ��䷽����
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
* i2c_transfer��ִ�е�������ϵ�I2C��Ϣ
* @adap: I2C���ߵľ��
* @msgs:�ڷ�����ֹ������STOP֮ǰ����Ҫִ��һ���������Ϣ;ÿ����Ϣ���Կ�ͷ��ͷ��
* @num:Ҫִ�е���Ϣ������
*
* ���ظ���errno������ִ�е���Ϣ����
*
* ��ע�⣬û��Ҫ��ÿ����Ϣ�����͵���ͬ�Ĵ�����ַ���������������ģ�͡�
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
/*����Ĺ��ϱ���ģ�ͱȽ���:
* -�������յ�һ������󣬴�һ��slave��N�ֽڣ�û�а취���桰N����
* -�����ǵõ�һ��NAK�󣬴���N���ֽڸ�һ��slave��û�а취���桰N��������
* �������̼���ִ�������Ϣ�����ಿ��(��������ʵ�����Ӧ)��
* -���磬����num��Ϊ2ʱ�����ǳɹ�������˵�һ����Ϣ�����ڵڶ�����Ϣ�г����˲��ִ���
* �����Ӧ�ý��䱨��Ϊ1(�ڶ�����Ϣ�Ķ���״̬)����errno(��һ����Ϣ�Ķ���״̬)��
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
* i2c_transfer_buffer_flags������һ��I2C��Ϣ�������ݴ��䵽��������ӻ���������
* @client:���豸�ľ��
* @buf:���ݴ洢�ĵط�
* @count:Ҫ��������ֽڣ�����С��64k����Ϊmsg��len��u16
* @flags:������Ϣ�ı�־������ I2C_M_RD ���ڶ�ȡ
*
*���ظ���errno��������ֽ�����
*/
// f����lags����д���豸��һ��������Ϣ
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
	// ���һ������(��1 msg����)������#�ֽڴ��䣬���������롣
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
* i2c_get_device_id -��ȡ�豸�������̡�����id��ģ���޶�	
* @client:Ҫ��ѯ���豸	
* @id:��ѯ����Ϣ	
*	
* ���󷵻ظ���errno���ɹ�����0��	
*/
int i2c_get_device_id(const struct i2c_client *client,
		      struct i2c_device_identity *id)
{
	struct i2c_adapter *adap = client->adapter;
	union i2c_smbus_data raw_id;
	int ret;
	// ������֧��������Ҫ�� I2C_FUNC_SMBUS_READ_I2C_BLOCK,����֧���˳���
	if (!i2c_check_functionality(adap, I2C_FUNC_SMBUS_READ_I2C_BLOCK))
		return -EOPNOTSUPP;

	raw_id.block[0] = 3;
	// ִ��SMBusЭ�����
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
* ������Ĭ��̽�⹦�ܣ���Ҫ��SMBus��ء�
* Ĭ�ϵ�̽�ⷽ���ǿ���д��������������һ��״̬�����������ƻ�24RF08 eeprom�����һ����ܲ�����ת��д����һЩeeprom��
* ���Զ��ڵ�ַ��Χ0x30-0x37��0x50-0x5f������ʹ��һ�����ֽڶ�ȡ��
* ���⣬һЩ������������û��ʵ�ֿ���д�룬�������������������Ҳ����˵��ֽڶ�ȡ��
* ��x86�ϣ�FSCӲ�����оƬ������һ�������������Ҫ���ڶ�ȡ�ֽ�(��ַ0x73)��
* ���˵��ǣ���Щ��Ψһ��֪����PCӲ����ʹ�����I2C��ַ��оƬ��
* ���̽��ɹ�������1;���ʧ�ܣ�����0��
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

// ����i2c_client�豸��adapter��addr��Ա��Ϣ����豸�ĵ�i2c���ߵ�ע�ᣬ��������driver�İ󶨡�
static int i2c_detect_address(struct i2c_client *temp_client,
			      struct i2c_driver *driver)
{
	struct i2c_board_info info;
	struct i2c_adapter *adapter = temp_client->adapter;
	int addr = temp_client->addr;
	int err;

	/* Make sure the address is valid */
	// ȷ����ַ����Ч��
	err = i2c_check_7bit_addr_validity_strict(addr);
	if (err) {
		dev_warn(&adapter->dev, "Invalid probe address 0x%02x\n",
			 addr);
		return err;
	}

	/* Skip if already in use (7 bit, no need to encode flags) */
	// ����Ѿ�ʹ�����˳�(7λ������Ҫ������)
	if (i2c_check_addr_busy(adapter, addr))
		return 0;

	/* Make sure there is something at this address */
	// �����ַһ����ʲô����.ȷʵ�ҽ��˿�������������һ���豸��
	if (!i2c_default_probe(adapter, addr))
		return 0;

	/* Finally call the custom detection function */
	// �������Զ����⺯��
	memset(&info, 0, sizeof(struct i2c_board_info));
	info.addr = addr;
	//�ص��豸����ʵ�ֵ�̽�ⷽ������֪�����Ѿ�̽�⵽���豸��ַ��
	err = driver->detect(temp_client, &info);
	if (err) {
		/* -ENODEV is returned if the detection fails. We catch it
		   here as this isn't an error. */
		return err == -ENODEV ? 0 : err;
	}

	/* Consistency check */
	// һ���Լ���
	if (info.type[0] == '\0') {
		// ���ʧ�ܡ�
		dev_err(&adapter->dev,
			"%s detection function provided no name for 0x%x\n",
			driver->driver.name, addr);
	} else {
		struct i2c_client *client;

		/* Detection succeeded, instantiate the device */
		// ���ɹ���ʵ�����豸
		if (adapter->class & I2C_CLASS_DEPRECATED)
			dev_warn(&adapter->dev,
				"This adapter will soon drop class based instantiation of devices. "
				"Please make sure client 0x%02x gets instantiated by other means. "
				"Check 'Documentation/i2c/instantiating-devices' for details.\n",
				info.addr);

		dev_dbg(&adapter->dev, "Creating %s at 0x%02x\n",
			info.type, info.addr);
		// ͨ�� i2c_board_info ʵ����һ��i2c�豸������������adap��ע�ᵽi2c���ߡ�
		client = i2c_new_device(adapter, &info);
		if (client)
			// ����豸�������İ󶨡�
			list_add_tail(&client->detected, &driver->clients);
		else
			dev_err(&adapter->dev, "Failed creating %s at 0x%02x\n",
				info.type, info.addr);
	}
	return 0;
}

// �����豸���� i2c_driver->address_list �ṩ���豸��ַ�б������������豸����ͬ�������£�
// ���ʵ������ʵ�ֽ��豸�ĵ�i2c���ߵ�ע�ᣬ����������driver�İ󶨡�
static int i2c_detect(struct i2c_adapter *adapter, struct i2c_driver *driver)
{
	const unsigned short *address_list;
	struct i2c_client *temp_client;
	int i, err = 0;
	// ����ض�����������������
	int adap_id = i2c_adapter_id(adapter);

	address_list = driver->address_list;
	if (!driver->detect || !address_list)
		return 0;

	/* Warn that the adapter lost class based instantiation */
	// ������������ʧ�˻������ʵ����
	if (adapter->class == I2C_CLASS_DEPRECATED) {
		dev_dbg(&adapter->dev,
			"This adapter dropped support for I2C classes and won't auto-detect %s devices anymore. "
			"If you need it, check 'Documentation/i2c/instantiating-devices' for alternatives.\n",
			driver->driver.name);
		return 0;
	}

	/* Stop here if the classes do not match */
	// ����಻ƥ�䣬���ڴ�ֹͣ
	if (!(adapter->class & driver->class))
		return 0;

	/* Set up a temporary client to help detect callback */
	// ����һ����ʱ�ͻ������������ص�.
	temp_client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (!temp_client)
		return -ENOMEM;
	temp_client->adapter = adapter;

	for (i = 0; address_list[i] != I2C_CLIENT_END; i += 1) {
		dev_dbg(&adapter->dev,
			"found normal entry for adapter %d, addr 0x%02x\n",
			adap_id, address_list[i]);
		temp_client->addr = address_list[i];
		// ����i2c_client�豸��adapter��addr��Ա��Ϣ����豸�ĵ�i2c���ߵ�ע�ᣬ��������driver�İ󶨡�
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
* ����㲻֪��I2C�豸��ȷ�е�ַ������ʹ��������壬������̽����ܵĵ�ַ�б��е��豸�Ƿ���ڡ�
* ��̽�롱�ص������ǿ�ѡ�ġ�����ṩ����������̽��ɹ�ʱ���뷵��1�����򷵻�0��
* ���û���ṩ����ʹ��Ĭ��̽�ⷽ����
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
		// ����ַ������(7λ������Ҫ�����־)
		if (i2c_check_addr_busy(adap, addr_list[i])) {
			dev_dbg(&adap->dev,
				"Address 0x%02x already in use, not probing\n",
				addr_list[i]);
			continue;
		}

		/* Test address responsiveness */
		// ���Ե�ַ��Ӧ����,���̽�⵽Ӵ��Ӧ���豸����break.
		if (probe(adap, addr_list[i]))
			break;
	}

	if (addr_list[i] == I2C_CLIENT_END) {
		dev_dbg(&adap->dev, "Probing failed, no device found\n");
		return NULL;
	}

	info->addr = addr_list[i];
	// ͨ�� i2c_board_info ʵ����һ��i2c�豸������������adap��ע�ᵽi2c���ߡ�
	return i2c_new_device(adap, info);
}
EXPORT_SYMBOL_GPL(i2c_new_probed_device);

// ͨ�����ߣ�����������id���i2c_adapter����
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

// ����i2c_adapter��ģ�͵����ü�����
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
* i2c_get_dma_safe_msg_buf()����Ϊ������i2c_msg��ȡһ��DMA��ȫ������	
* @msg:Ҫ������Ϣ	
* @threshold:ʹ��DMA���������С�ֽ���������Ӧ����1��
* ����:NULL���û�л��DMA��ȫ����������PIOһ��ʹ��msg->buf��
* ����DMAһ��ʹ�õ���Чָ�롣ʹ��֮��ͨ������i2c_put_dma_safe_msg_buf()���ͷ�����
* �˺���ֻ�ܴӽ��������ĵ���!	
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
* i2c_put_dma_safe_msg_buf -�ͷ�DMA��ȫ����������i2c_msgͬ��
* @buf:��i2c_get_dma_safe_msg_buf()��õĻ�������������NULL��	
* @msg:��������Ӧ����Ϣ
* @xferred: bool��ʾ��Ϣ�Ƿ��ѱ�����
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
