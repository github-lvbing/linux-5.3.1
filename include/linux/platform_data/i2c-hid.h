/*
 * HID over I2C protocol implementation
 *
 * Copyright (c) 2012 Benjamin Tissoires <benjamin.tissoires@gmail.com>
 * Copyright (c) 2012 Ecole Nationale de l'Aviation Civile, France
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 */

#ifndef __LINUX_I2C_HID_H
#define __LINUX_I2C_HID_H

#include <linux/regulator/consumer.h>
#include <linux/types.h>

/**
 * struct i2chid_platform_data - used by hid over i2c implementation.
 * @hid_descriptor_address: i2c register where the HID descriptor is stored.
 * @supplies: regulators for powering on the device.
 * @post_power_delay_ms: delay after powering on before device is usable.
 *
 * Note that it is the responsibility of the platform driver (or the acpi 5.0
 * driver, or the flattened device tree) to setup the irq related to the gpio in
 * the struct i2c_board_info.
 * The platform driver should also setup the gpio according to the device:
 *
 * A typical example is the following:
 *	irq = gpio_to_irq(intr_gpio);
 *	hkdk4412_i2c_devs5[0].irq = irq; // store the irq in i2c_board_info
 *	gpio_request(intr_gpio, "elan-irq");
 *	s3c_gpio_setpull(intr_gpio, S3C_GPIO_PULL_UP);
 */
/**
* struct i2chid_platform_data -用于i2c实现。
* @hid_descriptor_address:存储HID描述符的i2c寄存器。
* @supplies:为设备供电的调节器。
* @post_power_delay_ms:在设备可用之前，打开电源后的延迟。
*
* 请注意，平台驱动程序(或acpi 5.0驱动程序，或扁平设备树)负责在结构i2c_board_info中设置与gpio相关的irq。
* 平台驱动还应该根据设备设置gpio:
一个典型的例子如下:
* irq = gpio_to_irq(intr_gpio);
* hkdk4412_i2c_devs5[0].irq = irq;  //将irq保存在i2c_board_info gpio_request中(intr_gpio， "elan-irq");
* s3c_gpio_setpull (intr_gpio S3C_GPIO_PULL_UP);
*/
struct i2c_hid_platform_data {
	u16 hid_descriptor_address;
	struct regulator_bulk_data supplies[2];
	int post_power_delay_ms;
};

#endif /* __LINUX_I2C_HID_H */
