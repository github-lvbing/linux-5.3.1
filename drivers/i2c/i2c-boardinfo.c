// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * i2c-boardinfo.c - collect pre-declarations of I2C devices
 */

#include <linux/export.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/property.h>
#include <linux/rwsem.h>
#include <linux/slab.h>

#include "i2c-core.h"


/* These symbols are exported ONLY FOR the i2c core.
 * No other users will be supported.
 * 这些符号仅用于i2c核心。不支持其他用户
 */

// 声明一个信号
DECLARE_RWSEM(__i2c_board_lock);
EXPORT_SYMBOL_GPL(__i2c_board_lock);

// 声明一个链表头，管理 struct i2c_devinfo 结构链表。
LIST_HEAD(__i2c_board_list);
EXPORT_SYMBOL_GPL(__i2c_board_list);

// 第一个动态总线编号；静态与动态总线号的分界线
int __i2c_first_dynamic_bus_num;
EXPORT_SYMBOL_GPL(__i2c_first_dynamic_bus_num);

/**
 * i2c_register_board_info - statically declare I2C devices
 * @busnum: identifies the bus to which these devices belong
 * @info: vector of i2c device descriptors
 * @len: how many descriptors in the vector; may be zero to reserve
 *	the specified bus number.
 *
 * Systems using the Linux I2C driver stack can declare tables of board info
 * while they initialize.  This should be done in board-specific init code
 * near arch_initcall() time, or equivalent, before any I2C adapter driver is
 * registered.  For example, mainboard init code could define several devices,
 * as could the init code for each daughtercard in a board stack.
 *
 * The I2C devices will be created later, after the adapter for the relevant
 * bus has been registered.  After that moment, standard driver model tools
 * are used to bind "new style" I2C drivers to the devices.  The bus number
 * for any device declared using this routine is not available for dynamic
 * allocation.
 *
 * The board info passed can safely be __initdata, but be careful of embedded
 * pointers (for platform_data, functions, etc) since that won't be copied.
 * Device properties are deep-copied though.
 */
/**
* i2c_register_board_info -静态声明I2C设备
* @busnum:标识这些设备所属的总线
* @info: i2c设备描述符的向量
* @len:向量中有多少个描述符;可为零预留指定的总线号
* 
* 使用Linux I2C驱动程序堆栈的系统可以在初始化时声明板信息表。
* 在注册任何I2C适配器驱动程序之前，这应该在arch_initcall()时间或等效时间附近的特定于主板的init代码中完成。
* 例如，主板init代码可以定义多个设备，就像可以为板堆栈中的每个子程序卡定义init代码一样。
*
* I2C设备将在相关总线的适配器注册之后创建。在那之后，标准驱动程序模型工具被用来
* 将“新风格”的I2C驱动程序绑定到设备上。使用此例程声明的任何设备的总线号都不能用于动态分配。
*
* 传递的板信息可以是安全的，但是要小心嵌入的指针(对于platform_data，函数等)，
* 因为它不会被复制。设备属性是深度复制的。
*/
int i2c_register_board_info(int busnum, struct i2c_board_info const *info, unsigned len)
{
	int status;

	down_write(&__i2c_board_lock);

	/* dynamic bus numbers will be assigned after the last static one */
	// 动态总线号将在最后一个静态总线号之后分配。
	if (busnum >= __i2c_first_dynamic_bus_num)
		__i2c_first_dynamic_bus_num = busnum + 1;

	for (status = 0; len; len--, info++) {
		struct i2c_devinfo	*devinfo;

		devinfo = kzalloc(sizeof(*devinfo), GFP_KERNEL);
		if (!devinfo) {
			pr_debug("i2c-core: can't register boardinfo!\n");
			status = -ENOMEM;
			break;
		}

		devinfo->busnum = busnum;
		devinfo->board_info = *info;

		if (info->properties) {
			devinfo->board_info.properties =
					property_entries_dup(info->properties);
			if (IS_ERR(devinfo->board_info.properties)) {
				status = PTR_ERR(devinfo->board_info.properties);
				kfree(devinfo);
				break;
			}
		}

		if (info->resources) {
			devinfo->board_info.resources =
				kmemdup(info->resources,
					info->num_resources *
						sizeof(*info->resources),
					GFP_KERNEL);
			if (!devinfo->board_info.resources) {
				status = -ENOMEM;
				kfree(devinfo);
				break;
			}
		}

		list_add_tail(&devinfo->list, &__i2c_board_list);
	}

	up_write(&__i2c_board_lock);

	return status;
}
