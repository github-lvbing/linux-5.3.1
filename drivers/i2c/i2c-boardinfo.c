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
 * ��Щ���Ž�����i2c���ġ���֧�������û�
 */

// ����һ���ź�
DECLARE_RWSEM(__i2c_board_lock);
EXPORT_SYMBOL_GPL(__i2c_board_lock);

// ����һ������ͷ������ struct i2c_devinfo �ṹ����
LIST_HEAD(__i2c_board_list);
EXPORT_SYMBOL_GPL(__i2c_board_list);

// ��һ����̬���߱�ţ���̬�붯̬���ߺŵķֽ���
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
* i2c_register_board_info -��̬����I2C�豸
* @busnum:��ʶ��Щ�豸����������
* @info: i2c�豸������������
* @len:�������ж��ٸ�������;��Ϊ��Ԥ��ָ�������ߺ�
* 
* ʹ��Linux I2C���������ջ��ϵͳ�����ڳ�ʼ��ʱ��������Ϣ��
* ��ע���κ�I2C��������������֮ǰ����Ӧ����arch_initcall()ʱ����Чʱ�丽�����ض��������init��������ɡ�
* ���磬����init������Զ������豸���������Ϊ���ջ�е�ÿ���ӳ��򿨶���init����һ����
*
* I2C�豸����������ߵ�������ע��֮�󴴽�������֮�󣬱�׼��������ģ�͹��߱�����
* �����·�񡱵�I2C��������󶨵��豸�ϡ�ʹ�ô������������κ��豸�����ߺŶ��������ڶ�̬���䡣
*
* ���ݵİ���Ϣ�����ǰ�ȫ�ģ�����ҪС��Ƕ���ָ��(����platform_data��������)��
* ��Ϊ�����ᱻ���ơ��豸��������ȸ��Ƶġ�
*/
int i2c_register_board_info(int busnum, struct i2c_board_info const *info, unsigned len)
{
	int status;

	down_write(&__i2c_board_lock);

	/* dynamic bus numbers will be assigned after the last static one */
	// ��̬���ߺŽ������һ����̬���ߺ�֮����䡣
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
