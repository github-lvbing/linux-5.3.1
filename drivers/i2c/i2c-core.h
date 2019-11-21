/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * i2c-core.h - interfaces internal to the I2C framework
 */

#include <linux/rwsem.h>

// 全局链表 __i2c_board_list 的节点结构，用于管理一系列的 i2c_board_info（i2c设备信息表） 结构
struct i2c_devinfo {
	struct list_head	list;  // 节点
	int			busnum;        // 将依附的 总线号。
	struct i2c_board_info	board_info;
};

/* board_lock protects board_list and first_dynamic_bus_num.
 * only i2c core components are allowed to use these symbols.
 */
// board_lock保护board_list和first_dynamic_bus_num。只有i2c核心组件可以使用这些符号。
extern struct rw_semaphore	__i2c_board_lock;
extern struct list_head	__i2c_board_list;
extern int		__i2c_first_dynamic_bus_num;

int i2c_check_7bit_addr_validity_strict(unsigned short addr);
int i2c_dev_irq_from_resources(const struct resource *resources,
			       unsigned int num_resources);

/*
 * We only allow atomic transfers for very late communication, e.g. to send
 * the powerdown command to a PMIC. Atomic transfers are a corner case and not
 * for generic use!
 */
/*
* 我们只允许在非常晚的时候进行原子传输，例如将powerdown命令发送到PMIC。
* 原子传输是一种特殊情况，不适合一般使用!	
*/
static inline bool i2c_in_atomic_xfer_mode(void)
{
	return system_state > SYSTEM_RUNNING && irqs_disabled();
}

static inline int __i2c_lock_bus_helper(struct i2c_adapter *adap)
{
	int ret = 0;

	if (i2c_in_atomic_xfer_mode()) {
		WARN(!adap->algo->master_xfer_atomic && !adap->algo->smbus_xfer_atomic,
		     "No atomic I2C transfer handler for '%s'\n", dev_name(&adap->dev));
		ret = i2c_trylock_bus(adap, I2C_LOCK_SEGMENT) ? 0 : -EAGAIN;
	} else {
		// 获得对I2C总线段的独占访问
		i2c_lock_bus(adap, I2C_LOCK_SEGMENT);
	}

	return ret;
}

static inline int __i2c_check_suspended(struct i2c_adapter *adap)
{
	if (test_bit(I2C_ALF_IS_SUSPENDED, &adap->locked_flags)) {
		if (!test_and_set_bit(I2C_ALF_SUSPEND_REPORTED, &adap->locked_flags))
			dev_WARN(&adap->dev, "Transfer while suspended\n");
		return -ESHUTDOWN;
	}

	return 0;
}

#ifdef CONFIG_ACPI
const struct acpi_device_id *
i2c_acpi_match_device(const struct acpi_device_id *matches,
		      struct i2c_client *client);
void i2c_acpi_register_devices(struct i2c_adapter *adap);

int i2c_acpi_get_irq(struct i2c_client *client);
#else /* CONFIG_ACPI */
static inline void i2c_acpi_register_devices(struct i2c_adapter *adap) { }
static inline const struct acpi_device_id *
i2c_acpi_match_device(const struct acpi_device_id *matches,
		      struct i2c_client *client)
{
	return NULL;
}

static inline int i2c_acpi_get_irq(struct i2c_client *client)
{
	return 0;
}
#endif /* CONFIG_ACPI */
extern struct notifier_block i2c_acpi_notifier;

#ifdef CONFIG_ACPI_I2C_OPREGION
int i2c_acpi_install_space_handler(struct i2c_adapter *adapter);
void i2c_acpi_remove_space_handler(struct i2c_adapter *adapter);
#else /* CONFIG_ACPI_I2C_OPREGION */
static inline int i2c_acpi_install_space_handler(struct i2c_adapter *adapter) { return 0; }
static inline void i2c_acpi_remove_space_handler(struct i2c_adapter *adapter) { }
#endif /* CONFIG_ACPI_I2C_OPREGION */

#ifdef CONFIG_OF
void of_i2c_register_devices(struct i2c_adapter *adap);
#else
static inline void of_i2c_register_devices(struct i2c_adapter *adap) { }
#endif
extern struct notifier_block i2c_of_notifier;
