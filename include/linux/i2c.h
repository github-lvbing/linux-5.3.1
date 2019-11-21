/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * i2c.h - definitions for the Linux i2c bus interface
 * Copyright (C) 1995-2000 Simon G. Vogl
 * Copyright (C) 2013-2019 Wolfram Sang <wsa@the-dreams.de>
 *
 * With some changes from Ky枚sti M盲lkki <kmalkki@cc.hut.fi> and
 * Frodo Looijaard <frodol@dds.nl>
 */
#ifndef _LINUX_I2C_H
#define _LINUX_I2C_H

#include <linux/acpi.h>		/* for acpi_handle */
#include <linux/mod_devicetable.h>
#include <linux/device.h>	/* for struct device */
#include <linux/sched.h>	/* for completion */
#include <linux/mutex.h>
#include <linux/rtmutex.h>
#include <linux/irqdomain.h>		/* for Host Notify IRQ */
#include <linux/of.h>		/* for struct device_node */
#include <linux/swab.h>		/* for swab16 */
#include <uapi/linux/i2c.h>

extern struct bus_type i2c_bus_type;
extern struct device_type i2c_adapter_type;
extern struct device_type i2c_client_type;

/* --- General options ------------------------------------------------	*/

struct i2c_msg;
struct i2c_algorithm;
struct i2c_adapter;
struct i2c_client;
struct i2c_driver;
struct i2c_device_identity;
union i2c_smbus_data;
struct i2c_board_info;
enum i2c_slave_event;
typedef int (*i2c_slave_cb_t)(struct i2c_client *client,
			      enum i2c_slave_event event, u8 *val);

struct module;
struct property_entry;

#if IS_ENABLED(CONFIG_I2C)
/*
 * The master routines are the ones normally used to transmit data to devices
 * on a bus (or read from them). Apart from two basic transfer functions to
 * transmit one message at a time, a more complex version can be used to
 * transmit an arbitrary number of messages without interruption.
 * @count must be be less than 64k since msg.len is u16.
 */
extern int i2c_transfer_buffer_flags(const struct i2c_client *client,
				     char *buf, int count, u16 flags);

/**
 * i2c_master_recv - issue a single I2C message in master receive mode
 * @client: Handle to slave device
 * @buf: Where to store data read from slave
 * @count: How many bytes to read, must be less than 64k since msg.len is u16
 *
 * Returns negative errno, or else the number of bytes read.
 */
static inline int i2c_master_recv(const struct i2c_client *client,
				  char *buf, int count)
{
	return i2c_transfer_buffer_flags(client, buf, count, I2C_M_RD);
};

/**
 * i2c_master_recv_dmasafe - issue a single I2C message in master receive mode
 *			     using a DMA safe buffer
 * @client: Handle to slave device
 * @buf: Where to store data read from slave, must be safe to use with DMA
 * @count: How many bytes to read, must be less than 64k since msg.len is u16
 *
 * Returns negative errno, or else the number of bytes read.
 */
static inline int i2c_master_recv_dmasafe(const struct i2c_client *client,
					  char *buf, int count)
{
	return i2c_transfer_buffer_flags(client, buf, count,
					 I2C_M_RD | I2C_M_DMA_SAFE);
};

/**
 * i2c_master_send - issue a single I2C message in master transmit mode
 * @client: Handle to slave device
 * @buf: Data that will be written to the slave
 * @count: How many bytes to write, must be less than 64k since msg.len is u16
 *
 * Returns negative errno, or else the number of bytes written.
 */
static inline int i2c_master_send(const struct i2c_client *client,
				  const char *buf, int count)
{
	return i2c_transfer_buffer_flags(client, (char *)buf, count, 0);
};

/**
 * i2c_master_send_dmasafe - issue a single I2C message in master transmit mode
 *			     using a DMA safe buffer
 * @client: Handle to slave device
 * @buf: Data that will be written to the slave, must be safe to use with DMA
 * @count: How many bytes to write, must be less than 64k since msg.len is u16
 *
 * Returns negative errno, or else the number of bytes written.
 */
static inline int i2c_master_send_dmasafe(const struct i2c_client *client,
					  const char *buf, int count)
{
	return i2c_transfer_buffer_flags(client, (char *)buf, count,
					 I2C_M_DMA_SAFE);
};

/* Transfer num messages.
 */
extern int i2c_transfer(struct i2c_adapter *adap, struct i2c_msg *msgs,
			int num);
/* Unlocked flavor */
extern int __i2c_transfer(struct i2c_adapter *adap, struct i2c_msg *msgs,
			  int num);

/* This is the very generalized SMBus access routine. You probably do not
   want to use this, though; one of the functions below may be much easier,
   and probably just as fast.
   Note that we use i2c_adapter here, because you do not need a specific
   smbus adapter to call this function. */
s32 i2c_smbus_xfer(struct i2c_adapter *adapter, u16 addr,
		   unsigned short flags, char read_write, u8 command,
		   int protocol, union i2c_smbus_data *data);

/* Unlocked flavor */
s32 __i2c_smbus_xfer(struct i2c_adapter *adapter, u16 addr,
		     unsigned short flags, char read_write, u8 command,
		     int protocol, union i2c_smbus_data *data);

/* Now follow the 'nice' access routines. These also document the calling
   conventions of i2c_smbus_xfer. */

extern s32 i2c_smbus_read_byte(const struct i2c_client *client);
extern s32 i2c_smbus_write_byte(const struct i2c_client *client, u8 value);
extern s32 i2c_smbus_read_byte_data(const struct i2c_client *client,
				    u8 command);
extern s32 i2c_smbus_write_byte_data(const struct i2c_client *client,
				     u8 command, u8 value);
extern s32 i2c_smbus_read_word_data(const struct i2c_client *client,
				    u8 command);
extern s32 i2c_smbus_write_word_data(const struct i2c_client *client,
				     u8 command, u16 value);

static inline s32
i2c_smbus_read_word_swapped(const struct i2c_client *client, u8 command)
{
	s32 value = i2c_smbus_read_word_data(client, command);

	return (value < 0) ? value : swab16(value);
}

static inline s32
i2c_smbus_write_word_swapped(const struct i2c_client *client,
			     u8 command, u16 value)
{
	return i2c_smbus_write_word_data(client, command, swab16(value));
}

/* Returns the number of read bytes */
extern s32 i2c_smbus_read_block_data(const struct i2c_client *client,
				     u8 command, u8 *values);
extern s32 i2c_smbus_write_block_data(const struct i2c_client *client,
				      u8 command, u8 length, const u8 *values);
/* Returns the number of read bytes */
extern s32 i2c_smbus_read_i2c_block_data(const struct i2c_client *client,
					 u8 command, u8 length, u8 *values);
extern s32 i2c_smbus_write_i2c_block_data(const struct i2c_client *client,
					  u8 command, u8 length,
					  const u8 *values);
extern s32
i2c_smbus_read_i2c_block_data_or_emulated(const struct i2c_client *client,
					  u8 command, u8 length, u8 *values);
int i2c_get_device_id(const struct i2c_client *client,
		      struct i2c_device_identity *id);
#endif /* I2C */

/**
 * struct i2c_device_identity - i2c client device identification
 * @manufacturer_id: 0 - 4095, database maintained by NXP
 * @part_id: 0 - 511, according to manufacturer
 * @die_revision: 0 - 7, according to manufacturer
 */
struct i2c_device_identity {
	u16 manufacturer_id;
#define I2C_DEVICE_ID_NXP_SEMICONDUCTORS                0
#define I2C_DEVICE_ID_NXP_SEMICONDUCTORS_1              1
#define I2C_DEVICE_ID_NXP_SEMICONDUCTORS_2              2
#define I2C_DEVICE_ID_NXP_SEMICONDUCTORS_3              3
#define I2C_DEVICE_ID_RAMTRON_INTERNATIONAL             4
#define I2C_DEVICE_ID_ANALOG_DEVICES                    5
#define I2C_DEVICE_ID_STMICROELECTRONICS                6
#define I2C_DEVICE_ID_ON_SEMICONDUCTOR                  7
#define I2C_DEVICE_ID_SPRINTEK_CORPORATION              8
#define I2C_DEVICE_ID_ESPROS_PHOTONICS_AG               9
#define I2C_DEVICE_ID_FUJITSU_SEMICONDUCTOR            10
#define I2C_DEVICE_ID_FLIR                             11
#define I2C_DEVICE_ID_O2MICRO                          12
#define I2C_DEVICE_ID_ATMEL                            13
#define I2C_DEVICE_ID_NONE                         0xffff
	u16 part_id;
	u8 die_revision;
};

enum i2c_alert_protocol {
	I2C_PROTOCOL_SMBUS_ALERT,
	I2C_PROTOCOL_SMBUS_HOST_NOTIFY,
};

/**
 * struct i2c_driver - represent an I2C device driver
 * @class: What kind of i2c device we instantiate (for detect)
 * @probe: Callback for device binding - soon to be deprecated
 * @probe_new: New callback for device binding
 * @remove: Callback for device unbinding
 * @shutdown: Callback for device shutdown
 * @alert: Alert callback, for example for the SMBus alert protocol
 * @command: Callback for bus-wide signaling (optional)
 * @driver: Device driver model driver
 * @id_table: List of I2C devices supported by this driver
 * @detect: Callback for device detection
 * @address_list: The I2C addresses to probe (for detect)
 * @clients: List of detected clients we created (for i2c-core use only)
 * @disable_i2c_core_irq_mapping: Tell the i2c-core to not do irq-mapping
 *
 * The driver.owner field should be set to the module owner of this driver.
 * The driver.name field should be set to the name of this driver.
 *
 * For automatic device detection, both @detect and @address_list must
 * be defined. @class should also be set, otherwise only devices forced
 * with module parameters will be created. The detect function must
 * fill at least the name field of the i2c_board_info structure it is
 * handed upon successful detection, and possibly also the flags field.
 *
 * If @detect is missing, the driver will still work fine for enumerated
 * devices. Detected devices simply won't be supported. This is expected
 * for the many I2C/SMBus devices which can't be detected reliably, and
 * the ones which can always be enumerated in practice.
 *
 * The i2c_client structure which is handed to the @detect callback is
 * not a real i2c_client. It is initialized just enough so that you can
 * call i2c_smbus_read_byte_data and friends on it. Don't do anything
 * else with it. In particular, calling dev_dbg and friends on it is
 * not allowed.
 */
 /**
 * struct i2c_driver—表示一个I2C设备驱动程序
 * @class:我们实例化什么样的i2c设备(用于检测)
 * @probe:设备绑定的回调——不久将被弃用
 * @probe_new:设备绑定的新回调
 * @remove:设备解除绑定的回调
 * @shutdown:设备关闭回调
 * @alert:警报回调，例如用于SMBus警报协议
 * @command:总线范围的回调信号(可选)
 * @driver:设备驱动程序模型驱动程序
 * @id_table:该驱动程序支持的I2C设备列表
 * @detect:设备检测回调
 * @address_list:要探测的I2C地址(用于检测)
 * @clients:我们创建的检测到的客户端列表(仅用于i2c-core)
 * @disable_i2c_core_irq_mapping:告诉i2c-core不要做irq-mapping
 *
 * 驱动应将owner字段设置为此驱动程序的模块所有者。
 * name字段应该设置为这个驱动程序的名称。
 * 对于自动设备检测，必须定义@detect和@address_list。还应该设置@class，否则只创建带有模块参数的设备。
 * 检测函数必须至少填充它在成功检测时传递的i2c_board_info结构的name字段，也可能填充flags字段。
 *
 * 如果没有@detect，驱动程序仍然可以在枚举的设备中正常工作。检测到的设备将不受支持。这对于许多不能可靠地检测到的I2C/SMBus设备以及在实践中总是可以枚举的那些设备来说是期望的。
 * 传递给@detect回调的i2c_client结构不是真正的i2c_client。它的初始化程度刚刚好，因此可以在上面调用i2c_smbus_read_byte_data和friends。不要用它做其他任何事。特别是，不允许在上面调用dev_dbg和好友。
 */
struct i2c_driver {
	unsigned int class;  // such as struct i2c_adapter class成员.eg: I2C_CLASS_DDC

	/* Standard driver model interfaces */  //标准驱动程序模型接口
	int (*probe)(struct i2c_client *client, const struct i2c_device_id *id);
	int (*remove)(struct i2c_client *client);

	/* New driver model interface to aid the seamless removal of the
	 * current probe()'s, more commonly unused than used second parameter.
	 */ // 新的驱动程序模型接口，以帮助无缝删除当前探测器()的，通常不使用比使用的第二个参数。
	int (*probe_new)(struct i2c_client *client);

	/* driver model interfaces that don't relate to enumeration  */
	// 与枚举无关的驱动程序模型接口
	void (*shutdown)(struct i2c_client *client);

	/* Alert callback, for example for the SMBus alert protocol.
	 * The format and meaning of the data value depends on the protocol.
	 * For the SMBus alert protocol, there is a single bit of data passed
	 * as the alert response's low bit ("event flag").
	 * For the SMBus Host Notify protocol, the data corresponds to the
	 * 16-bit payload data reported by the slave device acting as master.
	 */
	// 警报回调，例如用于SMBus警报协议。数据值的格式和含义取决于协议。
	// 对于SMBus警报协议，有一个单一位的数据作为警报响应的低位(“事件标志”)传递。
	// 对于SMBus主机通知协议，数据对应于从设备作为主设备报告的16位有效载荷数据。
	void (*alert)(struct i2c_client *client, enum i2c_alert_protocol protocol,
		      unsigned int data);

	/* a ioctl like command that can be used to perform specific functions
	 * with the device.
	 */
	// 类似ioctl的命令，可用于执行设备的特定功能
	int (*command)(struct i2c_client *client, unsigned int cmd, void *arg);

	struct device_driver driver;
	const struct i2c_device_id *id_table; // 该驱动程序支持的I2C设备列表

	/* Device detection callback for automatic device creation */
	// 用于自动设备创建的设备检测回调
	int (*detect)(struct i2c_client *client, struct i2c_board_info *info);
	const unsigned short *address_list;  // 要探测的I2C地址(用于检测),被上面的api detect()使用。I2C_CLIENT_END结尾。
	struct list_head clients;  // 链表，用于管理本驱动的i2c设备。

	bool disable_i2c_core_irq_mapping;
};
#define to_i2c_driver(d) container_of(d, struct i2c_driver, driver)

/**
 * struct i2c_client - represent an I2C slave device
 * @flags: see I2C_CLIENT_* for possible flags
 * @addr: Address used on the I2C bus connected to the parent adapter.
 * @name: Indicates the type of the device, usually a chip name that's
 *	generic enough to hide second-sourcing and compatible revisions.
 * @adapter: manages the bus segment hosting this I2C device
 * @dev: Driver model device node for the slave.
 * @irq: indicates the IRQ generated by this device (if any)
 * @detected: member of an i2c_driver.clients list or i2c-core's
 *	userspace_devices list
 * @slave_cb: Callback when I2C slave mode of an adapter is used. The adapter
 *	calls it to pass on slave events to the slave driver.
 *
 * An i2c_client identifies a single device (i.e. chip) connected to an
 * i2c bus. The behaviour exposed to Linux is defined by the driver
 * managing the device.
 */
 /**
 * struct i2c_client—表示一个I2C从设备
 * @flags:查看I2C_CLIENT_*可能的标志
 * @addr:用于连接到父适配器的I2C总线上的地址。
 * @name:表示设备的类型，通常是一个芯片名称，它足够通用，可以隐藏二次源和兼容的修订。
 * @adapter:管理承载这个I2C设备的总线段
 * @dev:驱动程序模型设备节点的奴隶。
 * @irq:表示该设备产生的IRQ(如果有的话)
 * @detect: i2c_driver的成员。clients list或i2c-core的userspace_devices列表
 * @slave_cb:使用适配器的I2C从属模式时的回调。适配器调用它来将从事件传递给从驱动程序。
 *
 * i2c_client标识连接到i2c总线的单个设备(即芯片)。暴露给Linux的行为是由管理设备的驱动程序定义的。
 */
 /*使用数据包错误检查*/

struct i2c_client {
	unsigned short flags;		    /* div., see below		*/
#define I2C_CLIENT_PEC		0x04	/* Use Packet Error Checking */           // 使用包错误检查
#define I2C_CLIENT_TEN		0x10	/* we have a ten bit chip address */      // 我们有一个10位的芯片地址
					/* Must equal I2C_M_TEN below */
#define I2C_CLIENT_SLAVE	0x20	/* we are the slave */                    // 我们是从设备
#define I2C_CLIENT_HOST_NOTIFY	0x40	/* We want to use I2C host notify */  // 我们想使用I2C主机notify
#define I2C_CLIENT_WAKE		0x80	/* for board_info; true iff can wake */   // board_info;若可以醒来为true
#define I2C_CLIENT_SCCB		0x9000	/* Use Omnivision SCCB protocol */        // 使用全景SCCB协议
					/* Must match I2C_M_STOP|IGNORE_NAK */

	unsigned short addr;		/* chip address - NOTE: 7bit	*/     // 设备的 _LOWER_ 7 bits 物理地址
					/* addresses are stored in the	*/
					/* _LOWER_ 7 bits		*/
	char name[I2C_NAME_SIZE];                                           // 设备的名字标识（可能来自struct i2c_board_info.name）
	struct i2c_adapter *adapter;	/* the adapter we sit on	*/      // 依附的适配器
	struct device dev;		/* the device structure		*/              // 设备结构
	int init_irq;			/* irq set at initialization	*/          // 初始化时设置irq
	int irq;			/* irq issued by device		*/                  // 由设备发出的irq
	struct list_head detected;  // 它依附设备驱动，用于被struct i2c_driver 探测和管理支持和驱动。
#if IS_ENABLED(CONFIG_I2C_SLAVE)
	i2c_slave_cb_t slave_cb;	/* callback for slave mode	*/          // 从模式回调
#endif
};

// 根据设备struct device 指针获得其外围封装的对象（struct i2c_client）的地址。
#define to_i2c_client(d) container_of(d, struct i2c_client, dev)

extern struct i2c_client *i2c_verify_client(struct device *dev);
extern struct i2c_adapter *i2c_verify_adapter(struct device *dev);
extern const struct i2c_device_id *i2c_match_id(const struct i2c_device_id *id,
					const struct i2c_client *client);

static inline struct i2c_client *kobj_to_i2c_client(struct kobject *kobj)
{
	struct device * const dev = container_of(kobj, struct device, kobj);
	return to_i2c_client(dev);
}

// 获得i2c设备的驱动的数据           struct i2c_client->struct device->void	 *driver_data。
static inline void *i2c_get_clientdata(const struct i2c_client *dev)
{
	return dev_get_drvdata(&dev->dev);
}

// 将data 记录在 struct i2c_client  dev->dev->driver_data（设备的驱动数据）中。
static inline void i2c_set_clientdata(struct i2c_client *dev, void *data)
{
	dev_set_drvdata(&dev->dev, data);
}

/* I2C slave support */

#if IS_ENABLED(CONFIG_I2C_SLAVE)
enum i2c_slave_event {
	I2C_SLAVE_READ_REQUESTED,
	I2C_SLAVE_WRITE_REQUESTED,
	I2C_SLAVE_READ_PROCESSED,
	I2C_SLAVE_WRITE_RECEIVED,
	I2C_SLAVE_STOP,
};

extern int i2c_slave_register(struct i2c_client *client, i2c_slave_cb_t slave_cb);
extern int i2c_slave_unregister(struct i2c_client *client);
extern bool i2c_detect_slave_mode(struct device *dev);

static inline int i2c_slave_event(struct i2c_client *client,
				  enum i2c_slave_event event, u8 *val)
{
	return client->slave_cb(client, event, val);
}
#else
static inline bool i2c_detect_slave_mode(struct device *dev) { return false; }
#endif

/**
 * struct i2c_board_info - template for device creation    
 * @type: chip type, to initialize i2c_client.name           
 * @flags: to initialize i2c_client.flags                   
 * @addr: stored in i2c_client.addr                         
 * @dev_name: Overrides the default <busnr>-<addr> dev_name if set  
 * @platform_data: stored in i2c_client.dev.platform_data    
 * @of_node: pointer to OpenFirmware device node             
 * @fwnode: device node supplied by the platform firmware    
 * @properties: additional device properties for the device  
 * @resources: resources associated with the device          
 * @num_resources: number of resources in the @resources array 
 * @irq: stored in i2c_client.irq                            
 *
 * I2C doesn't actually support hardware probing, although controllers and
 * devices may be able to use I2C_SMBUS_QUICK to tell whether or not there's
 * a device at a given address.  Drivers commonly need more information than
 * that, such as chip type, configuration, associated IRQ, and so on.
 *
 *   I2C实际上不支持硬件探测，尽管控制器和设备可以使用 I2C_SMBUS_QUICK 来判断给定地址是否有设备。
 *   驱动程序通常需要更多的信息，如芯片类型、配置、相关的IRQ等等。
 *
 * i2c_board_info is used to build tables of information listing I2C devices
 * that are present.  This information is used to grow the driver model tree.
 * For mainboards this is done statically using i2c_register_board_info();
 * bus numbers identify adapters that aren't yet available.  For add-on boards,
 * i2c_new_device() does this dynamically with the adapter already known.
 *
 *   i2c_board_info用于构建列出I2C设备的信息表。此信息用于生成驱动程序模型树。
 *   对于主板，这是使用i2c_register_board_info()静态完成的;总线号标识尚未可用的适配器。
 *   对于附加板，i2c_new_device()使用已知的适配器动态执行此操作。
 */
struct i2c_board_info {   //  设备创建模板
	char		type[I2C_NAME_SIZE];   //  芯片类型，用于初始化 i2c_client.name
	unsigned short	flags;       //  初始化 i2c_client.flags, val eg I2C_CLIENT_TEN
	unsigned short	addr;        //  存储在 i2c_client.addr中
	const char	*dev_name;       // 如果设置,覆盖默认的<busnr>-<addr>dev_name
	void		*platform_data;  //  存储在 i2c_client.dev.platform_data中
	struct device_node *of_node; //  指向OpenFirmware设备节点的指针
	struct fwnode_handle *fwnode;//  由平台固件提供的设备节点
	const struct property_entry *properties;  //  设备的附加属性
	const struct resource *resources;//  与设备相关的资源
	unsigned int	num_resources;   // resources数组中的资源数量
	int		irq;                     //  存储在i2c_client.irq中
};

/**
 * I2C_BOARD_INFO - macro used to list an i2c device and its address
 * @dev_type: identifies the device type
 * @dev_addr: the device's address on the bus.
 *
 * This macro initializes essential fields of a struct i2c_board_info,
 * declaring what has been provided on a particular board.  Optional
 * fields (such as associated irq, or device-specific platform_data)
 * are provided using conventional syntax.
 */
#define I2C_BOARD_INFO(dev_type, dev_addr) \
	.type = dev_type, .addr = (dev_addr)


#if IS_ENABLED(CONFIG_I2C)
/* Add-on boards should register/unregister their devices; e.g. a board
 * with integrated I2C, a config eeprom, sensors, and a codec that's
 * used in conjunction with the primary hardware.
 */
extern struct i2c_client *
i2c_new_device(struct i2c_adapter *adap, struct i2c_board_info const *info);

extern struct i2c_client *
i2c_new_client_device(struct i2c_adapter *adap, struct i2c_board_info const *info);

/* If you don't know the exact address of an I2C device, use this variant
 * instead, which can probe for device presence in a list of possible
 * addresses. The "probe" callback function is optional. If it is provided,
 * it must return 1 on successful probe, 0 otherwise. If it is not provided,
 * a default probing method is used.
 */
extern struct i2c_client *
i2c_new_probed_device(struct i2c_adapter *adap,
		      struct i2c_board_info *info,
		      unsigned short const *addr_list,
		      int (*probe)(struct i2c_adapter *adap, unsigned short addr));

/* Common custom probe functions */
extern int i2c_probe_func_quick_read(struct i2c_adapter *adap, unsigned short addr);

/* For devices that use several addresses, use i2c_new_dummy() to make
 * client handles for the extra addresses.
 */
extern struct i2c_client *
i2c_new_dummy(struct i2c_adapter *adap, u16 address);

extern struct i2c_client *
i2c_new_dummy_device(struct i2c_adapter *adapter, u16 address);

extern struct i2c_client *
devm_i2c_new_dummy_device(struct device *dev, struct i2c_adapter *adap, u16 address);

extern struct i2c_client *
i2c_new_secondary_device(struct i2c_client *client,
				const char *name,
				u16 default_addr);

extern void i2c_unregister_device(struct i2c_client *client);
#endif /* I2C */

/* Mainboard arch_initcall() code should register all its I2C devices.
 * This is done at arch_initcall time, before declaring any i2c adapters.
 * Modules for add-on boards must use other calls.
 */
#ifdef CONFIG_I2C_BOARDINFO
extern int
i2c_register_board_info(int busnum, struct i2c_board_info const *info,
			unsigned n);
#else
static inline int
i2c_register_board_info(int busnum, struct i2c_board_info const *info,
			unsigned n)
{
	return 0;
}
#endif /* I2C_BOARDINFO */

/**
 * struct i2c_algorithm - represent I2C transfer method
 * @master_xfer: Issue a set of i2c transactions to the given I2C adapter
 *   defined by the msgs array, with num messages available to transfer via
 *   the adapter specified by adap.
 * @master_xfer_atomic: same as @master_xfer. Yet, only using atomic context
 *   so e.g. PMICs can be accessed very late before shutdown. Optional.
 * @smbus_xfer: Issue smbus transactions to the given I2C adapter. If this
 *   is not present, then the bus layer will try and convert the SMBus calls
 *   into I2C transfers instead.
 * @smbus_xfer_atomic: same as @smbus_xfer. Yet, only using atomic context
 *   so e.g. PMICs can be accessed very late before shutdown. Optional.
 * @functionality: Return the flags that this algorithm/adapter pair supports
 *   from the I2C_FUNC_* flags.
 * @reg_slave: Register given client to I2C slave mode of this adapter
 * @unreg_slave: Unregister given client from I2C slave mode of this adapter
 *
 * The following structs are for those who like to implement new bus drivers:
 * i2c_algorithm is the interface to a class of hardware solutions which can
 * be addressed using the same bus algorithms - i.e. bit-banging or the PCF8584
 * to name two of the most common.
 *
 * The return codes from the @master_xfer{_atomic} fields should indicate the
 * type of error code that occurred during the transfer, as documented in the
 * Kernel Documentation file Documentation/i2c/fault-codes.
 */
/*
/**
* struct i2c_algorithm—表示I2C传输方法
* @master_xfer:向msgs数组定义的给定i2c适配器发出一组i2c事务，其中num消息可通过adap指定的适配器进行传输。
* @master_xfer_atomic:与@master_xfer相同。然而，只有使用原子上下文，例如PMICs，才能在关机前很晚访问。可选的。
* @smbus_xfer:向给定的I2C适配器发出smbus事务。如果不存在这种情况，则总线层将尝试将SMBus调用转换为I2C传输。
* @smbus_xfer_atomic:与@smbus_xfer相同。然而，只有使用原子上下文，例如PMICs，才能在关机前很晚访问。可选的。
* @ functions:从I2C_FUNC_*标志返回该算法/适配器对支持的标志。
* @reg_slave:将客户端注册到该适配器的I2C从模式
* @unreg_slave:从这个适配器的I2C从属模式中注销给定的客户端
*
* 以下结构是为那些喜欢实现新的总线驱动程序:i2c_algorithm是一类硬件解决方案的接口，可以使用相同的总线算法来解决，即位敲或PCF8584，说出两个最常见的。
*
* 来自@master_xfer{_atomic}字段的返回代码应该指示在传输过程中发生的错误代码类型，如内核文档文件Documentation/i2c/fault-codes中记录的那样。
*/
struct i2c_algorithm {
	/*
	 * If an adapter algorithm can't do I2C-level access, set master_xfer
	 * to NULL. If an adapter algorithm can do SMBus access, set
	 * smbus_xfer. If set to NULL, the SMBus protocol is simulated
	 * using common I2C messages.
	 *
	 * master_xfer should return the number of messages successfully
	 * processed, or a negative value on error
	 */
	/*
     * 如果适配器算法不能进行i2c级访问，则将master_xfer设置为NULL。
     * 如果适配器算法可以进行SMBus访问，则设置smbus_xfer。如果设置为NULL，则使用公共I2C消息模拟SMBus协议。
     * master_xfer应该返回成功处理的消息数量，或者错误时返回一个负值
     */
	int (*master_xfer)(struct i2c_adapter *adap, struct i2c_msg *msgs,
			   int num);
	int (*master_xfer_atomic)(struct i2c_adapter *adap,
				   struct i2c_msg *msgs, int num);      // 只有使用原子上下文
	int (*smbus_xfer)(struct i2c_adapter *adap, u16 addr,
			  unsigned short flags, char read_write,
			  u8 command, int size, union i2c_smbus_data *data);
	int (*smbus_xfer_atomic)(struct i2c_adapter *adap, u16 addr,
				 unsigned short flags, char read_write,
				 u8 command, int size, union i2c_smbus_data *data);  // 只有使用原子上下文

	/* To determine what the adapter supports */
    // 以确定适配器支持什么
	u32 (*functionality)(struct i2c_adapter *adap);

#if IS_ENABLED(CONFIG_I2C_SLAVE)
	int (*reg_slave)(struct i2c_client *client);
	int (*unreg_slave)(struct i2c_client *client);
#endif
};

/**
 * struct i2c_lock_operations - represent I2C locking operations
 * @lock_bus: Get exclusive access to an I2C bus segment
 * @trylock_bus: Try to get exclusive access to an I2C bus segment
 * @unlock_bus: Release exclusive access to an I2C bus segment
 *
 * The main operations are wrapped by i2c_lock_bus and i2c_unlock_bus.
 */
/**
 * struct i2c_lock_operations—表示I2C锁定操作
 * @lock_bus:获得对I2C总线段的独占访问
 * @trylock_bus:尝试获得对I2C总线段的独占访问
 * @unlock_bus:释放对I2C总线段的独占访问
 *
 * 主操作由i2c_lock_bus和i2c_unlock_bus封装。
 */
struct i2c_lock_operations {
	void (*lock_bus)(struct i2c_adapter *adapter, unsigned int flags);
	int (*trylock_bus)(struct i2c_adapter *adapter, unsigned int flags);
	void (*unlock_bus)(struct i2c_adapter *adapter, unsigned int flags);
};

/**
 * struct i2c_timings - I2C timing information
 * @bus_freq_hz: the bus frequency in Hz
 * @scl_rise_ns: time SCL signal takes to rise in ns; t(r) in the I2C specification
 * @scl_fall_ns: time SCL signal takes to fall in ns; t(f) in the I2C specification
 * @scl_int_delay_ns: time IP core additionally needs to setup SCL in ns
 * @sda_fall_ns: time SDA signal takes to fall in ns; t(f) in the I2C specification
 * @sda_hold_ns: time IP core additionally needs to hold SDA in ns
 */
struct i2c_timings {  // I2C计时信息
	u32 bus_freq_hz;  // 总线频率，单位为Hz
	u32 scl_rise_ns;  // SCL信号在ns中上升的时间;I2C规范中的t(r
	u32 scl_fall_ns;  // SCL信号在ns中下降的时间;I2C规范中的t(f
	u32 scl_int_delay_ns; // 时间IP核额外需要在ns设置SCL
	u32 sda_fall_ns;  // SDA信号落入ns的时间;I2C规范中的t(f
	u32 sda_hold_ns;  // 时间IP核额外需要持有SDA在ns
};

/**
 * struct i2c_bus_recovery_info - I2C bus recovery information
 * @recover_bus: Recover routine. Either pass driver's recover_bus() routine, or
 *	i2c_generic_scl_recovery().
 * @get_scl: This gets current value of SCL line. Mandatory for generic SCL
 *      recovery. Populated internally for generic GPIO recovery.
 * @set_scl: This sets/clears the SCL line. Mandatory for generic SCL recovery.
 *      Populated internally for generic GPIO recovery.
 * @get_sda: This gets current value of SDA line. This or set_sda() is mandatory
 *	for generic SCL recovery. Populated internally, if sda_gpio is a valid
 *	GPIO, for generic GPIO recovery.
 * @set_sda: This sets/clears the SDA line. This or get_sda() is mandatory for
 *	generic SCL recovery. Populated internally, if sda_gpio is a valid GPIO,
 *	for generic GPIO recovery.
 * @get_bus_free: Returns the bus free state as seen from the IP core in case it
 *	has a more complex internal logic than just reading SDA. Optional.
 * @prepare_recovery: This will be called before starting recovery. Platform may
 *	configure padmux here for SDA/SCL line or something else they want.
 * @unprepare_recovery: This will be called after completing recovery. Platform
 *	may configure padmux here for SDA/SCL line or something else they want.
 * @scl_gpiod: gpiod of the SCL line. Only required for GPIO recovery.
 * @sda_gpiod: gpiod of the SDA line. Only required for GPIO recovery.
 */
/**
* struct i2c_bus_recovery_info - I2C总线恢复信息
* @recover_bus:恢复例程。传递驱动程序的recover_bus()例程，或者i2c_generic_scl_recovery()。
* @get_scl:获取SCL行的当前值。强制通用SCL恢复。内部填充用于通用GPIO恢复。
* @set_scl:设置/清除SCL行。强制通用SCL恢复。内部填充用于通用GPIO恢复。
* @get_sda:获取SDA行的当前值。对于通用SCL恢复，这个或set_sda()是必需的。如果sda_gpio是有效的GPIO，则在内部填充，用于通用GPIO恢复。
* @set_sda:设置/清除SDA行。这个或get_sda()对于通用SCL恢复是必需的。如果sda_gpio是有效的GPIO，则在内部填充，用于通用GPIO恢复。
* @get_bus_free:返回从IP核看到的总线空闲状态，以防它具有比读取SDA更复杂的内部逻辑。可选的。
* @prepare_recovery:这将在开始恢复之前调用。平台可以在这里为SDA/SCL线或其他他们想要的东西配置padmux。
* @unprepare_recovery:这将在完成恢复后调用。平台可以在这里为SDA/SCL线或其他他们想要的东西配置padmux。
* @scl_gpiod: SCL线的gpiod。只需要GPIO恢复。
* @sda_gpiod: SDA线路的gpiod。只需要GPIO恢复。
*/	
struct i2c_bus_recovery_info {
	int (*recover_bus)(struct i2c_adapter *adap);

	int (*get_scl)(struct i2c_adapter *adap);
	void (*set_scl)(struct i2c_adapter *adap, int val);
	int (*get_sda)(struct i2c_adapter *adap);
	void (*set_sda)(struct i2c_adapter *adap, int val);
	int (*get_bus_free)(struct i2c_adapter *adap);

	void (*prepare_recovery)(struct i2c_adapter *adap);
	void (*unprepare_recovery)(struct i2c_adapter *adap);

	/* gpio recovery */
	struct gpio_desc *scl_gpiod;
	struct gpio_desc *sda_gpiod;
};

int i2c_recover_bus(struct i2c_adapter *adap);

/* Generic recovery routines */
int i2c_generic_scl_recovery(struct i2c_adapter *adap);

/**
 * struct i2c_adapter_quirks - describe flaws of an i2c adapter
 * @flags: see I2C_AQ_* for possible flags and read below
 * @max_num_msgs: maximum number of messages per transfer
 * @max_write_len: maximum length of a write message
 * @max_read_len: maximum length of a read message
 * @max_comb_1st_msg_len: maximum length of the first msg in a combined message
 * @max_comb_2nd_msg_len: maximum length of the second msg in a combined message
 *
 * Note about combined messages: Some I2C controllers can only send one message
 * per transfer, plus something called combined message or write-then-read.
 * This is (usually) a small write message followed by a read message and
 * barely enough to access register based devices like EEPROMs. There is a flag
 * to support this mode. It implies max_num_msg = 2 and does the length checks
 * with max_comb_*_len because combined message mode usually has its own
 * limitations. Because of HW implementations, some controllers can actually do
 * write-then-anything or other variants. To support that, write-then-read has
 * been broken out into smaller bits like write-first and read-second which can
 * be combined as needed.
 */
/**
* struct i2c_adapter_quirks -描述i2c适配器的缺陷
* @flags:查看I2C_AQ_*以获得可能的标志，并在下面阅读
* @max_num_msgs:每次传输的最大消息数
* @max_write_len:写消息的最大长度
* @max_read_len:读取消息的最大长度
* @max_comb_1st_msg_len:合并消息中第一个msg的最大长度
* @max_comb_2nd_msg_len:合并消息中第二个msg的最大长度
*
* 合并消息注意:一些I2C控制器每次传输只能发送一条消息，另外还有所谓的“合并消息”或“先写后读”功能。
* 这(通常)是一条小的写消息和一条读消息，几乎不足以访问基于寄存器的设备(如EEPROMs)。
* 有一个标志支持这种模式。它意味着max_num_msg = 2，并使用max_comb_*_len进行长度检查，因为组合消息模式通常有其自身的限制。
* 由于HW实现，一些控制器实际上可以执行write-then-anything或其他变体。
* 为了支持这一点，write-then-read被分解成更小的部分，比如write-first和read-second，可以根据需要组合。
*/
struct i2c_adapter_quirks {
	u64 flags;   // eg: I2C_AQ_COMB
	int max_num_msgs;
	u16 max_write_len;
	u16 max_read_len;
	u16 max_comb_1st_msg_len;
	u16 max_comb_2nd_msg_len;
};

/* enforce max_num_msgs = 2 and use max_comb_*_len for length checks */
#define I2C_AQ_COMB			BIT(0)          // 执行max_num_msgs = 2并使用max_comb_*_len进行长度检查
/* first combined message must be write */
#define I2C_AQ_COMB_WRITE_FIRST		BIT(1)  // 首先必须写入合并的消息
/* second combined message must be read */
#define I2C_AQ_COMB_READ_SECOND		BIT(2)  // 必须读取第二个组合消息
/* both combined messages must have the same target address */
#define I2C_AQ_COMB_SAME_ADDR		BIT(3)  // 两个组合消息必须具有相同的目标地址
/* convenience macro for typical write-then read case */
#define I2C_AQ_COMB_WRITE_THEN_READ	(I2C_AQ_COMB | I2C_AQ_COMB_WRITE_FIRST | \
					 I2C_AQ_COMB_READ_SECOND | I2C_AQ_COMB_SAME_ADDR)  // 方便宏为典型的写然后读的情况
/* clock stretching is not supported */
#define I2C_AQ_NO_CLK_STRETCH		BIT(4)   // 不支持时钟拉伸
/* message cannot have length of 0 */        // 消息的长度不能为0
#define I2C_AQ_NO_ZERO_LEN_READ		BIT(5)
#define I2C_AQ_NO_ZERO_LEN_WRITE	BIT(6)
#define I2C_AQ_NO_ZERO_LEN		(I2C_AQ_NO_ZERO_LEN_READ | I2C_AQ_NO_ZERO_LEN_WRITE)

/*
 * i2c_adapter is the structure used to identify a physical i2c bus along
 * with the access algorithms necessary to access it.
 */
/*
i2c_adapter是用来识别物理i2c总线的结构，以及访问它所需的访问算法。
*/
struct i2c_adapter {
	struct module *owner;
	unsigned int class;		  /* classes to allow probing for */  // 允许探测的类,eg: I2C_CLASS_DDC
	const struct i2c_algorithm *algo; /* the algorithm to access the bus */ // 访问总线的算法
	void *algo_data;

	/* data fields that are valid for all devices	*/
	// 对所有设备都有效的数据字段
	const struct i2c_lock_operations *lock_ops; // 表示I2C锁定操作
	struct rt_mutex bus_lock;
	struct rt_mutex mux_lock;

	int timeout;			/* in jiffies */
	int retries;            // 重试次数
	struct device dev;		/* the adapter device */  //适配器设备
	unsigned long locked_flags;	/* owned by the I2C core */ // 属于I2C核心, eg: I2C_ALF_IS_SUSPENDED
#define I2C_ALF_IS_SUSPENDED		0
#define I2C_ALF_SUSPEND_REPORTED	1

	int nr;   // 被分配的adapter bus号。特定适配器的适配器号
	char name[48];
	struct completion dev_released;

	struct mutex userspace_clients_lock;
	struct list_head userspace_clients;

	struct i2c_bus_recovery_info *bus_recovery_info;
	const struct i2c_adapter_quirks *quirks; // 描述i2c适配器的缺陷

	struct irq_domain *host_notify_domain;
};
// 根据struct device类型的 d,获得其外围结构体（ struct i2c_adapter）指针。
#define to_i2c_adapter(d) container_of(d, struct i2c_adapter, dev)

static inline void *i2c_get_adapdata(const struct i2c_adapter *adap)
{
	return dev_get_drvdata(&adap->dev);
}

static inline void i2c_set_adapdata(struct i2c_adapter *adap, void *data)
{
	dev_set_drvdata(&adap->dev, data);
}

// 获得适配器（ struct i2c_adapter ）设备的父设备，若也是一个适配器就返回其地址，否则返回NULL。
static inline struct i2c_adapter *
i2c_parent_is_i2c_adapter(const struct i2c_adapter *adapter)
{
#if IS_ENABLED(CONFIG_I2C_MUX)
	struct device *parent = adapter->dev.parent;

	if (parent != NULL && parent->type == &i2c_adapter_type)
		return to_i2c_adapter(parent);
	else
#endif
		return NULL;
}

int i2c_for_each_dev(void *data, int (*fn)(struct device *dev, void *data));

/* Adapter locking functions, exported for shared pin cases */
#define I2C_LOCK_ROOT_ADAPTER BIT(0)
#define I2C_LOCK_SEGMENT      BIT(1)

/**
 * i2c_lock_bus - Get exclusive access to an I2C bus segment
 * @adapter: Target I2C bus segment
 * @flags: I2C_LOCK_ROOT_ADAPTER locks the root i2c adapter, I2C_LOCK_SEGMENT
 *	locks only this branch in the adapter tree
 */
/**
* i2c_lock_bus -获得对I2C总线段的独占访问
* @adapter:目标I2C总线段
* @flags: I2C_LOCK_ROOT_ADAPTER锁定根i2c适配器I2C_LOCK_SEGMENT
*
* 只锁定适配器树中的这个分支
*/
static inline void
i2c_lock_bus(struct i2c_adapter *adapter, unsigned int flags)
{
	adapter->lock_ops->lock_bus(adapter, flags);
}

/**
 * i2c_trylock_bus - Try to get exclusive access to an I2C bus segment
 * @adapter: Target I2C bus segment
 * @flags: I2C_LOCK_ROOT_ADAPTER tries to locks the root i2c adapter,
 *	I2C_LOCK_SEGMENT tries to lock only this branch in the adapter tree
 *
 * Return: true if the I2C bus segment is locked, false otherwise
 */
static inline int
i2c_trylock_bus(struct i2c_adapter *adapter, unsigned int flags)
{
	return adapter->lock_ops->trylock_bus(adapter, flags);
}

/**
 * i2c_unlock_bus - Release exclusive access to an I2C bus segment
 * @adapter: Target I2C bus segment
 * @flags: I2C_LOCK_ROOT_ADAPTER unlocks the root i2c adapter, I2C_LOCK_SEGMENT
 *	unlocks only this branch in the adapter tree
 */
/**
* i2c_unlock_bus -释放对I2C总线段的独占访问
* @adapter:目标I2C总线段
* @flags: I2C_LOCK_ROOT_ADAPTER解锁根i2c适配器I2C_LOCK_SEGMENT
* 
* 只解锁适配器树中的这个分支
*/
static inline void
i2c_unlock_bus(struct i2c_adapter *adapter, unsigned int flags)
{
	adapter->lock_ops->unlock_bus(adapter, flags);
}

/**
 * i2c_mark_adapter_suspended - Report suspended state of the adapter to the core
 * @adap: Adapter to mark as suspended
 *
 * When using this helper to mark an adapter as suspended, the core will reject
 * further transfers to this adapter. The usage of this helper is optional but
 * recommended for devices having distinct handlers for system suspend and
 * runtime suspend. More complex devices are free to implement custom solutions
 * to reject transfers when suspended.
 */
static inline void i2c_mark_adapter_suspended(struct i2c_adapter *adap)
{
	i2c_lock_bus(adap, I2C_LOCK_ROOT_ADAPTER);
	set_bit(I2C_ALF_IS_SUSPENDED, &adap->locked_flags);
	i2c_unlock_bus(adap, I2C_LOCK_ROOT_ADAPTER);
}

/**
 * i2c_mark_adapter_resumed - Report resumed state of the adapter to the core
 * @adap: Adapter to mark as resumed
 *
 * When using this helper to mark an adapter as resumed, the core will allow
 * further transfers to this adapter. See also further notes to
 * @i2c_mark_adapter_suspended().
 */
static inline void i2c_mark_adapter_resumed(struct i2c_adapter *adap)
{
	i2c_lock_bus(adap, I2C_LOCK_ROOT_ADAPTER);
	clear_bit(I2C_ALF_IS_SUSPENDED, &adap->locked_flags);
	i2c_unlock_bus(adap, I2C_LOCK_ROOT_ADAPTER);
}

/* i2c adapter classes (bitmask) */
#define I2C_CLASS_HWMON		(1<<0)	/* lm_sensors, ... */  // 传感器
#define I2C_CLASS_DDC		(1<<3)	/* DDC bus on graphics adapters */ // 图形适配器上的DDC总线
#define I2C_CLASS_SPD		(1<<7)	/* Memory modules */  // 内存模块
/* Warn users that the adapter doesn't support classes anymore */
#define I2C_CLASS_DEPRECATED	(1<<8)  // 警告用户适配器不再支持类

/* Internal numbers to terminate lists */
#define I2C_CLIENT_END		0xfffeU    // 终止列表的内部号码

/* Construct an I2C_CLIENT_END-terminated array of i2c addresses */
#define I2C_ADDRS(addr, addrs...) \
	((const unsigned short []){ addr, ## addrs, I2C_CLIENT_END })


/* ----- functions exported by i2c.o */

/* administration...
 */
#if IS_ENABLED(CONFIG_I2C)
extern int i2c_add_adapter(struct i2c_adapter *adap);
extern void i2c_del_adapter(struct i2c_adapter *adap);
extern int i2c_add_numbered_adapter(struct i2c_adapter *adap);

extern int i2c_register_driver(struct module *owner, struct i2c_driver *driver);
extern void i2c_del_driver(struct i2c_driver *driver);

/* use a define to avoid include chaining to get THIS_MODULE */
// 使用定义来避免包含获取THIS_MODULE的链接
#define i2c_add_driver(driver) \
	i2c_register_driver(THIS_MODULE, driver)

extern struct i2c_client *i2c_use_client(struct i2c_client *client);
extern void i2c_release_client(struct i2c_client *client);

/* call the i2c_client->command() of all attached clients with
 * the given arguments */
extern void i2c_clients_command(struct i2c_adapter *adap,
				unsigned int cmd, void *arg);

extern struct i2c_adapter *i2c_get_adapter(int nr);
extern void i2c_put_adapter(struct i2c_adapter *adap);
extern unsigned int i2c_adapter_depth(struct i2c_adapter *adapter);

void i2c_parse_fw_timings(struct device *dev, struct i2c_timings *t, bool use_defaults);

/* Return the functionality mask */
// 返回功能掩码
static inline u32 i2c_get_functionality(struct i2c_adapter *adap)
{
	return adap->algo->functionality(adap);
}

/* Return 1 if adapter supports everything we need, 0 if not. */
// 如果适配器支持我们需要的所有内容，则返回1;如果不支持，则返回0。
static inline int i2c_check_functionality(struct i2c_adapter *adap, u32 func)
{
	return (func & i2c_get_functionality(adap)) == func;
}

/**
 * i2c_check_quirks() - Function for checking the quirk flags in an i2c adapter
 * @adap: i2c adapter
 * @quirks: quirk flags
 *
 * Return: true if the adapter has all the specified quirk flags, false if not
 */
static inline bool i2c_check_quirks(struct i2c_adapter *adap, u64 quirks)
{
	if (!adap->quirks)
		return false;
	return (adap->quirks->flags & quirks) == quirks;
}

/* Return the adapter number for a specific adapter */
// 返回特定适配器的适配器号
static inline int i2c_adapter_id(struct i2c_adapter *adap)
{
	return adap->nr;
}

static inline u8 i2c_8bit_addr_from_msg(const struct i2c_msg *msg)
{
	return (msg->addr << 1) | (msg->flags & I2C_M_RD ? 1 : 0);
}

u8 *i2c_get_dma_safe_msg_buf(struct i2c_msg *msg, unsigned int threshold);
void i2c_put_dma_safe_msg_buf(u8 *buf, struct i2c_msg *msg, bool xferred);

int i2c_handle_smbus_host_notify(struct i2c_adapter *adap, unsigned short addr);
/**
 * module_i2c_driver() - Helper macro for registering a modular I2C driver
 * @__i2c_driver: i2c_driver struct
 *
 * Helper macro for I2C drivers which do not do anything special in module
 * init/exit. This eliminates a lot of boilerplate. Each module may only
 * use this macro once, and calling it replaces module_init() and module_exit()
 */
#define module_i2c_driver(__i2c_driver) \
	module_driver(__i2c_driver, i2c_add_driver, \
			i2c_del_driver)

/**
 * builtin_i2c_driver() - Helper macro for registering a builtin I2C driver
 * @__i2c_driver: i2c_driver struct
 *
 * Helper macro for I2C drivers which do not do anything special in their
 * init. This eliminates a lot of boilerplate. Each driver may only
 * use this macro once, and calling it replaces device_initcall().
 */
#define builtin_i2c_driver(__i2c_driver) \
	builtin_driver(__i2c_driver, i2c_add_driver)

#endif /* I2C */

#if IS_ENABLED(CONFIG_OF)
/* must call put_device() when done with returned i2c_client device */
extern struct i2c_client *of_find_i2c_device_by_node(struct device_node *node);

/* must call put_device() when done with returned i2c_adapter device */
extern struct i2c_adapter *of_find_i2c_adapter_by_node(struct device_node *node);

/* must call i2c_put_adapter() when done with returned i2c_adapter device */
struct i2c_adapter *of_get_i2c_adapter_by_node(struct device_node *node);

extern const struct of_device_id
*i2c_of_match_device(const struct of_device_id *matches,
		     struct i2c_client *client);

int of_i2c_get_board_info(struct device *dev, struct device_node *node,
			  struct i2c_board_info *info);

#else

static inline struct i2c_client *of_find_i2c_device_by_node(struct device_node *node)
{
	return NULL;
}

static inline struct i2c_adapter *of_find_i2c_adapter_by_node(struct device_node *node)
{
	return NULL;
}

static inline struct i2c_adapter *of_get_i2c_adapter_by_node(struct device_node *node)
{
	return NULL;
}

static inline const struct of_device_id
*i2c_of_match_device(const struct of_device_id *matches,
		     struct i2c_client *client)
{
	return NULL;
}

static inline int of_i2c_get_board_info(struct device *dev,
					struct device_node *node,
					struct i2c_board_info *info)
{
	return -ENOTSUPP;
}

#endif /* CONFIG_OF */

struct acpi_resource;
struct acpi_resource_i2c_serialbus;

#if IS_ENABLED(CONFIG_ACPI)
bool i2c_acpi_get_i2c_resource(struct acpi_resource *ares,
			       struct acpi_resource_i2c_serialbus **i2c);
u32 i2c_acpi_find_bus_speed(struct device *dev);
struct i2c_client *i2c_acpi_new_device(struct device *dev, int index,
				       struct i2c_board_info *info);
struct i2c_adapter *i2c_acpi_find_adapter_by_handle(acpi_handle handle);
#else
static inline bool i2c_acpi_get_i2c_resource(struct acpi_resource *ares,
					     struct acpi_resource_i2c_serialbus **i2c)
{
	return false;
}
static inline u32 i2c_acpi_find_bus_speed(struct device *dev)
{
	return 0;
}
static inline struct i2c_client *i2c_acpi_new_device(struct device *dev,
					int index, struct i2c_board_info *info)
{
	return NULL;
}
static inline struct i2c_adapter *i2c_acpi_find_adapter_by_handle(acpi_handle handle)
{
	return NULL;
}
#endif /* CONFIG_ACPI */

#endif /* _LINUX_I2C_H */
