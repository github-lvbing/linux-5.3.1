/*
 * HID over I2C protocol implementation
 *
 * Copyright (c) 2012 Benjamin Tissoires <benjamin.tissoires@gmail.com>
 * Copyright (c) 2012 Ecole Nationale de l'Aviation Civile, France
 * Copyright (c) 2012 Red Hat, Inc
 *
 * This code is partly based on "USB HID support for Linux":
 *
 *  Copyright (c) 1999 Andreas Gal
 *  Copyright (c) 2000-2005 Vojtech Pavlik <vojtech@suse.cz>
 *  Copyright (c) 2005 Michael Haboustak <mike-@cinci.rr.com> for Concept2, Inc
 *  Copyright (c) 2007-2008 Oliver Neukum
 *  Copyright (c) 2006-2010 Jiri Kosina
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/hid.h>
#include <linux/mutex.h>
#include <linux/acpi.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <linux/platform_data/i2c-hid.h>

#include "../hid-ids.h"
#include "i2c-hid.h"

/* quirks to control the device */
// 控制设备的 quirks
#define I2C_HID_QUIRK_SET_PWR_WAKEUP_DEV	BIT(0)
#define I2C_HID_QUIRK_NO_IRQ_AFTER_RESET	BIT(1)
#define I2C_HID_QUIRK_NO_RUNTIME_PM		    BIT(2)
#define I2C_HID_QUIRK_DELAY_AFTER_SLEEP		BIT(3)
#define I2C_HID_QUIRK_BOGUS_IRQ			    BIT(4)

/* flags */
#define I2C_HID_STARTED		0
#define I2C_HID_RESET_PENDING	1
#define I2C_HID_READ_PENDING	2

#define I2C_HID_PWR_ON		0x00
#define I2C_HID_PWR_SLEEP	0x01

/* debug option */
static bool debug;
module_param(debug, bool, 0444);
MODULE_PARM_DESC(debug, "print a lot of debug information");

#define i2c_hid_dbg(ihid, fmt, arg...)					  \
do {									  \
	if (debug)							  \
		dev_printk(KERN_DEBUG, &(ihid)->client->dev, fmt, ##arg); \
} while (0)

struct i2c_hid_desc { //下表显示了所需的 I2C HID 描述符。
	__le16 wHIDDescLength;     // 	2 	完整的 HID 说明符 （以字节为单位） 的长度。(也就是本结构 30byte)
	__le16 bcdVersion;         // 	2 	版本号，是二进制编码的十进制 (BCD) 格式。
	__le16 wReportDescLength;  // 	2 	（以字节为单位） 的报告描述符的长度。
	__le16 wReportDescRegister;// 	2 	包含报表描述符的寄存器索引。
	__le16 wInputRegister;     // 	2 	要读取输入的报表 （位于无符号字节为单位） 的注册号。
	__le16 wMaxInputLength;    //  2 	要从输入寄存器中读取的最大输入报表的长度。
	__le16 wOutputRegister;    //  2 	要将输出发送 （以无符号字节为单位） 的注册号。
	__le16 wMaxOutputLength;   //  2 	要发送的最大输出报表的长度。
	__le16 wCommandRegister;   //  2 	要发送命令请求 （无符号字节为单位） 的注册号。
	__le16 wDataRegister;      //  2 	寄存器号与命令请求数 （以无符号字节为单位） 交换数据。
	__le16 wVendorID;          //  2 	USB-如果分配供应商 id。
	__le16 wProductID;         //  2 	设备 ID。
	__le16 wVersionID;         //	2 	固件版本的版本号。
	__le32 reserved;           //   4
} __packed;

struct i2c_hid_cmd {
	unsigned int registerIndex;  // struct i2c_hid_desc中的一个索引值
	__u8 opcode;
	unsigned int length;    // registerIndex 开始到结束的长度
	bool wait;
};

// HID i2c 要发送cmd的数据序列
union command {
	u8 data[0];
	struct cmd {
		__le16 reg;         // 寄存器的位置。
		__u8 reportTypeID;  // reportID | reportType << 4;
		__u8 opcode;        // 操作码
	} __packed c;
};

#define I2C_HID_CMD(opcode_) \
	.opcode = opcode_,  \
	.length = 4, \
	.registerIndex = offsetof(struct i2c_hid_desc, wCommandRegister)

/* fetch HID descriptor */
// 获取 HID 描述符
static const struct i2c_hid_cmd hid_descr_cmd = { .length = 2 };
/* fetch report descriptors */
// 获得 report 描述符
static const struct i2c_hid_cmd hid_report_descr_cmd = {
		.registerIndex = offsetof(struct i2c_hid_desc,
			wReportDescRegister),
		.opcode = 0x00,
		.length = 2 };
/* commands */
static const struct i2c_hid_cmd hid_reset_cmd =	{ I2C_HID_CMD(0x01), .wait = true };
static const struct i2c_hid_cmd hid_get_report_cmd =	{ I2C_HID_CMD(0x02) };
static const struct i2c_hid_cmd hid_set_report_cmd =	{ I2C_HID_CMD(0x03) };
static const struct i2c_hid_cmd hid_set_power_cmd =	{ I2C_HID_CMD(0x08) };
static const struct i2c_hid_cmd hid_no_cmd =		{ .length = 0 };

/*
 * These definitions are not used here, but are defined by the spec.
 * Keeping them here for documentation purposes.
 *
 * static const struct i2c_hid_cmd hid_get_idle_cmd = { I2C_HID_CMD(0x04) };
 * static const struct i2c_hid_cmd hid_set_idle_cmd = { I2C_HID_CMD(0x05) };
 * static const struct i2c_hid_cmd hid_get_protocol_cmd = { I2C_HID_CMD(0x06) };
 * static const struct i2c_hid_cmd hid_set_protocol_cmd = { I2C_HID_CMD(0x07) };
 */

/* The main device structure  主要设备结构 */
struct i2c_hid {
	struct i2c_client	*client;	/* i2c client */  // 记录i2c客户
	struct hid_device	*hid;	/* pointer to corresponding HID dev */ // 指向相应的HID dev的指针
	union {
		__u8 hdesc_buffer[sizeof(struct i2c_hid_desc)];
		struct i2c_hid_desc hdesc;	/* the HID Descriptor */ // HID 描述符
	};
	__le16	 wHIDDescRegister; /* location of the i2c
						   * register of the HID
						   * descriptor. */  // HID 描述符的i2c寄存器的位置。
						   // cpu_to_le16(ihid->pdata.hid_descriptor_address)
	unsigned int		bufsize;	/* i2c buffer size */ // i2c缓冲区大小
	u8			*inbuf;		/* Input buffer */     // 输入缓冲区
	u8			*rawbuf;	/* Raw Input buffer */ // 原始输入缓冲区
	u8			*cmdbuf;	/* Command buffer */   // 命令缓冲区
	u8			*argsbuf;	/* Command arguments buffer */ // 命令参数的缓冲区

	unsigned long		flags;		/* device flags */   // 设备标志.eg: I2C_HID_READ_PENDING.(__i2c_hid_command)
	unsigned long		quirks;		/* Various quirks */ // 各种怪癖.eg: I2C_HID_QUIRK_SET_PWR_WAKEUP_DEV.  i2c_hid_quirks

	wait_queue_head_t	wait;		/* For waiting the interrupt */ // 等待中断的队列

	struct i2c_hid_platform_data pdata;  // 设备平台数据

	bool			irq_wake_enabled;
	struct mutex		reset_lock;

	unsigned long		sleep_delay;
};

static const struct i2c_hid_quirks {
	__u16 idVendor;
	__u16 idProduct;
	__u32 quirks;
} i2c_hid_quirks[] = {
	{ USB_VENDOR_ID_WEIDA, USB_DEVICE_ID_WEIDA_8752,
		I2C_HID_QUIRK_SET_PWR_WAKEUP_DEV },
	{ USB_VENDOR_ID_WEIDA, USB_DEVICE_ID_WEIDA_8755,
		I2C_HID_QUIRK_SET_PWR_WAKEUP_DEV },
	{ I2C_VENDOR_ID_HANTICK, I2C_PRODUCT_ID_HANTICK_5288,
		I2C_HID_QUIRK_NO_IRQ_AFTER_RESET |
		I2C_HID_QUIRK_NO_RUNTIME_PM },
	{ I2C_VENDOR_ID_RAYDIUM, I2C_PRODUCT_ID_RAYDIUM_4B33,
		I2C_HID_QUIRK_DELAY_AFTER_SLEEP },
	{ USB_VENDOR_ID_LG, I2C_DEVICE_ID_LG_8001,
		I2C_HID_QUIRK_NO_RUNTIME_PM },
	{ I2C_VENDOR_ID_GOODIX, I2C_DEVICE_ID_GOODIX_01F0,
		I2C_HID_QUIRK_NO_RUNTIME_PM },
	{ USB_VENDOR_ID_ELAN, HID_ANY_ID,
		 I2C_HID_QUIRK_BOGUS_IRQ },
	{ 0, 0 }
};

/*
 * i2c_hid_lookup_quirk: return any quirks associated with a I2C HID device
 * @idVendor: the 16-bit vendor ID
 * @idProduct: the 16-bit product ID
 *
 * Returns: a u32 quirks value.
 */
/*
* i2c_hid_lookup_quirk:返回与I2C HID设备相关的任何异常
* @idVendor: 16位供应商ID
* @idProduct: 16位产品ID
*
*返回:一个u32怪癖值。
*/
static u32 i2c_hid_lookup_quirk(const u16 idVendor, const u16 idProduct)
{
	u32 quirks = 0;
	int n;

	for (n = 0; i2c_hid_quirks[n].idVendor; n++)
		if (i2c_hid_quirks[n].idVendor == idVendor &&
		    (i2c_hid_quirks[n].idProduct == (__u16)HID_ANY_ID ||
		     i2c_hid_quirks[n].idProduct == idProduct))
			quirks = i2c_hid_quirks[n].quirks;

	return quirks;
}

/*  发送 HID 命令，并获得响应数据。
struct i2c_client *client,
const struct i2c_hid_cmd *command,  // 要发送的 HID 命令头
u8 reportID,           // ID
u8 reportType,         // Type
u8 *args,              // 要发送的 HID 命令头后的数据
int args_len,          // 要发送的 HID 命令头后的数据的长度
unsigned char *buf_recv,   // 要接受数据缓存区
int data_len               // 要接收多长的数据
*/
static int __i2c_hid_command(struct i2c_client *client,
		const struct i2c_hid_cmd *command, u8 reportID,
		u8 reportType, u8 *args, int args_len,
		unsigned char *buf_recv, int data_len)
{
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	union command *cmd = (union command *)ihid->cmdbuf;
	int ret;
	struct i2c_msg msg[2];
	int msg_num = 1;

	int length = command->length;
	bool wait = command->wait;
	unsigned int registerIndex = command->registerIndex;

	/* special case for hid_descr_cmd */
	// 若cmd是 获取 HID 描述符 的特殊情况
	if (command == &hid_descr_cmd) {
		cmd->c.reg = ihid->wHIDDescRegister;
	} else {
		cmd->data[0] = ihid->hdesc_buffer[registerIndex];
		cmd->data[1] = ihid->hdesc_buffer[registerIndex + 1];
	}

	if (length > 2) {
		cmd->c.opcode = command->opcode;
		cmd->c.reportTypeID = reportID | reportType << 4;
	}

	// cmd->data  为将要发送的数据暂存区。length 为数据暂存区中数据的长度。
	memcpy(cmd->data + length, args, args_len);
	length += args_len;

	i2c_hid_dbg(ihid, "%s: cmd=%*ph\n", __func__, length, cmd->data);

	// 构建 i2c 要传输的 信息 struct i2c_msg。
	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN; // 告知地址是否是10bit访问
	msg[0].len = length;   // 要写的长度（默认操作是写）
	msg[0].buf = cmd->data;// 要写的数据地址
	if (data_len > 0) {
		msg[1].addr = client->addr;
		msg[1].flags = client->flags & I2C_M_TEN;
		msg[1].flags |= I2C_M_RD; // 操作是读
		msg[1].len = data_len;   // 要读的长度
		msg[1].buf = buf_recv;   // 要读的数据缓存地址
		msg_num = 2;// 告知有几个传输的信息结构
		set_bit(I2C_HID_READ_PENDING, &ihid->flags);
	}

	if (wait)
		set_bit(I2C_HID_RESET_PENDING, &ihid->flags);

	// 开始访问i2c设备
	ret = i2c_transfer(client->adapter, msg, msg_num);

	if (data_len > 0)
		clear_bit(I2C_HID_READ_PENDING, &ihid->flags);

	if (ret != msg_num)
		return ret < 0 ? ret : -EIO;

	ret = 0;

	if (wait && (ihid->quirks & I2C_HID_QUIRK_NO_IRQ_AFTER_RESET)) {
		msleep(100);
	} else if (wait) {
		i2c_hid_dbg(ihid, "%s: waiting...\n", __func__);
		// 主动休眠，直到条件变为真或超时结束
		if (!wait_event_timeout(ihid->wait,
				!test_bit(I2C_HID_RESET_PENDING, &ihid->flags),
				msecs_to_jiffies(5000)))
			ret = -ENODATA;
		i2c_hid_dbg(ihid, "%s: finished.\n", __func__);
	}

	return ret;
}

static int i2c_hid_command(struct i2c_client *client,
		const struct i2c_hid_cmd *command,
		unsigned char *buf_recv, int data_len)
{
	return __i2c_hid_command(client, command, 0, 0, NULL, 0,
				buf_recv, data_len);
}

static int i2c_hid_get_report(struct i2c_client *client, u8 reportType,
		u8 reportID, unsigned char *buf_recv, int data_len)
{
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	u8 args[3];
	int ret;
	int args_len = 0;
	u16 readRegister = le16_to_cpu(ihid->hdesc.wDataRegister);

	i2c_hid_dbg(ihid, "%s\n", __func__);

	if (reportID >= 0x0F) {
		args[args_len++] = reportID;
		reportID = 0x0F;
	}

	args[args_len++] = readRegister & 0xFF;
	args[args_len++] = readRegister >> 8;

	ret = __i2c_hid_command(client, &hid_get_report_cmd, reportID,
		reportType, args, args_len, buf_recv, data_len);
	if (ret) {
		dev_err(&client->dev,
			"failed to retrieve report from device.\n");
		return ret;
	}

	return 0;
}

/**
 * i2c_hid_set_or_send_report: forward an incoming report to the device
 * @client: the i2c_client of the device
 * @reportType: 0x03 for HID_FEATURE_REPORT ; 0x02 for HID_OUTPUT_REPORT
 * @reportID: the report ID
 * @buf: the actual data to transfer, without the report ID
 * @len: size of buf
 * @use_data: true: use SET_REPORT HID command, false: send plain OUTPUT report
 */
// 将传入的报告转发给设备
static int i2c_hid_set_or_send_report(struct i2c_client *client, u8 reportType,
		u8 reportID, unsigned char *buf, size_t data_len, bool use_data)
{
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	u8 *args = ihid->argsbuf;
	const struct i2c_hid_cmd *hidcmd;
	int ret;
	u16 dataRegister = le16_to_cpu(ihid->hdesc.wDataRegister);
	u16 outputRegister = le16_to_cpu(ihid->hdesc.wOutputRegister);
	u16 maxOutputLength = le16_to_cpu(ihid->hdesc.wMaxOutputLength);
	u16 size;
	int args_len;
	int index = 0;

	i2c_hid_dbg(ihid, "%s\n", __func__);

	if (data_len > ihid->bufsize)
		return -EINVAL;

	size =		2			/* size */ +
			(reportID ? 1 : 0)	/* reportID */ +
			data_len		/* buf */;
	args_len =	(reportID >= 0x0F ? 1 : 0) /* optional third byte */ +
			2			/* dataRegister */ +
			size			/* args */;

	if (!use_data && maxOutputLength == 0)
		return -ENOSYS;

	if (reportID >= 0x0F) {
		args[index++] = reportID;
		reportID = 0x0F;
	}

	/*
	 * use the data register for feature reports or if the device does not
	 * support the output register
	 */
	// 如果设备不支持输出寄存器，则使用数据寄存器生成功能报告
	if (use_data) {
		args[index++] = dataRegister & 0xFF;
		args[index++] = dataRegister >> 8;
		hidcmd = &hid_set_report_cmd;
	} else {
		args[index++] = outputRegister & 0xFF;
		args[index++] = outputRegister >> 8;
		hidcmd = &hid_no_cmd;
	}

	args[index++] = size & 0xFF;
	args[index++] = size >> 8;

	if (reportID)
		args[index++] = reportID;

	memcpy(&args[index], buf, data_len);

	ret = __i2c_hid_command(client, hidcmd, reportID,
		reportType, args, args_len, NULL, 0);
	if (ret) {
		dev_err(&client->dev, "failed to set a report to device.\n");
		return ret;
	}

	return data_len;
}

static int i2c_hid_set_power(struct i2c_client *client, int power_state)
{
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	int ret;
	unsigned long now, delay;

	i2c_hid_dbg(ihid, "%s\n", __func__);

	/*
	 * Some devices require to send a command to wakeup before power on.
	 * The call will get a return value (EREMOTEIO) but device will be
	 * triggered and activated. After that, it goes like a normal device.
	 */
	if (power_state == I2C_HID_PWR_ON &&
	    ihid->quirks & I2C_HID_QUIRK_SET_PWR_WAKEUP_DEV) {
		ret = i2c_hid_command(client, &hid_set_power_cmd, NULL, 0);

		/* Device was already activated */
		if (!ret)
			goto set_pwr_exit;
	}

	if (ihid->quirks & I2C_HID_QUIRK_DELAY_AFTER_SLEEP &&
	    power_state == I2C_HID_PWR_ON) {
		now = jiffies;
		if (time_after(ihid->sleep_delay, now)) {
			delay = jiffies_to_usecs(ihid->sleep_delay - now);
			usleep_range(delay, delay + 1);
		}
	}

	ret = __i2c_hid_command(client, &hid_set_power_cmd, power_state,
		0, NULL, 0, NULL, 0);

	if (ihid->quirks & I2C_HID_QUIRK_DELAY_AFTER_SLEEP &&
	    power_state == I2C_HID_PWR_SLEEP)
		ihid->sleep_delay = jiffies + msecs_to_jiffies(20);

	if (ret)
		dev_err(&client->dev, "failed to change power setting.\n");

set_pwr_exit:
	return ret;
}

// 给 HID 设备发送 硬件复位 CMD
static int i2c_hid_hwreset(struct i2c_client *client)
{
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	int ret;

	i2c_hid_dbg(ihid, "%s\n", __func__);

	/*
	 * This prevents sending feature reports while the device is
	 * being reset. Otherwise we may lose the reset complete
	 * interrupt.
	 */
	mutex_lock(&ihid->reset_lock);

	ret = i2c_hid_set_power(client, I2C_HID_PWR_ON);
	if (ret)
		goto out_unlock;

	/*
	 * The HID over I2C specification states that if a DEVICE needs time
	 * after the PWR_ON request, it should utilise CLOCK stretching.
	 * However, it has been observered that the Windows driver provides a
	 * 1ms sleep between the PWR_ON and RESET requests and that some devices
	 * rely on this.
	 */
	usleep_range(1000, 5000);

	i2c_hid_dbg(ihid, "resetting...\n");

	ret = i2c_hid_command(client, &hid_reset_cmd, NULL, 0);
	if (ret) {
		dev_err(&client->dev, "failed to reset device.\n");
		i2c_hid_set_power(client, I2C_HID_PWR_SLEEP);
	}

out_unlock:
	mutex_unlock(&ihid->reset_lock);
	return ret;
}

static void i2c_hid_get_input(struct i2c_hid *ihid)
{
	int ret;
	u32 ret_size;
	int size = le16_to_cpu(ihid->hdesc.wMaxInputLength);

	if (size > ihid->bufsize)
		size = ihid->bufsize;

	ret = i2c_master_recv(ihid->client, ihid->inbuf, size);
	if (ret != size) {
		if (ret < 0)
			return;

		dev_err(&ihid->client->dev, "%s: got %d data instead of %d\n",
			__func__, ret, size);
		return;
	}

	ret_size = ihid->inbuf[0] | ihid->inbuf[1] << 8;

	if (!ret_size) {
		/* host or device initiated RESET completed */
		if (test_and_clear_bit(I2C_HID_RESET_PENDING, &ihid->flags))
			wake_up(&ihid->wait);
		return;
	}

	if (ihid->quirks & I2C_HID_QUIRK_BOGUS_IRQ && ret_size == 0xffff) {
		dev_warn_once(&ihid->client->dev, "%s: IRQ triggered but "
			      "there's no data\n", __func__);
		return;
	}

	if ((ret_size > size) || (ret_size < 2)) {
		dev_err(&ihid->client->dev, "%s: incomplete report (%d/%d)\n",
			__func__, size, ret_size);
		return;
	}

	i2c_hid_dbg(ihid, "input: %*ph\n", ret_size, ihid->inbuf);

	if (test_bit(I2C_HID_STARTED, &ihid->flags))
		hid_input_report(ihid->hid, HID_INPUT_REPORT, ihid->inbuf + 2,
				ret_size - 2, 1);

	return;
}

// HID 中断处理函数
static irqreturn_t i2c_hid_irq(int irq, void *dev_id)
{
	struct i2c_hid *ihid = dev_id;

	if (test_bit(I2C_HID_READ_PENDING, &ihid->flags))
		return IRQ_HANDLED;

	i2c_hid_get_input(ihid);

	return IRQ_HANDLED;
}

static int i2c_hid_get_report_length(struct hid_report *report)
{
	return ((report->size - 1) >> 3) + 1 +
		report->device->report_enum[report->type].numbered + 2;
}

/*
 * Traverse the supplied list of reports and find the longest
 */
// 遍历提供的报告列表，找到最长的
static void i2c_hid_find_max_report(struct hid_device *hid, unsigned int type,
		unsigned int *max)
{
	struct hid_report *report;
	unsigned int size;

	/* We should not rely on wMaxInputLength, as some devices may set it to
	 * a wrong length. */
	// 我们不应该依赖于wMaxInputLength，因为一些设备可能会将其设置为错误的长度
	list_for_each_entry(report, &hid->report_enum[type].report_list, list) {
		size = i2c_hid_get_report_length(report);
		if (*max < size)
			*max = size;
	}
}

static void i2c_hid_free_buffers(struct i2c_hid *ihid)
{
	kfree(ihid->inbuf);
	kfree(ihid->rawbuf);
	kfree(ihid->argsbuf);
	kfree(ihid->cmdbuf);
	ihid->inbuf = NULL;
	ihid->rawbuf = NULL;
	ihid->cmdbuf = NULL;
	ihid->argsbuf = NULL;
	ihid->bufsize = 0;
}
// 为 struct i2c_hid的inbuf、rawbuf、argsbuf、cmdbuf 分配空间。
static int i2c_hid_alloc_buffers(struct i2c_hid *ihid, size_t report_size)
{
	/* the worst case is computed from the set_report command with a
	 * reportID > 15 and the maximum report length */
	// 最坏的情况是使用reporttid > 15和最大报告长度计算set_report命令
	int args_len = sizeof(__u8) + /* ReportID  报告ID*/
		       sizeof(__u8) +    /* optional ReportID byte 可选ReportID字节*/
		       sizeof(__u16) +   /* data register 数据寄存器 */
		       sizeof(__u16) +   /* size of the report 报告的大小 */
		       report_size;      /* report 报告 */

	ihid->inbuf = kzalloc(report_size, GFP_KERNEL);
	ihid->rawbuf = kzalloc(report_size, GFP_KERNEL);
	ihid->argsbuf = kzalloc(args_len, GFP_KERNEL);
	ihid->cmdbuf = kzalloc(sizeof(union command) + args_len, GFP_KERNEL);

	if (!ihid->inbuf || !ihid->rawbuf || !ihid->argsbuf || !ihid->cmdbuf) {
		i2c_hid_free_buffers(ihid);
		return -ENOMEM;
	}

	ihid->bufsize = report_size;

	return 0;
}

static int i2c_hid_get_raw_report(struct hid_device *hid,
		unsigned char report_number, __u8 *buf, size_t count,
		unsigned char report_type)
{
	struct i2c_client *client = hid->driver_data;
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	size_t ret_count, ask_count;
	int ret;

	if (report_type == HID_OUTPUT_REPORT)
		return -EINVAL;

	/* +2 bytes to include the size of the reply in the query buffer */
	ask_count = min(count + 2, (size_t)ihid->bufsize);

	ret = i2c_hid_get_report(client,
			report_type == HID_FEATURE_REPORT ? 0x03 : 0x01,
			report_number, ihid->rawbuf, ask_count);

	if (ret < 0)
		return ret;

	ret_count = ihid->rawbuf[0] | (ihid->rawbuf[1] << 8);

	if (ret_count <= 2)
		return 0;

	ret_count = min(ret_count, ask_count);

	/* The query buffer contains the size, dropping it in the reply */
	count = min(count, ret_count - 2);
	memcpy(buf, ihid->rawbuf + 2, count);

	return count;
}

static int i2c_hid_output_raw_report(struct hid_device *hid, __u8 *buf,
		size_t count, unsigned char report_type, bool use_data)
{
	struct i2c_client *client = hid->driver_data;
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	int report_id = buf[0];
	int ret;

	if (report_type == HID_INPUT_REPORT)
		return -EINVAL;

	mutex_lock(&ihid->reset_lock);

	if (report_id) {
		buf++;
		count--;
	}

	// 将传入的报告转发给设备
	ret = i2c_hid_set_or_send_report(client,
				report_type == HID_FEATURE_REPORT ? 0x03 : 0x02,
				report_id, buf, count, use_data);

	if (report_id && ret >= 0)
		ret++; /* add report_id to the number of transfered bytes */

	mutex_unlock(&ihid->reset_lock);

	return ret;
}

static int i2c_hid_output_report(struct hid_device *hid, __u8 *buf,
		size_t count)
{
	return i2c_hid_output_raw_report(hid, buf, count, HID_OUTPUT_REPORT,
			false);
}

static int i2c_hid_raw_request(struct hid_device *hid, unsigned char reportnum,
			       __u8 *buf, size_t len, unsigned char rtype,
			       int reqtype)
{
	switch (reqtype) {
	case HID_REQ_GET_REPORT:
		return i2c_hid_get_raw_report(hid, reportnum, buf, len, rtype);
	case HID_REQ_SET_REPORT:
		if (buf[0] != reportnum)
			return -EINVAL;
		return i2c_hid_output_raw_report(hid, buf, len, rtype, true);
	default:
		return -EIO;
	}
}

static int i2c_hid_parse(struct hid_device *hid)
{
	struct i2c_client *client = hid->driver_data;
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	struct i2c_hid_desc *hdesc = &ihid->hdesc;
	unsigned int rsize;
	char *rdesc;
	int ret;
	int tries = 3;
	char *use_override;

	i2c_hid_dbg(ihid, "entering %s\n", __func__);

	// 获得 报告描述符 的长度
	rsize = le16_to_cpu(hdesc->wReportDescLength);
	if (!rsize || rsize > HID_MAX_DESCRIPTOR_SIZE) {
		dbg_hid("weird size of report descriptor (%u)\n", rsize);
		return -EINVAL;
	}

	do {
		// 给 HID 设备发送 硬件复位 CMD
		ret = i2c_hid_hwreset(client);
		if (ret)
			msleep(1000);
	} while (tries-- > 0 && ret);

	if (ret)
		return ret;

	use_override = i2c_hid_get_dmi_hid_report_desc_override(client->name,
								&rsize);

	if (use_override) {
		rdesc = use_override;
		i2c_hid_dbg(ihid, "Using a HID report descriptor override\n");
	} else {
		rdesc = kzalloc(rsize, GFP_KERNEL);

		if (!rdesc) {
			dbg_hid("couldn't allocate rdesc memory\n");
			return -ENOMEM;
		}

		i2c_hid_dbg(ihid, "asking HID report descriptor\n");

		// 获得 report 描述符 到 rdesc
		ret = i2c_hid_command(client, &hid_report_descr_cmd,
				      rdesc, rsize);
		if (ret) {
			hid_err(hid, "reading report descriptor failed\n");
			kfree(rdesc);
			return -EIO;
		}
	}

	i2c_hid_dbg(ihid, "Report Descriptor: %*ph\n", rsize, rdesc);

	// 解析 report 描述符 的数据到 struct hid_device->dev_rdesc;
	ret = hid_parse_report(hid, rdesc, rsize);
	if (!use_override)
		kfree(rdesc);

	if (ret) {
		dbg_hid("parsing report descriptor failed\n");
		return ret;
	}

	return 0;
}

static int i2c_hid_start(struct hid_device *hid)
{
	struct i2c_client *client = hid->driver_data;
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	int ret;
	unsigned int bufsize = HID_MIN_BUFFER_SIZE;

	i2c_hid_find_max_report(hid, HID_INPUT_REPORT, &bufsize);
	i2c_hid_find_max_report(hid, HID_OUTPUT_REPORT, &bufsize);
	i2c_hid_find_max_report(hid, HID_FEATURE_REPORT, &bufsize);

	if (bufsize > ihid->bufsize) {
		disable_irq(client->irq);
		i2c_hid_free_buffers(ihid);

		ret = i2c_hid_alloc_buffers(ihid, bufsize);
		enable_irq(client->irq);

		if (ret)
			return ret;
	}

	return 0;
}

static void i2c_hid_stop(struct hid_device *hid)
{
	hid->claimed = 0;
}

static int i2c_hid_open(struct hid_device *hid)
{
	struct i2c_client *client = hid->driver_data;
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	int ret = 0;

	ret = pm_runtime_get_sync(&client->dev);
	if (ret < 0)
		return ret;

	set_bit(I2C_HID_STARTED, &ihid->flags);
	return 0;
}

static void i2c_hid_close(struct hid_device *hid)
{
	struct i2c_client *client = hid->driver_data;
	struct i2c_hid *ihid = i2c_get_clientdata(client);

	clear_bit(I2C_HID_STARTED, &ihid->flags);

	/* Save some power */
	pm_runtime_put(&client->dev);
}

static int i2c_hid_power(struct hid_device *hid, int lvl)
{
	struct i2c_client *client = hid->driver_data;
	struct i2c_hid *ihid = i2c_get_clientdata(client);

	i2c_hid_dbg(ihid, "%s lvl:%d\n", __func__, lvl);

	switch (lvl) {
	case PM_HINT_FULLON:
		pm_runtime_get_sync(&client->dev);
		break;
	case PM_HINT_NORMAL:
		pm_runtime_put(&client->dev);
		break;
	}
	return 0;
}

struct hid_ll_driver i2c_hid_ll_driver = {
	.parse = i2c_hid_parse,
	.start = i2c_hid_start,
	.stop = i2c_hid_stop,
	.open = i2c_hid_open,
	.close = i2c_hid_close,
	.power = i2c_hid_power,
	.output_report = i2c_hid_output_report,
	.raw_request = i2c_hid_raw_request,
};
EXPORT_SYMBOL_GPL(i2c_hid_ll_driver);

// 注册中断处理函数 i2c_hid_irq
static int i2c_hid_init_irq(struct i2c_client *client)
{
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	unsigned long irqflags = 0;
	int ret;

	dev_dbg(&client->dev, "Requesting IRQ: %d\n", client->irq);

	if (!irq_get_trigger_type(client->irq))
		irqflags = IRQF_TRIGGER_LOW;
	// 分配中断线
	ret = request_threaded_irq(client->irq, NULL, i2c_hid_irq,
				   irqflags | IRQF_ONESHOT, client->name, ihid);
	if (ret < 0) {
		dev_warn(&client->dev,
			"Could not register for %s interrupt, irq = %d,"
			" ret = %d\n",
			client->name, client->irq, ret);

		return ret;
	}

	return 0;
}

// 获得 i2c HID 描述符 到 struct i2c_hid->hdesc 即 (struct i2c_hid->hdesc_buffer)
static int i2c_hid_fetch_hid_descriptor(struct i2c_hid *ihid)
{
	struct i2c_client *client = ihid->client;
	struct i2c_hid_desc *hdesc = &ihid->hdesc;
	unsigned int dsize;
	int ret;

	/* i2c hid fetch using a fixed descriptor size (30 bytes) */
	// 使用固定的描述符大小(30字节)进行i2c hid获取
	if (i2c_hid_get_dmi_i2c_hid_desc_override(client->name)) {
		i2c_hid_dbg(ihid, "Using a HID descriptor override\n");
		ihid->hdesc =
			*i2c_hid_get_dmi_i2c_hid_desc_override(client->name);
	} else {
		i2c_hid_dbg(ihid, "Fetching the HID descriptor\n");
		// 发送获取 HID 描述符的cmd.
		ret = i2c_hid_command(client, &hid_descr_cmd,
				      ihid->hdesc_buffer,
				      sizeof(struct i2c_hid_desc));
		if (ret) {
			dev_err(&client->dev, "hid_descr_cmd failed\n");
			return -ENODEV;
		}
	}

	/* Validate the length of HID descriptor, the 4 first bytes:
	 * bytes 0-1 -> length
	 * bytes 2-3 -> bcdVersion (has to be 1.00) */
	/* check bcdVersion == 1.0 */
	/*验证HID描述符的长度，前4个字节:	
	* bytes 0-1 ->长度
	* bytes 2-3-> bcdVersion(必须是1.00)*/
	/*检查bcdVersion == 1.0*/
	if (le16_to_cpu(hdesc->bcdVersion) != 0x0100) {
		dev_err(&client->dev,
			"unexpected HID descriptor bcdVersion (0x%04hx)\n",
			le16_to_cpu(hdesc->bcdVersion));
		return -ENODEV;
	}

	/* Descriptor length should be 30 bytes as per the specification */
	// 描述符长度应该是30字节
	dsize = le16_to_cpu(hdesc->wHIDDescLength);
	if (dsize != sizeof(struct i2c_hid_desc)) {
		dev_err(&client->dev, "weird size of HID descriptor (%u)\n",
			dsize);
		return -ENODEV;
	}
	i2c_hid_dbg(ihid, "HID Descriptor: %*ph\n", dsize, ihid->hdesc_buffer);
	return 0;
}

#ifdef CONFIG_ACPI
static const struct acpi_device_id i2c_hid_acpi_blacklist[] = {
	/*
	 * The CHPN0001 ACPI device, which is used to describe the Chipone
	 * ICN8505 controller, has a _CID of PNP0C50 but is not HID compatible.
	 */
	{"CHPN0001", 0 },
	{ },
};

static int i2c_hid_acpi_pdata(struct i2c_client *client,
		struct i2c_hid_platform_data *pdata)
{
	static guid_t i2c_hid_guid =
		GUID_INIT(0x3CDFF6F7, 0x4267, 0x4555,
			  0xAD, 0x05, 0xB3, 0x0A, 0x3D, 0x89, 0x38, 0xDE);
	union acpi_object *obj;
	struct acpi_device *adev;
	acpi_handle handle;

	handle = ACPI_HANDLE(&client->dev);
	if (!handle || acpi_bus_get_device(handle, &adev)) {
		dev_err(&client->dev, "Error could not get ACPI device\n");
		return -ENODEV;
	}

	if (acpi_match_device_ids(adev, i2c_hid_acpi_blacklist) == 0)
		return -ENODEV;

	obj = acpi_evaluate_dsm_typed(handle, &i2c_hid_guid, 1, 1, NULL,
				      ACPI_TYPE_INTEGER);
	if (!obj) {
		dev_err(&client->dev, "Error _DSM call to get HID descriptor address failed\n");
		return -ENODEV;
	}

	pdata->hid_descriptor_address = obj->integer.value;
	ACPI_FREE(obj);

	return 0;
}

static void i2c_hid_acpi_fix_up_power(struct device *dev)
{
	struct acpi_device *adev;

	adev = ACPI_COMPANION(dev);
	if (adev)
		acpi_device_fix_up_power(adev);
}

static const struct acpi_device_id i2c_hid_acpi_match[] = {
	{"ACPI0C50", 0 },
	{"PNP0C50", 0 },
	{ },
};
MODULE_DEVICE_TABLE(acpi, i2c_hid_acpi_match);
#else
static inline int i2c_hid_acpi_pdata(struct i2c_client *client,
		struct i2c_hid_platform_data *pdata)
{
	return -ENODEV;
}

static inline void i2c_hid_acpi_fix_up_power(struct device *dev) {}
#endif

#ifdef CONFIG_OF
// 根据i2c_client的设备节点，获得 存储HID描述符的i2c寄存器。
static int i2c_hid_of_probe(struct i2c_client *client,
		struct i2c_hid_platform_data *pdata)
{
	struct device *dev = &client->dev;
	u32 val;
	int ret;

	// 在设备树上获得 "hid-descr-addr" 属性值到 val
	ret = of_property_read_u32(dev->of_node, "hid-descr-addr", &val);
	if (ret) {
		dev_err(&client->dev, "HID register address not provided\n");
		return -ENODEV;
	}
	if (val >> 16) {
		dev_err(&client->dev, "Bad HID register address: 0x%08x\n",
			val);
		return -EINVAL;
	}
	// 保存"hid-descr-addr" 属性值
	pdata->hid_descriptor_address = val;

	return 0;
}

// 该驱动程序支持的I2C设备树列表
static const struct of_device_id i2c_hid_of_match[] = {
	{ .compatible = "hid-over-i2c" },
	{},
};
MODULE_DEVICE_TABLE(of, i2c_hid_of_match);
#else
static inline int i2c_hid_of_probe(struct i2c_client *client,
		struct i2c_hid_platform_data *pdata)
{
	return -ENODEV;
}
#endif

// 在client设备树节点上获得 "post-power-on-delay-ms"  属性值到pdata
static void i2c_hid_fwnode_probe(struct i2c_client *client,
				 struct i2c_hid_platform_data *pdata)
{
	u32 val;
	if (!device_property_read_u32(&client->dev, "post-power-on-delay-ms",
				      &val))
		pdata->post_power_delay_ms = val;
}

// 设备struct i2c_client 与驱动匹配上了
static int i2c_hid_probe(struct i2c_client *client,
			 const struct i2c_device_id *dev_id)
{
	int ret;
	struct i2c_hid *ihid;
	struct hid_device *hid;
	__u16 hidRegister;
	// 获得设备平台数据
	struct i2c_hid_platform_data *platform_data = client->dev.platform_data;

	dbg_hid("HID probe called for i2c 0x%02x\n", client->addr);

	// 验证中断资源是否存在，和有效性
	if (!client->irq) {
		dev_err(&client->dev,
			"HID over i2c has not been provided an Int IRQ\n");
		return -EINVAL;
	}

	if (client->irq < 0) {
		if (client->irq != -EPROBE_DEFER)
			dev_err(&client->dev,
				"HID over i2c doesn't have a valid IRQ\n");
		return client->irq;
	}

	// 分配 struct i2c_hid 对象 
	ihid = devm_kzalloc(&client->dev, sizeof(*ihid), GFP_KERNEL);
	if (!ihid)
		return -ENOMEM;

	// 根据i2c_client，获得 存储HID描述符的i2c寄存器。
	if (client->dev.of_node) {
		// 来自设备对应的设备树节点
		ret = i2c_hid_of_probe(client, &ihid->pdata);
		if (ret)
			return ret;
	} else if (!platform_data) {
		// 来自 acpi
		ret = i2c_hid_acpi_pdata(client, &ihid->pdata);
		if (ret)
			return ret;
	} else {
		// 来自 设备的平台数据
		ihid->pdata = *platform_data;
	}

	/* Parse platform agnostic common properties from ACPI / device tree */
	// 解析来自ACPI /设备树的平台无关的公共属性

	// 在client设备树节点上获得 "post-power-on-delay-ms"  属性值到 &ihid->pdata
	i2c_hid_fwnode_probe(client, &ihid->pdata);

	ihid->pdata.supplies[0].supply = "vdd";
	ihid->pdata.supplies[1].supply = "vddl";

	//  managed获取多个调节器消费者
	ret = devm_regulator_bulk_get(&client->dev,
				      ARRAY_SIZE(ihid->pdata.supplies),
				      ihid->pdata.supplies);
	if (ret)
		return ret;

	// 启用多个调整器使用者
	ret = regulator_bulk_enable(ARRAY_SIZE(ihid->pdata.supplies),
				    ihid->pdata.supplies);
	if (ret < 0)
		return ret;

	// 开始延时指定时间
	if (ihid->pdata.post_power_delay_ms)
		msleep(ihid->pdata.post_power_delay_ms);

	// struct i2c_hid 记录在 i2c_client设备的驱动数据中。
	i2c_set_clientdata(client, ihid);
	// struct i2c_client 记录在i2c_hid中
	ihid->client = client;

	hidRegister = ihid->pdata.hid_descriptor_address;
	ihid->wHIDDescRegister = cpu_to_le16(hidRegister);

	init_waitqueue_head(&ihid->wait);
	mutex_init(&ihid->reset_lock);

	/* we need to allocate the command buffer without knowing the maximum
	 * size of the reports. Let's use HID_MIN_BUFFER_SIZE, then we do the
	 * real computation later. */
	// 我们需要在不知道报表最大大小的情况下分配命令缓冲区。 让我们使用HID_MIN_BUFFER_SIZE，然后再进行真正的计算。
	// 为 struct i2c_hid的inbuf、rawbuf、argsbuf、cmdbuf 分配空间
	ret = i2c_hid_alloc_buffers(ihid, HID_MIN_BUFFER_SIZE);
	if (ret < 0)
		goto err_regulator;

	i2c_hid_acpi_fix_up_power(&client->dev);

	pm_runtime_get_noresume(&client->dev);
	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	device_enable_async_suspend(&client->dev);

	/* Make sure there is something at this address */
	// 这个地址一定可以正常通信
	ret = i2c_smbus_read_byte(client);
	if (ret < 0) {
		dev_dbg(&client->dev, "nothing at this address: %d\n", ret);
		ret = -ENXIO;
		goto err_pm;
	}

	// 获得 i2c HID 描述符
	ret = i2c_hid_fetch_hid_descriptor(ihid);
	if (ret < 0)
		goto err_pm;

	// 注册中断处理函数 i2c_hid_irq
	ret = i2c_hid_init_irq(client);
	if (ret < 0)
		goto err_pm;

	// 分配新的hid设备描述符	struct hid_device.
	hid = hid_allocate_device();
	if (IS_ERR(hid)) {
		ret = PTR_ERR(hid);
		goto err_irq;
	}

	ihid->hid = hid;

	hid->driver_data = client;
	hid->ll_driver = &i2c_hid_ll_driver;
	hid->dev.parent = &client->dev;
	hid->bus = BUS_I2C;
	hid->version = le16_to_cpu(ihid->hdesc.bcdVersion);
	hid->vendor = le16_to_cpu(ihid->hdesc.wVendorID);
	hid->product = le16_to_cpu(ihid->hdesc.wProductID);

	snprintf(hid->name, sizeof(hid->name), "%s %04hX:%04hX",
		 client->name, hid->vendor, hid->product);
	strlcpy(hid->phys, dev_name(&client->dev), sizeof(hid->phys));

	ihid->quirks = i2c_hid_lookup_quirk(hid->vendor, hid->product);

	// 添加设备struct hid_device  到系统中
	ret = hid_add_device(hid);
	if (ret) {
		if (ret != -ENODEV)
			hid_err(client, "can't add hid device: %d\n", ret);
		goto err_mem_free;
	}

	if (!(ihid->quirks & I2C_HID_QUIRK_NO_RUNTIME_PM))
		pm_runtime_put(&client->dev);

	return 0;

err_mem_free:
	hid_destroy_device(hid);

err_irq:
	free_irq(client->irq, ihid);

err_pm:
	pm_runtime_put_noidle(&client->dev);
	pm_runtime_disable(&client->dev);

err_regulator:
	regulator_bulk_disable(ARRAY_SIZE(ihid->pdata.supplies),
			       ihid->pdata.supplies);
	i2c_hid_free_buffers(ihid);
	return ret;
}

static int i2c_hid_remove(struct i2c_client *client)
{
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	struct hid_device *hid;

	if (!(ihid->quirks & I2C_HID_QUIRK_NO_RUNTIME_PM))
		pm_runtime_get_sync(&client->dev);
	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);
	pm_runtime_put_noidle(&client->dev);

	hid = ihid->hid;
	hid_destroy_device(hid);

	free_irq(client->irq, ihid);

	if (ihid->bufsize)
		i2c_hid_free_buffers(ihid);

	regulator_bulk_disable(ARRAY_SIZE(ihid->pdata.supplies),
			       ihid->pdata.supplies);

	return 0;
}

static void i2c_hid_shutdown(struct i2c_client *client)
{
	struct i2c_hid *ihid = i2c_get_clientdata(client);

	i2c_hid_set_power(client, I2C_HID_PWR_SLEEP);
	free_irq(client->irq, ihid);
}

#ifdef CONFIG_PM_SLEEP
static int i2c_hid_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	struct hid_device *hid = ihid->hid;
	int ret;
	int wake_status;

	if (hid->driver && hid->driver->suspend) {
		/*
		 * Wake up the device so that IO issues in
		 * HID driver's suspend code can succeed.
		 */
		ret = pm_runtime_resume(dev);
		if (ret < 0)
			return ret;

		ret = hid->driver->suspend(hid, PMSG_SUSPEND);
		if (ret < 0)
			return ret;
	}

	if (!pm_runtime_suspended(dev)) {
		/* Save some power */
		i2c_hid_set_power(client, I2C_HID_PWR_SLEEP);

		disable_irq(client->irq);
	}

	if (device_may_wakeup(&client->dev)) {
		wake_status = enable_irq_wake(client->irq);
		if (!wake_status)
			ihid->irq_wake_enabled = true;
		else
			hid_warn(hid, "Failed to enable irq wake: %d\n",
				wake_status);
	} else {
		regulator_bulk_disable(ARRAY_SIZE(ihid->pdata.supplies),
				       ihid->pdata.supplies);
	}

	return 0;
}

static int i2c_hid_resume(struct device *dev)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	struct hid_device *hid = ihid->hid;
	int wake_status;

	if (!device_may_wakeup(&client->dev)) {
		ret = regulator_bulk_enable(ARRAY_SIZE(ihid->pdata.supplies),
					    ihid->pdata.supplies);
		if (ret)
			hid_warn(hid, "Failed to enable supplies: %d\n", ret);

		if (ihid->pdata.post_power_delay_ms)
			msleep(ihid->pdata.post_power_delay_ms);
	} else if (ihid->irq_wake_enabled) {
		wake_status = disable_irq_wake(client->irq);
		if (!wake_status)
			ihid->irq_wake_enabled = false;
		else
			hid_warn(hid, "Failed to disable irq wake: %d\n",
				wake_status);
	}

	/* We'll resume to full power */
	pm_runtime_disable(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	enable_irq(client->irq);

	/* Instead of resetting device, simply powers the device on. This
	 * solves "incomplete reports" on Raydium devices 2386:3118 and
	 * 2386:4B33 and fixes various SIS touchscreens no longer sending
	 * data after a suspend/resume.
	 */
	ret = i2c_hid_set_power(client, I2C_HIDWR_ON);
	if (ret)
		return ret;

	if (hid->driver && hid->driver->reset_resume) {
		ret = hid->driver->reset_resume(hid);
		return ret;
	}

	return 0;
}
#endif

#ifdef CONFIG_PM
static int i2c_hid_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	i2c_hid_set_power(client, I2C_HID_PWR_SLEEP);
	disable_irq(client->irq);
	return 0;
}

static int i2c_hid_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	enable_irq(client->irq);
	i2c_hid_set_power(client, I2C_HID_PWR_ON);
	return 0;
}
#endif

static const struct dev_pm_ops i2c_hid_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(i2c_hid_suspend, i2c_hid_resume)
	SET_RUNTIME_PM_OPS(i2c_hid_runtime_suspend, i2c_hid_runtime_resume,
			   NULL)
};

// 该驱动程序支持的I2C设备列表
static const struct i2c_device_id i2c_hid_id_table[] = {
	{ "hid", 0 },
	{ "hid-over-i2c", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, i2c_hid_id_table);


static struct i2c_driver i2c_hid_driver = {
	.driver = {
		.name	= "i2c_hid",
		.pm	= &i2c_hid_pm,
		.acpi_match_table = ACPI_PTR(i2c_hid_acpi_match),
		.of_match_table = of_match_ptr(i2c_hid_of_match),
	},

	.probe		= i2c_hid_probe,
	.remove		= i2c_hid_remove,
	.shutdown	= i2c_hid_shutdown,
	.id_table	= i2c_hid_id_table,
};

module_i2c_driver(i2c_hid_driver);
/*
等价于====
static int __init i2c_hid_driver_init(void) 
{ 
	return i2c_add_driver(&(i2c_hid_driver)); 
} 
module_init(i2c_hid_driver_init); 
static void __exit i2c_hid_driver_exit(void) 
{ 
	i2c_del_driver(&(i2c_hid_driver)); 
} 
module_exit(i2c_hid_driver_exit);
----------------------
#define module_i2c_driver(__i2c_driver) \
	module_driver(__i2c_driver, i2c_add_driver, \
			i2c_del_driver)
----------------------
#define module_driver(__driver, __register, __unregister, ...) \
static int __init __driver##_init(void) \
{ \
	return __register(&(__driver) , ##__VA_ARGS__); \
} \
module_init(__driver##_init); \
static void __exit __driver##_exit(void) \
{ \
	__unregister(&(__driver) , ##__VA_ARGS__); \
} \
module_exit(__driver##_exit);
*/

MODULE_DESCRIPTION("HID over I2C core driver");
MODULE_AUTHOR("Benjamin Tissoires <benjamin.tissoires@gmail.com>");
MODULE_LICENSE("GPL");
