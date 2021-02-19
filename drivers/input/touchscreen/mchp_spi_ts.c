/*
 * mchp_spi_ts.c
 *
 * Microchip maXTouch Touchscreen driver
 * For use with maXTouch products supporting the SPI interface.
 * 
 * Copyright (C) 2021 Microchip Technology

 * Author: Michael Gong <michael.gong@microchip.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/input/mt.h>
#include <linux/of.h>
#include <linux/irq.h>	
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <asm/unaligned.h>
#include <linux/spi/spi.h>

/* Firmware files */
#define MXT_FW_NAME		"maxtouch.fw"

/* Configuration file */
#define MXT_CFG_NAME		"maxtouch.cfg"
#define MXT_CFG_MAGIC		"OBP_RAW V1"

/* Registers */
#define MXT_OBJECT_START	0x07
#define MXT_OBJECT_SIZE		6
#define MXT_INFO_CHECKSUM_SIZE	3
#define MXT_MAX_SPI_BLOCK	64
#define USE_SPI_DMA		0
#define SPI_HEADER_SIZE		0x06
#define SPI_MAX_SIZE_LIMIT	MXT_MAX_SPI_BLOCK + SPI_HEADER_SIZE

/* mXT SPI Definitions */
#define MXT_SPI_WRITE_REQ	0x01
#define MXT_SPI_WRITE_OK	0x81
#define MXT_SPI_WRITE_FAIL	0x41
#define MXT_SPI_READ_REQ	0x02
#define MXT_SPI_READ_OK		0x82
#define MXT_SPI_READ_FAIL	0x42
#define MXT_SPI_INVALID_REQ	0x04
#define MXT_SPI_INVALID_CRC	0x08

/* Object types */
#define MXT_DEBUG_DIAGNOSTIC_T37	37
#define MXT_GEN_MESSAGE_T5		5
#define MXT_GEN_COMMAND_T6		6
#define MXT_GEN_POWER_T7		7
#define MXT_GEN_ACQUIRE_T8		8
#define MXT_PROCI_KEYTHRESHOLD_T14	14
#define MXT_TOUCH_KEYARRAY_T15		15
#define MXT_SPT_COMMSCONFIG_T18		18
#define MXT_SPT_GPIOPWM_T19		19
#define MXT_PROCG_NOISE_T22		22
#define MXT_PROCI_ONETOUCH_T24		24
#define MXT_SPT_SELFTEST_T25		25
#define MXT_PROCI_TWOTOUCH_T27		27
#define MXT_SPT_USERDATA_T38		38
#define MXT_PROCI_GRIP_T40		40
#define MXT_PROCI_TOUCHSUPPRESSION_T42	42
#define MXT_SPT_DIGITIZER_T43		43
#define MXT_SPT_MESSAGECOUNT_T44	44
#define MXT_SPT_CTECONFIG_T46		46
#define MXT_PROCI_STYLUS_T47		47
#define MXT_PROCI_SHIELDLESS_T56	56
#define MXT_SPT_TIMER_T61		61
#define MXT_PROCI_LENSBENDING_T65	65
#define MXT_SPI_SERIALDATACOMMAND_T68	68
#define MXT_SPT_DYNAMICCONFIGURATIONCONTROLLER_T70 70
#define MXT_SPT_DYNAMICCONFIGURATIONCONTAINER_T71  71
#define MXT_NOISESUPPRESSION_T72		   72
#define MXT_PROCI_GLOVEDETECTION_T78		   78
#define MXT_SPT_TOUCHEVENTTRIGGER_T79		   79
#define MXT_PROCI_RETRANSMISSIONCOMPENSATION_T80   80
#define MXT_TOUCH_MULTITOUCHSCREEN_T100 	   100

/* MXT_GEN_MESSAGE_T5 object */
#define MXT_RPTID_NOMSG		0xff
#define MXT_RPTID_RVSD		0x00

/* Delay times */
#define MXT_BACKUP_TIME		50	/* msec */
#define MXT_RESET_GPIO_TIME	20	/* msec */
#define MXT_RESET_INVALID_CHG	1000	/* msec */
#define MXT_RESET_TIME		1000	/* msec */
#define MXT_RESET_TIMEOUT	3000	/* msec */
#define MXT_CRC_TIMEOUT		1000	/* msec */
#define MXT_FW_FLASH_TIME	1000	/* msec */
#define MXT_FW_RESET_TIME	3000	/* msec */
#define MXT_FW_CHG_TIMEOUT	300	/* msec */
#define MXT_BOOTLOADER_WAIT	3000	/* msec */
#define MXT_CS_MAX_DELAY 	50 	/* msec */
#define MXT_REPORTALL_MAX_DELAY	50 	/* msec */
#define MXT_CONFIG_MAX_DELAY 	3 	/* msec */
#define MXT_BOOTLOADER_READ 	0x01	/* msec */

/* MXT_GEN_COMMAND_T6 field */
#define MXT_COMMAND_RESET	0
#define MXT_COMMAND_BACKUPNV	1
#define MXT_COMMAND_CALIBRATE	2
#define MXT_COMMAND_REPORTALL	3
#define MXT_COMMAND_DIAGNOSTIC	5

/* Command to unlock bootloader */
#define MXT_UNLOCK_CMD_MSB	0xaa
#define MXT_UNLOCK_CMD_LSB	0xdc

/* Bootloader mode status */
#define MXT_WAITING_BOOTLOAD_CMD	0xc0	/* valid 7 6 bit only */
#define MXT_WAITING_FRAME_DATA		0x80	/* valid 7 6 bit only */
#define MXT_FRAME_CRC_CHECK		0x02
#define MXT_FRAME_CRC_FAIL		0x03
#define MXT_FRAME_CRC_PASS		0x04
#define MXT_APP_CRC_FAIL		0x40	/* valid 7 8 bit only */
#define MXT_BOOT_STATUS_MASK		0x3f
#define MXT_BOOT_ID_MASK		0x1f

/* Define for T6 status byte */
#define MXT_T6_STATUS_RESET	BIT(7)
#define MXT_T6_STATUS_OFL	BIT(6)
#define MXT_T6_STATUS_SIGERR	BIT(5)

/* Define for MXT_GEN_COMMAND_T6 */
#define MXT_FLASH_CMD		0xa5
#define MXT_RESET_VALUE		0x01
#define MXT_BACKUP_VALUE	0x55
#define MXT_BACKUP_W_STOP	0x33
#define MXT_T6_STATUS_CAL	BIT(4)
#define MXT_T6_STATUS_CFGERR	BIT(3)
#define MXT_T6_STATUS_COMSERR	BIT(2)

/* T100 Multiple Touch Touchscreen */
#define MXT_T100_CTRL		0
#define MXT_T100_CFG1		1
#define MXT_T100_TCHAUX		3
#define MXT_T100_XSIZE		9
#define MXT_T100_XRANGE		13
#define MXT_T100_YSIZE		20
#define MXT_T100_YRANGE		24
#define MXT_T100_AUX_OFFSET	6
#define MXT_RSVD_RPTIDS		2
#define MXT_MIN_RPTID_SEC	18

#define MXT_T100_CFG_SWITCHXY	BIT(5)
#define MXT_T100_CFG_INVERTY	BIT(6)
#define MXT_T100_CFG_INVERTX	BIT(7)

#define MXT_T100_TCHAUX_VECT	BIT(0)
#define MXT_T100_TCHAUX_AMPL	BIT(1)
#define MXT_T100_TCHAUX_AREA	BIT(2)
#define MXT_T100_DETECT			BIT(7)

#define MXT_T100_TYPE_MASK		0x70
#define MXT_T100_ENABLE_BIT_MASK	0x01

/* Define for MXT_PROCI_TOUCHSUPPRESSION_T42 */
#define MXT_T42_MSG_TCHSUP	BIT(0)

enum t100_type {
	MXT_T100_TYPE_FINGER		= 1,
	MXT_T100_TYPE_PASSIVE_STYLUS	= 2,
	MXT_T100_TYPE_ACTIVE_STYLUS	= 3,
	MXT_T100_TYPE_HOVERING_FINGER	= 4,
	MXT_T100_TYPE_GLOVE		= 5,
	MXT_T100_TYPE_LARGE_TOUCH	= 6,
};

#define MXT_DISTANCE_ACTIVE_TOUCH	0
#define MXT_DISTANCE_HOVERING		1

#define MXT_TOUCH_MAJOR_DEFAULT		1
#define MXT_PRESSURE_DEFAULT		1

#define MXT_POWER_CFG_RUN		0
#define MXT_POWER_CFG_DEEPSLEEP		1

/* MXT_SPT_COMMSCONFIG_T18 */
#define MXT_COMMS_CTRL		0
#define MXT_COMMS_CMD		1
#define MXT_COMMS_RETRIGEN	BIT(6)

/* MXT_GEN_POWER_T7 field */
struct t7_config {
	u8 idle;
	u8 active;
} __packed;

/* Debug message size max */
#define DEBUG_MSG_MAX		200

struct mxt_info {
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u8 matrix_xsize;
	u8 matrix_ysize;
	u8 object_num;
};

/* Config update context */
struct mxt_cfg {
	u8 *raw;
	size_t raw_size;
	off_t raw_pos;
	u8 *mem;
	size_t mem_size;
	int start_ofs;
	struct mxt_info info;
};

enum interrupt_state {
	MXT_IRQ_PROCESS_READ	= 1,
	MXT_IRQ_READ_REQ	= 2,
	MXT_IRQ_READ_DONE 	= 3,
	MXT_IRQ_WRITE_REQ 	= 4,
	MXT_IRQ_WRITE_DONE 	= 5,
};

struct mxt_object {
	u8 type;
	u16 start_address;
	u8 size_minus_one;
	u8 instances_minus_one;
	u8 num_report_ids;
} __packed;

struct request {
	struct spi_message msg;
	struct spi_transfer xfer[2];
};

struct mxt_data {
	struct spi_device *spi;
	struct input_dev *input_dev;
	char phys[64];			/* device physical location */
	struct mxt_object *object_table;
	struct mxt_info *info;
	void *raw_info_block;

	u8 spi_rd_buf[SPI_MAX_SIZE_LIMIT];  /* SPI read buffer */
	u8 spi_wr_buf[SPI_MAX_SIZE_LIMIT];  /* SPI write buffer */

	unsigned int irq;
	unsigned int max_x;
	unsigned int max_y;
	bool invertx;
	bool inverty;
	bool xy_switch;
	u8 xsize;
	u8 ysize;
	bool in_bootloader;
	u16 mem_size;
	u8 t100_aux_ampl;
	u8 t100_aux_area;
	u8 t100_aux_vect;
	struct bin_attribute mem_access_attr;
	bool debug_enabled;
	bool debug_v2_enabled;
	u8 *debug_msg_data;
	u16 debug_msg_count;
	struct bin_attribute debug_msg_attr;
	struct mutex debug_msg_lock;
	struct mutex spi_transfer_lock;
	u8 max_reportid;
	u32 config_crc;
	u32 info_crc;
	u8 *msg_buf;
	bool update_input;
	u8 num_touchids;
	u8 multitouch;
	struct t7_config t7_cfg;
	unsigned long t15_keystatus;
	struct gpio_desc *reset_gpio;

	/* Cached parameters from object table */
	u16 T5_address;
	u8 T5_msg_size;
	u8 T6_reportid;
	u16 T6_address;
	u16 T7_address;
	u16 T14_address;
	u16 T38_address;
	u16 T71_address;
	u8 T15_reportid_min;
	u8 T15_reportid_max;
	u16 T18_address;
	u8 T19_reportid;
	u8 T19_reportid_min;
	u8 T42_reportid_min;
	u8 T42_reportid_max;
	u16 T44_address;
	u8 T100_reportid_min;
	u8 T100_reportid_max;

	/* Cached instance parameter */
	u8 T100_instances;
	u8 T15_instances;

	enum interrupt_state irq_state;

	/* for fw update in bootloader */
	struct completion bl_completion;

	/* for reset handling */
	struct completion reset_completion;

	/* for config update handling */
	struct completion crc_completion;

	struct completion t6_cmd_completion;
	struct completion read_completion;
	struct completion write_completion;

	u32 *t19_keymap;
	unsigned int t19_num_keys;

	u32 *t15_keymap;
	unsigned int t15_num_keys;

	/* Indicates whether device is updating configuration */
	volatile bool sysfs_updating_config_fw;

	bool irq_processing;
	bool system_power_up;
};

/*
 *  mxt_wait_for_completion - Manage completion operations
 * 
 *  Used to catch interrupt after SPI transfer
 *   request or timeout(ms) otherwise
 * 
 *  Returns -ERESTARTSYS if interrupted, 0 if timed out
 *   positive if completed
 *
*/

static int mxt_wait_for_completion(struct mxt_data *data,
				   struct completion *comp,
				   unsigned int timeout_ms)
{
	struct device *dev = &data->spi->dev;
	unsigned long timeout = msecs_to_jiffies(timeout_ms);
	long ret;

	ret = wait_for_completion_interruptible_timeout(comp, timeout);
	if (ret < 0) {
		return ret;
	} else if (ret == 0) {
		dev_err(dev, "Wait for completion timed out.\n");
		return -ETIMEDOUT;
	} else {
		dev_dbg(dev, "Time left: %u ms\n", jiffies_to_msecs(ret));
	}

	return 0;
}

static int mxt_set_tx_buffer(struct mxt_data *data)
{
	/* Initialize global tx buffer for SPI transfers */

	memset(data->spi_wr_buf, 0xFF, sizeof(data->spi_wr_buf));

	return 0;
}

/*   __mxt_calc_crc8 - Calculate 8bit CRC
 * 
 *  Used for SPI read/write requests and to valid touch messages 
*/

static u8 __mxt_calc_crc8(unsigned char crc, unsigned char data)
{
	static const u8 crcpoly = 0x8C;
	u8 index;
	u8 fb;
	index = 8;
		
	do {
		fb = (crc ^ data) & 0x01;
		data >>= 1;
		crc >>= 1;
		if (fb)
			crc ^= crcpoly;
	} while (--index);
	
	return crc;
}

/*  mxt_spi_read - System powerup read
 *
 *  Initializes SPI IO to correct state
 *
*/

static int mxt_spi_read(struct mxt_data *data)
{
	struct spi_device *spi = data->spi;
	int ret;

	ret = spi_read(spi, data->spi_rd_buf, sizeof(data->spi_rd_buf));

	return ret;
}

/*  mxt_spi_write_xfer - Write full SPI message
 *
 *  Writes one or two bytes of data to config 
 *  registers. Ignores IRQ message on response.
 *  Send request and response message within
 *  same function.
 *  
*/

static int mxt_spi_write_xfer(struct mxt_data *data, u16 reg, u16 len, void *val)
{
	
	struct spi_device *spi = data->spi;
	struct request *tr;
	u8 tx_buf[16];
	u8 rx_buf[16];
	u16 msg_length;
	u8 crc_data = 0;
	int ret = 0;
	int i;

	tr = kzalloc(sizeof *tr, GFP_KERNEL);
	if (!tr) {
		return -ENOMEM;
	}

	msg_length = len;
	crc_data = 0;
	
	tx_buf[0] = MXT_SPI_WRITE_REQ;
	tx_buf[1] = reg & 0xff;
	tx_buf[2] = ((reg >> 8) & 0xff);
	tx_buf[3] = msg_length & 0xff;
	tx_buf[4] = ((msg_length >> 8) & 0xff);

	for (i = 0; i < (SPI_HEADER_SIZE-1); i++) {
		crc_data = __mxt_calc_crc8(crc_data, tx_buf[i]);
	}
		
	tx_buf[5] = crc_data;

	/* Append message to transmit buffer*/

	memcpy(tx_buf + SPI_HEADER_SIZE, val, msg_length);

	spi_message_init(&tr->msg);
	tr->msg.spi = spi;

	tr->xfer[0].tx_buf = tx_buf;
	tr->xfer[0].rx_buf = rx_buf;
	tr->xfer[0].len = (SPI_HEADER_SIZE + msg_length) ;
	//request->xfer[0].delay_usecs = 500;
	tr->xfer[0].cs_change = 1;
	tr->xfer[0].cs_change_delay.unit = 0;
	tr->xfer[0].cs_change_delay.value = 1000;

	spi_message_add_tail(&tr->xfer[0], &tr->msg);

	tr->xfer[1].tx_buf = data->spi_wr_buf;
	tr->xfer[1].rx_buf = rx_buf;
	tr->xfer[1].len = SPI_HEADER_SIZE;
	
	spi_message_add_tail(&tr->xfer[1], &tr->msg);

	ret = spi_sync(spi, &tr->msg);

	if (ret < 0)
		dev_err(&spi->dev, "SPI transactions failed\n");

	if (rx_buf[0] != MXT_SPI_WRITE_OK) {
		dev_err(&spi->dev, "spi_write_transfer: SPI write error\n");
	}
	
	kfree(tr);
	
	return ret;

}

/*  mxt_spi_write_process - Process write
 *
 *  Used to process write request message
 *  Return -EIO if write response is not OK
 *
*/

static int mxt_spi_write_process(struct mxt_data *data)
{
	
	struct spi_device *spi = data->spi;
	struct request *tr;
	u8 tx_buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	u8 rx_buf[8];
	int ret = 0;

	tr = kzalloc(sizeof *tr, GFP_KERNEL);
	if (!tr) {
		return -ENOMEM;
	}

	spi_message_init(&tr->msg);
	tr->msg.spi = spi;

	tr->xfer[0].tx_buf = tx_buf;
	tr->xfer[0].rx_buf = rx_buf;
	tr->xfer[0].len = SPI_HEADER_SIZE;
	tr->xfer[0].cs_change = 1;
	
	spi_message_add_tail(&tr->xfer[0], &tr->msg);

	ret = spi_sync(spi, &tr->msg);

	if (ret < 0)
		dev_err(&spi->dev, "SPI transactions failed\n");

	if (rx_buf[0] != MXT_SPI_WRITE_OK) {
		dev_err(&spi->dev, "SPI write error: %x\n", rx_buf[0]);
		ret = -EIO;
	}
	
	kfree(tr);

	return ret;
}

/*  mxt_spi_write_req - Send write request
 *
 *  Used to send SPI request message
 *   
*/

static int mxt_spi_write_req(struct mxt_data *data, u16 reg, u16 len, const void *val)
{
	
	struct spi_device *spi = data->spi;
	struct request *tr;
	u8 tx_buf[SPI_MAX_SIZE_LIMIT];
	u16 msg_length;
	u8 crc_data = 0;
	int ret = 0;
	int i;

	data->irq_state = MXT_IRQ_WRITE_REQ;

	tr = kzalloc(sizeof *tr, GFP_KERNEL);
	if (!tr) {
		return -ENOMEM;
	}

	msg_length = len;
	crc_data = 0;
	
	tx_buf[0] = MXT_SPI_WRITE_REQ;
	tx_buf[1] = reg & 0xff;
	tx_buf[2] = ((reg >> 8) & 0xff);
	tx_buf[3] = msg_length & 0xff;
	tx_buf[4] = ((msg_length >> 8) & 0xff);

	for (i = 0; i < (SPI_HEADER_SIZE-1); i++) {
		crc_data = __mxt_calc_crc8(crc_data, tx_buf[i]);
	}
		
	tx_buf[5] = crc_data;

	/* Append message to transmit buffer*/

	memcpy(tx_buf + SPI_HEADER_SIZE, val, msg_length);

	spi_message_init(&tr->msg);
	tr->msg.spi = spi;

	tr->xfer[0].tx_buf = tx_buf;
	tr->xfer[0].rx_buf = data->spi_rd_buf;
	tr->xfer[0].len = (SPI_HEADER_SIZE + msg_length) ;
	tr->xfer[0].delay_usecs = 50;

	spi_message_add_tail(&tr->xfer[0], &tr->msg);

	ret = spi_sync(spi, &tr->msg);

	if (ret < 0)
		dev_err(&spi->dev, "SPI transactions failed\n");

	kfree(tr);

	return ret;
}

/*  mxt_write_block - Block write routine
 *
 *  Write multiple message packets. Maximum packet specified by 
 *   MXT_MAX_SPI_BLOCK.  Completion flags triggered by interrupt
 *   generated by write request.
 *   
*/

static int mxt_write_block (struct mxt_data *data, u16 reg, u16 len, const void *val)
{

	size_t totalBytesToWrite;
	void *databuf;
	u16 msg_length = 0;
	u16 data_ptr = 0;
	int ret = 0;

	databuf = kzalloc(len, GFP_KERNEL);
	if (!databuf)
		return -ENOMEM;

	totalBytesToWrite = len;

	do {
		reinit_completion(&data->write_completion);

 		if (totalBytesToWrite > MXT_MAX_SPI_BLOCK)
 			msg_length = MXT_MAX_SPI_BLOCK;
 		else 
 			msg_length = totalBytesToWrite;

		ret = mxt_spi_write_req(data, (reg + data_ptr), msg_length, val);
 
		mxt_wait_for_completion(data, &data->write_completion,
					      MXT_CS_MAX_DELAY);

		ret = mxt_spi_write_process(data);
		msleep(1);

		data_ptr += msg_length;
		totalBytesToWrite -= msg_length;

	} while (totalBytesToWrite > 0);

	kfree(databuf);

	return ret;
}

/*  mxt_spi_read_req - Request a read
 *
 *  Send request to read a register
 *   
*/

static int mxt_spi_read_req(struct mxt_data *data, u16 reg, u16 len)
{
	struct spi_device *spi = data->spi;
	struct request *tr;
	int i;
	u8 tx_buf[7];
	u8 rx_buf[7];
	u16 msg_length;
	u8 crc_data = 0;
	int ret = 0;

	tr = kzalloc(sizeof *tr, GFP_KERNEL);
	if (!tr)
		return -ENOMEM;

	msg_length = len;
	
	crc_data = 0;
	
	tx_buf[0] = MXT_SPI_READ_REQ;
	tx_buf[1] = reg & 0xff;
	tx_buf[2] = ((reg >> 8) & 0xff);
	tx_buf[3] = msg_length & 0xff;
	tx_buf[4] = ((msg_length >> 8) & 0xff);

	for (i = 0; i < (SPI_HEADER_SIZE-1); i++) {
		crc_data = __mxt_calc_crc8(crc_data, tx_buf[i]);
	}
		
	tx_buf[5] = crc_data;

	spi_message_init(&tr->msg);
	tr->msg.spi = spi;

	tr->xfer[0].tx_buf = tx_buf;
	tr->xfer[0].rx_buf = rx_buf;
	tr->xfer[0].len = (SPI_HEADER_SIZE);
	//request->xfer[0].delay_usecs = 500;
	tr->xfer[0].cs_change = 1;

	spi_message_add_tail(&tr->xfer[0], &tr->msg);

	ret = spi_sync(spi, &tr->msg);

	if (ret < 0)
		dev_err(&spi->dev, "SPI transactions failed\n");

	kfree(tr);

	return ret;	

}

/*  mxt_spi_read_xfer - Read full SPI message
 *
 *  Read one or multiple bytes of data.
 *  Ignores IRQ on response.
 *  Send request and response message within
 *  same transfer
 *  
*/

static int mxt_spi_transfer(struct mxt_data *data, u16 reg, u16 len, void *val)
{
	
	struct spi_device *spi = data->spi;
	struct request *tr;
	u16 total_byte_count;
	u16 write_addr = 0;
	u16 data_ptr = 0;
	u16 msg_length;
	u8 crc_data = 0;
	u8 tx_buf[7];	
	u8 rx_buf[7];
	int ret = 0;
	int i;

	tr = kzalloc(sizeof *tr, GFP_ATOMIC);
	if (!tr)
		return -ENOMEM;

	total_byte_count = len;

	do {
		if ((reg == data->T5_address) && (reg != 0x0000)) {
			msg_length = data->T5_msg_size;

		} else if (total_byte_count > MXT_MAX_SPI_BLOCK) {
			msg_length = MXT_MAX_SPI_BLOCK;

		} else {
			msg_length = total_byte_count;
		}
	
		crc_data = 0;

		/* data_ptr: tracks location in buffer */
		if ((reg == data->T5_address) && (reg != 0x0000))
			write_addr = reg;	
		else
			write_addr = reg + data_ptr;

		tx_buf[0] = MXT_SPI_READ_REQ;
		tx_buf[1] = write_addr & 0xff;
		tx_buf[2] = ((write_addr >> 8) & 0xff);
		tx_buf[3] = msg_length & 0xff;
		tx_buf[4] = ((msg_length >> 8) & 0xff);

		for (i = 0; i < (SPI_HEADER_SIZE-1); i++) {
			crc_data = __mxt_calc_crc8(crc_data, tx_buf[i]);
		}
		
		tx_buf[5] = crc_data;

		/* Zero-initialize message for future compatiblity */
		memset(&tr->msg, 0x00, sizeof(tr->msg));

		spi_message_init(&tr->msg);
		tr->msg.spi = spi;
		tr->msg.is_dma_mapped = USE_SPI_DMA;

		tr->xfer[0].tx_buf = tx_buf;
		tr->xfer[0].rx_buf = rx_buf;
		tr->xfer[0].len = (SPI_HEADER_SIZE);
		
		tr->xfer[0].cs_change = 1;
		tr->xfer[0].cs_change_delay.unit = 0;
		tr->xfer[0].cs_change_delay.value = 1000;
		//request->xfer[0].delay_usecs = 500;

		spi_message_add_tail(&tr->xfer[0], &tr->msg);

		tr->xfer[1].tx_buf = data->spi_wr_buf;
		tr->xfer[1].rx_buf = data->spi_rd_buf;
		tr->xfer[1].len = (msg_length + SPI_HEADER_SIZE);
		tr->xfer[1].cs_change_delay.unit = 0;
		tr->xfer[1].cs_change_delay.value = 1000;
	
		spi_message_add_tail(&tr->xfer[1], &tr->msg);

		ret = spi_sync(spi, &tr->msg);

		if (ret < 0)
			dev_err(&spi->dev, "SPI transactions failed\n");
		
		memcpy((val + data_ptr), &data->spi_rd_buf[SPI_HEADER_SIZE], msg_length);

		data_ptr += msg_length;
		total_byte_count -= msg_length;

	} while (total_byte_count > 0);

	kfree(tr);

	return ret;
}

/*  mxt_read_data - Read and process message
 *
 *  Read one or multiple SPI messages
 *  Send request and response message within
 *  same transfer
 *  
*/

static int mxt_read_data(struct mxt_data *data, u16 reg, u16 len, void *val)
{
	
	struct spi_device *spi = data->spi;
	struct request *tr;
	u16 msg_length;
	int ret = 0;
	u16 total_byte_count;
	u16 data_ptr = 0;
	
	tr = kzalloc(sizeof *tr, GFP_KERNEL);
	if (!tr)
		return -ENOMEM;

	total_byte_count = len;

	do {
		if (reg == data->T5_address) {
			msg_length = data->T5_msg_size;

		} else if (total_byte_count > MXT_MAX_SPI_BLOCK) {
			msg_length = MXT_MAX_SPI_BLOCK;

		} else {
			msg_length = total_byte_count;
		}

		spi_message_init(&tr->msg);
		tr->msg.spi = spi;

		tr->xfer[0].tx_buf = data->spi_wr_buf;
		tr->xfer[0].rx_buf = data->spi_rd_buf;
		tr->xfer[0].len = (msg_length + SPI_HEADER_SIZE);
		tr->xfer[0].cs_change = 1;
	
		spi_message_add_tail(&tr->xfer[0], &tr->msg);

		ret = spi_sync(spi, &tr->msg);

		if (ret < 0)
			dev_err(&spi->dev, "SPI transactions failed\n");
		
		memcpy((val + data_ptr), &data->spi_rd_buf[SPI_HEADER_SIZE], msg_length);

		data_ptr += msg_length;
		total_byte_count -= msg_length;

		} while (total_byte_count > 0);

	kfree(tr);

	return ret;
}

/*  mxt_read_block - Read block of messages
 *
 *  Perform multiple block transfers
 *  
 *  
*/

static int mxt_read_block (struct mxt_data *data, u16 reg, unsigned int count, u8 *val)

{
	u16 totalBytesToRead = 0;
	int msg_length = 0;
	u16 data_ptr = 0;
	int ret = 0;

	totalBytesToRead = count;

	do {

		reinit_completion(&data->read_completion);
 		
 		if (totalBytesToRead > MXT_MAX_SPI_BLOCK)
 			msg_length = MXT_MAX_SPI_BLOCK;
 		else 
 			msg_length = totalBytesToRead;
 
		ret = mxt_spi_read_req(data, (reg + data_ptr), msg_length);
 
		mxt_wait_for_completion(data, &data->read_completion,
					      MXT_CS_MAX_DELAY);

		ret = mxt_read_data(data, (reg + data_ptr), msg_length, data->spi_rd_buf); 

		memcpy((val + data_ptr), data->spi_rd_buf, msg_length);

		data_ptr += msg_length;
		totalBytesToRead -= msg_length;

	} while (totalBytesToRead > 0);

	return ret;
}

/*  mxt_proc_t100_message - Process T100 messages
 *
 *  Process incoming t100 messages
 *  Send to userspace
 *  
*/

static void mxt_proc_t100_message(struct mxt_data *data, u8 *message)
{
	struct spi_device *spi = data->spi;
	struct input_dev *input_dev = data->input_dev;
	u8 orientation = 0;
	bool hover = false;
	int distance = 0;
	u8 pressure = 0;
	int tool = 0;
	u8 major = 0;
	int id = 0;
	u8 type = 0;
	u8 status;
	u16 x;
	u16 y;
	
	/* Determine id of touch messages only */
	id = (message[0] - data->T100_reportid_min - MXT_RSVD_RPTIDS);

	/* Skip SCRSTATUS events */
	if (id < 0)
		return;

	status = message[1];
	x = get_unaligned_le16(&message[2]);
	y = get_unaligned_le16(&message[4]);
	
	/* Get other auxdata[] bytes if present */
	
	if (status & MXT_T100_DETECT) {
		type = (status & MXT_T100_TYPE_MASK) >> 4;

		switch (type) {
		case MXT_T100_TYPE_HOVERING_FINGER:
			tool = MT_TOOL_FINGER;
			distance = MXT_DISTANCE_HOVERING;
			hover = true;

			if (data->t100_aux_vect)
				orientation = message[data->t100_aux_vect];

			break;

		case MXT_T100_TYPE_FINGER:
		case MXT_T100_TYPE_GLOVE:
			tool = MT_TOOL_FINGER;
			distance = MXT_DISTANCE_ACTIVE_TOUCH;
			hover = false;

			if (data->t100_aux_area)
				major = message[data->t100_aux_area];

			if (data->t100_aux_ampl)
				pressure = message[data->t100_aux_ampl];

			if (data->t100_aux_vect)
				orientation = message[data->t100_aux_vect];

			break;

		case MXT_T100_TYPE_PASSIVE_STYLUS:
			tool = MT_TOOL_PEN;
			distance = MXT_DISTANCE_ACTIVE_TOUCH;
			hover = false;

			/*
			 * Passive stylus is reported with size zero so
			 * hardcode.
			 */
			major = MXT_TOUCH_MAJOR_DEFAULT;

			if (data->t100_aux_ampl)
				pressure = message[data->t100_aux_ampl];

			break;

		case MXT_T100_TYPE_LARGE_TOUCH:
			/* Ignore suppressed touch */
			break;

		default:
			dev_dbg(&spi->dev, "Unexpected T100 type\n");
			return;
		}
	}

	/*
	 * Values reported should be non-zero if tool is touching the
	 * device
	 */
	if (!pressure && !hover)
		pressure = MXT_PRESSURE_DEFAULT;

	input_mt_slot(input_dev, id);

	if (status & MXT_T100_DETECT) {

		dev_dbg(&spi->dev, "id:[%u] type:%u x:%u y:%u a:%02X p:%02X v:%02X\n",
		id, type, x, y, major, pressure, orientation);

  		input_mt_report_slot_state(input_dev, tool, 1);
  		input_report_abs(input_dev, ABS_MT_POSITION_X, x);
  		input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
  		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, major);
  		input_report_abs(input_dev, ABS_MT_PRESSURE, pressure);
  		input_report_abs(input_dev, ABS_MT_DISTANCE, distance);
  		input_report_abs(input_dev, ABS_MT_ORIENTATION, orientation);
	} else {

		dev_dbg(&spi->dev, "[%u] release\n", id);

			/* close out slot */
		input_mt_report_slot_state(input_dev, 0, 0);

	}
	
	data->update_input = true;
}

/*  mxt_proc_t42_messages - Touch suppression msg
 *
 *  Output touch suppression message when received
 *  
*/

static void mxt_proc_t42_messages(struct mxt_data *data, u8 *msg)
{
	struct spi_device *spi = data->spi; 
	u8 status = msg[1];

	if (status & MXT_T42_MSG_TCHSUP)
		dev_info(&spi->dev, "T42 suppress\n");
	else
		dev_info(&spi->dev, "T42 normal\n");
}

/*  mxt_dump_message - Dump debug messages
 *
 *  Capture T5 messages to be dumped using mxt-app
 *  
*/

static void mxt_dump_message(struct mxt_data *data, u8 *message)
{
	struct spi_device *spi = data->spi; 

	dev_dbg(&spi->dev, "message: %*ph\n",
		data->T5_msg_size, message);
}

/*  mxt_input_button - Button reporting
 *
 *  Report button presses if enabled
 *  
*/

static void mxt_input_button(struct mxt_data *data, u8 *message)
{
	struct input_dev *input = data->input_dev;
	int i;

	for (i = 0; i < data->t19_num_keys; i++) {
		if (data->t19_keymap[i] == KEY_RESERVED)
			continue;

		/* Active-low switch */
		input_report_key(input, data->t19_keymap[i],
				 !(message[1] & BIT(i)));
	}
}

/*  mxt_proc_t6_messages - Report T6 messages
 *
 *  Report status messages for T6
 *  
*/

static void mxt_proc_t6_messages(struct mxt_data *data, u8 *msg)
{
	struct spi_device *spi = data->spi; 
	u8 status = msg[1];
	u32 crc = msg[2] | (msg[3] << 8) | (msg[4] << 16);

	if (crc != data->config_crc) {
		data->config_crc = crc;
		dev_dbg(&spi->dev, "T6 Config Checksum: 0x%06X\n", crc);
	}

	complete(&data->crc_completion);

	/* Output debug if status has changed */
	dev_dbg(&spi->dev, "T6 Status 0x%02X%s%s%s%s%s%s%s\n",
		status,
		status == 0 ? " OK" : "",
		status & MXT_T6_STATUS_RESET ? " RESET" : "",
		status & MXT_T6_STATUS_OFL ? " OFL" : "",
		status & MXT_T6_STATUS_SIGERR ? " SIGERR" : "",
		status & MXT_T6_STATUS_CAL ? " CAL" : "",
		status & MXT_T6_STATUS_CFGERR ? " CFGERR" : "",
		status & MXT_T6_STATUS_COMSERR ? " COMSERR" : "");

}

/*  mxt_debug_msg_add - Add debug messages to queue
 *
 *  Adds debug messages to queue read back using mxt-app
 *  
*/

static void mxt_debug_msg_add(struct mxt_data *data, u8 *msg)
{
	struct spi_device *spi = data->spi;

	mutex_lock(&data->debug_msg_lock);

	if (!data->debug_msg_data) {
		dev_err(&spi->dev, "No buffer!\n");
		return;
	}

	if (data->debug_msg_count < DEBUG_MSG_MAX) {
		memcpy(data->debug_msg_data +
		       data->debug_msg_count * data->T5_msg_size,
		       msg,
		       data->T5_msg_size);
		data->debug_msg_count++;
	} else {
		dev_dbg(&spi->dev, "Discarding %u messages\n", data->debug_msg_count);
		data->debug_msg_count = 0;
	}

	mutex_unlock(&data->debug_msg_lock);

	sysfs_notify(&data->spi->dev.kobj, NULL, "debug_notify");
}

/*  mxt_proc_message - Process touch messages
 *
 *  Process and parse touch message in parallel
 *   dump messages into debug buffer.
 *  
*/

static int mxt_proc_message(struct mxt_data *data, u8 *message)
{

	u8 report_id = message[0];
	bool dump = data->debug_enabled;

	if (report_id == MXT_RPTID_NOMSG || report_id == MXT_RPTID_RVSD)
		return 0;

	if (report_id == data->T6_reportid) {
		mxt_proc_t6_messages(data, message);
	} 

	else if (report_id >= data->T42_reportid_min
		   && report_id <= data->T42_reportid_max) {
		mxt_proc_t42_messages(data, message);
	} else if (!data->input_dev) {

		mxt_dump_message(data, message);
	} else if (report_id >= data->T100_reportid_min &&
		   report_id <= data->T100_reportid_max) {
		mxt_proc_t100_message(data, message);
	} else if (report_id == data->T19_reportid_min) {
		mxt_input_button(data, message);
		data->update_input = true;
	} else {
		mxt_dump_message(data, message);
	}

	if (dump)
		mxt_dump_message(data, message);

	if (data->debug_v2_enabled)
		mxt_debug_msg_add(data, message);

	return 1;
}

static void mxt_input_sync(struct mxt_data *data)
{
	
	if (data->update_input) 
		input_sync(data->input_dev);
}

/*
 * mxt_read_and_process_messages - process non-IRQ messages
 *
 *  Used to read and process messages T5 messages
 *   outside of interrupt routine.
 *
*/

static int mxt_read_and_process_messages(struct mxt_data *data, u8 count)
{
	struct spi_device *spi = data->spi;
	int ret, i;
	u8 num_valid = 0;

	/* Safety check for msg_buf */
	if (count > data->max_reportid)
		return -EINVAL;

	data->irq_state = MXT_IRQ_READ_REQ;

	for(i=0; i<count; i++) {
		ret = mxt_spi_transfer(data, data->T5_address, 
			data->T5_msg_size, data->msg_buf); 

		if (ret)
			dev_info(&spi->dev, "Failed to read %u messages (%d)\n", count, ret);

		if (data->msg_buf[0] == 0xFF)
			break;

		ret = mxt_proc_message(data, data->msg_buf);

		if (ret == 1)
			num_valid++;
	}

	return num_valid;
}

/*
 * mxt_process_messages_until_invalid - Process non-IRQ messages
 *
 *  Used to read and process messages
 *   outside of interrupt routine.
 *
*/

static int mxt_process_messages_until_invalid(struct mxt_data *data)
{
	struct spi_device *spi = data->spi;
	int read;
	u8 tries = 1;
	uint8_t count;

	count = data->max_reportid;

	do {
		read = mxt_read_and_process_messages(data, count);

		if (read < count)
			return 0;
	} while (--tries);

	if (data->update_input) {
		mxt_input_sync(data);
		data->update_input = false;
	}

	dev_err(&spi->dev, "CHG pin isn't cleared\n");
	return -EBUSY;
}

/*
 * mxt_request_message - IRQ request message
 *
 *  Request read message triggered from IRQ
 *
*/

static int mxt_request_message(struct mxt_data *data)
{
	struct device *dev = &data->spi->dev;
	int ret;

	ret = mxt_spi_read_req(data, data->T5_address, 
		data->T5_msg_size);

	if (ret) {
		dev_dbg(dev, "Failed to send read request (%d)\n", ret);
		return ret;
	}

	data->irq_state = MXT_IRQ_PROCESS_READ;

	return ret;
}

/*
 * mxt_process_msg_req - IRQ process message
 *
 *  Request read message triggered from IRQ
 *
*/

static int mxt_process_msg_req(struct mxt_data *data)
{
	struct spi_device *spi = data->spi;
	int ret;

	ret = mxt_read_data(data, data->T5_address, 
		data->T5_msg_size, data->msg_buf);

	if (ret) {
		dev_dbg(&spi->dev, "Failed to process T5 message (%d)\n", ret);
		return ret;
	}

	/* Process message after read, one msg per IRQ */
	ret = mxt_proc_message(data, data->msg_buf);
	if (ret < 0) {
		dev_warn(&spi->dev, "Process: Unexpected invalid message\n");
		return ret;
	}

	if (data->update_input) {
		mxt_input_sync(data);
		
		data->update_input = false;
	}

	data->irq_state = MXT_IRQ_READ_DONE;

	return ret;
}

static irqreturn_t mxt_spi_interrupt(int irq, void *dev_id)
{
	struct mxt_data *data = dev_id;
	int error; 

	if (data->in_bootloader) {
		complete(&data->bl_completion);
		return IRQ_HANDLED;
	}

	complete(&data->t6_cmd_completion);
	complete(&data->read_completion);
	complete(&data->write_completion);

	/* Required for SPI controller */
	if (data->system_power_up == false){
		mxt_spi_read(data);
		data->system_power_up = true;
		return IRQ_HANDLED;
	}

	if (!data->object_table)
		return IRQ_HANDLED;

	if (data->irq_processing) {
	switch (data->irq_state)
	{
		case MXT_IRQ_READ_DONE:
		case MXT_IRQ_WRITE_DONE:
			error = mxt_request_message(data);
		break;

		case MXT_IRQ_PROCESS_READ:
			error = mxt_process_msg_req(data);
		break;

		case MXT_IRQ_READ_REQ:
		case MXT_IRQ_WRITE_REQ:
		break;

		default:
		break;
	}
}
	return IRQ_HANDLED;
}

static size_t mxt_obj_size(const struct mxt_object *obj)
{
	return obj->size_minus_one + 1;
}

static size_t mxt_obj_instances(const struct mxt_object *obj)
{
	return obj->instances_minus_one + 1;
}	

/*  mxt_object_readable 
 *  
 *  List of readable register when using object 
 *   attribute to dump content. List must be less
 *   then PAGE_SIZE
 *   
*/
static bool mxt_object_readable(unsigned int type)
{
	switch (type) {
	case MXT_GEN_COMMAND_T6:
	case MXT_GEN_POWER_T7:
	case MXT_GEN_ACQUIRE_T8:
	case MXT_PROCI_KEYTHRESHOLD_T14:
	case MXT_TOUCH_KEYARRAY_T15:
	case MXT_SPT_COMMSCONFIG_T18:
	case MXT_SPT_GPIOPWM_T19:
	case MXT_PROCI_ONETOUCH_T24:
	case MXT_SPT_SELFTEST_T25:
	case MXT_PROCI_TWOTOUCH_T27:
	case MXT_PROCI_GRIP_T40:
	case MXT_PROCI_TOUCHSUPPRESSION_T42:
	case MXT_PROCI_STYLUS_T47:
	case MXT_SPT_USERDATA_T38:
	case MXT_SPT_DIGITIZER_T43:
	case MXT_SPT_CTECONFIG_T46:
	case MXT_PROCI_SHIELDLESS_T56:
	case MXT_SPT_TIMER_T61:
	case MXT_PROCI_LENSBENDING_T65:
	case MXT_SPT_DYNAMICCONFIGURATIONCONTROLLER_T70:
	case MXT_SPT_DYNAMICCONFIGURATIONCONTAINER_T71:
	case MXT_NOISESUPPRESSION_T72:
	case MXT_PROCI_GLOVEDETECTION_T78:
	case MXT_SPT_TOUCHEVENTTRIGGER_T79:
	case MXT_PROCI_RETRANSMISSIONCOMPENSATION_T80:
	case MXT_TOUCH_MULTITOUCHSCREEN_T100:
		return true;
	default:
		return false;
	}
}

static void mxt_free_object_table(struct mxt_data *data)
{

	data->object_table = NULL;
	data->info = NULL;
	kfree(data->raw_info_block);
	data->raw_info_block = NULL;
	kfree(data->msg_buf);
	data->msg_buf = NULL;
	data->T5_address = 0;
	data->T5_msg_size = 0;
	data->T6_reportid = 0;
	data->T7_address = 0;
	data->T71_address = 0;
	data->T15_reportid_min = 0;
	data->T15_reportid_max = 0;
	data->T18_address = 0;
	data->T19_reportid_min = 0;
	data->T42_reportid_min = 0;
	data->T42_reportid_max = 0;
	data->T44_address = 0;
	data->T100_reportid_min = 0;
	data->T100_reportid_max = 0;
	data->max_reportid = 0;
}

static int mxt_parse_object_table(struct mxt_data *data,
				  struct mxt_object *object_table)
{

	struct device *dev = &data->spi->dev; 
	int i;
	u8 reportid;
	u16 end_address;

	/* Valid Report IDs start counting from 1 */
	reportid = 1;
	data->mem_size = 0;
	for (i = 0; i < data->info->object_num; i++) {
		struct mxt_object *object = object_table + i;
		u8 min_id, max_id;

		le16_to_cpus(&object->start_address);

		if (object->num_report_ids) {
			min_id = reportid;
			reportid += object->num_report_ids *
					mxt_obj_instances(object);
			max_id = reportid - 1;
		} else {
			min_id = 0;
			max_id = 0;
		}

		dev_dbg(dev,
			"T%u Start:%u Size:%zu Instances:%zu Report IDs:%u-%u\n",
			object->type, object->start_address,
			mxt_obj_size(object), mxt_obj_instances(object),
			min_id, max_id);

		switch (object->type) {
		case MXT_GEN_MESSAGE_T5:

				/* CRC not enabled, so skip last byte */
			data->T5_msg_size = mxt_obj_size(object) - 1;
			data->T5_address = object->start_address;
			break;
		case MXT_GEN_COMMAND_T6:
			data->T6_reportid = min_id;
			data->T6_address = object->start_address;
			break;
		case MXT_GEN_POWER_T7:
			data->T7_address = object->start_address;
			break;
		case MXT_PROCI_KEYTHRESHOLD_T14:
			data->T14_address = object->start_address;
			break;
		case MXT_SPT_USERDATA_T38:
			data->T38_address = object->start_address;
			break;
		case MXT_SPT_DYNAMICCONFIGURATIONCONTAINER_T71:
			data->T71_address = object->start_address;
			break;
		case MXT_TOUCH_KEYARRAY_T15:
			data->T15_reportid_min = min_id;
			data->T15_reportid_max = max_id;
			break;
		case MXT_SPT_COMMSCONFIG_T18:
			data->T18_address = object->start_address;
			break;
		case MXT_PROCI_TOUCHSUPPRESSION_T42:
			data->T42_reportid_min = min_id;
			data->T42_reportid_max = max_id;
			break;
		case MXT_SPT_MESSAGECOUNT_T44:
			data->T44_address = object->start_address;
			break;
		case MXT_SPT_GPIOPWM_T19:
			data->T19_reportid = min_id;
			break;
		case MXT_TOUCH_MULTITOUCHSCREEN_T100:
			data->multitouch = MXT_TOUCH_MULTITOUCHSCREEN_T100;
			data->T100_reportid_min = min_id;
			data->T100_reportid_max = max_id;
			/* first two report IDs reserved */
			data->num_touchids = object->num_report_ids - 2;
			break;
		}

		end_address = object->start_address
			+ mxt_obj_size(object) * mxt_obj_instances(object) - 1;

		if (end_address >= data->mem_size)
			data->mem_size = end_address + 1;
	}

	/* Store maximum reportid */
	data->max_reportid = reportid;

	/* If T44 exists, T5 position has to be directly after */
	if (data->T44_address && (data->T5_address != data->T44_address + 1)) {
		dev_err(dev, "Invalid T44 position\n");
		return -EINVAL;
	}	

	data->msg_buf = kcalloc(data->max_reportid,
				(data->T5_msg_size), GFP_KERNEL);
	if (!data->msg_buf)
		return -ENOMEM;

	return 0;
}

static void mxt_calc_crc24(u32 *crc, u8 firstbyte, u8 secondbyte)
{
	static const unsigned int crcpoly = 0x80001B;
	u32 result;
	u32 data_word;

	data_word = (secondbyte << 8) | firstbyte;
	result = ((*crc << 1) ^ data_word);

	if (result & 0x1000000)
		result ^= crcpoly;

	*crc = result;
}

static u32 mxt_calculate_crc(u8 *base, off_t start_off, off_t end_off)
{
	u32 crc = 0;
	u8 *ptr = base + start_off;
	u8 *last_val = base + end_off - 1;

	if (end_off < start_off)
		return -EINVAL;

	while (ptr < last_val) {
		mxt_calc_crc24(&crc, *ptr, *(ptr + 1));
		ptr += 2;
	}

	/* if len is odd, fill the last byte with 0 */
	if (ptr == last_val)
		mxt_calc_crc24(&crc, *ptr, 0);

	/* Mask to 24-bit */
	crc &= 0x00FFFFFF;

	return crc;
}

/*  mxt_read_info_block 
 *
 *  Reads infoblock and calculates checksum
 *   Read is done during probe. SPI transfers
 *   ignore IRQ behavior.
 *
*/
  
static int mxt_read_info_block(struct mxt_data *data)
{
	struct spi_device *spi = data->spi;
	int error;
	size_t size;
	void *id_buf, *buf;
	uint8_t num_objects;
	u32 calculated_crc;
	u8 *crc_ptr;

	/* If info block already allocated, free it */
	if (data->raw_info_block)
		mxt_free_object_table(data);

	/* Read 7-byte ID information block starting at address 0 */
	size = sizeof(struct mxt_info);

	id_buf = kzalloc(size, GFP_KERNEL);
	if (!id_buf)
		return -ENOMEM;

	error = mxt_spi_transfer(data, 0, size, id_buf);

	if (error < 0) {
		dev_info(&spi->dev, "Error: SPI transfer did not complete or failed\n");
		goto err_free_mem;
	}

	/* Resize buffer to give space for rest of info block */
	num_objects = ((struct mxt_info *)id_buf)->object_num;

	size += (num_objects * sizeof(struct mxt_object))
		+ MXT_INFO_CHECKSUM_SIZE;

	buf = krealloc(id_buf, size, GFP_KERNEL);
	
	if (!buf) {
		error = -ENOMEM;
		goto err_free_mem;
	}

	id_buf = buf;

	/* Read rest of info block */
	error = mxt_spi_transfer(data, MXT_OBJECT_START, 
				(size - MXT_OBJECT_START), 
				(id_buf + MXT_OBJECT_START));
	if (error < 0) {
		dev_info(&spi->dev, "Error: Read of infoblock failed\n");
		goto err_free_mem;
	}

	/* Extract & calculate checksum */
	crc_ptr = id_buf + size - MXT_INFO_CHECKSUM_SIZE;
	data->info_crc = crc_ptr[0] | (crc_ptr[1] << 8) | (crc_ptr[2] << 16);
  	
	calculated_crc = mxt_calculate_crc(id_buf, 0,
					   size - MXT_INFO_CHECKSUM_SIZE);

	/*
	 * CRC mismatch can be caused by data corruption issue or 
	 * else device is not using Object Based Protocol (eg i2c-hid)
	 */
	if ((data->info_crc == 0) || (data->info_crc != calculated_crc)) {
		dev_err(&spi->dev,
			"Info Block CRC error calculated=0x%06X read=0x%06X\n",
			calculated_crc, data->info_crc);
		error = -EIO;
		goto err_free_mem;
	}

	data->raw_info_block = id_buf;
	data->info = (struct mxt_info *)id_buf;

	dev_info(&spi->dev,
		 "Family: %u Variant: %u Firmware V%u.%u.%02X Objects: %u\n",
		 data->info->family_id, data->info->variant_id,
		 data->info->version >> 4, data->info->version & 0xf,
		 data->info->build, data->info->object_num);

	/* Parse object table information */
	error = mxt_parse_object_table(data, id_buf + MXT_OBJECT_START);
	if (error) {
		dev_err(&spi->dev, "Error %d parsing object table\n", error);
		mxt_free_object_table(data);
		goto err_free_mem;
	}

	data->object_table = (struct mxt_object *)(id_buf + MXT_OBJECT_START);

	return 0;

err_free_mem:
	kfree(id_buf);
	return error;
}

/*  mxt_check_retrigen
 *  
 *  Enables debug RETRIGEN bit to detect and resolve
 *   missed IRQ edge
*/

static int mxt_check_retrigen(struct mxt_data *data)
{
	struct spi_device *spi = data->spi;
	int error;
	uint8_t val;

	if (data->T18_address) {

		error = mxt_spi_transfer(data, 
			data->T18_address + MXT_COMMS_CTRL,
			1, &val);
		
		if (error) {
			dev_info(&spi->dev, "Error: Read T18 register failed\n");
			return error;
		}

		if (val & MXT_COMMS_RETRIGEN) {
			dev_info(&spi->dev, "RETRIGEN Enabled\n");
			return 0;
		}

		dev_info(&spi->dev, "Enabling RETRIGEN feature\n");

		val |= MXT_COMMS_RETRIGEN;

		error = mxt_spi_write_xfer(data, 
			data->T18_address + MXT_COMMS_CTRL,
			1, &val);

		if (error) {
			dev_info(&spi->dev, "Error: Failed to enable retrig\n");
	   		return error;
		} else {
			dev_info(&spi->dev, "RETRIGEN enabled\n");
		}
	} else {
		dev_info(&spi->dev, "RETRIGEN feature not available\n");
	}

	return 0;
}

/*  mxt_set_t7_power_cfg
 *  
 *  Sets the T7 active and idle acquistion values
 *   Ignores IRQ behavior
*/

static int mxt_set_t7_power_cfg(struct mxt_data *data, u8 sleep)
{
	struct device *dev = &data->spi->dev;
	struct t7_config *new_config;
	struct t7_config deepsleep = { .active = 0, .idle = 0 };
	int error;

	if (sleep == MXT_POWER_CFG_DEEPSLEEP)
		new_config = &deepsleep;
	else
		new_config = &data->t7_cfg;

	data->irq_state = MXT_IRQ_READ_REQ;

	error = mxt_spi_write_xfer(data, data->T7_address, 
		sizeof(data->t7_cfg), new_config);

	data->irq_state = MXT_IRQ_READ_DONE;

	msleep(2);

	if (error)
		return error;

	dev_info(dev, "Set T7 ACTV:%d IDLE:%d\n",	
		new_config->active, new_config->idle);

	return 0;
}

static struct mxt_object *mxt_get_object(struct mxt_data *data, u8 type)
{
	struct device *dev = &data->spi->dev;
	struct mxt_object *object;
	int i;

	for (i = 0; i < data->info->object_num; i++) {
		object = data->object_table + i;
		if (object->type == type)
			return object;
	}

	dev_warn(dev, "Invalid object type T%u\n", type);
	return NULL;
}

/*  mxt_prepare_cfg_mem
 *  
 *  Writes configuration block memory from buffer 
 *  
 *
*/

static int mxt_prepare_cfg_mem (struct mxt_data *data, struct mxt_cfg *cfg)
{
	struct device *dev = &data->spi->dev;
	struct mxt_object *object;
	unsigned int type, instance, size, byte_offset = 0;
	int offset, write_offset = 0;
	int ret, i, error;
	u16 reg;
	u8 val;

	while (cfg->raw_pos < cfg->raw_size) {
		/* Read type, instance, length */
		ret = sscanf(cfg->raw + cfg->raw_pos, "%x %x %x%n",
			     &type, &instance, &size, &offset);
		if (ret == 0) {
			/* EOF */
			break;
		} else if (ret != 3) {
			dev_err(dev, "Bad format: failed to parse object\n");
			return -EINVAL;
		}
		cfg->raw_pos += offset;

		object = mxt_get_object(data, type);
		if (!object) {
			/* Skip object */
			for (i = 0; i < size; i++) {
				ret = sscanf(cfg->raw + cfg->raw_pos, "%hhx%n",
					     &val, &offset);
				if (ret != 1) {
					dev_err(dev, "Bad format in T%d at %d\n",
						type, i);
					return -EINVAL;
				}
				cfg->raw_pos += offset;
			}
			continue;
		}

		if (size > mxt_obj_size(object)) {
			/*
			 * Either we are in fallback mode due to wrong
			 * config or config from a later fw version,
			 * or the file is corrupt or hand-edited.
			 */
			dev_warn(dev, "Discarding %zu byte(s) in T%u\n",
				 size - mxt_obj_size(object), type);
		} else if (mxt_obj_size(object) > size) {
			/*
			 * If firmware is upgraded, new bytes may be added to
			 * end of objects. It is generally forward compatible
			 * to zero these bytes - previous behaviour will be
			 * retained. However this does invalidate the CRC and
			 * will force fallback mode until the configuration is
			 * updated. We warn here but do nothing else - the
			 * malloc has zeroed the entire configuration.
			 */
			dev_warn(dev, "Zeroing %zu byte(s) in T%d\n",
				 mxt_obj_size(object) - size, type);
		}

		if (instance >= mxt_obj_instances(object)) {
			dev_err(dev, "Object instances exceeded!\n");
			return -EINVAL;
		}

		reg = object->start_address + mxt_obj_size(object) * instance;

		for (i = 0; i < size; i++) {
			ret = sscanf(cfg->raw + cfg->raw_pos, "%hhx%n",
				     &val,
				     &offset);
			if (ret != 1) {
				dev_err(dev, "Bad format in T%d at %d\n",
					type, i);
				return -EINVAL;
			}
			cfg->raw_pos += offset;

			if (i > mxt_obj_size(object))
				continue;

			byte_offset = reg + i - cfg->start_ofs;

			/* Data starts only at selected object address */
			if (reg >= cfg->start_ofs) {
				*(cfg->mem + byte_offset) = val;
			} else {
				dev_dbg(dev, "Skipping object: reg:%d, T%d, ofs=%d\n",
					reg, object->type, byte_offset);
			}
		}

		if (reg >= cfg->start_ofs) {

			data->irq_state = MXT_IRQ_WRITE_REQ;
			error = mxt_write_block (data, reg, size, (cfg->mem + write_offset));
			write_offset = write_offset + size;
		}
	}

	return 0;
}

/*  mxt_t6_command
 *  
 *  Sends T6 command to device
 *   If wait is true, command byte is checked until cleared 
 *  
*/

static int mxt_t6_command(struct mxt_data *data, u16 cmd_offset,
			  u8 value, bool wait)
{
	
	struct spi_device *spi = data->spi;
	u8 command_register;
	u16 reg;
	int ret;

	reinit_completion(&data->write_completion);

	reg = data->T6_address + cmd_offset;

	ret = mxt_spi_write_req(data, reg, 1, &value);

	mxt_wait_for_completion(data, &data->write_completion,
					      MXT_CS_MAX_DELAY);

	ret = mxt_spi_write_process(data);

	if (ret) {
		return ret;
	}

	/* Wait for REPORTALL bit to be cleared */
	if (!wait) {
		return 0;
	}

	reinit_completion(&data->t6_cmd_completion);

	/* Took min of 30ms for /CHG line to drop */
	/* Processing report_all command */
	mxt_wait_for_completion(data, &data->t6_cmd_completion, 
		MXT_REPORTALL_MAX_DELAY);

	reinit_completion(&data->read_completion);

	ret = mxt_spi_read_req(data, reg, 1);

	mxt_wait_for_completion(data, &data->read_completion,
					      MXT_CS_MAX_DELAY);

	ret = mxt_read_data(data, reg, 1, &command_register);

	if (ret)
		return ret;

	if (command_register != 0x00)
		dev_warn(&data->spi->dev, "T6 Command failed\n");
	else
		dev_info(&spi->dev, "T6 Command Succeeded\n");

	ret = mxt_process_messages_until_invalid(data);

	data->irq_state = MXT_IRQ_READ_DONE;

	return 0;
}

static int mxt_send_flash_command(struct mxt_data *data)
{
	struct spi_device *spi = data->spi;
	u16 cmd_offset = MXT_COMMAND_RESET;
	u8 command;
	u16 reg;
	int ret;

	reg = data->T6_address + cmd_offset;
	command = MXT_FLASH_CMD;

	ret = mxt_spi_write_xfer(data, reg, 1, &command); 

	if (ret) {
		dev_dbg(&spi->dev, "Failed to write flash command\n");
		return ret;
	}

	return 0;
}

static int mxt_acquire_irq(struct mxt_data *data)
{

	struct spi_device *spi = data->spi;
	int error;

	error = mxt_process_messages_until_invalid(data);

	data->irq_state = MXT_IRQ_READ_DONE;

	dev_info(&spi->dev, "Eabling interrupt\n");

	enable_irq(spi->irq);

	return 0;
}

static int mxt_soft_reset(struct mxt_data *data, bool reset_enabled) {

	struct device *dev = &data->spi->dev;
	int ret = 0;

	dev_info(dev, "Resetting device\n");

	ret = mxt_t6_command(data, MXT_COMMAND_RESET, MXT_RESET_VALUE, false);
	if (ret)
		return ret;

	/* Ignore CHG line after reset */
	msleep(MXT_RESET_INVALID_CHG);

	data->irq_state = MXT_IRQ_WRITE_DONE;

	if (ret)
		return ret;

	return 0;
}

static int mxt_init_t7_power_cfg(struct mxt_data *data)
{
	struct spi_device *spi = data->spi;
	int error;
	bool retry = false;

recheck:
	error = mxt_spi_transfer(data, data->T7_address, 
		sizeof(data->t7_cfg), &data->t7_cfg);

	if (error)
		return error;

	if (data->t7_cfg.active == 0 || data->t7_cfg.idle == 0) {
		if (!retry) {
			dev_info(&spi->dev, "T7 cfg zero, resetting\n");
			retry = true;
			goto recheck;
		} else {
			dev_info(&spi->dev, "T7 cfg zero after reset, overriding\n");
			data->t7_cfg.active = 20;
			data->t7_cfg.idle = 100;
			return mxt_set_t7_power_cfg(data, MXT_POWER_CFG_RUN);
		}
	}

	dev_info(&spi->dev, "Initialized power cfg: ACTV %d, IDLE %d\n",
		data->t7_cfg.active, data->t7_cfg.idle);
	return 0;
}

static int mxt_read_t100_config(struct mxt_data *data, u8 instance)
{
	struct spi_device *spi = data->spi;
	int error;
	struct mxt_object *object;
	u16 range_x, range_y;
	u8 cfg, tchaux;
	u8 aux;
	u16 obj_size = 0;

	object = mxt_get_object(data, MXT_TOUCH_MULTITOUCHSCREEN_T100);
	if (!object)
		return -EINVAL;
	
	/* read touchscreen dimensions */

	error = mxt_spi_transfer(data, 
		object->start_address + obj_size + MXT_T100_XRANGE, 
		sizeof(range_x), &range_x);

	if (error)
		return error;

	data->max_x = get_unaligned_le16(&range_x);

	error = mxt_spi_transfer(data, 
		object->start_address + obj_size + MXT_T100_YRANGE,
			       sizeof(range_y), &range_y);

	if (error)
		return error;

	data->max_y = get_unaligned_le16(&range_y);			       

	error = mxt_spi_transfer(data, 
		object->start_address + obj_size + MXT_T100_XSIZE,
		sizeof(data->xsize), &data->xsize);

	if (error)
		return error;

	error = mxt_spi_transfer(data,
		object->start_address + obj_size + MXT_T100_YSIZE, 
		sizeof(data->ysize), &data->ysize);

	if (error)
		return error;

	/* read orientation config */
	error = mxt_spi_transfer(data,
		object->start_address + obj_size + MXT_T100_CFG1,
		1, &cfg);
	
	if (error)
		return error;

	data->xy_switch = cfg & MXT_T100_CFG_SWITCHXY;
	data->invertx = cfg & MXT_T100_CFG_INVERTX;
	data->inverty = cfg & MXT_T100_CFG_INVERTY;

	/* allocate aux bytes */
	error = mxt_spi_transfer(data,
		object->start_address + obj_size+ MXT_T100_TCHAUX,
		1, &tchaux);	

	if (error)
		return error;

	aux = MXT_T100_AUX_OFFSET;

	if (tchaux & MXT_T100_TCHAUX_VECT)
		data->t100_aux_vect = aux++;

	if (tchaux & MXT_T100_TCHAUX_AMPL)
		data->t100_aux_ampl = aux++;

	if (tchaux & MXT_T100_TCHAUX_AREA)
		data->t100_aux_area = aux++;

	dev_dbg(&spi->dev,
		"T100 aux mappings vect:%u ampl:%u area:%u\n",
		data->t100_aux_vect, data->t100_aux_ampl, data->t100_aux_area);
				
	return 0;
}

static int mxt_input_open(struct input_dev *dev);
static void mxt_input_close(struct input_dev *dev);

static int mxt_initialize_input_devices(struct mxt_data *data)
{
	struct spi_device *spi = data->spi;
	struct input_dev *input_dev;
	int error;
	unsigned int num_mt_slots;
	unsigned int mt_flags = 0;
	int i;

	switch (data->multitouch) {
		case MXT_TOUCH_MULTITOUCHSCREEN_T100:
		num_mt_slots = (data->num_touchids);
		error = mxt_read_t100_config(data, 1);
		if (error)
			dev_warn(&spi->dev, "Failed to read T100 config\n");
		break;

	default:
		dev_err(&spi->dev, "Invalid multitouch object\n");
		return -EINVAL;
	}

	/* Handle default values and orientation switch */
	if (data->max_x == 0)
		data->max_x = 1023;

	if (data->max_y == 0)
		data->max_y = 1023;

	if (data->xy_switch)
		swap(data->max_x, data->max_y);

	dev_info(&spi->dev, "Touchscreen size {x,y} = {%u,%u}\n", data->max_x, data->max_y);

	/* Register input device */
	input_dev = input_allocate_device();
	if (!input_dev)
		return -ENOMEM;

	input_dev->name = "SPI maXTouch Touchscreen";
	input_dev->phys = data->phys;
	input_dev->id.bustype = BUS_SPI;
	input_dev->dev.parent = &spi->dev;
	input_dev->open = mxt_input_open;
	input_dev->close = mxt_input_close;

	input_set_capability(input_dev, EV_KEY, BTN_TOUCH);

	/* For single touch */
	input_set_abs_params(input_dev, ABS_X, 0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, data->max_y, 0, 0);

	if (data->multitouch == MXT_TOUCH_MULTITOUCHSCREEN_T100 &&
	     data->t100_aux_ampl) {
		input_set_abs_params(input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	}

	mt_flags |= INPUT_MT_DIRECT;

	/* For multi touch */
	error = input_mt_init_slots(input_dev, num_mt_slots, mt_flags);
	if (error) {
		dev_err(&spi->dev, "Error %d initialising slots\n", error);
		goto err_free_mem;
	}

	if (data->multitouch == MXT_TOUCH_MULTITOUCHSCREEN_T100) {
		input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE,
				     0, MT_TOOL_MAX, 0, 0);
		input_set_abs_params(input_dev, ABS_MT_DISTANCE,
				     MXT_DISTANCE_ACTIVE_TOUCH,
				     MXT_DISTANCE_HOVERING,
				     0, 0);
	}

	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, data->max_y, 0, 0);

	/* For T15 Key Array */
	if (data->T15_reportid_min) {
		data->t15_keystatus = 0;

		for (i = 0; i < data->t15_num_keys; i++)
			input_set_capability(input_dev, EV_KEY,
					data->t15_keymap[i]);
	}

	input_set_drvdata(input_dev, data);

	error = input_register_device(input_dev);
	if (error) {
		dev_err(&spi->dev, "Error %d registering input device\n", error);
		goto err_free_mem;
	}

	data->input_dev = input_dev;

	return 0;

err_free_mem:
	input_free_device(input_dev);
	return error;
}

static int mxt_bootloader_read(struct mxt_data *data,
			       u8 *val, unsigned int count)
{
	struct request * tr;
	struct spi_device *spi = data->spi;
	u8 tx_buf[8];
	u8 rx_buf[8];
	int ret;

	memset(tx_buf, 0x00, sizeof(tx_buf));

	tx_buf[0] = MXT_BOOTLOADER_READ;

	tr = kzalloc(sizeof *tr, GFP_KERNEL);
	if (!tr)
		return -ENOMEM;

	spi_message_init(&tr->msg);
	tr->msg.spi = spi;

	tr->xfer[0].tx_buf = tx_buf;
	tr->xfer[0].rx_buf = rx_buf;
	tr->xfer[0].len = count + 2; //includes 2 byte header
	tr->xfer[0].cs_change = 1;

	spi_message_add_tail(&tr->xfer[0], &tr->msg);

	ret = spi_sync(spi, &tr->msg);

	if (ret < 0)
		dev_err(&spi->dev, "SPI bootloader read failed\n");

	memcpy(val, rx_buf + 2, count);  //Return val, skip header

	kfree(tr);

	return ret;
}

static int mxt_probe_bootloader(struct mxt_data *data)
{
	struct device *dev = &data->spi->dev;
	bool crc_failure;
	int error;
	u8 val;
	
	error = mxt_bootloader_read(data, &val, 1);
	if (error)
		return error;

	/* Check app crc fail mode */
	crc_failure = (val & ~MXT_BOOT_STATUS_MASK) == MXT_APP_CRC_FAIL;

	dev_err(dev, "Detected bootloader, status:%02X%s\n",
			val, crc_failure ? ", APP_CRC_FAIL" : "");

	return 0;
}

static int mxt_bootloader_write(struct mxt_data *data,
				const u8 * const val, unsigned int count)
{
	struct spi_device *spi = data->spi;
	struct request *tr;
	u8 *tx_buf;
	u8 *rx_buf;
	int ret;

	tr = kzalloc(sizeof *tr, GFP_KERNEL);
	if (!tr)
		return -ENOMEM;

	tx_buf = kzalloc(count + 2, GFP_KERNEL); //Include write header, 2 bytes
	if (!tx_buf)
		return -ENOMEM;

	memcpy(&tx_buf[2], val, count);

	// zero rx_buf
	rx_buf = kzalloc(count + 2, GFP_KERNEL);
	if (!rx_buf)
		return -ENOMEM;

	tx_buf[0] = 0x00;	//Write header 0x00, 0x00
	tx_buf[1] = 0x00;

	spi_message_init(&tr->msg);
	tr->msg.spi = spi;

	tr->xfer[0].tx_buf = tx_buf;
	tr->xfer[0].rx_buf = rx_buf;
	tr->xfer[0].len = count + 2;
	tr->xfer[0].cs_change = 1;

	spi_message_add_tail(&tr->xfer[0], &tr->msg);

	ret = spi_sync(spi, &tr->msg);

	reinit_completion(&data->bl_completion);

	if (ret < 0)
		dev_err(&spi->dev, "SPI bootloader write failed\n");

	return ret;
}

static int mxt_send_bootloader_cmd(struct mxt_data *data, bool unlock)
{
	u8 buf[2];
	int ret;

	if (unlock) {
		buf[0] = MXT_UNLOCK_CMD_LSB;
		buf[1] = MXT_UNLOCK_CMD_MSB;
	} else {
		buf[0] = 0x01;
		buf[1] = 0x01;
	}

	ret = mxt_bootloader_write(data, buf, 2);
	if (ret)
		return ret;

	return 0;
}

static int mxt_initialize(struct mxt_data *data)
{
	struct spi_device *spi = data->spi;
	int recovery_attempts = 0;
	int error;

	while (1) {
		error = mxt_read_info_block(data);
		if (!error)
			break;

		error = mxt_probe_bootloader(data);
		if (error) {
			dev_info(&spi->dev, "Chip is not in appmode or bootloader mode\n");
		}

		if (++recovery_attempts > 1) {
			dev_err(&spi->dev, "Could not recover from bootloader mode\n");
			/*
			* We can reflash from ths state, so do not
			* abort initialization
			*/
			data->in_bootloader = true;
			return 0;
		}

		/* Attempt to exit bootloader into app mode */
			mxt_send_bootloader_cmd(data, false);
			msleep(MXT_FW_RESET_TIME);
	}	

	error = mxt_init_t7_power_cfg(data);

	if (error) {
		dev_err(&spi->dev, "Failed to initialize power cfg\n");
		return error;
	}
	
	if (data->multitouch) {
		dev_info(&spi->dev, "mxt_init: Registering devices");
		error = mxt_initialize_input_devices(data);

	} else {
		dev_err(&spi->dev, "No touch object detected\n");
	}

	error = mxt_acquire_irq(data);

	return 0;
}

static void mxt_update_crc(struct mxt_data *data, u8 cmd, u8 value)
{
	/*
	 * On failure, CRC is set to 0 and config will always be
	 * downloaded.
	 */
	bool wait = false;

	data->config_crc = 0;

	if (cmd == MXT_COMMAND_REPORTALL)
		wait = true;

	mxt_t6_command(data, cmd, value, wait);	
}

/*
 * mxt_update_cfg - download configuration to chip
 *
 * Atmel Raw Config File Format
 *
 * The first four lines of the raw config file contain:
 *  1) Version
 *  2) Chip ID Information (first 7 bytes of device memory)
 *  3) Chip Information Block 24-bit CRC Checksum
 *  4) Chip Configuration 24-bit CRC Checksum
 *
 * The rest of the file consists of one line per object instance:
 *   <TYPE> <INSTANCE> <SIZE> <CONTENTS>
 *
 *   <TYPE> - 2-byte object type as hex
 *   <INSTANCE> - 2-byte object instance number as hex
 *   <SIZE> - 2-byte object size as hex
 *   <CONTENTS> - array of <SIZE> 1-byte hex values
 */

static int mxt_update_cfg(struct mxt_data *data, const struct firmware *fw)
{
	struct device *dev = &data->spi->dev;
	u32 info_crc, config_crc, calculated_crc;
	struct mxt_cfg cfg;
	u16 crc_start = 0;
	int ret, error;
	int offset;
	int i;

	/* Make zero terminated copy of the OBP_RAW file */
	cfg.raw = kmemdup_nul(fw->data, fw->size, GFP_KERNEL);
	if (!cfg.raw)
		return -ENOMEM;

	cfg.raw_size = fw->size;

	mxt_update_crc(data, MXT_COMMAND_REPORTALL, 1);

	error = mxt_process_messages_until_invalid(data);
	if (error)
		dev_dbg(dev, "Unable to read CRC\n");

	if (strncmp(cfg.raw, MXT_CFG_MAGIC, strlen(MXT_CFG_MAGIC))) {
		dev_err(dev, "Unrecognised config file\n");
		ret = -EINVAL;
		goto release_raw;
	}

	cfg.raw_pos = strlen(MXT_CFG_MAGIC);

	/* Load information block and check */
	for (i = 0; i < sizeof(struct mxt_info); i++) {
		ret = sscanf(cfg.raw + cfg.raw_pos, "%hhx%n",
			     (unsigned char *)&cfg.info + i,
			     &offset);
		if (ret != 1) {
			dev_err(dev, "Bad format\n");
			ret = -EINVAL;
			goto release_raw;
		}

		cfg.raw_pos += offset;
	}

	if (cfg.info.family_id != data->info->family_id) {
		dev_err(dev, "Family ID mismatch!\n");
		ret = -EINVAL;
		goto release_raw;
	}

	if (cfg.info.variant_id != data->info->variant_id) {
		dev_err(dev, "Variant ID mismatch!\n");
		ret = -EINVAL;
		goto release_raw;
	}

	/* Read CRCs */
	ret = sscanf(cfg.raw + cfg.raw_pos, "%x%n", &info_crc, &offset);
	if (ret != 1) {
		dev_err(dev, "Bad format: failed to parse Info CRC\n");
		ret = -EINVAL;
		goto release_raw;
	}
	cfg.raw_pos += offset;

	ret = sscanf(cfg.raw + cfg.raw_pos, "%x%n", &config_crc, &offset);
	if (ret != 1) {
		dev_err(dev, "Bad format: failed to parse Config CRC\n");
		ret = -EINVAL;
		goto release_raw;
	}
	cfg.raw_pos += offset;

	/*
	 * The Info Block CRC is calculated over mxt_info and the object
	 * table. If it does not match then we are trying to load the
	 * configuration from a different chip or firmware version, so
	 * the configuration CRC is invalid anyway.
	 */
	if (info_crc == data->info_crc) {
		if (config_crc == 0 || data->config_crc == 0) {
			dev_info(dev, "CRC zero, attempting to apply config\n");
		} else if (config_crc == data->config_crc) {
			dev_info(dev, "Config CRC 0x%06X: OK. No update required.\n",
				 data->config_crc);
			ret = 0;
			goto release_raw;
		} else {
			dev_info(dev, "Config CRC 0x%06X: does not match file 0x%06X\n",
				 data->config_crc, config_crc);
		}
	} else {
		dev_warn(dev,
			 "Warning: Info CRC error - device=0x%06X file=0x%06X\nFailed Config Programming\n",
			 data->info_crc, info_crc);
		goto release_raw; 
	}

	/* Stop T70 Dynamic Configuration before calculation of CRC */

	mxt_update_crc(data, MXT_COMMAND_BACKUPNV, MXT_BACKUP_W_STOP);

	msleep(5);

	cfg.start_ofs = data->T38_address;

	cfg.mem_size = data->mem_size - cfg.start_ofs;

	cfg.mem = kzalloc(cfg.mem_size, GFP_KERNEL);
	if (!cfg.mem) {
		ret = -ENOMEM;
		goto release_mem;
	}

	/* Prepares and programs configuration */
	ret = mxt_prepare_cfg_mem(data, &cfg);
	if (ret)
		goto release_mem;

	/* Calculate CRC of config held in cfg.mem buffer */

	crc_start = data->T38_address;

	dev_dbg(dev, "cfg.mem_size %i, cfg.start_ofs %i, cfg.raw_pos %lld, offset %i", 
	cfg.mem_size, cfg.start_ofs, (long long)cfg.raw_pos, offset);

	if (crc_start > cfg.start_ofs) {
		calculated_crc = mxt_calculate_crc(cfg.mem, 0, cfg.mem_size);

		if (config_crc > 0 && config_crc != calculated_crc)
			dev_info(dev, "Config CRC in file inconsistent, calculated=%06X, file=%06X\n",
				 calculated_crc, config_crc);
	}

	msleep(5);	//Allow delay before issuing backup and reset

	mxt_update_crc(data, MXT_COMMAND_BACKUPNV, MXT_BACKUP_VALUE);

	msleep(50);	//Allow 50ms before issuing reset

	mxt_soft_reset(data, true);

	dev_info(dev, "Config successfully updated\n");

	/* T7 config may have changed */
	mxt_init_t7_power_cfg(data);

release_mem:
	kfree(cfg.mem);
release_raw:
	kfree(cfg.raw);

	return ret;
}

static int mxt_clear_cfg(struct mxt_data *data)
{
	struct device *dev = &data->spi->dev;
	struct mxt_cfg config;
	int totalBytesToWrite = 0;
	int write_offset = 0;
	int msg_size = 0;
	int error;
	
	//Start at beginning of config space

	config.start_ofs = MXT_OBJECT_START +
			data->info->object_num * sizeof(struct mxt_object) +
			MXT_INFO_CHECKSUM_SIZE;		

	config.mem_size = data->mem_size - config.start_ofs;

	config.mem = kzalloc(config.mem_size, GFP_KERNEL);
	if (!config.mem) {
		error = -ENOMEM;
		goto release_mem;
	}

	dev_dbg(dev, "clear_cfg: config.mem_size %i, config.start_ofs %i\n", 
		config.mem_size, config.start_ofs);

	totalBytesToWrite = config.mem_size;

	data->irq_state = MXT_IRQ_WRITE_REQ;

	do {
		reinit_completion(&data->write_completion);

		if (totalBytesToWrite > MXT_MAX_SPI_BLOCK)
			msg_size = MXT_MAX_SPI_BLOCK;
		else
			msg_size = totalBytesToWrite;

		error = mxt_spi_write_req(data, (config.start_ofs + write_offset),
			msg_size, (config.mem + write_offset));

		mxt_wait_for_completion(data, &data->write_completion, MXT_CS_MAX_DELAY);

		error = mxt_spi_write_process(data);

		//Delay required to allow data to be processed
		//before next write, else bad response

		msleep(1);

		if (error) {
			dev_info(dev, "Error writing configuration\n");
			goto release_mem;
		}

		write_offset += msg_size;

		totalBytesToWrite -= msg_size;
		
	} while (totalBytesToWrite > 0);

	data->irq_state = MXT_IRQ_WRITE_DONE;

	msleep(10);

	mxt_update_crc(data, MXT_COMMAND_BACKUPNV, MXT_BACKUP_VALUE);

	/* Possible change to completion command */
	msleep(100);

	dev_info(dev, "Config successfully cleared\n");

release_mem:
	kfree(config.mem);
	return error;
}

static int mxt_configure_objects(struct mxt_data *data,
				 const struct firmware *cfg)
{
	struct device *dev = &data->spi->dev;
	int error;

	error = mxt_init_t7_power_cfg(data);
	if (error) {
		dev_err(dev, "Failed to initialize power cfg\n");
		return error;
	}

	if (cfg) {
		error = mxt_update_cfg(data, cfg);
		if (error)
			dev_warn(dev, "Error %d updating config\n", error);
		} else {
	}

	if (data->system_power_up && !(data->sysfs_updating_config_fw)) {
		if (data->multitouch) {
			dev_info(dev, "mxt_config: Registering devices\n");
			error = mxt_initialize_input_devices(data);
			if (error)
				return error;
		} else {
			dev_warn(dev, "No touch object detected\n");
		}
	}
	msleep(100); 

	data->irq_processing = true;
	data->system_power_up = false;
	data->sysfs_updating_config_fw = false;

	return 0;
}

static int mxt_check_firmware_format(struct device *dev,
				     const struct firmware *fw)
{
	unsigned int pos = 0;
	char c;

	while (pos < fw->size) {
		c = *(fw->data + pos);

		if (c < '0' || (c > '9' && c < 'A') || c > 'F')
			return 0;
		pos++;
	}

	/*
	 * To convert file try:
	 * xxd -r -p mXTXXX__APP_VX-X-XX.enc > maxtouch.fw
	 */
	dev_err(dev, "Aborting: firmware file must be in binary format\n");

	return -EINVAL;
}

static u8 mxt_get_bootloader_version(struct mxt_data *data, u8 val)
{
	struct device *dev = &data->spi->dev;
	u8 buf[3];

	if (mxt_bootloader_read(data, &buf[0], 3) != 0) {
		dev_err(dev, "%s: spi failure\n", __func__);
		return val;
	}

	dev_info(dev, "Bootloader ID:%d Version:%d\n", buf[1], buf[2]);

	return buf[0];
}

static int mxt_check_bootloader(struct mxt_data *data, unsigned int state,
				bool wait)
{
	struct device *dev = &data->spi->dev;
	u8 val;
	int ret;

recheck:

	if (wait) {
		msleep(1);
	}

	ret = mxt_bootloader_read(data, &val, 1);
	if (ret)
		return ret;

	if (state == MXT_WAITING_BOOTLOAD_CMD)
		val = mxt_get_bootloader_version(data, val);

	switch (state) {
	case MXT_WAITING_BOOTLOAD_CMD:
	case MXT_WAITING_FRAME_DATA:
	case MXT_APP_CRC_FAIL:
		val &= ~MXT_BOOT_STATUS_MASK;
		break;
	case MXT_FRAME_CRC_PASS:
		if (val == MXT_FRAME_CRC_CHECK) {
			goto recheck;
		} else if (val == MXT_FRAME_CRC_FAIL) {
			dev_err(dev, "Bootloader CRC fail\n");
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	if (val != state) {
		dev_err(dev, "Invalid bootloader state %02X != %02X\n",
			val, state);
		return -EINVAL;
	}

	return 0;
}

static int mxt_load_fw(struct device *dev, const char *fn)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	const struct firmware *fw = NULL;
	unsigned int frame_size;
	unsigned int pos = 0;
	unsigned int retry = 0;
	unsigned int frame = 0;
	int ret;

	ret = request_firmware(&fw, fn, dev);
	if (ret) {
		dev_err(dev, "Unable to open firmware %s\n", fn);
		return ret;
	} else {

		dev_info(dev, "Opened firmware file: %s\n", fn);
	}  

	/* Check for incorrect enc file */
	ret = mxt_check_firmware_format(dev, fw);

	if (ret) {
		goto release_firmware;
	} else {
		dev_info(dev, "File format is okay\n");		
	}

	if (!data->in_bootloader) {
		/* Change to the bootloader mode */
		data->in_bootloader = true;

		ret = mxt_send_flash_command(data);

		msleep(5);

		if (ret) {
			goto release_firmware;
		} else {	
			dev_info(dev, "Sent bootloader command.\n");
		}

		reinit_completion(&data->bl_completion);
	}

	ret = mxt_check_bootloader(data, MXT_WAITING_BOOTLOAD_CMD, true);

	dev_info(dev, "Unlocking bootloader\n");

	/* Unlock bootloader */
	ret = mxt_send_bootloader_cmd(data, true);

	if (ret)
		goto disable_irq;

	msleep(30);

	/* reinit done afer spi write, possible conflict */
	//reinit_completion(&data->bl_completion);

	/* wait here until we get a completion flag */

	//ret = mxt_wait_for_completion(data, &data->bl_completion,
					     // MXT_FW_CHG_TIMEOUT);

	while (pos < fw->size) {

		ret = mxt_check_bootloader(data, MXT_WAITING_FRAME_DATA, true);
		if (ret)
			goto disable_irq;

		frame_size = ((*(fw->data + pos) << 8) | *(fw->data + pos + 1));

		/* Take account of CRC bytes */
		frame_size += 2;

		/* Write one frame to device */
		ret = mxt_bootloader_write(data, &fw->data[pos], frame_size);
		if (ret)
			goto disable_irq;

		ret = mxt_check_bootloader(data, MXT_FRAME_CRC_PASS, true);
		if (ret) {
			retry++;

			/* Back off by 20ms per retry */
			msleep(retry * 1);

			if (retry > 20) {
				dev_err(dev, "Retry count exceeded\n");
				goto disable_irq;
			}
		} else {
			retry = 0;
			pos += frame_size;
			frame++;
		}

		if (pos >= fw->size) {
			dev_info(dev, "Sent %u frames, %zu bytes\n",
				frame, fw->size);
		}
		else if (frame % 50 == 0) {
			dev_info(dev, "Sent %u frames, %d/%zu bytes\n",
				frame, pos, fw->size);
		}
	}

	dev_dbg(dev, "Sent %d frames, %d bytes\n", frame, pos);

	/*
	 * Wait for device to reset. Some bootloader versions do not assert
	 * the CHG line after bootloading has finished, so ignore potential
	 * errors. 
	 */

	msleep(MXT_BOOTLOADER_WAIT);	/* Wait for chip to leave bootloader*/
	
	//ret = mxt_wait_for_completion(data, &data->bl_completion,
	//			      MXT_BOOTLOADER_WAIT);
	//if (ret)
	//	goto disable_irq;

	data->in_bootloader = false;

disable_irq:
	disable_irq(data->irq);
release_firmware:
	release_firmware(fw);
	return ret;
}

static ssize_t mxt_update_fw_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{

	struct spi_device *spi = to_spi_device(dev);
	struct mxt_data *data = spi_get_drvdata(spi);
	int error;

	data->sysfs_updating_config_fw = true;

	data->irq_state = MXT_IRQ_WRITE_REQ;

 	//error = mxt_clear_cfg(data);

	//if (error)
	//	dev_err(dev, "Failed clear configuration\n");

	error = mxt_load_fw(dev, MXT_FW_NAME);
	if (error) {
		dev_err(dev, "The firmware update failed(%d)\n. IRQ disabled.", error);
		count = error;
	} else {
		dev_info(dev, "The firmware update succeeded\n");
	}

	msleep(MXT_FW_FLASH_TIME);

	data->irq_state = MXT_IRQ_WRITE_DONE;

	error = mxt_read_info_block(data);
	if (error)
		return error;

	//error = mxt_acquire_irq(data);
	//if (error)
		//return error;

	//mxt_soft_reset(data, true);

	//error = request_firmware_nowait(THIS_MODULE, true, MXT_CFG_NAME,
	//				dev, GFP_KERNEL, data,
	//				mxt_config_cb);

	//if (error) {
		//dev_warn(dev, "Failed to invoke firmware loader: %d\n",
			//error);
		//return error;
	//}
	
	return count;
}

static ssize_t mxt_update_cfg_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	const struct firmware *cfg;
	int ret; 
	int error;

	data->sysfs_updating_config_fw = true;

	error = mxt_clear_cfg(data);

	if (error)
		dev_err(dev, "Failed clear configuration\n");
	else
		dev_info(dev, "Done with clear configuration\n");

	ret = request_firmware(&cfg, MXT_CFG_NAME, dev);
	if (ret < 0) {
		dev_err(dev, "Failure to request config file %s\n",
			MXT_CFG_NAME);
		ret = -ENOENT;
		goto out;
	} else {
		dev_info(dev, "Found configuration file: %s\n",
			MXT_CFG_NAME);
	}
	
	data->irq_state = MXT_IRQ_WRITE_REQ;

	mxt_set_t7_power_cfg(data, MXT_POWER_CFG_RUN);

	error = mxt_process_messages_until_invalid(data);
	
	if (error)
		dev_err(dev, "Process message until invalid failed\n");

	ret = mxt_configure_objects(data, cfg);

	data->irq_state = MXT_IRQ_WRITE_DONE;

	data->sysfs_updating_config_fw = false;

	if (ret)
		goto release;

	ret = count;

release:
	release_firmware(cfg);
out:
	return ret;
}

static ssize_t mxt_debug_msg_write(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off,
	size_t count)
{
	return -EIO;
}

static ssize_t mxt_debug_msg_read(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off, size_t bytes)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mxt_data *data = dev_get_drvdata(dev);
	int count;
	size_t bytes_read;

	if (!data->debug_msg_data) {
		dev_err(dev, "No buffer!\n");
		return 0;
	}

	count = bytes / data->T5_msg_size;

	if (count > DEBUG_MSG_MAX)
		count = DEBUG_MSG_MAX;

	mutex_lock(&data->debug_msg_lock);

	if (count > data->debug_msg_count)
		count = data->debug_msg_count;

	bytes_read = count * data->T5_msg_size;

	memcpy(buf, data->debug_msg_data, bytes_read);
	data->debug_msg_count = 0;

	mutex_unlock(&data->debug_msg_lock);

	return bytes_read;
}

static int mxt_debug_msg_init(struct mxt_data *data)
{
	struct spi_device *spi = data->spi;

	sysfs_bin_attr_init(&data->debug_msg_attr);
	data->debug_msg_attr.attr.name = "debug_msg";
	data->debug_msg_attr.attr.mode = 0666;
	data->debug_msg_attr.read = mxt_debug_msg_read;
	data->debug_msg_attr.write = mxt_debug_msg_write;
	data->debug_msg_attr.size = data->T5_msg_size * DEBUG_MSG_MAX;

	if (sysfs_create_bin_file(&spi->dev.kobj,
				  &data->debug_msg_attr) < 0) {
		dev_err(&spi->dev, "Failed to create %s\n",
			data->debug_msg_attr.attr.name);
		return -EINVAL;
	}

	return 0;
}

/* Hardware Version is returned as FamilyID.VariantID */
static ssize_t mxt_hw_version_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct mxt_info *info = data->info;
	return scnprintf(buf, PAGE_SIZE, "%u.%u\n",
			 info->family_id, info->variant_id);
}

/* Firmware Version is returned as Major.Minor.Build */
ssize_t mxt_fw_version_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct mxt_info *info = data->info;
	return scnprintf(buf, PAGE_SIZE, "%u.%u.%02X\n",
			 info->version >> 4, info->version & 0xf, info->build);
}

/* Configuration crc check sum is returned as hex xxxxxx */
static ssize_t mxt_config_crc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%06x\n", data->config_crc);
}

static ssize_t mxt_debug_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	u8 i;
	ssize_t ret;

	if (kstrtou8(buf, 0, &i) == 0 && i < 2) {
		data->debug_enabled = (i == 1);

		dev_dbg(dev, "%s\n", i ? "debug enabled" : "debug disabled");
		ret = count;
	} else {
		dev_dbg(dev, "debug_enabled write error\n");
		ret = -EINVAL;
	}

	return ret;
}

static void mxt_debug_msg_enable(struct mxt_data *data)
{
	struct spi_device *spi = data->spi;
	if (data->debug_v2_enabled)
		return;

	mutex_lock(&data->debug_msg_lock);

	data->debug_msg_data = kcalloc(DEBUG_MSG_MAX,
				data->T5_msg_size, GFP_KERNEL);
	if (!data->debug_msg_data)
		return;

	data->debug_v2_enabled = true;
	mutex_unlock(&data->debug_msg_lock);

	dev_dbg(&spi->dev, "Enabled message output\n");
}

static void mxt_debug_msg_disable(struct mxt_data *data)
{
	struct spi_device *spi = data->spi;

	if (!data->debug_v2_enabled)
		return;

	data->debug_v2_enabled = false;

	mutex_lock(&data->debug_msg_lock);
	kfree(data->debug_msg_data);
	data->debug_msg_data = NULL;
	data->debug_msg_count = 0;
	mutex_unlock(&data->debug_msg_lock);
	dev_dbg(&spi->dev, "Disabled message output\n");
}

static ssize_t mxt_debug_v2_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	u8 i;
	ssize_t ret;

	if (kstrtou8(buf, 0, &i) == 0 && i < 2) {
		if (i == 1)
			mxt_debug_msg_enable(data);
		else
			mxt_debug_msg_disable(data);

		ret = count;
	} else {
		dev_dbg(dev, "debug_enabled_v2 write error\n");
		ret = -EINVAL;
	}

	return ret;
}

static ssize_t mxt_debug_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	char c;

	c = data->debug_enabled ? '1' : '0';
	return scnprintf(buf, PAGE_SIZE, "%c\n", c);
}

static ssize_t mxt_debug_notify_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0\n");
}

static ssize_t mxt_bootloader_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	char c;

	c = data->in_bootloader ? '1' : '0';
	return scnprintf(buf, PAGE_SIZE, "%c\n", c);
}

static ssize_t mxt_bootloader_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	u8 i;
	ssize_t ret;

	if (kstrtou8(buf, 0, &i) == 0 && i < 2) {
		data->in_bootloader = i;

		dev_dbg(dev, "%s\n", i ? "Bootloader mode enabled" : 
			"Bootloader mode disabled");
		ret = count;
	} else {
		dev_dbg(dev, "bootloader write error\n");
		ret = -EINVAL;
	}

	return ret;
}

static ssize_t mxt_debug_irq_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	char c;

	c = data->irq_processing ? '1' : '0';
	return scnprintf(buf, PAGE_SIZE, "%c\n", c);
}

static ssize_t mxt_debug_irq_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	u8 i;
	ssize_t ret;

	if (kstrtou8(buf, 0, &i) == 0 && i < 2) {
		data->irq_processing = i;

		dev_dbg(dev, "%s\n", i ? "Debug IRQ enabled" : "Debug IRQ disabled");
		ret = count;
	} else {
		dev_dbg(dev, "debug_irq write error\n");
		ret = -EINVAL;
	}

	return ret;
}

static ssize_t mxt_show_instance(char *buf, int count,
				 struct mxt_object *object, int instance,
				 const u8 *val)
{
	int i;

	if (mxt_obj_instances(object) > 1)
		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "Inst: %u\n", instance);

	for (i = 0; i < mxt_obj_size(object); i++)
		count += scnprintf(buf + count, PAGE_SIZE - count,
				"%02x ",val[i]);

	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");

	return count;
}

static ssize_t mxt_object_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct mxt_object *object;
	int count = 0;
	int error = 0;
	u8 *obuf;
	int i, j;

	/* Pre-allocate buffer large enough to hold max sized object. */
	/* This attribute is optional as the buffer size is large */
	obuf = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!obuf)
		return -ENOMEM;

	for (i = 0; i < data->info->object_num; i++) {
		object = data->object_table + i;

	if (!mxt_object_readable(object->type))
			continue;

		count += scnprintf(buf + count, PAGE_SIZE - count,
				"T%u:\n", object->type);

		for (j = 0; j < mxt_obj_instances(object); j++) {
			u16 size = mxt_obj_size(object);
			u16 addr = object->start_address + j * size;

			error = mxt_read_block (data, addr, size, obuf);

			if (error)
				goto done;

			count = mxt_show_instance(buf, count, object, j, obuf);
		}
	}
done:
	kfree(obuf);
	return error ?: count;
}

static int mxt_check_mem_access_params(struct mxt_data *data, loff_t off,
				       size_t *count)
{
	if (off >= data->mem_size)
		return -EIO;

	if (off + *count > data->mem_size)
		*count = data->mem_size - off;

	// Maximum data size for sysfs transfers PAGE_SIZE = 4096
	if (*count > PAGE_SIZE)
		*count = PAGE_SIZE;

	return 0;
}

static ssize_t mxt_mem_access_read(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mxt_data *data = dev_get_drvdata(dev);
	int ret = 0;

	ret = mxt_check_mem_access_params(data, off, &count);
	if (ret < 0)
		return ret;

	if (count > 0){
		if (data->in_bootloader) {
			ret = mxt_bootloader_read(data, buf, count);
			if (ret)
				return ret;
		} else {

			data->irq_state = MXT_IRQ_READ_REQ;
			ret = mxt_read_block (data, off, count, buf);
			data->irq_state = MXT_IRQ_READ_DONE;
		}
	}

	return ret == 0 ? count : ret;
}

static ssize_t mxt_mem_access_write(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off,
	size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mxt_data *data = dev_get_drvdata(dev);
	int ret = 0;

	ret = mxt_check_mem_access_params(data, off, &count);
	if (ret < 0)
		return ret;

	if (count > 0) {
		if (data->in_bootloader) {
			ret = mxt_bootloader_write(data, buf, count);
		} else {

			data->irq_state = MXT_IRQ_WRITE_REQ;
			ret = mxt_write_block (data, off, count, buf);
			data->irq_state = MXT_IRQ_WRITE_DONE;
		}
	}
	return ret == 0 ? count : ret;
}

static DEVICE_ATTR(fw_version, S_IRUGO, mxt_fw_version_show, NULL);
static DEVICE_ATTR(hw_version, S_IRUGO, mxt_hw_version_show, NULL);
static DEVICE_ATTR(object, S_IRUGO, mxt_object_show, NULL);
static DEVICE_ATTR(update_cfg, S_IWUSR, NULL, mxt_update_cfg_store);
static DEVICE_ATTR(config_crc, S_IRUGO, mxt_config_crc_show, NULL);
static DEVICE_ATTR(update_fw, S_IWUSR, NULL, mxt_update_fw_store);
static DEVICE_ATTR(debug_enable, S_IWUSR | S_IRUSR, mxt_debug_enable_show,
		   mxt_debug_enable_store);
static DEVICE_ATTR(debug_v2_enable, S_IWUSR | S_IRUSR, NULL,
		   mxt_debug_v2_enable_store);
static DEVICE_ATTR(debug_notify, S_IRUGO, mxt_debug_notify_show, NULL);
static DEVICE_ATTR(debug_irq, S_IWUSR | S_IRUSR, mxt_debug_irq_show,
		   mxt_debug_irq_store);
static DEVICE_ATTR(in_bootloader, S_IWUSR | S_IRUSR, mxt_bootloader_show,
		   mxt_bootloader_store);

static struct attribute *mxt_attrs[] = {
	&dev_attr_fw_version.attr,
	&dev_attr_hw_version.attr,
	&dev_attr_in_bootloader.attr,
	&dev_attr_debug_irq.attr,
	&dev_attr_object.attr,
	&dev_attr_update_cfg.attr,
	&dev_attr_config_crc.attr,
	&dev_attr_update_fw.attr,
	&dev_attr_debug_enable.attr,
	&dev_attr_debug_v2_enable.attr,
	&dev_attr_debug_notify.attr,
	NULL
};

static const struct attribute_group mxt_attr_group = {
	.attrs = mxt_attrs,
};

static int mxt_sysfs_init(struct mxt_data *data)
{
	struct spi_device *spi = data->spi;
	int error;

	error = sysfs_create_group(&spi->dev.kobj, &mxt_attr_group);
	if (error) {
		dev_err(&spi->dev, "Failure %d creating sysfs group\n",
			error);
		return error;
	}

	sysfs_bin_attr_init(&data->mem_access_attr);
	data->mem_access_attr.attr.name = "mem_access";
	data->mem_access_attr.attr.mode = S_IRUGO | S_IWUSR;
	data->mem_access_attr.read = mxt_mem_access_read;
	data->mem_access_attr.write = mxt_mem_access_write;
	data->mem_access_attr.size = data->mem_size;

	error = sysfs_create_bin_file(&spi->dev.kobj,
				  &data->mem_access_attr);
	if (error) {
		dev_err(&spi->dev, "Failed to create %s\n",
			data->mem_access_attr.attr.name);
		goto err_remove_sysfs_group;
	}

	return 0;

err_remove_sysfs_group:
	sysfs_remove_group(&spi->dev.kobj, &mxt_attr_group);
	return error;
}

static int mxt_spi_probe(struct spi_device *spi)
{
	struct mxt_data *data;
	int error;

	/* Set up SPI*/
	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_3;

	error = spi_setup(spi);
	if (error)
		return error;

	data = devm_kzalloc(&spi->dev, sizeof(struct mxt_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->spi = spi;
	spi_set_drvdata(spi, data);

	init_completion(&data->bl_completion);
	//init_completion(&data->reset_completion);

	init_completion(&data->crc_completion);
	init_completion(&data->read_completion);
	init_completion(&data->write_completion);
	init_completion(&data->t6_cmd_completion);

	data->system_power_up = false;
	data->irq_processing = true;

	data->irq_state = MXT_IRQ_READ_DONE;

	snprintf(data->phys, sizeof(data->phys), "%s/input0", dev_name(&spi->dev));

	data->reset_gpio = devm_gpiod_get_optional(&spi->dev,
						   "reset", GPIOD_OUT_HIGH);

	if (IS_ERR(data->reset_gpio)) {
		error = PTR_ERR(data->reset_gpio);
		dev_err(&spi->dev, "Failed to get reset gpio: %d\n", error);
		return error;
	} else {
		dev_info(&spi->dev, "Got Reset GPIO\n");
	}

	error = devm_request_threaded_irq(&spi->dev, spi->irq,
					  NULL, mxt_spi_interrupt, IRQF_ONESHOT,
					  dev_name(&data->spi->dev), data);
	if (error) {
		dev_err(&spi->dev, "Failed to register interrupt\n");
		return error;
	}

	disable_irq_nosync(data->spi->irq);

	//Duplicate. don't need if using devm_gpiod_get_optional
	//Consider which one to remove

	if(!(IS_ERR(data->reset_gpio))) {
		gpiod_direction_output(data->reset_gpio, 1);	/* GPIO in device tree is active-low */
		dev_info(&spi->dev, "Direction is ouput\n");	/* Deassert reset value = 0 */
	}
	
	if(!(IS_ERR(data->reset_gpio))) {
		dev_info(&spi->dev, "Resetting chip\n");
		gpiod_set_value(data->reset_gpio, 1);
		msleep(MXT_RESET_GPIO_TIME);
		gpiod_set_value(data->reset_gpio, 0);
		msleep(MXT_RESET_INVALID_CHG);
	}

	data->in_bootloader = false;

	error = mxt_set_tx_buffer(data);
	if (error)
		return error;

	error = mxt_initialize(data);
	if (error)
		return error;

	error = mxt_sysfs_init(data);
	if (error)
		return error;

	error = mxt_debug_msg_init(data);
	if (error)
		return error;

	mutex_init(&data->debug_msg_lock);

	return 0;

}

static void mxt_start(struct mxt_data *data)
{
	
	dev_info (&data->spi->dev, "mxt_start:  Starting . . .\n");

	/* TBD - may be executed with irq disabled */

	//mxt_set_t7_power_cfg(data, MXT_POWER_CFG_RUN);

}

static void mxt_stop(struct mxt_data *data)
{

	dev_info (&data->spi->dev, "mxt_stop:  Stopping . . .\n");
	
	/* TBD - Continue using deep sleep when driver is paused? */	
	//mxt_set_t7_power_cfg(data, MXT_POWER_CFG_DEEPSLEEP);

}

static int mxt_input_open(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);

	mxt_start(data);

	return 0;
}

static void mxt_input_close(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);

	mxt_stop(data);
}

static void mxt_free_input_device(struct mxt_data *data)
{
	if (data->input_dev) {
		input_unregister_device(data->input_dev);
		data->input_dev = NULL;
	}
}

static void mxt_sysfs_remove(struct mxt_data *data)
{
	if (data->mem_access_attr.attr.name)
		sysfs_remove_bin_file(&data->spi->dev.kobj,
			&data->mem_access_attr);

	sysfs_remove_group(&data->spi->dev.kobj, &mxt_attr_group);
}

static void mxt_debug_msg_remove(struct mxt_data *data)
{
	if (data->debug_msg_attr.attr.name)
		sysfs_remove_bin_file(&data->spi->dev.kobj,
			&data->debug_msg_attr);
}

static int mxt_spi_remove(struct spi_device *spi)
{
	struct mxt_data *data = spi_get_drvdata(spi);

	mxt_debug_msg_remove(data);
	mxt_sysfs_remove(data);

	disable_irq(data->spi->irq);
	sysfs_remove_group(&spi->dev.kobj, &mxt_attr_group);
	mxt_free_input_device(data);
	mxt_free_object_table(data);

	return 0;
}

static int __maybe_unused mxt_resume(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct mxt_data *data = spi_get_drvdata(spi);
	struct input_dev *input_dev = data->input_dev;

	if (!input_dev)
		return 0;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
	mxt_start(data);

	mutex_unlock(&input_dev->mutex);

	return 0;
}

static int __maybe_unused mxt_suspend(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct mxt_data *data = spi_get_drvdata(spi);
	struct input_dev *input_dev = data->input_dev;

	if (!input_dev)
		return 0;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
	mxt_stop(data);

	mutex_unlock(&input_dev->mutex);

	return 0;
}

static SIMPLE_DEV_PM_OPS(mxt_pm_ops, mxt_suspend, mxt_resume);

#ifdef CONFIG_OF
static const struct of_device_id mxt_ts_of_match[] = {
	{ .compatible = "microchip, mchp_spi_ts"},
	{ },
};
MODULE_DEVICE_TABLE(of, mxt_ts_of_match);
#endif

static const struct spi_device_id mxt_spi_id[] = {
	{ "mchp_spi_ts", 0 },
	{ },
};

static struct spi_driver maxtouch_spi_driver = {
	.driver = {
		.name	= "mchp_spi_ts",
		.of_match_table = of_match_ptr(mxt_ts_of_match),
		.pm 	= &mxt_pm_ops,
	},
	.probe		= mxt_spi_probe,
	.remove 	= mxt_spi_remove,
	.id_table 	= mxt_spi_id,
};

module_spi_driver(maxtouch_spi_driver);

MODULE_AUTHOR("Michael Gong <michael.gong@microchip.com>");
MODULE_DESCRIPTION("Microchip maXTouch SPI Touchscreen");
MODULE_LICENSE("GPL v2");
