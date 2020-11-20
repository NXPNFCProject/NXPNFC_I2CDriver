/******************************************************************************
 *  Copyright (C) 2019-2020 NXP
 *   *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 ******************************************************************************/
#ifndef _COMMON_H_
#define _COMMON_H_
#include <linux/types.h>
#include <linux/version.h>
#include <linux/semaphore.h>
#include <linux/completion.h>
#include <linux/ioctl.h>
#include <linux/cdev.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>

#include "i2c_drv.h"

#define DEV_COUNT               1	/* Max device count for this driver */
#define CLASS_NAME              "nfc"	/* i2c device class */

//  NFC character device name, this will be in /dev/
#define NFC_CHAR_DEV_NAME       "pn553"

// NCI packet details
#define NCI_MSG_CMD                 0x20
#define NCI_MSG_RSP                 0x40
#define NCI_HDR_LEN                 3
#define NCI_PAYLOAD_IDX             3
#define NCI_PAYLOAD_LEN_IDX         2

// FW DNLD packet details
#define FW_HDR_LEN                  2
#define FW_PAYLOAD_LEN_IDX          1
#define FW_CRC_LEN                  2
#define MIN_NFC_DL_FRAME_SIZE       3

#define NCI_RESET_CMD_LEN           (4)
#define NCI_RESET_RSP_LEN           (4)
#define NCI_RESET_NTF_LEN           (13)

#define DL_GET_VERSION_CMD_LEN      (8)
#define DL_GET_VERSION_RSP_LEN_1    (12)
#define DL_GET_VERSION_RSP_LEN_2    (20)

#define DL_RESET_CMD_LEN                (8)
#define DL_GET_SESSION_STATE_CMD_LEN    (8)
#define DL_GET_SESSION_STATE_RSP_LEN    (8)
#define GET_SESSION_STS_OFF             (3)
#define NFCC_SESSION_STS_CLOSED         (0x0)
#define MAX_NCI_PAYLOAD_LEN             (255)
#define MAX_BUFFER_SIZE                 (NCI_HDR_LEN + MAX_NCI_PAYLOAD_LEN)
#define MAX_DL_PAYLOAD_LEN              (550)
#define MAX_DL_BUFFER_SIZE              (FW_HDR_LEN + FW_CRC_LEN + MAX_DL_PAYLOAD_LEN)
// Maximum retry count for standby writes
#define MAX_RETRY_COUNT                 (3)
// Retry count for normal write
#define NO_RETRY                        (1)
#define MAX_IRQ_WAIT_TIME               (90)
#define WAKEUP_SRC_TIMEOUT              (2000)

/*command response timeout*/
#define NCI_CMD_RSP_TIMEOUT             (2000)	//2s

#define NFC_MAGIC 0xE9

/*Ioctls*/
// The type should be aligned with MW HAL definitions
#define NFC_SET_PWR            _IOW(NFC_MAGIC, 0x01, long)
#define ESE_SET_PWR            _IOW(NFC_MAGIC, 0x02, long)
#define ESE_GET_PWR            _IOR(NFC_MAGIC, 0x03, long)
#define NFC_GET_PLATFORM_TYPE  _IO(NFC_MAGIC, 0x04)
#define NFC_GET_NFC_STATE      _IO(NFC_MAGIC, 0x05)
/* NFC HAL can call this ioctl to get the current IRQ state */
#define NFC_GET_IRQ_STATE      _IOW(NFC_MAGIC, 0x06, long)

#define DTS_IRQ_GPIO_STR    "nxp,pn544-irq"
#define DTS_VEN_GPIO_STR    "nxp,pn544-ven"
#define DTS_FWDN_GPIO_STR   "nxp,pn544-fw-dwnld"

enum nfcc_ioctl_request {
	/* NFC disable request with VEN LOW */
	NFC_POWER_OFF = 0,
	/* NFC enable request with VEN Toggle */
	NFC_POWER_ON,
	/* firmware download request with VEN Toggle */
	NFC_FW_DWL_VEN_TOGGLE,
	/* ISO reset request */
	NFC_ISO_RESET,
	/* request for firmware download gpio HIGH */
	NFC_FW_DWL_HIGH,
	/* VEN hard reset request */
	NFC_VEN_FORCED_HARD_RESET,
	/* request for firmware download gpio LOW */
	NFC_FW_DWL_LOW,
};

/*nfc platform interface type*/
enum interface_flags {
	/*I2C physical IF for NFCC */
	PLATFORM_IF_I2C = 0,
};

/*nfc state flags*/
enum nfc_state_flags {
	/*nfc in unknown state */
	NFC_STATE_UNKNOWN = 0,
	/*nfc in download mode */
	NFC_STATE_FW_DWL = 0x1,
	/*nfc booted in NCI mode */
	NFC_STATE_NCI = 0x2,
	/*nfc booted in Fw teared mode */
	NFC_STATE_FW_TEARED = 0x4,
};
/*
 * Power state for IBI handing, mainly needed to defer the IBI handling
 *  for the IBI received in suspend state to do it later in resume call
 */
enum pm_state_flags {
	PM_STATE_NORMAL = 0,
	PM_STATE_SUSPEND,
	PM_STATE_IBI_BEFORE_RESUME,
};

/* Enum for GPIO values*/
enum gpio_values {
	GPIO_INPUT = 0x0,
	GPIO_OUTPUT = 0x1,
	GPIO_HIGH = 0x2,
	GPIO_OUTPUT_HIGH = 0x3,
	GPIO_IRQ = 0x4,
};

// NFC GPIO variables
typedef struct platform_gpio {
	unsigned int irq;
	unsigned int ven;
	unsigned int dwl_req;
} platform_gpio_t;

//cold reset Features specific Parameters
typedef struct cold_reset {
	bool rsp_pending;	/*cmd rsp pending status */
	bool in_progress;	/*for cold reset when gurad timer in progress */
	bool reset_protection;	/*reset protection enabled/disabled */
	uint8_t status;		/*status from response buffer */
	uint8_t rst_prot_src;	/*reset protection source (SPI, NFC) */
	struct mutex sync_mutex;
	struct timer_list timer;
	wait_queue_head_t read_wq;
} cold_reset_t;

/* Device specific structure */
typedef struct nfc_dev {
	wait_queue_head_t read_wq;
	struct mutex read_mutex;
	struct mutex ese_access_mutex;
	struct mutex dev_ref_mutex;
	unsigned int dev_ref_count;
	struct class *nfc_class;
	struct device *nfc_device;
	struct cdev c_dev;
	dev_t devno;
	/* Interface flag */
	uint8_t interface;
	/* nfc state flags */
	uint8_t nfc_state;
	/* NFC VEN pin state */
	bool nfc_ven_enabled;
	union {
		i2c_dev_t i2c_dev;
	};
	platform_gpio_t gpio;
	cold_reset_t cold_reset;

	/*funtion pointers for the common i2c functionality */
	int (*nfc_read) (struct nfc_dev *dev, char *buf, size_t count);
	int (*nfc_write) (struct nfc_dev *dev, const char *buf, const size_t count,
			  int max_retry_cnt);
	int (*nfc_enable_intr) (struct nfc_dev *dev);
	int (*nfc_disable_intr) (struct nfc_dev *dev);
	int (*nfc_flush_readq) (struct nfc_dev *dev);
} nfc_dev_t;

int nfc_dev_open(struct inode *inode, struct file *filp);
int nfc_dev_close(struct inode *inode, struct file *filp);
long nfc_dev_ioctl(struct file *pfile, unsigned int cmd, unsigned long arg);
int nfc_parse_dt(struct device *dev, platform_gpio_t *nfc_gpio,
		 uint8_t interface);
int nfc_misc_register(nfc_dev_t *nfc_dev,
		      const struct file_operations *nfc_fops, int count, char *devname,
		      char *classname);
void nfc_misc_unregister(nfc_dev_t *nfc_dev, int count);
int configure_gpio(unsigned int gpio, int flag);
void gpio_set_ven(nfc_dev_t *nfc_dev, int value);
void gpio_free_all(nfc_dev_t *nfc_dev);
int validate_nfc_state_nci(nfc_dev_t *nfc_dev);
#endif //_COMMON_H_
