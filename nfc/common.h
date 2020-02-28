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

#ifdef CONFIG_NXP_NFC_I2C
#include "i2c_drv.h"
#endif
#ifdef CONFIG_NXP_NFC_I3C
#include "i3c_drv.h"
#endif

#define DEV_COUNT               1   /* Max device count for this driver */
#define CLASS_NAME              "nfc"   /* i2c device class */

//  NFC character device name, this will be in /dev/
#define NFC_CHAR_DEV_NAME       "pn553"
#define NCI_HDR_LEN             3   /* HDR length of NCI packet */
#define NCI_PAYLOAD_LEN_OFFSET  2
#define NCI_PAYLOAD_IDX         3
#define MAX_NCI_PAYLOAD_LEN     (255)
#define MAX_BUFFER_SIZE         (NCI_HDR_LEN + MAX_NCI_PAYLOAD_LEN)
#define MAX_RETRY_COUNT         (3)
#define NO_RETRY                (1)
#define MAX_IRQ_WAIT_TIME       (90)
#define WAKEUP_SRC_TIMEOUT      (2000)

/* ESE_COLD_RESET MACROS */
#define COLD_RESET_CMD_LEN        3
#define COLD_RESET_RSP_LEN        4
#define COLD_RESET_CMD_GID        0x2F
#define COLD_RESET_CMD_PAYLOAD_LEN    0x00
#define COLD_RESET_RSP_GID        0x4F
#define COLD_RESET_OID            0x1E
/*
 * ESE_RESET: Bit mask to check if ese_reset_guard timer is started (bit b7)
 * */
#define ESE_COLD_RESET_GUARD_TIMER_MASK  (0x80)
/*
 * ESE_RESET: Guard time to allow eSE cold reset from the driver
 * */
#define ESE_COLD_RESET_GUARD_TIME        (3000) //3s
/*
 * ESE_RESET: NCI command response timeout
*/
#define ESE_COLD_RESET_CMD_RSP_TIMEOUT   (2000) //2s
/*
 * ESE_RESET: Guard time to reboot the JCOP
*/
#define ESE_COLD_RESET_REBOOT_GUARD_TIME   (50) //50ms
/*
 * ESE_RESET: Checks if eSE cold reset has been requested
 */
#define IS_COLD_RESET_REQ(arg) ((arg == ESE_COLD_RESET_NFC) ||                      \
        (arg == ESE_COLD_RESET_SPI) || (arg == ESE_COLD_RESET_UWB))
/*
 *  ESE_RESET: macro evaluates to 1 if eSE cold reset response is received
 * */
#define IS_COLD_RESET_RSP(buf) ((COLD_RESET_RSP_GID == buf[0]) && (COLD_RESET_OID == buf[1]))

#define NFC_MAGIC 0xE9

/*Ioctls*/
// The type should be aligned with MW HAL definitions
#define NFC_SET_PWR            _IOW(NFC_MAGIC, 0x01, long)
#define ESE_SET_PWR            _IOW(NFC_MAGIC, 0x02, long)
#define ESE_GET_PWR            _IOR(NFC_MAGIC, 0x03, long)
#define NFC_GET_PLATFORM_TYPE  _IO(NFC_MAGIC, 0x0B)

#define DTS_IRQ_GPIO_STR    "nxp,pn544-irq"
#define DTS_VEN_GPIO_STR    "nxp,pn544-ven"
#define DTS_FWDN_GPIO_STR   "nxp,pn544-fw-dwnld"
#define DTS_ESE_GPIO_STR    "nxp,pn544-ese-pwr"

enum ese_ioctl_request {
    /* eSE POWER ON */
    ESE_POWER_ON = 0,
    /* eSE POWER OFF */
    ESE_POWER_OFF,
    /* eSE POWER STATE */
    ESE_POWER_STATE,
    /* eSE COLD RESET */
    ESE_COLD_RESET_NFC,
    ESE_COLD_RESET_SPI,
    ESE_COLD_RESET_UWB,
};

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
    /*for HDR size change in FW mode */
    NFC_FW_HDR_LEN,
    /* Cold reset request for eSE */
    NFC_ESE_COLD_RST,
};

/*nfc platform interface type*/
enum interface_flags {
    /*I2C physical IF for NFCC */
    PLATFORM_IF_I2C = 0,
    /*I3C physical IF for NFCC */
    PLATFORM_IF_I3C,
};
/*nfc platform interface type*/
enum ven_policy_flags {
    /*VEN usage in lagacy platform */
    VEN_LEGACY = 0,
    /*VEN reset only to recover from failure usecases */
    VEN_ALWAYS_ENABLED,
};
/* Power state for IBI handing, mainly needed to defer the IBI handling
   for the IBI received in suspend state to do it later in resume call*/
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
    unsigned int ese_pwr;
} platform_gpio_t;

//Features specific Parameters
typedef struct cold_reset {
    wait_queue_head_t read_wq;
    bool rsp_pending;
    unsigned int ntf;
    uint8_t status;
    /* NFC device opened by MW */
    bool nfc_enabled;
    /* eSe cold reset guard timer is started */
    bool timer_started;
    struct mutex sync_mutex;
    struct timer_list timer;
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

    /*store the ven functioning */
    uint8_t ven_policy;
    /* NFC VEN pin state */
    bool nfc_ven_enabled;
    union {
#ifdef CONFIG_NXP_NFC_I2C
        i2c_dev_t i2c_dev;
#endif
#ifdef CONFIG_NXP_NFC_I3C
        i3c_dev_t i3c_dev;
#endif
    };
    platform_gpio_t gpio;
    cold_reset_t cold_reset;
} nfc_dev_t;

int nfc_dev_open(struct inode *inode, struct file *filp);
int nfc_dev_close(struct inode *inode, struct file *filp);
long nfc_dev_ioctl(struct file *pfile, unsigned int cmd, unsigned long arg);
int nfc_parse_dt(struct device *dev, platform_gpio_t *nfc_gpio,
                 uint8_t interface);
int nfc_misc_register(nfc_dev_t *nfc_dev,
                      const struct file_operations *nfc_fops,
                      int count, char *devname, char *classname);
void nfc_misc_unregister(nfc_dev_t *nfc_dev, int count);
int configure_gpio(unsigned int gpio, int flag);
void read_cold_reset_rsp(nfc_dev_t *nfc_dev, char *buf);
void gpio_set_ven(nfc_dev_t *nfc_dev, int value);
void gpio_free_all(nfc_dev_t *nfc_dev);
#endif //_COMMON_H_
