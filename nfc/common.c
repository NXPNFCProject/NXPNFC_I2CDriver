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
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/delay.h>
#include "../nfc/cold_reset.h"
#include "common.h"

nfc_dev_t* nfc_dev_platform = NULL;

int nfc_parse_dt(struct device *dev, platform_gpio_t *nfc_gpio, int platform)
{
    struct device_node *np = dev->of_node;

    //required for i2c based chips only
    if (PLATFORM_IF_I2C == platform) {
        nfc_gpio->irq = of_get_named_gpio(np, DTS_IRQ_GPIO_STR, 0);
        if ((!gpio_is_valid(nfc_gpio->irq))) {
            return -EINVAL;
        }
    } else {
        nfc_gpio->irq =  -EINVAL;
    }
    nfc_gpio->ven = of_get_named_gpio(np, DTS_VEN_GPIO_STR, 0);
    if ((!gpio_is_valid(nfc_gpio->ven)))
        return -EINVAL;

    nfc_gpio->dwl_req = of_get_named_gpio(np, DTS_FWDN_GPIO_STR, 0);
    if ((!gpio_is_valid(nfc_gpio->dwl_req)))
        return -EINVAL;

    //required for old platform only
    nfc_gpio->ese_pwr = of_get_named_gpio(np, DTS_ESE_GPIO_STR, 0);
    if ((!gpio_is_valid(nfc_gpio->ese_pwr)))
        nfc_gpio->ese_pwr =  -EINVAL;

    pr_info("%s: %d, %d, %d, %d\n", __func__,
                nfc_gpio->irq, nfc_gpio->ven, nfc_gpio->dwl_req,
                nfc_gpio->ese_pwr);
    return 0;
}

void gpio_set_ven_value(nfc_dev_t *nfc_dev, int value)
{
    if (nfc_dev->ven_logic == VEN_ALWAYS_ENABLED) {
        value |= 1;
    }
    if (gpio_get_value(nfc_dev->gpio.ven) != value) {
        gpio_set_value(nfc_dev->gpio.ven, value);
        /* hardware dependent delay */
        usleep_range(1000, 1100);
    }
}

int configure_gpio(unsigned int gpio, int flag)
{   int ret;
    pr_debug("%s: nfc gpio [%d] flag [%01x]\n", __func__, gpio, flag);
    if (gpio_is_valid(gpio)) {
        ret = gpio_request(gpio, "nfc_gpio");
        if (ret) {
            pr_err("%s: unable to request nfc gpio [%d]\n",
                        __func__, gpio);
            return ret;
        }
        /*set direction and value for output pin*/
        if (flag & GPIO_OUTPUT) {
            ret = gpio_direction_output(gpio, (GPIO_HIGH & flag));
        } else {
            ret = gpio_direction_input(gpio);
        }
        if (ret) {
            pr_err("%s: unable to set direction for nfc gpio [%d]\n",
                        __func__, gpio);
            gpio_free(gpio);
            return ret;
        }
        /*Consider value as control for input IRQ pin*/
        if (flag & GPIO_IRQ) {
            ret = gpio_to_irq(gpio);
            if (ret < 0) {
                pr_err("%s: unable to set irq for nfc gpio [%d]\n",
                        __func__, gpio);
                gpio_free(gpio);
                return ret;
            } else {
                pr_debug("%s: platform_gpio_to_irq succesful [%d]\n",
                        __func__, gpio);
                return ret;
            }
        }
    } else {
        pr_err("%s: invalid gpio\n", __func__);
        ret = -EINVAL;
    }
    return ret;
}

void gpio_free_all(nfc_dev_t *nfc_dev)
{
    if (gpio_is_valid(nfc_dev->gpio.ese_pwr)) {
        gpio_free(nfc_dev->gpio.ese_pwr);
    }
    if (gpio_is_valid(nfc_dev->gpio.dwl_req)) {
        gpio_free(nfc_dev->gpio.dwl_req);
    }
    if (gpio_is_valid(nfc_dev->gpio.irq)) {
        gpio_free(nfc_dev->gpio.irq);
    }
    if (gpio_is_valid(nfc_dev->gpio.ven)) {
        gpio_free(nfc_dev->gpio.ven);
    }
}

void nfc_misc_unregister(nfc_dev_t *nfc_dev, int count)
{
    pr_debug("%s: entry\n", __func__);
    device_destroy(nfc_dev->nfc_class , nfc_dev->devno);
    cdev_del(&nfc_dev->c_dev);
    class_destroy(nfc_dev->nfc_class);
    unregister_chrdev_region(nfc_dev->devno, count);
}

int nfc_misc_register(nfc_dev_t *nfc_dev, const struct file_operations *nfc_fops,
        int count, char *devname, char *classname)
{
    int ret = 0;
    ret = alloc_chrdev_region(&nfc_dev->devno, 0, count, devname);
    if (ret < 0) {
        pr_err("%s: failed to alloc chrdev region\n", __func__);
        return ret;
    }
    nfc_dev->nfc_class = class_create(THIS_MODULE, classname);
    if (IS_ERR(nfc_dev->nfc_class)) {
        ret = PTR_ERR(nfc_dev->nfc_class);
        pr_err("%s: failed to register device class\n", __func__);
        unregister_chrdev_region(nfc_dev->devno, count);
        return ret;
    }
    cdev_init(&nfc_dev->c_dev, nfc_fops);
    ret = cdev_add(&nfc_dev->c_dev, nfc_dev->devno, count);
    if (ret < 0) {
        pr_err("%s: failed to add cdev\n", __func__);
        class_destroy(nfc_dev->nfc_class);
        unregister_chrdev_region(nfc_dev->devno, count);
        return ret;
    }
    nfc_dev->nfc_device = device_create(nfc_dev->nfc_class, NULL,
                    nfc_dev->devno, nfc_dev, devname);
    if (IS_ERR(nfc_dev->nfc_device)) {
        ret = PTR_ERR(nfc_dev->nfc_device);
        pr_err("%s: failed to create the device\n", __func__);
        cdev_del(&nfc_dev->c_dev);
        class_destroy(nfc_dev->nfc_class);
        unregister_chrdev_region(nfc_dev->devno, count);
        return ret;
    }
    return 0;
}

static void enable_interrupt(nfc_dev_t *nfc_dev, bool ibi)
{
    if (nfc_dev->platform == PLATFORM_IF_I2C) {
        i2c_enable_irq(&nfc_dev->i2c_dev);
    } else if (ibi == true) {
#ifdef CONFIG_NXP_NFC_I3C
        i3c_enable_ibi(&nfc_dev->i3c_dev);
#endif //CONFIG_NXP_NFC_I3C
    }
}

static void disable_interrupt(nfc_dev_t *nfc_dev, bool ibi)
{
    if (nfc_dev->platform == PLATFORM_IF_I2C) {
        i2c_disable_irq(&nfc_dev->i2c_dev);
    } else if (ibi == true) {
#ifdef CONFIG_NXP_NFC_I3C
        i3c_disable_ibi(&nfc_dev->i3c_dev);
#endif //CONFIG_NXP_NFC_I3C
    }
}

static int send_cold_reset_cmd(nfc_dev_t *nfc_dev)
{
    int ret = 0;
    int write_retry = 3;
    char cmd[COLD_RESET_CMD_LEN];

    cmd[0] = COLD_RESET_CMD_GID;
    cmd[1] = COLD_RESET_OID;
    cmd[2] = COLD_RESET_CMD_PAYLOAD_LEN;
    do {
        if(nfc_dev->platform == PLATFORM_IF_I2C) {
            ret = i2c_write(&nfc_dev->i2c_dev, cmd, COLD_RESET_CMD_LEN);
        } else {
            //TODO: Handling Cold reset for I3C
            //ret = i3c_write(nfc_dev->i3c_dev, cmd, COLD_RESET_CMD_LEN);
        }
        write_retry--;
        if (ret == COLD_RESET_CMD_LEN) break;
        usleep_range(5000, 6000);
    } while(write_retry);
    if(!write_retry && (ret != COLD_RESET_CMD_LEN)) {
        pr_err("%s : i2c_master_send returned %d\n", __func__, ret);
        nfc_dev->cold_reset.timer_started = false;
        return -EIO;
    }
    pr_info("%s: NxpNciX: %d > %02X%02X%02X \n", __func__, ret,cmd[0],cmd[1],cmd[2]);
    return ret;
}

void read_cold_reset_rsp(nfc_dev_t *nfc_dev, char *buf)
{
    int ret = -1;
    char rsp[COLD_RESET_RSP_LEN];
    i2c_dev_t *i2c_dev = &nfc_dev->i2c_dev;
    cold_reset_t *cold_reset = &nfc_dev->cold_reset;
    cold_reset->status = -EIO;
    /*
     * read header also if NFC is disabled
     * for enable case, will be taken care by nfc read thread
     */
    if (!cold_reset->nfc_enabled) {
        if(nfc_dev->platform == PLATFORM_IF_I2C) {
            ret = i2c_read(i2c_dev, rsp, NCI_HDR_LEN);
        } else {
            //TODO: Handling Cold reset for I3C
            //ret = i3c_read(i3c_dev, rsp, NCI_HDR_LEN);
        }
        if (ret != NCI_HDR_LEN) {
            pr_err("%s: failure to read cold reset rsp header\n", __func__);
            return;
        }
    } else {
        memcpy(rsp, buf, NCI_HDR_LEN);
    }
    if ((NCI_HDR_LEN + rsp[NCI_PAYLOAD_LEN_OFFSET]) != COLD_RESET_RSP_LEN) {
        pr_err("%s: - invalid response for cold_reset\n", __func__);
        return;
    }
    if(nfc_dev->platform == PLATFORM_IF_I2C) {
        ret = i2c_read(i2c_dev, &rsp[NCI_PAYLOAD_IDX], rsp[2]);
    } else {
        //TODO:Handling Cold Reset for I3C
        //ret = i3c_read(nfc_dev->i3c_dev, &rsp[NCI_PAYLOAD_IDX], rsp[2]);
    }
    if (ret != rsp[2]) {
        pr_err("%s: failure to read cold reset rsp header\n", __func__);
        return;
    }
    pr_info("%s NxpNciR : len = 4 > %02X%02X%02X%02X\n", __func__,rsp[0],rsp[1],rsp[2],rsp[3]);
    cold_reset->status = rsp[NCI_PAYLOAD_IDX];
}

static void ese_cold_reset_gaurd_timer_callback(unsigned long data)
{
    (void)data;
    pr_info("%s: Enter\n",__func__);
    nfc_dev_platform->cold_reset.timer_started = false;
    return;
}

static long start_ese_cold_reset_guard_timer(void)
{
    long ret = -EINVAL;
    if(timer_pending(&nfc_dev_platform->cold_reset.timer) == 1)
    {
        pr_info("ese_cold_reset_guard_timer: delete pending timer \n");
        /* delete timer if already pending */
        del_timer(&nfc_dev_platform->cold_reset.timer);
    }
    nfc_dev_platform->cold_reset.timer_started = true;
    init_timer(&nfc_dev_platform->cold_reset.timer);
    setup_timer( &nfc_dev_platform->cold_reset.timer, ese_cold_reset_gaurd_timer_callback, 0 );
    ret = mod_timer(&nfc_dev_platform->cold_reset.timer,
            jiffies + msecs_to_jiffies(ESE_COLD_RESET_GUARD_TIME));
    return ret;
}

static int perform_ese_cold_reset(nfc_dev_t *nfc_dev, ese_cold_reset_origin_t origin) {
    int ret = 0;

    if (gpio_get_value(nfc_dev->gpio.dwl_req)) {
        pr_err("FW download in-progress\n");
        return -EBUSY;
    }
    if (!gpio_get_value(nfc_dev->gpio.ven)) {
        pr_err("VEN LOW - NFCC powered off\n");
        return -ENODEV;
    }

    mutex_lock(&nfc_dev->cold_reset.sync_mutex);
    if(!nfc_dev->cold_reset.timer_started) {
        ret = start_ese_cold_reset_guard_timer();
        if(ret) {
            pr_err("%s: Error in mod_timer\n",__func__);
            mutex_unlock(&nfc_dev->cold_reset.sync_mutex);
            return ret;
        }
        /* set default value for status as failure */
        nfc_dev->cold_reset.status = -EIO;
        ret = send_cold_reset_cmd(nfc_dev);
        if (ret <= 0) {
            pr_err("failed to send cold reset command\n");
            mutex_unlock(&nfc_dev->cold_reset.sync_mutex);
            return ret;
        }
        ret = 0;
        nfc_dev->cold_reset.rsp_pending = true;
        /* check if NFC is enabled */
        if (nfc_dev->cold_reset.nfc_enabled) {
            /* Pending read from NFC_HAL will read the cold reset rsp and signal read_wq */
            if(!wait_event_interruptible_timeout(nfc_dev->cold_reset.read_wq,
                    nfc_dev->cold_reset.rsp_pending == false,
                    msecs_to_jiffies(ESE_COLD_RESET_CMD_RSP_TIMEOUT))){
                pr_err("%s:Cold Reset Response Timeout\n",__func__);
            }
        } else {
            /* Read data as NFC thread is not active */
            enable_interrupt(nfc_dev, false);
            read_cold_reset_rsp(nfc_dev, NULL);
            nfc_dev->cold_reset.rsp_pending = false;
        }
        if(!ret) { /* wait for reboot guard timer*/
            if(wait_event_interruptible_timeout(nfc_dev->cold_reset.read_wq,true,
                    msecs_to_jiffies(ESE_COLD_RESET_REBOOT_GUARD_TIME)) == 0){
                pr_err("%s: guard Timeout interrupted", __func__);
            }
        }
    }
    mutex_unlock(&nfc_dev->cold_reset.sync_mutex);
    if(ret == 0) /* success case */
        ret = nfc_dev->cold_reset.status;
    return ret;
}

/*
 * Power management of the eSE
 * eSE and NFCC both are powered using VEN gpio,
 * VEN HIGH - eSE and NFCC both are powered on
 * VEN LOW - eSE and NFCC both are power down
 */
int nfc_ese_pwr(nfc_dev_t *nfc_dev, unsigned long arg)
{
    int ret = 0;
    if (arg == ESE_POWER_ON) {
        /**
         * Let's store the NFC VEN pin state
         * will check stored value in case of eSE power off request,
         * to find out if NFC MW also sent request to set VEN HIGH
         * VEN state will remain HIGH if NFC is enabled otherwise
         * it will be set as LOW
         */
        nfc_dev->nfc_ven_enabled =
            gpio_get_value(nfc_dev->gpio.ven);
        if (!nfc_dev->nfc_ven_enabled) {
            pr_debug("eSE HAL service setting ven HIGH\n");
            gpio_set_ven_value(nfc_dev, 1);
        } else {
            pr_debug("ven already HIGH\n");
        }
    } else if (arg == ESE_POWER_OFF) {
        if (!nfc_dev->nfc_ven_enabled) {
            pr_debug("NFC not enabled, disabling ven\n");
            gpio_set_ven_value(nfc_dev, 0);
        } else {
            pr_debug("keep ven high as NFC is enabled\n");
        }
    } else if (IS_COLD_RESET_REQ(arg) && nfc_dev->platform == PLATFORM_IF_I2C) {
        ret = perform_ese_cold_reset(nfc_dev, arg);
    } else if (arg == ESE_POWER_STATE) {
        // eSE power state
        ret = gpio_get_value(nfc_dev->gpio.ven);
    } else {
        pr_err("%s bad arg %lu\n", __func__, arg);
        ret = -ENOIOCTLCMD;
    }
    return ret;
}
EXPORT_SYMBOL(nfc_ese_pwr);

/*
 * This function shall be called from SPI, UWB, NFC driver to perform eSE cold reset.
 */
int ese_cold_reset(ese_cold_reset_origin_t origin) {
    int ret = 0;
    unsigned long arg;
    pr_info("%s: Enter origin:%d", __func__, origin);

    switch(origin) {
      case ESE_COLD_RESET_SOURCE_SPI:
          arg = ESE_COLD_RESET_SPI;
          break;
      case ESE_COLD_RESET_SOURCE_UWB:
          arg = ESE_COLD_RESET_UWB;
        break;
      default:
        pr_info("%s: Invalid argument", __func__);
        return -EINVAL;
    }
    if(nfc_dev_platform == NULL)
        return -ENODEV;
    ret = nfc_ese_pwr(nfc_dev_platform, arg);
    pr_info("%s:%d exit, Status:%d", __func__, origin, ret);
    return ret;
}
EXPORT_SYMBOL(ese_cold_reset);

/*
 * nfc_ioctl_power_states() - power control
 * @filp:    pointer to the file descriptor
 * @arg:    mode that we want to move to
 *
 * Device power control. Depending on the arg value, device moves to
 * different states
 * (arg = 0): NFC_ENABLE    GPIO = 0, FW_DL GPIO = 0
 * (arg = 1): NFC_ENABLE    GPIO = 1, FW_DL GPIO = 0
 * (arg = 2): FW_DL GPIO = 1
 *
 * Return: -ENOIOCTLCMD if arg is not supported, 0 in any other case
 */
static int nfc_ioctl_power_states(nfc_dev_t *nfc_dev, unsigned long arg)
{
    int ret = 0;
    if (arg == NFC_POWER_OFF) {
        /*
         * We are attempting a hardware reset so let us disable
         * interrupts to avoid spurious notifications to upper
         * layers.
         */
        disable_interrupt(nfc_dev, false);
        pr_debug("gpio firm disable\n");
        if (gpio_is_valid(nfc_dev->gpio.dwl_req)) {
            gpio_set_value(nfc_dev->gpio.dwl_req, 0);
            usleep_range(10000, 10100);
        }
        if (gpio_is_valid(nfc_dev->gpio.ese_pwr)) {
            if (!gpio_get_value(nfc_dev->gpio.ese_pwr)) {
                pr_debug("disabling ven\n");
                gpio_set_ven_value(nfc_dev, 0);
            } else {
                pr_debug("keeping ven high\n");
            }
        } else {
            pr_debug("ese_pwr invalid, set ven to low\n");
            gpio_set_ven_value(nfc_dev, 0);
        }
        nfc_dev->nfc_ven_enabled = false;
    } else if (arg == NFC_POWER_ON) {
         enable_interrupt(nfc_dev, false);
        pr_debug("gpio_set_value enable: %s: info: %p\n",
            __func__, nfc_dev);
        if (gpio_is_valid(nfc_dev->gpio.dwl_req)) {
            gpio_set_value(nfc_dev->gpio.dwl_req, 0);
            usleep_range(10000, 10100);
        }
        gpio_set_ven_value(nfc_dev, 1);
        nfc_dev->nfc_ven_enabled = true;
#ifdef CONFIG_NXP_NFC_I3C
        if (nfc_dev->platform == PLATFORM_IF_I3C)
            nfc_dev->i3c_dev.read_hdr = NCI_HDR_LEN;
#endif //CONFIG_NXP_NFC_I3C
    } else if (arg == NFC_FW_DWL_VEN_TOGGLE) {
        /*
         * We are switching to Dowload Mode, toggle the enable pin
         * in order to set the NFCC in the new mode
         */
        if (gpio_is_valid(nfc_dev->gpio.ese_pwr)) {
            if (gpio_get_value(nfc_dev->gpio.ese_pwr)) {
                pr_err("FW download forbidden while ese is on\n");
                return -EBUSY; /* Device or resource busy */
            }
        }
        gpio_set_value(nfc_dev->gpio.ven, 1);
        usleep_range(10000, 10100);
        if (gpio_is_valid(nfc_dev->gpio.dwl_req)) {
            gpio_set_value(nfc_dev->gpio.dwl_req, 1);
            usleep_range(10000, 10100);
        }
        if(nfc_dev->platform == PLATFORM_IF_I2C) {
            gpio_set_value(nfc_dev->gpio.ven, 0);
            usleep_range(10000, 10100);
        }
        gpio_set_value(nfc_dev->gpio.ven, 1);
        usleep_range(10000, 10100);
    } else if (arg == NFC_FW_DWL_HIGH) {
        /*
         * Setting firmware download gpio to HIGH
         * before FW download start
         */
        pr_debug("set fw gpio high\n");
        if (gpio_is_valid(nfc_dev->gpio.dwl_req)) {
            gpio_set_value(nfc_dev->gpio.dwl_req, 1);
            usleep_range(10000, 10100);
        } else
            pr_debug("gpio.dwl_req is invalid\n");
    } else if (arg == NFC_VEN_FORCED_HARD_RESET && nfc_dev->platform == PLATFORM_IF_I2C) {
            /*TODO: Enable Ven reset for I3C only after hot join integration*/
            disable_interrupt(nfc_dev, false);
            wake_up(&nfc_dev->read_wq);
            usleep_range(10000, 10100);
            gpio_set_value(nfc_dev->gpio.ven, 0);
            usleep_range(10000, 10100);
            gpio_set_value(nfc_dev->gpio.ven, 1);
            usleep_range(10000, 10100);
            pr_info("%s VEN forced reset done\n", __func__);
    } else if (arg == NFC_FW_DWL_LOW) {
        /*
         * Setting firmware download gpio to LOW
         * FW download finished
         */
        pr_debug("set fw gpio LOW\n");
        if (gpio_is_valid(nfc_dev->gpio.dwl_req)) {
            gpio_set_value(nfc_dev->gpio.dwl_req, 0);
            usleep_range(10000, 10100);
        } else {
            pr_debug("gpio.dwl_req is invalid\n");
        }
#ifdef CONFIG_NXP_NFC_I3C
        if (nfc_dev->platform == PLATFORM_IF_I3C)
            nfc_dev->i3c_dev.read_hdr = NCI_HDR_LEN;
#endif //CONFIG_NXP_NFC_I3C
#ifdef CONFIG_NXP_NFC_I3C
    } else if (arg == NFC_FW_HDR_LEN) {
        if (nfc_dev->platform == PLATFORM_IF_I3C)
            nfc_dev->i3c_dev.read_hdr = FW_HDR_LEN;
#endif //CONFIG_NXP_NFC_I3C
    } else {
        pr_err("%s bad arg %lu\n", __func__, arg);
        ret = -ENOIOCTLCMD;
    }
    return ret;
}

/** @brief   IOCTL function  to be used to set or get data from upper layer.
 *
 *  @param   pfile  fil node for opened device.
 *  @cmd     IOCTL type from upper layer.
 *  @arg     IOCTL arg from upper layer.
 *
 *  @return 0 on success, error code for failures.
 */
long nfc_dev_ioctl(struct file *pfile, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    struct nfc_dev *nfc_dev = pfile->private_data;
    if (!nfc_dev)
    return -ENODEV;

    pr_debug("%s cmd = %x arg = %zx",__func__, cmd, arg);
    switch (cmd) {
        case NFC_SET_PWR:
            ret = nfc_ioctl_power_states(nfc_dev, arg);
            break;
        case ESE_SET_PWR:
            ret = nfc_ese_pwr(nfc_dev, arg);
            break;
        case ESE_GET_PWR:
            ret = nfc_ese_pwr(nfc_dev, 3);
            break;
        case NFC_GET_PLATFORM_TYPE:
            ret = nfc_dev->platform;
            break;
        default:
            pr_err("%s bad arg %lu\n", __func__, arg);
            ret = -ENOIOCTLCMD;
    };
    return ret;
}

int nfc_dev_open(struct inode *inode, struct file *filp)
{
    nfc_dev_t *nfc_dev = container_of(inode->i_cdev, nfc_dev_t, c_dev);
    if(!nfc_dev)
        return -ENODEV;
    pr_debug("%s: %d, %d\n", __func__, imajor(inode), iminor(inode));
    filp->private_data = nfc_dev;
    mutex_lock(&nfc_dev->dev_ref_mutex);
    if (nfc_dev->dev_ref_count == 0) {
        if (gpio_is_valid(nfc_dev->gpio.dwl_req)) {
            gpio_set_value(nfc_dev->gpio.dwl_req, 0);
            usleep_range(10000, 10100);
        }
        nfc_dev->cold_reset.nfc_enabled = true;
        enable_interrupt(nfc_dev, true);
    }
    nfc_dev->dev_ref_count = nfc_dev->dev_ref_count + 1;
    mutex_unlock(&nfc_dev->dev_ref_mutex);
    return 0;
}

int nfc_dev_close(struct inode *inode, struct file *filp)
{
    nfc_dev_t *nfc_dev = container_of(inode->i_cdev, nfc_dev_t, c_dev);
    if(!nfc_dev)
        return -ENODEV;
    pr_debug("%s: %d, %d\n", __func__, imajor(inode), iminor(inode));
    mutex_lock(&nfc_dev->dev_ref_mutex);
    if (nfc_dev->dev_ref_count == 1) {
        disable_interrupt(nfc_dev, true);
        if (gpio_is_valid(nfc_dev->gpio.dwl_req)) {
            gpio_set_value(nfc_dev->gpio.dwl_req, 0);
            usleep_range(10000, 10100);
        }
        nfc_dev->cold_reset.nfc_enabled = false;
    }
    if (nfc_dev->dev_ref_count > 0)
        nfc_dev->dev_ref_count = nfc_dev->dev_ref_count - 1;
    mutex_unlock(&nfc_dev->dev_ref_mutex);
    filp->private_data = NULL;
    return 0;
}
