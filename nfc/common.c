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
#include <linux/version.h>
#include "common.h"
#include "common_ese.h"

int nfc_parse_dt(struct device *dev, platform_gpio_t * nfc_gpio,
		 uint8_t interface)
{
	struct device_node *np = dev->of_node;

	if (!np) {
		pr_err("nfc of_node NULL\n");
		return -EINVAL;
	}

	nfc_gpio->irq = -EINVAL;
	nfc_gpio->dwl_req = -EINVAL;
	nfc_gpio->ven = -EINVAL;

	//required for i2c based chips only
	if (interface == PLATFORM_IF_I2C) {
		nfc_gpio->irq = of_get_named_gpio(np, DTS_IRQ_GPIO_STR, 0);
		if ((!gpio_is_valid(nfc_gpio->irq))) {
			pr_err("nfc irq gpio invalid %d\n", nfc_gpio->irq);
			return -EINVAL;
		}
		pr_info("%s: irq %d\n", __func__, nfc_gpio->irq);

		nfc_gpio->dwl_req = of_get_named_gpio(np, DTS_FWDN_GPIO_STR, 0);
		if ((!gpio_is_valid(nfc_gpio->dwl_req))) {
			pr_err("nfc dwl_req gpio invalid %d\n", nfc_gpio->dwl_req);
		}
	}
	nfc_gpio->ven = of_get_named_gpio(np, DTS_VEN_GPIO_STR, 0);
	if ((!gpio_is_valid(nfc_gpio->ven))) {
		pr_err("nfc ven gpio invalid %d\n", nfc_gpio->ven);
		return -EINVAL;
	}

	pr_info("%s: %d, %d, %d, %d\n", __func__, nfc_gpio->irq, nfc_gpio->ven,
		nfc_gpio->dwl_req);
	return 0;
}

void set_valid_gpio(int gpio, int value)
{
	if (gpio_is_valid(gpio)) {
		pr_debug("%s gpio %d value %d\n", __func__, gpio, value);
		gpio_set_value(gpio, value);
		// hardware dependent delay
		usleep_range(10000, 10100);
	}
}

int get_valid_gpio(int gpio)
{
	int value = -1;
	if (gpio_is_valid(gpio)) {
		value = gpio_get_value(gpio);
		pr_debug("%s gpio %d value %d\n", __func__, gpio, value);
	}
	return value;
}

void gpio_set_ven(struct nfc_dev *nfc_dev, int value)
{
	if (gpio_get_value(nfc_dev->gpio.ven) != value) {
		pr_debug("%s: gpio_set_ven %d\n", __func__, value);
		/*reset on change in level from high to low */
		if (value) {
			common_ese_on_hard_reset(nfc_dev);
		}
		gpio_set_value(nfc_dev->gpio.ven, value);
		// hardware dependent delay
		usleep_range(10000, 10100);
	}
}

int configure_gpio(unsigned int gpio, int flag)
{
	int ret;
	pr_debug("%s: nfc gpio [%d] flag [%01x]\n", __func__, gpio, flag);
	if (gpio_is_valid(gpio)) {
		ret = gpio_request(gpio, "nfc_gpio");
		if (ret) {
			pr_err("%s: unable to request nfc gpio [%d]\n", __func__, gpio);
			return ret;
		}
		/*set direction and value for output pin */
		if (flag & GPIO_OUTPUT) {
			ret = gpio_direction_output(gpio, (GPIO_HIGH & flag));
			pr_debug("nfc o/p gpio %d level %d\n", gpio, gpio_get_value(gpio));
		} else {
			ret = gpio_direction_input(gpio);
			pr_debug("nfc i/p gpio %d\n", gpio);
		}

		if (ret) {
			pr_err("%s: unable to set direction for nfc gpio [%d]\n", __func__, gpio);
			gpio_free(gpio);
			return ret;
		}
		/*Consider value as control for input IRQ pin */
		if (flag & GPIO_IRQ) {
			ret = gpio_to_irq(gpio);
			if (ret < 0) {
				pr_err("%s: unable to set irq for nfc gpio [%d]\n", __func__, gpio);
				gpio_free(gpio);
				return ret;
			}
			pr_debug("%s: gpio_to_irq successful [%d]\n", __func__, gpio);
			return ret;
		}
	} else {
		pr_err("%s: invalid gpio\n", __func__);
		ret = -EINVAL;
	}
	return ret;
}

void gpio_free_all(nfc_dev_t *nfc_dev)
{
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
	device_destroy(nfc_dev->nfc_class, nfc_dev->devno);
	cdev_del(&nfc_dev->c_dev);
	class_destroy(nfc_dev->nfc_class);
	unregister_chrdev_region(nfc_dev->devno, count);
}

int nfc_misc_register(nfc_dev_t *nfc_dev,
		      const struct file_operations *nfc_fops,
		      int count, char *devname, char *classname)
{
	int ret = 0;
	ret = alloc_chrdev_region(&nfc_dev->devno, 0, count, devname);
	if (ret < 0) {
		pr_err("%s: failed to alloc chrdev region ret %d\n", __func__, ret);
		return ret;
	}
	nfc_dev->nfc_class = class_create(THIS_MODULE, classname);
	if (IS_ERR(nfc_dev->nfc_class)) {
		ret = PTR_ERR(nfc_dev->nfc_class);
		pr_err("%s: failed to register device class ret %d\n", __func__, ret);
		unregister_chrdev_region(nfc_dev->devno, count);
		return ret;
	}
	cdev_init(&nfc_dev->c_dev, nfc_fops);
	ret = cdev_add(&nfc_dev->c_dev, nfc_dev->devno, count);
	if (ret < 0) {
		pr_err("%s: failed to add cdev ret %d\n", __func__, ret);
		class_destroy(nfc_dev->nfc_class);
		unregister_chrdev_region(nfc_dev->devno, count);
		return ret;
	}
	nfc_dev->nfc_device = device_create(nfc_dev->nfc_class, NULL,
					    nfc_dev->devno, nfc_dev, devname);
	if (IS_ERR(nfc_dev->nfc_device)) {
		ret = PTR_ERR(nfc_dev->nfc_device);
		pr_err("%s: failed to create the device ret %d\n", __func__, ret);
		cdev_del(&nfc_dev->c_dev);
		class_destroy(nfc_dev->nfc_class);
		unregister_chrdev_region(nfc_dev->devno, count);
		return ret;
	}
	return 0;
}

/*
 * nfc_ioctl_power_states() - power control
 * @nfc_dev:    nfc device data structure
 * @arg:    mode that we want to move to
 *
 * Device power control. Depending on the arg value, device moves to
 * different states, refer common.h for args
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
		nfc_dev->nfc_disable_intr(nfc_dev);
		set_valid_gpio(nfc_dev->gpio.dwl_req, 0);
		gpio_set_ven(nfc_dev, 0);
		nfc_dev->nfc_ven_enabled = false;
	} else if (arg == NFC_POWER_ON) {
		nfc_dev->nfc_enable_intr(nfc_dev);
		set_valid_gpio(nfc_dev->gpio.dwl_req, 0);

		gpio_set_ven(nfc_dev, 1);
		nfc_dev->nfc_ven_enabled = true;
	} else if (arg == NFC_FW_DWL_VEN_TOGGLE) {
		/*
		 * We are switching to download Mode, toggle the enable pin
		 * in order to set the NFCC in the new mode
		 */
		nfc_dev->nfc_disable_intr(nfc_dev);
		set_valid_gpio(nfc_dev->gpio.dwl_req, 1);
		if (nfc_dev->interface == PLATFORM_IF_I2C) {
			nfc_dev->nfc_state = NFC_STATE_FW_DWL;
		}
		gpio_set_ven(nfc_dev, 0);
		gpio_set_ven(nfc_dev, 1);
		nfc_dev->nfc_enable_intr(nfc_dev);
	} else if (arg == NFC_FW_DWL_HIGH) {
		/*
		 * Setting firmware download gpio to HIGH
		 * before FW download start
		 */
		set_valid_gpio(nfc_dev->gpio.dwl_req, 1);
		if (nfc_dev->interface == PLATFORM_IF_I2C) {
			nfc_dev->nfc_state = NFC_STATE_FW_DWL;
		}

	} else if (arg == NFC_VEN_FORCED_HARD_RESET) {
		nfc_dev->nfc_disable_intr(nfc_dev);
		gpio_set_ven(nfc_dev, 0);
		gpio_set_ven(nfc_dev, 1);
		nfc_dev->nfc_enable_intr(nfc_dev);
	} else if (arg == NFC_FW_DWL_LOW) {
		/*
		 * Setting firmware download gpio to LOW
		 * FW download finished
		 */
		set_valid_gpio(nfc_dev->gpio.dwl_req, 0);
		if (nfc_dev->interface == PLATFORM_IF_I2C) {
			nfc_dev->nfc_state = NFC_STATE_NCI;
		}
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

	pr_debug("%s cmd = %x arg = %zx\n", __func__, cmd, arg);
	switch (cmd) {
	case NFC_SET_PWR:
		ret = nfc_ioctl_power_states(nfc_dev, arg);
		break;
	case ESE_SET_PWR:
		ret = nfc_ese_pwr(nfc_dev, arg);
		break;
	case ESE_GET_PWR:
		ret = nfc_ese_pwr(nfc_dev, ESE_POWER_STATE);
		break;
	case NFC_GET_PLATFORM_TYPE:
		ret = nfc_dev->interface;
		break;
	case NFC_GET_NFC_STATE:
		ret = nfc_dev->nfc_state;
		pr_debug("nfc get state %d\n", ret);
		break;
	case NFC_GET_IRQ_STATE:
		ret = 0;
		ret = gpio_get_value(nfc_dev->gpio.irq);
		break;
	default:
		pr_err("%s bad cmd %lu\n", __func__, arg);
		ret = -ENOIOCTLCMD;
	};
	return ret;
}

int nfc_dev_open(struct inode *inode, struct file *filp)
{
	nfc_dev_t *nfc_dev = container_of(inode->i_cdev, nfc_dev_t, c_dev);
	pr_debug("%s: %d, %d\n", __func__, imajor(inode), iminor(inode));

	mutex_lock(&nfc_dev->dev_ref_mutex);

	filp->private_data = nfc_dev;

	if (nfc_dev->dev_ref_count == 0) {
		set_valid_gpio(nfc_dev->gpio.dwl_req, 0);

		nfc_dev->nfc_enable_intr(nfc_dev);
	}
	nfc_dev->dev_ref_count = nfc_dev->dev_ref_count + 1;
	mutex_unlock(&nfc_dev->dev_ref_mutex);
	return 0;
}

int nfc_dev_close(struct inode *inode, struct file *filp)
{
	nfc_dev_t *nfc_dev = container_of(inode->i_cdev, nfc_dev_t, c_dev);
	pr_debug("%s: %d, %d\n", __func__, imajor(inode), iminor(inode));
	mutex_lock(&nfc_dev->dev_ref_mutex);
	if (nfc_dev->dev_ref_count == 1) {
		nfc_dev->nfc_disable_intr(nfc_dev);
		set_valid_gpio(nfc_dev->gpio.dwl_req, 0);
	}
	if (nfc_dev->dev_ref_count > 0)
		nfc_dev->dev_ref_count = nfc_dev->dev_ref_count - 1;
	else {
		nfc_ese_pwr(nfc_dev, ESE_RST_PROT_DIS_NFC);
		/* Uncomment below line incase of eSE calls flow is via NFC driver
		 * i.e. direct calls from SPI HAL to NFC driver*/
		//nfc_ese_pwr(nfc_dev, ESE_RST_PROT_DIS);
	}
	filp->private_data = NULL;

	mutex_unlock(&nfc_dev->dev_ref_mutex);
	return 0;
}

static int get_nfcc_boot_state(struct nfc_dev *nfc_dev)
{
	int ret = 0;
	char get_version_cmd[] = { 0x00, 0x04, 0xF1, 0x00, 0x00, 0x00, 0x6E, 0xEF };
	char get_session_state_cmd[] = { 0x00, 0x04, 0xF2, 0x00, 0x00, 0x00, 0xF5, 0x33 };
	char rsp_buf[MAX_BUFFER_SIZE];
	/*clearing any data in the kbuf store */
	do {
		ret = nfc_dev->nfc_read(nfc_dev, rsp_buf, MAX_BUFFER_SIZE);
		if (ret < 0) {
			pr_err("%s: - nfc read ret %d\n", __func__, ret);
		}
		pr_info("%s: - nfc read ret %d\n", __func__, ret);
	} while (ret > 0);

	pr_debug("%s:Sending GET_VERSION cmd\n", __func__);
	ret = nfc_dev->nfc_write(nfc_dev, get_version_cmd,
				 sizeof(get_version_cmd), MAX_RETRY_COUNT);
	if (ret <= 0) {
		pr_err("%s: - nfc get version cmd error ret %d\n", __func__, ret);
		goto err;
	}
	memset(rsp_buf, 0x00, MAX_BUFFER_SIZE);
	pr_debug("%s:Reading response of GET_VERSION cmd\n", __func__);
	ret = nfc_dev->nfc_read(nfc_dev, rsp_buf, MAX_BUFFER_SIZE);
	if (ret <= 0) {
		pr_err("%s: - nfc get version rsp error ret %d\n", __func__, ret);
		goto err;
	} else if (rsp_buf[0] == 0x0
		   && ret == (FW_HDR_LEN + rsp_buf[FW_PAYLOAD_LEN_IDX] + FW_CRC_LEN)
		   && (ret == DL_GET_VERSION_RSP_LEN_1 || ret == DL_GET_VERSION_RSP_LEN_2)) {

		pr_info("%s:NFC chip_type 0x%02x rom_version 0x%02x fw_minor 0x%02x fw_major 0x%02x\n",
			__func__, rsp_buf[3], rsp_buf[4], rsp_buf[6], rsp_buf[7]);
	} else if (rsp_buf[0] != 0x0
		   && ret == (NCI_HDR_LEN + rsp_buf[NCI_PAYLOAD_LEN_IDX])) {
		pr_info("%s:NFC response bytes 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n", __func__,
			rsp_buf[0], rsp_buf[1], rsp_buf[2], rsp_buf[3], rsp_buf[3]);
		pr_debug("%s NFCC booted in NCI mode %d\n", __func__, __LINE__);
		return NFC_STATE_NCI;
	}

	pr_debug("%s:Sending GET_SESSION_STATE cmd \n", __func__);
	ret = nfc_dev->nfc_write(nfc_dev, get_session_state_cmd,
				 sizeof(get_session_state_cmd),
				 MAX_RETRY_COUNT);
	if (ret <= 0) {
		pr_err("%s: - nfc get session state cmd err ret %d\n", __func__, ret);
		goto err;
	}
	memset(rsp_buf, 0x00, DL_GET_SESSION_STATE_RSP_LEN);
	pr_debug("%s:Reading response of GET_SESSION_STATE cmd\n", __func__);
	ret = nfc_dev->nfc_read(nfc_dev, rsp_buf, DL_GET_SESSION_STATE_RSP_LEN);
	if (ret <= 0) {
		pr_err("%s: - nfc get session state rsp err %d\n", __func__, ret);
		goto err;
	}
	pr_debug("Response bytes are %02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x",
		 rsp_buf[0], rsp_buf[1], rsp_buf[2], rsp_buf[3], rsp_buf[4], rsp_buf[5],
		 rsp_buf[6], rsp_buf[7]);
	/*verify fw in non-teared state */
	if (rsp_buf[GET_SESSION_STS_OFF] != NFCC_SESSION_STS_CLOSED) {
		pr_debug("%s NFCC  booted in teared fw state %d\n", __func__, __LINE__);
		return NFC_STATE_FW_TEARED;
	}
	pr_debug("%s NFCC booted in FW DN mode %d\n", __func__, __LINE__);
	return NFC_STATE_FW_DWL;
err:
	pr_err("%s Unlikely NFCC not booted in FW DN mode %d\n", __func__, __LINE__);
	return NFC_STATE_UNKNOWN;
}

int validate_nfc_state_nci(nfc_dev_t *nfc_dev)
{
	if (!gpio_get_value(nfc_dev->gpio.ven)) {
		pr_err("VEN LOW - NFCC powered off\n");
		return -ENODEV;
	} else {
		if (get_valid_gpio(nfc_dev->gpio.dwl_req) == 1) {
			pr_err("FW download in-progress\n");
			return -EBUSY;
		} else if (nfc_dev->nfc_state == NFC_STATE_FW_DWL) {
			pr_err("FW download state \n");
			return -EBUSY;
		}
	}
	return 0;
}
