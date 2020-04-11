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
#ifndef _NFC_COMMON_H_
#define _NFC_COMMON_H_
#include "nfc_drv.h"

int nfc_dev_open(struct inode *inode, struct file *filp);
int nfc_dev_close(struct inode *inode, struct file *filp);
long nfc_dev_ioctl(struct file *pfile, unsigned int cmd, unsigned long arg);
int nfc_parse_dt(struct device *dev, platform_gpio_t *nfc_gpio, int platform);
int nfc_misc_register(nfc_dev_t *nfc_dev, const struct file_operations *nfc_fops,
        int count, char *devname, char *classname);
void nfc_misc_unregister(nfc_dev_t *nfc_dev, int count);
int configure_gpio(unsigned int gpio, int flag);
void read_cold_reset_rsp(nfc_dev_t *nfc_dev, char *buf);
void gpio_set_ven_value(nfc_dev_t *nfc_dev, int value);
void gpio_free_all(nfc_dev_t *nfc_dev);
#endif //_NFC_COMMON_H_
