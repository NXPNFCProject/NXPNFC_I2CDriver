/******************************************************************************
 *  Copyright (C) 2019 NXP
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
#ifndef _NFC_I3C_H_
#define _NFC_I3C_H_

#include <linux/i3c/master.h>
#include <linux/i3c/device.h>
#include "nfc_drv.h"

#define NFC_I3C_DEVICE_NAME     "pn553"
#define NFC_I3C_DEVICE_ID       "nxp,pn544_i3c" /*to to kept same as dt*/
/*initialy fixed for sn110, should be intialized by kconfig for other chips*/
#define NFC_I3C_MANU_ID         (0x011B)
#define NFC_I3C_PART_ID         (0)

#define NFC_I3C_READ            (1)         /* Byte indicating I3C Read*/
#define NFC_I3C_WRITE           (0)         /* Byte indicating I3C Write*/
#define NUM_NFC_IBI_SLOT        2           /* Maximum no of IBI slot*/
#define MAX_IBI_PAYLOAD_LEN     0           /* Maximum  IBI payload length*/
#define FW_CRC_LEN              2           /* CRC len to be read */
#define FW_HDR_LEN              2           /* FW DNLD HDR length */

#define I3C_WORKQUEUE_NAME      "i3c_workq" /* I3C WorkQueue name*/
#define MAX_IRQ_WAIT_TIME        (90)    //in ms
#define MAX_BUFFER_SIZE            (320)
#define WAKEUP_SRC_TIMEOUT        (2000)

/**
 * struct nci_buf - NCI buffer used to store and retrieve read data from device.
 * @read_offset: The offset pointing to data available to read in nci buf.
 * @write_offset: The offset pointing to free buf available to write.
 * @total_size: Size of nci buf.
 * @kbuf: allocated nci buf.This will usually be one page size allocated during probe.
 */
struct nci_buf {
    unsigned int read_offset;
    unsigned int write_offset;
    size_t total_size;
    char* kbuf;
};

/**
 * struct i3c_dev   Structure representing device and driver data.
 * @i3c_device        Structure to represent I3C device.
 * @wq:               NCI workqueue for handling IBI request.
 * @work:             Work added to workqueue to read data from IBI handler.
 * @buf               Driver buf store read data from device.Read system call will
 *                    fetch from this buffer.
 * @nci_buf_mutex:    mutex to protect NCI buf retrieve/store .
 * @read_cplt:        Completion to wait for read data to be available.
 * @read_kbuf_len:    Temp buf len to hold I3C data.
 * @read_kbuf:        Temp buf to hold I3C data.
 * @read_hdr          Read mode NCI / FW download.
 * @ibi_enabled:      IBI from SN110 device is enabled or not.
 * @count_ibi:        Number of IBI received.
 * @pm_state:         State for suspend/resume to defer the workqueue task
 * @ibi_enabled_lock  spin lock acquired in ibi handler.
 */
typedef struct i3c_dev {
    struct i3c_device   *device;
    /*IBI handling parameters */
    struct workqueue_struct *wq;
    struct work_struct  work;
    struct nci_buf      buf;
    struct mutex        nci_buf_mutex;
    struct completion   read_cplt;
    size_t              read_kbuf_len;
    char                *read_kbuf;
    unsigned char       read_hdr;
    /*IBI parameters */
    bool                ibi_enabled;
    atomic_t            count_ibi;
    atomic_t            pm_state;
    spinlock_t          ibi_enabled_lock;
} i3c_dev_t;
int nfc_i3c_dev_probe(struct i3c_device *device);
int nfc_i3c_dev_remove(struct i3c_device *device);
int nfc_i3c_dev_suspend(struct device *device);
int nfc_i3c_dev_resume(struct device *device);
int i3c_enable_ibi(i3c_dev_t* i3c_dev);
int i3c_disable_ibi(i3c_dev_t* i3c_dev);
#endif //_NFC_I3C_H_
