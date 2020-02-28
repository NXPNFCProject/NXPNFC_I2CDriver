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
#ifndef _I3C_DRV_H_
#define _I3C_DRV_H_

#include <linux/i3c/master.h>
#include <linux/i3c/device.h>

//to to kept same as dt
#define NFC_I3C_DRV_STR         "nxp,pn544_i3c"
#define NFC_I3C_MANU_ID         (0x011B)
#define NFC_I3C_PART_ID         (0)

//Byte indicating I3C Read
#define NFC_I3C_READ            1

//Byte indicating I3C Write
#define NFC_I3C_WRITE           0

// Maximum no of IBI slot
#define NUM_NFC_IBI_SLOT        1

// Maximum  IBI payload length
#define MAX_IBI_PAYLOAD_LEN     0

// CRC len to be read
#define FW_CRC_LEN              2

// FW DNLD HDR length
#define FW_HDR_LEN              2

// Time to wait before retrying I3C writes, in micro seconds
#define RETRY_WAIT_TIME_USEC    (2000)

// Retry count for enable/disable IBI CCC
#define RETRY_COUNT_IBI         (3)

// I3C WorkQueue name
#define I3C_WORKQUEUE_NAME      "i3c_workq"
/**
 * struct nci_buf - NCI buffer used to store and retrieve read data from device.
 * @read_offset: The offset pointing to data available to read in nci buf.
 * @write_offset: The offset pointing to free buf available to write.
 * @total_size: Size of nci buf.
 * @kbuf: allocated nci buf.
 */
struct nci_buf {
    unsigned int read_offset;
    unsigned int write_offset;
    size_t total_size;
    char *kbuf;
};

/**
 * struct i3c_dev   Structure representing device and driver data.
 * @i3c_device        Structure to represent I3C device.
 * @wq:               NCI workqueue for handling IBI request.
 * @work:             Work added to workqueue to read data from IBI handler.
 * @buf               Driver buf store read data from device.Read call will
 *                    fetch from this buffer.
 * @nci_buf_mutex:    mutex to protect NCI buf retrieve/store .
 * @read_cplt:        Completion to wait for read data to be available.
 * @read_kbuf_len:    Temp buf len to hold I3C data.
 * @read_kbuf:        Temp buf to hold I3C data.
 * @read_hdr          Header size for reads.
 * @ibi_enabled:      IBI enabled or not.
 * @pm_state:         PM state of NFC I3C device.
 */
typedef struct i3c_dev {
    struct i3c_device *device;
    struct workqueue_struct *wq;
    struct work_struct work;
    struct nci_buf buf;
    struct mutex nci_buf_mutex;
    struct completion read_cplt;
    size_t read_kbuf_len;
    char *read_kbuf;
    unsigned char read_hdr;
    bool ibi_enabled;
    atomic_t pm_state;

} i3c_dev_t;

int nfc_i3c_dev_probe(struct i3c_device *device);
int nfc_i3c_dev_remove(struct i3c_device *device);
int nfc_i3c_dev_suspend(struct device *device);
int nfc_i3c_dev_resume(struct device *device);
int i3c_enable_ibi(i3c_dev_t *i3c_dev);
int i3c_disable_ibi(i3c_dev_t *i3c_dev);
ssize_t i3c_write(i3c_dev_t *i3c_dev, const char *buf, const size_t count,
                  int max_retry_cnt);
ssize_t i3c_read(i3c_dev_t *, char *buf, size_t count);
#endif //_I3C_DRV_H_
