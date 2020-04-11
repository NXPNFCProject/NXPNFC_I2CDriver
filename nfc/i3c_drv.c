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
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include "nfc_drv.h"
#include "common.h"
#include "sn110.h"


/** @brief   This API used to write I3C data to I3C device.
 *
 *  @param dev   the i3c_dev for the i3c device.
 *  @param buf   the data to write
 *  @param count the number of bytes of data to be written.
 *  @return ret  number of bytes written ,negative error core otherwise.
 */
ssize_t i3c_write(i3c_dev_t *i3c_dev, const char *buf,
                               const size_t count) {
    int ret = -EIO;
    int retry_count = 0;
    struct i3c_priv_xfer write_buf = {
        .rnw = NFC_I3C_WRITE,
        .len = count,
        .data.out = buf
    };
    do {
        ret = i3c_device_do_priv_xfers(i3c_dev->device, &write_buf, 1);

        pr_debug("%s exit ret =%x",__func__, ret);
        if(!ret) {
            ret = count;
            break;
        }
        pr_err("%s errno =%x",__func__, write_buf.err);
        retry_count++;

        msleep(10);
    } while(retry_count < MAX_WRITE_RETRY);
    return ret;
}

/** @brief   This API used to read data from I3C device.
 *
 *  @param dev   the i3c_dev for the slave.
 *  @param buf   the buffer to copy the data.
 *  @param count the number of bytes to be read.
 *  @return number of bytes read ,negative error core otherwise.
 */
ssize_t i3c_read(i3c_dev_t *i3c_dev, char *buf , size_t count)
{
    int ret = -EIO;
    struct i3c_priv_xfer read_buf = {
        .rnw = NFC_I3C_READ,
        .len = count,
        .data.in = buf
    };
    ret = i3c_device_do_priv_xfers(i3c_dev->device , &read_buf, 1);
    pr_debug("%s exit ret =%x",__func__, ret);
    if(!ret)
        ret = count;
    else
        pr_err("%s errno =%x",__func__, read_buf.err);
    return ret;
}

/** @brief   This API can be used to write data to nci buf.
 *           The API will overwrite the existing memory if
 *           it reaches the end of total allocated memory.
 *  @param dev   the dev structure for driver.
 *  @param readbuf   the buffer to be copied data from
 *  @param count the number of bytes to copy to nci_buffer.
 *  @return number of bytes copied to nci buffer , error code otherwise
 */
static ssize_t i3c_kbuf_store(i3c_dev_t *i3c_dev, const char* buf,
        const size_t count)
{
    size_t buf_offset = 0;
    size_t requested_size = count;
    size_t available_size = 0;
    if(i3c_dev == NULL)
        return -ENODEV;
    else if(buf == NULL || count == 0)
        return -EINVAL;

    pr_debug("%s enter",__func__);
    if(count > i3c_dev->buf.total_size) {
        pr_err("%s No memory to copy the data",__func__);
        return -ENOMEM;
    }
    mutex_lock(&i3c_dev->nci_buf_mutex);
    pr_debug("%s:total_size %zx write_offset =%x read_offset=%x \n",__func__,
            i3c_dev->buf.total_size, i3c_dev->buf.write_offset,i3c_dev->buf.read_offset);
    available_size = i3c_dev->buf.total_size - i3c_dev->buf.write_offset;
    /* When available buffer is less than requested count , copy the data upto
       available memory. The remaining data is copied to the start of memory.
       The write offset is incremented by the remaining copied bytes from the beginning.*/
    if(requested_size > available_size) {
        pr_err("%s: memory not available need to swap circular memory requested_size =%zx available_size=%zx \n",__func__,requested_size,available_size);
        memcpy(i3c_dev->buf.kbuf + i3c_dev->buf.write_offset,
            buf + buf_offset, available_size);
        requested_size = requested_size - available_size;
        i3c_dev->buf.write_offset = 0;
        buf_offset = available_size;
        pr_debug("%s: requested_size =%zx available_size=%zx \n",__func__,requested_size,available_size);
    }
    if(requested_size) {
        memcpy(i3c_dev->buf.kbuf + i3c_dev->buf.write_offset, buf + buf_offset,
            requested_size);
        i3c_dev->buf.write_offset += requested_size;
        if(i3c_dev->buf.write_offset == i3c_dev->buf.total_size) {
            i3c_dev->buf.write_offset = 0;
        }
    }
    complete(&i3c_dev->read_cplt);
    mutex_unlock(&i3c_dev->nci_buf_mutex);
    pr_debug("%s: total bytes  requested_size =%zx available_size=%zx \n",__func__,requested_size,available_size);
    return count;
}

/** @brief   This API can be used to retrieve data from driver buffer.
 *
 *  When data is not available, it waits for required data to be present.When data
 *  is present it copies the data into buffer reuested by read.
 *  @param dev  the dev structure for driver.
 *  @param buf   the buffer to copy the data.
 *  @param count the number of bytes to be read.
 *  @return number of bytes copied , error code for failures .
 */
static ssize_t i3c_nci_kbuf_retrieve(i3c_dev_t *i3c_dev , char *buf , size_t count)
{
    int ret = 0;
    size_t requested_size = count;
    size_t available_size = 0;
    size_t copied_size = 0;
    if(i3c_dev == NULL)
        return -ENODEV;
    else if(buf == NULL || count == 0)
        return -EINVAL;
    pr_debug("%s enter",__func__);

    /*When the requested data count is more than available data to read, wait on
    completion till the requested bytes are available.
    If write offset is more than read offset and available data is more than
    requested count . Copy the requested bytes directly and increment the read_offset.
    If read offset is more than write offset , available size is total_size size - read_offset
    and upto write offset from the beginning of buffer.*/
    do {
        mutex_lock(&i3c_dev->nci_buf_mutex);
        pr_debug("%s: read_offset=%x write_offset =%x\n",__func__,
                i3c_dev->buf.read_offset,i3c_dev->buf.write_offset);
        if(i3c_dev->buf.read_offset <= i3c_dev->buf.write_offset)
            available_size = i3c_dev->buf.write_offset - i3c_dev->buf.read_offset;
        else
            available_size = (i3c_dev->buf.total_size - i3c_dev->buf.read_offset) +
                      i3c_dev->buf.write_offset;
        mutex_unlock(&i3c_dev->nci_buf_mutex);
        if(available_size >= requested_size) {
            break;
        }
        reinit_completion(&i3c_dev->read_cplt);
        ret = wait_for_completion_interruptible(&i3c_dev->read_cplt);
        if(ret != 0) {
            pr_err("didnt get completion, interrupted!! ret %d\n",ret);
            return EINVAL;
        }
    } while(available_size < requested_size);

    mutex_lock(&i3c_dev->nci_buf_mutex);

    if(i3c_dev->buf.write_offset >= i3c_dev->buf.read_offset + requested_size) {
        /*Write offset is more than read offset + count , copy the data
          directly and increment the read offset*/
        memcpy(buf, i3c_dev->buf.kbuf + i3c_dev->buf.read_offset , requested_size);
        i3c_dev->buf.read_offset += requested_size;
    } else {
        copied_size = i3c_dev->buf.total_size - i3c_dev->buf.read_offset;
        if(copied_size > requested_size)
            copied_size = requested_size;
        /*Read offset is more than write offset.Copy requested data from read_offset
         to the total size and increment the read offset.If requested data is still
         greater than zero , copy the data from beginning of buffer.*/
        memcpy(buf, i3c_dev->buf.kbuf + i3c_dev->buf.read_offset, copied_size);
        requested_size = requested_size - copied_size;
        i3c_dev->buf.read_offset += copied_size;
        if(requested_size) {
            pr_debug("%s  remaining copied bytes",__func__);
            i3c_dev->buf.read_offset = 0;
            memcpy(buf + copied_size, i3c_dev->buf.kbuf + i3c_dev->buf.read_offset,
                requested_size);
            i3c_dev->buf.read_offset += requested_size;
        }
    }
    mutex_unlock(&i3c_dev->nci_buf_mutex);
    pr_debug("%s , count =%zx exit" ,__func__, count);
    return count;
}

/** @brief   This API can be used to read data from I3C device from HAL layer.
 *
 *  This read function is registered during probe.
 *  When data is not available, it waits for required data to be present.
 *  @param filp  the device file handle opened by HAL.
 *  @param buf   the buffer to read the data.
 *  @param count the number of bytes to be read.
 *  @param offset the offset in the buf.
 *  @return Number of bytes read from I3C device ,error code for failures.
 */
ssize_t nfc_i3c_dev_read(struct file* filp, char __user *buf,
                    size_t count, loff_t* offset)
{
    int ret;
    char tmp[MAX_BUFFER_SIZE];
    nfc_dev_t *nfc_dev = filp->private_data;
    i3c_dev_t *i3c_dev = &nfc_dev->i3c_dev;
    if (count > MAX_BUFFER_SIZE)
        count = MAX_BUFFER_SIZE;
    memset(tmp, 0x00, count);
    pr_debug("%s : reading %zu bytes.\n", __func__, count);
    ret = i3c_nci_kbuf_retrieve(i3c_dev ,tmp, count);
    if (ret != count) {
        pr_err("%s: buf read  from I3C device returned error(%x)\n",
            __func__, ret);
        ret = -EIO;
    } else if (copy_to_user(buf, tmp, ret)) {
        pr_warn("%s : failed to copy to user space\n", __func__);
        ret = -EFAULT;
    }
    return ret;
}

/** @brief   This API can be used to write data to I3C device.
 *
 *  @param dev   the i3c_dev for the slave.
 *  @param buf   the buffer to copy the data.
 *  @param count the number of bytes to be read.
 *  @return ret count number of btes written, error code for failures.
 */
ssize_t nfc_i3c_dev_write(struct file* filp, const char __user *buf,
            size_t count, loff_t* offset)
{
    int ret;
    char tmp[MAX_BUFFER_SIZE];
    nfc_dev_t *nfc_dev = filp->private_data;
    i3c_dev_t *i3c_dev = &nfc_dev->i3c_dev;
    if(count > MAX_BUFFER_SIZE)
        return -ENOMEM;
    if (copy_from_user(tmp, buf, count)) {
        pr_err("%s : failed to copy from user space\n", __func__);
        ret = PTR_ERR(tmp);
        return ret;
    }
    ret = i3c_write(i3c_dev, tmp, count);
    if (ret != count) {
        pr_err("%s: failed to write %d\n", __func__, ret);
        ret = -EIO;
    }
    pr_debug("%s : i3c-%d: NfcNciTx %x %x %x\n", __func__, iminor(file_inode(filp)),
            tmp[0], tmp[1], tmp[2]);
    pr_debug("%s : ret = %x\n", __func__, ret);
    return ret;
}

/** @brief   This API shall be called from  workqueue queue from IBI handler.
 *           First it will read HDR byte from I3C chip.Based on the length byte
 *           it will read the next length bytes.Then it will write these bytes
 *           to nci write buf.
 *  @param work   the work added into the workqueue.
 *
 *  @return void
 */
static void i3c_workqueue_handler(struct work_struct *work)
{
    int ret = 0;
    int length_byte = 0;
    unsigned char *tmp = NULL;
    unsigned char hdr_len = NCI_HDR_LEN;
    i3c_dev_t *i3c_dev = container_of(work, i3c_dev_t, work);

    if(!i3c_dev) {
        pr_err("%s: dev not found\n", __func__);
        return;
    }
    hdr_len = i3c_dev->read_hdr;
    tmp = i3c_dev->read_kbuf;
    if (!tmp) {
        pr_err("%s: No memory to copy read data\n", __func__);
        return;
    }
    pr_err("%s: hdr_len = %d\n", __func__, hdr_len);
    memset(tmp, 0x00, i3c_dev->read_kbuf_len);

    ret = i3c_read(i3c_dev, tmp, hdr_len);
    if (ret < 0) {
        pr_err("%s: i3c_read returned error %d\n", __func__, ret);
        return;
    }
    if(hdr_len == FW_HDR_LEN)
        length_byte = tmp[hdr_len - 1] + FW_CRC_LEN;
    else
        length_byte = tmp[hdr_len - 1];
    ret = i3c_read(i3c_dev, tmp + hdr_len, length_byte);
    if (ret < 0) {
        pr_err("%s: i3c_read returned error %d\n", __func__, ret);
        i3c_kbuf_store(i3c_dev, tmp, hdr_len);
        return;
    }
    i3c_kbuf_store(i3c_dev, tmp, hdr_len + length_byte);
}

/** @brief   This API  is used to handle IBI coming from the I3C device.
 *  This will add work into the workqueue , which will call workqueue
 *  handler to read data from I3C device.
 *  @param device   I3C device.
 *  @param payload  payload shall be NULL for NFC device.
 *  @return void.
 */
static void i3c_ibi_handler(struct i3c_device *device, const struct i3c_ibi_payload *payload)
{
    nfc_dev_t* nfc_dev = i3cdev_get_drvdata(device);
    i3c_dev_t *i3c_dev = &nfc_dev->i3c_dev;
    pr_debug("%s: Received read IBI request from slave",__func__);
    if (device_may_wakeup(&device->dev)) {
        pm_wakeup_event(&device->dev, WAKEUP_SRC_TIMEOUT);
    }
    if(atomic_read(&i3c_dev->pm_state) == PM_STATE_NORMAL) {
        if(!queue_work(i3c_dev->wq, &i3c_dev->work))
            pr_debug("%s: Added workqueue successfully",__func__);
    } else {
         /*assume suspend state and expect only 1 IBI in suspend state*/
         atomic_set(&i3c_dev->pm_state, PM_STATE_IBI_BEFORE_RESUME);
    }
}

/** @brief   This API can be used to enable IBI from the I3C device.
 *  @param i3c_dev   the i3c_dev for the slave.
 *  @return 0 on success, error code for failures.
 */
int i3c_enable_ibi(i3c_dev_t* i3c_dev)
{
    int ret = 0;
    if (i3c_dev->ibi_enabled) {
        ret = i3c_device_enable_ibi(i3c_dev->device);
        if(!ret)
            i3c_dev->ibi_enabled = true;
        pr_debug("%s ret=%d\n",__func__, ret);
    } else {
        pr_debug("%s: already disabled\n",__func__);
    }
    return ret;
}

/** @brief   This API can be used to disable IBI from the I3C device.
 *  @param i3c_dev   the i3c_dev for the slave.
 *  @return 0 on success, error code for failures.
 */
int i3c_disable_ibi(i3c_dev_t* i3c_dev)
{
    int ret = 0;
    if (i3c_dev->ibi_enabled) {
        ret = i3c_device_disable_ibi(i3c_dev->device);
        if(!ret)
            i3c_dev->ibi_enabled = false;
        pr_debug("%s ret=%d",__func__, ret);
    } else {
        pr_debug("%s: already disabled",__func__);
    }
    return ret;
}

/** @brief   This API can be used to  request IBI from the I3C device.
 *  This function will request IBI from master controller for the device
 *  and register ibi handler, enable IBI .
 *  @param i3c_dev   the i3c_dev for the slave.
 *  @return 0 on success, error code for failures.
 */
static int i3c_request_ibi(i3c_dev_t* i3c_dev) {
    int ret = 0;
    struct i3c_ibi_setup ibireq = {
        .handler = i3c_ibi_handler,
        .max_payload_len = MAX_IBI_PAYLOAD_LEN,
        .num_slots = NUM_NFC_IBI_SLOT,
    };
    ret = i3c_device_request_ibi(i3c_dev->device , &ibireq);
    pr_debug("%s Request IBI status = %d",__func__, ret);
    return ret;
}

/** @brief   This API can be used to create a workqueue for
 *           handling IBI request from I3C device.
 *
 *  @param dev   the i3c_dev for the slave.
 *  @return 0 on success, error code for failures.
 */
static int i3c_init_workqueue(i3c_dev_t* i3c_dev)
{
    i3c_dev->wq = alloc_workqueue(I3C_WORKQUEUE_NAME, 0, 0);
    if(!i3c_dev->wq) {
        return -ENOMEM;
    }
    INIT_WORK(&i3c_dev->work, i3c_workqueue_handler);
    pr_debug("%s ibi workqueue created successfully ",__func__);
    return 0;
}

/** @brief   This API can be used to set the NCI buf to zero.
 *
 *  @param  dev   the dev for the driver.
 *  @return 0 on success, error code for failures.
 */
static int i3c_reset_nci_buf(i3c_dev_t* i3c_dev)
{
    i3c_dev->buf.write_offset = 0;
    i3c_dev->buf.read_offset = 0;
    memset(i3c_dev->buf.kbuf, 0, i3c_dev->buf.total_size);
    return 0;
}

static const struct file_operations nfc_i3c_dev_fops = {
    .owner = THIS_MODULE,
    .llseek = no_llseek,
    .read  = nfc_i3c_dev_read,
    .write = nfc_i3c_dev_write,
    .open = nfc_dev_open,
    .release = nfc_dev_close,
    .unlocked_ioctl = nfc_dev_ioctl,
};

/** @brief   This API can be used to probe I3c device.
 *
 *  @param device   the i3c_dev for the slave.
 *  @return 0 on success, error code for failures.
 */
int nfc_i3c_dev_probe(struct i3c_device *device)
{
    int ret = 0;
    nfc_dev_t *nfc_dev = NULL;
    platform_gpio_t  nfc_gpio;
    pr_debug("%s: enter\n", __func__);
    /*retrive details of gpios from dt*/
    ret = nfc_parse_dt(&device->dev, &nfc_gpio, PLATFORM_IF_I3C);
    if (ret) {
        pr_err("%s : failed to parse\n", __func__);
        goto err;
    }
    nfc_dev = kzalloc(sizeof(nfc_dev_t), GFP_KERNEL);
    if (nfc_dev == NULL) {
        ret = -ENOMEM;
        goto err;
    }
    /* Memory allocation for the read from the I3C before storing it in Kbuf store*/
    nfc_dev->i3c_dev.read_hdr = NCI_HDR_LEN;
    nfc_dev->i3c_dev.read_kbuf_len = MAX_BUFFER_SIZE;
    nfc_dev->i3c_dev.read_kbuf = kzalloc(MAX_BUFFER_SIZE, GFP_KERNEL);
    if(!nfc_dev->i3c_dev.read_kbuf) {
        ret = -ENOMEM;
        goto err;
    }
    /* Kbuf memory for storing NCI/Firmware Mode Buffers before actual read from the user*/
    nfc_dev->i3c_dev.buf.kbuf = (char*)__get_free_pages(GFP_KERNEL, 0);
    if(!nfc_dev->i3c_dev.buf.kbuf) {
        ret = -ENOMEM;
        goto err;
    }
    nfc_dev->i3c_dev.buf.total_size = PAGE_SIZE;
    i3c_reset_nci_buf(&nfc_dev->i3c_dev);
    nfc_dev->platform = PLATFORM_IF_I3C;
    nfc_dev->i3c_dev.device = device;

    ret = configure_gpio(nfc_gpio.ven, GPIO_OUTPUT_HIGH);
    if (ret) {
        pr_err("%s: unable to request nfc reset gpio [%d]\n",
            __func__, nfc_gpio.ven);
        goto err;
    }
    ret = configure_gpio(nfc_gpio.dwl_req, GPIO_OUTPUT);
    if (ret) {
        pr_err("%s: unable to request nfc firm downl gpio [%d]\n",
            __func__, nfc_gpio.dwl_req);
        goto err;
    }
    nfc_dev->gpio.ven = nfc_gpio.ven;
    nfc_dev->gpio.irq = nfc_gpio.irq;
    nfc_dev->gpio.dwl_req  = -EINVAL;
    nfc_dev->gpio.ese_pwr  = -EINVAL;

    /* init mutex and queues */
    init_completion(&nfc_dev->i3c_dev.read_cplt);
    mutex_init(&nfc_dev->i3c_dev.nci_buf_mutex);
    mutex_init(&nfc_dev->dev_ref_mutex);
    ret = i3c_init_workqueue(&nfc_dev->i3c_dev);
    if(ret) {
        pr_err("%s: alloc workqueue failed\n", __func__);
        goto err_mutex_destroy;
    }
    ret = nfc_misc_register(nfc_dev, &nfc_i3c_dev_fops, DEV_COUNT,
            NFC_I3C_DEVICE_NAME, CLASS_NAME);
    if (ret) {
        pr_err("%s: nfc_misc_register failed\n", __func__);
        goto err_mutex_destroy;
    }
    i3cdev_set_drvdata(device, nfc_dev);

    ret = i3c_request_ibi(&nfc_dev->i3c_dev);
    if(ret) {
        pr_err("%s: i3c_request_ibi failed\n", __func__);
        goto err_nfc_misc_unregister;
    }
    atomic_set(&nfc_dev->i3c_dev.pm_state, PM_STATE_NORMAL);

    device_init_wakeup(&device->dev, true);
    device_set_wakeup_capable(&device->dev, true);
    /*call to platform specific probe*/
    ret = sn110_i3c_probe(nfc_dev);
    if (ret != 0) {
        pr_err("%s: probing platform failed\n", __func__);
        goto err_nfc_misc_unregister;
    };
    pr_info("%s probing nfc i3c successfully",__func__);
    return 0;
err_nfc_misc_unregister:
    nfc_misc_unregister(nfc_dev, DEV_COUNT);
err_mutex_destroy:
    mutex_destroy(&nfc_dev->dev_ref_mutex);
    mutex_destroy(&nfc_dev->i3c_dev.nci_buf_mutex);
err:
    gpio_free_all(nfc_dev);
    if(nfc_dev->i3c_dev.buf.kbuf)
        free_pages((unsigned long)nfc_dev->i3c_dev.buf.kbuf,0);
    if(nfc_dev->i3c_dev.read_kbuf)
        kfree(nfc_dev->i3c_dev.read_kbuf);
    if (nfc_dev)
        kfree(nfc_dev);
    pr_err("%s: probing not successful, check hardware\n", __func__);
    return ret;
}

/** @brief   This API is automatically called on shutdown or crash.
 *
 *  @param device   the i3c_dev for the slave.
 *  @return 0 on success, error code for failures.
 */
int nfc_i3c_dev_remove(struct i3c_device *device) {
    nfc_dev_t* nfc_dev = i3cdev_get_drvdata(device);
    i3c_dev_t *i3c_dev = NULL;
    if(!nfc_dev) {
        pr_err("%s: device doesn't exist anymore\n", __func__);
        return -ENODEV;
    }
    i3c_dev = &nfc_dev->i3c_dev;
    i3c_device_disable_ibi(device);
    i3c_device_free_ibi(device);
    if (i3c_dev->wq)
        destroy_workqueue(i3c_dev->wq);
    if(i3c_dev->buf.kbuf )
        free_pages((unsigned long)i3c_dev->buf.kbuf, 0);
    nfc_misc_unregister(nfc_dev, DEV_COUNT);
    mutex_destroy(&nfc_dev->read_mutex);
    mutex_destroy(&i3c_dev->nci_buf_mutex);
    gpio_free_all(nfc_dev);
    kfree(i3c_dev->read_kbuf);
    free_pages((unsigned long)i3c_dev->buf.kbuf,0);
    kfree(nfc_dev);
    return 0;
}

int nfc_i3c_dev_suspend(struct device *pdev)
{
    struct i3c_device* device=dev_to_i3cdev(pdev);
    nfc_dev_t *nfc_dev = i3cdev_get_drvdata(device);
    i3c_dev_t *i3c_dev = NULL;
    pr_debug("%s: enter\n", __func__);
    if (!nfc_dev) {
        pr_err("%s: device doesn't exist anymore\n", __func__);
        return -ENODEV;
    }
    i3c_dev = &nfc_dev->i3c_dev;
    atomic_set(&i3c_dev->pm_state, PM_STATE_SUSPEND);

    if (!device_may_wakeup(&device->dev)) {
        pr_debug("%s: device wakeup disabled\n", __func__);
        return 0;
    }
    i3c_enable_ibi(i3c_dev);
    return 0;
}

int nfc_i3c_dev_resume(struct device *pdev)
{
    struct i3c_device* device=dev_to_i3cdev(pdev);
    nfc_dev_t *nfc_dev = i3cdev_get_drvdata(device);
    i3c_dev_t *i3c_dev = NULL;
    pr_debug("%s: enter\n", __func__);
    if (!nfc_dev) {
        pr_err("%s: device doesn't exist anymore\n", __func__);
        return -ENODEV;
    }
    i3c_dev = &nfc_dev->i3c_dev;
    if (!device_may_wakeup(&device->dev)) {
        pr_debug("%s: device wakeup disabled\n", __func__);
        return 0;
    }
    if(atomic_read(&i3c_dev->pm_state) == PM_STATE_IBI_BEFORE_RESUME) {
        /*queue the deferered work to work queue*/
        if (!queue_work(i3c_dev->wq, &i3c_dev->work))
            dev_dbg(&device->dev, "%s: Added workqueue successfully", __func__);
    }
    atomic_set(&i3c_dev->pm_state, PM_STATE_NORMAL);

    i3c_disable_ibi(i3c_dev);
    return 0;
}

static const struct i3c_device_id nfc_i3c_dev_id[] = {
    I3C_DEVICE(NFC_I3C_MANU_ID, NFC_I3C_PART_ID, 0),
    {  },
};

struct of_device_id nfc_i3c_dev_match_table[] = {
    {.compatible = NFC_I3C_DEVICE_NAME,},
    {}
};

static const struct dev_pm_ops nfc_i3c_dev_pm_ops = {
    SET_SYSTEM_SLEEP_PM_OPS(nfc_i3c_dev_suspend, nfc_i3c_dev_resume)
};

static struct i3c_driver nfc_i3c_dev_driver = {
    .id_table = nfc_i3c_dev_id,
    .probe = nfc_i3c_dev_probe,
    .remove = nfc_i3c_dev_remove,
    .driver = {
        .owner = THIS_MODULE,
        .name = NFC_I3C_DEVICE_NAME,
        .pm = &nfc_i3c_dev_pm_ops,
        .of_match_table = nfc_i3c_dev_match_table,
    }
};
MODULE_DEVICE_TABLE(of, nfc_i3c_dev_match_table);

static int __init nfc_dev_i3c_init(void)
{
    pr_info("Loading NXP NFC I3C driver\n");
    return i3c_driver_register(&nfc_i3c_dev_driver);
}
module_init(nfc_dev_i3c_init);

static void __exit nfc_i3c_dev_exit(void)
{
    pr_info("Unloading NXP NFC I3C driver\n");
    i3c_driver_unregister(&nfc_i3c_dev_driver);
}
module_exit(nfc_i3c_dev_exit);

MODULE_DESCRIPTION("NXP NFC I3C driver");
MODULE_LICENSE("GPL");
