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
#include <linux/delay.h>
#include "common.h"
#include "sn110.h"

int sn110_i2c_probe(nfc_dev_t *nfc_dev)
{
    pr_debug("%s: enter\n", __func__);
    usleep_range(5000, 5100);
    gpio_set_value(nfc_dev->gpio.ven, 1);
    usleep_range(5000, 5100);
    nfc_dev->ven_policy = VEN_ALWAYS_ENABLED;
    return 0;
}

int sn110_i3c_probe(nfc_dev_t *nfc_dev)
{
    pr_debug("%s: enter\n", __func__);
    nfc_dev->ven_policy = VEN_ALWAYS_ENABLED;
    return 0;
}
