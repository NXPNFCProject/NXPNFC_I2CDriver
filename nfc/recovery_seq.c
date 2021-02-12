/******************************************************************************
 *  Copyright (C) 2021 NXP
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
#if defined(RECOVERY_ENABLE)
#include "recovery_seq.h"

recovery_info_t g_recovery_info;
recovery_frame_t g_recovery_frame;

extern const uint32_t gphDnldNfc_DlSeqSz;     /* Recovery user buffer size */
extern const uint8_t gphDnldNfc_DlSequence[]; /* Recovery user buffer */

/** @brief Function to calculate crc value.
 *
 *  @param   pbuffer:  input buffer for crc calculation.
 *           dwLength: length of input buffer
 *  @return  calculated uint16_t crc valueof input buffer.
 */
static uint16_t calcCrc16(uint8_t* pbuffer, uint32_t dwLength) {
	uint32_t i = 0;
	uint16_t crc_new = 0;
	uint16_t crc = DL_INVALID_CRC_VALUE;
	if(NULL == pbuffer) {
		pr_err("%s, invalid params", __func__);
		return crc;
	}
	for (i = 0; i < dwLength; i++) {
		crc_new = (uint8_t)(crc >> MSB_POS) | (crc << MSB_POS );
		crc_new ^= pbuffer[i];
		crc_new ^= (uint8_t)(crc_new & DL_CRC_MASK) >> 4;
		crc_new ^= crc_new << 12;
		crc_new ^= (crc_new & DL_CRC_MASK) << 5;
		crc = crc_new;
	}
	return crc;
}

/** @brief Function to build command frame for recover.
 *
 *  @return  status code of recovery_status_t type.
 */
static recovery_status_t build_cmd_frame() {
	uint16_t len = 0;
	uint16_t wCrc = 0;
	uint16_t writeOffset = 0;
	pr_debug(" %s Entry", __func__);
	if(gphDnldNfc_DlSeqSz == 0) {
		pr_err(" %s invalid params", __func__);
		return STATUS_FAILED;
	}
	memset(g_recovery_frame.p_buffer, 0x00, MAX_FRAME_SIZE);
	g_recovery_frame.len = 0;
	if(g_recovery_info.bFrameSegmented == false) {
		len = gphDnldNfc_DlSequence[g_recovery_info.currentReadOffset];
		len <<= MSB_POS;
		len |= gphDnldNfc_DlSequence[g_recovery_info.currentReadOffset + 1];
	} else {
		/* last frame was segmented frame
		* read length reamaining length */
		len = g_recovery_info.wRemChunkBytes;
	}
	if(len > MAX_DATA_SIZE) {
		/* set remaining chunk */
		g_recovery_info.wRemChunkBytes = (len - MAX_DATA_SIZE);
		len = MAX_DATA_SIZE;
		/* set chunk bit to write in header */
		len = DL_SET_HDR_FRAGBIT(len);
		g_recovery_frame.p_buffer[writeOffset++] = (len >> MSB_POS) & SHIFT_MASK;
		g_recovery_frame.p_buffer[writeOffset++] = len & SHIFT_MASK;
		/* clear chunk bit for length variable */
		len = DL_CLR_HDR_FRAGBIT(len);
		/* first chunk of segmented frame*/
		if(!g_recovery_info.bFrameSegmented) {
			/* ignore header from user buffer */
			g_recovery_info.currentReadOffset += FW_HDR_LEN;
			g_recovery_info.remBytes -= FW_HDR_LEN;
		}
		g_recovery_frame.len += FW_HDR_LEN;
		g_recovery_info.bFrameSegmented = true;
	} else {
		/* last chunk of segmented frame */
		if(g_recovery_info.bFrameSegmented) {
			/* write header with user chunk length */
			g_recovery_frame.p_buffer[writeOffset++] = (len >> MSB_POS) & SHIFT_MASK;
			g_recovery_frame.p_buffer[writeOffset++] = len & SHIFT_MASK;
			g_recovery_frame.len += FW_HDR_LEN;
		} else {
			/* normal Frame with in supported size increase
			* len to read header from user data */
			len += FW_HDR_LEN;
		}
		g_recovery_info.wRemChunkBytes = 0;
		g_recovery_info.bFrameSegmented = false;
	}
	if(((writeOffset + len) > MAX_FRAME_SIZE) ||
		((g_recovery_info.currentReadOffset + len) > gphDnldNfc_DlSeqSz)) {
		pr_err("%s frame offsets out of bound",__func__);
		return STATUS_FAILED;
	}
	memcpy(&g_recovery_frame.p_buffer[writeOffset],
		&gphDnldNfc_DlSequence[g_recovery_info.currentReadOffset], len);
	g_recovery_info.currentReadOffset += len;
	g_recovery_frame.len += len;
	writeOffset += len;
	g_recovery_info.remBytes -= len;
	wCrc = calcCrc16(g_recovery_frame.p_buffer,
		g_recovery_frame.len);
	g_recovery_frame.p_buffer[writeOffset++] = (wCrc >> MSB_POS) & SHIFT_MASK;
	g_recovery_frame.p_buffer[writeOffset++] = wCrc & SHIFT_MASK;
	g_recovery_frame.len += FW_CRC_LEN;
	return STATUS_SUCCESS;
}

/** @brief Function to tramsmit recovery frame.
 *  @param   nfc_dev nfc driver object.
 *  @return  status code of recovery_status_t type.
 */
static recovery_status_t transmit(nfc_dev_t* nfc_dev) {
	char rsp_buf[MAX_BUFFER_SIZE];
	int ret = 0;
	int frame_resp_len = 0;
	uint16_t respCRC = 0;
	uint16_t respCRCOffset = 0;

	pr_debug("%s Entry", __func__);
	if(NULL == nfc_dev || g_recovery_frame.len <= 0) {
		pr_err("%s invalid Params ", __func__);
		return STATUS_FAILED;
	}
	ret = nfc_dev->nfc_write(nfc_dev, g_recovery_frame.p_buffer,
		g_recovery_frame.len, MAX_RETRY_COUNT);
	if (ret <= 0) {
		pr_err(" %s: Write recovery frame error %d\n", __func__, ret);
		return STATUS_FAILED;
	}
	pr_debug(" %s Reading response \n", __func__);
	memset(rsp_buf, 0x00, MAX_BUFFER_SIZE);
	ret = nfc_dev->nfc_read(nfc_dev, rsp_buf, FW_HDR_LEN, NCI_CMD_RSP_TIMEOUT);
	if (ret < FW_HDR_LEN) {
		pr_err(" %s - Read recovery frame response error ret %d\n", __func__, ret);
		return STATUS_FAILED;
	}
	if( rsp_buf[0] != FW_MSG_CMD_RSP ||
		rsp_buf[DL_FRAME_RESP_LEN_OFFSET] != DL_FRAME_RESP_LEN) {
	  pr_err("%s, invalid response", __func__);
	  return STATUS_FAILED;
	}
	frame_resp_len = rsp_buf[DL_FRAME_RESP_LEN_OFFSET] + FW_CRC_LEN;
	ret = nfc_dev->nfc_read(nfc_dev, rsp_buf + FW_HDR_LEN, frame_resp_len, NCI_CMD_RSP_TIMEOUT);
	if (ret < frame_resp_len) {
		pr_err(" %s - Read recovery frame response error ret %d\n", __func__, ret);
		return STATUS_FAILED;
	}
	pr_debug(" %s: recovery frame Response 0x%02x  0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
		__func__, rsp_buf[0], rsp_buf[1],rsp_buf[2], rsp_buf[3], rsp_buf[4], rsp_buf[5]);
	respCRCOffset = FW_HDR_LEN + rsp_buf[DL_FRAME_RESP_LEN_OFFSET];
	respCRC = rsp_buf[respCRCOffset++];
	respCRC <<= MSB_POS;
	respCRC |= rsp_buf[respCRCOffset];
	if(respCRC != calcCrc16(rsp_buf, DL_FRAME_RESP_LEN + FW_HDR_LEN)) {
		pr_err("%s, invalid response crc", __func__);
		return STATUS_FAILED;
	}
	if(g_recovery_info.bFrameSegmented &&
		(rsp_buf[DL_FRAME_RESP_STAT_OFFSET] != DL_SEGMENTED_FRAME_RESP_STAT1
		 && rsp_buf[DL_FRAME_RESP_STAT_OFFSET] != DL_SEGMENTED_FRAME_RESP_STAT2)) {
		pr_err("%s, invalid stat flag in chunk response", __func__);
		return STATUS_FAILED;
	}
	if(!g_recovery_info.bFrameSegmented &&
		rsp_buf[DL_FRAME_RESP_STAT_OFFSET] != DL_NON_SEGMENTED_FRAME_RESP_STAT) {
		pr_err("%s, invalid stat flag in response", __func__);
		return STATUS_FAILED;
	}
	return STATUS_SUCCESS;
}

/** @brief Function to detect the fw state and print.
 *  @param   nfc_dev nfc driver object.
 *  @return  no return.
 */
static void detect_fw_state(nfc_dev_t* nfc_dev) {
	const char get_session_state_cmd[] = { 0x00, 0x04, 0xF2, 0x00, 0x00, 0x00, 0xF5, 0x33 };
	char rsp_buf[MAX_BUFFER_SIZE];
	int ret = 0;
	pr_debug("%s:Sending GET_SESSION_STATE cmd \n", __func__);
	ret = nfc_dev->nfc_write(nfc_dev, get_session_state_cmd,
		sizeof(get_session_state_cmd), MAX_RETRY_COUNT);
	if (ret <= 0) {
		pr_err("%s: - nfc get session state cmd err ret %d\n", __func__, ret);
		return;
	}
	memset(rsp_buf, 0x00, DL_GET_SESSION_STATE_RSP_LEN);
	pr_debug("%s: Reading response of GET_SESSION_STATE cmd\n", __func__);
	ret = nfc_dev->nfc_read(nfc_dev, rsp_buf, DL_GET_SESSION_STATE_RSP_LEN, NCI_CMD_RSP_TIMEOUT);
	if (ret <= 0) {
		pr_err("%s: - nfc get session state rsp err %d\n", __func__, ret);
		return;
	}
	pr_debug("Response bytes are %02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x",
		rsp_buf[0], rsp_buf[1], rsp_buf[2], rsp_buf[3], rsp_buf[4], rsp_buf[5],
			rsp_buf[6], rsp_buf[7]);
	/*verify fw in non-teared state */
	if (rsp_buf[GET_SESSION_STS_OFF] != NFCC_SESSION_STS_CLOSED) {
		pr_err("%s NFCC is in teared state %d\n", __func__, __LINE__);
	} else {
		pr_info("%s NFCC is in recoverd state %d\n", __func__, __LINE__);
	}
}

/** @brief Function to check input version with recovery fw version.
 *  @param   fw_major_version: input major_version to check.
 *  @return  true if input major_version matches with recovery fw major version
 *           otherwise returns false.
 */
static bool check_major_version(uint8_t fw_major_version) {
	if(gphDnldNfc_DlSeqSz < RECOVERY_FW_MJ_VER_OFFSET) {
		/* Recovery data corrupted */
		pr_err("%s Not able to extract major version from recovery fw\n", __func__);
		return false;
	}
	return (fw_major_version == gphDnldNfc_DlSequence[RECOVERY_FW_MJ_VER_OFFSET]);
}

/** @brief   Function to recover the nfcc.
 *  @param   nfc_dev nfc driver object.
 *  @return status code of type recovery_status_t.
 */
recovery_status_t do_recovery(nfc_dev_t *nfc_dev) {
	recovery_status_t status = STATUS_SUCCESS;
	g_recovery_info.remBytes = gphDnldNfc_DlSeqSz;
	g_recovery_info.currentReadOffset = 0;
	g_recovery_info.bFrameSegmented = false;
	g_recovery_info.wRemChunkBytes = 0;

	pr_debug("%s Entry", __func__);
	if(NULL == nfc_dev) {
		pr_err("%s invalid params ", __func__);
		return STATUS_FAILED;
	}
	if(!nfc_dev->recovery_required
		|| !(check_major_version(nfc_dev->fw_major_version))) {
		pr_err("%s recovery not required or unsupported version", __func__);
		status = STATUS_FAILED;
		goto EXIT_RECOVERY;
	}
	while(g_recovery_info.remBytes > 0) {
		status = build_cmd_frame();
		if(status != STATUS_SUCCESS) {
			pr_err(" %s Unable to create recovery frame");
			break;
		}
		status = transmit(nfc_dev);
		if(status != STATUS_SUCCESS) {
			pr_err(" %s Unable to send recovery frame");
			break;
		}
	}
EXIT_RECOVERY:
	detect_fw_state(nfc_dev);
	/*set NCI mode for i2c products with dwl pin */
	enable_dwnld_mode(nfc_dev, false);
	pr_info("%s Recovery done status %d", __func__, status);
	return status;
}
#endif
