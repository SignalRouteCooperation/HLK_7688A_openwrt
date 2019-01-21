/*
 * Copyright (c) 2013 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 */

#ifndef REMOVE_PKT_LOG
#include "ol_txrx_types.h"
#include "ol_htt_tx_api.h"
#include "ol_tx_desc.h"
#include "adf_os_mem.h"
#include "htt.h"
#include "htt_internal.h"
#include "pktlog_ac_i.h"
#include <dbglog_host.h>
#include "pktlog_ac_fmt.h"
//#include "ratectrl_11ac.h"

#define TX_DESC_ID_LOW_MASK 0xffff
#define TX_DESC_ID_LOW_SHIFT 0
#define TX_DESC_ID_HIGH_MASK 0xffff0000
#define TX_DESC_ID_HIGH_SHIFT 16
char dbglog_print_buffer[1024];
void
pktlog_getbuf_intsafe(struct ath_pktlog_arg *plarg)
{
    struct ath_pktlog_buf *log_buf;
    int32_t buf_size;
    struct ath_pktlog_hdr *log_hdr;
    int32_t cur_wr_offset;
    char *log_ptr;
    struct ath_pktlog_info *pl_info = NULL;
    u_int16_t log_type = 0;
    size_t log_size = 0;
    uint32_t flags = 0;

    if(!plarg) {
        printk("Invalid parg in %s\n", __FUNCTION__);
        return;
    }

    pl_info  = plarg->pl_info;
    log_type = plarg->log_type;
    log_size = plarg->log_size;
    flags    = plarg->flags;
    log_buf  = pl_info->buf;

    if(!log_buf) {
        printk("Invalid log_buf in %s\n", __FUNCTION__);
        return;
    }
    buf_size = pl_info->buf_size;
    cur_wr_offset = log_buf->wr_offset;
    /* Move read offset to the next entry if there is a buffer overlap */
    if (log_buf->rd_offset >= 0) {
        if ((cur_wr_offset <= log_buf->rd_offset)
            && (cur_wr_offset + sizeof(struct ath_pktlog_hdr)) >
            log_buf->rd_offset) {
            PKTLOG_MOV_RD_IDX(log_buf->rd_offset, log_buf, buf_size);
        }
    } else {
        log_buf->rd_offset = cur_wr_offset;
    }

    log_hdr =
        (struct ath_pktlog_hdr *) (log_buf->log_data + cur_wr_offset);
    log_hdr->log_type = log_type;
    log_hdr->flags = flags;
    log_hdr->size = (u_int16_t)log_size;
    log_hdr->missed_cnt = plarg->missed_cnt;
    log_hdr->timestamp = plarg->timestamp;

    cur_wr_offset += sizeof(*log_hdr);

    if ((buf_size - cur_wr_offset) < log_size) {
        while ((cur_wr_offset <= log_buf->rd_offset)
               && (log_buf->rd_offset < buf_size)) {
            PKTLOG_MOV_RD_IDX(log_buf->rd_offset, log_buf, buf_size);
        }
        cur_wr_offset = 0;
    }

    while ((cur_wr_offset <= log_buf->rd_offset)
           && (cur_wr_offset + log_size) > log_buf->rd_offset) {
        PKTLOG_MOV_RD_IDX(log_buf->rd_offset, log_buf, buf_size);
    }

    log_ptr = &(log_buf->log_data[cur_wr_offset]);

    cur_wr_offset += log_hdr->size;

    log_buf->wr_offset =
        ((buf_size - cur_wr_offset) >=
         sizeof(struct ath_pktlog_hdr)) ? cur_wr_offset : 0;

    plarg->buf = log_ptr;
}

char *
pktlog_getbuf(ol_pktlog_dev_t *pl_dev,
                struct ath_pktlog_info *pl_info,
                size_t log_size,
                struct ath_pktlog_hdr *pl_hdr)
{
    struct ath_pktlog_arg plarg;
    uint8_t flags = 0;
    plarg.pl_info = pl_info;
    plarg.log_type = pl_hdr->log_type;
    plarg.log_size = log_size;
    plarg.flags = pl_hdr->flags;
    plarg.missed_cnt = pl_hdr->missed_cnt;
    plarg.timestamp = pl_hdr->timestamp;
    plarg.buf = NULL;

    if(flags & PHFLAGS_INTERRUPT_CONTEXT) {
        /*
         * We are already in interupt context, no need to make it intsafe
         * call the function directly.
         */
        pktlog_getbuf_intsafe(&plarg);
    }
    else {
        PKTLOG_LOCK(pl_info);
        OS_EXEC_INTSAFE(pl_dev->sc_osdev, pktlog_getbuf_intsafe, &plarg);
        PKTLOG_UNLOCK(pl_info);
    }

    return plarg.buf;
}

static struct txctl_frm_hdr frm_hdr;

static void process_ieee_hdr(void *data)
{
    uint8_t dir;
    struct ieee80211_frame *wh = (struct ieee80211_frame *)(data);
    frm_hdr.framectrl = *(u_int16_t *)(wh->i_fc);
    frm_hdr.seqctrl   = *(u_int16_t *)(wh->i_seq);
    dir = (wh->i_fc[1] & IEEE80211_FC1_DIR_MASK);
    if(dir == IEEE80211_FC1_DIR_TODS) {
        frm_hdr.bssid_tail = (wh->i_addr1[IEEE80211_ADDR_LEN-2] << 8) |
                             (wh->i_addr1[IEEE80211_ADDR_LEN-1]);
        frm_hdr.sa_tail    = (wh->i_addr2[IEEE80211_ADDR_LEN-2] << 8) |
                             (wh->i_addr2[IEEE80211_ADDR_LEN-1]);
        frm_hdr.da_tail    = (wh->i_addr3[IEEE80211_ADDR_LEN-2] << 8) |
                             (wh->i_addr3[IEEE80211_ADDR_LEN-1]);
    }
    else if(dir == IEEE80211_FC1_DIR_FROMDS) {
        frm_hdr.bssid_tail = (wh->i_addr2[IEEE80211_ADDR_LEN-2] << 8) |
                             (wh->i_addr2[IEEE80211_ADDR_LEN-1]);
        frm_hdr.sa_tail    = (wh->i_addr3[IEEE80211_ADDR_LEN-2] << 8) |
                             (wh->i_addr3[IEEE80211_ADDR_LEN-1]);
        frm_hdr.da_tail    = (wh->i_addr1[IEEE80211_ADDR_LEN-2] << 8) |
                             (wh->i_addr1[IEEE80211_ADDR_LEN-1]);
    }
    else {
        frm_hdr.bssid_tail = (wh->i_addr3[IEEE80211_ADDR_LEN-2] << 8) | 
                             (wh->i_addr3[IEEE80211_ADDR_LEN-1]);
        frm_hdr.sa_tail    = (wh->i_addr2[IEEE80211_ADDR_LEN-2] << 8) | 
                             (wh->i_addr2[IEEE80211_ADDR_LEN-1]);
        frm_hdr.da_tail    = (wh->i_addr1[IEEE80211_ADDR_LEN-2] << 8) |
                             (wh->i_addr1[IEEE80211_ADDR_LEN-1]);
    }
}

A_STATUS
process_tx_info(struct ol_txrx_pdev_t *txrx_pdev, void *data)
{
    /*
     *  Must include to process different types
     *  TX_CTL, TX_STATUS, TX_MSDU_ID, TX_FRM_HDR
     */
    ol_pktlog_dev_t *pl_dev;
    struct ath_pktlog_hdr pl_hdr;
    struct ath_pktlog_info *pl_info;
    uint32_t *pl_tgt_hdr;
    if(!txrx_pdev) {
        printk("Invalid pdev in %s\n", __FUNCTION__);
        return A_ERROR;
    }
    adf_os_assert(txrx_pdev->pl_dev);
    adf_os_assert(data);
    pl_dev = txrx_pdev->pl_dev;

    pl_tgt_hdr = (uint32_t *)data;
    /*
     * Makes the short words (16 bits) portable b/w little endian 
     * and big endian
     */ 
    pl_hdr.flags = (*(pl_tgt_hdr + ATH_PKTLOG_HDR_FLAGS_OFFSET) &
                                    ATH_PKTLOG_HDR_FLAGS_MASK) >>
                                    ATH_PKTLOG_HDR_FLAGS_SHIFT;
    pl_hdr.missed_cnt =  (*(pl_tgt_hdr + ATH_PKTLOG_HDR_MISSED_CNT_OFFSET) &
                                        ATH_PKTLOG_HDR_MISSED_CNT_MASK) >>
                                        ATH_PKTLOG_HDR_MISSED_CNT_SHIFT;
    pl_hdr.log_type =  (*(pl_tgt_hdr + ATH_PKTLOG_HDR_LOG_TYPE_OFFSET) &
                                        ATH_PKTLOG_HDR_LOG_TYPE_MASK) >>
                                        ATH_PKTLOG_HDR_LOG_TYPE_SHIFT;
    pl_hdr.size =  (*(pl_tgt_hdr + ATH_PKTLOG_HDR_SIZE_OFFSET) &
                                    ATH_PKTLOG_HDR_SIZE_MASK) >>
                                    ATH_PKTLOG_HDR_SIZE_SHIFT;
    pl_hdr.timestamp = *(pl_tgt_hdr + ATH_PKTLOG_HDR_TIMESTAMP_OFFSET);

    pl_info = pl_dev->pl_info;

    if(pl_hdr.log_type == PKTLOG_TYPE_TX_FRM_HDR) {
        /* Valid only for the TX CTL */
        process_ieee_hdr(data + sizeof(pl_hdr));
    }

    if (pl_hdr.log_type == PKTLOG_TYPE_TX_VIRT_ADDR) {
        A_UINT32 virt_addr = (A_UINT32) *((A_UINT32 *)(data + sizeof(pl_hdr)));
        wbuf_t wbuf = (wbuf_t) virt_addr;
        process_ieee_hdr(wbuf_header(wbuf));
    }

    if(pl_hdr.log_type == PKTLOG_TYPE_TX_CTRL) {
        struct ath_pktlog_txctl txctl_log;
        size_t log_size = sizeof(txctl_log.priv);
        txctl_log.txdesc_hdr_ctl = (void *)
            pktlog_getbuf(pl_dev, pl_info, log_size, &pl_hdr);

        if(!txctl_log.txdesc_hdr_ctl) {
            printk("Invalid pktlog buf in txctl %s\n", __FUNCTION__);
            return A_ERROR;
        }
        /*
         *  frm hdr is currently Valid only for local frames
         *  Add capability to include the fmr hdr for remote frames
         */
        txctl_log.priv.frm_hdr = frm_hdr;
        adf_os_assert(txctl_log.priv.txdesc_ctl);
        adf_os_mem_copy((void *)&txctl_log.priv.txdesc_ctl,
                ((void *)data + sizeof(struct ath_pktlog_hdr)),
                 pl_hdr.size);
        adf_os_mem_copy(txctl_log.txdesc_hdr_ctl,
                        &txctl_log.priv, sizeof(txctl_log.priv));
        /* Add Protocol information and HT specific information */
    }

    if (pl_hdr.log_type == PKTLOG_TYPE_TX_STAT) {
        struct ath_pktlog_tx_status txstat_log;
        size_t log_size = pl_hdr.size;
        txstat_log.ds_status = (void *)
        pktlog_getbuf(pl_dev, pl_info, log_size, &pl_hdr);

        if(!txstat_log.ds_status) {
            printk("Invalid pktlog buf in txstat %s\n", __FUNCTION__);
            return A_ERROR;
        }

        adf_os_mem_copy(txstat_log.ds_status,
                ((void *)data + sizeof(struct ath_pktlog_hdr)),
                 pl_hdr.size);
    }

    if (pl_hdr.log_type == PKTLOG_TYPE_TX_MSDU_ID) {
        struct ath_pktlog_msdu_info pl_msdu_info;
        uint32_t i;
        uint32_t *htt_tx_desc;
        size_t log_size;
        struct ol_tx_desc_t *tx_desc;
        uint8_t msdu_id_offset = MSDU_ID_INFO_ID_OFFSET;
        uint16_t tx_desc_id;
        uint32_t *msdu_id_info = (uint32_t *)
                             ((void *)data + sizeof(struct ath_pktlog_hdr));
        uint32_t *msdu_id = (uint32_t *)((char *)msdu_id_info +
                                                msdu_id_offset);
        u_int8_t *addr, *vap_addr;
        u_int8_t vdev_id;
        adf_nbuf_t netbuf;
        u_int32_t len;

        pl_msdu_info.num_msdu = *msdu_id_info;
        pl_msdu_info.priv_size =
                        sizeof(uint32_t) * pl_msdu_info.num_msdu +
                                                        sizeof(uint32_t);
        log_size = sizeof(pl_msdu_info.priv);
        
        for (i = 0; i < pl_msdu_info.num_msdu; i++) {
            /*
             *  Handle big endianess
             *  Increment msdu_id once after retrieving
             *  lower 16 bits and uppper 16 bits
             */
            if (!(i % 2)) {
                tx_desc_id = ((*msdu_id & TX_DESC_ID_LOW_MASK)
                                        >> TX_DESC_ID_LOW_SHIFT);
            } else {
                tx_desc_id = ((*msdu_id & TX_DESC_ID_HIGH_MASK)
                                        >> TX_DESC_ID_HIGH_SHIFT);
                msdu_id += 1;
            }
            tx_desc = ol_tx_desc_find(txrx_pdev, tx_desc_id);
            adf_os_assert(tx_desc);
            netbuf = tx_desc->netbuf;
            htt_tx_desc = (uint32_t *) tx_desc->htt_tx_desc;
            adf_os_assert(htt_tx_desc);
            if(!(pl_hdr.flags & (1 << PKTLOG_FLG_FRM_TYPE_CLONE_S))){
                adf_nbuf_peek_header(netbuf, &addr, &len);
                if (len < (2 * IEEE80211_ADDR_LEN)) {
                    adf_os_print("TX frame does not have a valid address %d \n",len);
                    return -1;
                }
                /* Adding header information for the TX data frames */
                frm_hdr.da_tail    = (addr[IEEE80211_ADDR_LEN-2] << 8) |
                                     (addr[IEEE80211_ADDR_LEN-1]);
                frm_hdr.sa_tail    = (addr[2 * IEEE80211_ADDR_LEN-2] << 8) |
                                     (addr[2 * IEEE80211_ADDR_LEN-1]);
            
                vdev_id = (u_int8_t)(*(htt_tx_desc + HTT_TX_VDEV_ID_WORD) >>
                                       HTT_TX_VDEV_ID_SHIFT) &
                                       HTT_TX_VDEV_ID_MASK;
                vap_addr = ol_ath_vap_get_myaddr((struct ol_ath_softc_net80211 *)pl_dev->scn, vdev_id);

                if (vap_addr) {
                    frm_hdr.bssid_tail = (vap_addr[IEEE80211_ADDR_LEN-2] << 8) |
                                         (vap_addr[IEEE80211_ADDR_LEN-1]);
                } else {
                    frm_hdr.bssid_tail = 0x0000;
                }
                pl_msdu_info.priv.msdu_len[i] = *(htt_tx_desc +
                                            HTT_TX_MSDU_LEN_DWORD)
                                            & HTT_TX_MSDU_LEN_MASK;
            } else {
                /* This is for cloned packet init with some constants*/ 
                frm_hdr.da_tail = 0x0000;
                frm_hdr.sa_tail = 0x0000;
                frm_hdr.bssid_tail = 0x0000;
                pl_msdu_info.priv.msdu_len[i] = 200;
            }
        }
            
         /*
          * Add more information per MSDU
          * e.g., protocol information
          */
        pl_msdu_info.ath_msdu_info = pktlog_getbuf(pl_dev, pl_info,
                                                log_size, &pl_hdr);
        if(!pl_msdu_info.ath_msdu_info) {
            printk("Invalid pktlog buf in msdu_info %s\n", __FUNCTION__);
            return A_ERROR;
        }

        adf_os_mem_copy((void *)&pl_msdu_info.priv.msdu_id_info,
                    ((void *)data + sizeof(struct ath_pktlog_hdr)),
                            sizeof(pl_msdu_info.priv.msdu_id_info));
        adf_os_mem_copy(pl_msdu_info.ath_msdu_info, &pl_msdu_info.priv,
                                        sizeof(pl_msdu_info.priv));
    }
    return A_OK;
}

A_STATUS
process_rx_info_remote(void *pdev, adf_nbuf_t amsdu)
{
    ol_pktlog_dev_t *pl_dev;
    struct ath_pktlog_info *pl_info;
    struct htt_host_rx_desc_base *rx_desc;
    struct ath_pktlog_hdr pl_hdr;
    struct ath_pktlog_rx_info rxstat_log;
    size_t log_size;
    adf_nbuf_t msdu;
    if(!pdev) {
        printk("Invalid pdev in %s\n",
                __FUNCTION__);
        return A_ERROR;
    }
    if(!amsdu) {
        printk("Invalid data in %s\n",
                __FUNCTION__);
        return A_ERROR;
    }
    pl_dev = ((struct ol_txrx_pdev_t *) pdev)->pl_dev;
    pl_info = pl_dev->pl_info;
    msdu = amsdu;
    while (msdu) {
        rx_desc = (struct htt_host_rx_desc_base *)(
                            adf_nbuf_data(msdu)) - 1;
        log_size = sizeof(*rx_desc) -
                sizeof(struct htt_host_fw_desc_base);

        /*
         * Construct the pktlog header pl_hdr
         * Because desc is DMA'd to the host memory
         */
        pl_hdr.flags = (1 << PKTLOG_FLG_FRM_TYPE_REMOTE_S);
        pl_hdr.missed_cnt = 0;
        pl_hdr.log_type = PKTLOG_TYPE_RX_STAT;
        pl_hdr.size = sizeof(*rx_desc) -
            sizeof(struct htt_host_fw_desc_base);
        pl_hdr.timestamp = rx_desc->ppdu_end.tsf_timestamp;
        rxstat_log.rx_desc = (void *)
        pktlog_getbuf(pl_dev, pl_info, log_size, &pl_hdr);

        if(!rxstat_log.rx_desc) {
            printk("Invalid pktlog buf in rxstat %s\n", __FUNCTION__);
            return A_ERROR;
        }

        adf_os_mem_copy(rxstat_log.rx_desc, (void *)rx_desc +
                sizeof(struct htt_host_fw_desc_base), pl_hdr.size);
        msdu = adf_nbuf_next(msdu);
    }
    return A_OK;
}

A_STATUS
process_rx_info(void *pdev, void *data)
{
    ol_pktlog_dev_t *pl_dev;
    htt_pdev_handle htt_pdev;
    struct ath_pktlog_info *pl_info;
    struct ath_pktlog_rx_info rxstat_log;
    struct ath_pktlog_hdr pl_hdr;
    size_t log_size;
    uint32_t *pl_tgt_hdr;
    if (!pdev) {
        printk("Invalid pdev in %s",
                __FUNCTION__);
        return A_ERROR;
    }
    pl_dev = ((struct ol_txrx_pdev_t *) pdev)->pl_dev;
    htt_pdev = ((struct ol_txrx_pdev_t *) pdev)->htt_pdev;
    pl_info = pl_dev->pl_info;
    pl_tgt_hdr = (uint32_t *)data;
    pl_hdr.flags = (*(pl_tgt_hdr + ATH_PKTLOG_HDR_FLAGS_OFFSET) &
                                    ATH_PKTLOG_HDR_FLAGS_MASK) >>
                                    ATH_PKTLOG_HDR_FLAGS_SHIFT;
    pl_hdr.missed_cnt =  (*(pl_tgt_hdr + ATH_PKTLOG_HDR_MISSED_CNT_OFFSET) &
                                        ATH_PKTLOG_HDR_MISSED_CNT_MASK) >>
                                        ATH_PKTLOG_HDR_MISSED_CNT_SHIFT;
    pl_hdr.log_type =  (*(pl_tgt_hdr + ATH_PKTLOG_HDR_LOG_TYPE_OFFSET) &
                                        ATH_PKTLOG_HDR_LOG_TYPE_MASK) >>
                                        ATH_PKTLOG_HDR_LOG_TYPE_SHIFT;
    pl_hdr.size =  (*(pl_tgt_hdr + ATH_PKTLOG_HDR_SIZE_OFFSET) &
                                    ATH_PKTLOG_HDR_SIZE_MASK) >>
                                    ATH_PKTLOG_HDR_SIZE_SHIFT;
    pl_hdr.timestamp = *(pl_tgt_hdr + ATH_PKTLOG_HDR_TIMESTAMP_OFFSET);
    log_size = pl_hdr.size;
    rxstat_log.rx_desc = (void *)
        pktlog_getbuf(pl_dev, pl_info, log_size, &pl_hdr);

    if(!rxstat_log.rx_desc) {
	    printk("Invalid pktlog buf in rxstat %s\n", __FUNCTION__);
	    return A_ERROR;
    }

    adf_os_mem_copy(rxstat_log.rx_desc,
                    (void *)data + sizeof(struct ath_pktlog_hdr),
                    pl_hdr.size);

    return A_OK;
}

A_STATUS
process_rate_find(void *pdev, void *data)
{
    ol_pktlog_dev_t *pl_dev;
    struct ath_pktlog_hdr pl_hdr;
    struct ath_pktlog_info *pl_info;
    size_t log_size;

    /*
     * Will be uncommented when the rate control find
     * for pktlog is implemented in the firmware.
     * Currently derived from the TX PPDU status
     */
    struct ath_pktlog_rc_find rcf_log;
    uint32_t *pl_tgt_hdr;
    if(!pdev) {
        adf_os_print("Invalid pdev in %s\n", __FUNCTION__);
        return A_ERROR;
    }
    if(!data) {
        adf_os_print("Invalid data in %s\n", __FUNCTION__);
        return A_ERROR;
    }

    pl_tgt_hdr = (uint32_t *)data;
    /*
     * Makes the short words (16 bits) portable b/w little endian 
     * and big endian
     */ 
    pl_hdr.flags      = (*(pl_tgt_hdr + ATH_PKTLOG_HDR_FLAGS_OFFSET) &
                                        ATH_PKTLOG_HDR_FLAGS_MASK) >>
                                        ATH_PKTLOG_HDR_FLAGS_SHIFT;
    pl_hdr.missed_cnt =  (*(pl_tgt_hdr + ATH_PKTLOG_HDR_MISSED_CNT_OFFSET) &
                                        ATH_PKTLOG_HDR_MISSED_CNT_MASK) >>
                                        ATH_PKTLOG_HDR_MISSED_CNT_SHIFT;
    pl_hdr.log_type   =  (*(pl_tgt_hdr + ATH_PKTLOG_HDR_LOG_TYPE_OFFSET) &
                                        ATH_PKTLOG_HDR_LOG_TYPE_MASK) >>
                                        ATH_PKTLOG_HDR_LOG_TYPE_SHIFT;
    pl_hdr.size       =  (*(pl_tgt_hdr + ATH_PKTLOG_HDR_SIZE_OFFSET) &
                                        ATH_PKTLOG_HDR_SIZE_MASK) >>
                                        ATH_PKTLOG_HDR_SIZE_SHIFT;
    //adf_os_print("Size of the RCF:%d\n", pl_hdr.size);
    pl_hdr.timestamp  = *(pl_tgt_hdr + ATH_PKTLOG_HDR_TIMESTAMP_OFFSET);
    pl_dev            = ((struct ol_txrx_pdev_t *) pdev)->pl_dev;
    pl_info           = pl_dev->pl_info;
    log_size          = pl_hdr.size;
    rcf_log.rcFind    = (void *) pktlog_getbuf(pl_dev, pl_info,
                                                log_size, &pl_hdr);
    if(!rcf_log.rcFind) {
	    printk("Invalid pktlog buf in rcf_log %s\n", __FUNCTION__);
	    return A_ERROR;
    }

    adf_os_mem_copy(rcf_log.rcFind,
            ((char *)data + sizeof(struct ath_pktlog_hdr)), pl_hdr.size);

    return A_OK;
}

A_STATUS
process_rate_update(void *pdev, void *data)
{
    ol_pktlog_dev_t *pl_dev;
    struct ath_pktlog_hdr pl_hdr;
    size_t log_size;
    struct ath_pktlog_info *pl_info;
    struct ath_pktlog_rc_update rcu_log;
    uint32_t *pl_tgt_hdr;

    if(!pdev) {
        printk("Invalid pdev in %s\n", __FUNCTION__);
        return A_ERROR;
    }
    if(!data) {
        printk("Invalid data in %s\n", __FUNCTION__);
        return A_ERROR;
    }
    pl_tgt_hdr = (uint32_t *)data;
    /*
     * Makes the short words (16 bits) portable b/w little endian 
     * and big endian
     */
    pl_hdr.flags = (*(pl_tgt_hdr + ATH_PKTLOG_HDR_FLAGS_OFFSET) &
                                    ATH_PKTLOG_HDR_FLAGS_MASK) >>
                                    ATH_PKTLOG_HDR_FLAGS_SHIFT;
    pl_hdr.missed_cnt =  (*(pl_tgt_hdr + ATH_PKTLOG_HDR_MISSED_CNT_OFFSET) &
                                        ATH_PKTLOG_HDR_MISSED_CNT_MASK) >>
                                        ATH_PKTLOG_HDR_MISSED_CNT_SHIFT;
    pl_hdr.log_type =  (*(pl_tgt_hdr + ATH_PKTLOG_HDR_LOG_TYPE_OFFSET) &
                                        ATH_PKTLOG_HDR_LOG_TYPE_MASK) >>
                                        ATH_PKTLOG_HDR_LOG_TYPE_SHIFT;
    pl_hdr.size =  (*(pl_tgt_hdr + ATH_PKTLOG_HDR_SIZE_OFFSET) &
                                    ATH_PKTLOG_HDR_SIZE_MASK) >>
                                    ATH_PKTLOG_HDR_SIZE_SHIFT;
    pl_hdr.timestamp = *(pl_tgt_hdr + ATH_PKTLOG_HDR_TIMESTAMP_OFFSET);
    pl_dev = ((struct ol_txrx_pdev_t *) pdev)->pl_dev;
    log_size = pl_hdr.size;
    pl_info = pl_dev->pl_info;

    /*
     * Will be uncommented when the rate control update
     * for pktlog is implemented in the firmware.
     * Currently derived from the TX PPDU status
     */
    rcu_log.txRateCtrl = (void *)
                    pktlog_getbuf(pl_dev, pl_info, log_size, &pl_hdr);

    if(!rcu_log.txRateCtrl) {
        printk("Invalid pktlog buf in rcu_log %s\n", __FUNCTION__);
        return A_ERROR;
    }

    adf_os_mem_copy(rcu_log.txRateCtrl,
            ((char *)data + sizeof(struct ath_pktlog_hdr)), pl_hdr.size);
    return A_OK;
}

A_STATUS
process_dbg_print(void *pdev, void *data)
{
    ol_pktlog_dev_t *pl_dev;
    struct ath_pktlog_hdr pl_hdr;
    size_t log_size;
    struct ath_pktlog_info *pl_info;
    struct ath_pktlog_dbg_print dbg_print_s;
    uint32_t *pl_tgt_hdr;

    if (!pdev) {
        adf_os_print("Invalid pdev in %s\n", __FUNCTION__);
        return A_ERROR;
    }
    if (!data) {
        adf_os_print("Invalid data in %s\n", __FUNCTION__);
        return A_ERROR;
    }

    pl_tgt_hdr = (uint32_t *)data;
    /*
     * Makes the short words (16 bits) portable b/w little endian 
     * and big endian
     */
    pl_hdr.flags = (*(pl_tgt_hdr + ATH_PKTLOG_HDR_FLAGS_OFFSET) &
                                    ATH_PKTLOG_HDR_FLAGS_MASK) >>
                                    ATH_PKTLOG_HDR_FLAGS_SHIFT;
    pl_hdr.missed_cnt =  (*(pl_tgt_hdr + ATH_PKTLOG_HDR_MISSED_CNT_OFFSET) &
                                        ATH_PKTLOG_HDR_MISSED_CNT_MASK) >>
                                        ATH_PKTLOG_HDR_MISSED_CNT_SHIFT;
    pl_hdr.log_type =  (*(pl_tgt_hdr + ATH_PKTLOG_HDR_LOG_TYPE_OFFSET) &
                                        ATH_PKTLOG_HDR_LOG_TYPE_MASK) >>
                                        ATH_PKTLOG_HDR_LOG_TYPE_SHIFT;
    pl_hdr.size =  (*(pl_tgt_hdr + ATH_PKTLOG_HDR_SIZE_OFFSET) &
                                    ATH_PKTLOG_HDR_SIZE_MASK) >>
                                    ATH_PKTLOG_HDR_SIZE_SHIFT;
    pl_hdr.timestamp = *(pl_tgt_hdr + ATH_PKTLOG_HDR_TIMESTAMP_OFFSET);
    pl_dev = ((struct ol_txrx_pdev_t *) pdev)->pl_dev;
    pl_info = pl_dev->pl_info;

    adf_os_mem_set(dbglog_print_buffer, 0, sizeof(dbglog_print_buffer));
    dbglog_parse_debug_logs((struct ol_ath_softc_net80211 *)(((struct ol_txrx_pdev_t *) pdev)->ctrl_pdev),
                            (char *)data + sizeof(pl_hdr), pl_hdr.size,
                            (void *)DBGLOG_PRT_PKTLOG);
    log_size = strlen(dbglog_print_buffer);
    dbg_print_s.dbg_print = (void *)
                    pktlog_getbuf(pl_dev, pl_info, log_size, &pl_hdr);

    if(!dbg_print_s.dbg_print) {
	    printk("Invalid pktlog buf in dbg_print %s\n", __FUNCTION__);
	    return A_ERROR;
    }

    adf_os_mem_copy(dbg_print_s.dbg_print, dbglog_print_buffer, log_size);
    return A_OK;
}
#endif /*REMOVE_PKT_LOG*/
