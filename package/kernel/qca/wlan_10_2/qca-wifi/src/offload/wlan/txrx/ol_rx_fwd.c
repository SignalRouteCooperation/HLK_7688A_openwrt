/*
 * Copyright (c) 2011-2014 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 */

/* standard header files */
#include <adf_nbuf.h>         /* adf_nbuf_map */
#include <adf_os_mem.h>       /* adf_os_mem_cmp */

/* external header files */
#include <ol_cfg.h>           /* wlan_op_mode_ap, etc. */
#include <ol_htt_rx_api.h>    /* htt_rx_msdu_desc_retrieve */

/* internal header files */
#include <ol_txrx_types.h>    /* ol_txrx_dev_t, etc. */
#include <ol_rx_fwd.h>        /* our own defs */
#include <ol_rx.h>            /* ol_rx_deliver */
#include <ol_txrx_internal.h> /* TXRX_ASSERT1 */
#include <ieee80211.h>         /* ieee80211_frame */
#include <ieee80211_var.h>     /* IEEE80211_ADDR_COPY */

static inline
void
ol_rx_fwd_to_tx(struct ol_txrx_vdev_t *vdev, adf_nbuf_t msdu)
{
    struct ol_txrx_pdev_t *pdev = vdev->pdev;
#if defined(PERE_IP_HDR_ALIGNMENT_WAR)
    if (pdev->host_80211_enable) {
	    transcap_nwifi_to_8023(msdu);
    }
#endif

    adf_nbuf_set_next(msdu, NULL); /* add NULL terminator */

    OL_VDEV_TX(vdev, msdu, pdev->osdev);
    TXRX_STATS_MSDU_INCR(vdev->pdev, rx.forwarded, msdu);
    }

void
ol_rx_fwd_check(
    struct ol_txrx_vdev_t *vdev,
    struct ol_txrx_peer_t *peer,
    unsigned tid,
    adf_nbuf_t msdu_list)
{
    struct ol_txrx_pdev_t *pdev = vdev->pdev;
    adf_nbuf_t deliver_list_head = NULL;
    adf_nbuf_t deliver_list_tail = NULL;
    adf_nbuf_t msdu;

    if (OL_CFG_RAW_RX_LIKELINESS(pdev->rx_decap_mode == htt_pkt_type_raw)) {
        /* Forwarding is not handled since keys would reside on Access
         * Controller.
         *
         * Full fledged Mixed VAP functionality can add requisite exceptions in
         * this function.
         */
        ol_rx_deliver(vdev, peer, tid, msdu_list);
        return;
    }
    
    msdu = msdu_list;
    while (msdu) {
        struct ol_txrx_vdev_t *tx_vdev;
        void *rx_desc;
        /*
         * Remember the next list elem, because our processing
         * may cause the MSDU to get linked into a different list.
         */
        msdu_list = adf_nbuf_next(msdu);

        rx_desc = htt_rx_msdu_desc_retrieve(pdev->htt_pdev, msdu);

        if (htt_rx_msdu_forward(pdev->htt_pdev, rx_desc)) {
            /*
             * Use the same vdev that received the frame to
             * transmit the frame.
             * This is exactly what we want for intra-BSS forwarding,
             * like STA-to-STA forwarding and multicast echo.
             * If this is a intra-BSS forwarding case (which is not
             * currently supported), then the tx vdev is different
             * from the rx vdev.
             * On the LL host the vdevs are not actually used for tx,
             * so it would still work to use the rx vdev rather than
             * the tx vdev.
             * For HL, the tx classification searches for the DA within
             * the given vdev, so we would want to get the DA peer ID
             * from the target, so we can locate the tx vdev.
             */
            tx_vdev = vdev;
            /*
             * This MSDU needs to be forwarded to the tx path.
             * Check whether it also needs to be sent to the OS shim,
             * in which case we need to make a copy (or clone?).
             */
            if (htt_rx_msdu_discard(pdev->htt_pdev, rx_desc)) {
                htt_rx_msdu_desc_free(pdev->htt_pdev, msdu);
                ol_rx_fwd_to_tx(tx_vdev, msdu);
                msdu = NULL; /* already handled this MSDU */
            } else {
                adf_nbuf_t copy;
                copy = adf_nbuf_copy(msdu);
                if (copy) {
                    ol_rx_fwd_to_tx(tx_vdev, copy);
                }
            }
        }
        if (msdu) {
            /* send this frame to the OS */
            OL_TXRX_LIST_APPEND(deliver_list_head, deliver_list_tail, msdu);
        }
        msdu = msdu_list;
    }
    if (deliver_list_head) {
        adf_nbuf_set_next(deliver_list_tail, NULL); /* add NULL terminator */
        ol_rx_deliver(vdev, peer, tid, deliver_list_head);
    }
}
