/*
 * Copyright (c) 2012 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 */
#ifndef _OL_TXRX__H_
#define _OL_TXRX__H_

#include <adf_nbuf.h> /* adf_nbuf_t */
#include <ol_txrx_types.h> /* ol_txrx_vdev_t, etc. */
#include <ol_tx_desc.h>

void
ol_txrx_peer_unref_delete(struct ol_txrx_peer_t *peer);


#if HOST_SW_TSO_ENABLE
extern adf_nbuf_t
ol_tx_tso_segment(ol_txrx_vdev_handle vdev, adf_nbuf_t msdu);

struct ol_tx_desc_t;
struct ol_tx_tso_desc_t;
extern void
ol_tx_tso_desc_prepare(
		struct ol_txrx_pdev_t *pdev,
		adf_nbuf_t tso_header_nbuf,
		adf_nbuf_t segment,
		struct ol_tx_desc_t *tx_desc,
		struct ol_tx_tso_desc_t *sw_tso_desc);

extern void
ol_tx_tso_failure_cleanup( struct ol_txrx_pdev_t *pdev, adf_nbuf_t segment_list);
#endif /* HOST_SW_TSO_ENABLE */

#if HOST_SW_TSO_SG_ENABLE

static inline u_int32_t
ol_tx_tso_sg_get_nbuf_len(adf_nbuf_t netbuf)
{
    struct ol_tx_tso_desc_t *sw_tso_sg_desc = (struct ol_tx_tso_desc_t *) adf_nbuf_get_fctx(netbuf);
    return (sw_tso_sg_desc->data_len + sw_tso_sg_desc->l2_l3_l4_hdr_size);
}
extern void
ol_tx_tso_sg_desc_prepare(
    struct ol_txrx_pdev_t *pdev,
    struct ol_tx_desc_t *tx_desc,
    struct ol_tx_tso_desc_t *sw_tso_sg_desc);

static inline void
ol_tx_tso_sg_desc_free(struct ol_txrx_pdev_t *pdev,
                    struct ol_tx_tso_desc_t *tx_tso_desc)
{
    struct ol_txrx_stats *stats = &pdev->stats.pub;
    adf_os_spin_lock_bh(&pdev->tx_tso_mutex);
    ((union ol_tx_tso_desc_list_elem_t *) tx_tso_desc)->next =
                                      pdev->tx_tso_desc.freelist;
    pdev->tx_tso_desc.freelist =
                (union ol_tx_tso_desc_list_elem_t *) tx_tso_desc;
    stats->tx.tso.tso_desc_cnt--;
    adf_os_spin_unlock_bh(&pdev->tx_tso_mutex);
}

#endif  /* HOST_SW_TSO_SG_ENABLE */

#if HOST_SW_SG_ENABLE

static inline void
__ol_tx_sg_desc_free(struct ol_txrx_pdev_t *pdev,
                    struct ol_tx_sg_desc_t *tx_sg_desc)
{
    struct ol_txrx_stats *stats = &pdev->stats.pub;
    adf_os_spin_lock_bh(&pdev->tx_sg_mutex);
    ((union ol_tx_sg_desc_list_elem_t *) tx_sg_desc)->next =
                                      pdev->tx_sg_desc.freelist;
    pdev->tx_sg_desc.freelist =
                (union ol_tx_sg_desc_list_elem_t *) tx_sg_desc;
    stats->tx.sg.sg_desc_cnt--;
    adf_os_spin_unlock_bh(&pdev->tx_sg_mutex);
}

#ifdef QCA_PARTNER_PLATFORM
extern void ol_tx_sg_desc_free(struct ol_txrx_pdev_t *pdev,
                                    struct ol_tx_sg_desc_t *tx_sg_desc);
#else
static inline void
ol_tx_sg_desc_free(struct ol_txrx_pdev_t *pdev,
                    struct ol_tx_sg_desc_t *tx_sg_desc)
{
    __ol_tx_sg_desc_free(pdev, tx_sg_desc);
}
#endif


#endif /* HOST_SW_SG_ENABLE */

static inline void
ol_tx_free_buf_generic(struct ol_txrx_pdev_t *pdev, adf_nbuf_t netbuf)
{
#if !ATH_SUPPORT_ME_FW_BASED
    if (adf_nbuf_get_ftype(netbuf) == CB_FTYPE_MCAST2UCAST) {
        ol_tx_me_free_buf(pdev, (struct ol_tx_me_buf_t *)adf_nbuf_get_fctx(netbuf));
    }
#endif

#if HOST_SW_TSO_SG_ENABLE
        if (adf_nbuf_get_ftype(netbuf) == CB_FTYPE_TSO_SG) {
            ol_tx_tso_sg_desc_free(pdev,(struct ol_tx_tso_desc_t *) adf_nbuf_get_fctx(netbuf));
        }
#endif

#if HOST_SW_SG_ENABLE
        if (adf_nbuf_get_ftype(netbuf) == CB_FTYPE_SG) {
            ol_tx_sg_desc_free(pdev, (struct ol_tx_sg_desc_t *) adf_nbuf_get_fctx(netbuf));
        }
#endif
        adf_nbuf_free(netbuf);
}


#endif /* _OL_TXRX__H_ */
