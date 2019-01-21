/*
 * Copyright (c) 2011-2014 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 */
#include <ol_cfg.h>
#include <ol_if_athvar.h>
#include <ol_ath.h>
/* FIX THIS - 
 * For now, all these configuration parameters are hardcoded.
 * Many of these should actually be determined dynamically instead.
 */

#if defined (ATHR_WIN_NWF)
#include <wbuf_private.h>
#pragma warning(disable:4100)   // unreferenced formal parameter
#endif

#if defined(CONFIG_HL_SUPPORT)
#include "wlan_tgt_def_config_hl.h"
#else
#include "wlan_tgt_def_config.h"
#endif

int ol_cfg_is_high_latency(ol_pdev_handle pdev)
{
/* FIX THIS: later the selection of high-latency vs. low-latency 
 * will need to be done at run-time rather than compile-time
 */
#if defined(CONFIG_HL_SUPPORT)
    return 1; 
#else
    return 0;
#endif
}

int ol_cfg_max_peer_id(ol_pdev_handle pdev)
{
    /* 
     * TBDXXX - this value must match the peer table 
     * size allocated in FW 
     */
    int peer_id, max_peer_id = 1;
    /* unfortunately, when we allocate the peer_id_to_obj_map, 
     * we don't know yet if LARGE_AP is enabled or not. so, we
     * assume the worst and allocate more (PEERS_MAX instead 
     * of PEERS).
     */
    peer_id = (CFG_TGT_NUM_PEERS_MAX + CFG_TGT_NUM_VDEV + 1 + 
               CFG_TGT_WDS_ENTRIES) * CFG_TGT_NUM_PEER_AST;
    /* nearest power of 2 - as per FW algo */
    while (peer_id > max_peer_id) {
        max_peer_id <<= 1;
    }
    return max_peer_id + CFG_TGT_AST_SKID_LIMIT - 1;
}

int ol_cfg_max_vdevs(ol_pdev_handle pdev)
{
    return CFG_TGT_NUM_VDEV;
}

int ol_cfg_rx_pn_check(ol_pdev_handle pdev)
{
    return 1;
}

int ol_cfg_rx_fwd_check(ol_pdev_handle pdev)
{
    return 1;
}

int ol_cfg_rx_fwd_inter_bss(ol_pdev_handle pdev)
{
    return 0;
}

enum wlan_frm_fmt ol_cfg_frame_type(ol_pdev_handle pdev)
{
#ifdef ATHR_WIN_NWF
    return wlan_frm_fmt_native_wifi;
#else
    return wlan_frm_fmt_802_3;
#endif
}

enum htt_pkt_type ol_cfg_pkt_type_init(ol_pdev_handle pdev)
{
#if defined(ATHR_WIN_NWF)
    /* Note: In the case of PERE_IP_HDR_ALIGNMENT_WAR, we later examine the
     * relevant driver load time parameter to determine if Native Wi-Fi is to be
     * used. We do not enforce any compile time decisions here.
     */
    return htt_pkt_type_native_wifi;
#else
    return htt_pkt_type_ethernet;
#endif
    /* Note: Raw mode is enabled post initialization, via an IOCTL call in line
     * with FW expectations. */
}

int ol_cfg_max_thruput_mbps(ol_pdev_handle pdev)
{
    return 800;
}

int ol_cfg_netbuf_frags_max(ol_pdev_handle pdev)
{
#ifdef ATHR_WIN_NWF
    return CVG_NBUF_MAX_FRAGS;
#else
#if QCA_OL_SUPPORT_RAWMODE_TXRX && (QCA_RAWMODE_OPTIMIZATION_CONFIG > 0)
    return QCA_OL_11AC_TX_MAX_FRAGS - 1;
#else
    return 1;
#endif /* QCA_OL_SUPPORT_RAWMODE_TXRX && (QCA_RAWMODE_OPTIMIZATION_CONFIG > 0) */
#endif
}

int ol_cfg_tx_free_at_download(ol_pdev_handle pdev)
{
    return 0;
}

int ol_cfg_target_tx_credit(ol_pdev_handle pdev)
{
  struct ol_ath_softc_net80211 *scn = (struct ol_ath_softc_net80211 *)pdev;
  u_int16_t vow_max_sta = ((scn->vow_config) & 0xffff0000) >> 16;
  u_int16_t vow_max_desc_persta = ((scn->vow_config) & 0x0000ffff);

#if defined(CONFIG_HL_SUPPORT)
    return CFG_TGT_NUM_MSDU_DESC;   /* FIX THIS : not HTT_DATA_SVC_PIPE_DEPTH - 4, should be HTTX_GUARANTEED_BUFS - 4 */
#else
    /*
     * FIX THIS: remove this limitation for LL, when the target supports
     * rejection of excessive tx traffic.
     */
    /*
     * Leave a small margin in case the target allocates tx descs
     * that the host is unaware of.
     */
    //return 1024;
    /* There is only space for 6 clients * 400 desc 
     * Hence limit allocation for only 6 clients.
     * Rest of the descriptors will be grabbed from VI clients*/
    if( (vow_max_sta * vow_max_desc_persta) > TOTAL_VOW_ALLOCABLE ) {
        u_int16_t vow_unrsvd_sta_num, vow_rsvd_num;
        
        vow_rsvd_num = TOTAL_VOW_ALLOCABLE/vow_max_desc_persta;

        vow_unrsvd_sta_num = vow_max_sta - vow_rsvd_num;

        if( (vow_unrsvd_sta_num * vow_max_desc_persta) <= VOW_DESC_GRAB_MAX ) {
            /*Can support the request.
             * Allocate only for vow_rsvd sta. Desc for 
             * other sta will be grabbed from BE/BK. */
            vow_max_sta = vow_rsvd_num;
        }
        else {
            A_ASSERT(0);
        }
    }
    return (CFG_TGT_NUM_MSDU_DESC + (vow_max_sta*vow_max_desc_persta));
#endif
}

int ol_cfg_tx_download_size(ol_pdev_handle pdev)
{
    /* FIX THIS - use symbolic constants rather than hard-coded numbers */
    /* 802.1Q and SNAP / LLC headers are accounted for elsewhere */
#if defined(CONFIG_HL_SUPPORT)
    return 1500; /* include payload, not only desc */
#else
    /* FIX THIS - may need more bytes in the future for deeper packet inspection */
    /* Need to change HTT_LL_TX_HDR_SIZE_IP accordingly */
    return 38; /* include payload, up to the end of UDP header for IPv4 case */
#endif
}

int ol_cfg_rx_host_defrag_timeout_duplicate_check(ol_pdev_handle pdev)
{
#if CFG_TGT_DEFAULT_RX_SKIP_DEFRAG_TIMEOUT_DUP_DETECTION_CHECK
    return 1;
#else
    return 0;
#endif
}

#if defined (ATHR_WIN_NWF)
#pragma warning(default:4100)   // unreferenced formal parameter
#endif
