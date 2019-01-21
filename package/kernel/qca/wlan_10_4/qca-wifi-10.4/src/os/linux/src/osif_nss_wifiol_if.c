/*
 * Copyright (c) 2015-2016 Qualcomm Atheros, Inc.
 *
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary. 
 */

/*
 * osif_nss_wifiol_if.c
 *
 * This file used for for interface   NSS WiFi Offload Radio
 * ------------------------REVISION HISTORY-----------------------------
 * Qualcomm Atheros         15/june/2015              Created
 */

#include <linux/types.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <net/ipv6.h>
#include <linux/version.h>

#if (LINUX_VERSION_CODE <= KERNEL_VERSION(3,9,0))
#include <net/ipip.h>
#else
#include <net/ip_tunnels.h>
#endif

#include <ol_txrx_types.h>
#include <ol_txrx_peer_find.h> /* ol_txrx_peer_find_by_id */
#include <linux/if_arp.h>
#include <linux/etherdevice.h>
#include <linux/in.h>
#include <asm/cacheflush.h>
#include <nss_api_if.h>
#include <nss_cmn.h>
#include "ath_pci.h"
#include "copy_engine_api.h"
#include "copy_engine_internal.h"
#include "osif_private.h"
#include "pktlog_ac_i.h"
#include <wdi_event.h>
#include "osif_nss_wifiol_vdev_if.h"
#include <htt_internal.h>
#include <ol_txrx_dbg.h>

#define NSS_PEER_MEMORY_DEFAULT_LENGTH 175000
#define NSS_RRA_MEMORY_DEFAULT_LENGTH 35400
#define STATS_COOKIE_LSB_OFFSET 3
#define STATS_COOKIE_MSB_OFFSET 4

static int osif_nss_ol_ce_raw_send(void *nss_wifiol_ctx, uint32_t radio_id, adf_nbuf_t netbuf, uint32_t  len);

int osif_nss_ol_get_ifnum(int radio_id){

    printk("NSS :Radio id %d\n", radio_id );
    if(radio_id == 0) {
        return NSS_WIFI_INTERFACE0;
    }else if(radio_id == 1) {
        return NSS_WIFI_INTERFACE1;

    }else if(radio_id == 2){

        return NSS_WIFI_INTERFACE2;
    }
    else{
        printk("bad radio id %d \n",radio_id);
        return 0;
    }
}


void update_ring_info(
        struct nss_wifi_ce_ring_state_msg *mring,
        struct CE_ring_state *sring) {

    mring->nentries = sring->nentries;
    mring->nentries_mask = sring->nentries_mask;
    mring->sw_index = sring->sw_index;
    mring->write_index = sring->write_index;
    mring->hw_index = sring->hw_index;
    mring->base_addr_CE_space = sring->base_addr_CE_space;
    mring->base_addr_owner_space = (unsigned int)sring->base_addr_owner_space;
    printk("NSS:nentries %d mask %x swindex %d write_index %d hw_index %d CE_space %x ownerspace %x \n",
            sring->nentries,
            sring->nentries_mask,
            sring->sw_index,
            sring->write_index,
            sring->hw_index,
            sring->base_addr_CE_space,
            (unsigned int)sring->base_addr_owner_space);
}

static int osif_nss_ol_send_peer_memory(void *if_ctx)
{
    struct ath_hif_pci_softc *sc = (struct ath_hif_pci_softc *) if_ctx;
    struct htt_pdev_t *htt_pdev = sc->scn->htt_pdev;
    struct ol_txrx_pdev_t *pdev = htt_pdev->txrx_pdev;
    struct nss_wifi_msg wifimsg;
    struct nss_wifi_peer_freelist_append_msg *pfam;
    void *nss_wifiol_ctx = pdev->nss_wifiol_ctx;
    uint32_t radio_id = pdev->nss_wifiol_id;
    uint32_t len = NSS_PEER_MEMORY_DEFAULT_LENGTH;
    int if_num;
    int pool_id;
    nss_tx_status_t status;


    if_num = osif_nss_ol_get_ifnum(radio_id);
    if (!if_num) {
        printk("no if found return ");
        return -1;
    }

    pool_id = pdev->last_peer_pool_used + 1;
    if (pool_id > (OSIF_NSS_OL_MAX_PEER_POOLS - 1)) {
        printk("pooil_id %d is out of limit\n", pool_id);
        return -1;
    }

    pdev->peer_mem_pool[pool_id] = kmalloc(len, GFP_KERNEL);
    if (!pdev->peer_mem_pool[pool_id]) {
        printk("Not able to allocate memory for peer mem pool with pool_id %d", pool_id);
        return -1;
    }

    pdev->last_peer_pool_used = pool_id;

    memset(&wifimsg, 0, sizeof(struct nss_wifi_msg));
    pfam = &wifimsg.msg.peer_freelist_append;

    pfam->addr = dma_map_single(NULL,  pdev->peer_mem_pool[pool_id], len, DMA_TO_DEVICE);
    pfam->length = len;
    pfam->num_peers = pdev->max_peers_per_pool;

    printk("sending peer pool with id %d start addr %x length %d num_peers %d\n", pool_id, pfam->addr, pfam->length, pfam->num_peers);
    /*
     * Send WIFI peer mem pool configure
     */
    nss_cmn_msg_init(&wifimsg.cm, if_num, NSS_WIFI_PEER_FREELIST_APPEND_MSG,
            sizeof(struct nss_wifi_peer_freelist_append_msg), NULL, NULL);


    status = nss_wifi_tx_msg(nss_wifiol_ctx, &wifimsg);
    if (status != NSS_TX_SUCCESS) {
        printk("WIFI peer memory send failed %d \n", status);
        kfree(pdev->peer_mem_pool[pool_id]);
        pdev->last_peer_pool_used--;
        return -1;
    }

    return 0;
}

static int osif_nss_ol_send_rra_memory(void *if_ctx)
{
    struct ath_hif_pci_softc *sc = (struct ath_hif_pci_softc *) if_ctx;
    struct htt_pdev_t *htt_pdev = sc->scn->htt_pdev;
    struct ol_txrx_pdev_t *pdev = htt_pdev->txrx_pdev;
    struct nss_wifi_msg wifimsg;
    struct nss_wifi_rx_reorder_array_freelist_append_msg *pfam;
    void *nss_wifiol_ctx = pdev->nss_wifiol_ctx;
    uint32_t radio_id = pdev->nss_wifiol_id;
    uint32_t len = NSS_RRA_MEMORY_DEFAULT_LENGTH;
    int if_num;
    int pool_id;
    nss_tx_status_t status;


    if_num = osif_nss_ol_get_ifnum(radio_id);
    if (!if_num) {
        printk("no if found return ");
        return -1;
    }

    pool_id = pdev->last_rra_pool_used + 1;
    if (pool_id > (OSIF_NSS_OL_MAX_RRA_POOLS - 1)) {
        printk("pooil_id %d is out of limit\n", pool_id);
        return -1;
    }

    pdev->rra_mem_pool[pool_id] = kmalloc(len, GFP_KERNEL);
    if (!pdev->rra_mem_pool[pool_id]) {
        printk("Not able to allocate memory for peer mem pool with pool_id %d", pool_id);
        return -1;
    }

    pdev->last_rra_pool_used = pool_id;

    memset(&wifimsg, 0, sizeof(struct nss_wifi_msg));
    pfam = &wifimsg.msg.rx_reorder_array_freelist_append;

    pfam->addr = dma_map_single(NULL,  pdev->rra_mem_pool[pool_id], len, DMA_TO_DEVICE);
    pfam->length = len;
    pfam->num_rra = pdev->max_rra_per_pool;

    printk("sending rra pool with id %d start addr %x length %d num_rra %d\n", pool_id, pfam->addr, pfam->length, pfam->num_rra);

    /*
     * Send WIFI rra mem pool configure
     */
    nss_cmn_msg_init(&wifimsg.cm, if_num, NSS_WIFI_RX_REORDER_ARRAY_FREELIST_APPEND_MSG,
            sizeof(struct nss_wifi_rx_reorder_array_freelist_append_msg), NULL, NULL);


    status = nss_wifi_tx_msg(nss_wifiol_ctx, &wifimsg);
    if (status != NSS_TX_SUCCESS) {
        printk("WIFI rra memory send failed %d \n", status);
        kfree(pdev->rra_mem_pool[pool_id]);
        pdev->last_rra_pool_used--;
        return -1;
    }

    return 0;
}

/*
 * osif_nss_ol_process_bs_peer_state_msg
 *     Process the peer state message from NSS-FW
 *      and inform the bs handler
 */
int osif_nss_ol_process_bs_peer_state_msg(void *if_ctx, struct nss_wifi_bs_peer_activity *msg)
{
#if ATH_BAND_STEERING
    struct ath_hif_pci_softc *sc = (struct ath_hif_pci_softc *) if_ctx;
    struct htt_pdev_t *htt_pdev = sc->scn->htt_pdev;
    struct ol_txrx_pdev_t *pdev = htt_pdev->txrx_pdev;
    struct ol_txrx_peer_t *peer;
    uint16_t i;

    /*
     *  NSS-FW has sent the message regarding the
     *  activity for each peer.
     *  This infomation has to be passed on to the
     *  band-steering module.
     */
    for (i = 0 ; i < msg->nentries ; i++) {
        if (msg->peer_id[i] > pdev->max_peers) {
	    continue;
        }
        adf_os_spin_lock_bh(&pdev->peer_ref_mutex);
        peer = ol_txrx_peer_find_by_id(pdev, msg->peer_id[i]);
        if (peer) {
            ol_txrx_mark_peer_inact(peer, false);
        }
        adf_os_spin_unlock_bh(&pdev->peer_ref_mutex);
    }
#endif
    return 0;
}

/*
 *  * osif_nss_ol_process_ol_stats_msg
 *   *     Process the enhanced stats message from NSS-FW
 *    */
int osif_nss_ol_process_ol_stats_msg(void *if_ctx, struct nss_wifi_ol_stats_msg *msg)
{
#if ENHANCED_STATS
    struct ol_ath_softc_net80211 *scn = NULL;
    struct ieee80211com *ic = NULL;
    struct ieee80211_node *ni = NULL;
    struct ol_txrx_vdev_t *vdev = NULL;
    struct ieee80211vap *vap = NULL;
    struct ieee80211_mac_stats *mac_stats = NULL;
    uint32_t num_msdus = 0, byte_cnt = 0, discard_cnt = 0, i = 0;
    struct ol_txrx_peer_t *peer = NULL;

    struct ath_hif_pci_softc *sc = (struct ath_hif_pci_softc *) if_ctx;
    struct htt_pdev_t *htt_pdev = sc->scn->htt_pdev;
    struct ol_txrx_pdev_t *pdev = htt_pdev->txrx_pdev;

    scn = (struct ol_ath_softc_net80211 *)pdev->ctrl_pdev;
    ic  = &scn->sc_ic;
    scn->scn_stats.tx_bawadv += msg->bawadv_cnt;
    scn->scn_stats.tx_beacon += msg->bcn_cnt;

    /*
     *  Extract per peer ol stats from the message sent by NSS-FW.
     */
    for (i = 0 ; i < msg->npeers ; i++) {
        peer = ol_txrx_peer_find_by_id(pdev, msg->peer_ol_stats[i].peer_id);
        if (peer) {
            num_msdus = msg->peer_ol_stats[i].tx_data;
            byte_cnt = msg->peer_ol_stats[i].tx_bytes;
            discard_cnt = msg->peer_ol_stats[i].tx_fail;
            peer->peer_data_stats.data_packets = num_msdus;
            peer->peer_data_stats.data_bytes = byte_cnt;
            peer->peer_data_stats.thrup_bytes = byte_cnt;
            peer->peer_data_stats.discard_cnt = discard_cnt;

            vdev = peer->vdev;
            vap = ol_ath_vap_get(scn, vdev->vdev_id);
            if (vap) {
                vap->iv_unicast_stats.ims_tx_discard += discard_cnt;
                vap->iv_stats.is_tx_not_ok += discard_cnt;
                ni = ieee80211_vap_find_node(vap,peer->mac_addr.raw);
                if (ni) {
                    ni->ni_stats.ns_tx_discard += discard_cnt;
                    ni->ni_stats.ns_tx_mcast += msg->peer_ol_stats[i].tx_mcast;
                    ni->ni_stats.ns_tx_ucast += msg->peer_ol_stats[i].tx_ucast;
                    ni->ni_stats.ns_tx_data_success += num_msdus;
                    ni->ni_stats.ns_tx_bytes_success += byte_cnt;
                    ni->ni_stats.ns_is_tx_not_ok += discard_cnt;
                    ieee80211_free_node(ni);
                }
                mac_stats = peer->bss_peer ? &vap->iv_multicast_stats :
                               &vap->iv_unicast_stats;
                mac_stats->ims_tx_packets += num_msdus;
                mac_stats->ims_tx_bytes += byte_cnt;
                mac_stats->ims_tx_data_packets += num_msdus;
                mac_stats->ims_tx_data_bytes += byte_cnt;
                mac_stats->ims_tx_datapyld_bytes = mac_stats->ims_tx_data_bytes -
                                                    (mac_stats->ims_tx_data_packets * (ETHERNET_HDR_LEN + 24));
            }
            scn->scn_stats.tx_num_data += num_msdus;
            scn->scn_stats.tx_bytes += byte_cnt;
            scn->scn_stats.tx_compaggr += msg->peer_ol_stats[i].tx_aggr;
            scn->scn_stats.tx_compunaggr += msg->peer_ol_stats[i].tx_unaggr;
        }
    }
#endif
    return 0;
}

/*
 * osif_nss_ol_process_sta_kickout_msg
 *     Process the sta kickout message from NSS-FW
 */
int osif_nss_ol_process_sta_kickout_msg(void *if_ctx, struct nss_wifi_sta_kickout_msg *msg)
{
    struct ath_hif_pci_softc *sc = (struct ath_hif_pci_softc *) if_ctx;
    struct ol_ath_softc_net80211 *scn = sc->scn;
    struct htt_pdev_t *htt_pdev = sc->scn->htt_pdev;
    struct ol_txrx_pdev_t *pdev = htt_pdev->txrx_pdev;
    struct ol_txrx_peer_t *peer;
    uint8_t peer_macaddr_raw[OL_TXRX_MAC_ADDR_LEN];

    adf_os_spin_lock_bh(&pdev->peer_ref_mutex);
    peer = ol_txrx_peer_find_by_id(pdev, msg->peer_id);
    if (peer) {
        memcpy(peer_macaddr_raw, peer->mac_addr.raw, OL_TXRX_MAC_ADDR_LEN);
    }
    adf_os_spin_unlock_bh(&pdev->peer_ref_mutex);
    peer_sta_kickout(scn, (A_UINT8 *)&peer_macaddr_raw);
    return 0;
}


/*
 * osif_nss_ol_process_wnm_peer_rx_activity_msg
 *     Process the rx activity message from NSS-FW
 *      and inform the wnm handler
 */
int osif_nss_ol_process_wnm_peer_rx_activity_msg(void *if_ctx, struct nss_wifi_wnm_peer_rx_activity_msg *msg)
{
#if UMAC_SUPPORT_WNM
    struct ath_hif_pci_softc *sc;
    struct ol_ath_softc_net80211 *scn;
    struct htt_pdev_t *htt_pdev;
    struct ol_txrx_pdev_t *pdev;
    struct ieee80211com *ic;
    struct ieee80211_node *ni;
    struct ol_txrx_peer_t *peer;
    struct ieee80211vap *vap;
    uint16_t i;

    sc = (struct ath_hif_pci_softc *) if_ctx;
    scn = sc->scn;
    ic = &scn->sc_ic;
    htt_pdev = scn->htt_pdev;
    pdev = htt_pdev->txrx_pdev;

    /*
     *  NSS-FW has sent the message regarding the
     *  rx activity for each peer.
     *  This infomation has to be passed on to wnm.
     */
    for (i = 0 ; i < msg->nentries ; i++) {
        if (msg->peer_id[i] > pdev->max_peers) {
            continue;
        }
        adf_os_spin_lock_bh(&pdev->peer_ref_mutex);

        peer = ol_txrx_peer_find_by_id(pdev, msg->peer_id[i]);
        if (peer) {
            vap = ol_ath_vap_get(scn, peer->vdev->vdev_id);
        } else {
            adf_os_spin_unlock_bh(&pdev->peer_ref_mutex);
            continue;
        }

        if (vap && ieee80211_vap_wnm_is_set(vap) && ieee80211_wnm_bss_is_set(vap->wnm)) {
            ni = ieee80211_find_node(&ic->ic_sta, peer->mac_addr.raw);
            if (ni) {
                ieee80211_wnm_bssmax_updaterx(ni, 0);
            }
            ieee80211_free_node(ni);
        }
        adf_os_spin_unlock_bh(&pdev->peer_ref_mutex);
    }
#endif
    return 0;
}

void osif_nss_ol_event_receive(void *if_ctx, struct nss_cmn_msg *wifimsg)
{
    struct ath_hif_pci_softc *sc = (struct ath_hif_pci_softc *) if_ctx;
    struct ol_ath_softc_net80211 *scn = sc->scn;
    struct htt_pdev_t *htt_pdev;
    uint32_t msg_type = wifimsg->type;
    enum nss_cmn_response response = wifimsg->response;
    struct ol_txrx_pdev_t *pdev = NULL;
    struct ol_txrx_stats *pdev_mestats = NULL;
    struct nss_wifi_mc_enhance_stats *nss_mestats = NULL;


    /*
     * Handle the message feedback returned from FW
     */
    switch (msg_type) {
        case NSS_WIFI_POST_RECV_MSG:
            {
                if (response == NSS_CMN_RESPONSE_EMSG) {
                    printk("NSS WIFI OFFLOAD - FAILED TO INITIALIZE CE\n");
                    scn->nss_wifiol_ce_enabled = 0;
                    return;
                }
                scn->nss_wifiol_ce_enabled = 1;
            }
            break;

        case NSS_WIFI_MGMT_SEND_MSG:
            {
                if (response == NSS_CMN_RESPONSE_EMSG) {
                    struct nss_wifi_mgmtsend_msg *msg=  &((struct nss_wifi_msg *)wifimsg)->msg.mgmtmsg;

                    uint32_t desc_id = msg->desc_id;
                    htt_pdev = sc->scn->htt_pdev;

                    if(scn->radio_attached == 0)
                        return;

                    printk("Mgmt send failed\n");
                    htt_tx_mgmt_desc_free(htt_pdev, desc_id, IEEE80211_TX_ERROR);
                }
            }
            break;

        case NSS_WIFI_FW_STATS_MSG:
            {

                if (response == NSS_CMN_RESPONSE_EMSG) {
                    struct nss_wifi_fw_stats_msg *msg = &((struct nss_wifi_msg *)wifimsg)->msg.fwstatsmsg;
                    struct ol_txrx_stats_req_internal *req;
                    uint8_t *data = msg->array;
                    uint32_t cookie_base = (uint32_t)(data + (HTC_HDR_LENGTH + HTC_HDR_ALIGNMENT_PADDING));
                    uint32_t *ptr = (uint32_t *)(cookie_base);

                    u_int64_t cookie = *(ptr + STATS_COOKIE_MSB_OFFSET);
                    cookie = ((cookie << 32) |(*(ptr + STATS_COOKIE_LSB_OFFSET)));

                    if(scn->radio_attached == 0)
                        return;
                    /*
                     * Drop the packet;
                     */
                    printk("Fw stats request send failed\n");
                    req = (struct ol_txrx_stats_req_internal *) ((size_t) cookie);

                    htt_pdev = sc->scn->htt_pdev;
                    pdev = htt_pdev->txrx_pdev;

                    htt_pkt_buf_list_del(htt_pdev, cookie);
                    if (req->base.wait.blocking) {
                        adf_os_mutex_release(pdev->osdev, &scn->scn_stats_sem);
                    }

                    adf_os_mem_free(req);
                }
            }
            break;

        case NSS_WIFI_STATS_MSG:
            {
                struct nss_wifi_stats_sync_msg *stats = &((struct nss_wifi_msg *)wifimsg)->msg.statsmsg;

                if(scn->radio_attached == 0)
                    return;

                if (stats->rx_bytes_deliverd) {
                    scn->scn_led_byte_cnt += stats->rx_bytes_deliverd;
                    ol_ath_led_event(scn, OL_ATH_LED_RX);
                }

                if (stats->tx_bytes_transmit_completions) {
                    scn->scn_led_byte_cnt += stats->tx_bytes_transmit_completions;
                    ol_ath_led_event(scn, OL_ATH_LED_TX);
                }

                htt_pdev = sc->scn->htt_pdev;
                pdev = htt_pdev->txrx_pdev;

                /*
                 * Update the me stats
                 */
                pdev_mestats = &pdev->stats.pub;
                nss_mestats = &stats->mc_enhance_stats;
                pdev_mestats->mcast_enhance.num_me_rcvd += nss_mestats->rcvd;
                pdev_mestats->mcast_enhance.num_me_ucast += nss_mestats->ucast_converted;
                pdev_mestats->mcast_enhance.num_me_dropped_a += nss_mestats->alloc_fail;
                pdev_mestats->mcast_enhance.num_me_dropped_i += nss_mestats->enqueue_fail + nss_mestats->copy_fail + nss_mestats->peer_flow_ctrl_send_fail;
                pdev_mestats->mcast_enhance.num_me_dropped_s += nss_mestats->loopback_err + nss_mestats->dst_addr_err;

            }
            break;

        case NSS_WIFI_SEND_PEER_MEMORY_REQUEST_MSG:
            {

                printk("event: sending peer pool memory\n");
                osif_nss_ol_send_peer_memory(if_ctx);
            }
            break;

        case NSS_WIFI_SEND_RRA_MEMORY_REQUEST_MSG:
            {
                printk("event: sending rra pool memory\n");
                osif_nss_ol_send_rra_memory(if_ctx);
            }
            break;

        case NSS_WIFI_PEER_BS_STATE_MSG:
            {
                struct nss_wifi_bs_peer_activity *msg =  &((struct nss_wifi_msg *)wifimsg)->msg.peer_activity;
                osif_nss_ol_process_bs_peer_state_msg(if_ctx, msg);
            }
            break;

        case NSS_WIFI_OL_STATS_MSG:
            {
                struct nss_wifi_ol_stats_msg *msg =  &((struct nss_wifi_msg *)wifimsg)->msg.ol_stats_msg;
                osif_nss_ol_process_ol_stats_msg(if_ctx, msg);
            }
            break;

        case NSS_WIFI_STA_KICKOUT_MSG:
            {
                struct nss_wifi_sta_kickout_msg *msg =  &((struct nss_wifi_msg *)wifimsg)->msg.sta_kickout_msg;
                osif_nss_ol_process_sta_kickout_msg(if_ctx, msg);
            }
            break;

        case NSS_WIFI_WNM_PEER_RX_ACTIVITY_MSG:
            {
                struct nss_wifi_wnm_peer_rx_activity_msg *msg;
                msg = &((struct nss_wifi_msg *)wifimsg)->msg.wprm;
                osif_nss_ol_process_wnm_peer_rx_activity_msg(if_ctx, msg);
            }
            break;

        default:
            break;
        }

    /* printk("%s\n",__FUNCTION__); */
    /*
     * TODO : Need to add stats updates
     */
    return;
}

void
htt_t2h_msg_handler_fast(void *htt_pdev, adf_nbuf_t *nbuf_cmpl_arr,
        uint32_t num_cmpls, uint32_t *num_htt_cmpls);

static void osif_nss_ol_wifi_ce_callback_func(void * ctx , struct sk_buff * nbuf, __attribute__((unused)) struct napi_struct *napi) {

    struct ath_hif_pci_softc *sc = (struct ath_hif_pci_softc *) ctx;
    struct htt_pdev_t *htt_pdev ;
    adf_nbuf_t  nbuf_cmpl_arr[32];
    uint32_t num_htt_cmpls;
    uint32_t count;


    if (sc->scn == NULL) {
        printk("%s: scn is NULL free the nbuf: %p\n", __func__, nbuf);
        goto fail;
    }

    htt_pdev=  sc->scn->htt_pdev;

    if (htt_pdev == NULL) {
        printk("%s: htt_pdev is NULL free the nbuf: %p for radio ID: %d\n", __func__, nbuf, sc->scn->radio_id);
        goto fail;
    }

    nbuf_cmpl_arr[0] = nbuf;

    htt_t2h_msg_handler_fast(htt_pdev, nbuf_cmpl_arr, 1, &num_htt_cmpls);
    return;

fail:
    printk("nbuf->data: ");
    for (count = 0; count < 16; count++) {
        printk("%x ", nbuf->data[count]);
    }
    printk("\n");
    dev_kfree_skb_any(nbuf);
}

void
ol_rx_process_inv_peer(ol_txrx_pdev_handle pdev, void *rx_mpdu_desc,
        adf_nbuf_t msdu);

/*
 * Extended data callback
 */
static void osif_nss_ol_wifi_ext_callback_func(void * ctx , struct sk_buff * nbuf,  __attribute__((unused)) struct napi_struct *napi)
{
    struct nss_wifi_rx_ext_metadata *wrem = (struct nss_wifi_rx_ext_metadata *)nbuf->data;
    struct ath_hif_pci_softc *sc = (struct ath_hif_pci_softc *) ctx;
    struct htt_pdev_t *htt_pdev = sc->scn->htt_pdev;
    uint32_t rx_desc_size = htt_pdev->ar_rx_ops->sizeof_rx_desc();
    void *rx_mpdu_desc = (void *)nbuf->head;
    struct ol_txrx_pdev_t *pdev = htt_pdev->txrx_pdev;
    adf_nbuf_t mon_skb;
    int discard = 1;
    uint32_t type  =  wrem->type;
    uint16_t peer_id;
    enum htt_rx_status status;

    switch (type) {
        case NSS_WIFI_RX_EXT_PKTLOG_TYPE:
            peer_id = wrem->peer_id;
            status = wrem->htt_rx_status;

            wdi_event_handler(WDI_EVENT_RX_DESC_REMOTE, pdev, nbuf, peer_id, status);
            break;

        case NSS_WIFI_RX_EXT_INV_PEER_TYPE:
        default :
            dma_unmap_single (NULL, virt_to_phys(rx_mpdu_desc), rx_desc_size, DMA_FROM_DEVICE);

            ol_rx_process_inv_peer(pdev, rx_mpdu_desc, nbuf);

	    adf_os_spin_lock_bh(&pdev->mon_mutex);
            if ((pdev->monitor_vdev) && (!pdev->filter_neighbour_peers)) {
                mon_skb = htt_pdev->ar_rx_ops->restitch_mpdu_from_msdus(
                        htt_pdev->arh, nbuf, pdev->rx_mon_recv_status, 1);

                if (mon_skb) {
                    pdev->monitor_vdev->osif_rx_mon(pdev->monitor_vdev->osif_vdev, mon_skb, pdev->rx_mon_recv_status);
                    discard = 0;
                }
            }
	    adf_os_spin_unlock_bh(&pdev->mon_mutex);
            break;
    }

    if (discard) {
        dev_kfree_skb_any(nbuf);
    }
}

/*
 * osif_nss_ol_mgmt_frame_send
 * 	Send Management frame to NSS
 */
int osif_nss_ol_mgmt_frame_send(void *nss_wifiol_ctx, int32_t radio_id, uint32_t desc_id, adf_nbuf_t netbuf)
{
    int32_t len;
    int32_t ifnum;
    struct nss_wifi_msg wifimsg;
    struct nss_wifi_mgmtsend_msg *mgmtmsg;
    nss_wifi_msg_callback_t msg_cb;
    nss_tx_status_t status;

    if (!nss_wifiol_ctx) {
        return 1;
    }

    msg_cb = (nss_wifi_msg_callback_t)osif_nss_ol_event_receive;

    if(radio_id == 0) {
        ifnum = NSS_WIFI_INTERFACE0;
    } else if(radio_id == 1) {
        ifnum = NSS_WIFI_INTERFACE1;
    } else if (radio_id == 2) {
        ifnum = NSS_WIFI_INTERFACE2;
    } else {
        printk("%s: Invalid radio id :%d\n", __FUNCTION__, radio_id);
        return 1;
    }

    len = adf_nbuf_len(netbuf);
    if (len > NSS_WIFI_MGMT_DATA_LEN) {
        printk("%s: The specified length :%d is beyond allowed length:%d\n", __FUNCTION__, len, NSS_WIFI_MGMT_DATA_LEN);
        return 1;
    }

    memset(&wifimsg, 0, sizeof(struct nss_wifi_msg));
    mgmtmsg = &wifimsg.msg.mgmtmsg;
    mgmtmsg->len = len;
    mgmtmsg->desc_id = desc_id;
    memcpy(mgmtmsg->array, (uint8_t *)netbuf->data, len);

    /*
     * Send wifi ce configure message
     */
    nss_cmn_msg_init(&wifimsg.cm, ifnum, NSS_WIFI_MGMT_SEND_MSG,
            sizeof(struct nss_wifi_mgmtsend_msg), msg_cb, NULL);

    status = nss_wifi_tx_msg(nss_wifiol_ctx, &wifimsg);
    if (status != NSS_TX_SUCCESS) {
        printk("%s: WIFI CE configure failt %d \n",__FUNCTION__, status);
        return 1;
    }

    return 0;
}

/*
 * osif_nss_ol_stats_frame_send
 * 	Send FW Stats frame to NSS
 */
int osif_nss_ol_stats_frame_send(void *nss_wifiol_ctx, int32_t radio_id, adf_nbuf_t netbuf)
{
    int32_t len = 0;
    int32_t ifnum = 0;
    struct nss_wifi_msg wifimsg;
    struct nss_wifi_fw_stats_msg *statsmsg;
    nss_wifi_msg_callback_t msg_cb;
    nss_tx_status_t status;

    if (!nss_wifiol_ctx) {
        return 0;
    }

    msg_cb = (nss_wifi_msg_callback_t)osif_nss_ol_event_receive;

    if(radio_id == 0) {
        ifnum = NSS_WIFI_INTERFACE0;
    } else if(radio_id == 1) {
        ifnum = NSS_WIFI_INTERFACE1;
    } else if (radio_id == 2) {
        ifnum = NSS_WIFI_INTERFACE2;
    } else {
        printk("%s: Invalid radio id :%d\n", __FUNCTION__, radio_id);
        return 1;
    }

    len = adf_nbuf_len(netbuf);
    if (len > NSS_WIFI_FW_STATS_DATA_LEN) {
        printk("%s: The specified length :%d is beyond allowed length:%d\n", __FUNCTION__, len, NSS_WIFI_FW_STATS_DATA_LEN);
        return 1;
    }

    memset(&wifimsg, 0, sizeof(struct nss_wifi_msg));
    statsmsg = &wifimsg.msg.fwstatsmsg;
    statsmsg->len = len;
    memcpy(statsmsg->array, (uint8_t *)netbuf->data, len);

    /*
     * Send wifi ce configure message
     */
    nss_cmn_msg_init(&wifimsg.cm, ifnum, NSS_WIFI_FW_STATS_MSG,
                      sizeof(struct nss_wifi_fw_stats_msg), msg_cb, NULL);

    status = nss_wifi_tx_msg(nss_wifiol_ctx, &wifimsg);
    if (status != NSS_TX_SUCCESS) {
        printk("%s: WIFI CE configure failed %d \n",__FUNCTION__, status);
        return 1;
    }

    return 0;
}

void osif_nss_ol_enable_dbdc_process(struct ieee80211com* ic, uint32_t enable)
{
    void *nss_wifiol_ctx = NULL;
    struct ol_ath_softc_net80211 *scn = OL_ATH_SOFTC_NET80211(ic);
    uint32_t radio_id;
    struct nss_wifi_msg wifimsg;
    struct nss_wifi_dbdc_process_enable_msg *dbdc_enable = NULL;
    int ifnum;
    nss_tx_status_t status;
    struct ol_txrx_pdev_t *pdev = scn->pdev_txrx_handle;

    if (!pdev) {
        return;
    }

    nss_wifiol_ctx = pdev->nss_wifiol_ctx;
    if (!nss_wifiol_ctx) {
        return;
    }

    radio_id = pdev->nss_wifiol_id;
    ifnum = osif_nss_ol_get_ifnum(radio_id);
    if (!ifnum) {
        printk("no if found return ");
        return;
    }

    memset(&wifimsg, 0, sizeof(struct nss_wifi_msg));
    dbdc_enable = &wifimsg.msg.dbdcpe_msg;

#if DBDC_REPEATER_SUPPORT
    if (enable) {
        printk("Enabling dbdc repeater process\n");
    } else {
        printk("Disabling dbdc repeater process\n");
    }

    dbdc_enable->dbdc_process_enable = enable;
#else
    dbdc_enable->dbdc_process_enable = 0;
#endif

    /*
     * Send enable dbdc process msg
     */
    nss_cmn_msg_init(&wifimsg.cm, ifnum, NSS_WIFI_DBDC_PROCESS_ENABLE_MSG,
            sizeof(struct nss_wifi_dbdc_process_enable_msg), NULL, NULL);

    status = nss_wifi_tx_msg(nss_wifiol_ctx, &wifimsg);
    if (status != NSS_TX_SUCCESS) {
        printk("WIFI enable dbdc process message send failed  %d \n", status);
        return;
    }

    return;
}

void osif_nss_ol_set_primary_radio(struct ieee80211com* ic, uint32_t enable)
{
    void *nss_wifiol_ctx = NULL;
    struct ol_ath_softc_net80211 *scn = OL_ATH_SOFTC_NET80211(ic);
    uint32_t radio_id;
    struct nss_wifi_msg wifimsg;
    struct nss_wifi_primary_radio_set_msg *primary_radio = NULL;
    int ifnum;
    nss_tx_status_t status;
    struct ol_txrx_pdev_t *pdev = scn->pdev_txrx_handle;

    if (!pdev) {
        return;
    }

    nss_wifiol_ctx = pdev->nss_wifiol_ctx;
    if (!nss_wifiol_ctx) {
        return;
    }

    radio_id = pdev->nss_wifiol_id;
    ifnum = osif_nss_ol_get_ifnum(radio_id);
    if (!ifnum) {
        printk("no if found return ");
        return;
    }

    memset(&wifimsg, 0, sizeof(struct nss_wifi_msg));
    primary_radio = &wifimsg.msg.wprs_msg;

#if DBDC_REPEATER_SUPPORT
    if (enable) {
        printk("Enabling primary_radio for if_num %d\n", ifnum);
    } else {
        printk("Disabling primary_radio for if_num %d\n", ifnum);
    }

    primary_radio->flag = enable;
#else
    primary_radio->flag = 0;
#endif

    /*
     * Send set primary radio msg
     */
    nss_cmn_msg_init(&wifimsg.cm, ifnum, NSS_WIFI_PRIMARY_RADIO_SET_MSG,
            sizeof(struct nss_wifi_primary_radio_set_msg), NULL, NULL);

    status = nss_wifi_tx_msg(nss_wifiol_ctx, &wifimsg);
    if (status != NSS_TX_SUCCESS) {
        printk("WIFI set primary radio message send failed  %d \n", status);
        return;
    }

    return;
}

void osif_nss_ol_set_force_client_mcast_traffic(struct ieee80211com* ic)
{
    void *nss_wifiol_ctx = NULL;
    struct ol_ath_softc_net80211 *scn = OL_ATH_SOFTC_NET80211(ic);
    uint32_t radio_id;
    struct nss_wifi_msg wifimsg;
    struct nss_wifi_force_client_mcast_traffic_set_msg *force_client_mcast_traffic = NULL;
    int ifnum;
    nss_tx_status_t status;
    struct ol_txrx_pdev_t *pdev = scn->pdev_txrx_handle;

    if (!pdev) {
        return;
    }

    nss_wifiol_ctx = pdev->nss_wifiol_ctx;
    if (!nss_wifiol_ctx) {
        return;
    }

    radio_id = pdev->nss_wifiol_id;
    ifnum = osif_nss_ol_get_ifnum(radio_id);
    if (!ifnum) {
        printk("no if found return ");
        return;
    }

    memset(&wifimsg, 0, sizeof(struct nss_wifi_msg));
    force_client_mcast_traffic = &wifimsg.msg.wfcmts_msg;

#if DBDC_REPEATER_SUPPORT
    printk("force_client_mcast_traffic for if_num %d is %d\n", ifnum, ic->ic_global_list->force_client_mcast_traffic);

    force_client_mcast_traffic->flag = ic->ic_global_list->force_client_mcast_traffic;
#else
    force_client_mcast_traffic->flag = 0;
#endif

    /*
     * Send force client mcast traffic msg
     */
    nss_cmn_msg_init(&wifimsg.cm, ifnum, NSS_WIFI_FORCE_CLIENT_MCAST_TRAFFIC_SET_MSG,
            sizeof(struct nss_wifi_force_client_mcast_traffic_set_msg), NULL, NULL);

    status = nss_wifi_tx_msg(nss_wifiol_ctx, &wifimsg);
    if (status != NSS_TX_SUCCESS) {
        printk("WIFI force client mcast traffic message send failed  %d \n", status);
        return;
    }

    return;
}

uint32_t osif_nss_ol_validate_dbdc_process(struct ieee80211com* ic) {
    struct global_ic_list *glist = ic->ic_global_list;
    int i;
    uint32_t nss_radio_sta_vap = 0;
    uint32_t non_nss_radio_sta_vap = 0;
    struct ieee80211com* tmpic;

    if (glist->dbdc_process_enable) {

        printk("DBDC process check  num IC %d \n",glist->num_global_ic);
        for(i=0; i < glist->num_global_ic; i++) {
            tmpic = glist->global_ic[i];
            if (tmpic->other_ic_stavap ) {
                if (tmpic->ic_is_mode_offload(tmpic)) {
                    struct ol_ath_softc_net80211 *scn = OL_ATH_SOFTC_NET80211(tmpic);
                    struct ol_txrx_pdev_t *pdev = scn->pdev_txrx_handle;
                    if (!pdev) {
                        continue;
                    }
                    if(pdev->nss_wifiol_ctx) {

                        nss_radio_sta_vap++;
                    }else {
                        non_nss_radio_sta_vap++;
                    }
                } else {
                    non_nss_radio_sta_vap++;
                }
            }
        }
        printk("nss_radio sta vap %d non nss radio sta vap %d \n", nss_radio_sta_vap, non_nss_radio_sta_vap);
        if( nss_radio_sta_vap && non_nss_radio_sta_vap) {
            printk("NSS Offload: Disable dbdc process ,non supported configuration\n");
            glist->dbdc_process_enable = 0;

            for(i=0; i < glist->num_global_ic; i++) {
                tmpic = glist->global_ic[i];
                if (tmpic->ic_is_mode_offload(tmpic)) {
			printk("NSS Offload: Send command to Disable dbdc process \n");
			osif_nss_ol_enable_dbdc_process(tmpic, 0);
                }
            }
            return 1;
        }
    }
    return 0;
}

void osif_nss_ol_store_other_pdev_stavap(struct ieee80211com* ic)
{
    void *nss_wifiol_ctx = NULL;
    struct ol_ath_softc_net80211 *scn = OL_ATH_SOFTC_NET80211(ic);
    uint32_t radio_id;
    struct nss_wifi_msg wifimsg;
    struct nss_wifi_store_other_pdev_stavap_msg *pdev_stavap = NULL;
    int ifnum;
    nss_tx_status_t status;
    struct ieee80211vap *vap = NULL;
    ol_txrx_vdev_handle vdev = NULL;
    osif_dev *osifp = NULL;
    struct ieee80211com* other_ic;
    struct ol_txrx_pdev_t *pdev;

    if (!ic->ic_is_mode_offload(ic)) {
	    printk("NSS offload : DBDC repeater IC is not in partial offload mode discard command to NSS \n");
	    return;
    }

    pdev = scn->pdev_txrx_handle;

    if (!pdev) {
        return;
    }

    nss_wifiol_ctx = pdev->nss_wifiol_ctx;
    if (!nss_wifiol_ctx) {
        return;
    }

    radio_id = pdev->nss_wifiol_id;
    ifnum = osif_nss_ol_get_ifnum(radio_id);
    if (!ifnum) {
        printk("no if found return ");
        return;
    }

    if(osif_nss_ol_validate_dbdc_process(ic)){
	    return;
    }

    memset(&wifimsg, 0, sizeof(struct nss_wifi_msg));
    pdev_stavap = &wifimsg.msg.wsops_msg;

    vap = ic->other_ic_stavap;

    if (!vap) {

        printk("other ic stavap is NULL\n");
        pdev_stavap->stavap_ifnum = -1;
    } else {

        other_ic = vap->iv_ic;
        if (!other_ic->ic_is_mode_offload(other_ic)) {
            printk("NSS offload : DBDC repeater other IC is not in partial offload mode discard command to NSS \n");
            return;
        }
        vdev = vap->iv_txrx_handle;
        osifp = (osif_dev *)vdev->osif_vdev;
        pdev_stavap->stavap_ifnum = osifp->nss_ifnum;
    }

    printk("pdev if_num %d other pdev stavap if_num is %d",ifnum,  pdev_stavap->stavap_ifnum);

    /*
     * Send store other pdev stavap msg
     */
    if(ifnum == NSS_PROXY_VAP_IF_NUMBER) {
	    printk("Error : Psta vap not supported for mcast recv \n");
	    return;
    }
    nss_cmn_msg_init(&wifimsg.cm, ifnum, NSS_WIFI_STORE_OTHER_PDEV_STAVAP_MSG,
            sizeof(struct nss_wifi_store_other_pdev_stavap_msg), NULL, NULL);

    status = nss_wifi_tx_msg(nss_wifiol_ctx, &wifimsg);
    if (status != NSS_TX_SUCCESS) {
        printk("WIFI store other pdev stavap message send failed  %d \n", status);
        return;
    }

    return;
}

void osif_nss_ol_set_perpkt_txstats(struct ol_ath_softc_net80211 *scn)
{
    void *nss_wifiol_ctx = NULL;
    uint32_t radio_id;
    struct nss_wifi_msg wifimsg;
    struct nss_wifi_enable_perpkt_txstats_msg *set_perpkt_txstats;
    int ifnum;
    nss_tx_status_t status;
    struct ol_txrx_pdev_t *pdev = scn->pdev_txrx_handle;

    if (!pdev) {
        return;
    }

    nss_wifiol_ctx = pdev->nss_wifiol_ctx;
    if (!nss_wifiol_ctx) {
        return;
    }

    radio_id = pdev->nss_wifiol_id;
    ifnum = osif_nss_ol_get_ifnum(radio_id);
    if(!ifnum){
        printk("no if found return ");
        return;
    }

    memset(&wifimsg, 0, sizeof(struct nss_wifi_msg));
    set_perpkt_txstats = &wifimsg.msg.ept_msg;

#if ATH_DATA_TX_INFO_EN
    set_perpkt_txstats->perpkt_txstats_flag = scn->enable_perpkt_txstats;
#else
    set_perpkt_txstats->perpkt_txstats_flag = 0;
#endif

    /*
     * Send perpkt_txstats configure msg
     */
    nss_cmn_msg_init(&wifimsg.cm, ifnum, NSS_WIFI_ENABLE_PERPKT_TXSTATS_MSG,
            sizeof(struct nss_wifi_enable_perpkt_txstats_msg), NULL, NULL);

    status = nss_wifi_tx_msg(nss_wifiol_ctx, &wifimsg);
    if (status != NSS_TX_SUCCESS) {
        printk("WIFI set perpkt txstats message send failed  %d \n", status);
        return;
    }

    return;
}

void osif_nss_ol_set_igmpmld_override_tos(struct ol_ath_softc_net80211 *scn)
{
    void *nss_wifiol_ctx = NULL;
    uint32_t radio_id;
    struct nss_wifi_msg wifimsg;
    struct nss_wifi_igmp_mld_override_tos_msg *igmp_mld_tos;
    int ifnum;
    nss_tx_status_t status;
    struct ol_txrx_pdev_t *pdev = scn->pdev_txrx_handle;

    if (!pdev) {
        return;
    }

    nss_wifiol_ctx = pdev->nss_wifiol_ctx;
    if (!nss_wifiol_ctx) {
        return;
    }

    radio_id = pdev->nss_wifiol_id;
    ifnum = osif_nss_ol_get_ifnum(radio_id);
    if(!ifnum){
        printk("no if found return ");
        return;
    }

    memset(&wifimsg, 0, sizeof(struct nss_wifi_msg));
    igmp_mld_tos = &wifimsg.msg.wigmpmldtm_msg;

    igmp_mld_tos->igmp_mld_ovride_tid_en = scn->igmpmld_override;
    igmp_mld_tos->igmp_mld_ovride_tid_val = scn->igmpmld_tid;

    /*
     * Send igmp_mld_tos override configure msg
     */
    nss_cmn_msg_init(&wifimsg.cm, ifnum, NSS_WIFI_IGMP_MLD_TOS_OVERRIDE_MSG,
            sizeof(struct nss_wifi_igmp_mld_override_tos_msg), NULL, NULL);

    status = nss_wifi_tx_msg(nss_wifiol_ctx, &wifimsg);
    if (status != NSS_TX_SUCCESS) {
        printk("WIFI set igmp/mld tos configuration failed :%d \n", status);
        return;
    }

    return;
}

void osif_nss_ol_set_msdu_ttl(struct ol_txrx_pdev_t *pdev)
{
    void *nss_wifiol_ctx = pdev->nss_wifiol_ctx;
    uint32_t radio_id = pdev->nss_wifiol_id;
    struct nss_wifi_msg wifimsg;
    struct nss_wifi_msdu_ttl_set_msg *set_msdu_ttl;
    int ifnum;
    nss_tx_status_t status;

    if (!nss_wifiol_ctx) {
        return;
    }

    ifnum = osif_nss_ol_get_ifnum(radio_id);
    if(!ifnum){
        printk("no if found return ");
        return;
    }

    memset(&wifimsg, 0, sizeof(struct nss_wifi_msg));
    set_msdu_ttl = &wifimsg.msg.msdu_ttl_set_msg;
    set_msdu_ttl->msdu_ttl = pdev->pflow_msdu_ttl;

    /*
     * Send msdu_ttl configure msg
     */
    nss_cmn_msg_init(&wifimsg.cm, ifnum, NSS_WIFI_MSDU_TTL_SET_MSG,
            sizeof(struct nss_wifi_msdu_ttl_set_msg), NULL, NULL);

    status = nss_wifi_tx_msg(nss_wifiol_ctx, &wifimsg);
    if (status != NSS_TX_SUCCESS) {
        printk("WIFI set msdu ttl message send failed  %d \n", status);
        return;
    }

    return;
}

int osif_nss_ol_wifi_monitor_set_filter(struct ieee80211com* ic, uint32_t filter_type)
{

    struct ol_ath_softc_net80211 *scn;
    struct ath_hif_pci_softc *sc;
    int ifnum ;
    struct nss_wifi_msg wifimsg;
    struct nss_wifi_monitor_set_filter_msg *set_filter_msg ;
    nss_tx_status_t status;
    void *nss_wifiol_ctx;
    int radio_id;

    if (!ic->ic_is_mode_offload(ic)) {
        return 0;
    }

    scn = OL_ATH_SOFTC_NET80211(ic);
    sc =  (struct ath_hif_pci_softc *) scn->hif_sc;
    nss_wifiol_ctx= sc->nss_wifiol_ctx;
    radio_id = sc->nss_wifiol_id;

    if (!nss_wifiol_ctx) {
        return 0;
    }

    ifnum = osif_nss_ol_get_ifnum(radio_id);
    if(!ifnum){
        printk("no if found return ");
        return -1;
    }

    printk("%s %p %p %d %d \n",__func__,nss_wifiol_ctx, sc, radio_id, filter_type);

    memset(&wifimsg, 0, sizeof(struct nss_wifi_msg));
    set_filter_msg = &wifimsg.msg.monitor_filter_msg;
    set_filter_msg->filter_type = filter_type;

    /*
     * Send WIFI CE configure
     */
    nss_cmn_msg_init(&wifimsg.cm, ifnum, NSS_WIFI_MONITOR_FILTER_SET_MSG,
            sizeof(struct nss_wifi_monitor_set_filter_msg), NULL, NULL);

    status = nss_wifi_tx_msg(nss_wifiol_ctx, &wifimsg);
    if (status != NSS_TX_SUCCESS) {
        printk("WIFI monitor set filter message send failed  %d \n", status);
        return -1;
    }

    return 0;
}

/*
 *osif_nss_ol_stats_cfg
 * 	configuring pkt log for wifi
 */

int osif_nss_ol_stats_cfg(void *hifctx, uint32_t cfg)
{
    struct ath_hif_pci_softc *sc =  (struct ath_hif_pci_softc *)hifctx;
    int ifnum ;
    struct nss_wifi_msg wifimsg;
    struct nss_wifi_ol_stats_cfg_msg *pcm_msg ;
    nss_tx_status_t status;
    void *nss_wifiol_ctx= sc->nss_wifiol_ctx;
    int radio_id = sc->nss_wifiol_id;

    if (!nss_wifiol_ctx) {
        return 0;
    }

    printk("%s %p %p %d ol stats enable %d\n",__func__, nss_wifiol_ctx, sc, radio_id, cfg);

    ifnum = osif_nss_ol_get_ifnum(radio_id);
    if(!ifnum){
        printk("no if found return ");
        return -1;
    }

    memset(&wifimsg, 0, sizeof(struct nss_wifi_msg));
    pcm_msg = &wifimsg.msg.scm_msg;
    pcm_msg->stats_cfg = cfg;

    /*
     * Send ol_stats configure
     */
    nss_cmn_msg_init(&wifimsg.cm, ifnum, NSS_WIFI_OL_STATS_CFG_MSG,
            sizeof(struct nss_wifi_ol_stats_cfg_msg), NULL, NULL);

    status = nss_wifi_tx_msg(nss_wifiol_ctx, &wifimsg);
    if (status != NSS_TX_SUCCESS) {
        printk("WIFI Pause message send failed  %d \n", status);
        return -1;
    }

    return 0;
}

/*
 *osif_nss_ol_pktlog_cfg
 * 	configuring pkt log for wifi
 */

int osif_nss_ol_pktlog_cfg(void *hifctx, int enable)
{
    struct ath_hif_pci_softc *sc =  (struct ath_hif_pci_softc *)hifctx;
    int ifnum ;
    struct nss_wifi_msg wifimsg;
    struct nss_wifi_pktlog_cfg_msg *pcm_msg ;
    nss_tx_status_t status;
    void *nss_wifiol_ctx= sc->nss_wifiol_ctx;
    int radio_id = sc->nss_wifiol_id;
    struct ol_ath_softc_net80211 *scn = sc->scn;
    scn->nss_pktlog_en = enable;

    if (!nss_wifiol_ctx) {
        return 0;
    }

    printk("%s %p %p %d PKT log enable %d buf 64 \n",__func__,nss_wifiol_ctx, sc, radio_id, enable);
    printk("pktlog hdrsize %d msdu_id_offset %d \n", scn->pl_dev->pktlog_hdr_size, scn->pl_dev->msdu_id_offset);

    ifnum = osif_nss_ol_get_ifnum(radio_id);
    if(!ifnum){
        printk("no if found return ");
        return -1;
    }

    memset(&wifimsg, 0, sizeof(struct nss_wifi_msg));
    pcm_msg = &wifimsg.msg.pcm_msg;
    pcm_msg->enable = enable;
    pcm_msg->bufsize = 64;
    pcm_msg->hdrsize = scn->pl_dev->pktlog_hdr_size;
    pcm_msg->msdu_id_offset = scn->pl_dev->msdu_id_offset;

    /*
     * Send WIFI CE configure
     */
    nss_cmn_msg_init(&wifimsg.cm, ifnum, NSS_WIFI_PKTLOG_CFG_MSG,
            sizeof(struct nss_wifi_pktlog_cfg_msg), NULL, NULL);

    status = nss_wifi_tx_msg(nss_wifiol_ctx, &wifimsg);
    if (status != NSS_TX_SUCCESS) {
        printk("WIFI Pause message send failed  %d \n", status);
        return -1;
    }
    return 0;
}


int osif_nss_ol_wifi_pause(void *hifctx)
{
    struct ath_hif_pci_softc *sc;
    int ifnum ;
    struct nss_wifi_msg wifimsg;
    struct nss_wifi_stop_msg *pausemsg ;
    nss_tx_status_t status;
    void *nss_wifiol_ctx;
    int radio_id;

    sc =  (struct ath_hif_pci_softc *)hifctx;
    radio_id = sc->nss_wifiol_id;
    nss_wifiol_ctx= sc->nss_wifiol_ctx;
    if (!nss_wifiol_ctx) {
        return 0;
    }

    printk("%s %p %p %d \n",__func__,nss_wifiol_ctx, sc, radio_id);

    ifnum = osif_nss_ol_get_ifnum(radio_id);
    if(!ifnum){
        printk("no if found return ");
        return -1;
    }



    memset(&wifimsg, 0, sizeof(struct nss_wifi_msg));
    pausemsg = &wifimsg.msg.stopmsg;
    pausemsg->radio_id = radio_id;

    /*
     * Send WIFI CE configure
     */
    nss_cmn_msg_init(&wifimsg.cm, ifnum, NSS_WIFI_STOP_MSG,
            sizeof(struct nss_wifi_stop_msg), NULL, NULL);

    status = nss_wifi_tx_msg(nss_wifiol_ctx, &wifimsg);
    if (status != NSS_TX_SUCCESS) {
        printk("WIFI Pause message send failed  %d \n", status);
        return -1;
    }
    return 0;
}

/*
 *osif_nss_tx_queue_cfg
 * 	configuring tx queue size for wifi
 */
int osif_nss_tx_queue_cfg(struct ol_txrx_pdev_t *pdev, uint32_t range, uint32_t buf_size)
{
    void *nss_wifiol_ctx = pdev->nss_wifiol_ctx;
    uint32_t radio_id = pdev->nss_wifiol_id;
    struct nss_wifi_msg wifimsg;
    struct nss_wifi_tx_queue_cfg_msg *wtxqcm ;
    int ifnum;
    nss_tx_status_t status;

    if (!nss_wifiol_ctx) {
        return 0;
    }

    ifnum = osif_nss_ol_get_ifnum(radio_id);
    if(!ifnum){
        printk("no if found return ");
        return 0;
    }

    memset(&wifimsg, 0, sizeof(struct nss_wifi_msg));
    wtxqcm = &wifimsg.msg.wtxqcm;
    wtxqcm->range = range;
    wtxqcm->size = buf_size;

    /*
     * Send ol_stats configure
     */
    nss_cmn_msg_init(&wifimsg.cm, ifnum, NSS_WIFI_TX_QUEUE_CFG_MSG,
            sizeof(struct nss_wifi_tx_queue_cfg_msg), NULL, NULL);

    status = nss_wifi_tx_msg(nss_wifiol_ctx, &wifimsg);
    if (status != NSS_TX_SUCCESS) {
        printk("WIFI radio mode message failed  %d \n", status);
        return -1;
    }

    return 0;
}

/*
 * osif_nss_tx_queue_min_threshold_cfg
 * 	configuring tx min threshold for queueing for wifi
 */
int osif_nss_tx_queue_min_threshold_cfg(struct ol_txrx_pdev_t *pdev, uint32_t min_threshold)
{
    void *nss_wifiol_ctx = pdev->nss_wifiol_ctx;
    uint32_t radio_id = pdev->nss_wifiol_id;
    struct nss_wifi_msg wifimsg;
    struct nss_wifi_tx_min_threshold_cfg_msg *wtx_min_threshold_cm;
    int ifnum;
    nss_tx_status_t status;

    if (!nss_wifiol_ctx) {
        return 0;
    }

    ifnum = osif_nss_ol_get_ifnum(radio_id);
    if(!ifnum){
        printk("no if found return ");
        return 0;
    }

    memset(&wifimsg, 0, sizeof(struct nss_wifi_msg));
    wtx_min_threshold_cm = &wifimsg.msg.wtx_min_threshold_cm;
    wtx_min_threshold_cm->min_threshold = min_threshold;

    /*
     * Send min_threshold configure
     */
    nss_cmn_msg_init(&wifimsg.cm, ifnum, NSS_WIFI_TX_MIN_THRESHOLD_CFG_MSG,
            sizeof(struct nss_wifi_tx_min_threshold_cfg_msg), NULL, NULL);

    status = nss_wifi_tx_msg(nss_wifiol_ctx, &wifimsg);
    if (status != NSS_TX_SUCCESS) {
        printk("WIFI radio mode message failed  %d \n", status);
        return -1;
    }

    return 0;
}

int osif_nss_ol_wifi_reset(void *hifctx, uint16_t delay)
{
    struct ath_hif_pci_softc *sc;
    int ifnum ;
    struct nss_wifi_msg wifimsg;
    struct nss_wifi_reset_msg *resetmsg ;
    nss_tx_status_t status;
    void *nss_wifiol_ctx;
    int radio_id;
    struct ol_txrx_pdev_t *pdev;
    int i = 0;

    sc =  (struct ath_hif_pci_softc *)hifctx;
    nss_wifiol_ctx= sc->nss_wifiol_ctx;
    if (!nss_wifiol_ctx) {
        return 0;
    }

    radio_id = sc->nss_wifiol_id;
    pdev = sc->scn->pdev_txrx_handle;

    printk("%s %p %p %d \n",__func__,nss_wifiol_ctx,sc,radio_id);

    if (pdev) {
        /*
         * clear the wifiol ctx
         */
        pdev->nss_wifiol_ctx = NULL;
    }

    sc->nss_wifiol_ctx = NULL;

    ifnum = osif_nss_ol_get_ifnum(radio_id);
    if(!ifnum){
        printk("no if found return ");
        return -1;
    }

    memset(&wifimsg, 0, sizeof(struct nss_wifi_msg));
    resetmsg = &wifimsg.msg.resetmsg;
    resetmsg->radio_id = radio_id;

    /*
     * Send WIFI CE configure
     */
    nss_cmn_msg_init(&wifimsg.cm, ifnum, NSS_WIFI_RESET_MSG,
            sizeof(struct nss_wifi_reset_msg), NULL, NULL);

    status = nss_wifi_tx_msg(nss_wifiol_ctx, &wifimsg);
    if (status != NSS_TX_SUCCESS) {
        printk("WIFI reset message send failed  %d \n", status);
        return -1;
    }

    if (pdev) {

        printk("freeing rra memory pool\n");
        for (i = 0; i < OSIF_NSS_OL_MAX_RRA_POOLS; i++) {
            if (pdev->rra_mem_pool[i]) {
                printk("freeing rra mem pool for pool_id %d", i);
                kfree(pdev->rra_mem_pool[i]);
            }
        }

        printk("freeing peer memory pool\n");
        for (i = 0; i < OSIF_NSS_OL_MAX_PEER_POOLS; i++) {
            if (pdev->peer_mem_pool[i]) {
                printk("freeing peer mem pool for pool_id %d", i);
                kfree(pdev->peer_mem_pool[i]);
            }
        }
    }

    printk("nss wifi offload reset\n");
    if (delay) {
        mdelay(delay);
    }
    /*
     * Unregister wifi with NSS
     */
    printk("unregister wifi with nss\n");
    nss_unregister_wifi_if(ifnum);

    return 0;
}

int osif_nss_ol_wifi_init( void *hifctx) {

    struct ath_hif_pci_softc *sc =  (struct ath_hif_pci_softc *)hifctx;

    struct ol_ath_softc_net80211 *scn = sc->scn;
    struct CE_state *CE_tx_state;
    struct CE_state *CE_rx_state;

    struct nss_wifi_msg wifimsg;
    struct nss_wifi_init_msg *msg;
    int ifnum ;
    void * nss_wifiol_ctx= NULL;
    int features = 0;
    nss_tx_status_t status;

    memset(&wifimsg, 0, sizeof(struct nss_wifi_msg));
    msg = &wifimsg.msg.initmsg;


    CE_tx_state = sc->CE_id_to_state[4];
    CE_rx_state = sc->CE_id_to_state[5];
    printk("NSS %s :sc %p NSS ID %d  Taret type %x \n",__FUNCTION__, hifctx, sc->nss_wifiol_id, scn->target_type);
    scn->nss_wifiol_id = sc->nss_wifiol_id;

    msg->radio_id = sc->nss_wifiol_id;
    msg->pci_mem = pci_resource_start(sc->pdev, 0);


    msg->ce_tx_state.ctrl_addr = CE_tx_state->ctrl_addr;
    msg->ce_rx_state.ctrl_addr = CE_rx_state->ctrl_addr;

    if(CE_tx_state->src_ring){
        update_ring_info(&msg->ce_tx_state.src_ring,CE_tx_state->src_ring);
    }

    if(CE_tx_state->dest_ring){
        update_ring_info(&msg->ce_tx_state.dest_ring,CE_tx_state->dest_ring);
    }

    if(CE_rx_state->src_ring){
        update_ring_info(&msg->ce_rx_state.src_ring,CE_rx_state->src_ring);
    }
    if(CE_rx_state->dest_ring){
        update_ring_info(&msg->ce_rx_state.dest_ring,CE_rx_state->dest_ring);
    }

    msg->target_type = scn->target_type;

    ifnum = osif_nss_ol_get_ifnum(sc->nss_wifiol_id);
    if(!ifnum){
        printk("no if found return ");
        return -1;
    }

    /*
     * Register wifi with NSS
     */
    nss_wifiol_ctx   = nss_register_wifi_if(ifnum,
            (nss_wifi_callback_t)osif_nss_ol_wifi_ce_callback_func,
            (nss_wifi_callback_t)osif_nss_ol_wifi_ext_callback_func,
            (nss_wifi_msg_callback_t)osif_nss_ol_event_receive,
             (struct net_device *)sc, features
             );

     if(nss_wifiol_ctx  == NULL){
         printk("NSS-WifiOffload: NSS regsiter fail\n");
     }

     /*
      * Enable new Data path for MU-MIMO
      */
     msg->mu_mimo_enhancement_en = 1;

     /*
      * Update NW process bypass cfg in NSS for ingress packets on radio
      */
     msg->bypass_nw_process = sc->nss_wifiol_bypass_nw_process;

     sc->nss_wifiol_ctx= nss_wifiol_ctx;
     printk("NSS-WifiOffload: nss_wifiol_ctx %p \n",nss_wifiol_ctx);

     /*
      * Send WIFI CE configure
      */
     nss_cmn_msg_init(&wifimsg.cm, ifnum, NSS_WIFI_INIT_MSG,
             sizeof(struct nss_wifi_init_msg), NULL, NULL);

     status = nss_wifi_tx_msg(nss_wifiol_ctx, &wifimsg);
     if (status != NSS_TX_SUCCESS) {
         printk("NSS-WifiOffload: wifi initialization fail%d \n", status);
         return -1;
     }
     mdelay(100);

     return 0;
}

void osif_nss_ol_post_recv_buffer( void *hifctx)
{

    struct ath_hif_pci_softc *sc;
    int ifnum ;
    struct nss_wifi_msg wifimsg;
    struct nss_wifi_init_msg *intmsg;
    nss_tx_status_t status;
    void *nss_wifiol_ctx;
    int radio_id;
    nss_wifi_msg_callback_t msg_cb;
    struct CE_state *CE_state;

    sc =  (struct ath_hif_pci_softc *)hifctx;
    nss_wifiol_ctx= sc->nss_wifiol_ctx;
    if (!nss_wifiol_ctx) {
        return;
    }

    radio_id = sc->nss_wifiol_id;
    CE_state = sc->CE_id_to_state[5];

    printk("%s %p %p %d \n",__func__,nss_wifiol_ctx,sc,radio_id);

    ifnum = osif_nss_ol_get_ifnum(radio_id);
    if(!ifnum){
        printk("no if found return ");
        return ;
    }

    memset(&wifimsg, 0, sizeof(struct nss_wifi_msg));
    intmsg = &wifimsg.msg.initmsg;

    msg_cb = (nss_wifi_msg_callback_t)osif_nss_ol_event_receive;

    /*
     * Send WIFI CE configure
     */
    nss_cmn_msg_init(&wifimsg.cm, ifnum, NSS_WIFI_POST_RECV_MSG,
            sizeof(struct nss_wifi_init_msg), msg_cb, NULL);

    status = nss_wifi_tx_msg(nss_wifiol_ctx, &wifimsg);
    if (status != NSS_TX_SUCCESS) {
        printk("WIFI CE configure failt %d \n", status);
        return;
    }
    CE_per_engine_disable_interupt((struct CE_handle *)CE_state);
    mdelay(100);
}

int osif_nss_ol_pdev_attach(void *hifctx, void *nss_wifiol_ctx,
        int radio_id,
        uint32_t desc_pool_size,
        uint32_t *tx_desc_array,
        uint32_t wlanextdesc_addr,
        uint32_t wlanextdesc_size,
        uint32_t htt_tx_desc_base_vaddr, uint32_t htt_tx_desc_base_paddr ,
        uint32_t htt_tx_desc_offset,
        uint32_t pmap_addr)
{

    struct ath_hif_pci_softc *sc;
    struct htt_pdev_t *htt_pdev;
    struct ol_txrx_pdev_t *pdev;
    int ifnum ;
    struct nss_wifi_msg wifimsg;
    struct nss_wifi_tx_init_msg *st ;
    nss_tx_status_t status;
    int mem_send = -1;

    if (!nss_wifiol_ctx) {
        return 0;
    }

    sc =  (struct ath_hif_pci_softc *)hifctx;
    htt_pdev = sc->scn->htt_pdev;
    pdev = htt_pdev->txrx_pdev;

    printk("%s :NSS %p NSS ID %d  target type  \n",__FUNCTION__, nss_wifiol_ctx, radio_id);
    printk("%s :NSS %p NSS ID %d \n",__FUNCTION__, nss_wifiol_ctx, radio_id);

    ifnum = osif_nss_ol_get_ifnum(radio_id);
    if(!ifnum){
        printk("no if found return ");
        return -1;
    }

    if (sc->scn->nss_wifiol_ce_enabled != 1) {
        printk(" CE init was unsuccesful \n");
        return -1;
    }

    memset(&wifimsg, 0, sizeof(struct nss_wifi_msg));
    st = &wifimsg.msg.pdevtxinitmsg;

    st->desc_pool_size   = desc_pool_size;
    st->tx_desc_array = 0;
    st->wlanextdesc_addr = wlanextdesc_addr;
    st->wlanextdesc_size = wlanextdesc_size;
    st->htt_tx_desc_base_vaddr = htt_tx_desc_base_vaddr;
    st->htt_tx_desc_base_paddr = htt_tx_desc_base_paddr;
    st->htt_tx_desc_offset = htt_tx_desc_offset;
    st->pmap_addr = pmap_addr;

    /*
     * Send WIFI CE configure
     */
    nss_cmn_msg_init(&wifimsg.cm, ifnum, NSS_WIFI_TX_INIT_MSG,
            sizeof(struct nss_wifi_tx_init_msg), NULL, NULL);

    status = nss_wifi_tx_msg(nss_wifiol_ctx, &wifimsg);
    if (status != NSS_TX_SUCCESS) {
        printk("NSS-WifiOffload: pdev init fail %d \n", status);
        return -1;
    }
    printk("NSS-WifiOffload: pdev init delay\n");
    mdelay(100);

    pdev->last_rra_pool_used = -1;
    pdev->last_peer_pool_used = -1;
    pdev->max_peers_per_pool = 67;
    pdev->max_rra_per_pool = 67;

    printk("sending peer pool memory\n");
    mem_send = osif_nss_ol_send_peer_memory(hifctx);
    if (mem_send != 0) {
        printk("Not able to send memory for peer to NSS\n");
        return -1;
    }
    mdelay(100);

    printk("sending rra pool memory\n");
    mem_send = osif_nss_ol_send_rra_memory(hifctx);
    if (mem_send != 0) {
        printk("Not able to send memory for rx reorder array to NSS\n");
        return -1;
    }
    mdelay(100);

    return 0;
}
#ifndef HTC_HEADER_LEN
#define HTC_HEADER_LEN 8
#endif
struct htc_frm_hdr_t {
    uint32_t   EndpointID : 8,
               Flags : 8,
               PayloadLen : 16;
    uint32_t   ControlBytes0 : 8,
               ControlBytes1 : 8,
               reserved : 16;
};

int osif_ol_nss_htc_send(void *nss_wifiol_ctx, int nssid, adf_nbuf_t netbuf, uint32_t transfer_id)
{

    struct htc_frm_hdr_t * htchdr;
    int status =0, len;

    if (!nss_wifiol_ctx) {
        return 0;
    }

    printk("%s :NSS %p NSS ID %d \n",__FUNCTION__, nss_wifiol_ctx, nssid);

    len =  adf_nbuf_len(netbuf);
    adf_nbuf_push_head(netbuf, HTC_HEADER_LEN);
    netbuf->len += HTC_HEADER_LEN;
    htchdr = (struct htc_frm_hdr_t *)adf_nbuf_data(netbuf);

    memset(htchdr, 0, HTC_HEADER_LEN);
    htchdr->EndpointID = transfer_id;
    htchdr->PayloadLen = len;

    osif_nss_ol_ce_raw_send(
            nss_wifiol_ctx,
            nssid,
            netbuf, len+HTC_HEADER_LEN);
    return status ;
}

void
htt_t2h_msg_handler_fast(void *htt_pdev, adf_nbuf_t *nbuf_cmpl_arr,
                         uint32_t num_cmpls, uint32_t *num_htt_cmpls);

int osif_nss_ol_htt_rx_init(void *htt_handle)
{
    struct htt_pdev_t *htt_pdev;
    void *nss_wifiol_ctx;
    int radio_id ;
    int ringsize;
    int fill_level;
    uint32_t paddrs_ringptr;
    uint32_t paddrs_ringpaddr;
    uint32_t alloc_idx_vaddr;
    uint32_t alloc_idx_paddr;
    int ifnum ;
    struct nss_wifi_msg wifimsg;
    struct nss_wifi_htt_init_msg *st ;
    nss_tx_status_t status;

    if (!htt_handle) {
        return 0;
    }


    htt_pdev = (struct htt_pdev_t *)htt_handle;
    nss_wifiol_ctx = htt_pdev->nss_wifiol_ctx;
    if (!nss_wifiol_ctx) {
        return 0;
    }

    radio_id = htt_pdev->nss_wifiol_id;
    ringsize = htt_pdev->rx_ring.size;
    fill_level = htt_pdev->rx_ring.fill_level;
    paddrs_ringptr = (uint32_t)htt_pdev->rx_ring.buf.paddrs_ring;
    paddrs_ringpaddr = (uint32_t)htt_pdev->rx_ring.base_paddr;
    alloc_idx_vaddr = (uint32_t)htt_pdev->rx_ring.alloc_idx.vaddr;
    alloc_idx_paddr = (uint32_t)htt_pdev->rx_ring.alloc_idx.paddr;

    printk("%s :NSS %p NSS ID %d \n",__FUNCTION__, nss_wifiol_ctx, radio_id);

    ifnum = osif_nss_ol_get_ifnum(radio_id);
    if(!ifnum){
        printk("no if found return ");
        return 1;
    }

    memset(&wifimsg, 0, sizeof(struct nss_wifi_msg));
    st = &wifimsg.msg.httinitmsg;

    st->radio_id = radio_id;
    st->ringsize         = ringsize;
    st->fill_level       = fill_level;
    st->paddrs_ringptr   = paddrs_ringptr;
    st->paddrs_ringpaddr = paddrs_ringpaddr;
    st->alloc_idx_vaddr  = alloc_idx_vaddr;
    st->alloc_idx_paddr  = alloc_idx_paddr;

    /*
     * Send WIFI CE configure
     */
    nss_cmn_msg_init(&wifimsg.cm, ifnum, NSS_WIFI_HTT_INIT_MSG,
            sizeof(struct nss_wifi_htt_init_msg), NULL, NULL);

    status = nss_wifi_tx_msg(nss_wifiol_ctx, &wifimsg);
    if (status != NSS_TX_SUCCESS) {
        printk("NSS-WifiOffload: htt rx configure fail %d \n", status);
        return 1;
    }

    printk("NSS-WifiOffload: htt rx configure  delay 100ms start \n");
    mdelay(100);
    return 0;
}

static int osif_nss_ol_ce_raw_send(void *nss_wifiol_ctx, uint32_t radio_id, adf_nbuf_t netbuf, uint32_t  len )
{
    int ifnum ;
    struct nss_wifi_msg wifimsg;
    struct nss_wifi_rawsend_msg *st ;
    nss_tx_status_t status;

    printk("%s :NSS %p NSS ID %d \n",__FUNCTION__, nss_wifiol_ctx, radio_id);
    ifnum = osif_nss_ol_get_ifnum(radio_id);
    if(!ifnum){
        printk("no if found return ");
        return 1;
    }

    memset(&wifimsg, 0, sizeof(struct nss_wifi_msg));
    st = &wifimsg.msg.rawmsg;

    st->len   = len;
    memcpy(st->array, netbuf->data, len);

    /*
     * Send WIFI CE configure
     */
    nss_cmn_msg_init(&wifimsg.cm, ifnum, NSS_WIFI_RAW_SEND_MSG,
            sizeof(struct nss_wifi_rawsend_msg), NULL, NULL);

    adf_nbuf_free(netbuf);

    status = nss_wifi_tx_msg(nss_wifiol_ctx, &wifimsg);
    if (status != NSS_TX_SUCCESS) {
        printk("WIFI CE configure failt %d \n", status);
        return 1;
    }
    printk("delay 100ms \n");
    mdelay(100);
    return 0;
}

static int osif_nss_ol_pdev_tx_configure(void *nss_wifiol_ctx,
        uint32_t radio_id,
        uint32_t  desc_pool_size,
        uint32_t  tx_desc_array,
        uint32_t wlanextdesc_addr,
        uint32_t wlanextdesc_size,
        uint32_t htt_tx_desc_base_vaddr,
        uint32_t htt_tx_desc_base_paddr ,
        uint32_t htt_tx_desc_offset,
        uint32_t pmap_addr)
{

    int ifnum ;
    struct nss_wifi_msg wifimsg;
    struct nss_wifi_tx_init_msg *st ;
    nss_tx_status_t status;

    printk("%s :NSS %p NSS ID %d \n",__FUNCTION__, nss_wifiol_ctx, radio_id);

    ifnum = osif_nss_ol_get_ifnum(radio_id);
    if(!ifnum){
        printk("no if found return ");
        return 1;
    }

    memset(&wifimsg, 0, sizeof(struct nss_wifi_msg));
    st = &wifimsg.msg.pdevtxinitmsg;
    st->desc_pool_size   = desc_pool_size;
    st->tx_desc_array = tx_desc_array;
    st->wlanextdesc_addr = wlanextdesc_addr;
    st->wlanextdesc_size = wlanextdesc_size;
    st->htt_tx_desc_base_vaddr = htt_tx_desc_base_vaddr;
    st->htt_tx_desc_base_paddr = htt_tx_desc_base_paddr;
    st->htt_tx_desc_offset = htt_tx_desc_offset;
    st->pmap_addr   = pmap_addr;

    /*
     * Send WIFI CE configure
     */
    nss_cmn_msg_init(&wifimsg.cm, ifnum, NSS_WIFI_TX_INIT_MSG,
            sizeof(struct nss_wifi_tx_init_msg), NULL, NULL);

    status = nss_wifi_tx_msg(nss_wifiol_ctx, &wifimsg);
    if (status != NSS_TX_SUCCESS) {
        printk("WIFI CE configure failt %d \n", status);
        return 1;
    }
    printk("delay 100ms start \n");
    mdelay(100);
    return 0;
}

int osif_nss_ol_pdev_add_wds_peer(ol_txrx_pdev_handle pdev, uint8_t *peer_mac, uint8_t *dest_mac)
{
    void *nss_wifiol_ctx;
    uint32_t radio_id;
    int ifnum ;
    struct nss_wifi_msg wifimsg;
    struct nss_wifi_wds_peer_msg *st ;
    nss_tx_status_t status;

    if (!pdev) {
        return 0;
    }

    nss_wifiol_ctx = pdev->nss_wifiol_ctx;
    radio_id = pdev->nss_wifiol_id;

    if (!nss_wifiol_ctx) {
        return 0;
    }

    printk("%s : Add NSS %p NSS ID %d peer: %02x:%02x:%02x:%02x:%02x:%02x "
           "dest: %02x:%02x:%02x:%02x:%02x:%02x\n",
           __FUNCTION__, nss_wifiol_ctx, radio_id,
           peer_mac[0], peer_mac[1], peer_mac[2],
           peer_mac[3], peer_mac[4], peer_mac[5],
           dest_mac[0], dest_mac[1], dest_mac[2],
           dest_mac[3], dest_mac[4], dest_mac[5]);
    ifnum = osif_nss_ol_get_ifnum(radio_id);
    if(!ifnum){
        printk("no if found return ");
        return 1;
    }

    memset(&wifimsg, 0, sizeof(struct nss_wifi_msg));
    st = &wifimsg.msg.pdevwdspeermsg;
    memcpy(st->peer_mac, peer_mac, 6);
    memcpy(st->dest_mac, dest_mac, 6);

    /*
     * Send WIFI CE configure
     */
    nss_cmn_msg_init(&wifimsg.cm, ifnum, NSS_WIFI_WDS_PEER_ADD_MSG,
            sizeof(struct nss_wifi_tx_init_msg), NULL, NULL);

    status = nss_wifi_tx_msg(nss_wifiol_ctx, &wifimsg);
    if (status != NSS_TX_SUCCESS) {
        printk("WIFI CE configure failt %d \n", status);
        return 1;
    }

    return 0;
}



int osif_nss_ol_pdev_del_wds_peer(ol_txrx_pdev_handle pdev, uint8_t *peer_mac, uint8_t *dest_mac)
{
    void *nss_wifiol_ctx;
    uint32_t radio_id;
    int ifnum ;
    struct nss_wifi_msg wifimsg;
    struct nss_wifi_wds_peer_msg *st ;
    nss_tx_status_t status;

    if (!pdev) {
        return 0;
    }

    nss_wifiol_ctx = pdev->nss_wifiol_ctx;
    if (!nss_wifiol_ctx) {
        return 0;
    }

    radio_id = pdev->nss_wifiol_id;

    printk("%s : Delete NSS %p NSS ID %d peer:  %02x:%02x:%02x:%02x:%02x:%02x "
           "dest: %02x:%02x:%02x:%02x:%02x:%02x\n",
           __FUNCTION__, nss_wifiol_ctx, radio_id,
           peer_mac[0], peer_mac[1], peer_mac[2],
           peer_mac[3], peer_mac[4], peer_mac[5],
           dest_mac[0], dest_mac[1], dest_mac[2],
           dest_mac[3], dest_mac[4], dest_mac[5]);
    ifnum = osif_nss_ol_get_ifnum(radio_id);
    if(!ifnum){
        printk("NSS-WifOffoad: No if num\n");
        return 1;
    }

    memset(&wifimsg, 0, sizeof(struct nss_wifi_msg));
    st = &wifimsg.msg.pdevwdspeermsg;
    memcpy(st->peer_mac, peer_mac, 6);
    memcpy(st->dest_mac, dest_mac, 6);

    /*
     * Send peer del message
     */
    nss_cmn_msg_init(&wifimsg.cm, ifnum, NSS_WIFI_WDS_PEER_DEL_MSG,
            sizeof(struct nss_wifi_tx_init_msg), NULL, NULL);

    status = nss_wifi_tx_msg(nss_wifiol_ctx, &wifimsg);
    if (status != NSS_TX_SUCCESS) {
        printk("NSS-WifOffoad: del wds fail %d \n", status);
        return 1;
    }

    return 0;
}

