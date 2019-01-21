/*
 * Copyright (c) 2015-2016 Qualcomm Atheros, Inc.
 *
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.�
 */

/*
 * osif_nss_wifiol_vdev_if.c
 *
 * This file used for for interface to NSS WiFi Offload  VAP
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

#include <linux/if_arp.h>
#include <linux/etherdevice.h>
#include <linux/in.h>
#include <asm/cacheflush.h>
#include "osif_private.h"
#include <ol_txrx_api.h>
#include <ol_ctrl_txrx_api.h>
#include <nss_api_if.h>
#include <nss_cmn.h>
#include <adf_nbuf.h>
#include <ol_txrx_types.h>
#include <ol_txrx_peer_find.h>


#include "osif_private.h"
#include <ol_txrx_types.h>
#include "osif_nss_wifiol_vdev_if.h"

#if MESH_MODE_SUPPORT
#include <if_meta_hdr.h>
#endif

static struct osif_nss_vdev_cfg_pvt osif_nss_vdev_cfgp;
static struct osif_nss_vdev_vow_dbg_stats_pvt osif_nss_vdev_vowp;

#define OL_TXRX_LIST_APPEND(head, tail, elem) \
    do {                                            \
        if (!(head)) {                              \
            (head) = (elem);                        \
        } else {                                    \
            adf_nbuf_set_next((tail), (elem));      \
        }                                           \
        (tail) = (elem);                            \
    } while (0)

/*
 * This file is responsible for interacting with qca-nss-drv's
 * WIFI to manage WIFI VDEVs.
 *
 * This driver also exposes few APIs which can be used by
 * another module to perform operations on CAPWAP tunnels. However, we create
 * one netdevice for all the CAPWAP tunnels which is done at the module's
 * init time if NSS_wifimgr_ONE_NETDEV is set in the Makefile.
 *
 * If your requirement is to create one netdevice per-CAPWAP tunnel, then
 * netdevice needs to be created before CAPWAP tunnel create. Netdevice are
 * created using nss_wifimgr_netdev_create() API.
 *
 */
#define OSIF_NSS_DEBUG_LEVEL 4

/*
 * NSS capwap mgr debug macros
 */
#if (OSIF_NSS_DEBUG_LEVEL < 1)
#define osif_nss_assert(fmt, args...)
#else
#define osif_nss_assert(c) if (!(c)) { BUG_ON(!(c)); }
#endif /* OSIF_NSS_DEBUG_LEVEL */

/*
 * Compile messages for dynamic enable/disable
 */
#if defined(CONFIG_DYNAMIC_DEBUG)
#define osif_nss_warn(s, ...) pr_debug("%s[%d]:" s, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define osif_nss_info(s, ...) pr_debug("%s[%d]:" s, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define osif_nss_trace(s, ...) pr_debug("%s[%d]:" s, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#else /* CONFIG_DYNAMIC_DEBUG */
/*
 * Statically compile messages at different levels
 */
#if (OSIF_NSS_DEBUG_LEVEL < 2)
#define osif_nss_warn(s, ...)
#else
#define osif_nss_warn(s, ...) pr_warn("%s[%d]:" s, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#endif

#if (OSIF_NSS_DEBUG_LEVEL < 3)
#define osif_nss_info(s, ...)
#else
#define osif_nss_info(s, ...)   pr_notice("%s[%d]:" s, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#endif

#if (OSIF_NSS_DEBUG_LEVEL < 4)
#define osif_nss_trace(s, ...)
#else
#define osif_nss_trace(s, ...)  pr_info("%s[%d]:" s, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#endif
#endif /* CONFIG_DYNAMIC_DEBUG */
struct net_device *osif_nss_vdev_process_mpsta_rx(struct net_device *netdev, struct sk_buff *skb, uint32_t peer_id);
void populate_tx_info(struct ol_ath_softc_net80211 *scn, adf_nbuf_t  netbuf, int i, int num_msdus, enum htt_tx_status status, struct ieee80211_tx_status  *ts);

#if DBDC_REPEATER_SUPPORT
int dbdc_rx_process (os_if_t *osif ,struct net_device **dev ,wlan_if_t vap, struct sk_buff *skb, int *nwifi);
int dbdc_tx_process (wlan_if_t vap, osif_dev **osdev , struct sk_buff *skb);
#endif

/*
 * osif_nss_radio_get_type()
 * 	The caller of this API is expected to do radio id validation
 */
static enum nss_dynamic_interface_type osif_nss_radio_get_type(uint32_t radio_id)
{
    switch(radio_id) {
        case 0:
            return NSS_DYNAMIC_INTERFACE_TYPE_RADIO_0;
        case 1:
            return NSS_DYNAMIC_INTERFACE_TYPE_RADIO_1;
        case 2:
            return NSS_DYNAMIC_INTERFACE_TYPE_RADIO_2;
    }

    printk("Invalid radio_id :%d\n", radio_id);
    return NSS_DYNAMIC_INTERFACE_TYPE_NONE;
}

int osif_nss_vdev_tx_raw(ol_txrx_vdev_handle vdev, adf_nbuf_t *pnbuf)
{
    adf_nbuf_t nbuf_list_head = NULL;
    adf_nbuf_t next = NULL;

    if (!vdev) {
        return 1;
    }

    nbuf_list_head = *pnbuf;
    if (nbuf_list_head->next == NULL) {
        return 0;
    }

    skb_frag_list_init(nbuf_list_head);

    nbuf_list_head->data_len = 0;
    next = nbuf_list_head->next;
    nbuf_list_head->next = NULL;
    if (!skb_has_frag_list(nbuf_list_head)) {
        skb_shinfo(nbuf_list_head)->frag_list = next;
    }

    skb_walk_frags(nbuf_list_head, next) {
        nbuf_list_head->len += adf_nbuf_len(next);
        nbuf_list_head->data_len += adf_nbuf_len(next);
        nbuf_list_head->truesize += adf_nbuf_len(next);
    }

    return 0;
}

void osif_nss_vdev_deliver_rawbuf(struct net_device *netdev, adf_nbuf_t msdu_list,  struct napi_struct *napi)
{
    adf_nbuf_t nbuf = msdu_list;
    adf_nbuf_t next = NULL;

    dev_hold(netdev);
    while (nbuf)
    {
        next = nbuf->next;
        nbuf->dev = netdev;
        nbuf->next = NULL;
        nbuf->protocol = eth_type_trans(nbuf, netdev);
        /*
         * GRO enable on wifi is causing 4n aligned frame sent to NSS
         * avoid GRO processing in stack for raw simulator output frames
         */
        netif_receive_skb(nbuf);
        nbuf = next;
    }
    dev_put(netdev);
}

void osif_nss_vdev_handle_rawbuf(struct net_device *netdev,
        adf_nbuf_t nbuf, __attribute__((unused)) struct napi_struct *napi)
{
    struct ol_txrx_vdev_t *vdev = NULL;
    struct ol_txrx_pdev_t *pdev = NULL;
    struct ol_txrx_peer_t *peer = NULL;
    struct nss_wifi_vdev_rawmode_rx_metadata *wmrm;
    uint16_t peer_id;

    adf_nbuf_t deliver_list_head = NULL;
    adf_nbuf_t deliver_list_tail = NULL;
    adf_nbuf_t tmp;
    osif_dev  *osdev;
    struct ieee80211vap *vap = NULL;

    osdev = ath_netdev_priv(netdev);
    vdev = osdev->iv_txrx_handle;
    pdev = vdev->pdev;

    wmrm = (struct nss_wifi_vdev_rawmode_rx_metadata *)nbuf->data;
    peer_id = wmrm->peer_id;
    skb_pull(nbuf, sizeof(struct nss_wifi_vdev_rawmode_rx_metadata));

    peer = ol_txrx_peer_find_by_id(pdev, peer_id);
    if (!peer) {
        printk("no peer available free packet \n");
        dev_kfree_skb_any(nbuf);
        return;
    }
    /*
     * Check if SKB has fraglist and convert the fraglist to
     * skb->next chains
     */
    memset(nbuf->cb, 0x0, sizeof(nbuf->cb));
    /* printk("\nRcvd pkt:0x%x of length:%d head:0x%X data:0x%x", nbuf, nbuf->len, nbuf->head, nbuf->data); */
    OL_TXRX_LIST_APPEND(deliver_list_head, deliver_list_tail, nbuf);
    if (skb_has_frag_list(nbuf)) {
        adf_nbuf_set_chfrag_start(nbuf, 1);
        nbuf->len  = skb_headlen(nbuf);
        nbuf->truesize -= nbuf->data_len;
        nbuf->data_len = 0;
        nbuf->next = skb_shinfo(nbuf)->frag_list;

        /* printk("\nSKB has fraglist"); */
        skb_walk_frags(nbuf, tmp) {
            OL_TXRX_LIST_APPEND(deliver_list_head, deliver_list_tail, tmp);
        }
        adf_nbuf_set_chfrag_end(deliver_list_tail, 1);
        /*
         * Reset the skb frag list
         */
        skb_frag_list_init(nbuf);
    } else {
        if (nbuf->next != NULL) {
            printk("\nUn Expected");
        }
    }

    vap = ol_ath_getvap(vdev);
    if ( vap->iv_rawmode_pkt_sim) {
	ol_rsim_rx_decap(vdev, peer, &deliver_list_head, &deliver_list_tail);
    }
    osif_nss_vdev_deliver_rawbuf(netdev, deliver_list_head, napi);
}

static void osif_nss_vdev_frag_to_chain(struct ol_txrx_vdev_t *vdev, adf_nbuf_t nbuf, adf_nbuf_t *list_head, adf_nbuf_t *list_tail)
{
    adf_nbuf_t deliver_list_head = NULL;
    adf_nbuf_t deliver_list_tail = NULL;
    adf_nbuf_t tmp;

    /*
     * Check if SKB has fraglist and convert the fraglist to
     * skb->next chains
     */
    memset(nbuf->cb, 0x0, sizeof(nbuf->cb));
    OL_TXRX_LIST_APPEND(deliver_list_head, deliver_list_tail, nbuf);
    if (skb_has_frag_list(nbuf)) {
        adf_nbuf_set_chfrag_start(nbuf, 1);
        nbuf->len  = skb_headlen(nbuf);
        nbuf->truesize -= nbuf->data_len;
        nbuf->data_len = 0;
        nbuf->next = skb_shinfo(nbuf)->frag_list;

        /* printk("\nSKB has fraglist"); */
        skb_walk_frags(nbuf, tmp) {
            OL_TXRX_LIST_APPEND(deliver_list_head, deliver_list_tail, tmp);
        }
        adf_nbuf_set_chfrag_end(deliver_list_tail, 1);

        /*
         * Reset the skb frag list
         */
        skb_frag_list_init(nbuf);
    } else {
        if (nbuf->next != NULL) {
            printk("\nUn Expected");
        }
    }

    *list_head = deliver_list_head;
    *list_tail = deliver_list_tail;
}

/*
 * osif_nss_vdev_event_receive()
 * 	Event callback to receive event data from NSS
 */
static void osif_nss_vdev_event_receive(void *if_ctx, struct nss_cmn_msg *wifivdevmsg)
{
    /*
     * TODO : Need to add stats updates
     */
    return;
}

static void osif_nss_vdev_peer_tx_buf(struct ol_txrx_vdev_t *vdev, struct sk_buff *skb, uint16_t peer_id)
{
    struct nss_wifi_vdev_msg wifivdevmsg;
    struct nss_wifi_vdev_txmsg *vdev_tx_special_data;
    bool status;
    uint8_t *data;
    void *nss_wifiol_ctx = vdev->pdev->nss_wifiol_ctx;
    adf_nbuf_t nbuf;
    uint32_t msg_len;
    osif_dev *osif =  (osif_dev *)vdev->osif_vdev;

    if (!nss_wifiol_ctx) {
        return;
    }

    msg_len =  sizeof(struct nss_wifi_vdev_txmsg);

    vdev_tx_special_data = &wifivdevmsg.msg.vdev_txmsgext;
    memset(&wifivdevmsg, 0, sizeof(struct nss_wifi_vdev_msg));

    vdev_tx_special_data->peer_id = peer_id;

    nss_wifi_vdev_msg_init(&wifivdevmsg, osif->nss_ifnum, NSS_WIFI_VDEV_SPECIAL_DATA_TX_MSG,
            msg_len , NULL, NULL);

    /*
     * Allocate a container SKB for carrying the message + SKB
     */
    if(!(nbuf = adf_nbuf_alloc(NULL, msg_len, 0, 4, FALSE))) {
        osif_nss_warn("Skb allocation failed \n");
        adf_nbuf_free(skb);
        return;
    }

    /*
     * Copy the message to the container skb
     */
    data = skb_put(nbuf, sizeof(struct nss_wifi_vdev_msg));
    memcpy(data, &wifivdevmsg, sizeof(struct nss_wifi_vdev_msg));

    /*
     * Copy buffer data into the newly allocated container buffer
     */
    data = skb_put(nbuf, skb->len);
    memcpy(data, skb->data, skb->len);

    /*
     * Send the vdev special data to NSS
     */
    status = nss_wifi_vdev_tx_msg_ext(nss_wifiol_ctx, nbuf);

    if (status != NSS_TX_SUCCESS) {
        adf_nbuf_free(nbuf);
        osif_nss_warn("Unable to send the grp_list create message to NSS\n");
    }

    adf_nbuf_free(skb);
}

/*
 * osif_nss_vdev_tx_inspect_handler()
 *	Handler for tx inspect packets exceptioned from WIFI
 */
static void osif_nss_vdev_tx_inspect_handler(struct ol_txrx_vdev_t *vdev, struct sk_buff *skb)
{
    struct ol_txrx_peer_t *peer;
    struct sk_buff *skb_copy;
    uint16_t peer_id = HTT_INVALID_PEER;

    if (vdev->osif_proxy_arp(vdev->osif_vdev, skb)) {
        goto out;
    }

    TAILQ_FOREACH(peer, &vdev->peer_list, peer_list_elem) {
        if (peer && (peer->peer_ids[0] != HTT_INVALID_PEER) && (peer->bss_peer)) {
            peer_id = ol_tx_classify_get_peer_id(vdev, skb);
            if (peer_id == HTT_INVALID_PEER) {
                goto out;
            }

            skb_copy = adf_nbuf_copy(skb);
            if (skb_copy) {
                adf_nbuf_reset_ctxt(skb_copy);
                osif_nss_vdev_peer_tx_buf(vdev, skb_copy, peer_id);
            }
        }
    }

out:
    adf_nbuf_free(skb);
}

/*
 * osif_nss_vdev_skb_needs_linearize()
 * 	Check if skb needs linearlize
 */
bool osif_nss_vdev_skb_needs_linearize(struct net_device *dev, struct sk_buff *skb)
{
    osif_dev *osdev;
    struct ieee80211vap *vap;

    osdev = netdev_priv(dev);
    vap = osdev->os_if;

    /*
     * if skb does not have fraglist return true
     */
    if (!skb_has_frag_list(skb)) {
        return true;
    }

    /*
     * Linearize skb if the mode in non monitor mode and rx decap type is not raw
     */
    if ((osdev->os_opmode != IEEE80211_M_MONITOR) && (vap->iv_rx_decap_type != osif_pkt_type_raw)) {
        return true;
    }

    return false;
}

#if MESH_MODE_SUPPORT
extern void os_if_tx_free_ext(struct sk_buff *skb);
#endif

/*
 * osif_nss_vdev_special_data_receive()
 *	Handler for data packets exceptioned from WIFI
 */
static void osif_nss_vdev_special_data_receive(struct net_device *dev, struct sk_buff *skb, __attribute__((unused)) struct napi_struct *napi)
{
    struct net_device *netdev;
    struct nss_wifi_vdev_per_packet_metadata *wifi_metadata = NULL;
    struct nss_wifi_vdev_igmp_per_packet_metadata *igmp_metadata = NULL;
    struct nss_wifi_vdev_txinfo_per_packet_metadata *txinfo_metadata = NULL;
    struct nss_wifi_vdev_mpsta_per_packet_rx_metadata *mpsta_rx_metadata = NULL;
    struct nss_wifi_vdev_rx_err_per_packet_metadata *rx_err_metadata = NULL;

    struct nss_wifi_vdev_mesh_per_packet_metadata *mesh_metadata = NULL;
    osif_dev *osifp;
    struct ol_txrx_pdev_t *pdev;
    struct ol_txrx_vdev_t *vdev;
    int discard = 0;
    struct ieee80211vap *vap;
    struct ol_ath_vap_net80211 *avn;
    struct ol_ath_softc_net80211 *scn;
    uint32_t peer_id = HTT_INVALID_PEER;
    struct net_device *rxnetdev;
#if ATH_DATA_TX_INFO_EN
    struct ieee80211_tx_status *ts;
#endif

#if MESH_MODE_SUPPORT
    struct meta_hdr_s *mhdr = NULL;
    struct ieee80211com *ic = NULL;
#endif

    if(dev == NULL) {
        printk(KERN_CRIT "%s , netdev is NULL, freeing skb", __func__);
        dev_kfree_skb_any(skb);
        return;
    }

    netdev = (struct net_device *)dev;
    dev_hold(netdev);

    osifp = netdev_priv(netdev);
    vdev = osifp->iv_txrx_handle;
    pdev = vdev->pdev;

    skb->dev = netdev;
    vap = osifp->os_if;
    avn = OL_ATH_VAP_NET80211(vap);
    scn = avn->av_sc;

    /*
     * non linear  skb with fraglist are processed in nss offload only if monitor mode is enabled
     * or decap type is raw otherwise linearize them
     */
    if (skb_is_nonlinear(skb)) {
        if (osif_nss_vdev_skb_needs_linearize(netdev, skb) && (__skb_linearize(skb))) {
            dev_kfree_skb_any(skb);
            dev_put(netdev);
            return;
        }
    }

    skb = skb_unshare(skb, GFP_ATOMIC);
    if (!skb) {
        dev_put(netdev);
        return;
    }

    dma_unmap_single (NULL, virt_to_phys(skb->head), NSS_WIFI_VDEV_PER_PACKET_METADATA_OFFSET + sizeof(struct nss_wifi_vdev_per_packet_metadata), DMA_FROM_DEVICE);

    wifi_metadata = (struct nss_wifi_vdev_per_packet_metadata *)(skb->head + NSS_WIFI_VDEV_PER_PACKET_METADATA_OFFSET);

    switch (wifi_metadata->pkt_type) {
        case NSS_WIFI_VDEV_EXT_DATA_PKT_TYPE_IGMP: {
            igmp_metadata = (struct nss_wifi_vdev_igmp_per_packet_metadata *)&wifi_metadata->metadata.igmp_metadata;

            /*
             * Notify UMAC to update wifi snooptable.
             */
            discard = ol_rx_notify(pdev->ctrl_pdev, vdev->vdev_id, igmp_metadata->peer_mac_addr, igmp_metadata->tid, igmp_metadata->tsf32,
                    OL_RX_NOTIFY_IPV4_IGMP, skb);

            if (discard) {
                dev_kfree_skb_any(skb);
                break;
            }

            skb->protocol = eth_type_trans(skb, netdev);
            /*
             * Send this skb to stack also.
             */
            napi_gro_receive(napi, skb);
            break;
       }

        case NSS_WIFI_VDEV_EXT_DATA_PKT_TYPE_MESH: {
            mesh_metadata = (struct nss_wifi_vdev_mesh_per_packet_metadata *)&wifi_metadata->metadata.mesh_metadata;

#if MESH_MODE_SUPPORT
            ic = &scn->sc_ic;

            if (adf_nbuf_headroom(skb) < sizeof(struct meta_hdr_s)) {
                printk("Unable to accomodate mesh mode meta header\n");
                dev_kfree_skb_any(skb);
                break;
            }

            adf_nbuf_push_head(skb, sizeof(struct meta_hdr_s));

            mhdr = (struct meta_hdr_s *)adf_nbuf_data(skb);
            mhdr->rssi =  mesh_metadata->rssi;
            mhdr->channel = ol_ath_mhz2ieee(ic, pdev->rx_mon_recv_status->rs_freq, 0);
            os_if_tx_free_ext(skb);
#else
            dev_kfree_skb_any(skb);
#endif
            break;
        }

        case NSS_WIFI_VDEV_EXT_DATA_PKT_TYPE_INSPECT: {
            osif_nss_vdev_tx_inspect_handler(vdev, skb);
            break;
        }

        case NSS_WIFI_VDEV_EXT_DATA_PKT_TYPE_TXINFO: {
            txinfo_metadata = (struct nss_wifi_vdev_txinfo_per_packet_metadata *)&wifi_metadata->metadata.txinfo_metadata;

#if ATH_DATA_TX_INFO_EN
            if(scn->enable_perpkt_txstats) {
                ts = scn->tx_status_buf;
                ts->ppdu_rate = txinfo_metadata->ppdu_rate;
                ts->ppdu_num_mpdus_success = txinfo_metadata->ppdu_num_mpdus_success;
                ts->ppdu_num_mpdus_fail = txinfo_metadata->ppdu_num_mpdus_fail;
                ts->ppdu_num_msdus_success = txinfo_metadata->ppdu_num_msdus_success;
                ts->ppdu_bytes_success = txinfo_metadata->ppdu_bytes_success;
                ts->ppdu_duration = txinfo_metadata->ppdu_duration;
                ts->ppdu_retries = txinfo_metadata->ppdu_retries;
                ts->ppdu_is_aggregate = txinfo_metadata->ppdu_is_aggregate;

                NBUF_SET_SUBMIT_TS(skb, txinfo_metadata->msdu_q_time);
                populate_tx_info(scn, skb, txinfo_metadata->msdu_count, txinfo_metadata->num_msdu, txinfo_metadata->status, ts);
            }
#endif
            dev_kfree_skb_any(skb);
            break;
        }

        case NSS_WIFI_VDEV_EXT_DATA_PKT_TYPE_MPSTA_RX: {
            if (vap->iv_mpsta) {
                mpsta_rx_metadata = (struct nss_wifi_vdev_mpsta_per_packet_rx_metadata *)&wifi_metadata->metadata.mpsta_rx_metadata;
                peer_id = mpsta_rx_metadata->peer_id;
                rxnetdev  = osif_nss_vdev_process_mpsta_rx(netdev, skb, peer_id);
                dev_put(netdev);
                if (!rxnetdev) {
                    return;
                }
                netdev = rxnetdev;
                dev_hold(netdev);
                skb->dev = netdev;
                osif_nss_vdev_process_mpsta_rx_to_tx(netdev, skb, napi);
                dev_put(netdev);
                return;
            }
            break;
        }


        case NSS_WIFI_VDEV_EXT_DATA_PKT_TYPE_MPSTA_TX: {
            if (vap->iv_mpsta || vap->iv_psta) {
#if UMAC_SUPPORT_WNM
            if (wlan_wnm_tfs_filter(vap, (wbuf_t) skb)) {
                if (skb != NULL) {
                    dev_kfree_skb_any(skb);
                    dev_put(netdev);
                    return;
                }
            }
#endif

#if DBDC_REPEATER_SUPPORT
                if (dbdc_tx_process(vap, &osifp, skb)) {
                    dev_put(netdev);
                    return;
                }
#endif
                osif_nss_vdev_process_mpsta_tx(netdev, skb);
                dev_put(netdev);
                return;
            } else {
                dev_kfree_skb_any(skb);
                dev_put(netdev);
                return;
            }
            break;
        }

        case NSS_WIFI_VDEV_EXT_DATA_PKT_TYPE_EXTAP_TX: {
            if (IEEE80211_VAP_IS_EXT_AP_ENABLED(vap) && (vap->iv_opmode == IEEE80211_M_STA)) {
#if UMAC_SUPPORT_WNM
            if (wlan_wnm_tfs_filter(vap, (wbuf_t) skb)) {
                if (skb != NULL) {
                    dev_kfree_skb_any(skb);
                    dev_put(netdev);
                    return;
                }
            }
#endif

#if DBDC_REPEATER_SUPPORT
                if (dbdc_tx_process(vap, &osifp, skb)) {
                    dev_put(netdev);
                    return;
                }
#endif
                osif_nss_vdev_process_extap_tx(netdev, skb);
                dev_put(netdev);
                return;
            } else {
                dev_kfree_skb_any(skb);
                dev_put(netdev);
                return;
            }
            break;
        }

        case NSS_WIFI_VDEV_EXT_DATA_PKT_TYPE_EXTAP_RX: {
            if (IEEE80211_VAP_IS_EXT_AP_ENABLED(vap) && (vap->iv_opmode == IEEE80211_M_STA)) {
                osif_nss_vdev_process_extap_rx_to_tx(netdev, skb);
                dev_put(netdev);
                return;
            } else {
                dev_kfree_skb_any(skb);
                dev_put(netdev);
                return;
            }
            break;
        }

        case NSS_WIFI_VDEV_EXT_DATA_PKT_TYPE_RX_ERR: {
            rx_err_metadata = (struct nss_wifi_vdev_rx_err_per_packet_metadata*)&wifi_metadata->metadata.rx_err_metadata;

            /*
             * Notify hostapd for WPA countermeasures
             */
            ol_rx_err(pdev->ctrl_pdev, rx_err_metadata->vdev_id, rx_err_metadata->peer_mac_addr,
                                              rx_err_metadata->tid, 0, OL_RX_ERR_TKIP_MIC, skb);
            break;
        }

        case NSS_WIFI_VDEV_EXT_DATA_PKT_TYPE_WNM_TFS: {
            osif_nss_vdev_process_wnm_tfs_tx(netdev, skb);
            break;
        }

        default:
            printk("wrong special pkt type %d\n", wifi_metadata->pkt_type);
            dev_kfree_skb_any(skb);
    }

    dev_put(netdev);
}

/*
 * osif_nss_vdev_handle_monitor_mode()
 *	handle monitor mode, returns false if packet is consumed
 */
static bool osif_nss_vdev_handle_monitor_mode(struct net_device *netdev, struct sk_buff *skb, __attribute__((unused)) struct napi_struct *napi)
{
    osif_dev  *osdev;
    struct ol_txrx_vdev_t *vdev;
    struct ieee80211vap *vap;
    struct ol_txrx_pdev_t *pdev;
    htt_pdev_handle htt_pdev;
    struct ol_txrx_peer_t *peer = NULL;
    struct nss_wifi_vdev_rawmode_rx_metadata *wmrm;
    uint16_t peer_id;
    struct ol_ath_softc_net80211 *scn = NULL;
    struct ieee80211com* ic = NULL;
    struct ether_header *eh = NULL;
    struct ieee80211_frame *wh = NULL;
    uint32_t is_mcast = 0;

    adf_nbuf_t skb_list_head = NULL;
    adf_nbuf_t skb_list_tail = NULL;
    adf_nbuf_t skb_next = NULL;
    adf_nbuf_t mon_skb = NULL;

    uint32_t rx_desc_size = 0;
    void *rx_mpdu_desc = NULL;

    osdev = ath_netdev_priv(netdev);
    vap = osdev->os_if;
    vdev = osdev->iv_txrx_handle;
    pdev = vdev->pdev;
    htt_pdev = pdev->htt_pdev;

    scn = (struct ol_ath_softc_net80211 *)pdev;
    ic = &(scn->sc_ic);

    /*
     * if decap mode is raw retrive peer_id information
     */
    if (vdev->rx_decap_type == htt_pkt_type_raw) {
        wmrm = (struct nss_wifi_vdev_rawmode_rx_metadata *)skb->data;
        peer_id = wmrm->peer_id;
        skb_pull(skb, sizeof(struct nss_wifi_vdev_rawmode_rx_metadata));

        peer = ol_txrx_peer_find_by_id(pdev, peer_id);
        if (!peer) {
            printk("no peer available free packet \n");
            dev_kfree_skb_any(skb);
            return false;
        }
    }

    /*
     * Always use the RA for peer lookup when vap is operating in 802.11 (raw) mode and expects 802.11 frames
     * because we always have to queue to corresponding  STA bss peer queue.
     *
     * In Normal mode (802.3) use the destination mac address from eth_hdr.
     */
    if (vdev->rx_decap_type == htt_pkt_type_raw) {
        wh = (struct ieee80211_frame *)skb->data;
        is_mcast = IEEE80211_IS_MULTICAST(wh->i_addr1);
    } else {
        eh = (struct ether_header *)(skb->data);
        is_mcast = IEEE80211_IS_MULTICAST(eh->ether_dhost);
    }

    osif_nss_vdev_frag_to_chain(vdev, skb, &skb_list_head, &skb_list_tail);

    if ((vdev->opmode == wlan_op_mode_monitor) || vap->iv_special_vap_mode || vap->iv_smart_monitor_vap) {
        rx_desc_size = htt_pdev->ar_rx_ops->sizeof_rx_desc();
        rx_mpdu_desc = (void *)skb->head;
        dma_unmap_single (NULL, virt_to_phys(rx_mpdu_desc), rx_desc_size, DMA_FROM_DEVICE);

        mon_skb = htt_pdev->ar_rx_ops->restitch_mpdu_from_msdus(
                htt_pdev->arh, skb_list_head, pdev->rx_mon_recv_status, 1);

        if (!mon_skb) {
            mon_skb = skb_list_head;
            while (mon_skb) {
                skb_next = adf_nbuf_next(mon_skb);
                adf_nbuf_free(mon_skb);
                mon_skb = skb_next;
            }
            return false;
        }
    } else if (is_mcast && (ic->mon_filter_mcast_data != 1)) {  /* check if mcast data filters are enabled */
        goto skip_monitor;
    } else if ((!is_mcast) && (ic->mon_filter_ucast_data != 1)) { /* check if ucast data filters are enabled */
        goto skip_monitor;
    } else {
        mon_skb = htt_pdev->ar_rx_ops->restitch_mpdu_from_msdus(
                htt_pdev->arh, skb_list_head, pdev->rx_mon_recv_status, 0);
    }

    if (mon_skb) {
        pdev->monitor_vdev->osif_rx_mon(pdev->monitor_vdev->osif_vdev, mon_skb, pdev->rx_mon_recv_status);
    }

    if ((vdev->opmode == wlan_op_mode_monitor) || vap->iv_special_vap_mode || vap->iv_smart_monitor_vap) {
        return false;
    }

skip_monitor:

    if (vdev->rx_decap_type == htt_pkt_type_raw) {
        vap = ol_ath_getvap(vdev);
        if (vap->iv_rawmode_pkt_sim) {
	    ol_rsim_rx_decap(vdev, peer, &skb_list_head, &skb_list_tail);
        }
        osif_nss_vdev_deliver_rawbuf(netdev, skb_list_head, napi);
        return false;
    }

    return true;
}

/*
 * osif_nss_vdev_data_receive()
 *	Handler for data packets exceptioned from WIFI
 */
static void
osif_nss_vdev_data_receive(struct net_device *dev, struct sk_buff *skb, __attribute__((unused)) struct napi_struct *napi)
{
    struct net_device *netdev;
    uint8_t *data;
    osif_dev  *osdev;
    struct ol_txrx_vdev_t *vdev;
    struct ieee80211vap *vap;
    struct ol_txrx_pdev_t *pdev;
    os_if_t osif = NULL;
    int nwifi;

#if ATH_DATA_RX_INFO_EN
    htt_pdev_handle htt_pdev;
    adf_nbuf_t msdu = (adf_nbuf_t) skb;
#endif

    /*
     * Need to move this code to wifi driver
     */
    if(dev == NULL) {
        printk(KERN_CRIT "%s , netdev is NULL, freeing skb", __func__);
        dev_kfree_skb_any(skb);
        return;
    }

    netdev = (struct net_device *)dev;

    osdev = ath_netdev_priv(netdev);
    vap = osdev->os_if;
    vdev = osdev->iv_txrx_handle;

    /*
     * It is possible that invalid_peer, monitor_vap frames reach here through wifi exception
     * path and VAP is being deleted. Adding this check for protection against such scenarios.
     */
    if ((vap == NULL) || (vdev == NULL)) {
        dev_kfree_skb_any(skb);
        return;
    }

    osif = vap->iv_ifp;
    pdev = vdev->pdev;

    /*
     * non linear  skb with fraglist are processed in nss offload only if monitor mode is enabled
     * or decap type is raw otherwise linearize them
     */
    if (skb_is_nonlinear(skb)) {
        if (osif_nss_vdev_skb_needs_linearize(netdev, skb) && (__skb_linearize(skb))) {
            dev_kfree_skb_any(skb);
            return;
        }
    }

#if ATH_DATA_RX_INFO_EN
    if (vap->rxinfo_perpkt) {
        htt_pdev = pdev->htt_pdev;
        htt_pdev->ar_rx_ops->update_pkt_info( htt_pdev->arh, msdu, NULL, 1 );
    }
#endif

    data = (uint8_t *)skb->data;

    adf_os_spin_lock_bh(&pdev->mon_mutex);
    if (pdev->monitor_vdev) {
        if (!osif_nss_vdev_handle_monitor_mode(netdev, skb, napi)) {
            adf_os_spin_unlock_bh(&pdev->mon_mutex);
            return;
        }
    }
    adf_os_spin_unlock_bh(&pdev->mon_mutex);
    if ((vdev->opmode == wlan_op_mode_monitor)) {
            dev_kfree_skb_any(skb);
	    return;
    }

    if (wlan_is_wrap(vap)) {
        if(osif_ol_wrap_rx_process(&osdev, &netdev, vap, skb)) {
            return;
        }

#if DBDC_REPEATER_SUPPORT
        nwifi = ((osif_dev *)osif)->nss_nwifi;
        if (dbdc_rx_process(&osif, &netdev, vap, skb, &nwifi)) {
            return;
        }
#endif
    }

    if (vdev->rx_decap_type != htt_pkt_type_raw) {
        dev_hold(netdev);
        skb->dev = netdev;
        skb->protocol = eth_type_trans(skb, netdev);

        napi_gro_receive(napi, skb);

        dev_put(netdev);
    } else {
        osif_nss_vdev_handle_rawbuf(netdev, skb, napi);
    }
}

/*
 * osif_nss_vdev_cfg_callback()
 *  call back function for the vdev configuration handler
 */
static void osif_nss_vdev_cfg_callback(void *app_data, struct nss_cmn_msg *msg) {

    struct nss_wifi_vdev_vow_dbg_stats *vow_dbg_stats;
    int count;

    switch (msg->type) {
        case NSS_WIFI_VDEV_INTERFACE_CONFIGURE_MSG:
            if (msg->response != NSS_CMN_RESPONSE_ACK) {
                osif_nss_warn("VDEV configuration failed with error: %d\n", msg->error);
                osif_nss_vdev_cfgp.response = NSS_TX_FAILURE;
                complete(&osif_nss_vdev_cfgp.complete);
                return;
            }

            osif_nss_info("VDEV configuration success: %d\n", msg->error);
            osif_nss_vdev_cfgp.response = NSS_TX_SUCCESS;
            complete(&osif_nss_vdev_cfgp.complete);
            break;

        case NSS_WIFI_VDEV_VOW_DBG_STATS_REQ_MSG:
            if (msg->response != NSS_CMN_RESPONSE_ACK) {
                osif_nss_warn("VDEV VoW DBG stats failed with error: %d\n", msg->error);
                osif_nss_vdev_vowp.response = NSS_TX_FAILURE;
                complete(&osif_nss_vdev_vowp.complete);
                return;
            }

            vow_dbg_stats = &((struct nss_wifi_vdev_msg *)msg)->msg.vdev_vow_dbg_stats;
            osif_nss_vdev_vowp.rx_vow_dbg_counter = vow_dbg_stats->rx_vow_dbg_counters;
            for (count = 0; count < 8; count++) {
                osif_nss_vdev_vowp.tx_vow_dbg_counter[count] = vow_dbg_stats->tx_vow_dbg_counters[count];
            }

            osif_nss_vdev_vowp.response = NSS_TX_SUCCESS;
            complete(&osif_nss_vdev_vowp.complete);
            break;

    }
}

int32_t osif_nss_vdev_alloc(void *pdev_txrx_handle, struct ieee80211vap *vap)
{

    struct ol_txrx_pdev_t *pdev = (struct ol_txrx_pdev_t *)pdev_txrx_handle;
    enum nss_dynamic_interface_type di_type;
    int32_t if_num;
    void *nss_wifiol_ctx;

    if (!pdev) {
        return 0;
    }

    nss_wifiol_ctx = pdev->nss_wifiol_ctx;
    if (!nss_wifiol_ctx){
        return 0;
    }

    if (vap->iv_psta && !vap->iv_mpsta) {
        return NSS_PROXY_VAP_IF_NUMBER;
    }

    di_type = osif_nss_radio_get_type(pdev->nss_wifiol_id);
    if (di_type == NSS_DYNAMIC_INTERFACE_TYPE_NONE) {
        osif_nss_warn(" di returned invalid type : %d\n",di_type);
        return -1;
    }
    /*
     * Allocate a dynamic interface in NSS which represents the vdev
     */
    if_num = nss_dynamic_interface_alloc_node(di_type);
    if (if_num < 0) {
        osif_nss_warn(" di returned error : %d\n",if_num);
        return -1;
    }
    return if_num;
}

int32_t osif_nss_ol_vap_create(struct ol_txrx_vdev_t * vdev, struct ieee80211vap *vap, osif_dev *os_dev, uint32_t if_num) {

    void *nss_wifiol_ctx;
    struct net_device *dev;
    uint32_t features = 0;
    struct nss_wifi_vdev_msg wifivdevmsg;
    struct nss_wifi_vdev_config_msg *wifivdevcfg;
    enum nss_dynamic_interface_type di_type;
    bool status;
    uint32_t i;
    int32_t ret;

    if (!vdev) {
        return 0;
    }

    nss_wifiol_ctx = vdev->pdev->nss_wifiol_ctx;
    if (!nss_wifiol_ctx){
        return 0;
    }


    dev = OSIF_TO_NETDEV(os_dev);

    if (vap->iv_psta && !vap->iv_mpsta) {
        printk("Dont create NSS vap for proxy ");
        os_dev->nss_ifnum = if_num;
        return 0;
    }


    printk("NSS wifi offload VAP create IF %d vdev id %d nss_id %d \n",
            if_num,
            vdev->vdev_id,
            vdev->pdev->nss_wifiol_id);



    di_type = osif_nss_radio_get_type(vdev->pdev->nss_wifiol_id);
    if (di_type == NSS_DYNAMIC_INTERFACE_TYPE_NONE) {
        osif_nss_warn(" di returned invalid type : %d\n",di_type);
        return -1;
    }

    /*
     * Initialize queue
     */
    spin_lock_init(&os_dev->queue_lock);
    __skb_queue_head_init(&os_dev->wifiol_txqueue);

    OS_INIT_TIMER(vdev->pdev->osdev, &os_dev->wifiol_stale_pkts_timer,
       osif_nss_vdev_stale_pkts_timer, (void *)os_dev);

    os_dev->stale_pkts_timer_interval = OSIF_NSS_VDEV_STALE_PKT_INTERVAL;

    /*
     * Register the WIFI vdev with NSS
     */
    status = nss_register_wifi_vdev_if(nss_wifiol_ctx, if_num,
            osif_nss_vdev_data_receive,
            osif_nss_vdev_special_data_receive,
            osif_nss_vdev_event_receive,
            dev,
            features);

    if (status != NSS_TX_SUCCESS) {
        osif_nss_warn("Unable to register the vdev with nss\n");
        goto dealloc;
    }

    /*
     * Send the vdev configure message down to NSS
     */
    memset(&wifivdevmsg, 0, sizeof(struct nss_wifi_vdev_msg));
    wifivdevcfg = &wifivdevmsg.msg.vdev_config;
    wifivdevcfg->vdev_id = vdev->vdev_id;
    wifivdevcfg->epid = vdev->epid;
    wifivdevcfg->downloadlen = vdev->downloadlen;
    wifivdevcfg->hdrcachelen = HTC_HTT_TRANSFER_HDRSIZE;
    wifivdevcfg->opmode = vdev->opmode;
    memcpy(wifivdevcfg->hdrcache, vdev->hdrcache, HTC_HTT_TRANSFER_HDRSIZE);
    memcpy(wifivdevcfg->mac_addr,  &vdev->mac_addr.raw[0], 6);

#if MESH_MODE_SUPPORT
    wifivdevcfg->mesh_mode_en = vap->iv_mesh_vap_mode;
#endif

    wifivdevcfg->is_psta = vap->iv_psta;
    wifivdevcfg->is_mpsta = vap->iv_mpsta;
    wifivdevcfg->special_vap_mode = vap->iv_special_vap_mode;
    wifivdevcfg->smartmesh_mode_en = vap->iv_smart_monitor_vap;

    osif_nss_info("Sending on interface : %d Vdev config data: vdev_id : %d, epid : %d, downloadlen : %d\n",
            if_num, wifivdevcfg->vdev_id, wifivdevcfg->epid, wifivdevcfg->downloadlen);


    osif_nss_info("Sending hdr cache data of length :%d\n", wifivdevcfg->hdrcachelen);


    for ( i = 0; i < (wifivdevcfg->hdrcachelen) / 4; i++) {
        osif_nss_info("hdrcache[%d]:0x%x\n", i, wifivdevcfg->hdrcache[i]);
    }

    init_completion(&osif_nss_vdev_cfgp.complete);

    /*
     * Send vdev configure command to NSS
     */
    nss_wifi_vdev_msg_init(&wifivdevmsg, if_num, NSS_WIFI_VDEV_INTERFACE_CONFIGURE_MSG,
            sizeof(struct nss_wifi_vdev_config_msg), (nss_wifi_vdev_msg_callback_t *)osif_nss_vdev_cfg_callback, NULL);

    /*
     * Send the vdev configure message down to NSS
     */
    status = nss_wifi_vdev_tx_msg(nss_wifiol_ctx, &wifivdevmsg);
    if (status != NSS_TX_SUCCESS) {
        osif_nss_warn("Unable to send the vdev config message to NSS\n");
        goto unregister;
    }

    /*
     * Blocking call, wait till we get ACK for this msg.
     */
    ret = wait_for_completion_timeout(&osif_nss_vdev_cfgp.complete, msecs_to_jiffies(OSIF_NSS_VDEV_CFG_TIMEOUT_MS));

    if (ret == 0) {
        osif_nss_warn("Waiting for vdev config msg ack timed out\n");
        goto unregister;
    }

    /*
     * ACK/NACK received from NSS FW
     */
    if (NSS_TX_FAILURE == osif_nss_vdev_cfgp.response) {

        osif_nss_warn("nack for vdev config msg\n");
        goto unregister;
    }
    os_dev->nss_ifnum = if_num;
    printk("vap create %p : if_num %d \n", os_dev, os_dev->nss_ifnum );

    return if_num;

unregister:
    nss_unregister_wifi_vdev_if(if_num);

dealloc:
    (void)nss_dynamic_interface_dealloc_node(if_num, di_type);
    return -1;
}




/*
 * osif_nss_vdev_detach()
 *	API for deleting nss interface for wifi vdev.
 */
static void osif_nss_vdev_detach(struct nss_ctx_instance *nss_wifiol_ctx, struct osif_nss_vdevinfo *vdev_info, int32_t if_num)
{
    enum nss_dynamic_interface_type di_type;
    uint32_t radio_id;

    if (unlikely(if_num == NSS_PROXY_VAP_IF_NUMBER)) {
        printk("interface number :%d not valid for dynamic interface node \n", if_num);
        return;
    }

    radio_id = vdev_info->radio_id;
    di_type = osif_nss_radio_get_type(radio_id);

    printk("Dealloc Dynamic interface Node :%d of type:%d\n", if_num, di_type);

    (void)nss_dynamic_interface_dealloc_node(if_num, di_type);
    nss_unregister_wifi_vdev_if(if_num);
}

/*
 * osif_nss_vdev_xmit()
 *	API for sending the packet to radio.
 */
static int32_t osif_nss_vdev_xmit(void *nss_wifiol_ctx, int32_t if_num, struct sk_buff *skb)
{
    nss_tx_status_t status = NSS_TX_SUCCESS;
    uint32_t needed_headroom = HTC_HTT_TRANSFER_HDRSIZE;

    /*
     * Check that valid interface number is passed in
     */
    BUG_ON(if_num == -1);
    if (unlikely(if_num == NSS_PROXY_VAP_IF_NUMBER)) {
        return -1;
    }

    /*
     * Sanity check the SKB to ensure that it's suitable for us
     */
    if (unlikely(skb->len <= ETH_HLEN)) {
        osif_nss_warn("%p: Rx packet: %p too short", nss_wifiol_ctx, skb);
        return OSIF_NSS_FAILUE_MSG_TOO_SHORT;
    }

    /*
     * Check for minimum headroom availability
     */
    if ((skb_headroom(skb) < needed_headroom) && (skb->len > ETH_FRAME_LEN)) {
        if (pskb_expand_head(skb, needed_headroom, 0, GFP_ATOMIC)) {
            osif_nss_warn("%p: Insufficient headroom %d needed %d", nss_wifiol_ctx, skb_headroom(skb), needed_headroom);
            return -1;
        }
    }

    status = nss_wifi_vdev_tx_buf(nss_wifiol_ctx, skb, if_num);
    if (status != NSS_TX_SUCCESS) {
        return -1;
    }

    return 0;
}

/*
 * osif_nss_vdev_up()
 *	API for notifying the interface up status to NSS.
 */
static int32_t osif_nss_vdev_up(struct nss_ctx_instance *nss_wifiol_ctx, int32_t if_num, struct net_device *os_dev)
{
    struct nss_wifi_vdev_msg wifivdevmsg;
    struct nss_wifi_vdev_enable_msg *wifivdeven;
    bool status;

    /*
     * Send the vdev mac address to NSS
     */
    wifivdeven = &wifivdevmsg.msg.vdev_enable;
    memset(&wifivdevmsg, 0, sizeof(struct nss_wifi_vdev_msg));
    memcpy(wifivdeven->mac_addr, os_dev->dev_addr, 6);

    /*
     * Send vdev configure command to NSS
     */
    nss_wifi_vdev_msg_init(&wifivdevmsg, if_num, NSS_WIFI_VDEV_INTERFACE_UP_MSG,
            sizeof(struct nss_wifi_vdev_enable_msg), NULL, NULL);

    /*
     * Send the vdev configure message down to NSS
     */
    status = nss_wifi_vdev_tx_msg(nss_wifiol_ctx, &wifivdevmsg);
    if (status != NSS_TX_SUCCESS) {
        osif_nss_warn("Unable to send the vdev enable message to NSS\n");
        return -1;
    }
    return 0;
}

/*
 * osif_nss_vdev_down()
 *	API for notifying the interface down status to NSS.
 */
static int32_t osif_nss_vdev_down(struct nss_ctx_instance *nss_wifiol_ctx, int32_t if_num, struct net_device *os_dev)
{
    struct nss_wifi_vdev_msg wifivdevmsg;
    bool status;

    /*
     * Send the vdev mac address to NSS
     */
    memset(&wifivdevmsg, 0, sizeof(struct nss_wifi_vdev_msg));

    /*
     * Send vdev configure command to NSS
     */
    nss_wifi_vdev_msg_init(&wifivdevmsg, if_num, NSS_WIFI_VDEV_INTERFACE_DOWN_MSG,
            sizeof(struct nss_wifi_vdev_disable_msg), NULL, NULL);

    /*
     * Send the vdev configure message down to NSS
     */
    status = nss_wifi_vdev_tx_msg(nss_wifiol_ctx, &wifivdevmsg);
    if (status != NSS_TX_SUCCESS) {
        osif_nss_warn("Unable to send the vdev disable message to NSS\n");
        return -1;

    }

    return 0;
}

/*
 * osif_nss_vdev_send_cmd()
 *	API for notifying the vdev related command to NSS.
 */
static void osif_nss_vdev_send_cmd(struct nss_ctx_instance *nss_wifiol_ctx, int32_t if_num,
        enum nss_wifi_vdev_cmd cmd, uint32_t value)
{
    struct nss_wifi_vdev_msg wifivdevmsg;
    struct nss_wifi_vdev_cmd_msg *wifivdevcmd;

    bool status;

    wifivdevcmd = &wifivdevmsg.msg.vdev_cmd;
    memset(&wifivdevmsg, 0, sizeof(struct nss_wifi_vdev_msg));

    wifivdevcmd->cmd = cmd;
    wifivdevcmd->value = value;
    nss_wifi_vdev_msg_init(&wifivdevmsg, if_num, NSS_WIFI_VDEV_INTERFACE_CMD_MSG,
            sizeof(struct nss_wifi_vdev_cmd_msg), NULL, NULL);

    /*
     * Send the vdev configure message down to NSS
     */
    status = nss_wifi_vdev_tx_msg(nss_wifiol_ctx, &wifivdevmsg);
    if (status != NSS_TX_SUCCESS) {
        osif_nss_warn("Unable to send the vdev command message to NSS\n");
    }
}


/*
 * osif_nss_ol_vap_delete
 */
int32_t osif_nss_ol_vap_delete(osif_dev *osif)
{

    struct osif_nss_vdevinfo vdevcfg;
    struct net_device *dev = OSIF_TO_NETDEV(osif);
    int32_t nss_ifnum = osif->nss_ifnum;
    void *nss_wifiol_ctx = osif->nss_wifiol_ctx;

    if (!nss_wifiol_ctx) {
        return 0;
    }

    if (osif->nss_ifnum == -1) {
        printk(" vap delete called in invalid interface\n");
        return -1;
    }

    vdevcfg.radio_id = osif->radio_id;
    vdevcfg.os_dev = dev;


    printk("vap detach %p: if_num %d ",osif, osif->nss_ifnum);
    /*
     * Attach the vdev with NSS and store the associated interface number
     * returned by NSS in VAP private info
     */
    osif_nss_vdev_detach(nss_wifiol_ctx, &vdevcfg, nss_ifnum);

    OS_FREE_TIMER(&osif->wifiol_stale_pkts_timer);

    /*
     * Empty the queue
     */
    spin_lock_bh(&osif->queue_lock);
    osif->stale_pkts_timer_interval = 0;
    __skb_queue_purge(&osif->wifiol_txqueue);
    spin_unlock_bh(&osif->queue_lock);

    return 0;
}

/*
 * osif_nss_ol_vap_up
 */
int osif_nss_ol_vap_up(osif_dev *osif)
{
    struct net_device *dev = OSIF_TO_NETDEV(osif);
    struct ol_txrx_vdev_t *vdev;
    int32_t nss_ifnum;
    void *nss_wifiol_ctx;

    vdev = (struct ol_txrx_vdev_t *)osif->iv_txrx_handle;
    if (!vdev) {
        return 0;
    }

    nss_ifnum = osif->nss_ifnum;
    nss_wifiol_ctx = vdev->pdev->nss_wifiol_ctx;

    if (!nss_wifiol_ctx) {
        return 0;
    }


    if (osif->nss_ifnum == -1) {
        printk("vap up called on invalid interface\n");
        return -1;
    }
    /*
     * Bring up the VAP interface
     */
    if (osif_nss_vdev_up(nss_wifiol_ctx, nss_ifnum, dev) ){
        return -1;
    }
    return 0;

}

/*
 * osif_nss_ol_vap_down
 */
int  osif_nss_ol_vap_down(osif_dev *osif) {

    struct net_device *dev = OSIF_TO_NETDEV(osif);
    struct ol_txrx_vdev_t *vdev = (struct ol_txrx_vdev_t *)osif->iv_txrx_handle;
    int32_t nss_ifnum;
    void *nss_wifiol_ctx;

    if (!vdev) {
        return 0;
    }

    nss_wifiol_ctx = vdev->pdev->nss_wifiol_ctx;
    if (!nss_wifiol_ctx) {
        return 0;
    }

    nss_ifnum = osif->nss_ifnum;
    if (nss_ifnum == -1) {
        printk("vap down called on invalid interface\n");
        return -1;
    }

    /*
     * Bring down the VAP interface
     */
    osif_nss_vdev_down(nss_wifiol_ctx, nss_ifnum, dev);
    return 0;
}

/*
 * osif_nss_ol_vap_xmit
 */
int32_t osif_nss_ol_vap_xmit(osif_dev *osif, struct sk_buff *skb)
{
    struct ol_txrx_vdev_t *vdev  = (struct ol_txrx_vdev_t *)osif->iv_txrx_handle;
    int32_t nss_ifnum;
    void *nss_wifiol_ctx;

    if (!vdev) {
        return -1;
    }

    nss_wifiol_ctx = vdev->pdev->nss_wifiol_ctx;
    if (!nss_wifiol_ctx) {
        return -1;
    }

    nss_ifnum = osif->nss_ifnum;
    if (nss_ifnum == -1) {
        printk(" vap transmit called on invalid interface %p \n",osif);
        return -1;
    }

    return osif_nss_vdev_xmit(nss_wifiol_ctx, nss_ifnum, skb);
}

/*
 * osif_nss_ol_vdev_set_cfg
 */
void osif_nss_ol_vdev_set_cfg(struct ol_txrx_vdev_t * vdev, enum osif_nss_vdev_cmd osif_cmd)
{
    struct net_device *dev;
    int32_t nss_ifnum;
    uint32_t val;
    enum nss_wifi_vdev_cmd cmd = 0;
    void *nss_wifiol_ctx;
    struct ieee80211vap *vap = NULL;
    osif_dev *osif;

    if (!vdev) {
        return;
    }

    nss_wifiol_ctx = vdev->pdev->nss_wifiol_ctx;
    osif = (osif_dev *)vdev->osif_vdev;

    if (!nss_wifiol_ctx) {
        return;
    }

    if (osif == NULL) {
        printk("Invalid os dev passed\n");
        return;
    }

    dev = OSIF_TO_NETDEV(osif);
    nss_ifnum = osif->nss_ifnum;

    if (osif->nss_ifnum == -1) {
        printk(" vap transmit called on invalid interface\n");
        return;
    }

    switch (osif_cmd) {
        case OSIF_NSS_VDEV_DROP_UNENC:
            cmd = NSS_WIFI_VDEV_DROP_UNENC_CMD;
            val = vdev->drop_unenc;
            break;

        case OSIF_NSS_VDEV_ENCAP_TYPE:
            cmd = NSS_WIFI_VDEV_ENCAP_TYPE_CMD;
            val = vdev->tx_encap_type;
            break;

        case OSIF_NSS_VDEV_DECAP_TYPE:
            cmd = NSS_WIFI_VDEV_DECAP_TYPE_CMD;
            val = vdev->rx_decap_type;
            break;

        case OSIF_NSS_VDEV_ENABLE_ME:
            cmd = NSS_WIFI_VDEV_ENABLE_ME_CMD;
            vap = osif->os_if;
            if (vap->iv_me->me_hifi_enable) {
                printk("setting hifi mode 5\n");
                val = MC_HYFI_ENABLE;
            } else {
                val = osif->os_if->iv_me->mc_mcast_enable;
            }
            break;

        case OSIF_NSS_WIFI_VDEV_NAWDS_MODE:
            cmd = NSS_WIFI_VDEV_NAWDS_MODE_CMD;
            val = vdev->nawds_enabled;
            break;

        case OSIF_NSS_VDEV_EXTAP_CONFIG:
            cmd = NSS_WIFI_VDEV_EXTAP_CONFIG_CMD;
            vap = osif->os_if;
            val = (IEEE80211_VAP_IS_EXT_AP_ENABLED(vap) ? 1 : 0);
            break;

        case OSIF_NSS_WIFI_VDEV_CFG_BSTEER:
            vap = osif->os_if;
            cmd = NSS_WIFI_VDEV_CFG_BSTEER_CMD;
            val = atomic_read(&vap->iv_bs_enabled) ? 1 : 0;
            break;

        case OSIF_NSS_WIFI_VDEV_VOW_DBG_MODE:
            cmd = NSS_WIFI_VDEV_VOW_DBG_MODE_CMD;
            val = osif->vow_dbg_en;
            break;

        case OSIF_NSS_WIFI_VDEV_VOW_DBG_RST_STATS:
            cmd = NSS_WIFI_VDEV_VOW_DBG_RST_STATS_CMD;
            val = 0;
            break;

        case OSIF_NSS_WIFI_VDEV_FILTER_NEIGH_PEERS:
            cmd = NSS_WIFI_VDEV_FILTER_NEIGH_PEERS_CMD;
            val = vdev->pdev->filter_neighbour_peers;
            break;

        case OSIF_NSS_WIFI_VDEV_CFG_DSCP_OVERRIDE:
            cmd = NSS_WIFI_VDEV_CFG_DSCP_OVERRIDE_CMD;
            vap = osif->os_if;
            val = vap->iv_override_dscp;
            break;

        case OSIF_NSS_WIFI_VDEV_CFG_WNM_CAP:
            cmd = NSS_WIFI_VDEV_CFG_WNM_CAP_CMD;
            vap = osif->os_if;
            val = vap->iv_wnm;
            break;

        case OSIF_NSS_WIFI_VDEV_CFG_WNM_TFS:
            cmd = NSS_WIFI_VDEV_CFG_WNM_TFS_CMD;
            vap = osif->os_if;
            val = vap->iv_wnm;
            break;

        default:
            printk("Command :%d is not supported in NSS\n", cmd);
            return;
    }

    osif_nss_vdev_send_cmd(nss_wifiol_ctx, nss_ifnum, cmd, val);
}

/*
 * osif_nss_vdev_get_nss_id
 *  Get the nss id of the vdev
 */
int osif_nss_vdev_get_nss_id( void *vdevctx)
{
    struct ol_txrx_vdev_t *vdev = (struct ol_txrx_vdev_t *)vdevctx;
    if(!vdev) {
        return -1;
    }
    return (vdev->pdev->nss_wifiol_id) ;
}

/*
 * osif_vdev_get_nss_wifiol_ctx
 *  Get the nss id of the vdev
 */
void *osif_nss_vdev_get_nss_wifiol_ctx( void *vdevctx)
{
    struct ol_txrx_vdev_t *vdev = (struct ol_txrx_vdev_t *)vdevctx;

    if(!vdev) {
        return NULL;
    }

    return (vdev->pdev->nss_wifiol_ctx) ;
}

/*
 * osif_nss_vdev_me_create_grp_list()
 * 	API to notify grp_list creation in snooplist in NSS
 */
void osif_nss_vdev_me_create_grp_list(struct ol_txrx_vdev_t *vdev, uint8_t *grp_addr, uint8_t *grp_ipaddr, uint32_t ether_type, uint32_t length)
{
    struct nss_ctx_instance *nss_ctx;
    osif_dev *osifp;
    int32_t if_num;
    struct nss_wifi_vdev_msg wifivdevmsg;
    struct nss_wifi_vdev_snooplist_grp_list_create_msg *wifivdevsglc;
    bool status;
    int8_t *dest = NULL;

    if (!vdev) {
        return;
    }

    nss_ctx = osif_nss_vdev_get_nss_wifiol_ctx(vdev);

    if (!nss_ctx) {
        return;
    }

    osifp = (osif_dev *)vdev->osif_vdev;
    if_num = osifp->nss_ifnum;

    wifivdevsglc = &wifivdevmsg.msg.vdev_grp_list_create;

    if (!grp_addr) {
        OS_MEMSET(wifivdevsglc->grp_addr, 0, IEEE80211_ADDR_LEN);
    } else {
        IEEE80211_ADDR_COPY(wifivdevsglc->grp_addr, grp_addr);
    }

    dest = (int8_t *)(&wifivdevsglc->u);
    OS_MEMCPY(dest, grp_ipaddr, length);

    wifivdevsglc->ether_type = ether_type;

    nss_wifi_vdev_msg_init(&wifivdevmsg, if_num, NSS_WIFI_VDEV_SNOOPLIST_GRP_LIST_CREATE_MSG,
                       sizeof(struct nss_wifi_vdev_snooplist_grp_list_create_msg), NULL, NULL);
    /*
     * Send the vdev snooplist message down to NSS
     */
    status = nss_wifi_vdev_tx_msg(nss_ctx, &wifivdevmsg);
    if (status != NSS_TX_SUCCESS) {
        printk("Unable to send the grp_list create message to NSS\n");
    }
}
EXPORT_SYMBOL(osif_nss_vdev_me_create_grp_list);


/*
 * osif_nss_vdev_me_delete_grp_list()
 * 	API to notify grp_list deletion in snooplist in NSS
 */
void osif_nss_vdev_me_delete_grp_list(struct ol_txrx_vdev_t *vdev, uint8_t *grp_addr, uint8_t *grp_ipaddr, uint32_t ether_type)
{
    struct nss_ctx_instance *nss_ctx;
    osif_dev *osifp;
    int32_t if_num;
    struct nss_wifi_vdev_msg wifivdevmsg;
    struct nss_wifi_vdev_snooplist_grp_list_delete_msg *wifivdevsgld;
    uint32_t length;
    int8_t *dest = NULL;
    bool status;

    if (!vdev) {
        return;
    }

    nss_ctx = osif_nss_vdev_get_nss_wifiol_ctx(vdev);

    if (!nss_ctx) {
        return;
    }

    osifp = (osif_dev *)vdev->osif_vdev;
    if_num = osifp->nss_ifnum;

    wifivdevsgld = &wifivdevmsg.msg.vdev_grp_list_delete;

    IEEE80211_ADDR_COPY(wifivdevsgld->grp_addr, grp_addr);

    if (ether_type == ETHERTYPE_IPV4) {
        length = IGMP_IP_ADDR_LENGTH;
    } else if (ether_type == ETHERTYPE_IPV6) {
        length = MLD_IP_ADDR_LENGTH;
    } else {
        printk("%s wrong ethertye %d",__FUNCTION__, ether_type);
        return;
    }

    dest = (int8_t *)(&wifivdevsgld->u);
    OS_MEMCPY(dest, grp_ipaddr, length);
    wifivdevsgld->ether_type = ether_type;

    nss_wifi_vdev_msg_init(&wifivdevmsg, if_num, NSS_WIFI_VDEV_SNOOPLIST_GRP_LIST_DELETE_MSG,
                       sizeof(struct nss_wifi_vdev_snooplist_grp_list_delete_msg), NULL, NULL);

    /*
     * Send the vdev snooplist message down to NSS
     */
    status = nss_wifi_vdev_tx_msg(nss_ctx, &wifivdevmsg);
    if (status != NSS_TX_SUCCESS) {
        osif_nss_warn("Unable to send the grp_list delete message to NSS\n");
    }
}
EXPORT_SYMBOL(osif_nss_vdev_me_delete_grp_list);

/*
 * osif_nss_vdev_me_add_member_list()
 * 	API to notify grp_member addition to grp_list in NSS.
 */
void osif_nss_vdev_me_add_member_list(struct ol_txrx_vdev_t *vdev, struct MC_LIST_UPDATE* list_entry, uint32_t ether_type, uint32_t length, uint32_t peer_id)
{
    struct nss_ctx_instance *nss_ctx;
    osif_dev *osifp;
    int32_t if_num;
    struct nss_wifi_vdev_msg wifivdevmsg;
    struct nss_wifi_vdev_snooplist_grp_member_add_msg *wifivdevsgma;
    struct ol_txrx_peer_t *peer = NULL;
    int8_t *dest = NULL;
    int8_t *src = NULL;
    bool status;

    if (!vdev) {
        return;
    }

    nss_ctx = osif_nss_vdev_get_nss_wifiol_ctx(vdev);

    if (!nss_ctx) {
        return;
    }
    osifp = (osif_dev *)vdev->osif_vdev;
    if_num = osifp->nss_ifnum;

    if (peer_id == HTT_INVALID_PEER) {
        if (list_entry->ni == NULL) {
            printk("listentry ni is NULL and peer is is invalid, hence returning\n");
            return;
        }

        peer = (OL_ATH_NODE_NET80211(list_entry->ni))->an_txrx_handle;
        peer_id = peer->peer_ids[0];
    }

    wifivdevsgma = &wifivdevmsg.msg.vdev_grp_member_add;
    wifivdevsgma->src_ip_addr = list_entry->src_ip_addr;

    dest = (int8_t *)(&wifivdevsgma->u);
    src = (int8_t *)(&list_entry->u);
    OS_MEMCPY(dest, src, length);

    wifivdevsgma->peer_id = peer_id;
    wifivdevsgma->mode = list_entry->cmd;
    wifivdevsgma->ether_type = ether_type;
    IEEE80211_ADDR_COPY(wifivdevsgma->grp_addr, list_entry->grp_addr);
    IEEE80211_ADDR_COPY(wifivdevsgma->grp_member_addr, list_entry->grp_member);

    nss_wifi_vdev_msg_init(&wifivdevmsg, if_num, NSS_WIFI_VDEV_SNOOPLIST_GRP_MEMBER_ADD_MSG,
                       sizeof(struct nss_wifi_vdev_snooplist_grp_member_add_msg), NULL, NULL);

    /*
     * Send the vdev snooplist message down to NSS
     */
    status = nss_wifi_vdev_tx_msg(nss_ctx, &wifivdevmsg);
    if (status != NSS_TX_SUCCESS) {
        osif_nss_warn("Unable to send the grp_list add member message to NSS\n");
    }
}
EXPORT_SYMBOL(osif_nss_vdev_me_add_member_list);

/*
 * osif_nss_vdev_me_remove_member_list()
 * 	API_to notify grp_member got removed from grp_list in NSS.
 */
void osif_nss_vdev_me_remove_member_list(struct ol_txrx_vdev_t *vdev, uint8_t *grp_ipaddr, uint8_t *grp_addr, uint8_t *grp_member_addr, uint32_t ether_type)
{
    struct nss_ctx_instance *nss_ctx;
    osif_dev *osifp;
    int32_t if_num;
    struct nss_wifi_vdev_msg wifivdevmsg;
    struct nss_wifi_vdev_snooplist_grp_member_remove_msg *wifivdevsgmr;
    uint32_t length;
    int8_t *dest = NULL;
    bool status;

    if (!vdev) {
        return;
    }

    nss_ctx = osif_nss_vdev_get_nss_wifiol_ctx(vdev);

    if (!nss_ctx) {
        return;
    }
    osifp = (osif_dev *)vdev->osif_vdev;
    if_num = osifp->nss_ifnum;

    if (ether_type == ETHERTYPE_IPV4) {
        length = IGMP_IP_ADDR_LENGTH;
    } else if (ether_type == ETHERTYPE_IPV6) {
        length = MLD_IP_ADDR_LENGTH;
    } else {
        printk("%s wrong ethertye %d",__FUNCTION__, ether_type);
        return;
    }

    wifivdevsgmr = &wifivdevmsg.msg.vdev_grp_member_remove;

    dest = (int8_t *)(&wifivdevsgmr->u);
    OS_MEMCPY(dest, grp_ipaddr, length);

    wifivdevsgmr->ether_type = ether_type;
    IEEE80211_ADDR_COPY(wifivdevsgmr->grp_addr, grp_addr);
    IEEE80211_ADDR_COPY(wifivdevsgmr->grp_member_addr, grp_member_addr);

    nss_wifi_vdev_msg_init(&wifivdevmsg, if_num, NSS_WIFI_VDEV_SNOOPLIST_GRP_MEMBER_REMOVE_MSG,
                        sizeof(struct nss_wifi_vdev_snooplist_grp_member_remove_msg), NULL, NULL);

    /*
     * Send the vdev snooplist message down to NSS
     */
    status = nss_wifi_vdev_tx_msg(nss_ctx, &wifivdevmsg);
    if (status != NSS_TX_SUCCESS) {
        osif_nss_warn("Unable to send the grp_list remove member message to NSS\n");
    }
}
EXPORT_SYMBOL(osif_nss_vdev_me_remove_member_list);

/*
 * osif_nss_vdev_me_update_member_list()
 * 	API to notify updation in grp_member to grp_list in NSS.
 */
void osif_nss_vdev_me_update_member_list(struct ol_txrx_vdev_t *vdev, struct MC_LIST_UPDATE* list_entry, uint32_t ether_type)
{
    struct nss_ctx_instance *nss_ctx;
    osif_dev *osifp;
    int32_t if_num;
    struct nss_wifi_vdev_msg wifivdevmsg;
    struct nss_wifi_vdev_snooplist_grp_member_update_msg *wifivdevsgmu;
    uint32_t length;
    int8_t *dest = NULL;
    int8_t *src = NULL;
    bool status;

    if (!vdev) {
        return;
    }

    nss_ctx = osif_nss_vdev_get_nss_wifiol_ctx(vdev);

    if (!nss_ctx) {
        return;
    }

    osifp = (osif_dev *)vdev->osif_vdev;
    if_num = osifp->nss_ifnum;

    if (ether_type == ETHERTYPE_IPV4) {
        length = IGMP_IP_ADDR_LENGTH;
    } else if (ether_type == ETHERTYPE_IPV6) {
        length = MLD_IP_ADDR_LENGTH;
    } else {
        printk("%s wrong ethertye %d",__FUNCTION__, ether_type);
        return;
    }

    wifivdevsgmu = &wifivdevmsg.msg.vdev_grp_member_update;

    dest = (int8_t *)(&wifivdevsgmu->u);
    src = (int8_t *)(&list_entry->u);
    OS_MEMCPY(dest, src, length);

    wifivdevsgmu->ether_type = ether_type;
    wifivdevsgmu->src_ip_addr = list_entry->src_ip_addr;
    wifivdevsgmu->mode = list_entry->cmd;
    IEEE80211_ADDR_COPY(wifivdevsgmu->grp_addr, list_entry->grp_addr);
    IEEE80211_ADDR_COPY(wifivdevsgmu->grp_member_addr, list_entry->grp_member);

    nss_wifi_vdev_msg_init(&wifivdevmsg, if_num, NSS_WIFI_VDEV_SNOOPLIST_GRP_MEMBER_UPDATE_MSG,
                        sizeof(struct nss_wifi_vdev_snooplist_grp_member_update_msg), NULL, NULL);

    /*
     * Send the vdev snooplist message down to NSS
     */
    status = nss_wifi_vdev_tx_msg(nss_ctx, &wifivdevmsg);
    if (status != NSS_TX_SUCCESS) {
        osif_nss_warn("Unable to send the grp_list update member message to NSS\n");
    }
}
EXPORT_SYMBOL(osif_nss_vdev_me_update_member_list);

/*
 * osif_nss_vdev_me_add_deny_member()
 * 	API to notify addition of member in Deny list in NSS.
 */
void osif_nss_vdev_me_add_deny_member(struct ol_txrx_vdev_t *vdev, uint32_t grp_ipaddr)
{
    struct nss_ctx_instance *nss_ctx;
    osif_dev *osifp;
    struct nss_wifi_vdev_msg wifivdevmsg;
    int32_t if_num;
    struct nss_wifi_vdev_snooplist_deny_member_add_msg *wifivdevsdma;
    bool status;

    if (!vdev) {
        return;
    }

    nss_ctx = osif_nss_vdev_get_nss_wifiol_ctx(vdev);

    if (!nss_ctx) {
        return;
    }

    osifp = (osif_dev *)vdev->osif_vdev;
    if_num = osifp->nss_ifnum;


    wifivdevsdma = &wifivdevmsg.msg.vdev_deny_member_add;

    wifivdevsdma->grpaddr = grp_ipaddr;

    nss_wifi_vdev_msg_init(&wifivdevmsg, if_num, NSS_WIFI_VDEV_SNOOPLIST_DENY_MEMBER_ADD_MSG,
                        sizeof(struct nss_wifi_vdev_snooplist_deny_member_add_msg), NULL, NULL);

    /*
     * Send the vdev snooplist message down to NSS
     */
    status = nss_wifi_vdev_tx_msg(nss_ctx, &wifivdevmsg);
    if (status != NSS_TX_SUCCESS) {
        osif_nss_warn("Unable to send the grp_list deny member add message to NSS\n");
    }
}
EXPORT_SYMBOL(osif_nss_vdev_me_add_deny_member);

/*
 * osif_nss_vdev_me_delete_deny_list()
 * 	API to notify Deletion of Deny table from Snooplist in NSS.
 *
 */
void osif_nss_vdev_me_delete_deny_list(struct ol_txrx_vdev_t *vdev)
{
    struct nss_ctx_instance *nss_ctx;
    osif_dev *osifp;
    int32_t if_num;
    struct nss_wifi_vdev_msg wifivdevmsg;
    bool status;

    if (!vdev) {
        return;
    }

    nss_ctx = osif_nss_vdev_get_nss_wifiol_ctx(vdev);

    if (!nss_ctx) {
        return;
    }

    osifp = (osif_dev *)vdev->osif_vdev;
    if_num = osifp->nss_ifnum;

    nss_wifi_vdev_msg_init(&wifivdevmsg, if_num, NSS_WIFI_VDEV_SNOOPLIST_DENY_LIST_DELETE_MSG, 0, NULL, NULL);

    /*
     * Send the vdev snooplist message down to NSS
     */
    status = nss_wifi_vdev_tx_msg(nss_ctx, &wifivdevmsg);
    if (status != NSS_TX_SUCCESS) {
        osif_nss_warn("Unable to send the grp_list deny table message to NSS\n");
    }
}
EXPORT_SYMBOL(osif_nss_vdev_me_delete_deny_list);

/*
 * osif_nss_vdev_me_dump_snooplist()
 * 	API to notify dump snooplist in NSS.
 */
void osif_nss_vdev_me_dump_snooplist(struct ol_txrx_vdev_t *vdev)
{
    struct nss_ctx_instance *nss_ctx;
    osif_dev *osifp;
    int32_t if_num;
    struct nss_wifi_vdev_msg wifivdevmsg;
    bool status;

    if (!vdev) {
        return;
    }

    nss_ctx = osif_nss_vdev_get_nss_wifiol_ctx(vdev);

    if (!nss_ctx) {
        return;
    }

    nss_ctx = osif_nss_vdev_get_nss_wifiol_ctx(vdev);
    osifp = (osif_dev *)vdev->osif_vdev;
    if_num = osifp->nss_ifnum;


    nss_wifi_vdev_msg_init(&wifivdevmsg, if_num, NSS_WIFI_VDEV_SNOOPLIST_DUMP_MSG, 0, NULL, NULL);

    /*
     * Send the vdev snooplist message down to NSS
     */
    status = nss_wifi_vdev_tx_msg(nss_ctx, &wifivdevmsg);
    if (status != NSS_TX_SUCCESS) {
        osif_nss_warn("Unable to send the grp_list dump message to NSS\n");
    }
}
EXPORT_SYMBOL(osif_nss_vdev_me_dump_snooplist);

/*
 * osif_nss_vdev_me_dump_snooplist()
 * 	API to reset snooplist in NSS.
 */
void osif_nss_vdev_me_reset_snooplist(struct ol_txrx_vdev_t *vdev)
{
    struct nss_ctx_instance *nss_ctx;
    osif_dev *osifp;
    int32_t if_num;
    struct nss_wifi_vdev_msg wifivdevmsg;
    bool status;
    struct ieee80211vap *vap = NULL;

    if (!vdev) {
        return;
    }

    nss_ctx = osif_nss_vdev_get_nss_wifiol_ctx(vdev);

    if (!nss_ctx) {
        return;
    }

    nss_ctx = osif_nss_vdev_get_nss_wifiol_ctx(vdev);
    osifp = (osif_dev *)vdev->osif_vdev;
    if_num = osifp->nss_ifnum;

    vap = osifp->os_if;
    if (!vap->iv_me) {
        return;
    }

    if ( !vap->iv_me->me_hifi_enable) {
        return;
    }

    nss_wifi_vdev_msg_init(&wifivdevmsg, if_num, NSS_WIFI_VDEV_SNOOPLIST_RESET_MSG, 0, NULL, NULL);

    /*
     * Send the vdev snooplist message down to NSS
     */
    status = nss_wifi_vdev_tx_msg(nss_ctx, &wifivdevmsg);
    if (status != NSS_TX_SUCCESS) {
        osif_nss_warn("Unable to send the grp_list reset message to NSS\n");
    }
}
EXPORT_SYMBOL(osif_nss_vdev_me_reset_snooplist);

/*
 * osif_nss_vdev_me_toggle_snooplist()
 * 	API to toggle snooplist table in NSS.
 */
void osif_nss_vdev_me_toggle_snooplist(struct ol_txrx_vdev_t *vdev)
{
    struct nss_ctx_instance *nss_ctx;
    osif_dev *osifp;
    int32_t if_num;
    struct nss_wifi_vdev_msg wifivdevmsg;
    bool status;
    struct ieee80211vap *vap = NULL;

    if (!vdev) {
        return;
    }

    nss_ctx = osif_nss_vdev_get_nss_wifiol_ctx(vdev);

    if (!nss_ctx) {
        return;
    }

    nss_ctx = osif_nss_vdev_get_nss_wifiol_ctx(vdev);
    osifp = (osif_dev *)vdev->osif_vdev;
    if_num = osifp->nss_ifnum;

    vap = osifp->os_if;
    if (!vap->iv_me) {
        return;
    }

    if ( !vap->iv_me->me_hifi_enable) {
        return;
    }

    nss_wifi_vdev_msg_init(&wifivdevmsg, if_num, NSS_WIFI_VDEV_SNOOPLIST_TOGGLE_MSG, 0, NULL, NULL);

    /*
     * Send the vdev snooplist message down to NSS
     */
    status = nss_wifi_vdev_tx_msg(nss_ctx, &wifivdevmsg);
    if (status != NSS_TX_SUCCESS) {
        printk("Unable to send toggle message to NSS\n");
    }
}
EXPORT_SYMBOL(osif_nss_vdev_me_toggle_snooplist);


/*
 * osif_nss_vdev_me_update_hifitbl()
 * 	API to update snooplist from hifi table.
 */
void osif_nss_vdev_me_update_hifitlb(struct ol_txrx_vdev_t *vdev, struct ieee80211_me_hifi_table *table)
{
    void *nss_wifiol_ctx = NULL;
    struct ieee80211_me_hifi_entry *entry = NULL;
    struct ieee80211_me_hifi_group *group = NULL;
    struct MC_LIST_UPDATE member;
    int cnt, n, j;
    struct ieee80211vap *vap = NULL;
    struct ieee80211_node *ni = NULL;
    struct ol_txrx_pdev_t *pdev = NULL;
    struct ol_txrx_ast_entry_t *ast_entry = NULL;
    osif_dev *osif = NULL;
    u_int32_t peer_id = HTT_INVALID_PEER;
    u_int32_t *srcs;
    u_int32_t length = 0;
    int8_t *dest = NULL;
    int8_t *src = NULL;
    u_int32_t ether_type;
    union {
        u_int32_t ip4;
        u_int8_t ip6[MLD_IP_ADDR_LENGTH];
    }u;

    if (!vdev) {
        return;
    }

    nss_wifiol_ctx = vdev->pdev->nss_wifiol_ctx;
    if (!nss_wifiol_ctx) {
        return;
    }
    osif = (osif_dev *)vdev->osif_vdev;
    pdev = vdev->pdev;

    vap = osif->os_if;

    if (!vap->iv_me) {
        return;
    }

    if (!vap->iv_me->me_hifi_enable) {
        return;
    }

    for (cnt = 0; cnt < table->entry_cnt; cnt++) {
        entry = &table->entry[cnt];
        group = &entry->group;

        ether_type = ntohs(group->pro);
        if (ether_type == ETHERTYPE_IPV4) {
            length = IGMP_IP_ADDR_LENGTH;
            u.ip4 = ntohl(group->u.ip4);
        } else if (ether_type == ETHERTYPE_IPV6) {
            length = MLD_IP_ADDR_LENGTH;
            OS_MEMCPY(u.ip6, group->u.ip6, length);
        } else {
            printk("protocol %d not supported for mcast enhancement in NSS\n", group->pro);
            continue;
        }
        osif_nss_vdev_me_create_grp_list(vdev, NULL, (uint8_t *)&u, ether_type, length);

        for (n = 0; n < entry->node_cnt; n++) {
            ni = ieee80211_vap_find_node(vap, entry->nodes[n].mac);
            if (!ni) {
                ast_entry = ol_txrx_ast_find_hash_find(pdev, entry->nodes[n].mac, 1);
                if (!(ast_entry)) {
                    printk("No ast entry found\n");
                    continue;
                }

                peer_id = ast_entry->peer_id;
                if (peer_id == HTT_INVALID_PEER) {
                    printk("No peer found\n");
                    continue;
                }
            }

            if (!entry->nodes[n].nsrcs) {
                memset(&member, 0, sizeof(struct MC_LIST_UPDATE));
                member.src_ip_addr = 0;
                dest = (int8_t *)(&member.u);
                src = (int8_t *)(&u);
                OS_MEMCPY(dest, src, length);

                member.ni = ni;
                member.cmd = entry->nodes[n].filter_mode;

                OS_MEMSET(member.grp_addr, 0, IEEE80211_ADDR_LEN);
                IEEE80211_ADDR_COPY(member.grp_member, entry->nodes[n].mac);

                osif_nss_vdev_me_add_member_list(vdev, &member, ether_type, length, peer_id);
                goto free_node;
            }

            srcs = (u_int32_t *)entry->nodes[n].srcs;
            for (j = 0; j < entry->nodes[n].nsrcs; j++) {
                memset(&member, 0, sizeof(struct MC_LIST_UPDATE));
                member.src_ip_addr = ntohl(srcs[j]);
                dest = (int8_t *)(&member.u);
                src = (int8_t *)(&u);
                OS_MEMCPY(dest, src, length);
                member.ni = ni;
                member.cmd = entry->nodes[n].filter_mode;
                OS_MEMSET(member.grp_addr, 0, IEEE80211_ADDR_LEN);
                IEEE80211_ADDR_COPY(member.grp_member, entry->nodes[n].mac);

                osif_nss_vdev_me_add_member_list(vdev, &member, ether_type, length, peer_id);
            }

free_node:
            /* remove extra node ref count added by find_node above */
            ieee80211_free_node(ni);
        }
    }
    osif_nss_vdev_me_toggle_snooplist(vdev);
}
EXPORT_SYMBOL(osif_nss_vdev_me_update_hifitlb);

/*
 * osif_nss_vdev_me_dump_denylist()
 * 	API to notify dump denylist in NSS.
 */
void osif_nss_vdev_me_dump_denylist(struct ol_txrx_vdev_t *vdev)
{
    struct nss_ctx_instance *nss_ctx;
    osif_dev *osifp;
    int32_t if_num;
    struct nss_wifi_vdev_msg wifivdevmsg;
    bool status;

    if (!vdev) {
        return;
    }

    nss_ctx = osif_nss_vdev_get_nss_wifiol_ctx(vdev);
    if (!nss_ctx) {
        return;
    }

    osifp = (osif_dev *)vdev->osif_vdev;
    if_num = osifp->nss_ifnum;

    nss_wifi_vdev_msg_init(&wifivdevmsg, if_num, NSS_WIFI_VDEV_SNOOPLIST_DENY_LIST_DUMP_MSG, 0, NULL, NULL);

    /*
     * Send the vdev snooplist message down to NSS
     */
    status = nss_wifi_vdev_tx_msg(nss_ctx, &wifivdevmsg);
    if (status != NSS_TX_SUCCESS) {
        osif_nss_warn("Unable to send the grp_list deny dump table message to NSS\n");
    }
}
EXPORT_SYMBOL(osif_nss_vdev_me_dump_denylist);

/*
 * osif_nss_vdev_vow_dbg_cfg()
 * 	API to configure the peer MAC for which VoW debug stats will be incremented in NSS.
 */
void osif_nss_vdev_vow_dbg_cfg(struct ol_txrx_vdev_t *vdev, int32_t vow_peer_list_idx)
{
    struct nss_ctx_instance *nss_ctx;
    osif_dev *osifp;
    int32_t if_num;
    struct nss_wifi_vdev_msg wifivdevmsg;
    struct nss_wifi_vdev_vow_dbg_cfg_msg *wifivdevvdbgcfg;
    struct ol_txrx_pdev_t *pdev;
    bool status;

    if (!vdev) {
        return;
    }

    nss_ctx = osif_nss_vdev_get_nss_wifiol_ctx(vdev);
    if (!nss_ctx) {
        return;
    }

    osifp = (osif_dev *)vdev->osif_vdev;
    if_num = osifp->nss_ifnum;
    pdev = vdev->pdev;

    wifivdevvdbgcfg = &wifivdevmsg.msg.vdev_vow_dbg_cfg;

    wifivdevvdbgcfg->vow_peer_list_idx = vow_peer_list_idx;
    wifivdevvdbgcfg->tx_dbg_vow_peer_mac4 = osifp->tx_dbg_vow_peer[vow_peer_list_idx][0];
    wifivdevvdbgcfg->tx_dbg_vow_peer_mac5 = osifp->tx_dbg_vow_peer[vow_peer_list_idx][1];

    nss_wifi_vdev_msg_init(&wifivdevmsg, if_num, NSS_WIFI_VDEV_VOW_DBG_CFG_MSG, sizeof(struct nss_wifi_vdev_vow_dbg_cfg_msg), NULL, NULL);

    /*
     * Send the vdev snooplist message down to NSS
     */
    status = nss_wifi_vdev_tx_msg(nss_ctx, &wifivdevmsg);
    if (status != NSS_TX_SUCCESS) {
        osif_nss_warn("Unable to send the vow debug configuration to NSS\n");
    }
}
EXPORT_SYMBOL(osif_nss_vdev_vow_dbg_cfg);

/*
 * osif_nss_vdev_vow_dbg_cfg()
 * 	API to configure the peer MAC for which VoW debug stats will be incremented in NSS.
 */
void osif_nss_vdev_get_vow_dbg_stats(struct ol_txrx_vdev_t *vdev)
{
    struct nss_ctx_instance *nss_ctx;
    osif_dev *osifp;
    int32_t if_num;
    struct nss_wifi_vdev_msg wifivdevmsg;
    struct ol_txrx_pdev_t *pdev;
    int ret, i;
    bool status;

    if (!vdev) {
        return;
    }

    nss_ctx = osif_nss_vdev_get_nss_wifiol_ctx(vdev);
    if (!nss_ctx) {
        return;
    }

    osifp = (osif_dev *)vdev->osif_vdev;
    if_num = osifp->nss_ifnum;
    pdev = vdev->pdev;

    init_completion(&osif_nss_vdev_vowp.complete);
    nss_wifi_vdev_msg_init(&wifivdevmsg, if_num, NSS_WIFI_VDEV_VOW_DBG_STATS_REQ_MSG,
            sizeof(struct nss_wifi_vdev_vow_dbg_stats), (nss_wifi_vdev_msg_callback_t *)osif_nss_vdev_cfg_callback, NULL);

    /*
     * Send the vdev snooplist message down to NSS
     */
    status = nss_wifi_vdev_tx_msg(nss_ctx, &wifivdevmsg);
    if (status != NSS_TX_SUCCESS) {
        osif_nss_warn("Unable to send the vow debug stats to NSS\n");
    }

    /*
     * Blocking call, wait till we get ACK for this msg.
     */
    ret = wait_for_completion_timeout(&osif_nss_vdev_vowp.complete, msecs_to_jiffies(OSIF_NSS_VDEV_CFG_TIMEOUT_MS));
    if (ret == 0) {
        osif_nss_warn("Waiting for vdev config msg ack timed out\n");
        return;
    }
    osifp->umac_vow_counter = osif_nss_vdev_vowp.rx_vow_dbg_counter;
    for (i = 0; i < 8; i++) {
        osifp->tx_dbg_vow_counter[i] = osif_nss_vdev_vowp.tx_vow_dbg_counter[i];
    }
}
EXPORT_SYMBOL(osif_nss_vdev_get_vow_dbg_stats);

void osif_nss_vdev_stale_pkts_timer(void *arg)
{
    struct sk_buff *tmp = NULL;
    uint32_t queue_cnt = 0;

    osif_dev *osifp = (osif_dev *)arg;

    spin_lock_bh(&osifp->queue_lock);

    if (osifp->stale_pkts_timer_interval == 0) {
        spin_unlock_bh(&osifp->queue_lock);
        return;
    }

    queue_cnt = skb_queue_len(&osifp->wifiol_txqueue);
    if (queue_cnt) {
        do {
            tmp = __skb_dequeue(&osifp->wifiol_txqueue);
            if (!tmp) {
                break;
            }
        } while (osif_nss_ol_vap_xmit(osifp, tmp) == 0);
    }

    if (tmp) {
        __skb_queue_head(&osifp->wifiol_txqueue, tmp);
        OS_SET_TIMER(&osifp->wifiol_stale_pkts_timer, osifp->stale_pkts_timer_interval);
    }

    spin_unlock_bh(&osifp->queue_lock);
}

void osif_nss_vdev_enqueue_packet_for_tx(osif_dev *osifp, struct sk_buff *skb)
{
    struct sk_buff *tmp = NULL;
    uint32_t queue_cnt = 0;

    spin_lock_bh(&osifp->queue_lock);
    queue_cnt = skb_queue_len(&osifp->wifiol_txqueue);

    if (queue_cnt) {
        if (queue_cnt > OSIF_NSS_VDEV_PKT_QUEUE_LEN) {
            dev_kfree_skb_any(skb);
        } else {
            __skb_queue_tail(&osifp->wifiol_txqueue, skb);
        }

        do {
            tmp = __skb_dequeue(&osifp->wifiol_txqueue);
            if (!tmp) {
                break;
            }
        } while (osif_nss_ol_vap_xmit(osifp, tmp) == 0);

        if (tmp) {
            __skb_queue_head(&osifp->wifiol_txqueue, tmp);
            OS_SET_TIMER(&osifp->wifiol_stale_pkts_timer, osifp->stale_pkts_timer_interval);
        }
    } else {
        if (osif_nss_ol_vap_xmit(osifp, skb) != 0) {
            __skb_queue_tail(&osifp->wifiol_txqueue, skb);
            OS_SET_TIMER(&osifp->wifiol_stale_pkts_timer, osifp->stale_pkts_timer_interval);
        }
    }
    spin_unlock_bh(&osifp->queue_lock);
}

void osif_nss_vdev_process_mpsta_rx_to_tx(struct net_device *netdev, struct sk_buff *skb, __attribute__((unused)) struct napi_struct *napi)
{
    osif_dev *osifp;
    osif_dev *osifpmpsta;
    struct ieee80211vap *vap;

    struct ol_txrx_vdev_t *pstavdev = NULL;
    struct nss_wifi_vdev_mpsta_per_packet_tx_metadata *nwtm;

    osifp = netdev_priv(netdev);
    vap = osifp->os_if;
    osifpmpsta = osifp; /*Store the present osifp*/

    if (!vap->iv_mpsta) {
        skb->protocol = eth_type_trans(skb, netdev);
        napi_gro_receive(napi, skb);
        return;
    }

    if (unlikely(skb_headroom(skb) < sizeof(struct nss_wifi_vdev_mpsta_per_packet_tx_metadata))){
        if(pskb_expand_head(skb, sizeof(struct nss_wifi_vdev_mpsta_per_packet_tx_metadata), 0, GFP_ATOMIC)){
            printk("Unable to allocated skb data%p head%p tail%p end %p\n", skb->data, skb->head, skb->tail, skb->end);
            dev_kfree_skb_any(skb);
            return ;
        }
    }

    pstavdev = osifp->iv_txrx_handle;

    nwtm = (struct nss_wifi_vdev_mpsta_per_packet_tx_metadata *) skb_push(skb, sizeof(struct nss_wifi_vdev_mpsta_per_packet_tx_metadata));
    nwtm->vdev_id = pstavdev->vdev_id;
    nwtm->metadata_type = NSS_WIFI_VDEV_QWRAP_TYPE_RX_TO_TX;

    osif_nss_vdev_enqueue_packet_for_tx(osifpmpsta, skb);
}

void osif_nss_vdev_process_extap_rx_to_tx(struct net_device *netdev, struct sk_buff *skb)
{
    osif_dev *osifp;
#if DBDC_REPEATER_SUPPORT
    wlan_if_t vap = NULL;
    os_if_t osif = NULL;
    int nwifi;
#endif
    struct nss_wifi_vdev_extap_per_packet_metadata *rxtm = NULL;

    osifp = netdev_priv(netdev);
    if (osif_nss_ol_ext_ap_rx(netdev, skb)) {
        dev_kfree_skb_any(skb);
        return;
    }

#if DBDC_REPEATER_SUPPORT
    vap = osifp->os_if;
    osif = vap->iv_ifp;
    nwifi = ((osif_dev *)osif)->nss_nwifi;
    if (dbdc_rx_process(&osif, &netdev, vap, skb, &nwifi)) {
        return;
    }
#endif

    if (unlikely(skb_headroom(skb) < sizeof(struct nss_wifi_vdev_extap_per_packet_metadata))){
        if(pskb_expand_head(skb, sizeof(struct nss_wifi_vdev_extap_per_packet_metadata), 0, GFP_ATOMIC)){
            printk("Unable to allocated skb data%p head%p tail%p end %p\n", skb->data, skb->head, skb->tail, skb->end);
            dev_kfree_skb_any(skb);
            return ;
        }
    }

    rxtm = (struct nss_wifi_vdev_extap_per_packet_metadata *) skb_push(skb, sizeof(struct nss_wifi_vdev_extap_per_packet_metadata));
    rxtm->pkt_type = NSS_WIFI_VDEV_EXTAP_PKT_TYPE_RX_TO_TX;

    osif_nss_vdev_enqueue_packet_for_tx(osifp, skb);
}

void osif_nss_vdev_process_extap_tx(struct net_device *netdev, struct sk_buff *skb)
{
    osif_dev *osifp;
    struct nss_wifi_vdev_extap_per_packet_metadata *txtm = NULL;

    osifp = netdev_priv(netdev);
    if (osif_nss_ol_ext_ap_tx(netdev, skb)) {
        dev_kfree_skb_any(skb);
        return;
    }

    if (unlikely(skb_headroom(skb) < sizeof(struct nss_wifi_vdev_extap_per_packet_metadata))){
        if(pskb_expand_head(skb, sizeof(struct nss_wifi_vdev_extap_per_packet_metadata), 0, GFP_ATOMIC)){
            printk("Unable to allocated skb data%p head%p tail%p end %p\n", skb->data, skb->head, skb->tail, skb->end);
            dev_kfree_skb_any(skb);
            return ;
        }
    }

    txtm = (struct nss_wifi_vdev_extap_per_packet_metadata *) skb_push(skb, sizeof(struct nss_wifi_vdev_extap_per_packet_metadata));
    txtm->pkt_type = NSS_WIFI_VDEV_EXTAP_PKT_TYPE_TX;

    osif_nss_vdev_enqueue_packet_for_tx(osifp, skb);
}

void osif_nss_vdev_process_mpsta_tx(struct net_device *netdev, struct sk_buff *skb)
{
    osif_dev *osifp;
    osif_dev *osifpmpsta;
    struct ieee80211vap *vap;

    struct ol_txrx_vdev_t *pstavdev = NULL;
    struct nss_wifi_vdev_mpsta_per_packet_tx_metadata *nwtm;


    osifp = netdev_priv(netdev);
    vap = osifp->os_if;

    if (vap->iv_psta  && !vap->iv_mpsta){
        struct ieee80211vap *mvap;
        struct ol_ath_vap_net80211 *avn = OL_ATH_VAP_NET80211(vap);
        struct ol_ath_softc_net80211 *scn = avn->av_sc;

        adf_os_spin_lock_bh(&scn->sc_mpsta_vap_lock);
        mvap = scn->sc_mcast_recv_vap;
        if (!mvap) {
            adf_os_spin_unlock_bh(&scn->sc_mpsta_vap_lock);
            dev_kfree_skb_any(skb);
            return;
        }

        osifpmpsta  = (osif_dev *)mvap->iv_ifp;
        if (!osifpmpsta) {
            adf_os_spin_unlock_bh(&scn->sc_mpsta_vap_lock);
            dev_kfree_skb_any(skb);
            return;
        }
        adf_os_spin_unlock_bh(&scn->sc_mpsta_vap_lock);
    } else  {
        osifpmpsta = osifp; /*Store the present osifp*/
        if (osif_ol_wrap_tx_process(&osifp, vap, &skb)) {
            if (skb) {
                dev_kfree_skb_any(skb);
	    }
            return;
        }
    }

    pstavdev = osifp->iv_txrx_handle;
    if (!pstavdev) {
        dev_kfree_skb_any(skb);
        return;
    }

    if (unlikely(skb_headroom(skb) < sizeof(struct nss_wifi_vdev_mpsta_per_packet_tx_metadata))){

        if(pskb_expand_head(skb, sizeof(struct nss_wifi_vdev_mpsta_per_packet_tx_metadata), 0, GFP_ATOMIC)){
            printk("Unable to allocated skb data%p head%p tail%p end %p\n", skb->data, skb->head, skb->tail, skb->end);
            dev_kfree_skb_any(skb);
            return ;
        }
    }

    nwtm = (struct nss_wifi_vdev_mpsta_per_packet_tx_metadata *) skb_push(skb, sizeof(struct nss_wifi_vdev_mpsta_per_packet_tx_metadata));
    nwtm->vdev_id = pstavdev->vdev_id;
    nwtm->metadata_type = NSS_WIFI_VDEV_QWRAP_TYPE_TX;

    osif_nss_vdev_enqueue_packet_for_tx(osifpmpsta, skb);
}

struct net_device *osif_nss_vdev_process_mpsta_rx(struct net_device *netdev, struct sk_buff *skb, uint32_t peer_id)
{
    osif_dev *osifp;
    osif_dev *psta_osifp;
    struct ol_txrx_vdev_t *vdev = NULL;
    struct ol_txrx_pdev_t *pdev = NULL;
    struct ol_txrx_vdev_t *pstavdev = NULL;
    struct ol_txrx_peer_t *pstapeer;
    struct net_device *pst_netdev;
    struct ieee80211vap *pst_vap;
#if DBDC_REPEATER_SUPPORT
    os_if_t osif = NULL;
    int nwifi;
#endif

    osifp = netdev_priv(netdev);
    vdev = osifp->iv_txrx_handle;

    if (!vdev) {
        dev_kfree_skb_any(skb);
        return NULL;
    }

    pdev = vdev->pdev;

    pstapeer = ol_txrx_peer_find_by_id(pdev, peer_id);

    if (!pstapeer) {
        printk("no peer available free packet \n");
        dev_kfree_skb_any(skb);
        return NULL;
    }

    pstavdev = pstapeer->vdev;
    if (!pstavdev) {
        printk("no vdev available free packet \n");
        dev_kfree_skb_any(skb);
        return NULL;
    }

    psta_osifp = (osif_dev *)pstavdev->osif_vdev;

    pst_vap =  psta_osifp->os_if;
    pst_netdev = psta_osifp->netdev;
    skb->dev = pst_netdev;
    /*
     * When return value is 1 packet is already sent out
     */
    if(osif_ol_wrap_rx_process(&osifp, &pst_netdev, pst_vap, skb)) {
        return NULL;
    }

#if DBDC_REPEATER_SUPPORT
    osif = pst_vap->iv_ifp;
    nwifi = ((osif_dev *)osif)->nss_nwifi;
    if(dbdc_rx_process(&osif, &pst_netdev, pst_vap, skb, &nwifi)) {
       return NULL;
    }
#endif

    return  pst_netdev;
}

/*
 * osif_nss_vdev_set_dscp_tid_map()
 * 	API to notify dscp-tid mapping in NSS.
 */
void osif_nss_vdev_set_dscp_tid_map(struct ol_txrx_vdev_t *vdev, uint32_t* dscp_map)
{
    struct nss_ctx_instance *nss_ctx = NULL;
    osif_dev *osifp = NULL;
    int32_t if_num = 0;
    struct nss_wifi_vdev_msg wifivdevmsg;
    struct nss_wifi_vdev_dscp_tid_map *dscpmap_msg = &wifivdevmsg.msg.vdev_dscp_tid_map;
    bool status;

    if (!vdev) {
        return;
    }

    nss_ctx = osif_nss_vdev_get_nss_wifiol_ctx(vdev);
    osifp = (osif_dev *)vdev->osif_vdev;
    if_num = osifp->nss_ifnum;
    if (!nss_ctx) {
        return;
    }

    memcpy(dscpmap_msg->dscp_tid_map, dscp_map, sizeof (dscpmap_msg->dscp_tid_map));
    nss_wifi_vdev_msg_init(&wifivdevmsg, if_num, NSS_WIFI_VDEV_DSCP_TID_MAP_MSG, sizeof(struct nss_wifi_vdev_dscp_tid_map), NULL, NULL);

    /*
     * Send the dscp to tid map message down to NSS
     */
    status = nss_wifi_vdev_tx_msg(nss_ctx, &wifivdevmsg);
    if (status != NSS_TX_SUCCESS) {
        osif_nss_warn("Unable to send the dscp-tid map message to NSS\n");
    }
}
EXPORT_SYMBOL(osif_nss_vdev_set_dscp_tid_map);

/*
 * osif_nss_vdev_process_wnm_tfs_tx()
 *     API to process pkts exceptioned from NSS-FW when wnm_tfs is set.
 */
void osif_nss_vdev_process_wnm_tfs_tx(struct net_device *netdev, struct sk_buff *skb)
{
    osif_dev *osifp;
    struct ieee80211vap *vap;

    osifp = netdev_priv(netdev);
    vap = osifp->os_if;

    if (vap) {
        if (wlan_wnm_tfs_filter(vap, skb)) {
            dev_kfree_skb_any(skb);
        } else {
            osif_nss_vdev_enqueue_packet_for_tx(osifp, skb);
        }

    } else {
        dev_kfree_skb_any(skb);
    }
}
