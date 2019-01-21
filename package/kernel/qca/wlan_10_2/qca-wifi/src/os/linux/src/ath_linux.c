/*
 * Copyright (c) 2010, Atheros Communications Inc.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */
/*
 * Copyright (c) 2013 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 */

#include <osdep.h>
#include <wbuf.h>

#include "if_media.h"
#include "ieee80211_var.h"
#include "ieee80211_aponly.h"

#include "if_athvar.h"
#include "ath_cwm.h"
#include "osif_private.h"
#include "if_athproto.h"

#ifdef ATH_BAND_STEERING
#include "ath_band_steering.h"
#endif

#ifdef ATH_SUPPORT_HTC
#include "ath_htc_wmi.h"
#include "htc_thread.h"
#endif

#ifdef ATH_TX99_DIAG
#include "ath_tx99.h"
#endif

#if ATH_SUPPORT_IBSS_NETLINK_NOTIFICATION
#include "ath_netlink.h"
#endif

#include "ald_netlink.h"

#if ATH_SUPPORT_FLOWMAC_MODULE
#include <flowmac_api.h>
#endif

#if UMAC_SUPPORT_ACFG
#include <ieee80211_ioctl_acfg.h>
#include <acfg_event_types.h>
#include <acfg_drv_if.h>
#include <acfg_drv_event.h>
#endif

/*
 * Maximum acceptable MTU
 * MAXFRAMEBODY - WEP - QOS - RSN/WPA:
 * 2312 - 8 - 2 - 12 = 2290
 */
#define ATH_MAX_MTU     2290
#define ATH_MIN_MTU     32


#if defined(ATH_TX_BUF_FLOW_CNTL) ||  defined(ATH_DEBUG)
extern int ACBKMinfree;
extern int ACBEMinfree;
extern int ACVIMinfree;
extern int ACVOMinfree;
extern int CABMinfree;
extern int UAPSDMinfree;
#endif

extern struct ath_softc_net80211 *global_scn[10];
extern int num_global_scn;

static struct ath_reg_parm ath_params = {
    .wModeSelect = MODE_SELECT_ALL,
    .NetBand = MODE_SELECT_ALL,
    .txAggrEnable = 1,
    .rxAggrEnable = 1,
    .txAmsduEnable = 1,
    .aggrLimit = IEEE80211_AMPDU_LIMIT_DEFAULT,
    .aggrSubframes = IEEE80211_AMPDU_SUBFRAME_DEFAULT,
    .aggrProtDuration = 8192,
    .aggrProtMax = 8192,
    .txRifsEnable = 0,
    .rifsAggrDiv = IEEE80211_RIFS_AGGR_DIV,
#ifdef ATH_RB
    .rxRifsEnable = ATH_RB_MODE_DETECT,
    .rxRifsTimeout = ATH_RB_DEF_TIMEOUT,
    .rxRifsSkipThresh = ATH_RB_DEF_SKIP_THRESH,
#endif
    .txChainMaskLegacy = 1,
    .rxChainMaskLegacy = 1,
    .rxChainDetectThreshA = 35,
    .rxChainDetectThreshG = 35,
    .rxChainDetectDeltaA = 30,
    .rxChainDetectDeltaG = 30,
    .calibrationTime = 30,
#if ATH_SUPPORT_LED
    .gpioPinFuncs = {GPIO_PIN_FUNC_0,GPIO_PIN_FUNC_1,GPIO_PIN_FUNC_2},
    .gpioLedCustom = ATH_LED_CUSTOMER,
#else
    .gpioPinFuncs = {1, 7, 7},
#endif
    .hwTxRetries = 4,
    .extendedChanMode = 1,
    .DmaStopWaitTime = 4,
    .swBeaconProcess = 1,
    .stbcEnable = 1,
    .ldpcEnable = 1,
    .cwmEnable = 1,
    .wpsButtonGpio = 0,
#ifdef ATH_SUPPORT_TxBF
    .TxBFSwCvTimeout = 1000 ,
#endif
#if ATH_SUPPORT_SPECTRAL
	.spectralEnable = 1,
#endif
#if ATH_SUPPORT_PAPRD
    .paprdEnable = 1,
#endif
#if ATH_TX_BUF_FLOW_CNTL
    .ACBKMinfree = 48,
    .ACBEMinfree = 32,
    .ACVIMinfree = 16,
    .ACVOMinfree = 0,
    .CABMinfree = 48,
    .UAPSDMinfree = 0,
#endif
#ifdef ATH_SWRETRY
    .numSwRetries = 2,
#endif
#if ATH_SUPPORT_FLOWMAC_MODULE
    .osnetif_flowcntrl = 0,
    .os_ethflowmac_enable = 0,
#endif
#if ATH_SUPPORT_AGGR_BURST
    .burstEnable = 0,
    .burstDur = 5100,
#endif
};

static struct ieee80211_reg_parameters wlan_reg_params = {
    .transmitRetrySet = 0x04040404,
    .sleepTimePwrSave = 100,         /* wake up every beacon */
    .sleepTimePwrSaveMax = 1000,     /* wake up every 10 th beacon */
    .sleepTimePerf=100,              /* station wakes after this many mS in max performance mode */
    .inactivityTimePwrSaveMax=400,   /* in max PS mode, how long (in mS) w/o Tx/Rx before going back to sleep */
    .inactivityTimePwrSave=200,      /* in normal PS mode, how long (in mS) w/o Tx/Rx before going back to sleep */
    .inactivityTimePerf=400,         /* in max perf mode, how long (in mS) w/o Tx/Rx before going back to sleep */
    .psPollEnabled=0,                /* Use PS-POLL to retrieve data frames after TIM is received */
    .wmeEnabled    = 1,
    .enable2GHzHt40Cap = 1,
    .cwmEnable = 1,
    .cwmExtBusyThreshold = IEEE80211_CWM_EXTCH_BUSY_THRESHOLD,
    .ignore11dBeacon = 1,
    .p2pGoUapsdEnable = 1,
    .extapUapsdEnable = 1,
    .shortPreamble = 1,               /*ShortPreamble is enabled default */
#ifdef ATH_SUPPORT_TxBF
    .autocvupdate = 0,
#define DEFAULT_PER_FOR_CVUPDATE 30
    .cvupdateper = DEFAULT_PER_FOR_CVUPDATE,
#endif
};

/* The code below is used to register a hw_caps file in sysfs */
static ssize_t wifi_hwcaps_show(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
    struct net_device *net = to_net_dev(dev);
    struct ath_softc_net80211 *scn = ath_netdev_priv(net);
    struct ieee80211com *ic = &scn->sc_ic;

    u_int32_t hw_caps = ic->ic_modecaps;

    strcpy(buf, "802.11");
    if(hw_caps &
        1 << IEEE80211_MODE_11A)
        strcat(buf, "a");
    if(hw_caps &
        1 << IEEE80211_MODE_11B)
        strcat(buf, "b");
    if(hw_caps &
        1 << IEEE80211_MODE_11G)
        strcat(buf, "g");
    if(hw_caps &
        (1 << IEEE80211_MODE_11NA_HT20 |
         1 << IEEE80211_MODE_11NG_HT20 |
         1 << IEEE80211_MODE_11NA_HT40PLUS |
         1 << IEEE80211_MODE_11NA_HT40MINUS |
         1 << IEEE80211_MODE_11NG_HT40PLUS |
         1 << IEEE80211_MODE_11NG_HT40MINUS |
         1 << IEEE80211_MODE_11NG_HT40 |
         1 << IEEE80211_MODE_11NA_HT40))
        strcat(buf, "n");
    if(hw_caps &
        (1 << IEEE80211_MODE_11AC_VHT20 |
         1 << IEEE80211_MODE_11AC_VHT40PLUS |
         1 << IEEE80211_MODE_11AC_VHT40MINUS |
         1 << IEEE80211_MODE_11AC_VHT40 |
         1 << IEEE80211_MODE_11AC_VHT80))
        strcat(buf, "/ac");
    return strlen(buf);
}
static DEVICE_ATTR(hwcaps, S_IRUGO, wifi_hwcaps_show, NULL);
static struct attribute *wifi_device_attrs[] = {
    &dev_attr_hwcaps.attr,
    NULL
};

static struct attribute_group wifi_attr_group = {
       .attrs  = wifi_device_attrs,
};


/*
** Prototype for iw attach
*/

#ifdef ATH_SUPPORT_LINUX_STA
#ifdef CONFIG_SYSCTL
void ath_dynamic_sysctl_register(struct ath_softc *sc);
void ath_dynamic_sysctl_unregister(struct ath_softc *sc);
#endif
#endif
#if OS_SUPPORT_ASYNC_Q
static void os_async_mesg_handler( void  *ctx, u_int16_t  mesg_type, u_int16_t  mesg_len, void  *mesg );
#endif

void ath_iw_attach(struct net_device *dev);
#if !NO_SIMPLE_CONFIG
extern int32_t unregister_simple_config_callback(char *name);
extern int32_t register_simple_config_callback (char *name, void *callback, void *arg1, void *arg2);
static irqreturn_t jumpstart_intr(int cpl, void *dev_id, struct pt_regs *regs, void *push_dur);
#endif

#ifdef QCA_PARTNER_PLATFORM
extern 	int osif_pltfrm_interupt_register(struct net_device *dev,int type);
extern int osif_pltfrm_interupt_unregister(struct net_device *dev,int level);
extern void ath_pltfrm_init( struct net_device *dev );
#endif

#ifdef ATH_TX99_DIAG
extern u_int8_t tx99_ioctl(ath_dev_t dev, struct ath_softc *sc, int cmd, void *addr);
#endif


/* begin: add by chenzejun for mac scan by 2015.12.4 */
extern void AP_Netlink_init(void);
extern void AP_Netlink_destroy(void);
/* end: add by chenzejun for mac scan by 2015.12.4 */


#ifndef ADF_SUPPORT
void *
OS_ALLOC_VAP(osdev_t osdev, u_int32_t len)
{
    void *netif;

    netif = OS_MALLOC(osdev, len, GFP_KERNEL);
    if (netif != NULL)
        OS_MEMZERO(netif, len);

    return netif;
}

void
OS_FREE_VAP(void *netif)
{
    OS_FREE(netif);
}

#endif

int
ath_get_netif_settings(ieee80211_handle_t ieee)
{
    struct ieee80211com *ic = NET80211_HANDLE(ieee);
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    osdev_t osdev = scn->sc_osdev;
    struct net_device *dev =  osdev->netdev;
    int flags = 0;

    if (dev->flags & IFF_RUNNING)
        flags |= ATH_NETIF_RUNNING;
    if (dev->flags & IFF_PROMISC)
        flags |= ATH_NETIF_PROMISCUOUS;
    if (dev->flags & IFF_ALLMULTI)
        flags |= ATH_NETIF_ALLMULTI;

    return flags;
}

#if ATH_VAP_DELETE_ON_RMMOD
static int
ath_vap_delete_on_rmmod(struct ath_softc_net80211 *scn) 
{
    struct ieee80211vap *vap = NULL, *tvap = NULL;
    struct ieee80211com *ic = &scn->sc_ic;
    int delete_in_progress = 0;
#define  WAIT_VAP_IDLE_INTERVALL 10 
    int gracetime = WAIT_VAP_IDLE_INTERVALL;

    do {
        IEEE80211_COMM_LOCK(ic);
        if (!TAILQ_EMPTY(&ic->ic_vaps)) {
            TAILQ_FOREACH_SAFE(vap, &ic->ic_vaps, iv_next, tvap) {
                if (((osif_dev *)((os_if_t)vap->iv_ifp))->is_delete_in_progress) {
                    delete_in_progress = 1;
                }
            }
        }
        IEEE80211_COMM_UNLOCK(ic);

        if (delete_in_progress) {
            msleep(1000);
            if (--gracetime == 0) {
                printk(KERN_ERR "timeout waiting for vap %s\n", 
                       ((osif_dev *)((os_if_t)vap->iv_ifp))->netdev->name);
                return -1;
            }
        } else {
            break;
        }
    } while(1);
    /* 
     *  If packets are heldup in the lmac queue, free the node ref count.
     */
    ath_draintxq(scn->sc_dev,0,0);

    /* 
     * Don't add any more VAPs after this.
     * Else probably the detach should be done with rtnl_lock() held.
     */
    scn->sc_in_delete = 1;

    /* Bring down and delete vaps */
    if (!TAILQ_EMPTY(&ic->ic_vaps)) {
        TAILQ_FOREACH_SAFE(vap, &ic->ic_vaps, iv_next, tvap) {
            printk(KERN_ERR "%s: vap %s still registered, cleaning up\n",
                   __func__, ((osif_dev *)((os_if_t)vap->iv_ifp))->netdev->name);
            dev_close(((osif_dev *)((os_if_t)vap->iv_ifp))->netdev);
            osif_ioctl_delete_vap(((osif_dev *)((os_if_t)vap->iv_ifp))->netdev);
        }
    }
    return 0;
}
#else
static int
ath_vap_delete_on_rmmod(struct ath_softc_net80211 *scn) 
{
    /* dummy */ 
    return 0;
}
#endif /* ATH_VAP_DELETE_ON_RMMOD */

/*
 * Merge multicast addresses from all vap's to form the
 * hardware filter.  Ideally we should only inspect our
 * own list and the 802.11 layer would merge for us but
 * that's a bit difficult so for now we put the onus on
 * the driver.
 */
void
ath_mcast_merge(ieee80211_handle_t ieee, u_int32_t mfilt[2])
{
    struct ieee80211com *ic = NET80211_HANDLE(ieee);
    struct ieee80211vap *vap;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
    struct netdev_hw_addr *ha;  
#else
    struct dev_mc_list *mc;
#endif
    u_int32_t val;
    u_int8_t pos;

    mfilt[0] = mfilt[1] = 0;
    /* XXX locking */
    TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) {
        struct net_device *dev;
        os_if_t osif = vap->iv_ifp;
        if (osif == NULL)
            continue;

        dev = ((osif_dev *)osif)->netdev;
        if (dev == NULL)
            continue;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
        netdev_for_each_mc_addr(ha, dev) {  
            /* calculate XOR of eight 6-bit values */ 
            val = LE_READ_4(ha->addr + 0); 
            pos = (val >> 18) ^ (val >> 12) ^ (val >> 6) ^ val; 
            val = LE_READ_4(ha->addr + 3); 
            pos ^= (val >> 18) ^ (val >> 12) ^ (val >> 6) ^ val; 
            pos &= 0x3f; 
            mfilt[pos / 32] |= (1 << (pos % 32)); 
        } 
#else
        for (mc = dev->mc_list; mc; mc = mc->next) {
            /* calculate XOR of eight 6bit values */
            val = LE_READ_4(mc->dmi_addr + 0);
            pos = (val >> 18) ^ (val >> 12) ^ (val >> 6) ^ val;
            val = LE_READ_4(mc->dmi_addr + 3);
            pos ^= (val >> 18) ^ (val >> 12) ^ (val >> 6) ^ val;
            pos &= 0x3f;
            mfilt[pos / 32] |= (1 << (pos % 32));
        }
#endif         
    }
}


static int
ath_netdev_open(struct net_device *dev)
{
    struct ath_softc_net80211 *scn = ath_netdev_priv(dev);
    int ath_ret;
    struct ieee80211com *ic = &scn->sc_ic;
    struct ieee80211vap *vap;
    osif_dev  *osifp;
    struct net_device *netdev;
    u_int8_t myaddr[IEEE80211_ADDR_LEN];
    u_int8_t id = 0;

#ifdef ATH_BUS_PM
    if (scn->sc_osdev->isDeviceAsleep)
        return -EPERM;
#endif /* ATH_BUS_PM */

    ath_ret = ath_resume(scn);
    if(ath_ret == 0){ 
        dev->flags |= IFF_UP | IFF_RUNNING;      /* we are ready to go */
        /*  If physical radio interface wifiX is shutdown,all virtual interfaces(athX) should gets shutdown and 
            all these downed virtual interfaces should gets up when physical radio interface(wifiX) is up.Refer EV 116786.
         */ 
        vap = TAILQ_FIRST(&ic->ic_vaps);
        while (vap != NULL) {
            osifp = (osif_dev *)vap->iv_ifp;
            netdev = osifp->netdev;
            ieee80211vap_get_macaddr(vap, myaddr);
            ATH_GET_VAP_ID(myaddr, wlan_vap_get_hw_macaddr(vap), id);
            if( ic->id_mask_vap_downed & ( 1 << id ) ){
                dev_change_flags(netdev,netdev->flags | ( IFF_UP ));
                ic->id_mask_vap_downed &= (~( 1 << id )); 
            }
            vap = TAILQ_NEXT(vap, iv_next);
        }
    }
    return ath_ret;
}

static int
ath_netdev_stop(struct net_device *dev)
{
    struct ath_softc_net80211 *scn = ath_netdev_priv(dev);

    struct ieee80211com *ic = &scn->sc_ic;
    struct ieee80211vap *vap;
    osif_dev  *osifp;
    struct net_device *netdev;
    u_int8_t myaddr[IEEE80211_ADDR_LEN];
    u_int8_t id = 0;

    /*  If physical radio interface wifiX is shutdown,all virtual interfaces(athX) should gets shutdown and
        all these downed virtual interfaces should gets up when physical radio interface(wifiX) is up.Refer EV 116786.
     */

    vap = TAILQ_FIRST(&ic->ic_vaps);
    while (vap != NULL) {
        osifp = (osif_dev *)vap->iv_ifp;
        netdev = osifp->netdev;
        if (IS_IFUP(netdev)) {
            dev_change_flags(netdev,netdev->flags & ( ~IFF_UP ));
            ieee80211vap_get_macaddr(vap, myaddr);
            ATH_GET_VAP_ID(myaddr, wlan_vap_get_hw_macaddr(vap), id);
            ic->id_mask_vap_downed |= ( 1 << id);
        }
        vap = TAILQ_NEXT(vap, iv_next);
    }
    dev->flags &= ~IFF_RUNNING;
    return ath_suspend(scn);
}



static int
ath_netdev_hardstart_generic(struct sk_buff *skb, struct net_device *dev)
{
    struct ath_softc_net80211 *scn = ath_netdev_priv(dev);
    struct ath_softc *sc = ATH_DEV_TO_SC(scn->sc_dev);
    struct ieee80211_cb *cb;
    struct ieee80211_node *ni;
    int error = 0;
#ifndef ATH_SUPPORT_HTC
    struct ieee80211com *ic = &scn->sc_ic;
    struct ether_header *eh = (struct ether_header *)skb->data;
    int ismulti = IEEE80211_IS_MULTICAST(eh->ether_dhost) ? 1 : 0;
    u_int16_t addba_status;
    u_int32_t txq_depth, txq_aggr_depth;
    int  buffer_limit;
    u_int32_t  txbuf_freecount;
    struct ath_txq *txq;
    int qnum;	
#endif
    int early_drop = 1; /* allow tx buffer calculation to drop the packet by default */
    /* make early_drop = 0 for important control plane packets like EAPOL and DHCP */

    cb = (struct ieee80211_cb *)skb->cb;
    ni = cb->ni;
#if !ATH_SUPPORT_VOWEXT
    if(wbuf_is_highpriority(skb))
	early_drop = 0;
#endif

#if defined(ATH_SUPPORT_P2P)
     cb->complete_handler = NULL;
     cb->complete_handler_arg = NULL;
#endif  /* ATH_SUPPORT_P2P */
    /*
     * device must be up and running
     */
    if ((dev->flags & (IFF_RUNNING|IFF_UP)) != (IFF_RUNNING|IFF_UP)) {
        error = -ENETDOWN;
        goto bad;
    }

    /*
     * NB: check for valid node in case kernel directly sends packets
     * on wifiX interface (such as broadcast packets generated by ipv6)
     */
    if (ni == NULL) {
        dev_kfree_skb(skb);
        return 0;
    }

#ifdef ATH_SUPPORT_UAPSD
    /* Limit UAPSD node queue depth to WME_UAPSD_NODE_MAXQDEPTH */
    if ((ni->ni_flags & IEEE80211_NODE_UAPSD) &&
        scn->sc_ops->uapsd_depth(ATH_NODE_NET80211(ni)->an_sta) >= WME_UAPSD_NODE_MAXQDEPTH)
    {
        goto bad;
    }
#endif

#ifndef ATH_SUPPORT_HTC

    qnum =  scn->sc_ac2q[skb->priority];
    txq  = &sc->sc_txq[qnum];

    txq_depth = scn->sc_ops->txq_depth(scn->sc_dev, scn->sc_ac2q[skb->priority]);
    txq_aggr_depth = scn->sc_ops->txq_aggr_depth(scn->sc_dev, scn->sc_ac2q[skb->priority]);
	
    ic->ic_addba_status(ni, cb->u_tid, &addba_status);

    /*
     * This logic throttles legacy and unaggregated HT frames if they share the hardware
     * queue with aggregates. This improves the transmit throughput performance to
     * aggregation enabled nodes when they coexist with legacy nodes.
     */
    /* Do not throttle EAPOL packets - this causes the REKEY packets
     * to be dropped and station disconnects.
     */
    DPRINTF(scn, ATH_DEBUG_RESET, "skb->priority=%d cb->u_tid=%d addba_status=%d txq_aggr_depth=%d txq_depth=%d\n",skb->priority, cb->u_tid, addba_status, txq_aggr_depth, txq_depth);

    if ((addba_status != IEEE80211_STATUS_SUCCESS) 
        && (txq_aggr_depth > 0)
        && early_drop)
		{
        if (txq_depth >= 25) {
            goto bad;
        } 
        }
#ifdef notyet /* This should be ported from 7.3 */
#ifndef ATH_SUPPORT_LINUX_STA
        /* Bug 38437 - If the current q depth is greater than the limit we have set for 
           legacy frames, we can put this frame in the holding Q for legacy frames or 
           discard it if the holding Q is also full */

        if (txq_depth >= scn->sc_limit_legacy_frames)  {

            if(scn->sc_limit_legacy_txq_count < (scn->sc_limit_legacy_frames)) {
                enq_legacy_wbuf(scn, skb);
                return 0;
            } else {
                // Holding Q is full, discard it
                goto bad;
            }
        }
        /* Bug 38437 - Holding Q contains older frames which must be sent out first
           so Q this one and send out an older frame from the holding Q*/
        if (!(TAILQ_EMPTY(&scn->sc_limit_legacy_txq)))  {
            error = deq_legacy_wbuf(scn);
            enq_legacy_wbuf(scn, skb);
            return 0;
        }   
#endif
#endif
   
    /*
     * Try to avoid running out of descriptors 
     */
     txbuf_freecount = scn->sc_ops->get_txbuf_free(scn->sc_dev);
#if !ATH_SUPPORT_VOWEXT
     if (ismulti && early_drop) {
#else
    if (ismulti) {
#endif
         buffer_limit = MULTICAST_DROP_THRESHOLD;
         if (txbuf_freecount <= buffer_limit)   /* check for 10% txbuf availability*/
         {   
            goto bad;
        }
    }
    /* Reserve 16 Tx buffers for EAPOL, DHCP, ARP and QOS NULL frames */
    if (early_drop && (txbuf_freecount <=  txq->axq_minfree + 16 )) {
            goto bad;
    }	
#endif

    error = ath_tx_send(skb);

#ifdef notyet /* This should be ported from 7.3 */
#ifndef ATH_SUPPORT_LINUX_STA
    if(scn->sc_limit_legacy_txq_count > (txq_depth)) {
        if ((addba_status == IEEE80211_STATUS_SUCCESS) && !(TAILQ_EMPTY(&scn->sc_limit_legacy_txq)))  {
                /* Bug 38437 - For non legacy frames, send from the holding Q if not empty
                   this prevents starvation of the legacy node */
                DPRINTF(scn, ATH_DEBUG_RESET, "txq_saved=%d txq_aggr_depth=%d txq_depth=%d\n",scn->sc_limit_legacy_txq_count, txq_aggr_depth, txq_depth);
                deq_legacy_wbuf(scn);
        }
    } 
#endif
#endif
    
    if (error) {
        DPRINTF(scn, ATH_DEBUG_XMIT, "%s: Tx failed with error %d\n", 
            __func__, error);
    }
    return 0;  

bad:
    __11nstats(((struct ath_softc *)(scn->sc_dev)),tx_drops);
    sc->sc_stats.ast_tx_nobuf++;
    sc->sc_stats.ast_txq_packets[scn->sc_ac2q[skb->priority]]++;
    sc->sc_stats.ast_txq_nobuf[scn->sc_ac2q[skb->priority]]++;
	
    IEEE80211_TX_COMPLETE_WITH_ERROR(skb);
    DPRINTF(scn, ATH_DEBUG_XMIT, "%s: Tx failed with error %d\n", 
            __func__, error);
    return 0;  
}



static int
ath_netdev_hardstart(struct sk_buff *skb, struct net_device *dev)
{
    do_ath_netdev_hardstart(skb,dev);
}
static void
ath_netdev_tx_timeout(struct net_device *dev)
{
    struct ath_softc_net80211 *scn = ath_netdev_priv(dev);

    DPRINTF(scn, ATH_DEBUG_WATCHDOG, "%s: %sRUNNING\n",
            __func__, (dev->flags & IFF_RUNNING) ? "" : "!");

	if (dev->flags & IFF_RUNNING) {
        scn->sc_ops->reset_start(scn->sc_dev, 0, 0, 0);
        scn->sc_ops->reset(scn->sc_dev);
        scn->sc_ops->reset_end(scn->sc_dev, 0);
	}
}

static int
ath_netdev_set_macaddr(struct net_device *dev, void *addr)
{
    struct ath_softc_net80211 *scn = ath_netdev_priv(dev);
    struct ieee80211com *ic = &scn->sc_ic;
    struct sockaddr *mac = addr;

    if (netif_running(dev)) {
        DPRINTF(scn, ATH_DEBUG_ANY,
            "%s: cannot set address; device running\n", __func__);
        return -EBUSY;
    }
    DPRINTF(scn, ATH_DEBUG_ANY, "%s: %.2x:%.2x:%.2x:%.2x:%.2x:%.2x\n",
        __func__,
        mac->sa_data[0], mac->sa_data[1], mac->sa_data[2],
        mac->sa_data[3], mac->sa_data[4], mac->sa_data[5]);

    /* XXX not right for multiple vap's */
    IEEE80211_ADDR_COPY(ic->ic_myaddr, mac->sa_data);
    IEEE80211_ADDR_COPY(dev->dev_addr, mac->sa_data);
    scn->sc_ops->set_macaddr(scn->sc_dev, dev->dev_addr);
    return 0;
}

static void
ath_netdev_set_mcast_list(struct net_device *dev)
{
    struct ath_softc_net80211 *scn = ath_netdev_priv(dev);
#ifdef ATH_SUPPORT_HTC
    if ((!scn) ||
        (scn && scn->sc_htc_delete_in_progress)){
        printk("%s : #### delete is in progress, scn %p \n", __func__, scn);
        return;
    }

    ath_put_defer_item(scn->sc_osdev, scn->sc_ops->mc_upload, WORK_ITEM_SET_MULTICAST,
            (void *)scn->sc_dev, NULL, NULL);
#else
    scn->sc_ops->mc_upload(scn->sc_dev);
#endif
}

static int
ath_change_mtu(struct net_device *dev, int mtu)
{
    if (!(ATH_MIN_MTU < mtu && mtu <= ATH_MAX_MTU)) {
        DPRINTF((struct ath_softc_net80211 *) ath_netdev_priv(dev),
            ATH_DEBUG_ANY, "%s: invalid %d, min %u, max %u\n",
            __func__, mtu, ATH_MIN_MTU, ATH_MAX_MTU);
        return -EINVAL;
    }
    DPRINTF((struct ath_softc_net80211 *) ath_netdev_priv(dev), ATH_DEBUG_ANY,
        "%s: %d\n", __func__, mtu);

    dev->mtu = mtu;
    return 0;
}

/*
 * Diagnostic interface to the HAL.  This is used by various
 * tools to do things like retrieve register contents for
 * debugging.  The mechanism is intentionally opaque so that
 * it can change frequently w/o concern for compatiblity.
 */
static int
ath_ioctl_diag(struct ath_softc_net80211 *scn, struct ath_diag *ad)
{
    struct ath_hal *ah = (ATH_DEV_TO_SC(scn->sc_dev))->sc_ah;
    u_int id = ad->ad_id & ATH_DIAG_ID;
    void *indata = NULL;
    void *outdata = NULL;
    u_int32_t insize = ad->ad_in_size;
    u_int32_t outsize = ad->ad_out_size;
    int error = 0;
    if (ad->ad_id & ATH_DIAG_IN) {
        /*
         * Copy in data.
         */
        indata = kmalloc(insize, GFP_KERNEL);
        if (indata == NULL) {
            error = -ENOMEM;
            goto bad;
        }
        if (__xcopy_from_user(indata, ad->ad_in_data, insize)) {
            error = -EFAULT;
            goto bad;
        }
    }
    if (ad->ad_id & ATH_DIAG_DYN) {
        /*
         * Allocate a buffer for the results (otherwise the HAL
         * returns a pointer to a buffer where we can read the
         * results).  Note that we depend on the HAL leaving this
         * pointer for us to use below in reclaiming the buffer;
         * may want to be more defensive.
         */
        outdata = kmalloc(outsize, GFP_KERNEL);
        if (outdata == NULL) {
            error = -ENOMEM;
            goto bad;
        }
    }
    if (ath_hal_getdiagstate(ah, id, indata, insize, &outdata, &outsize)) {
		printk("alloc size = %d, new = %d\n", ad->ad_out_size, outsize);
        if (outsize < ad->ad_out_size)
            ad->ad_out_size = outsize;
        if (outdata &&
             _copy_to_user(ad->ad_out_data, outdata, ad->ad_out_size))
            error = -EFAULT;
    } else {
        error = -EINVAL;
    }
bad:
    if ((ad->ad_id & ATH_DIAG_IN) && indata != NULL)
        kfree(indata);
    if ((ad->ad_id & ATH_DIAG_DYN) && outdata != NULL)
        kfree(outdata);
    return error;

}

#ifdef ATH_USB
#include "usb_eth.h"
#else
extern int ath_ioctl_ethtool(struct ath_softc_net80211 *scn, int cmd, void *addr);
#endif

#if defined(ATH_SUPPORT_DFS) || defined(ATH_SUPPORT_SPECTRAL)
static int
ath_ioctl_phyerr(struct ath_softc_net80211 *scn, struct ath_diag *ad)
{
     void *indata=NULL;
     void *outdata=NULL;
     int error = -EINVAL;
     u_int32_t insize = ad->ad_in_size;
     u_int32_t outsize = ad->ad_out_size;
     u_int id= ad->ad_id & ATH_DIAG_ID;
    struct ieee80211com *ic = &scn->sc_ic;

    if (ad->ad_id & ATH_DIAG_IN) {
                /*
                 * Copy in data.
                 */
                indata = OS_MALLOC(scn->sc_osdev,insize, GFP_KERNEL);
                if (indata == NULL) {
                        error = -ENOMEM;
                        goto bad;
                }
                if (__xcopy_from_user(indata, ad->ad_in_data, insize)) {
                        error = -EFAULT;
                        goto bad;
                }
                id = id & ~ATH_DIAG_IN;
        }
        if (ad->ad_id & ATH_DIAG_DYN) {
                /*
                 * Allocate a buffer for the results (otherwise the HAL
                 * returns a pointer to a buffer where we can read the
                 * results).  Note that we depend on the HAL leaving this
                 * pointer for us to use below in reclaiming the buffer;
                 * may want to be more defensive.
                 */
                outdata = OS_MALLOC(scn->sc_osdev, outsize, GFP_KERNEL);
                if (outdata == NULL) {
                        error = -ENOMEM;
                        goto bad;
                }
                id = id & ~ATH_DIAG_DYN;
        }

#if 1 // UMACDFS: Move this call to net80211 layer as DFS moved out of lmac. ATH_SUPPORT_DFS
	error = ic->ic_dfs_control(
            ic, id, indata, insize, outdata, &outsize);
#endif

#if ATH_SUPPORT_SPECTRAL
    if (error ==  -EINVAL ) {
        error = scn->sc_ops->ath_spectral_control(
            scn->sc_dev, id, indata, insize, outdata, &outsize);
    }
#endif

         if (outsize < ad->ad_out_size)
                ad->ad_out_size = outsize;

        if (outdata &&
            _copy_to_user(ad->ad_out_data, outdata, ad->ad_out_size))
                error = -EFAULT;
bad:
        if ((ad->ad_id & ATH_DIAG_IN) && indata != NULL)
                OS_FREE(indata);
        if ((ad->ad_id & ATH_DIAG_DYN) && outdata != NULL)
                OS_FREE(outdata);

        return error;
}
#endif

#if  ATH_SUPPORT_AOW

static int
ath_ioctl_aow(struct ath_softc_net80211 *scn, struct ath_diag *ad)
{
    void *indata = NULL;
    void *outdata = NULL;
    int error = 0;

    u_int32_t insize = ad->ad_in_size;
    u_int32_t outsize = ad->ad_out_size;
    u_int   id = ad->ad_id & ATH_DIAG_ID;

    if (ad->ad_id & ATH_DIAG_IN) {
        /* copy in data */
        indata = OS_MALLOC(scn->sc_osdev, insize, GFP_KERNEL);
        if (!indata){
            error = -ENOMEM;
            goto bad;
        }
        if (__xcopy_from_user(indata, ad->ad_in_data, insize)) {
            error = -EFAULT;
            goto bad;
        }
    }

    /* in case of outdata, allocate space in the 
     * buffer for out data
     */
    if (ad->ad_id & ATH_DIAG_DYN) {
        outdata = OS_MALLOC(scn->sc_osdev, outsize, GFP_KERNEL);
        if (!outdata) {
            error = -ENOMEM;
            goto bad;
        }
    }        

    switch(id) {
        case IOCTL_AOW_DATA:
           {
              
              u_int64_t tsf = 0;
              u_int32_t seqno = 0;
              if (insize > MAX_AOW_PKT_SIZE) {
                  error = -EINVAL;
                  break;
              }                

              tsf = scn->sc_ops->ath_get_tsf64(scn->sc_dev);
              seqno = scn->sc_ops->ath_get_aow_seqno(scn->sc_dev);

              error = scn->sc_ops->ath_aow_control(scn->sc_dev, id, indata, insize, outdata, &outsize, seqno, tsf);
           }
           break;
        
        case IOCTL_AOW_CTRL:
           {
              u_int64_t tsf = 0;
              
              if (insize > MAX_AOW_PKT_SIZE) {
                  error = -EINVAL;
                  break;
              }                

              tsf = scn->sc_ops->ath_get_tsf64(scn->sc_dev);

              error = scn->sc_ops->ath_aow_tx_ctrl(scn->sc_dev, indata, insize, tsf);
           }
           break;

        default:
           break;
    }            

bad:
    if ((ad->ad_id & ATH_DIAG_IN) && indata != NULL)
        OS_FREE(indata);
    
    if ((ad->ad_id & ATH_DIAG_DYN) && outdata != NULL)
        OS_FREE(outdata);

    return error;


}


#endif  /* ATH_SUPPORT_AOW */

extern unsigned long ath_ioctl_debug;           /* defined in ah_osdep.c  */

struct ioctl_name_tbl {
    unsigned int ioctl;
    char *name;
};

static struct ioctl_name_tbl ioctl_names[] =
{
    {SIOCGATHSTATS, "SIOCGATHSTATS"},
    {SIOCGATHSTATSCLR,"SIOCGATHSTATSCLR"},
    {SIOCGATHPHYSTATS, "SIOCGATHPHYSTATS"},
    {SIOCGATHPHYSTATSCUR, "SIOCGATHPHYSTATSCUR"},
    {SIOCGATHDIAG, "SIOCGATHDIAG"},
#if defined(ATH_SUPPORT_DFS) || defined(ATH_SUPPORT_SPECTRAL)
    {SIOCGATHPHYERR,"SIOCGATHPHYERR"},
#endif
    {SIOCETHTOOL,"SIOCETHTOOL"},
    {SIOC80211IFCREATE, "SIOC80211IFCREATE"},
#ifdef ATH_TX99_DIAG
    {SIOCIOCTLTX99, "SIOCIOCTLTX99"},
#endif
    {0, NULL},
};

static char *find_ath_std_ioctl_name(int param)
{
    int i = 0;
    for (i = 0; ioctl_names[i].ioctl; i++) {
        if (ioctl_names[i].ioctl == param)
            return(ioctl_names[i].name ? ioctl_names[i].name : "UNNAMED");
    }
    return("Unknown IOCTL");
}


static int
ath_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
    struct ath_softc_net80211 *scn = ath_netdev_priv(dev);
    struct ieee80211com *ic = &scn->sc_ic;
    enum ieee80211_phymode mode;
    struct ath_stats *as;
    struct ath_stats_container *asc = NULL;
    struct ath_phy_stats_container *apsc = NULL;
    struct ath_phy_stats *ps = NULL;
    struct ath_chan_stats chanstats;
    struct ath_mib_mac_stats mibstats;
    WIRELESS_MODE wmode;
    int error=0;

#ifdef ATH_SUPPORT_HTC
    if ((!scn) ||
        (scn && scn->sc_htc_delete_in_progress)){
        printk("%s : #### delete is in progress, scn %p \n", __func__, scn);
        return -EINVAL;
    }
#endif

    if (ath_ioctl_debug) {
        printk("***%s:  dev=%s  ioctl=0%04X  name=%s\n", __func__, dev->name, cmd, find_ath_std_ioctl_name(cmd));
    }

    switch (cmd) {
    case SIOCGATHEACS:
#if ATH_SUPPORT_SPECTRAL
        error = osif_ioctl_eacs(dev, ifr, scn->sc_osdev); 
#endif
        break;
    case SIOCGATHSTATS:
        if(((dev->flags & IFF_UP) == 0 ) || ((atomic_read( &ATH_DEV_TO_SC(scn->sc_dev)->sc_nap_vaps_up) <= 0) \
                    && (atomic_read( &ATH_DEV_TO_SC(scn->sc_dev)->sc_nsta_vaps_up) <= 0))){
            return -ENXIO;
        }
        asc = (struct ath_stats_container *)ifr->ifr_data;
        if(asc->size == 0 || asc->address == NULL) {
           error = -EFAULT;
        }else {
            as = scn->sc_ops->get_ath_stats(scn->sc_dev);
            if (ATH_DEV_TO_SC(scn->sc_dev)->sc_nvaps > 0) {
                scn->sc_ops->ath_dev_get_chan_stats(scn->sc_dev, &chanstats);
                as->ast_chan_clr_cnt = chanstats.chan_clr_cnt;
                as->ast_cycle_cnt = chanstats.cycle_cnt;
                as->ast_noise_floor = scn->sc_ops->ath_dev_get_noisefloor(scn->sc_dev);
            } else {
                as->ast_chan_clr_cnt = 0;
                as->ast_cycle_cnt = 0;
                as->ast_noise_floor = 0;
            }
            scn->sc_ops->ath_update_mibMacstats(scn->sc_dev);
            scn->sc_ops->ath_get_mibMacstats(scn->sc_dev, &mibstats);
            memcpy((void *)&(as->ast_mib_stats), (void *)&mibstats,
                                     sizeof(struct ath_mib_mac_stats));
            if (_copy_to_user(asc->address, as,
                         sizeof(struct ath_stats)))
               error = -EFAULT;
            else
               error = 0;
        }
        asc->offload_if = 0;
        asc->size = sizeof(struct ath_stats);
        break;
    case SIOCGATHSTATSCLR:
        as = scn->sc_ops->get_ath_stats(scn->sc_dev);
        memset(as, 0, sizeof(struct ath_stats));
        error = 0;
        break;
    case SIOCGATHPHYSTATS:
        if((( dev->flags & IFF_UP) == 0 ) || ((atomic_read( &ATH_DEV_TO_SC(scn->sc_dev)->sc_nap_vaps_up) <= 0) \
                    && (atomic_read( &ATH_DEV_TO_SC(scn->sc_dev)->sc_nsta_vaps_up) <= 0))){
            return -ENXIO;
        }
        ps = scn->sc_ops->get_phy_stats_allmodes(scn->sc_dev);
        if (_copy_to_user(ifr->ifr_data, ps,
                          sizeof(struct ath_phy_stats) * WIRELESS_MODE_MAX)) {
            error = -EFAULT;
        }
        else {
            error = 0;
        }
        break;
    case SIOCGATHPHYSTATSCUR:
        if(((dev->flags & IFF_UP) == 0) || ((atomic_read( &ATH_DEV_TO_SC(scn->sc_dev)->sc_nap_vaps_up) <= 0) \
                    &&  (atomic_read( &ATH_DEV_TO_SC(scn->sc_dev)->sc_nsta_vaps_up) <= 0))){
            return -ENXIO;
        }
        mode = ieee80211_get_current_phymode(ic);
        wmode = ath_ieee2wmode(mode);

        if (wmode == WIRELESS_MODE_MAX) {
            ASSERT(0);
            return 0;
        }

        apsc = (struct ath_phy_stats_container *)ifr->ifr_data;
        if (apsc->size == 0 || apsc->address == NULL) {
           error = -EFAULT;
        } else {
            ps = scn->sc_ops->get_phy_stats(scn->sc_dev, wmode);
            if (_copy_to_user(apsc->address, ps,
                          sizeof(struct ath_phy_stats))) {
                error = -EFAULT;
            }
            else {
                error = 0;
            }
        }
        break;
    case SIOCGATHDIAG:
        if (!capable(CAP_NET_ADMIN))
            error = -EPERM;
        else
            error = ath_ioctl_diag(scn, (struct ath_diag *) ifr);
        break;
#if defined(ATH_SUPPORT_DFS) || defined(ATH_SUPPORT_SPECTRAL)
    case SIOCGATHPHYERR:
        if (!capable(CAP_NET_ADMIN)) {
            error = -EPERM;
        } else {
            /*
                EV904010 -- Make sure there is a VAP on the interface
                before the request is processed
            */

            if ((ATH_DEV_TO_SC(scn->sc_dev))->sc_nvaps) {
                error = ath_ioctl_phyerr(scn, ifr->ifr_data);
            } else {
                error = -EINVAL;
            }
        }
        break;
#endif
    case SIOCETHTOOL:
        if (__xcopy_from_user(&cmd, ifr->ifr_data, sizeof(cmd)))
            error = -EFAULT;
        else
            error = ath_ioctl_ethtool(scn, cmd, ifr->ifr_data);
        break;
    case SIOC80211IFCREATE:
	{
       	    struct ieee80211_clone_params cp;
            if (scn->sc_in_delete) {
                printk("%s: Can't create VAP, in detach\n", __func__);
                return -ENODEV;
            }
            if (__xcopy_from_user(&cp, ifr->ifr_data, sizeof(cp))) {
       	        return -EFAULT;
            }	
            error = osif_ioctl_create_vap(dev, ifr, cp, scn->sc_osdev);
	}
        break;
#if  ATH_SUPPORT_AOW
    case SIOCGATHAOW:
        error = ath_ioctl_aow(scn, ifr->ifr_data);
        break;
#endif  /* ATH_SUPPORT_AOW */

#ifdef ATH_TX99_DIAG
    case SIOCIOCTLTX99:
        error = tx99_ioctl(dev, ATH_DEV_TO_SC(scn->sc_dev), cmd, ifr->ifr_data);
        break;
#endif
#ifdef ATH_SUPPORT_LINUX_VENDOR
    case SIOCDEVVENDOR:
        error = osif_ioctl_vendor(dev, ifr, 0);
        break;
#endif
#ifdef ATH_BUS_PM
    case SIOCSATHSUSPEND:
        {
            struct ieee80211com *ic = &scn->sc_ic;
            struct ieee80211vap *tmpvap;
            int val = 0;
            if (__xcopy_from_user(&val, ifr->ifr_data, sizeof(int)))
                return -EFAULT;

            if(val) {
              /* suspend only if all vaps are down */
                TAILQ_FOREACH(tmpvap, &ic->ic_vaps, iv_next) {
                    struct net_device *tmpdev = ((osif_dev *)tmpvap->iv_ifp)->netdev;
                    if (tmpdev->flags & IFF_RUNNING)
                        return -EBUSY;
                }
                error = bus_device_suspend(scn->sc_osdev);
                if (!error)
                    scn->sc_osdev->isDeviceAsleep = val;
            }
            else {
                int isDeviceAsleepSave = scn->sc_osdev->isDeviceAsleep;
                scn->sc_osdev->isDeviceAsleep = val;
                error = bus_device_resume(scn->sc_osdev);
                if (error)
                    scn->sc_osdev->isDeviceAsleep = isDeviceAsleepSave;
            }
        }
      break;
#endif /* ATH_BUS_PM */
	case SIOCG80211PROFILE:
        {
            struct ieee80211_profile *profile;

            profile = (struct ieee80211_profile *)kmalloc(
                            sizeof (struct ieee80211_profile), GFP_KERNEL);
            if (profile == NULL) {
                break;
            }
            OS_MEMSET(profile, 0, sizeof (struct ieee80211_profile));
            error = osif_ioctl_get_vap_info(dev, profile);
            error = _copy_to_user(ifr->ifr_data, profile,
                            sizeof (struct ieee80211_profile));
            if (profile != NULL) {
                kfree(profile);
                profile = NULL;
            }
        }
        break;
    case SIOCGSETCTLPOW:
        {
            u_int8_t ctl[1024];
            u_int32_t i = 0;
            __xcopy_from_user(ctl, ifr->ifr_data, sizeof(ctl));
            printk("ctl ioctl :\n");
            for (i = 0; i < 1024; i++) {
                if ((i % 16) == 15) {
                    printk("\n");
                }
                printk(" %02X", ctl[i]);
            }
            printk("\n");
                
            scn->sc_ops->set_ctl_pwr(scn->sc_dev, ctl + 4, *(u_int32_t *)ctl, 0);
            
        }
        break;

#if UMAC_SUPPORT_ACFG
	 case ACFG_PVT_IOCTL:
        error = acfg_handle_ioctl(dev, ifr->ifr_data);
        break;
#endif

    default:
        error = -EINVAL;
        break;
    }
	return error;
}

/*
 * Return netdevice statistics.
 */
static struct net_device_stats *
ath_getstats(struct net_device *dev)
{
    struct ath_softc_net80211 *scn = ath_netdev_priv(dev);
    struct net_device_stats *stats;
    struct ath_stats *as;
    struct ath_phy_stats *ps;
    struct ath_11n_stats *ans;
    struct ath_dfs_stats *dfss;
    WIRELESS_MODE wmode;

#ifdef ATH_SUPPORT_HTC
    if ((!scn) ||
        (scn && scn->sc_htc_delete_in_progress)){
        printk("%s : #### delete is in progress, scn %p \n", __func__, scn);

        /* Kernel expect return net_device_stats if *dev is not NULL. */
        if (scn)
            return (&scn->sc_osdev->devstats);
        else
            return NULL;
    }
#endif


    stats = &scn->sc_osdev->devstats;
    as = scn->sc_ops->get_ath_stats(scn->sc_dev);
    ans = scn->sc_ops->get_11n_stats(scn->sc_dev);
    dfss = scn->sc_ops->get_dfs_stats(scn->sc_dev);
    /* update according to private statistics */
    stats->tx_errors = as->ast_tx_xretries
             + as->ast_tx_fifoerr
             + as->ast_tx_filtered
             ;
    stats->tx_dropped = as->ast_tx_nobuf
            + as->ast_tx_encap
            + as->ast_tx_nonode
            + as->ast_tx_nobufmgt;
    /* Add tx beacons, tx mgmt, tx, 11n tx */
    stats->tx_packets = as->ast_be_xmit
            + as->ast_tx_mgmt
            + as->ast_tx_packets
            + ans->tx_pkts;
    /* Add rx, 11n rx (rx mgmt is included) */
    stats->rx_packets = as->ast_rx_packets
            + ans->rx_pkts;

    for (wmode = 0; wmode < WIRELESS_MODE_MAX; wmode++) {
        ps = scn->sc_ops->get_phy_stats(scn->sc_dev, wmode);
        
        stats->rx_errors = ps->ast_rx_fifoerr;
        stats->rx_dropped = ps->ast_rx_tooshort;
        stats->rx_crc_errors = ps->ast_rx_crcerr;
    }
    
    return stats;
}

#ifndef ATH_SUPPORT_HTC
static void
ath_tasklet(TQUEUE_ARG data)
{
    struct net_device *dev = (struct net_device *)data;
    struct ath_softc_net80211 *scn = ath_netdev_priv(dev);
#ifdef ATH_SUPPORT_MSI    
    if (bus_msi_enabled(scn->sc_osdev)) {
        spin_lock_bh(&scn->msi_poll_lock_bh);
        do_ath_handle_intr(scn->sc_dev);
        spin_unlock_bh(&scn->msi_poll_lock_bh);
    }
    else
#endif
    do_ath_handle_intr(scn->sc_dev);
}
#endif



#ifndef ATH_SUPPORT_HTC
irqreturn_t
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19)
ath_isr_generic(int irq, void *dev_id)
#else
ath_isr_generic(int irq, void *dev_id, struct pt_regs *regs)
#endif
{
    struct net_device *dev = dev_id;
    struct ath_softc_net80211 *scn = ath_netdev_priv(dev);
    int sched, needmark = 0;

#if PCI_INTERRUPT_WAR_ENABLE
    scn->int_scheduled_cnt++;
#endif

#ifdef ATH_SUPPORT_MSI

    /* always acknowledge the interrupt */
    if (bus_msi_enabled(scn->sc_osdev)) {
       spin_lock(&scn->msi_poll_lock);
       sched = scn->sc_ops->msisr(scn->sc_dev,0);
       spin_unlock(&scn->msi_poll_lock);
    }
    else
#endif
    sched = scn->sc_ops->isr(scn->sc_dev);
    
    switch(sched)
    {
    case ATH_ISR_NOSCHED:
        return  IRQ_HANDLED;
        
    case ATH_ISR_NOTMINE:
        return IRQ_NONE;
        
    default:
        if ((dev->flags & (IFF_RUNNING|IFF_UP)) != (IFF_RUNNING|IFF_UP))
        {
            DPRINTF_INTSAFE(scn, ATH_DEBUG_INTR, "%s: flags 0x%x\n", __func__, dev->flags);
            scn->sc_ops->disable_interrupt(scn->sc_dev);     /* disable further intr's */
            return IRQ_HANDLED;
        }
    }
    
    /*
    ** See if the transmit queue processing needs to be scheduled
    */
    
    ATH_SCHEDULE_TQUEUE(&scn->sc_osdev->intr_tq, &needmark);
    if (needmark)
        mark_bh(IMMEDIATE_BH);

    return IRQ_HANDLED;
}
#endif
/*
Interrupt could missing on some 3rd party platform,
and cause stop beaconning, or loading FW fail.
This WAR would detect interrupt count periodically,
and do a chip reset when interrupt missing detected.
*/
#if PCI_INTERRUPT_WAR_ENABLE
static
OS_TIMER_FUNC(ath_int_status_poll_timer)
{
    struct net_device *dev;
    struct ath_softc_net80211 *scn;
    struct ath_softc *sc;
    OS_GET_TIMER_ARG(dev, struct net_device *);
    scn = ath_netdev_priv(dev);
    sc = ATH_DEV_TO_SC(scn->sc_dev);

    if (!sc->sc_nvaps || sc->sc_invalid || sc->sc_scanning || sc->sc_dfs_wait) {
        scn->pre_int_scheduled_cnt = 0;
        OS_SET_TIMER(&scn->int_status_poll_timer, scn->int_status_poll_interval);
        return;     
    }

    /* check whether internal reset is necessary to recover from wifi interrupt stop */
    if (scn->pre_int_scheduled_cnt !=0 && (scn->pre_int_scheduled_cnt == scn->int_scheduled_cnt)) {
        ath_bstuck_tasklet(sc);
                ATH_CLEAR_HANGS(sc);
        printk("%s: issue internal reset to recover wifi interrupt stop, count=%d\n",__func__,scn->int_scheduled_cnt);
    }

    scn->pre_int_scheduled_cnt = scn->int_scheduled_cnt;

    OS_SET_TIMER(&scn->int_status_poll_timer, scn->int_status_poll_interval);
    return;
}
#endif


#ifndef ATH_SUPPORT_HTC
irqreturn_t
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19)
ath_isr(int irq, void *dev_id)
{
    do_ath_isr(irq,dev_id);
}
#else
ath_isr(int irq, void *dev_id, struct pt_regs *regs)
{
    do_ath_isr(irq,dev_id,regs);
}
#endif
#endif


#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,30)
static const struct net_device_ops athdev_net_ops = {
    .ndo_open    = ath_netdev_open,
    .ndo_stop    = ath_netdev_stop,
    .ndo_start_xmit = ath_netdev_hardstart,
    .ndo_set_mac_address = ath_netdev_set_macaddr,
    .ndo_tx_timeout = ath_netdev_tx_timeout,
    .ndo_get_stats = ath_getstats,
    .ndo_change_mtu = ath_change_mtu,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,2,0)
    .ndo_set_multicast_list = ath_netdev_set_mcast_list,
#else
    .ndo_set_rx_mode = ath_netdev_set_mcast_list,
#endif
    .ndo_do_ioctl = ath_ioctl,
};
#endif

#if ATH_SUPPORT_FLOWMAC_MODULE
int8_t
ath_flowmac_pause_wifi (struct net_device *dev, u_int8_t pause, u_int32_t duration)
{
    return 0;
}

int8_t
ath_flowmac_ratelimit_wifi(struct net_device *dev, flow_dir_t dir,
                void *rate)
{
    return 0;
}

int
ath_flowmac_notify_wifi(struct net_device *dev, flowmac_event_t event,
                void *event_data)
{
    struct ath_softc_net80211 *scn = ath_netdev_priv(dev);
    struct ath_softc *sc = ATH_DEV_TO_SC(scn->sc_dev);

    if (event == FLOWMAC_ENABLE) {
        printk("%s Flowmac is enabled\n",__func__);
        sc->sc_osnetif_flowcntrl = 1;
    } else if (event == FLOWMAC_DISABLE)  {
        printk("%s flowmac is disabled \n", __func__);
        sc->sc_osnetif_flowcntrl = 0;
    }
    sc->sc_ieee_ops->notify_flowmac_state(sc->sc_ieee, sc->sc_osnetif_flowcntrl);
    return 0;
}

int
ath_flowmac_attach(ath_dev_t dev)
{
    flow_ops_t ops;
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);

    if (!dev) {
        printk("%s dev pointer is wrong \n", __func__);
        return -EINVAL;
    }
    /* regiser with the flow mac driver */
    ops.pause = ath_flowmac_pause_wifi;
    ops.rate_limit  = ath_flowmac_ratelimit_wifi;
    ops.notify_event = ath_flowmac_notify_wifi;

    sc->sc_flowmac_dev = flowmac_register(sc->sc_osdev->netdev, &ops, 0, 0, 0, 0);
    if (sc->sc_flowmac_dev) {
        printk("%s flowmac registration is successful %p\n",
                __func__, sc->sc_flowmac_dev);
        return 0;
    }
    return -EINVAL;
    
}
EXPORT_SYMBOL(ath_flowmac_attach);
int
ath_flowmac_detach(ath_dev_t dev)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);

    if (!dev) {
        printk("%s dev pointer is wrong \n", __func__);
        return -EINVAL;
    }
    if (sc->sc_flowmac_dev) {
        flowmac_unregister(sc->sc_flowmac_dev);
        sc->sc_flowmac_dev = NULL;
    }
    return 0;
}
#endif
#ifdef ATH_SUPPORT_MSI
static
OS_TIMER_FUNC(ath_msi_poll_timer)
{
    struct ath_softc_net80211 *scn;
    int sched;
    unsigned long flags;
    struct net_device *dev;

    OS_GET_TIMER_ARG(dev, struct net_device *);

    scn = ath_netdev_priv(dev);

    do {
        spin_lock_irqsave(&scn->msi_poll_lock, flags);
#if UMAC_SUPPORT_APONLY
        if(likely(irq_run_aponly((void *)dev)))
            sched = ath_intr_aponly(scn->sc_dev);
        else
#endif
        sched = scn->sc_ops->msisr(scn->sc_dev, 0);
        spin_unlock_irqrestore(&scn->msi_poll_lock, flags);

        if (sched == ATH_ISR_SCHED)
        {
            spin_lock_bh(&scn->msi_poll_lock_bh);
            do_ath_handle_intr(scn->sc_dev);
            spin_unlock_bh(&scn->msi_poll_lock_bh);            
        }
    } while (FALSE);

    OS_SET_TIMER(&scn->msi_poll_timer, scn->msi_poll_interval);
    return;
}
#endif

#ifdef CONFIG_COMCERTO_CUSTOM_SKB_LAYOUT
/** ath_show_custom_skb_enable
 *
 */
static ssize_t ath_show_custom_skb_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
       struct ath_softc_net80211 *scn = ath_netdev_priv(to_net_dev(dev));

       return  sprintf(buf, "\n%d\n", scn->custom_skb_enabled);
}

/** ath_set_custom_skb_enable
 *
 */
static ssize_t ath_set_custom_skb_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
       struct ath_softc_net80211 *scn = ath_netdev_priv(to_net_dev(dev));
       struct net_device *netdev = to_net_dev(dev);
       int enable;

       enable = (int)simple_strtol(buf, NULL, 10);

       enable = enable ? 1 : 0;

       if (enable)
           printk(KERN_INFO "%s: (%s) Enabled custom skb feature in WiFi=>LAN/WAN path\n", __func__, netdev->name);
       else
           printk(KERN_INFO "%s: (%s) Disabled custom skb feature in WiFi=>LAN/WAN path\n", __func__, netdev->name);

       scn->custom_skb_enabled = enable;

       return count;
}
static DEVICE_ATTR(custom_skb_enable, 0644, ath_show_custom_skb_enable, ath_set_custom_skb_enable);
#endif


int
__ath_attach(u_int16_t devid, struct net_device *dev, HAL_BUS_CONTEXT *bus_context, osdev_t osdev)
{
    struct ath_softc_net80211 *scn = ath_netdev_priv(dev);
    struct ieee80211com *ic = &scn->sc_ic;
    int error = 0;
    struct hal_reg_parm hal_conf_parm;
    unsigned int irq_type = IRQF_SHARED;
    hal_conf_parm.calInFlash = bus_context->bc_info.cal_in_flash;

    printk("%s: Set global_scn[%d]\n", __func__, num_global_scn);
    global_scn[num_global_scn++] = scn;
#if ATH_DEBUG
    ath_params.ACBKMinfree = ath_params.ACBKMinfree == ACBKMinfree ? ath_params.ACBKMinfree : ACBKMinfree;
    ath_params.ACBEMinfree = ath_params.ACBEMinfree == ACBEMinfree ? ath_params.ACBEMinfree : ACBEMinfree;
    ath_params.ACVIMinfree = ath_params.ACVIMinfree == ACVIMinfree ? ath_params.ACVIMinfree : ACVIMinfree;
    ath_params.ACVOMinfree = ath_params.ACVOMinfree == ACVOMinfree ? ath_params.ACVOMinfree : ACVOMinfree;
    ath_params.CABMinfree = ath_params.CABMinfree == CABMinfree ? ath_params.CABMinfree : CABMinfree;
    ath_params.UAPSDMinfree = ath_params.UAPSDMinfree == UAPSDMinfree ? ath_params.UAPSDMinfree : UAPSDMinfree;
    printk("*** All the minfree values should be <= ATH_TXBUF-32, otherwise default value will be used instead ***\n");
    printk("ACBKMinfree = %d\n", ath_params.ACBKMinfree);
    printk("ACBEMinfree = %d\n", ath_params.ACBEMinfree);
    printk("ACVIMinfree = %d\n", ath_params.ACVIMinfree);
    printk("ACVOMinfree = %d\n", ath_params.ACVOMinfree);
    printk("CABMinfree = %d\n", ath_params.CABMinfree);
    printk("UAPSDMinfree = %d\n", ath_params.UAPSDMinfree);
    printk("ATH_TXBUF=%d\n", ATH_TXBUF);
#endif

    /* show that no dedicated amem instance has been created yet */
    scn->amem.handle = NULL;
    /*
     * create and initialize ath layer
     */
#if ATH_SUPPORT_FLOWMAC_MODULE
    ath_params.osnetif_flowcntrl = 0; /* keep it disabled by default */
    ath_params.os_ethflowmac_enable = 0; /* keep it disabled by default */
#endif

#ifdef QCA_PARTNER_PLATFORM
    ath_pltfrm_init( dev );
#endif
	/*
	 * Set bus context to osdev
	 */
	osdev->bc = *bus_context;
    error = ath_attach(devid, bus_context, scn, osdev, &ath_params, &hal_conf_parm, &wlan_reg_params);

#ifndef REMOVE_PKT_LOG
    scn->pl_dev->name = dev->name; 
#endif
    if (error != 0)
        goto bad;

#if ATH_BAND_STEERING
    if(EOK != ath_band_steering_netlink_init()) {
        printk("Band steering socket init failed __investigate__\n");
    }
#endif
#if ATH_SUPPORT_IBSS_NETLINK_NOTIFICATION
    ath_adhoc_netlink_init();
#endif
#if ATH_RXBUF_RECYCLE
    ath_rxbuf_recycle_init(osdev);
#endif    

    ald_init_netlink();

#if UMAC_SUPPORT_ACFG 
    acfg_event_netlink_init();
#endif

    osif_attach(dev);
    /* For STA Mode default CWM mode is Auto */	
    if ( ic->ic_opmode == IEEE80211_M_STA)
        ic->ic_cwm_set_mode(ic, IEEE80211_CWM_MODE2040);

#if 0 /* TBD: is it taken care in UMAC ? */
    /*
     * This will be used in linux client while processing 
     * country ie in 11d beacons
     */
    ic->ic_ignore_11dbeacon = 0;

    /*
     * commommode is used while determining the channel power
     * in standard client mode
     */ 
    ic->ic_commonmode = ic->ic_country.isMultidomain; 

#endif
	
    /*
     * initialize tx/rx engine
     */
    error = scn->sc_ops->tx_init(scn->sc_dev, ATH_TXBUF);
    if (error != 0)
        goto bad1;

    error = scn->sc_ops->rx_init(scn->sc_dev, ATH_RXBUF);
    if (error != 0)
        goto bad2;

    /*
     * setup net device
     */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,30)
    dev->netdev_ops = &athdev_net_ops;
#if ATH_PERF_PWR_OFFLOAD
    ((struct net_device_ops *) (dev->netdev_ops))->ndo_do_ioctl = ath_ioctl;
#endif
#else
    dev->open = ath_netdev_open;
    dev->stop = ath_netdev_stop;
    dev->hard_start_xmit = ath_netdev_hardstart;
    dev->set_mac_address = ath_netdev_set_macaddr;
    dev->tx_timeout = ath_netdev_tx_timeout;
    dev->set_multicast_list = ath_netdev_set_mcast_list;
    dev->do_ioctl = ath_ioctl;
    dev->get_stats = ath_getstats;
    dev->change_mtu = ath_change_mtu;
#endif
    dev->watchdog_timeo = 5 * HZ;           /* XXX */
    dev->tx_queue_len = ATH_TXBUF-1;        /* 1 for mgmt frame */
#ifdef USE_HEADERLEN_RESV
    dev->hard_header_len += sizeof (struct ieee80211_qosframe) + sizeof(struct llc) + IEEE80211_ADDR_LEN + IEEE80211_WEP_IVLEN + IEEE80211_WEP_KIDLEN;
#ifdef ATH_SUPPORT_HTC
    dev->hard_header_len += ATH_HTC_HDRSPACE + ATH_HTC_RESERVED + ATH_HTC_TX_FRAME_HDRSPACE;
#endif
#endif

    /*
    ** Attach the iwpriv handlers
    */
    
    ath_iw_attach(dev);

    /*
     * setup interrupt serivce routine
     */
     
#ifndef ATH_SUPPORT_HTC
#ifdef ADF_SUPPORT
    ATH_INIT_TQUEUE(&osdev->intr_tq, (adf_os_defer_fn_t)ath_tasklet, (void*)dev);
#else
   ATH_INIT_TQUEUE(&osdev->intr_tq, ath_tasklet, dev);
#endif
#endif

#if OS_SUPPORT_ASYNC_Q
   OS_MESGQ_INIT(osdev, &osdev->async_q, sizeof(os_async_q_mesg),
        OS_ASYNC_Q_MAX_MESGS,os_async_mesg_handler,  osdev,MESGQ_PRIORITY_NORMAL,MESGQ_ASYNCHRONOUS_EVENT_DELIVERY);      
#endif

	/*
     * Resolving name to avoid a crash in request_irq() on new kernels
     */ 
    
    dev_alloc_name(dev, dev->name);
 
#ifdef ATH_SUPPORT_HTC
    /*
     * do not register PCI IRQ in USB mode
     */
    if (0) {
        goto bad3;
    }
#else
#ifdef QCA_PARTNER_PLATFORM
    error = osif_pltfrm_interupt_register(dev,DIRECT_TYPE);
#else
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
    error = request_irq(dev->irq, ath_isr, SA_SHIRQ | SA_SAMPLE_RANDOM, dev->name, dev);
#else

#ifdef ATH_AHB
    error = request_irq(dev->irq, ath_isr, IRQF_DISABLED, dev->name, dev);
#else
    
#ifdef ATH_SUPPORT_MSI
    if (bus_msi_enabled(scn->sc_osdev)) {
       irq_type = 0;
    }
#endif
    printk("request_irq type = 0x%x\n", irq_type);
    error = request_irq(dev->irq, ath_isr, irq_type, dev->name, dev);
#endif
#endif
    
#endif
    if (error)
    {
        printk(KERN_WARNING "%s: request_irq failed\n", dev->name);
        error = -EIO;
        goto bad3;
    }
#ifdef ATH_SUPPORT_MSI
    if (bus_msi_enabled(scn->sc_osdev)) {
        printk(KERN_INFO "%s : MSI WA timer is enabled\n", __func__);
        scn->msi_poll_interval = 200; //ms
        OS_INIT_TIMER(scn->sc_osdev, &scn->msi_poll_timer, ath_msi_poll_timer, dev);
        spin_lock_init(&scn->msi_poll_lock);
        spin_lock_init(&scn->msi_poll_lock_bh);

       /* FIXME:
        * ath_resume will fire it. In case there is no vap, still the timer is fired.
        * See if we need interrupt, even before VAP is created. If no, we can start
        * timer only when vap is created.
         * OS_SET_TIMER(&scn->msi_poll_timer, scn->msi_poll_interval);
        */
    }
#endif

#if PCI_INTERRUPT_WAR_ENABLE
    {
        printk(KERN_INFO "%s : Interrupt WA timer is enabled\n", __func__);
        scn->int_status_poll_interval = 200; //ms
        OS_INIT_TIMER(scn->sc_osdev, &scn->int_status_poll_timer, ath_int_status_poll_timer, dev);
        OS_SET_TIMER(&scn->int_status_poll_timer, 1000); /* start polling after 1 sec */
    }   
#endif    
#endif
    /* Kernel 2.6.25 needs valid dev_addr before  register_netdev */
    IEEE80211_ADDR_COPY(dev->dev_addr,ic->ic_myaddr);

    /*
     * finally register netdev and ready to go
     */
    if ((error = register_netdev(dev)) != 0) {
        printk(KERN_ERR "%s: unable to register device\n", dev->name);
        goto bad4;
    }
#if !NO_SIMPLE_CONFIG
    /* Request Simple Config intr handler */
    register_simple_config_callback (dev->name, (void *) jumpstart_intr, (void *) dev,
                                     (void *)&osdev->sc_push_button_dur);
#endif
    sysfs_create_group(&dev->dev.kobj, &wifi_attr_group);

#ifdef ATH_SUPPORT_LINUX_STA
#ifdef CONFIG_SYSCTL
    ath_dynamic_sysctl_register(ATH_DEV_TO_SC(scn->sc_dev));
#endif
#endif

#if ATH_SUPPORT_FLOWMAC_MODULE
    ath_flowmac_attach(scn->sc_dev);
#endif
#ifdef CONFIG_COMCERTO_CUSTOM_SKB_LAYOUT
    /* Enable custom skb, by default */
    scn->custom_skb_enabled = 1;
    device_create_file(&dev->dev, &dev_attr_custom_skb_enable);
#endif
/* begin: add by chenzejun for mac scan by 2015.12.4 */
    AP_Netlink_init();
/* end: add by chenzejun for mac scan by 2015.12.4 */

    return 0;
    
bad4:
#ifdef QCA_PARTNER_PLATFORM	
    osif_pltfrm_interupt_unregister(dev,0);
#else	
    free_irq(dev->irq, dev);
#endif
bad3:
    scn->sc_ops->rx_cleanup(scn->sc_dev);
bad2:
    scn->sc_ops->tx_cleanup(scn->sc_dev);
bad1:
    ath_detach(scn);
bad:
    return error;
}

int
__ath_detach(struct net_device *dev)
{
    struct ath_softc_net80211 *scn = ath_netdev_priv(dev);
    int ret;

    /* == Fixed crash removing umac.ko while downloading target firmware */
    if (!scn->sc_dev) {
        return -EINVAL;
    }
    rtnl_lock();
/* begin: add by chenzejun for mac scan by 2015.12.4 */
    AP_Netlink_destroy();
/* end: add by chenzejun for mac scan by 2015.12.4 */
#ifdef CONFIG_COMCERTO_CUSTOM_SKB_LAYOUT
    device_remove_file(&dev->dev, &dev_attr_custom_skb_enable);
#endif
    ath_vap_delete_on_rmmod(scn);
    
    osif_detach(dev);
#ifndef ATH_SUPPORT_HTC
    if (dev->irq)
    {
#ifdef QCA_PARTNER_PLATFORM	
        osif_pltfrm_interupt_unregister(dev,0);
#else	
        free_irq(dev->irq, dev);
#endif
    }		
#endif

#ifdef ATH_SUPPORT_LINUX_STA
#ifdef CONFIG_SYSCTL
    ath_dynamic_sysctl_unregister(ATH_DEV_TO_SC(scn->sc_dev));
#endif
#endif

#ifndef NO_SIMPLE_CONFIG
    unregister_simple_config_callback(dev->name);
#endif
    sysfs_remove_group(&dev->dev.kobj, &wifi_attr_group);
    unregister_netdevice(dev);

#if OS_SUPPORT_ASYNC_Q
   OS_MESGQ_DRAIN(&scn->sc_osdev->async_q,NULL);
   OS_MESGQ_DESTROY(&scn->sc_osdev->async_q);
#endif
    scn->sc_ops->rx_cleanup(scn->sc_dev);
    scn->sc_ops->tx_cleanup(scn->sc_dev);
#ifdef ATH_SUPPORT_MSI
    if (bus_msi_enabled(scn->sc_osdev)) {
       OS_CANCEL_TIMER(&scn->msi_poll_timer);
    }  
#endif

#if PCI_INTERRUPT_WAR_ENABLE
    OS_CANCEL_TIMER(&scn->int_status_poll_timer);
    OS_FREE_TIMER(&scn->int_status_poll_timer);
#endif 

#if ATH_SUPPORT_IBSS_NETLINK_NOTIFICATION
    ath_adhoc_netlink_delete();
#endif
#if ATH_BAND_STEERING
    ath_band_steering_netlink_delete();
#endif

#if UMAC_SUPPORT_ACFG
    acfg_event_netlink_delete();
#endif

#if ATH_RXBUF_RECYCLE
    ath_rxbuf_recycle_destroy(scn->sc_osdev);
#endif /* ATH_RXBUF_RECYCLE */

    ald_destroy_netlink();

#if ATH_SUPPORT_FLOWMAC_MODULE
    ath_flowmac_detach(scn->sc_dev);
#endif
    ret = ath_detach(scn);
    rtnl_unlock();
    return ret; 
}

int
__ath_remove_interface(struct net_device *dev)
{
    struct ath_softc_net80211 *scn = ath_netdev_priv(dev);
    struct ieee80211com *ic = &scn->sc_ic;
    struct ieee80211vap *vap;
    struct ieee80211vap *vapnext;
    osif_dev  *osifp;
    struct net_device *netdev;

    vap = TAILQ_FIRST(&ic->ic_vaps);
    while (vap != NULL) {
        /* osif_ioctl_delete_vap() destroy vap->iv_next information,
        so need to store next VAP address in vapnext */
        vapnext = TAILQ_NEXT(vap, iv_next);
        osifp = (osif_dev *)vap->iv_ifp;
        netdev = osifp->netdev;
        printk("Remove interface on %s\n",netdev->name);
        rtnl_lock();
        dev_close(netdev);
        osif_ioctl_delete_vap(netdev);
        rtnl_unlock();
        vap = vapnext;
    }    
    return 0;
}

void
__ath_set_delete(struct net_device *dev)
{
#ifdef ATH_SUPPORT_HTC
    struct ath_softc_net80211 *scn = ath_netdev_priv(dev);
    struct ieee80211com *ic = &scn->sc_ic;

    ic->ic_delete_in_progress = 
    scn->sc_htc_delete_in_progress = 1;
#endif    
}

int
__ath_suspend(struct net_device *dev)
{
    return ath_netdev_stop(dev);
}

int
__ath_resume(struct net_device *dev)
{
    return ath_netdev_open(dev);
}
#if !NO_SIMPLE_CONFIG
/*
 * Handler for front panel SW jumpstart switch
 */
static irqreturn_t
jumpstart_intr (int cpl, void *dev_id, struct pt_regs *regs, void *push_time)
{
    struct net_device *dev = dev_id;
    struct ath_softc_net80211 *scn = ath_netdev_priv(dev);
    struct ieee80211com *ic = &scn->sc_ic;
    struct ieee80211vap *vap;
    u_int32_t           push_duration;
    int is_ap_vap_notified = 0;
    /*
    ** Iterate through all VAPs, since any of them may have WPS enabled
    */

    vap = TAILQ_FIRST(&ic->ic_vaps);
    while (vap != NULL) {
        if (push_time) {
            push_duration = *(u_int32_t *)push_time;
        } else {
            push_duration = 0;
        }
#if ATH_SUPPORT_HYFI_ENHANCEMENTS
        if (!ieee80211_vap_nopbn_is_set(vap))
        {
#endif
            /* Since we are having single physical push button on device , 
               and for push button + muti bss combination mode we will be have limitations, 
               we designed that,physical push button notification will be sent to 
               first AP vap(main BSS) and all sta vaps.*/
            if( vap->iv_opmode != IEEE80211_M_HOSTAP || is_ap_vap_notified == 0 ){ 
                printk("SC Pushbutton Notify on %s for %d sec(s) and the vap %p dev %p:\n",dev->name,
                        push_duration, vap, (struct net_device *)(((osif_dev *)vap->iv_ifp)->netdev));
                osif_notify_push_button ((struct net_device *)(((osif_dev *)vap->iv_ifp)->netdev), push_duration);
                if(vap->iv_opmode == IEEE80211_M_HOSTAP)
                    is_ap_vap_notified = 1;
            }
#if ATH_SUPPORT_HYFI_ENHANCEMENTS
        }
#endif
        vap = TAILQ_NEXT(vap, iv_next);
    }
    return IRQ_HANDLED;
}
#endif

#if OS_SUPPORT_ASYNC_Q
static void os_async_mesg_handler( void  *ctx, u_int16_t  mesg_type, u_int16_t  mesg_len, void  *mesg )
{
    if (mesg_type == OS_SCHEDULE_ROUTING_MESG_TYPE) {
        os_schedule_routing_mesg  *s_mesg = (os_schedule_routing_mesg *) mesg;
        s_mesg->routine(s_mesg->context, NULL);
    }
}
#endif

void
init_wlan(void)
{
   
}

#ifdef ATH_USB

#define IS_UP(_dev) \
    (((_dev)->flags & (IFF_RUNNING|IFF_UP)) == (IFF_RUNNING|IFF_UP))

static int ath_reinit_interface(struct net_device *dev)
{
    struct ath_softc_net80211 *scn = ath_netdev_priv(dev);
    struct ieee80211com *ic = &scn->sc_ic;
    struct ieee80211vap *vap;
    struct ieee80211vap *vapnext;
    osif_dev  *osifp;
    struct net_device *netdev;

    vap = TAILQ_FIRST(&ic->ic_vaps);
    while (vap != NULL) {
        vapnext = TAILQ_NEXT(vap, iv_next);
        osifp = (osif_dev *)vap->iv_ifp;
        netdev = osifp->netdev;
        rtnl_lock();
        if (IS_UP(netdev)) {
            dev_close(netdev);
            msleep(50);
            dev_open(netdev);
        }
        rtnl_unlock();
        vap = vapnext;
    }

    return 0;
}

void ath_usb_vap_restart(struct net_device *dev)
{
    struct ath_softc_net80211 *scn = ath_netdev_priv(dev);

    cmpxchg((int*)(&(scn->sc_dev_enabled)), 1, 0);
    ath_netdev_open(dev);
    ath_reinit_interface(dev);
}

EXPORT_SYMBOL(__ath_detach);
EXPORT_SYMBOL(init_wlan);
EXPORT_SYMBOL(__ath_remove_interface);
EXPORT_SYMBOL(__ath_attach);
EXPORT_SYMBOL(ath_usb_vap_restart);
EXPORT_SYMBOL(__ath_set_delete);
#endif

#ifndef REMOVE_PKT_LOG

struct ath_softc * 
ath_get_softc(struct net_device *dev)                                         
{           
    struct ath_softc_net80211 *scn = ath_netdev_priv(dev);                    
    return (struct ath_softc *)scn->sc_dev;                                   
}    
EXPORT_SYMBOL(ath_get_softc);

#endif
