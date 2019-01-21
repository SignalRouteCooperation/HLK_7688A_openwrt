/*
 * Copyright (c) 2006-2007 Atheros Communications, Inc.
 * Copyright (c) 2010, Atheros Communications Inc.
 * All rights reserved.
 *
 */
/*
 * ath_tx99: continuous transmit diagnostic utility.
 *
 * To use, build the module.  Load the driver as usual followed by
 * the ath_tx99 module.  On module load specify the base device to
 * use for testing; e.g.
 *	insmod ath_tx99.ko device=wifi0
 *
 * To start continuous transmit bring the device up; e.g.
 *	ifconfig wifi0 up
 * To stop continuous transmit mark the device down:
 *	ifconfig wifi0 down
 * or unload the ath_tx99 module.  Note that so long as the
 * module is loaded you can only do continuous transmit.
 *
 * By default the current channel is used.  To specify a channel
 * for transmits use sysctl; e.g.
 *	sysctl -w dev.wifi0.tx99freq=2437
 * The frequency is used to find a channel based on the current
 * band/wireless mode.  This can be overridden with:
 *	sysctl -w dev.wifi0.tx99txmode=8
 * IEEE80211_MODE_11NG; e.g. 8 is the only mode currently supported.
 *
 * The default CWM mode is HT20(0), to set in HT40 mode(2), use:
 *      sysctl -w dev.wifi0.tx99htmode=2
 * Extension offset can be set in 0(none), 1(plus) and 2(minus) by:
 *      sysctl -w dev.wifi0.tx99htext=1
 *
 * Finally the transmit rate can be specified in Kbps; e.g.
 *	sysctl -w dev.wifi0.tx99rate=108000
 * and the transmit power in dBm with
 *	sysctl -w dev.wifi0.tx99power=30
 *
 */
/*
 * Copyright (c) 2013 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 */

#ifdef ATH_TX99_DIAG
#include "ath_internal.h"
#include "if_athvar.h"
#include "ah.h"
#include "ah_desc.h"
#include "ah_devid.h"

#ifdef ATH_PCI      /* PCI BUS */
#include "if_ath_pci.h"
#endif          /* PCI BUS */
#ifdef ATH_AHB      /* AHB BUS */
#include "if_ath_ahb.h"
#endif          /* AHB BUS */
#include "tx99_wcmd.h"
#include "ath_tx99.h"

#if defined(_MAVERICK_STA_) || defined(WIN32) || defined(WIN64)
#define TX99_IS_CHAN_2GHZ(_c)       (((_c)->channel_flags & CHANNEL_2GHZ) != 0)
#define TX99_IS_CHAN_5GHZ(_c)       (((_c)->channel_flags & CHANNEL_5GHZ) != 0)
#define TX99_IS_CHAN_HT_CAPABLE(_c) ((((_c)->channel_flags & CHANNEL_HT20) != 0)     ||  \
(((_c)->channel_flags & CHANNEL_HT40PLUS) != 0) ||  \
(((_c)->channel_flags & CHANNEL_HT40MINUS) != 0))

struct ath_tx99_tgt {
    u_int32_t	txrate;		/* tx rate */
    u_int32_t	txrc;		/* tx rate */
    u_int32_t	txpower;	/* tx power */
    u_int32_t	txchain;	/* tx chainmask */
    u_int32_t	htmode;		/* channel width mode, e.g. HT20, HT40 */
    u_int32_t	type;		/* single carrier or frame type */
    u_int32_t	chtype;		/* channel type */
    u_int32_t	txantenna;	/* tx antenna */
};
static int tx99_start(struct ath_softc *sc);
static void tx99_stop(struct ath_softc *sc, int flag);


/*
 * Drain the transmit queues and reclaim resources.
 */
static void
ath_drain_txq(struct ath_softc *sc)
{
#ifdef ATH_SUPPORT_HTC
    u_int16_t i;
    struct ath_hal *ah = sc->sc_ah;

    /* drain all host endpoints */
    for (i = 0; i < NUM_TX_EP; i++)
        if (ATH_TXQ_SETUP(sc, i))
            ath_tx_drainep(sc, &sc->sc_txep[i]);

    HTCDrainAllEp(sc->sc_host_htc_handle);

    /* stop target wireless DMA */
    if (!sc->sc_invalid) {
        for (i = 0; i < NUM_TX_EP; i++)
            if (ATH_TXQ_SETUP(sc, i)) {
                u_int32_t hwq_num;

                hwq_num = adf_os_htonl(sc->sc_txep[i].hwq_num);
                ath_hal_tx99_tx_stopdma(ah, hwq_num);
            }
    }
    ath_hal_tx99_drain_alltxq(ah);
#else
    /* stop beacon queue. stop and abort all tx qs, flush ath buffer queue */
    ath_draintxq(sc, AH_FALSE, 0);  /* stop xmit side */
#endif
}

static int
tx99_set_cwm_param(struct ath_softc *sc, struct ath_tx99 *tx99)
{
    if ((tx99->htmode == ATH_CWM_MODE20) || (tx99->htmode == ATH_CWM_MODE40)) {
        if (tx99->htmode == ATH_CWM_MODE20) {
            DPRINTF(sc, ATH_DEBUG_TX99, "%s: tx99 in HT20 mode\n", __func__);
            if (tx99->txfreq < 4900)
                tx99->txmode = IEEE80211_MODE_11NG_HT20;
            else
                tx99->txmode = IEEE80211_MODE_11NA_HT20;
        } else {
            DPRINTF(sc, ATH_DEBUG_TX99, "%s: tx99 in HT40 mode\n", __func__);
        }
    } else {
        DPRINTF(sc, ATH_DEBUG_TX99, "%s: htmode not supported!\n", __func__);
        return -EINVAL;
    }

    if ( tx99->htext < 3 ) { //0: extension is minus. 1: extension is plus
        if (tx99->htmode == ATH_CWM_MODE40) {
            switch (tx99->htext) {
                case 1:
                    DPRINTF(sc, ATH_DEBUG_TX99, "%s: set extension mode to plus\n", __func__);
                    if (tx99->txfreq < 4900)
                        tx99->txmode = IEEE80211_MODE_11NG_HT40PLUS;
                    else
                        tx99->txmode = IEEE80211_MODE_11NA_HT40PLUS;
                    break;
                case 2:
                    DPRINTF(sc, ATH_DEBUG_TX99, "%s: set extension mode to minus\n", __func__);
                    if (tx99->txfreq < 4900)
                        tx99->txmode = IEEE80211_MODE_11NG_HT40MINUS;
                    else
                        tx99->txmode = IEEE80211_MODE_11NA_HT40MINUS;
                    break;

                default:
                    DPRINTF(sc, ATH_DEBUG_TX99, "%s: extension offset in HT40 must be 1(plus), 2(minus)!\n", __func__);
                    return -EINVAL;
            }
        } else {   //in ATH_CWM_MODE20 mode
            if(tx99->htext != 0)
            {
                DPRINTF(sc, ATH_DEBUG_TX99, "%s: Can not set extension offset in HT20, always 0!\n", __func__);
                return -EINVAL;
            }
        }
    } else {
        DPRINTF(sc, ATH_DEBUG_TX99, "%s: extension offset must be 0(none), 1(plus), 2(minus)!\n", __func__);
        return -EINVAL;
    }
    return 0;
}

static int
tx99_channel_setup(struct ath_softc *sc)
{
    struct ath_tx99 *tx99 = sc->sc_tx99;
    struct ieee80211com *ic = (struct ieee80211com *)sc->sc_ieee;
    struct ath_hal *ah;
    u_int8_t ieee;

    if(tx99_set_cwm_param(sc, tx99))
    {
        DPRINTF(sc, ATH_DEBUG_TX99, "%s: set cwm parm fail!\n", __func__);
        return -EINVAL;
    }

    /*
     * Locate the channel to use if user-specified; otherwise
     * user the current channel.
     */
    if (tx99->txfreq != 0) {
        ah = sc->sc_ah;
        ieee = ath_hal_mhz2ieee(ah, tx99->txfreq, 0);
        DPRINTF(sc, ATH_DEBUG_TX99, "%s: ieee channel %d!\n", __func__, ieee);
        struct ieee80211_channel *c =
        sc->sc_ieee_ops->ath_net80211_find_channel(sc, ieee, tx99->txmode);
        if (c == NULL) {
            DPRINTF(sc, ATH_DEBUG_TX99, "%s: channel %u mode %u not available\n",
                __func__, tx99->txfreq, tx99->txmode);
             return -EINVAL;
        }
        DPRINTF(sc, ATH_DEBUG_TX99, "%s: switching to channel %u (flags 0x%x mode %u)\n",
            __func__, c->ic_freq, c->ic_flags, tx99->txmode);
        ic->ic_curchan = c;

        if(ic->ic_set_channel(ic))
        {
            DPRINTF(sc, ATH_DEBUG_TX99, "%s: reset channel fail, check frequency settings!\n", __func__);
            return -EINVAL;
        }
    }
    return 0;
}

/*
 * Start continuous transmit operation.
 */
static int
tx99_start(struct ath_softc *sc)
{
    static struct ath_tx99_tgt tx99_tgt;
    struct ath_tx99 *tx99 = sc->sc_tx99;
    struct ath_hal *ah = sc->sc_ah;
    int is2GHz = 0;
    wbuf_t 	wbuf;
    HAL_CHANNEL *chan = NULL;

    if(tx99->recv)
    {
        ath_hal_phydisable(ah);
        DPRINTF(sc, ATH_DEBUG_TX99, "%s: tx99 continuous receive mode, return!\n", __func__);
        return 0;
    }

    /* check tx99 running state */
    if(tx99->tx99_state){		/* already active */
        DPRINTF(sc, ATH_DEBUG_TX99, "%s: already running, return!\n", __func__);
        return 0;
    }

    OS_DELAY(10000);

    /* set tx99 state active */
    tx99->tx99_state = 1;
    /* allocate diag packet buffer */
    if (tx99->wbuf == NULL) {
        tx99->wbuf = wbuf_alloc(sc->sc_osdev, WBUF_TX_DATA, 2000);
        if (tx99->wbuf == NULL) {
            DPRINTF(sc, ATH_DEBUG_TX99, "%s: unable to allocate wbuf!\n", __func__);
            tx99->tx99_state = 0;
            return -ENOMEM;
        }
    } else {
        DPRINTF(sc, ATH_DEBUG_TX99, "%s: wbuf was allocated before!\n", __func__);
    }
    wbuf = tx99->wbuf;
    /* drain all tx queue */
    ath_drain_txq(sc);
    /*
     * Setup channel using configured frequency+flags.
     */
    if (tx99_channel_setup(sc)) {
        DPRINTF(sc, ATH_DEBUG_TX99, "%s: unable to setup channel!\n", __func__);
		/* recover the default channel and mode */
        tx99->txfreq = 2412;/* ieee channel frequecy */
        tx99->htmode = 0;
        tx99->htext = 0;

        goto bad;
    }
    /*
     * Setup tx power limit
     */
    chan = &sc->sc_curchan;
    is2GHz = TX99_IS_CHAN_2GHZ(chan);
    ath_hal_settxpowlimit(ah,tx99->txpower,0,is2GHz);
    /* set tx99 enable */
    tx99_tgt.txrate = htonl(tx99->txrate);
    tx99_tgt.txrc = htonl(tx99->txrc);
    tx99_tgt.txpower = htonl(tx99->txpower);
    tx99_tgt.txchain = htonl(tx99->chanmask);
    tx99_tgt.htmode = htonl(tx99->htmode);
    tx99_tgt.type = htonl(tx99->type);
    tx99_tgt.chtype = htonl(TX99_IS_CHAN_5GHZ(chan));
    tx99_tgt.txantenna = htonl(0);

    if( tx99->txpower < 60 ) /* only update channel pwr if not default MAX power */
        ath_hal_tx99_channel_pwr_update(ah, chan, tx99->txpower);

#ifdef ATH_SUPPORT_HTC
    ah_tx99_start(ah, (u_int8_t *)&tx99_tgt);

    /* send diag packet */
    {
        struct  ath_txep *txep;
        A_STATUS ret;

        adf_nbuf_put_tail(skb, 1500);
        txep = sc->sc_ac2ep[WME_AC_VO];
        /* send packet to target */
        ret = HTCSendPkt(sc->sc_host_htc_handle, NULL ,skb, sc->sc_data_VO_ep);
        if(ret)
        {
            DPRINTF(sc, ATH_DEBUG_TX99, "%s: tx99 fail!\n", __func__);
            tx99_stop(sc, 0);
        }
    }
#else
    if (ath_tx99_tgt_start(sc, tx99_tgt.chtype) != EOK) {
        goto bad;
    }
#endif

    /* wait a while to make sure target setting ready */
    OS_DELAY(50000);
    DPRINTF(sc, ATH_DEBUG_TX99, "%s: continuous transmit started!\n", __func__);
    return 0;
bad:
    tx99_stop(sc, 0);
    return -EIO;
}

/*
 * Terminate continuous transmit operation.
 */
static void
tx99_stop(struct ath_softc *sc, int flag)
{
    struct ath_tx99 *tx99 = sc->sc_tx99;
    struct ieee80211com *ic = (struct ieee80211com *)sc->sc_ieee;

#ifdef ATH_SUPPORT_HTC
    struct ath_hal *ah = sc->sc_ah;
#endif
    if (tx99->tx99_state == 0)
        return;

    if(tx99->wbuf) {
        DPRINTF(sc, ATH_DEBUG_TX99, "%s: release wbuf!\n", __func__);
        wbuf_release(sc->sc_osdev, tx99->wbuf);
        tx99->wbuf = NULL;
	}

#ifdef ATH_SUPPORT_HTC
    ah_tx99_stop(ah);

    ic->ic_reset_start(ic,0);
    ic->ic_reset(ic);
    ic->ic_reset_end(ic,0);
#else
    ath_tx99_tgt_stop(sc);
    ic->ic_reset(ic);
#endif

    tx99->tx99_state = 0;
    DPRINTF(sc, ATH_DEBUG_TX99, "%s: continuous transmit stopped!\n", __func__);
}

u_int8_t tx99_ioctl(ath_dev_t dev, int cmd, void *addr)
{
    struct ath_softc *sc = ATH_DEV_TO_SC(dev);
    struct ath_tx99 *tx99 = sc->sc_tx99;
    static tx99_wcmd_t i_req;
    u_int8_t ret_val = 0;

    OS_MEMCPY(&i_req, addr, sizeof(i_req));
    DPRINTF(sc, ATH_DEBUG_TX99, "%s: request type %d\n", __func__, i_req.type);

    switch(i_req.type){
        case TX99_WCMD_TEST:
            DPRINTF(sc, ATH_DEBUG_TX99, "%s: test\n", __func__);
            ret_val = tx99_start(sc);
            break;
        case TX99_WCMD_ENABLE:
            DPRINTF(sc, ATH_DEBUG_TX99, "%s: enable\n", __func__);
            ret_val = tx99_start(sc);
            break;
        case TX99_WCMD_DISABLE:
            DPRINTF(sc, ATH_DEBUG_TX99, "%s: disable\n", __func__);
            tx99_stop(sc, 0);
            break;
        case TX99_WCMD_SET_FREQ:
            DPRINTF(sc, ATH_DEBUG_TX99, "%s: freq %d %d %d\n",
                __func__, i_req.data.freq, i_req.data.htmode, i_req.data.htext);
            tx99->txfreq = i_req.data.freq;
            tx99->htmode = i_req.data.htmode;
            tx99->htext = i_req.data.htext;
            break;
        case TX99_WCMD_SET_RATE:
        {
            u_int32_t rc;
            DPRINTF(sc, ATH_DEBUG_TX99, "%s: rate %d\n", __func__, i_req.data.rate);
            if (tx99_rate_setup(sc, &rc, i_req.data.rate) == EOK)
            {
                tx99->txrate = i_req.data.rate;
                tx99->txrc = i_req.data.rc = rc;
            }
            break;
        }
        case TX99_WCMD_SET_RC:
        {
            u_int32_t rateKBPS;
            DPRINTF(sc, ATH_DEBUG_TX99, "%s: rate code %d\n", __func__, i_req.data.rc);
            if (tx99_get_rateKBPS(sc, i_req.data.rc, &rateKBPS) == EOK)
            {
                tx99->txrc = i_req.data.rc;
                tx99->txrate = rateKBPS;
            }
            break;
        }
        case TX99_WCMD_SET_POWER:
            DPRINTF(sc, ATH_DEBUG_TX99, "%s: power %d\n", __func__, i_req.data.power);
            tx99->txpower = (i_req.data.power*2);
            break;
        case TX99_WCMD_SET_TXMODE:
            DPRINTF(sc, ATH_DEBUG_TX99, "%s: txmode %d\n", __func__, i_req.data.txmode);
            tx99->txmode = i_req.data.txmode;
            break;
        case TX99_WCMD_SET_CHANMASK:
            DPRINTF(sc, ATH_DEBUG_TX99, "%s: chanmask %d\n", __func__, i_req.data.chanmask);
            tx99->chanmask = i_req.data.chanmask;
            break;
        case TX99_WCMD_SET_TYPE:
            DPRINTF(sc, ATH_DEBUG_TX99, "%s: type %d\n", __func__, i_req.data.type);
            tx99->type = i_req.data.type;
            break;
        case TX99_WCMD_GET:
            if(tx99 == NULL)
                return -EINVAL;
            else
            {
                i_req.data.rate = tx99->txrate;
                i_req.data.rc = tx99->txrc;
                i_req.data.freq = tx99->txfreq;
                i_req.data.htmode = tx99->htmode;
                i_req.data.htext = tx99->htext;
                i_req.data.chanmask = tx99->chanmask;
                i_req.data.power = (tx99->txpower/2);
                i_req.data.txmode = tx99->txmode;

                OS_MEMCPY(addr, &i_req, sizeof(i_req));
            }
            break;
        default:
            return -EINVAL;
            break;
    }
    return ret_val;
}

int
tx99_attach(struct ath_softc *sc)
{
    struct ath_tx99 *tx99 = sc->sc_tx99;

    if (tx99 != NULL) {
        DPRINTF(sc, ATH_DEBUG_TX99, "%s: sc_tx99 was not NULL\n", __FUNCTION__);
        return EINVAL;
	}

    tx99 = (struct ath_tx99 *)OS_MALLOC(sc->sc_osdev, sizeof(struct ath_tx99), GFP_KERNEL);
    if (tx99 == NULL) {
        DPRINTF(sc, ATH_DEBUG_TX99, "%s: no memory for tx99 attach\n", __FUNCTION__);
        return ENOMEM;
	}
    OS_MEMZERO(tx99, sizeof(struct ath_tx99));
    sc->sc_tx99 = tx99;

    tx99->stop = tx99_stop;
    tx99->start = tx99_start;
    tx99->tx99_state = 0;
    tx99->txpower = 60;
    tx99->txrate = 54000;
    tx99->txrc = 0x0c;
    tx99->txfreq = 2412;/* ieee channel frequency */
    tx99->txmode = 0;
    tx99->chanmask = 7;
    tx99->recv = 0;
    tx99->htmode = 0;
    tx99->htext = 0;

    return 0;
}

void
tx99_detach(struct ath_softc *sc)
{
    tx99_stop(sc, 0);

    if (sc->sc_tx99) {
        OS_FREE(sc->sc_tx99);
    }
    sc->sc_tx99 = NULL;
}

#else
#include <adf_os_module.h>
#include <adf_os_stdtypes.h>
#include <adf_os_types.h>
#include <adf_os_dma.h>
#include <adf_os_time.h>
#include <adf_os_lock.h>
#include <adf_os_io.h>
#include <adf_os_mem.h>
#include <adf_os_util.h>
#include <adf_os_defer.h>
#include <adf_os_atomic.h>
#include <adf_nbuf.h>
#include <adf_net_types.h>
#include <adf_net.h>
#include "if_athvar.h"
#include "ah_desc.h"
#include "ah_devid.h"
#ifdef ATH_PCI		/* PCI BUS */
#include "if_ath_pci.h"
#endif			/* PCI BUS */
#ifdef ATH_AHB		/* AHB BUS */
#include "if_ath_ahb.h"
#endif			/* AHB BUS */
#include "tx99_wcmd.h"
#include "ath_tx99.h"
#include "ath_internal.h"
#include "ieee80211_channel.h"
#include "ah.h"
#define TX99_IS_CHAN_2GHZ(_c)       (((_c)->channel_flags & CHANNEL_2GHZ) != 0)
#define TX99_IS_CHAN_5GHZ(_c)       (((_c)->channel_flags & CHANNEL_5GHZ) != 0)
#define TX99_IS_CHAN_HT_CAPABLE(_c) ((((_c)->channel_flags & CHANNEL_HT20) != 0)     ||  \
                                     (((_c)->channel_flags & CHANNEL_HT40PLUS) != 0) ||  \
                                     (((_c)->channel_flags & CHANNEL_HT40MINUS) != 0))
struct ath_tx99_tgt {
    u_int32_t	txrate;		/* tx rate */
    u_int32_t	txpower;	/* tx power */
    u_int32_t	txchain;	/* tx chainmask */
    u_int32_t	htmode;		/* channel width mode, e.g. HT20, HT40 */
    u_int32_t	type;		/* single carrier or frame type */
    u_int32_t	chtype;		/* channel type */
    u_int32_t	txantenna;	/* tx antenna */
};
static void tx99_stop(struct ath_softc *sc, int flag);

#ifdef ATH_SUPPORT_HTC
static void
ath_tx_drainep(struct ath_softc *sc, struct ath_txep *txep)
{
    HTCDrainEp(sc->sc_host_htc_handle,txep->ep_num);
}
#endif
/*
 * Drain the transmit queues and reclaim resources.
 */
static void
ath_drain_txq(struct ath_softc *sc)
{
#ifdef ATH_SUPPORT_HTC
    u_int16_t i;
    struct ath_hal *ah = sc->sc_ah;
    /* drain all host endpoints */
    for (i = 0; i < NUM_TX_EP; i++)
        if (ATH_TXQ_SETUP(sc, i))
            ath_tx_drainep(sc, &sc->sc_txep[i]);
    HTCDrainAllEp(sc->sc_host_htc_handle);
    /* stop target wireless DMA */
    if (!sc->sc_invalid) {
        for (i = 0; i < NUM_TX_EP; i++)
            if (ATH_TXQ_SETUP(sc, i)) {
			    u_int32_t hwq_num;
	            hwq_num = adf_os_htonl(sc->sc_txep[i].hwq_num);
                ath_hal_tx99_tx_stopdma(ah, hwq_num);
            }
    }
    ath_hal_tx99_drain_alltxq(ah);
#else
    /* stop beacon queue. The beacon will be freed when we go to INIT state */
    if (!sc->sc_invalid) {
        if ( !(sc->sc_fastabortenabled && (sc->sc_reset_type == ATH_RESET_NOLOSS)) ){
            (void) ath_hal_stoptxdma(sc->sc_ah, sc->sc_bhalq, 0);
        }
        if (sc->sc_fastabortenabled) {
            /* fast tx abort */
            ath_hal_aborttxdma(sc->sc_ah);
        }
    }
    /* to drain tx dataq */
    sc->sc_ath_ops.tx_flush(sc);
#endif
}
static adf_nbuf_t
ath_alloc_skb_tx99(adf_os_handle_t os_hdl, u_int size, u_int align)
{
    adf_nbuf_t skb;
    skb = adf_nbuf_alloc(os_hdl, size, 0, align, FALSE);
    return skb;
}
static int
tx99_set_cwm_param(struct ath_tx99 *tx99)
{
    if ((tx99->htmode == IEEE80211_CWM_MODE20) || (tx99->htmode == IEEE80211_CWM_MODE40)) {
        if (tx99->htmode == IEEE80211_CWM_MODE20) {
            adf_os_print("tx99 in HT20 mode\n");
            if (tx99->txfreq < 4900)
            {
                tx99->txmode = IEEE80211_MODE_11NG_HT20;
                tx99->phymode = IEEE80211_MODE_11NG_HT20;
            }
            else {
                tx99->txmode = IEEE80211_MODE_11NA_HT20;
                tx99->phymode = IEEE80211_MODE_11NA_HT20;
            }
        } else {
            adf_os_print("tx99 in HT40 mode\n");
        }
    }

    if ( tx99->htext < 3 ) { //2: extension is minus. 1: extension is plus 0: 11B
        if (tx99->htmode == IEEE80211_CWM_MODE40) {
            switch (tx99->htext) {
            case 1:
                adf_os_print("tx99: set extension mode to plus\n");
                if (tx99->txfreq < 4900)
                {
                    tx99->txmode = IEEE80211_MODE_11NG_HT40PLUS;
                    tx99->phymode = IEEE80211_MODE_11NG_HT40PLUS;
                }
                else
                {
                    tx99->txmode = IEEE80211_MODE_11NA_HT40PLUS;
                    tx99->phymode = IEEE80211_MODE_11NA_HT40PLUS;
                }
                break;
            case 2:
                adf_os_print("tx99: set extension mode to minus\n");
                if (tx99->txfreq < 4900)
                {
                    tx99->txmode = IEEE80211_MODE_11NG_HT40MINUS;
                    tx99->phymode = IEEE80211_MODE_11NG_HT40MINUS;
                }
                else {
                    tx99->txmode = IEEE80211_MODE_11NA_HT40MINUS;
                    tx99->phymode = IEEE80211_MODE_11NA_HT40MINUS;
                }
                break;
            }
        }
        else {   //in IEEE80211_CWM_MODE20 mode
            if(tx99->htext != 0) {
                adf_os_print("tx99: Can not set extension offset in HT20, always 0.\n");
                return -EINVAL;
            }
        }
    }
    else {
        adf_os_print("extension offset must be 0(none), 1(plus), 2(minus).\n");
        return -EINVAL;
    }
    return EOK;
}
static int
tx99_channel_setup(struct ath_softc *sc)
{
    struct ath_tx99 *tx99 = sc->sc_tx99;
    struct ieee80211com *ic = (struct ieee80211com *)sc->sc_ieee;
    struct ath_hal *ah;
    u_int8_t ieee;
    struct ieee80211_channel *c;
    if(tx99_set_cwm_param(tx99) != EOK)
    {
        adf_os_print("%s: set cwm parm fail\n", __FUNCTION__);
        return -EINVAL;
    }

    /*
     * Locate the channel to use if user-specified; otherwise
     * user the current channel.
     */
    if (tx99->txfreq != 0) {
        ah = sc->sc_ah;
        ieee = ath_hal_mhz2ieee(ah, tx99->txfreq, 0);
        adf_os_print("%s: ieee = %d\n", __FUNCTION__, ieee);
        adf_os_print("%s: tx99->txfreq=%d,tx99->txmode=%d\n", __FUNCTION__, tx99->txfreq, tx99->txmode);
       c = sc->sc_ieee_ops->ath_net80211_find_channel(sc, ieee, tx99->txmode);

        if (c == NULL) {
            adf_os_print("%s: channel %u mode %u not available\n",
            __FUNCTION__, tx99->txfreq, tx99->txmode);
            return -EINVAL;
        }
        adf_os_print("%s: switching to channel %u (flags 0x%x mode %u)\n",
            __FUNCTION__, c->ic_freq, c->ic_flags, tx99->txmode);
        ic->ic_curchan = c;
	    if(ic->ic_set_channel(ic))
        {
            adf_os_print("%s: reset channel fail, check frequency settings\n", __FUNCTION__);
            return -EINVAL;
        }
    }
    return EOK;
}
/*
 * Start continuous transmit operation.
 */
static int
tx99_start(struct ath_softc *sc)
{
    static struct ath_tx99_tgt tx99_tgt;
    struct ath_tx99 *tx99 = sc->sc_tx99;
    struct ath_hal *ah = sc->sc_ah;
    int is2GHz = 0;
    adf_nbuf_t 	skb;
    HAL_CHANNEL *c = NULL;
    if(tx99->recv)
    {
        ath_hal_phydisable(ah);
        //adf_os_print("%s: device %s tx99 continuous receive mode\n", __FUNCTION__, adf_net_ifname(sc->sc_ic->ic_dev));
        return 0;
    }
    /* check tx99 running state */
    if(tx99->tx99_state){		/* already active */
        adf_os_print("%s: already running\n", __FUNCTION__);
        return 0;
    }
    /* set tx99 state active */
    tx99->tx99_state = 1;
    /* allocate diag packet buffer */
    tx99->skb = ath_alloc_skb_tx99(sc->sc_osdev, 2000, 32);
    if (tx99->skb == NULL) {
        adf_os_print("%s: unable to allocate skb\n", __FUNCTION__);
        tx99->tx99_state = 0;
        return -ENOMEM;
    }
    skb = tx99->skb;
    /* drain all tx queue */
    ath_drain_txq(sc);
    /*
     * Setup channel using configured frequency+flags.
     */
    if (tx99_channel_setup(sc) != EOK) {
        adf_os_print("%s: unable to setup operation\n", __FUNCTION__);
        tx99->tx99_state = 0;
        adf_nbuf_free(skb);
        return -EIO;
    }
    /*disable desc tpc */
    ath_hal_settpc(ah,0);
    /*
     * Setup tx power limit
     */
    c = &sc->sc_curchan;
    is2GHz = TX99_IS_CHAN_2GHZ(c);
    ath_hal_settxpowlimit(ah,tx99->txpower,0,is2GHz);
    /* set tx99 enable */
    tx99_tgt.txrate = adf_os_htonl(tx99->txrate);
    tx99_tgt.txpower = adf_os_htonl(tx99->txpower);
    tx99_tgt.txchain = adf_os_htonl(tx99->chanmask);
    tx99_tgt.htmode = adf_os_htonl(tx99->htmode);
    tx99_tgt.type = adf_os_htonl(tx99->type);
    tx99_tgt.chtype = adf_os_htonl(TX99_IS_CHAN_5GHZ(c));
    tx99_tgt.txantenna = adf_os_htonl(0);
    if( tx99->txpower < 60 ) /* only update channel pwr if not default MAX power */
        ath_hal_tx99_channel_pwr_update(ah, c, tx99->txpower);
#ifdef ATH_SUPPORT_HTC
    ah_tx99_start(ah, (u_int8_t *)&tx99_tgt);
    /* send diag packet */
    {
        struct  ath_txep *txep;
        adf_nbuf_t 	skb;

        A_STATUS ret;
        adf_nbuf_put_tail(skb, 1500);
        txep = sc->sc_ac2ep[WME_AC_VO];
        /* send packet to target */
        ret = HTCSendPkt(sc->sc_host_htc_handle, NULL ,skb, sc->sc_data_VO_ep);
        if(ret)
        {
            adf_os_print("%s: tx99 fail \n", __FUNCTION__);
            tx99_stop(sc, 0);
        }
    }
#else
    adf_nbuf_put_tail(skb, 1500);
    if (ath_tx99_tgt_start(sc, tx99_tgt.chtype) != EOK) {
        adf_os_print("%s: tx99 fail \n", __FUNCTION__);
        tx99_stop(sc, 0);
    }
#endif
    /* wait a while to make sure target setting ready */
    adf_os_mdelay(50);
    adf_os_print("%s: continuous transmit started\n", __FUNCTION__);
    return 0;
}
/*
 * Terminate continuous transmit operation.
 */
static void
tx99_stop(struct ath_softc *sc, int flag)
{
    struct ath_tx99 *tx99 = sc->sc_tx99;
    struct ieee80211com *ic = (struct ieee80211com *)sc->sc_ieee;
#ifdef ATH_SUPPORT_HTC
    struct ath_hal *ah = sc->sc_ah;
#else
    adf_nbuf_t  skb = tx99->skb;
#endif
    if (tx99->tx99_state == 0)
        return;
#ifdef ATH_SUPPORT_HTC
    ah_tx99_stop(ah);
    ic->ic_reset_start(ic,0);
    ic->ic_reset(ic);
    ic->ic_reset_end(ic,0);
#else
    ath_tx99_tgt_stop(sc);
    if (flag == 0) {
        ic->ic_reset_start(ic,0);
        ic->ic_reset(ic);
        ic->ic_reset_end(ic,0);
    } else if (flag == 1) {
        ic->ic_reset_start(ic,0);
        ic->ic_reset(ic);
    } else if (flag == 2) {
        ic->ic_reset_end(ic,0);
    }
    adf_nbuf_free(skb);
#endif
    tx99->tx99_state = 0;
    adf_os_print("%s: continuous transmit stopped\n", __FUNCTION__);
}

u_int8_t
tx99_ioctl(ath_dev_t device, struct ath_softc *sc, int cmd, void *addr)
{
    struct ath_tx99 *tx99 = sc->sc_tx99;
    struct net_device *dev = (struct net_device *)device;
    static tx99_wcmd_t i_req;
    OS_MEMCPY(&i_req, addr, sizeof(i_req));
    switch(i_req.type){
        case TX99_WCMD_ENABLE:
            adf_os_print("tx99_ioctl command : enable\n");
            if ((dev->flags & (IFF_RUNNING|IFF_UP)) != (IFF_RUNNING|IFF_UP)) {
#ifdef __linux__
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,30)
                dev->netdev_ops->ndo_open(dev);
#else
                dev->open(dev);
#endif
#else
                adf_os_print("Interface is not active\n");
                return -EINVAL;
#endif
            }
#ifndef ATH_SUPPORT_HTC
            if (tx99->testmode != TX99_TESTMODE_SINGLE_CARRIER) {
                /**/
                tx99_start(sc);
                adf_os_mdelay(1000);
                tx99_stop(sc, 0);
                adf_os_mdelay(50);
            }
#endif
            tx99_start(sc);
            break;
        case TX99_WCMD_DISABLE:
            adf_os_print("tx99_ioctl command : disable\n");
            tx99_stop(sc, 0);
            break;
        case TX99_WCMD_SET_FREQ:
            adf_os_print("tx99_ioctl command : freq %d %d %d\n",i_req.data.freq,i_req.data.htmode,i_req.data.htext);
            tx99->txfreq = i_req.data.freq;
            tx99->htmode = i_req.data.htmode;
            tx99->htext = i_req.data.htext;
            break;
        case TX99_WCMD_SET_RATE:
            adf_os_print("tx99_ioctl command : rate %d\n",i_req.data.rate);
            tx99->txrate = i_req.data.rate;
            break;
        case TX99_WCMD_SET_POWER:
            adf_os_print("tx99_ioctl command : pwr %d\n",i_req.data.power);
            tx99->txpower = (i_req.data.power*2);
            break;
        case TX99_WCMD_SET_TXMODE:
            adf_os_print("tx99_ioctl command : txmode %d\n",i_req.data.txmode);
            tx99->txmode = i_req.data.txmode;
            break;
        case TX99_WCMD_SET_CHANMASK:
            adf_os_print("tx99_ioctl command : chanmask %d\n",i_req.data.chanmask);
            tx99->chanmask = i_req.data.chanmask;
            break;
        case TX99_WCMD_SET_TYPE:
            adf_os_print("tx99_ioctl command : type %d\n",i_req.data.type);
            tx99->type = i_req.data.type;
            break;
        case TX99_WCMD_SET_TESTMODE:
            adf_os_print("tx99_ioctl command : testmode %d\n",i_req.data.testmode);
            tx99->testmode = i_req.data.testmode;
            if (tx99->testmode == TX99_TESTMODE_RX_ONLY)
                tx99->recv = 1;
            else
                tx99->recv = 0;
            break;
        case TX99_WCMD_GET:
            if(tx99 == NULL)
                return -EINVAL;
            else
            {
                i_req.data.rate = tx99->txrate;
                i_req.data.freq = tx99->txfreq;
                i_req.data.htmode = tx99->htmode;
                i_req.data.htext = tx99->htext;
                i_req.data.power = (tx99->txpower/2);
                i_req.data.txmode = tx99->txmode;
            }
            break;
        default:
            return -EINVAL;
            break;
    }
    return EOK;
}

int
tx99_attach(struct ath_softc *sc)
{
    struct ath_tx99 *tx99 = sc->sc_tx99;
    if (tx99 != NULL) {
		adf_os_print("%s: sc_tx99 was not NULL\n", __FUNCTION__);
		return EINVAL;
	}
    tx99 = adf_os_mem_alloc(NULL, sizeof(struct ath_tx99));
    if (tx99 == NULL) {
		adf_os_print("%s: no memory for tx99 attach\n", __FUNCTION__);
		return ENOMEM;
	}
    adf_os_mem_set(tx99, 0, sizeof(struct ath_tx99));
    sc->sc_tx99 = tx99;

    tx99->stop = tx99_stop;
    tx99->start = tx99_start;
    tx99->tx99_state = 0;
    tx99->txpower = 60;
    tx99->txrate = 300000;
    tx99->txfreq = 6;/* ieee channel number */
    tx99->txmode = IEEE80211_MODE_11NG_HT40PLUS;
    tx99->phymode = IEEE80211_MODE_AUTO;
    tx99->chanmask = 1;
    tx99->recv = 0;
    tx99->testmode = TX99_TESTMODE_TX_PN;

    return EOK;
}

void
tx99_detach(struct ath_softc *sc)
{
    if(sc) {
        tx99_stop(sc, 0);

        if (sc->sc_tx99) {
            adf_os_mem_free(sc->sc_tx99);
        }
        sc->sc_tx99 = NULL;
    }
}

/*
 * Module glue.
 */
static	char *dev_info = "ath_tx99";

static int __init
init_ath_tx99(void)
{
	adf_os_print("%s: Version 2.0\n"
        "Copyright (c) 2010 Atheros Communications, Inc, "
        "All Rights Reserved\n", dev_info);
	return EOK;
}

static void __exit
exit_ath_tx99(void)
{
	adf_os_print("%s: driver unloaded\n", dev_info);
}

adf_os_virt_module_init(init_ath_tx99);
adf_os_virt_module_exit(exit_ath_tx99);
adf_os_virt_module_name(ath_tx99);
EXPORT_SYMBOL(tx99_attach);
EXPORT_SYMBOL(tx99_detach);
EXPORT_SYMBOL(tx99_ioctl);
#endif
#endif
