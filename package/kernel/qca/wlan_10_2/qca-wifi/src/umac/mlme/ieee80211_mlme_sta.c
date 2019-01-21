/*
 * Copyright (c) 2010, Atheros Communications Inc. 
 * All Rights Reserved.
 * 
 * Copyright (c) 2011 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 * 
 */

#include "ieee80211_mlme_priv.h"    /* Private to MLME module */
#include <ieee80211_target.h>

#if UMAC_SUPPORT_STA || UMAC_SUPPORT_BTAMP


static int mlme_assoc_reassoc_request(wlan_if_t vaphandle, 
                                      int       reassoc,
                                      u_int8_t  *prev_bssid, 
                                      u_int32_t timeout);
static int mlme_process_asresp_elements(struct ieee80211_node *ni,
                                        u_int8_t              *frm,
                                        u_int32_t             ie_len);
void ieee80211_vap_iter_notify_beacon_rssi(void *arg, wlan_if_t vap);

static int mlme_process_timeout_interval_elements(struct ieee80211_node *ni,
                                        u_int8_t              *frm,
                                        u_int32_t             ie_len);

ieee80211_scan_entry_t ieee80211_mlme_get_bss_entry(struct ieee80211vap *vap)
{
    /*
     * XXX: TO DO: MLME should save information about Home AP or associated clients.
     */
    return NULL;
}

/* 
 * Function to get the correct mode of the given channel(arg) to be used by the
 * hardware.
 * It iterates through other vaps and returns the channel mode which is a superset
 * of all modes.
 * This is usually used in repeater configuration, where the STA VAP is connected
 * to a 11G rootAP and still we want the AP VAP to be 11N. Here 11N is a superset
 * of 11G.
 *        rootap mode   Repeater-mode 
 * case 1    11A         11NAHT40PLUS or 11NAHT40MINUS
 * case 2    11G         11NGHT20
 * case 3    11NGHT20    11NGHT20
 */
void
ieee80211_vap_iter_get_chan(void *arg, wlan_if_t vap) 
{
    struct ieee80211_channel *chan = (struct ieee80211_channel *)arg;
    struct ieee80211_channel *vap_des_chan = vap->iv_des_chan[vap->iv_des_mode];
    enum ieee80211_phymode chan_mode;
    enum ieee80211_phymode des_chan_mode;

    if ((ieee80211vap_get_opmode(vap) != IEEE80211_M_HOSTAP) || 
        (vap_des_chan == IEEE80211_CHAN_ANYC) ||
        (NULL == chan) ||
        (NULL == vap_des_chan)) 
    {
        return;
    }
    
    IEEE80211_DPRINTF(vap, IEEE80211_MSG_DEBUG, "%s : desired mode %d vap_des_chan %p", 
                      __func__, vap->iv_des_mode, vap_des_chan);

    chan_mode = ieee80211_chan2mode(chan);
    des_chan_mode = ieee80211_chan2mode(vap_des_chan);
    
    if (IEEE80211_IS_CHAN_HT_CAPABLE(chan) && 
        IEEE80211_IS_CHAN_HT_CAPABLE(vap_des_chan)) {
        /* 11g channel uses rootap mode if it is HT capable */
        if (IEEE80211_IS_CHAN_2GHZ(chan)) {
           return;
        }
    }

    if ((IEEE80211_IS_CHAN_5GHZ(chan) && IEEE80211_IS_CHAN_5GHZ(vap_des_chan)) ||
        (IEEE80211_IS_CHAN_2GHZ(chan) && IEEE80211_IS_CHAN_2GHZ(vap_des_chan))) {
        IEEE80211_DPRINTF(vap, IEEE80211_MSG_MLME, "Old chmode %s ", 
               ieee80211_phymode_name[chan_mode]);
       if (des_chan_mode > chan_mode) {
            if (IEEE80211_IS_CHAN_ANYG(chan)) { 
                chan->ic_flags = IEEE80211_CHAN_11NG_HT20;
            }
            else {
                chan->ic_flags = vap_des_chan->ic_flags;
            }
        }
        chan_mode = ieee80211_chan2mode(chan);
        IEEE80211_DPRINTF(vap, IEEE80211_MSG_MLME, " New chmode %s\n", 
               ieee80211_phymode_name[chan_mode]);
    }
}

/*
 * Sets that hardware up for joining to the specified
 * infrastructure network. The hardware should 
 * set itself up to synchronize to beacons from the specified 
 * access point. Once the join has completed, the mlme_join_complete_infra 
 * routine should be called.
 *
 * timeout - change from number of beacon intervals to milliseconds to meet win8 requirement
 *           xijin. 2011/05/31
 */
int wlan_mlme_join_infra(wlan_if_t vaphandle, wlan_scan_entry_t bss_entry, u_int32_t timeout)
{
    struct ieee80211vap           *vap = vaphandle;
    struct ieee80211com           *ic = vap->iv_ic;
    ieee80211_scan_entry_t        scan_entry = bss_entry;
    struct ieee80211_mlme_priv    *mlme_priv = vap->iv_mlme_priv;
    struct ieee80211_node         *ni;
    int                           error = 0;

    IEEE80211_DPRINTF(vap, IEEE80211_MSG_MLME, "%s\n", __func__);

    /* Initialize join state variables */
    ASSERT(mlme_priv->im_request_type == MLME_REQ_NONE);
    mlme_priv->im_request_type = MLME_REQ_JOIN_INFRA;
    atomic_set(&(mlme_priv->im_join_wait_beacon_to_synchronize), 0);
    mlme_priv->im_connection_up = 0;

    error = ieee80211_sta_join(vap, scan_entry);
    if (error) {
        IEEE80211_DPRINTF(vap, IEEE80211_MSG_MLME, "%s: Error %d (0x%08X) in ieee80211_sta_join\n", 
            __func__, error, error);
        goto err;
    }

    /* iv_bss is valid only after ieee80211_sta_join */ 
    ni = vap->iv_bss;
    ni->ni_assocstarttime = OS_GET_TICKS();
    /*
     * issue a vap start request to resource manager.
     * if the function returns EOK (0) then its ok to change the channel synchronously
     * if the function returns EBUSY  then resource manager will 
     * switch channel asynchronously and post an event event handler registred by vap and
     * vap handler will intern call the wlan_mlme_join_infra_continue .
     */
    mlme_priv->im_timeout = timeout;
    error = ieee80211_resmgr_vap_start(ic->ic_resmgr, vap,ni->ni_chan, MLME_REQ_ID, 0);
    if (error == EOK) {
        /* 
         * If WDS is enabled in STA VAP, we are most likely a repeater with another
         * AP VAP (pending to be up). Set the PHY/Radio in a super mode 
         * (11ng is a supermode for 11g), so that, the AP VAP can be in 11n mode,
         * though the STA VAP is connected to 11g rootAP
         */
        struct ieee80211_channel *phychan = NULL;

        {
            struct ieee80211_channel chan;
            enum ieee80211_phymode chmode;
           
            chan.ic_ieee = ni->ni_chan->ic_ieee;           
            chan.ic_freq = ni->ni_chan->ic_freq;
            chan.ic_flags = ni->ni_chan->ic_flags;
            wlan_iterate_vap_list(ic, ieee80211_vap_iter_get_chan, &chan);
            
            phychan = ieee80211_find_channel(ic, chan.ic_freq, chan.ic_flags);
            if (phychan == NULL) {
                /* This could be due to rootap channel and repeater-ap mode 
                   inconsistency, retry again updating the channel mode */
                switch(chan.ic_flags)
                {
                   case IEEE80211_CHAN_11NG_HT40PLUS:
                       chan.ic_flags = IEEE80211_CHAN_11NG_HT40MINUS;
                       break;
                     
                   case IEEE80211_CHAN_11NG_HT40MINUS:
                       chan.ic_flags = IEEE80211_CHAN_11NG_HT40PLUS;
                       break;
                   
                   case IEEE80211_CHAN_11NA_HT40PLUS:
                       chan.ic_flags = IEEE80211_CHAN_11NA_HT40MINUS;
                       break;

                   case IEEE80211_CHAN_11NA_HT40MINUS:
                       chan.ic_flags = IEEE80211_CHAN_11NA_HT40PLUS;
                       break;

                   default:
                       break;
                }
                chmode = ieee80211_chan2mode(&chan);
               IEEE80211_DPRINTF(vap, IEEE80211_MSG_MLME,"Finding channel with channel %d new mode %s\n", 
                       chan.ic_ieee, ieee80211_phymode_name[chmode]);
                phychan = ieee80211_find_channel(ic, chan.ic_freq, chan.ic_flags);
                if (phychan == NULL) {
                   IEEE80211_DPRINTF(vap, IEEE80211_MSG_MLME,"%s: Failed to get the channel!\n", __func__);
                }
            }
        }
        if (phychan == NULL)
            phychan = ni->ni_chan;

        /* XXX: Always switch channel even if we are already on that channel.
         * This is to make sure the ATH layer always syncs beacon when we set
         * our home channel.  
         */
        vap->iv_bsschan = ni->ni_chan;
        IEEE80211_DPRINTF(vap, IEEE80211_MSG_MLME, "Setting channel number %d\n", ni->ni_chan->ic_ieee);
        printf("%s.. SETTING CHANNEL NUMBER %d CURR CHANNEl is %d flags 0x%x\n", 
                __func__, ni->ni_chan->ic_ieee, vap->iv_ic->ic_curchan->ic_ieee, ni->ni_chan->ic_flags );
        ieee80211_set_channel(ic, phychan);
        ieee80211_wme_initparams(vap);
        ieee80211_mlme_join_infra_continue(vap,EOK);
    } else if (error == EBUSY) {
        /* 
         * resource manager is handling the request asynchronously,
         *  this is transparent to  the caller , return EOK to the caller.
         */
        error = EOK;
    }  else {
        IEEE80211_DPRINTF(vap, IEEE80211_MSG_MLME, "%s: Error %d (0x%08X) in ieee80211_resmgr_vap_start\n", 
            __func__, error, error);
        return error;
    }

    return error;

err:
    mlme_priv->im_request_type = MLME_REQ_NONE;
    return error;
}

void ieee80211_mlme_join_infra_continue(struct ieee80211vap *vap, int32_t status)
{
    struct ieee80211com           *ic = vap->iv_ic;
    struct ieee80211_node         *ni;
    struct ieee80211_mlme_priv    *mlme_priv = vap->iv_mlme_priv;
    u_int32_t                     join_timeout_ms;

    if (mlme_priv->im_request_type != MLME_REQ_JOIN_INFRA) {
        IEEE80211_DPRINTF(vap, IEEE80211_MSG_MLME, "%s : im_request_type != JOIN_INFRA\n", 
            __func__);
        return;
    }
    
    if (status != EOK) {
        mlme_priv->im_request_type = MLME_REQ_NONE;
        IEEE80211_DELIVER_EVENT_MLME_JOIN_COMPLETE_INFRA(vap, IEEE80211_STATUS_UNSPECIFIED);
        return;
     }
    
    /* iv_bss is valid only after ieee80211_sta_join */ 
    ni = vap->iv_bss;

    IEEE80211_DPRINTF(vap, IEEE80211_MSG_MLME, "%s ni=%02X:%02X:%02X:%02X:%02X:%02X\n", 
        __func__,
        ni->ni_macaddr[0], ni->ni_macaddr[1], ni->ni_macaddr[2],
        ni->ni_macaddr[3], ni->ni_macaddr[4], ni->ni_macaddr[5]);
    
    /* Update erp info */
    if (ni->ni_erp & IEEE80211_ERP_USE_PROTECTION)
        ic->ic_flags |= IEEE80211_F_USEPROT;
    else
        ic->ic_flags &= ~IEEE80211_F_USEPROT;
    ic->ic_update_protmode(ic);

    if(ni->ni_erp & IEEE80211_ERP_LONG_PREAMBLE)
        ic->ic_flags |= IEEE80211_F_USEBARKER;
    else
        ic->ic_flags &= (~IEEE80211_F_USEBARKER);

    /* Update slot time info */
    ieee80211_set_shortslottime(ic,
                                IEEE80211_IS_CHAN_A(vap->iv_bsschan) ||
                                IEEE80211_IS_CHAN_11NA(vap->iv_bsschan) ||
                                (ni->ni_capinfo & IEEE80211_CAPINFO_SHORT_SLOTTIME));

    /* Put underlying H/W to JOIN state */
    ieee80211_vap_join(vap);
    
#ifdef ATH_SUPPORT_TxBF
    ieee80211_init_txbf(ic, ni);
#endif
    /* Send a direct probe to increase the odds of receiving a probe response */
    ieee80211_send_probereq(ni, vap->iv_myaddr, ni->ni_bssid,
                            ni->ni_bssid, ni->ni_essid, ni->ni_esslen,
                            vap->iv_opt_ie.ie, vap->iv_opt_ie.length);

    /* Set the timeout timer for Join Failure case. */
    join_timeout_ms = mlme_priv->im_timeout;//IEEE80211_TU_TO_MS(mlme_priv->im_timeout * ni->ni_intval);
    IEEE80211_DPRINTF(vap, IEEE80211_MSG_MLME, "Setting Join Timeout timer for %d ms\n", join_timeout_ms);
    OS_SET_TIMER(&mlme_priv->im_timeout_timer, join_timeout_ms);

    /* Set the appropriate filtering function and wait for Join Beacon */
    MLME_WAIT_FOR_BSS_JOIN(mlme_priv);
}

int wlan_mlme_auth_request(wlan_if_t vaphandle, u_int32_t timeout)
{
    struct ieee80211vap           *vap = vaphandle;
    struct ieee80211_mlme_priv    *mlme_priv = vap->iv_mlme_priv;
    struct ieee80211_node         *ni = vap->iv_bss; /* bss node */
    int                           error = 0;

    IEEE80211_DPRINTF(vap, IEEE80211_MSG_MLME, "%s\n", __func__);

    /* wait for auth seq number 2 (open response or shared challenge) */
    ASSERT(mlme_priv->im_request_type == MLME_REQ_NONE);
    mlme_priv->im_request_type = MLME_REQ_AUTH;
    mlme_priv->im_expected_auth_seq_number = IEEE80211_AUTH_OPEN_RESPONSE;

    /* Set the timeout timer for authenticate failure case */
    OS_SET_TIMER(&mlme_priv->im_timeout_timer, timeout);

    /*  Send the authentication packet */
    error = ieee80211_send_auth(ni, IEEE80211_AUTH_SHARED_REQUEST, 0, NULL, 0, NULL);
    if (error) {
        goto err;
    }

    return error;

err:
    mlme_priv->im_request_type = MLME_REQ_NONE;
    OS_CANCEL_TIMER(&mlme_priv->im_timeout_timer);
    return error;
}

int wlan_mlme_assoc_request(wlan_if_t vaphandle, u_int32_t timeout)
{
    IEEE80211_DPRINTF(vaphandle, IEEE80211_MSG_MLME, "%s\n", __func__);
    return mlme_assoc_reassoc_request(vaphandle, 0, NULL, timeout);
}

int wlan_mlme_reassoc_request(wlan_if_t vaphandle, u_int8_t *prev_bssid, u_int32_t timeout)
{
    IEEE80211_DPRINTF(vaphandle, IEEE80211_MSG_MLME, "%s\n", __func__);
    return mlme_assoc_reassoc_request(vaphandle, 1, prev_bssid, timeout);
}

/*
 * Private APIs 
 *      - private to mlme implementation
 *      - called by mgmt frame processing (ieee80211_mgmt_input.c)
 */

/* Confirmations */
void ieee80211_mlme_join_complete_infra(struct ieee80211_node *ni)
{
    struct ieee80211vap           *vap = ni->ni_vap;
    struct ieee80211_mlme_priv    *mlme_priv = vap->iv_mlme_priv;

    if ((mlme_priv->im_request_type == MLME_REQ_JOIN_INFRA) && (MLME_STOP_WAITING_FOR_JOIN(mlme_priv) == 1)) 
    {
        IEEE80211_DPRINTF(vap, IEEE80211_MSG_MLME, "%s\n", __func__);

        /* Request complete */
        mlme_priv->im_request_type = MLME_REQ_NONE;

        /*
         * We have received the beacon that synchronises us with the BSS.
         * We don't care whether the Timer got cancelled or not. The macro
         * HW_STOP_WAITING_FOR_JOIN synchronizes us with the cancel operation
         */
        OS_CANCEL_TIMER(&mlme_priv->im_timeout_timer);

        /* Call MLME confirmation handler */
        IEEE80211_DELIVER_EVENT_MLME_JOIN_COMPLETE_INFRA(vap, IEEE80211_STATUS_SUCCESS);
        }
    else 
    {
        if (mlme_priv->im_request_type != MLME_REQ_NONE) {
            IEEE80211_DPRINTF(vap, IEEE80211_MSG_MLME, "%s: Failed: im_request_type=%d\n", 
                __func__,
                mlme_priv->im_request_type);
        }        
    }
    
    /* start SW bmiss. will be here for every beacon received from our AP  */
    mlme_sta_swbmiss_timer_start(vap);
}

/* Receive assoc/reassoc response 
 * - the caller of this routine validates the frame and ensures that the opmode == STA
 */
void ieee80211_mlme_recv_assoc_response(struct ieee80211_node *ni, 
                                        int                   subtype,
                                        u_int16_t             capability, 
                                        u_int16_t             status_code, 
                                        u_int16_t             aid,
                                        u_int8_t              *ie_data, 
                                        u_int32_t             ie_length, 
                                        wbuf_t                wbuf)
{
    struct ieee80211vap           *vap = ni->ni_vap;
    struct ieee80211_mlme_priv    *mlme_priv = vap->iv_mlme_priv;
#if ATH_SUPPORT_AOW || ATH_SUPPORT_WRAP
    struct ieee80211com           *ic = ni->ni_ic;    
#endif
    int                           mlme_request_type = mlme_priv->im_request_type;
    int                           error;
    u_int32_t                     rxlinkspeed, txlinkspeed; /* bits/sec */
#if ATH_SUPPORT_WRAP
	struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
	struct ath_softc *sc = (struct ath_softc *)scn->sc_dev;
	u_int8_t mac[6];
	HAL_KEYVAL hk;
	memset(&hk, 0, sizeof(hk)); 
#endif	
    IEEE80211_DPRINTF(vap, IEEE80211_MSG_MLME, "%s im_request_type=%d status=%d (0x%08X)\n", 
        __func__,
        mlme_priv->im_request_type,
        status_code, status_code);

    /* Ignore if no request in progress */
    if ((mlme_priv->im_request_type != MLME_REQ_ASSOC) &&
        (mlme_priv->im_request_type != MLME_REQ_REASSOC))
    {
        IEEE80211_DPRINTF(vap, IEEE80211_MSG_MLME, "%s: Incorrect request type %d\n", 
            __func__, mlme_priv->im_request_type);
        return;
    }

    if (!OS_CANCEL_TIMER(&mlme_priv->im_timeout_timer)) {
        IEEE80211_DPRINTF(vap, IEEE80211_MSG_MLME, "%s: Timed-out already\n", __func__);
        return;
    }

    if (status_code != IEEE80211_STATUS_SUCCESS) {
        goto complete;
    }

    /* Validate AID */
    aid  &= ~IEEE80211_FIELD_TYPE_AID;
    if ((aid > 2007) || (aid == 0))
    {
        IEEE80211_DPRINTF(vap, IEEE80211_MSG_MLME, "%s: Association response contains invalid AID=%d\n", __func__, aid);
        status_code = IEEE80211_STATUS_UNSPECIFIED;
        goto complete;
    }

    error = mlme_process_asresp_elements(ni, ie_data, ie_length);
    if (error) {
        IEEE80211_DPRINTF(vap, IEEE80211_MSG_MLME, "%s: mlme_process_asresp_elements failed\n", __func__);
        status_code = IEEE80211_STATUS_UNSPECIFIED;
        goto complete;
    }

    /* Association successful */

complete:
    switch (mlme_priv->im_request_type) {
    case MLME_REQ_ASSOC:
        IEEE80211_DPRINTF(vap, IEEE80211_MSG_MLME, "%s: mlme_assoc_complete status %d\n", __func__, status_code);
        if (subtype != IEEE80211_FC0_SUBTYPE_ASSOC_RESP) {
			IEEE80211_DPRINTF(vap, IEEE80211_MSG_ANY, "%s: mlme_assoc_complete status type mismatched %d\n", __func__, subtype);
			return;
        }
        break;

    case MLME_REQ_REASSOC:
        IEEE80211_DPRINTF(vap, IEEE80211_MSG_MLME, "%s: mlme_reassoc_complete status %d\n", __func__, status_code);
        if (subtype != IEEE80211_FC0_SUBTYPE_REASSOC_RESP) {
			IEEE80211_DPRINTF(vap, IEEE80211_MSG_ANY, "%s: mlme_assoc_complete status type mismatched %d\n", __func__, subtype);
			return;
        }
        break;

    default:
        IEEE80211_DPRINTF(vap, IEEE80211_MSG_ANY, "%s: mlme_reassoc_complete status %d unexpected request type %d\n", 
            __func__, status_code, mlme_priv->im_request_type);
        return;
    }

    /* Request complete */
    mlme_priv->im_request_type = MLME_REQ_NONE;

    if (status_code == IEEE80211_STATUS_SUCCESS) {
        ASSERT(aid != 0);
        ni->ni_associd = aid;
        ni->ni_assoctime = OS_GET_TICKS() - ni->ni_assocstarttime;
#if ATH_SUPPORT_HTC
        ieee80211_update_node_target(ni, ni->ni_vap);
#endif            

#if ATH_SUPPORT_AOW
        ieee80211_aow_join_indicate(ic, AOW_STA_CONNECTED, ni);
#endif  /* ATH_SUPPORT_AOW */

        /* Association successful, put underlying H/W into ready state */
        ieee80211_vap_start(vap);

#if ATH_SUPPORT_WRAP
		if(!ic->ic_is_mode_offload(ic)) { 
			if ((ath_hal_getcapability(sc->sc_ah, HAL_CAP_WRAP_PROMISC, 0, NULL) == HAL_ENOTSUPP) && wlan_is_mpsta(vap)) {
				/*setting root AP mac address at keyindex 1, to receive beacons.
				 * keyindex 1 is chosen because group key for STA vap will use key index 1 or 2*/
				hk.kv_type = HAL_CIPHER_CLR;
				OS_MEMCPY(mac,ni->ni_bssid,6);
				mac[0] |= 0x01;
				scn->sc_ops->key_set(scn->sc_dev, 1, &hk, mac);           
			}
		}
#endif
    }
#if UMAC_SUPPORT_SMARTANTENNA        
        ieee80211_smartantenna_node_init(ni);
#endif
    if (status_code == IEEE80211_STATUS_REJECT_TEMP) {
        error = mlme_process_timeout_interval_elements(ni, ie_data, ie_length);
    }

    /* indicate linkspeed */
     mlme_get_linkrate(ni, &rxlinkspeed, &txlinkspeed);
     IEEE80211_DELIVER_EVENT_LINK_SPEED(vap, rxlinkspeed, txlinkspeed);

    /* Association complete (success or failure) */
    switch (mlme_request_type) {
    case MLME_REQ_ASSOC:
        IEEE80211_DELIVER_EVENT_MLME_ASSOC_COMPLETE(vap, status_code, aid, wbuf);
        break;

    case MLME_REQ_REASSOC:
        IEEE80211_DELIVER_EVENT_MLME_REASSOC_COMPLETE(vap, status_code, aid, wbuf);
        break;

    default:
        break;
    }
}

/* Send association or reassociation request */
static int mlme_assoc_reassoc_request(wlan_if_t vaphandle, int reassoc, u_int8_t *prev_bssid, u_int32_t timeout)
{
    struct ieee80211vap           *vap = vaphandle;
    struct ieee80211_mlme_priv    *mlme_priv = vap->iv_mlme_priv;
    struct ieee80211_node         *ni = vap->iv_bss; /* bss node */
    int                           error = 0;

    IEEE80211_DPRINTF(vap, IEEE80211_MSG_MLME, "%s\n", __func__);

    ASSERT(mlme_priv->im_request_type == MLME_REQ_NONE);
    mlme_priv->im_request_type = reassoc ? MLME_REQ_REASSOC : MLME_REQ_ASSOC;

    /* Set the timeout timer for association failure case */
    OS_SET_TIMER(&mlme_priv->im_timeout_timer, timeout);

    /* Transmit frame */
    error = ieee80211_send_assoc(ni, reassoc, prev_bssid);
    if (error) {
        IEEE80211_DPRINTF(vap, IEEE80211_MSG_MLME, "%s: ieee80211_send_assoc error %d (0x%08X)\n", 
            __func__, error, error);
        goto err;
    }

    return error;

err:
    mlme_priv->im_request_type = MLME_REQ_NONE;
    OS_CANCEL_TIMER(&mlme_priv->im_timeout_timer);
    return error;
}

/*
 * Process information elements from association response frame.
 * This includes rate negociation, wmm parameter updating and etc.
 */
static int mlme_process_asresp_elements(struct ieee80211_node *ni,
                                        u_int8_t              *frm,
                                        u_int32_t             ie_len)
{
    struct ieee80211vap          *vap = ni->ni_vap;
    struct ieee80211_rsnparms    *rsn = &vap->iv_rsn;
    struct ieee80211com          *ic = ni->ni_ic;
    u_int8_t                     *efrm = frm + ie_len;
    u_int8_t                     *rates, *xrates, *wme, *htcap, *tspecie, *athextcap, *extcap, *vhtcap, *vhtop;
    u_int8_t                     qosinfo;
    int                          ht_rates_allowed;

    ASSERT((vap->iv_opmode != IEEE80211_M_STA) || (ni == vap->iv_bss));
    
    rates = xrates = wme = htcap = tspecie = athextcap = extcap = vhtcap = vhtop = NULL;

    while (frm < efrm) {
        switch (*frm) {
        case IEEE80211_ELEMID_RATES:
            rates = frm;
            break;
        case IEEE80211_ELEMID_XRATES:
            xrates = frm;
            break;
        case IEEE80211_ELEMID_XCAPS:
            extcap = frm;
            break;
        case IEEE80211_ELEMID_HTCAP_ANA:
            htcap = (u_int8_t *)&((struct ieee80211_ie_htcap *)frm)->hc_ie;
            break;
       case IEEE80211_ELEMID_VHTCAP:
            if(vhtcap == NULL) {
                vhtcap = (u_int8_t *)(struct ieee80211_ie_vhtcap *)frm;
            }
            break;
       case IEEE80211_ELEMID_VHTOP:
            if(vhtop == NULL) {
                vhtop = (u_int8_t *)(struct ieee80211_ie_vhtop *)frm;
            }
            break;

        case IEEE80211_ELEMID_OP_MODE_NOTIFY:
            /* We can ignore this because the info we need is available in VHTCAP and VHTOP */
        break;

        case IEEE80211_ELEMID_VENDOR:
            if (iswmeoui(frm, WME_PARAM_OUI_SUBTYPE))
                wme = frm;
            else if (iswmeoui(frm, WME_INFO_OUI_SUBTYPE))
                wme = frm;
            else if (iswmeoui(frm, WME_TSPEC_OUI_SUBTYPE))
                tspecie = frm;
            else if(ishtcap(frm)) {
                if (htcap == NULL) {
                    htcap = (u_int8_t *)&((struct vendor_ie_htcap *)frm)->hc_ie;
                }
            }
            else if (isatheros_extcap_oui(frm))
                 athextcap = frm;

            else if (isinterop_vht(frm) && !(vhtcap) && 
                        ieee80211vap_11ng_vht_interopallowed(vap)) {
                 /* Vht cap & Op location in Vendor specific Vht IE */
                 vhtcap = (u_int8_t *)(struct ieee80211_ie_vhtcap *) (frm + 7);
                 vhtop = (u_int8_t *)(struct ieee80211_ie_vhtop *) (frm + 21);
                 ni->ni_flags |= IEEE80211_NODE_11NG_VHT_INTEROP_AMSDU_DISABLE;                 
            }
            break;
#if UMAC_SUPPORT_WNM
        case IEEE80211_ELEMID_BSSMAX_IDLE_PERIOD:
			ieee80211_wnm_parse_bssmax_ie(ni, frm);
            break;
        case IEEE80211_ELEMID_TIM_BCAST_RESPONSE:
            ieee80211_parse_timresp_ie(frm, frm + frm[1] + 2, ni);
            break;
#endif
        }
        frm += frm[1] + 2;
    }

    if (!rates || (rates[1] > IEEE80211_RATE_MAXSIZE)) {
        /* XXX: msg + stats */
        return -EINVAL;
    }

    if (!ieee80211_setup_rates(ni, rates, xrates,
                                 IEEE80211_F_DOSORT | IEEE80211_F_DOFRATE |
                                 IEEE80211_F_DOXSECT)) {
        printk("%s: association failed (rate set mismatch)\n", __func__);
        return -EINVAL;
    }

    /* 
     * U-APSD is enabled/disabled per-association.
     */
    if (wme != NULL) {
        /* Parse IE according to subtype */
        if (iswmeparam(wme)) {
            /* Association is QOS-enabled */
            ieee80211node_set_flag(ni, IEEE80211_NODE_QOS);

            if (vap->iv_opmode != IEEE80211_M_BTAMP &&
                ieee80211_parse_wmeparams(vap, wme, &qosinfo, 1) >= 0) {

                /* Check if association is UAPSD-enabled */
                if (qosinfo & WME_CAPINFO_UAPSD_EN) {
                    ieee80211node_set_flag(ni, IEEE80211_NODE_UAPSD);
                }
                else {
                    ieee80211node_clear_flag(ni, IEEE80211_NODE_UAPSD);
                }

                ieee80211_wme_updateparams(vap);
            }
        }
        else {
            /*** QOS requires WME Param */ 
            ieee80211node_clear_flag(ni, IEEE80211_NODE_QOS);

            if (ieee80211_parse_wmeinfo(vap, wme, &qosinfo) >= 0) {
                /* Check if association is UAPSD-enabled */
                if (qosinfo & WME_CAPINFO_UAPSD_EN) {
                    ieee80211node_set_flag(ni, IEEE80211_NODE_UAPSD);
                }
                else {
                    ieee80211node_clear_flag(ni, IEEE80211_NODE_UAPSD);
                }

                ieee80211_wme_updateinfo(vap);
            }
        }
    }
    else {
        ieee80211node_clear_flag(ni, IEEE80211_NODE_QOS);
        ieee80211node_clear_flag(ni, IEEE80211_NODE_UAPSD);
    }

    if ((tspecie != NULL) &&
        (ieee80211_parse_tspecparams(vap, tspecie) >= 0)) {
        /* store the tspec */
    }

    if (athextcap != NULL)
        ieee80211_process_athextcap_ie(ni, athextcap);

    if (extcap != NULL)
        ieee80211_process_extcap_ie(ni, extcap);

    /*
     * With WEP and TKIP encryption algorithms:
     * Disable aggregation if IEEE80211_NODE_WEPTKIPAGGR is not set.
     * Disable 11n if IEEE80211_FEXT_WEP_TKIP_HTRATE is not set.
     */
    ht_rates_allowed = (IEEE80211_IS_CHAN_11AC(ic->ic_curchan) ||
                         IEEE80211_IS_CHAN_11N(ic->ic_curchan));

    if (IEEE80211_VAP_IS_PRIVACY_ENABLED(vap) && 
        (RSN_CIPHER_IS_WEP(rsn) ||
        (RSN_CIPHER_IS_TKIP(rsn) && !RSN_CIPHER_IS_CCMP(rsn)))) {
        ieee80211node_set_flag(ni, IEEE80211_NODE_WEPTKIP);
        if (ieee80211_ic_wep_tkip_htrate_is_set(ic)) {
            if (!ieee80211_has_weptkipaggr(ni))
                ieee80211node_set_flag(ni, IEEE80211_NODE_NOAMPDU);
        } else {
            ht_rates_allowed = 0;
        }
    }

    if (IEEE80211_IS_CHAN_5GHZ(ic->ic_curchan)) {
        ni->ni_phymode = IEEE80211_MODE_11A;
    } else if (( xrates != NULL)  &&
             IEEE80211_SUPPORT_PHY_MODE(ic, IEEE80211_MODE_11G)) {
        ni->ni_phymode = IEEE80211_MODE_11G;
    } else {
        ni->ni_phymode = IEEE80211_MODE_11B;
    }


    /*
     * Channel width and Nss will get adjusted with HT parse and VHT parse
     * if those modes are enabled
     */
    ni->ni_chwidth = IEEE80211_CWM_WIDTH20;
    ni->ni_streams = 1;
    /* 11n - HT rates not allowed using WEP and TKIP */
    if ((htcap != NULL) && (ht_rates_allowed)) {
        /* record capabilities, mark node as capable of HT */
        if(!ieee80211_parse_htcap(ni, htcap)) {
            printk("%s: association failed (Rx MCS  set mismatch)\n", __func__);
            return -EINVAL;
        }

        if (!ieee80211_setup_ht_rates(ni, htcap,
            IEEE80211_F_DOFRATE | IEEE80211_F_DOXSECT |
            IEEE80211_F_DOBRS)) {
            printk("%s: association failed (rate set mismatch)\n", __func__);
            return -EINVAL;
        }
#ifdef ATH_SUPPORT_TxBF
        // set keycache for txbf after sta associated successfully.
        if ( ni->ni_explicit_compbf || ni->ni_explicit_noncompbf || ni->ni_implicit_bf){
            struct ieee80211com     *ic = vap->iv_ic;
        
            ieee80211_set_TxBF_keycache(ic,ni);
            ni->ni_bf_update_cv = 1;
            ni->ni_allow_cv_update = 1;
        }
#endif
    } else {
        /*
         * Flush any state from a previous association.
         */
        ieee80211node_clear_flag(ni, IEEE80211_NODE_HT);
        IEEE80211_NODE_CLEAR_HTCAP(ni);
    }

      /* Add vht cap for 2.4G mode, if 256QAM is enabled */
    if ((IEEE80211_IS_CHAN_11AC(vap->iv_bsschan) || IEEE80211_IS_CHAN_11NG(vap->iv_bsschan)) &&
                           ieee80211vap_vhtallowed(vap)) {

        if (vhtcap != NULL) {
            /* record capabilities, mark node as capable of VHT */
            ieee80211_parse_vhtcap(ni, vhtcap);

            if (!ieee80211_setup_vht_rates(ni, vhtcap,
                IEEE80211_F_DOFRATE | IEEE80211_F_DOXSECT | IEEE80211_F_DOBRS)) {
                printk("%s: association failed (vht rate set mismatch)\n", __func__);
                return -EINVAL;
            }

            if (vhtop != NULL) {
                ieee80211_parse_vhtop(ni, vhtop);
            }


        } else {
            /*
             * Flush any state from a previous association.
             */
            ni->ni_flags &= ~IEEE80211_NODE_VHT;
            ni->ni_vhtcap = 0;
        }

    } else {
        /*
         * Flush any state from a previous association.
         */
        ni->ni_flags &= ~IEEE80211_NODE_VHT;
        ni->ni_vhtcap = 0;
    }

    /* 
     * ieee80211_parse_vhtop would hav set the channel width based on APs operating mode/channel.
     * if vap is forced to operate in a different lower mode than what AP is opearing,
     *  then set the channel width based on  the forced channel/phy mode .
     */
    if (vap->iv_des_mode != IEEE80211_MODE_AUTO) {
        switch(ieee80211_chan2mode(ni->ni_chan)) {
        case IEEE80211_MODE_11A          : 
        case IEEE80211_MODE_11B          :
        case IEEE80211_MODE_11G          : 
        case IEEE80211_MODE_11NA_HT20    : 
        case IEEE80211_MODE_11NG_HT20    :  
        case IEEE80211_MODE_11AC_VHT20   :  
            ni->ni_chwidth = IEEE80211_CWM_WIDTH20;
            break;
        case IEEE80211_MODE_11NA_HT40PLUS:  
        case IEEE80211_MODE_11NA_HT40MINUS:
        case IEEE80211_MODE_11NG_HT40PLUS :
        case IEEE80211_MODE_11NG_HT40MINUS:
        case IEEE80211_MODE_11AC_VHT40PLUS:
        case IEEE80211_MODE_11AC_VHT40MINUS:
            if(ni->ni_chwidth > IEEE80211_CWM_WIDTH40) {
                ni->ni_chwidth = IEEE80211_CWM_WIDTH40;
            }
            break;
        case IEEE80211_MODE_11AC_VHT80:
            if(ni->ni_chwidth > IEEE80211_CWM_WIDTH80) {
                ni->ni_chwidth = IEEE80211_CWM_WIDTH80;
            }
            break;
        default :
            break;
            
        }
    }
    /* Update the PHY mode */
    ieee80211_update_ht_vht_phymode(ic, ni);

    return EOK;
}

/*
 * Process Timeout Interval information elements from association response frame.
 * This includes an (re)association is failed with 802.11 reason code 30 .
 */
static int mlme_process_timeout_interval_elements(struct ieee80211_node *ni,
                                        u_int8_t              *frm,
                                        u_int32_t             ie_len)
{
    struct ieee80211vap          *vap = ni->ni_vap;
    u_int8_t                     *efrm = frm + ie_len;
    u_int8_t                     *comebacktiem;

    ASSERT((vap->iv_opmode != IEEE80211_M_STA) || (ni == vap->iv_bss));
    
    comebacktiem = NULL;

    while (frm < efrm) {
        if (IEEE80211_ELEMID_TIMEOUT_INTERVAL == *frm) {
            comebacktiem = frm;
            break;
        }
        frm += frm[1] + 2;
    }

    if ((comebacktiem != NULL) &&
        (ieee80211_parse_timeieparams(vap, comebacktiem) >= 0)) {
        /* store the tie info */
    }

    return EOK;
}

static unsigned long
ieee80211_get_current_assoc_comeback_time(struct ieee80211vap *vap)
{
    unsigned long assoc_comeback_time = vap->iv_assoc_comeback_time;
    ASSERT(assoc_comeback_time != 0);

	return assoc_comeback_time;
}

ieee80211_cipher_type 
wlan_get_current_assoc_comeback_time(wlan_if_t vaphandle)
{
    return ieee80211_get_current_assoc_comeback_time(vaphandle);
}

static u_int32_t get_max_phyrate(struct ieee80211com *ic)
{
    u_int32_t    linkspeed; /* bits/sec */
    u_int8_t     tx_streams = ieee80211_getstreams(ic, ic->ic_tx_chainmask);

    /* 11n card, report highest supported HT rate. */
    switch (tx_streams) {
    default:
        /* Default to single stream */
    case 1:
        linkspeed = 150000000;
        break;
    case 2:
        linkspeed = 300000000;
        break;
    case 3:
        linkspeed = 450000000;
        break;
    case 4:
        linkspeed = 600000000;
        break;        
    }

    return linkspeed;
}
static void mlme_calculate_11n_connection_speed(struct ieee80211_node *ni, u_int32_t* rxlinkspeed, u_int32_t* txlinkspeed)
{
    struct ieee80211com    *ic = ni->ni_ic;
    u_int32_t              linkspeed; /* bits/sec */
    u_int8_t     tx_streams = ieee80211_getstreams(ic, ic->ic_tx_chainmask);
    u_int8_t     rx_streams = ieee80211_getstreams(ic, ic->ic_rx_chainmask);

    linkspeed = ic->ic_get_maxphyrate(ic, ni) * 1000;

    if (linkspeed == 0) {
        *txlinkspeed = get_max_phyrate(ic);
        *rxlinkspeed = *txlinkspeed;
    } else {
        *txlinkspeed = linkspeed;

        ASSERT(tx_streams <= rx_streams);

        /*
         * WAR for HB95 (1x1) low DownLink throughput issue. Report higher rx link speed to increase TCP RWIN size.
         * For 1x1 and 1x2, we report the rxlinkspeed as double of the txlinkspeed.
         */

        if((tx_streams == 1) && (rx_streams == 1)) {
            *rxlinkspeed = linkspeed * 2;
        } else {
            *rxlinkspeed = (linkspeed * rx_streams)/tx_streams;
            ASSERT(rx_streams <= 4);
        }
    }
}

void
mlme_get_linkrate(struct ieee80211_node *ni, u_int32_t* rxlinkspeed, u_int32_t* txlinkspeed)
{
    struct ieee80211vap    *vap = ni->ni_vap;
    struct ieee80211com    *ic = ni->ni_ic;
    int                    ht_rates_allowed;
    u_int8_t               rate;

    /* Check if connected to BSS */
    if ((ni->ni_htrates.rs_nrates == 0) && (ni->ni_rates.rs_nrates == 0)) {

        /* Not connected */

        if (ic->ic_caps & IEEE80211_C_HT) {
            *txlinkspeed = get_max_phyrate(ic);
            *rxlinkspeed = *txlinkspeed;
        } else {
            struct ieee80211_rateset *rs;

            /* Legacy card, report highest rate */
            rs = &vap->iv_op_rates[ieee80211_chan2mode(ic->ic_curchan)];
            rate = rs->rs_rates[rs->rs_nrates - 1];
            *txlinkspeed = mlme_dot11rate_to_bps(rate & IEEE80211_RATE_VAL);
            *rxlinkspeed = *txlinkspeed;
        }
    } else {

        /* Connected */
        /*
         * With WEP and TKIP encryption algorithms:
         * Disable 11n if IEEE80211_FEXT_WEP_TKIP_HTRATE is not set.
         */
        ht_rates_allowed = ((IEEE80211_IS_CHAN_11AC(ic->ic_curchan) ||
                            IEEE80211_IS_CHAN_11N(ic->ic_curchan)) && 
                            ieee80211vap_htallowed(vap));

        if (ni->ni_htrates.rs_nrates &&
            ht_rates_allowed) {
            mlme_calculate_11n_connection_speed(ni, rxlinkspeed, txlinkspeed);
            if (ic->ic_reg_parm.indicateRxLinkSpeed)    *txlinkspeed = *rxlinkspeed;
        } else {
            /* get maximum rate from node rate set */
            rate = ni->ni_rates.rs_rates[ni->ni_rates.rs_nrates - 1] & IEEE80211_RATE_VAL;
            *txlinkspeed = mlme_dot11rate_to_bps(rate);
            *rxlinkspeed = *txlinkspeed;

        }
    }

    IEEE80211_DPRINTF(vap, IEEE80211_MSG_MLME, "%s: Rx Link speed Data rate: %d, Tx Link speed Data rate: %d\n", 
                      __func__, *rxlinkspeed, *txlinkspeed);
}

void mlme_recv_auth_sta(struct ieee80211_node *ni,
                        u_int16_t             algo, 
                        u_int16_t             seq, 
                        u_int16_t             status_code,
                        u_int8_t              *challenge, 
                        u_int8_t              challenge_length, 
                        wbuf_t                wbuf)
{
    struct ieee80211vap           *vap = ni->ni_vap;
    struct ieee80211_mlme_priv    *mlme_priv = vap->iv_mlme_priv;

    IEEE80211_DPRINTF(vap, IEEE80211_MSG_MLME, "%s\n", __func__);
    
    if (mlme_priv->im_request_type != MLME_REQ_AUTH) {
        IEEE80211_DPRINTF(vap, IEEE80211_MSG_MLME, "%s: invalid request type\n", 
            __func__,
            mlme_priv->im_request_type);
            
        return;
    }
    /* Validate algo */
    if (algo == IEEE80211_AUTH_ALG_SHARED) {
        if (!RSN_AUTH_IS_SHARED_KEY(&vap->iv_rsn)) {
            IEEE80211_DPRINTF(vap, IEEE80211_MSG_MLME, "%s: Received a shared auth response in open mode.\n", 
                __func__);
        }    
        else {
        /* Mark node as shared key authentication */
        RSN_RESET_AUTHMODE(&ni->ni_rsn);
        RSN_SET_AUTHMODE(&ni->ni_rsn, IEEE80211_AUTH_SHARED);
    }
    }
        
    /* Validate seq */
    if (seq != mlme_priv->im_expected_auth_seq_number) {
        IEEE80211_DPRINTF(vap, IEEE80211_MSG_MLME, "%s: Invalid seq %d,%d\n", 
            __func__,
            seq, mlme_priv->im_expected_auth_seq_number);
        return;
    }

    if (RSN_AUTH_IS_SHARED_KEY(&vap->iv_rsn) && 
        (seq == IEEE80211_AUTH_SHARED_CHALLENGE) &&
        (status_code == IEEE80211_STATUS_SUCCESS)) 
    {
        /* Send the challenge response authentication packet. 
         * We don't care if the send fails. If it does, the timeout routine will do
         * the necessary cleanup.
         */
        ieee80211_send_auth(ni, seq + 1, 0, challenge, challenge_length, NULL);
        mlme_priv->im_expected_auth_seq_number = IEEE80211_AUTH_SHARED_PASS;
        return;
    } 

    if (!OS_CANCEL_TIMER(&mlme_priv->im_timeout_timer)) {
        IEEE80211_DPRINTF(vap, IEEE80211_MSG_MLME, "%s: Timed-out already\n", __func__);
        return;
    }

    IEEE80211_DPRINTF(vap, IEEE80211_MSG_MLME, "%s: mlme_auth_complete\n", __func__);

    /* Request complete */
    mlme_priv->im_request_type = MLME_REQ_NONE;

    /* Authentication complete (success or failure) */
    IEEE80211_DELIVER_EVENT_MLME_AUTH_COMPLETE(vap, status_code);
}

void
mlme_sta_bmiss_ind(wlan_if_t vap)
{
    ieee80211_mlme_event   event;
    struct ieee80211_mlme_priv    *mlme_priv = vap->iv_mlme_priv;

    /* deliver MLME event */
    event.u.event_bmiss.cur_bmiss_count = vap->iv_bmiss_count;
    event.u.event_bmiss.max_bmiss_count = vap->iv_bmiss_count_for_reset;
    event.type = IEEE80211_MLME_EVENT_BEACON_MISS;
    ieee80211_mlme_deliver_event(mlme_priv, &event);

    /* indicate beacon miss */
    IEEE80211_DELIVER_EVENT_BEACON_MISS(vap);
}

void
ieee80211_vap_iter_beacon_miss(void *arg, wlan_if_t vap) 
{
    systime_t              tstamp;
    systime_t              last_link_time;
    systime_t              last_traffic_time;
    struct ieee80211com    *ic = vap->iv_ic;    
    struct ieee80211_mlme_priv    *mlme_priv = vap->iv_mlme_priv;
    ieee80211_mlme_event   event;

    IEEE80211_DPRINTF(vap, IEEE80211_MSG_SCAN,
                      "%s: %s iv_bmiss_count=%d reset=%d max=%d arg=%08p swbmiss=%d\n",
                      __func__,
                      (arg != NULL) ? "SW" : "HW",
                      vap->iv_bmiss_count,
                      vap->iv_bmiss_count_for_reset,
                      vap->iv_bmiss_count_max,
                      arg,
                      mlme_sta_swbmiss_active(vap));

    /*
     * Our handling is only meaningful for stations that are
     * associated; any other conditions else will be handled
     * through different means (e.g. the tx timeout on mgt frames).
     */
    if ((vap->iv_opmode != IEEE80211_M_STA) ||
        !ieee80211_vap_ready_is_set(vap)) {
        mlme_sta_swbmiss_timer_print_status(vap); /* print the status of sw bmiss */
        return;
    }

    /* 
     * ignore HW beacon miss and completely rely on SW beacon miss
     * if 1. HW beacon processing is _not_ in use AND
     *    2. SW beacon miss is enabled.
     */
    if ((arg == NULL) && (!wlan_is_hwbeaconproc_active(vap) &&
            mlme_sta_swbmiss_active(vap))) {
        return;
    }

    /* When swbmiss comes, we need to ignore it when Hardware Beacon Processing is on */
    if ((arg != NULL) && wlan_is_hwbeaconproc_active(vap))
    {
        return ;
    }

    /*
     * WAR for excessive beacon miss problem on SoC.
     * Consider a beacon miss only when we have two consecutive
     * beacon misses and there are no rx activities in between.
     *
     * Count beacon misses only if we gave the AP a chance by sending a
     * directed Probe Request.
     *
     * Don't do anything if we are scanning a foreign channel.
     * Trying to transmit a frame (Probe Request) during a channel change 
     * (which includes a channel reset) can cause a NMI due to invalid HW 
     * addresses. 
     * Trying to transmit a Probe Request while in a foreign channel 
     * wouldn't do us any good either.
     *
     * Query current time only after retrieving LastLinkTime. This avoids 
     * possible negative values if this routine is preempted by reception of 
     * a beacon or directed frame which would update the fields used to 
     * calculate LastLinkTime.
     */

    last_traffic_time = ieee80211_get_directed_frame_timestamp(vap);
    last_link_time = (vap->iv_last_beacon_time > last_traffic_time) ? 
        vap->iv_last_beacon_time : last_traffic_time;
    tstamp = OS_GET_TIMESTAMP();

    {
        IEEE80211_DPRINTF(vap, IEEE80211_MSG_SCAN,
            "%d.%03d | %s: count=%d probe=%d beacon:%lums directed:%lums data:%lums ap_frame:%lums traffic_ind:%lums\n",
            ((u_int32_t) CONVERT_SYSTEM_TIME_TO_MS(tstamp)) / 1000,
            ((u_int32_t) CONVERT_SYSTEM_TIME_TO_MS(tstamp)) % 1000,
            __func__, vap->iv_bmiss_count,
            ieee80211_scan_can_transmit(ic->ic_scanner),
            (u_int32_t) CONVERT_SYSTEM_TIME_TO_MS(tstamp - vap->iv_last_beacon_time),
            (u_int32_t) CONVERT_SYSTEM_TIME_TO_MS(tstamp - last_traffic_time),
            (u_int32_t) CONVERT_SYSTEM_TIME_TO_MS(tstamp - vap->iv_lastdata),
            (u_int32_t) CONVERT_SYSTEM_TIME_TO_MS(tstamp - vap->iv_last_ap_frame),
            (u_int32_t) CONVERT_SYSTEM_TIME_TO_MS(tstamp - vap->iv_last_traffic_indication));
    }

    /*
     * Do not count beacon misses received when we're off-channel, or
     * within IEEE80211_MINIMUM_BMISS_TIME ms of the last valid beacon.
     */ 
    if ((! ieee80211_scan_in_home_channel(ic->ic_scanner)) || 
        (CONVERT_SYSTEM_TIME_TO_MS(tstamp - last_link_time) < IEEE80211_MINIMUM_BMISS_TIME)) {
        mlme_sta_swbmiss_timer_start(vap); /* restart beacon miss timer */
        return;
    }

#if UMAC_SUPPORT_WNM
    /* Ignore bmiss in WNM-Sleep Mode */
    if (wlan_get_powersave(vap) == IEEE80211_PWRSAVE_WNM) {
        mlme_sta_swbmiss_timer_start(vap); /* restart beacon miss timer */
        return;
    }
#endif

    vap->iv_bmiss_count++;
    
  
    event.u.event_bmiss.cur_bmiss_count = vap->iv_bmiss_count;
    event.u.event_bmiss.max_bmiss_count = vap->iv_bmiss_count_for_reset;
    event.type = IEEE80211_MLME_EVENT_BEACON_MISS;

    ieee80211_mlme_deliver_event(mlme_priv,&event);

    if (vap->iv_bmiss_count < vap->iv_bmiss_count_for_reset) {
#ifdef ATH_SWRETRY
        /* Turn off the sw retry mechanism until we receive
         * any data frame or probe response for the BSS we are
         * associated to.
         */
        if (ic->ic_set_swretrystate)
            ic->ic_set_swretrystate(vap->iv_bss, FALSE);
#endif                        

        /*
         * It is possible that the hardware gets into
         * deaf mode. Reset the hardware to see if it can recover
         * from the condition.
         */

        /* indicate device error */
        IEEE80211_DPRINTF(vap, IEEE80211_MSG_SCAN, "%s: Beacon miss, do internal reset!!\n", __func__);
        IEEE80211_DELIVER_EVENT_DEVICE_ERROR(vap);
                
        mlme_sta_swbmiss_timer_start(vap); /* restart beacon miss timer */
        return;
    }

    /*  max bmiss count reached */

    vap->iv_bmiss_count = 0;    /* reset bmiss counter */

    IEEE80211_DPRINTF(vap, IEEE80211_MSG_ANY, "%s: Beacon miss, will indicate to OS!!\n", __func__);
    /* indicate beacon miss */
    IEEE80211_DELIVER_EVENT_BEACON_MISS(vap);
}


void mlme_sta_reset_bmiss(struct ieee80211vap *vap)
{
    struct ieee80211_mlme_priv    *mlme_priv = vap->iv_mlme_priv;
    ieee80211_mlme_event   event;

    vap->iv_bmiss_count = 0;    /* reset bmiss counter */

    event.type = IEEE80211_MLME_EVENT_BEACON_MISS_CLEAR;
    ieee80211_mlme_deliver_event(mlme_priv,&event);
}

void 
ieee80211_beacon_miss(struct ieee80211com *ic)
{
    /*
     * Do not ignore a beacon miss during scan now that the scan algorithm 
     * has frequent returns to the home channel,
     */

    if (IEEE80211_IS_RADAR_ENABLED(ic))
        return;

    wlan_iterate_vap_list(ic, ieee80211_vap_iter_beacon_miss, NULL);
}


void
ieee80211_vap_iter_notify_beacon_rssi(void *arg, wlan_if_t vap)
{
    IEEE80211_DPRINTF(vap, IEEE80211_MSG_SCAN, "%s\n", __func__);

    /*
     * Verify hw beacon processing is active.
     * Our handling is only meaningful for stations that are
     * associated; any other conditions else will be handled
     * through different means (e.g. the tx timeout on mgt frames).
     */
    if (!wlan_is_hwbeaconproc_active(vap) ||
        (vap->iv_opmode != IEEE80211_M_STA) ||
        !ieee80211_vap_ready_is_set(vap)) {
        return;
    }

    IEEE80211_DPRINTF(vap, IEEE80211_MSG_ANY,
            "%s: Beacon RSSI notification, will indicate to OS!!\n", __func__);

    /* indicate beacon rssi */
    IEEE80211_DELIVER_EVENT_BEACON_RSSI(vap);
}

void
ieee80211_notify_beacon_rssi(struct ieee80211com *ic)
{
    wlan_iterate_vap_list(ic, ieee80211_vap_iter_notify_beacon_rssi, NULL);
}

void ieee80211_inact_timeout_sta(struct ieee80211vap *vap)
{

    struct ieee80211_mlme_priv    *mlme_priv = vap->iv_mlme_priv;

    if (!mlme_priv->im_connection_up) {
        return;
    }
    /*
     * if there was an activity in the last IEEE80211_INACT_WAIT period.
     * then reset the counter.
     */
    if (CONVERT_SYSTEM_TIME_TO_MS(OS_GET_TIMESTAMP() - vap->iv_last_directed_frame) < (IEEE80211_INACT_WAIT * 1000)) {
        vap->iv_inact_count = (vap->iv_keep_alive_timeout + IEEE80211_INACT_WAIT -1)/IEEE80211_INACT_WAIT;
    }
    if (vap->iv_inact_count) {
        --vap->iv_inact_count;
        if ((vap->iv_inact_count == 0) && (vap->iv_pwrsave_sta)) {
            if (ieee80211_sta_power_send_keepalive(vap) == ENXIO) {
                ieee80211_send_nulldata(vap->iv_bss,false); 
            }
            if (vap->iv_keep_alive_timeout) {
                vap->iv_inact_count =  (vap->iv_keep_alive_timeout + IEEE80211_INACT_WAIT -1)/IEEE80211_INACT_WAIT;
            }
        }
    }
} 

void mlme_sta_vattach(struct ieee80211vap *vap)
{
#if (UMAC_SUPPORT_TDLS == 1) && (ATH_TDLS_AUTO_CONNECT == 1)
    struct ieee80211com           *ic = vap->iv_ic;
    ic->ic_tdls_path_select_attach(vap);
#endif

    mlme_sta_swbmiss_timer_attach(vap);
}

void mlme_sta_vdetach(struct ieee80211vap *vap)
{
#if (UMAC_SUPPORT_TDLS == 1) && (ATH_TDLS_AUTO_CONNECT == 1)
    struct ieee80211com           *ic = vap->iv_ic;
    ic->ic_tdls_path_select_detach(vap);
#endif

    mlme_sta_swbmiss_timer_detach(vap);
}


#if (UMAC_SUPPORT_TDLS == 1) && (ATH_TDLS_AUTO_CONNECT == 1)
static void mlme_sta_delete_tdls_node(struct ieee80211vap *vap)
{
    struct ieee80211com *ic = vap->iv_ic;
    struct ieee80211_node_table *nt = &ic->ic_sta;
    struct ieee80211_node *ni;
    rwlock_state_t lock_state;

#define IEEE80211_TDLS_DEL_NODE    1

    if ((ic->ic_tdls->tdls_enable) && (ic->ic_tdls_auto_enable)) {
        OS_RWLOCK_READ_LOCK(&nt->nt_nodelock, &lock_state);
        TAILQ_FOREACH(ni, &nt->nt_node, ni_list) {
            if ((ni->ni_flags & IEEE80211_NODE_TDLS) && (IEEE80211_IS_TDLS_SETUP_COMPLETE(ni))) {
                ieee80211_tdls_ioctl(vap, IEEE80211_TDLS_DEL_NODE, ni->ni_macaddr);
            }
        }
        OS_RWLOCK_READ_UNLOCK(&nt->nt_nodelock, &lock_state);
    }
}
#endif

void mlme_sta_connection_reset(struct ieee80211vap *vap)
{
    /*
     * connection has been reset.
     */
    mlme_sta_swbmiss_timer_stop(vap);

#if (UMAC_SUPPORT_TDLS == 1) && (ATH_TDLS_AUTO_CONNECT == 1)
    mlme_sta_delete_tdls_node(vap);
#endif
}

#endif /* UMAC_SUPPORT_STA */
