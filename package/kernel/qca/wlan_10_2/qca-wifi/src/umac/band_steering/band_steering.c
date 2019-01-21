/*
 * @@-COPYRIGHT-START-@@
 *
 * Copyright (c) 2014 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 *
 * @@-COPYRIGHT-END-@@
 */

/*
 Band steering module
*/

#include "band_steering_priv.h"
#include <ieee80211_ioctl.h>  /* for ieee80211req_athdbg */
#include <ieee80211_var.h>

#if ATH_BAND_STEERING

#if !UMAC_SUPPORT_ACL
#error "ACL support must be enabled when band steering is enabled"
#endif /* !UMAC_SUPPORT_ACL */

/* Number of seconds to wait for instaneous RSSI report */
#define INST_RSSI_TIMEOUT 5

/* forward declaration */
static OS_TIMER_FUNC(wlan_bsteering_chan_util_timeout_handler);
static void ieee80211_bsteering_reset_chan_utilization(ieee80211_bsteering_t bsteering);
static void ieee80211_bsteering_send_activity_change_event(struct ieee80211vap *vap,
                                                           const u_int8_t *mac_addr,
                                                           bool activity);
static void ieee80211_bsteering_send_rssi_measurement_event(struct ieee80211vap *vap,
                                                            const u_int8_t *mac_addr,
                                                            u_int8_t rssi, bool is_debug);
static OS_TIMER_FUNC(wlan_bsteering_inst_rssi_timeout_handler);

/**
 * @brief Verify that the band steering handle is valid within the
 *        struct ieee80211com provided.
 *
 * @param [in] ic  the handle to the radio where the band steering state
 *                 resides
 *
 * @return true if handle is valid; otherwise false
 */
static INLINE bool ieee80211_bsteering_is_valid(const struct ieee80211com *ic)
{
    return ic && ic->ic_bsteering;
}

/**
 * @brief Determine whether the band steering module is enabled or not.
 *
 * @param [in] ic  the handle to the radio where the band steering state
 *                 resides
 *
 * @return non-zero if it is enabled; otherwise 0
 */
static u_int8_t ieee80211_bsteering_is_enabled(const struct ieee80211com *ic)
{
    return ieee80211_bsteering_is_valid(ic) &&
        atomic_read(&ic->ic_bsteering->bs_enabled);
}

/**
 * @brief To set param to band steering modules 
 *
 * @param vap and ath diag structure 
 *
 * @return EOK for success EINVAL for error cases
 *         
 */

/**
 * @brief Determine whether the VAP handle is valid, has a valid band
 *        steering handle, is operating in a mode where band steering
 *        is relevant, and is not in the process of being deleted.
 *
 * @return true if the VAP is valid; otherwise false
 */
static  bool ieee80211_bsteering_is_vap_valid(const struct ieee80211vap *vap)
{
    /* Unfortunately these functions being used do not take a const pointer
       even though they do not modify the VAP. Thus, we have to cast away
       the const-ness in order to call them.*/
    struct ieee80211vap *nonconst_vap = (struct ieee80211vap *) vap;
    return vap && wlan_vap_get_opmode(nonconst_vap) == IEEE80211_M_HOSTAP &&
           !ieee80211_vap_deleted_is_set(nonconst_vap);
}

/**
 * @brief Determine whether the VAP has band steering enabled.
 *
 * This is currently limited to just validating that the VAP has a valid
 * band steering handle and that it is operating in the right mode (AP mode).
 * Eventually it will be extended to also check that band steering has been
 * enabled on the VAP as well. Currently band steering is only enabled/disabled
 * at the radio level.
 *
 * @param [in] vap  the VAP to check
 *
 * @return true if the VAP is valid and has band steering enabled; otherwise
 *         false
 */
static INLINE bool ieee80211_bsteering_is_vap_enabled(const struct ieee80211vap *vap)
{
    /* Eventually this should also check that band steering has been enabled
       on the VAP*/
    return ieee80211_bsteering_is_vap_valid(vap);
}

/**
 * @brief Determine whether an ioctl request is valid or not, along with the
 *        associated parameters.
 *
 * @param [in] vap  the VAP on which the ioctl was made
 * @param [in] req  the parameters provided in the ioctl
 *
 * @return true if all parameters are valid; otherwise false
 */
static bool ieee80211_bsteering_is_req_valid(const struct ieee80211vap *vap,
                                                    const struct ieee80211req_athdbg *req)
{
    return ieee80211_bsteering_is_vap_valid(vap) &&
        ieee80211_bsteering_is_valid(vap->iv_ic) && NULL != req;
}

int wlan_bsteering_set_params(struct ieee80211vap *vap,
                              const struct ieee80211req_athdbg *req)
{
    ieee80211_bsteering_t bsteering = NULL;
    int retval = -EINVAL;
    ieee80211_bsteering_lmac_param_t lmac_params;

    /* Sanity check the values provided. If any are invalid, error out
       with EINVAL (per the default retval).*/

    if (!ieee80211_bsteering_is_req_valid(vap, req) ||
        !req->data.bsteering_param.inactivity_check_period ||
        !req->data.bsteering_param.utilization_sample_period ||
        !req->data.bsteering_param.utilization_average_num_samples ||
        (req->data.bsteering_param.inactivity_timeout_normal <=
         req->data.bsteering_param.inactivity_check_period) ||
        (req->data.bsteering_param.inactivity_timeout_overload <=
         req->data.bsteering_param.inactivity_check_period)) {
        return -EINVAL;
    }

    bsteering = vap->iv_ic->ic_bsteering;

    do {
        spin_lock_bh(&bsteering->bs_lock);
        if (ieee80211_bsteering_is_enabled(vap->iv_ic)) {
            retval = -EBUSY;
            break;
        }

        bsteering->bs_config_params = req->data.bsteering_param;

        lmac_params.inactivity_check_period =
            bsteering->bs_config_params.inactivity_check_period;
        lmac_params.inactivity_threshold_normal =
            bsteering->bs_config_params.inactivity_timeout_normal /
            bsteering->bs_config_params.inactivity_check_period;
        lmac_params.inactivity_threshold_overload =
            bsteering->bs_config_params.inactivity_timeout_overload /
            bsteering->bs_config_params.inactivity_check_period;

        retval = EOK;
    } while (0);

    if (retval == EOK) {
        /* Note: To avoid a possible deadlock, the lower layer should never
           call a band steering function that grabs the band steering
           spinlock while holding the peer lock.*/
        retval = vap->iv_ic->ic_bs_set_params(vap->iv_ic, &lmac_params) ?
            EOK : -EINVAL;
    }
    spin_unlock_bh(&bsteering->bs_lock);
    return retval;
}

/**
 * @brief get band steering paramters from user space
 *
 * @param 
 *
 * @return EOK for success and EINVAL for error cases
 *         
 */
int wlan_bsteering_get_params(const struct ieee80211vap *vap,
                              struct ieee80211req_athdbg *req)
{
    ieee80211_bsteering_t bsteering = NULL;

    if (!ieee80211_bsteering_is_req_valid(vap, req)) {
        return -EINVAL;
    }

    bsteering = vap->iv_ic->ic_bsteering; 
    spin_lock_bh(&bsteering->bs_lock);
    req->data.bsteering_param = bsteering->bs_config_params;
    spin_unlock_bh(&bsteering->bs_lock);

    return EOK;
}

/**
 * @brief 
 *
 * @param 
 *
 * @return 
 *         
 */
int wlan_bsteering_set_dbg_params(struct ieee80211vap *vap,
                                  const struct ieee80211req_athdbg *req)
{
    ieee80211_bsteering_t bsteering = NULL;

    if (!ieee80211_bsteering_is_req_valid(vap, req)) {
        return -EINVAL;
    }

    bsteering = vap->iv_ic->ic_bsteering;

    spin_lock_bh(&bsteering->bs_lock);

    /* These parameters can be changed while band steering is enabled,
       hence no safety check here that it is disabled.*/
    bsteering->bs_dbg_config_params = req->data.bsteering_dbg_param;

    spin_unlock_bh(&bsteering->bs_lock);
    return EOK;
}
/**
 * @brief 
 *
 * @param 
 *
 * @return 
 *         
 */
int wlan_bsteering_get_dbg_params(const struct ieee80211vap *vap,
                                  struct ieee80211req_athdbg *req)
{
    ieee80211_bsteering_t bsteering = NULL;

    if (!ieee80211_bsteering_is_req_valid(vap, req)) {
        return -EINVAL;
    }

    bsteering = vap->iv_ic->ic_bsteering;

    spin_lock_bh(&bsteering->bs_lock);
    req->data.bsteering_dbg_param = bsteering->bs_dbg_config_params;
    spin_unlock_bh(&bsteering->bs_lock);

    return EOK;
}
/**
 * @brief set inactivity overload threshold
 *
 * @param 
 *
 * @return 
 *         
 */
int wlan_bsteering_set_overload(struct ieee80211vap *vap,
                                const struct ieee80211req_athdbg *req)
{
    ieee80211_bsteering_t bsteering = NULL;

    if (!ieee80211_bsteering_is_req_valid(vap, req)) {
        return -EINVAL;
    }

    bsteering = vap->iv_ic->ic_bsteering;
    spin_lock_bh(&bsteering->bs_lock);
    bsteering->bs_vap_overload = req->data.bsteering_overload ? true : false;

    vap->iv_ic->ic_bs_set_overload(vap->iv_ic, bsteering->bs_vap_overload);

    spin_unlock_bh(&bsteering->bs_lock);

    return EOK;
}

/**
 * @brief get inactivity threshold
 *
 * @param 
 *
 * @return 
 *         
 */

int wlan_bsteering_get_overload(const struct ieee80211vap *vap,
                                struct ieee80211req_athdbg *req)
{
    ieee80211_bsteering_t bsteering = NULL;

    if (!ieee80211_bsteering_is_req_valid(vap, req)) {
        return -EINVAL;
    }

    bsteering = vap->iv_ic->ic_bsteering;

    spin_lock_bh(&bsteering->bs_lock);
    req->data.bsteering_overload = bsteering->bs_vap_overload ? 1 : 0;
    spin_unlock_bh(&bsteering->bs_lock);

    return EOK;
}

/**
 * @brief get rssi for particualr node , user space is using this api
 * @param 
 * @return rssi for particular node , or EINVAL for invalid values
 *         
 */

int wlan_bsteering_trigger_rssi_measurement(struct ieee80211vap *vap,
                                            struct ieee80211req_athdbg *req)
{
    struct ieee80211com *ic = NULL;
    ieee80211_bsteering_t bsteering = NULL;
    struct ieee80211_node *ni = NULL;
    int retVal = EOK;

    if (!ieee80211_bsteering_is_req_valid(vap, req)) {
        return -EINVAL;
    }

    ic = vap->iv_ic;
    bsteering = ic->ic_bsteering;

    ni = ieee80211_find_node(&ic->ic_sta, req->dstmac);
    if (!ni) {
        printk("%s: Requested STA %02x:%02x:%02x:%02x:%02x:%02x is not "
               "associated\n", __func__, req->dstmac[0], req->dstmac[1],
               req->dstmac[2], req->dstmac[3], req->dstmac[4], req->dstmac[5]);
        return -EINVAL;
    }

    spin_lock_bh(&bsteering->bs_lock);
    do {
        if (!ieee80211_bsteering_is_enabled(ic)) {
            printk("%s: Band steering is not enabled when measuring RSSI for "
                   "STA %02x:%02x:%02x:%02x:%02x:%02x\n",
                   __func__, req->dstmac[0], req->dstmac[1], req->dstmac[2],
                   req->dstmac[3], req->dstmac[4], req->dstmac[5]);
            retVal = -EINVAL;
            break;
        }

        if (bsteering->bs_inst_rssi_inprogress) {
            printk("%s: Ignore RSSI measurement request for STA "
                   "%02x:%02x:%02x:%02x:%02x:%02x, since another "
                   "RSSI measurement is in progress (count=%u) for "
                   "STA %02x:%02x:%02x:%02x:%02x:%02x\n",
                   __func__, req->dstmac[0], req->dstmac[1], req->dstmac[2],
                   req->dstmac[3], req->dstmac[4], req->dstmac[5],
                   bsteering->bs_inst_rssi_count,
                   bsteering->bs_inst_rssi_macaddr[0],
                   bsteering->bs_inst_rssi_macaddr[1],
                   bsteering->bs_inst_rssi_macaddr[2],
                   bsteering->bs_inst_rssi_macaddr[3],
                   bsteering->bs_inst_rssi_macaddr[4],
                   bsteering->bs_inst_rssi_macaddr[5]);
            break;
        }

        bsteering->bs_inst_rssi_num_samples = req->data.bsteering_rssi_num_samples;
        bsteering->bs_inst_rssi_count = 0;
        bsteering->bs_inst_rssi_err_count = 0;
        memcpy(bsteering->bs_inst_rssi_macaddr, req->dstmac, IEEE80211_ADDR_LEN);
        OS_SET_TIMER(&bsteering->bs_inst_rssi_timer, INST_RSSI_TIMEOUT * 1000);
        bsteering->bs_inst_rssi_inprogress = true;

        wlan_send_rssi(vap, ni->ni_macaddr);
    } while (0);

    spin_unlock_bh(&bsteering->bs_lock);

    ieee80211_free_node(ni);

    return retVal;
}

/**
 * @brief To enable band steering logic from user space api
 *
 * @param 
 *
 * @return successufl EOK unsuccessfull EINVAL
 *         
 */

int wlan_bsteering_enable(struct ieee80211vap *vap, const struct ieee80211req_athdbg *req)
{
    ieee80211_bsteering_t bsteering = NULL;
    int retval = -EINVAL;

    if (!ieee80211_bsteering_is_req_valid(vap, req)) {
        return -EINVAL;
    }

    bsteering = vap->iv_ic->ic_bsteering;

    do {
        spin_lock_bh(&bsteering->bs_lock);

        if (req->data.bsteering_enable) {  /* enable */
            /* Ensure it is not a double enable.*/
            if (ieee80211_bsteering_is_enabled(vap->iv_ic)) {
                retval = -EBUSY;
                break;
            }
            /* Sanity check to make sure valid config parameters have been set */
            if (!bsteering->bs_config_params.inactivity_check_period ||
                !bsteering->bs_config_params.utilization_sample_period ||
                !bsteering->bs_config_params.utilization_average_num_samples) {
                retval = -EINVAL;
                break;
            }

            /* Remember the VAP for later, as we need one to trigger the
               channel utilization. */
            bsteering->bs_iv = vap;

            ieee80211_bsteering_reset_chan_utilization(bsteering);

            OS_SET_TIMER(&bsteering->bs_chan_util_timer,
                         bsteering->bs_config_params.utilization_sample_period * 1000);
        } else {  /* disable */
            /* Ensure it is not a double disable.*/
            if (!ieee80211_bsteering_is_enabled(vap->iv_ic)) {
                retval = -EALREADY;
                break;
            }
            OS_CANCEL_TIMER(&bsteering->bs_chan_util_timer);
            OS_CANCEL_TIMER(&bsteering->bs_inst_rssi_timer);
        }

        retval = EOK;
    } while (0);

    if (retval == EOK) {
        /* Note: To avoid a possible deadlock, the lower layer should never
           call a band steering function that grabs the band steering
           spinlock while holding the peer lock.*/
        if (!vap->iv_ic->ic_bs_enable(vap->iv_ic, req->data.bsteering_enable) &&
            req->data.bsteering_enable) {
            return -EINVAL;
        } else {
            atomic_set(&bsteering->bs_enabled, req->data.bsteering_enable);
        }
    }
    spin_unlock_bh(&bsteering->bs_lock);
    return retval;
}

/**
 * @brief entry point for band steering logic.
 *
 * @param 
 *
 * @return 
 *         
 */

int ieee80211_bsteering_attach(struct ieee80211com *ic)
{
    ieee80211_bsteering_t bsteering = NULL;

    if (ic->ic_bsteering) {
        return -EINPROGRESS;
    }
    bsteering = (ieee80211_bsteering_t )
        OS_MALLOC(ic->ic_osdev, sizeof(struct ieee80211_bsteering), 0);
    if(NULL == bsteering) {
        return -ENOMEM;
    }

    OS_MEMZERO(bsteering,sizeof(struct ieee80211_bsteering));
    ic->ic_bsteering = bsteering;
    bsteering->bs_osdev = ic->ic_osdev;
    bsteering->bs_ic = ic;

    spin_lock_init(&bsteering->bs_lock);
    atomic_set(&bsteering->bs_enabled, false);
    bsteering->bs_chan_util_requested = false;

    OS_INIT_TIMER(bsteering->bs_osdev, &bsteering->bs_chan_util_timer,
                  wlan_bsteering_chan_util_timeout_handler, (void *) ic);

    OS_INIT_TIMER(bsteering->bs_osdev, &bsteering->bs_inst_rssi_timer,
                  wlan_bsteering_inst_rssi_timeout_handler, (void *) ic);

    printk("%s: Band steering initialized\n", __func__);
    return EOK;
}

/**
 * @brief detach function for band steering memory will be free
 *
 * @param ic 
 *
 * @return 
 *         
 */

int ieee80211_bsteering_detach(struct ieee80211com *ic)
{
    ieee80211_bsteering_t bsteering = ic->ic_bsteering;

    if(NULL == bsteering) {
        return EOK;
    }

    OS_FREE_TIMER(&bsteering->bs_chan_util_timer);
    OS_FREE_TIMER(&bsteering->bs_inst_rssi_timer);
    spin_lock_destroy(&bsteering->bs_lock);

    OS_FREE(bsteering);
    bsteering = NULL;
    printk("%s: Band steering terminated\n", __func__);
    return EOK;
}

/**
 * @brief to send inactive event per node to user space daemon
 *
 * @param 
 *
 * @return void
 *         
 */

void
ieee80211_bsteering_record_act_change(struct ieee80211com *ic,
                                      const u_int8_t *mac_addr,
                                      bool active)
{
    struct ieee80211_node *ni;

    if (!ieee80211_bsteering_is_enabled(ic) || !mac_addr) {
        return;
    }

    ni = ieee80211_find_node(&ic->ic_sta, mac_addr);

    if ((ni->ni_flags & IEEE80211_NODE_AUTH) &&
        ieee80211_bsteering_is_vap_enabled(ni->ni_vap)) {
        ieee80211_bsteering_send_activity_change_event(
                                                       ni->ni_vap, ni->ni_macaddr, active);        
    }

    ieee80211_free_node(ni);
}

/**
 * @brief Determine the band of operation for the VAP based on its mode.
 *
 * @param vap  the VAP for which to resolve the band
 *
 * @return  the band steering index or BSTEERING_INVALID if it could not be
 *          resolved
 */
static BSTEERING_BAND ieee80211_bsteering_resolve_band(struct ieee80211vap *vap)
{
    u_int32_t band_index;
    switch (vap->iv_cur_mode) {
    case IEEE80211_MODE_11A:
    case IEEE80211_MODE_TURBO_A:
    case IEEE80211_MODE_11NA_HT20:
    case IEEE80211_MODE_11NA_HT40PLUS:
    case IEEE80211_MODE_11NA_HT40MINUS:
    case IEEE80211_MODE_11NA_HT40:
    case IEEE80211_MODE_11AC_VHT20:
    case IEEE80211_MODE_11AC_VHT40PLUS:
    case IEEE80211_MODE_11AC_VHT40MINUS:
    case IEEE80211_MODE_11AC_VHT40:
    case IEEE80211_MODE_11AC_VHT80:
        band_index = BSTEERING_5G;
        break;
    case IEEE80211_MODE_11B:
    case IEEE80211_MODE_11G:
    case IEEE80211_MODE_TURBO_G:
    case IEEE80211_MODE_11NG_HT20:
    case IEEE80211_MODE_11NG_HT40PLUS:
    case IEEE80211_MODE_11NG_HT40MINUS:
    case IEEE80211_MODE_11NG_HT40:
        band_index = BSTEERING_24G;
        break;
    default:
        band_index = BSTEERING_INVALID;
        break;
    }

    return band_index;
}

/**
 * @brief To send probe request to daemon , it will beused to find out whether 
 * we have dual band capable clients in network
 *
 * @param 
 *
 * @return 
 *         
 */

void ieee80211_bsteering_send_probereq_event(struct ieee80211vap *vap,
                                             const u_int8_t *mac_addr,
                                             u_int8_t rssi)
{
    u_int32_t band_index;
    struct bs_probe_req_ind probe;

    if(!ieee80211_bsteering_is_vap_enabled(vap) ||
       !ieee80211_bsteering_is_enabled(vap->iv_ic)) {
        return;
    }

    band_index = ieee80211_bsteering_resolve_band(vap);

    OS_MEMCPY(probe.sender_addr, mac_addr, IEEE80211_ADDR_LEN);
    probe.rssi = rssi;
    IEEE80211_DELIVER_BSTEERING_EVENT(vap, ATH_EVENT_BSTEERING_PROBE_REQ,
                                      sizeof(probe),
                                      (const char *) &probe, band_index);
}

/**
 * @brief to intimate application about auth fail for particualr node
 *
 * @param 
 *
 * @return 
 *         
 */

void ieee80211_bsteering_send_auth_fail_event(struct ieee80211vap *vap, 
                                              const u_int8_t *mac_addr, 
                                              u_int8_t rssi)
{
    u_int32_t band_index;
    struct bs_auth_reject_ind auth;

    if(!ieee80211_bsteering_is_vap_enabled(vap) ||
       !ieee80211_bsteering_is_enabled(vap->iv_ic)) {
        return;
    }

    band_index = ieee80211_bsteering_resolve_band(vap);

    OS_MEMCPY(auth.client_addr, mac_addr, IEEE80211_ADDR_LEN);
    auth.rssi = rssi;
    IEEE80211_DELIVER_BSTEERING_EVENT(vap,
                                      ATH_EVENT_BSTEERING_TX_AUTH_FAIL,
                                      sizeof(auth),
                                      (const char *) &auth, band_index);
}

/**
 * @brief To intimate daemon successfull association of node.
 *
 * @param 
 *
 * @return 
 *         
 */

void ieee80211_bsteering_send_node_authorized_event(struct ieee80211vap *vap,
                                                    const u_int8_t *mac_addr)
{
    u_int32_t band_index;
    struct bs_node_authorized_ind auth;

    if(!ieee80211_bsteering_is_vap_enabled(vap) ||
       !ieee80211_bsteering_is_enabled(vap->iv_ic)) {
        return;
    }

    band_index = ieee80211_bsteering_resolve_band(vap);

    OS_MEMCPY(auth.client_addr, mac_addr, IEEE80211_ADDR_LEN);
    IEEE80211_DELIVER_BSTEERING_EVENT(vap,
                                      ATH_EVENT_BSTEERING_NODE_AUTHORIZED,
                                      sizeof(auth),
                                      (const char *) &auth, band_index);
}

/**
 * @brief Send an event to user space on activity status change
 *
 * @pre vap has already been checked and confirmed to be valid
 *
 * @param [in] vap  the VAP that the activity status change client associated to
 * @param [in] mac_addr  the MAC address of the client
 * @param [in] activity  flag indicating active or not
 */
static void ieee80211_bsteering_send_activity_change_event(struct ieee80211vap *vap,
                                                           const u_int8_t *mac_addr,
                                                           bool activity)
{
    u_int32_t band_index;
    struct bs_activity_change_ind ind;

    if(!ieee80211_bsteering_is_enabled(vap->iv_ic)) {
        return;
    }

    band_index = ieee80211_bsteering_resolve_band(vap);

    OS_MEMCPY(ind.client_addr, mac_addr, IEEE80211_ADDR_LEN);
    ind.activity = activity ? 1 : 0;

    IEEE80211_DELIVER_BSTEERING_EVENT(vap, ATH_EVENT_BSTEERING_CLIENT_ACTIVITY_CHANGE,
                                      sizeof(struct bs_activity_change_ind),
                                      (const char *) &ind, band_index);
}

/**
 * @brief Generate an event with the provided channel utilization measurement.
 *
 * @pre vap has already been checked and confirmed to be valid and band
 *      steering has been confirmed to be enabled
 *
 * @param [in] vap  the VAP that was used for the sampling
 * @param [in] chan_utilization  the utilization in percent
 * @param [in] is_debug  whether the log generated should be a debug event
 *                       or a regular event; debug events represent an
 *                       instantaneous measurement whereas normal (non-debug)
 *                       represent a filtered/averaged measurement
 */
static void ieee80211_bsteering_send_utilization_event(struct ieee80211vap *vap,
                                                       u_int8_t chan_utilization,
                                                       bool is_debug)
{
    u_int32_t band_index;
    struct bs_chan_utilization_ind ind;
    ATH_BSTEERING_EVENT event = ATH_EVENT_BSTEERING_CHAN_UTIL;

    band_index = ieee80211_bsteering_resolve_band(vap);

    ind.utilization = chan_utilization;

    if (is_debug) {
        event = ATH_EVENT_BSTEERING_DBG_CHAN_UTIL;
    }

    IEEE80211_DELIVER_BSTEERING_EVENT(vap, event, sizeof(ind),
                                      (const char *) &ind, band_index);
}

/**
 * @brief Send an event to user space on RSSI measurement crossed threshold
 *
 * @pre vap has already been checked and confirmed to be valid and band
 *      steering has confirmed to be enabled
 *
 * @param [in] vap  the VAP that the client whose RSSI is measured associated to
 * @param [in] mac_addr  the MAC address of the client
 * @param [in] rssi  the measured RSSI
 * @param [in] inact_xing  flag indicating if the RSSI crossed inactivity RSSI threshold.
 * @param [in] low_xing  flag indicating if the RSSI crossed low RSSI threshold
 */
static void ieee80211_bsteering_send_rssi_xing_event(struct ieee80211vap *vap,
                                                     const u_int8_t *mac_addr,
                                                     u_int8_t rssi,
                                                     BSTEERING_RSSI_XING_DIRECTION inact_xing,
                                                     BSTEERING_RSSI_XING_DIRECTION low_xing)
{
    u_int32_t band_index;
    struct bs_rssi_xing_threshold_ind ind;
    ATH_BSTEERING_EVENT event = ATH_EVENT_BSTEERING_CLIENT_RSSI_CROSSING;

    band_index = ieee80211_bsteering_resolve_band(vap);

    OS_MEMCPY(ind.client_addr, mac_addr, IEEE80211_ADDR_LEN);
    ind.rssi = rssi;
    ind.inact_rssi_xing = inact_xing;
    ind.low_rssi_xing = low_xing;


    IEEE80211_DELIVER_BSTEERING_EVENT(vap, event, sizeof(ind),
                                      (const char *) &ind, band_index);
}

/**
 * @brief Send an event to user space when requested RSSI measurement is available
 *
 * @pre vap has already been checked and confirmed to be valid and band
 *      steering has already been confirmed to be enabled
 *
 * @param [in] vap  the VAP that the client whose RSSI is measured associated to
 * @param [in] mac_addr  the MAC address of the client
 * @param [in] rssi  the measured RSSI
 * @param [in] is_debug  whether the log generated should be a debug event
 *                       or a regular event; debug events represent an
 *                       instantaneous (but averaged by firmware already) RSSI
 *                       measurement whereas normal (non-debug) represent the
 *                       RSSI measured by sending NDPs
 */
static void ieee80211_bsteering_send_rssi_measurement_event(struct ieee80211vap *vap,
                                                            const u_int8_t *mac_addr,
                                                            u_int8_t rssi,
                                                            bool is_debug)
{
    u_int32_t band_index;
    struct bs_rssi_measurement_ind ind;
    ATH_BSTEERING_EVENT event = ATH_EVENT_BSTEERING_CLIENT_RSSI_MEASUREMENT;

    band_index = ieee80211_bsteering_resolve_band(vap);

    OS_MEMCPY(ind.client_addr, mac_addr, IEEE80211_ADDR_LEN);
    ind.rssi = rssi;

    if (is_debug) {
        event = ATH_EVENT_BSTEERING_DBG_RSSI;
    }

    IEEE80211_DELIVER_BSTEERING_EVENT(vap, event, sizeof(ind),
                                      (const char *) &ind, band_index);
}

/**
 * @brief VAP iteration callback that measures the channel utilization
 *        on the provided VAP if it is the VAP that was used to enable
 *        band steering.
 *
 * If the VAP is not the one used to enable band steering, this does nothing.
 *
 * @note This must be called with the band steering lock held.
 *
 * @param [in] arg  the band steering handle
 * @param [in] vap  the current VAP being considered
 */
static void wlan_bsteering_measure_chan_util(void *arg, wlan_if_t vap)
{
    ieee80211_bsteering_t bsteering = (ieee80211_bsteering_t) arg;

    /* Check whether the VAP still exists and is in AP mode */
    if (vap == bsteering->bs_iv && ieee80211_bsteering_is_vap_valid(vap)) {
        u_int32_t band_index = ieee80211_bsteering_resolve_band(vap);

        /* This check that the VAP is ready should be sufficient to ensure we do
           not trigger a scan while in DFS wait state.
        
           Note that there is still a small possibility that the channel or state
           will change after we check this flag. I do not know how to avoid this
           at this time.*/
        if (band_index != BSTEERING_INVALID && ieee80211_vap_ready_is_set(vap) &&
            !bsteering->bs_chan_util_requested && vap->iv_bsschan) {
            /* Remember the channel so we can ignore any callbacks that are
               not for our channel. */
            u_int8_t chanlist[2] = { 0 };

            if (vap->iv_bsschan->ic_ieee != bsteering->bs_active_ieee_chan_num) {
                /* Channel changed, so invalidate everything. The utilization
                   is not expected to be correlated across channels. */
                ieee80211_bsteering_reset_chan_utilization(bsteering);
            }

            bsteering->bs_active_ieee_chan_num = vap->iv_bsschan->ic_ieee;

            chanlist[0] = bsteering->bs_active_ieee_chan_num;
            if (wlan_acs_set_user_chanlist(vap, chanlist) == EOK) {
                if (wlan_acs_start_scan_report(vap, 1, IEEE80211_START_ACS_REPORT,
                                               1) == EOK) {
                    bsteering->bs_chan_util_requested = true;
                } else {
                    printk("%s: Failed to start scan report on band %u; "
                           "will retry in next timer expiry\n",
                           __func__, band_index);
                }
            } else {
                printk("%s: Failed to set channel list on band %u; "
                       "will retry in next timer expiry\n",
                       __func__, band_index);
            }
        } else {
            printk("%s: Already waiting for utilization, VAP is not ready, "
                   "or bsschan is invalid on band %u: %p\n",
                   __func__, band_index, vap->iv_bsschan);
        }

        OS_SET_TIMER(&bsteering->bs_chan_util_timer,
                     bsteering->bs_config_params.utilization_sample_period * 1000);
    }
}


/**
 * @brief Timeout handler for periodic channel utilization measurements.
 *
 * This will trigger the utilization measurement (assuming one is not already
 * in progress) and reschedule the timer for the next measurement. The
 * assumption is that the measurement will be complete prior to the next
 * timer expiry, but if it is not, this is guarded against and the next
 * measurement will just be delayed.
 *
 * @param [in] arg  ieee80211com
 *
 */
static OS_TIMER_FUNC(wlan_bsteering_chan_util_timeout_handler)
{
    struct ieee80211com *ic;
    ieee80211_bsteering_t bsteering = NULL;
    OS_GET_TIMER_ARG(ic, struct ieee80211com *);

    if (!ieee80211_bsteering_is_valid(ic)) {
        return;
    }

    bsteering = ic->ic_bsteering;
    spin_lock_bh(&bsteering->bs_lock);
    if (!ieee80211_bsteering_is_enabled(ic)) {
        spin_unlock_bh(&bsteering->bs_lock);
        return;
    }
    wlan_iterate_vap_list(ic, wlan_bsteering_measure_chan_util, bsteering);

    spin_unlock_bh(&bsteering->bs_lock);
}

/**
 * @brief Reset the utilization measurements for the next round of samples.
 *
 * @pre the caller should be holding the band steering spinlock
 *
 * @param [in] bsteering  the object to use for the reset
 */
static void ieee80211_bsteering_reset_chan_utilization(ieee80211_bsteering_t bsteering)
{
    bsteering->bs_chan_util_samples_sum = 0;
    bsteering->bs_chan_util_num_samples = 0;
    bsteering->bs_chan_util_requested = false;
}

/**
 * @brief send channel utilization event for set channel 
 *  function is reporting handler to upper layer for acsreport
 *
 * @param 
 *
 * @return 
 *         
 */

void ieee80211_bsteering_record_utilization(struct ieee80211vap *vap,
                                            u_int ieee_chan_num,
                                            u_int32_t chan_utilization)
{
#define MAX_CHANNEL_UTILIZATION 100
    ieee80211_bsteering_t bsteering = NULL;

    if (!ieee80211_bsteering_is_vap_enabled(vap) ||
        !ieee80211_bsteering_is_valid(vap->iv_ic)) {
        return;
    }

    bsteering = vap->iv_ic->ic_bsteering;

    do {
        spin_lock_bh(&bsteering->bs_lock);

        if (!ieee80211_bsteering_is_enabled(vap->iv_ic) ||
            !bsteering->bs_chan_util_requested ||
            !bsteering->bs_iv || bsteering->bs_iv != vap) {
            break;
        }

        if (ieee_chan_num == bsteering->bs_active_ieee_chan_num) {
            /* We have sometimes seen a channel utilization value greater than
               100%. The current suspicion is that the ACS module did not
               complete the scan properly and thus when it calls back into band
               steering, it is only providing a raw channel clear count instead
               of the computed percentage. This is possible because both the
               intermediate result and the final value are stored in the same
               value and by the time the scan event handler is called
               indicating completion, ACS does not know whether the value is
               the intermediate or final result.
            
               By checking for this, we force a new measurement to be taken
               in the off chance that this occurs. The real fix will be of
               course to determine why this is happening in ACS. */
            if (chan_utilization <= MAX_CHANNEL_UTILIZATION) {
                bsteering->bs_chan_util_samples_sum += chan_utilization;
                bsteering->bs_chan_util_num_samples++;

                if (bsteering->bs_dbg_config_params.raw_chan_util_log_enable) {
                    ieee80211_bsteering_send_utilization_event(vap,
                                                               chan_utilization,
                                                               true /* isDebug */);
                }

                /* If we have reached our desired number of samples, generate an
                   event with the average.*/
                if (bsteering->bs_chan_util_num_samples ==
                    bsteering->bs_config_params.utilization_average_num_samples) {
                    u_int8_t average = bsteering->bs_chan_util_samples_sum /
                        bsteering->bs_chan_util_num_samples;
                    ieee80211_bsteering_send_utilization_event(vap, average,
                                                               false /* isDebug */);

                    ieee80211_bsteering_reset_chan_utilization(bsteering);
                }
            } else {
                printk("%s: Ignoring invalid utilization %u on channel %u\n",
                       __func__, chan_utilization, ieee_chan_num);
            }

            bsteering->bs_chan_util_requested = false;
        }
    } while (0);

    spin_unlock_bh(&bsteering->bs_lock);
#undef MAX_CHANNEL_UTILIZATION
}

/**
 * @brief To record instant rssi from ol layer , its  for null frame we have send through wmi command
 *
 * @param 
 *
 * @return 
 *         
 */

void ieee80211_bsteering_record_inst_rssi(struct ieee80211_node *ni, u_int8_t rssi)
{
    ieee80211_bsteering_t bsteering = NULL;
    bool generate_event = false;
    u_int8_t report_rssi = BSTEERING_INVALID_RSSI;

    if (!ni || !ieee80211_bsteering_is_vap_enabled(ni->ni_vap) ||
        !ieee80211_bsteering_is_valid(ni->ni_vap->iv_ic)) {
        return;
    }

    bsteering = ni->ni_vap->iv_ic->ic_bsteering;

    do {
        spin_lock_bh(&bsteering->bs_lock);
        if (!ieee80211_bsteering_is_enabled(ni->ni_vap->iv_ic)) {
            break;
        }

        if (!bsteering->bs_inst_rssi_inprogress ||
            memcmp(bsteering->bs_inst_rssi_macaddr, ni->ni_macaddr, IEEE80211_ADDR_LEN)) {
            /* The RSSI measurement is not for the one requested */
            break;
        }

        if (BSTEERING_INVALID_RSSI != rssi) {
            bsteering->bs_avg_inst_rssi =
                ((bsteering->bs_avg_inst_rssi * bsteering->bs_inst_rssi_count) + rssi) /
                (bsteering->bs_inst_rssi_count + 1);
            ++bsteering->bs_inst_rssi_count;
            if (bsteering->bs_inst_rssi_count >= bsteering->bs_inst_rssi_num_samples) {
                generate_event = true;
                report_rssi = bsteering->bs_avg_inst_rssi;
            }
        } else {
            ++bsteering->bs_inst_rssi_err_count;
            /* If we get twice as many failed samples as the number of samples
               requested, just give up and indicate a failure to measure the
               RSSI. */
            if (bsteering->bs_inst_rssi_err_count >= 2 * bsteering->bs_inst_rssi_num_samples) {
                generate_event = true;
            }
        }

        if (generate_event) {
            ieee80211_bsteering_send_rssi_measurement_event(ni->ni_vap, ni->ni_macaddr,
                                                            report_rssi, false /* is_debug */);
            OS_CANCEL_TIMER(&bsteering->bs_inst_rssi_timer);
            bsteering->bs_inst_rssi_inprogress = false;
            break;
        }

        /* More measurements are needed */
        wlan_send_rssi(ni->ni_vap, ni->ni_macaddr);

    } while (0);

    spin_unlock_bh(&bsteering->bs_lock);
}

/**
 * @brief to handle error code for rssi measurement from ol layer.
 *
 * @param 
 *
 * @return 
 *         
 */

void ieee80211_bsteering_record_inst_rssi_err(struct ieee80211_node *ni) 
{
    ieee80211_bsteering_record_inst_rssi(ni, BSTEERING_INVALID_RSSI);
}

/**
 * @brief to record rssi for node , current implementation is from ol layer only.
 *        value is reported by firmware through stats command.
 * @param 
 *
 * @return 
 *         
 */

void ieee80211_bsteering_record_rssi(struct ieee80211_node *ni, u_int8_t rssi)
{
    ieee80211_bsteering_t bsteering = NULL;
    u_int32_t inact_rssi_threshold, low_rssi_threshold;
    BSTEERING_RSSI_XING_DIRECTION inact_xing = BSTEERING_RSSI_UNCHANGED;
    BSTEERING_RSSI_XING_DIRECTION low_xing = BSTEERING_RSSI_UNCHANGED;

    if (!ni || !ieee80211_bsteering_is_vap_enabled(ni->ni_vap) ||
        !ieee80211_bsteering_is_valid(ni->ni_vap->iv_ic)) {
        return;
    }

    bsteering = ni->ni_vap->iv_ic->ic_bsteering;

    do {
        spin_lock_bh(&bsteering->bs_lock);

        if (!ieee80211_bsteering_is_enabled(ni->ni_vap->iv_ic)) {
            break;
        }

        if (bsteering->bs_dbg_config_params.raw_rssi_log_enable) {
            ieee80211_bsteering_send_rssi_measurement_event(ni->ni_vap, ni->ni_macaddr,
                                                            rssi, true /* is_debug */);
        }

        if (!ni->ni_bs_rssi) {
            /* First RSSI measurement */
            break;
        }

        if (ni->ni_vap->iv_ic->ic_node_isinact(ni)) {
            /* Check inactivity rssi threshold crossing */
            inact_rssi_threshold = bsteering->bs_config_params.inactive_rssi_crossing_threshold;
            if (rssi >= inact_rssi_threshold && ni->ni_bs_rssi < inact_rssi_threshold) {
                inact_xing = BSTEERING_RSSI_UP;
            } else if (rssi < inact_rssi_threshold && ni->ni_bs_rssi >= inact_rssi_threshold) {
                inact_xing = BSTEERING_RSSI_DOWN;
            }
        }

        /* Check low rssi thresold crossing */
        low_rssi_threshold = bsteering->bs_config_params.low_rssi_crossing_threshold;
        if (rssi >= low_rssi_threshold && ni->ni_bs_rssi < low_rssi_threshold) {
            low_xing = BSTEERING_RSSI_UP;
        } else if (rssi < low_rssi_threshold && ni->ni_bs_rssi >= low_rssi_threshold) {
            low_xing = BSTEERING_RSSI_DOWN;
        }
    } while (0);

    if (inact_xing != BSTEERING_RSSI_UNCHANGED || low_xing != BSTEERING_RSSI_UNCHANGED) {
        ieee80211_bsteering_send_rssi_xing_event(ni->ni_vap, ni->ni_macaddr, rssi,
                                                 inact_xing, low_xing);
    }

    ni->ni_bs_rssi = rssi;

    spin_unlock_bh(&bsteering->bs_lock);
}
/**
 * @brief Timeout handler for inst RSSI measurement timer.
 *
 * Inst RSSI measurement has timed out at this point, should generate
 * an event with invalid RSSI value.
 *
 * @param [in] arg  ieee80211com
 *
 */
static OS_TIMER_FUNC(wlan_bsteering_inst_rssi_timeout_handler)
{
    struct ieee80211com *ic;
    ieee80211_bsteering_t bsteering = NULL;
    struct ieee80211_node *ni = NULL;
    OS_GET_TIMER_ARG(ic, struct ieee80211com *);

    if (!ieee80211_bsteering_is_valid(ic)) {
        return;
    }

    bsteering = ic->ic_bsteering;
    spin_lock_bh(&bsteering->bs_lock);

    do {
        if (!ieee80211_bsteering_is_enabled(ic)) {
            break;
        }

        if (!bsteering->bs_inst_rssi_inprogress) {
            break;
        }

        ni = ieee80211_find_node(&ic->ic_sta, bsteering->bs_inst_rssi_macaddr);
        if (ni) {
            ieee80211_bsteering_send_rssi_measurement_event(ni->ni_vap, ni->ni_macaddr,
                                                            BSTEERING_INVALID_RSSI,
                                                            false /* is_debug */);
            ieee80211_free_node(ni);
        } else {
            /* In case the node is deleted from node table before timeout,
               we still want to notify LBD of invalid RSSI.
               Send the event with first VAP on this radio. */
            struct ieee80211vap *_vap;
            struct ieee80211_node *bss_node = NULL;
            IEEE80211_COMM_LOCK(ic);
            TAILQ_FOREACH(_vap, &ic->ic_vaps, iv_next) {
                if (ieee80211_bsteering_is_vap_enabled(_vap)) {
                    bss_node = ieee80211_ref_bss_node(_vap);
                    break;
                }
            }
            IEEE80211_COMM_UNLOCK(ic);
            if (bss_node) {
                ieee80211_bsteering_send_rssi_measurement_event(
                                                                bss_node->ni_vap, bsteering->bs_inst_rssi_macaddr,
                                                                BSTEERING_INVALID_RSSI, false /* is_debug */);
                ieee80211_free_node(bss_node);
            }
            /* If no VAP is band steering enabled on this band, it probably means
               band steering feature has been disabled. So we can live with not
               sending an RSSI measurement event since no one will be interested. */
        }

        bsteering->bs_inst_rssi_inprogress = false;

        printk("%s: Inst RSSI measurement request for STA "
               "%02x:%02x:%02x:%02x:%02x:%02x timeout.\n",
               __func__, bsteering->bs_inst_rssi_macaddr[0],
               bsteering->bs_inst_rssi_macaddr[1],
               bsteering->bs_inst_rssi_macaddr[2],
               bsteering->bs_inst_rssi_macaddr[3],
               bsteering->bs_inst_rssi_macaddr[4],
               bsteering->bs_inst_rssi_macaddr[5]);
    } while (0);

    spin_unlock_bh(&bsteering->bs_lock);
}
#endif
