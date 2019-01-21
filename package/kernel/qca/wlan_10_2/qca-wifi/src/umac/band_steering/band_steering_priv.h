/*
 * @@-COPYRIGHT-START-@@
 *
 * Copyright (c) 2014 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 *
 * @@-COPYRIGHT-END-@@
 */

#ifndef _UMAC_BAND_STEERING_PRIV__
#define _UMAC_BAND_STEERING_PRIV__

#include <osdep.h>
#include <ieee80211_var.h>

struct ieee80211_bsteering {
    osdev_t                             bs_osdev;
    wlan_dev_t                          bs_ic;
    wlan_if_t                           bs_iv;

    spinlock_t                          bs_lock;

    atomic_t                            bs_enabled;

    /// Configuration parameters set from userspace.
    ieee80211_bsteering_param_t         bs_config_params;

    /// Debugging control parameters set from userspace.
    ieee80211_bsteering_dbg_param_t     bs_dbg_config_params;

    /// Flag indicating the VAP overload or not
    bool                                bs_vap_overload;


    /// ===============================================================
    /// Members for inst RSSI measument
    /// ===============================================================

    /// Flag indicating if an inst RSSI measurement is in progress
    bool                                bs_inst_rssi_inprogress;

    /// The MAC address of the STA, whose RSSI measurement is in progress.
    u_int8_t                            bs_inst_rssi_macaddr[IEEE80211_ADDR_LEN];

    /// The number of valid RSSI samples measured through NDP
    /// before generating an instant RSSI measurement event
    u_int8_t                            bs_inst_rssi_num_samples;

    /// The average of RSSI samples
    u_int8_t                            bs_avg_inst_rssi;

    /// The number of valid RSSI samples gathered
    u_int8_t                            bs_inst_rssi_count;

    /// The number of invalid RSSI samples gathered
    u_int8_t                            bs_inst_rssi_err_count;

    /// Inst RSSI measurement timer
    os_timer_t                          bs_inst_rssi_timer;

    /// ===============================================================
    /// Members for channel utilization monitoring
    /// ===============================================================

    /// Periodic timer to check channel utilization.
    os_timer_t                          bs_chan_util_timer;

    /// Whether utilization has been requested but not yet completed or not.
    bool                                bs_chan_util_requested;

    /// Channel on which the device is operating.
    ///
    /// This caches the last active channel for two purposes:
    ///
    /// 1) To determine whether a channel utilization report is relevant.
    /// 2) To detect when the channel has changed between utilization
    ///    measurements (and thus the data should be invalidated).
    u_int8_t                            bs_active_ieee_chan_num;

    /// Running total of the utilization data collected so far.
    u_int32_t                           bs_chan_util_samples_sum;

    /// The number of valid samples in the utilization_samples array.
    u_int16_t                           bs_chan_util_num_samples;
};

#endif /* _UMAC_BAND_STEERING_PRIV__ */
