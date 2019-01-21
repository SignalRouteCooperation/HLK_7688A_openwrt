/*
 * @@-COPYRIGHT-START-@@
 *
 * Copyright (c) 2014 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 *
 * @@-COPYRIGHT-END-@@
 */

#ifndef _UMAC_IEEE80211_BAND_STEERING__
#define _UMAC_IEEE80211_BAND_STEERING__

/*opaque handle in ic structure */
typedef struct ieee80211_bsteering    *ieee80211_bsteering_t;

// forward decls
struct ieee80211vap;
struct ieee80211com;
struct ieee80211req_athdbg;

#if ATH_BAND_STEERING

/**
 * @brief Initialize the band steering infrastructure.
 *
 * @param [in] ic  the radio on which to initialize
 *
 * @return 0 on success; non-zero on error
 */
int ieee80211_bsteering_attach(struct ieee80211com *ic);

/**
 * @brief Destroy the band steering infrastructure.
 *
 * @param [in] ic  the radio for which to destroy
 *
 * @return 0 on success; non-zero on error
 */
int ieee80211_bsteering_detach(struct ieee80211com *ic);

/**
 * @brief Set the band steering parameters.
 *
 * This can only be done when band steering is disabled. The parameters
 * include timer values and thresholds for RSSI reporting.
 *
 * @param [in] vap  the VAP on which the set operation was done
 * @param [in] req  the parameters to be set
 */
int wlan_bsteering_set_params(struct ieee80211vap *vap,
                              const struct ieee80211req_athdbg *req);

/**
 * @brief Get the band steering parameters.
 *
 * The parameters include timer values and thresholds for RSSI reporting.
 *
 * @param [in] vap  the VAP on which the get operation was done
 * @param [in] req  the parameters to be retrieved
 */
int wlan_bsteering_get_params(const struct ieee80211vap *vap,
                              struct ieee80211req_athdbg *req);

/**
 * @brief Set the parameters that control whether debugging events are
 *        generated or not.
 *
 * @param [in] vap  the VAP on which the request came in
 * @param [in] req  the actual request parameters containing the new values
 */
int wlan_bsteering_set_dbg_params(struct ieee80211vap *vap,
                                  const struct ieee80211req_athdbg *req);

/**
 * @brief Get the parameters that control whether debugging events are
 *        generated or not.
 *
 * @param [in] vap  the VAP on which the request came in
 * @param [out] req  the object to update with the current debugging parameters
 */
int wlan_bsteering_get_dbg_params(const struct ieee80211vap *vap,
                                  struct ieee80211req_athdbg *req);

/**
 * @brief Generate an event indicating that a probe request was received.
 *
 * @param [in] vap  the VAP on which the probe was received
 * @param [in] mac_addr  the MAC address of the client that sent the probe
 *                       request
 * @param [in] rssi  the RSSI of the received probe request
 */
void ieee80211_bsteering_send_probereq_event(
                                             struct ieee80211vap *vap, const u_int8_t *mac_addr, u_int8_t rssi);

/**
 * @brief Generate an event indicating that an authentication message
 *        was sent with a failure code.
 *
 * @param [in] vap  the VAP on which the message was sent
 * @param [in] mac_addr  the MAC address of the client to which the
 *                       message was sent
 * @param [in] rssi  the RSSI of the received authentication message which
 *                   caused the rejection
 */
void ieee80211_bsteering_send_auth_fail_event(
                                              struct ieee80211vap *vap, const u_int8_t *mac_addr, u_int8_t rssi);

/**
 * @brief Inform the band steering module that a node is now authorized.
 *
 * @param [in] vap  the VAP on which the change occurred
 * @param [in] mac_addr  the MAC address of the client who had its
 *                       authorization status change
 */
void ieee80211_bsteering_send_node_authorized_event(struct ieee80211vap *vap,
                                                    const u_int8_t *mac_addr);

/**
 * @brief Toggle a VAP's overloaded status
 *
 * @param [inout] vap  the VAP whose overload status changes
 * @param [in] req  request from user space containing the flag indicating overload or not
 *
 * @return EOK if overload status is set, otherwise return EINVAL
 */
int wlan_bsteering_set_overload(struct ieee80211vap *vap,
                                const struct ieee80211req_athdbg *req);

/**
 * @brief Retrieve a VAP's overloaded status
 *
 * @param [in] vap  the VAP whose overload status to retrieve
 * @param [out] req  the object to update with the current overload status
 *
 * @return EOK if overload status is retrieved successfully, otherwise return EINVAL
 */
int wlan_bsteering_get_overload(const struct ieee80211vap *vap,
                                struct ieee80211req_athdbg *req);

/**
 * @brief Enable/Disable band steering on a VAP
 *
 * @pre  wlan_bsteering_set_params must be called
 *
 * @param [inout] vap  the VAP whose band steering status changes
 * @param [in] req  request from user space containing the flag indicating enable or disable
 *
 * @return EINVAL if band steering not initialized, otherwise
 *         return EOK
 */
int wlan_bsteering_enable(struct ieee80211vap *vap,
                          const struct ieee80211req_athdbg *req);

/**
 * @brief Start RSSI measurement on a specific station
 *
 * @param [in] vap  the VAP to start RSSI measurement
 * @param [in] req  request from user space containing the station MAC address and
 *                  number of measurements to average before reporting back
 *
 * @return EINVAL if band steering not initialized; EBUSY if the previous measurement is not done;
 *         otherwise return EOK
 */
int wlan_bsteering_trigger_rssi_measurement(struct ieee80211vap *vap,
                                            struct ieee80211req_athdbg *req);

/**
 * @brief Record a node's activity status change
 *
 * @param [in] ic  the radio on which the node is associated
 * @param [in] mac_addr  the MAC address of the node
 * @param [in] active  true if the node becomes active;
 *                     false if it becomes inactive
 */
void ieee80211_bsteering_record_act_change(struct ieee80211com *ic,
                                           const u_int8_t *mac_addr,
                                           bool active);

/**
 * @brief Inform the band steering module of a channel utilization measurement.
 *
 * If the necessary number of utilization measurements have been obtained,
 * this will result in an event being generated.
 *
 * @param [in] vap  the VAP for which the utilization report occurred
 * @param [in] ieee_chan_num  the channel on which the utilization measurement
 *                            took place
 * @param [in] chan_utilization  the actual utilization measurement
 */
void ieee80211_bsteering_record_utilization(struct ieee80211vap *vap,
                                            u_int ieee_chan_num,
                                            u_int32_t chan_utilization);

/**
 * @brief Inform the band steering module of a RSSI measurement.
 *
 * If the RSSI measurement crossed the threshold, this will result in an event
 * being generated.
 *
 * @param [in] ni  the node for which the RSSI measurement occurred
 * @param [in] rssi  the measured RSSI
 */
void ieee80211_bsteering_record_rssi(struct ieee80211_node *ni,
                                     u_int8_t rssi);

/**
 * @brief Inform the band steering module of an inst RSSI measurement obtained by
 *        sending Null Data Packet
 *
 * If the necessary number of inst RSSI measurements have been obtained,
 * this will result in an event being generated.
 *
 * @param [in] ni  the node for which the RSSI measurement occurred
 * @param [in] rssi  the measured RSSI
 */
void ieee80211_bsteering_record_inst_rssi(struct ieee80211_node *ni,
                                          u_int8_t rssi);
/**
 * @brief Inform the band steering module of an invalid RSSI measurement when
 *        retrying sending Null Data Packet reaches limit
 *
 * If the error count reaches the limit, an event will be generated with
 * rssi of BSTEERING_INVALID RSSI,
 *
 * @param [in] ni  the node for which the RSSI measurement occurred
 */
void ieee80211_bsteering_record_inst_rssi_err(struct ieee80211_node *ni);

#else

// Stub functions that return error codes to the band steering ioctls.

static inline int wlan_bsteering_set_params(struct ieee80211vap *vap,
                                            const struct ieee80211req_athdbg *req)
{
    return -EINVAL;
}

static inline int wlan_bsteering_get_params(const struct ieee80211vap *vap,
                                            struct ieee80211req_athdbg *req)
{
    return -EINVAL;
}

static inline int wlan_bsteering_set_dbg_params(struct ieee80211vap *vap,
                                                const struct ieee80211req_athdbg *req)
{
    return -EINVAL;
}

static inline int wlan_bsteering_get_dbg_params(const struct ieee80211vap *vap,
                                                struct ieee80211req_athdbg *req)
{
    return -EINVAL;
}

static inline int wlan_bsteering_set_overload(struct ieee80211vap *vap,
                                              const struct ieee80211req_athdbg *req)
{
    return -EINVAL;
}

static inline int wlan_bsteering_get_overload(const struct ieee80211vap *vap,
                                              struct ieee80211req_athdbg *req)
{
    return -EINVAL;
}

static inline int wlan_bsteering_enable(struct ieee80211vap *vap,
                                        const struct ieee80211req_athdbg *req)
{
    return -EINVAL;
}

static inline int wlan_bsteering_trigger_rssi_measurement(const struct ieee80211vap *vap,
                                                          struct ieee80211req_athdbg *req)
{
    return -EINVAL;
}

#endif /* ATH_BAND_STEERING */

#endif /* _UMAC_IEEE80211_BAND_STEERING__ */
