/*
 * Copyright (c) 2011, Atheros Communications Inc.
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
 * LMAC offload interface functions for UMAC - for power and performance offload model
 */

#if ATH_SUPPORT_SPECTRAL

#include "ol_if_athvar.h"
#include "ol_if_athpriv.h"
#include "bmi.h"
#include "sw_version.h"
#include "targaddrs.h"
#include "ol_helper.h"
#include "ol_txrx_ctrl_api.h"
#include "adf_os_mem.h"   /* adf_os_mem_alloc,free */
#include "adf_os_lock.h"  /* adf_os_spinlock_* */
#include "adf_os_types.h" /* adf_os_vprint */

#include "spectral.h"
#include "ol_if_spectral.h"

#define STATUS_PASS  1
#define STATUS_FAIL     0

#if ATH_PERF_PWR_OFFLOAD

/*
 * Spectral function tables, holds the Spectral functions
 * that depends on the direct/offload architecture
 * This is used to populate the actual Spectral function table
 * present in the SPECTRAL module
 */
SPECTRAL_OPS spectral_ops;


/*
 * Function     : ol_spectral_info_init_defaults
 * Description  : Helper function to load defaults for spectral information
 *                (parameters and state) into cache. It is assumed that the caller
 *                has obtained the requisite lock if applicable.
 *                Note that this is currently treated as a temporary function.
 *                Ideally, we would like to get defaults from the firmware.
 * Input        : Pointer to ieee80211com
 */
void ol_spectral_info_init_defaults(struct ieee80211com *ic)
{
    struct ath_spectral* spectral = ic->ic_spectral;
    OL_SPECTRAL_PARAM_STATE_INFO *info = &spectral->ol_info;

    /* State */
    info->osps_cache.osc_spectral_active =
        SPECTRAL_SCAN_ACTIVE_DEFAULT;

    info->osps_cache.osc_spectral_enabled =
        SPECTRAL_SCAN_ENABLE_DEFAULT;

    /* Parameters */

    info->osps_cache.osc_params.ss_count =
        SPECTRAL_SCAN_COUNT_DEFAULT;

    info->osps_cache.osc_params.ss_period =
        SPECTRAL_SCAN_PERIOD_DEFAULT;

    info->osps_cache.osc_params.ss_spectral_pri =
        SPECTRAL_SCAN_PRIORITY_DEFAULT;

    info->osps_cache.osc_params.ss_fft_size =
        SPECTRAL_SCAN_FFT_SIZE_DEFAULT;

    info->osps_cache.osc_params.ss_gc_ena =
        SPECTRAL_SCAN_GC_ENA_DEFAULT;

    info->osps_cache.osc_params.ss_restart_ena =
        SPECTRAL_SCAN_RESTART_ENA_DEFAULT;

    info->osps_cache.osc_params.ss_noise_floor_ref =
        SPECTRAL_SCAN_NOISE_FLOOR_REF_DEFAULT;

    info->osps_cache.osc_params.ss_init_delay =
        SPECTRAL_SCAN_INIT_DELAY_DEFAULT;

    info->osps_cache.osc_params.ss_nb_tone_thr =
        SPECTRAL_SCAN_NB_TONE_THR_DEFAULT;

    info->osps_cache.osc_params.ss_str_bin_thr =
        SPECTRAL_SCAN_STR_BIN_THR_DEFAULT;

    info->osps_cache.osc_params.ss_wb_rpt_mode =
        SPECTRAL_SCAN_WB_RPT_MODE_DEFAULT;

    info->osps_cache.osc_params.ss_rssi_rpt_mode =
        SPECTRAL_SCAN_RSSI_RPT_MODE_DEFAULT;

    info->osps_cache.osc_params.ss_rssi_thr =
        SPECTRAL_SCAN_RSSI_THR_DEFAULT;

    info->osps_cache.osc_params.ss_pwr_format =
        SPECTRAL_SCAN_PWR_FORMAT_DEFAULT;

    info->osps_cache.osc_params.ss_rpt_mode =
        SPECTRAL_SCAN_RPT_MODE_DEFAULT;

    info->osps_cache.osc_params.ss_bin_scale =
        SPECTRAL_SCAN_BIN_SCALE_DEFAULT;

    info->osps_cache.osc_params.ss_dBm_adj =
        SPECTRAL_SCAN_DBM_ADJ_DEFAULT;

    info->osps_cache.osc_params.ss_chn_mask =
        SPECTRAL_SCAN_CHN_MASK_DEFAULT;

    /* The cache is now valid */
    info->osps_cache.osc_is_valid = 1;
}

/*
 * Function     : ol_spectral_info_read
 * Description  : Read Spectral parameters or the desired
 *                state information from the cache.
 * Input        : Pointer to ieee80211com, OL_SPECTRAL_INFO_SPEC_T specifying
 *                which information is required, size of object pointed
 *                to by void output pointer
 * Output       : Void output pointer into which the information will be read.
 * Return       : 0 on success, negative error code on failure
 */
int ol_spectral_info_read(struct ieee80211com *ic,
                          OL_SPECTRAL_INFO_SPEC_T specifier,
                          void *output,
                          int output_len)
{

    /* Note: This function is designed to be able to accommodate
       WMI reads for defaults, non-cacheable information,etc
       if required. */

    struct ath_spectral* spectral = ic->ic_spectral;
    OL_SPECTRAL_PARAM_STATE_INFO *info = &spectral->ol_info;
    int is_cacheable = 0;

    if (output == NULL) {
        return -EINVAL;
    }

    switch(specifier)
    {
        case OL_SPECTRAL_INFO_SPEC_ACTIVE:
            if (output_len != sizeof(info->osps_cache.osc_spectral_active)) {
                return -EINVAL;
            }

            is_cacheable = 1;
            break;

         case OL_SPECTRAL_INFO_SPEC_ENABLED:
            if (output_len != sizeof(info->osps_cache.osc_spectral_enabled)) {
                return -EINVAL;
            }

            is_cacheable = 1;
            break;

         case OL_SPECTRAL_INFO_SPEC_PARAMS:
            if (output_len != sizeof(info->osps_cache.osc_params)) {
                return -EINVAL;
            }

            is_cacheable = 1;
            break;

         default:
            adf_os_print("%s: Unknown OL_SPECTRAL_INFO_SPEC_T specifier\n",
                         __func__);
            return -EINVAL;
    }

    OL_SPECTRAL_LOCK(&info->osps_lock);

    if (is_cacheable) {
        if (info->osps_cache.osc_is_valid) {
            switch(specifier)
            {
                case OL_SPECTRAL_INFO_SPEC_ACTIVE:
                    OS_MEMCPY(output,
                              &info->osps_cache.osc_spectral_active,
                              sizeof(info->osps_cache.osc_spectral_active));
#ifdef OL_SPECTRAL_DEBUG_CONFIG_INTERACTIONS
                    adf_os_print("%s: OL_SPECTRAL_INFO_SPEC_ACTIVE. Returning "
                                 "val=%u\n",
                                 __FUNCTION__,
                                 *((unsigned char*)output));
#endif /* OL_SPECTRAL_DEBUG_CONFIG_INTERACTIONS */
                    break;

                case OL_SPECTRAL_INFO_SPEC_ENABLED:
                    OS_MEMCPY(output,
                              &info->osps_cache.osc_spectral_enabled,
                              sizeof(info->osps_cache.osc_spectral_enabled));
#ifdef OL_SPECTRAL_DEBUG_CONFIG_INTERACTIONS
                    adf_os_print("%s: OL_SPECTRAL_INFO_SPEC_ENABLED. Returning "
                                 "val=%u\n",
                                 __FUNCTION__,
                                 *((unsigned char*)output));
#endif /* OL_SPECTRAL_DEBUG_CONFIG_INTERACTIONS */

                    break;

                case OL_SPECTRAL_INFO_SPEC_PARAMS:
                    OS_MEMCPY(output,
                              &info->osps_cache.osc_params,
                              sizeof(info->osps_cache.osc_params));
#ifdef OL_SPECTRAL_DEBUG_CONFIG_INTERACTIONS
                    {
                        HAL_SPECTRAL_PARAM *pparam =
                            (HAL_SPECTRAL_PARAM *)output;

                        adf_os_print("%s: OL_SPECTRAL_INFO_SPEC_PARAMS. "
                                     "Returning following params:\n"
                                     "ss_count = %u\n"
                                     "ss_period = %u\n"
                                     "ss_spectral_pri = %u\n"
                                     "ss_fft_size = %u\n"
                                     "ss_gc_ena = %u\n"
                                     "ss_restart_ena = %u\n"
                                     "ss_noise_floor_ref = %d\n"
                                     "ss_init_delay = %u\n"
                                     "ss_nb_tone_thr = %u\n"
                                     "ss_str_bin_thr = %u\n"
                                     "ss_wb_rpt_mode = %u\n"
                                     "ss_rssi_rpt_mode = %u\n"
                                     "ss_rssi_thr = %d\n"
                                     "ss_pwr_format = %u\n"
                                     "ss_rpt_mode = %u\n"
                                     "ss_bin_scale = %u\n"
                                     "ss_dBm_adj = %u\n"
                                     "ss_chn_mask = %u\n\n",
                                     __FUNCTION__,
                                     pparam->ss_count,
                                     pparam->ss_period,
                                     pparam->ss_spectral_pri,
                                     pparam->ss_fft_size,
                                     pparam->ss_gc_ena,
                                     pparam->ss_restart_ena,
                                     (int8_t)pparam->ss_noise_floor_ref,
                                     pparam->ss_init_delay,
                                     pparam->ss_nb_tone_thr,
                                     pparam->ss_str_bin_thr,
                                     pparam->ss_wb_rpt_mode,
                                     pparam->ss_rssi_rpt_mode,
                                     (int8_t)pparam->ss_rssi_thr,
                                     pparam->ss_pwr_format,
                                     pparam->ss_rpt_mode,
                                     pparam->ss_bin_scale,
                                     pparam->ss_dBm_adj,
                                     pparam->ss_chn_mask);
                    }
#endif /* OL_SPECTRAL_DEBUG_CONFIG_INTERACTIONS */
                    break;

                default:
                    /* We can't reach this point */
                    break;
            }

            OL_SPECTRAL_UNLOCK(&info->osps_lock);
            return 0;
        }
    }

    /* Cache is invalid */

    /* If WMI Reads are implemented to fetch defaults/non-cacheable info,
       then the below implementation will change */
    ol_spectral_info_init_defaults(ic);
    /* ol_spectral_info_init_defaults() has set the cache to valid */

    switch(specifier)
    {
        case OL_SPECTRAL_INFO_SPEC_ACTIVE:
            OS_MEMCPY(output,
                      &info->osps_cache.osc_spectral_active,
                      sizeof(info->osps_cache.osc_spectral_active));
#ifdef OL_SPECTRAL_DEBUG_CONFIG_INTERACTIONS
            adf_os_print("%s: OL_SPECTRAL_INFO_SPEC_ACTIVE on "
                         "initial cache validation\n"
                         "Returning val=%u\n",
                         __FUNCTION__,
                         *((unsigned char*)output));
#endif /* OL_SPECTRAL_DEBUG_CONFIG_INTERACTIONS */
            break;

        case OL_SPECTRAL_INFO_SPEC_ENABLED:
            OS_MEMCPY(output,
                      &info->osps_cache.osc_spectral_enabled,
                      sizeof(info->osps_cache.osc_spectral_enabled));
#ifdef OL_SPECTRAL_DEBUG_CONFIG_INTERACTIONS
            adf_os_print("%s: OL_SPECTRAL_INFO_SPEC_ENABLED on "
                         "initial cache validation\n"
                         "Returning val=%u\n",
                         __FUNCTION__,
                         *((unsigned char*)output));
#endif /* OL_SPECTRAL_DEBUG_CONFIG_INTERACTIONS */
            break;

        case OL_SPECTRAL_INFO_SPEC_PARAMS:
            OS_MEMCPY(output,
                      &info->osps_cache.osc_params,
                      sizeof(info->osps_cache.osc_params));
#ifdef OL_SPECTRAL_DEBUG_CONFIG_INTERACTIONS
            {
                HAL_SPECTRAL_PARAM *pparam =
                    (HAL_SPECTRAL_PARAM *)output;

                adf_os_print("%s: OL_SPECTRAL_INFO_SPEC_PARAMS on "
                             "initial cache validation\n"
                             "Returning following params:\n"
                             "ss_count = %u\n"
                             "ss_period = %u\n"
                             "ss_spectral_pri = %u\n"
                             "ss_fft_size = %u\n"
                             "ss_gc_ena = %u\n"
                             "ss_restart_ena = %u\n"
                             "ss_noise_floor_ref = %d\n"
                             "ss_init_delay = %u\n"
                             "ss_nb_tone_thr = %u\n"
                             "ss_str_bin_thr = %u\n"
                             "ss_wb_rpt_mode = %u\n"
                             "ss_rssi_rpt_mode = %u\n"
                             "ss_rssi_thr = %d\n"
                             "ss_pwr_format = %u\n"
                             "ss_rpt_mode = %u\n"
                             "ss_bin_scale = %u\n"
                             "ss_dBm_adj = %u\n"
                             "ss_chn_mask = %u\n\n",
                             __FUNCTION__,
                             pparam->ss_count,
                             pparam->ss_period,
                             pparam->ss_spectral_pri,
                             pparam->ss_fft_size,
                             pparam->ss_gc_ena,
                             pparam->ss_restart_ena,
                             (int8_t)pparam->ss_noise_floor_ref,
                             pparam->ss_init_delay,
                             pparam->ss_nb_tone_thr,
                             pparam->ss_str_bin_thr,
                             pparam->ss_wb_rpt_mode,
                             pparam->ss_rssi_rpt_mode,
                             (int8_t)pparam->ss_rssi_thr,
                             pparam->ss_pwr_format,
                             pparam->ss_rpt_mode,
                             pparam->ss_bin_scale,
                             pparam->ss_dBm_adj,
                             pparam->ss_chn_mask);
            }
#endif /* OL_SPECTRAL_DEBUG_CONFIG_INTERACTIONS */

            break;

        default:
            /* We can't reach this point */
            break;
    }

    OL_SPECTRAL_UNLOCK(&info->osps_lock);

    return 0;
}

/*
 * Function     : wmi_send_vdev_spectral_configure_cmd
 * Description  : Send WMI command to configure Spectral parameters
 * Input        : Pointer to ieee80211com,
 *                Pointer to HAL_SPECTRAL_PARAM
 * Return       : EOK on success, negative error code on failure
 */
int wmi_send_vdev_spectral_configure_cmd(struct ieee80211com *ic,
                                         HAL_SPECTRAL_PARAM *param)
{
    struct ol_ath_softc_net80211 *scn = OL_ATH_SOFTC_NET80211(ic);
    wmi_vdev_spectral_configure_cmd *cmd;
    struct ieee80211vap *vap = NULL;
    wmi_buf_t buf;
    int len = 0;
    int ret;

    len = sizeof(wmi_vdev_spectral_configure_cmd);
    buf = wmi_buf_alloc(scn->wmi_handle, len);
    if (!buf) {
        adf_os_print("%s:wmi_buf_alloc failed\n", __FUNCTION__);
        return -ENOMEM;
    }

    cmd = (wmi_vdev_spectral_configure_cmd *)wmi_buf_data(buf);
    
    /* Use the first VAP under the radio for now */
    vap = TAILQ_FIRST(&ic->ic_vaps);

    if (vap == NULL) {
        adf_os_print("%s: First VAP pointer is NULL\n", __FUNCTION__);
        return -ENODEV;
    }
    
    cmd->vdev_id = (OL_ATH_VAP_NET80211(vap))->av_if_id;

    cmd->spectral_scan_count = param->ss_count;
    cmd->spectral_scan_period = param->ss_period;
    cmd->spectral_scan_priority = param->ss_spectral_pri;
    cmd->spectral_scan_fft_size = param->ss_fft_size;
    cmd->spectral_scan_gc_ena = param->ss_gc_ena;
    cmd->spectral_scan_restart_ena = param->ss_restart_ena;
    cmd->spectral_scan_noise_floor_ref = param->ss_noise_floor_ref;
    cmd->spectral_scan_init_delay = param->ss_init_delay;
    cmd->spectral_scan_nb_tone_thr = param->ss_nb_tone_thr;
    cmd->spectral_scan_str_bin_thr = param->ss_str_bin_thr;
    cmd->spectral_scan_wb_rpt_mode = param->ss_wb_rpt_mode;
    cmd->spectral_scan_rssi_rpt_mode = param->ss_rssi_rpt_mode;
    cmd->spectral_scan_rssi_thr = param->ss_rssi_thr;
    cmd->spectral_scan_pwr_format = param->ss_pwr_format;
    cmd->spectral_scan_rpt_mode = param->ss_rpt_mode;
    cmd->spectral_scan_bin_scale = param->ss_bin_scale;
    cmd->spectral_scan_dBm_adj = param->ss_dBm_adj;
    cmd->spectral_scan_chn_mask = param->ss_chn_mask;

    ret = wmi_unified_cmd_send(scn->wmi_handle,
                               buf,
                               len,
                               WMI_VDEV_SPECTRAL_SCAN_CONFIGURE_CMDID);

#ifdef OL_SPECTRAL_DEBUG_CONFIG_INTERACTIONS
    adf_os_print("%s: Sent "
                 "WMI_VDEV_SPECTRAL_SCAN_CONFIGURE_CMDID\n", __FUNCTION__);

    adf_os_print("vdev_id = %u\n"
                 "spectral_scan_count = %u\n"
                 "spectral_scan_period = %u\n"
                 "spectral_scan_priority = %u\n"
                 "spectral_scan_fft_size = %u\n"
                 "spectral_scan_gc_ena = %u\n"
                 "spectral_scan_restart_ena = %u\n"
                 "spectral_scan_noise_floor_ref = %u\n"
                 "spectral_scan_init_delay = %u\n"
                 "spectral_scan_nb_tone_thr = %u\n"
                 "spectral_scan_str_bin_thr = %u\n"
                 "spectral_scan_wb_rpt_mode = %u\n"
                 "spectral_scan_rssi_rpt_mode = %u\n"
                 "spectral_scan_rssi_thr = %u\n"
                 "spectral_scan_pwr_format = %u\n"
                 "spectral_scan_rpt_mode = %u\n"
                 "spectral_scan_bin_scale = %u\n"
                 "spectral_scan_dBm_adj = %u\n"
                 "spectral_scan_chn_mask = %u\n",
                 cmd->vdev_id,
                 cmd->spectral_scan_count,
                 cmd->spectral_scan_period,
                 cmd->spectral_scan_priority,
                 cmd->spectral_scan_fft_size,
                 cmd->spectral_scan_gc_ena,
                 cmd->spectral_scan_restart_ena,
                 cmd->spectral_scan_noise_floor_ref,
                 cmd->spectral_scan_init_delay,
                 cmd->spectral_scan_nb_tone_thr,
                 cmd->spectral_scan_str_bin_thr,
                 cmd->spectral_scan_wb_rpt_mode,
                 cmd->spectral_scan_rssi_rpt_mode,
                 cmd->spectral_scan_rssi_thr,
                 cmd->spectral_scan_pwr_format,
                 cmd->spectral_scan_rpt_mode,
                 cmd->spectral_scan_bin_scale,
                 cmd->spectral_scan_dBm_adj,
                 cmd->spectral_scan_chn_mask);
    
    adf_os_print("%s: Status: %d\n\n", __FUNCTION__, ret);
#endif /* OL_SPECTRAL_DEBUG_CONFIG_INTERACTIONS */

    return ret;
}


/*
 * Function     : wmi_send_vdev_spectral_enable_cmd
 * Description  : Send WMI command to enable/disable spectral
 * Input        : Pointer to ieee80211com,
 *                Flag to indicate if spectral activate (trigger) is valid,
 *                Value of spectral activate,
 *                Flag to indicate if spectral enable is valid,
 *                Value of spectral enable
 * Return       : 0 on success, negative error code on failure
 */
int wmi_send_vdev_spectral_enable_cmd(struct ieee80211com *ic,
                                      u_int8_t is_spectral_active_valid,
                                      u_int8_t is_spectral_active,
                                      u_int8_t is_spectral_enabled_valid,
                                      u_int8_t is_spectral_enabled)
{
    struct ol_ath_softc_net80211 *scn = OL_ATH_SOFTC_NET80211(ic);
    wmi_vdev_spectral_enable_cmd *cmd;
    struct ieee80211vap *vap = NULL;
    wmi_buf_t buf;
    int len = 0;
    int ret;

    len = sizeof(wmi_vdev_spectral_enable_cmd);
    buf = wmi_buf_alloc(scn->wmi_handle, len);
    if (!buf) {
        adf_os_print("%s:wmi_buf_alloc failed\n", __FUNCTION__);
        return -ENOMEM;
    }

    cmd = (wmi_vdev_spectral_enable_cmd *)wmi_buf_data(buf);
   
    /* Use the first VAP under the radio for now */
    vap = TAILQ_FIRST(&ic->ic_vaps);

    if (vap == NULL) {
        adf_os_print("%s: First VAP pointer is NULL\n", __FUNCTION__);
        return -ENODEV;
    }
    
    cmd->vdev_id = (OL_ATH_VAP_NET80211(vap))->av_if_id;

    if (is_spectral_active_valid) {
        cmd->trigger_cmd = is_spectral_active ? 1 : 2; /* 1: Trigger,
                                                          2: Clear Trigger */
    } else {
        cmd->trigger_cmd = 0; /* 0: Ignore */
    }

    if (is_spectral_enabled_valid) {
        cmd->enable_cmd = is_spectral_enabled ? 1 : 2; /* 1: Enable
                                                          2: Disable */
    } else {
        cmd->enable_cmd = 0; /* 0: Ignore */
    }


    ret = wmi_unified_cmd_send(scn->wmi_handle,
                               buf,
                               len,
                               WMI_VDEV_SPECTRAL_SCAN_ENABLE_CMDID);

#ifdef OL_SPECTRAL_DEBUG_CONFIG_INTERACTIONS
    adf_os_print("%s: Sent "
                 "WMI_VDEV_SPECTRAL_SCAN_ENABLE_CMDID\n", __FUNCTION__);

    adf_os_print("vdev_id = %u\n"
                 "trigger_cmd = %u\n"
                 "enable_cmd = %u\n",
                 cmd->vdev_id,
                 cmd->trigger_cmd,
                 cmd->enable_cmd);
    
    adf_os_print("%s: Status: %d\n\n", __FUNCTION__, ret);
#endif /* OL_SPECTRAL_DEBUG_CONFIG_INTERACTIONS */

    return ret;
}

/*
 * Function     : ol_spectral_info_write
 * Description  : Write Spectral parameters or the desired
 *                state information to the firmware, and update cache
 * Input        : Pointer to ieee80211com, OL_SPECTRAL_INFO_SPEC_T specifying
 *                which information is involved, size of object pointed
 *                to by void input pointer, void input pointer containing
 *                the information to be written.
 * Return       : 0 on success, negative error code on failure
 */
int ol_spectral_info_write(struct ieee80211com *ic,
                           OL_SPECTRAL_INFO_SPEC_T specifier,
                           void *input,
                           int input_len)
{
    struct ath_spectral* spectral = ic->ic_spectral;
    OL_SPECTRAL_PARAM_STATE_INFO *info = &spectral->ol_info;
    int ret;
    u_int8_t *pval = NULL;
    HAL_SPECTRAL_PARAM *param = NULL;

    if (input == NULL) {
        return -EINVAL;
    }

    switch(specifier)
    {
        case OL_SPECTRAL_INFO_SPEC_ACTIVE:
            if (input_len != sizeof(info->osps_cache.osc_spectral_active)) {
                return -EINVAL;
            }

            pval = (u_int8_t *)input;

            OL_SPECTRAL_LOCK(&info->osps_lock);
            ret = wmi_send_vdev_spectral_enable_cmd(ic, 1, *pval, 0, 0);

#ifdef OL_SPECTRAL_DEBUG_CONFIG_INTERACTIONS
            adf_os_print("%s: OL_SPECTRAL_INFO_SPEC_ACTIVE with "
                         "val=%u status=%d\n",
                         __FUNCTION__,
                         *pval,
                         ret);
#endif /* OL_SPECTRAL_DEBUG_CONFIG_INTERACTIONS */

            if (ret < 0) {
                adf_os_print("%s: wmi_send_vdev_spectral_enable_cmd "
                             "failed with error=%d\n",
                             __func__,
                             ret);
                OL_SPECTRAL_UNLOCK(&info->osps_lock);
                return ret;
            }

            info->osps_cache.osc_spectral_active = *pval;
            OL_SPECTRAL_UNLOCK(&info->osps_lock);
            break;

         case OL_SPECTRAL_INFO_SPEC_ENABLED:
           if (input_len != sizeof(info->osps_cache.osc_spectral_enabled)) {
                return -EINVAL;
            }

            pval = (u_int8_t *)input;
            
            OL_SPECTRAL_LOCK(&info->osps_lock);
            ret = wmi_send_vdev_spectral_enable_cmd(ic, 0, 0, 1, *pval);

#ifdef OL_SPECTRAL_DEBUG_CONFIG_INTERACTIONS
            adf_os_print("%s: OL_SPECTRAL_INFO_SPEC_ENABLED with "
                         "val=%u status=%d\n",
                         __FUNCTION__,
                         *pval,
                         ret);
#endif /* OL_SPECTRAL_DEBUG_CONFIG_INTERACTIONS */

            if (ret < 0) {
                adf_os_print("%s: wmi_send_vdev_spectral_enable_cmd "
                             "failed with error=%d\n",
                             __func__,
                             ret);
                OL_SPECTRAL_UNLOCK(&info->osps_lock);
                return ret;
            }

            info->osps_cache.osc_spectral_enabled = *pval;
            OL_SPECTRAL_UNLOCK(&info->osps_lock);
            break;

         case OL_SPECTRAL_INFO_SPEC_PARAMS:
           if (input_len != sizeof(info->osps_cache.osc_params)) {
                return -EINVAL;
            }

            param = (HAL_SPECTRAL_PARAM *)input;

            OL_SPECTRAL_LOCK(&info->osps_lock);
            ret = wmi_send_vdev_spectral_configure_cmd(ic, param);

#ifdef OL_SPECTRAL_DEBUG_CONFIG_INTERACTIONS
            adf_os_print("%s: OL_SPECTRAL_INFO_SPEC_PARAMS. "
                         "Params:\n"
                         "ss_count = %u\n"
                         "ss_period = %u\n"
                         "ss_spectral_pri = %u\n"
                         "ss_fft_size = %u\n"
                         "ss_gc_ena = %u\n"
                         "ss_restart_ena = %u\n"
                         "ss_noise_floor_ref = %d\n"
                         "ss_init_delay = %u\n"
                         "ss_nb_tone_thr = %u\n"
                         "ss_str_bin_thr = %u\n"
                         "ss_wb_rpt_mode = %u\n"
                         "ss_rssi_rpt_mode = %u\n"
                         "ss_rssi_thr = %d\n"
                         "ss_pwr_format = %u\n"
                         "ss_rpt_mode = %u\n"
                         "ss_bin_scale = %u\n"
                         "ss_dBm_adj = %u\n"
                         "ss_chn_mask = %u\n"
                         "status = %d\n\n",
                         __FUNCTION__,
                         param->ss_count,
                         param->ss_period,
                         param->ss_spectral_pri,
                         param->ss_fft_size,
                         param->ss_gc_ena,
                         param->ss_restart_ena,
                         (int8_t)param->ss_noise_floor_ref,
                         param->ss_init_delay,
                         param->ss_nb_tone_thr,
                         param->ss_str_bin_thr,
                         param->ss_wb_rpt_mode,
                         param->ss_rssi_rpt_mode,
                         (int8_t)param->ss_rssi_thr,
                         param->ss_pwr_format,
                         param->ss_rpt_mode,
                         param->ss_bin_scale,
                         param->ss_dBm_adj,
                         param->ss_chn_mask,
                         ret);
#endif /* OL_SPECTRAL_DEBUG_CONFIG_INTERACTIONS */

            if (ret < 0) {
                adf_os_print("%s: wmi_send_vdev_spectral_configure_cmd "
                             "failed with error=%d\n",
                             __func__,
                             ret);
                OL_SPECTRAL_UNLOCK(&info->osps_lock);
                return ret;
            }

            OS_MEMCPY(&info->osps_cache.osc_params,
                      param,
                      sizeof (info->osps_cache.osc_params));
            OL_SPECTRAL_UNLOCK(&info->osps_lock);
            break;

         default:
            adf_os_print("%s: Unknown OL_SPECTRAL_INFO_SPEC_T specifier\n",
                         __func__);
            return -EINVAL;
    }

    return 0;
}

/*
 * Function     : ol_if_spectral_get_tsf64
 * Description  : Returns the last TSF received in WMI buffer
 * Input        : Pointer to Spectral
 * Output       : TSF value
 *
 */
u_int64_t
ol_if_spectral_get_tsf64(void* arg)
{
    struct ath_spectral* spectral = (struct ath_spectral*)arg;
    return spectral->tsf64;
}

/*
 * Function     : ol_if_spectral_get_capability
 * Description  : Get the hardware capability
 * Input        : Pointer to Spectral and Capability type
 * Output       : True/False
 *
 */
u_int32_t ol_if_spectral_get_capability(void* arg, HAL_CAPABILITY_TYPE type)
{
    int status = STATUS_FAIL;

    switch (type) {
      case HAL_CAP_PHYDIAG:
      case HAL_CAP_RADAR:
      case HAL_CAP_SPECTRAL_SCAN:
      case HAL_CAP_ADVNCD_SPECTRAL_SCAN:
        status = STATUS_PASS;
          break;
      default:
        status = STATUS_FAIL;
    }
    return status;
}

/*
 * Function     : ol_if_spectral_set_rxfilter
 * Description  : Set the RX Filter before Spectral start
 * Input        : Pointer to Spectral and RX Filter config
 * Output       : Success/Failure
 *
 */
u_int32_t ol_if_spectral_set_rxfilter(void* arg, int rxfilter)
{
    /* Will not be required since enabling of spectral in firmware
       will take care of this */
    return 0;
}

/*
 * Function     : ol_if_spectral_get_rxfilter
 * Description  : Get the current RX Filter settings
 * Input        : Pointer to spectral
 * Output       : RX Filter config
 *
 */
u_int32_t ol_if_spectral_get_rxfilter(void* arg)
{
    /* Will not be required since enabling of spectral in firmware
       will take care of this */
    return 0;
}

/*
 * Function     : ol_if_is_spectral_active
 * Description  : Checks if Spectral is active
 * Input        : Pointer to Spectral
 * Output       : True (1)/False(0))
 */
u_int32_t ol_if_is_spectral_active(void* arg)
{
    struct ath_spectral *spectral = (struct ath_spectral *)arg;
    struct ieee80211com* ic = spectral->ic;
    u_int8_t val = 0;
    int ret;

    ret = ol_spectral_info_read(ic,
                                OL_SPECTRAL_INFO_SPEC_ACTIVE,
                                &val,
                                sizeof(val));

    if (ret != 0) {
        /* Could not determine if Spectral is active.
           Return false as a safe value.
           XXX: Consider changing the function prototype
           to be able to indicate failure to fetch value. */
           return 0;
    }

    return val;
}

/*
 * Function     : ol_if_spectral_enabled
 * Description  : Checks if Spectral is enabled
 * Input        : Pointer to Spectral
 * Output       : True(1)/False(0)
 */
u_int32_t ol_if_is_spectral_enabled(void* arg)
{
    struct ath_spectral *spectral = (struct ath_spectral *)arg;
    struct ieee80211com* ic = spectral->ic;
    u_int8_t val = 0;
    int ret;

    ret = ol_spectral_info_read(ic,
                                OL_SPECTRAL_INFO_SPEC_ENABLED,
                                &val,
                                sizeof(val));

    if (ret != 0) {
        /* Could not determine if Spectral is enabled.
           Return false as a safe value.
           XXX: Consider changing the function prototype
           to be able to indicate failure to fetch value. */
           return 0;
    }

    return val;
}

/*
 * Function     : ol_if_start_spectral_scan
 * Description  : Starts the Spectral Scan
 * Input        : Pointer to Spectral
 * Output       : Success(1)/Failure(0)
 *
 */
u_int32_t ol_if_start_spectral_scan(void* arg)
{
    struct ath_spectral *spectral = (struct ath_spectral *)arg;
    struct ieee80211com* ic = spectral->ic;
    u_int8_t val = 1;
    u_int8_t enabled = 0;
    int ret;

    ret = ol_spectral_info_read(ic,
                                OL_SPECTRAL_INFO_SPEC_ENABLED,
                                &enabled,
                                sizeof(enabled));

    if (ret != 0) {
        /* Could not determine if Spectral is enabled. Assume we need
           to enable it*/
        enabled = 0;
    }

    if (!enabled) {
        ret = ol_spectral_info_write(ic,
                                     OL_SPECTRAL_INFO_SPEC_ENABLED,
                                     &val,
                                     sizeof(val));

        if (ret != 0) {
            return 0;
        }
    }

    ret = ol_spectral_info_write(ic,
                                 OL_SPECTRAL_INFO_SPEC_ACTIVE,
                                 &val,
                                 sizeof(val));

    if (ret != 0) {
        return 0;
    }

    return 1;
}

/*
 * Function     : ol_if_ic_start_spectral_scan
 * Description  : Starts the Spectral Scan.
 *                This is a wrapper over ol_if_start_spectral_scan()
 * Input        : Pointer to ic
 *
 */
void ol_if_ic_start_spectral_scan(struct ieee80211com* ic, u_int8_t priority)
{
    ol_if_start_spectral_scan(ic->ic_spectral);

    return;
}

/*
 * Function     : ol_if_stop_spectral_scan
 * Description  : Stops the Spectral scan
 * Input        : Pointer to Spectral
 * Output       : Success (1)/Failure (0)
 *
 */
u_int32_t ol_if_stop_spectral_scan(void* arg)
{
    struct ath_spectral *spectral = (struct ath_spectral *)arg;
    struct ieee80211com* ic = spectral->ic;
    u_int8_t val = 0;
    int tempret, ret = 1;

    tempret = ol_spectral_info_write(ic,
                                     OL_SPECTRAL_INFO_SPEC_ACTIVE,
                                     &val,
                                     sizeof(val));

    if (tempret != 0) {
        ret = 0;
    }

    tempret = ol_spectral_info_write(ic,
                                     OL_SPECTRAL_INFO_SPEC_ENABLED,
                                     &val,
                                     sizeof(val));

    if (tempret != 0) {
        ret = 0;
    }

    return ret;
}

/*
 * Function     : ol_if_ic_stop_spectral_scan
 * Description  : Stops the Spectral Scan.
 *                This is a wrapper over ol_if_stop_spectral_scan()
 * Input        : Pointer to ic
 *
 */
void ol_if_ic_stop_spectral_scan(struct ieee80211com* ic)
{
    ol_if_stop_spectral_scan(ic->ic_spectral);

    return;
}

/*
 * Function     : ol_if_spectral_get_extension_channel
 * Description  : Get the current Extension channel (in MHz)
 * Input        : Pointer to Spectral
 * Output       : Channel info
 *
 */
u_int32_t ol_if_spectral_get_extension_channel(void* arg)
{
    struct ath_spectral *spectral = (struct ath_spectral *)arg;
    struct ieee80211com* ic = spectral->ic;
    int offset = 0;

    offset = ic->ic_cwm_get_extoffset(ic);

    /* XXX - Currently, we assume half/quarter rates are not used.
       Look into supporting this */

    if (offset) {
        return ic->ic_curchan->ic_freq + (20 * offset);
    }

    return 0;
}

/*
 * Function     : ol_if_spectral_get_current_channel
 * Description  : Get the current channel (in MHz)
 * Input        : Pointer to current channel
 * Output       : Channel info
 *
 */
u_int32_t ol_if_spectral_get_current_channel(void* arg)
{
    struct ath_spectral *spectral = (struct ath_spectral *)arg;
    struct ieee80211com* ic = spectral->ic;
    return ic->ic_curchan->ic_freq;
}

/*
 * Function     : ol_if_spectral_reset_hw
 * Description  : Reset the hardware
 * Input        : Pointer to Spectral
 * Output       : Success/Failure
 *
 */
u_int32_t ol_if_spectral_reset_hw(void* arg)
{
    not_yet_implemented();
    return 0;
}

/*
 * Function     : ol_if_spectral_get_chain_noise_floor
 * Description  : Get the Chain noise floor from Noisefloor history buffer
 * Input        : Pointer to Spectral, pointer to buffer into which chain
 *                Noise Floor Data should be copied.
 * Output       : Noisefloor value
 *
 */
u_int32_t ol_if_spectral_get_chain_noise_floor(void* arg, int16_t* nfBuf)
{
    not_yet_implemented();
    return 0;
}
/*
 * Function     : ol_if_spectral_get_ext_noisefloor
 * Description  : Get the extension channel noisefloor
 * Input        : Pointer to Spectral
 * Output       : Noisefloor
 *
 */

int8_t ol_if_spectral_get_ext_noisefloor(void* arg)
{
    not_yet_implemented();
    return 0;
}

/*
 * Function     : ol_if_spectral_get_ctl_noisefloor
 * Description  : Get the control channel noisefloor
 * Input        : Pointer to Spectral
 * Output       : Noisefloor
 *
 */
int8_t ol_if_spectral_get_ctl_noisefloor(void* arg)
{
    not_yet_implemented();
    return 0;
}

/*
 * Function     : ol_if_spectral_get_nominal_noisefloor
 * Description  : Get the nominal noise floor
 * Input        : Pointer to Spectral
 * Output       : Noisefloor
 *
 * XXX
 * Till we develop the way to get the nominal noisefloor value
 * from firmware, we are exporting the values from the Host layers.
 * These values are taken from the file ar6000_reset.h
 */

#define SPECTRAL_PHY_CCA_NOM_VAL_AR9888_2GHZ    (-108)
#define SPECTRAL_PHY_CCA_NOM_VAL_AR9888_5GHZ    (-105)
int16_t ol_if_spectral_get_nominal_noisefloor(void* arg, HAL_FREQ_BAND band)
{
    int16_t nominal_nf = 0;

    if (band == HAL_FREQ_BAND_5GHZ) {
        nominal_nf = SPECTRAL_PHY_CCA_NOM_VAL_AR9888_5GHZ;
    } else {
        nominal_nf = SPECTRAL_PHY_CCA_NOM_VAL_AR9888_2GHZ;
    }

    return nominal_nf;
}


/*
 * Function     : ol_if_spectral_configure_params
 * Description  : Configure the user supplied Spectral params
 * Input        : Pointer to Spectral and params
 * Output       : Success(1)/Failure(0)
 *
 */
u_int32_t ol_if_spectral_configure_params(void* arg, HAL_SPECTRAL_PARAM* params)
{
    struct ath_spectral *spectral = (struct ath_spectral *)arg;
    struct ieee80211com* ic = spectral->ic;
    int ret;

    ret = ol_spectral_info_write(ic,
                                 OL_SPECTRAL_INFO_SPEC_PARAMS,
                                 params,
                                 sizeof(*params));

    if (ret != 0) {
        return 0;
    }

    return 1;
}

/*
 * Function     : ol_if_spectral_get_params
 * Description  : Get the user configured Spectral params
 * Input        : Pointer to Spectral and params
 * Output       : Success(1)/Failure(0)
 *
 */
u_int32_t ol_if_spectral_get_params(void* arg, HAL_SPECTRAL_PARAM* params)
{
    struct ath_spectral *spectral = (struct ath_spectral *)arg;
    struct ieee80211com* ic = spectral->ic;
    int ret;

    ret = ol_spectral_info_read(ic,
                                OL_SPECTRAL_INFO_SPEC_PARAMS,
                                params,
                                sizeof(*params));

    if (ret != 0) {
           return 0;
    }

    return 1;
}

/*
 * Function     : ol_if_spectral_get_ent_mask
 * Description  :
 * Input        :
 * Output       :
 *
 */
u_int32_t ol_if_spectral_get_ent_mask(void* arg)
{
    not_yet_implemented();
    return 0;
}

/*
 * Function     : ol_if_spectral_get_macaddr
 * Description  : Get the current Mac address
 * Input        : Pointer to Spectral and addr buffer
 * Output       : Success/Failure
 *
 */
u_int32_t ol_if_spectral_get_macaddr(void* arg, char* addr)
{
    struct ath_spectral *spectral = (struct ath_spectral *)arg;
    struct ieee80211com* ic = spectral->ic;
     
    OS_MEMCPY(addr, ic->ic_myaddr, IEEE80211_ADDR_LEN);

    return 0;
}

/*
 * Function     : ol_if_spectral_set_icm_active
 * Description  : Set whether ICM is active
 * Input        : Pointer to Spectral, flag specifying whether to mark
 *                ICM as active (1) or not (0)
 * Output       : Success/Failure
 *
 */
u_int32_t ol_if_spectral_set_icm_active(void* arg, int is_active)
{
    struct ath_spectral *spectral = (struct ath_spectral *)arg;
    struct ieee80211com* ic = spectral->ic;
    struct ol_ath_softc_net80211 *scn = OL_ATH_SOFTC_NET80211(ic);
     
    scn->scn_icm_active = is_active;
    return 0;
}

/*
 * Function     : ol_init_spectral_ops
 * Description  : Initialize the Spectral ops
 * Input        : Pounter to IC
 * Output       : Void
 *
 */
void ol_init_spectral_ops(struct ieee80211com* ic)
{
    SPECTRAL_OPS* p_sops = &spectral_ops;

    p_sops->get_tsf64               = ol_if_spectral_get_tsf64;
    p_sops->get_capability          = ol_if_spectral_get_capability;
    p_sops->set_rxfilter            = ol_if_spectral_set_rxfilter;
    p_sops->get_rxfilter            = ol_if_spectral_get_rxfilter;
    p_sops->is_spectral_enabled     = ol_if_is_spectral_enabled;
    p_sops->is_spectral_active      = ol_if_is_spectral_active;
    p_sops->start_spectral_scan     = ol_if_start_spectral_scan;
    p_sops->stop_spectral_scan      = ol_if_stop_spectral_scan;
    p_sops->get_extension_channel   = ol_if_spectral_get_extension_channel;
    p_sops->get_ctl_noisefloor      = ol_if_spectral_get_ctl_noisefloor;
    p_sops->get_ext_noisefloor      = ol_if_spectral_get_ext_noisefloor;
    p_sops->configure_spectral      = ol_if_spectral_configure_params;
    p_sops->get_spectral_config     = ol_if_spectral_get_params;
    p_sops->get_ent_spectral_mask   = ol_if_spectral_get_ent_mask;
    p_sops->get_mac_address         = ol_if_spectral_get_macaddr;
    p_sops->get_current_channel     = ol_if_spectral_get_current_channel;
    p_sops->reset_hw                = ol_if_spectral_reset_hw;
    p_sops->get_chain_noise_floor   = ol_if_spectral_get_chain_noise_floor;
    p_sops->set_icm_active          = ol_if_spectral_set_icm_active;
    p_sops->get_nominal_nf          = ol_if_spectral_get_nominal_noisefloor;
}

/*
 * Function     : init_spectral_capability
 * Description  : Temp hack to enable Spectral capability
 * Input        : Pointer to spectral
 * Output       : Void
 *
 */
void init_spectral_capability(struct ath_spectral* spectral)
{
    struct ath_spectral_caps* pcap = &spectral->capability;
    /* XXX : Hack Spectral capability */
    pcap = &spectral->capability;
    pcap->phydiag_cap = 1;
    pcap->radar_cap = 1;
    pcap->spectral_cap = 1;
    pcap->advncd_spectral_cap = 1;
}

/*
 * Function     : ol_if_spectral_attach
 * Description  : Attach Spectral module
 * Input        : Pointer to IC
 * Output       : Success/Failure
 *
 */
static int
ol_if_spectral_attach(struct ieee80211com* ic)
{
    struct ath_spectral* spectral = NULL;

    int err = STATUS_PASS;
    if (spectral_attach(ic)) {
        ol_init_spectral_ops(ic);

        spectral = ic->ic_spectral;
        OL_SPECTRAL_LOCK_INIT(&spectral->ol_info.osps_lock);
        spectral->ol_info.osps_cache.osc_is_valid = 0;

        spectral_register_funcs(ic, &spectral_ops);

        if (spectral_check_hw_capability(ic) == AH_FALSE) {
            ol_if_spectral_detach(ic);
            err = STATUS_FAIL;
        }
    } else {
        err = STATUS_FAIL;
    }
    return err;
}

/*
 * Function     : ol_if_spectral_detach
 * Description  : Detach Spectral module
 * Input        : Pointer to IC
 * Output       : Success/Failure
 *
 */
int
ol_if_spectral_detach(struct ieee80211com *ic)
{
    printk("ol_if_spectral_detach\n");

    OL_SPECTRAL_LOCK_DESTROY(&ic->ic_spectral->ol_info.osps_lock);

    spectral_detach(ic);

    return (0); /* XXX error? */
}

/*
 * Function     : ol_if_spectral_attached
 * Description  : Check if Spectral module is attached
 * Input        : Pointer to IC
 * Output       : True/False
 *
 */
static int
ol_if_spectral_attached(struct ieee80211com *ic)
{
    int status = STATUS_PASS;
    printk("ol_if_spectral_attached\n");
    if (ic->ic_spectral == NULL) {
        if (ol_if_spectral_attach(ic) != AH_TRUE) {
            status = STATUS_FAIL;
        }
    }
    return status;
}

/*
 * Function     : ol_if_spectral_enable
 * Description  : Enable Spectral
 * Input        : Pointer to Spectral
 * Output       : Success/Failure
 *
 */
int
ol_if_spectral_enable(void* arg)
{
    printk("ol_if_spectral_enable\n");
    return (0);
}

/*
 * Function     : ol_if_spectral_disable
 * Description  : Disable Spectral
 * Input        : Pointer to Spectral
 * Output       : Success/Failure
 *
 */
static int
ol_if_spectral_disable(void* arg)
{
    printk("ol_if_spectral_disable\n");
    return (0);
}


/*
 * Function     : ol_if_spectral_setup
 * Description  : Initialzise the Spectral module for off-load driver
 * Input        : Pointer to IC
 * Output       : Void
 *
 */
int
ol_if_spectral_setup(struct ieee80211com *ic)
{

    int status = AH_TRUE;
    printk("ol_if_spectral_setup\n");
    ic->ic_spectral_control         = spectral_control;
    ic->ic_start_spectral_scan      = ol_if_ic_start_spectral_scan;
    ic->ic_stop_spectral_scan       = ol_if_ic_stop_spectral_scan;

    status = ol_if_spectral_attach(ic);

    return status;
}


#endif /* ATH_PERF_PWR_OFFLOAD */

#endif /* ATH_SUPPORT_DFS */
