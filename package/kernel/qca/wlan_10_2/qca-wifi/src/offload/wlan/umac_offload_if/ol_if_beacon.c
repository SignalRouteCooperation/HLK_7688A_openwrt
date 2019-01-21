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
 * UMAC beacon specific offload interface functions - for power and performance offload model
 */
#include "ol_if_athvar.h"
#include <ol_txrx_types.h>
#include <if_smart_ant.h>

#if ATH_PERF_PWR_OFFLOAD

/*
 *  WMI API for sending beacons
 */
#define BCN_SEND_BY_REF

void
wmi_unified_beacon_send(wmi_unified_t wmi_handle,
        struct ol_ath_softc_net80211 *scn,
        int vid,
        wbuf_t wbuf)
{
    if (ol_cfg_is_high_latency(NULL)) {
        wmi_bcn_tx_cmd *cmd;
        wmi_buf_t wmi_buf;
        int bcn_len= wbuf_get_pktlen(wbuf);
        int len = sizeof(wmi_bcn_tx_hdr) + bcn_len;

        /**********************************************************************
         * TODO: Once we have the host target transport framework for
         * sending management frames this wmi function will be replaced
         * with calls to HTT. The buffer will changed to match the right
         * format to be used with HTT.
         **********************************************************************/
        wmi_buf = wmi_buf_alloc(wmi_handle, roundup(len,sizeof(u_int32_t)));
        if (!wmi_buf) {
            printk("%s: wmi_buf_alloc failed\n", __FUNCTION__);
            return;
        }
        cmd = (wmi_bcn_tx_cmd *)wmi_buf_data(wmi_buf);
        cmd->hdr.vdev_id = vid;
        cmd->hdr.buf_len = bcn_len;

#ifdef BIG_ENDIAN_HOST
        {
            /* for big endian host, copy engine byte_swap is enabled
             * But the beacon buffer content is in network byte order
             * Need to byte swap the beacon buffer content - so when copy engine
             * does byte_swap - target gets buffer content in the correct order
             */ 
            int i;
            u_int32_t *destp, *srcp;
            destp = (u_int32_t *)cmd->bufp;
            srcp =  (u_int32_t *)wbuf_header(wbuf);
            for(i=0; i < (roundup(bcn_len, sizeof(u_int32_t))/4); i++) {
                *destp = le32_to_cpu(*srcp);
                destp++; srcp++;
            }
        }
#else
        OS_MEMCPY(cmd->bufp, wbuf_header(wbuf), bcn_len);
#endif

#ifdef DEBUG_BEACON
        printk("%s frm length %d \n",__func__,bcn_len);
#endif
        wmi_unified_cmd_send(wmi_handle, wmi_buf, roundup(len,sizeof(u_int32_t)), WMI_BCN_TX_CMDID);
    } else {
        wmi_bcn_send_from_host_cmd_t  *cmd;
        wmi_buf_t wmi_buf;
        A_UINT16  frame_ctrl;
        int bcn_len= wbuf_get_pktlen(wbuf);
        int len = sizeof(wmi_bcn_send_from_host_cmd_t);
        struct ieee80211_frame *wh;
        struct ieee80211_node *ni = wbuf_get_node(wbuf);
        struct ieee80211vap *vap = ni->ni_vap;
        struct ol_ath_vap_net80211 *avn = OL_ATH_VAP_NET80211(vap);
        struct ieee80211_beacon_offsets *bo = &avn->av_beacon_offsets;
        struct ieee80211_tim_ie *tim_ie = (struct ieee80211_tim_ie *)
            bo->bo_tim;
        struct ol_txrx_vdev_t *vdev = vap->iv_txrx_handle;
        struct ol_txrx_pdev_t *pdev = vdev->pdev;
        A_UINT32   dtim_flag = 0;
        A_UINT32 bcn_txant = 0;

        /* Get the frame ctrl field */
        wh = (struct ieee80211_frame *)wbuf_header(wbuf);
        frame_ctrl = *((A_UINT16 *)wh->i_fc);

        /* get the DTIM count */

        if (tim_ie->tim_count == 0) {
            dtim_flag |= WMI_BCN_SEND_DTIM_ZERO;
            if (tim_ie->tim_bitctl & 0x01) {
                /* deliver CAB traffic in next DTIM beacon */
                dtim_flag |= WMI_BCN_SEND_DTIM_BITCTL_SET;
            }
        }
        /* Map the beacon buffer to DMA region */
        adf_nbuf_map_single(pdev->osdev, wbuf, ADF_OS_DMA_TO_DEVICE);

        wmi_buf = wmi_buf_alloc(wmi_handle, roundup(len,sizeof(u_int32_t)));
        if (!wmi_buf) {
            printk("%s: wmi_buf_alloc failed\n", __FUNCTION__);
            return;
        }

        ieee80211_smart_ant_get_bcn_txantenna(ni->ni_ic, &bcn_txant);

        cmd = (wmi_bcn_send_from_host_cmd_t *)wmi_buf_data(wmi_buf);
        cmd->vdev_id = vid;
        cmd->data_len = bcn_len;
        cmd->frame_ctrl = frame_ctrl;
        cmd->dtim_flag = dtim_flag;
        cmd->frag_ptr = adf_nbuf_get_frag_paddr_lo(wbuf, 0);
        cmd->virt_addr = (A_UINT32)wbuf;
        cmd->bcn_antenna = bcn_txant;
        wmi_unified_cmd_send(wmi_handle, wmi_buf, len,  WMI_PDEV_SEND_BCN_CMDID);
    }
    return;
}

/*
 *  WMI API for sending beacon probe template
 */
void
wmi_unified_bcn_prb_template_send(wmi_unified_t wmi_handle, int vid,
                       int buf_len,  struct ieee80211_bcn_prb_info *bufp)
{
    wmi_bcn_prb_tmpl_cmd *cmd;
    wmi_buf_t buf;
    wmi_bcn_prb_info *template;
    int len = sizeof(wmi_bcn_prb_tmpl_cmd);  

    /*
     * The target will store this  information for use with
     * the beacons and probes. 
     */
    buf = wmi_buf_alloc(wmi_handle, len);
    if (!buf) {
        printk("%s: wmi_buf_alloc failed\n", __FUNCTION__);
        return;
    }
    cmd = (wmi_bcn_prb_tmpl_cmd *)wmi_buf_data(buf);
    cmd->vdev_id = vid;
    cmd->buf_len = buf_len;
    template = &cmd->bcn_prb_info;
    template->caps = bufp->caps;
    template->erp  = bufp->erp;

    /* TODO: Few more elements to be added and copied to the template buffer */ 

    /* Send the beacon probe template to the target */
    wmi_unified_cmd_send(wmi_handle, buf, len, WMI_BCN_PRB_TMPL_CMDID);
    return;
}

/* 
 * Function to update beacon probe template
 */
static void
ol_ath_beacon_probe_template_update(struct ieee80211_node *ni)
{
    struct ieee80211vap *vap = ni->ni_vap;
    struct ieee80211com *ic = vap->iv_ic;
    struct ol_ath_softc_net80211 *scn = OL_ATH_SOFTC_NET80211(ic);
    struct ol_ath_vap_net80211 *avn = OL_ATH_VAP_NET80211(vap);

    /* Update required only if we are in firmware offload mode */
    if (!avn->av_beacon_offload) {
         return;       
    }

    /* Populate the beacon probe template */
    if (!(ieee80211_bcn_prb_template_update(vap->iv_bss,
                                             &avn->av_bcn_prb_templ))) {
        wmi_unified_bcn_prb_template_send(scn->wmi_handle, avn->av_if_id,
                   sizeof(avn->av_bcn_prb_templ), &avn->av_bcn_prb_templ);
    } 

    return;
}

#if UMAC_SUPPORT_QUIET
/*
 *  WMI API for sending cmd to set/unset Quiet Mode
 */
static void
ol_ath_set_quiet_mode(struct ieee80211com *ic,uint8_t enable)
{
    struct ol_ath_softc_net80211 *scn = OL_ATH_SOFTC_NET80211(ic);
    struct ieee80211_quiet_param *quiet_ic = ic->ic_quiet;
    wmi_buf_t buf;
    wmi_pdev_set_quiet_cmd *quiet_cmd;
    int len = sizeof(wmi_pdev_set_quiet_cmd);
    buf = wmi_buf_alloc(scn->wmi_handle, len);
    if (!buf) {
        printk("%s:wmi_buf_alloc failed\n", __FUNCTION__);
        return ;
    }
    quiet_cmd = (wmi_pdev_set_quiet_cmd *)wmi_buf_data(buf);
    quiet_cmd->enabled = enable;
    quiet_cmd->period = quiet_ic->period*ic->ic_intval;
    quiet_cmd->duration = quiet_ic->duration;
    quiet_cmd->next_start = quiet_ic->offset;
    wmi_unified_cmd_send(scn->wmi_handle, buf, len, WMI_PDEV_SET_QUIET_MODE_CMDID);
}
#endif

int 
ol_ath_set_beacon_filter(wlan_if_t vap, u_int32_t *ie)
{
    /* Issue WMI command to set beacon filter */
    int i;
    struct ol_ath_vap_net80211 *avn = OL_ATH_VAP_NET80211(vap);
    struct ol_ath_softc_net80211 *scn = avn->av_sc;
    wmi_add_bcn_filter_cmd_t *cmd;
    A_STATUS res;
    wmi_buf_t buf = NULL;
    int len = sizeof(wmi_add_bcn_filter_cmd_t);
    buf = wmi_buf_alloc(scn->wmi_handle, len);
    if(!buf)
    {
        printk("buf alloc failed\n");
        return -ENOMEM;
    }
    cmd = (wmi_add_bcn_filter_cmd_t *)wmi_buf_data(buf);
    cmd->vdev_id = avn->av_if_id;
    printk("vdev_id: %d\n", cmd->vdev_id);
    
    for(i=0; i<BCN_FLT_MAX_ELEMS_IE_LIST; i++)
    {
        cmd->ie_map[i]=0;
    }
    
    if(ie)
    {
        for(i=0; i<BCN_FLT_MAX_ELEMS_IE_LIST; i++)
        {
            cmd->ie_map[i]=ie[i];
        }
    }
    res = wmi_unified_cmd_send(scn->wmi_handle, buf, len, WMI_ADD_BCN_FILTER_CMDID);
    return ((res == A_OK) ? EOK : -1);
}

int 
ol_ath_remove_beacon_filter(wlan_if_t vap)
{
    struct ol_ath_vap_net80211 *avn = OL_ATH_VAP_NET80211(vap);
    struct ol_ath_softc_net80211 *scn = avn->av_sc;
    wmi_rmv_bcn_filter_cmd_t *cmd;
    A_STATUS res;
    wmi_buf_t buf = NULL;
    int len = sizeof(wmi_rmv_bcn_filter_cmd_t);
    buf = wmi_buf_alloc(scn->wmi_handle, len);
    if(!buf)
    {
        printk("buf alloc failed\n");
        return -ENOMEM;
    }
    cmd = (wmi_rmv_bcn_filter_cmd_t *)wmi_buf_data(buf);
    cmd->vdev_id = avn->av_if_id;
    res = wmi_unified_cmd_send(scn->wmi_handle, buf, len, WMI_RMV_BCN_FILTER_CMDID);
    return ((res == A_OK) ? EOK : -1);
}

void 
ol_ath_set_probe_filter(void *data)
{
    /* TODO: Issue WMI command to set probe response filter */
    return;
}

static void
ol_ath_beacon_update(struct ieee80211_node *ni, int rssi)
{
    /* Stub for peregrine */
    return;
}

static int 
ol_ath_net80211_is_hwbeaconproc_active(struct ieee80211com *ic)
{
    /* Stub for peregrine */
    return 0;
}

static void 
ol_ath_net80211_hw_beacon_rssi_threshold_enable(struct ieee80211com *ic,
                                            u_int32_t rssi_threshold)
{
    /* TODO: Issue WMI command to set beacon RSSI filter */
    return;

}

static void 
ol_ath_net80211_hw_beacon_rssi_threshold_disable(struct ieee80211com *ic)
{
    /* TODO: Issue WMI command to disable beacon RSSI filter */
    return;
}

struct ol_ath_iter_update_beacon_arg {
    struct ieee80211com *ic;
    int if_id;
};

/* Move the beacon buffer to deferred_bcn_list */
static void
ol_ath_vap_defer_beacon_buf_free(struct ol_ath_vap_net80211 *avn)
{
    struct bcn_buf_entry* buf_entry;
    struct ol_ath_softc_net80211 *scn = avn->av_sc;
    buf_entry = (struct bcn_buf_entry *)adf_os_mem_alloc(scn->adf_dev,
	         sizeof(struct bcn_buf_entry));
    if (buf_entry) {
        adf_os_spin_lock(&avn->avn_lock);
	if(avn->av_wbuf == NULL){
		adf_os_spin_unlock(&avn->avn_lock);
		adf_os_mem_free(buf_entry);
		return;
	}
#ifdef BCN_SEND_BY_REF
        buf_entry->is_dma_mapped = avn->is_dma_mapped;
        /* cleat dma_mapped flag */
        avn->is_dma_mapped = 0;
#endif
        buf_entry->bcn_buf = avn->av_wbuf;
        TAILQ_INSERT_TAIL(&avn->deferred_bcn_list, buf_entry, deferred_bcn_list_elem);
        avn->av_wbuf =  NULL;
        adf_os_spin_unlock(&avn->avn_lock);
    }
    else {
        printk("ERROR: adf_os_mem_alloc failed %s: %d \n", __func__, __LINE__);
        ASSERT(0);
    }
}

/* 
 * Function to allocate beacon in host mode
 */
static void
ol_ath_vap_iter_beacon_alloc(void *arg, wlan_if_t vap) 
{
    struct ol_ath_iter_update_beacon_arg* params = (struct ol_ath_iter_update_beacon_arg *)arg;
    struct ol_ath_vap_net80211 *avn = OL_ATH_VAP_NET80211(vap);
    struct ieee80211_node *ni;

    if (avn->av_if_id == params->if_id) {
        if (avn->av_beacon_offload) {
            printk("Beacon processing offloaded to the firmware\n");
        } else {
            ni = vap->iv_bss;
	    /* Beacon buffer is already allocate 
	     * Move the beacon buffer to deferred_bcn_list to 
	     * free the buffer on vap stop
	     * and allocate a new beacon buufer
	     */
	     ol_ath_vap_defer_beacon_buf_free(avn);
	     avn->av_wbuf = ieee80211_beacon_alloc(ni, &avn->av_beacon_offsets);
	     if (avn->av_wbuf == NULL) {
                   printk("ERROR ieee80211_beacon_alloc failed in %s:%d\n", __func__, __LINE__);
             A_ASSERT(0);
            }
        }
    }
    return;
}

/* 
 * Function to free beacon in host mode
 */
static void
ol_ath_vap_iter_beacon_free(void *arg, wlan_if_t vap) 
{
    struct ol_ath_iter_update_beacon_arg* params = (struct ol_ath_iter_update_beacon_arg *)arg;
    struct ol_ath_vap_net80211 *avn = OL_ATH_VAP_NET80211(vap);
#ifdef BCN_SEND_BY_REF
    struct ieee80211com *ic = vap->iv_ic;
    struct ol_ath_softc_net80211 *scn = OL_ATH_SOFTC_NET80211(ic);
#endif

    if (avn->av_if_id == params->if_id) {
        if (avn->av_beacon_offload) {
            printk("Beacon processing offloaded to the firmware\n");
        } else {
            struct bcn_buf_entry* buf_entry,*buf_temp;
            struct ieee80211_tx_status ts;
            ts.ts_flags = 0;
            ts.ts_retries = 0;
            adf_os_spin_lock(&avn->avn_lock);
            TAILQ_FOREACH_SAFE(buf_entry, &avn->deferred_bcn_list, deferred_bcn_list_elem,buf_temp) {
                TAILQ_REMOVE(&avn->deferred_bcn_list, buf_entry, deferred_bcn_list_elem);
#ifdef BCN_SEND_BY_REF
                if (buf_entry->is_dma_mapped == 1) {
                    adf_nbuf_unmap_single(scn->adf_dev,
                              buf_entry->bcn_buf,
                              ADF_OS_DMA_TO_DEVICE);
                    buf_entry->is_dma_mapped = 0;
                }
#endif
                ieee80211_complete_wbuf(buf_entry->bcn_buf, &ts);
                adf_os_mem_free(buf_entry);
            }
            adf_os_spin_unlock(&avn->avn_lock);
        }
    }
    return;
}

/*
 * offload beacon APIs for other offload modules
 */
void
ol_ath_beacon_alloc(struct ieee80211com *ic, int if_id)
{
    struct ol_ath_iter_update_beacon_arg params; 

    params.ic = ic;
    params.if_id = if_id;
    wlan_iterate_vap_list(ic,ol_ath_vap_iter_beacon_alloc,(void *) &params);
}

void
ol_ath_beacon_stop(struct ol_ath_softc_net80211 *scn,
                   struct ol_ath_vap_net80211 *avn)
{
    if (avn->av_beacon_offload) {
        printk("Beacon processing offloaded to the firmware\n");
    } else {
        struct ieee80211_tx_status ts;
        ts.ts_flags = 0;
        ts.ts_retries = 0;
	/* Move the beacon buffer to deferred_bcn_list 
	 * and wait for stooped event from Target.
	 * beacon buffer in deferred_bcn_list gets freed - on 
	 * vap stopped event from Target
	 */
	 ol_ath_vap_defer_beacon_buf_free(avn);
     }
     return;
}


void
ol_ath_beacon_free(struct ieee80211com *ic, int if_id)
{
    struct ol_ath_iter_update_beacon_arg params;

    params.ic = ic;
    params.if_id = if_id;
    wlan_iterate_vap_list(ic,ol_ath_vap_iter_beacon_free,(void *) &params);
}
   
#if UMAC_SUPPORT_QUIET
/*
 * Function to update quiet element in the beacon and the VAP quite params 
 */
static void
ol_ath_update_quiet_params(struct ieee80211com *ic, struct ieee80211vap *vap)
{

    struct ieee80211_quiet_param *quiet_iv = vap->iv_quiet;
    struct ieee80211_quiet_param *quiet_ic = ic->ic_quiet;

    /* Update quiet params for the vap with beacon offset 0 */
    if (quiet_ic->is_enabled) {

        struct ol_ath_vap_net80211 *avn = OL_ATH_VAP_NET80211(vap);
        u_int32_t tsf_adj = avn->av_tsfadjust;

        /* convert tsf adjust in to TU */
        tsf_adj = tsf_adj >> 10;

        /* Compute the beacon_offset from slot 0 */
        if (tsf_adj) {
           quiet_iv->beacon_offset = ic->ic_intval - tsf_adj;
        }
        else {
            quiet_iv->beacon_offset = 0;
        }

		if (vap->iv_unit == 0) {
			quiet_ic->beacon_offset = quiet_iv->beacon_offset;

			if (quiet_ic->tbttcount == 1) {
				quiet_ic->tbttcount = quiet_ic->period;
			}
			else {
				quiet_ic->tbttcount--;
			}
		
			if (quiet_ic->tbttcount == 1) {
				ol_ath_set_quiet_mode(ic,1);
			}
			else if (quiet_ic->tbttcount == (quiet_ic->period-1)) {
				ol_ath_set_quiet_mode(ic,0);
			}
				
		} 
	} else if (quiet_ic->tbttcount != quiet_ic->period) {
  		/* quiet support is disabled
         * since tbttcount is not '0', the hw quiet period was set before
         * so just disable the hw quiet period and 
         * tbttcount to 0.
         */
		quiet_ic->tbttcount = quiet_ic->period;
		ol_ath_set_quiet_mode(ic,0);
	}
}
#endif /* UMAC_SUPPORT_QUIET */

/*
 * Return the appropriate VAP given the Id
 */
struct ieee80211vap *
ol_ath_get_vap(struct ieee80211com *ic, u_int32_t if_id)
{
    struct ieee80211vap *vap = NULL;
    struct ol_ath_vap_net80211 *avn;

    /*  Get a VAP with the given id.  */
    TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) {
        if ((OL_ATH_VAP_NET80211(vap))->av_if_id == if_id) {
            break;
        }
    }

    if (vap == NULL) {
        return NULL;
    }

#if UMAC_SUPPORT_WDS
    /* Disable beacon if VAP is operating in NAWDS bridge mode */
    if (ieee80211_nawds_disable_beacon(vap)){
        return NULL;
    }
#endif

    /* Allow this only for host beaconing mode */
    avn = OL_ATH_VAP_NET80211(vap);
    if (avn->av_beacon_offload) {
        return NULL;
    }

    return vap;
}

/* 
 * Handler for Host SWBA events in host mode
 */
static int 
ol_ath_beacon_swba_handler(struct ol_ath_softc_net80211 *scn,
                           wmi_host_swba_event *swba_event)
{
    struct ieee80211com *ic = &scn->sc_ic;
    struct ieee80211vap *vap = NULL;
    struct ol_ath_vap_net80211 *avn;
    wmi_bcn_info *bcn_info;
    wmi_tim_info *tim_info;
    u_int32_t if_id; 
    int error = 0;
    u_int32_t vdev_map = swba_event->vdev_map;
    u_int8_t vdev_id=0,i=0;

    /* Generate a beacon for all the vaps specified in the list */
    for (;vdev_map; vdev_id++, vdev_map >>= 1) {

        if (!(vdev_map & 0x1)) {
            continue;    
        }

        bcn_info = &swba_event->bcn_info[i];
        if_id = vdev_id;

        /* Get the VAP corresponding to the id */
        vap = ol_ath_get_vap( ic, if_id);
        if (vap == NULL) {
            return -1;
        }

        /*
         * Update the TIM bitmap. At VAP attach memory will be allocated for TIM
         * based on the iv_max_aid set. Update this field and beacon update will
         * automatically take care of populating the bitmap in the beacon buffer.
         */
        tim_info = &bcn_info->tim_info;
        if (vap->iv_tim_bitmap && tim_info->tim_changed) {

            /* The tim bitmap is a byte array that is passed through WMI as a
             * 32bit word array. The CE will correct for endianess which is
             * _not_ what we want here. Un-swap the words so that the byte
             * array is in the correct order.
             */
#ifdef BIG_ENDIAN_HOST 
            int j;
            for (j = 0; j < WMI_TIM_BITMAP_ARRAY_SIZE; j++) {
                tim_info->tim_bitmap[j] = le32_to_cpu(tim_info->tim_bitmap[j]);
            }
#endif

            vap->iv_tim_len = (u_int16_t)tim_info->tim_len;
            OS_MEMCPY(vap->iv_tim_bitmap, tim_info->tim_bitmap, vap->iv_tim_len);  
            vap->iv_ps_pending = tim_info->tim_num_ps_pending;

            IEEE80211_VAP_TIMUPDATE_ENABLE(vap);
        }

        /* Update quiet params and beacon update will take care of the rest */
        avn = OL_ATH_VAP_NET80211(vap);
     	adf_os_spin_lock(&avn->avn_lock);

        if (avn->av_wbuf == NULL) {
            printk("beacon buffer av_wbuf is NULL - Ignoring SWBA event \n");
	    adf_os_spin_unlock(&avn->avn_lock);
            return -1;
        }

#if UMAC_SUPPORT_QUIET
        ol_ath_update_quiet_params(ic, vap);
#endif /* UMAC_SUPPORT_QUIET */
#if UMAC_SUPPORT_WNM
        error = ieee80211_beacon_update(vap->iv_bss, &avn->av_beacon_offsets,
                                    avn->av_wbuf, tim_info->tim_mcast,0);
#else
        error = ieee80211_beacon_update(vap->iv_bss, &avn->av_beacon_offsets,
                                    avn->av_wbuf, tim_info->tim_mcast);
#endif
        if (error != -1) {
            /* Send beacon to target */
#ifdef BCN_SEND_BY_REF
    if (avn->is_dma_mapped) {
    struct ol_txrx_vdev_t *vdev = vap->iv_txrx_handle;  
    struct ol_txrx_pdev_t *pdev = vdev->pdev;  

        adf_nbuf_unmap_single(pdev->osdev,
                              avn->av_wbuf,
                              ADF_OS_DMA_TO_DEVICE);
        avn->is_dma_mapped = 0;
    }

#endif

            wmi_unified_beacon_send(scn->wmi_handle, scn, if_id, avn->av_wbuf);

#ifdef BCN_SEND_BY_REF
    avn->is_dma_mapped = 1;
#endif
        }
	adf_os_spin_unlock(&avn->avn_lock); 

        i++;
    }

    return 0;
}

/*
 * TSF Offset event handler 
 */
static int
ol_ath_tsf_offset_event_handler(struct ol_ath_softc_net80211 *scn,
                         u_int32_t vdev_map, u_int32_t *adjusted_tsf)
{
    struct ieee80211com *ic = &scn->sc_ic;
    struct ieee80211vap *vap = NULL;
    struct ol_ath_vap_net80211 *avn;
    struct ieee80211_frame  *wh;
    u_int32_t if_id=0;
    u_int8_t i=0;
    u_int64_t adjusted_tsf_le;

    for ( ;(vdev_map); vdev_map >>= 1, if_id++) {

        if (!(vdev_map & 0x1)) {
            continue;
        }

        /* Get the VAP corresponding to the id */
        vap = ol_ath_get_vap( ic, if_id);
        if (vap == NULL) {
            return -1;
        }

        avn = OL_ATH_VAP_NET80211(vap);
        if (avn->av_wbuf == NULL) {
            printk("beacon buffer av_wbuf is NULL - Ignoring tsf offset event \n");
            return -1;
        }

        /* Save the adjusted TSF */
        avn->av_tsfadjust = adjusted_tsf[i];

        /* 
         * Make the TSF offset negative so beacons in the same staggered batch
         * have the same TSF. 
         */
        adjusted_tsf_le = cpu_to_le64(0ULL - avn->av_tsfadjust);

        i++;

        /* Update the timstamp in the beacon buffer with adjusted TSF */
        wh = (struct ieee80211_frame *)wbuf_header(avn->av_wbuf);
        OS_MEMCPY(&wh[1], &adjusted_tsf_le, sizeof(adjusted_tsf_le));
    }

    return 0;
}

/* WMI Beacon related Event APIs */
static int
wmi_beacon_swba_handler(ol_scn_t scn, u_int8_t *data, u_int16_t datalen, void *context)
{
    wmi_host_swba_event *swba_event = (wmi_host_swba_event *)data;
    return ol_ath_beacon_swba_handler(scn, swba_event);
}

static int
wmi_tbttoffset_update_event_handler(ol_scn_t scn, u_int8_t *data, u_int16_t datalen, void *context)
{
    wmi_tbtt_offset_event *tbtt_offset_event = (wmi_tbtt_offset_event *)data;
    return ol_ath_tsf_offset_event_handler(scn, tbtt_offset_event->vdev_map,
                     tbtt_offset_event->tbttoffset_list);
}


/*
 * Beacon related attach functions for offload solutions
 */
void
ol_ath_beacon_attach(struct ieee80211com *ic)
{
    struct ol_ath_softc_net80211 *scn = OL_ATH_SOFTC_NET80211(ic);
    ic->ic_beacon_probe_template_update = ol_ath_beacon_probe_template_update;
    ic->ic_beacon_update = ol_ath_beacon_update;
    ic->ic_is_hwbeaconproc_active = ol_ath_net80211_is_hwbeaconproc_active;
    ic->ic_hw_beacon_rssi_threshold_enable = ol_ath_net80211_hw_beacon_rssi_threshold_enable;
    ic->ic_hw_beacon_rssi_threshold_disable = ol_ath_net80211_hw_beacon_rssi_threshold_disable;

    /* Register WMI event handlers */
    wmi_unified_register_event_handler(scn->wmi_handle, WMI_HOST_SWBA_EVENTID, 
                                            wmi_beacon_swba_handler, NULL);
    wmi_unified_register_event_handler(scn->wmi_handle, WMI_TBTTOFFSET_UPDATE_EVENTID,
                                           wmi_tbttoffset_update_event_handler, NULL);
}
#endif
