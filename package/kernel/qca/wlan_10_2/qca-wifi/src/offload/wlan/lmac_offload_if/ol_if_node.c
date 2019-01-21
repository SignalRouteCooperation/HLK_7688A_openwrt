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
 * LMAC VAP specific offload interface functions for UMAC - for power and performance offload model
 */
#include "ol_if_athvar.h"
#include "ol_ctrl_txrx_api.h"
#include "wmi_unified_api.h"
#include "wmi_unified.h"
#if ATH_SUPPORT_GREEN_AP
#include "ath_green_ap.h"
#endif  /* ATH_SUPPORT_GREEN_AP */

#if ATH_PERF_PWR_OFFLOAD

#ifndef HT_RC_2_STREAMS
#define HT_RC_2_STREAMS(_rc)    ((((_rc) & 0x78) >> 3) + 1)
#endif

#define NUM_LEGACY_RATES 12
#define L_BPS_COL         0
#define L_RC_COL          1

#define ALD_MSDU_SIZE 1300
#define DEFAULT_MSDU_SIZE 1000
static const int legacy_rate_idx[][2] = {
    {1,        0x1b},
    {2,        0x1a},
    {5,        0x19},
    {6,        0xb},
    {9,        0xf},
    {11,       0x18},
    {12,       0xa},
    {18,       0xe},
    {24,       0x9},
    {36,       0xd},
    {48,       0x8},
    {54,       0xc},
};

#define NUM_VHT_HT_RATES 30
#define BW_COL 0
#define MCS_COL 1
#define NBPS_COL 2
static const int vht_ht_tbl[][3] = {
   /*BW        MCS       Data bits per symbol*/
    {0,        0,        26},
    {0,        1,        52},
    {0,        2,        78},
    {0,        3,        104},
    {0,        4,        156},
    {0,        5,        208},
    {0,        6,        234},
    {0,        7,        260},
    {0,        8,        312},
    {0,        9,        347},
    {1,        0,        54},
    {1,        1,        108},
    {1,        2,        162},
    {1,        3,        216},
    {1,        4,        324},
    {1,        5,        432},
    {1,        6,        486},
    {1,        7,        540},
    {1,        8,        648},
    {1,        9,        720},
    {2,        0,        117},
    {2,        1,        234},
    {2,        2,        351},
    {2,        3,        468},
    {2,        4,        702},
    {2,        5,        936},
    {2,        6,        1053},
    {2,        7,        1170},
    {2,        8,        1404},
    {2,        9,        1560},
};

#define NG_VHT_MODES 3

                                                                         /* 1x1    1x1    2x2    2x2    3x3    3x3
                                                                             GI    SGI    GI     SGI    GI     SGI */
static u_int32_t max_rates[IEEE80211_MODE_11AC_VHT80+NG_VHT_MODES+1][6] = {{0,     0,     0,     0,     0,     0},
                                                                           {5400,  5400,  5400,  5400,  5400,  5400},
                                                                           {1100,  1100,  1100,  1100,  1100,  1100},
                                                                           {5400,  5400,  5400,  5400,  5400,  5400},
                                                                           {0,     0,     0,     0,     0,     0},
                                                                           {0,     0,     0,     0,     0,     0},
                                                                           {0,     0,     0,     0,     0,     0},
                                                                           {6500,  7222, 13000, 14444, 19500, 21667},
                                                                           {6500,  7222, 13000, 14444, 19500, 21667},
                                                                           {13500, 15000, 27000, 30000, 40500, 45000},
                                                                           {13500, 15000, 27000, 30000, 40500, 45000},
                                                                           {13500, 15000, 27000, 30000, 40500, 45000},
                                                                           {13500, 15000, 27000, 30000, 40500, 45000},
                                                                           {13500, 15000, 27000, 30000, 40500, 45000},
                                                                           {13500, 15000, 27000, 30000, 40500, 45000},
                                                                           {7800,  8670,  15600, 17330, 26000, 28890},
                                                                           {18000, 20000, 36000, 40000, 54000, 60000},
                                                                           {18000, 20000, 36000, 40000, 54000, 60000},
                                                                           {18000, 20000, 36000, 40000, 54000, 60000},
                                                                           {39000, 43330, 78000, 86670, 117000,130000},
                                                                           {7800,  8670,  15600, 17330, 26000, 28890},
                                                                           {8650,  9600,  17300, 19200, 26000, 28890},
                                                                           {18000, 20000, 36000, 40000, 54000, 60000},
                                                                                                                     };

/* WMI interface functions */
int
wmi_unified_peer_create_send(wmi_unified_t wmi_handle, const u_int8_t *peer_addr, u_int32_t vdev_id )
{
    wmi_peer_create_cmd* cmd;
    wmi_buf_t buf;
    int len = sizeof(wmi_peer_create_cmd);
    buf = wmi_buf_alloc(wmi_handle, len);
    if (!buf) {
        printk("%s:wmi_buf_alloc failed\n", __FUNCTION__);
        return ENOMEM;
    }
    cmd = (wmi_peer_create_cmd *)wmi_buf_data(buf);
    WMI_CHAR_ARRAY_TO_MAC_ADDR(peer_addr, &cmd->peer_macaddr);
    cmd->vdev_id = vdev_id;
    return wmi_unified_cmd_send(wmi_handle, buf, len, WMI_PEER_CREATE_CMDID);
}

int
wmi_unified_peer_delete_send(wmi_unified_t wmi_handle, u_int8_t peer_addr[IEEE80211_ADDR_LEN], u_int32_t vdev_id )
{
    wmi_peer_delete_cmd* cmd;
    wmi_buf_t buf;
    int len = sizeof(wmi_peer_delete_cmd);
    buf = wmi_buf_alloc(wmi_handle, len);
    if (!buf) {
        printk("%s:wmi_buf_alloc failed\n", __FUNCTION__);
        return ENOMEM;
    }
    cmd = (wmi_peer_delete_cmd *)wmi_buf_data(buf);
    WMI_CHAR_ARRAY_TO_MAC_ADDR(peer_addr, &cmd->peer_macaddr);
    cmd->vdev_id = vdev_id;
    return wmi_unified_cmd_send(wmi_handle, buf, len, WMI_PEER_DELETE_CMDID);

}

int
wmi_unified_peer_flush_tids_send(wmi_unified_t wmi_handle,
                                 u_int8_t peer_addr[IEEE80211_ADDR_LEN],
                                 u_int32_t peer_tid_bitmap,
                                 u_int32_t vdev_id)
{
    wmi_peer_flush_tids_cmd* cmd;
    wmi_buf_t buf;
    int len = sizeof(wmi_peer_flush_tids_cmd);
    buf = wmi_buf_alloc(wmi_handle, len);
    if (!buf) {
        printk("%s:wmi_buf_alloc failed\n", __FUNCTION__);
        return ENOMEM;
    }
    cmd = (wmi_peer_flush_tids_cmd *)wmi_buf_data(buf);
    WMI_CHAR_ARRAY_TO_MAC_ADDR(peer_addr, &cmd->peer_macaddr);
    cmd->peer_tid_bitmap = peer_tid_bitmap;
    cmd->vdev_id = vdev_id;
    return wmi_unified_cmd_send(wmi_handle, buf, len, WMI_PEER_FLUSH_TIDS_CMDID);
}

int
wmi_unified_node_add_wds_entry(wmi_unified_t wmi_handle, const u_int8_t *dest_addr,
    u_int8_t *peer_addr, u_int32_t flags)
{
    wmi_peer_add_wds_entry_cmd* cmd;
    wmi_buf_t buf;
    int len = sizeof(wmi_peer_add_wds_entry_cmd);

    buf = wmi_buf_alloc(wmi_handle, len);
    if (!buf) {
        printk("%s: wmi_buf_alloc failed\n", __func__);
        return -1;
    }
    cmd = (wmi_peer_add_wds_entry_cmd *)wmi_buf_data(buf);
    WMI_CHAR_ARRAY_TO_MAC_ADDR(dest_addr, &cmd->wds_macaddr);
    WMI_CHAR_ARRAY_TO_MAC_ADDR(peer_addr, &cmd->peer_macaddr);
    cmd->flags = flags;
    return wmi_unified_cmd_send(wmi_handle, buf, len, WMI_PEER_ADD_WDS_ENTRY_CMDID);
}

int
wmi_unified_node_del_wds_entry(wmi_unified_t wmi_handle, u_int8_t *dest_addr)
{
    wmi_peer_remove_wds_entry_cmd* cmd;
    wmi_buf_t buf;
    int len = sizeof(wmi_peer_remove_wds_entry_cmd);

    buf = wmi_buf_alloc(wmi_handle, len);
    if (!buf) {
        printk("%s: wmi_buf_alloc failed\n", __func__);
        return ENOMEM;
    }
    cmd = (wmi_peer_remove_wds_entry_cmd *)wmi_buf_data(buf);
    WMI_CHAR_ARRAY_TO_MAC_ADDR(dest_addr, &cmd->wds_macaddr);
    return wmi_unified_cmd_send(wmi_handle, buf, len, WMI_PEER_REMOVE_WDS_ENTRY_CMDID);
}

int
wmi_unified_node_set_param(wmi_unified_t wmi_handle, u_int8_t *peer_addr,u_int32_t param_id,
        u_int32_t param_val,u_int32_t vdev_id)
{
    wmi_peer_set_param_cmd *cmd;
    wmi_buf_t buf;
    int len = sizeof(wmi_peer_set_param_cmd);

    buf = wmi_buf_alloc(wmi_handle, len);
    if(!buf) {
        printk("%s: wmi_buf_alloc failed\n", __func__);
        return ENOMEM;
    }
    cmd = (wmi_peer_set_param_cmd *)wmi_buf_data(buf);
    WMI_CHAR_ARRAY_TO_MAC_ADDR(peer_addr, &cmd->peer_macaddr);
    cmd->param_id = param_id;
    cmd->param_value = param_val;
    cmd->vdev_id = vdev_id;
    return wmi_unified_cmd_send(wmi_handle, buf, len, WMI_PEER_SET_PARAM_CMDID);
}

int
wmi_send_node_rate_sched(struct ol_ath_softc_net80211 *scn,
        wmi_peer_rate_retry_sched_cmd *cmd_buf)
{
    wmi_unified_t wmi_handle = scn->wmi_handle;

    wmi_peer_rate_retry_sched_cmd *cmd;
    wmi_buf_t buf;
    int len = sizeof(wmi_peer_rate_retry_sched_cmd);

    buf = wmi_buf_alloc(wmi_handle, len);
    if(!buf) {
        printk("%s: wmi_buf_alloc failed\n", __func__);
        return ENOMEM;
    }

    cmd = (wmi_peer_rate_retry_sched_cmd *)wmi_buf_data(buf);

    adf_os_mem_copy(cmd, cmd_buf, len);

    return (wmi_unified_cmd_send(
                    wmi_handle,
                    buf,
                    len,
                    WMI_PEER_RATE_RETRY_SCHED_CMDID));
}

static void
set_node_wep_keys(struct ieee80211vap *vap, const u_int8_t *macaddr)
{
    struct ieee80211com *ic = vap->iv_ic;
    struct ol_ath_softc_net80211 *scn = OL_ATH_SOFTC_NET80211(ic);
    u_int8_t keymac[IEEE80211_ADDR_LEN];
    struct ieee80211_key *wkey;
    struct ieee80211_rsnparms *rsn = &vap->iv_rsn;
    int i, opmode;

    wkey = (struct ieee80211_key *)OS_MALLOC(scn->sc_osdev,
						sizeof(struct ieee80211_key),
                                                0);
    OS_MEMZERO(wkey, sizeof(struct ieee80211_key));
    opmode = ieee80211vap_get_opmode(vap);

    /* push only valid static WEP keys from vap */

    if (RSN_AUTH_IS_8021X(rsn)) {
        OS_FREE(wkey);
        return ;
    }

    for(i=0;i<IEEE80211_WEP_NKID;i++) {

        OS_MEMCPY(wkey,&vap->iv_nw_keys[i],sizeof(struct ieee80211_key));

        if(wkey->wk_valid && wkey->wk_cipher->ic_cipher==IEEE80211_CIPHER_WEP) {
               IEEE80211_ADDR_COPY(keymac,macaddr);

            /* setting the broadcast/multicast key for sta */
            if(opmode == IEEE80211_M_STA || opmode == IEEE80211_M_IBSS){
                vap->iv_key_set(vap, wkey, keymac);
            }

            /* setting unicast key */
            wkey->wk_flags &= ~IEEE80211_KEY_GROUP;
            vap->iv_key_set(vap, wkey, keymac);
        }
    }
    OS_FREE(wkey);

}

static A_BOOL
is_node_self_peer(struct ieee80211vap *vap, const u_int8_t *macaddr)
{
    A_BOOL is_self_peer = FALSE;

    switch (vap->iv_opmode) {
    case IEEE80211_M_STA:
        if (IEEE80211_ADDR_EQ(macaddr, vap->iv_myaddr)) {
            is_self_peer = TRUE;
        }
        break;
    default:
        break;
    }

    return is_self_peer;
}

/* Interface functions */
static struct ieee80211_node *
ol_ath_node_alloc(struct ieee80211vap *vap, const u_int8_t *macaddr, bool tmpnode)
{
    struct ieee80211com *ic = vap->iv_ic;
    struct ol_ath_vap_net80211 *avn = OL_ATH_VAP_NET80211(vap);
    struct ol_ath_softc_net80211 *scn = OL_ATH_SOFTC_NET80211(ic);
    struct ol_ath_node_net80211 *anode;

    adf_os_spin_lock_bh(&scn->scn_lock);
    scn->peer_count++;
    if (scn->peer_count > scn->wlan_resource_config.num_peers) {
        adf_os_spin_unlock_bh(&scn->scn_lock);
        printk("%s: vap (%p) scn (%p) the peer count exceeds the supported number %d \n",
                __func__, vap, scn, scn->wlan_resource_config.num_peers);

        struct ieee80211vap *tmpvap;
        TAILQ_FOREACH(tmpvap, &ic->ic_vaps, iv_next) {
            if (tmpvap) {
                printk("=NODE_INFO=:%s:<-------------Node Status info for VAP (%p) ---------->\n", __func__, tmpvap);
                printk("Total Node count:%d auth_req_cnt:%d deauth_req_cnt:%d assoc_req_cnt:%d disassoc_req_cnt:%d node_create_cnt:%d node_del_cnt:%d  kickout_cnt:%d\n\n"
                        , tmpvap->auth_req_cnt, tmpvap->deauth_req_cnt, tmpvap->iv_node_count, tmpvap->assoc_req_cnt
                        , tmpvap->disassoc_req_cnt, tmpvap->node_create_cnt, tmpvap->node_del_cnt , tmpvap->kickout_cnt);
            }
        }
        goto err_node_alloc;
    }
    adf_os_spin_unlock_bh(&scn->scn_lock);

    anode = (struct ol_ath_node_net80211 *)OS_MALLOC(scn->sc_osdev,
                                                  sizeof(struct ol_ath_node_net80211),
                                                  GFP_ATOMIC);
    if (anode == NULL)
        goto err_node_alloc;

    OS_MEMZERO(anode, sizeof(struct ol_ath_node_net80211));

    anode->an_node.ni_vap = vap;

    /* do not create/delete peer on target for temp nodes and self-peers */
    if (!tmpnode && !is_node_self_peer(vap, macaddr)) {
        if (wmi_unified_peer_create_send(scn->wmi_handle, macaddr,avn->av_if_id)) {
            printk("%s : Unable to create peer in Target \n", __func__);
            OS_FREE(anode);
            goto err_node_alloc;
        }

        adf_os_spin_lock_bh(&scn->scn_lock);
        anode->an_txrx_handle = ol_txrx_peer_attach(scn->pdev_txrx_handle,
                avn->av_txrx_handle, (u_int8_t *) macaddr);

        if (anode->an_txrx_handle == NULL) {
			adf_os_spin_unlock_bh(&scn->scn_lock);
            printk("%s : Unable to attach txrx peer\n", __func__);
            OS_FREE(anode);
            goto err_node_alloc;
        }
        adf_os_spin_unlock_bh(&scn->scn_lock);

        /* static wep keys stored in vap needs to be
         * pushed to all nodes except self node
         */
        if(IEEE80211_VAP_IS_PRIVACY_ENABLED(vap) &&
                (OS_MEMCMP(macaddr,vap->iv_myaddr,IEEE80211_ADDR_LEN) != 0 )) {
            set_node_wep_keys(vap,macaddr);
        }
    }

    vap->node_create_cnt++;

    return &anode->an_node;

err_node_alloc:
    adf_os_spin_lock_bh(&scn->scn_lock);
    scn->peer_count--;
    adf_os_spin_unlock_bh(&scn->scn_lock);
    return NULL;

}

static void
ol_ath_node_free(struct ieee80211_node *ni)
{
    struct ieee80211com *ic = ni->ni_ic;
    struct ol_ath_softc_net80211 *scn = OL_ATH_SOFTC_NET80211(ic);
    u_int32_t ni_flags = ni->ni_flags;

    adf_os_spin_lock_bh(&scn->scn_lock);
    if(scn->peer_count) {
        scn->peer_count--;
    } else {
        printk("%s ni (%p) vap (%p) scn (%p) Decremnting '0' peer count \n",
                __func__, ni, ni->ni_vap, scn);
    }
    if (ni_flags & IEEE80211_NODE_EXT_STATS) {
        scn->peer_ext_stats_count--;
    }
    adf_os_spin_unlock_bh(&scn->scn_lock);
    /* Call back the umac node free function */
    scn->net80211_node_free(ni);
}

static void
ol_ath_node_cleanup(struct ieee80211_node *ni)
{
    struct ieee80211com *ic = ni->ni_ic;
    struct ol_ath_softc_net80211 *scn = OL_ATH_SOFTC_NET80211(ic);
    struct ol_ath_vap_net80211 *avn = OL_ATH_VAP_NET80211(ni->ni_vap);
    u_int32_t peer_tid_bitmap = 0xffffffff; /* TBD : fill with all valid TIDs */

    /* flush all TIDs except MGMT TID for this peer in Target */
    peer_tid_bitmap &= ~(0x1 << WMI_MGMT_TID);
    if (wmi_unified_peer_flush_tids_send(scn->wmi_handle, ni->ni_macaddr, peer_tid_bitmap, avn->av_if_id)) {
        printk("%s : Unable to Flush tids peer in Target \n", __func__);
    }
    /* TBD: Cleanup the key index mapping */

    adf_os_spin_lock_bh(&scn->scn_lock);
    if ((OL_ATH_NODE_NET80211(ni))->an_txrx_handle) {
        ol_txrx_peer_detach( (OL_ATH_NODE_NET80211(ni))->an_txrx_handle);

#if ATH_SUPPORT_GREEN_AP
        if ((ic->ic_opmode == IEEE80211_M_HOSTAP) && (ni != ni->ni_bss_node)) {
            ath_green_ap_state_mc(ic, ATH_PS_EVENT_DEC_STA);
        }
#endif  /* ATH_SUPPORT_GREEN_AP */

        /* Delete key */
        ieee80211_node_clear_keys(ni);
        /* Delete peer in Target */
        if (wmi_unified_peer_delete_send(scn->wmi_handle, ni->ni_macaddr, avn->av_if_id)) {
            printk("%s : Unable to Delete peer in Target \n", __func__);
        }
        /*
         * It is possible that a node will be cleaned up for multiple times
         * before it is freed. Make sure we only remove TxRx/FW peer once.
         */
        (OL_ATH_NODE_NET80211(ni))->an_txrx_handle = NULL;
    }
    adf_os_spin_unlock_bh(&scn->scn_lock);

    /* Call back the umac node cleanup function */
    scn->net80211_node_cleanup(ni);


}

static int8_t
ol_ath_node_getrssi(const struct ieee80211_node *ni,int8_t chain, u_int8_t flags )
{
    return ni->ni_rssi;
}

static u_int32_t
ol_ath_node_getrate(const struct ieee80211_node *ni, u_int8_t type)
{
   if (type == IEEE80211_RATE_TX) {
        return (OL_ATH_NODE_NET80211(ni))->an_ni_tx_rate;

    } else if (type == IEEE80211_RATE_RX) {
        return (OL_ATH_NODE_NET80211(ni))->an_ni_rx_rate;
    } else {
        return 0;
    }
   return 0;
}

static void
ol_ath_node_psupdate(struct ieee80211_node *ni, int pwrsave, int pause_resume)
{
    /* The peer power save state is maintained by the target. This
     * function exist only as a stub for completeness?
     */
}

static u_int32_t
ol_ath_node_get_maxphyrate(struct ieee80211com *ic, struct ieee80211_node *ni)
{
    u_int8_t mcs;
    u_int8_t bw;
    struct ieee80211vap *vap = ni->ni_vap;
    u_int8_t curr_phy_mode = wlan_get_current_phymode(vap);
    enum ieee80211_fixed_rate_mode rate_mode = vap->iv_fixed_rate.mode;
    u_int8_t nss = 0;
    u_int8_t sgi = 0;

    bw = wlan_get_param(vap, IEEE80211_CHWIDTH);

    if (ieee80211_vap_get_opmode(vap) == IEEE80211_M_STA) {
        if (((ni->ni_htcap & IEEE80211_HTCAP_C_SHORTGI40) && (bw == IEEE80211_CWM_WIDTH40)) ||
            ((ni->ni_htcap & IEEE80211_HTCAP_C_SHORTGI20) && (bw == IEEE80211_CWM_WIDTH20)) ||
            ((ni->ni_vhtcap & IEEE80211_VHTCAP_SHORTGI_80) && (bw == IEEE80211_CWM_WIDTH80))) {
           sgi = 1;
        }
    } else {
           sgi = wlan_get_param(vap, IEEE80211_SHORT_GI);
    }

    if (rate_mode != IEEE80211_FIXED_RATE_NONE) {
        /* Get rates for fixed rate */
        u_int32_t nbps = 0; /*Number of bits per symbol*/
        u_int32_t rc; /* rate code*/
        u_int32_t i;

	/* For fixed rate ensure that SGI is enabled by user */
	sgi = vap->iv_data_sgi;

        switch (rate_mode)
        {
        case IEEE80211_FIXED_RATE_MCS:
            nss = HT_RC_2_STREAMS(vap->iv_fixed_rateset);
            rc = wlan_get_param(vap, IEEE80211_FIXED_RATE);
            mcs = (rc & 0x07);
            for (i = 0; i < NUM_VHT_HT_RATES; i++) {
                if (vht_ht_tbl[i][BW_COL] == bw &&
                    vht_ht_tbl[i][MCS_COL] == mcs) {
                    nbps = vht_ht_tbl[i][NBPS_COL];
                }
            }
            break;
        case IEEE80211_FIXED_RATE_VHT:
            nss = vap->iv_nss;
            mcs = wlan_get_param(vap, IEEE80211_FIXED_VHT_MCS);
            for (i = 0; i < NUM_VHT_HT_RATES; i++) {
                if (vht_ht_tbl[i][BW_COL] == bw &&
                    vht_ht_tbl[i][MCS_COL] == mcs) {
                    nbps = vht_ht_tbl[i][NBPS_COL];
                }
            }
            break;
        case IEEE80211_FIXED_RATE_LEGACY:
            rc = wlan_get_param(vap, IEEE80211_FIXED_RATE);
            for (i = 0; i < NUM_LEGACY_RATES; i++) {
                if (legacy_rate_idx[i][L_RC_COL] == (rc & 0xff)) {
                    return legacy_rate_idx[i][L_BPS_COL] * 1000;
                }
            }
            break;
        default:
            break;
        }

        if (sgi) {
            return (nbps * 5 * 1000 * nss / 18) ;
        } else {
            return (nbps * 1000 * nss / 4) ;
        }
    } else {
        /* Get rates for auto rate */
        nss = ni->ni_streams;
        if(ieee80211_vap_get_opmode(vap) == IEEE80211_M_HOSTAP) {
            nss = (vap->iv_nss >
                   ieee80211_getstreams(ic, ic->ic_tx_chainmask)) ?
                   ieee80211_getstreams(ic, ic->ic_tx_chainmask) :
                   vap->iv_nss;
        }
        if ((nss < 1) || (nss > 3)) {
            printk("%s : NSS greater than 3 or less than 1!(nss:%d)\n", __func__, nss);
            return 0;
        }

    }

    if (ni != vap->iv_bss) {
        curr_phy_mode = ni->ni_phymode;
        nss = ni->ni_streams;

        if (ni->ni_htcap) {
            sgi = ((ni->ni_htcap & IEEE80211_HTCAP_C_SHORTGI20) || (ni->ni_htcap & IEEE80211_HTCAP_C_SHORTGI40));
        }
        if (ni->ni_vhtcap) {
            sgi = ((ni->ni_vhtcap & IEEE80211_VHTCAP_SHORTGI_80) || (ni->ni_vhtcap & IEEE80211_VHTCAP_SHORTGI_160));
        }
    }

    if(ieee80211_vap_256qam_is_set(ni->ni_vap) &&
        (((ieee80211_vap_get_opmode(vap) == IEEE80211_M_HOSTAP) && (ic->ic_vhtcap)) ||
        ((ieee80211_vap_get_opmode(vap) == IEEE80211_M_STA) && (ni->ni_vhtcap)))) {
       switch(curr_phy_mode) {
          case IEEE80211_MODE_11NG_HT20:
             if(((ieee80211_vap_get_opmode(vap) == IEEE80211_M_HOSTAP) &&
                                 !ieee80211_vap_ldpc_is_set(ni->ni_vap)) ||
                      ((ieee80211_vap_get_opmode(vap) == IEEE80211_M_STA) &&
                        !((ni->ni_htcap & IEEE80211_HTCAP_C_ADVCODING) &&
                          (ni->ni_vhtcap & IEEE80211_VHTCAP_RX_LDPC)))) {
                   /*256QAM 2G BCC rateset */
                 curr_phy_mode = IEEE80211_MODE_11AC_VHT80 + 1;
             } else {
                   /*256 QAM 2G LDPC rateset */
                 curr_phy_mode = IEEE80211_MODE_11AC_VHT80 + 2;
             }
          break;
          case IEEE80211_MODE_11NG_HT40PLUS:
          case IEEE80211_MODE_11NG_HT40MINUS:
          case IEEE80211_MODE_11NG_HT40:
                /*256 QAM 2G */
             curr_phy_mode = IEEE80211_MODE_11AC_VHT80 + 3;
          break;
          default:
          break;
       }
    }

    if (sgi) {
        return (max_rates[curr_phy_mode][(nss * 2) - 1] * 10);
    } else {
        return (max_rates[curr_phy_mode][(nss - 1) * 2] * 10);
    }
}

static int
ol_ath_node_add_wds_entry(struct ieee80211com *ic, const u_int8_t *dest_mac,
                          u_int8_t *peer_mac, u_int32_t flags)
{
    struct ol_ath_softc_net80211 *scn = OL_ATH_SOFTC_NET80211(ic);
    u_int8_t wmi_wds_flags = 0;

    if (flags & IEEE80211_NODE_F_WDS_HM) {
        wmi_wds_flags |= WMI_WDS_FLAG_STATIC;
    } else {
        /* Currently this interface is used only for host managed WDS entries */
        return -1;
    }

    if (wmi_unified_node_add_wds_entry(scn->wmi_handle, dest_mac, peer_mac,
            wmi_wds_flags)) {
        printk("%s:Unable to add wds entry\n", __func__);
        return -1;
    }
    return 0;
}

static void
ol_ath_node_del_wds_entry(struct ieee80211com *ic, u_int8_t *dest_mac)
{
    struct ol_ath_softc_net80211 *scn = OL_ATH_SOFTC_NET80211(ic);

    if (wmi_unified_node_del_wds_entry(scn->wmi_handle, dest_mac)){
        printk("%s:Unable to delete wds entry\n", __func__);
    }
}

#if ATH_SUPPORT_HYFI_ENHANCEMENTS
static int
ol_ath_node_update_wds_entry(struct ieee80211com *ic, u_int8_t *wds_macaddr, u_int8_t *peer_macaddr, u_int32_t flags)
{
    struct ol_ath_softc_net80211 *scn = OL_ATH_SOFTC_NET80211(ic);
    wmi_peer_update_wds_entry_cmd* cmd;
    wmi_buf_t buf;
    int len = sizeof(wmi_peer_update_wds_entry_cmd);

    buf = wmi_buf_alloc(scn->wmi_handle, len);
    if (!buf) {
        printk("%s: wmi_buf_alloc failed\n", __func__);
        return -ENOMEM;
    }

    /* wmi_buf_alloc returns zeroed command buffer */
    cmd = (wmi_peer_update_wds_entry_cmd *)wmi_buf_data(buf);
    cmd->flags |= (flags & IEEE80211_NODE_F_WDS_HM) ? WMI_WDS_FLAG_STATIC : 0;
    if (wds_macaddr) {
        WMI_CHAR_ARRAY_TO_MAC_ADDR(wds_macaddr, &cmd->wds_macaddr);
    }
    if (peer_macaddr) {
        WMI_CHAR_ARRAY_TO_MAC_ADDR(peer_macaddr, &cmd->peer_macaddr);
    }
    return wmi_unified_cmd_send(scn->wmi_handle, buf, len, WMI_PEER_UPDATE_WDS_ENTRY_CMDID);
}

static int
ol_ath_node_ext_stats_enable(struct ieee80211_node *ni, u_int32_t enable)
{
    struct ieee80211com *ic = ni->ni_ic;
    struct ol_ath_softc_net80211 *scn = OL_ATH_SOFTC_NET80211(ic);
    struct ol_ath_vap_net80211 *avn = OL_ATH_VAP_NET80211(ni->ni_vap);
    int retval = 0;

    if (enable && scn->peer_ext_stats_count >=
        scn->wlan_resource_config.max_peer_ext_stats) {
        return -ENOMEM;
    }

    if ((retval = wmi_unified_node_set_param(scn->wmi_handle, ni->ni_macaddr,
                WMI_PEER_EXT_STATS_ENABLE, enable, avn->av_if_id)) == 0) {
        if (enable) {
            scn->peer_ext_stats_count++;
        } else {
            scn->peer_ext_stats_count--;
        }
    }

    return retval;
}
#if ATH_SUPPORT_HYFI_ENHANCEMENTS
static void ol_ath_node_collect_stats(struct ieee80211_node *ni)
{
	u_int32_t old_phyerr, new_phyerr;
	u_int32_t old_ostime, new_ostime;
	u_int32_t phyerr_rate;
	u_int32_t msdu_size, max_msdu_size;
    old_phyerr = ni->ni_vap->iv_ald->ald_phyerr;
    old_ostime = ni->ni_vap->iv_ald->ald_ostime;
    new_phyerr = (OL_ATH_NODE_NET80211(ni))->an_phy_err_cnt;
    new_ostime = CONVERT_SYSTEM_TIME_TO_SEC(OS_GET_TIMESTAMP());

    if((new_ostime > old_ostime) && (old_ostime > 0))
	    phyerr_rate = (new_phyerr - old_phyerr)/(new_ostime - old_ostime);
    else
        phyerr_rate = 0;
    ni->ni_vap->iv_ald->ald_phyerr = new_phyerr;
    ni->ni_vap->iv_ald->ald_ostime = new_ostime;

	ni->ni_ald.ald_capacity = (OL_ATH_NODE_NET80211(ni))->an_tx_cnt ?
					((OL_ATH_NODE_NET80211(ni))->an_tx_rates_used/ (OL_ATH_NODE_NET80211(ni))->an_tx_cnt) : 0;

	if(ni->ni_ald.ald_capacity == 0)
		ni->ni_ald.ald_capacity = (OL_ATH_NODE_NET80211(ni))->an_ni_tx_rate/1000;


	if((OL_ATH_NODE_NET80211(ni))->an_tx_bytes == 0)
		msdu_size = ALD_MSDU_SIZE;
	else
		msdu_size = (OL_ATH_NODE_NET80211(ni))->an_tx_bytes/ni->ni_ald.ald_txcount;

	if (msdu_size < DEFAULT_MSDU_SIZE)
		ni->ni_ald.ald_msdusize = ALD_MSDU_SIZE;
	else
		ni->ni_ald.ald_msdusize = msdu_size;
	max_msdu_size = (msdu_size > ALD_MSDU_SIZE) ? msdu_size : ALD_MSDU_SIZE;
	if ((OL_ATH_NODE_NET80211(ni))->an_tx_ratecount > 0)
	    ni->ni_ald.ald_avgmax4msaggr = ni->ni_ald.ald_max4msframelen / ((OL_ATH_NODE_NET80211(ni))->an_tx_ratecount * max_msdu_size);

    if(ni->ni_ald.ald_avgmax4msaggr > 192)
	    ni->ni_ald.ald_avgmax4msaggr = 192;

	if(ni->ni_ald.ald_avgmax4msaggr > 0)
		ni->ni_ald.ald_aggr = ni->ni_ald.ald_avgmax4msaggr/2;
	else
		ni->ni_ald.ald_aggr = 96; //Max aggr 192/2

	/* Avg Aggr should be atleast 1 */
	ni->ni_ald.ald_aggr = ni->ni_ald.ald_aggr > 1 ? ni->ni_ald.ald_aggr : 1;

}
#endif
static void ol_ath_node_reset_ald_stats(struct ieee80211_node *ni)
{
	/* Do not reset ald_avgmax4msaggr and ald_aggr.
	 Past values are used when there is no traffic */
    (OL_ATH_NODE_NET80211(ni))->an_tx_cnt = 0;
    (OL_ATH_NODE_NET80211(ni))->an_tx_rates_used = 0;
    (OL_ATH_NODE_NET80211(ni))->an_tx_bytes = 0;
    (OL_ATH_NODE_NET80211(ni))->an_tx_ratecount = 0;
	ni->ni_ald.ald_txcount = 0;
	ni->ni_ald.ald_max4msframelen = 0;
	ni->ni_ald.ald_phyerr = 0;
	ni->ni_ald.ald_msdusize = 0;
	ni->ni_ald.ald_retries = 0;
	ni->ni_ald.ald_capacity = 0;
	ni->ni_ald.ald_lastper = 0;
	OS_MEMZERO(&ni->ni_ald.ald_ac_excretries, sizeof(ni->ni_ald.ald_ac_excretries));
	OS_MEMZERO(&ni->ni_ald.ald_ac_nobufs, sizeof(ni->ni_ald.ald_ac_nobufs));
	OS_MEMZERO(&ni->ni_ald.ald_ac_txpktcnt, sizeof(ni->ni_ald.ald_ac_txpktcnt));

}
#endif /* #if ATH_SUPPORT_HYFI_ENHANCEMENTS */

static void
ol_ath_node_authorize(struct ieee80211_node *ni, u_int32_t authorize)
{
    struct ieee80211com *ic = ni->ni_ic;
    struct ol_ath_softc_net80211 *scn = OL_ATH_SOFTC_NET80211(ic);
    struct ol_ath_vap_net80211 *avn = OL_ATH_VAP_NET80211(ni->ni_vap);

    /* Authorize/unauthorize the peer */
    ol_txrx_peer_authorize((OL_ATH_NODE_NET80211(ni))->an_txrx_handle, authorize);

   /*FIXME Currently UMAC authorizes PAE on assoc and supplicant driver
    * interface fails to unauthorize before 4-way handshake and authorize
    * on completing 4-way handshake. WAR is to suppress authorizations
    * for all AUTH modes that need 4-way handshake and authorize on install
    * key cmd.
    * Need to check for WAPI case
    * safemode bypass the normal process
    */
    if(authorize && (RSN_AUTH_IS_WPA(&ni->ni_rsn) ||
            RSN_AUTH_IS_WPA2(&ni->ni_rsn) ||
            RSN_AUTH_IS_8021X(&ni->ni_rsn)||
            RSN_AUTH_IS_WAI(&ni->ni_rsn)) && !IEEE80211_VAP_IS_SAFEMODE_ENABLED(ni->ni_vap)) {
       return;
    }

    if(wmi_unified_node_set_param(scn->wmi_handle,ni->ni_macaddr,WMI_PEER_AUTHORIZE,
            authorize,avn->av_if_id)) {
        printk("%s:Unable to authorize peer\n", __func__);
    }
}

void
ol_ath_node_update(struct ieee80211_node *ni)
{
}

static void
ol_ath_node_smps_update(
        struct ieee80211_node *ni,
        int smen,
        int dyn,
        int ratechg)
{
    struct ieee80211com *ic = ni->ni_ic;
    struct ol_ath_softc_net80211 *scn = OL_ATH_SOFTC_NET80211(ic);
    struct ol_ath_vap_net80211 *avn = OL_ATH_VAP_NET80211(ni->ni_vap);
    A_UINT32 value;

    if (smen) {
        value = WMI_PEER_MIMO_PS_NONE;
    } else if (dyn) {
        value = WMI_PEER_MIMO_PS_DYNAMIC;
    } else {
        value = WMI_PEER_MIMO_PS_STATIC;
    }

    (void)wmi_unified_node_set_param(scn->wmi_handle, ni->ni_macaddr,
            WMI_PEER_MIMO_PS_STATE, value, avn->av_if_id);
}


#if UMAC_SUPPORT_ADMCTL
static void
ol_ath_node_update_dyn_uapsd(struct ieee80211_node *ni, uint8_t ac, int8_t ac_delivery, int8_t ac_trigger)
{
	uint8_t i;
	uint8_t uapsd=0;
	struct ieee80211vap *vap = ni->ni_vap;

	if (ac_delivery <= WME_UAPSD_AC_MAX_VAL) {
		ni->ni_uapsd_dyn_delivena[ac] = ac_delivery;
	}

	if (ac_trigger <= WME_UAPSD_AC_MAX_VAL) {
		ni->ni_uapsd_dyn_trigena[ac] = ac_trigger;
	}

	for (i=0;i<WME_NUM_AC;i++) {
		if (ni->ni_uapsd_dyn_trigena[i] == -1) {
			if (ni->ni_uapsd_ac_trigena[i]) {
				uapsd |= WMI_UAPSD_AC_BIT_MASK(i,WMI_UAPSD_AC_TYPE_TRIG);
			}
		} else {
			if (ni->ni_uapsd_dyn_trigena[i]) {
				uapsd |= WMI_UAPSD_AC_BIT_MASK(i,WMI_UAPSD_AC_TYPE_TRIG);
			}
		}
	}

	for (i=0;i<WME_NUM_AC;i++) {
		if (ni->ni_uapsd_dyn_delivena[i] == -1) {
			if (ni->ni_uapsd_ac_delivena[i]) {
				uapsd |= WMI_UAPSD_AC_BIT_MASK(i,WMI_UAPSD_AC_TYPE_DELI);
			}
		} else {
			if (ni->ni_uapsd_dyn_delivena[i]) {
				uapsd |= WMI_UAPSD_AC_BIT_MASK(i,WMI_UAPSD_AC_TYPE_DELI);
			}
		}
	}

	(void)wmi_unified_set_ap_ps_param(OL_ATH_VAP_NET80211(vap),
             OL_ATH_NODE_NET80211(ni), WMI_AP_PS_PEER_PARAM_UAPSD, uapsd);
	return;
}
#endif /* UMAC_SUPPORT_ADMCTL */

static int
wmi_peer_sta_ps_state_change_handler(ol_scn_t scn, u_int8_t *data, u_int16_t datalen, void *context)
{
    struct ieee80211com *ic = &scn->sc_ic;
    A_UINT8 peer_macaddr[ATH_MAC_LEN];
    struct ieee80211_node *ni;
    wmi_peer_sta_ps_statechange_event *event = (wmi_peer_sta_ps_statechange_event *)data;
    WMI_MAC_ADDR_TO_CHAR_ARRAY(&event->peer_macaddr,peer_macaddr);
    ni = ieee80211_find_node(&ic->ic_sta, peer_macaddr);
    if (!ni) {
        return -1;
    }
    ni->ps_state = event->peer_ps_state;
    ieee80211_free_node(ni);
    return 0;
}
#ifdef ATH_SUPPORT_QUICK_KICKOUT
static int
wmi_peer_sta_kickout_event_handler(ol_scn_t scn, u_int8_t *data, u_int16_t datalen, void *context)
{
    struct ieee80211com *ic = &scn->sc_ic;
    A_UINT8 peer_macaddr[ATH_MAC_LEN];
    struct ieee80211_node *ni;
    wmi_peer_sta_kickout_event *kickout_event = (wmi_peer_sta_kickout_event *)data;
    WMI_MAC_ADDR_TO_CHAR_ARRAY(&kickout_event->peer_macaddr,peer_macaddr);
    ni = ieee80211_find_node(&ic->ic_sta, peer_macaddr);
    if (!ni) {
        return -1;
    }
    ieee80211_kick_node(ni);
    ieee80211_free_node(ni);
    return 0;
}
#endif /* ATH_SUPPORT_QUICK_KICKOUT */

#if ATH_BAND_STEERING
static bool
ol_ath_node_is_inact(struct ieee80211_node *ni)
{
    return ol_txrx_peer_is_inact(OL_ATH_NODE_NET80211(ni)->an_txrx_handle);
}
#endif /* ATH_BAND_STEERING */

/* Intialization functions */
void
ol_ath_node_attach(struct ol_ath_softc_net80211 *scn, struct ieee80211com *ic)
{
    /* Register the umac callback functions */
    scn->net80211_node_free = ic->ic_node_free;
    scn->net80211_node_cleanup = ic->ic_node_cleanup;

    /* Register the node specific offload interface functions */
    ic->ic_node_alloc = ol_ath_node_alloc;
    ic->ic_node_free = ol_ath_node_free;
    ic->ic_node_cleanup = ol_ath_node_cleanup;
    ic->ic_node_getrssi = ol_ath_node_getrssi;
    ic->ic_node_getrate = ol_ath_node_getrate;
    ic->ic_node_psupdate = ol_ath_node_psupdate;
    ic->ic_get_maxphyrate = ol_ath_node_get_maxphyrate;
    ic->ic_node_add_wds_entry = ol_ath_node_add_wds_entry;
    ic->ic_node_del_wds_entry = ol_ath_node_del_wds_entry;
#if ATH_SUPPORT_HYFI_ENHANCEMENTS
    ic->ic_node_update_wds_entry = ol_ath_node_update_wds_entry;
    ic->ic_node_ext_stats_enable = ol_ath_node_ext_stats_enable;
    ic->ic_reset_ald_stats = ol_ath_node_reset_ald_stats;
	ic->ic_collect_stats = ol_ath_node_collect_stats;
#endif
    ic->ic_node_authorize = ol_ath_node_authorize;
    ic->ic_sm_pwrsave_update = ol_ath_node_smps_update;
#if UMAC_SUPPORT_ADMCTL
    ic->ic_node_update_dyn_uapsd = ol_ath_node_update_dyn_uapsd;
#endif
#if ATH_BAND_STEERING
    ic->ic_node_isinact = ol_ath_node_is_inact;
#endif
#ifdef ATH_SUPPORT_QUICK_KICKOUT
    /* register for STA kickout function */
    wmi_unified_register_event_handler(scn->wmi_handle, WMI_PEER_STA_KICKOUT_EVENTID, wmi_peer_sta_kickout_event_handler, NULL);
    wmi_unified_register_event_handler(scn->wmi_handle, WMI_PEER_STA_PS_STATECHG_EVENTID, wmi_peer_sta_ps_state_change_handler, NULL);
#endif
}

void
ol_rx_err(
    ol_pdev_handle pdev,
    u_int8_t vdev_id,
    u_int8_t *peer_mac_addr,
    int tid,
    u_int32_t tsf32,
    enum ol_rx_err_type err_type,
    adf_nbuf_t rx_frame)
{
    struct ieee80211_frame wh;
    struct ether_header *eh;
    struct ol_ath_softc_net80211 *scn ;
    struct ieee80211vap *vap;
    enum ieee80211_opmode opmode;
    A_BOOL notify = TRUE;

    eh = (struct ether_header *)adf_nbuf_data(rx_frame);
    scn = (struct ol_ath_softc_net80211 *)pdev;
    vap = ol_ath_vap_get(scn, vdev_id);
    if(vap == NULL) {
       printk("%s: vap is NULL \n", __func__);
       return;
    }
    opmode = ieee80211_vap_get_opmode(vap);

    if (err_type == OL_RX_ERR_TKIP_MIC) {
        /*TODO: Reconstructing the WLAN header for now from ether header
         * since WLAN header is not available for HL case.
         */
        wh.i_fc[0] = IEEE80211_FC0_VERSION_0 | IEEE80211_FC0_TYPE_DATA | IEEE80211_FC0_SUBTYPE_DATA;
        wh.i_dur[0] = wh.i_dur[1] = 0;
        wh.i_seq[0] = wh.i_seq[1] = 0;

        adf_os_mem_copy(&wh.i_addr1, &vap->iv_myaddr, IEEE80211_ADDR_LEN);
        adf_os_mem_copy(&wh.i_addr2, peer_mac_addr, IEEE80211_ADDR_LEN);

        if (opmode == IEEE80211_M_HOSTAP || opmode == IEEE80211_M_WDS) {
            wh.i_fc[1] = IEEE80211_FC1_DIR_TODS;
            adf_os_mem_copy(&wh.i_addr3, &eh->ether_dhost , IEEE80211_ADDR_LEN);
        } else if (opmode == IEEE80211_M_STA) {
            wh.i_fc[1] = IEEE80211_FC1_DIR_FROMDS;
            adf_os_mem_copy(&wh.i_addr3, &eh->ether_shost , IEEE80211_ADDR_LEN);
        } else {
            /*TODO: Handle other cases*/
            notify = FALSE;
        }

        if (notify) {
            printk("%s: TKIP MIC failure \n",__func__);
            ieee80211_notify_michael_failure(vap,(const struct ieee80211_frame *)&wh,0);
        }
    }
}

int
ol_rx_notify(
    ol_pdev_handle pdev,
    u_int8_t vdev_id,
    u_int8_t *peer_mac_addr,
    int tid,
    u_int32_t tsf32,
    enum ol_rx_notify_type notify_type,
    adf_nbuf_t rx_frame)
{
    struct ieee80211vap *vap = NULL;
    struct ieee80211_node *ni;
    struct ol_ath_softc_net80211 *scn = (struct ol_ath_softc_net80211 *)pdev;
	struct ieee80211com *ic;
    int discard = 0;
#if ATH_SUPPORT_HYFI_ENHANCEMENTS
    int igmp_type = 0;
#endif
    vap = ol_ath_vap_get(scn, vdev_id);
    if(!vap) {
        return discard;
    }
    ic = vap->iv_ic;
    ni = ieee80211_vap_find_node(vap, peer_mac_addr);
    if(!ni) {
        return discard;
    }
#if ATH_SUPPORT_HYFI_ENHANCEMENTS
    if ( vap->iv_ique_ops.me_inspect && vap->iv_me->me_hifi_enable == 0) {
        igmp_type = vap->iv_ique_ops.me_inspect(vap, ni, rx_frame);
    }

    if(igmp_type == IEEE80211_QUERY_FROM_STA && ic->ic_dropstaquery)
        discard = 1;
    if(igmp_type == IEEE80211_REPORT_FROM_STA && ic->ic_blkreportflood)
        discard = 1;
#else
    if ( vap->iv_ique_ops.me_inspect ) {
	       vap->iv_ique_ops.me_inspect(vap, ni, rx_frame);
    }
#endif
    /* remove extra node ref count added by find_node above */
    ieee80211_free_node(ni);
    return discard;
}

#endif
