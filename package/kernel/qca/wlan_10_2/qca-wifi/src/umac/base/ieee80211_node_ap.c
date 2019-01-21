/*
 *  Copyright (c) 2008 Atheros Communications Inc. 
 * All Rights Reserved.
 * 
 * Copyright (c) 2011 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 * 
 */

#include "ieee80211_node_priv.h"

#if UMAC_SUPPORT_AP || UMAC_SUPPORT_BTAMP

#if ATH_TxBF_DYNAMIC_LOF_ON_N_CHAIN_MASK
static INLINE void iee80211_txbf_loforce_check(struct ieee80211_node *ni, bool nodejoin)
{
#ifdef ATH_SUPPORT_TxBF
    struct ieee80211com *ic = ni->ni_ic;
    if ( ni->ni_explicit_compbf || ni->ni_explicit_noncompbf || ni->ni_implicit_bf){
        if(!nodejoin)
            ic->ic_ht_txbf_sta_assoc--;

        if(ic->ic_ht_txbf_sta_assoc == 0){
            if(ic->ic_txbf_loforceon_update)
                ic->ic_txbf_loforceon_update(ic,nodejoin);
        }
        if(nodejoin)
            ic->ic_ht_txbf_sta_assoc++;
    }
#endif
}
#endif
/*
 * Create a HOSTAP node  on current channel based on ssid.
 */
int
ieee80211_create_infra_bss(struct ieee80211vap *vap,
                      const u_int8_t *essid,
                      const u_int16_t esslen)
{
    struct ieee80211com *ic = vap->iv_ic;
    struct ieee80211_node *ni;

    ni = ieee80211_alloc_node(&ic->ic_sta, vap, vap->iv_myaddr);
    if (ni == NULL)
        return -ENOMEM;

    IEEE80211_ADDR_COPY(ni->ni_bssid, vap->iv_myaddr);
    ni->ni_esslen = esslen;
    OS_MEMCPY(ni->ni_essid, essid, ni->ni_esslen);
    ni->ni_intval = ic->ic_intval;
    if (IEEE80211_VAP_IS_PRIVACY_ENABLED(vap)) {
        ni->ni_capinfo |= IEEE80211_CAPINFO_PRIVACY;
        ni->ni_rsn = vap->iv_rsn; /* use local RSN settings as the BSS setting */
    }
    /* Set the node htcap to be same as ic htcap */
    ni->ni_htcap = ic->ic_htcap;
    IEEE80211_ADD_NODE_TARGET(ni, ni->ni_vap, 1);

    /* copy the original bssinfo into the new ni BugID: Ev# 73905 */
    ieee80211_copy_bss(ni, vap->iv_bss);

    return ieee80211_sta_join_bss(ni);
}

/*
 * Handle a station leaving an 11g network.
 */
static void
ieee80211_node_leave_11g(struct ieee80211_node *ni)
{
    struct ieee80211com *ic = ni->ni_ic;
    struct ieee80211vap *vap = ni->ni_vap;
   
    KASSERT((IEEE80211_IS_CHAN_ANYG(vap->iv_bsschan)
             || IEEE80211_IS_CHAN_11NG(vap->iv_bsschan)),
            ("not in 11g, bss %u:0x%x, curmode %u", vap->iv_bsschan->ic_freq,
             vap->iv_bsschan->ic_flags, ic->ic_curmode));

    KASSERT((ni->ni_vap->iv_opmode == IEEE80211_M_HOSTAP ||
             ni->ni_vap->iv_opmode == IEEE80211_M_BTAMP ||
             ni->ni_vap->iv_opmode == IEEE80211_M_IBSS), (" node leave in invalid opmode "));

    /*
     * If a long slot station do the slot time bookkeeping.
     */
    if ((ni->ni_capinfo & IEEE80211_CAPINFO_SHORT_SLOTTIME) == 0) {
        if (ic->ic_longslotsta) {
           ic->ic_longslotsta--; 
           IEEE80211_NOTE(vap, IEEE80211_MSG_ASSOC, ni,
                       "long slot time station leaves, count now %d\n",
                       ic->ic_longslotsta);
        } else {
#ifdef NOT_YET
           KASSERT(ic->ic_longslotsta > 0,
                ("bogus long slot station count %d", ic->ic_longslotsta));
#endif
           IEEE80211_NOTE(vap, IEEE80211_MSG_ASSOC, ni,
                "bogus long slot station count %d", ic->ic_longslotsta);
        }

        if (ic->ic_longslotsta == 0) {
            /*
             * Re-enable use of short slot time if supported
             * and not operating in IBSS mode (per spec).
             */
            if (ic->ic_caps & IEEE80211_C_SHSLOT) {
                IEEE80211_DPRINTF(vap, IEEE80211_MSG_ASSOC,
                                  "%s: re-enable use of short slot time\n",
                                  __func__);
                ieee80211_set_shortslottime(ic, 1);
            }
        }
    }
    
    /*
     * If a non-ERP station do the protection-related bookkeeping.
     */
    if (((ni->ni_flags & IEEE80211_NODE_ERP) == 0) && (ic->ic_nonerpsta > 0)) {
        KASSERT(ic->ic_nonerpsta > 0, ("bogus non-ERP station count %d", ic->ic_nonerpsta));
        ic->ic_nonerpsta--;
        IEEE80211_NOTE(vap, IEEE80211_MSG_ASSOC, ni,
                       "non-ERP station leaves, count now %d", ic->ic_nonerpsta);
        if (ic->ic_nonerpsta == 0) {
	        struct ieee80211vap *tmpvap;
            ieee80211_update_erp_info(vap); 
            /* XXX verify mode? */
            if (ic->ic_caps & IEEE80211_C_SHPREAMBLE) {
                IEEE80211_DPRINTF(vap, IEEE80211_MSG_ASSOC,
                                  "%s: re-enable use of short preamble\n",
                                  __func__);
                ic->ic_flags |= IEEE80211_F_SHPREAMBLE;
                ic->ic_flags &= ~IEEE80211_F_USEBARKER;
            }
 	        TAILQ_FOREACH(tmpvap, &(ic)->ic_vaps, iv_next)
    		    ieee80211_vap_erpupdate_set(tmpvap);
        }
    }
}

extern void AP_SendDeleteData(const u_int8_t ac_mac[] , u_int16_t ic_sta)  ;

#ifdef IEEE80211_DEBUG_REFCNT
bool ieee80211_node_leave_debug(struct ieee80211_node *ni, const char *func, int line)
#else
bool _ieee80211_node_leave(struct ieee80211_node *ni)
#endif
{
    struct ieee80211com *ic = ni->ni_ic;
    struct ieee80211vap *vap = ni->ni_vap;
	bool   retval = false;
#ifdef IEEE80211_DEBUG_REFCNT
    TRACENODE(ni, func, line);
#endif
    IEEE80211_NOTE(vap, IEEE80211_MSG_ASSOC, ni,
                   "station with aid %d leaves (refcnt %u) \n",
                   IEEE80211_NODE_AID(ni), ieee80211_node_refcnt(ni));
    IEEE80211_NOTE(vap, IEEE80211_MSG_DEBUG, ni,
                   "station with aid %d leaves (refcnt %u) \n",
                   IEEE80211_NODE_AID(ni), ieee80211_node_refcnt(ni));
#ifdef ATH_SWRETRY
    if (ic->ic_reset_pause_tid)
        ic->ic_reset_pause_tid(ni->ni_ic, ni);
#endif

    if (!IEEE80211_IS_TDLS_NODE(ni)) 
    KASSERT(vap->iv_opmode == IEEE80211_M_HOSTAP
            || vap->iv_opmode == IEEE80211_M_WDS ||
            vap->iv_opmode == IEEE80211_M_BTAMP  ||
            vap->iv_opmode == IEEE80211_M_IBSS,
            ("unexpected operating mode %u", vap->iv_opmode));

    /* Multicast enhancement: If the entry with the node's address exists in 
     * the snoop table, it should be removed.
     */
    if (vap->iv_ique_ops.me_clean) {
        vap->iv_ique_ops.me_clean(ni);
    }
	/*    
     * HBR / headline block removal: delete the node entity from the table
     * for HBR purpose
     */
    if (vap->iv_ique_ops.hbr_nodeleave) {
        vap->iv_ique_ops.hbr_nodeleave(vap, ni);
    }
    /*
     * If node wasn't previously associated all
     * we need to do is reclaim the reference.
     */
    /* XXX ibss mode bypasses 11g and notification */

    IEEE80211_NODE_STATE_LOCK_BH(ni);

    /*
     * Prevent _ieee80211_node_leave() from reentry which would mess up the 
     * value of iv_sta_assoc. Before AP received the tx ack for "disassoc 
     * request", it may have received the "auth (not SUCCESS status)" to do 
     * node leave. With the flag, follow-up cleanup wouldn't call 
     * _ieee80211_node_leave() again when execuating the tx_complete handler.
     */
    ni->ni_flags |= IEEE80211_NODE_LEAVE_ONGOING;

    if (ni->ni_associd) {
        IEEE80211_VAP_LOCK(vap);
        vap->iv_sta_assoc--;
        IEEE80211_VAP_UNLOCK(vap);		
        IEEE80211_COMM_LOCK(ic);
        ic->ic_sta_assoc--;
        /* Update bss load element in beacon */
        ieee80211_vap_bssload_update_set(vap);

        if (IEEE80211_NODE_USE_HT(ni)) {
            ic->ic_ht_sta_assoc--;
            if (ni->ni_htcap & IEEE80211_HTCAP_C_GREENFIELD) {
                ASSERT(ic->ic_ht_gf_sta_assoc > 0);
                ic->ic_ht_gf_sta_assoc--;
            }
#if ATH_TxBF_DYNAMIC_LOF_ON_N_CHAIN_MASK
            iee80211_txbf_loforce_check(ni,0);
#endif
            if ((ni->ni_chwidth == IEEE80211_CWM_WIDTH40) || (ni->ni_chwidth == IEEE80211_CWM_WIDTH80))
	      ic->ic_ht40_sta_assoc--;
	  }
    

        if ((IEEE80211_IS_CHAN_ANYG(vap->iv_bsschan) ||
            IEEE80211_IS_CHAN_11NG(vap->iv_bsschan))  && !IEEE80211_IS_TDLS_NODE(ni))
            ieee80211_node_leave_11g(ni);

        ieee80211_admctl_node_leave(vap, ni);

        /*
         * Cleanup station state.  In particular clear various state that 
         * might otherwise be reused if the node is reused before the
         * reference count goes to zero (and memory is reclaimed).
         *
         * If ni is not in node table, it has been reclaimed in another thread.
         */
        retval = ieee80211_sta_leave(ni);

        IEEE80211_COMM_UNLOCK(ic);
        ni->ni_flags &= ~IEEE80211_NODE_LEAVE_ONGOING;
        IEEE80211_NODE_STATE_UNLOCK_BH(ni);
        IEEE80211_DELETE_NODE_TARGET(ni, ic, vap, 0);
    } else {
        ieee80211_admctl_node_leave(vap, ni);
        retval = ieee80211_sta_leave(ni);
        ni->ni_flags &= ~IEEE80211_NODE_LEAVE_ONGOING;
        IEEE80211_NODE_STATE_UNLOCK_BH(ni);
    }

    if ((ni->ni_flags & IEEE80211_NODE_HT) && 
        (ni->ni_flags & IEEE80211_NODE_40_INTOLERANT)) {
        ieee80211_change_cw(ic);
    }


/* begin: add by chenzejun for mac scan by 2016.1.22 */
    AP_SendDeleteData(ni->ni_macaddr , ni->ni_ic->ic_sta_assoc);
/* end: add by chenzejun for mac scan by 2016.1.22 */

    return retval;
}


/*
 * Handle a station joining an 11g network.
 */
static void
ieee80211_node_join_11g(struct ieee80211_node *ni)
{
    struct ieee80211com *ic = ni->ni_ic;
    struct ieee80211vap *vap = ni->ni_vap;

    KASSERT((IEEE80211_IS_CHAN_ANYG(vap->iv_bsschan) ||
             IEEE80211_IS_CHAN_11NG(vap->iv_bsschan)),
            ("not in 11g, bss %u:0x%x, curmode %u", vap->iv_bsschan->ic_freq,
             vap->iv_bsschan->ic_flags, ic->ic_curmode));

    KASSERT((ni->ni_vap->iv_opmode == IEEE80211_M_HOSTAP ||
             ni->ni_vap->iv_opmode == IEEE80211_M_BTAMP  ||
             ni->ni_vap->iv_opmode == IEEE80211_M_IBSS), (" node join in invalid opmode "));
    /*
     * Station isn't capable of short slot time.  Bump
     * the count of long slot time stations and disable
     * use of short slot time.  Note that the actual switch
     * over to long slot time use may not occur until the
     * next beacon transmission (per sec. 7.3.1.4 of 11g).
     */
    if ((ni->ni_capinfo & IEEE80211_CAPINFO_SHORT_SLOTTIME) == 0) {
        ic->ic_longslotsta++;
        IEEE80211_NOTE(ni->ni_vap, IEEE80211_MSG_ASSOC, ni,
                       "station needs long slot time, count %d",
                       ic->ic_longslotsta);
        /* XXX vap's w/ conflicting needs won't work */
        if (!IEEE80211_IS_CHAN_108G(vap->iv_bsschan)) {
            /*
             * Don't force slot time when switched to turbo
             * mode as non-ERP stations won't be present; this
             * need only be done when on the normal G channel.
             */
            ieee80211_set_shortslottime(ic, 0);
        }
    }
    /*
     * If the new station is not an ERP station
     * then bump the counter and enable protection
     * if configured.
     */
    if (!ieee80211_iserp_rateset(ic, &ni->ni_rates)) {
        ic->ic_nonerpsta++;
        IEEE80211_NOTE(ni->ni_vap, IEEE80211_MSG_ASSOC, ni,
                       "station is !ERP, %d non-ERP stations associated",
                       ic->ic_nonerpsta);
            /*
             * If protection is configured, enable it.
             */
            if (ic->ic_protmode != IEEE80211_PROT_NONE) {
                IEEE80211_DPRINTF(ni->ni_vap, IEEE80211_MSG_ASSOC,
                                  "%s: enable use of protection\n", __func__);
                ic->ic_flags |= IEEE80211_F_USEPROT;
                ic->ic_update_protmode(ic);
            }
            /*
             * If station does not support short preamble
             * then we must enable use of Barker preamble.
             */
            if ((ni->ni_capinfo & IEEE80211_CAPINFO_SHORT_PREAMBLE) == 0) {
                IEEE80211_NOTE(ni->ni_vap, IEEE80211_MSG_ASSOC, ni,
                               "%s", "station needs long preamble");
                ic->ic_flags |= IEEE80211_F_USEBARKER;
                ic->ic_flags &= ~IEEE80211_F_SHPREAMBLE;
            }

            /* Update ERP element if this is first non ERP station */
            if (ic->ic_nonerpsta == 1) {
	          struct ieee80211vap *tmpvap;
	          TAILQ_FOREACH(tmpvap, &(ic)->ic_vaps, iv_next)
		    ieee80211_vap_erpupdate_set(tmpvap);
	      }
    } else
        ni->ni_flags |= IEEE80211_NODE_ERP;
}

/*
 * function to handle station joining infrastructure network.
 * used for AP mode vap only.
 */  
int ieee80211_node_join(struct ieee80211_node *ni)
{
    struct ieee80211com *ic = ni->ni_ic;
    struct ieee80211vap *vap = ni->ni_vap;

    IEEE80211_NODE_STATE_LOCK(ni);
    if (ni->ni_associd == 0) {
        u_int16_t aid;

        if(vap->iv_sta_assoc >= vap->iv_max_aid - 1) {
            IEEE80211_NODE_STATE_UNLOCK(ni);
            return -1; /* soft client limit reached */
        }

        if (vap->iv_aid_bitmap == NULL || IEEE80211_IS_TDLS_NODE(ni)) {
            IEEE80211_NODE_STATE_UNLOCK(ni);
            return -1; /* vap is being deleted */
        }

        /*
         * It would be good to search the bitmap
         * more efficiently, but this will do for now.
         */
        if (!IEEE80211_IS_TDLS_NODE(ni)) {
            for (aid = 1; aid < vap->iv_max_aid; aid++) {
                if (!IEEE80211_AID_ISSET(vap, aid))
                break;
            }

            if (aid >= vap->iv_max_aid) {
            /*
             * Keep stats on this situation.
             */
            IEEE80211_DPRINTF(vap, IEEE80211_MSG_ASSOC, "aid (%d)"
                              " greater than max aid (%d)\n", aid,
                              vap->iv_max_aid);
            IEEE80211_NODE_STATE_UNLOCK(ni);
            IEEE80211_NODE_LEAVE(ni);
            return -1;
            }

            ni->ni_associd = aid | 0xc000;
            IEEE80211_AID_SET(vap, ni->ni_associd);
        } else {
            /* To keep the other parts of code happy. Otherwise the code assumes
             * node disconnected and tries to delete it.There are no associd for
            * TDLS nodes.
            */
            ni->ni_associd = vap->iv_bss->ni_associd;
        }
        IEEE80211_VAP_LOCK(vap);
        vap->iv_sta_assoc++;
        IEEE80211_VAP_UNLOCK(vap);
        /* Update bss load element in beacon */
        ieee80211_vap_bssload_update_set(vap);
        if ((ni->ni_flags & IEEE80211_NODE_HT) &&
            (ni->ni_flags & IEEE80211_NODE_40_INTOLERANT)) {
            ieee80211_change_cw(ic);
        }
        if (IEEE80211_IS_AMPDU_ENABLED(ic)) {
            ieee80211node_clear_flag(ni, IEEE80211_NODE_NOAMPDU);
        }
        else {
            ieee80211node_set_flag(ni, IEEE80211_NODE_NOAMPDU);
        }
        
        IEEE80211_COMM_LOCK(ic);
        ic->ic_sta_assoc++;
        if (IEEE80211_NODE_USE_HT(ni)) {
            ic->ic_ht_sta_assoc++;
            if (ni->ni_htcap & IEEE80211_HTCAP_C_GREENFIELD)
                ic->ic_ht_gf_sta_assoc++;
#if ATH_TxBF_DYNAMIC_LOF_ON_N_CHAIN_MASK
             iee80211_txbf_loforce_check(ni,1);
#endif
	    if ((ni->ni_chwidth == IEEE80211_CWM_WIDTH40 ) || (ni->ni_chwidth == IEEE80211_CWM_WIDTH80)) {
	      ic->ic_ht40_sta_assoc++;
      	    }
	  }
      

        if ((IEEE80211_IS_CHAN_ANYG(vap->iv_bsschan) ||
            IEEE80211_IS_CHAN_11NG(vap->iv_bsschan)) && !IEEE80211_IS_TDLS_NODE(ni))
            ieee80211_node_join_11g(ni);
        IEEE80211_COMM_UNLOCK(ic);
    }
#if ATH_SUPPORT_WIFIPOS
    // Time stamp to send in status response
    ni->last_rx_desc_tsf = ic->ic_get_TSF32(ic);
#endif
    ni->ni_inact_reload = ni->ni_vap->iv_inact_auth;
    ni->ni_inact = ni->ni_inact_reload;
    
    /* Multicast enhancement: If the entry with the node's address exists in 
     * the snoop table, it should be removed.
     */
    if (vap->iv_ique_ops.me_clean) {
        vap->iv_ique_ops.me_clean(ni);
    }
    /*
     * HBR / headline block removal: add the node entity to the table
     * for HBR purpose
     */
    if (vap->iv_ique_ops.hbr_nodejoin) {
        vap->iv_ique_ops.hbr_nodejoin(vap, ni);
    }
    IEEE80211_NODE_STATE_UNLOCK(ni);
    IEEE80211_ADD_NODE_TARGET(ni, ni->ni_vap, 0);
	return 0;
}

/*
 * Craft a temporary node suitable for sending a management frame
 * to the specified station.  We craft only as much state as we
 * need to do the work since the node will be immediately reclaimed
 * once the send completes, and the temporary node will NOT be put
 * into node table.
 */
struct ieee80211_node *
ieee80211_tmp_node(struct ieee80211vap *vap, const u_int8_t *macaddr)
{
    struct ieee80211com *ic = vap->iv_ic;
    struct ieee80211_node *ni;
    int i;        
    
    if(!(IEEE80211_ADDR_IS_VALID(macaddr))){
        printk("INVALID MAC ADDRESS \n");
        return NULL;
    }


    /*
     * if vap is being deleted, do not allow new allocations.
     */
    if (ieee80211_vap_deleted_is_set(vap)) {
        return NULL;
    }

    ni = ic->ic_node_alloc(vap, macaddr, TRUE /* temp node */);
    if (ni == NULL) {
        vap->iv_stats.is_rx_nodealloc++;
        return NULL;
    }

#ifdef IEEE80211_DEBUG_NODELEAK
    do {
        rwlock_state_t lock_state;
        OS_RWLOCK_WRITE_LOCK(&ic->ic_nodelock,&lock_state);
        TAILQ_INSERT_TAIL(&ic->ic_nodes, ni, ni_alloc_list);
        OS_RWLOCK_WRITE_UNLOCK(&ic->ic_nodelock,&lock_state);
    } while(0);
#endif
    ni->ni_flags |= IEEE80211_NODE_TEMP; /* useful for debugging */

    ieee80211_ref_node(ni);     /* mark referenced */

    IEEE80211_VAP_LOCK(vap);
    vap->iv_node_count++;
    IEEE80211_VAP_UNLOCK(vap);

    ni->ni_bss_node = ieee80211_ref_bss_node(vap);

    ni->ni_vap = vap;
    ni->ni_ic = ic;
    ni->ni_table = NULL;

    /* copy some default variables from parent */
    IEEE80211_ADDR_COPY(ni->ni_macaddr, macaddr);
    ni->ni_intval = ic->ic_intval; /* default beacon interval */

    /* set default rate and channel */
    ieee80211_node_set_chan(ni);

    ni->ni_txpower = ic->ic_txpowlimit;	/* max power */

    /* init our unicast / receive key state */
    ieee80211_crypto_resetkey(vap, &ni->ni_ucastkey, IEEE80211_KEYIX_NONE);
    ieee80211_crypto_resetkey(vap, &ni->ni_persta.nips_hwkey, IEEE80211_KEYIX_NONE);
    for (i = 0; i < IEEE80211_WEP_NKID; i++) {
        ieee80211_crypto_resetkey(vap, &ni->ni_persta.nips_swkey[i], IEEE80211_KEYIX_NONE);
    }
    ni->ni_ath_defkeyindex = IEEE80211_INVAL_DEFKEY;

    /* IBSS-only: mark as unassociated by default. */
    ni->ni_assoc_state = IEEE80211_NODE_ADHOC_STATE_UNAUTH_UNASSOC;

    /* 11n  or 11ac */
    ni->ni_chwidth = ic->ic_cwm_get_width(ic);

    IEEE80211_ADDR_COPY(ni->ni_bssid, vap->iv_bss->ni_bssid);


    return ni;
}

int wlan_add_sta_node(wlan_if_t vap, const u_int8_t *macaddr, u_int16_t auth_alg)
{
    struct ieee80211_node *ni;

    ni = ieee80211_vap_find_node(vap, macaddr);
    if (ni == NULL) {
        ni = ieee80211_dup_bss(vap, macaddr);
    }

    if (ni == NULL) {
        return -ENOMEM;
    }
    /* claim node immediately */
    ieee80211_free_node(ni);
    return 0;
}

#endif
