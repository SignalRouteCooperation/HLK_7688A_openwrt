/*
 *  Copyright (c) 2008 Atheros Communications Inc.
 * All Rights Reserved.
 *
 * Copyright (c) 2011 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 *
 */

#ifndef _ATH_STA_IEEE80211_NODE_H
#define _ATH_STA_IEEE80211_NODE_H

#include <osdep.h>
#include <sys/queue.h>
#include <ieee80211_admctl.h>


#include "ieee80211_scan.h"
#include <ieee80211_smartantenna.h>
#include <umac_lmac_common.h>
#include <ieee80211_smart_ant_api.h>
#include <ieee80211_ald.h>
//#define IEEE80211_DEBUG_REFCNT
//#define IEEE80211_DEBUG_NODELEAK

/* Forward declarations */
struct ieee80211com;
struct ieee80211vap;
struct ieee80211_node_table;
struct ieee80211_rsnparms;

#ifndef ATH_USB
typedef rwlock_t   ieee80211_node_lock_t;
typedef spinlock_t ieee80211_node_state_lock_t;

#define IEEE80211_NODE_STATE_LOCK_INIT(_node)     spin_lock_init(&(_node)->ni_state_lock);
#define IEEE80211_NODE_STATE_LOCK_DESTROY(_node)  spin_lock_destroy(&(_node)->ni_state_lock)
#define IEEE80211_NODE_STATE_LOCK(_node)          spin_lock(&(_node)->ni_state_lock)
#define IEEE80211_NODE_STATE_LOCK_BH(_node)       spin_lock_dpc(&(_node)->ni_state_lock)
#define IEEE80211_NODE_STATE_UNLOCK(_node)        spin_unlock(&(_node)->ni_state_lock)
#define IEEE80211_NODE_STATE_UNLOCK_BH(_node)     spin_unlock_dpc(&(_node)->ni_state_lock)
#define IEEE80211_NODE_STATE_PAUSE_LOCK(_node)         IEEE80211_NODE_STATE_LOCK(_node)
#define IEEE80211_NODE_STATE_PAUSE_UNLOCK(_node)       IEEE80211_NODE_STATE_UNLOCK(_node)




#else
typedef usb_readwrite_lock_t ieee80211_node_lock_t;
typedef usblock_t            ieee80211_node_state_lock_t;

#define IEEE80211_NODE_STATE_LOCK_INIT(_node)    IEEE80211_NODE_STATE_PAUSE_LOCK_INIT(_node); \
                                                 OS_USB_LOCK_INIT(&(_node)->ni_state_lock)
#define IEEE80211_NODE_STATE_LOCK_DESTROY(_node)   IEEE80211_NODE_STATE_PAUSE_LOCK_DESTROY(_node) ;\
                                         OS_USB_LOCK_DESTROY(&(_node)->ni_state_lock)

#define IEEE80211_NODE_STATE_LOCK(_node)          OS_USB_LOCK(&(_node)->ni_state_lock)
#define IEEE80211_NODE_STATE_LOCK_BH(_node)       IEEE80211_NODE_STATE_LOCK(_node)
#define IEEE80211_NODE_STATE_UNLOCK(_node)        OS_USB_UNLOCK(&(_node)->ni_state_lock)
#define IEEE80211_NODE_STATE_UNLOCK_BH(_node)     IEEE80211_NODE_STATE_UNLOCK(_node)
#define IEEE80211_NODE_STATE_PAUSE_LOCK_INIT(_node)     spin_lock_init(&(_node)->ni_pause_lock);
#define IEEE80211_NODE_STATE_PAUSE_LOCK_DESTROY(_node)  spin_lock_destroy(&(_node)->ni_pause_lock)
#define IEEE80211_NODE_STATE_PAUSE_LOCK(_node)          spin_lock_dpc(&(_node)->ni_pause_lock)
#define IEEE80211_NODE_STATE_PAUSE_UNLOCK(_node)        spin_unlock_dpc(&(_node)->ni_pause_lock)


#endif

#define WDS_AGING_COUNT 2
#define WDS_AGING_TIME 60000   /* in ms */
#define WDS_AGING_TIMER_VAL (WDS_AGING_TIME/2)

#if ATH_SUPPORT_AOW
/**
 * @brief : Node info pertaining to AoW capablities
 *
 */
typedef struct ieee80211_node_aow {
    u_int8_t    capable;       /* AoW Capable device */
    u_int8_t    dev_type;      /* Type of AoW device Transmit/Receive */
    u_int8_t    id;        /* AoW Network ID */
    u_int8_t    audio_type;    /* Type of Audio supported */
    u_int8_t    party_mode;    /* If Party Mode is enabled */
    u_int32_t   flags;   /* AoW flags */
}ieee80211_node_aow_t;

#endif  /* ATH_SUPPORT_AOW */

#if UMAC_SUPPORT_SMARTANTENNA
struct per_ratetable {
    u_int16_t nframes;
    u_int16_t nbad;
    int8_t rssi[SA_MAX_RECV_CHAINS];
};

struct smartantenna_state {
    u_int8_t rateidx;       /* rate index used for training */
    u_int8_t antenna;       /* training antenna */
    u_int8_t per;           /* training PER */
    u_int8_t nextant_per;   /* training PER */
    u_int8_t first_per;     /* flag for storing firt PER */
    u_int8_t rate_code;     /* current trainig rateCode */
    u_int8_t last_rateidx;  /* previous train rate index  */
    int8_t last_trained;    /* last trained index ; 1 - above the rate, -1 - below the rate */
    u_int16_t nbad;         /* falied frames for current training rate and antenna */
    u_int16_t nframes;      /* total frames sent for current training rate and antenna */
    int8_t rssi[SA_MAX_RECV_CHAINS][SA_MAX_RSSI_SAMPLES];   /* Block-ACK/ACK RSSI for all chains */
    u_int16_t nextant_nbad; /* falied frames for current training next rate and antenna */
    u_int16_t nextant_nframes; /* total frames sent for current training next rate and antenna */
    int8_t nextant_rssi[SA_MAX_RECV_CHAINS][SA_MAX_RSSI_SAMPLES];   /* next antennas Block-ACK/ACK RSSI */
    u_int8_t ant_map[SA_ANTENNA_COMBINATIONS];    /* Antenna combinations index map */
    u_int8_t extra_stats;   /* flag to collect extra stats */
    u_int8_t stats_dir;     /* direction of PER for extra stats */
    u_int8_t double_train;  /* flag to traing with double packets */
    u_int8_t extra_sel;     /* flag indicating collection of extra stats for selected antenna */
    u_int8_t extra_cmp;     /* flag indicating collection of extra stats for comparing antenna */
    u_int16_t extra_nbad;   /* falied frames for current training rate and antenna when collecting extra stats */
    u_int16_t extra_nframes; /* falied frames for current training rate and antenna when collecting extra stats */
};

#endif

struct  ieee80211_wnm_node {
    struct ieee80211_tfsreq         *tfsreq;
    struct ieee80211_tfsrsp         *tfsrsp;
    struct ieee80211_fmsreq         *fmsreq;
    struct ieee80211_fmsrsp         *fmsrsp;
    TAILQ_HEAD(, ieee80211_fmsreq_active_element) fmsreq_act_head;
    u_int8_t                        *timbcast_ie; /* captured TIM ie */
    u_int32_t                       timbcast_status;
    u_int8_t                        timbcast_interval;
    u_int8_t                        timbcast_dialogtoken;
    systime_t                       last_rcvpkt_tstamp;   /* to capture receive packet time */
};

/*
 * Node information. A node could represents a BSS in infrastructure network,
 * or an ad-hoc station in IBSS mode, or an associated station in HOSTAP mode.
 */
typedef struct ieee80211_node {
    TAILQ_ENTRY(ieee80211_node) ni_list; /* node table list */
    LIST_ENTRY(ieee80211_node)  ni_hash; /* hash list */
    TAILQ_ENTRY(ieee80211_node)   nodeleave_list;
#ifdef IEEE80211_DEBUG_NODELEAK
    TAILQ_ENTRY(ieee80211_node) ni_alloc_list; /* all allocated nodes */
#endif

#ifdef IEEE80211_DEBUG_REFCNT
#define NUM_TRACE_BUF           (1 << 5)
    const char  *func[NUM_TRACE_BUF];
    int         line[NUM_TRACE_BUF];
    atomic_t    refcnt[NUM_TRACE_BUF];
    atomic_t    index;
#endif

    struct ieee80211_node_table *ni_table;
    struct ieee80211vap     *ni_vap;
    struct ieee80211com     *ni_ic;
    struct ieee80211_node   *ni_bss_node;
    ieee80211_node_state_lock_t    ni_state_lock;   /* Node for the WAR for bug 58187 */
#ifdef ATH_USB
    spinlock_t            ni_pause_lock ;           /*htc specific as ni_state_lock is semaphore in case of HTC */
#endif
    atomic_t                ni_refcnt;
    u_int8_t                ni_authmode;            /* authentication mode */
    u_int8_t                ni_authalg;             /* authentication algorithm */
    u_int8_t                ni_authstatus;          /* authentication response status */
    u_int32_t               ni_flags;               /* special-purpose state */
#define IEEE80211_NODE_AUTH     0x00000001          /* authorized for data */
#define IEEE80211_NODE_QOS      0x00000002          /* QoS enabled */
#define IEEE80211_NODE_ERP      0x00000004          /* ERP enabled */
#define IEEE80211_NODE_HT       0x00000008          /* HT enabled */
/* NB: this must have the same value as IEEE80211_FC1_PWR_MGT */
#define IEEE80211_NODE_PWR_MGT  0x00000010          /* power save mode enabled */
#define IEEE80211_NODE_TSC_SET  0x00000020           /* keytsc for node has already been updated */
#define IEEE80211_NODE_UAPSD    0x00000040          /* U-APSD power save enabled */
#define IEEE80211_NODE_UAPSD_TRIG 0x00000080        /* U-APSD triggerable state */
#define IEEE80211_NODE_UAPSD_SP 0x00000100          /* U-APSD SP in progress */
#define IEEE80211_NODE_ATH      0x00000200          /* Atheros Owl or follow-on device */
#define IEEE80211_NODE_OWL_WDSWAR 0x00000400        /* Owl WDS workaround needed*/
#define IEEE80211_NODE_WDS      0x00000800          /* WDS link */
#define	IEEE80211_NODE_NOAMPDU  0x00001000          /* No AMPDU support */
#define IEEE80211_NODE_WEPTKIPAGGR 0x00002000       /* Atheros proprietary wep/tkip aggregation support */
#define IEEE80211_NODE_WEPTKIP  0x00004000
#define IEEE80211_NODE_TEMP     0x00008000          /* temp node (not in the node table) */
#define IEEE80211_NODE_11NG_VHT_INTEROP_AMSDU_DISABLE   0x00010000  /* 2.4ng VHT interop AMSDU disabled */
#define IEEE80211_NODE_40_INTOLERANT    0x00020000  /* HT40 Intolerant  */
#define IEEE80211_NODE_PAUSED   0x00040000          /* node is  paused*/
#define IEEE80211_NODE_EXTRADELIMWAR 0x00080000
#define IEEE80211_NODE_NAWDS 0x00100000          /* node is an NAWDS repeater */
#define IEEE80211_NODE_TDLS     0x00200000          /* TDLS node */
#define IEEE80211_NODE_REQ_HT20     0x00400000      /* HT20 requesting node */
#define IEEE80211_NODE_ATH_PAUSED 0x00800000         /* all the tid queues in ath layer are paused*/
#define IEEE80211_NODE_UAPSD_CREDIT_UPDATE 0x01000000  /*Require credit update*/
#define IEEE80211_NODE_KICK_OUT_DEAUTH     0x02000000  /*Require send deauth when h/w queue no data*/
#define IEEE80211_NODE_RRM                 0x04000000  /* RRM enabled node */
#if ATH_SUPPORT_WIFIPOS
#define IEEE80211_NODE_WAKEUP              0x08000000  /* Wakeup node */
#endif
#define IEEE80211_NODE_VHT                 0x10000000  /* VHT enabled node */
/* deauth/Disassoc wait for node cleanup till frame goes on
   air and tx feedback received */
#define IEEE80211_NODE_DELAYED_CLEANUP   0x20000000
#define IEEE80211_NODE_EXT_STATS           0x40000000  /* Extended stats enabled node */
#define IEEE80211_NODE_LEAVE_ONGOING       0x80000000  /* Prevent _ieee80211_node_leave() from reentry */

    u_int8_t                ni_ath_flags;       /* Atheros feature flags */
/* NB: These must have the same values as IEEE80211_ATHC_* */
#define IEEE80211_NODE_TURBOP   0x01          /* Turbo prime enable */
#define IEEE80211_NODE_AR       0x10          /* AR capable */
#define IEEE80211_NODE_BOOST    0x80

#if WDS_VENDOR_EXTENSION
    u_int8_t                ni_wds_tx_policy;  /* permissible ucast/mcast framing (3-addr or 4-addr) to wds peer */
#define WDS_POLICY_TX_UCAST_4ADDR        0x01
#define WDS_POLICY_TX_MCAST_4ADDR        0x02
#define WDS_POLICY_TX_DEFAULT            0x03
#define WDS_POLICY_TX_MASK               0x03
#endif

    u_int16_t               ni_ath_defkeyindex; /* Atheros def key index */
#define IEEE80211_INVAL_DEFKEY  0x7FFF

    u_int16_t               ni_associd; /* association id */
    u_int32_t               ni_scangen;
    systick_t               ni_assocuptime;  /* association up time */
    systick_t               ni_assocstarttime;   /* association start time */
    systick_t               ni_assoctime;        /* association process time */
    u_int16_t               ni_assocstatus;  /* association status code */
    u_int16_t               ni_p2p_assocstatus;    /* P2P assoc status code */
    u_int16_t               ni_txpower; /* current transmit power */

    u_int8_t                ni_record_status;
    systick_t               ni_4wayhstarttime;   /* 4-way handshake start time */
    systick_t               ni_2wayhstarttime;   /* 2-way handshake start time */
    systick_t               ni_4wayhstime;   /* 4-way handshake process time */
    systick_t               ni_2wayhstime;   /* 2-way handshake process time */

    u_int16_t               ni_vlan;    /* vlan tag */
    u_int32_t               *ni_challenge;  /* shared-key challenge */
    u_int8_t                *ni_wpa_ie;     /* captured WPA/RSN ie */
    u_int8_t                *ni_wps_ie;     /* captured WSC ie */
    u_int8_t                 ni_cc[3];      /* captured country code */

    u_int8_t                *ni_ath_ie; /* captured Atheros ie */
    u_int8_t                *ni_wme_ie; /* captured WME ie */

    u_int8_t                ni_wme_miss_threshold; /* wme_miss threshold */

    u_int16_t               ni_txseqs[IEEE80211_TID_SIZE];      /* tx seq per-tid */
    u_int16_t               ni_rxseqs[IEEE80211_TID_SIZE+1];/* rx seq previous per-tid,
                                                             * the additional one is for check seq on
                                                             * management frames. */
    u_int16_t               ni_last_rxseqs[IEEE80211_TID_SIZE+1];/* rx seq 2nd last(previous) per-tid,
                                                                  * the additional one is for check seq on
                                                                  * management frames. */
    systime_t               ni_rxfragstamp; /* timestamp of last rx frag */
    atomic_t                ni_rxfrag_lock; /* indicate timer handler is currently executing */
    wbuf_t                  ni_rxfrag[1];  /* rx frag reassembly.
                                             * XXX ???: do we have to have a reassembly line
                                             * for each TID */

    struct ieee80211_rsnparms ni_rsn;       /* RSN/WPA parameters */
    struct ieee80211_key    ni_ucastkey;    /* unicast key */
    struct {   // for adhoc mcast rx
        struct ieee80211_key nips_hwkey; // allocated clear key used to hand frame to sw
        struct ieee80211_key nips_swkey[IEEE80211_WEP_NKID]; // key used by sw to decrypt
    } ni_persta;
    int                     ni_rxkeyoff;    /* Receive key offset */
#if ATH_SUPPORT_WEP_MBSSID
    struct ieee80211_wep_mbssid ni_wep_mbssid;
#endif /*ATH_SUPPORT_WEP_MBSSID*/

    /*support for WAPI: keys for WAPI*/
#if ATH_SUPPORT_WAPI
	int ni_wkused;
	u32 ni_wapi_rekey_pkthresh;  /*wapi packets threshold for rekey, unicast or multicast depending on node*/
#endif

    /* 11n Capabilities */
    u_int16_t               ni_htcap;              /* HT capabilities */
    u_int32_t               ni_maxampdu;           /* maximum rx A-MPDU length */
    u_int32_t               ni_mpdudensity;        /* MPDU density in nano-sec */

#ifdef ATH_SUPPORT_TxBF
    union ieee80211_hc_txbf ni_txbf;               /* txbf capabilities */
#endif


    u_int8_t                ni_obssnonhtpresent;   /* OBSS non-HT STA present */
    u_int8_t                ni_txburstlimit;       /* TX burst limit */
    u_int8_t                ni_nongfpresent;       /* Non-green field devices present */
    u_int8_t                ni_streams;            /* number of streams supported */
    u_int8_t                ni_rxstreams;          /* number of rx streams supported */

    /* 11n information */
    enum ieee80211_cwm_width ni_chwidth;        /* recommended tx channel width */
    u_int8_t                ni_newchwidth;      /* channel width changed */
    int8_t                  ni_extoffset;       /* recommended ext channel offset */
    u_int8_t                ni_updaterates;     /* update rate table on SM power save */
#define	IEEE80211_NODE_SM_EN                    1
#define	IEEE80211_NODE_SM_PWRSAV_STAT	        2
#define	IEEE80211_NODE_SM_PWRSAV_DYN	        4
#define	IEEE80211_NODE_RATECHG                  8

    /* activity indicators */
    systime_t               ni_beacon_rstamp; /* host timestamp of received beacon and probes */
    u_int32_t               ni_probe_ticks;   /* inactivity mark count */

    u_int8_t		        ni_weptkipaggr_rxdelim; /* number of delimeters required to receive wep/tkip w/aggr */

    /* in activity indicators for AP mode */
    u_int16_t               ni_inact;       /* inactivity mark count */
    u_int16_t               ni_inact_reload;/* inactivity reload value */
#if ATH_BAND_STEERING
    u_int32_t               ni_rssi_seq; /* last RSSI sequence number */
    u_int8_t                ni_bs_rssi;    /* recv ssi used in bsteering */
#endif
#define ATH_TX_MAX_CONSECUTIVE_XRETRIES     50 /* sta gets kicked out after this */
    /* kick out STA when excessive retries occur */
    u_int16_t               ni_consecutive_xretries;

    /* hardware, not just beacon and probes */
    u_int8_t                ni_rssi;    /* recv ssi */

    u_int8_t                ni_macaddr[IEEE80211_ADDR_LEN]; /* MAC address */
    u_int8_t                ni_bssid[IEEE80211_ADDR_LEN]; /* BSSID */

    /* beacon, probe response */
    union {
        u_int8_t            data[8];
        u_int64_t           tsf;
    } ni_tstamp;                        /* from last rcv'd beacon */

    u_int16_t               ni_intval;  /* beacon interval */
    u_int16_t               ni_capinfo; /* negociated capabilities */

    u_int16_t               ni_ext_caps;/* exteneded node capabilities */
#define IEEE80211_NODE_C_QOS    0x0002  /* Wmm capable */
#define IEEE80211_NODE_C_UAPSD  0x0004  /* U-APSD capable */

    u_int8_t                ni_esslen;
    u_int8_t                ni_essid[IEEE80211_NWID_LEN+1];

    struct ieee80211_rateset ni_rates;   /* negotiated rate set */
    struct ieee80211_rateset ni_htrates; /* negotiated ht rate set */
    struct ieee80211_channel *ni_chan;

    u_int8_t                ni_erp;     /* ERP from beacon/probe resp */

    u_int32_t               ni_wait0_ticks;       /* ticks that we stay in dot11_assoc_state_zero state */
    u_int8_t                ni_dtim_period;       /* DTIM period */
    u_int8_t                ni_dtim_count;        /* DTIM count for last bcn */
    u_int16_t               ni_lintval;           /* listen interval */
    u_int8_t                ni_minbasicrate;      /* Min basic rate */

    systime_t               ss_last_data_time;    /* last time data RX/TX time */
#if ATH_SUPPORT_WIFIPOS
    u_int32_t               last_rx_desc_tsf;
#endif
    u_int16_t               ni_pause_count;
    /* power save queues */
    IEEE80211_NODE_POWERSAVE_QUEUE(ni_dataq)
    IEEE80211_NODE_POWERSAVE_QUEUE(ni_mgmtq)

    /* AP side UAPSD */
    u_int8_t                ni_uapsd;	/* U-APSD per-node flags matching WMM STA Qos Info field */
    u_int8_t                ni_uapsd_maxsp; /* maxsp from flags above */
    u_int16_t               ni_uapsd_trigseq[WME_NUM_AC]; /* trigger suppression on retry */
    u_int8_t                ni_uapsd_ac_trigena[WME_NUM_AC];    /* U-APSD per-node flags matching WMM STA Qos Info field */
    u_int8_t                ni_uapsd_ac_delivena[WME_NUM_AC];    /* U-APSD per-node flags matching WMM STA Qos Info field */
    int8_t                  ni_uapsd_dyn_trigena[WME_NUM_AC];    /* U-APSD per-node flags matching WMM STA Qos Info field */
    int8_t                  ni_uapsd_dyn_delivena[WME_NUM_AC];    /* U-APSD per-node flags matching WMM STA Qos Info field */
    ieee80211_admctl_priv_t ni_admctl_priv;         /* opaque handle with admctl private info */

    /* node statistics */
    struct ieee80211_nodestats   ni_stats;

    u_int8_t                ni_assoc_state; /* IBSS only */
#define IEEE80211_NODE_ADHOC_STATE_ZERO             0
#define IEEE80211_NODE_ADHOC_STATE_UNAUTH_UNASSOC   1
#define IEEE80211_NODE_ADHOC_STATE_AUTH_ASSOC       2

    u_int8_t                ni_chanswitch_tbtt;
#ifdef ATH_SUPPORT_TxBF
    /* beam forming flag */
    u_int8_t                ni_bf_update_cv        : 1, /* 1: request CV update */
                            ni_explicit_noncompbf  : 1, /* 1: set explicit non-compressed bf */
                            ni_explicit_compbf     : 1, /* 1: set explicit compressed bf*/
                            ni_implicit_bf         : 1, /* 1: set implicit bf */
                            Calibration_Done       : 1,
                            ni_rpt_received        : 1, /* 1: V/CV report recieved */
                            ni_txbf_timer_initialized   : 1,    /* 1: txbf related timer initialized */
                            ni_hw_cv_requested     : 1,     /* 1: cv requested by incorrect HW status*/
                            ni_allow_cv_update     : 1;     /* 1: sw time out , allow cv request */

    u_int8_t                ni_mmss;
    u_int32_t               ni_sw_cv_timeout;
    os_timer_t              ni_cv_timer;
    os_timer_t              ni_report_timer;

    u_int32_t               ni_cvtstamp;
    u_int8_t                ni_cvretry;
#endif

    struct ieee80211vap     *ni_wdsvap;     /* associated WDS vap */
#if ATH_SUPPORT_IQUE
    u_int8_t	ni_hbr_block;
    u_int32_t	ni_ique_flag;
#endif
#if UMAC_SUPPORT_TDLS
    struct ieee80211_tdls_node *ni_tdls;
#endif /* UMAC_SUPPORT_TDLS */
#ifdef ATH_SUPPORT_P2P
    u_int8_t    ni_p2p_awared;
#endif

#if ATH_SUPPORT_AOW
    u_int8_t    *ni_aow_ie;
    ieee80211_node_aow_t ni_aow;

    /* Whether procedures related to new association are done. */
    int          ni_new_assoc_done;

#if ATH_SUPPORT_AOW_TXSCHED
    u_int32_t   aow_min_goodput;    /* Contains the minimum goodput
                                       value measured for this node,
                                       prior to scheduling transmission
                                       for a batch of AoW MPDUs. */
    bool        aow_min_goodput_valid;
#endif  /* ATH_SUPPORT_AOW_TXSCHED */
#endif  /* ATH_SUPPORT_AOW */

    u_int16_t ni_tdls_caps;
#if UMAC_SUPPORT_RRM
    u_int8_t   ni_rrmreq_type;
    u_int8_t   ni_rrmlci_loc; /* RRM LCI request location subject */
    void       *ni_rrm_stats; /* RRM statistics */
    u_int8_t   rrm_dialog_token;
    u_int8_t   lm_dialog_token;
    u_int8_t   nr_dialog_token;
    u_int8_t   br_measure_token;
    u_int8_t   chload_measure_token;
    u_int8_t   stastats_measure_token;
    u_int8_t   nhist_measure_token;
    u_int8_t   tsm_measure_token;
    u_int8_t   frame_measure_token;
    u_int8_t   lci_measure_token;
#endif
#if UMAC_SUPPORT_SMARTANTENNA
    u_int8_t  current_tx_antenna; /* currently used tx antenna */
    u_int8_t  default_rx_antenna; /* default receive antenna */
    u_int8_t  current_rate_index; /* rate index of current used rate */
    u_int8_t  smartantenna_state; /* node specific smart antenna state */
    u_int8_t  is_training; /* current training status. Setting this indicates training is happenning */
    u_int8_t  train_type; /* frame based or protocol based */
    u_int8_t  previous_txant; /* previous used tx antenna */
    u_int8_t  prev_ratemax ; /* previous rate */
    u_int8_t  intense_train; /* setting this indicates to use more packets for training */
    u_int32_t ns_prev_tx_ucast; /* tx unicast packets trasmited in retraining intervel */
    u_int8_t  retrain_miss;  /* Number of consecutive retrain intervels with out doing retraining*/
    u_int8_t  long_retrain_count;  /* counter for long retrain */
    u_int8_t  traffic_none;  /* Number of consecutive retrain intervels with out any traffic*/
    u_int8_t  hysteresis;    /* hysteresis for throughput drop */
    u_int8_t  hybrid_train;    /* setting this to indicate hybrid mode for training */
    int8_t gput_avg_interval; /* number of intervals to ignore goodput for retraining*/
    int8_t retrain_trigger_type; /* +ve trigger or -ve trigger */
#if SA_DEBUG
    u_int32_t ts_trainstart;      /* train start time stamp */
    u_int32_t ts_traindone;       /* train done time stamp */
    u_int16_t prd_trigger_count;  /* periodic trigger count */
    u_int16_t perf_trigger_count; /* performence trigger count */
#endif
    struct smartantenna_state train_state; /* current training sate info */
    TAILQ_ENTRY(ieee80211_node) smartant_nodelist; /* smart antenna node table list */
    int32_t prev_goodput; /* average good put for previous intervel */
    int32_t current_goodput; /* average good put for current intervel */
    struct sa_rate_info rate_info; /* valid rates and information regarding them */
#endif
#if UNIFIED_SMARTANTENNA
    void *smart_ant_ccp; /* configuration control pointer */
    struct sa_rate_info rate_info; /* valid rates and information regarding them */
    struct sa_rate_cap rate_cap; /* rate capability information */
#endif
#if ATH_SUPPORT_HYFI_ENHANCEMENTS
    struct ath_ni_linkdiag ni_ald;
#endif
#ifdef ATH_SUPPORT_IBSS_NETLINK_NOTIFICATION
    u_int8_t	ni_rssi_class;
#endif



#if UMAC_SUPPORT_WNM
    struct ieee80211_wnm_node       *ni_wnm;
#endif

#if UMAC_SUPPORT_PROXY_ARP
#define IEEE80211_NODE_IPV6_MAX 4
    LIST_ENTRY(ieee80211_node) ni_ipv4_hash; /* ipv4 hash list */
    uint32_t ni_ipv4_addr;
    uint32_t ni_ipv4_lease_timeout;
    uint8_t ni_ipv6_addr[IEEE80211_NODE_IPV6_MAX][16];
    int ni_ipv6_nidx; /* index of the next IPv6 address for this node */
#endif
     u_int8_t ni_maxrate; /* Max Rate Per STA */
     u_int8_t ni_maxrate_legacy;
     u_int8_t ni_maxrate_ht;
    u_int8_t ni_maxrate_vht; /* b0-b3: mcs idx; b4-b7: # streams */
    u_int32_t   ni_vhtcap;        /* VHT capability */
    u_int16_t   ni_vht_cfreq1;    /* VHT Channel center frequency Index segment 1 */
    u_int16_t   ni_tx_vhtrates;   /* Negotiated Tx VHT rates */
    u_int16_t   ni_tx_max_rate;   /* Max VHT Tx Data rate */
    u_int16_t   ni_rx_vhtrates;   /* Negotiated Rx VHT rates */
    u_int16_t   ni_rx_max_rate;   /* Max VHT Rx Data rate */
    u_int16_t   ni_vht_basic_mcs; /* Basic VHT MCS map */
    u_int16_t   ni_phymode;       /* Phy mode */

    u_int32_t   ni_inst_rssi;
    u_int32_t   ps_state;
    bool        ni_pspoll;
    systime_t   ni_pspoll_time;      /* absolute system time; not TSF */
} IEEE80211_NODE, *PIEEE80211_NODE;


struct ieee80211_wds_addr {
	LIST_ENTRY(ieee80211_wds_addr)    wds_hash;
	u_int8_t    wds_macaddr[IEEE80211_ADDR_LEN];
	struct ieee80211_node    *wds_ni;
    /* ni_macaddr can be accessed from ni pointer. In case of quick
     * disconnect and connect, wds entry for this would move from active to
     * staged state.
     * When in staged, this should not be refering to stale
     * node pointer. So cache the mac address alone, so that, we can use
     * this information to figure out the actual node pointer. Also new hash
     */
    u_int8_t    wds_ni_macaddr[IEEE80211_ADDR_LEN];

#define IEEE80211_NODE_F_WDS_BEHIND   0x00001
#define IEEE80211_NODE_F_WDS_REMOTE   0x00002
#define IEEE80211_NODE_F_WDS_STAGE    0x00004
/* Set for first time, when node is added (for TDLS)*/
#define IEEE80211_NODE_F_WDS_START    0x00010
#define IEEE80211_NODE_F_WDS_HM       0x00020
    u_int32_t   flags;
	u_int16_t   wds_agingcount;
    u_int16_t   wds_staging_age;
    systime_t   wds_last_pkt_time;
    atomic_t    wds_refcnt; /* For host managed WDS feature */
};

#ifdef IEEE80211_DEBUG_REFCNT
#define TRACENODE(_ni, _func, _line) do {                                 \
    int index = 0;                                                        \
    int rfcnt = 0;                                                        \
    atomic_inc(&((_ni)->index));                                          \
    index = (int)(atomic_read(&((_ni)->index))) & (NUM_TRACE_BUF - 1);    \
    rfcnt = (int)(ieee80211_node_refcnt(_ni));                            \
    (_ni)->func[index] = _func;                                           \
    (_ni)->line[index] = _line;                                           \
    (_ni)->refcnt[index] = *((atomic_t *)(&rfcnt));                       \
} while (0)
#endif

#define IEEE80211_RSSI_RX       0x00000001
#define IEEE80211_RSSI_TX       0x00000002
#define IEEE80211_RSSI_EXTCHAN  0x00000004
#define IEEE80211_RSSI_BEACON   0x00000008
#define IEEE80211_RSSI_RXDATA   0x00000010

#define IEEE80211_RATE_TX 0
#define IEEE80211_RATE_RX 1
#define IEEE80211_LASTRATE_TX 2
#define IEEE80211_LASTRATE_RX 3
#define IEEE80211_RATECODE_TX 4
#define IEEE80211_RATECODE_RX 5

#if UMAC_SUPPORT_P2P
#define WME_UAPSD_NODE_MAXQDEPTH   100
#else
#define WME_UAPSD_NODE_MAXQDEPTH   8
#endif

#define WME_UAPSD_AC_CAN_TRIGGER(_ac, _ni)  \
             ((((_ni)->ni_uapsd_dyn_trigena[_ac] == -1) ? ((_ni)->ni_uapsd_ac_trigena[_ac]) \
                            :((_ni)->ni_uapsd_dyn_trigena[_ac])) &&  ((_ni)->ni_flags & IEEE80211_NODE_UAPSD_TRIG))
#define WME_UAPSD_AC_ISDELIVERYENABLED(_ac, _ni)  \
             ((((_ni)->ni_uapsd_dyn_delivena[_ac] == -1) ? ((_ni)->ni_uapsd_ac_delivena[_ac]) \
                            :((_ni)->ni_uapsd_dyn_delivena[_ac])) &&  ((_ni)->ni_flags & IEEE80211_NODE_UAPSD_TRIG))
#define IEEE80211_NODE_UAPSD_USETIM(_ni) \
        (WME_UAPSD_AC_ISDELIVERYENABLED(WME_AC_BK, _ni) && \
        WME_UAPSD_AC_ISDELIVERYENABLED(WME_AC_BE, _ni) && \
        WME_UAPSD_AC_ISDELIVERYENABLED(WME_AC_VI, _ni) && \
        WME_UAPSD_AC_ISDELIVERYENABLED(WME_AC_VO, _ni))
#define WME_UAPSD_NODE_INVALIDSEQ    0xffff
#define WME_UAPSD_NODE_TRIGSEQINIT(_ni)    (memset(&(_ni)->ni_uapsd_trigseq[0], 0xff, sizeof((_ni)->ni_uapsd_trigseq)))


#define    IEEE80211_NODE_STAT(ni,stat)    (ni->ni_stats.ns_##stat++)
#define    IEEE80211_NODE_STAT_ADD(ni,stat,v)    (ni->ni_stats.ns_##stat += v)
#define    IEEE80211_NODE_STAT_SUB(ni,stat,v)    (ni->ni_stats.ns_##stat -= v)
#define    IEEE80211_NODE_STAT_SET(ni,stat,v)    (ni->ni_stats.ns_##stat = v)

#define    IEEE80211_NODE_STAT_ADDRBASED(_vap, _macaddr, _stat) \
                    {\
                        struct ieee80211_node *_ni = NULL;\
                        if ((_ni = ieee80211_find_node(&(_vap)->iv_ic->ic_sta, (_macaddr))) != NULL) {\
                            IEEE80211_NODE_STAT(_ni, _stat);\
                            ieee80211_free_node(_ni);\
                        }\
                    }

 #define    IEEE80211_NODE_STAT_ADD_ADDRBASED(_vap, _macaddr, _stat, _v) \
                    {\
                        struct ieee80211_node *_ni = NULL;\
                        if ((_ni = ieee80211_find_node(&(_vap)->iv_ic->ic_sta, (_macaddr))) != NULL) {\
                            IEEE80211_NODE_STAT_ADD(_ni, _stat, (_v));\
                            ieee80211_free_node(_ni);\
                        }\
                    }

#define    IEEE80211_NODE_STAT_SUB_ADDRBASED(_vap, _macaddr, _stat, _v) \
                    {\
                        struct ieee80211_node *_ni = NULL;\
                        if ((_ni = ieee80211_find_node(&(_vap)->iv_ic->ic_sta, (_macaddr))) != NULL) {\
                            IEEE80211_NODE_STAT_SUB(_ni, _stat, (_v));\
                            ieee80211_free_node(_ni);\
                        }\
                    }

#define    IEEE80211_NODE_STAT_SET_ADDRBASED(_vap, _macaddr, _stat, _v) \
                    {\
                        struct ieee80211_node *_ni = NULL;\
                        if ((_ni = ieee80211_find_node(&(_vap)->iv_ic->ic_sta, (_macaddr))) != NULL) {\
                            IEEE80211_NODE_STAT_SET(_ni, _stat, (_v));\
                            ieee80211_free_node(_ni);\
                        }\
                    }


/*
 * Table of node instances.
 */

#define	IEEE80211_NODE_HASHSIZE	32

/* simple hash is enough for variation of macaddr */
#define	IEEE80211_NODE_HASH(addr)   \
    (((const u_int8_t *)(addr))[IEEE80211_ADDR_LEN - 1] % IEEE80211_NODE_HASHSIZE)

#if UMAC_SUPPORT_PROXY_ARP
#define IEEE80211_IPV4_HASHSIZE 32
#define IEEE80211_IPV4_HASH(n) \
    (((const uint8_t *)(&n))[3] % IEEE80211_IPV4_HASHSIZE)
#define IEEE80211_IPV6_HASHSIZE 32
#define IEEE80211_IPV6_HASH(n) \
    (((const uint8_t *)(n))[15] % IEEE80211_IPV6_HASHSIZE)
#endif

#if UMAC_SUPPORT_PROXY_ARP
struct ieee80211_ipv6_node {
    struct ieee80211_node *node;
    int index;
    LIST_ENTRY(ieee80211_ipv6_node) ni_hash;       /* ipv6 hash list */
    TAILQ_ENTRY(ieee80211_ipv6_node) ni_list; /* ipv6 node table list */
};
#endif

struct ieee80211_node_table {
    struct ieee80211com             *nt_ic;         /* back reference */
    ieee80211_node_lock_t           nt_nodelock;    /* on node table */
    ieee80211_node_lock_t           nt_wds_nodelock;    /* on node table */
    TAILQ_HEAD(, ieee80211_node)    nt_node;        /* information of all nodes */
#if UMAC_SUPPORT_SMARTANTENNA
    TAILQ_HEAD(, ieee80211_node)    nt_smartant_node;  /* information of all nodes in smart antena train list */
#endif
    ATH_LIST_HEAD(, ieee80211_node) nt_hash[IEEE80211_NODE_HASHSIZE];
    ATH_LIST_HEAD(, ieee80211_wds_addr) nt_wds_hash[IEEE80211_NODE_HASHSIZE];
    const char                      *nt_name;
    u_int32_t                       nt_scangen;
    os_timer_t                      nt_wds_aging_timer;    /* timer to age out wds entries */
#if UMAC_SUPPORT_PROXY_ARP
    ATH_LIST_HEAD(, ieee80211_node)      nt_ipv4_hash[IEEE80211_IPV4_HASHSIZE];
    TAILQ_HEAD(, ieee80211_ipv6_node)    nt_ipv6_node;     /* all ipv6 nodes */
    ATH_LIST_HEAD(, ieee80211_ipv6_node) nt_ipv6_hash[IEEE80211_IPV6_HASHSIZE];
#endif
};

void ieee80211_node_attach(struct ieee80211com *ic);
void ieee80211_node_detach(struct ieee80211com *ic);
void ieee80211_node_vattach(struct ieee80211vap *vap);
void ieee80211_node_vdetach(struct ieee80211vap *vap);
int ieee80211_node_latevattach(struct ieee80211vap *vap);
void ieee80211_node_latevdetach(struct ieee80211vap *vap);

void ieee80211_copy_bss(struct ieee80211_node *nbss, const struct ieee80211_node *obss);
int ieee80211_reset_bss(struct ieee80211vap *vap);
int ieee80211_sta_join_bss(struct ieee80211_node *selbs);

struct ieee80211_node *
ieee80211_ref_bss_node(struct ieee80211vap *vap);

#ifdef IEEE80211_DEBUG_REFCNT
#define	ieee80211_ref_bss_node(vap) \
    ieee80211_ref_bss_node_debug(vap, __func__, __LINE__)

struct ieee80211_node *
ieee80211_ref_bss_node_debug(struct ieee80211vap *vap,
                         const char *func, int line);

#else  /* !IEEE80211_DEBUG_REFCNT */
struct ieee80211_node *
ieee80211_ref_bss_node(struct ieee80211vap *vap);
#endif  /* IEEE80211_DEBUG_REFCNT */


struct ieee80211_node *
ieee80211_add_neighbor(struct ieee80211_node *ni,
                       ieee80211_scan_entry_t scan_entry);

struct ieee80211_node *
ieee80211_tmp_node(struct ieee80211vap *vap, const u_int8_t *macaddr);

int
ieee80211_dup_ibss(struct ieee80211vap *vap, struct ieee80211_node *org_ibss);

int
ieee80211_sta_join(struct ieee80211vap *vap, ieee80211_scan_entry_t scan_entry);

int
ieee80211_join_ibss(struct ieee80211vap *vap, ieee80211_scan_entry_t scan_entry);

int
ieee80211_ibss_merge(struct ieee80211_node *ni, ieee80211_scan_entry_t scan_entry);

int
ieee80211_create_ibss(struct ieee80211vap *vap,
                      const u_int8_t *bssid,
                      const u_int8_t *essid,
                      const u_int16_t esslen);
int
ieee80211_create_infra_bss(struct ieee80211vap *vap,
                      const u_int8_t *essid,
                           const u_int16_t esslen);

struct ieee80211_node *
ieee80211_alloc_node(struct ieee80211_node_table *nt,
                     struct ieee80211vap *vap,
                     const u_int8_t *macaddr);

void
ieee80211_wnm_nattach(struct ieee80211_node *ni);

void
ieee80211_wnm_ndetach(struct ieee80211_node *ni);

void ieee80211_timeout_stations(struct ieee80211_node_table *nt);

/*
 * ieee80211_node_refcnt	reference count for printing (only)
 */
#define	ieee80211_node_refcnt(_ni)	atomic_read(&(_ni)->ni_refcnt)

#ifdef IEEE80211_DEBUG_REFCNT

void ieee80211_free_node_debug(struct ieee80211_node *,
                               const char *func, int line);
struct ieee80211_node *
ieee80211_find_node_debug(struct ieee80211_node_table *nt,
                                                 const u_int8_t *macaddr,
                                                 const char *func, int line);
struct ieee80211_node *
ieee80211_find_txnode_debug(struct ieee80211vap *vap, const u_int8_t *macaddr,
                            const char *func, int line);
struct ieee80211_node *
ieee80211_find_rxnode_debug(struct ieee80211com *ic,
                            const struct ieee80211_frame_min *wh,
                            const char *func, int line);
/* No debug version */
struct ieee80211_node *
ieee80211_find_rxnode_nolock(struct ieee80211com *ic,
                      const struct ieee80211_frame_min *wh);

struct ieee80211_node *
ieee80211_ref_node_debug(struct ieee80211_node *ni,
                         const char *func, int line);
void
ieee80211_unref_node_debug(struct ieee80211_node **ni,
                           const char *func, int line);
bool
ieee80211_node_leave_debug(struct ieee80211_node *ni,
                           const char *func, int line);
bool
ieee80211_sta_leave_debug(struct ieee80211_node *ni,
                           const char *func, int line);

#define	ieee80211_free_node(ni) \
    ieee80211_free_node_debug(ni, __func__, __LINE__)
#define ieee80211_find_node(nt, mac)    \
    ieee80211_find_node_debug(nt, mac, __func__, __LINE__)
#define ieee80211_find_txnode(vap, mac) \
    ieee80211_find_txnode_debug(vap, mac, __func__, __LINE__)
#define ieee80211_find_rxnode(ic, wh)   \
    ieee80211_find_rxnode_debug(ic, wh, __func__, __LINE__)

#if ATH_SUPPORT_WRAP
#define ieee80211_vap_find_node(_vap, _mac)     \
    ieee80211_find_wrap_node_debug(_vap, _mac, __func__, __LINE__)
#else
#define ieee80211_vap_find_node(_vap, _mac)     \
    ieee80211_find_node_debug(&(_vap)->iv_ic->ic_sta, _mac, __func__, __LINE__)
#endif

#define	ieee80211_ref_node(ni) \
    ieee80211_ref_node_debug(ni, __func__, __LINE__)
#define	ieee80211_unref_node(ni) \
    ieee80211_unref_node_debug(ni, __func__, __LINE__)

#define _ieee80211_node_leave(ni) \
    ieee80211_node_leave_debug(ni, __func__, __LINE__)

#define ieee80211_sta_leave(ni) \
    ieee80211_sta_leave_debug(ni, __func__, __LINE__)

#else  /* !IEEE80211_DEBUG_REFCNT */

void ieee80211_free_node(struct ieee80211_node *ni);
struct ieee80211_node *
ieee80211_find_node(struct ieee80211_node_table *nt,
                    const u_int8_t *macaddr);
struct ieee80211_node *
ieee80211_find_txnode(struct ieee80211vap *vap,
                      const u_int8_t *macaddr);
struct ieee80211_node *
ieee80211_find_rxnode(struct ieee80211com *ic,
                      const struct ieee80211_frame_min *wh);

#if ATH_SUPPORT_WRAP
#define ieee80211_vap_find_node(_vap, _mac)     \
    ieee80211_find_wrap_node(_vap, _mac)
#else
#define ieee80211_vap_find_node(_vap, _mac)     \
    ieee80211_find_node(&(_vap)->iv_ic->ic_sta, _mac)
#endif

struct ieee80211_node *
ieee80211_find_rxnode_nolock(struct ieee80211com *ic,
                      const struct ieee80211_frame_min *wh);

static INLINE struct ieee80211_node *
ieee80211_ref_node(struct ieee80211_node *ni)
{
    atomic_inc(&(ni->ni_refcnt));
    return ni;
}

static INLINE void
ieee80211_unref_node(struct ieee80211_node **ni)
{
    atomic_dec(&((*ni)->ni_refcnt));
    *ni = NULL;			/* guard against use */
}

bool _ieee80211_node_leave(struct ieee80211_node *ni);

void ieee80211_node_clear_keys(struct ieee80211_node *ni);

#ifndef ATH_HTC_MII_RXIN_TASKLET
#define     IEEE80211_NODE_LEAVE(_ni) _ieee80211_node_leave(_ni)
#else
bool IEEE80211_NODE_LEAVE(struct ieee80211_node *ni);

#endif
bool ieee80211_sta_leave(struct ieee80211_node *ni);

#endif /* IEEE80211_DEBUG_REFCNT */


/* after adding the new staging flag for the wds, ieee80211_find_wds_node
 * would get called in the context of ieee80211_find_node OR
 * ieee80211_find_txnode. Calling function without '_' would call recursion.
 * To avoid this, _ieee80211_find_node, is made global by removing the
 * static, also moved the definition here.
 */
struct ieee80211_node *
#ifdef IEEE80211_DEBUG_REFCNT
_ieee80211_find_node_debug(
        struct ieee80211_node_table *nt, const u_int8_t *macaddr,
        const char *func, int line);
#else
_ieee80211_find_node(struct ieee80211_node_table *nt,
                    const u_int8_t *macaddr);
#endif

#if UMAC_SUPPORT_PROXY_ARP
struct ieee80211_node *
ieee80211_find_node_by_ipv4(struct ieee80211_node_table *nt,
                            const uint32_t addr);
void
ieee80211_node_add_ipv4(struct ieee80211_node_table *nt,
                        struct ieee80211_node *ni,
                        const uint32_t ipaddr);
void
ieee80211_node_remove_ipv4(struct ieee80211_node *ni);
struct ieee80211_node *
ieee80211_find_node_by_ipv6(struct ieee80211_node_table *nt,
                            u8 *ip6addr);
int
ieee80211_node_add_ipv6(struct ieee80211_node_table *nt,
                        struct ieee80211_node *ni,
                        u8 *ip6addr);
void
ieee80211_node_remove_ipv6(struct ieee80211_node_table *nt, u8 *ip6addr);
#endif

#if ATH_SUPPORT_WRAP
struct ieee80211_node *
#ifdef IEEE80211_DEBUG_REFCNT
ieee80211_find_wrap_node_debug(struct ieee80211vap *vap, const u_int8_t *macaddr,
                               const char *func, int line);
#else
ieee80211_find_wrap_node(struct ieee80211vap *vap, const u_int8_t *macaddr);
#endif
#endif

void ieee80211_kick_node(struct ieee80211_node *ni);

#ifdef IEEE80211_DEBUG_NODELEAK
void
ieee80211_dump_alloc_nodes(struct ieee80211com *ic);
#endif

void ieee80211_node_set_chan(struct ieee80211_node *ni);
int ieee80211_node_join(struct ieee80211_node *ni);
struct ieee80211_node *
ieee80211_dup_bss(struct ieee80211vap *vap, const u_int8_t *macaddr);


static INLINE int
ieee80211_node_is_authorized(const struct ieee80211_node *ni)
{
    return (ni->ni_flags & IEEE80211_NODE_AUTH);
}

void ieee80211_node_authorize(struct ieee80211_node *ni);

void ieee80211_node_unauthorize(struct ieee80211_node *ni);

static INLINE void
ieee80211_node_activity(struct ieee80211_node *ni)
{
    ni->ni_inact = ni->ni_inact_reload;
}

#define	IEEE80211_INACT_INIT    (30/IEEE80211_INACT_WAIT)   /* initial */
#define IEEE80211_INACT_AUTH    (180/IEEE80211_INACT_WAIT)    /* associated but not authorized */
#define IEEE80211_INACT_RUN     (300/IEEE80211_INACT_WAIT)    /* authorized */
#define IEEE80211_INACT_PROBE   (90/IEEE80211_INACT_WAIT)    /* probe */

typedef void ieee80211_iter_func(void *, struct ieee80211_node *);
void ieee80211_iterate_node(struct ieee80211com *ic, ieee80211_iter_func *func, void *arg);


/*
 * Accessor methods for node
 */

/* Get the VAP object that this node belongs to */
static INLINE struct ieee80211vap *
ieee80211_node_get_vap(struct ieee80211_node *ni)
{
    return ni->ni_vap;
}

static INLINE u_int16_t
ieee80211_node_get_txpower(struct ieee80211_node *ni)
{
    return ni->ni_txpower;
}

/* Return the beacon interval of associated BSS */
static INLINE u_int16_t
ieee80211_node_get_beacon_interval(struct ieee80211_node *ni)
{
    return ni->ni_intval;
}

static INLINE u_int64_t
ieee80211_node_get_tsf(struct ieee80211_node *ni)
{
    return ni->ni_tstamp.tsf;
}

static INLINE u_int16_t
ieee80211_node_get_associd(struct ieee80211_node *ni)
{
    return ni->ni_associd;
}

static INLINE u_int8_t *
ieee80211_node_get_bssid(struct ieee80211_node *ni)
{
    return ni->ni_bssid;
}

static INLINE u_int8_t *
ieee80211_node_get_macaddr(struct ieee80211_node *ni)
{
    return ni->ni_macaddr;
}

static INLINE u_int16_t
ieee80211_node_get_maxampdu(struct ieee80211_node *ni)
{
    return ni->ni_maxampdu;
}

#ifdef ATH_SWRETRY
static INLINE u_int16_t
ieee80211_node_get_pwrsaveq_len(struct ieee80211_node *ni)
{
    struct node_powersave_queue *dataq,*mgmtq;
    dataq = &ni->ni_dataq;
    mgmtq  = &ni->ni_mgmtq;

    return (dataq->nsq_len + mgmtq->nsq_len);
}
#endif

#define IEEE80211_NODE_CLEAR_HTCAP(_ni)     ((_ni)->ni_htcap = 0)

#define IEEE80211_NODE_USE_HT(_ni)          ((_ni)->ni_flags & IEEE80211_NODE_HT)
#define IEEE80211_NODE_USEAMPDU(_ni)        ieee80211node_use_ampdu(_ni)

#define IEEE80211_NODE_USE_VHT(_ni)          ((_ni)->ni_flags & IEEE80211_NODE_VHT)
static INLINE int ieee80211node_use_ampdu(struct ieee80211_node *ni)
{
    if (IEEE80211_NODE_USE_HT(ni) &&
        !(ni->ni_flags & IEEE80211_NODE_NOAMPDU) && !ni->ni_pspoll) {
        return(1);  /* Supports AMPDU */
    }
    return(0);  /* Do not use AMPDU since non HT */
}

#define IEEE80211_NODE_AC_UAPSD_ENABLED(_ni, _ac) ((_ni)->ni_uapsd_ac_delivena[(_ac)])


/*
 * ************************************
 * IEEE80211_NODE Interfaces
 * ************************************
 */
static INLINE void
ieee80211node_set_flag(struct ieee80211_node *ni, u_int32_t flag)
{
    ni->ni_flags |= flag;
}

static INLINE void
ieee80211node_clear_flag(struct ieee80211_node *ni, u_int32_t flag)
{
    ni->ni_flags &= ~flag;
}

static INLINE int
ieee80211node_has_flag(struct ieee80211_node *ni, u_int32_t flag)
{
    return ((ni->ni_flags & flag) != 0);
}



static INLINE u_int16_t
ieee80211node_get_phymodes(struct ieee80211_node *ni)
{
    return ni->ni_phymode;
}


static INLINE void
ieee80211node_set_athflag(struct ieee80211_node *ni, u_int8_t flag)
{
    ni->ni_ath_flags |= flag;
}

static INLINE void
ieee80211node_clear_athflag(struct ieee80211_node *ni, u_int8_t flag)
{
    ni->ni_ath_flags &= ~flag;
}

static INLINE int
ieee80211node_has_athflag(struct ieee80211_node *ni, u_int8_t flag)
{
    return ((ni->ni_ath_flags & flag) != 0);
}

static INLINE void
ieee80211node_set_txpower(struct ieee80211_node *ni, u_int16_t txpower)
{
    ni->ni_txpower = txpower;
}

static INLINE void
ieee80211_node_set_beacon_interval(struct ieee80211_node *ni, u_int16_t intval)
{
    ni->ni_intval = intval;
}

static INLINE u_int8_t
ieee80211node_get_rssi(struct ieee80211_node *ni)
{
    return ni->ni_rssi;
}

static INLINE struct ieee80211_rateset *
ieee80211node_get_rateset(struct ieee80211_node *ni)
{
    return &ni->ni_rates;
}

static INLINE u_int8_t *
ieee80211node_get_tstamp(struct ieee80211_node *ni)
{
    return ni->ni_tstamp.data;
}

static INLINE u_int16_t
ieee80211node_get_capinfo(struct ieee80211_node *ni)
{
    return ni->ni_capinfo;
}

static INLINE int
ieee80211node_has_cap(struct ieee80211_node *ni, u_int16_t cap)
{
    return ((ni->ni_capinfo & cap) != 0);
}

static INLINE int
ieee80211node_is_paused(struct ieee80211_node *ni)
{
    return ( (ni->ni_flags & IEEE80211_NODE_PAUSED) != 0);
}

static INLINE int
ieee80211node_pause(struct ieee80211_node *ni)
{
    u_int16_t pause_count;
    IEEE80211_NODE_STATE_PAUSE_LOCK(ni);
    pause_count = ++ni->ni_pause_count;
    ni->ni_flags |= IEEE80211_NODE_PAUSED;
    IEEE80211_NODE_STATE_PAUSE_UNLOCK(ni);
    return pause_count;
}

static INLINE int
ieee80211node_unpause(struct ieee80211_node *ni)
{
	/*
	** Note:
	** Check NI is valid, just in case
	*/

	if( ni )
	{
        IEEE80211_NODE_STATE_PAUSE_LOCK(ni);

		/*
		** Simply set the pause count to zero.  This should
		** NOT have any detrimental effects
		****** PERMINANT FIX REQUIRED, THIS IS TEMPORARY *****
		*/

		if ( ni->ni_pause_count > 0 ) {
			ni->ni_pause_count--;
        }
		else {
			//printk("%s: Pause Count already zero\n",__func__);
        }

		/*
		** You can put a debug message here in case you want
		** notification of an extra unpause.  At this point, we
		** don't want to assert
		*/

    	if (ni->ni_pause_count == 0 && (ni->ni_flags & IEEE80211_NODE_PAUSED))
    	{
    	    ni->ni_flags &= ~IEEE80211_NODE_PAUSED;
    	    ieee80211_node_saveq_flush(ni);
    	}
        IEEE80211_NODE_STATE_PAUSE_UNLOCK(ni);

    	return ni->ni_pause_count;
	}
	return (0);
}
/*
 * 11n
 */
static INLINE int
ieee80211_node_has_htcap(struct ieee80211_node *ni, u_int16_t htcap)
{
    return ((ni->ni_htcap & htcap) != 0);
}

#if ATH_SUPPORT_IQUE
static INLINE u_int8_t
ieee80211_node_get_hbr_block_state(struct ieee80211_node *ni)
{
    return ni->ni_hbr_block;
}
#endif

#define    IEEE80211_NODE_AID(ni)    IEEE80211_AID(ni->ni_associd)


/* Not supported */
#define IEEE80211_NODE_WDSWAR_ISSENDDELBA(_ni)    \
    ( 0 )

int wlan_node_alloc_aid_bitmap(wlan_if_t vap, u_int16_t old_len);

#endif /* end of _ATH_STA_IEEE80211_NODE_H */
