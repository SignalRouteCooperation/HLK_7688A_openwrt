/*
 * Copyright (c) 2010, Atheros Communications Inc.
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
 * Defintions for the Atheros Wireless LAN controller driver.
 */
#ifndef _DEV_OL_ATH_ATHVAR_H
#define _DEV_OL_ATH_ATHVAR_H

#include <osdep.h>
#include <a_types.h>
#include <a_osapi.h>
#include "ieee80211_channel.h"
#include "ieee80211_prdperfstats.h"
#include "ieee80211_proto.h"
#include "ieee80211_rateset.h"
#include "ieee80211_regdmn.h"
#include "ieee80211_wds.h"
#include "ieee80211_ique.h"
#include "ieee80211_acs.h"
#include "ieee80211_csa.h"
#include "asf_amem.h"
#include "adf_os_types.h"
#include "adf_os_lock.h"
#include "wmi_unified_api.h"
#include "htc_api.h"
#include "bmi_msg.h"
#if QCA_OL_11AC_FAST_PATH
#include "ol_htt_api.h"
#endif /* QCA_OL_11AC_FAST_PATH */
#include "ol_txrx_api.h"
#include "ol_txrx_ctrl_api.h"
#include "ol_txrx_osif_api.h"
#include "ol_params.h"
#include <pktlog_ac_api.h>
#include "epping_test.h"
#include "wdi_event_api.h"
#include <ieee80211_smart_ant_api.h>
#include "ol_helper.h"
#if PERF_FIND_WDS_NODE
#include "wds_addr.h"
#endif
#if !(ATH_SUPPORT_WRAP_BRIDGE)
/* WRAP SKB marks used by the hooks to optimize */
#define WRAP_ATH_MARK              0x8000

#define WRAP_FLOOD                 0x0001  /*don't change see NF_BR_FLOOD netfilter_bridge.h*/
#define WRAP_DROP                  0x0002  /*mark used to drop short circuited pkt*/
#define WRAP_REFLECT               0x0004  /*mark used to identify reflected multicast*/
#define WRAP_ROUTE                 0x0008  /*mark used allow local deliver to the interface*/

#define WRAP_MARK_FLOOD            (WRAP_ATH_MARK | WRAP_FLOOD)
#define WRAP_MARK_DROP             (WRAP_ATH_MARK | WRAP_DROP)
#define WRAP_MARK_REFLECT          (WRAP_ATH_MARK | WRAP_REFLECT)
#define WRAP_MARK_ROUTE            (WRAP_ATH_MARK | WRAP_ROUTE)

#define WRAP_MARK_IS_FLOOD(_mark)  ((_mark & WRAP_ATH_MARK)?((_mark & WRAP_FLOOD)?1:0):0)
#define WRAP_MARK_IS_DROP(_mark)   ((_mark & WRAP_ATH_MARK)?((_mark & WRAP_DROP)?1:0):0)
#define WRAP_MARK_IS_REFLECT(_mark) ((_mark & WRAP_ATH_MARK)?((_mark & WRAP_REFLECT)?1:0):0)
#define WRAP_MARK_IS_ROUTE(_mark)  ((_mark & WRAP_ATH_MARK)?((_mark & WRAP_ROUTE)?1:0):0)
#endif
typedef void * hif_handle_t;
typedef void * hif_softc_t;

struct ath_version {
    u_int32_t    host_ver;
    u_int32_t    target_ver;
    u_int32_t    wlan_ver;
    u_int32_t    abi_ver;
};

typedef enum _ATH_BIN_FILE {
    ATH_OTP_FILE,
    ATH_FIRMWARE_FILE,
    ATH_PATCH_FILE,
    ATH_BOARD_DATA_FILE,
    ATH_FLASH_FILE,
    ATH_TARGET_EEPROM_FILE,    
} ATH_BIN_FILE;

typedef enum _ol_target_status  {
     OL_TRGET_STATUS_CONNECTED = 0,    /* target connected */
     OL_TRGET_STATUS_RESET,        /* target got reset */
     OL_TRGET_STATUS_EJECT,        /* target got ejected */
} ol_target_status;

enum ol_ath_tx_ecodes  {
    TX_IN_PKT_INCR=0,
    TX_OUT_HDR_COMPL,
    TX_OUT_PKT_COMPL,
    PKT_ENCAP_FAIL,
    TX_PKT_BAD,
    RX_RCV_MSG_RX_IND,
    RX_RCV_MSG_PEER_MAP,
    RX_RCV_MSG_TYPE_TEST
} ;

#ifndef ATH_CAP_DCS_CWIM
#define ATH_CAP_DCS_CWIM 0x1
#define ATH_CAP_DCS_WLANIM 0x2
#endif
#define STATS_MAX_RX_CES      8
/*
 * structure to hold the packet error count for CE and hif layer
*/
struct ol_ath_stats {
    int hif_pipe_no_resrc_count;
    int ce_ring_delta_fail_count;
    int sw_index[STATS_MAX_RX_CES];
    int write_index[STATS_MAX_RX_CES];
};

struct ol_ath_target_cap {
    A_UINT32                wmi_service_bitmap[WMI_SERVICE_BM_SIZE]; /* wmi services bitmap received from Target */
    wmi_resource_config     wlan_resource_config; /* default resource config,the os shim can overwrite it */
    /* any other future capabilities of the target go here */

};
/* callback to be called by durin target initialization sequence
 * to pass the target
 * capabilities and target default resource config to os shim.
 * the os shim can change the default resource config (or) the
 * service bit map to enable/disable the services. The change will
 * pushed down to target.
 */
typedef void   (* ol_ath_update_fw_config_cb)\
    (struct ol_ath_softc_net80211 *scn, struct ol_ath_target_cap *tgt_cap);

/*
 * memory chunck allocated by Host to be managed by FW
 * used only for low latency interfaces like pcie
 */
struct ol_ath_mem_chunk {
    u_int32_t *vaddr;
    u_int32_t paddr;
    adf_os_dma_mem_context(memctx);
    u_int32_t len;
    u_int32_t req_id;
};
/** dcs wlan wireless lan interfernce mitigation stats */

/*
 * wlan_dcs_im_tgt_stats_t is defined in wmi_unified.h
 * The below stats are sent from target to host every one second.
 * prev_dcs_im_stats - The previous statistics at last known time
 * im_intr_count, number of times the interfernce is seen continuously
 * sample_count - int_intr_count of sample_count, the interference is seen
 */
typedef struct _wlan_dcs_im_host_stats {
	wlan_dcs_im_tgt_stats_t prev_dcs_im_stats;
    A_UINT8   im_intr_cnt;              		/* Interefernce detection counter */
    A_UINT8   im_samp_cnt;              		/* sample counter */
} wlan_dcs_im_host_stats_t;

typedef enum {
    DCS_DEBUG_DISABLE=0,
    DCS_DEBUG_CRITICAL=1,
    DCS_DEBUG_VERBOSE=2,
} wlan_dcs_debug_t;

#if ATH_SUPPORT_HYFI_ENHANCEMENTS
struct ol_ath_ald_record {
    u_int32_t               free_descs;
    u_int32_t               pool_size;
    u_int16_t               ald_free_buf_lvl; /* Buffer Full warning threshold */
    u_int                   ald_buffull_wrn;
};
#endif

#define DCS_PHYERR_PENALTY      500
#define DCS_PHYERR_THRESHOLD    300
#define DCS_RADARERR_THRESHOLD 1000
#define DCS_COCH_INTR_THRESHOLD  30 /* 30 % excessive channel utilization */
#define DCS_USER_MAX_CU          50 /* tx channel utilization due to our tx and rx */
#define DCS_INTR_DETECTION_THR    6
#define DCS_SAMPLE_SIZE          10

typedef struct _wlan_host_dcs_params {
	u_int8_t                dcs_enable; 			/* if dcs enabled or not, along with running state*/
    wlan_dcs_debug_t        dcs_debug;              /* 0-disable, 1-critical, 2-all */
	u_int32_t			    phy_err_penalty;     	/* phy error penalty*/
	u_int32_t			    phy_err_threshold;
	u_int32_t				radar_err_threshold;
	u_int32_t				coch_intr_thresh ;
	u_int32_t				user_max_cu; 			/* tx_cu + rx_cu */
	u_int32_t 				intr_detection_threshold;
	u_int32_t 				intr_detection_window;
	wlan_dcs_im_host_stats_t scn_dcs_im_stats;
} wlan_host_dcs_params_t;

#if OL_ATH_SUPPORT_LED
#define PEREGRINE_LED_GPIO    1
typedef enum _OL_BLINK_STATE {
    OL_BLINK_DONE = 0,
    OL_BLINK_OFF_START = 1,
    OL_BLINK_ON_START = 2,
    OL_BLINK_STOP = 3,
    OL_NUMER_BLINK_STATE,
} OL_BLINK_STATE;

typedef enum _OL_LED_EVENT {
    OL_ATH_LED_TX = 0,
    OL_ATH_LED_RX = 1,
    OL_ATH_LED_POLL = 2,
    OL_NUMER_LED_EVENT,
} OL_LED_EVENT;

typedef struct {
    u_int32_t    timeOn;      // LED ON time in ms
    u_int32_t    timeOff;     // LED OFF time in ms
} OL_LED_BLINK_RATES;

#endif

#if DBGLOG_WQ_BASED
struct dbglog_workqueue_ctx {
    adf_os_work_t      debug_print_work;
    adf_nbuf_queue_t   dbglog_evt_list_head;
    adf_os_spinlock_t  dbglog_lock;
};
#endif

#define MAX_MEM_CHUNKS 32

struct ol_ath_softc_net80211 {
    struct ieee80211com     sc_ic;      /* NB: base class, must be first */
    ol_pktlog_dev_t 		*pl_dev;    /* Must be second- pktlog handle */
    u_int32_t               sc_prealloc_idmask;   /* preallocated vap id bitmap: can only support 32 vaps */
    u_int32_t               macreq_enabled;     /* user mac request feature enable/disable */
    struct {
        asf_amem_instance_handle handle;
        adf_os_spinlock_t        lock;
    } amem;
#if ATH_DEBUG
    unsigned long rtsctsenable;
#endif
    /*
     * handle for code that uses the osdep.h version of OS
     * abstraction primitives
     */
    osdev_t         sc_osdev;

    /**
     * call back set by the os shim
     */
    ol_ath_update_fw_config_cb cfg_cb;

    /*
     * handle for code that uses adf version of OS
     * abstraction primitives
     */
    adf_os_device_t   adf_dev;

    struct ath_version      version;

    /* Packet statistics */
    struct ol_ath_stats     pkt_stats;

    u_int32_t target_type;  /* A_TARGET_TYPE_* */
    u_int32_t target_version;
    ol_target_status  target_status; /* target status */
    bool             is_sim;   /* is this a simulator */
    u_int8_t *cal_in_flash; /* calibration data is stored in flash */
    void *cal_mem; /* virtual address for the calibration data on the flash */

    bool                wmi_ready;
    WLAN_INIT_STATUS    wlan_init_status; /* status of target init */

    /* BMI info */
    void            *bmi_ol_priv; /* OS-dependent private info for BMI */
    bool            bmiDone;
    bool            bmiUADone;
    u_int8_t        *pBMICmdBuf;
    dma_addr_t      BMICmd_pa;
    OS_DMA_MEM_CONTEXT(bmicmd_dmacontext)

    u_int8_t        *pBMIRspBuf;
    dma_addr_t      BMIRsp_pa;
    u_int32_t       last_rxlen; /* length of last response */
    OS_DMA_MEM_CONTEXT(bmirsp_dmacontext)

    void            *diag_ol_priv; /* OS-dependent private info for DIAG access */

    /* Handles for Lower Layers : filled in at init time */
    hif_handle_t            hif_hdl;
    hif_softc_t             *hif_sc;

    uint8_t                 device_index;       // arDeviceIndex from toba.

    /* HTC handles */
    void                    *htc_handle;
    wmi_unified_t           wmi_handle;
    uint8_t                 ps_report;
#if QCA_OL_11AC_FAST_PATH
    htt_pdev_handle         htt_pdev;
#endif /* QCA_OL_11AC_FAST_PATH */

    /* ol data path handle */
    ol_txrx_pdev_handle     pdev_txrx_handle;

    /* target resource config */
    wmi_resource_config     wlan_resource_config;

    /* UMAC callback functions */
    void                    (*net80211_node_cleanup)(struct ieee80211_node *);
    void                    (*net80211_node_free)(struct ieee80211_node *);

    void		    (*pci_reconnect)(struct ol_ath_softc_net80211 *);

    A_UINT32                phy_capability; /* PHY Capability from Target*/
	A_UINT32                max_frag_entry; /* Max number of Fragment entry */
    A_UINT32                wmi_service_bitmap[WMI_SERVICE_BM_SIZE]; /* wmi services bitmap received from Target */

    /* UTF event information */
    struct {
        u_int8_t            *data;
        u_int32_t           length;
        u_int16_t           offset;
        u_int8_t            currentSeq;
        u_int8_t            expectedSeq;
    } utf_event_info;

    struct ol_wow_info      *scn_wowInfo;

#if PERE_IP_HDR_ALIGNMENT_WAR
    bool                    host_80211_enable; /* Enables native-wifi mode on host */
#endif
    bool                    enableuartprint;    /* enable uart/serial prints from target */
#if defined(EPPING_TEST) && !defined(HIF_USB)
    /* for mboxping */
    HTC_ENDPOINT_ID         EppingEndpoint[4];
    adf_os_spinlock_t       data_lock;
    struct sk_buff_head     epping_nodrop_queue;
    struct timer_list       epping_timer;
    bool                    epping_timer_running;
#endif
    u_int8_t                is_target_paused;
    HAL_REG_CAPABILITIES hal_reg_capabilities;
    struct ol_regdmn *ol_regdmn_handle;
    u_int8_t                bcn_mode;
    u_int8_t                burst_enable;
    u_int16_t               burst_dur;
    u_int8_t                arp_override;
    u_int8_t                igmpmld_override;
    u_int8_t                igmpmld_tid;
    struct ieee80211_mib_cycle_cnts  mib_cycle_cnts;  /* used for channel utilization for ol model */
    /*
     * Includes host side stack level stats +
     * radio level athstats
     */
    struct wal_dbg_stats	            ath_stats;

    /* This structure is used to update the radio level stats, the stats
        are directly fetched from the descriptors
    */
    struct ol_ath_radiostats     scn_stats;
    struct ieee80211_chan_stats chan_stats;     /* Used for channel radio-level stats */
    int16_t               chan_nf;            /* noise_floor */
    u_int32_t               min_tx_power;
    u_int32_t               max_tx_power;
    u_int32_t               txpowlimit2G;
    u_int32_t               txpowlimit5G;
    u_int32_t               txpower_scale;
    u_int32_t               chan_tx_pwr;
    u_int32_t               vdev_count;
    u_int32_t               peer_count;
    adf_os_spinlock_t       scn_lock;

    u_int32_t               vow_config;

    u_int8_t                vow_extstats;
    u_int32_t               max_peers;
    u_int32_t               max_vdevs;

    /** DCS configuration and running state */
	wlan_host_dcs_params_t   scn_dcs;
    wdi_event_subscribe     scn_rx_peer_invalid_subscriber;
    u_int8_t                dtcs; /* Dynamic Tx Chainmask Selection enabled/disabled */
    u_int32_t               set_ht_vht_ies:1, /* true if vht ies are set on target */
                            sc_in_delete:1;   /* don't add any more VAPs */
#if PERF_FIND_WDS_NODE
    struct wds_table        scn_wds_table;
#endif
    u_int32_t               num_mem_chunks;
    struct ol_ath_mem_chunk mem_chunks[MAX_MEM_CHUNKS];
    bool                    scn_cwmenable;    /*CWM enable/disable state*/
    bool                    is_ani_enable;    /*ANI enable/diable state*/
#if ATH_SUPPORT_HYFI_ENHANCEMENTS
	struct ol_ath_ald_record  buff_thresh;
#endif
#if ATH_RX_LOOPLIMIT_TIMER
    adf_os_timer_t          rx_looplimit_timer;
    u_int32_t               rx_looplimit_timeout;        /* timeout intval */
    bool                    rx_looplimit_valid;
    bool                    rx_looplimit;
#endif
#if ATH_SUPPORT_SPECTRAL
    u_int8_t                scn_icm_active;  /* Whether Intelligent Channel Manager
                                                is active. */
#endif
    u_int32_t               peer_ext_stats_count;
#if ATH_SUPPORT_WRAP
    int                     sc_nwrapvaps; /* # of WRAP vaps */
    int                     sc_npstavaps; /* # of ProxySTA vaps */
    int                     sc_nscanpsta; /* # of scan-able non-Main ProxySTA vaps */
    struct ieee80211vap     *sc_mcast_recv_vap; /* the ProxySTA vap to receive multicast frames */
#endif
    int                     sc_chan_freq;           /* channel change freq in mhz */
    int                     sc_chan_band_center_f1; /* channel change band center freq in mhz */
    int                     sc_chan_band_center_f2; /* channel change band center freq in mhz */
    int                     sc_chan_phy_mode;    /* channel change PHY mode */
    int                     recovery_enable;	 /* enable/disable target recovery feature */
    //For Mcast
    u_int16_t               pend_desc_removal;
    u_int16_t               pend_desc_addition;
#if OL_ATH_SUPPORT_LED
    os_timer_t                      scn_led_blink_timer;     /* led blinking timer */
    os_timer_t                      scn_led_poll_timer;     /* led polling timer */
    OL_BLINK_STATE                scn_blinking; /* LED blink operation active */
    u_int32_t               scn_led_time_on;  /* LED ON time for current blink in ms */
    u_int32_t               scn_led_byte_cnt;
    u_int32_t               scn_led_last_time;
    u_int32_t               scn_led_max_blink_rate_idx;
    u_int8_t                scn_led_gpio;
    const OL_LED_BLINK_RATES  *scn_led_blink_rate_table;   /* blinking rate table to be used */
#endif
    u_int32_t               sa_validate_sw; /* validate Smart Antenna Software */
    struct ol_ath_cookie    scn_cookie; /*To manage WMI communication with Target per radio basis */
    u_int32_t               ol_rts_cts_rate;
#if DBGLOG_WQ_BASED
    struct dbglog_workqueue_ctx dbglog_ctx;
#endif
    int                     sc_nstavaps;
    bool                    sc_is_blockdfs_set;
#if WDI_EVENT_ENABLE
    u_int32_t               scn_last_peer_invalid_time;  /*Time interval since last invalid was sent */
    u_int32_t               scn_peer_invalid_cnt;      /* number of permissible dauth in interval */
    u_int32_t               scn_user_peer_invalid_cnt; /* configurable by user  */
#endif
    u_int32_t               fwsuspendfailed;
#if UNIFIED_SMARTANTENNA
    wdi_event_subscribe sa_event_sub;
#endif
};
#define OL_ATH_DCS_ENABLE(__arg1, val) ((__arg1) |= (val))
#define OL_ATH_DCS_DISABLE(__arg1, val) ((__arg1) &= ~(val))
#define OL_ATH_DCS_SET_RUNSTATE(__arg1) ((__arg1) |= 0x10)
#define OL_ATH_DCS_CLR_RUNSTATE(__arg1) ((__arg1) &= ~0x10)
#define OL_IS_DCS_ENABLED(__arg1) ((__arg1) & 0x0f)
#define OL_IS_DCS_RUNNING(__arg1) ((__arg1) & 0x10)
#define OL_ATH_CAP_DCS_CWIM     0x1
#define OL_ATH_CAP_DCS_WLANIM   0x2
#define OL_ATH_CAP_DCS_MASK  (OL_ATH_CAP_DCS_CWIM | OL_ATH_CAP_DCS_WLANIM)

#if PERE_IP_HDR_ALIGNMENT_WAR
#define ol_scn_host_80211_enable_get(_ol_pdev_hdl) \
    ((struct ol_ath_softc_net80211 *)(_ol_pdev_hdl))->host_80211_enable
#endif

#define OL_ATH_SOFTC_NET80211(_ic)     ((struct ol_ath_softc_net80211 *)(_ic))

struct bcn_buf_entry {
    A_BOOL                        is_dma_mapped;
    wbuf_t                        bcn_buf;
    TAILQ_ENTRY(bcn_buf_entry)    deferred_bcn_list_elem;
};

struct ol_ath_vap_net80211 {
    struct ieee80211vap             av_vap;     /* NB: base class, must be first */
    struct ol_ath_softc_net80211    *av_sc;     /* back pointer to softc */
    ol_txrx_vdev_handle             av_txrx_handle;    /* ol data path handle */
    int                             av_if_id;   /* interface id */
    u_int64_t                       av_tsfadjust;       /* Adjusted TSF, host endian */
    bool                            av_beacon_offload;  /* Handle beacons in FW */
    wbuf_t                          av_wbuf;            /* Beacon buffer */
    A_BOOL                          is_dma_mapped;
    struct ieee80211_bcn_prb_info   av_bcn_prb_templ;   /* Beacon probe template */
    struct ieee80211_beacon_offsets av_beacon_offsets;  /* beacon fields offsets */
    os_timer_t                      av_timer;
    bool                            av_ol_resmgr_wait;  /* UMAC waits for target */
                                                        /*   event to bringup vap*/
    adf_os_spinlock_t               avn_lock;
    TAILQ_HEAD(, bcn_buf_entry)     deferred_bcn_list;  /* List of deferred bcn buffers */
    struct ieee80211_channel        *av_ol_resmgr_chan;  /* Channel ptr configured in the target*/
#if ATH_SUPPORT_WRAP
    u_int32_t                   av_is_psta:1,   /* is ProxySTA VAP */
                                av_is_mpsta:1,  /* is Main ProxySTA VAP */
                                av_is_wrap:1,   /* is WRAP VAP */
                                av_use_mat:1;   /* use MAT for this VAP */
    u_int8_t                    av_mat_addr[IEEE80211_ADDR_LEN];    /* MAT addr */
#endif
};
#define OL_ATH_VAP_NET80211(_vap)      ((struct ol_ath_vap_net80211 *)(_vap))

struct ol_ath_node_net80211 {
    struct ieee80211_node       an_node;     /* NB: base class, must be first */
    ol_txrx_peer_handle         an_txrx_handle;    /* ol data path handle */
    u_int32_t                   an_ni_rx_rate;
    u_int32_t                   an_ni_tx_rate;
#if ATH_SUPPORT_HYFI_ENHANCEMENTS
	u_int32_t					an_tx_rates_used;
	u_int32_t					an_tx_cnt;
	u_int32_t					an_phy_err_cnt;
	u_int32_t					an_tx_bytes;
	u_int32_t					an_tx_ratecount;
#endif
};

#define OL_ATH_NODE_NET80211(_ni)      ((struct ol_ath_node_net80211 *)(_ni))

 void ol_target_failure(void *instance, A_STATUS status);

int ol_ath_attach(u_int16_t devid, struct ol_ath_softc_net80211 *scn, IEEE80211_REG_PARAMETERS *ieee80211_conf_parm, ol_ath_update_fw_config_cb cb);

int ol_asf_adf_attach(struct ol_ath_softc_net80211 *scn);

void ol_ath_target_status_update(struct ol_ath_softc_net80211 *scn, ol_target_status status);


int ol_ath_detach(struct ol_ath_softc_net80211 *scn, int force);

void ol_ath_utf_detach(struct ol_ath_softc_net80211 *scn);
#ifdef QVIT
void ol_ath_qvit_detach(struct ol_ath_softc_net80211 *scn);
void ol_ath_qvit_attach(struct ol_ath_softc_net80211 *scn);
#endif

void ol_ath_suspend_resume_attach(struct ol_ath_softc_net80211 *scn);

int ol_ath_resume(struct ol_ath_softc_net80211 *scn);

int ol_ath_suspend(struct ol_ath_softc_net80211 *scn);

void ol_ath_vap_attach(struct ieee80211com *ic);

int ol_ath_cwm_attach(struct ol_ath_softc_net80211 *scn);

struct ieee80211vap *ol_ath_vap_get(struct ol_ath_softc_net80211 *scn, u_int8_t vdev_id);

u_int8_t *ol_ath_vap_get_myaddr(struct ol_ath_softc_net80211 *scn, u_int8_t vdev_id);

void ol_ath_beacon_attach(struct ieee80211com *ic);

void ol_ath_node_attach(struct ol_ath_softc_net80211 *scn, struct ieee80211com *ic);

void ol_ath_resmgr_attach(struct ieee80211com *ic);

void ol_ath_scan_attach(struct ieee80211com *ic);

void ol_ath_rtt_meas_report_attach(struct ieee80211com *ic);

void ol_ath_power_attach(struct ieee80211com *ic);

int ol_scan_update_channel_list(ieee80211_scanner_t ss);

struct ieee80211_channel *
ol_ath_find_full_channel(struct ieee80211com *ic, u_int32_t freq);

void ol_ath_utf_attach(struct ol_ath_softc_net80211 *scn);

int ol_ath_vap_send_data(struct ieee80211vap *vap, wbuf_t wbuf);

void ol_ath_vap_send_hdr_complete(void *ctx, HTC_PACKET_QUEUE *htc_pkt_list);


void ol_rx_indicate(void *ctx, wbuf_t wbuf);

void ol_rx_handler(void *ctx, HTC_PACKET *htc_packet);

void ol_ath_mgmt_attach(struct ieee80211com *ic);

int ol_ath_tx_mgmt_send(struct ieee80211com *ic, wbuf_t wbuf);

void ol_ath_beacon_alloc(struct ieee80211com *ic, int if_id);

void ol_ath_beacon_stop(struct ol_ath_softc_net80211 *scn,
                   struct ol_ath_vap_net80211 *avn);

void ol_ath_beacon_free(struct ieee80211com *ic, int if_id);

void ol_ath_net80211_newassoc(struct ieee80211_node *ni, int isnew);

void ol_ath_phyerr_attach(struct ieee80211com *ic);
void ol_ath_phyerr_detach(struct ieee80211com *ic);
void ol_ath_phyerr_enable(struct ieee80211com *ic);
void ol_ath_phyerr_disable(struct ieee80211com *ic);

int
ol_transfer_target_eeprom_caldata(struct ol_ath_softc_net80211 *scn, u_int32_t address, bool compressed);

int
ol_transfer_bin_file(struct ol_ath_softc_net80211 *scn, ATH_BIN_FILE file,
                    u_int32_t address, bool compressed);

int
__ol_ath_check_wmi_ready(struct ol_ath_softc_net80211 *scn);

void
__ol_ath_wmi_ready_event(struct ol_ath_softc_net80211 *scn);

void __ol_target_paused_event(struct ol_ath_softc_net80211 *scn);

u_int32_t host_interest_item_address(u_int32_t target_type, u_int32_t item_offset);

int
ol_ath_set_config_param(struct ol_ath_softc_net80211 *scn, ol_ath_param_t param, void *buff);

int
ol_ath_get_config_param(struct ol_ath_softc_net80211 *scn, ol_ath_param_t param, void *buff);

int
ol_hal_set_config_param(struct ol_ath_softc_net80211 *scn, ol_hal_param_t param, void *buff);

int
ol_hal_get_config_param(struct ol_ath_softc_net80211 *scn, ol_hal_param_t param, void *buff);

void
ol_ath_set_vap_nss_wifi_func(void *arg , struct ieee80211vap *vap);

void
ol_ath_set_vap_nss_wifi(struct ieee80211com *ic, bool nss_nwifi);


unsigned int ol_ath_bmi_user_agent_init(struct ol_ath_softc_net80211 *scn);
int ol_ath_wait_for_bmi_user_agent(struct ol_ath_softc_net80211 *scn);
void ol_ath_signal_bmi_user_agent_done(struct ol_ath_softc_net80211 *scn);

void ol_ath_diag_user_agent_init(struct ol_ath_softc_net80211 *scn);
void ol_ath_diag_user_agent_fini(struct ol_ath_softc_net80211 *scn);
void ol_ath_host_config_update(struct ol_ath_softc_net80211 *scn);

void ol_ath_suspend_resume_attach(struct ol_ath_softc_net80211 *scn);
int ol_ath_suspend_target(struct ol_ath_softc_net80211 *scn, int disable_target_intr);
int ol_ath_resume_target(struct ol_ath_softc_net80211 *scn);
u_int ol_ath_mhz2ieee(struct ieee80211com *ic, u_int freq, u_int flags);
void ol_ath_set_ht_vht_ies(struct ieee80211_node *ni);

int wmi_unified_set_ap_ps_param(struct ol_ath_vap_net80211 *avn,
        struct ol_ath_node_net80211 *anode, A_UINT32 param, A_UINT32 value);
int wmi_unified_set_sta_ps_param(struct ol_ath_vap_net80211 *avn,
        A_UINT32 param, A_UINT32 value);
int wmi_unified_vdev_set_param_send(wmi_unified_t wmi_handle, u_int8_t if_id,
                           u_int32_t param_id, u_int32_t param_value);
int wmi_unified_pdev_get_tpc_config(wmi_unified_t wmi_handle, u_int32_t param);
int wmi_unified_pdev_set_fw_hang(wmi_unified_t wmi_handle, u_int32_t delay_time_ms);

int ol_ath_set_beacon_filter(wlan_if_t vap, u_int32_t *ie);
int ol_ath_remove_beacon_filter(wlan_if_t vap);
void ol_get_wal_dbg_stats(struct ol_ath_softc_net80211 *scn, struct wal_dbg_stats *dbg_stats);
int
ol_get_tx_free_desc(struct ol_ath_softc_net80211 *scn);
void ol_get_radio_stats(struct ol_ath_softc_net80211 *scn, struct ol_ath_radiostats *stats);
int
wmi_send_node_rate_sched(struct ol_ath_softc_net80211 *scn,
        wmi_peer_rate_retry_sched_cmd *cmd_buf);

int
wmi_unified_peer_flush_tids_send(wmi_unified_t wmi_handle,
                                 u_int8_t peer_addr[IEEE80211_ADDR_LEN],
                                 u_int32_t peer_tid_bitmap,
                                 u_int32_t vdev_id);
int
wmi_unified_node_set_param(wmi_unified_t wmi_handle, u_int8_t *peer_addr,u_int32_t param_id,
        u_int32_t param_val,u_int32_t vdev_id);
#if UNIFIED_SMARTANTENNA
int ol_ath_smart_ant_rxfeedback(struct ol_txrx_pdev_t *pdev, struct ol_txrx_peer_t *peer, struct sa_rx_feedback *rx_feedback);
int ol_smart_ant_enabled(struct ol_ath_softc_net80211 *scn);
int ol_smart_ant_rx_feedback_enabled(struct ol_ath_softc_net80211 *scn);
#endif /* UNIFIED_SMARTANTENNA */

int
wmi_unified_gpio_config(wmi_unified_t wmi_handle, u_int32_t gpio_num, u_int32_t input,
                        u_int32_t pull_type, u_int32_t intr_mode);
int
wmi_unified_gpio_output(wmi_unified_t wmi_handle, u_int32_t gpio_num, u_int32_t set);

#if OL_ATH_SUPPORT_LED
void
ol_ath_led_event(struct ol_ath_softc_net80211 *scn, OL_LED_EVENT event);
#endif

#ifdef BIG_ENDIAN_HOST
     /* This API is used in copying in elements to WMI message,
        since WMI message uses multilpes of 4 bytes, This API
        converts length into multiples of 4 bytes, and performs copy
     */
#define OL_IF_MSG_COPY_CHAR_ARRAY(destp, srcp, len)  do { \
      int j; \
      u_int32_t *src, *dest; \
      src = (u_int32_t *)srcp; \
      dest = (u_int32_t *)destp; \
      for(j=0; j < roundup(len, sizeof(u_int32_t))/4; j++) { \
          *(dest+j) = adf_os_le32_to_cpu(*(src+j)); \
      } \
   } while(0)

/* This macro will not work for anything other than a multiple of 4 bytes */
#define OL_IF_SWAPBO(x, len)  do { \
      int numWords; \
      int i; \
      void *pv = &(x); \
      u_int32_t *wordPtr; \
      numWords = (len)/sizeof(u_int32_t); \
      wordPtr = (u_int32_t *)pv; \
      for (i = 0; i < numWords; i++) { \
          *(wordPtr + i) = __cpu_to_le32(*(wordPtr + i)); \
      } \
   } while(0)

#define OUI_ENTRY(x) (x)

#else

#define OL_IF_MSG_COPY_CHAR_ARRAY(destp, srcp, len)  do { \
    OS_MEMCPY(destp, srcp, len); \
   } while(0)

#define SWAP32(x) (((x << 24) & 0xff000000) | \
                   ((x << 8)  & 0x00ff0000) | \
                   ((x >> 8)  & 0x0000ff00) | \
                   ((x >> 24) & 0x000000ff))
#define OUI_ENTRY(x) (SWAP32(x))
#endif
#define AR9887_DEVICE_ID    (0x0050)
#endif /* _DEV_OL_ATH_ATHVAR_H  */
