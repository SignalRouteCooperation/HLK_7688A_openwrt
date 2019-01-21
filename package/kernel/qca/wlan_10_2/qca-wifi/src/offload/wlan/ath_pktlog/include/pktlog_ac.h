/*
 * Copyright (c) 2012 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 */
#ifndef _PKTLOG_AC_H_
#define _PKTLOG_AC_H_
#ifndef REMOVE_PKT_LOG

#include "ol_if_athvar.h"
#include <pktlog_ac_api.h>
/* Include pktlog_ac_fmt.h below ol_if_athvar.h to ensure MacOS driver compiles.
 * TODO: Fix __ATTRIB_PACK defination in this file, to remove above requirement.
 */
#include <pktlog_ac_fmt.h>  
#include "osdep.h"
#include <wmi_unified.h>
#include <wmi_unified_api.h>
#include <wdi_event_api.h>

#define NO_REG_FUNCS 4

/* Locking interface for pktlog */
#define PKTLOG_LOCK_INIT(_pl_info)    spin_lock_init(&(_pl_info)->log_lock)
#define	PKTLOG_LOCK_DESTROY(_pl_info)
#define PKTLOG_LOCK(_pl_info)         spin_lock(&(_pl_info)->log_lock)
#define PKTLOG_UNLOCK(_pl_info)       spin_unlock(&(_pl_info)->log_lock)

#define PKTLOG_MODE_SYSTEM   1
#define PKTLOG_MODE_ADAPTER  2

/* Opaque softc */
struct ol_ath_generic_softc_t;
typedef struct ol_ath_generic_softc_t* ol_ath_generic_softc_handle;
extern void pktlog_disable_adapter_logging(void);
extern int pktlog_alloc_buf(struct ol_ath_softc_net80211 *scn);
extern void pktlog_release_buf(struct ol_ath_softc_net80211 *scn);
/*extern void pktlog_disable_adapter_logging(void);
extern int pktlog_alloc_buf(ol_ath_generic_softc_handle sc,
                                struct ath_pktlog_info *pl_info);
extern void pktlog_release_buf(struct ath_pktlog_info *pl_info);*/

struct ol_pl_arch_dep_funcs {
    void (*pktlog_init)(struct ol_ath_softc_net80211 *scn);
    int (*pktlog_enable)(struct ol_ath_softc_net80211 *scn,
            int32_t log_state);
    int (*pktlog_setsize)(struct ol_ath_softc_net80211 *scn,
            int32_t log_state);
    int (*pktlog_disable)(struct ol_ath_softc_net80211 *scn);
};

struct ol_pl_os_dep_funcs{
    int (*pktlog_attach) (struct ol_ath_softc_net80211 *scn);
    void (*pktlog_detach) (struct ol_ath_softc_net80211 *scn);
};

extern struct ol_pl_arch_dep_funcs ol_pl_funcs; 
extern struct ol_pl_os_dep_funcs *g_ol_pl_os_dep_funcs;

/* Pktlog handler to save the state of the pktlogs */
struct ol_pktlog_dev_t
{
    struct ol_pl_arch_dep_funcs *pl_funcs;
    struct ath_pktlog_info *pl_info;  
    ol_ath_generic_softc_handle scn;
    char *name;
    bool tgt_pktlog_enabled;
    osdev_t sc_osdev;
};

extern struct ol_pktlog_dev_t ol_pl_dev;

/*
 * pktlog WMI command
 * Selective modules enable TBD
 * TX/ RX/ RCF/ RCU
 */
   
int pktlog_wmi_send_cmd(struct ol_ath_softc_net80211 *scn,
                WMI_PKTLOG_EVENT PKTLOG_EVENT, WMI_CMD_ID CMD_ID);

extern int ol_smart_ant_enabled(struct ol_ath_softc_net80211 *scn);
#define PKTLOG_DISABLE(_scn) \
    pktlog_wmi_send_cmd(_scn, 0, WMI_PDEV_PKTLOG_DISABLE_CMDID)
 
/*
 * WDI related data and functions
 * Callback function to the WDI events
 */
void pktlog_callback(void *pdev, enum WDI_EVENT event, void *log_data, u_int16_t peer_id, enum htt_rx_status status);

#define ol_pktlog_attach(_scn)                          \
    do {                                                    \
        if (g_ol_pl_os_dep_funcs) {                             \
            g_ol_pl_os_dep_funcs->pktlog_attach(_scn);              \
        }                                                       \
    } while(0)

#define ol_pktlog_detach(_scn)                          \
    do {                                                    \
        if (g_ol_pl_os_dep_funcs) {                             \
            g_ol_pl_os_dep_funcs->pktlog_detach(_scn);              \
        }                                                       \
    } while(0)

#else /* REMOVE_PKT_LOG */
#define ol_pktlog_attach(_scn)    /* */
#define ol_pktlog_detach(_scn)    /* */
#endif /* REMOVE_PKT_LOG */
#endif /* _PKTLOG_AC_H_ */
