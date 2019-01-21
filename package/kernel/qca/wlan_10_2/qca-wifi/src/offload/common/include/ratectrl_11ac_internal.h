/*
 * Copyright (c) 2011-2012 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 * $ATH_LICENSE_TARGET_C$
 */

#ifndef _RATECTRL_INTERNAL_H_
#define _RATECTRL_INTERNAL_H_

#include "ratectrl_11ac.h" /*TODO: Cleanup */

/*
    REVISIT
    Currently disable META_SUPPORT
*/
#define DISABLE_META_SUPPORT 1


enum defMCSMaskType {
    MODE_STBC,
    MODE_NO_STBC,
} ;

#ifndef DISABLE_FUNCTION_INDIRECTION
typedef struct {
    A_UINT16 (*_rcRateFind)(RATE_CONTEXT *g_pRATE, struct rate_node *pSib,
        A_UINT32 frameLen,A_BOOL isLowest, RC_MASK_INFO_t *rc_mask_info,
        A_UINT8 *is_probe, A_UINT32 nowMsec);
#if RC_SUPPORT_1D_RATECTRL
    A_UINT16 (*_rcRateFind_1D)(RATE_CONTEXT *g_pRATE, struct rate_node *pSib,
        A_UINT32 frameLen,A_BOOL isLowest, RC_MASK_INFO_t *rc_mask_info,
        A_UINT8 *is_probe, A_UINT32 nowMsec);
    A_UINT16 (*_rcRateFind_2D)(RATE_CONTEXT *g_pRATE, struct rate_node *pSib,
        A_UINT32 frameLen,A_BOOL isLowest, RC_MASK_INFO_t *rc_mask_info,
        A_UINT8 *is_probe, A_UINT32 nowMsec);
#endif
    void (*_rcUpdate_HT)(RATE_CONTEXT *g_pRATE, struct rate_node *pSib,
            RC_TX_RATE_SCHEDULE *rate_sched, RC_TX_DONE_PARAMS * args,
            A_UINT8 probe_aborted); /* frame was a unicast data packet */
#if RC_SUPPORT_1D_RATECTRL
    void (*_rcUpdate_HT_1D)(RATE_CONTEXT *g_pRATE, struct rate_node *pSib,
            RC_TX_RATE_SCHEDULE *rate_sched, RC_TX_DONE_PARAMS * args,
            A_UINT8 probe_aborted); /* frame was a unicast data packet */
    void (*_rcUpdate_HT_2D)(RATE_CONTEXT *g_pRATE, struct rate_node *pSib,
            RC_TX_RATE_SCHEDULE *rate_sched, RC_TX_DONE_PARAMS * args,
            A_UINT8 probe_aborted); /* frame was a unicast data packet */
#endif
    void (*_rcSibUpdate)(RATE_CONTEXT * g_pRATE, struct rate_node *pSib, void *peer);
    void (*_GetRateTblAndIndex)(RATE_CONTEXT *g_pRATE, A_UINT8 *is_probe,
        A_BOOL shortPreamble, struct rate_node *an, A_UINT8 * txrate, 
        const WHAL_RATE_TABLE ** rt,A_UINT8 * rix, A_BOOL isLowest, 
        RC_MASK_INFO_t *rc_mask, A_UINT32 nowMsec);
    A_BOOL (*_RateValidCheck)(const WHAL_RATE_TABLE *pRateTable, A_UINT16 rix, struct rate_node *rc_node);
    A_BOOL (*_rcGetNextValidTxRate)(const WHAL_RATE_TABLE *pRateTable, struct TxRateCtrl_s *pRc,
        RC_MASK_INFO_t *rc_mask_info, A_UINT8 curValidTxRate, 
        A_UINT8 *pNextIndex);
    A_UINT8 (*_rcGetNextLowerValidTxRate)(const WHAL_RATE_TABLE *pRateTable, RC_MASK_INFO_t *rc_mask_info,
           struct TxRateCtrl_s  *pRc, A_UINT8 curValidTxRate, 
           A_UINT8 *pNextRateIndex);
    A_BOOL (*_rcGetLowerValid)(const WHAL_RATE_TABLE *pRateTable, 
        A_UINT8 index, struct TxRateCtrl_s  *pRc,
        RC_MASK_INFO_t *rc_mask_info, A_UINT8 *pNextIndex, A_UINT8 nss_idx);
    A_BOOL (*_RateValidCheckNodeFlags)(struct rate_node *rc_node, const WHAL_RATE_TABLE *pRateTable, A_UINT16 rix);
    void (*_rcSortValidRates)(const WHAL_RATE_TABLE *pRateTable, A_UINT8 *pArray, int array_len);
    A_UINT8 (*_GetLSB)(A_RATEMASK mask);
    A_UINT8 (*_GetMSB)(A_RATEMASK mask);
    void (*_rate_setup)(RATE_CONTEXT *g_pRATE, WLAN_PHY_MODE mode);
    A_UINT8 (*_GetBestRate)(RATE_CONTEXT *g_pRATE, RC_MASK_INFO_t *rc_mask_info,
        const WHAL_RATE_TABLE *pRateTable, struct TxRateCtrl_s  *pRc, 
        A_RSSI rssiLast, A_UINT8 forced_max_rix);
    A_UINT8 (*_GetBestRate_2D)(RATE_CONTEXT *g_pRATE, RC_MASK_INFO_t *rc_mask_info,
            const WHAL_RATE_TABLE *pRateTable, struct TxRateCtrl_s  *pRc, A_UINT8 bw,
            A_UINT8 pastRate);
    A_BOOL (*_RateCheckDefaultMCSMask)(struct rate_node *rc_node, A_UINT8 mcs_idx);
    void (*_RateValidateRateSet)(struct rate_node *rc_node);
#ifdef SMART_ANTENNA
    void (*_RateFillValidRatecodeList)(struct rate_node *pSib, const WHAL_RATE_TABLE *pRateTable, RATE_CONTEXT * g_pRATE, wmi_peer_rate_info_t *ratecode_list);
    A_UINT32 (*_RATEGetRateSchedFlags)(struct rate_node *pSib, const WHAL_RATE_TABLE *pRateTable, RATE_CONTEXT *g_pRATE, A_UINT32 rix);
#endif    
    A_BOOL (*_rcGetValidTxRateMaxMcs)(const WHAL_RATE_TABLE *pRateTable,
        struct TxRateCtrl_s *pRc, RC_MASK_INFO_t *rc_mask_info,
        A_UINT8 nss_idx, A_UINT8 maxMcs, A_UINT8 *pFoundIndex);
    void (*_rcUpdatePer_2D)(const WHAL_RATE_TABLE *pRateTable,
        struct TxRateCtrl_s *pRc, A_UINT8 txRate, A_UINT8 m_idx, A_UINT8 newPer);
}RATE_INTERNAL_INDIRECTION_TABLE;

extern RATE_INTERNAL_INDIRECTION_TABLE rateInternalIndirectionTable;
#define RATE_INTERNAL_FN(fn) rateInternalIndirectionTable.fn
#else /*DISABLE_FUNCTION_INDIRECTION*/
A_UINT16 _rcRateFind(RATE_CONTEXT *g_pRATE, struct rate_node *pSib,
    A_UINT32 frameLen,A_BOOL isLowest, RC_MASK_INFO_t *rc_mask_info,
    A_UINT8 *is_probe, A_UINT32 nowMsec);
#if RC_SUPPORT_1D_RATECTRL
A_UINT16 _rcRateFind_1D(RATE_CONTEXT *g_pRATE, struct rate_node *pSib,
    A_UINT32 frameLen,A_BOOL isLowest, RC_MASK_INFO_t *rc_mask_info,
    A_UINT8 *is_probe, A_UINT32 nowMsec);
A_UINT16 _rcRateFind_2D(RATE_CONTEXT *g_pRATE, struct rate_node *pSib,
    A_UINT32 frameLen,A_BOOL isLowest, RC_MASK_INFO_t *rc_mask_info,
    A_UINT8 *is_probe, A_UINT32 nowMsec);
#endif
void _rcUpdate_HT(RATE_CONTEXT *g_pRATE, struct rate_node *pSib,
        RC_TX_RATE_SCHEDULE *rate_sched, RC_TX_DONE_PARAMS * args,
        A_UINT8 probe_aborted); /* frame was a unicast data packet */
#if RC_SUPPORT_1D_RATECTRL
void _rcUpdate_HT_1D(RATE_CONTEXT *g_pRATE, struct rate_node *pSib,
        RC_TX_RATE_SCHEDULE *rate_sched, RC_TX_DONE_PARAMS * args,
        A_UINT8 probe_aborted); /* frame was a unicast data packet */
void _rcUpdate_HT_2D(RATE_CONTEXT *g_pRATE, struct rate_node *pSib,
        RC_TX_RATE_SCHEDULE *rate_sched, RC_TX_DONE_PARAMS * args,
        A_UINT8 probe_aborted); /* frame was a unicast data packet */
#endif
A_BOOL      _RATE_GetTxRetrySchedule(void *pdev,
                  const WHAL_RATE_TABLE ** rt,
                  void *wlan_peer,
                  void *rc_args,
                  void *pkt_info_rcf);

void _rcSibUpdate(RATE_CONTEXT * g_pRATE, struct rate_node *pSib, void *peer);
void _GetRateTblAndIndex(RATE_CONTEXT *g_pRATE, A_UINT8 *is_probe,
    A_BOOL shortPreamble, struct rate_node *an, A_UINT8 * txrate, 
    const WHAL_RATE_TABLE ** rt,A_UINT8 * rix, A_BOOL isLowest, 
    RC_MASK_INFO_t *rc_mask, A_UINT32 nowMsec);
A_BOOL _RateValidCheck(const WHAL_RATE_TABLE *pRateTable, A_UINT16 rix, struct rate_node *rc_node);
A_BOOL _rcGetNextValidTxRate(const WHAL_RATE_TABLE *pRateTable, struct TxRateCtrl_s *pRc,
    RC_MASK_INFO_t *rc_mask_info, A_UINT8 curValidTxRate,
    A_UINT8 *pNextIndex);
A_UINT8 _rcGetNextLowerValidTxRate(const WHAL_RATE_TABLE *pRateTable, RC_MASK_INFO_t *rc_mask_info,
    struct TxRateCtrl_s  *pRc, A_UINT8 curValidTxRate, 
    A_UINT8 *pNextRateIndex);
A_BOOL _rcGetLowerValid(const WHAL_RATE_TABLE *pRateTable, A_UINT8 index,
    struct TxRateCtrl_s  *pRc, RC_MASK_INFO_t *rc_mask_info, 
    A_UINT8 *pNextIndex, A_UINT8 nss_idx);
A_BOOL _RateValidCheckNodeFlags(struct rate_node *rc_node, const WHAL_RATE_TABLE *pRateTable, A_UINT16 rix);
void _rcSortValidRates(const WHAL_RATE_TABLE *pRateTable, A_UINT8 *pArray, int array_len);
A_UINT8 _GetLSB(A_RATEMASK mask);
A_UINT8 _GetMSB(A_RATEMASK mask);
void _rate_setup(RATE_CONTEXT *g_pRATE, WLAN_PHY_MODE mode);
A_UINT8 _GetBestRate(RATE_CONTEXT *g_pRATE, RC_MASK_INFO_t *rc_mask_info,
     const WHAL_RATE_TABLE *pRateTable, struct TxRateCtrl_s  *pRc, 
     A_RSSI rssiLast, A_UINT8 forced_max_rix);
A_UINT8 _GetBestRate_2D(RATE_CONTEXT *g_pRATE, RC_MASK_INFO_t *rc_mask_info,
        const WHAL_RATE_TABLE *pRateTable, struct TxRateCtrl_s  *pRc, A_UINT8 bw,
        A_UINT8 pastRate);
A_BOOL _RateCheckDefaultMCSMask(struct rate_node *rc_node, A_UINT8 mcs_idx);
void _RateValidateRateSet(struct rate_node *rc_node);
#ifdef SMART_ANTENNA
void _RateFillValidRatecodeList(struct rate_node *pSib, const WHAL_RATE_TABLE *pRateTable, RATE_CONTEXT * g_pRATE, wmi_peer_rate_info_t *ratecode_list);
A_UINT32 _RATEGetRateSchedFlags(struct rate_node *pSib, const WHAL_RATE_TABLE *pRateTable, RATE_CONTEXT *g_pRATE, A_UINT32 rix);
#endif
A_BOOL _rcGetValidTxRateMaxMcs(const WHAL_RATE_TABLE *pRateTable,
    struct TxRateCtrl_s *pRc, RC_MASK_INFO_t *rc_mask_info, A_UINT8 nss_idx, A_UINT8 maxMcs,
    A_UINT8 *pFoundIndex);
void _rcUpdatePer_2D(const WHAL_RATE_TABLE *pRateTable, struct TxRateCtrl_s *pRc,
    A_UINT8 txRate, A_UINT8 m_idx, A_UINT8 newPer);

#define RATE_INTERNAL_FN(fn) fn
#endif /*DISABLE_FUNCTION_INDIRECTION*/

#define rcRateFind(g_pRATE, pSib, frameLen,isLowest,rateMask, is_probe, nowms) \
    RATE_INTERNAL_FN(_rcRateFind((g_pRATE), (pSib), (frameLen),(isLowest),(rateMask), (is_probe), (nowms)))

#if RC_SUPPORT_1D_RATECTRL

#define rcRateFind_1D(g_pRATE, pSib, frameLen,isLowest,rateMask, is_probe, nowms) \
    RATE_INTERNAL_FN(_rcRateFind_1D((g_pRATE), (pSib), (frameLen),(isLowest),(rateMask), (is_probe), (nowms)))

#define rcRateFind_2D(g_pRATE, pSib, frameLen,isLowest,rateMask, is_probe, nowms) \
    RATE_INTERNAL_FN(_rcRateFind_2D((g_pRATE), (pSib), (frameLen),(isLowest),(rateMask), (is_probe), (nowms)))

#endif

#define rcSibUpdate(g_pRATE, pSib, peer) \
    RATE_INTERNAL_FN(_rcSibUpdate((g_pRATE), (pSib), (peer)))

#define rcUpdate_HT(g_pRATE, pSib, rate_sched, args, probe_aborted) \
    RATE_INTERNAL_FN(_rcUpdate_HT((g_pRATE), (pSib), (rate_sched), (args), (probe_aborted)))

#if RC_SUPPORT_1D_RATECTRL

#define rcUpdate_HT_1D(g_pRATE, pSib, rate_sched, args, probe_aborted) \
    RATE_INTERNAL_FN(_rcUpdate_HT_1D((g_pRATE), (pSib), (rate_sched), (args), (probe_aborted)))

#define rcUpdate_HT_2D(g_pRATE, pSib, rate_sched, args, probe_aborted) \
    RATE_INTERNAL_FN(_rcUpdate_HT_2D((g_pRATE), (pSib), (rate_sched), (args), (probe_aborted)))

#endif

#define GetRateTblAndIndex(g_pRATE, prb,try0,an,txrate,rt,rix,isLowest,rateMask, nowms) \
            RATE_INTERNAL_FN(_GetRateTblAndIndex((g_pRATE), (prb),(try0),(an),(txrate),(rt),(rix),(isLowest),(rateMask), (nowms)))

#define RateValidCheck(an,rix,rc_node) \
        RATE_INTERNAL_FN(_RateValidCheck((an),(rix),(rc_node)))

#define rcGetNextValidTxRate(pRateTable, pRc, mask, curValidTxRate, pNextIndex) \
        RATE_INTERNAL_FN(_rcGetNextValidTxRate((pRateTable), (pRc), (mask), (curValidTxRate), (pNextIndex)))

#define rcGetNextLowerValidTxRate(pRateTable, mask, pRc, curValidTxRate, pNextRateIndex) \
        RATE_INTERNAL_FN(_rcGetNextLowerValidTxRate((pRateTable), (mask), (pRc), (curValidTxRate), (pNextRateIndex)))

#define rcGetLowerValid(pRateTable, index, pRc, mask, pNextIndex, nss_idx) \
        RATE_INTERNAL_FN(_rcGetLowerValid((pRateTable), (index), (pRc), (mask), (pNextIndex), (nss_idx)))

#define RateValidCheckNodeFlags(rc_node, pRateTable, rix) \
        RATE_INTERNAL_FN(_RateValidCheckNodeFlags((rc_node), (pRateTable), (rix)))

#define rcSortValidRates(pRateTable, pArray, array_len) \
        RATE_INTERNAL_FN(_rcSortValidRates((pRateTable), (pArray), (array_len)))

#define GetLSB(mask) \
        RATE_INTERNAL_FN(_GetLSB((mask)))

#define GetMSB(mask) \
        RATE_INTERNAL_FN(_GetMSB((mask)))

#define rate_setup(g_pRATE, mode) \
        RATE_INTERNAL_FN(_rate_setup((g_pRATE), (mode)))

#define GetBestRate(g_pRATE, vMask, pRateTable, pRc, rssiLast, forced_max_rix) \
        RATE_INTERNAL_FN(_GetBestRate((g_pRATE), (vMask), (pRateTable), (pRc), (rssiLast), (forced_max_rix)))

#define GetBestRate_2D(g_pRATE, vMask, pRateTable, pRc, bw, pastRate) \
        RATE_INTERNAL_FN(_GetBestRate_2D((g_pRATE), (vMask), (pRateTable), (pRc), (bw), (pastRate)))

#define RateCheckDefaultMCSMask(rc_node, mcs_idx) \
        RATE_INTERNAL_FN(_RateCheckDefaultMCSMask((rc_node), (mcs_idx)))

#define RateValidateRateSet(rc_node) \
        RATE_INTERNAL_FN(_RateValidateRateSet((rc_node)))

#define rcGetValidTxRateMaxMcs(pRateTable, pRc, rc_mask_info, nss_idx, maxMcs, pFoundIndex) \
        RATE_INTERNAL_FN(_rcGetValidTxRateMaxMcs((pRateTable), (pRc), (rc_mask_info), (nss_idx), (maxMcs), (pFoundIndex)))

#ifdef SMART_ANTENNA
#define RateFillValidRatecodeList(pSib, pRateTable, g_pRATE, ratecode_list) \
                RATE_INTERNAL_FN(_RateFillValidRatecodeList((pSib), (pRateTable), (g_pRATE), (ratecode_list)))
#define RATEGetRateSchedFlags(pSib, pRateTable, g_pRATE, rix) \
                        RATE_INTERNAL_FN(_RATEGetRateSchedFlags((pSib), (pRateTable), (g_pRATE), (rix)))
#endif

#define rcUpdatePer_2D(pRateTable, pRc, txRate, m_idx, newPer) \
        RATE_INTERNAL_FN(_rcUpdatePer_2D((pRateTable), (pRc), (txRate), (m_idx), (newPer)))

void rate_init_vdev_ratectxt(RATE_CONTEXT *g_pRATE, void *dev);
void rate_init_vdev_params(RATE_CONTEXT *g_pRATE);
void rate_init_peer_ratectxt(struct rate_node *pSib, void *peer);
void ratectrl_update_rate_sched(struct rate_node *an,
                                RC_TX_RATE_SCHEDULE *rate_sched,
                                void *pkt_info_rcf);

inline A_UINT8 rc_get_nss_idx_from_rix(const WHAL_RATE_TABLE *pRateTable, A_UINT8 rix);

#endif /* _RATECTRL_H_ */
