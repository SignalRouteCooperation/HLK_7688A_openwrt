/*
 * Copyright (c) 2011 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 */
#ifndef _HTT_TYPES__H_
#define _HTT_TYPES__H_

#include <osdep.h>        /* u_int16_t, dma_addr_t */
#include <adf_os_types.h> /* adf_os_device_t */
#include <adf_os_lock.h>  /* adf_os_spinlock_t */
#include <adf_os_timer.h> /* adf_os_timer_t */
#include <adf_os_atomic.h>/* adf_os_atomic_inc */
#include <adf_nbuf.h>     /* adf_nbuf_t */
#include <htc_api.h>      /* HTC_PACKET */

#include <ol_ctrl_api.h>  /* ol_pdev_handle */
#include <ol_txrx_api.h>  /* ol_txrx_pdev_handle */
#include <ol_htt_rx_api.h> 

#define HTT_TX_MUTEX_TYPE adf_os_spinlock_t

struct htt_pdev_t;

struct htt_htc_pkt {
    void *pdev_ctxt;
    dma_addr_t nbuf_paddr;
    HTC_PACKET htc_pkt;
    u_int16_t  msdu_id;
};

struct htt_htc_pkt_union {
    union {
        struct htt_htc_pkt pkt;
        struct htt_htc_pkt_union *next;
    } u;
};

/*
 * HTT host descriptor:
 * Include the htt_tx_msdu_desc that gets downloaded to the target,
 * but also include the HTC_FRAME_HDR and alignment padding that
 * precede the htt_tx_msdu_desc.
 * HTCSendDataPkt expects this header space at the front of the
 * initial fragment (i.e. tx descriptor) that is downloaded.
 */
struct htt_host_tx_desc_t {
    u_int8_t htc_header[HTC_HEADER_LEN];
    /* force the tx_desc field to begin on a 4-byte boundary */
    union {
        u_int32_t dummy_force_align;
        struct htt_tx_msdu_desc_t tx_desc;
    } align32;
};
struct htt_pdev_buf_entry{
    void* buf;
    u_int64_t cookie;
    void *htt_htc_packet;
    LIST_ENTRY(htt_pdev_buf_entry) buf_list;
};

struct htt_pdev_t {
    ol_pdev_handle ctrl_pdev;
    ol_txrx_pdev_handle txrx_pdev;
    HTC_HANDLE htc_pdev;
    adf_os_device_t osdev;
#if QCA_OL_TX_CACHEDHDR
    u_int32_t htt_hdr_cache[(HTC_HTT_TRANSFER_HDRSIZE + 3)/4];
#endif

    int download_len;

#if QCA_OL_11AC_FAST_PATH
    #define HTT_NBUF_ARRAY_SZ 32
    /*
     * TODO : change the static allocation of nbuf_arr
     */
    adf_nbuf_t nbuf_cmpl_arr[HTT_NBUF_ARRAY_SZ];
#endif /* QCA_OL_11AC_FAST_PATH */

    HTC_ENDPOINT_ID htc_endpoint;
#if ATH_11AC_TXCOMPACT
    HTT_TX_MUTEX_TYPE    txnbufq_mutex;
    adf_nbuf_queue_t     txnbufq;
#endif

    struct htt_htc_pkt_union *htt_htc_pkt_freelist;    
    struct {
        int is_high_latency;
    } cfg;
    struct {
        u_int8_t major;
        u_int8_t minor;
    } tgt_ver;
    struct {
        struct {
           /*
            * Ring of network buffer objects -
            * This ring is used exclusively by the host SW.
            * This ring mirrors the dev_addrs_ring that is shared
            * between the host SW and the MAC HW.
            * The host SW uses this netbufs ring to locate the network
            * buffer objects whose data buffers the HW has filled.
            */
           adf_nbuf_t *netbufs_ring;
           /*
            * Ring of buffer addresses -
            * This ring holds the "physical" device address of the
            * rx buffers the host SW provides for the MAC HW to fill.
            */
           u_int32_t *paddrs_ring;
           adf_os_dma_mem_context(memctx);
        } buf;
        /*
         * Base address of ring, as a "physical" device address rather than a
         * CPU address.
         */
        u_int32_t base_paddr;
        int size;           /* how many elems in the ring (power of 2) */
        unsigned size_mask; /* size - 1 */

        int fill_level; /* how many rx buffers to keep in the ring */
        int fill_cnt;   /* how many rx buffers (full+empty) are in the ring */

        /*
         * alloc_idx - where HTT SW has deposited empty buffers
         * This is allocated in consistent mem, so that the FW can read
         * this variable, and program the HW's FW_IDX reg with the value
         * of this shadow register.
         */
        struct {
            u_int32_t *vaddr;
            u_int32_t paddr;
            adf_os_dma_mem_context(memctx);
        } alloc_idx;

        /* sw_rd_idx - where HTT SW has processed bufs filled by rx MAC DMA */
        struct {
            unsigned msdu_desc;
            unsigned msdu_payld;
        } sw_rd_idx;

        /* 
         * refill_retry_timer - timer triggered when the ring is not 
         * refilled to the level expected 
         */
        adf_os_timer_t refill_retry_timer;

        /* 
         * refill_ref_cnt - ref cnt for Rx buffer replenishment - this
         * variable is used to guarantee that only one thread tries
         * to replenish Rx ring.
         */
        adf_os_atomic_t refill_ref_cnt;
    } rx_ring;
    int rx_desc_size_hl;
    int rx_fw_desc_offset;
    int rx_mpdu_range_offset_words;
    int rx_ind_msdu_byte_idx;

    struct {
        int size; /* of each HTT tx desc */
        int pool_elems;
        int alloc_cnt;
        char *pool_vaddr;
        u_int32_t pool_paddr;
        u_int32_t *freelist;
        adf_os_dma_mem_context(memctx);
    } tx_descs;
    void (*tx_send_complete_part2)(
        void *pdev, A_STATUS status, adf_nbuf_t msdu, u_int16_t msdu_id);

    HTT_TX_MUTEX_TYPE htt_tx_mutex;

    struct {
        int htc_err_cnt;
    } stats;

    int cur_seq_num_hl;
    ATH_LIST_HEAD(, htt_pdev_buf_entry)  buf_list_head;    
#if RX_DEBUG
    uint32_t g_peer_id;
    uint8_t g_peer_mac[6];
    uint32_t g_num_mpdus;
    uint32_t g_mpdu_ranges;
    uint8_t g_msdu_chained;
    uint8_t g_tid;
    enum htt_rx_status g_htt_status;

    int htt_rx_donebit_assertflag;
#endif
};

/*
 * Macro to htc_endpoint from outside HTT
 * e.g. OL in the data-path
 */
#define HTT_EPID_GET(_htt_pdev_hdl)  \
    (((struct htt_pdev_t *)(_htt_pdev_hdl))->htc_endpoint)

#define HTT_DOWNLOADLEN_GET(_htt_pdev_hdl)  \
    (((struct htt_pdev_t *)(_htt_pdev_hdl))->download_len)


#endif /* _HTT_TYPES__H_ */
