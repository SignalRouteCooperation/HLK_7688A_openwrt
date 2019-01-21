/*
 *  Copyright (c) 2005 Atheros Communications Inc.  All rights reserved.
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
 * Copyright (c) 2013 Qualcomm Atheros, Inc.
 * All Rights Reserved. 
 * Qualcomm Atheros Confidential and Proprietary. 
 */ 

#ifndef _WBUF_ADF_PRIVATE_LINUX_H
#define _WBUF_ADF_PRIVATE_LINUX_H

#include <osdep_adf.h>
#include <ieee80211.h>

typedef __adf_nbuf_t __wbuf_t;
static int is_dhcp(adf_nbuf_t nbuf, struct ether_header * eh);
static int is_arp(adf_nbuf_t nbuf, struct ether_header * eh);
 

/*
 * WBUF private API's for Linux
 */

/*
 * NB: This function shall only be called for wbuf
 * with type WBUF_RX or WBUF_RX_INTRENAL.
 */
#if USE_MULTIPLE_BUFFER_RCV
#define __wbuf_init(_nbf, _pktlen)  do {              \
    __adf_nbuf_trim_tail(_nbf, __adf_nbuf_len(_nbf)); \
    __adf_nbuf_put_tail(_nbf, _pktlen);               \
    __adf_nbuf_set_protocol(_nbf, ETH_P_CONTROL);     \
} while (0)
#else
#define __wbuf_init(_nbf, _pktlen)  do {              \
    __adf_nbuf_put_tail(_nbf, _pktlen);               \
    __adf_nbuf_set_protocol(_nbf, ETH_P_CONTROL);     \
} while (0)
#endif

#define __wbuf_header                 __adf_nbuf_data
#define __wbuf_getcb                  __adf_nbuf_cb
#define __wbuf_set_cboffset           __adf_nbuf_set_cboffset
#define __wbuf_get_cboffset           __adf_nbuf_get_cboffset

/*
 * NB: The following two API's only work when nbf's header
 * has not been ajusted, i.e., no one calls skb_push or skb_pull
 * on this skb yet.
 */
#define __wbuf_raw_data               __adf_nbuf_data

#if USE_MULTIPLE_BUFFER_RCV
#define __wbuf_get_len(wbuf)          (N_BUF_SIZE_GET(wbuf))
#else
#define __wbuf_get_len(wbuf)          (__adf_nbuf_tailroom(wbuf)) 
#endif

#define __wbuf_get_datalen_temp       __adf_nbuf_len


#define __wbuf_get_pktlen             __adf_nbuf_len
#define __wbuf_get_tailroom           __adf_nbuf_tailroom
#define __wbuf_get_priority           __adf_nbuf_get_priority
#define __wbuf_set_priority           __adf_nbuf_set_priority

#define __wbuf_hdrspace               __adf_nbuf_headroom

#define __wbuf_next                   __adf_nbuf_queue_next
#define __wbuf_next_buf               __adf_nbuf_queue_next
#define __wbuf_set_next               __adf_nbuf_set_next
#define __wbuf_setnextpkt             __adf_nbuf_set_next

#define __wbuf_free                   __adf_nbuf_free
#define __wbuf_push                   __adf_nbuf_push_head
#define __wbuf_clone(_dev, _nbf)      __adf_nbuf_clone(_nbf)
#define __wbuf_trim                   __adf_nbuf_trim_tail
#define __wbuf_pull                   __adf_nbuf_pull_head
#define __wbuf_set_age                __adf_nbuf_set_age
#define __wbuf_get_age                __adf_nbuf_get_age
#define __wbuf_complete               __adf_nbuf_free

#define __wbuf_copydata               __adf_nbuf_copy_bits
#define __wbuf_copy                   __adf_nbuf_copy



#define __wbuf_set_pktlen             __adf_nbuf_set_pktlen
#define __wbuf_set_smpsactframe         N_SMPSACTM_SET
#define __wbuf_is_smpsactframe          N_SMPSACTM_IS
#define __wbuf_set_moredata             N_MOREDATA_SET
#define __wbuf_is_moredata              N_MOREDATA_IS
#define __wbuf_set_smpsframe            N_NULL_PWR_SAV_SET
#define __wbuf_is_smpsframe             N_NULL_PWR_SAV_IS
#define __wbuf_set_qosframe             N_QOS_SET
#define __wbuf_is_qosframe              N_QOS_IS
#define __wbuf_set_pwrsaveframe         N_PWR_SAV_SET
#define __wbuf_is_pwrsaveframe          N_PWR_SAV_IS
#define __wbuf_set_status               N_STATUS_SET
#define __wbuf_get_status               N_STATUS_GET
#define __wbuf_set_type                 N_TYPE_SET
#define __wbuf_get_type                 N_TYPE_GET
#define __wbuf_set_node                 N_NODE_SET
#define __wbuf_get_node                 N_NODE_GET
#define __wbuf_set_exemption_type       N_EXMTYPE_SET
#define __wbuf_get_exemption_type       N_EXMTYPE_GET
#define __wbuf_set_tid                  N_TID_SET
#define __wbuf_get_tid                  N_TID_GET
#define __wbuf_set_rate                 N_RATE_SET
#define __wbuf_get_rate                 N_RATE_GET
#define __wbuf_set_retries              N_RETRIES_SET
#define __wbuf_get_retries              N_RETRIES_GET
#define __wbuf_set_txpower              N_TXPOWER_SET
#define __wbuf_get_txpower              N_TXPOWER_GET
#define __wbuf_set_txchainmask          N_TXCHAINMASK_SET
#define __wbuf_get_txchainmask          N_TXCHAINMASK_GET
#define __wbuf_set_eapol                N_EAPOL_SET
#define __wbuf_is_eapol                 N_EAPOL_IS
#define __wbuf_set_dhcp                 N_DHCP_SET
#define __wbuf_is_dhcp                  N_DHCP_IS
#define __wbuf_set_qosnull              N_QOSNULL_SET
#define __wbuf_is_qosnull               N_QOSNULL_IS
#define __wbuf_set_arp                  N_ARP_SET
#define __wbuf_is_arp                   N_ARP_IS
#define __wbuf_set_offchan_tx           N_OFFCHANTX_SET
#define __wbuf_is_offchan_tx            N_OFFCHANTX_IS
 


#ifdef ATH_SUPPORT_WAPI
#define __wbuf_set_wai                  N_WAI_SET
#define __wbuf_is_wai                   N_WAI_IS
#endif

#define __wbuf_set_qosframe             N_QOS_SET
#define __wbuf_get_context              N_CONTEXT_GET
#define __wbuf_set_amsdu                N_AMSDU_SET
#define __wbuf_is_amsdu                 N_AMSDU_IS
#define __wbuf_set_probing              N_PROBING_SET
#define __wbuf_is_probing               N_PROBING_IS
#define __wbuf_clear_probing            N_PROBING_CLR
#define __wbuf_set_fastframe            N_FF_SET
#define __wbuf_is_fastframe             N_FF_IS
#define __wbuf_set_cloned              	N_CLONED_SET
#define __wbuf_is_cloned               	N_CLONED_IS
#define __wbuf_clear_cloned            	N_CLONED_CLR
#if ATH_SUPPORT_WIFIPOS
#define __wbuf_set_wifipos             	N_WIFIPOS_SET
#define __wbuf_get_wifipos             	N_WIFIPOS_GET
#define __wbuf_set_wifipos_req_id       N_WIFIPOS_REQ_ID_SET
#define __wbuf_get_wifipos_req_id       N_WIFIPOS_REQ_ID_GET
#define __wbuf_set_pos              	N_POSITION_SET
#define __wbuf_is_pos               	N_POSITION_IS
#define __wbuf_clear_pos            	N_POSITION_CLR
#define __wbuf_set_keepalive            N_POSITION_KEEPALIVE_SET
#define __wbuf_is_keepalive             N_POSITION_KEEPALIVE_IS
#define __wbuf_set_vmf                  N_POSITION_VMF_SET
#define __wbuf_is_vmf                   N_POSITION_VMF_IS
#define __wbuf_set_cts_frame            N_CTS_FRAME_SET
#define __wbuf_get_cts_frame            N_CTS_FRAME_GET
#endif
#ifdef ATH_SUPPORT_UAPSD
#define __wbuf_set_uapsd                N_UAPSD_SET
#define __wbuf_clear_uapsd              N_UAPSD_CLR
#define __wbuf_is_uapsd                 N_UAPSD_IS
#endif

#if LMAC_SUPPORT_POWERSAVE_QUEUE
#define __wbuf_set_legacy_ps                N_LEGACY_PS_SET
#define __wbuf_clear_legacy_ps              N_LEGACY_PS_CLR
#define __wbuf_is_legacy_ps                 N_LEGACY_PS_IS
#endif

#define __wbuf_is_encap_done            N_ENCAP_DONE_IS
#define __wbuf_set_encap_done           N_ENCAP_DONE_SET
#define __wbuf_clr_encap_done           N_ENCAP_DONE_CLR

#if UMAC_SUPPORT_SMARTANTENNA
#define __wbuf_sa_get_antenna              N_ANT_GET
#define __wbuf_sa_get_rateidx              N_RATE_INDEX_GET
#define __wbuf_is_sa_train_packet          N_TRAIN_PKT_IS
#define __wbuf_is_sa_train_lastpacket      N_TRAIN_LASTPKT_IS 

#define __wbuf_sa_set_antenna              N_ANT_SET
#define __wbuf_sa_set_rateidx              N_RATE_INDEX_SET
#define __wbuf_sa_set_train_packet         N_TRAIN_PKT_SET
#define __wbuf_sa_unset_train_packet       N_TRAIN_PKT_UNSET
#define __wbuf_sa_set_train_lastpacket     N_TRAIN_LASTPKT_SET 
#endif

#if UMAC_SUPPORT_WNM
#define __wbuf_set_fmsstream            N_FMSS_SET
#define __wbuf_clear_fmsstream          N_FMSS_CLR
#define __wbuf_is_fmsstream             N_FMSS_IS
#define __wbuf_set_fmsqid               N_FMSQID_SET
#define __wbuf_get_fmsqid               N_FMSQID_GET
#endif /* UMAC_SUPPORT_WNM */

#define wbuf_set_initimbf(nbf)
#define wbuf_set_pktlen                 __wbuf_set_pktlen
#define wbuf_set_smpsactframe           __wbuf_set_smpsactframe
#define wbuf_is_smpsactframe            __wbuf_is_smpsactframe
#define wbuf_set_smpsframe              __wbuf_set_smpsframe
#define wbuf_set_status                 __wbuf_set_status
#define wbuf_set_type                   __wbuf_set_type
#define wbuf_get_type                   __wbuf_get_type
#define wbuf_set_moredata               __wbuf_set_moredata
#define wbuf_is_moredata                __wbuf_is_moredata
#define wbuf_set_len                    __wbuf_set_pktlen

#define DHCP_SRCPORT 68                                                                                                                                
#define DHCP_DSTPORT 67


int __wbuf_map_sg(osdev_t osdev, adf_nbuf_t nbf, dma_context_t context, void *arg);
void __wbuf_unmap_sg(osdev_t osdev, adf_nbuf_t nbf, dma_context_t context);
dma_addr_t __wbuf_map_single(osdev_t osdev, adf_nbuf_t nbf, int direction, dma_context_t context);
void __wbuf_unmap_single(osdev_t osdev, adf_nbuf_t nbf, int direction, dma_context_t context);
void __wbuf_uapsd_update(adf_nbuf_t nbf);

int wbuf_start_dma(adf_nbuf_t nbf, sg_t *sg, u_int32_t n_sg, void *arg);

static INLINE int
__wbuf_append(adf_nbuf_t nbf, u_int16_t size)
{
    if (__adf_nbuf_put_tail(nbf, size) == NULL) {
        KASSERT(0, ("No enough tailroom for wbuf, failed to alloc"));
    }
    return 0;
}

static INLINE __wbuf_t 
__wbuf_coalesce(osdev_t os_handle, adf_nbuf_t nbf)
{
    /* The sk_buff is already contiguous in memory, no need to coalesce. */
    return nbf;
}

static INLINE int
__wbuf_is_initimbf(adf_nbuf_t nbf)
{
    return 0;
}

static INLINE int
wbuf_classify(adf_nbuf_t nbf)
{
    struct ether_header *eh = (struct ether_header *) nbf->data;
    u_int8_t tos = 0;
    /*
     * Find priority from IP TOS DSCP field
     */
    if (eh->ether_type == __constant_htons(ETHERTYPE_IP))
    {
        const struct iphdr *ip = (struct iphdr *)
                    (nbf->data + sizeof (struct ether_header));
	if (is_dhcp(nbf, eh)) {
              if (!IEEE80211_IS_MULTICAST(eh->ether_dhost)) {  /* Only for unicast frames */
                  N_EAPOL_SET(nbf);
                 tos = OSDEP_EAPOL_TID;  /* send it on VO queue */;
              } else {
                  /* else set M_DHCP - so that early drop does not affect it */
                  N_DHCP_SET(nbf);
                  tos = 0;
             }
         } else {
 
        /*
         * IP frame: exclude ECN bits 0-1 and map DSCP bits 2-7
         * from TOS byte.
         */
        tos = (ip->tos & (~INET_ECN_MASK)) >> IP_PRI_SHIFT;
    }
    else if (eh->ether_type == htons(ETHERTYPE_IPV6)) {
        /* TODO
	 * use flowlabel
	 */
        unsigned long ver_pri_flowlabel;
	unsigned long pri;
	/*
            IPv6 Header.
            +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
            |Version| TrafficClass. |                   Flow Label          |
            +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
            |         Payload Length        |  Next Header  |   Hop Limit   |
            +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
            |                                                               |
            +                                                               +
            |                                                               |
            +                         Source Address                        +
            |                                                               |
            +                                                               +
            |                                                               |
            +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
            |                                                               |
            +                                                               +
            |                                                               |
            +                      Destination Address                      +
            |                                                               |
            +                                                               +
            |                                                               |
            +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	*/

        ver_pri_flowlabel = *(unsigned long*)(eh + 1);
        pri = (ntohl(ver_pri_flowlabel) & IPV6_PRIORITY_MASK) >> IPV6_PRIORITY_SHIFT;
        tos = (pri & (~INET_ECN_MASK)) >> IP_PRI_SHIFT;
    }
    else if (eh->ether_type == __constant_htons(ETHERTYPE_PAE)) {
        /* mark as EAPOL frame */
         N_EAPOL_SET(nbf);
         tos = OSDEP_EAPOL_TID;  /* send it on VO queue */;
    }
    else if (is_arp(nbf, eh)) {
         N_ARP_SET(nbf);
          if (!IEEE80211_IS_MULTICAST(eh->ether_dhost)) {  /* Only for unicast frames */
              tos = OSDEP_EAPOL_TID;  /* send ucast arp on VO queue */;
          } else {
              tos = 0;                /* send mcast arp on BE queue */;
         }
      }   

#ifdef ATH_SUPPORT_WAPI
    else if (eh->ether_type == __constant_htons(ETHERTYPE_WAI)) {
        /* mark as WAI frmae */
        N_WAI_SET(nbf);
        tos = OSDEP_EAPOL_TID; /* send it on VO queue */
    }
#endif

    return tos;
}

static INLINE int
wbuf_mark_eapol(adf_nbuf_t nbf)
{
    struct ether_header *eh = (struct ether_header *) nbf->data;
    if (eh->ether_type == __constant_htons(ETHERTYPE_PAE)) {
        /* mark as EAPOL frame */
         N_EAPOL_SET(nbf);
         return 1;
    }

    return 0;
}


static INLINE int
wbuf_get_iptos(adf_nbuf_t nbf, u_int32_t *is_igmp, void **iph)
{
    struct ether_header *eh = (struct ether_header *) nbf->data;
    u_int8_t tos = 0;

    *iph = NULL;
    *is_igmp = 0;
    /* 
     * Find priority from IP TOS DSCP field
     */
    if (eh->ether_type == __constant_htons(ETHERTYPE_IP))
    {
        const struct iphdr *ip = (struct iphdr *)
            (nbf->data + sizeof (struct ether_header));

        *iph = (void *)ip;
        tos = ip->tos;
        if (ip->protocol == IPPROTO_IGMP)
            *is_igmp = 1;
    }
	else if(__constant_ntohs(eh->ether_type) == ETHERTYPE_IPV6)
	{
		struct ipv6hdr *ip6h=(struct ipv6hdr *)(eh+1);
		u_int8_t *nexthdr = (u_int8_t *)(ip6h + 1);
		if (ip6h->nexthdr == IPPROTO_HOPOPTS) {
	            *is_igmp = 1;
        }
	}

    return tos;
}

static INLINE int
wbuf_UserPriority(adf_nbuf_t nbf)
{
    return 0;
}

static INLINE int
wbuf_VlanId(adf_nbuf_t nbf)
{
    return 0;
}
 static
 
 int is_dhcp(adf_nbuf_t nbf, struct ether_header * eh)
 {
  
     int iplen ;
     struct udphdr *udp ;
     int udpSrc, udpDst;
     
     if (eh->ether_type == __constant_htons(ETHERTYPE_IP))
     {
         const struct iphdr *ip = (struct iphdr *) (nbf->data + sizeof (struct ether_header));
 
         iplen = __constant_htons(ip->tot_len);
         if (ip->protocol == 17)
         {
             udp = (struct udphdr *) (nbf->data + sizeof (struct ether_header) + sizeof(struct iphdr) );
             udpSrc = __constant_htons(udp->source);
             udpDst = __constant_htons(udp->dest);
             if ((udpSrc == 67) || (udpSrc == 68)) {
                 return 1;
             }
         }
     }
 
     return 0;
 
}
 
 static
 
int is_arp(adf_nbuf_t nbf, struct ether_header * eh)
 
{
 
    if (eh->ether_type == __constant_htons(ETHERTYPE_ARP))
    {
         return 1;
     }
     return 0;
 }
 
 

#if ATH_SUPPORT_VLAN

#define VLAN_PRI_SHIFT  13
#define VLAN_PRI_MASK   7

/*
** Public Prototypes
*/

unsigned short	adf_net_get_vlan(osdev_t os_handle);
int 			adf_net_is_vlan_defined(osdev_t os_handle);

static INLINE int
wbuf_8021p(adf_nbuf_t nbf)
{
    struct vlan_ethhdr *veth = (struct vlan_ethhdr *) nbf->data;
    u_int8_t tos = 0;

	/*
	** Determine if this is an 802.1p frame, and get the proper
	** priority information as required
	*/
	
    if ( veth->h_vlan_proto == __constant_htons(ETH_P_8021Q) )
	{
    	tos = (veth->h_vlan_TCI >> VLAN_PRI_SHIFT) & VLAN_PRI_MASK;
	}

    return tos;
}

static INLINE int
adf_net_get_vlan_tag(adf_nbuf_t nbf)
{
	return ( vlan_tx_tag_get(nbf) );
}

static INLINE int
adf_net_vlan_tag_present(adf_nbuf_t nbf)
{
	return ( vlan_tx_tag_present(nbf) );
}

#endif


static INLINE void
wbuf_concat(adf_nbuf_t head, adf_nbuf_t nbf)
{
    if (__adf_nbuf_is_nonlinear(head))
    {
        KASSERT(0,("wbuf_concat: skb is nonlinear\n"));
    }
    if (__adf_nbuf_tailroom(head) < nbf->len)
    {

#if USE_MULTIPLE_BUFFER_RCV

        /* For multiple buffer case, it's not a abnormal case if tailroom is 
         * not enough. We should not call assert but handle it carefully.
         */
        
        if (!__adf_nbuf_realloc_tailroom(head, __wbuf_get_len(nbf)))
        {
            __adf_nbuf_free(nbf);
            return;
        }
#else
        KASSERT(0,("wbuf_concat: Not enough space to concat\n"));
#endif
    }
    /* copy the skb data to the head */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22))
    memcpy(head->tail, nbf->data, nbf->len);
#else
    memcpy(skb_tail_pointer(head),nbf->data, nbf->len),
#endif
    /* Update tail and length */
    __adf_nbuf_put_tail(head, nbf->len);
    /* free the skb */
    __adf_nbuf_free(nbf);
}

#if defined(ATH_SUPPORT_VOWEXT) || defined(ATH_SUPPORT_IQUE) || defined(VOW_LOGLATENCY) || UMAC_SUPPORT_NAWDS != 0
static INLINE u_int8_t
__wbuf_get_firstxmit(struct sk_buff *skb)
{
    if (!skb) return 0;
    return ((struct ieee80211_cb *)skb->cb)->firstxmit;
}
static INLINE void
__wbuf_set_firstxmit(struct sk_buff *skb, u_int8_t firstxmit)
{
    if (!skb) return;
    ((struct ieee80211_cb *)skb->cb)->firstxmit = firstxmit;
}

static INLINE u_int32_t
__wbuf_get_firstxmitts(struct sk_buff *skb)
{
    if (!skb) return 0;
    return ((struct ieee80211_cb *)skb->cb)->_u.firstxmitts;
}
static INLINE void
__wbuf_set_firstxmitts(struct sk_buff *skb, u_int32_t firstxmitts)
{
    if (!skb) return;
    ((struct ieee80211_cb *)skb->cb)->_u.firstxmitts = firstxmitts;
}

static INLINE void
__wbuf_clear_flags(struct sk_buff *skb)
{
    N_FLAG_CLR(skb, 0xFFFF);
}
#endif /* ATH_SUPPORT_VOWEXT*/

static INLINE void 
__wbuf_set_complete_handler(struct sk_buff *skb,void *handler, void *arg)
{
#if (ATH_SUPPORT_P2P != 0) || (ATH_SUPPORT_TDLS != 0) || (ATH_SUPPORT_DEFERRED_NODE_CLEANUP != 0)
    struct ieee80211_cb *ctx = (struct ieee80211_cb *)skb->cb;
    ctx->complete_handler = handler;
    ctx->complete_handler_arg = arg;
#endif
}

static INLINE void 
__wbuf_get_complete_handler(struct sk_buff *skb,void **phandler, void **parg)
{
#if (ATH_SUPPORT_P2P != 0) || (ATH_SUPPORT_TDLS != 0) || (ATH_SUPPORT_DEFERRED_NODE_CLEANUP != 0)
    struct ieee80211_cb *ctx = (struct ieee80211_cb *)skb->cb;
    *phandler = ctx->complete_handler;
    *parg = ctx->complete_handler_arg;
#else
    *phandler = NULL;
    *parg = NULL;
#endif
}

#if defined(VOW_TIDSCHED) || defined (ATH_SUPPORT_IQUE) || defined(VOW_LOGLATENCY)
static INLINE u_int32_t
__wbuf_get_qin_timestamp(struct sk_buff *skb)
{
    if (!skb) return 0;
    return ((struct ieee80211_cb *)skb->cb)->qin_timestamp;
}
static INLINE void
__wbuf_set_qin_timestamp(struct sk_buff *skb, u_int32_t qin_timestamp)
{
    if (!skb) return;
    ((struct ieee80211_cb *)skb->cb)->qin_timestamp = qin_timestamp;
}
#endif

#if UMAC_SUPPORT_RPTPLACEMENT  || ATH_SUPPORT_AOW || ATH_SUPPORT_WIFIPOS 
static INLINE void
__wbuf_set_netlink_pid(struct sk_buff *skb, u_int32_t val)
{
    NETLINK_CB(skb).pid = val;
}
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)	
static INLINE void
__wbuf_set_netlink_dst_pid(struct sk_buff *skb, u_int32_t val)
{
    NETLINK_CB(skb).dst_pid = val;
}
#endif    
static INLINE void
__wbuf_set_netlink_dst_group(struct sk_buff *skb, u_int32_t val)
{
    NETLINK_CB(skb).dst_group = val;
}
#endif /*UMAC_SUPPORT_RPTPLACEMENT  || ATH_SUPPORT_AOW || ATH_SUPPORT_WIFIPOS*/

static INLINE u_int8_t *
__wbuf_get_scatteredbuf_header(__wbuf_t wbuf, u_int16_t len)
{
    u_int8_t *pHead = NULL;
    return pHead;
}

#endif
