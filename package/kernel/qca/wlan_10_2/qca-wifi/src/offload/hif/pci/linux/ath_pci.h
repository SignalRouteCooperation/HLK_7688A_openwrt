/*
 * Copyright (c) 2013 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 */

#ifndef __ATH_PCI_H__
#define __ATH_PCI_H__

#include <linux/version.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
#include <asm/semaphore.h>
#else
#include <linux/semaphore.h>
#endif
#include <linux/interrupt.h>

#define CONFIG_COPY_ENGINE_SUPPORT /* TBDXXX: here for now */
#undef CONFIG_ATH_SYSFS_CE
#define ATH_DBG_DEFAULT   0

#include <osdep.h>
#include <if_bus.h>
#include <ol_if_athvar.h>
#include <athdefs.h>
#include "a_osapi.h"
#if 0
#include "a_types.h"
#include "athdefs.h"
#endif
#include "hif.h"
#include "cepci.h"
#include "ol_ath.h"

/* Maximum number of Copy Engine's supported */
#define CE_COUNT_MAX 8
#if QCA_OL_11AC_FAST_PATH
#define CE_HTT_H2T_MSG_SRC_NENTRIES 4096
#else   /* QCA_OL_11AC_FAST_PATH */
#define CE_HTT_H2T_MSG_SRC_NENTRIES 2048
#endif  /* QCA_OL_11AC_FAST_PATH */

struct CE_state;
struct ol_ath_softc_net80211;

/* An address (e.g. of a buffer) in Copy Engine space. */
typedef ath_dma_addr_t CE_addr_t;

#if defined(CONFIG_ATH_SYSFS_CE)
struct ath_sysfs_buf_list {
    struct ath_sysfs_buf_list *next;
    char *host_data;     /* buffer in Host space */
    CE_addr_t CE_data;   /* buffer in CE space */
    size_t nbytes;
};

int ath_sysfs_CE_init(struct ol_ath_softc_net80211 *sc);
#endif

struct ath_hif_pci_softc {
    void __iomem *mem; /* PCI address. */
                       /* For efficiency, should be first in struct */

    struct device *dev;
    struct pci_dev *pdev;
    struct pci_device_id *id;
    struct _NIC_DEV aps_osdev;
    struct ol_ath_softc_net80211 *scn;
    int num_msi_intrs; /* number of MSI interrupts granted */
			/* 0 --> using legacy PCI line interrupts */
    struct tasklet_struct intr_tq;    /* tasklet */
    struct work_struct pci_reconnect_work;	/* work queue */
#if QCA_OL_11AC_FAST_PATH
    int hif_started; /* Duplicating this for data path efficiency */
#if PCI_INTERRUPT_WAR_ENABLE
    os_timer_t intr_timeout_timer;
#endif
#endif /* QCA_OL_11AC_FAST_PATH */

    int irq;
    int irq_event;
    int cacheline_sz;
    /*
    * Guard changes to Target HW state and to software
    * structures that track hardware state.
    */
    adf_os_spinlock_t target_lock;

#if defined(CONFIG_ATH_SYSFS_CE)
    struct CE_handle *sysfs_copyeng_recv;
    spinlock_t CE_recvq_lock;
    struct semaphore CE_recvdata_sem; /* tracks # full bufs recv'd */
    struct ath_sysfs_buf_list *CE_recvq_head;
    struct ath_sysfs_buf_list *CE_recvq_tail;

    struct CE_handle *sysfs_copyeng_send;
    spinlock_t CE_sendq_lock;
    struct semaphore CE_sendbuf_sem; /* tracks # free bufs */
    struct ath_sysfs_buf_list *CE_sendq_freehead;
    struct ath_sysfs_buf_list *CE_sendq_freetail;
#endif
    unsigned int ce_count; /* Number of Copy Engines supported */
    struct CE_state *CE_id_to_state[CE_COUNT_MAX]; /* Map CE id to CE_state */
    HIF_DEVICE *hif_device;

    bool force_break;  /* Flag to indicate whether to break out the DPC context */
    unsigned int receive_count; /* count Num Of Receive Buffers handled for one interrupt DPC routine */
    bool sc_valid;
};

#define TARGID(sc) ((A_target_id_t)(&(sc)->mem))
#define TARGID_TO_HIF(targid) (((struct ath_hif_pci_softc *)((char *)(targid) - (char *)&(((struct ath_hif_pci_softc *)0)->mem)))->hif_device)

bool ath_pci_targ_is_awake(void *__iomem *mem);

bool ath_pci_targ_is_present(A_target_id_t targetid, void *__iomem *mem);

bool ath_max_num_receives_reached(unsigned int count);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
#define DMA_MAPPING_ERROR(dev, addr) dma_mapping_error((addr))
#else
#define DMA_MAPPING_ERROR(dev, addr) dma_mapping_error((dev), (addr))
#endif

extern int ol_ath_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id);
extern void ol_ath_pci_remove(struct pci_dev *pdev);
extern int ol_ath_pci_suspend(struct pci_dev *pdev, pm_message_t state);
extern int ol_ath_pci_resume(struct pci_dev *pdev);
extern void ol_ath_pci_disable_device(struct pci_dev *pdev);

extern void ath_pci_device_reset(struct ath_hif_pci_softc *sc);
extern void ath_pci_device_warm_reset(struct ath_hif_pci_softc *sc);

extern void pci_defer_reconnect(struct work_struct *pci_reconnect_work);

int HIF_PCIDeviceProbed(hif_handle_t hif_hdl);
irqreturn_t HIF_fw_interrupt_handler(int irq, void *arg);

#if 0
extern void kgdb_breakpoint(void);
#define BREAK() \
do { \
    kgdb_breakpoint(); \
} while (0)
#endif

/*
 * A firmware interrupt to the Host is indicated by the
 * low bit of SCRATCH_3_ADDRESS being set.
 */
#define FW_EVENT_PENDING_REG_ADDRESS SCRATCH_3_ADDRESS

/*
 * Typically, MSI Interrupts are used with PCIe. To force use of legacy
 * "ABCD" PCI line interrupts rather than MSI, define FORCE_LEGACY_PCI_INTERRUPTS.
 * Even when NOT forced, the driver may attempt to use legacy PCI interrupts
 * MSI allocation fails
 */
#define LEGACY_INTERRUPTS(sc) ((sc)->num_msi_intrs == 0)
#endif /* __ATH_PCI_H__ */
