/*
 * Copyright (c) 2012 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 */

/**
 * @file htt_fw_stats.c
 * @brief Provide functions to process FW status retrieved from FW.
 */

#include <htc_api.h>         /* HTC_PACKET */
#include <htt.h>             /* HTT_T2H_MSG_TYPE, etc. */
#include <adf_nbuf.h>        /* adf_nbuf_t */
#include <adf_os_mem.h>      /* adf_os_mem_set */
#include <ol_fw_tx_dbg.h>    /* ol_fw_tx_dbg_ppdu_base */
#include <bmi.h>
#include <ol_txrx_types.h>
#include <ol_htt_rx_api.h>
#include <ol_txrx_htt_api.h> /* htt_tx_status */
#include <ol_htt_api.h>

#include <htt_internal.h>
#include <htt_types.h>

/* Target status exported */
#include <wlan_defs.h>

#define ROUND_UP_TO_4(val) (((val) + 3) & ~0x3)

#if OL_STATS_WORK_QUEUE
void stats_deferred_work(void *buf)
{
    void *stats_buffer = (void *)buf;
    struct ol_ath_softc_net80211 *scn = NULL;
    void *wqptr = NULL;

    if (!stats_buffer) {
        return;
    }
    scn = (struct ol_ath_softc_net80211 *)(*(uint32_t *)((uint8_t *)stats_buffer + 4));

    htt_t2h_stats_print(((uint8_t *)stats_buffer + 8), 0, scn->target_type);
    /* release stats workqueue buffer */
    wqptr = (void *)(*(uint32_t *)stats_buffer);
    if (wqptr) {
        adf_os_mem_free(wqptr);
    }
    adf_os_mem_free(stats_buffer);
    return;
}
#endif

static void htt_t2h_stats_txbf_data_print(
    wlan_dbg_txbf_data_stats_t *txbf_stats_info, int concise)
{
    adf_os_print("TXBF Data Info:\n");

    adf_os_print("VHT Tx TxBF counts(0..9): ");
    adf_os_print("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
                  txbf_stats_info->tx_txbf_vht[0],
                  txbf_stats_info->tx_txbf_vht[1],
                  txbf_stats_info->tx_txbf_vht[2],
                  txbf_stats_info->tx_txbf_vht[3],
                  txbf_stats_info->tx_txbf_vht[4],
                  txbf_stats_info->tx_txbf_vht[5],
                  txbf_stats_info->tx_txbf_vht[6],
                  txbf_stats_info->tx_txbf_vht[7],
                  txbf_stats_info->tx_txbf_vht[8],
                  txbf_stats_info->tx_txbf_vht[9]
                  );

    adf_os_print("VHT Rx TxBF counts(0..9): ");
    adf_os_print("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
                  txbf_stats_info->rx_txbf_vht[0],
                  txbf_stats_info->rx_txbf_vht[1],
                  txbf_stats_info->rx_txbf_vht[2],
                  txbf_stats_info->rx_txbf_vht[3],
                  txbf_stats_info->rx_txbf_vht[4],
                  txbf_stats_info->rx_txbf_vht[5],
                  txbf_stats_info->rx_txbf_vht[6],
                  txbf_stats_info->rx_txbf_vht[7],
                  txbf_stats_info->rx_txbf_vht[8],
                  txbf_stats_info->rx_txbf_vht[9]
                  );

    adf_os_print("HT Tx TxBF counts(0..7): ");
    adf_os_print("%d, %d, %d, %d, %d, %d, %d, %d, \n",
                  txbf_stats_info->tx_txbf_ht[0],
                  txbf_stats_info->tx_txbf_ht[1],
                  txbf_stats_info->tx_txbf_ht[2],
                  txbf_stats_info->tx_txbf_ht[3],
                  txbf_stats_info->tx_txbf_ht[4],
                  txbf_stats_info->tx_txbf_ht[5],
                  txbf_stats_info->tx_txbf_ht[6],
                  txbf_stats_info->tx_txbf_ht[7]
                  );
    adf_os_print("OFDM Tx TxBF counts(0..7): ");
    adf_os_print("%d, %d, %d, %d, %d, %d, %d, %d, \n",
                  txbf_stats_info->tx_txbf_ofdm[0],
                  txbf_stats_info->tx_txbf_ofdm[1],
                  txbf_stats_info->tx_txbf_ofdm[2],
                  txbf_stats_info->tx_txbf_ofdm[3],
                  txbf_stats_info->tx_txbf_ofdm[4],
                  txbf_stats_info->tx_txbf_ofdm[5],
                  txbf_stats_info->tx_txbf_ofdm[6],
                  txbf_stats_info->tx_txbf_ofdm[7]
                  );

    adf_os_print("OFDM Tx TxBF IBF VHT counts(0..9): ");
    adf_os_print("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
                  txbf_stats_info->tx_txbf_ibf_vht[0],
                  txbf_stats_info->tx_txbf_ibf_vht[1],
                  txbf_stats_info->tx_txbf_ibf_vht[2],
                  txbf_stats_info->tx_txbf_ibf_vht[3],
                  txbf_stats_info->tx_txbf_ibf_vht[4],
                  txbf_stats_info->tx_txbf_ibf_vht[5],
                  txbf_stats_info->tx_txbf_ibf_vht[6],
                  txbf_stats_info->tx_txbf_ibf_vht[7],
                  txbf_stats_info->tx_txbf_ibf_vht[8],
                  txbf_stats_info->tx_txbf_ibf_vht[9]
                  );

    adf_os_print("OFDM Tx TxBF IBF HT counts(0..7): ");
    adf_os_print("%d, %d, %d, %d, %d, %d, %d, %d, \n",
                  txbf_stats_info->tx_txbf_ibf_ht[0],
                  txbf_stats_info->tx_txbf_ibf_ht[1],
                  txbf_stats_info->tx_txbf_ibf_ht[2],
                  txbf_stats_info->tx_txbf_ibf_ht[3],
                  txbf_stats_info->tx_txbf_ibf_ht[4],
                  txbf_stats_info->tx_txbf_ibf_ht[5],
                  txbf_stats_info->tx_txbf_ibf_ht[6],
                  txbf_stats_info->tx_txbf_ibf_ht[7]
                  );

    adf_os_print("OFDM Tx TxBF IBF OFDM counts(0..7): ");
    adf_os_print("%d, %d, %d, %d, %d, %d, %d, %d, \n",
                  txbf_stats_info->tx_txbf_ibf_ofdm[0],
                  txbf_stats_info->tx_txbf_ibf_ofdm[1],
                  txbf_stats_info->tx_txbf_ibf_ofdm[2],
                  txbf_stats_info->tx_txbf_ibf_ofdm[3],
                  txbf_stats_info->tx_txbf_ibf_ofdm[4],
                  txbf_stats_info->tx_txbf_ibf_ofdm[5],
                  txbf_stats_info->tx_txbf_ibf_ofdm[6],
                  txbf_stats_info->tx_txbf_ibf_ofdm[7]
                  );

}

static void htt_t2h_stats_txbf_snd_print(
    wlan_dbg_txbf_snd_stats_t *snd_stats_info, int concise, u_int32_t target_type)
{
    if ((target_type == TARGET_TYPE_QCA9984)||(target_type == TARGET_TYPE_QCA9888)){
        adf_os_print("TXBF Sounding Info:\n");
        /* Sounding */
        adf_os_print("Sounding User 1   : ");
        adf_os_print("20Mhz %d, 40Mhz %d, 80Mhz %d, 160Mhz %d  \n",
                snd_stats_info->sounding[0],
                snd_stats_info->sounding[1],
                snd_stats_info->sounding[2],
                snd_stats_info->sounding[3]
                );


        adf_os_print("Sounding User 2   : ");
        adf_os_print("20Mhz %d, 40Mhz %d, 80Mhz %d, 160Mhz %d  \n",
                snd_stats_info->sounding[4],
                snd_stats_info->sounding[5],
                snd_stats_info->sounding[6],
                snd_stats_info->sounding[7]
                );

        adf_os_print("Sounding User 3   : ");
        adf_os_print("20Mhz %d, 40Mhz %d, 80Mhz %d, 160Mhz %d  \n",
                snd_stats_info->sounding[8],
                snd_stats_info->sounding[9],
                snd_stats_info->sounding[10],
                snd_stats_info->sounding[11]
                );

        adf_os_print("CBF 20 :");
        adf_os_print("IBF %d, EBF %d, MU %d\n",
                snd_stats_info->cbf_20[0],
                snd_stats_info->cbf_20[1],
                snd_stats_info->cbf_20[2]
                );

        adf_os_print("CBF 40 :");
        adf_os_print("IBF %d, EBF %d, MU %d\n",
                snd_stats_info->cbf_40[0],
                snd_stats_info->cbf_40[1],
                snd_stats_info->cbf_40[2]
                );

        adf_os_print("CBF 80 :");
        adf_os_print("IBF %d, EBF %d, MU %d\n",
                snd_stats_info->cbf_80[0],
                snd_stats_info->cbf_80[1],
                snd_stats_info->cbf_80[2]
                );

        adf_os_print("CBF 160 :");
        adf_os_print("IBF %d, EBF %d, MU %d\n",
                snd_stats_info->cbf_160[0],
                snd_stats_info->cbf_160[1],
                snd_stats_info->cbf_160[2]
                );
        adf_os_print("\n\n");
    } else {
        adf_os_print("TXBF Sounding Info:\n");
        /* Sounding */
        adf_os_print("Sounding User 1   : ");
        adf_os_print("20Mhz %d, 40Mhz %d, 80Mhz %d  \n",
                snd_stats_info->sounding[0],
                snd_stats_info->sounding[1],
                snd_stats_info->sounding[2]
                );


        adf_os_print("Sounding User 2   : ");
        adf_os_print("20Mhz %d, 40Mhz %d, 80Mhz %d  \n",
                snd_stats_info->sounding[3],
                snd_stats_info->sounding[4],
                snd_stats_info->sounding[5]
                );

        adf_os_print("Sounding User 3   : ");
        adf_os_print("20Mhz %d, 40Mhz %d, 80Mhz %d  \n",
                snd_stats_info->sounding[6],
                snd_stats_info->sounding[7],
                snd_stats_info->sounding[8]
                );

        adf_os_print("CBF 20 :");
        adf_os_print("IBF %d, EBF %d, MU %d\n",
                snd_stats_info->cbf_20[0],
                snd_stats_info->cbf_20[1],
                snd_stats_info->cbf_20[2]
                );

        adf_os_print("CBF 40 :");
        adf_os_print("IBF %d, EBF %d, MU %d\n",
                snd_stats_info->cbf_40[0],
                snd_stats_info->cbf_40[1],
                snd_stats_info->cbf_40[2]
                );

        adf_os_print("CBF 80 :");
        adf_os_print("IBF %d, EBF %d, MU %d\n",
                snd_stats_info->cbf_80[0],
                snd_stats_info->cbf_80[1],
                snd_stats_info->cbf_80[2]
                );

        adf_os_print("\n\n");
    }

}

static void htt_t2h_stats_wifi2_error_print(
    wlan_dbg_wifi2_error_stats_t *error_stats_info, int concise)
{
    int i;

    adf_os_print("HWSCH Error  (0..3):");
    adf_os_print("%d, %d, %d, %d \n\n",
                  error_stats_info->schd_stall_errs[0],
                  error_stats_info->schd_stall_errs[1],
                  error_stats_info->schd_stall_errs[2],
                  error_stats_info->schd_stall_errs[3]
                  );

    adf_os_print("SchCmdResult (0..7):");
    adf_os_print("%d, %d, %d, %d, %d, %d, %d, %d\n\n",
                  error_stats_info->schd_cmd_result[0],
                  error_stats_info->schd_cmd_result[1],
                  error_stats_info->schd_cmd_result[2],
                  error_stats_info->schd_cmd_result[3],
                  error_stats_info->schd_cmd_result[4],
                  error_stats_info->schd_cmd_result[5],
                  error_stats_info->schd_cmd_result[6],
                  error_stats_info->schd_cmd_result[7]
                  );


    adf_os_print("SIFS Status   (0..7):");
    adf_os_print("%d, %d, %d, %d, %d, %d, %d, %d\n\n",
                  error_stats_info->sifs_status[0],
                  error_stats_info->sifs_status[1],
                  error_stats_info->sifs_status[2],
                  error_stats_info->sifs_status[3],
                  error_stats_info->sifs_status[4],
                  error_stats_info->sifs_status[5],
                  error_stats_info->sifs_status[6],
                  error_stats_info->sifs_status[7]
                  );


    adf_os_print("URRN_stats Error  (0..3):");
    for(i=0;i < WHAL_MAX_URRN_STATS;i++)
        adf_os_print("%d, ", error_stats_info->urrn_stats[i]);
    adf_os_print("\n");

    adf_os_print("Flush Error  (0..9):");
    adf_os_print("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
                  error_stats_info->flush_errs[0],
                  error_stats_info->flush_errs[1],
                  error_stats_info->flush_errs[2],
                  error_stats_info->flush_errs[3],
                  error_stats_info->flush_errs[4],
                  error_stats_info->flush_errs[5],
                  error_stats_info->flush_errs[6],
                  error_stats_info->flush_errs[7],
                  error_stats_info->flush_errs[8],
                  error_stats_info->flush_errs[9]
                  );

    adf_os_print("Flush Error  (10..17):");
    adf_os_print("%d, %d, %d, %d, %d, %d, %d, %d\n\n",
                  error_stats_info->flush_errs[10],
                  error_stats_info->flush_errs[11],
                  error_stats_info->flush_errs[12],
                  error_stats_info->flush_errs[13],
                  error_stats_info->flush_errs[14],
                  error_stats_info->flush_errs[15],
                  error_stats_info->flush_errs[16],
                  error_stats_info->flush_errs[17]
                  );

    adf_os_print("Phy Error    (0..9):");
    adf_os_print("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
                  error_stats_info->phy_errs[0],
                  error_stats_info->phy_errs[1],
                  error_stats_info->phy_errs[2],
                  error_stats_info->phy_errs[3],
                  error_stats_info->phy_errs[4],
                  error_stats_info->phy_errs[5],
                  error_stats_info->phy_errs[6],
                  error_stats_info->phy_errs[7],
                  error_stats_info->phy_errs[8],
                  error_stats_info->phy_errs[9]
                  );

    adf_os_print("Phy  Error   (9..20):");
    adf_os_print("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
                  error_stats_info->phy_errs[10],
                  error_stats_info->phy_errs[11],
                  error_stats_info->phy_errs[12],
                  error_stats_info->phy_errs[13],
                  error_stats_info->phy_errs[14],
                  error_stats_info->phy_errs[15],
                  error_stats_info->phy_errs[16],
                  error_stats_info->phy_errs[17],
                  error_stats_info->phy_errs[18],
                  error_stats_info->phy_errs[19],
                  error_stats_info->phy_errs[20]
                  );

    adf_os_print("\n\n");
}

static void htt_t2h_stats_wifi2_tx_selfgen_print(
    struct wlan_dbg_tx_selfgen_stats *tx_selfgen, int concise)
{
    adf_os_print("TX_SELFGEN Info:\n\n");

    /* SU NDPAs sent */
    adf_os_print("su_ndpa           :\t%d\n",tx_selfgen->su_ndpa);
    /* SU NDPs sent */
    adf_os_print("su_ndp            :\t%d\n",tx_selfgen->su_ndp);
    adf_os_print("su_bar            :\t%d\n",tx_selfgen->su_bar);
    adf_os_print("su_cts2self       :\t%d\n",tx_selfgen->su_cts);
    adf_os_print("su_ndpa_err       :\t%d\n",tx_selfgen->su_ndpa_err);
    adf_os_print("su_ndp_err        :\t%d\n",tx_selfgen->su_ndp_err);
    /* MU NDPAs sent */
    adf_os_print("mu_ndpa           :\t%d\n",tx_selfgen->mu_ndpa);
    /* MU NDPs sent */
    adf_os_print("mu_ndp            :\t%d\n",tx_selfgen->mu_ndp);
    /* BRPOLLs sent as a result of mu-snd seq */
    adf_os_print("mu_brpoll_1       :\t%d\n",tx_selfgen->mu_brpoll_1);
    adf_os_print("mu_brpoll_2       :\t%d\n",tx_selfgen->mu_brpoll_2);
    /* BARs sent typically due to delayed ack in mu-data seq */
    adf_os_print("mu_bar_1          :\t%d\n",tx_selfgen->mu_bar_1);
    adf_os_print("mu_bar_2          :\t%d\n",tx_selfgen->mu_bar_2);
    /* CTSs sent to extend chan ownership for bursting ppdus */
    adf_os_print("mu_cts2self       :\t%d\n",tx_selfgen->mu_cts);
    adf_os_print("mu_ndpa_err        :\t%d\n",tx_selfgen->mu_ndpa_err);
    adf_os_print("mu_ndp_err        :\t%d\n",tx_selfgen->mu_ndp_err);
    adf_os_print("mu_brp1_err       :\t%d\n",tx_selfgen->mu_brp1_err);
    adf_os_print("mu_brp2_err       :\t%d\n",tx_selfgen->mu_brp2_err);

    adf_os_print("\n\n");
}

static void htt_t2h_stats_wifi2_tx_mu_print(
    struct wlan_dbg_tx_mu_stats *tx_mu, int concise)
{
    adf_os_print("TX_MU Info:\n\n");

    adf_os_print("mu_sch_nusers_2         :\t%d\n",tx_mu->mu_sch_nusers_2);
    adf_os_print("mu_sch_nusers_3         :\t%d\n",tx_mu->mu_sch_nusers_3);
    adf_os_print("mu_mpdus_queued_usr0    :\t%d\n",tx_mu->mu_mpdus_queued_usr[0]);
    adf_os_print("mu_mpdus_queued_usr1    :\t%d\n",tx_mu->mu_mpdus_queued_usr[1]);
    adf_os_print("mu_mpdus_queued_usr2    :\t%d\n",tx_mu->mu_mpdus_queued_usr[2]);
    adf_os_print("mu_mpdus_queued_usr3    :\t%d\n",tx_mu->mu_mpdus_queued_usr[3]);
    adf_os_print("mu_mpdus_tried_usr0     :\t%d\n",tx_mu->mu_mpdus_tried_usr[0]);
    adf_os_print("mu_mpdus_tried_usr1     :\t%d\n",tx_mu->mu_mpdus_tried_usr[1]);
    adf_os_print("mu_mpdus_tried_usr2     :\t%d\n",tx_mu->mu_mpdus_tried_usr[2]);
    adf_os_print("mu_mpdus_tried_usr3     :\t%d\n",tx_mu->mu_mpdus_tried_usr[3]);
    adf_os_print("mu_mpdus_failed_usr0    :\t%d\n",tx_mu->mu_mpdus_failed_usr[0]);
    adf_os_print("mu_mpdus_failed_usr1    :\t%d\n",tx_mu->mu_mpdus_failed_usr[1]);
    adf_os_print("mu_mpdus_failed_usr2    :\t%d\n",tx_mu->mu_mpdus_failed_usr[2]);
    adf_os_print("mu_mpdus_failed_usr3    :\t%d\n",tx_mu->mu_mpdus_failed_usr[3]);
    adf_os_print("mu_mpdus_requeued_usr0  :\t%d\n",tx_mu->mu_mpdus_requeued_usr[0]);
    adf_os_print("mu_mpdus_requeued_usr1  :\t%d\n",tx_mu->mu_mpdus_requeued_usr[1]);
    adf_os_print("mu_mpdus_requeued_usr2  :\t%d\n",tx_mu->mu_mpdus_requeued_usr[2]);
    adf_os_print("mu_mpdus_requeued_usr3  :\t%d\n",tx_mu->mu_mpdus_requeued_usr[3]);
    adf_os_print("mu_err_no_ba_usr0       :\t%d\n",tx_mu->mu_err_no_ba_usr[0]);
    adf_os_print("mu_err_no_ba_usr1       :\t%d\n",tx_mu->mu_err_no_ba_usr[1]);
    adf_os_print("mu_err_no_ba_usr2       :\t%d\n",tx_mu->mu_err_no_ba_usr[2]);
    adf_os_print("mu_err_no_ba_usr3       :\t%d\n",tx_mu->mu_err_no_ba_usr[3]);
    adf_os_print("mu_mpdu_underrun_usr0   :\t%d\n",tx_mu->mu_mpdu_underrun_usr[0]);
    adf_os_print("mu_mpdu_underrun_usr1   :\t%d\n",tx_mu->mu_mpdu_underrun_usr[1]);
    adf_os_print("mu_mpdu_underrun_usr2   :\t%d\n",tx_mu->mu_mpdu_underrun_usr[2]);
    adf_os_print("mu_mpdu_underrun_usr3   :\t%d\n",tx_mu->mu_mpdu_underrun_usr[3]);
    adf_os_print("mu_ampdu_underrun_usr0  :\t%d\n",tx_mu->mu_ampdu_underrun_usr[0]);
    adf_os_print("mu_ampdu_underrun_usr1  :\t%d\n",tx_mu->mu_ampdu_underrun_usr[1]);
    adf_os_print("mu_ampdu_underrun_usr2  :\t%d\n",tx_mu->mu_ampdu_underrun_usr[2]);
    adf_os_print("mu_ampdu_underrun_usr3  :\t%d\n",tx_mu->mu_ampdu_underrun_usr[3]);

    adf_os_print("\n\n");
}

static void htt_t2h_stats_wifi2_sifs_resp_print(
    wlan_dgb_sifs_resp_stats_t *sifs_resp_stats, int concise)
{
    adf_os_print("SIFS RESP RX stats:\n\n");

    adf_os_print(" ps-poll trigger           :\t%d\n",sifs_resp_stats->ps_poll_trigger);
    adf_os_print(" u-apsd trigger            :\t%d\n",sifs_resp_stats->uapsd_trigger);
    adf_os_print(" qboost trigger data[exp]  :\t%d\n",sifs_resp_stats->qb_data_trigger[0]);
    adf_os_print(" qboost trigger bar[exp]   :\t%d\n",sifs_resp_stats->qb_bar_trigger[0]);
    adf_os_print(" qboost trigger data[imp]  :\t%d\n",sifs_resp_stats->qb_data_trigger[1]);
    adf_os_print(" qboost trigger bar[imp]   :\t%d\n",sifs_resp_stats->qb_bar_trigger[1]);

    adf_os_print("\n\n");

    adf_os_print("SIFS RESP TX stats:\n\n");

    adf_os_print(" SIFS response data           :\t%d\n",sifs_resp_stats->sifs_resp_data);
    adf_os_print(" SIFS response timing err     :\t%d\n",sifs_resp_stats->sifs_resp_err);

    adf_os_print("\n\n");
}
static void htt_t2h_stats_wifi2_reset_stats_print(
        wlan_dbg_reset_stats_t *reset_stats, int concise)
{
    adf_os_print("RESET stats:\n\n");

    adf_os_print(" warm reset               :\t%d\n",reset_stats->warm_reset);
    adf_os_print(" cold reset               :\t%d\n",reset_stats->cold_reset);
    adf_os_print(" tx flush                 :\t%d\n",reset_stats->tx_flush);
    adf_os_print(" tx glb reset             :\t%d\n",reset_stats->tx_glb_reset);
    adf_os_print(" tx txq reset             :\t%d\n",reset_stats->tx_txq_reset);
    adf_os_print(" rx timeout reset         :\t%d\n",reset_stats->rx_timeout_reset);
    adf_os_print(" hw status mismatch       :\t%d\n",reset_stats->hw_status_mismatch);
    adf_os_print(" hw status multi mismatch :\t%d\n",reset_stats->hw_status_multi_mismatch);

    adf_os_print("\n\n");

}
static void htt_t2h_stats_wifi2_wdog_stats_print(
        wlan_dbg_mac_wdog_stats_t *wdog_stats, int concise)
{
    adf_os_print("MAC WDOG timeout stats:\n\n");

    adf_os_print(" RXPCU                :\t%d\n",wdog_stats->rxpcu);
    adf_os_print(" TXPCU                :\t%d\n",wdog_stats->txpcu);
    adf_os_print(" OLE                  :\t%d\n",wdog_stats->ole);
    adf_os_print(" RXDMA                :\t%d\n",wdog_stats->rxdma);
    adf_os_print(" HWSCH                :\t%d\n",wdog_stats->hwsch);
    adf_os_print(" CRYPTO               :\t%d\n",wdog_stats->crypto);
    adf_os_print(" PDG                  :\t%d\n",wdog_stats->pdg);
    adf_os_print(" TXDMA                :\t%d\n",wdog_stats->txdma);

    adf_os_print("\n\n");

}

static void htt_t2h_stats_tx_rate_stats_print(
    wlan_dbg_tx_rate_info_t *tx_rate_info, int concise)
{
    adf_os_print("TX Rate Info:\n");

    /* MCS */
    adf_os_print("MCS counts (0..9): ");
    adf_os_print("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
                  tx_rate_info->mcs[0],
                  tx_rate_info->mcs[1],
                  tx_rate_info->mcs[2],
                  tx_rate_info->mcs[3],
                  tx_rate_info->mcs[4],
                  tx_rate_info->mcs[5],
                  tx_rate_info->mcs[6],
                  tx_rate_info->mcs[7],
                  tx_rate_info->mcs[8],
                  tx_rate_info->mcs[9]
                  );

   /* SGI */
    adf_os_print("SGI counts (0..9): ");
    adf_os_print("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
                  tx_rate_info->sgi[0],
                  tx_rate_info->sgi[1],
                  tx_rate_info->sgi[2],
                  tx_rate_info->sgi[3],
                  tx_rate_info->sgi[4],
                  tx_rate_info->sgi[5],
                  tx_rate_info->sgi[6],
                  tx_rate_info->sgi[7],
                  tx_rate_info->sgi[8],
                  tx_rate_info->sgi[9]
                  );

    /* NSS */
    adf_os_print("NSS  counts: ");
#if ATH_SUPPORT_4SS
    adf_os_print("1x1 %d, 2x2 %d, 3x3 %d 4x4 %d\n",
            tx_rate_info->nss[0],
            tx_rate_info->nss[1],
            tx_rate_info->nss[2],
            tx_rate_info->nss[3]
            );
#else
    adf_os_print("1x1 %d, 2x2 %d, 3x3 %d\n",
            tx_rate_info->nss[0],
            tx_rate_info->nss[1],
            tx_rate_info->nss[2]
            );
#endif
    /* BW */
    adf_os_print("BW counts: ");
    adf_os_print("20MHz %d, 40MHz %d, 80MHz %d, 160MHz %d\n",
                  tx_rate_info->bw[0],
                  tx_rate_info->bw[1],
                  tx_rate_info->bw[2],
                  tx_rate_info->bw[3]
                  );

     /* Preamble */
    adf_os_print("Preamble (O C H V) counts: ");
    adf_os_print("%d, %d, %d, %d\n",
                  tx_rate_info->pream[0],
                  tx_rate_info->pream[1],
                  tx_rate_info->pream[2],
                  tx_rate_info->pream[3]
                  );

     /* STBC rate counts */
    adf_os_print("STBC rate counts (0..9): ");
    adf_os_print("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
                  tx_rate_info->stbc[0],
                  tx_rate_info->stbc[1],
                  tx_rate_info->stbc[2],
                  tx_rate_info->stbc[3],
                  tx_rate_info->stbc[4],
                  tx_rate_info->stbc[5],
                  tx_rate_info->stbc[6],
                  tx_rate_info->stbc[7],
                  tx_rate_info->stbc[8],
                  tx_rate_info->stbc[9]
                  );

     /* LDPC and TxBF counts */
    adf_os_print("LDPC Counts: ");
    adf_os_print("%d\n", tx_rate_info->ldpc);
    adf_os_print("RTS Counts: ");
    adf_os_print("%d\n", tx_rate_info->rts_cnt);
    /* RSSI Values for last ack frames */
    adf_os_print("Ack RSSI: %d\n",tx_rate_info->ack_rssi);
}

static void htt_t2h_stats_rx_rate_stats_print(
    wlan_dbg_rx_rate_info_t *rx_phy_info, int concise)
{
    adf_os_print("RX Rate Info:\n");

    /* MCS */
    adf_os_print("MCS counts (0..9): ");
    adf_os_print("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
                  rx_phy_info->mcs[0],
                  rx_phy_info->mcs[1],
                  rx_phy_info->mcs[2],
                  rx_phy_info->mcs[3],
                  rx_phy_info->mcs[4],
                  rx_phy_info->mcs[5],
                  rx_phy_info->mcs[6],
                  rx_phy_info->mcs[7],
                  rx_phy_info->mcs[8],
                  rx_phy_info->mcs[9]
                  );

   /* SGI */
    adf_os_print("SGI counts (0..9): ");
    adf_os_print("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
                  rx_phy_info->sgi[0],
                  rx_phy_info->sgi[1],
                  rx_phy_info->sgi[2],
                  rx_phy_info->sgi[3],
                  rx_phy_info->sgi[4],
                  rx_phy_info->sgi[5],
                  rx_phy_info->sgi[6],
                  rx_phy_info->sgi[7],
                  rx_phy_info->sgi[8],
                  rx_phy_info->sgi[9]
                  );

    /* NSS */
    adf_os_print("NSS  counts: ");
    /* nss[0] just holds the count of non-stbc frames that were sent at 1x1
     * rates and nsts holds the count of frames sent with stbc. It was decided
     * to not include PPDUs sent w/ STBC in nss[0] since it would be easier to
     * change the value that needs to be printed (from "stbc+non-stbc count to
     * only non-stbc count") if needed in the future. Hence the addition in the
     * host code at this line. */
    adf_os_print("1x1 %d, 2x2 %d, 3x3 %d, 4x4 %d\n",
                  rx_phy_info->nss[0] + rx_phy_info->nsts,
                  rx_phy_info->nss[1],
                  rx_phy_info->nss[2],
                  rx_phy_info->nss[3]
                  );

    /* NSTS */
    adf_os_print("NSTS count: ");
    adf_os_print("%d\n", rx_phy_info->nsts);

    /* BW */
    adf_os_print("BW counts: ");
    adf_os_print("20MHz %d, 40MHz %d, 80MHz %d, 160MHz %d\n",
                  rx_phy_info->bw[0],
                  rx_phy_info->bw[1],
                  rx_phy_info->bw[2],
                  rx_phy_info->bw[3]
                  );

     /* Preamble */
    adf_os_print("Preamble counts: ");
    adf_os_print("%d, %d, %d, %d, %d, %d\n",
                  rx_phy_info->pream[0],
                  rx_phy_info->pream[1],
                  rx_phy_info->pream[2],
                  rx_phy_info->pream[3],
                  rx_phy_info->pream[4],
                  rx_phy_info->pream[5]
                  );

     /* STBC rate counts */
    adf_os_print("STBC rate counts (0..9): ");
    adf_os_print("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
                  rx_phy_info->stbc[0],
                  rx_phy_info->stbc[1],
                  rx_phy_info->stbc[2],
                  rx_phy_info->stbc[3],
                  rx_phy_info->stbc[4],
                  rx_phy_info->stbc[5],
                  rx_phy_info->stbc[6],
                  rx_phy_info->stbc[7],
                  rx_phy_info->stbc[8],
                  rx_phy_info->stbc[9]
                  );

     /* LDPC and TxBF counts */
    adf_os_print("LDPC Counts: ");
    adf_os_print("%d \n", rx_phy_info->ldpc);

    /* RSSI Values for last received frames */
    adf_os_print("RSSI (data, mgmt): %d, %d\n",rx_phy_info->data_rssi,
                    rx_phy_info->mgmt_rssi);

    adf_os_print("RSSI Chain 0 (0x%02x 0x%02x 0x%02x 0x%02x)\n",
        ((rx_phy_info->rssi_chain0 >> 24) & 0xff),
        ((rx_phy_info->rssi_chain0 >> 16) & 0xff),
        ((rx_phy_info->rssi_chain0 >> 8) & 0xff),
        ((rx_phy_info->rssi_chain0 >> 0) & 0xff));

    adf_os_print("RSSI Chain 1 (0x%02x 0x%02x 0x%02x 0x%02x)\n",
        ((rx_phy_info->rssi_chain1 >> 24) & 0xff),
        ((rx_phy_info->rssi_chain1 >> 16) & 0xff),
        ((rx_phy_info->rssi_chain1 >> 8) & 0xff),
        ((rx_phy_info->rssi_chain1 >> 0) & 0xff));

    adf_os_print("RSSI Chain 2 (0x%02x 0x%02x 0x%02x 0x%02x)\n",
        ((rx_phy_info->rssi_chain2 >> 24) & 0xff),
        ((rx_phy_info->rssi_chain2 >> 16) & 0xff),
        ((rx_phy_info->rssi_chain2 >> 8) & 0xff),
        ((rx_phy_info->rssi_chain2 >> 0) & 0xff));

    adf_os_print("RSSI Chain 3 (0x%02x 0x%02x 0x%02x 0x%02x)\n",
            ((rx_phy_info->rssi_chain3 >> 24) & 0xff),
            ((rx_phy_info->rssi_chain3 >> 16) & 0xff),
            ((rx_phy_info->rssi_chain3 >> 8) & 0xff),
            ((rx_phy_info->rssi_chain3 >> 0) & 0xff));

    adf_os_print("RSSI (comb_ht): %d \n",rx_phy_info->rssi_comb_ht);
}

static void
htt_t2h_stats_pdev_tidq_stats_print(struct wlan_dbg_tidq_stats *wal_pdev_tidq_stats, int concise)
{
    struct wlan_dbg_txq_stats *tidq_st = &wal_pdev_tidq_stats->txq_st;
    int i = 0;
    adf_os_print(" WAL TID QUEUE STATS PER H/W Q \n");
    if(wal_pdev_tidq_stats->wlan_dbg_tid_txq_status == 1)
      printk(" Could not read TIDQ stats from firmware \n");
    else
    {
     adf_os_print(" ============================= \n");
     adf_os_print("   Frames queued to h/w Q \n");
     for (i=0; i < 10; i++)
       adf_os_print("\tQ%d",i);
     adf_os_print("\n");
     for (i=0; i < DBG_STATS_MAX_HWQ_NUM; i++){
        adf_os_print("\t%d", (unsigned short)tidq_st->num_pkts_queued[i]);
     }
     adf_os_print("\n TID Queue stats \n");
     adf_os_print(" S/W queue stats <---> H/W queue stats\n");
     adf_os_print("\t------------------------------\n");
     for(i=0; i< DBG_STATS_MAX_TID_NUM; i++)
     {
      adf_os_print("TID %d\t%d\t\t%d\n",i,tidq_st->tid_sw_qdepth[i], tidq_st->tid_hw_qdepth[i]);
     }
     adf_os_print("\n\t------------------------------\n");
    }
}
static void
htt_t2h_stats_pdev_stats_print(
    struct wlan_dbg_stats *wal_pdev_stats, int concise)
{
    struct wlan_dbg_tx_stats *tx = &wal_pdev_stats->tx;
    struct wlan_dbg_rx_stats *rx = &wal_pdev_stats->rx;
    struct wlan_dbg_mem_stats *mem = &wal_pdev_stats->mem;

    adf_os_print("WAL Pdev stats:\n");
    adf_os_print("\n### MEM ###\n");
    adf_os_print(" ############## Free memory###########\n");
    adf_os_print("IRAM Remaining: \t%d\n",mem->iram_free_size);
    adf_os_print("DRAM Remaining: \t%d\n",mem->dram_free_size);
    adf_os_print("SRAM Remaining: \t%d\n",mem->sram_free_size);

    adf_os_print("\n### Tx ###\n");

    /* Num HTT cookies queued to dispatch list */
    adf_os_print("comp_queued       :\t%d\n",tx->comp_queued);
    /* Num HTT cookies dispatched */
	adf_os_print("comp_delivered    :\t%d\n",tx->comp_delivered);
    /* Num MSDU queued to WAL */
	adf_os_print("msdu_enqued       :\t%d\n",tx->msdu_enqued);
    /* Num MSDUs dropped by WMM limit */
	adf_os_print("wmm_drop          :\t%d\n",tx->wmm_drop);
    /* Num Local frames queued */
	adf_os_print("local_enqued      :\t%d\n",tx->local_enqued);
    /* Num Local frames done */
	adf_os_print("local_freed       :\t%d\n",tx->local_freed);
    /* Num queued to HW */
	adf_os_print("hw_queued         :\t%d\n",tx->hw_queued);
    /* Num PPDU reaped from HW */
	adf_os_print("hw_reaped         :\t%d\n",tx->hw_reaped);
    /* Num underruns */
	adf_os_print("mac underrun      :\t%d\n",tx->underrun);
    /* Num underruns */
	adf_os_print("phy underrun      :\t%d\n",tx->phy_underrun);
    /* Num PPDUs cleaned up in TX abort */
    	adf_os_print("hw_paused         :\t%d\n",tx->hw_paused);
    /* Seq posted */
	adf_os_print("seq_posted        :\t%d\n",tx->seq_posted);
    /* MU Seq posted */
	adf_os_print("mu_seq_posted     :\t%d\n",tx->mu_seq_posted);
    /* Seq failed */
	adf_os_print("seq_failed        :\t%d\n",tx->seq_failed_queueing);
    /* Seq restarted */
	adf_os_print("seq_restarted     :\t%d\n",tx->seq_restarted);
    /* Num PPDUs cleaned up in TX abort */
    adf_os_print("tx_abort          :\t%d\n",tx->tx_abort);
    /* Num MPDUs requed by SW */
    adf_os_print("mpdus_requed      :\t%d\n",tx->mpdus_requed);
    /* Num MPDUs flushed by SW, HWPAUSED, SW TXABORT (Reset,channel change) */
    adf_os_print("mpdus_sw_flush    :\t%d\n",tx->mpdus_sw_flush);
    /* Num MPDUs filtered by HW, all filter condition (TTL expired) */
    adf_os_print("mpdus_hw_filter   :\t%d\n",tx->mpdus_hw_filter);
    /* Num MPDUs truncated by PDG (TXOP, TBTT, PPDU_duration based on rate, dyn_bw) */
    adf_os_print("mpdus_truncated   :\t%d\n",tx->mpdus_truncated);
    /* Num MPDUs that was tried but didn't receive ACK or BA */
    adf_os_print("mpdus_ack_failed  :\t%d\n",tx->mpdus_ack_failed);
    /* Num MPDUs that was discarded due to TTL expired */
    adf_os_print("mpdus_expired     :\t%d\n",tx->mpdus_expired);
    /* Excessive retries */
    adf_os_print("excess retries    :\t%d\n",tx->tx_xretry);
    /* last data rate */
    adf_os_print("last rc           :\t%d\n",tx->data_rc);
    /* scheduler self triggers */
    adf_os_print("sched self trig   :\t%d\n",tx->self_triggers);
    /* SW retry failures */
    adf_os_print("ampdu retry failed:\t%d\n",tx->sw_retry_failure);
    /* ilegal phy rate errirs */
    adf_os_print("illegal rate errs :\t%d\n",tx->illgl_rate_phy_err);
    /* pdev continous excessive retries  */
    adf_os_print("pdev cont xretry  :\t%d\n",tx->pdev_cont_xretry);
    /* pdev continous excessive retries  */
    adf_os_print("pdev tx timeout   :\t%d\n",tx->pdev_tx_timeout);
    /* pdev resets  */
    adf_os_print("pdev resets       :\t%d\n",tx->pdev_resets);
    /* PPDU > txop duration  */
    adf_os_print("ppdu txop ovf     :\t%d\n",tx->txop_ovf);
    adf_os_print("mcast Drop        :\t%d\n",tx->mc_drop);

    adf_os_print("\n### Rx ###\n");
    /* Cnts any change in ring routing mid-ppdu */
	adf_os_print("ppdu_route_change :\t%d\n",rx->mid_ppdu_route_change);
    /* Total number of statuses processed */
	adf_os_print("status_rcvd       :\t%d\n",rx->status_rcvd);
    /* Extra frags on rings 0-3 */
	adf_os_print("r0_frags          :\t%d\n",rx->r0_frags);
	adf_os_print("r1_frags          :\t%d\n",rx->r1_frags);
	adf_os_print("r2_frags          :\t%d\n",rx->r2_frags);
	//adf_os_print("r3_frags          :\t%d\n",rx->r3_frags);
    /* MSDUs / MPDUs delivered to HTT */
	adf_os_print("htt_msdus         :\t%d\n",rx->htt_msdus);
	adf_os_print("htt_mpdus         :\t%d\n",rx->htt_mpdus);
    /* MSDUs / MPDUs delivered to local stack */
	adf_os_print("loc_msdus         :\t%d\n",rx->loc_msdus);
	adf_os_print("loc_mpdus         :\t%d\n",rx->loc_mpdus);
    /* AMSDUs that have more MSDUs than the status ring size */
	adf_os_print("oversize_amsdu    :\t%d\n",rx->oversize_amsdu);
    /* Number of PHY errors */
	adf_os_print("phy_errs          :\t%d\n",rx->phy_errs);
    /* Number of PHY errors dropped */
	adf_os_print("phy_errs dropped  :\t%d\n",rx->phy_err_drop);
    /* Number of mpdu errors - FCS, MIC, ENC etc. */
	adf_os_print("mpdu_errs         :\t%d\n",rx->mpdu_errs);
	adf_os_print("pdev_rx_timeout   :\t%d\n",rx->pdev_rx_timeout);
    /* Number of overflow mpdu errors. */
    adf_os_print("ovfl_mpdu_errs    :\t%d\n",rx->rx_ovfl_errs);

}

static void
htt_t2h_stats_rx_reorder_stats_print(
    struct rx_reorder_stats *stats_ptr, int concise)
{
    adf_os_print("Rx reorder statistics:\n");
    adf_os_print("  %u non-QoS frames received\n",
                 stats_ptr->deliver_non_qos);
    adf_os_print("  %u frames received in-order\n",
                 stats_ptr->deliver_in_order);
    adf_os_print("  %u frames flushed due to timeout\n",
                 stats_ptr->deliver_flush_timeout);
    adf_os_print("  %u frames flushed due to moving out of window\n",
                 stats_ptr->deliver_flush_oow);
    adf_os_print("  %u frames flushed due to receiving DELBA\n",
                 stats_ptr->deliver_flush_delba);
    adf_os_print("  %u frames discarded due to FCS error\n",
                 stats_ptr->fcs_error);
    adf_os_print("  %u frames discarded due to invalid peer\n",
                 stats_ptr->invalid_peer);
    adf_os_print("  %u frames discarded due to duplication (non aggregation)\n",
                 stats_ptr->dup_non_aggr);
    adf_os_print("  %u frames discarded due to duplication in "
                 "reorder queue\n", stats_ptr->dup_in_reorder);
    adf_os_print("  %u frames discarded due to processed before\n",
                 stats_ptr->dup_past);
    adf_os_print("  %u times reorder timeout happened\n",
                 stats_ptr->reorder_timeout);
    adf_os_print("  %u times bar ssn reset happened\n",
                 stats_ptr->ssn_reset);
    adf_os_print("  %u times incorrect bar received\n",
                 stats_ptr->invalid_bar_ssn);
}

static void htt_t2h_stats_wifi2_desc_stats_print(
        wlan_dbg_tx_desc_stats_t *desc, int concise)
{
    uint32_t i;
    uint32_t *word = NULL;

    adf_os_print("\n ## FW Descriptor Monitor Stats ## \n");

    adf_os_print(" Total FW Desc count            => %u \n",
                  WLAN_DBG_TX_DESC_CFG_TOTAL_GET(desc->word1));
    adf_os_print(" Current Desc Available         => %u \n",
                 WLAN_DBG_TX_DESC_TOTAL_AVAIL_GET(desc->word1));
    for(i=0; i < WLAN_TX_DESC_MAX_BINS; i++)
    {
        word = (uint32_t *)&desc->bin_stats[i];

        adf_os_print("<====================================>\n");

        adf_os_print(" BIN id                        => %u\n",
                WLAN_DBG_TX_DESC_BIN_IDX_GET(word));
        adf_os_print(" Min Desc                      => %u\n",
                WLAN_DBG_TX_DESC_CFG_MIN_GET(word));
        adf_os_print(" Max Desc                      => %u\n",
                WLAN_DBG_TX_DESC_CFG_MAX_GET(word));
        adf_os_print(" Priority                      => %u\n",
                WLAN_DBG_TX_DESC_CFG_PRIO_GET(word));
        adf_os_print(" Hysteresis threshold          => %u\n",
                WLAN_DBG_TX_DESC_CFG_BIN_HYST_THR_GET(word));
        adf_os_print(" Desc consumed                 => %u\n",
                WLAN_DBG_TX_DESC_CURR_TOT_GET(word));
        adf_os_print(" Pre-alloc count               => %u\n",
                WLAN_DBG_TX_DESC_PREALLOC_CNT_GET(word));
        adf_os_print(" Max Desc consumed             => %u\n",
                WLAN_DBG_TX_DESC_BIN_MAX_GET(word));
        adf_os_print(" Low threshold cnt             => %u\n", desc->bin_stats[i].bin_hist_low);
        adf_os_print(" High threshold cnt            => %u\n", desc->bin_stats[i].bin_hist_high);
    }
}

static void htt_t2h_stats_wifi2_fetch_stats_print(
        wlan_dbg_tx_fetch_mgr_stats_t *fetch, int concise)
{
    uint32_t i;
    adf_os_print("\n ## Fetch Manager Stats ## \n");

    adf_os_print(" Total Outstanding Fetch desc       %u\n", fetch->fetch_mgr_total_outstanding_fetch_desc);

    for(i=0; i < WAL_STATS_PREFETCH_MAX_QUEUES; i++)
    {
        adf_os_print("Outstanding Fetch Duration/AC [%u] Outstanding Fetch Desc/AC [%u]\n",
                WLAN_DBG_FETCH_MGR_OUTSTANDING_FETCH_DUR_GET(fetch->fetch_desc__fetch_dur[i]),
                WLAN_DBG_FETCH_MGR_OUTSTANDING_FETCH_DESC_GET(fetch->fetch_desc__fetch_dur[i]));
    }
#if PEER_FLOW_CONTROL
    adf_os_print("\n -- FETCH HIST 500 USEC BIN -- \n");
    adf_os_print("    0 USEC -  500 USEC %010u\n",fetch->fetch_mgr_rtt_histogram_500us[0]);
    adf_os_print("  500 USEC - 1000 USEC %010u\n",fetch->fetch_mgr_rtt_histogram_500us[1]);
    adf_os_print(" 1000 USEC - 1500 USEC %010u\n",fetch->fetch_mgr_rtt_histogram_500us[2]);
    adf_os_print(" 1500 USEC - 2000 USEC %010u\n",fetch->fetch_mgr_rtt_histogram_500us[3]);
    adf_os_print(" 2000 USEC - 2500 USEC %010u\n",fetch->fetch_mgr_rtt_histogram_500us[4]);
    adf_os_print(" 2500 USEC - 3000 USEC %010u\n",fetch->fetch_mgr_rtt_histogram_500us[5]);
    adf_os_print(" 3000 USEC - 3500 USEC %010u\n",fetch->fetch_mgr_rtt_histogram_500us[6]);
    adf_os_print(" 3500 USEC - 4000 USEC %010u\n",fetch->fetch_mgr_rtt_histogram_500us[7]);

    adf_os_print("\n -- FETCH HIST 4 MSEC BIN -- \n");
    adf_os_print("  0 MSEC -  4 MSEC %010u\n",fetch->fetch_mgr_rtt_histogram_4ms[0]);
    adf_os_print("  4 MSEC -  8 MSEC %010u\n",fetch->fetch_mgr_rtt_histogram_4ms[1]);
    adf_os_print("  8 MSEC - 12 MSEC %010u\n",fetch->fetch_mgr_rtt_histogram_4ms[2]);
    adf_os_print(" 12 MSEC - 16 MSEC %010u\n",fetch->fetch_mgr_rtt_histogram_4ms[3]);
    adf_os_print(" 16 MSEC - 20 MSEC %010u\n",fetch->fetch_mgr_rtt_histogram_4ms[4]);
    adf_os_print(" 20 MSEC - 24 MSEC %010u\n",fetch->fetch_mgr_rtt_histogram_4ms[5]);
    adf_os_print(" 24 MSEC - 28 MSEC %010u\n",fetch->fetch_mgr_rtt_histogram_4ms[6]);
    adf_os_print(" 28 MSEC - 32 MSEC %010u\n",fetch->fetch_mgr_rtt_histogram_4ms[7]);
#endif
}


static void htt_t2h_stats_wifi2_prefetch_stats_print(
        wlan_dbg_tx_pf_sched_stats_t *prefetch, int concise)
{
    uint32_t i;
    adf_os_print("\n #### Pre-Fetch Manager Stats #### \n");

    for(i=0; i < WAL_STATS_PREFETCH_MAX_QUEUES; i++)
    {
        adf_os_print("\n <============== AC  [ %u ] ========= >\n", i);
        adf_os_print(" Tx Queued              ==> %u\n", prefetch->tx_queued[i]);
        adf_os_print(" Tx Reaped              ==> %u\n", prefetch->tx_reaped[i]);
        adf_os_print(" Tx Sched               ==> %u\n", prefetch->tx_sched[i]);
#if PEER_FLOW_CONTROL
        adf_os_print(" Tx ReQueued            ==> %u\n", prefetch->tx_requeued[i]);
#endif
        adf_os_print(" Abort Sched            ==> %u\n", prefetch->abort_sched[i]);
#if PEER_FLOW_CONTROL
        adf_os_print(" Sched Fail             ==> %u\n", prefetch->sched_fail[i]);
#endif
        adf_os_print(" Sched Timeout          ==> %u\n", prefetch->sched_timeout[i]);
        adf_os_print(" Sched WaitQ            ==> %u\n", prefetch->tx_sched_waitq[i]);
#if PEER_FLOW_CONTROL
        adf_os_print(" Fetch Request          ==> %u\n", prefetch->fetch_request[i]);
#endif
        adf_os_print(" Fetch Response         ==> %u\n", prefetch->fetch_resp[i]);
        adf_os_print(" Fetch Response Invalid ==> %u\n", prefetch->fetch_resp_invld[i]);
        adf_os_print(" Fetch Response Delayed ==> %u\n", prefetch->fetch_resp_delayed[i]);
    }
}


#define HTT_T2H_STATS_TX_PPDU_TIME_TO_MICROSEC(ticks, microsec_per_tick) \
    (ticks * microsec_per_tick)
static inline int
HTT_T2H_STATS_TX_PPDU_RATE_FLAGS_TO_MHZ(u_int8_t rate_flags)
{
    if (rate_flags & 0x20) return 40;   /* WHAL_RC_FLAG_40MHZ */
    if (rate_flags & 0x40) return 80;   /* WHAL_RC_FLAG_80MHZ */
    if (rate_flags & 0x80) return 160;  /* WHAL_RC_FLAG_160MHZ */
    return 20;
}

#define HTT_FW_STATS_MAX_BLOCK_ACK_WINDOW 64

static void
htt_t2h_tx_ppdu_log_bitmaps_print(
    u_int32_t *queued_ptr,
    u_int32_t *acked_ptr)
{
    char queued_str[HTT_FW_STATS_MAX_BLOCK_ACK_WINDOW+1];
    char acked_str[HTT_FW_STATS_MAX_BLOCK_ACK_WINDOW+1];
    int i, j, word;

    adf_os_mem_set(queued_str, '0', HTT_FW_STATS_MAX_BLOCK_ACK_WINDOW);
    adf_os_mem_set(acked_str, '-', HTT_FW_STATS_MAX_BLOCK_ACK_WINDOW);
    i = 0;
    for (word = 0; word < 2; word++) {
        u_int32_t queued = *(queued_ptr + word);
        u_int32_t acked = *(acked_ptr + word);
        for (j = 0; j < 32; j++, i++) {
            if (queued & (1 << j)) {
                queued_str[i] = '1';
                acked_str[i] = (acked & (1 << j)) ? 'y' : 'N';
            }
        }
    }
    queued_str[HTT_FW_STATS_MAX_BLOCK_ACK_WINDOW] = '\0';
    acked_str[HTT_FW_STATS_MAX_BLOCK_ACK_WINDOW] = '\0';
    adf_os_print("%s\n", queued_str);
    adf_os_print("%s\n", acked_str);
}

static inline u_int16_t htt_msg_read16(u_int16_t *p16)
{
#ifdef BIG_ENDIAN_HOST
    /*
     * During upload, the bytes within each u_int32_t word were
     * swapped by the HIF HW.  This results in the lower and upper bytes
     * of each u_int16_t to be in the correct big-endian order with
     * respect to each other, but for each even-index u_int16_t to
     * have its position switched with its successor neighbor u_int16_t.
     * Undo this u_int16_t position swapping.
     */
    return (((size_t) p16) & 0x2) ? *(p16 - 1) : *(p16 + 1);
#else
    return *p16;
#endif
}

static inline u_int8_t htt_msg_read8(u_int8_t *p8)
{
#ifdef BIG_ENDIAN_HOST
    /*
     * During upload, the bytes within each u_int32_t word were
     * swapped by the HIF HW.
     * Undo this byte swapping.
     */
    switch (((size_t) p8) & 0x3) {
    case 0:
        return *(p8 + 3);
    case 1:
        return *(p8 + 1);
    case 2:
        return *(p8 - 1);
    default /* 3 */:
        return *(p8 - 3);
    }
#else
    return *p8;
#endif
}

void htt_make_u8_list_str(
    u_int32_t *aligned_data,
    char *buffer,
    int space,
    int max_elems)
{
    u_int8_t *p8 = (u_int8_t *) aligned_data;
    char *buf_p = buffer;
    while (max_elems-- > 0) {
        int bytes;
        u_int8_t val;

        val = htt_msg_read8(p8);
        if (val == 0) {
            break; /* not enough data to fill the reserved msg buffer space */
        }
        bytes = adf_os_snprint(buf_p, space, "%d,", val);
        space -= bytes;
        if (space > 0) {
            buf_p += bytes;
        } else {
            break; /* not enough print buffer space for all the data */
        }
        p8++;
    }
    if (buf_p == buffer) {
        *buf_p = '\0'; /* nothing was written */
    } else {
        *(buf_p - 1) = '\0'; /* erase the final comma */
    }
}

void htt_make_u16_list_str(
    u_int32_t *aligned_data,
    char *buffer,
    int space,
    int max_elems)
{
    u_int16_t *p16 = (u_int16_t *) aligned_data;
    char *buf_p = buffer;
    while (max_elems-- > 0) {
        int bytes;
        u_int16_t val;

        val = htt_msg_read16(p16);
        if (val == 0) {
            break; /* not enough data to fill the reserved msg buffer space */
        }
        bytes = adf_os_snprint(buf_p, space, "%d,", val);
        space -= bytes;
        if (space > 0) {
            buf_p += bytes;
        } else {
            break; /* not enough print buffer space for all the data */
        }
        p16++;
    }
    if (buf_p == buffer) {
        *buf_p = '\0'; /* nothing was written */
    } else {
        *(buf_p - 1) = '\0'; /* erase the final comma */
    }
}

void
htt_t2h_tx_ppdu_log_print(
    struct ol_fw_tx_dbg_ppdu_msg_hdr *hdr,
    struct ol_fw_tx_dbg_ppdu_base *record,
    int length, int concise)
{
    int i;
    int record_size;
    int num_records;

    record_size =
        sizeof(*record) +
        hdr->mpdu_bytes_array_len * sizeof(u_int16_t) +
        hdr->mpdu_msdus_array_len * sizeof(u_int8_t) +
        hdr->msdu_bytes_array_len * sizeof(u_int16_t);
    num_records = (length - sizeof(*hdr)) / record_size;
    adf_os_print("Tx PPDU log elements:\n");

    for (i = 0; i < num_records; i++) {
        u_int16_t start_seq_num;
        u_int16_t start_pn_lsbs;
        u_int8_t  num_msdus;
        u_int8_t  num_mpdus;
        u_int16_t peer_id;
        u_int8_t  ext_tid;
        u_int8_t  rate_code;
        u_int8_t  rate_flags;
        u_int8_t  tries;
        u_int8_t  complete;
        u_int32_t time_enqueue_us;
        u_int32_t time_completion_us;
        u_int32_t *msg_word = (u_int32_t *) record;

        /* fields used for both concise and complete printouts */
        start_seq_num =
            ((*(msg_word + OL_FW_TX_DBG_PPDU_START_SEQ_NUM_WORD)) &
            OL_FW_TX_DBG_PPDU_START_SEQ_NUM_M) >>
            OL_FW_TX_DBG_PPDU_START_SEQ_NUM_S;
        complete =
            ((*(msg_word + OL_FW_TX_DBG_PPDU_COMPLETE_WORD)) &
            OL_FW_TX_DBG_PPDU_COMPLETE_M) >>
            OL_FW_TX_DBG_PPDU_COMPLETE_S;

        /* fields used only for complete printouts */
        if (! concise) {
            #define BUF_SIZE 80
            char buf[BUF_SIZE];
            u_int8_t *p8;
            time_enqueue_us = HTT_T2H_STATS_TX_PPDU_TIME_TO_MICROSEC(
                    record->timestamp_enqueue, hdr->microsec_per_tick);
            time_completion_us = HTT_T2H_STATS_TX_PPDU_TIME_TO_MICROSEC(
                    record->timestamp_completion, hdr->microsec_per_tick);

            start_pn_lsbs =
                ((*(msg_word + OL_FW_TX_DBG_PPDU_START_PN_LSBS_WORD)) &
                OL_FW_TX_DBG_PPDU_START_PN_LSBS_M) >>
                OL_FW_TX_DBG_PPDU_START_PN_LSBS_S;
            num_msdus =
                ((*(msg_word + OL_FW_TX_DBG_PPDU_NUM_MSDUS_WORD)) &
                OL_FW_TX_DBG_PPDU_NUM_MSDUS_M) >>
                OL_FW_TX_DBG_PPDU_NUM_MSDUS_S;
            num_mpdus =
                ((*(msg_word + OL_FW_TX_DBG_PPDU_NUM_MPDUS_WORD)) &
                OL_FW_TX_DBG_PPDU_NUM_MPDUS_M) >>
                OL_FW_TX_DBG_PPDU_NUM_MPDUS_S;
            peer_id =
                ((*(msg_word + OL_FW_TX_DBG_PPDU_PEER_ID_WORD)) &
                OL_FW_TX_DBG_PPDU_PEER_ID_M) >>
                OL_FW_TX_DBG_PPDU_PEER_ID_S;
            ext_tid =
                ((*(msg_word + OL_FW_TX_DBG_PPDU_EXT_TID_WORD)) &
                OL_FW_TX_DBG_PPDU_EXT_TID_M) >>
                OL_FW_TX_DBG_PPDU_EXT_TID_S;
            rate_code =
                ((*(msg_word + OL_FW_TX_DBG_PPDU_RATE_CODE_WORD)) &
                OL_FW_TX_DBG_PPDU_RATE_CODE_M) >>
                OL_FW_TX_DBG_PPDU_RATE_CODE_S;
            rate_flags =
                ((*(msg_word + OL_FW_TX_DBG_PPDU_RATE_FLAGS_WORD)) &
                OL_FW_TX_DBG_PPDU_RATE_FLAGS_M) >>
                OL_FW_TX_DBG_PPDU_RATE_FLAGS_S;
            tries =
                ((*(msg_word + OL_FW_TX_DBG_PPDU_TRIES_WORD)) &
                OL_FW_TX_DBG_PPDU_TRIES_M) >>
                OL_FW_TX_DBG_PPDU_TRIES_S;

            adf_os_print("  - PPDU tx to peer %d, TID %d\n", peer_id, ext_tid);
            adf_os_print("    start seq num = %u, start PN LSBs = %#04x\n",
                start_seq_num, start_pn_lsbs);
            adf_os_print("    PPDU is %d MPDUs, (unknown) MSDUs, %d bytes\n",
                num_mpdus,
                //num_msdus, /* not yet being computed in target */
                record->num_bytes);
            if (complete) {
                adf_os_print("    enqueued at %u, completed at %u (microsec)\n",
                    time_enqueue_us, time_completion_us);
                adf_os_print(
                    "    %d total tries, last tx used rate %d "
                    "on %d MHz chan (flags = %#x)\n",
                    tries, rate_code,
                    HTT_T2H_STATS_TX_PPDU_RATE_FLAGS_TO_MHZ(rate_flags),
                    rate_flags);
                adf_os_print("    enqueued and acked MPDU bitmaps:\n");
                htt_t2h_tx_ppdu_log_bitmaps_print(
                    msg_word + OL_FW_TX_DBG_PPDU_ENQUEUED_LSBS_WORD,
                    msg_word + OL_FW_TX_DBG_PPDU_BLOCK_ACK_LSBS_WORD);
            } else {
                adf_os_print(
                    "    enqueued at %d ms (microsec), not yet completed\n",
                    time_enqueue_us);
            }
            /* skip past the regular message fields to reach the tail area */
            p8 = (u_int8_t *) record;
            p8 += sizeof(struct ol_fw_tx_dbg_ppdu_base);
            if (hdr->mpdu_bytes_array_len) {
                htt_make_u16_list_str(
                    (u_int32_t *) p8, buf, BUF_SIZE, hdr->mpdu_bytes_array_len);
                adf_os_print("    MPDU bytes: %s\n", buf);
            }
            p8 += hdr->mpdu_bytes_array_len * sizeof(u_int16_t);
            if (hdr->mpdu_msdus_array_len) {
                htt_make_u8_list_str(
                    (u_int32_t *) p8, buf, BUF_SIZE, hdr->mpdu_msdus_array_len);
                adf_os_print("    MPDU MSDUs: %s\n", buf);
            }
            p8 += hdr->mpdu_msdus_array_len * sizeof(u_int8_t);
            if (hdr->msdu_bytes_array_len) {
                htt_make_u16_list_str(
                    (u_int32_t *) p8, buf, BUF_SIZE, hdr->msdu_bytes_array_len);
                adf_os_print("    MSDU bytes: %s\n", buf);
            }
        } else {
            /* concise */
            adf_os_print(
                "start seq num = %u, enqueued and acked MPDU bitmaps:\n",
                start_seq_num);
            if (complete) {
                htt_t2h_tx_ppdu_log_bitmaps_print(
                    msg_word + OL_FW_TX_DBG_PPDU_ENQUEUED_LSBS_WORD,
                    msg_word + OL_FW_TX_DBG_PPDU_BLOCK_ACK_LSBS_WORD);
            } else {
                adf_os_print("(not completed)\n");
            }
        }
        record = (struct ol_fw_tx_dbg_ppdu_base *)
            (((u_int8_t *) record) + record_size);
    }
}

void
htt_t2h_stats_print(u_int8_t *stats_data, int concise, u_int32_t target_type)
{
    u_int32_t *msg_word = (u_int32_t *)stats_data;
    enum htt_dbg_stats_type   type;
    enum htt_dbg_stats_status status;
    int length;

    type = HTT_T2H_STATS_CONF_TLV_TYPE_GET(*msg_word);
    status = HTT_T2H_STATS_CONF_TLV_STATUS_GET(*msg_word);
    length = HTT_T2H_STATS_CONF_TLV_LENGTH_GET(*msg_word);

    /* check that we've been given a valid stats type */
    if (status == HTT_DBG_STATS_STATUS_SERIES_DONE) {
        return;
    } else if (status == HTT_DBG_STATS_STATUS_INVALID) {
        adf_os_print(
            "Target doesn't support stats type %d\n", type);
        return;
    } else if (status == HTT_DBG_STATS_STATUS_ERROR) {
        adf_os_print(
            "Target couldn't upload stats type %d (no mem?)\n", type);
        return;
    }
    /* got valid (though perhaps partial) stats - process them */
    switch (type) {
    case HTT_DBG_STATS_WAL_PDEV_TXRX:
        {
            struct wlan_dbg_stats *wlan_dbg_stats_ptr;

            wlan_dbg_stats_ptr = (struct wlan_dbg_stats *)(msg_word + 1);
            htt_t2h_stats_pdev_stats_print(wlan_dbg_stats_ptr, concise);
            break;
        }
    case HTT_DBG_STATS_TIDQ:
        {
            struct wlan_dbg_tidq_stats *wlan_dbg_tidq_stats_ptr;

            wlan_dbg_tidq_stats_ptr = (struct wlan_dbg_tidq_stats *)(msg_word + 1);
            htt_t2h_stats_pdev_tidq_stats_print(wlan_dbg_tidq_stats_ptr, concise);
            break;
        }
    case HTT_DBG_STATS_RX_REORDER:
        {
            struct rx_reorder_stats *rx_reorder_stats_ptr;

            rx_reorder_stats_ptr = (struct rx_reorder_stats *)(msg_word + 1);
            htt_t2h_stats_rx_reorder_stats_print(rx_reorder_stats_ptr, concise);
            break;
        }

    case HTT_DBG_STATS_RX_RATE_INFO:
        {
            wlan_dbg_rx_rate_info_t *rx_phy_info;
            rx_phy_info = (wlan_dbg_rx_rate_info_t *)(msg_word + 1);
            htt_t2h_stats_rx_rate_stats_print(rx_phy_info, concise);
            break;
        }
    case HTT_DBG_STATS_TX_PPDU_LOG:
        {
            struct ol_fw_tx_dbg_ppdu_msg_hdr *hdr;
            struct ol_fw_tx_dbg_ppdu_base *record;

            hdr = (struct ol_fw_tx_dbg_ppdu_msg_hdr *)(msg_word + 1);
            record = (struct ol_fw_tx_dbg_ppdu_base *)(hdr + 1);
            htt_t2h_tx_ppdu_log_print(hdr, record, length, concise);
        }
        break;
    case HTT_DBG_STATS_TX_RATE_INFO:
        {
            wlan_dbg_tx_rate_info_t *tx_rate_info;
            tx_rate_info = (wlan_dbg_tx_rate_info_t *)(msg_word + 1);
            htt_t2h_stats_tx_rate_stats_print(tx_rate_info, concise);
            break;
        }

    case HTT_DBG_STATS_TXBF_INFO:
	{
           wlan_dbg_txbf_data_stats_t *txbf_data_stats;
           txbf_data_stats = (wlan_dbg_txbf_data_stats_t *)(msg_word + 1);
           htt_t2h_stats_txbf_data_print(txbf_data_stats, concise);
	   break;
	}

    case HTT_DBG_STATS_SND_INFO:
	{
           wlan_dbg_txbf_snd_stats_t *snd_stats;
           snd_stats = (wlan_dbg_txbf_snd_stats_t *)(msg_word + 1);
           htt_t2h_stats_txbf_snd_print(snd_stats, concise, target_type);
	   break;
	}

    case HTT_DBG_STATS_ERROR_INFO:
	{
            wlan_dbg_wifi2_error_stats_t *error_stats;
            error_stats = (wlan_dbg_wifi2_error_stats_t *)(msg_word + 1);
            htt_t2h_stats_wifi2_error_print(error_stats, concise);
	   break;
	}

    case HTT_DBG_STATS_TX_SELFGEN_INFO:
	{
            struct wlan_dbg_tx_selfgen_stats *tx_selfgen_stats;
            tx_selfgen_stats = (struct wlan_dbg_tx_selfgen_stats *)(msg_word + 1);
            htt_t2h_stats_wifi2_tx_selfgen_print(tx_selfgen_stats, concise);
	   break;
	}

    case HTT_DBG_STATS_TX_MU_INFO:
	{
            struct wlan_dbg_tx_mu_stats *tx_mu_stats;
            tx_mu_stats = (struct wlan_dbg_tx_mu_stats *)(msg_word + 1);
            htt_t2h_stats_wifi2_tx_mu_print(tx_mu_stats, concise);
	   break;
	}

    case HTT_DBG_STATS_SIFS_RESP_INFO:
	{
            wlan_dgb_sifs_resp_stats_t *sifs_resp_stats;
            sifs_resp_stats = (wlan_dgb_sifs_resp_stats_t*)(msg_word + 1);
            htt_t2h_stats_wifi2_sifs_resp_print(sifs_resp_stats, concise);
	   break;
    }
    case HTT_DBG_STATS_RESET_INFO:
    {
        wlan_dbg_reset_stats_t *reset_stats;
        reset_stats = (wlan_dbg_reset_stats_t*)(msg_word + 1);
        htt_t2h_stats_wifi2_reset_stats_print(reset_stats, concise);
        break;
    }
    case HTT_DBG_STATS_MAC_WDOG_INFO:
    {
        wlan_dbg_mac_wdog_stats_t *wdog_stats;
        wdog_stats = (wlan_dbg_mac_wdog_stats_t*)(msg_word + 1);
        htt_t2h_stats_wifi2_wdog_stats_print(wdog_stats, concise);
        break;
    }
    case HTT_DBG_STATS_TX_DESC_INFO:
    {
        wlan_dbg_tx_desc_stats_t *desc_stats;
        desc_stats = (wlan_dbg_tx_desc_stats_t *)(msg_word + 1);
        htt_t2h_stats_wifi2_desc_stats_print(desc_stats, concise);
        break;
    }
    case HTT_DBG_STATS_TX_FETCH_MGR_INFO:
    {
        wlan_dbg_tx_fetch_mgr_stats_t *fetch_stats;
        fetch_stats = (wlan_dbg_tx_fetch_mgr_stats_t *)(msg_word + 1);
        htt_t2h_stats_wifi2_fetch_stats_print(fetch_stats, concise);
        break;
    }
    case HTT_DBG_STATS_TX_PFSCHED_INFO:
    {
        wlan_dbg_tx_pf_sched_stats_t *prefetch_stats;
        prefetch_stats = (wlan_dbg_tx_pf_sched_stats_t *)(msg_word + 1);
        htt_t2h_stats_wifi2_prefetch_stats_print(prefetch_stats, concise);
        break;
    }
    default:
        break;
    }
}



