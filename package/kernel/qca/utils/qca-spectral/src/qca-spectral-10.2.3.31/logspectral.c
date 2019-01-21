/*
 * Copyright (c) 2011 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 */

/* Application to log spectral data from 11ac chipsets

   IMPORTANT:
   This is only temporary code that has been quickly written to
   validate spectral functionality. This code is NOT for
   any sort of production use and is likely to be in need
   of many improvements. */

#include <stdio.h>
#include <sys/types.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <getopt.h>
#include <netdb.h>
#include <errno.h>
#include <errno.h>
#include <signal.h>
#include <sys/ioctl.h>
#include "logspectral.h"
#include "if_athioctl.h"
#include "ah.h"
#include "ah_internal.h"
#include "drivers/include/AR9888/hw/mac_pcu_reg.h"
#include "drivers/include/AR9888/hw/bb_reg_map.h"
#include "drivers/include/AR9888/hw/mac_descriptors/rx_ppdu_start.h"
#include "drivers/include/AR9888/hw/mac_descriptors/rx_ppdu_end.h"
#include "drivers/include/spectral_adc_defs.h"


/* Enable Reg hack */
#define CONFIGURE_SPECTRAL_VIA_DIRECT_REG_OPERATION 1
#define SPECTRAL_DATA_IN_TLV_FORMAT                 1

/* Global config - access this directly only where required. Else pass pointer */
logspectral_config_t config;

/* device Id */
int dev;

/* Options processing */
static const char *optString = "f:g:i:a:b:c:d:D:e:j:k:l:m:n:o:p:Pq:r:s:t:u:v:xh?";

void dump_spectral_register_params();

static const struct option longOpts[] = {
    { "file", required_argument, NULL, 'f' },
    { "file2", required_argument, NULL, 'g' },
    { "device_name", required_argument, NULL, 'D' },
    { "interface", required_argument, NULL, 'i' },
    { "spectral_scan_priority", required_argument, NULL, 'a' },
    { "spectral_scan_count", required_argument, NULL, 'b' },
    { "spectral_scan_dBm_adj", required_argument, NULL, 'c' },
    { "spectral_scan_pwr_format", required_argument, NULL, 'd' },
    { "spectral_scan_fft_size", required_argument, NULL, 'e' },
    { "spectral_scan_rssi_rpt_mode", required_argument, NULL, 'j' },
    { "spectral_scan_wb_rpt_mode", required_argument, NULL, 'k' },
    { "spectral_scan_str_bin_thr", required_argument, NULL, 'l' },
    { "spectral_scan_nb_tone_thr", required_argument, NULL, 'm' },
    { "spectral_scan_rssi_thr", required_argument, NULL, 'n' },
    { "spectral_scan_restart_ena", required_argument, NULL, 'o' },
    { "spectral_scan_gc_ena", required_argument, NULL, 'p' },
    { "Pause", no_argument, NULL, 'P' },
    { "spectral_scan_period", required_argument, NULL, 'q' },
    { "spectral_scan_bin_scale", required_argument, NULL, 'r' },
    { "spectral_scan_rpt_mode", required_argument, NULL, 's' },
    { "spectral_scan_chn_mask", required_argument, NULL, 't' },
    { "spectral_scan_init_delay", required_argument, NULL, 'u' },
    { "spectral_scan_noise_floor_ref", required_argument, NULL, 'v' },
    { "dump_reg", no_argument, NULL, 'x' },
    { NULL, no_argument, NULL, 0 },
};


/* Static functions */
static int configure_registers(logspectral_config_t *config);
static void signal_handler(int signal);
void cleanup(logspectral_config_t *config);
static void hexdump(FILE* fp, unsigned char *buf, unsigned int len);
static int process_spectral_adc_sample(logspectral_config_t *config);
static int obtain_spectral_samples(logspectral_config_t *config);
static void display_help();
static uint32_t regread(logspectral_config_t *config, u_int32_t off);
static void regwrite(logspectral_config_t *config, uint32_t off, uint32_t value);
static int process_spectral_summary_report(logspectral_config_t *config, uint32_t serial_num, uint8_t *tlv, uint32_t tlvlen);
static int process_search_fft_report(logspectral_config_t *config, uint32_t serial_num, uint8_t *tlv, uint32_t tlvlen);
static int process_adc_report(logspectral_config_t *config, uint32_t serial_num, uint8_t *tlv, uint32_t tlvlen);
static int process_phy_tlv(logspectral_config_t *config, uint32_t serial_num, void* buf);
static int process_rx_ppdu_start(logspectral_config_t *config, uint32_t serial_num, struct rx_ppdu_start *ppdu_start);
static int process_rx_ppdu_end(logspectral_config_t *config, uint32_t serial_num, struct rx_ppdu_end *ppdu_end);
static int dw_configure_registers(logspectral_config_t* config);
static void print_spectral_config(logspectral_config_t *config);

#define regupdate(config, REGISTERNAME, REGISTERFIELD, value) \
{\
    uint32_t regval;\
    regval= (regread((config), PHY_##REGISTERNAME##_ADDRESS) & (~PHY_##REGISTERNAME##_##REGISTERFIELD##_MASK)) | PHY_##REGISTERNAME##_##REGISTERFIELD##_SET((value));\
    regwrite((config), PHY_##REGISTERNAME##_ADDRESS, regval);\
}

#define macregupdate(config, REGISTERNAME, REGISTERFIELD, value) \
{\
    uint32_t regval;\
    regval= (regread((config), REGISTERNAME##_ADDRESS) & (~REGISTERNAME##_##REGISTERFIELD##_MASK)) | REGISTERNAME##_##REGISTERFIELD##_SET((value));\
    regwrite((config), REGISTERNAME##_ADDRESS, regval);\
}


#if CONFIGURE_SPECTRAL_VIA_DIRECT_REG_OPERATION
/*
 * Temp functions to use direct read/write from PCI space
 */
#define update_reg(dev, REGISTERNAME, REGISTERFIELD, value) \
{\
    uint32_t regval;\
    uint32_t temp;\
    regval= (read_reg((dev), PHY_##REGISTERNAME##_ADDRESS, &temp) & (~PHY_##REGISTERNAME##_##REGISTERFIELD##_MASK)) | PHY_##REGISTERNAME##_##REGISTERFIELD##_SET((value));\
    write_reg((dev), PHY_##REGISTERNAME##_ADDRESS, regval);\
    printf("\n");\
}

/*
 * Temp functions to use direct read/write from PCI space
 */
#define update_mac_reg(dev, REGISTERNAME, REGISTERFIELD, value) \
{\
    uint32_t regval;\
    regval= (read_reg((dev), REGISTERNAME##_ADDRESS, &regval) & (~REGISTERNAME##_##REGISTERFIELD##_MASK)) | REGISTERNAME##_##REGISTERFIELD##_SET((value));\
    write_reg((dev), REGISTERNAME##_ADDRESS, regval);\
    printf("\n");\
}

#endif  // CONFIGURE_SPECTRAL_VIA_DIRECT_REG_OPERATION


void dump_spectral_register_params()
{
    int val;
    int reg_ss_1;
    int reg_ss_2;
    int reg_ss_3;
    int fft_ctl_1;
    int fft_ctl_2;
    int fft_ctl_4;
    reg_ss_1 = read_reg(dev, 0x2a228, &val);

    printf("Enable                = %d\n", PHY_BB_SPECTRAL_SCAN_SPECTRAL_SCAN_ENA_GET(reg_ss_1));
    printf("Active                = %d\n", PHY_BB_SPECTRAL_SCAN_SPECTRAL_SCAN_ACTIVE_GET(reg_ss_1));
    printf("FFT Size              = %d\n", PHY_BB_SPECTRAL_SCAN_SPECTRAL_SCAN_FFT_SIZE_GET(reg_ss_1));
    printf("Scan Period           = %d\n", PHY_BB_SPECTRAL_SCAN_SPECTRAL_SCAN_PERIOD_GET(reg_ss_1));
    printf("Scan Count            = %d\n", PHY_BB_SPECTRAL_SCAN_SPECTRAL_SCAN_COUNT_GET(reg_ss_1));
    printf("Priority              = %d\n", PHY_BB_SPECTRAL_SCAN_SPECTRAL_SCAN_PRIORITY_GET(reg_ss_1));
    printf("GC Enable             = %d\n", PHY_BB_SPECTRAL_SCAN_SPECTRAL_SCAN_GC_ENA_GET(reg_ss_1));
    printf("Restart Enable        = %d\n", PHY_BB_SPECTRAL_SCAN_SPECTRAL_SCAN_RESTART_ENA_GET(reg_ss_1));

    reg_ss_2 = read_reg(dev, 0x29e98, &val);
    printf("Noise Floor ref (hex) = 0x%02x\n", PHY_BB_SPECTRAL_SCAN_2_SPECTRAL_SCAN_NOISE_FLOOR_REF_GET(reg_ss_2));
    printf("Noise Floor ref       = %d\n", (signed char)PHY_BB_SPECTRAL_SCAN_2_SPECTRAL_SCAN_NOISE_FLOOR_REF_GET(reg_ss_2));
    printf("INT Delay             = %d\n", PHY_BB_SPECTRAL_SCAN_2_SPECTRAL_SCAN_INIT_DELAY_GET(reg_ss_2));
    printf("NB Tone thr           = %d\n", PHY_BB_SPECTRAL_SCAN_2_SPECTRAL_SCAN_NB_TONE_THR_GET(reg_ss_2));
    printf("String bin thr        = %d\n", PHY_BB_SPECTRAL_SCAN_2_SPECTRAL_SCAN_STR_BIN_THR_GET(reg_ss_2));
    printf("WB report mode        = %d\n", PHY_BB_SPECTRAL_SCAN_2_SPECTRAL_SCAN_WB_RPT_MODE_GET(reg_ss_2));
    printf("RSSI report           = %d\n", PHY_BB_SPECTRAL_SCAN_2_SPECTRAL_SCAN_RSSI_RPT_MODE_GET(reg_ss_2));

    reg_ss_3 = read_reg(dev, 0x29e9c, &val);
    printf("RSSI Threshold (hex)  = 0x%02x\n", PHY_BB_SPECTRAL_SCAN_3_SPECTRAL_SCAN_RSSI_THR_GET(reg_ss_3));
    printf("RSSI Threshold        = %d\n", (signed char)PHY_BB_SPECTRAL_SCAN_3_SPECTRAL_SCAN_RSSI_THR_GET(reg_ss_3));

    fft_ctl_1 = read_reg(dev, 0x29e80, &val);
    printf("PWR format            = %d\n", PHY_BB_SRCH_FFT_CTRL_1_SPECTRAL_SCAN_PWR_FORMAT_GET(fft_ctl_1));
    printf("RPT Mode              = %d\n", PHY_BB_SRCH_FFT_CTRL_1_SPECTRAL_SCAN_RPT_MODE_GET(fft_ctl_1));
    printf("Bin Scale             = %d\n", PHY_BB_SRCH_FFT_CTRL_1_SPECTRAL_SCAN_BIN_SCALE_GET(fft_ctl_1));
    printf("dBm Adjust            = %d\n", PHY_BB_SRCH_FFT_CTRL_1_SPECTRAL_SCAN_DBM_ADJ_GET(fft_ctl_1));

    fft_ctl_4 = read_reg(dev, 0x29e8c, &val);
    printf("Chain Mask            = %d\n", PHY_BB_SRCH_FFT_CTRL_4_SPECTRAL_SCAN_CHN_MASK_GET(fft_ctl_4));
}



/* Function definitions */

int main(int argc, char *argv[])
{
    int opt = 0;
    int longIndex;
    int err;
    char* dev_name = NULL;
    int dump_reg = FALSE;
    int pause = FALSE;

    logspectral_config_init(&config);

    opt = getopt_long(argc, argv, optString, longOpts, &longIndex);

    while( opt != -1 ) {
        switch(opt) {
            case 'f':
                strncpy(config.logfile_name, optarg, sizeof(config.logfile_name));
                break;

            case 'g':
                strncpy(config.logfile2_name, optarg, sizeof(config.logfile2_name));
                break;

            case 'i':
                /* TODO: Validate input */
                strncpy(config.radio_ifname, optarg, sizeof(config.radio_ifname));
                break;

            case 'a':
                /* TODO: Validate input */
                config.spectral_scan_priority = atoi(optarg);
                break;

            case 'b':
                /* TODO: Validate input */
                config.spectral_scan_count = atoi(optarg);
                break;

            case 'c':
                /* TODO: Validate input */
                config.spectral_scan_dBm_adj = atoi(optarg);
                break;

            case 'd':
                /* TODO: Validate input */
                config.spectral_scan_pwr_format = atoi(optarg);
                break;

            case 'D':
                dev_name = optarg;
                break;

            case 'e':
                /* TODO: Validate input */
                config.spectral_scan_fft_size = atoi(optarg);
                break;

            case 'j':
                /* TODO: Validate input */
                config.spectral_scan_rssi_rpt_mode = atoi(optarg);
                break;

            case 'k':
                /* TODO: Validate input */
                config.spectral_scan_wb_rpt_mode = atoi(optarg);
                break;

            case 'l':
                /* TODO: Validate input */
                config.spectral_scan_str_bin_thr = atoi(optarg);
                break;

            case 'm':
                /* TODO: Validate input */
                config.spectral_scan_nb_tone_thr = atoi(optarg);
                break;

            case 'n':
                /* TODO: Validate input */
                config.spectral_scan_rssi_thr = atoi(optarg);
                break;

            case 'o':
                /* TODO: Validate input */
                config.spectral_scan_restart_ena = atoi(optarg);
                break;

            case 'p':
                /* TODO: Validate input */
                config.spectral_scan_gc_ena = atoi(optarg);
                break;

            case 'P':
                pause = TRUE;
                break;

            case 'q':
                /* TODO: Validate input */
                config.spectral_scan_period = atoi(optarg);
                break;

            case 'r':
                /* TODO: Validate input */
                config.spectral_scan_bin_scale = atoi(optarg);
                break;

            case 's':
                /* TODO: Validate input */
                config.spectral_scan_rpt_mode = atoi(optarg);
                break;

            case 't':
                /* TODO: Validate input */
                config.spectral_scan_chn_mask = atoi(optarg);
                break;

            case 'u':
                /* TODO: Validate input */
                config.spectral_scan_init_delay = atoi(optarg);
                break;

            case 'v':
                /* TODO: Validate input */
                config.spectral_scan_noise_floor_ref = atoi(optarg);
                break;

            case 'x':
                dump_reg = TRUE;
                break;

            case 'h':
            case '?':
                display_help();
                return 0;

            default:
                fprintf(stderr, "Unrecognized option\n");
                display_help();
                return -1;
        }

        opt = getopt_long(argc, argv, optString, longOpts, &longIndex);
    }

    signal(SIGINT, signal_handler);
    signal(SIGCHLD, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGHUP, signal_handler);


    if ((config.logfile = fopen(config.logfile_name, "a")) == NULL) {
        perror("fopen");
        return -1;
    }

    /* init netlink connection to driver */
    config.netlink_fd = socket(PF_NETLINK, SOCK_RAW, NETLINK_ATHEROS);

    /* validate */
    if (config.netlink_fd < 0) {
        perror("netlink error");
        cleanup(&config);
        return -1;
    }
    /* init netlink socket */
    memset(&config.src_addr, 0, sizeof(config.src_addr));
    config.src_addr.nl_family  = PF_NETLINK;
    config.src_addr.nl_pid     = getpid();
    config.src_addr.nl_groups  = 1;

    if (bind(config.netlink_fd, (struct sockaddr*)&config.src_addr, sizeof(config.src_addr)) < 0) {
        perror("netlink bind error\n");
        cleanup(&config);
        return -1;
    }

    config.reg_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (config.reg_fd < 0) {
        perror("netlink bind error\n");
        cleanup(&config);
        return -1;
    }

#if CONFIGURE_SPECTRAL_VIA_DIRECT_REG_OPERATION
    /*
     * Temp hack to get logspectral working on QCA Main driver
     * Open the device, conigure and wait
     */
    dev = dev_init(dev_name);

    /* check if reg dump is required */
    if (dump_reg) {
        dump_spectral_register_params();
        return 0;
    }
    dw_configure_registers(&config);
#else  /* CONFIGURE_SPECTRAL_VIA_DIRECT_REG_OPERATION */
    configure_registers(&config);
#endif  /* CONFIGURE_SPECTRAL_VIA_DIRECT_REG_OPERATION */


    /* Tool hack: Helps in just configuration */
    if (pause) {
        while(1);
    }

    //Carry out the main processing
    return obtain_spectral_samples(&config);
}


#if CONFIGURE_SPECTRAL_VIA_DIRECT_REG_OPERATION
/*
 * Configure registers, but directly writing/reading
 * via PCI address space
 */

static int dw_configure_registers(logspectral_config_t* config)
{
     /* Editing note: Keep column alignment to help easy regex based changes */

    update_reg(dev, BB_SPECTRAL_SCAN,   SPECTRAL_SCAN_ENA,            0x0);
    update_reg(dev, BB_SPECTRAL_SCAN,   SPECTRAL_SCAN_ACTIVE,         0x0);

    update_reg(dev, BB_SPECTRAL_SCAN,   SPECTRAL_SCAN_ENA,            0x1);

    update_mac_reg(dev, MAC_PCU_RX_FILTER,  PHY_DATA,                  0x1);
    update_mac_reg(dev, MAC_PCU_RX_FILTER,  PROMISCUOUS,               0x0);


    write_reg(dev,  PHY_BB_WATCHDOG_CTRL_1_ADDRESS, 0x00080013);
    write_reg(dev,  PHY_BB_WATCHDOG_CTRL_2_ADDRESS, 0x700);
    /* Program the PHY ERROR mask, remove magic number */
    write_reg(dev, 0x28338, 0x50);

    update_reg(dev, BB_EXTENSION_RADAR, RADAR_LB_DC_CAP, 60);

    update_reg(dev, BB_SPECTRAL_SCAN,   SPECTRAL_SCAN_FFT_SIZE,       config->spectral_scan_fft_size);
    update_reg(dev, BB_SPECTRAL_SCAN,   SPECTRAL_SCAN_COUNT,          config->spectral_scan_count);
    update_reg(dev, BB_SPECTRAL_SCAN,   SPECTRAL_SCAN_PERIOD,         config->spectral_scan_period);

    update_reg(dev, BB_SPECTRAL_SCAN,   SPECTRAL_SCAN_PRIORITY,       config->spectral_scan_priority);
    update_reg(dev, BB_SPECTRAL_SCAN,   SPECTRAL_SCAN_RESTART_ENA,    config->spectral_scan_restart_ena);

    update_reg(dev, BB_SRCH_FFT_CTRL_1, SPECTRAL_SCAN_DBM_ADJ,        config->spectral_scan_dBm_adj);
    update_reg(dev, BB_SRCH_FFT_CTRL_1, SPECTRAL_SCAN_RPT_MODE,       config->spectral_scan_rpt_mode);
    update_reg(dev, BB_SRCH_FFT_CTRL_1, SPECTRAL_SCAN_PWR_FORMAT,     config->spectral_scan_pwr_format);
    update_reg(dev, BB_SPECTRAL_SCAN_2, SPECTRAL_SCAN_WB_RPT_MODE,    config->spectral_scan_wb_rpt_mode);
    update_reg(dev, BB_SPECTRAL_SCAN_2, SPECTRAL_SCAN_RSSI_RPT_MODE,  config->spectral_scan_rssi_rpt_mode);
    update_reg(dev, BB_SRCH_FFT_CTRL_1, SPECTRAL_SCAN_BIN_SCALE,      config->spectral_scan_bin_scale);
    update_reg(dev, BB_SRCH_FFT_CTRL_4, SPECTRAL_SCAN_CHN_MASK,       config->spectral_scan_chn_mask);

    update_reg(dev, BB_SPECTRAL_SCAN_2, SPECTRAL_SCAN_STR_BIN_THR,    config->spectral_scan_str_bin_thr);
    update_reg(dev, BB_SPECTRAL_SCAN_2, SPECTRAL_SCAN_NB_TONE_THR,    config->spectral_scan_nb_tone_thr);
    update_reg(dev, BB_SPECTRAL_SCAN_2, SPECTRAL_SCAN_INIT_DELAY,     config->spectral_scan_init_delay);
    update_reg(dev, BB_SPECTRAL_SCAN_2, SPECTRAL_SCAN_NOISE_FLOOR_REF,config->spectral_scan_noise_floor_ref); //default=0xa0 #0x96 for emulation due to the scaled down BW and the missing analog filter

    update_reg(dev, BB_SPECTRAL_SCAN_3, SPECTRAL_SCAN_RSSI_THR,       config->spectral_scan_rssi_thr);
    update_reg(dev, BB_SPECTRAL_SCAN,   SPECTRAL_SCAN_GC_ENA,         config->spectral_scan_gc_ena);

    update_reg(dev, BB_SPECTRAL_SCAN,   SPECTRAL_SCAN_ACTIVE,         0x1);
    //print_spectral_config(config);
    dump_spectral_register_params();
   return 0;
}

#endif  // CONFIGURE_SPECTRAL_VIA_DIRECT_REG_OPERATION

static int configure_registers(logspectral_config_t *config)
{
    /* Editing note: Keep column alignment to help easy regex based changes */

    regupdate(config, BB_SPECTRAL_SCAN,   SPECTRAL_SCAN_ENA,            0x0);
    regupdate(config, BB_SPECTRAL_SCAN,   SPECTRAL_SCAN_ACTIVE,         0x0);

    regupdate(config, BB_SPECTRAL_SCAN,   SPECTRAL_SCAN_ENA,            0x1);

  //regupdate(config, BB_MODES_SELECT,    DYN_OFDM_CCK_MODE,            0x0);
    macregupdate(config, MAC_PCU_RX_FILTER,  PHY_DATA,                  0x1);
    macregupdate(config, MAC_PCU_RX_FILTER,  PROMISCUOUS,               0x0);

  //regwrite(config,  BB_WATCHDOG_CTRL_1_ADDRESS, 0xfffffffd);
#if 0
    regwrite(config,  BB_WATCHDOG_CTRL_1_ADDRESS, 0x00080013);
    regwrite(config,  BB_WATCHDOG_CTRL_2_ADDRESS, 0x700);
#endif
    regwrite(config,  PHY_BB_WATCHDOG_CTRL_1_ADDRESS, 0x00080013);
    regwrite(config,  PHY_BB_WATCHDOG_CTRL_2_ADDRESS, 0x700);

    regupdate(config, BB_EXTENSION_RADAR, RADAR_LB_DC_CAP, 60);

    regupdate(config, BB_SPECTRAL_SCAN,   SPECTRAL_SCAN_FFT_SIZE,       config->spectral_scan_fft_size);
    regupdate(config, BB_SPECTRAL_SCAN,   SPECTRAL_SCAN_COUNT,          config->spectral_scan_count);
    regupdate(config, BB_SPECTRAL_SCAN,   SPECTRAL_SCAN_PERIOD,         config->spectral_scan_period);

    regupdate(config, BB_SPECTRAL_SCAN,   SPECTRAL_SCAN_PRIORITY,       config->spectral_scan_priority);
    regupdate(config, BB_SPECTRAL_SCAN,   SPECTRAL_SCAN_RESTART_ENA,    config->spectral_scan_restart_ena);

    regupdate(config, BB_SRCH_FFT_CTRL_1, SPECTRAL_SCAN_DBM_ADJ,        config->spectral_scan_dBm_adj);
    regupdate(config, BB_SRCH_FFT_CTRL_1, SPECTRAL_SCAN_RPT_MODE,       config->spectral_scan_rpt_mode);
    regupdate(config, BB_SRCH_FFT_CTRL_1, SPECTRAL_SCAN_PWR_FORMAT,     config->spectral_scan_pwr_format);
    regupdate(config, BB_SPECTRAL_SCAN_2, SPECTRAL_SCAN_WB_RPT_MODE,    config->spectral_scan_wb_rpt_mode);
    regupdate(config, BB_SPECTRAL_SCAN_2, SPECTRAL_SCAN_RSSI_RPT_MODE,  config->spectral_scan_rssi_rpt_mode);
    regupdate(config, BB_SRCH_FFT_CTRL_1, SPECTRAL_SCAN_BIN_SCALE,      config->spectral_scan_bin_scale);
    regupdate(config, BB_SRCH_FFT_CTRL_4, SPECTRAL_SCAN_CHN_MASK,       config->spectral_scan_chn_mask);

    regupdate(config, BB_SPECTRAL_SCAN_2, SPECTRAL_SCAN_STR_BIN_THR,    config->spectral_scan_str_bin_thr);
    regupdate(config, BB_SPECTRAL_SCAN_2, SPECTRAL_SCAN_NB_TONE_THR,    config->spectral_scan_nb_tone_thr);
    regupdate(config, BB_SPECTRAL_SCAN_2, SPECTRAL_SCAN_INIT_DELAY,     config->spectral_scan_init_delay);
    regupdate(config, BB_SPECTRAL_SCAN_2, SPECTRAL_SCAN_NOISE_FLOOR_REF,config->spectral_scan_noise_floor_ref); //default=0xa0 #0x96 for emulation due to the scaled down BW and the missing analog filter

    regupdate(config, BB_SPECTRAL_SCAN_3, SPECTRAL_SCAN_RSSI_THR,       config->spectral_scan_rssi_thr);
    regupdate(config, BB_SPECTRAL_SCAN,   SPECTRAL_SCAN_GC_ENA,         config->spectral_scan_gc_ena);

    regupdate(config, BB_SPECTRAL_SCAN,   SPECTRAL_SCAN_ACTIVE,         0x1);
}

static void signal_handler(int signal)
{
    switch (signal) {
        case SIGHUP:
        case SIGTERM:
        case SIGCHLD:
        case SIGINT:
            cleanup(&config);
            break;
        default:
            printf("%s:Unknown signal!", __func__);
            break;
    }
}

void cleanup(logspectral_config_t *config)
{
#if CONFIGURE_SPECTRAL_VIA_DIRECT_REG_OPERATION
    update_reg(dev, BB_SPECTRAL_SCAN, SPECTRAL_SCAN_ACTIVE, 0x0);
    usleep(20);
    update_reg(dev, BB_SPECTRAL_SCAN, SPECTRAL_SCAN_GC_ENA, 0x0);
#else
    regupdate(config, BB_SPECTRAL_SCAN,   SPECTRAL_SCAN_ACTIVE,         0x0);
    usleep(20);
    regupdate(config, BB_SPECTRAL_SCAN,   SPECTRAL_SCAN_ENA,            0x0);
#endif

    if (config->logfile != NULL) {
        fclose(config->logfile);
    }

    if (config->logfile2 != NULL) {
        fclose(config->logfile2);
    }

    if (config->netlink_fd) {
        close(config->netlink_fd);
    }

    if (config->reg_fd) {
        close(config->reg_fd);
    }

    //print_spectral_config(config);
    dump_spectral_register_params();

    exit(0);
}

int logspectral_config_init(logspectral_config_t *config)
{
    if (NULL == config) {
        return -1;
    }

    config->logfile = NULL;
    config->logfile2 = NULL;
    strncpy(config->logfile_name, "/tmp/SS_data.txt", sizeof (config->logfile_name));
    strncpy(config->logfile2_name, "/tmp/SS_data2.txt", sizeof (config->logfile2_name));
    strncpy(config->radio_ifname, "wifi0", sizeof(config->radio_ifname));
    config->spectral_scan_priority =      SPECTRAL_SCAN_PRIORITY_DEFAULT;
    config->spectral_scan_count =         SPECTRAL_SCAN_COUNT_DEFAULT;
    config->spectral_scan_dBm_adj =       SPECTRAL_SCAN_DBM_ADJ_DEFAULT;
    config->spectral_scan_pwr_format =    SPECTRAL_SCAN_PWR_FORMAT_DEFAULT;
    config->spectral_scan_fft_size =      SPECTRAL_SCAN_FFT_SIZE_DEFAULT;
    config->spectral_scan_rssi_rpt_mode = SPECTRAL_SCAN_RSSI_RPT_MODE_DEFAULT;
    config->spectral_scan_wb_rpt_mode =   SPECTRAL_SCAN_WB_RPT_MODE_DEFAULT;
    config->spectral_scan_str_bin_thr =   SPECTRAL_SCAN_STR_BIN_THR_DEFAULT;
    config->spectral_scan_nb_tone_thr =   SPECTRAL_SCAN_NB_TONE_THR_DEFAULT;
    config->spectral_scan_rssi_thr =      SPECTRAL_SCAN_RSSI_THR_DEFAULT;
    config->spectral_scan_restart_ena =   SPECTRAL_SCAN_RESTART_ENA_DEFAULT;
    config->spectral_scan_gc_ena =        SPECTRAL_SCAN_GC_ENA_DEFAULT;
    config->spectral_scan_period =        SPECTRAL_SCAN_PERIOD_DEFAULT;
    config->spectral_scan_bin_scale =     SPECTRAL_SCAN_BIN_SCALE_DEFAULT;
    config->spectral_scan_rpt_mode =      SPECTRAL_SCAN_RPT_MODE_DEFAULT;
    config->spectral_scan_chn_mask =      SPECTRAL_SCAN_CHN_MASK_DEFAULT;
    config->spectral_scan_init_delay =    SPECTRAL_SCAN_INIT_DELAY_DEFAULT;
    config->spectral_scan_noise_floor_ref = SPECTRAL_SCAN_NOISE_FLOOR_REF_DEFAULT;

    return 0;
}

static void hexdump(FILE *fp, unsigned char *buf, unsigned int len)
{
    while (len--) {
        fprintf(fp, "%02x", *buf++);
    }

    fprintf(fp, "\n");
}

static int process_spectral_adc_sample(logspectral_config_t *config)
{
    struct nlmsghdr  *nlh = NULL;
    /* XXX Change this approach if we go multi-threaded!! */
    static char buf[NLMSG_SPACE(SPECTRAL_MAX_TLV_SIDE)];
    int bytes_read = 0;
    uint8_t *tempbuf;
    spectral_adc_log_desc *desc;

    nlh = (struct nlmsghdr *)buf;

    memset(nlh, 0, NLMSG_SPACE(sizeof(SPECTRAL_MAX_TLV_SIDE)));
    nlh->nlmsg_len  = NLMSG_SPACE(sizeof(SPECTRAL_MAX_TLV_SIDE));
    nlh->nlmsg_pid  = getpid();
    nlh->nlmsg_flags = 0;

    bytes_read = recvfrom(config->netlink_fd,
                      buf,
                      sizeof(buf),
                      MSG_WAITALL,
                      NULL,
                      NULL);

    tempbuf = (uint8_t *)NLMSG_DATA(nlh);

#if SPECTRAL_DATA_IN_TLV_FORMAT
    /*
     * If the data is in TLV format, use the TLV processing
     * functions else use old PPDU format processing 
     */

    spectral_process_phyerr_data(tempbuf);
#else   /* SPECTRAL_DATA_IN_TLV_FORMAT */

    desc = (spectral_adc_log_desc *)tempbuf;

    switch(desc->type)
    {
        case SPECTRAL_ADC_LOG_TYPE_RX_PPDU_START:
            return process_rx_ppdu_start(config, desc->serial_num, (struct rx_ppdu_start*)(tempbuf + sizeof(spectral_adc_log_desc)));
            break;
        case SPECTRAL_ADC_LOG_TYPE_RX_PPDU_END:
            return process_rx_ppdu_end(config, desc->serial_num, (struct rx_ppdu_end*)(tempbuf + sizeof(spectral_adc_log_desc)));
            break;
        case SPECTRAL_ADC_LOG_TYPE_TLV:
            return process_phy_tlv(config, desc->serial_num, (void*)(tempbuf + sizeof(spectral_adc_log_desc)));
            break;
        default:
            fprintf(stderr, "Unknown log description type!\n");
            return -1;
    }
#endif  /* SPECTRAL_ADC_LOG_TYPE_TLV */

    return 0;
}

static int process_rx_ppdu_start(logspectral_config_t *config, uint32_t serial_num, struct rx_ppdu_start *ppdu_start)
{
    printf("\n===Rx PPDU Start. Sr no.%u ===\n", serial_num);
    printf("\nrssi_chain0_pri20 = %d, rssi_chain0_sec20 = %d, rssi_chain0_sec40 = %d, rssi_comb=%d\n", ppdu_start->rssi_chain0_pri20, ppdu_start->rssi_chain0_sec20, ppdu_start->rssi_chain0_sec40, ppdu_start->rssi_comb);
}

static int process_rx_ppdu_end(logspectral_config_t *config, uint32_t serial_num, struct rx_ppdu_end *ppdu_end)
{
    static uint32_t last_ts = 0;

    printf("\n===Rx PPDU End. Sr no.%u ===\n", serial_num);
    printf("\nTS = %u, BB Len = %d, Error code = %d, Delta time = %u\n", ppdu_end->tsf_timestamp, ppdu_end->bb_length, ppdu_end->phy_err_code, ppdu_end->tsf_timestamp - last_ts);
    last_ts = ppdu_end->tsf_timestamp;
}


static int process_phy_tlv(logspectral_config_t *config, uint32_t serial_num, void* buf)
{
    uint8_t *tlv;
    uint32_t *tlvheader;
    uint16_t tlvlen;
    uint8_t tlvtag;
    uint8_t signature;

    tlv = (uint8_t *)buf;

    tlvheader = (uint32_t *) tlv;
    signature = (tlvheader[0] >> 24) & 0xff;
    tlvlen = tlvheader[0] & 0x0000ffff;
    tlvtag = (tlvheader[0] >> 16) & 0xff;

    if (signature != 0xbb) {
        fprintf(stderr, "Invalid signature 0x%x! Hexdump follows\n", signature);
        hexdump(stderr, tlv, tlvlen + 4);
        return -1;
    }

    switch(tlvtag)
    {
        case TLV_TAG_SPECTRAL_SUMMARY_REPORT:
            process_spectral_summary_report(config, serial_num, tlv, tlvlen);
            break;
        case TLV_TAG_SEARCH_FFT_REPORT:
            process_search_fft_report(config, serial_num, tlv, tlvlen);
            break;
        case TLV_TAG_ADC_REPORT:
            process_adc_report(config, serial_num, tlv, tlvlen);
            break;
        default:
            fprintf(stderr, "Unknown TLV Tag ID 0x%x! Hexdump follows\n", tlvtag);
            hexdump(stderr, tlv, tlvlen + 4);
            return -1;
    }

    return 0;
}

static int obtain_spectral_samples(logspectral_config_t *config)
{
    int fdmax;
    int fd;
    fd_set  master;
    fd_set  read_fds;

    FD_ZERO(&master);
    FD_ZERO(&read_fds);

    /* Actually, there is no need for the select call right now. But we put this in
       for extensibility */
    FD_SET(config->netlink_fd, &master);
    fdmax = config->netlink_fd;

    for (;;) {

        read_fds = master;

        if (select(fdmax + 1, &read_fds, NULL, NULL, NULL) == -1) {
            continue;
        }

        for (fd = 0; fd <= fdmax; fd++) {
            if (FD_ISSET(fd, &read_fds)) {
                if (fd ==  config->netlink_fd) {
#if 0
                        if (process_spectral_adc_sample(config) < 0) {
                            cleanup(config);
                            return -1;
                        }
#endif
                        process_spectral_adc_sample(config);
                }
             }
        }
    }
}

void read_agc()
{
    int val;
    /* TODO : Change the magic number */
    printf("\nagc  = 0x%x\n", read_reg(dev, 0x29e00, &val));
}

void read_cca()
{
    int val;
    /* TODO : Change the magic number */
    printf("mincca  0   = 0x%x\n", read_reg(dev, 0x29e1c, &val));
    printf("mincca  1   = 0x%x\n", read_reg(dev, 0x2ae1c, &val));
    printf("mincca  2   = 0x%x\n", read_reg(dev, 0x2be1c, &val));

}
void read_bb_spectral_scan_1()
{
    int val;
    /* TODO : Change the magic number */
    printf("bb spectral 1  = 0x%x\n", read_reg(dev, 0x2a228, &val));
}

void read_bb_spectral_scan_2()
{
    int val;
    /* TODO : Change the magic number */
    printf("bb spectral 2  = 0x%x\n", read_reg(dev, 0x29e98, &val));
}
void read_bb_spectral_scan_3()
{
    int val;
    /* TODO : Change the magic number */
    printf("bb spectral 3  = 0x%x\n", read_reg(dev, 0x29e9c, &val));
}


void print_spectral_config(logspectral_config_t *config)
{
    if (NULL == config) {
        return;
    }

    printf("----------------- Current Spectral Configuration--------------------\n");
    printf("SPECTRAL_SCAN_PRIORITY_DEFAULT          = %d\n", (u_int32_t)config->spectral_scan_priority);
    printf("SPECTRAL_SCAN_COUNT_DEFAULT             = %d\n", (u_int32_t)config->spectral_scan_count);
    printf("SPECTRAL_SCAN_DBM_ADJ_DEFAULT           = %d\n", (u_int32_t)config->spectral_scan_dBm_adj);
    printf("SPECTRAL_SCAN_PWR_FORMAT_DEFAULT        = %d\n", (u_int32_t)config->spectral_scan_pwr_format);
    printf("SPECTRAL_SCAN_FFT_SIZE_DEFAULT          = %d\n", (u_int32_t)config->spectral_scan_fft_size);
    printf("SPECTRAL_SCAN_RSSI_RPT_MODE_DEFAULT     = %d\n", (u_int32_t)config->spectral_scan_rssi_rpt_mode);
    printf("SPECTRAL_SCAN_WB_RPT_MODE_DEFAULT       = %d\n", (u_int32_t)config->spectral_scan_wb_rpt_mode);
    printf("SPECTRAL_SCAN_STR_BIN_THR_DEFAULT       = %d\n", (u_int32_t)config->spectral_scan_str_bin_thr);
    printf("SPECTRAL_SCAN_NB_TONE_THR_DEFAULT       = %d\n", (u_int32_t)config->spectral_scan_nb_tone_thr);
    printf("SPECTRAL_SCAN_RSSI_THR_DEFAULT          = %d\n", (u_int32_t)config->spectral_scan_rssi_thr);
    printf("SPECTRAL_SCAN_RESTART_ENA_DEFAULT       = %d\n", (u_int32_t)config->spectral_scan_restart_ena);
    printf("SPECTRAL_SCAN_GC_ENA_DEFAULT            = %d\n", (u_int32_t)config->spectral_scan_gc_ena);
    printf("SPECTRAL_SCAN_PERIOD_DEFAULT            = %d\n", (u_int32_t)config->spectral_scan_period);
    printf("SPECTRAL_SCAN_BIN_SCALE_DEFAULT         = %d\n", (u_int32_t)config->spectral_scan_bin_scale);
    printf("SPECTRAL_SCAN_RPT_MODE_DEFAULT          = %d\n", (u_int32_t)config->spectral_scan_rpt_mode);
    printf("SPECTRAL_SCAN_CHN_MASK_DEFAULT          = %d\n", (u_int32_t)config->spectral_scan_chn_mask);
    printf("SPECTRAL_SCAN_INIT_DELAY_DEFAULT        = %d\n", (u_int32_t)config->spectral_scan_init_delay);
    printf("SPECTRAL_SCAN_NOISE_FLOOR_REF_DEFAULT   = %d\n", (u_int32_t)config->spectral_scan_noise_floor_ref);

    //read_agc();
    //read_cca();
    read_bb_spectral_scan_1();
    read_bb_spectral_scan_2();
    read_bb_spectral_scan_3();
    printf("----------------- Current Spectral Configuration--------------------\n");


    return ;
}

static void display_help()
{
    printf("\nlogspectral v"LOGSPECTRAL_VERSION": Log spectral data received from 11ac chipsets for debug purposes.\n"
           "\n"
           "Usage:\n"
           "\n"
           "logspectral [OPTIONS]\n"
           "\n"
           "OPTIONS:\n"
           "\n"
           "-D, --device_name\n"
           "    Name of device\n"
           "\n"
           "-f, --file\n"
           "    Name of file to log spectral FFT data to. Default: /tmp/SS_data.txt\n"
           "-g, --file2\n"
           "    Name of secondary file to log spectral FFT data to for spectral_scan_pwr_format=1. Default: /tmp/SS_data2.txt\n"
           "\n"
           "-i, --interface\n"
           "    Name of radio interface. Default: wifi0\n"
           "\n"
           "--spectral_scan_priority\n"
           "    Value of spectral_scan_priority. Default: "WSTR(SPECTRAL_SCAN_PRIORITY_DEFAULT)"\n"
           "\n"
           "--spectral_scan_count. Default: "WSTR(SPECTRAL_SCAN_COUNT_DEFAULT)"\n"
           "    Value of spectral_scan_count.\n"
           "\n"
           "--spectral_scan_dBm_adj. Default: "WSTR(SPECTRAL_SCAN_DBM_ADJ_DEFAULT)"\n"
           "    Value of spectral_scan_dBm_adj.\n"
           "\n"
           "--spectral_scan_pwr_format. Default: "WSTR(SPECTRAL_SCAN_PWR_FORMAT_DEFAULT)"\n"
           "    Value of spectral_scan_pwr_format.\n"
           "\n"
           "--spectral_scan_fft_size. Default: "WSTR(SPECTRAL_SCAN_FFT_SIZE_DEFAULT)"\n"
           "    Value of spectral_scan_fft_size.\n"
           "\n"
           "--spectral_scan_rssi_rpt_mode. Default: "WSTR(SPECTRAL_SCAN_RSSI_RPT_MODE_DEFAULT)"\n"
           "    Value of spectral_scan_rssi_rpt_mode.\n"
           "\n"
           "--spectral_scan_wb_rpt_mode. Default: "WSTR(SPECTRAL_SCAN_WB_RPT_MODE_DEFAULT)"\n"
           "    Value of spectral_scan_wb_rpt_mode.\n"
           "\n"
           "--spectral_scan_str_bin_thr. Default: "WSTR(SPECTRAL_SCAN_STR_BIN_THR_DEFAULT)"\n"
           "    Value of spectral_scan_str_bin_thr.\n"
           "\n"
           "--spectral_scan_nb_tone_thr. Default: "WSTR(SPECTRAL_SCAN_NB_TONE_THR_DEFAULT)"\n"
           "    Value of spectral_scan_nb_tone_thr.\n"
           "\n"
           "--spectral_scan_rssi_thr. Default: "WSTR(SPECTRAL_SCAN_RSSI_THR_DEFAULT)"\n"
           "    Value of spectral_scan_rssi_thr.\n"
           "\n"
           "--spectral_scan_restart_ena. Default: "WSTR(SPECTRAL_SCAN_RESTART_ENA_DEFAULT)"\n"
           "    Value of spectral_scan_restart_ena.\n"
           "\n"
           "--spectral_scan_gc_ena. Default: "WSTR(SPECTRAL_SCAN_GC_ENA_DEFAULT)"\n"
           "    Value of spectral_scan_gc_ena.\n"
           "\n"
           "--spectral_scan_period. Default: "WSTR(SPECTRAL_SCAN_PERIOD_DEFAULT)"\n"
           "    Value of spectral_scan_period.\n"
           "\n"
           "--spectral_scan_bin_scale. Default: "WSTR(SPECTRAL_SCAN_BIN_SCALE_DEFAULT)"\n"
           "    Value of spectral_scan_bin_scale.\n"
           "\n"
           "--spectral_scan_rpt_mode. Default: "WSTR(SPECTRAL_SCAN_RPT_MODE_DEFAULT)"\n"
           "    Value of spectral_scan_rpt_mode.\n"
           "\n"
           "--spectral_scan_chn_mask. Default: "WSTR(SPECTRAL_SCAN_CHN_MASK_DEFAULT)"\n"
           "    Value of spectral_scan_chn_mask.\n"
           "\n"
           "--spectral_scan_init_delay. Default: "WSTR(SPECTRAL_SCAN_INIT_DELAY_DEFAULT)"\n"
           "    Value of spectral_scan_init_delay.\n"
           "\n"
           "--spectral_scan_noise_floor_ref. Default: "WSTR(SPECTRAL_SCAN_NOISE_FLOOR_REF_DEFAULT)"\n"
           "    Value of spectral_scan_noise_floor_ref. Note that this is a signed 8-bit value.\n"
           "\n"
           "OTHER OPTIONS TO BE ADDED\n"
           "\n");
}

static uint32_t regread(logspectral_config_t *config, u_int32_t off)
{
    u_int32_t regdata;
    struct  ath_diag atd;

    strncpy(atd.ad_name, config->radio_ifname, sizeof (atd.ad_name));
    atd.ad_id = HAL_DIAG_REGREAD | ATH_DIAG_IN | ATH_DIAG_DYN;
    atd.ad_in_size = sizeof(off);
    atd.ad_in_data = (caddr_t) &off;
    atd.ad_out_size = sizeof(regdata);
    atd.ad_out_data = (caddr_t) &regdata;
    if (ioctl(config->reg_fd, SIOCGATHDIAG, &atd) < 0)
        err(1, "%s", atd.ad_name);

    return regdata;
}

static void regwrite(logspectral_config_t *config, uint32_t off, uint32_t value)
{
    HAL_DIAG_REGVAL regval;
    struct  ath_diag atd;

    regval.offset = (u_int) off ;
    regval.val = (u_int32_t) value;

    strncpy(atd.ad_name, config->radio_ifname, sizeof (atd.ad_name));
    atd.ad_id = HAL_DIAG_REGWRITE | ATH_DIAG_IN;
    atd.ad_in_size = sizeof(regval);
    atd.ad_in_data = (caddr_t) &regval;
    atd.ad_out_size = 0;
    atd.ad_out_data = NULL;
    if (ioctl(config->reg_fd, SIOCGATHDIAG, &atd) < 0)
        err(1, "%s", atd.ad_name);
}

static int process_spectral_summary_report(logspectral_config_t *config, uint32_t serial_num, uint8_t *tlv, uint32_t tlvlen)
{
    /* For simplicity, everything is defined as uint32_t (except one). Proper code will later use the right sizes. */

    /* For easy comparision between MDK team and OS team, the MDK script variable names have been used */

    uint32_t agc_mb_gain;
    uint32_t sscan_gidx;
    uint32_t agc_total_gain;
    uint32_t recent_rfsat;
    uint32_t ob_flag;
    uint32_t nb_mask;
    uint32_t peak_mag;
    int16_t peak_inx;

    uint32_t ss_summary_A;
    uint32_t ss_summary_B;
    uint32_t *pss_summary_A;
    uint32_t *pss_summary_B;

    printf("\n===Spectral Summary Report. Sr no. %u===\n", serial_num);

    if (tlvlen != 8) {
        fprintf(stderr, "Unexpected TLV length %d for Spectral Summary Report! Hexdump follows\n", tlvlen);
        hexdump(stderr, tlv, tlvlen + 4);
        return -1;
    }

    pss_summary_A = (uint32_t*)(tlv + 4);
    pss_summary_B = (uint32_t*)(tlv + 8);
    ss_summary_A = *pss_summary_A;
    ss_summary_B = *pss_summary_B;

    nb_mask = ((ss_summary_B >> 22) & 0xff);
    ob_flag = ((ss_summary_B >> 30) & 0x1);
    peak_inx = (ss_summary_B  & 0xfff);

    if (peak_inx > 2047) {
        peak_inx = peak_inx - 4096;
    }

    peak_mag = ((ss_summary_B >> 12) & 0x3ff);
    agc_mb_gain = ((ss_summary_A >> 24)& 0x7f);
    agc_total_gain = (ss_summary_A  & 0x3ff);
    sscan_gidx = ((ss_summary_A >> 16) & 0xff);
    recent_rfsat = ((ss_summary_B >> 31) & 0x1);

    printf("nb_mask=0x%.2x, ob_flag=%d, peak_index=%d, peak_mag=%d, agc_mb_gain=%d, agc_total_gain=%d, sscan_gidx=%d, recent_rfsat=%d\n", nb_mask, ob_flag, peak_inx, peak_mag, agc_mb_gain, agc_total_gain, sscan_gidx, recent_rfsat);
}

static int process_search_fft_report(logspectral_config_t *config, uint32_t serial_num, uint8_t *tlv, uint32_t tlvlen)
{
    int i;
    uint8_t fft_mag;
    int8_t signed_fft_mag;

    /* For simplicity, everything is defined as uint32_t (except one). Proper code will later use the right sizes. */
    /* For easy comparision between MDK team and OS team, the MDK script variable names have been used */
    uint32_t relpwr_db;
    uint32_t num_str_bins_ib;
    uint32_t base_pwr;
    uint32_t total_gain_info;

    uint32_t fft_chn_idx;
    int16_t peak_inx;
    uint32_t avgpwr_db;
    uint32_t peak_mag;

    uint32_t fft_summary_A;
    uint32_t fft_summary_B;
    uint32_t *pfft_summary_A;
    uint32_t *pfft_summary_B;

    printf("\n===Search FFT Report. Sr no.%u ===\n", serial_num);

    /* Relook this */
    if (tlvlen < 8) {
        fprintf(stderr, "Unexpected TLV length %d for Spectral Summary Report! Hexdump follows\n", tlvlen);
        hexdump(stderr, tlv, tlvlen + 4);
        return -1;
    }

    if ((config->logfile2 = fopen(config->logfile2_name, "w")) == NULL) {
        perror("fopen");
        return -1;
    }

    pfft_summary_A = (uint32_t*)(tlv + 4);
    pfft_summary_B = (uint32_t*)(tlv + 8);
    fft_summary_A = *pfft_summary_A;
    fft_summary_B = *pfft_summary_B;

    relpwr_db= ((fft_summary_B >>26) & 0x3f);
    num_str_bins_ib=fft_summary_B & 0xff;
    base_pwr = ((fft_summary_A >> 14) & 0x1ff);
    total_gain_info = ((fft_summary_A >> 23) & 0x1ff);

    fft_chn_idx= ((fft_summary_A >>12) & 0x3);
    peak_inx=fft_summary_A & 0xfff;

    if (peak_inx > 2047) {
        peak_inx = peak_inx - 4096;
    }

    avgpwr_db = ((fft_summary_B >> 18) & 0xff);
    peak_mag = ((fft_summary_B >> 8) & 0x3ff);

    printf("Base Power= 0x%x, Total Gain= %d, relpwr_db=%d, num_str_bins_ib=%d fft_chn_idx=%d peak_inx=%d avgpwr_db=%d peak_mag=%d\n", base_pwr, total_gain_info, relpwr_db, num_str_bins_ib, fft_chn_idx, peak_inx, avgpwr_db, peak_mag);

    printf("MAC_PCU_RX_CLEAR_CNT=%u\n", regread(config, MAC_PCU_RX_CLEAR_CNT_ADDRESS));
    printf("MAC_PCU_RX_FRAME_CNT=%u\n", regread(config, MAC_PCU_RX_FRAME_CNT_ADDRESS));
    printf("MAC_PCU_TX_FRAME_CNT=%u\n", regread(config, MAC_PCU_TX_FRAME_CNT_ADDRESS));

    for (i = 0; i < (tlvlen-8); i++){
        fft_mag = tlv[12 + i];
        printf("%d %d, ", i, fft_mag);
        fprintf(config->logfile,"%u ", fft_mag);
        if ((config->spectral_scan_dBm_adj==1) && (config->spectral_scan_pwr_format==1)){
            signed_fft_mag = fft_mag - 256;
            fprintf(config->logfile2,"%d %d \n", i, signed_fft_mag);
        } else {
            fprintf(config->logfile2,"%d %d \n", i, fft_mag);
        }
    }

    fclose(config->logfile2);
    config->logfile2 = NULL;

    printf("\n");
}

static int process_adc_report(logspectral_config_t *config, uint32_t serial_num, uint8_t *tlv, uint32_t tlvlen)
{
    printf("%s: Use application logadc for detailed logging. Hexdump follows:\n", __func__);
    hexdump(stdout, tlv, tlvlen);
    printf("\n");
}

