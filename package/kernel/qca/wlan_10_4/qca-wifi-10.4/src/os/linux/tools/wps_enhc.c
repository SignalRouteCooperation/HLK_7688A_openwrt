/*
 * Copyright (c) 2015 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <linux/wireless.h>
#include <dirent.h>
#include <sys/wait.h>
#include <signal.h>
#include <semaphore.h>
#include <time.h>
#include <sys/ioctl.h>


#define WPS_CFG_LN_SZ      100
#define WPS_NORMAL_DUR     120
#define LOGLEVEL_MASK      0x7f
#define LOGLEVEL_SHIFT     0
#define LOGTIME_MASK       0x80
#define LOGTIME_SHIFT      7
#define SKIP_AP_IF_STA_DC  0x1
#define TRY_STA_ALW        0x2


/* Log levels */
enum wps_loglevel_t { WPS_DBG, WPS_INF, WPS_WARN, WPS_ERR, WPS_FTL, WPS_MSK };

/* Interface types possible for vap */
enum wps_intf_type { WPS_UNUSED, WPS_MANAGED, WPS_MASTER };

/* Items of the config list */
struct wps_cfg_t {
    char vap[IFNAMSIZ];
    int start_time;
    int duration;
    char radio[IFNAMSIZ];
    enum wps_intf_type type;
    int wps_active;
    int wid_act;
    int wid_deact;
    unsigned int option;
    struct wps_cfg_t *radio_ptr;
    struct wps_cfg_t *next_ptr;
};

/* Items of the option list */
struct wps_opt_t {
    char radio[IFNAMSIZ];
    unsigned int option;
    struct wps_opt_t *next_ptr;
};

/* Items of the work list */
struct wps_work_t {
    int time;
    int activation;
    struct wps_cfg_t *cfg_ptr;
};


/* Which logs are enabled */
static enum wps_loglevel_t loglevel = WPS_WARN | LOGTIME_MASK;

/* Name of file which has the config */
static const char *config_file = "/var/run/wifi-wps-enhc-extn.conf";

/* Name of file in which completion indication is passed */
static const char *cmpl_file = "/var/run/wifi-wps-enhc-extn.done";

/* Whether to daemonize, if so the file name to store the PID of daemon */
static int background;
static const char *pid_file;

/* Whether logging to file is enabled, if yes the file name and file handle */
static int logfile;
static const char *log_file;
static FILE *logf;

/* Number of supported radio's (and vap's)  */
static int num_radio = 1, num_vap = 2;

/* Signal related handling */
static int terminate_on_signal;
static sem_t sem;
static void sig_handler(int);


/*
 * Prefix the debug messages with their level.
 */
static const char *loglvl_to_string(enum wps_loglevel_t level)
{
    switch (level) {
    case WPS_DBG:
        return "debug";
    case WPS_INF:
        return "information";
    case WPS_WARN:
        return "warning";
    case WPS_ERR:
        return "error";
    case WPS_FTL:
        return "fatal";
    default:
        return "unknown";
    }
}

/*
 * Print debug messages, based on loglevel selected,
 * on console or in a file.
 */
static void log_msg(int level, const char *fmt, ...)
{
    va_list msg;

    if (level >= ((LOGLEVEL_MASK & loglevel) >> LOGLEVEL_SHIFT)) {
        va_start(msg, fmt);
        if (!logf) {
            printf("[%s] ", loglvl_to_string(level));
            if ((loglevel & LOGTIME_MASK) >> LOGTIME_SHIFT)
                printf("[%04u] ", (unsigned int)time(NULL) % 10000);
            vprintf(fmt, msg);
        } else {
            fprintf(logf, "[%s] ", loglvl_to_string(level));
            if ((loglevel & LOGTIME_MASK) >> LOGTIME_SHIFT)
                fprintf(logf, "[%04u] ", (unsigned int) time(NULL) % 10000);
            vfprintf(logf, fmt, msg);
        }
        va_end(msg);
    }
}

/*
 * Process the command line options.
 *
 * The config file can be overridden using -f <file>.
 * The configuration is read from <file>.
 *
 * The logs to be enabled can be overridden using -d <level>.
 * All logs having level >= <level> get enabled.
 *
 * The application will be run as a daemon if -b <file> is specified.
 * The PID of the daemon is written in <file>.
 *
 * The logs will be dumped to a file instead of console if -l <file> 
 * is specified. The logs will get written to <file>.
 * 
 * The number of supported radio's (and vap's) can be overridden
 * using -n <radio>. The number of radio's will be set to <radio> and
 * number of vap's will be set to 2 * <radio>. We support having
 * 1 AP vap + 1 STA vap per radio.
 *
 * The completion file can be overridden using -c <file>.
 * The completion indication is read from <file>.
 */
static void parse_args(int argc, char *argv[])
{
    int i = 0;

    argc--;
    i++;

    while (argc) {
        /* Config file */
        if (!strcmp(argv[i], "-f")) {
            if (argc > 1) {
                config_file = argv[i+1];
                argc--;
                i++;
            }
        /* Log Level */
        } else if (!strcmp(argv[i], "-d")) {
            if (argc > 1) {
                loglevel = strtoul(argv[i+1], NULL, 0);
                argc--;
                i++;
            }
        /* Backgroud */
        } else if (!strcmp(argv[i], "-b")) {
            if (argc > 1) {
                pid_file = argv[i+1];
                background = 1;
                argc--;
                i++;
            }
        /* Log File */
        } else if (!strcmp(argv[i], "-l")) {
            if (argc > 1) {
                log_file = argv[i+1];
                logfile = 1;
                argc--;
                i++;
            }
        /* Num radio's (and vap's) */
        } else if (!strcmp(argv[i], "-n")) {
            if (argc > 1) {
                num_radio = strtoul(argv[i+1], NULL, 0);
                num_vap = num_radio * 2;
                argc--;
                i++;
            }
        /* Completion File */
        } else if (!strcmp(argv[i], "-c")) {
            if (argc > 1) {
                cmpl_file = argv[i+1];
                argc--;
                i++;
            }
        }

        argc--;
        i++;
    }
}

/*
 * Start WPS PBC on an AP vap.
 *
 * This is derived from the wps-hotplug.sh script 
 * and is coupled with the QCA qsdk runtime environment.
 * It may need porting if the runtime environment is
 * different from QCA qsdk.
 */
static int start_wps_on_ap(char *vap, char *radio)
{
    char name[50];
#ifndef TEST
    struct dirent *d_ent;
    DIR *d;
#endif
    pid_t pid;
    int status;

#ifndef TEST
    name[0] = '\0';
    snprintf(name, sizeof(name), "/var/run/hostapd-%s", radio);
    log_msg(WPS_DBG, "opening directory %s\n", name);
    d = opendir(name);
    if (!d) {
        log_msg(WPS_WARN, "can't open directory %s\n", name);
        return -1;
    }

    name[0] = '\0';
    d_ent = readdir(d);
    while (d_ent) {
        if (!strncmp(d_ent->d_name, vap, IFNAMSIZ)) {
            snprintf(name, sizeof(name), "/var/run/hostapd-%s", radio);
            break;
        }
        d_ent = readdir(d);
    }
    closedir(d);
#else
    snprintf(name, sizeof(name), "/var/run/hostapd-%s", radio);
#endif

    if (name[0] != '\0') {
        char *const args[] = {
                   "hostapd_cli", "-i", vap, "-p", name, "wps_pbc", NULL };

        pid = fork();

        switch (pid) {
        case 0: // Child
#ifdef TEST
            // if (execve("/local2/mnt/workspace2/"
            //            "anirban/hostapd_cli", args, NULL) < 0) {}
            if (execvp("/local2/mnt/workspace2/"
                       "anirban/hostapd_cli", args) < 0) {
                log_msg(WPS_WARN, "execvp failed (errno "
                        "%d %s)\n", errno, strerror(errno));
            }
#else
            // if (execve("/usr/sbin/hostapd_cli", args, NULL) < 0) {}
            if (execvp("/usr/sbin/hostapd_cli", args) < 0) {
                log_msg(WPS_WARN, "exexecvp failed (errno "
                        "%d %s)\n", errno, strerror(errno));
            }
#endif

            /* Shouldn't come here if execvp succeeds */
            exit(EXIT_FAILURE);

        case -1:
            log_msg(WPS_WARN, "fork failed (errno "
                    "%d %s)\n", errno, strerror(errno));
            return -1;

        default: // Parent
            if (wait(&status) < 0) {
                log_msg(WPS_WARN, "wait failed (errno "
                        "%d %s)\n", errno, strerror(errno));
                return -1;
            }

            if (!WIFEXITED(status)
                || WEXITSTATUS(status) == EXIT_FAILURE) {
                log_msg(WPS_WARN, "child exited with error %d\n",
                !WIFEXITED(status) ? -1 : WEXITSTATUS(status));
                return -1;
            }

            return 0;
        }
    } else {
        log_msg(WPS_WARN, "file %s not found\n", vap);
        return -1;
    }
}

/*
 * Start WPS PBC on a STA vap.
 *
 * This is derived from the wps-supplicant-hotplug.sh script 
 * and is coupled with the QCA qsdk runtime environment.
 * It may need porting if the runtime environment is
 * different from QCA qsdk.
 */
static int start_wps_on_sta(char *vap, char *radio)
{
    char name[50];
    char name1[50];
#ifndef TEST
    FILE *f;
    struct dirent *d_ent;
    DIR *d;
#endif
    pid_t pid;
    int status;
    int pid_file_exist = 0;

    name1[0] = '\0';
    snprintf(name1, sizeof(name1), "/var/run/wps-hotplug-%s.pid", vap);

#ifndef TEST
    name[0] = '\0';
    snprintf(name, sizeof(name), "/var/run/wpa_supplicant-%s", vap);
    log_msg(WPS_DBG, "opening directory %s\n", name);
    d = opendir(name);
    if (!d) {
        log_msg(WPS_WARN, "can't open directory %s\n", name);
        return -1;
    }

    name[0] = '\0';
    d_ent = readdir(d);
    while (d_ent) {
        if (!strncmp(d_ent->d_name, vap, IFNAMSIZ)) {
            snprintf(name, sizeof(name), "/var/run/wpa_supplicant-%s", vap);
            break;
        }
        d_ent = readdir(d);
    }
    closedir(d);
#else
    snprintf(name, sizeof(name), "/var/run/wpa_supplicant-%s", vap);
#endif

    if (name[0] != '\0') {
        char *const args[] = {
                   "wpa_cli", "-p", name, "wps_pbc", NULL };
        char *const args1[] = {
                   "wpa_cli", "-p", name, "-a",
                   "/lib/wifi/wps-supplicant-update-uci",
                   "-P", name1, "-B", NULL };

        pid = fork();

        switch (pid) {
        case 0: // Child
#ifdef TEST
            // if (execve("/local2/mnt/workspace2/"
            //            "anirban/wpa_cli", args, NULL) < 0) {}
            if (execvp("/local2/mnt/workspace2/"
                       "anirban/wpa_cli", args) < 0) {
                log_msg(WPS_WARN, "execvp failed (errno "
                        "%d %s)\n", errno, strerror(errno));
            }
#else
            // if (execve("/usr/sbin/wpa_cli", args, NULL) < 0) {}
            if (execvp("/usr/sbin/wpa_cli", args) < 0) {
                log_msg(WPS_WARN, "exexecvp failed (errno "
                        "%d %s)\n", errno, strerror(errno));
            }
#endif

            /* Shouldn't come here if execvp succeeds */
            exit(EXIT_FAILURE);

        case -1:
            log_msg(WPS_WARN, "fork failed (errno "
                    "%d %s)\n", errno, strerror(errno));
            return -1;

        default: // Parent
            if (wait(&status) < 0) {
                log_msg(WPS_WARN, "wait failed (errno "
                        "%d %s)\n", errno, strerror(errno));
                return -1;
            }

            if (!WIFEXITED(status)
                || WEXITSTATUS(status) == EXIT_FAILURE) {
                log_msg(WPS_WARN, "child exited with error %d\n",
                !WIFEXITED(status) ? -1 : WEXITSTATUS(status));
                return -1;
            }

#ifndef TEST
            if ((f = fopen(name1, "r")) == NULL) {
                log_msg(WPS_DBG, "pid file not found vap %s\n", vap);
            } else {
                pid_file_exist = 1;
                fclose(f);
            }
#endif

            if (pid_file_exist)
                return 0;

            pid = fork();

            switch (pid) {
            case 0: // Child
#ifdef TEST
                // if (execve("/local2/mnt/workspace2/"
                //            "anirban/wpa_cli", args1, NULL) < 0) {}
                if (execvp("/local2/mnt/workspace2/"
                           "anirban/wpa_cli", args1) < 0) {
                    log_msg(WPS_WARN, "execvp failed (errno "
                            "%d %s)\n", errno, strerror(errno));
                }
#else
                // if (execve("/usr/sbin/wpa_cli", args1, NULL) < 0) {}
                if (execvp("/usr/sbin/wpa_cli", args1) < 0) {
                    log_msg(WPS_WARN, "exexecvp failed (errno "
                            "%d %s)\n", errno, strerror(errno));
                }
#endif

                /* Shouldn't come here if execvp succeeds */
                exit(EXIT_FAILURE);

            case -1:
                log_msg(WPS_WARN, "fork failed (errno "
                        "%d %s)\n", errno, strerror(errno));
                return -1;

            default: // Parent
                if (wait(&status) < 0) {
                    log_msg(WPS_WARN, "wait failed (errno "
                            "%d %s)\n", errno, strerror(errno));
                    return -1;
                }

                if (!WIFEXITED(status)
                    || WEXITSTATUS(status) == EXIT_FAILURE) {
                    log_msg(WPS_WARN, "child exited with error %d\n",
                    !WIFEXITED(status) ? -1 : WEXITSTATUS(status));
                    return -1;
                }

                return 0;
            }
        }
    } else {
        log_msg(WPS_WARN, "file %s not found\n", vap);
        return -1;
    }
}

/*
 * Stop WPS PBC on an AP vap.
 *
 * This is coupled with the QCA qsdk runtime environment.
 * It may need porting if the runtime environment
 * is different from QCA qsdk.
 */
static int stop_wps_on_ap(char *vap, char *radio)
{
    char name[50];
#ifndef TEST
    struct dirent *d_ent;
    DIR *d;
#endif
    pid_t pid;
    int status;

#ifndef TEST
    name[0] = '\0';
    snprintf(name, sizeof(name), "/var/run/hostapd-%s", radio);
    log_msg(WPS_DBG, "opening directory %s\n", name);
    d = opendir(name);
    if (!d) {
        log_msg(WPS_WARN, "can't open directory %s\n", name);
        return -1;
    }

    name[0] = '\0';
    d_ent = readdir(d);
    while (d_ent) {
        if (!strncmp(d_ent->d_name, vap, IFNAMSIZ)) {
            snprintf(name, sizeof(name), "/var/run/hostapd-%s", radio);
            break;
        }
        d_ent = readdir(d);
    }
    closedir(d);
#else
    snprintf(name, sizeof(name), "/var/run/hostapd-%s", radio);
#endif

    if (name[0] != '\0') {
        char *const args[] = {
                   "hostapd_cli", "-i", vap, "-p", name, "wps_cancel", NULL };

        pid = fork();

        switch (pid) {
        case 0: // Child
#ifdef TEST
            // if (execve("/local2/mnt/workspace2/"
            //            "anirban/hostapd_cli", args, NULL) < 0) {}
            if (execvp("/local2/mnt/workspace2/"
                       "anirban/hostapd_cli", args) < 0) {
                log_msg(WPS_WARN, "execvp failed (errno "
                        "%d %s)\n", errno, strerror(errno));
            }
#else
            // if (execve("/usr/sbin/hostapd_cli", args, NULL) < 0) {}
            if (execvp("/usr/sbin/hostapd_cli", args) < 0) {
                log_msg(WPS_WARN, "execvp failed (errno "
                        "%d %s)\n", errno, strerror(errno));
            }
#endif

            /* Shouldn't come here if execvp succeeds */
            exit(EXIT_FAILURE);

        case -1:
            log_msg(WPS_WARN, "fork failed (errno "
                    "%d %s)\n", errno, strerror(errno));
            return -1;

        default: // Parent
            if (wait(&status) < 0) {
                log_msg(WPS_WARN, "wait failed (errno "
                        "%d %s)\n", errno, strerror(errno));
                return -1;
            }

            if (!WIFEXITED(status)
                || WEXITSTATUS(status) == EXIT_FAILURE) {
                log_msg(WPS_WARN, "child exited with error %d\n",
                !WIFEXITED(status) ? -1 : WEXITSTATUS(status));
                return -1;
            }

            return 0;
        }
    } else {
        log_msg(WPS_WARN, "file %s not found\n", vap);
        return -1;
    }
}

/*
 * Stop WPS PBC on a STA vap.
 *
 * This is coupled with the QCA qsdk runtime environment.
 * It may need porting if the runtime environment is
 * different from QCA qsdk.
 */
static int stop_wps_on_sta(char *vap, char *radio)
{
    char name[50];
    char name1[50];
#ifndef TEST
    FILE *f;
    struct dirent *d_ent;
    DIR *d;
#endif
    pid_t pid;
    int status;
    pid_t pid_from_file = 0;

    name1[0] = '\0';
    snprintf(name1, sizeof(name1), "/var/run/wps-hotplug-%s.pid", vap);

#ifndef TEST
    name[0] = '\0';
    snprintf(name, sizeof(name), "/var/run/wpa_supplicant-%s", vap);
    log_msg(WPS_DBG, "opening directory %s\n", name);
    d = opendir(name);
    if (!d) {
        log_msg(WPS_WARN, "can't open directory %s\n", name);
        return -1;
    }

    name[0] = '\0';
    d_ent = readdir(d);
    while (d_ent) {
        if (!strncmp(d_ent->d_name, vap, IFNAMSIZ)) {
            snprintf(name, sizeof(name), "/var/run/wpa_supplicant-%s", vap);
            break;
        }
        d_ent = readdir(d);
    }
    closedir(d);
#else
    snprintf(name, sizeof(name), "/var/run/wpa_supplicant-%s", vap);
#endif

    if (name[0] != '\0') {
        char *const args[] = {
                   "wpa_cli", "-p", name, "wps_cancel", NULL };

        pid = fork();

        switch (pid) {
        case 0: // Child
#ifdef TEST
            // if (execve("/local2/mnt/workspace2/"
            //            "anirban/wpa_cli", args, NULL) < 0) {}
            if (execvp("/local2/mnt/workspace2/"
                       "anirban/wpa_cli", args) < 0) {
                log_msg(WPS_WARN, "execvp failed (errno "
                        "%d %s)\n", errno, strerror(errno));
            }
#else
            // if (execve("/usr/sbin/wpa_cli", args, NULL) < 0) {}
            if (execvp("/usr/sbin/wpa_cli", args) < 0) {
                log_msg(WPS_WARN, "execvp failed (errno "
                        "%d %s)\n", errno, strerror(errno));
            }
#endif

            /* Shouldn't come here if execvp succeeds */
            exit(EXIT_FAILURE);

        case -1:
            log_msg(WPS_WARN, "fork failed (errno "
                    "%d %s)\n", errno, strerror(errno));
            return -1;

        default: // Parent
            if (wait(&status) < 0) {
                log_msg(WPS_WARN, "wait failed (errno "
                        "%d %s)\n", errno, strerror(errno));
                return -1;
            }

            if (!WIFEXITED(status)
                || WEXITSTATUS(status) == EXIT_FAILURE) {
                log_msg(WPS_WARN, "child exited with error %d\n",
                !WIFEXITED(status) ? -1 : WEXITSTATUS(status));
                return -1;
            }

#ifndef TEST
            if ((f = fopen(name1, "r")) == NULL) {
                log_msg(WPS_INF, "pid file not found vap %s\n", vap);
            } else {
                fscanf(f, "%u", &pid_from_file);
                fclose(f);
            }
#endif

            if (pid_from_file) {
                log_msg(WPS_DBG, "killing pid %u with "
                        "signal %d\n", pid_from_file, SIGTERM);
                if (kill(pid_from_file, SIGTERM) < 0) {
                    log_msg(WPS_WARN, "kill %u failed (errno "
                            "%d %s)\n", pid_from_file, errno, strerror(errno));
                }
            }

            return 0;
        }
    } else {
        log_msg(WPS_WARN, "file %s not found\n", vap);
        return -1;
    }
}

/*
 * If a STA vap is connected or not.
 * This is used to determine things like whether to do
 * WPS on this STA vap, whether to do WPS on the AP
 * vap of the same radio etc.
 */
static int is_sta_connected(int sock, const char *vap)
{
#ifdef TEST
    return 1;
#else
    struct ifreq ir;

    memset(&ir, 0, sizeof(ir));
    strncpy(ir.ifr_name, vap, IFNAMSIZ);
    if (ioctl(sock, SIOCGIFFLAGS, &ir) < 0) {
        log_msg(WPS_WARN, "ioctl SIOCGIFFLAGS failed "
                "(errno %d %s)\n", errno, strerror(errno));
        return -1;
    }

    log_msg(WPS_DBG, "station vap %s flags %s and %s\n",
            vap, ir.ifr_flags & IFF_UP ? "up" : "not up",
            ir.ifr_flags & IFF_RUNNING ? "running" : "not running");
    return ((ir.ifr_flags & IFF_UP) && (ir.ifr_flags & IFF_RUNNING)) ? 1 : 0;
#endif
}

/*
 * Check whether the vap interface is in AP mode or STA mode.
 * And, make sure sure the radio interface is valid.
 */
static int find_interface_type(int sock, const char *vap, const char *radio)
{
#ifdef TEST
    if (strncmp(radio, "wifi0", IFNAMSIZ) && strncmp(radio, "wifi1", IFNAMSIZ))
        return -1;
    if ((!strncmp(vap, "ath0", IFNAMSIZ)) ||
        (!strncmp(vap, "ath1", IFNAMSIZ)) ||
        (!strncmp(vap, "ath02", IFNAMSIZ)) ||
        (!strncmp(vap, "ath12", IFNAMSIZ))) {
        return WPS_MASTER;
    }
    if ((!strncmp(vap, "ath01", IFNAMSIZ)) ||
        (!strncmp(vap, "ath11", IFNAMSIZ)) ||
        (!strncmp(vap, "ath03", IFNAMSIZ)) ||
        (!strncmp(vap, "ath13", IFNAMSIZ))) {
        return WPS_MANAGED;
    }
    return -1;
#else
    struct iwreq wr;
    struct ifreq ir;

    memset(&ir, 0, sizeof(ir));
    strncpy(ir.ifr_name, radio, IFNAMSIZ);
    if (ioctl(sock, SIOCGIFFLAGS, &ir) < 0) {
        log_msg(WPS_WARN, "ioctl SIOCGIFFLAGS failed "
                "(errno %d %s)\n", errno, strerror(errno));
        return -1;
    }

    memset(&wr, 0, sizeof(wr));
    strncpy(wr.ifr_name, vap, IFNAMSIZ);
    if (ioctl(sock, SIOCGIWMODE, &wr) < 0) {
        log_msg(WPS_WARN, "ioctl SIOCGIWMODE failed "
                "(errno %d %s)\n", errno, strerror(errno));
        return -1;
    }

    if (wr.u.mode == IW_MODE_INFRA)
        return WPS_MANAGED;

    if (wr.u.mode == IW_MODE_MASTER)
        return WPS_MASTER;

    log_msg(WPS_WARN, "mode is neither AP nor STA %d\n", wr.u.mode);
    return -1;
#endif
}

/*
 * Check the config's passed. And, drop the config items
 * which fail to pass the required criteria.
 */
static int validate_config(struct wps_cfg_t **p_head)
{
    struct wps_cfg_t *head = *p_head;
    struct wps_cfg_t *trav, *prev, *next;
    struct wps_cfg_t *trav1, *prev1, *next1;
    int vaps = 0, radios = 0;

    trav = head;
    prev = NULL;
    while (trav) {
        next = trav->next_ptr;

        /* Validate interface type */
        if (trav->type != WPS_MANAGED && trav->type != WPS_MASTER) {
            log_msg(WPS_INF, "%s: %s: interface type %d isn't "
                    "valid\n", trav->vap, trav->radio, trav->type);
            goto next_cfg;
        }

        /* Validate start time */
        if (trav->start_time < 0 ||
            trav->start_time >= WPS_NORMAL_DUR * num_radio) {
            log_msg(WPS_INF, "%s: %s: start time %d isn't "
                    "valid\n", trav->vap, trav->radio, trav->start_time);
            goto next_cfg;
            /*
            log_msg(WPS_WARN, "%s: %s: changing start time "
                    "from %d to %d, duration is %d\n", trav->vap,
                    trav->radio, trav->start_time, 0, trav->duration);
            trav->start_time = 0;
            */
        }

        /* Validate duration */
        if (trav->start_time + trav->duration <= 0 ||
            trav->start_time + trav->duration > WPS_NORMAL_DUR * num_radio) {
            log_msg(WPS_INF, "%s: %s: duration %d isn't "
                    "valid\n", trav->vap, trav->radio, trav->duration);
            goto next_cfg;
            /*
            log_msg(WPS_WARN, "%s: %s: changing duration "
                    "from %d to %d, start time is %d\n", trav->vap,
                    trav->radio, trav->duration, (WPS_NORMAL_DUR *
                    num_radio) - trav->start_time, trav->start_time);
            trav->duration = (WPS_NORMAL_DUR * num_radio) - trav->start_time;
            */
        }

        /* Validate vap */
        trav1 = next;
        while (trav1) {
            /* One config for one vap */
            if (!strncmp(trav->vap, trav1->vap, IFNAMSIZ)) {
                log_msg(WPS_INF, "%s: %s: vap occurs more "
                        "than once\n", trav->vap, trav->radio);
                goto next_cfg;
            }
            trav1 = trav1->next_ptr;
        }

        vaps++;
        radios++; // Will be adjusted later for links
        prev = trav;
        trav = next;
        continue;

next_cfg:
        if (prev == NULL)
            head = trav->next_ptr;
        else
            prev->next_ptr = trav->next_ptr;
        free(trav);
        trav = next;
    }

    trav = head;
    while (trav) {
        /* Validate radio */
        prev1 = trav;
        trav1 = trav->next_ptr;
        while (trav1) {
            next1 = trav1->next_ptr;

            /* Two configs for one radio, if the interface types aren't same */
            if (!strncmp(trav->radio, trav1->radio, IFNAMSIZ)) {
                if (trav->type != trav1->type && !trav->radio_ptr) {
                    trav->radio_ptr = trav1;
                    trav1->radio_ptr = trav;
                    radios--; // Adjustment for links
                    if (trav->option != trav1->option) {
                        log_msg(WPS_FTL, "same radio different options "
                                "%x for %s and %x for %s\n", trav->option,
                                trav->vap, trav1->option, trav1->vap);
                    }
                } else {
                    if (trav->radio_ptr && trav->radio_ptr == trav1) {
                        log_msg(WPS_FTL, "already same radio "
                                "is linked, but to link is to same entry\n");
                    }
                    log_msg(WPS_INF, "%s: %s: radio occurs "
                            "more than twice or twice with same "
                            "interface type\n", trav1->vap, trav1->radio);
                    prev1->next_ptr = trav1->next_ptr;
                    free(trav1);
                    trav1 = next1;
                    vaps--;
                    radios--;
                    continue;
                }
            }

            prev1 = trav1;
            trav1 = next1;
        }

        trav = trav->next_ptr;
    }

    if (vaps > num_vap || radios > num_radio) {
        log_msg(WPS_ERR, "vaps %d, max vap %d, radios %d, max "
                "radio %d\n", vaps, num_vap, radios, num_radio);
        return -1;
    }

    *p_head = head;
    while (head) {
        log_msg(WPS_DBG, "vap:%s start_time:%d duration:%d radio:%s "
                "intf_type:%d same_radio_vap:%s option:%x\n", head->vap,
                head->start_time, head->duration, head->radio, head->type,
                head->radio_ptr ? head->radio_ptr->vap : "-", head->option);
        head = head->next_ptr;
    }
    return vaps;
}

/*
 * Handle passing of the WPS duration of cfg1 to cfg2.
 */
static int pass_wps_duration(struct wps_work_t *work, int items, int sta,
                                struct wps_cfg_t *cfg1, struct wps_cfg_t *cfg2)
{
    int x, ret = 0, i;

    if (!cfg2)
        return ret;

    if (cfg1->wid_act < 0 || cfg1->wid_act >= items ||
        cfg1->wid_deact < 0 || cfg1->wid_deact >= items ||
        cfg2->wid_act < 0 || cfg2->wid_act >= items ||
        cfg2->wid_deact < 0 || cfg2->wid_deact >= items) {
        log_msg(WPS_FTL, "bad id's %d %d %d %d (%d %d)\n", cfg1->wid_act,
                cfg1->wid_deact, cfg2->wid_act, cfg2->wid_deact, 0, items);
    }

    /* Can't adjust if the duration of AP and STA vap's don't overlap */
    if ((work[cfg2->wid_deact].time < work[cfg1->wid_act].time) ||
        (work[cfg2->wid_act].time > work[cfg1->wid_deact].time)) {
        log_msg(WPS_DBG, "non-overlapping case, can't "
                "adjust, start %d end %d, start %d end %d\n",
                work[cfg1->wid_act].time, work[cfg1->wid_deact].time,
                work[cfg2->wid_act].time, work[cfg2->wid_deact].time);
        return ret;
    }

    if (work[cfg1->wid_act].activation != 1 ||
        work[cfg2->wid_act].activation != 1) {
        log_msg(WPS_FTL, "bad activ %d %d\n",
                work[cfg1->wid_act].activation,
                work[cfg2->wid_act].activation);
    }

    if (work[cfg1->wid_deact].activation != -1 ||
        work[cfg2->wid_deact].activation != -1) {
        log_msg(WPS_FTL, "bad deactiv %d %d\n",
                work[cfg1->wid_deact].activation,
                work[cfg2->wid_deact].activation);
    }

    if (work[cfg2->wid_act].time > work[cfg1->wid_act].time) {
        log_msg(WPS_DBG, "early start for %s from %d to %d\n", sta ? "AP" :
                "STA", work[cfg2->wid_act].time, work[cfg1->wid_act].time);
        /*
         * Swap the activation work items. Change the pointers from
         * work item to config and the work item id in the config.
         */
        work[cfg1->wid_act].cfg_ptr = cfg2;
        work[cfg2->wid_act].cfg_ptr = cfg1;
        x = cfg1->wid_act;
        cfg1->wid_act = cfg2->wid_act;
        cfg2->wid_act = x;
        /* For activation, also need to start WPS now */
        log_msg(WPS_DBG, "starting %s wps on %s vap of %s "
                "radio\n", sta ? "AP" : "STA", cfg2->vap, cfg2->radio);
        if (cfg2->wps_active) {
            log_msg(WPS_FTL, "%s wps already started on %s vap of %s "
                    "radio\n", sta ? "AP" : "STA", cfg2->vap, cfg2->radio);
        }
        cfg2->wps_active = 1;
        if (sta)
            ret = start_wps_on_ap(cfg2->vap, cfg2->radio);
        else
            ret = start_wps_on_sta(cfg2->vap, cfg2->radio);
        if (ret < 0)
            cfg2->wps_active = 0;
    }

    if (work[cfg2->wid_deact].time < work[cfg1->wid_deact].time) {
        log_msg(WPS_DBG, "late finish for %s from %d to %d\n", sta ? "AP" :
                "STA", work[cfg2->wid_deact].time, work[cfg1->wid_deact].time);
        /*
         * Swap the deactivation work items. Change the pointers from
         * work item to config and the work item id in the config.
         */
        work[cfg1->wid_deact].cfg_ptr = cfg2;
        work[cfg2->wid_deact].cfg_ptr = cfg1;
        x = cfg1->wid_deact;
        cfg1->wid_deact = cfg2->wid_deact;
        cfg2->wid_deact = x;
        /* As deactivation can't happen now, no need to stop WPS */
    }

    /* Print this again */
    for (i = 0; i < items; i++) {
        int w, x, y = -1, z = -1;
        log_msg(WPS_DBG, "at %d do %s for vap %s of radio %s\n",
                work[i].time, work[i].activation > 0 ? "start" :
                "stop", work[i].cfg_ptr->vap, work[i].cfg_ptr->radio);
        w = work[i].cfg_ptr->wid_act;
        x = work[i].cfg_ptr->wid_deact;
        if (work[i].cfg_ptr->radio_ptr) {
            y = work[i].cfg_ptr->radio_ptr->wid_act;
            z = work[i].cfg_ptr->radio_ptr->wid_deact;
        }
        log_msg(WPS_DBG, "our start id %d stop id %d, "
                "same_radio_vap start id %d stop id %d\n", w, x, y, z);
    }

    return ret;
}

/*
 * Processes items in the work list one by one, sleeping in 
 * between as required. Finally the linked list of config's
 * and the work list are freed. The processing ends
 * prematurely if a signal is received.
 */
static void process_work(int sock, int items,
                         struct wps_work_t *work, struct wps_cfg_t *head)
{
    int current_time = 0, i = 0, ret;
    struct wps_cfg_t *cfg, *cfg_ap, *cfg_sta;
    char wps_completed_vap[IFNAMSIZ];
    FILE *f;

    while (i < items) {
        if (terminate_on_signal) {
            if (terminate_on_signal == 2) { // SIGUSR1
                wps_completed_vap[0] = '\0';
                f = fopen(cmpl_file, "r");
                if (f) {
                    fscanf(f, "%s", wps_completed_vap);
                    log_msg(WPS_DBG, "wps completed "
                            "on %s vap\n", wps_completed_vap);
                    fclose(f);
                    if (unlink(cmpl_file) < 0) {
                        log_msg(WPS_ERR, "can't remove file %s "
                            "(errno %d %s)\n", cmpl_file, errno, strerror(errno));
                    }
                } else {
                    log_msg(WPS_WARN, "received %d signal "
                            "but no completion file\n", SIGUSR1);
                }
            }

            cfg = head;
            while (cfg) {
                if (cfg->wps_active && cfg->type == WPS_MASTER) {
                    if (terminate_on_signal != 2 ||
                        strncmp(wps_completed_vap, cfg->vap, IFNAMSIZ)) {
                        log_msg(WPS_DBG, "stopping AP wps on %s "
                                "vap of %s radio\n", cfg->vap, cfg->radio);
                        stop_wps_on_ap(cfg->vap, cfg->radio);
                    }
                } else if (cfg->wps_active && cfg->type == WPS_MANAGED) {
                    if (terminate_on_signal != 2 ||
                        strncmp(wps_completed_vap, cfg->vap, IFNAMSIZ)) {
                        log_msg(WPS_DBG, "stopping STA wps on %s "
                                "vap of %s radio\n", cfg->vap, cfg->radio);
                        stop_wps_on_sta(cfg->vap, cfg->radio);
                    }
                }
                cfg->wps_active = 0;
                cfg = cfg->next_ptr;
            }

            log_msg(WPS_INF, "stopping loop\n");
            break;
        }

        if (current_time == work[i].time) {
            ret = 0;
            if (work[i].activation > 0) {
                switch (work[i].cfg_ptr->type) {
                case WPS_MASTER:
                    cfg = work[i].cfg_ptr;
                    cfg_sta = work[i].cfg_ptr->radio_ptr; // STA vap
                    if ((cfg->option & SKIP_AP_IF_STA_DC) && cfg_sta &&
                        !is_sta_connected(sock, cfg_sta->vap)) {
                        log_msg(WPS_DBG, "skipping wps on "
                                "disconnected STA's AP vap\n");
                        ret = pass_wps_duration(work, items, 0, cfg, cfg_sta);
                        break;
                    }
                    log_msg(WPS_DBG, "starting AP wps on %s vap "
                            "of %s radio\n", cfg->vap, cfg->radio);
                    if (cfg->wps_active) {
                        log_msg(WPS_FTL, "AP wps already started on %s "
                                "vap of %s radio\n", cfg->vap, cfg->radio);
                    }
                    cfg->wps_active = 1;
                    ret = start_wps_on_ap(cfg->vap, cfg->radio);
                    if (ret < 0)
                        cfg->wps_active = 0;
                    break;
                case WPS_MANAGED:
                    cfg = work[i].cfg_ptr;
                    cfg_ap = work[i].cfg_ptr->radio_ptr; // AP vap
                    if (!(cfg->option & TRY_STA_ALW) &&
                        is_sta_connected(sock, cfg->vap) > 0) {
                        log_msg(WPS_DBG, "skipping wps on connected STA vap\n");
                        ret = pass_wps_duration(work, items, 1, cfg, cfg_ap);
                        break;
                    }
                    log_msg(WPS_DBG, "starting STA wps on %s vap "
                            "of %s radio\n", cfg->vap, cfg->radio);
                    if (cfg->wps_active) {
                        log_msg(WPS_FTL, "STA wps already started on %s "
                                "vap of %s radio\n", cfg->vap, cfg->radio);
                    }
                    cfg->wps_active = 1;
                    ret = start_wps_on_sta(cfg->vap, cfg->radio);
                    if (ret < 0)
                        cfg->wps_active = 0;
                    break;
                case WPS_UNUSED:
                    break;
                }
            } else {
                switch (work[i].cfg_ptr->type) {
                case WPS_MASTER:
                    cfg = work[i].cfg_ptr;
                    if (cfg->wps_active) {
                        log_msg(WPS_DBG, "stopping AP wps on %s "
                                "vap of %s radio\n", cfg->vap, cfg->radio);
                        ret = stop_wps_on_ap(cfg->vap, cfg->radio);
                    }
                    cfg->wps_active = 0;
                    break;
                case WPS_MANAGED:
                    cfg = work[i].cfg_ptr;
                    if (cfg->wps_active) {
                        log_msg(WPS_DBG, "stopping STA wps on %s "
                                "vap of %s radio\n", cfg->vap, cfg->radio);
                        ret = stop_wps_on_sta(cfg->vap, cfg->radio);
                    }
                    cfg->wps_active = 0;
                    break;
                case WPS_UNUSED:
                    break;
                }
            }

            if (ret < 0) {
                log_msg(WPS_ERR, "wps failure\n");
            }

            i++;
        } else {
            struct timespec timeout;
            log_msg(WPS_DBG, "sleep %d secs\n", work[i].time - current_time);
            // sleep(work[i].time - current_time);
            timeout.tv_sec = time(NULL) + (work[i].time - current_time);
            timeout.tv_nsec = 0;
            if ((ret = sem_timedwait(&sem, &timeout)) < 0 && errno == EINTR) {
                log_msg(WPS_INF, "sem_wait received signal stopping loop\n");
            } else if (ret < 0) {
                log_msg(WPS_DBG, "sem_wait failed "
                        "(errno %d %s)\n", errno, strerror(errno));
            }
            current_time = work[i].time;
            /* Process this again, now current_time == work[i].time */
        }
    }

    free(work);

    /* Freeing the list of config's */
    while (head) {
        cfg = head->next_ptr;
        free(head);
        head = cfg;
    }
}

/*
 * Populates the work list from the config's
 * and sorts it in ascending order of time.
 */
static int get_sorted_work(struct wps_cfg_t *head, struct wps_work_t *work_list)
{
    int i, j, k;

    i = 0;
    while (head) {
        work_list[i].time = head->start_time;
        work_list[i].activation = 1;
        work_list[i].cfg_ptr = head;
        i++;

        work_list[i].time = head->start_time + head->duration;
        work_list[i].activation = -1;
        work_list[i].cfg_ptr = head;
        i++;

        head = head->next_ptr;
    }

    /*
     * Sort the work list, acivations ahead of deactivations 
     * (this is important because we check whether duration has
     * to be passed to be another vap only when processing
     * an activation item)
     */
    for (j = 0; j < i; j++) {
        for (k = j + 1; k < i; k++) {
            if (work_list[j].time > work_list[k].time ||
                (work_list[j].time == work_list[k].time &&
                work_list[j].activation < work_list[k].activation)) {
                struct wps_work_t tmp;
                tmp = work_list[j];
                work_list[j] = work_list[k];
                work_list[k] = tmp;
            }
        }
    }

    for (j = 0; j < i; j++) {
        if (work_list[j].activation > 0)
            work_list[j].cfg_ptr->wid_act = j;
        else
            work_list[j].cfg_ptr->wid_deact = j;
    }

    for (j = 0; j < i; j++) {
        int w, x, y = -1, z = -1;
        log_msg(WPS_DBG, "at %d do %s for vap %s of radio %s\n",
                work_list[j].time, work_list[j].activation > 0 ? "start" :
                "stop", work_list[j].cfg_ptr->vap, work_list[j].cfg_ptr->radio);
        w = work_list[j].cfg_ptr->wid_act;
        x = work_list[j].cfg_ptr->wid_deact;
        if (work_list[j].cfg_ptr->radio_ptr) {
            y = work_list[j].cfg_ptr->radio_ptr->wid_act;
            z = work_list[j].cfg_ptr->radio_ptr->wid_deact;
        }
        log_msg(WPS_DBG, "our start id %d stop id %d, "
                "same_radio_vap start id %d stop id %d\n", w, x, y, z);
    }

    return i;
}

/*
 * This allocates a linked list of config's 
 * and a work list of activations / deactivatons.
 * These lists are returned to the caller, and 
 * freeing is done by caller.
 */
static int prepare_work(const char *file, int sock, int *p_items,
                        struct wps_work_t **p_work, struct wps_cfg_t **p_head)
{
    char time[10];
    char *start, *end;
    int n, ret = 0;
    struct wps_cfg_t *cfg, *head = NULL;
    struct wps_work_t *work_list = NULL;
    char *line_buf = NULL;
    size_t line_sz = WPS_CFG_LN_SZ;
    FILE *cfgf = NULL;
    struct wps_opt_t *opt, *options = NULL;
    char option[30];

    log_msg(WPS_DBG, "opening file %s for reading config\n", file);
    cfgf = fopen(file, "r");
    if (!cfgf) {
        log_msg(WPS_ERR, "can't open %s for reading config\n", file);
        ret = -1;
        goto err;
    }

    line_buf = malloc(line_sz);
    if (!line_buf) {
        log_msg(WPS_ERR, "malloc %d bytes failed\n", line_sz);
        ret = -1;
        goto err;
    }

    while (getline(&line_buf, &line_sz, cfgf) != -1) {
        cfg = malloc(sizeof(struct wps_cfg_t));
        if (!cfg) {
            log_msg(WPS_WARN, "malloc %d bytes failed\n", sizeof(*cfg));
            continue;
        }

        /* VAP name */
        start = line_buf;
        if (*start == '\0' || *start == '\n')
            goto spl_radio_opts;

        end = strstr(start, ":");
        if (!end || (n = end - start) >= IFNAMSIZ || n <= 0)
            goto spl_radio_opts;

        memcpy(cfg->vap, start, n);
        cfg->vap[n] = '\0';

        /* Start time */
        start = end + 1;
        if (*start == '\0' || *start == '\n')
            goto spl_radio_opts;

        end = strstr(start, ":");
        if (!end || (n = end - start) >= sizeof(time) || n <= 0)
            goto spl_radio_opts;

        memcpy(time, start, n);
        time[n] = '\0';
        if (n == 1 && time[0] == '-') {
            log_msg(WPS_INF, "empty start_time for vap %s\n", cfg->vap);
            free(cfg);
            continue;
        }
        cfg->start_time = strtoul(time, NULL, 0);

        /* Duration */
        start = end + 1;
        if (*start == '\0' || *start == '\n')
            goto spl_radio_opts;

        end = strstr(start, ":");
        if (!end || (n = end - start) >= sizeof(time) || n <= 0)
            goto spl_radio_opts;

        memcpy(time, start, n);
        time[n] = '\0';
        if (n == 1 && time[0] == '-') {
            log_msg(WPS_INF, "empty duration for vap %s\n", cfg->vap);
            free(cfg);
            continue;
        }
        cfg->duration = strtoul(time, NULL, 0);

        /* Radio name */
        start = end + 1;
        if (*start == '\0' || *start == '\n')
            goto spl_radio_opts;

        end = line_buf + strlen(line_buf) - 1;
        if (*end != '\n')
            end++; // '\0'
        if (!end || (n = end - start) >= IFNAMSIZ || n <= 0)
            goto spl_radio_opts;

        memcpy(cfg->radio, start, n);
        cfg->radio[n] = '\0';

        cfg->radio_ptr = NULL;
        cfg->type = find_interface_type(sock, cfg->vap, cfg->radio);
        cfg->wps_active = 0;
        cfg->wid_act = cfg->wid_deact = -1;
        cfg->option = 0;

        log_msg(WPS_DBG, "vap:%s start_time:%d "
                "duration:%d radio:%s intf_type:%d\n", cfg->vap,
                cfg->start_time, cfg->duration, cfg->radio, cfg->type);

        /* Making a list of config's read from file */
        cfg->next_ptr = head;
        head = cfg;
        continue;

spl_radio_opts:
        free(cfg);

        opt = malloc(sizeof(struct wps_opt_t));
        if (!opt) {
            log_msg(WPS_WARN, "malloc %d bytes failed\n", sizeof(*opt));
            continue;
        }

        /* Radio name */
        start = line_buf;
        if (*start == '\0' || *start == '\n')
            goto next_line;

        end = strstr(start, ":");
        if (!end || (n = end - start) >= IFNAMSIZ || n <= 0)
            goto next_line;

        memcpy(opt->radio, start, n);
        opt->radio[n] = '\0';

        /* Option */
        start = end + 1;
        if (*start == '\0' || *start == '\n')
            goto next_line;

        end = line_buf + strlen(line_buf) - 1;
        if (*end != '\n')
            end++; // '\0'
        if (!end || (n = end - start) >= sizeof(option) || n <= 0)
            goto next_line;

        memcpy(option, start, n);
        option[n] = '\0';
        if (!strncmp(option, "try_sta_always", n))
            opt->option = TRY_STA_ALW;
        else if (!strncmp(option, "skip_ap_if_sta_disconnected", n))
            opt->option = SKIP_AP_IF_STA_DC;
        else
            goto next_line;

        opt->next_ptr = options;
        options = opt;
        continue;

next_line:
        log_msg(WPS_INF, "failure in parsing %s", line_buf);
        free(opt);
    }

    /* Apply the options */
    cfg = head;
    while (cfg) {
        opt = options;
        while (opt) {
            if (!strncmp(cfg->radio, opt->radio, IFNAMSIZ)) {
                cfg->option |= opt->option;
            }
            opt = opt->next_ptr;
        }
        cfg = cfg->next_ptr;
    }

    while (options) {
        opt = options->next_ptr;
        free(options);
        options = opt;
    }

    ret = validate_config(&head);
    if (ret < 0) {
        log_msg(WPS_WARN, "invalid config\n");
        goto err;
    }

    if (ret == 0) {
        log_msg(WPS_INF, "no work to be done\n");
        *p_items = 0;
        if (head)
            log_msg(WPS_FTL, "non-empty config list\n");
        goto err;
    }

    work_list = malloc(2 * ret * sizeof(struct wps_work_t));
    if (!work_list) {
        log_msg(WPS_ERR, "malloc %d bytes "
                "failed\n", 2 * ret * sizeof(*work_list));
        ret = -1;
        goto err;
    }

    if (get_sorted_work(head, work_list) > 2 * ret)
        log_msg(WPS_FTL, "written more entries than list size %d\n", 2 * ret);

    *p_items = 2 * ret;
    *p_work = work_list;
    *p_head = head;
    if (!head)
        log_msg(WPS_FTL, "empty config list\n");
    ret = 0;
    goto no_err;

err:
    /* Freeing the list of config's */
    while (head) {
        cfg = head->next_ptr;
        free(head);
        head = cfg;
    }

no_err:
    if (line_buf)
        free(line_buf);

    if (cfgf)
        fclose(cfgf);

    return ret;
}

/*
 * Daemonize.
 */
static void run_as_daemon(void)
{
    pid_t pid, sid;
    int fd;
    FILE *pidf;

    pid = fork();

    switch (pid) {
    case 0: // Child
        sid = setsid();
        if (sid < 0) {
            log_msg(WPS_ERR, "setsid failed (errno "
                    "%d %s)\n", errno, strerror(errno));
            goto err;
        }

        if (chdir("/") < 0) {
            log_msg(WPS_ERR, "chdir failed (errno "
                    "%d %s)\n", errno, strerror(errno));
            goto err;
        }

        pidf = fopen(pid_file, "w");
        if (!pidf) {
            log_msg(WPS_ERR, "can't open %s for writing PID\n", pid_file);
            goto err;
        }
        fprintf(pidf, "%u\n", getpid());
        fclose(pidf);

        close(STDIN_FILENO);
        fd = open("/dev/null", O_WRONLY);
        if (fd >= 0) {
            dup2(fd, STDOUT_FILENO);
            dup2(fd, STDERR_FILENO);
            close(fd);
        } else {
            log_msg(WPS_INF, "can't open /dev/null "
                    "(errno %d %s)\n", errno, strerror(errno));
            close(STDOUT_FILENO);
            close(STDERR_FILENO);
        }

        return;

    case -1:
        log_msg(WPS_ERR, "fork failed (errno "
                "%d %s)\n", errno, strerror(errno));
        goto err;

    default: // Parent
        exit(EXIT_SUCCESS);
    }

err:
    exit(EXIT_FAILURE);
}

/*
 * Main.
 */
int main(int argc, char *argv[])
{
    int ret = EXIT_SUCCESS, items, sock = -1;
    struct wps_work_t *work;
    struct wps_cfg_t *head;

    parse_args(argc, argv);

    if (background)
        run_as_daemon();

    if (sem_init(&sem, 0, 0) < 0) {
        log_msg(WPS_WARN, "sem_init failed (errno "
                "%d %s)\n", errno, strerror(errno));
        if (background) {
            if (unlink(pid_file) < 0) {
                log_msg(WPS_ERR, "can't remove file %s "
                        "(errno %d %s)\n", pid_file, errno, strerror(errno));
            }
        }
        exit(EXIT_FAILURE);
    }

    if (signal(SIGINT, sig_handler) == SIG_ERR ||
        signal(SIGTERM, sig_handler) == SIG_ERR ||
        signal(SIGUSR1, sig_handler) == SIG_ERR) {
        log_msg(WPS_WARN, "signal failed (errno "
                "%d %s)\n", errno, strerror(errno));
        ret = EXIT_FAILURE;
        goto out;
    }

    if (logfile) {
        logf = fopen(log_file, "w"); // Truncate
        if (!logf)
            log_msg(WPS_WARN, "can't open %s for logging\n", log_file);
    }

    sock = socket(PF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        log_msg(WPS_WARN, "socket failed (errno "
                "%d %s)\n", errno, strerror(errno));
        ret = EXIT_FAILURE;
        goto out;
    }

    log_msg(WPS_DBG, "radios %d and vaps %d\n", num_radio, num_vap);

    if (prepare_work(config_file, sock, &items, &work, &head)) {
        log_msg(WPS_WARN, "can't prepare work list\n");
        ret = EXIT_FAILURE;
        goto out;
    }

    if (items)
        process_work(sock, items, work, head);

out:
    if (background) {
        if (unlink(pid_file) < 0) {
            log_msg(WPS_ERR, "can't remove file %s "
                    "(errno %d %s)\n", pid_file, errno, strerror(errno));
        }
    }

    if (sock >= 0)
        close(sock);

    if (logf)
        fclose(logf);
    logf = NULL;

    sem_destroy(&sem);

    exit(ret);
}

/*
 * Signal handler for SIGINT, SIGTERM and SIGUSR1.
 */
void sig_handler(int signal)
{
    int ret;

    log_msg(WPS_INF, "received signal %d\n", signal);

    /*
     * If application were blocked on the semaphore the signal will
     * interrupt the wait too, but if it weren't then the release
     * makes sure that the next timed-wait doesn't get blocked.
     * And, if the signal comes at a time when the sempahore is not in
     * usable state, the release should fail with an error.
     */
    if ((ret = sem_post(&sem)) < 0) {
        log_msg(WPS_INF, "sem_post failed (errno "
                "%d %s)\n", errno, strerror(errno));
    }

    terminate_on_signal = 1;
    if (signal == SIGUSR1)
        terminate_on_signal = 2;
}

