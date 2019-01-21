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
 *  
 */ 


#ifndef __APPBR_TYPES_H_
#define __APPBR_TYPES_H_

#include <a_base_types.h>

#define APPBR_NETLINK_NUM           21

/* Application IDs */
enum {
    APPBR_BYP       =   1,
    APPBR_ACFG      =   2,
    APPBR_WSUP_BR   =   3
};

#define APPBR_MAX_APPS                          16

/*
 * Error codes defined for setting args
 */
enum APPBR_STAT_VAL {
    APPBR_STAT_OK          =   A_STATUS_OK, 
    APPBR_STAT_EARGSZ      =   A_STATUS_E2BIG,
    APPBR_STAT_ENOSOCK     =   A_STATUS_EIO,
    APPBR_STAT_ENOREQACK   =   A_STATUS_EINPROGRESS,
    APPBR_STAT_ENORESPACK  =   A_STATUS_EBUSY,
    APPBR_STAT_ERECV       =   A_STATUS_EFAULT,
    APPBR_STAT_ESENDCMD    =   A_STATUS_FAILED,
    APPBR_STAT_ENOMEM      =   A_STATUS_ENOMEM,
};

typedef a_status_t  appbr_status_t;

#define APPBR_MSGADDR_TO_APPID(addr)            (addr)
#define APPBR_APPID_TO_MSGADDR(addr)            (addr)

#endif
