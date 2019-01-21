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

#include <stdio.h> 
#include <string.h> 
#include <malloc.h> 
#include <sys/socket.h> 
#include <sys/ioctl.h>
#include <ctype.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <linux/types.h>
#include <linux/if.h>
#include <linux/netlink.h>
#include <stdlib.h>

#include <linux/wireless.h>

#include "os/linux/src/ath_netlink.h"

#include <asm/byteorder.h>
#if defined(__LITTLE_ENDIAN)
#define	_BYTE_ORDER	_LITTLE_ENDIAN
#elif defined(__BIG_ENDIAN)
#define	_BYTE_ORDER	_BIG_ENDIAN
#else
#error "Please fix asm/byteorder.h"
#endif
#include "os/linux/include/ieee80211_external.h"

#ifndef NETLINK_ATH_EVENT
    #define NETLINK_ATH_EVENT 19
#endif

#define MAX_PAYLOAD 1024  /* maximum payload size*/
struct sockaddr_nl src_addr, dest_addr;
struct nlmsghdr *nlh = NULL;
struct iovec iov;
int sock_fd;

u_int8_t ath_group_key[IEEE80211_KEYBUF_SIZE];
u_int8_t ath_ucast_key[IEEE80211_KEYBUF_SIZE];
int ath_group_key_len;
int ath_ucast_key_len;

static int
digittoint(int c)
{
	return isdigit(c) ? c - '0' : isupper(c) ? c - 'A' + 10 : c - 'a' + 10;
}

static int
getdata(const char *arg, u_int8_t *data, size_t maxlen)
{
	const char *cp = arg;
	int len;

	if (cp[0] == '0' && (cp[1] == 'x' || cp[1] == 'X'))
		cp += 2;
	len = 0;
	while (*cp) {
		int b0, b1;
		if (cp[0] == ':' || cp[0] == '-' || cp[0] == '.') {
			cp++;
			continue;
		}
		if (!isxdigit(cp[0])) {
			printf("invalid data value %c (not hex)\n", cp[0]);
			exit(-1);
		}
		b0 = digittoint(cp[0]);
		if (cp[1] != '\0') {
			if (!isxdigit(cp[1])) {
			    printf("invalid data value %c (not hex)\n", cp[1]);
				exit(-1);
			}
			b1 = digittoint(cp[1]);
			cp += 2;
		} else {			/* fake up 0<n> */
			b1 = b0, b0 = 0;
			cp += 1;
		}
		if (len > maxlen) {
			printf("too much data in %s, max %u bytes\n", arg, maxlen);
				
		}
		data[len++] = (b0<<4) | b1;
	}
	return len;
}

static int
getsocket(void)
{
	static int s = -1;

	if (s < 0) {
		s = socket(AF_INET, SOCK_DGRAM, 0);
		if (s < 0)
			err(1, "socket(SOCK_DRAGM)");
	}
	return s;
}

static int
set80211priv(const char *dev, int op, void *data, int len, int show_err)
{
	struct iwreq iwr;

	memset(&iwr, 0, sizeof(iwr));
	strncpy(iwr.ifr_name, dev, IFNAMSIZ);
	if (len < IFNAMSIZ) {
		/*
		 * Argument data fits inline; put it there.
		 */
		memcpy(iwr.u.name, data, len);
	} else {
		/*
		 * Argument data too big for inline transfer; setup a
		 * parameter block instead; the kernel will transfer
		 * the data for the driver.
		 */
		iwr.u.data.pointer = data;
		iwr.u.data.length = len;
	}

	if (ioctl(getsocket(), op, &iwr) < 0) {
		if (show_err) {
			static const char *opnames[] = {
				"ioctl[IEEE80211_IOCTL_SETPARAM]",
				"ioctl[IEEE80211_IOCTL_GETPARAM]",
				"ioctl[IEEE80211_IOCTL_SETKEY]",
				"ioctl[IEEE80211_IOCTL_GETKEY]",
				"ioctl[IEEE80211_IOCTL_DELKEY]",
				NULL,
				"ioctl[IEEE80211_IOCTL_SETMLME]",
				NULL,
				"ioctl[IEEE80211_IOCTL_SETOPTIE]",
				"ioctl[IEEE80211_IOCTL_GETOPTIE]",
				"ioctl[IEEE80211_IOCTL_ADDMAC]",
				NULL,
				"ioctl[IEEE80211_IOCTL_DELMAC]",
				"ioctl[IEEE80211_IOCTL_GETCHANLIST]",
				"ioctl[IEEE80211_IOCTL_SETCHANLIST]",
			};
			if (IEEE80211_IOCTL_SETPARAM <= op &&
			    op <= IEEE80211_IOCTL_SETCHANLIST)
				perror(opnames[op - SIOCIWFIRSTPRIV]);
			else
				perror("ioctl[unknown???]");
		}
		return -1;
	}
	return 0;
}

static int
set80211param(const char *dev, int op, int arg, int show_err)
{
    struct iwreq iwr;

	memset(&iwr, 0, sizeof(iwr));
	strncpy(iwr.ifr_name, dev, IFNAMSIZ);
    iwr.u.mode = op;
    memcpy(iwr.u.name+sizeof(__u32), &arg, sizeof(arg));

	if (ioctl(getsocket(), IEEE80211_IOCTL_SETPARAM, &iwr) < 0) {
        if (show_err) 
            perror("ioctl[IEEE80211_IOCTL_SETPARAM]");
        return -1;
    }
    return 0;
}

#if 0
static int
//set_security(const char *ifname, char *type, char *value)
set_security(const char *ifname)
{
    struct iwreq iwr;
    struct ieee80211_wlanconfig	config; 
    int len;
    
	//const char *ifname = "ath0";
	
    memset(&iwr, 0, sizeof(struct iwreq));
    memset(&config, 0, sizeof(struct ieee80211_wlanconfig));
    
    strncpy(iwr.ifr_name, ifname, IFNAMSIZ);

//	if (streq(type, "mode"))
	{
		config.cmdtype = IEEE80211_WLANCONFIG_IBSS_SET_SECURITY_MODE;
//		if (streq(value, "0"))
//			config.data.ibss_security.mode=0;
//		else
			config.data.ibss_security.mode=1;
	}

    iwr.u.data.pointer = (void*) &config;
    iwr.u.data.length = sizeof(struct ieee80211_wlanconfig);

    if ( ioctl(getsocket(), IEEE80211_IOCTL_CONFIG_GENERIC, &iwr) < 0 )
    {
        printf("ioctl: IEEE80211_IOCTL_CONFIG_GENERIC failed\n");
        return -1;
    }

	if (IEEE80211_WLANCONFIG_OK != config.status)
	{
		printf("Security command failed\n");
		return -1;
	}

    return 0;
}
#endif

static void
ieee80211_security_set_privacy(const char *ifname, int privacy)
{
    set80211param(ifname, IEEE80211_PARAM_PRIVACY, privacy, 1);
}

static void
ieee80211_security_set_drop_unencrypted(const char *ifname, int enabled)
{
    set80211param(ifname, IEEE80211_PARAM_DROPUNENCRYPTED, enabled, 1);
}

static void
ieee80211_security_set_authmode(const char *ifname)
{
    set80211param(ifname, IEEE80211_PARAM_AUTHMODE, IEEE80211_AUTH_RSNA, 1);
}

static void
ieee80211_security_set_groupkey(const char *ifname)
{
	struct ieee80211req_key setkey;

	/* set ik */
	memset(&setkey, 0, sizeof(setkey));
	setkey.ik_flags = IEEE80211_KEY_XMIT | IEEE80211_KEY_RECV | IEEE80211_KEY_DEFAULT | IEEE80211_KEY_GROUP;
	setkey.ik_keyix = 1;
	setkey.ik_type = IEEE80211_CIPHER_AES_CCM;
	setkey.ik_keylen = IEEE80211_KEYBUF_SIZE;
		memcpy(setkey.ik_keydata, (char *)ath_group_key, IEEE80211_KEYBUF_SIZE);
	memset(setkey.ik_macaddr, 0xff, IEEE80211_ADDR_LEN);

    set80211priv(ifname, IEEE80211_IOCTL_SETKEY, &setkey, sizeof(struct ieee80211req_key), 1);
}

static void
ieee80211_security_set_ucastkey(const char *ifname, unsigned char macaddr[IEEE80211_ADDR_LEN])
{
	struct ieee80211req_key setkey;

	/* set ik */
	memset(&setkey, 0, sizeof(setkey));
	setkey.ik_flags = IEEE80211_KEY_XMIT | IEEE80211_KEY_RECV;
	setkey.ik_keyix = IEEE80211_KEYIX_NONE;
	setkey.ik_type = IEEE80211_CIPHER_AES_CCM;
	setkey.ik_keylen = IEEE80211_KEYBUF_SIZE;
		memcpy(setkey.ik_keydata, (char *)ath_ucast_key, IEEE80211_KEYBUF_SIZE);
	memcpy(setkey.ik_macaddr, macaddr, IEEE80211_ADDR_LEN);

    set80211priv(ifname, IEEE80211_IOCTL_SETKEY, &setkey, sizeof(struct ieee80211req_key), 1);
}



int
main(int argc, char *argv[])
{
	struct msghdr msg;
	struct ath_netlink_event event;
	
	const char *ifname;   
	int  privacy = 0;
	
	ifname  = argv[1];
	privacy = atoi(argv[2]);

	memset(ath_group_key, 0, IEEE80211_KEYBUF_SIZE);
	memset(ath_ucast_key, 0, IEEE80211_KEYBUF_SIZE);
	
	/* Get Key data */
	ath_group_key_len = getdata(argv[3], ath_group_key, IEEE80211_KEYBUF_SIZE);
	ath_ucast_key_len = getdata(argv[4], ath_ucast_key, IEEE80211_KEYBUF_SIZE);
	
	{
	    int i;
	    printf("Group key: len = %d\n", ath_group_key_len);
	    printf("  content: ");
	    for (i=0; i<IEEE80211_KEYBUF_SIZE; i++)
            printf(" %x", ath_group_key[i]);
	    printf("\n");
            
	    printf("Ucast key: len = %d\n", ath_ucast_key_len);
	    printf("  content: ");
	    for (i=0; i<IEEE80211_KEYBUF_SIZE; i++)
            printf(" %x", ath_ucast_key[i]);
	    printf("\n");
    }
		
	sock_fd = socket(PF_NETLINK, SOCK_RAW, NETLINK_ATH_EVENT);
	
	memset(&src_addr, 0, sizeof(src_addr));
	src_addr.nl_family = AF_NETLINK;
	src_addr.nl_pid = getpid();  /* self pid */
	/* interested in group 1<<0 */
	src_addr.nl_groups = 1;
	bind(sock_fd, (struct sockaddr*)&src_addr, sizeof(src_addr));
	
	memset(&dest_addr, 0, sizeof(dest_addr));
	
	nlh = (struct nlmsghdr *)malloc(NLMSG_SPACE(MAX_PAYLOAD));
	if(nlh == NULL) 
	{
	   close(sock_fd);
	   return 0;
	}
	memset(nlh, 0, NLMSG_SPACE(MAX_PAYLOAD));
	
	iov.iov_base = (void *)nlh;
	iov.iov_len = NLMSG_SPACE(MAX_PAYLOAD);
	msg.msg_name = (void *)&dest_addr;
	msg.msg_namelen = sizeof(dest_addr);
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
	
	printf("DEMO Server is up\n");

	 if (privacy) {
	    printf("Enabling privacy\n");
   	    /*
	     * enable privacy
	     */
	    ieee80211_security_set_privacy(ifname, privacy);
    
	    /*
	     *  set : IEEE80211_AUTH_RSNA
	     *        
	     */
	    ieee80211_security_set_authmode(ifname);

	    /*
	     * set IEEE80211_PARAM_DROPUNENCRYPTED
	     */
	    ieee80211_security_set_drop_unencrypted(ifname, 1);	    
    }
                
	while(1)
	{	
		/* Read message from kernel */
		recvmsg(sock_fd, &msg, 0);
		memcpy(&event, NLMSG_DATA(nlh), sizeof(event));
		
		if (event.type == ATH_EVENT_NODE_JOIN) {
		    printf("Node %02x:%02x:%02x:%02x:%02x:%02x join\n", 
		                                  event.mac[0], event.mac[1], event.mac[2],
										  event.mac[3], event.mac[4], event.mac[5]);

   	        if (privacy) {
                //set_security(ifname);		        
	            
                /* 
                 * test : because groupkey is static only one for all per.
                 *        It just only need set up once.
                 */
	            ieee80211_security_set_groupkey(ifname);
	            	
                /*
                 * setup ucast key
                 */
		        ieee80211_security_set_ucastkey(ifname, event.mac);
		        
		        /*
		         * set IEEE80211_PARAM_DROPUNENCRYPTED
		         */
		        ieee80211_security_set_drop_unencrypted(ifname, 1);
		    }
		}
		else if (event.type == ATH_EVENT_NODE_LEAVE) {
		    printf("Node %02x:%02x:%02x:%02x:%02x:%02x leave\n", 
		                                  event.mac[0], event.mac[1], event.mac[2],
										  event.mac[3], event.mac[4], event.mac[5]);
		}
		else if (event.type == ATH_EVENT_NODE_RSSI_MONITOR) {
		    char *event_data = (char*)(((struct ath_netlink_event*)NLMSG_DATA(nlh)) + 1);
		    int rssi_class;
		    memcpy(&rssi_class, event_data, sizeof(int));
		    printf("Node %02x:%02x:%02x:%02x:%02x:%02x RSSI Class %d\n",
		                                  event.mac[0], event.mac[1], event.mac[2],
			   event.mac[3], event.mac[4], event.mac[5], rssi_class);
 		  
		}
		else if (event.type == ATH_EVENT_NODE_CHLOAD) {
		    char *event_data = (char*)(((struct ath_netlink_event*)NLMSG_DATA(nlh)) + 1);
		    unsigned char chload;
		    chload = *event_data;
		    printf("ATH-ADHOC chload is %d \n",chload);
		}
		else if (event.type == ATH_EVENT_NODE_NONERP_JOINED ){
		    char *event_data = (char*)(((struct ath_netlink_event*)NLMSG_DATA(nlh)) + 1);
		    unsigned char nonerp;
		    nonerp  = *event_data;
		    if(nonerp)
			    printf(" ATH-ADHOC NON ERP present   \n");
		    else
			    printf(" ATH-ADHOC NON ERP NOT presnt \n");
		}    
		else if (event.type == ATH_EVENT_NODE_BG_JOINED){
		    char *event_data = (char*)(((struct ath_netlink_event*)NLMSG_DATA(nlh)) + 1);
		    unsigned char bg;
		    bg = *event_data;
		    if(bg)
			    printf(" ATH-ADHOC BG station  presnt   \n");
		    else
			    printf(" ATH-ADHOC BG station  not presnt \n");
		}else if (event.type == ATH_EVENT_NODE_COCHANNEL_AP_CNT){
		    char *event_data = (char*)(((struct ath_netlink_event*)NLMSG_DATA(nlh)) + 1);
		    unsigned char cnt;
		    cnt = *event_data;
		    printf("ATH-ADHOC No of cochannel ap  %d \n",cnt);
		}
		else {
		    printf("unknown event : type (%d)\n", event.type);
		}
		
	}
		
	close(sock_fd);
	
	return 0;
}

