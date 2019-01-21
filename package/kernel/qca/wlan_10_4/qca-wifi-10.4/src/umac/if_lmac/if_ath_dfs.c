/*
 * Copyright (c) 2010, Atheros Communications Inc.
 * All Rights Reserved.
 *
 * Copyright (c) 2011 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 *
 */

#include <ieee80211_channel.h>
#include <ieee80211_var.h>
#include <ieee80211_scan.h>
#include <ieee80211_resmgr.h>

#include "ieee80211_sme_api.h"
#include "ieee80211_sm.h"
#include "if_athvar.h"

#if UMAC_SUPPORT_DFS

#define IS_CHANNEL_WEATHER_RADAR(freq) ((freq >= 5600) && (freq <= 5650))
#define ADJACENT_WEATHER_RADAR_CHANNEL   5580
#define CH100_START_FREQ                 5490
#define CH100                            100

/*
 * Print a console message with the device name prepended.
 */
void
if_printf( osdev_t dev, const char *fmt, ...)
{
    va_list ap;
    char buf[512];              /* XXX */

    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    printk("\n %s\n", buf);     /* TODO: print device name also */
}

static void
change_channel(struct ieee80211com *ic,
	       struct ieee80211_channel *chan)
{
    ic->ic_curchan = chan;
    ic->ic_set_channel(ic);
}

/*
    This function will retrun the first avialble channel
    in the mode specified. This will be called as a last
    resort when random channel search fails.
    If no channel is availabe then it will return -1
*/
static int find_any_valid_channel(struct ieee80211com *ic,  u_int32_t chan_mode)
{
    int i;
    u_int32_t chan_flags = 0;
    u_int32_t chan_flagext = 0;
    int ret_val = -1;

   /* find a valid channel in alt_chan_mode */

    for (i = 0; i < ic->ic_nchans; i++) {
        chan_flags = ic->ic_channels[i].ic_flags;
        chan_flagext = ic->ic_channels[i].ic_flagext;

        /* skip the channel if this is not desired mode */
        if ((chan_flags & chan_mode) == 0) {
            continue;
        }
        /* skip if weathere channles are not allowed */
        if(ic->ic_no_weather_radar_chan) {
            u_int32_t freq = ieee80211_chan2freq(ic,&ic->ic_channels[i]);
            if(IS_CHANNEL_WEATHER_RADAR(freq)) {
                continue;
            }
        }

        /* skip if radar was found in the channel */
        if (chan_flags & IEEE80211_CHAN_RADAR) {
            continue;
        }

        /* skip if DFS channles are blocked */

        if ((chan_flagext & IEEE80211_CHAN_DFS) &&
            (ic->ic_flags_ext & IEEE80211_FEXT_BLKDFSCHAN)) {
            continue;
        }
        /* we found a channel */
        ret_val = i;
        break;
    }
    return ret_val;
}
/*
    This function tries to find a random channel.
    Current channel mode is possibly HT160 or HT80_80. However it
    shold work for other 11AC modes.
    This function is called when random channel searh cannot find
    a channel in current (HT160/Ht80_80) mode
    possibly because of radar flag or the channle is
    not permitted for some other reason.

    alt_chan_mode can be HT80_80/HT80 when current channel mode is HT160.
    alt_chan_mode can be HT80 when current channle mode is HT80_80.
    However, this will work for other 11AC modes.

    chan_count is the total number of channels available in alt_chan_mode that
    can be picked. This would exclude any channel that can't be selected
    because it has radar or black listed.

    This function will return an index to the selected channel or
    -1 if search fails
*/
static int find_alternate_mode_channel(struct ieee80211com *ic, u_int32_t alt_chan_mode, int chan_count)
{
    u_int32_t   random_byte = 0;
    int n;
    int i;
    int count = 0;
    int ret_val = -1;

    int use_lower_5g_only = 0;
    int use_upper_5g_only = 0;
    u_int32_t chan_flags = 0;
    u_int32_t chan_flagext = 0;

    struct ieee80211_channel *ptarget_channel = NULL;

    osdev_t dev = ic->ic_osdev;
    OS_GET_RANDOM_BYTES(&random_byte,1);
    n = (random_byte + OS_GET_TICKS() ) % chan_count;

    /* FR 27305:
       In Japan domain, if current channel is below channel 100
       then find a new channel that is below 100. Similarly
       if the current channel is 100 or higher then pick a channel
       that is 100 or higher
    */
    if (DFS_MKK4_DOMAIN == ic->ic_get_dfsdomain(ic)) {
        if (IEEE80211_IS_CHAN_11AC_VHT80_80(ic->ic_curchan)) {
            /* no action required for now */
            use_lower_5g_only = 0;
            use_upper_5g_only = 0;
        } else if (IEEE80211_IS_CHAN_11AC_VHT160(ic->ic_curchan)){
            /* no action required for now */
            use_lower_5g_only = 0;
            use_upper_5g_only = 0;
        } else {
            if (ic->ic_curchan->ic_freq < CH100_START_FREQ) {
                use_lower_5g_only = 1;
                use_upper_5g_only = 0;
            } else {
                use_lower_5g_only = 0;
                use_upper_5g_only = 1;
            }
        }
    }

   /* find n-th valid channel in alt_chan_mode */

    for (i = 0; i < ic->ic_nchans; i++) {
        chan_flags = ic->ic_channels[i].ic_flags;
        chan_flagext = ic->ic_channels[i].ic_flagext;

        /* skip the channel if this is not desired alternate mode */
        if ((chan_flags & alt_chan_mode) == 0) {
            continue;
        }

        /* skip if weathere channles are not allowed */
        if(ic->ic_no_weather_radar_chan) {
            u_int32_t freq = ieee80211_chan2freq(ic,&ic->ic_channels[i]);
            if(IS_CHANNEL_WEATHER_RADAR(freq)) {
                continue;
            }
        }

        /* skip if radar was found in the channel */
        if (chan_flags & IEEE80211_CHAN_RADAR) {
            continue;
        }

        /* skip if DFS channles are blocked */

        if ((chan_flagext & IEEE80211_CHAN_DFS) &&
            (ic->ic_flags_ext & IEEE80211_FEXT_BLKDFSCHAN)) {
            continue;
        }
        /* FR 27305:
           In Japan domain, if current channel is below channel 100
           then find a new channel that is below 100. Similarly
           if the current channel is 100 or higher then pick a channel
           that is 100 or higher
        */

        if (use_lower_5g_only) {
            if (IEEE80211_IS_CHAN_11AC_VHT80_80(&ic->ic_channels[i])) {
                if ((ic->ic_channels[i].ic_freq > CH100_START_FREQ) ||
                    (ic->ic_channels[i].ic_vhtop_ch_freq_seg2 > CH100)) {
                    /* skip this channel */
                    continue;
                }
            } else {
                if (ic->ic_channels[i].ic_freq > CH100_START_FREQ) {
                    /* skip this channel */
                    continue;
                }
            }
        }

        if (use_upper_5g_only) {
            if (IEEE80211_IS_CHAN_11AC_VHT80_80(&ic->ic_channels[i])) {
                if ((ic->ic_channels[i].ic_freq < CH100_START_FREQ) ||
                    (ic->ic_channels[i].ic_vhtop_ch_freq_seg2 < CH100)) {
                    /* skip this channel */
                    continue;
                }
            } else {
                if (ic->ic_channels[i].ic_freq < CH100_START_FREQ) {
                    /* skip this channel */
                    continue;
                }
            }
        }

        if (count == n) {
            ret_val = i;
            ptarget_channel = &ic->ic_channels[i];

            if (IEEE80211_IS_CHAN_11AC_VHT160(ptarget_channel)) {
                if_printf (dev, "%s: VHT160 channel %d(%dMHz)\n",
                           __func__,
                           ptarget_channel->ic_ieee,
                           ptarget_channel->ic_freq);
            } else if (IEEE80211_IS_CHAN_11AC_VHT80_80(ptarget_channel)) {
                if_printf (dev, "%s: VHT80_80 primary channel %d(%dMHz), extension channel %d(%dMHz)\n",
                           __func__,
                           ptarget_channel->ic_ieee,
                           ptarget_channel->ic_freq,
                           ptarget_channel->ic_vhtop_ch_freq_seg2,
                           ieee80211_ieee2mhz(ic, ptarget_channel->ic_vhtop_ch_freq_seg2, ptarget_channel->ic_flags));
            } else if (IEEE80211_IS_CHAN_11AC_VHT80(ptarget_channel)) {
                if_printf (dev, "%s: VHT80 channel %d(%dMHz)\n",
                           __func__,
                           ptarget_channel->ic_ieee,
                           ptarget_channel->ic_freq);
            }
            break;
        } else {
            count++;
        }
    }
    return ret_val;
}

static int ieee80211_random_channel(struct ieee80211com *ic)
{
    int chanStart,n=0;
    u_int32_t curChanFlags, chan_flags, chan_flagext=0;

    int numGChannels=0;
    int numAChannels=0;
    int j;

    int ht160_count = 0;
    int ht80_80_count = 0;
    int ht80_count = 0;
    int ht40plus_count = 0;
    int ht40minus_count = 0;
    int ht20_count = 0;
    int use_lower_5g_only = 0;
    int use_upper_5g_only = 0;

    /* IR: 107025 -- Random channel selction not correct             */
    /* instead of u_int8_t available_chan_idx[IEEE80211_CHAN_MAX+1]  */
    /* use int *available_chan_idx and dynamically allocate it       */
    /* storing int charStart in byte array available_chan_idx[] is   */
    /* reason for random channel selection failure when number of    */
    /* max channel (IEEE80211_CHAN_MAX) is more than 255             */
    int *available_chan_idx;
    int available_chan_count = 0;
    int ret_val = -1;
    u_int32_t alt_chan_mode = 0;
    int       chan_count    = 0;

    osdev_t dev = ic->ic_osdev;
    if ((available_chan_idx = OS_MALLOC(ic->ic_osdev, (IEEE80211_CHAN_MAX+1) * sizeof(int) , GFP_KERNEL)) == NULL) {
        printk("%s: cannot allocate memory\n", __func__);
        return ret_val;
    }

    /* FR 27305:
       In Japan domain, if current channel is below channel 100
       then find a new channel that is below 100. Similarly
       if the current channel is 100 or higher then pick a channel
       that is 100 or higher
    */
    if (DFS_MKK4_DOMAIN == ic->ic_get_dfsdomain(ic)) {
        if (IEEE80211_IS_CHAN_11AC_VHT80_80(ic->ic_curchan)) {
            /* no action required for now */
            use_lower_5g_only = 0;
            use_upper_5g_only = 0;
            if_printf(dev,"%s -- MMK4 domain, HT80_80, no restriction on using upper or lower 5G channel\n", __func__);
        } else if (IEEE80211_IS_CHAN_11AC_VHT160(ic->ic_curchan)){
            /* no action required for now */
            use_lower_5g_only = 0;
            use_upper_5g_only = 0;
            if_printf(dev,"%s -- MMK4 domain, HT160, will look for HT160. if can't find no restriction on using upper or lower 5G channel\n", __func__);
        } else {
            if (ic->ic_curchan->ic_freq < CH100_START_FREQ) {
                use_lower_5g_only = 1;
                use_upper_5g_only = 0;
                if_printf(dev,"%s -- MMK4 domain, search for lower 5G (less than 5490 MHz) channels\n", __func__);
            } else {
                use_lower_5g_only = 0;
                use_upper_5g_only = 1;
                if_printf(dev,"%s -- MMK4 domain, search for upper 5G (more than 5490 MHz) channels\n", __func__);
            }
        }
    }


    /* Pick a random channel */

    /* Find how many G channels are present in the channel list
    * Assuming all G channels are present at the begining of
    * the list, followed by all A channels
     */

    for (j = 0; j < ic->ic_nchans; j++)
    {
        chan_flags = ic->ic_channels[j].ic_flags;
        if(chan_flags & IEEE80211_CHAN_2GHZ)
        {
            numGChannels++;
            continue;
        }
        else
            break;
    }

    numAChannels = (ic->ic_nchans - numGChannels);
    chanStart = numGChannels ; //+ ( OS_GET_TICKS() % numAChannels);

    curChanFlags = (ic->ic_curchan->ic_flags) & IEEE80211_CHAN_ALL;
    if ( ic->ic_flags_ext & IEEE80211_FEXT_BLKDFSCHAN )
	curChanFlags &= ~IEEE80211_CHAN_DFS;

    for (n = 0; n < ic->ic_nchans; chanStart++, n++)
	{
	    if (chanStart == ic->ic_nchans)
		chanStart = 0;
	    chan_flags = ic->ic_channels[chanStart].ic_flags;
	    chan_flagext = ic->ic_channels[chanStart].ic_flagext;
        /* these channels have CAC of 10 minutes so skipping these */

        if(ic->ic_no_weather_radar_chan) {
        /*we should also avoid this channel in
          HT40 mode as extension channel will be on 5600 */
            u_int32_t freq = ieee80211_chan2freq(ic,&ic->ic_channels[chanStart]);

            if(((IS_CHANNEL_WEATHER_RADAR(freq))
                        || ((IEEE80211_CHAN_11NA_HT40PLUS & chan_flags)
                            && (ADJACENT_WEATHER_RADAR_CHANNEL == freq)))
                    && (DFS_ETSI_DOMAIN == ic->ic_get_dfsdomain(ic)))
            {
				continue;
			}
#undef ADJACENT_WEATHER_RADAR_CHANNEL
		}

	    /*
	     * (1) skip static turbo channel as it will require STA to be in
	     * static turbo to work.
	     * (2) skip channel which's marked with radar detction
	     * (3) WAR: we allow user to config not to use any DFS channel
	     */
	    /* When we pick a channel, skip excluded 11D channels. See bug 3124 */

	    if ((chan_flags & IEEE80211_CHAN_STURBO) ||
		(chan_flags & IEEE80211_CHAN_RADAR)  ||
		(chan_flagext & IEEE80211_CHAN_11D_EXCLUDED) ||
		(chan_flagext & IEEE80211_CHAN_DFS &&
		 ic->ic_flags_ext & IEEE80211_FEXT_BLKDFSCHAN ))
		continue;


            /* FR 27305:
               In Japan domain, if current channel is below channel 100
               then find a new channel that is below 100. Similarly
               if the current channel is 100 or higher then pick a channel
               that is 100 or higher
            */

            if (use_lower_5g_only) {
                if (IEEE80211_IS_CHAN_11AC_VHT80_80(&ic->ic_channels[chanStart])) {
                    if ((ic->ic_channels[chanStart].ic_freq > CH100_START_FREQ) ||
                        (ic->ic_channels[chanStart].ic_vhtop_ch_freq_seg2 > CH100)) {
                        /* skip this channel */
                        continue;
                    }
                } else {
                    if (ic->ic_channels[chanStart].ic_freq > CH100_START_FREQ) {
                        /* skip this channel */
                        continue;
                    }
                }
            }

            if (use_upper_5g_only) {
                if (IEEE80211_IS_CHAN_11AC_VHT80_80(&ic->ic_channels[chanStart])) {
                    if ((ic->ic_channels[chanStart].ic_freq < CH100_START_FREQ) ||
                        (ic->ic_channels[chanStart].ic_vhtop_ch_freq_seg2 < CH100)) {
                        /* skip this channel */
                        continue;
                    }
                } else {
                    if (ic->ic_channels[chanStart].ic_freq < CH100_START_FREQ) {
                        /* skip this channel */
                        continue;
                    }
                }
            }

            /* keep a count of VHT160, VHT80_80 and VHT80 channels
               so that we can move from VHT160 to VHT80_80 to VHT80
               if we cannot find a channel in current mode
            */

            if (chan_flags & IEEE80211_CHAN_VHT20) {
                ht20_count++;
            } else if (chan_flags & IEEE80211_CHAN_VHT40PLUS) {
                ht40plus_count++;
            } else if (chan_flags & IEEE80211_CHAN_VHT40MINUS) {
                ht40minus_count++;
            } else if (chan_flags & IEEE80211_CHAN_VHT80) {
                ht80_count++;
            } else if (chan_flags & IEEE80211_CHAN_VHT80_80) {
                ht80_80_count++;
            } else if (chan_flags & IEEE80211_CHAN_VHT160) {
                ht160_count++;
            }
	    if ((chan_flags & IEEE80211_CHAN_ALL) == curChanFlags) {
		//break;
             available_chan_idx[available_chan_count++] = chanStart;
             if (available_chan_count >= IEEE80211_CHAN_MAX + 1)
                break;
           }
	}
    if(available_chan_count)
    {
        u_int32_t   random_byte = 0;
        OS_GET_RANDOM_BYTES(&random_byte,1);
        j = (random_byte + OS_GET_TICKS() ) % available_chan_count;
        chanStart = (available_chan_idx[j]);
#if 0 //DEBUG code

        for (n = 0; n < available_chan_count; n++) {
        printk("%s[%d] idx %d chan %d flags 0x%08x\n", __func__, __LINE__,
                available_chan_idx[n], ic->ic_channels[available_chan_idx[n]].ic_ieee, ic->ic_channels[available_chan_idx[n]].ic_flags);
        }
        printk("%s[%d] random %d avail_count %d rand_idx %d\n", __func__, __LINE__, random_byte, available_chan_count, j);
        printk("%s[%d] Selected chan %d flags 0x%08x chanStart %d \n", __func__, __LINE__,
                ic->ic_channels[chanStart].ic_ieee, ic->ic_channels[chanStart].ic_flags, chanStart);
 #endif
        ret_val = chanStart;
    } else {
        if_printf (dev, "%s: Cannot find a channel, looking for channel in other mode. ht80_count=%d, ht80_80_count=%d, ht160_count=%d\n", __func__, ht80_count, ht80_80_count, ht160_count);

        /* we need to handle HT160/HT80_80 in a special way   */
        /* HT160 has only two channels available. We will     */
        /* try to change to HT80_80 if we cannot find any     */
        /* 160 MHz contiguous channel. If there is no HT80_80 */
        /* channel then we will look for HT80 channel. Also   */
        /* we will change HT80_80 to HT80 in case we can't    */
        /* find a HT80_80 channel. This can happen in some    */
        /* design with two 5G radios where one radio operates */
        /* in channel 36 through 64.                          */
        /* The same could be done for other 11AC modes but    */
        /* we have plenty of HT80, HT40 and HT20 channels     */
        /* The following code can also be enhanced to switch  */
        /* automatically to a wider channel whenever one      */
        /* is present                                         */

        if (ht160_count > 0) {
            alt_chan_mode = IEEE80211_CHAN_VHT160;
            chan_count    = ht160_count;
        } else if (ht80_80_count > 0) {
            alt_chan_mode = IEEE80211_CHAN_VHT80_80;
            chan_count    = ht80_80_count;
        } else if (ht80_count > 0) {
            alt_chan_mode = IEEE80211_CHAN_VHT80;
            chan_count    = ht80_count;
        } else if (ht40plus_count > 0) {
            alt_chan_mode = IEEE80211_CHAN_VHT40PLUS;
            chan_count    = ht40plus_count;
        } else if (ht40minus_count > 0) {
            alt_chan_mode = IEEE80211_CHAN_VHT40MINUS;
            chan_count    = ht40minus_count;
        } else if (ht20_count > 0) {
            alt_chan_mode = IEEE80211_CHAN_VHT20;
            chan_count    = ht20_count;
        }
        ret_val = find_alternate_mode_channel(ic, alt_chan_mode, chan_count);
        if (ret_val == -1) {
            /* last attempt to get a valid channel */
            if_printf(dev, "%s: Cannot find a channel. Forcing to first available HT20 channel\n", __func__);
            ret_val = find_any_valid_channel(ic, IEEE80211_CHAN_VHT20);
        }
    }
    OS_FREE(available_chan_idx);
    return ret_val;

}
/*update the bss channel info on all vaps */
static void ieee80211_vap_iter_update_bss_chan(void *arg, struct ieee80211vap *vap)
{
      struct ieee80211_channel *bsschan = (struct ieee80211_channel *) arg;

      vap->iv_bsschan = bsschan;

      /* we may change from VHT160 to VHT80_80 under some condition */
      /* ias we have only two channels and both are DFS.            */
      /* for HT80_80 mode we may switch to VHT80 mode when we have  */
      /* two 5G radios in a sysems and to avoid interfernce we may  */
      /* confine them to a limited number of channels.              */

      if (IEEE80211_IS_CHAN_11AC_VHT160(bsschan)) {
          vap->iv_des_mode   = IEEE80211_MODE_11AC_VHT160;
      } else if (IEEE80211_IS_CHAN_11AC_VHT80_80(bsschan)) {
          vap->iv_des_mode   = IEEE80211_MODE_11AC_VHT80_80;
          vap->iv_des_cfreq2 =  bsschan->ic_vhtop_ch_freq_seg2;
      } else if (IEEE80211_IS_CHAN_11AC_VHT80(bsschan)) {
          vap->iv_des_mode   = IEEE80211_MODE_11AC_VHT80;
      }


      if(vap->iv_bss)
      {
          vap->iv_bss->ni_chan = bsschan;
          vap->iv_des_chan[vap->iv_des_mode] = bsschan;
      }
}

#if ATH_SUPPORT_IBSS_DFS

/*
 *This function is also defined at ieee80211_wireless.c. However, it is not active yet with compilation flag.
 *Once it is actived, maybe we should remove one of it.
 */
#define IEEE80211_MODE_TURBO_STATIC_A   IEEE80211_MODE_MAX
static int
ieee80211_check_mode_consistency(struct ieee80211com *ic,int mode,struct ieee80211_channel *c)
{
    if (c == IEEE80211_CHAN_ANYC) return 0;
    switch (mode)
    {
    case IEEE80211_MODE_11B:
        if(IEEE80211_IS_CHAN_B(c))
            return 0;
        else
            return 1;
        break;

    case IEEE80211_MODE_11G:
        if(IEEE80211_IS_CHAN_ANYG(c))
            return 0;
        else
            return 1;
        break;

    case IEEE80211_MODE_11A:
        if(IEEE80211_IS_CHAN_A(c))
            return 0;
        else
            return 1;
        break;

    case IEEE80211_MODE_TURBO_STATIC_A:
        if(IEEE80211_IS_CHAN_A(c) && IEEE80211_IS_CHAN_STURBO(c) )
            return 0;
        else
            return 1;
        break;

    case IEEE80211_MODE_AUTO:
        return 0;
        break;

    case IEEE80211_MODE_11NG_HT20:
        if(IEEE80211_IS_CHAN_11NG_HT20(c))
            return 0;
        else
            return 1;
        break;

    case IEEE80211_MODE_11NG_HT40PLUS:
        if(IEEE80211_IS_CHAN_11NG_HT40PLUS(c))
            return 0;
        else
            return 1;
        break;

    case IEEE80211_MODE_11NG_HT40MINUS:
        if(IEEE80211_IS_CHAN_11NG_HT40MINUS(c))
            return 0;
        else
            return 1;
        break;

    case IEEE80211_MODE_11NG_HT40:
        if(IEEE80211_IS_CHAN_11NG_HT40MINUS(c) || IEEE80211_IS_CHAN_11NG_HT40PLUS(c))
            return 0;
        else
            return 1;
        break;

    case IEEE80211_MODE_11NA_HT20:
        if(IEEE80211_IS_CHAN_11NA_HT20(c))
            return 0;
        else
            return 1;
        break;

    case IEEE80211_MODE_11NA_HT40PLUS:
        if(IEEE80211_IS_CHAN_11NA_HT40PLUS(c))
            return 0;
        else
            return 1;
        break;

    case IEEE80211_MODE_11NA_HT40MINUS:
        if(IEEE80211_IS_CHAN_11NA_HT40MINUS(c))
            return 0;
        else
            return 1;
        break;

    case IEEE80211_MODE_11NA_HT40:
        if(IEEE80211_IS_CHAN_11NA_HT40MINUS(c) || IEEE80211_IS_CHAN_11NA_HT40PLUS(c))
            return 0;
        else
            return 1;
        break;
    }
    return 1;

}
#undef  IEEE80211_MODE_TURBO_STATIC_A

/*
 * Check with our own IBSS DFS list and see if this channel is radar free.
 * return 1 if it is radar free.
 */
static int ieee80211_checkDFS_free(struct ieee80211vap *vap, struct ieee80211_channel *pchannel)
{
    int isradarfree = 1;
    u_int   i = 0;

    for (i = (vap->iv_ibssdfs_ie_data.len - IBSS_DFS_ZERO_MAP_SIZE)/sizeof(struct channel_map_field); i > 0; i--) {
        if (vap->iv_ibssdfs_ie_data.ch_map_list[i-1].ch_num == pchannel->ic_ieee) {
            if (vap->iv_ibssdfs_ie_data.ch_map_list[i-1].ch_map.radar) {
                isradarfree = 0;
            }
            break;
        }
    }

    return isradarfree;
}

/*
 * Found next DFS free channel for ibss. Noted that this function currently only used for ibss dfs.
 */
static void *
ieee80211_next_channel(struct ieee80211vap *vap, struct ieee80211_channel *pchannel)
{
    int target_channel = 0;
    struct ieee80211_channel *ptarget_channel = pchannel;
    u_int i;
    u_int8_t startfromhead = 0;
    u_int8_t foundfitchan = 0;
    struct ieee80211com *ic = vap->iv_ic;

    for (i = 0; i < ic->ic_nchans; i++) {
        if(!ieee80211_check_mode_consistency(ic, ic->ic_curmode, &ic->ic_channels[i])) {
           if(ic->ic_channels[i].ic_ieee == pchannel->ic_ieee) {
               break;
           }
        }
    }

    target_channel = i + 1;
    if (target_channel >= ic->ic_nchans) {
        target_channel = 0;
        startfromhead = 1;
    }

    for (; target_channel < ic->ic_nchans; target_channel++) {
        if(!ieee80211_check_mode_consistency(ic, ic->ic_curmode, &ic->ic_channels[target_channel]) &&
           ieee80211_checkDFS_free(vap, &ic->ic_channels[target_channel])){
            foundfitchan = 1;
            ptarget_channel = &ic->ic_channels[target_channel];
            break;
        } else if ((target_channel >= ic->ic_nchans - 1) && !startfromhead) {
            /* if we could not find next in the trail. restart from head once */
            target_channel = 0;
            startfromhead = 1;
        } else if (ic->ic_channels[target_channel].ic_ieee == pchannel->ic_ieee &&
                   ic->ic_channels[target_channel].ic_flags == pchannel->ic_flags &&
                   startfromhead) {
            /* we already restart to find channel from head but could not find a proper one , just jump to a random one  */
            target_channel = ieee80211_random_channel(ic);
            if ( target_channel != -1) {
                ptarget_channel = &ic->ic_channels[target_channel];
            } else {
                ptarget_channel = pchannel;
            }
            break;
        }
    }

	return ptarget_channel;
}
#endif /* ATH_SUPPORT_IBSS_DFS */

/*
 * Execute radar channel change. This is called when a radar/dfs
 * signal is detected.  AP mode only.  Return 1 on success, 0 on
 * failure
 */
int
ieee80211_dfs_action(struct ieee80211vap *vap, struct ieee80211_channelswitch_ie *pcsaie)
{
    struct ieee80211com *ic = vap->iv_ic;
    osdev_t dev = ic->ic_osdev;
    int target_channel;
    struct ieee80211_channel *ptarget_channel = NULL;
    struct ieee80211vap *tmp_vap = NULL;
    if (vap->iv_opmode != IEEE80211_M_HOSTAP &&
        vap->iv_opmode != IEEE80211_M_IBSS) {
        return 0;
    }

#if ATH_SUPPORT_IBSS_DFS
    if (vap->iv_opmode == IEEE80211_M_IBSS) {

        if (pcsaie) {
            ptarget_channel = ieee80211_doth_findchan(vap, pcsaie->newchannel);
        } else if(ic->ic_flags & IEEE80211_F_CHANSWITCH) {
            ptarget_channel = ieee80211_doth_findchan(vap, ic->ic_chanchange_chan);
        } else {
            target_channel = ieee80211_random_channel(ic);
            if ( target_channel != -1) {
                ptarget_channel = &ic->ic_channels[target_channel];
            } else {
                ptarget_channel = NULL;
            }
        }
    }
#endif /* ATH_SUPPORT_IBSS_DFS */

    if (vap->iv_opmode == IEEE80211_M_HOSTAP) {
        target_channel = ieee80211_random_channel(ic);
        if ( target_channel != -1) {
            ptarget_channel = &ic->ic_channels[target_channel];
        } else {
            ptarget_channel = NULL;
        }
    }

    /* If we do have a scan entry, make sure its not an excluded 11D channel.
       See bug 31246 */
    /* No channel was found via scan module, means no good scanlist
       was found */

        if (ptarget_channel)
		{
                if (IEEE80211_IS_CHAN_11AC_VHT160 (ptarget_channel)) {
                    if_printf(dev,"Changing to HT160 %s channel %d (%d MHz)\n",
                        IEEE80211_IS_CHAN_DFS(ptarget_channel) ? "DFS" : "non-DFS",
                        ptarget_channel->ic_ieee,
                        ptarget_channel->ic_freq);
                } else if (IEEE80211_IS_CHAN_11AC_VHT80_80(ptarget_channel)) {
                    if_printf(dev,"Changing to HT80_80 Primary %s channel %d (%d MHz) secondary %s chan %d (center freq %d)\n",
                        IEEE80211_IS_CHAN_DFS(ptarget_channel) ? "DFS" : "non-DFS",
                        ptarget_channel->ic_ieee,
                        ptarget_channel->ic_freq,
                        IEEE80211_IS_CHAN_DFS_CFREQ2(ptarget_channel) ? "DFS" : "non-DFS",
                        ptarget_channel->ic_vhtop_ch_freq_seg2,
                        ieee80211_ieee2mhz(ic, ptarget_channel->ic_vhtop_ch_freq_seg2, ptarget_channel->ic_flags));
                } else if (IEEE80211_IS_CHAN_11AC_VHT80(ptarget_channel)) {
                    if_printf(dev,"Changing to HT80 %s channel %d (%d MHz)\n",
                        IEEE80211_IS_CHAN_DFS(ptarget_channel) ? "DFS" : "non-DFS",
                        ptarget_channel->ic_ieee,
                        ptarget_channel->ic_freq);
                } else if (IEEE80211_IS_CHAN_11AC_VHT40(ptarget_channel)) {
                    if_printf(dev,"Changing to HT40 %s channel %d (%d MHz)\n",
                        IEEE80211_IS_CHAN_DFS(ptarget_channel) ? "DFS" : "non-DFS",
                        ptarget_channel->ic_ieee,
                        ptarget_channel->ic_freq);
                } else if (IEEE80211_IS_CHAN_11AC_VHT20(ptarget_channel)) {
                    if_printf(dev,"Changing to HT20 %s channel %d (%d MHz)\n",
                        IEEE80211_IS_CHAN_DFS(ptarget_channel) ? "DFS" : "non-DFS",
                        ptarget_channel->ic_ieee,
                        ptarget_channel->ic_freq);
                }

		    if (vap->iv_state_info.iv_state == IEEE80211_S_RUN)
            {
                if (pcsaie) {
                    ic->ic_chanchange_chan = pcsaie->newchannel;
                    ic->ic_chanchange_tbtt = pcsaie->tbttcount;
                } else {
                    ic->ic_chanchange_channel = ptarget_channel;
                    ic->ic_chanchange_secoffset =
                        ieee80211_sec_chan_offset(ic->ic_chanchange_channel);
                    ic->ic_chanchange_chwidth =
                        ieee80211_get_chan_width(ic->ic_chanchange_channel);
                    ic->ic_chanchange_chan = ptarget_channel->ic_ieee;
                    ic->ic_chanchange_tbtt = IEEE80211_RADAR_11HCOUNT;
                }

                if (IEEE80211_IS_CHAN_11AC_VHT160(ptarget_channel)) {
                    vap->iv_des_mode   = IEEE80211_MODE_11AC_VHT160;
                }
                if (IEEE80211_IS_CHAN_11AC_VHT80_80(ptarget_channel)) {
                    vap->iv_des_cfreq2 =  ptarget_channel->ic_vhtop_ch_freq_seg2;
                    vap->iv_des_mode   = IEEE80211_MODE_11AC_VHT80_80;
                }
                if (IEEE80211_IS_CHAN_11AC_VHT80(ptarget_channel)) {
                    vap->iv_des_mode   = IEEE80211_MODE_11AC_VHT80;
                }

#ifdef MAGPIE_HIF_GMAC
                TAILQ_FOREACH(tmp_vap, &ic->ic_vaps, iv_next) {
                    ic->ic_chanchange_cnt += ic->ic_chanchange_tbtt;
                }
#endif
                ic->ic_flags |= IEEE80211_F_CHANSWITCH;

#if ATH_SUPPORT_IBSS_DFS
            if (vap->iv_opmode == IEEE80211_M_IBSS) {
                struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
                struct ieee80211_action_mgt_args *actionargs;
                /* overwirte with our own value if we are DFS owner */
                if ( pcsaie == NULL &&
                    vap->iv_ibssdfs_state == IEEE80211_IBSSDFS_OWNER) {
                    ic->ic_chanchange_tbtt = vap->iv_ibss_dfs_csa_threshold;
                }

                vap->iv_ibssdfs_state = IEEE80211_IBSSDFS_CHANNEL_SWITCH;

                if(vap->iv_csa_action_count_per_tbtt > vap->iv_ibss_dfs_csa_measrep_limit) {
                    return 1;
                }
                vap->iv_csa_action_count_per_tbtt++;

                ieee80211_ic_doth_set(ic);

                vap->iv_channelswitch_ie_data.newchannel = ic->ic_chanchange_chan;

                /* use beacon_update function to do real channel switch */
                scn->sc_ops->ath_ibss_beacon_update_start(scn->sc_dev);

                if(IEEE80211_ADDR_EQ(vap->iv_ibssdfs_ie_data.owner, vap->iv_myaddr))
                {
                    actionargs = OS_MALLOC(vap->iv_ic->ic_osdev, sizeof(struct ieee80211_action_mgt_args) , GFP_KERNEL);
                    if (actionargs == NULL) {
                        IEEE80211_DPRINTF(vap, IEEE80211_MSG_ANY, "%s: Unable to alloc arg buf. Size=%d\n",
                                                    __func__, sizeof(struct ieee80211_action_mgt_args));
                        return 0;
                    }
                    OS_MEMZERO(actionargs, sizeof(struct ieee80211_action_mgt_args));

                    actionargs->category = IEEE80211_ACTION_CAT_SPECTRUM;
                    actionargs->action   = IEEE80211_ACTION_CHAN_SWITCH;
                    actionargs->arg1     = 1;   /* mode? no use for now */
                    actionargs->arg2     = ic->ic_chanchange_chan;
                    ieee80211_send_action(vap->iv_bss, actionargs, NULL);
                    OS_FREE(actionargs);
                }
            }
#endif /* ATH_SUPPORT_IBSS_DFS */
            }
            else
			{
			    /*
			     * vap is not in run  state yet. so
			     * change the channel here.
			     */
                ic->ic_chanchange_chan = ptarget_channel->ic_ieee;

                /* update the bss channel of all the vaps */
                wlan_iterate_vap_list(ic, ieee80211_vap_iter_update_bss_chan, ptarget_channel);
                ic->ic_prevchan = ic->ic_curchan;
                ic->ic_curchan = ptarget_channel;
                TAILQ_FOREACH(tmp_vap, &ic->ic_vaps, iv_next) {
                    if (tmp_vap->iv_opmode == IEEE80211_M_MONITOR || (tmp_vap->iv_opmode == IEEE80211_M_HOSTAP &&
                                (tmp_vap->iv_state_info.iv_state == IEEE80211_S_DFS_WAIT || tmp_vap->iv_state_info.iv_state == IEEE80211_S_RUN))) {
                            channel_switch_set_channel(tmp_vap, ic);
                    }
                }

			}
		}
	    else
		{
		    /* should never come here? */
		    if_printf(dev,"Cannot change to any channel\n");
		    return 0;
		}
    return 1;
}

/*
 * Fetch a mute test channel which matches the operational mode/flags of the
 * current channel.
 *
 * Simply returning '36' when the AP is in HT40D mode will fail; the channel
 * lookup will be done with the channel flags requiring HT40D and said lookup
 * won't find a channel.
 *
 * XXX TODO: figure out the correct mute channel to return for VHT operation.
 *   It may be that we instead have to return the actual full VHT channel
 *   configuration (freq1, freq2, legacy ctl/ext info) when it's time to do
 *   this.
 */
static int
ieee80211_get_test_mute_chan(struct ieee80211com *ic,
  const struct ieee80211_channel *chan)
{
    if (chan == NULL)
        return IEEE80211_RADAR_TEST_MUTE_CHAN_11A;

    if (IEEE80211_IS_CHAN_VHT(chan))
        adf_os_print("%s: VHT not yet supported here (please fix); "
          "freq=%d, flags=0x%08x, "
          "falling through\n",
          __func__, chan->ic_freq, chan->ic_flags);

    if (IEEE80211_IS_CHAN_11N_HT40MINUS(chan))
        return IEEE80211_RADAR_TEST_MUTE_CHAN_11NHT40D;
    else if (IEEE80211_IS_CHAN_11N_HT40PLUS(chan))
        return IEEE80211_RADAR_TEST_MUTE_CHAN_11NHT40U;
    else if (IEEE80211_IS_CHAN_11N_HT20(chan))
        return IEEE80211_RADAR_TEST_MUTE_CHAN_11NHT20;
    else if (IEEE80211_IS_CHAN_A(chan))
        return IEEE80211_RADAR_TEST_MUTE_CHAN_11A;
    else {
        adf_os_print("%s: unknown channel mode, freq=%d, flags=0x%08x\n",
          __func__, chan->ic_freq, chan->ic_flags);
        return IEEE80211_RADAR_TEST_MUTE_CHAN_11A;
    }
}

void
ieee80211_mark_dfs(struct ieee80211com *ic, struct ieee80211_channel *ichan)
  {
     struct ieee80211_channel *c=NULL;
     struct ieee80211vap *vap;
#ifdef MAGPIE_HIF_GMAC
     struct ieee80211vap* tmp_vap = NULL;
#endif

     if (ic->ic_opmode == IEEE80211_M_HOSTAP ||
         ic->ic_opmode == IEEE80211_M_IBSS)
     {
         ieee80211_dfs_cac_cancel(ic);
         /* Mark the channel in the ic_chan list */
         /*
          * XXX TODO: this isn't exactly correct.
          * Specifically - it only marks the channels that match
          * the given centre frequency as having DFS, rather than
          * actually checking for channel overlap.  So it's not
          * entirely correct behaviour for VHT or HT40 operation.
          *
          * In any case, it should eventually be taught to just
          * use the channel centre and channel overlap code
          * that now exists in umac/base/ieee80211_channel.c
          * and flag them all.
          *
          * In actual reality, this also gets set correctly by
          * the NOL dfs channel list update method.
          */
         if ((ic->ic_flags_ext & IEEE80211_FEXT_MARKDFS) &&
             (ic->ic_dfs_usenol(ic) == 1))
         {
            /*
               IR: 115114 -- Primary Non-DFS channel is excluded
               from channel slection in HT80_80 mode

               The code that was used previously to set channel flag
               to indicate radar was found in the channel is discarded
               because of the following:

               1. Redundant -- as flags are already set (marked) by combination
               of dfs_channel_mark_radar/dfs_nol_update/ic_dfs_clist_update
               functions.

               2. The code does not work where we can mix DFS/Non-DFS channels
               and channel marking of radar. This can happen specially in
               in ht80_80 mode and ht160. The code (erroneously) prvents
               use of non-DFS primary 20 MHz control channel if radar is
               found.
            */

             c = ieee80211_find_channel(ic, ichan->ic_freq, ichan->ic_vhtop_ch_freq_seg2, ichan->ic_flags);

             if (c == NULL)
             {
                 return;
             }

             /*
              * If the reported event is on the current channel centre
              * frequency, begin the process of moving to another
              * channel.
              *
              * Unfortunately for now, this is not entirely correct -
              * If we report a radar event on a subchannel of the current
              * channel, this test will fail and we'll end up not
              * starting a CSA.
              *
              * XXX TODO: this API needs to change to take a radar event
              * frequency/width, instead of a channel.  It's then up
              * to the alternative umac implementations to write glue
              * to correctly handle things.
              */
             if  (ic->ic_curchan->ic_freq == c->ic_freq)
             {
                 /* get an AP vap */
                 vap = TAILQ_FIRST(&ic->ic_vaps);
                 while ((vap != NULL) && (vap->iv_state_info.iv_state != IEEE80211_S_RUN)  &&
                     (vap->iv_ic != ic))
                 {
                     vap = TAILQ_NEXT(vap, iv_next);
                 }
                 if (vap == NULL)
                 {
                     /*
                      * No running VAP was found, check
                      * any one is scanning.
                      */
                     vap = TAILQ_FIRST(&ic->ic_vaps);
                     while ((vap != NULL) && (vap->iv_ic != ic) &&
                            (vap->iv_state_info.iv_state != IEEE80211_S_SCAN))
                     {
                         vap = TAILQ_NEXT(vap, iv_next);
                     }
                     /*
                     * No running/scanning VAP was found, so they're all in
                     * INIT state, no channel change needed
                     */
                     if(!vap) return;
                     /* is it really Scanning */
                     /* XXX race condition ?? */
                     if(ic->ic_flags & IEEE80211_F_SCAN) return;
                     /* it is not scanning , but waiting for ath driver to move he vap to RUN */
                 }
#if ATH_SUPPORT_IBSS_DFS
               /* call the dfs action */
               if (vap->iv_opmode == IEEE80211_M_IBSS) {
                    u_int8_t index;
                    /* mark ibss dfs element, only support radar for now */
                    for (index = (vap->iv_ibssdfs_ie_data.len - IBSS_DFS_ZERO_MAP_SIZE)/sizeof(struct channel_map_field); index >0; index--) {
                        if (vap->iv_ibssdfs_ie_data.ch_map_list[index-1].ch_num == ichan->ic_ieee) {
                            vap->iv_ibssdfs_ie_data.ch_map_list[index-1].ch_map.radar = 1;
                            vap->iv_ibssdfs_ie_data.ch_map_list[index-1].ch_map.unmeasured = 0;
                            break;
                        }
                    }

                   if(IEEE80211_ADDR_EQ(vap->iv_myaddr, vap->iv_ibssdfs_ie_data.owner)) {
                        ieee80211_dfs_action(vap, NULL);
                   } else {
                        /* send out measurement report in this case */
                        ieee80211_measurement_report_action(vap, NULL);
                        if (vap->iv_ibssdfs_state == IEEE80211_IBSSDFS_JOINER) {
                            vap->iv_ibssdfs_state = IEEE80211_IBSSDFS_WAIT_RECOVERY;
                        }
                   }
               }
#endif /* ATH_SUPPORT_IBSS_DFS */
                if (vap->iv_opmode == IEEE80211_M_HOSTAP)
                    ieee80211_dfs_action(vap, NULL);
             }
           else
           {
           }
         }
         else
         {
             /* Change to a radar free 11a channel for dfstesttime seconds */
             ic->ic_chanchange_chan = ieee80211_get_test_mute_chan(ic,
               ic->ic_curchan);
             ic->ic_chanchange_tbtt = IEEE80211_RADAR_11HCOUNT;
#ifdef MAGPIE_HIF_GMAC
             TAILQ_FOREACH(tmp_vap, &ic->ic_vaps, iv_next) {
                 ic->ic_chanchange_cnt += ic->ic_chanchange_tbtt;
             }
#endif
             ic->ic_flags |= IEEE80211_F_CHANSWITCH;
             /* A timer is setup in the radar task if markdfs is not set and
              * we are in hostap mode.
              */
         }
     }
     else
     {
         /* Are we in sta mode? If so, send an action msg to ap saying we found  radar? */
#if ATH_SUPPORT_DFS && ATH_SUPPORT_STA_DFS
         if (ic->ic_opmode == IEEE80211_M_STA) {
             ieee80211_dfs_stacac_cancel(ic);
             /* Mark the channel in the ic_chan list */
             /*
              * XXX TODO: this isn't exactly correct.
              * Specifically - it only marks the channels that match
              * the given centre frequency as having DFS, rather than
              * actually checking for channel overlap.  So it's not
              * entirely correct behaviour for VHT or HT40 operation.
              *
              * In any case, it should eventually be taught to just
              * use the channel centre and channel overlap code
              * that now exists in umac/base/ieee80211_channel.c
              * and flag them all.
              *
              * In actual reality, this also gets set correctly by
              * the NOL dfs channel list update method.
              */
             if ((ic->ic_flags_ext & IEEE80211_FEXT_MARKDFS) &&
                 (ic->ic_dfs_usenol(ic) == 1)) {
                 c = ieee80211_find_channel(ic, ichan->ic_freq, ichan->ic_vhtop_ch_freq_seg2, ichan->ic_flags);

                 if (c == NULL) {
                     return;
                 }
             }
             vap = TAILQ_FIRST(&ic->ic_vaps);
             while (vap != NULL) {
                 if(vap->iv_opmode == IEEE80211_M_STA) {
                     wlan_scan_table_flush(vap);
                     mlme_indicate_sta_radar_detect(vap->iv_bss);
                     break;
                 }
                 vap = TAILQ_NEXT(vap, iv_next);
             }
             if (vap == NULL) {
             }
             return;
         }
#endif
     } /* End of else for STA mode */
 }
#if ATH_SUPPORT_IBSS_DFS
void ieee80211_ibss_beacon_update_start(struct ieee80211com *ic)
{
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    scn->sc_ops->ath_ibss_beacon_update_start(scn->sc_dev);
}

void ieee80211_ibss_beacon_update_stop(struct ieee80211com *ic)
{
    struct ath_softc_net80211 *scn = ATH_SOFTC_NET80211(ic);
    scn->sc_ops->ath_ibss_beacon_update_stop(scn->sc_dev);
}

/*
 * Build the ibss DFS element.
 */
u_int
ieee80211_create_dfs_channel_list(struct ieee80211vap *vap, struct channel_map_field *ch_map_list)
{
    u_int i, dfs_ch_count = 0;
    struct ieee80211com *ic = vap->iv_ic;

    for (i = 0; i < ic->ic_nchans; i++) {
        if((ic->ic_channels[i].ic_flagext & IEEE80211_CHAN_DFS) &&
            !ieee80211_check_mode_consistency(ic, ic->ic_curmode, &ic->ic_channels[i]))
        {
            ch_map_list[dfs_ch_count].ch_num = ic->ic_channels[i].ic_ieee;
            ch_map_list[dfs_ch_count].ch_map.bss = 0;
            ch_map_list[dfs_ch_count].ch_map.ofdem_preamble = 0;
            ch_map_list[dfs_ch_count].ch_map.und_signal = 0;
            ch_map_list[dfs_ch_count].ch_map.radar = 0;
            ch_map_list[dfs_ch_count].ch_map.unmeasured = 1;
            ch_map_list[dfs_ch_count].ch_map.reserved = 0;
            if (ic->ic_channels[i].ic_flags & IEEE80211_CHAN_RADAR) {
                ch_map_list[dfs_ch_count].ch_map.unmeasured = 0;
                ch_map_list[dfs_ch_count].ch_map.radar = 1;
            } else if (ic->ic_channels[i].ic_flagext & IEEE80211_CHAN_DFS_CLEAR) {
                ch_map_list[dfs_ch_count].ch_map.unmeasured = 0;
            }
            dfs_ch_count ++;
        }
    }
    return dfs_ch_count;
}

/*
* initialize a IBSS dfs ie
*/
void
ieee80211_build_ibss_dfs_ie(struct ieee80211vap *vap)
{
    u_int ch_count;
    if (vap->iv_ibss_dfs_enter_recovery_threshold_in_tbtt == 0) {
        vap->iv_ibss_dfs_enter_recovery_threshold_in_tbtt = INIT_IBSS_DFS_OWNER_RECOVERY_TIME_IN_TBTT;
        vap->iv_ibss_dfs_csa_measrep_limit = DEFAULT_MAX_CSA_MEASREP_ACTION_PER_TBTT;
        vap->iv_ibss_dfs_csa_threshold     = IEEE80211_RADAR_11HCOUNT;
    }
    OS_MEMZERO(vap->iv_ibssdfs_ie_data.ch_map_list, sizeof(struct channel_map_field) * (IEEE80211_CHAN_MAX + 1));
    vap->iv_ibssdfs_ie_data.ie = IEEE80211_ELEMID_IBSSDFS;
    vap->iv_ibssdfs_ie_data.rec_interval = vap->iv_ibss_dfs_enter_recovery_threshold_in_tbtt;
    ch_count = ieee80211_create_dfs_channel_list(vap, vap->iv_ibssdfs_ie_data.ch_map_list);
    vap->iv_ibssdfs_ie_data.len = IBSS_DFS_ZERO_MAP_SIZE + (sizeof(struct channel_map_field) * ch_count);
    vap->iv_measrep_action_count_per_tbtt = 0;
    vap->iv_csa_action_count_per_tbtt = 0;
    vap->iv_ibssdfs_recovery_count = vap->iv_ibss_dfs_enter_recovery_threshold_in_tbtt;
}

#endif /* ATH_SUPPORT_IBSS_DFS */
#else

void
ieee80211_mark_dfs(struct ieee80211com *ic, struct ieee80211_channel *ichan)
{
    return;
}

#if ATH_SUPPORT_IBSS_DFS
void ieee80211_ibss_beacon_update_start(struct ieee80211com *ic)
{
    return;
}

void ieee80211_ibss_beacon_update_stop(struct ieee80211com *ic)
{
    return;
}
#endif /* ATH_SUPPORT_IBSS_DFS */

#endif    // UMAC_SUPPORT_DFS
