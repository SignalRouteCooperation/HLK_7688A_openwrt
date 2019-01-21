#!/bin/sh
append DRIVERS "mt7620"

. /lib/wifi/ralink_common.sh

prepare_mt7620() {
	prepare_ralink_wifi mt7620
}

scan_mt7620() {
	scan_ralink_wifi mt7620 mt7620
}


disable_mt7620() {
	disable_ralink_wifi mt7620
}

enable_mt7620() {
	enable_ralink_wifi mt7620 mt7620
}

detect_mt7620() {
#	detect_ralink_wifi mt7620 mt7620

#victor@20151223 get wifi mac address from flash bdinfo partition
	. /lib/functions/system.sh
	. /lib/functions.sh
	local wifi_mac=`mtd_get_mac_ascii bdinfo "Vfac_mac "`
	wifi_mac=`echo $wifi_mac|cut -c 10- | sed 's/://g' | awk '{print toupper($1)}'`
#end victor

	#hostname=`uci -q get system.@system[0].hostname`
	hostname=K.T.Wi-Fi
	#ssid=${hostname:-ApFree}-`ifconfig eth0 | grep HWaddr | cut -c 51- | sed 's/://g'`
	ssid=${hostname:-ApFree}_$wifi_mac
	cd /sys/module/
	[ -d $module ] || return
	[ -e /etc/config/wireless ] && return
         cat <<EOF
config wifi-device      mt7620
        option type     mt7620
        option vendor   ralink
        option band     2.4G
        option channel  0
    	option autoch   2
        option bw       0

config wifi-iface
        option device   mt7620
        option ifname   ra0
        option network  lan
        option mode     ap
        option ssid     $ssid
	option maxsta    25
        option encryption none
        option key      12345678
        option apcli_enable	0
        option disabled 0
        option loadbalancebase 1

EOF


}


