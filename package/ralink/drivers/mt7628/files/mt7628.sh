#!/bin/sh
append DRIVERS "mt7628"

. /lib/wifi/ralink_common.sh

prepare_mt7628() {
	prepare_ralink_wifi mt7628
}

scan_mt7628() {
	scan_ralink_wifi mt7628 mt7628
}


disable_mt7628() {
	disable_ralink_wifi mt7628
}

enable_mt7628() {
	enable_ralink_wifi mt7628 mt7628
}

detect_mt7628() {
#	detect_ralink_wifi mt7628 mt7628
#victor@20160216 get wifi mac address from flash bdinfo partition
        . /lib/functions/system.sh
        . /lib/functions.sh
        local wifi_mac=`mtd_get_mac_ascii bdinfo "Vfac_mac "`
		[ -z $wifi_mac ] &&  wifi_mac=`mtd_get_mac_ascii bdinfo "&fac_mac "` 
		[ -z $wifi_mac ] &&  wifi_mac=`mtd_get_mac_ascii bdinfo "fac_mac "` 
		
        wifi_mac=`echo $wifi_mac|cut -c 10- | sed 's/://g' | awk '{print toupper($1)}'`

	hostname=K.T.Wi-Fi
	ssid=${hostname:-ApFree}_$wifi_mac
#end victor

	#ssid=mt7628-`ifconfig eth0 | grep HWaddr | cut -c 51- | sed 's/://g'`
	cd /sys/module/
	[ -d $module ] || return
	[ -e /etc/config/wireless ] && return
         cat <<EOF
config wifi-device      mt7628
        option type     mt7628
        option vendor   ralink
        option band     2.4G
        option channel  0
        option autoch   2
        option bw       0

config wifi-iface
        option device   mt7628
        option ifname   ra0
        option network  lan
        option mode     ap
        option ssid     $ssid
	option maxsta    25
        option encryption none
        option key      12345678
        option disabled 0
        option loadbalancebase 1

EOF


}


