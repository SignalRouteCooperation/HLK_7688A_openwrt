#!/bin/sh
append DRIVERS "mt7610e"

. /lib/wifi/ralink_common.sh

prepare_mt7610e() {
	prepare_ralink_wifi mt7610e
}

scan_mt7610e() {
	scan_ralink_wifi mt7610e mt7610e
}

disable_mt7610e() {
	disable_ralink_wifi mt7610e
}

enable_mt7610e() {
	enable_ralink_wifi mt7610e mt7610e
}

detect_mt7610e() {
#	detect_ralink_wifi mt7610e mt7610e

#victor@20151223 get wifi mac address from flash bdinfo partition
	. /lib/functions/system.sh
	. /lib/functions.sh
	local wifi_mac=`mtd_get_mac_ascii bdinfo "Vfac_mac "`
	wifi_mac=`echo $wifi_mac|cut -c 10- | sed 's/://g' | awk '{print toupper($1)}'`
#end victor

	#hostname=`uci -q get system.@system[0].hostname`
	hostname=K.T.Wi-Fi
	#ssid=${hostname:-ApFree}_5G-`ifconfig eth0 | grep HWaddr | cut -c 51- | sed 's/://g'`
	ssid=${hostname:-ApFree}_$wifi_mac
	cd /sys/module
	[ -d $module ] || return
        [ -e /etc/config/wireless ] && return
        cat <<EOF
config wifi-device      mt7610e
        option type     mt7610e
        option vendor   ralink
        option band     5G
        option channel  153
	option autoch   2
	option version	1

config wifi-iface
        option device   mt7610e
        option ifname   rai0
        option network  lan
        option mode     ap
        option ssid     $ssid
	option maxsta    25
        option encryption none
        option key      12345678
        option loadbalancebase 1

EOF


}


