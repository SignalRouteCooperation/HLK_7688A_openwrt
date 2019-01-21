#!/bin/sh
#
# Copyright (c) 2014, The Linux Foundation. All rights reserved.
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#


IFNAME=$1
CMD=$2
. /sbin/wifi detect

local parent=$(cat /sys/class/net/${IFNAME}/parent)

is_section_ifname() {
	local config=$1
	local ifname
	config_get ifname "$config" ifname
	[ "${ifname}" = "$2" ] && echo ${config}
}

case "$CMD" in
	WPS-NEW-AP-SETTINGS)
		local ssid=$(hostapd_cli -i$IFNAME -p/var/run/hostapd-$parent wps_get_config | grep ^ssid= | cut -f2- -d =)
		local wpa=$(hostapd_cli -i$IFNAME -p/var/run/hostapd-$parent wps_get_config | grep ^wpa= | cut -f2- -d=)
		local psk=$(hostapd_cli -i$IFNAME -p/var/run/hostapd-$parent wps_get_config | grep ^wpa_passphrase= | cut -f2- -d=)
		local wps_state=$(hostapd_cli -i$IFNAME -p/var/run/hostapd-$parent wps_get_config | grep ^wps_state= | cut -f2- -d=)
		local section=$(config_foreach is_section_ifname wifi-iface $IFNAME)

		case "$wpa" in
			3)
				uci set wireless.${section}.encryption='mixed-psk'
				uci set wireless.${section}.key=$psk
				;;
			2)
				uci set wireless.${section}.encryption='psk2'
				uci set wireless.${section}.key=$psk
				;;
			1)
				uci set wireless.${section}.encryption='psk'
				uci set wireless.${section}.key=$psk
				;;
			*)
				uci set wireless.${section}.encryption='none'
				uci set wireless.${section}.key=''
				;;
		esac

		uci set wireless.${section}.ssid="$ssid"
		uci set wireless.${section}.wps_state=$wps_state
		uci commit
		kill "$(cat "/var/run/hostapd-cli-$IFNAME.pid")"
		#post hotplug event to whom take care of
		env -i ACTION="wps-configured" INTERFACE=$IFNAME /sbin/hotplug-call iface
		;;
	WPS-TIMEOUT)
		kill "$(cat "/var/run/hostapd-cli-$IFNAME.pid")"
		env -i ACTION="wps-timeout" INTERFACE=$IFNAME /sbin/hotplug-call iface
		;;
	DISCONNECTED)
		kill "$(cat "/var/run/hostapd_cli-$IFNAME.pid")"
		;;
esac
