#
# Copyright (C) 2015 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

define Profile/HC5962
	NAME:=HiWiFi HC5962
	PACKAGES:=\
		kmod-usb-core kmod-usb3 kmod-usb-hid \
		kmod-ledtrig-netdev
endef

define Profile/HC5962/Description
	Support HiWiFi HC5962 Gee 4.
endef

#-m <min io size> -e <LEB size> -c <Eraseblocks count>
HC5962_UBIFS_OPTS:="-m 2048 -e 129024 -c 1024"
HC5962_UBI_OPTS:="-m 2048 -p 128KiB -s 512"

$(eval $(call Profile,HC5962))

define Profile/KT9962
	NAME:=KunTeng KT9962
	PACKAGES:=\
		kmod-usb-core kmod-usb3 kmod-usb-hid \
		kmod-ledtrig-netdev
endef

define Profile/KT9962/Description
	Support KunTeng KT9962 model(MT7621-NAND).
endef

#-m <min io size> -e <LEB size> -c <Eraseblocks count>
KT9962_UBIFS_OPTS:="-m 2048 -e 129024 -c 1024"
KT9962_UBI_OPTS:="-m 2048 -p 128KiB -s 512"

$(eval $(call Profile,KT9962))

define Profile/KT9990
	NAME:=KunTeng KT9990
	PACKAGES:=\
		kmod-usb-core kmod-usb3 kmod-usb-hid \
		kmod-ledtrig-netdev
endef

define Profile/KT9990/Description
	Support KunTeng KT9990 model(MT7621-NAND).
endef

#-m <min io size> -e <LEB size> -c <Eraseblocks count>
KT9990_UBIFS_OPTS:="-m 2048 -e 129024 -c 1024"
KT9990_UBI_OPTS:="-m 2048 -p 128KiB -s 512"

$(eval $(call Profile,KT9990))
