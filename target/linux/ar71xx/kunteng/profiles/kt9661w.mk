#
# Copyright (C) 2013 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

define Profile/KT9661W
	NAME:=KunTeng KT9661W
	PACKAGES:=kmod-usb-core kmod-usb2 kmod-ledtrig-usbdev kmod-ath10k
endef

define Profile/KT9661W/Description
  Package set optimized for the KunTeng kt-9661w device.
endef
$(eval $(call Profile,KT9661W))
