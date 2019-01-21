#
# Copyright (C) 2013 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

# renamed by gukq 20160526 from kf-2010 to kt9661
define Profile/KT9661
	NAME:=KunTeng KT9661
	PACKAGES:=kmod-usb-core kmod-usb2 kmod-ledtrig-usbdev kmod-ath10k
endef

define Profile/KT9661/Description
  Package set optimized for the KunTeng kt-9661 device.
endef
$(eval $(call Profile,KT9661))
