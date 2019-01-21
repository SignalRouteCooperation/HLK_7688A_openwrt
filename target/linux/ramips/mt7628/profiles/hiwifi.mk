#
# Copyright (C) 2015 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

define Profile/HC5661A
	NAME:=HiWiFi HC5661A
	PACKAGES:=\
		kmod-usb-core kmod-usb-dwc2 kmod-usb2 \
		kmod-mmc-spi kmod-sdhci \
   		kmod-ledtrig-usbdev kmod-mt7628 apfree-wireless apfree_wifidog
endef

define Profile/HC5661A/Description
	Support HiWiFi HC5661A model
endef
$(eval $(call Profile,HC5661A))
