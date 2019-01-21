#
# Copyright (C) 2015 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#


define Profile/ZC9525
	NAME:=KunTeng ZC9525
	PACKAGES:=\
		kmod-usb-core kmod-usb-dwc2 kmod-usb2 \
		kmod-mmc-spi kmod-sdhci kmod-sdhci-mt7620 \
   		kmod-ledtrig-usbdev kmod-mt7620 apfree-wireless
endef

define Profile/ZC9525/Description
	Support KunTeng ZC9525 model(J1S)
endef
$(eval $(call Profile,ZC9525))


define Profile/ZC9526
	NAME:=KunTeng ZC9526
	PACKAGES:=\
		kmod-usb-core kmod-usb-dwc2 kmod-usb2 \
		kmod-mmc-spi kmod-sdhci kmod-sdhci-mt7620 \
		kmod-ledtrig-usbdev kmod-mt7620 kmod-mt7610e
endef

define Profile/ZC9526/Description
	Support KunTeng ZC9526 model(J2)
endef
$(eval $(call Profile,ZC9526))


define Profile/ZC9527
	NAME:=KunTeng ZC9527
	PACKAGES:=\
		kmod-usb-core kmod-usb-dwc2 kmod-usb2 \
		kmod-mmc-spi kmod-sdhci kmod-sdhci-mt7620 \
		kmod-ledtrig-usbdev kmod-mt7620 kmod-mt7610e bridged_ap
endef

define Profile/ZC9527/Description
	Support KunTeng ZC9527 mode(bridged-ap support)
endef
$(eval $(call Profile,ZC9527))
