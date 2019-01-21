#
# Copyright (C) 2015 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

define Profile/ZC9525A
	NAME:=KunTeng ZC9525A
	PACKAGES:=\
		kmod-usb-core kmod-usb-dwc2 kmod-usb2 \
		kmod-mmc-spi kmod-sdhci \
   		kmod-ledtrig-usbdev kmod-mt7628 apfree-wireless apfree_wifidog
endef

define Profile/ZC9525A/Description
	Support KunTeng ZC9525A model
endef
$(eval $(call Profile,ZC9525A))

define Profile/KT9761
	NAME:=KunTeng KT9761
	PACKAGES:=\
		kmod-usb-core kmod-usb-dwc2 kmod-usb2 \
		kmod-mmc-spi kmod-sdhci \
   		kmod-ledtrig-usbdev kmod-mt7628 kmod-mt7612e apfree-wireless apfree_wifidog
endef

define Profile/KT9761/Description
	Support KunTeng KT9761 model
endef
$(eval $(call Profile,KT9761))

define Profile/KT9762
	NAME:=KunTeng KT9762
	PACKAGES:=\
		kmod-usb-core kmod-usb-dwc2 kmod-usb2 \
		kmod-mmc-spi kmod-sdhci \
   		kmod-ledtrig-usbdev kmod-mt7628 kmod-mt7612e apfree-wireless apfree_wifidog
endef

define Profile/KT9762/Description
	Support KunTeng KT9762 model
endef
$(eval $(call Profile,KT9762))

define Profile/KT8761SK
	NAME:=KunTeng KT8761SK
	PACKAGES:=\
		kmod-usb-core kmod-usb-dwc2 kmod-usb2 \
		kmod-mmc-spi kmod-sdhci \
		kmod-ledtrig-usbdev kmod-mt7628 apfree-wireless apfree_wifidog
endef

define Profile/KT8761SK/Description
	Support KunTeng KT8761SK model
endef
$(eval $(call Profile,KT8761SK))
