#
# Copyright (C) 2009-2010 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

define Profile/KT9672SC
	NAME:=KunTeng KT9672SC
	PACKAGES:=kmod-usb-core kmod-usb2 kmod-usb-storage
endef

define Profile/KT9672SC/Description
	Package set optimized for the KunTeng KT9672SC (Sicun ShangHai) reference board.
endef

$(eval $(call Profile,KT9672SC))
