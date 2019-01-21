#
# Copyright (C) 2015 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

define Profile/KT9962a
	NAME:=KunTeng KT9962a
	PACKAGES:=\
		kmod-ledtrig-netdev
endef

define Profile/KT9962a/Description
	Support KunTeng KT9962a model(MT7621-16M).
endef

$(eval $(call Profile,KT9962a))
