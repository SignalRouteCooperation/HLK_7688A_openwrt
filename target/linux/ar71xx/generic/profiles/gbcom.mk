#
# Copyright (C) 2009-2010 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

define Profile/GB_CGW600
        NAME:=GBCOM CGW600 board
        PACKAGES:=kmod-usb-core kmod-usb2 kmod-usb-storage
endef

define Profile/GB_CGW600/Description
        Package set optimized for the GBCOM CWW600 board.
endef

$(eval $(call Profile,GB_CGW600))

define Profile/GB_CGW800
        NAME:=GBCOM CGW800 board
        PACKAGES:=kmod-usb-core kmod-usb2 kmod-usb-storage
endef

define Profile/GB_CGW800/Description
        Package set optimized for the GBCOM CWW800 board.
endef

$(eval $(call Profile,GB_CGW800))

define Profile/GB_TA2025AC
        NAME:=GBCOM TA2025AC board
        PACKAGES:=kmod-usb-core kmod-usb2 kmod-usb-storage
endef

define Profile/GB_TA2025AC/Description
        Package set optimized for the GBCOM TA2025AC board.
endef

$(eval $(call Profile,GB_TA2025AC))

define Profile/GB_OA5025
        NAME:=GBCOM OA5025 board
        PACKAGES:=kmod-usb-core kmod-usb2 kmod-usb-storage
endef

define Profile/GB_OA5025/Description
        Package set optimized for the GBCOM OA5025 board.
endef

$(eval $(call Profile,GB_OA5025))
