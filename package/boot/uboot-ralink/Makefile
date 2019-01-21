#
# Copyright (C) 2010 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=u-boot
PKG_VERSION:=2015.09
PKG_RELEASE:=1

PKG_BUILD_DIR:=$(KERNEL_BUILD_DIR)/$(BUILD_VARIANT)/$(PKG_NAME)-$(PKG_VERSION)
PKG_MD5SUM:=
PKG_TARGETS:=bin

include $(INCLUDE_DIR)/package.mk


define Build/Prepare
	$(call Build/Prepare/Default)
	$(CP) -r src/. $(PKG_BUILD_DIR)
	$(CP) $(PKG_BUILD_DIR)/autoconf_$(BUILD_VARIANT).h $(PKG_BUILD_DIR)/autoconf.h
	$(CP) $(PKG_BUILD_DIR)/config-$(BUILD_VARIANT) $(PKG_BUILD_DIR)/.config
endef

define Build/Configure
endef

define Build/Compile
	$(MAKE) -C $(PKG_BUILD_DIR) \
		CROSS_COMPILE=$(TARGET_CROSS)
endef



#############  uboot-zc9525a
define Package/uboot-zc9525a
  SECTION:=boot
  CATEGORY:=Boot Loaders
  TITLE:=U-boot for zc9525a board
  DEPENDS:=@TARGET_ramips
  URL:=http://www.denx.de/wiki/U-Boot
  DEFAULT:=y if (CONFIG_TARGET_ramips_rt305x)
  VARIANT:=$(1)
endef



define Package/uboot-zc9525a/install
	$(CP) $(PKG_BUILD_DIR)/uboot_128k.bin $(BIN_DIR)/openwrt-$(BUILD_VARIANT).bin
endef


#############  uboot-zc9525
define Package/uboot-zc9525
  SECTION:=boot
  CATEGORY:=Boot Loaders
  TITLE:=U-boot for zc9525 board
  DEPENDS:=@TARGET_ramips
  URL:=http://www.denx.de/wiki/U-Boot
  DEFAULT:=y if (CONFIG_TARGET_ramips_rt305x)
  VARIANT:=$(1)
endef


define Package/uboot-zc9525/install
	$(CP) $(PKG_BUILD_DIR)/uboot_128k.bin $(BIN_DIR)/openwrt-$(BUILD_VARIANT).bin
endef




#############  uboot-kt9761
define Package/uboot-kt9761
  SECTION:=boot
  CATEGORY:=Boot Loaders
  TITLE:=U-boot for kt9761 board
  DEPENDS:=@TARGET_ramips
  URL:=http://www.denx.de/wiki/U-Boot
  DEFAULT:=y if (CONFIG_TARGET_ramips_rt305x)
  VARIANT:=$(1)
endef


define Package/uboot-kt9761/install
	$(CP) $(PKG_BUILD_DIR)/uboot_128k.bin $(BIN_DIR)/openwrt-$(BUILD_VARIANT).bin
endef



$(eval $(call BuildPackage,uboot-zc9525a))
$(eval $(call BuildPackage,uboot-zc9525))
$(eval $(call BuildPackage,uboot-kt9761))

