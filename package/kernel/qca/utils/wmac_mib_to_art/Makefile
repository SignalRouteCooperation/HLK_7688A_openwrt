#
# Copyright (c) 2013 Qualcomm Atheros, Inc..
#
# All Rights Reserved.
# Qualcomm Atheros Confidential and Proprietary.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=wmac_mib_to_art
PKG_VERSION:=G1.0

PKG_BUILD_DIR := $(BUILD_DIR)/$(PKG_NAME)

include $(INCLUDE_DIR)/package.mk

define Package/wmac_mib_to_art
  SECTION:=QCA
  CATEGORY:=QCA Proprietary software
  DEPENDS:=@TARGET_ar71xx 
  TITLE:=QCA SoC wmac mib to art
endef

define Package/wmac_mib_to_art/description
  Wmac from mib0 to art
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

# gukq added init.d fields for exected at router starting
define Package/wmac_mib_to_art/install
	$(INSTALL_DIR) $(1)/usr/bin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/$(PKG_NAME) $(1)/usr/bin/
	$(INSTALL_DIR) $(1)/etc/init.d
	$(INSTALL_BIN) ./files/init.d/$(PKG_NAME) $(1)/etc/init.d/$(PKG_NAME)
endef

$(eval $(call BuildPackage,wmac_mib_to_art))
