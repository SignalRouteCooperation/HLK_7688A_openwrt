#
# Copyright (c) 2014 Qualcomm Atheros, Inc..
#
# All Rights Reserved.
# Qualcomm Atheros Confidential and Proprietary.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=qca-wifi-fw
PKG_VERSION:=10.2.2.78
PKG_RELEASE:=1

PKG_SOURCE:=$(PKG_NAME)-$(PKG_VERSION).tar.bz2
PKG_BRANCH:=$(basename $(PKG_VERSION))
PKG_SOURCE_URL:=http://tritium.qca.qualcomm.com/software/$(PKG_BRANCH)/AP$(PKG_VERSION:999.999.0%=QCAMain%)/src
PKG_BUILD_DIR:=$(BUILD_DIR)/$(PKG_NAME)-$(PKG_VERSION)

include $(INCLUDE_DIR)/package-version-override.mk
include $(INCLUDE_DIR)/package.mk

define Package/qca-wifi-fw
  SECTION:=QCA
  CATEGORY:=QCA Proprietary software
  URL:=http://www.qca.qualcomm.com
  MAINTAINER:=Qualcomm Atheros
  TITLE:=QCA Wifi Firmware
endef

define Package/qca-wifi-fw/description
  This package contains the qca-wifi firmware used in 10.x drivers
endef

define Package/qca-wifi-fw/config
$(call Package/$(PKG_NAME)/override_source_path,$(PKG_NAME))
$(call Package/$(PKG_NAME)/override_version,$(PKG_NAME),$(PKG_SUPPORTED_VERSION))
endef

define Package/qca-wifi-fw/install
	$(INSTALL_DIR) $(1)/lib/firmware/AR9888
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/AR9888/*.bin $(1)/lib/firmware/AR9888/

	$(INSTALL_DIR) $(1)/lib/firmware/AR9887
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/AR9887/*.bin $(1)/lib/firmware/AR9887/
endef

Build/Compile=true

$(eval $(call BuildPackage,qca-wifi-fw))
