#
# Copyright (c) 2014 Qualcomm Atheros, Inc.
#
# All Rights Reserved.
# Qualcomm Atheros Confidential and Proprietary.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=qca-spectral

PKG_VERSION:=10.2.3.31
PKG_RELEASE:=1

#PKG_SOURCE:=$(PKG_NAME)-$(PKG_VERSION).tar.bz2

include $(INCLUDE_DIR)/package-version-override.mk
include $(INCLUDE_DIR)/package.mk

#PKG_BRANCH:=$(basename $(PKG_VERSION))
#PKG_SOURCE_URL:=http://tritium.qca.qualcomm.com/software/$(PKG_BRANCH)/AP$(PKG_VERSION:999.999.0%=QCAMain%)/src/components

define Package/qca-spectral
  SECTION:=QCA
  CATEGORY:=QCA Proprietary software
  URL:=http://www.qca.qualcomm.com
  MAINTAINER:=Qualcomm Atheros
  TITLE:=QCA Spectral Analysis
  DEPENDS:=kmod-qca-wifi +libpthread
endef

define KernelPackage/qca-spectral/config
$(call Package/$(PKG_NAME)/override_source_path,kmod-$(PKG_NAME))
$(call Package/$(PKG_NAME)/override_version,kmod-$(PKG_NAME),$(PKG_SUPPORTED_VERSION))
endef

MAKEOPTS+= \
	$(TARGET_CONFIGURE_OPTS) \
	COPTS="$(TARGET_CFLAGS)" \
	ATHEROSPATH=$(STAGING_DIR)/usr/include \
	ATH_SUPPORT_ICM=1 \
	ATH_SUPPORT_SPECTRAL=1 \
	QSDK_SPECTRAL=1

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) -rf ./src/qca-spectral-10.2.3.31/* $(PKG_BUILD_DIR)
endef

define Build/Compile
	$(MAKE) -C $(PKG_BUILD_DIR) $(MAKEOPTS)
endef

define Package/qca-spectral/install
	$(INSTALL_DIR) $(1)/usr/sbin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/athssd $(1)/usr/sbin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/icm $(1)/usr/sbin

	$(INSTALL_DIR) $(1)/lib/wifi
	$(INSTALL_BIN) ./files/icm.sh $(1)/lib/wifi

	$(INSTALL_DIR) $(1)/etc/ath
	$(INSTALL_DATA) ./files/icm.conf $(1)/etc/ath
endef

$(eval $(call BuildPackage,qca-spectral))
