#
# Copyright (c) 2014 Qualcomm Atheros, Inc
#
# All Rights Reserved.
# Qualcomm Atheros Confidential and Proprietary.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=qca-wpc

PKG_VERSION:=10.2.2.76
PKG_RELEASE:=1

PKG_BUILD_DIR:=$(BUILD_DIR)/$(PKG_NAME)-$(PKG_VERSION)
PKG_SOURCE:=$(PKG_NAME)-$(PKG_VERSION).tar.bz2

include $(INCLUDE_DIR)/package-version-override.mk
include $(INCLUDE_DIR)/package.mk

PKG_BRANCH:=$(basename $(PKG_VERSION))
PKG_SOURCE_URL:=http://tritium.qca.qualcomm.com/software/$(PKG_BRANCH)/AP$(PKG_VERSION:999.999.0%=QCAMain%)/src/components

define Package/qca-wpc
  SECTION:=QCA
  CATEGORY:=QCA Proprietary software
  URL:=http://www.qca.qualcomm.com
  MAINTAINER:=Qualcomm Atheros
  TITLE:=QCA Wifi Positioning
  DEPENDS:=kmod-qca-wifi +libpthread
endef

define KernelPackage/qca-wpc/config
$(call Package/$(PKG_NAME)/override_source_path,kmod-$(PKG_NAME))
$(call Package/$(PKG_NAME)/override_version,kmod-$(PKG_NAME),$(PKG_SUPPORTED_VERSION))
endef

MAKEOPTS+= \
	$(TARGET_CONFIGURE_OPTS) \
	COPTS="$(TARGET_CFLAGS)" \
	ATHEROSPATH=$(STAGING_DIR)/usr/include \
	ATH_SUPPORT_WIFIPOS=1

define Build/RunMake
	CFLAGS="$(TARGET_CPPFLAGS) $(TARGET_CFLAGS)" \
	$(MAKE) -C $(PKG_BUILD_DIR)/$(1) \
	$(MAKEOPTS) \
	$(TARGET_CONFIGURE_OPTS) \
	LIBS="$(TARGET_LDFLAGS)" \
	$(2)
endef

define Build/Compile/qca-wpc-rtte
	$(call Build/RunMake,wpc_rtte)
endef

define Build/Compile/qca-wpc
	$(call Build/RunMake,wpc, \
		wpc \
	)
endef

define Build/Compile
	$(Build/Compile/qca-wpc-rtte)
	$(Build/Compile/qca-wpc)
endef

define Package/qca-wpc/install
	$(INSTALL_DIR) $(1)/usr/sbin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/wpc/wpc $(1)/usr/sbin

	$(INSTALL_DIR) $(1)/lib/wifi
	$(INSTALL_BIN) ./files/wpc.sh $(1)/lib/wifi
endef

$(eval $(call BuildPackage,qca-wpc))
