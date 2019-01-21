#
# Copyright (c) 2014 Qualcomm Atheros, Inc..
#
# All Rights Reserved.
# Qualcomm Atheros Confidential and Proprietary.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=qca-thermald
PKG_RELEASE:=1

#include $(INCLUDE_DIR)/local-development.mk
ifeq ($(DUMP)$(PKG_VERSION),)
  PKG_REV:=$(shell git ls-remote $(PKG_SOURCE_URL) $(PKG_BRANCH) | cut -b -7)
  PKG_VERSION:=g$(PKG_REV)
endif

QCA_THERMALD_MAKE_OPTS:= \
        CROSS=$(TARGET_CROSS) \
        THERMALDIR=$(PKG_BUILD_DIR) \

include $(INCLUDE_DIR)/package-version-override.mk
include $(INCLUDE_DIR)/package.mk

define Package/qca-thermald/config
  $(call Package/$(PKG_NAME)/override_version,$(PKG_NAME))
  $(call Package/$(PKG_NAME)/override_source_path,$(PKG_NAME))
endef

define Package/qca-thermald
  SECTION:=QCA
  CATEGORY:=QCA Proprietary software
  URL:=http://www.qca.qualcomm.com
  MAINTAINER:=Qualcomm Atheros
  DEPENDS:=+libpthread @TARGET_ipq806x
  TITLE:=Thermal Mitigation daemon
endef

define Package/qca-thermald/description
  This package is IPQ Thermal Mitigation daemon.
endef

define Build/Compile
	$(MAKE) -C $(PKG_BUILD_DIR) $(strip $(QCA_THERMALD_MAKE_OPTS))
endef

define Package/qca-thermald/install
	$(INSTALL_DIR) $(1)/usr/sbin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/install/usr/sbin/thermald $(1)/usr/sbin
	$(INSTALL_DIR) $(1)/etc/thermal
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/install/etc/thermal/ipq-thermald-8064.conf $(1)/etc/thermal
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/install/etc/thermal/ipq-thermald-8064.conf $(1)/etc/thermal
	$(INSTALL_DIR) $(1)/etc/init.d
	$(INSTALL_BIN) ./files/thermal.init $(1)/etc/init.d/thermal
	$(INSTALL_DIR) $(1)/etc/config
	$(INSTALL_BIN) ./files/thermal.config $(1)/etc/config/thermal
endef

$(eval $(call BuildPackage,qca-thermald))
