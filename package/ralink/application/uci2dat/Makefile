#
# Copyright (C) 2010-2013 hua.shao@mediatek.com
#
# Ralink Property Software.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=uci2dat
PKG_RELEASE:=2

#PKG_BUILD_DIR:=$(BUILD_DIR)/$(PKG_NAME)
PKG_CONFIG_DEPENDS:=


PKG_SOURCE_PROTO:=git
PKG_SOURCE_URL:=ssh://git@121.194.169.198:8022/kt-package/uci2dat.git
PKG_SOURCE_SUBDIR:=$(PKG_NAME)-$(PKG_VERSION)
#PKG_SOURCE_VERSION:=master
PKG_SOURCE_VERSION:=cb47d1ffcb6d9eb13ee20bc2dae4b6210691d5e5
PKG_SOURCE:=$(PKG_NAME)-$(PKG_VERSION)-$(PKG_SOURCE_VERSION).tar.gz

PKG_BUILD_DIR := $(BUILD_DIR)/$(PKG_NAME)/$(PKG_NAME)-$(PKG_VERSION)


include $(INCLUDE_DIR)/package.mk

define Package/uci2dat
  SECTION:=Ralink Properties
  CATEGORY:=Ralink Properties
  DEPENDS:=+libuci
  TITLE:=Translate uci config into ralink wifi dat format
  SUBMENU:=Applications
endef

define Package/uci2dat/description
  Translate uci config into ralink wifi dat format
endef

#define Build/Prepare
#	mkdir -p $(PKG_BUILD_DIR)
#	$(CP) ./src/* $(PKG_BUILD_DIR)/
#endef

TARGET_CFLAGS += -luci -Wall
TARGET_CFLAGS += -Wno-error=format-security 

define Build/Configure
endef

define Package/uci2dat/install
	$(INSTALL_DIR) $(1)/usr/bin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/uci2dat $(1)/usr/bin
endef


$(eval $(call BuildPackage,uci2dat))

