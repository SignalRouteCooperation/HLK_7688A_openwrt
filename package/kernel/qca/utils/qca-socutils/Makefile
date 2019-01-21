#
# Copyright (c) 2013 Qualcomm Atheros, Inc..
#
# All Rights Reserved.
# Qualcomm Atheros Confidential and Proprietary.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=qca-socutils
PKG_VERSION:=1.0

include $(INCLUDE_DIR)/package.mk

define Package/dumpregs
  SECTION:=utils
  CATEGORY:=Utilities
  DEPENDS:=@TARGET_ar71xx +microperl +devmem2
  TITLE:=QCA SoC Register dumping command
  URL:=http://www.qca.qualcomm.com
endef

define Package/dumpregs/description
  Dump the SoC registers content in a user friendly way
endef

define Build/Prepare
endef

define Build/Compile
endef

define Package/dumpregs/install
	$(INSTALL_DIR) $(1)/usr/sbin
	$(INSTALL_BIN) ./src/dumpregs.pl $(1)/usr/sbin
endef

$(eval $(call BuildPackage,dumpregs))
