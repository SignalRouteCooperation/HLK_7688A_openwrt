#
# Copyright (c) 2013 Qualcomm Atheros, Inc..
#
# All Rights Reserved.
# Qualcomm Atheros Confidential and Proprietary.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=LinuxART2
PKG_VERSION:=CS10.2v4.9.354
PKG_RELEASE:=1

#PKG_BRANCH:=$(basename $(PKG_VERSION))
#PKG_SOURCE:=$(PKG_NAME)$(PKG_VERSION).tar.bz2
#PKG_SOURCE_URL:=http://tritium.qca.qualcomm.com/software/ART2/$(PKG_NAME)$(PKG_BRANCH)/$(PKG_NAME)$(PKG_VERSION)

PKG_BUILD_DIR:=$(KERNEL_BUILD_DIR)/$(PKG_NAME)$(PKG_VERSION)

include $(INCLUDE_DIR)/package-version-override.mk
include $(INCLUDE_DIR)/package.mk

#########################################################
############ ART DRIVER BUILD CONFIGURATION ############
#########################################################

ART2_APP_OPTS+= \
	$(TARGET_CONFIGURE_OPTS) \
	ART_ROOT=$(PKG_BUILD_DIR) \
	WORKAREA=$(PKG_BUILD_DIR) \
	TOOL_PREFIX=$(TARGET_CROSS)

EXTRA_CFLAGS+=	\
	-DUSE_PLATFORM_FRAMEWORK=1 \
	-DCFG_64BIT=0 \
	-I$(PKG_BUILD_DIR) \
	-I$(PKG_BUILD_DIR)/driver/linux/modules/include \

ifneq ($(CONFIG_DEBUG),)
ART2_APP_OPTS+= \
	GDB=1
endif

ifeq ($(BOARD),ipq806x)
ART2_APP_OPTS+=AKRONITE=1
EXTRA_CFLAGS+=-DAKRONITE_ART
endif

ifeq ($(BOARD),ar71xx)
EXTRA_CFLAGS+= -DAP83 -DWASP
endif

define KernelPackage/art2
    SECTION:=QCA
    CATEGORY:=QCA Proprietary software
    URL:=http://www.qca.qualcomm.com
    MAINTAINER:=Qualcomm Atheros
    TITLE:=$(1) - QCA ART Driver & tools
    DEPENDS:=+libpthread +libc +librt
    FILES:=$(PKG_BUILD_DIR)/driver/linux/modules/art.$(LINUX_KMOD_SUFFIX)
endef

define KernelPackage/art2/Description
    This package contains a driver for ART and the tools to use it:
      - art.ko
      - nart.out
      - NART related libs
    It has been build specifically for $(1)
endef

define KernelPackage/art2/config
$(call Package/$(PKG_NAME)/override_source_path,kmod-art2)
$(call Package/$(PKG_NAME)/override_version,kmod-art2,$(PKG_SUPPORTED_VERSION))
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) -rf ./src/LinuxART2CS10.2v4.9.354/* $(PKG_BUILD_DIR)
endef

define Build/Compile
	$(MAKE) -C "$(LINUX_DIR)" \
		CROSS_COMPILE="$(TARGET_CROSS)" \
		ARCH="$(LINUX_KARCH)" \
		SUBDIRS="$(PKG_BUILD_DIR)/driver/linux/modules" \
		EXTRA_CFLAGS="$(EXTRA_CFLAGS)" \
		modules
	$(MAKE) -C $(PKG_BUILD_DIR) $(strip $(ART2_APP_OPTS)) \
		-f makefile.nart all
endef

define KernelPackage/art2/install
	$(INSTALL_DIR) $(1)/usr/sbin
	$(INSTALL_DIR) $(1)/usr/lib
	$(INSTALL_DIR) $(1)/etc/init.d
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/nartbuild/*.out $(1)/usr/sbin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/nartbuild/*.so $(1)/usr/lib
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/BoardData/*.bin $(1)/usr/sbin
	$(INSTALL_BIN) ./files/art.init $(1)/etc/init.d/art
endef

$(eval $(call KernelPackage,art2))
