include $(TOPDIR)/rules.mk

PKG_NAME:=kpsh
PKG_VERSION:=1
PKG_RELEASE:=1

PKG_BUILD_DIR := $(BUILD_DIR)/$(PKG_NAME)
include $(INCLUDE_DIR)/package.mk

define Package/$(PKG_NAME)
	SECTION:=Ralink Properties
	CATEGORY:=Ralink Properties
	TITLE:=emulate shell daemon for ATE control
	DEPENDS:=+libgnutls +libubox +libjson-c
	SUBMENU:=Applications
endef

define Package/$(PKG_NAME)/description
	Provide supporting ATE command read and reply
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Configure
endef

define Build/Compile
endef

TARGET_CFLAGS += -D_GNU_SOURCE

MAKE_FLAGS += \
	FPIC="$(FPIC)" \
	CFLAGS="$(TARGET_CFLAGS)" \
	LDFLAGS="$(TARGET_LDFLAGS)" \
	LIBS="-lgnutls -lcrypt -lubox -ljson-c"

define Package/$(PKG_NAME)/install
	$(INSTALL_DIR) $(1)/usr/sbin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/kpshd_$(ARCH_PACKAGES) $(1)/usr/sbin/kpshd
	$(INSTALL_DIR) $(1)/etc/init.d
	$(INSTALL_BIN) ./files/kpsh.init $(1)/etc/init.d/kpsh
endef

$(eval $(call BuildPackage,$(PKG_NAME)))
