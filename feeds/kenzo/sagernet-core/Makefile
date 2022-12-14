# SPDX-License-Identifier: GPL-3.0-only
#
# Copyright (C) 2021 ImmortalWrt.org

include $(TOPDIR)/rules.mk

PKG_NAME:=sagernet-core
BASE_VERSION:=5.0.16
PKG_RELEASE:=1

PKG_SOURCE_PROTO:=git
PKG_SOURCE_URL:=https://github.com/SagerNet/v2ray-core.git
PKG_SOURCE_DATE:=2022-06-27
PKG_SOURCE_VERSION:=d81715258c3bc178eaeaccbd9bc19ad1dc2d971f
PKG_MIRROR_HASH:=61b522a994d0ba9f08d81bb81625b2063d5e35c62e0e13a06322e19ddc5088ad
PKG_VERSION:=$(BASE_VERSION)-$(PKG_SOURCE_DATE)-$(call version_abbrev,$(PKG_SOURCE_VERSION))

PKG_LICENSE:=GPL-3.0-or-later
PKG_LICENSE_FILES:=LICENSE
PKG_MAINTAINER:=Tianling Shen <cnsztl@immortalwrt.org>

PKG_BUILD_DEPENDS:=golang/host
PKG_BUILD_PARALLEL:=1
PKG_USE_MIPS16:=0

GO_PKG:=github.com/v2fly/v2ray-core/v5
GO_PKG_BUILD_PKG:=$(GO_PKG)/main
GO_PKG_LDFLAGS_X:= \
	$(GO_PKG).build=OpenWrt \
	$(GO_PKG).version=$(BASE_VERSION)

include $(INCLUDE_DIR)/package.mk
include $(TOPDIR)/feeds/packages/lang/golang/golang-package.mk

define Package/sagernet/template
  TITLE:=An enhanced v2ray edition for SagerNet
  SECTION:=net
  CATEGORY:=Network
  URL:=https://sagernet.org
endef

define Package/sagernet-core
  $(call Package/sagernet/template)
  DEPENDS:=$(GO_ARCH_DEPENDS) +ca-bundle
  CONFLICTS:=v2ray-core xray-core
endef

define Package/sagernet-extra
  $(call Package/sagernet/template)
  TITLE+= (extra resources)
  DEPENDS:=sagernet-core
  CONFLICTS:=v2ray-extra
  PKGARCH:=all
endef

define Package/sagernet/description
  Project V is a set of network tools that help you to build your own computer network.
  It secures your network connections and thus protects your privacy.
endef

define Package/sagernet-core/description
  $(call Package/sagernet/description)

  This is a v2ray-core fork for SagerNet.
endef

define Package/sagernet-extra/description
  $(call Package/sagernet/description)

  This includes extra resources for sagernet-core.
endef

LIBCORE_NAME:=LibSagerNetCore
LIBCORE_DATE:=2022-06-10
LIBCORE_VERSION:=9ef7ab1de7604ab959222d4de8ab6152a402e432
LIBCORE_FILE:=$(LIBCORE_NAME)-$(LIBCORE_DATE)-$(call version_abbrev,$(LIBCORE_VERSION)).tar.xz
define Download/libcore
	PROTO:=git
	URL:=https://github.com/SagerNet/LibSagerNetCore.git
	VERSION:=$(LIBCORE_VERSION)
	FILE:=$(LIBCORE_FILE)
	SUBDIR:=$(LIBCORE_NAME)
	MIRROR_HASH:=9420189c45d5fbe682acee1da0b8f727cc5ea8c847ceeff5b026cd71ce60cf0f
endef

define Build/Prepare
	$(call Build/Prepare/Default)

	xzcat $(DL_DIR)/$(LIBCORE_FILE) | tar -C $(PKG_BUILD_DIR) $(TAR_OPTIONS)
	$(CP) $(PKG_BUILD_DIR)/$(LIBCORE_NAME)/{obfs,ssr}.go \
		$(PKG_BUILD_DIR)/proxy/shadowsocks/plugin/self/
	$(SED) '/plugin\/self/d' -e 's/self.ParsePluginOptions/ParsePluginOptions/g' \
		$(PKG_BUILD_DIR)/proxy/shadowsocks/plugin/self/obfs.go
	$(SED) 's/package libcore/package self/g' \
		$(PKG_BUILD_DIR)/proxy/shadowsocks/plugin/self/{obfs,ssr}.go
endef

define Package/sagernet-core/install
	$(call GoPackage/Package/Install/Bin,$(PKG_INSTALL_DIR))
	$(INSTALL_DIR) $(1)/usr/bin/
	$(INSTALL_BIN) $(PKG_INSTALL_DIR)/usr/bin/main $(1)/usr/bin/v2ray
	$(LN) v2ray $(1)/usr/bin/xray
endef

define Package/sagernet-extra/install
	$(INSTALL_DIR) $(1)/usr/share/v2ray/
	$(CP) $(PKG_BUILD_DIR)/release/extra/* $(1)/usr/share/v2ray/
endef

$(eval $(call Download,libcore))
$(eval $(call BuildPackage,sagernet-core))
$(eval $(call BuildPackage,sagernet-extra))
