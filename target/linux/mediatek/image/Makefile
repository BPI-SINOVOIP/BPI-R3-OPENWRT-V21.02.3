# SPDX-License-Identifier: GPL-2.0-only
#
# Copyright (C) 2012-2015 OpenWrt.org
# Copyright (C) 2016-2017 LEDE project

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/image.mk

ifeq ($(SUBTARGET),mt7986)
define Image/Build/SDImage
	./make_bpi-r3_bundle_image.sh 	$(KDIR)/mtk-bpi-r3-SD-WAN1-SFP1.img \
                               		$(STAGING_DIR_IMAGE)/GPT_SD \
                               		$(STAGING_DIR_IMAGE)/bl2_sd.img \
                               		$(STAGING_DIR_IMAGE)/fip_sd.bin \
                               		$(KDIR)/BPI-R3-SD-WAN1-SFP1-kernel.bin \
                               		$(KDIR)/root.squashfs
	./make_bpi-r3_bundle_image.sh 	$(KDIR)/mtk-bpi-r3-SD-WAN1-RJ45.img \
                               		$(STAGING_DIR_IMAGE)/GPT_SD \
                               		$(STAGING_DIR_IMAGE)/bl2_sd.img \
                               		$(STAGING_DIR_IMAGE)/fip_sd.bin \
                               		$(KDIR)/BPI-R3-SD-WAN1-RJ45-kernel.bin \
                               		$(KDIR)/root.squashfs
endef

define Image/Build/EMMCImage
	./make_bpi-r3_bundle_image.sh 	$(KDIR)/mtk-bpi-r3-EMMC-WAN1-SFP1.img \
                               		$(STAGING_DIR_IMAGE)/GPT_EMMC \
                               		$(STAGING_DIR_IMAGE)/bl2_emmc.img \
                               		$(STAGING_DIR_IMAGE)/fip_emmc.bin \
                               		$(KDIR)/BPI-R3-EMMC-WAN1-SFP1-kernel.bin \
                               		$(KDIR)/root.squashfs
	./make_bpi-r3_bundle_image.sh 	$(KDIR)/mtk-bpi-r3-EMMC-WAN1-RJ45.img \
                               		$(STAGING_DIR_IMAGE)/GPT_EMMC \
                               		$(STAGING_DIR_IMAGE)/bl2_emmc.img \
                               		$(STAGING_DIR_IMAGE)/fip_emmc.bin \
                               		$(KDIR)/BPI-R3-EMMC-WAN1-RJ45-kernel.bin \
                               		$(KDIR)/root.squashfs
endef
endif

define Build/sysupgrade-emmc
	rm -f $@.recovery
	mkfs.fat -C $@.recovery 3070

	./gen_$(SUBTARGET)_emmc_img.sh $@ \
		$(IMAGE_KERNEL) \
		$@.recovery \
		$(IMAGE_ROOTFS)
endef

# default all platform image(fit) build 
define Device/Default
  PROFILES = Default $$(DEVICE_NAME)
  KERNEL_NAME := Image
  KERNEL = kernel-bin | lzma | \
	fit lzma $$(KDIR)/image-$$(firstword $$(DEVICE_DTS)).dtb
  KERNEL_INITRAMFS = kernel-bin | lzma | \
	fit lzma $$(KDIR)/image-$$(firstword $$(DEVICE_DTS)).dtb
  FILESYSTEMS := squashfs
  DEVICE_DTS_DIR := $(DTS_DIR)
  IMAGES := sysupgrade.bin
  IMAGE/sysupgrade.bin := append-kernel | pad-to 128k | append-rootfs | \
	pad-rootfs | append-metadata
endef

include $(SUBTARGET).mk

define Image/Build
	$(call Image/Build/SDImage,$(1))
	$(call Image/Build/EMMCImage,$(1))
	$(call Image/Build/$(1),$(1))
endef

$(eval $(call BuildImage))

