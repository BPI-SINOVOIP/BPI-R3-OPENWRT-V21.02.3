KERNEL_LOADADDR := 0x48080000

define Device/BPI-R3-SD-WAN1-SFP1
  DEVICE_VENDOR := Banana Pi
  DEVICE_MODEL := Banana Pi R3
  DEVICE_TITLE := MTK7986a BPI R3 SD WAN1 SFP1
  DEVICE_DTS := mt7986a-bananapi-bpi-r3-sd-wan1-sfp1
  DEVICE_DTS_DIR := $(DTS_DIR)/mediatek
  SUPPORTED_DEVICES := bananapi,bpi-r3
  DEVICE_PACKAGES := mkf2fs e2fsprogs blkid blockdev losetup kmod-fs-ext4 \
		     kmod-mmc kmod-fs-f2fs kmod-fs-vfat kmod-nls-cp437 \
		     kmod-nls-iso8859-1
  IMAGE/sysupgrade.bin := sysupgrade-tar | append-metadata
endef
TARGET_DEVICES += BPI-R3-SD-WAN1-SFP1

define Device/BPI-R3-SD-WAN1-RJ45
  DEVICE_VENDOR := Banana Pi
  DEVICE_MODEL := Banana Pi R3
  DEVICE_TITLE := MTK7986a BPI R3 SD WAN1 RJ45
  DEVICE_DTS := mt7986a-bananapi-bpi-r3-sd-wan1-rj45
  DEVICE_DTS_DIR := $(DTS_DIR)/mediatek
  SUPPORTED_DEVICES := bananapi,bpi-r3
  DEVICE_PACKAGES := mkf2fs e2fsprogs blkid blockdev losetup kmod-fs-ext4 \
		     kmod-mmc kmod-fs-f2fs kmod-fs-vfat kmod-nls-cp437 \
		     kmod-nls-iso8859-1
  IMAGE/sysupgrade.bin := sysupgrade-tar | append-metadata
endef
TARGET_DEVICES += BPI-R3-SD-WAN1-RJ45

define Device/BPI-R3-EMMC-WAN1-SFP1
  DEVICE_VENDOR := Banana Pi
  DEVICE_MODEL := Banana Pi R3
  DEVICE_TITLE := MTK7986a BPI R3 EMMC WAN1 SFP1
  DEVICE_DTS := mt7986a-bananapi-bpi-r3-emmc-wan1-sfp1
  DEVICE_DTS_DIR := $(DTS_DIR)/mediatek
  SUPPORTED_DEVICES := bananapi,bpi-r3
  DEVICE_PACKAGES := mkf2fs e2fsprogs blkid blockdev losetup kmod-fs-ext4 \
		     kmod-mmc kmod-fs-f2fs kmod-fs-vfat kmod-nls-cp437 \
		     kmod-nls-iso8859-1
  IMAGE/sysupgrade.bin := sysupgrade-tar | append-metadata
endef
TARGET_DEVICES += BPI-R3-EMMC-WAN1-SFP1

define Device/BPI-R3-EMMC-WAN1-RJ45
  DEVICE_VENDOR := Banana Pi
  DEVICE_MODEL := Banana Pi R3
  DEVICE_TITLE := MTK7986a BPI R3 EMMC WAN1 RJ45
  DEVICE_DTS := mt7986a-bananapi-bpi-r3-emmc-wan1-rj45
  DEVICE_DTS_DIR := $(DTS_DIR)/mediatek
  SUPPORTED_DEVICES := bananapi,bpi-r3
  DEVICE_PACKAGES := mkf2fs e2fsprogs blkid blockdev losetup kmod-fs-ext4 \
		     kmod-mmc kmod-fs-f2fs kmod-fs-vfat kmod-nls-cp437 \
		     kmod-nls-iso8859-1
  IMAGE/sysupgrade.bin := sysupgrade-tar | append-metadata
endef
TARGET_DEVICES += BPI-R3-EMMC-WAN1-RJ45

define Device/BPI-R3-NOR-WAN1-SFP1
  DEVICE_VENDOR := Banana Pi
  DEVICE_MODEL := Banana Pi R3
  DEVICE_TITLE := MTK7986a BPI R3 NOR WAN1 SFP1
  DEVICE_DTS := mt7986a-bananapi-bpi-r3-nor-wan1-sfp1
  DEVICE_DTS_DIR := $(DTS_DIR)/mediatek
  SUPPORTED_DEVICES := bananapi,bpi-r3
endef
TARGET_DEVICES += BPI-R3-NOR-WAN1-SFP1

define Device/BPI-R3-NOR-WAN1-RJ45
  DEVICE_VENDOR := Banana Pi
  DEVICE_MODEL := Banana Pi R3
  DEVICE_TITLE := MTK7986a BPI R3 NOR WAN1 RJ45
  DEVICE_DTS := mt7986a-bananapi-bpi-r3-nor-wan1-rj45
  DEVICE_DTS_DIR := $(DTS_DIR)/mediatek
  SUPPORTED_DEVICES := bananapi,bpi-r3
endef
TARGET_DEVICES += BPI-R3-NOR-WAN1-RJ45

define Device/BPI-R3-NAND-WAN1-SFP1
  DEVICE_VENDOR := Banana Pi
  DEVICE_MODEL := Banana Pi R3
  DEVICE_TITLE := MTK7986a BPI R3 NAND WAN1 SFP1
  DEVICE_DTS := mt7986a-bananapi-bpi-r3-nand-wan1-sfp1
  DEVICE_DTS_DIR := $(DTS_DIR)/mediatek
  SUPPORTED_DEVICES := bananapi,bpi-r3
  UBINIZE_OPTS := -E 5
  BLOCKSIZE := 128k
  PAGESIZE := 2048
  IMAGE_SIZE := 65536k
  KERNEL_IN_UBI := 1
  IMAGES += factory.bin
  IMAGE/factory.bin := append-ubi | check-size $$$$(IMAGE_SIZE)
  IMAGE/sysupgrade.bin := sysupgrade-tar | append-metadata
endef
TARGET_DEVICES += BPI-R3-NAND-WAN1-SFP1


define Device/BPI-R3-NAND-WAN1-RJ45
  DEVICE_VENDOR := Banana Pi
  DEVICE_MODEL := Banana Pi R3
  DEVICE_TITLE := MTK7986a BPI R3 NAND WAN1 RJ45
  DEVICE_DTS := mt7986a-bananapi-bpi-r3-nand-wan1-rj45
  DEVICE_DTS_DIR := $(DTS_DIR)/mediatek
  SUPPORTED_DEVICES := bananapi,bpi-r3
  UBINIZE_OPTS := -E 5
  BLOCKSIZE := 128k
  PAGESIZE := 2048
  IMAGE_SIZE := 65536k
  KERNEL_IN_UBI := 1
  IMAGES += factory.bin
  IMAGE/factory.bin := append-ubi | check-size $$$$(IMAGE_SIZE)
  IMAGE/sysupgrade.bin := sysupgrade-tar | append-metadata
endef
TARGET_DEVICES += BPI-R3-NAND-WAN1-RJ45
