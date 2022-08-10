#!/bin/bash
#
# There are 2 env-variables set for you, you can use it in your script.
# ${BUILD_DIR} , working dir of this script, eg: openwrt/lede/
# ${INSTALL_DIR}, where to install your build result, including: image, build log.
#

#Global variable
BUILD_TIME=`date +%Y%m%d%H%M%S`
build_flag=0

if [ -z ${BUILD_DIR} ]; then
	LOCAL=1
	BUILD_DIR=`pwd`
fi

MTK_FEED_DIR=${BUILD_DIR}/feeds/mtk_openwrt_feed
MTK_MANIFEST_FEED=${BUILD_DIR}/../mtk-openwrt-feeds

if [ -z ${INSTALL_DIR} ]; then
	INSTALL_DIR=autobuild_release
	mkdir -p ${INSTALL_DIR}
	if [ ! -d target/linux ]; then
		echo "You should call this scripts from openwrt's root directory."
	fi
fi

OPENWRT_VER=`cat ${BUILD_DIR}/feeds.conf.default | grep "src-git packages" | awk -F ";openwrt" '{print $2}'`
cp ${BUILD_DIR}/feeds.conf.default ${BUILD_DIR}/feeds.conf.default_ori

clean() {
	echo "clean start!"
	echo "It will take some time ......"
	make distclean
	rm -rf ${INSTALL_DIR}
	echo "clean done!"
}

do_patch(){
	files=`find $1 -name "*.patch" | sort`
	for file in $files
	do
	patch -f -p1 -i ${file} || exit 1
	done
}

prepare() {
	echo "Preparing...."
	#FIXME : workaround HOST PC build issue
	#cd package/mtk/applications/luci-app-mtk/;git checkout Makefile;cd -
	#mv package/mtk package/mtk_soc/ ./
	#rm -rf tmp/ feeds/ target/ package/ scripts/ tools/ include/ toolchain/ rules.mk
	#git checkout target/ package/ scripts/ tools/ include/ toolchain/ rules.mk
	#mv ./mtk ./mtk_soc/ package/
	cp ${BUILD_DIR}/autobuild/feeds.conf.default${OPENWRT_VER} ${BUILD_DIR}/feeds.conf.default

	#update feed
	${BUILD_DIR}/scripts/feeds update -a

        #check if manifest mtk_feed exist,if yes,overwrite and update it in feeds/
	if [ -d ${MTK_MANIFEST_FEED} ]; then
		rm -rf ${MTK_FEED_DIR}
		ln -s ${MTK_MANIFEST_FEED} ${MTK_FEED_DIR}
		${BUILD_DIR}/scripts/feeds update -a
	fi

	#do mtk_feed prepare_sdk.sh
	cp ${MTK_FEED_DIR}/prepare_sdk.sh ${BUILD_DIR}

	#if $1 exist(mt76), keep origin openwrt patches and remove mtk local eth driver
	if [ -z ${1} ]; then
		${BUILD_DIR}/prepare_sdk.sh ${MTK_FEED_DIR} || exit 1
        else
		${BUILD_DIR}/prepare_sdk.sh ${MTK_FEED_DIR} ${1} || exit 1
		rm -rf ${BUILD_DIR}/target/linux/mediatek/files-5.4/drivers/net/ethernet/mediatek/
	fi
	#install feed
	${BUILD_DIR}/scripts/feeds install -a
	${BUILD_DIR}/scripts/feeds install -a luci

	#do mtk_soc openwrt patch
	do_patch ${BUILD_DIR}/autobuild/openwrt_patches${OPENWRT_VER}/mtk_soc || exit 1
}

add_proprietary_kernel_files() {
	#cp mtk proprietary ko_module source to mtk target
	#and also need to be done in release mtk target
	mkdir -p ${BUILD_DIR}/target/linux/mediatek/files-5.4/drivers/net/wireless
	cp -rf ${BUILD_DIR}/../ko_module/gateway/proprietary_driver/drivers/wifi_utility/ ${BUILD_DIR}/target/linux/mediatek/files-5.4/drivers/net/wireless

	mkdir -p ${BUILD_DIR}/target/linux/mediatek/files-5.4/include/uapi/linux/
	cp -rf ${BUILD_DIR}/../kernel/wapp_uapi_includes ${BUILD_DIR}/target/linux/mediatek/files-5.4/include/uapi/linux/wapp

	cp -fpR ${BUILD_DIR}/autobuild/target/ ${BUILD_DIR}
}

prepare_mtwifi() {
	#remove officail OpenWRT wifi script
	#wifi-profile pkg will install wifi_jedi instead
	rm -rf ${BUILD_DIR}/package/base-files/files/sbin/wifi

	add_proprietary_kernel_files

	#do mtk_wifi openwrt patch
	do_patch ${BUILD_DIR}/autobuild/openwrt_patches${OPENWRT_VER}/mtk_wifi || exit 1
}

prepare_mac80211() {
	rm -rf ${BUILD_DIR}/package/network/services/hostapd
	cp -fpR ${BUILD_DIR}/./../mac80211_package/package/network/services/hostapd ${BUILD_DIR}/package/network/services

	rm -rf ${BUILD_DIR}/package/libs/libnl-tiny
	cp -fpR ${BUILD_DIR}/./../mac80211_package/package/libs/libnl-tiny ${BUILD_DIR}/package/libs

	rm -rf ${BUILD_DIR}/package/network/utils/iw
	cp -fpR ${BUILD_DIR}/./../mac80211_package/package/network/utils/iw ${BUILD_DIR}/package/network/utils

	rm -rf ${BUILD_DIR}/package/network/utils/iwinfo
	cp -fpR ${BUILD_DIR}/./../mac80211_package/package/network/utils/iwinfo ${BUILD_DIR}/package/network/utils

	rm -rf ${BUILD_DIR}/package/kernel/mac80211
	cp -fpR ${BUILD_DIR}/./../mac80211_package/package/kernel/mac80211 ${BUILD_DIR}/package/kernel

	rm -rf ${BUILD_DIR}/package/firmware/wireless-regdb
	cp -fpR ${BUILD_DIR}/./../mac80211_package/package/firmware/wireless-regdb ${BUILD_DIR}/package/firmware

	cp -fpR ${BUILD_DIR}/./../mac80211_package/package/kernel/mt76 ${BUILD_DIR}/package/kernel

	patch -f -p1 -i ${MTK_FEED_DIR}/autobuild_mac80211_release/0001-master-mac80211-generate-hostapd-setting-from-ap-cap.patch
	patch -f -p1 -i ${MTK_FEED_DIR}/autobuild_mac80211_release/0002-master-hostapd-makefile-for-utils.patch
	patch -f -p1 -i ${MTK_FEED_DIR}/autobuild_mac80211_release/0003-master-mt76-makefile-for-new-chip.patch
	cp -rfa ${MTK_FEED_DIR}/autobuild_mac80211_release/package/ ${BUILD_DIR}
	cp -rfa ${MTK_FEED_DIR}/autobuild_mac80211_release/target/ ${BUILD_DIR}
}

copy_main_Config() {
	echo cp -rfa autobuild/$1/.config ./.config
	cp -rfa autobuild/$1/.config ./.config
}

install_output_Image() {
	mkdir -p ${INSTALL_DIR}/$1

	files=`find bin/targets/$3/*${2}* -name "*.bin" -o -name "*.img"`
	file_count=0

	for file in $files
	do
		tmp=${file%.*}
		cp -rf $file ${INSTALL_DIR}/$1/${tmp##*/}-${BUILD_TIME}.${file##*.}
		((file_count++))
        done

	if [ ${file_count} = 0 ]; then
		if [ ${build_flag} -eq 0 ]; then
			let  build_flag+=1
			echo " Restart to debug-build with "make V=s -j1", starting......"
			build $1 -j1 || [ "$LOCAL" != "1" ]
		else
			echo " **********Failed to build $1, bin missing.**********"
		fi
	else
		echo "Install image OK!!!"
		echo "Build $1 successfully!"
	fi
}

install_output_Config() {
	echo cp -rfa autobuild/$1/.config ${INSTALL_DIR}/$1/openwrt.config
	cp -rfa autobuild/$1/.config ${INSTALL_DIR}/$1/openwrt.config
	[ -f tmp/kernel.config ] && cp tmp/kernel.config ${INSTALL_DIR}/$1/kernel.config
}

install_output_KernelDebugFile() {
	KernelDebugFile=bin/targets/$3/mt${2}*/kernel-debug.tar.bz2
	if [ -f ${KernelDebugFile} ]; then
		echo cp -rfa ${KernelDebugFile} ${INSTALL_DIR}/$1/kernel-debug.tar.bz2
		cp -rfa ${KernelDebugFile} ${INSTALL_DIR}/$1/kernel-debug.tar.bz2
	fi
}

install_output_RootfsDebugFile() {
	STAGING_DIR_ROOT=$(make -f "autobuild/get_stagingdir_root.mk" get-staging-dir-root)
	if [ -d ${STAGING_DIR_ROOT} ]; then
		STAGING_DIR_ROOT_PREFIX=$(dirname ${STAGING_DIR_ROOT})
		STAGING_DIR_ROOT_NAME=$(basename ${STAGING_DIR_ROOT})
		echo "tar -jcf ${INSTALL_DIR}/$1/rootfs-debug.tar.bz2 -C \"$STAGING_DIR_ROOT_PREFIX\" \"$STAGING_DIR_ROOT_NAME\""
		tar -jcf ${INSTALL_DIR}/$1/rootfs-debug.tar.bz2 -C "$STAGING_DIR_ROOT_PREFIX" "$STAGING_DIR_ROOT_NAME"
	fi
}

install_output_feeds_buildinfo() {
        feeds_buildinfo=$(find bin/targets/$3/*${2}*/ -name "feeds.buildinfo")
        echo "feeds_buildinfo=$feeds_buildinfo"
        if [ -f ${feeds_buildinfo} ]; then
                cp -rf $feeds_buildinfo ${INSTALL_DIR}/$1/feeds.buildinfo
        else
                echo "feeds.buildinfo is not found!!!"
        fi
}

install_release() {
	temp=${1#*mt}
	chip_name=${temp:0:4}
	temp1=`grep "CONFIG_TARGET_ramips=y" autobuild/$1/.config`

	if [ "${temp1}" == "CONFIG_TARGET_ramips=y" ]; then
		arch_name="ramips"
	else
		arch_name="mediatek"
	fi

	#install output image
	install_output_Image $1 ${chip_name} ${arch_name}

	#install output config
	install_output_Config $1

	#install output Kernel-Debug-File
	install_output_KernelDebugFile $1 ${chip_name} ${arch_name}

	#tar unstripped rootfs for debug symbols
	install_output_RootfsDebugFile $1

        #install output feeds buildinfo
        install_output_feeds_buildinfo $1 ${chip_name} ${arch_name}
}

prepare_final() {
	#cp customized autobuild SDK patches
	cp -fpR ${BUILD_DIR}/autobuild/$1/target/ ${BUILD_DIR}
	cp -fpR ${BUILD_DIR}/autobuild/$1/package/ ${BUILD_DIR}


	#cp special subtarget patches
	case $1 in
	mt7986*)
		cp -rf ${BUILD_DIR}/autobuild/mt7986-AX6000/target/linux/mediatek/patches-5.4/*.* ${BUILD_DIR}/target/linux/mediatek/patches-5.4
		;;
	*)
		;;
	esac

	#rm old legacy patch, ex old nfi nand driver
	case $1 in
	mt7986*|\
	mt7981*)
		rm -rf ${BUILD_DIR}/target/linux/mediatek/patches-5.4/0303-mtd-spinand-disable-on-die-ECC.patch
		;;
	*)
		;;
	esac

	cd ${BUILD_DIR}
	[ -f autobuild/$1/.config ] || {
		echo "unable to locate autobuild/$1/.config !"
		return
	}

	rm -rf ./tmp
	#copy main test config(.config)
	copy_main_Config $1

	echo make defconfig
	make defconfig
}

build() {
	echo "###############################################################################"
	echo "# $1"
	echo "###############################################################################"
	echo "build $1"

	cd ${BUILD_DIR}

    	#make
	echo make V=s $2
	make V=s $2 || exit 1

	#tar unstripped rootfs for debug symbols
	install_release $1
}
