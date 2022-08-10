#!/bin/bash

MTK_FEEDS_DIR=${1}

if [ -f feeds.conf.default_ori ]; then
	OPENWRT_VER=`cat ./feeds.conf.default_ori | grep "src-git packages" | awk -F ";openwrt" '{print $2}'`
else
	OPENWRT_VER=`cat ./feeds.conf.default | grep "src-git packages" | awk -F ";openwrt" '{print $2}'`
fi

if [ -z ${1} ]; then
        MTK_FEEDS_DIR=feeds/mtk_openwrt_feed
fi

remove_patches(){
        echo "remove conflict patches"
        for aa in `cat ${MTK_FEEDS_DIR}/remove.patch.list`
        do
                echo "rm $aa"
                rm -rf ./$aa
        done
}

sdk_patch(){
	files=`find ${MTK_FEEDS_DIR}/openwrt_patches${OPENWRT_VER} -name "*.patch" | sort`
	for file in $files
	do
		patch -f -p1 -i ${file} || exit 1
	done
}

sdk_patch
#cp mtk target to OpenWRT
cp -fpR ${MTK_FEEDS_DIR}/target ./
cp -fpR ${MTK_FEEDS_DIR}/package${OPENWRT_VER}/* ./package
cp -fpR ${MTK_FEEDS_DIR}/tools ./
#remove patch if choose to not "keep" patch
if [ -z ${2} ]; then
	remove_patches
fi

