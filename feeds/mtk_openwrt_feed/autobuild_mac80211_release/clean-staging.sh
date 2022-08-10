#!/bin/sh

# This script is used to remove all changes made by autobuild scripts which
# allows autobuild scripts to be called again

# ATTENTION: commit all changes you made before running this script otherwise
#            all changes will be lost!

if [ ! -d target/linux ]; then
	echo "This script must be called from the root directory of OpenWrt!"
	exit 1
fi

git clean -f -d
git checkout .
rm -rf feeds/
rm -rf package/feeds
git -C package checkout .
git -C package/mtk/applications/luci-app-mtk checkout .
