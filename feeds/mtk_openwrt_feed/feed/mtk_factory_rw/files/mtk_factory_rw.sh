#!/bin/sh

usage()
{
	echo "This is a script to get or set mtk factory's data"
	echo "-Typically, get or set the eth lan/wan mac_address-"
	echo "Usage1: $0 <op> <side> [mac_address] "
	echo "	<op>: -r or -w (Read or Write action)"
	echo "	[mac_address]: MAC[1] MAC[2] MAC[3] MAC[4] MAC[5] MAC[6] (only for write action)"
	echo "Usage2: $0 <op> <length> <offset> [data] "
	echo "	<length>: length bytes of input"
	echo "	<offset>: Skip offset bytes from the beginning of the input"
	echo "Usage3: $0 -o <length> <get_from> <overwrite_to>"
	echo "Example:"
	echo "$0 -w lan 00 0c 43 68 55 56"
	echo "$0 -r lan"
	echo "$0 -w 8 0x22 11 22 33 44 55 66 77 88"
	echo "$0 -r 8 0x22"
	echo "$0 -o 12 0x24 0x7fff4"
	exit 1
}

factory_name="Factory"
factory_mtd=/dev/$(grep -i ''${factory_name}'' /proc/mtd | cut -c 1-4)

#default:7622
lan_mac_offset=0x2A
wan_mac_offset=0x24

case `cat /tmp/sysinfo/board_name` in
	*7621*ax*)
		# 256k - 12 byte
		lan_mac_offset=0x3FFF4
		wan_mac_offset=0x3FFFA
		;;
	*7621*)
		lan_mac_offset=0xe000
		wan_mac_offset=0xe006
		;;
	*7622*)
		#512k -12 byte
		lan_mac_offset=0x7FFF4
		wan_mac_offset=0x7FFFA
		;;
	*7623*)
		lan_mac_offset=0x1F800
		wan_mac_offset=0x1F806
		;;
	*)
		lan_mac_offset=0x2A
		wan_mac_offset=0x24
		;;
esac

#1.Read the offset's data from the Factory
#usage: Get_offset_data length offset
Get_offset_data()
{
	local length=$1
	local offset=$2

	hexdump -v -n ${length} -s ${offset} -e ''`expr ${length} - 1`'/1 "%02x-" "%02x "' ${factory_mtd}
}

overwrite_data=

Get_offset_overwrite_data()
{
        local length=$1
        local offset=$2

        overwrite_data=`hexdump -v -n ${length} -s ${offset} -e ''\`expr ${length} - 1\`'/1 "%02x " " %02x"' ${factory_mtd}`
}

#2.Write the offset's data from the Factory
#usage: Set_offset_data length offset data
Set_offset_data()
{
	local length=$1
	local offset=$2
	local index=`expr $# - ${length} + 1`
	local data=""

	for j in $(seq ${index} `expr ${length} + ${index} - 1`)
	do
		temp=`eval echo '$'{"$j"}`
		data=${data}"\x${temp}"
	done

	dd if=${factory_mtd} of=/tmp/Factory.backup
	printf "${data}" | dd conv=notrunc of=/tmp/Factory.backup bs=1 seek=$((${offset}))
	mtd write /tmp/Factory.backup ${factory_name}
	rm -rf /tmp/Factory.backup
}

#3.Read Factory lan/wan mac address
GetMac()
{
	if [ "$1" == "lan" ]; then
		#read lan mac
		Get_offset_data 6 ${lan_mac_offset}
	elif [ "$1" == "wan" ]; then
		#read wan mac
		Get_offset_data 6 ${wan_mac_offset}
	else
		usage
		exit 1
	fi
}


#4.write Factory lan/wan mac address
SetMac()
{
	if [ "$#" != "9" ]; then
		echo "Mac address must be 6 bytes!"
		exit 1
	fi

	if [ "$1" == "lan" ]; then
		#write lan mac
		Set_offset_data 6 ${lan_mac_offset} $@

	elif [ "$1" == "wan" ]; then
		#write wan mac
		Set_offset_data 6 ${wan_mac_offset} $@
	else
		usage
		exit 1
	fi
}

#usage:
# 1. Set/Get the mac_address: mtk_factory -r/-w lan/wan /data
# 2. Set/Get the offset data: mtk_factory -r/-w length offset /data
# 3. Overwrite from offset1 to offset2 by length byte : mtk_factory -o length from to
if [ "$1" == "-r" ]; then
	if [ "$2" == "lan" -o "$2" == "wan" ]; then
		GetMac $2
	elif [ "$2" -eq "$2" ]; then
		Get_offset_data $2 $3
	else
		echo "Unknown command!"
		usage
		exit 1
	fi
elif [ "$1" == "-w" ]; then
	if [ "$2" == "lan" -o "$2" == "wan" ]; then
		SetMac $2 $@
	else
		Set_offset_data $2 $3 $@
	fi
elif [ "$1" == "-o" ]; then
	Get_offset_overwrite_data $2 $3
	Set_offset_data $2 $4 ${overwrite_data}
else
	echo "Unknown command!"
	usage
	exit 1
fi
