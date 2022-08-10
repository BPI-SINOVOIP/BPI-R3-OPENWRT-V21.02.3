#!/bin/ash

interface=$1	# phy0/phy1/ra0
cmd_type=$2	    # set/show/e2p/mac
full_cmd=$3
interface_ori=${interface}

work_mode="RUN" # RUN/PRINT/DEBUG
tmp_file="/tmp/iwpriv_wrapper"
phy_idx=$(echo ${interface} | tr -dc '0-9')

function do_cmd() {
    case ${work_mode} in
        "RUN")
            eval "$1"
            ;;
        "PRINT")
            echo "$1"
            ;;
        "DEBUG")
            eval "$1"
            echo "$1"
            ;;
    esac
}

function print_debug() {
    if [ "${work_mode}" = "DEBUG" ]; then
        echo "$1"
    fi
}

function write_dmesg() {
    echo "$1" > /dev/kmsg
}

function record_config() {
    if [ -f ${tmp_file} ]; then
        if grep -q $1 ${tmp_file}; then
            sed -i "/$1/c\\$1=$2" ${tmp_file}
        else
            echo "$1=$2" >> ${tmp_file}
        fi
    else
        echo "$1=$2" >> ${tmp_file}
    fi
}

function get_config() {
    if [ ! -f ${tmp_file} ]; then
        echo ""
        return
    fi

    if grep -q $1 ${tmp_file}; then
        echo "$(cat ${tmp_file} | grep $1 | sed s/=/' '/g | cut -d " " -f 2)"
    else
        echo ""
    fi
}

function simple_convert() {
    if [ "$1" = "ATETXCNT" ]; then
        echo "tx_count"
    elif [ "$1" = "ATETXLEN" ]; then
        echo "tx_length"
    elif [ "$1" = "ATETXMCS" ]; then
        echo "tx_rate_idx"
    elif [ "$1" = "ATEVHTNSS" ]; then
        echo "tx_rate_nss"
    elif [ "$1" = "ATETXLDPC" ]; then
        echo "tx_rate_ldpc"
    elif [ "$1" = "ATETXSTBC" ]; then
        echo "tx_rate_stbc"
    elif [ "$1" = "ATEPKTTXTIME" ]; then
        echo "tx_time"
    elif [ "$1" = "ATEIPG" ]; then
        echo "tx_ipg"
    elif [ "$1" = "ATEDUTYCYCLE" ]; then
        echo "tx_duty_cycle"
    elif [ "$1" = "ATETXFREQOFFSET" ]; then
        echo "freq_offset"
    else
        echo "unknown param: $1"
    fi
}

function convert_tx_mode() {
    if [ "$1" = "0" ]; then
        echo "cck"
    elif [ "$1" = "1" ]; then
        echo "ofdm"
    elif [ "$1" = "2" ]; then
        echo "ht"
    elif [ "$1" = "4" ]; then
        echo "vht"
    elif [ "$1" = "8" ]; then
        echo "he_su"
    elif [ "$1" = "9" ]; then
        echo "he_er"
    elif [ "$1" = "10" ]; then
        echo "he_tb"
    elif [ "$1" = "11" ]; then
        echo "he_mu"
    else
        echo "unknown tx mode: $1"
    fi
}

function convert_gi {
    local tx_mode=$1
    local val=$2
    local sgi="0"
    local he_ltf="0"

    case ${tx_mode} in
        "ht"|"vht")
            sgi=${val}
            ;;
        "he_su"|"he_er")
            case ${val} in
                "0")
                    ;;
                "1")
                    he_ltf="1"
                    ;;
                "2")
                    sgi="1"
                    he_ltf="1"
                    ;;
                "3")
                    sgi="2"
                    he_ltf="2"
                    ;;
                "4")
                    he_ltf="2"
                    ;;
                *)
                    echo "unknown gi"
            esac
            ;;
        "he_mu")
            case ${val} in
                "0")
                    he_ltf="2"
                    ;;
                "1")
                    he_ltf="1"
                    ;;
                "2")
                    sgi="1"
                    he_ltf="1"
                    ;;
                "3")
                    sgi="2"
                    he_ltf="2"
                    ;;
                *)
                    echo "unknown gi"
            esac
            ;;
        "he_tb")
            case ${val} in
                "0")
                    sgi="1"
                    ;;
                "1")
                    sgi="1"
                    he_ltf="1"
                    ;;
                "2")
                    sgi="2"
                    he_ltf="2"
                    ;;
                *)
                    echo "unknown gi"
            esac
            ;;
        *)
            print_debug "legacy mode no need gi"
    esac

    do_cmd "mt76-test ${interface} set tx_rate_sgi=${sgi} tx_ltf=${he_ltf}"
}

function convert_channel {
    local band=$(echo $1 | sed s/:/' '/g | cut -d " " -f 2)
    local ch=$(echo $1 | sed s/:/' '/g | cut -d " " -f 1)
    local bw=$(get_config "ATETXBW" | cut -d ":" -f 1)
    local bw_str="HT20"

    if [[ $1 != *":"* ]] || [ "${band}" = "0" ]; then
        case ${bw} in
            "1")
                if [ "${ch}" -lt "3" ] || [ "${ch}" -gt "12" ]; then
                    local bw_str="HT20"
                else
                    local bw_str="HT40+"
                    ch=$(expr ${ch} - "2")
                fi
                ;;
        esac
    elif [ "${band}" = "1" ]; then
        case ${bw} in
            "5")
                bw_str="160MHz"
                if [ ${ch} -lt "68" ]; then
                    ch="36"
                elif [ ${ch} -lt "100" ]; then
                    ch="68"
                elif [ ${ch} -lt "132" ]; then
                    ch="100"
                elif [ ${ch} -lt "181" ]; then
                    ch="149"
                fi
                ;;
            "2")
                bw_str="80MHz"
                if [ ${ch} -lt "52" ]; then
                    ch="36"
                elif [ ${ch} -lt "68" ]; then
                    ch="52"
                elif [ ${ch} -lt "84" ]; then
                    ch="68"
                elif [ ${ch} -lt "100" ]; then
                    ch="84"
                elif [ ${ch} -lt "116" ]; then
                    ch="100"
                elif [ ${ch} -lt "132" ]; then
                    ch="116"
                elif [ ${ch} -lt "149" ]; then
                    ch="132"
                elif [ ${ch} -lt "165" ]; then
                    ch="149"
                elif [ ${ch} -lt "181" ]; then
                    ch="165"
                fi
                ;;
            "1")
                if [ ${ch} -lt "44" ]; then
                    ch=$([ "${ch}" -lt "40" ] && echo "36" || echo "40")
                    bw_str=$([ "${ch}" -le "38" ] && echo "HT40+" || echo "HT40-")
                elif [ ${ch} -lt "52" ]; then
                    ch=$([ "${ch}" -lt "48" ] && echo "44" || echo "48")
                    bw_str=$([ "${ch}" -le "46" ] && echo "HT40+" || echo "HT40-")
                elif [ ${ch} -lt "60" ]; then
                    ch=$([ "${ch}" -lt "56" ] && echo "52" || echo "56")
                    bw_str=$([ "${ch}" -le "54" ] && echo "HT40+" || echo "HT40-")
                elif [ ${ch} -lt "68" ]; then
                    ch=$([ "${ch}" -lt "64" ] && echo "60" || echo "64")
                    bw_str=$([ "${ch}" -le "62" ] && echo "HT40+" || echo "HT40-")
                elif [ ${ch} -lt "76" ]; then
                    ch=$([ "${ch}" -lt "72" ] && echo "68" || echo "72")
                    bw_str=$([ "${ch}" -le "70" ] && echo "HT40+" || echo "HT40-")
                elif [ ${ch} -lt "84" ]; then
                    ch=$([ "${ch}" -lt "80" ] && echo "76" || echo "80")
                    bw_str=$([ "${ch}" -le "78" ] && echo "HT40+" || echo "HT40-")
                elif [ ${ch} -lt "92" ]; then
                    ch=$([ "${ch}" -lt "88" ] && echo "84" || echo "88")
                    bw_str=$([ "${ch}" -le "86" ] && echo "HT40+" || echo "HT40-")
                elif [ ${ch} -lt "100" ]; then
                    ch=$([ "${ch}" -lt "96" ] && echo "92" || echo "96")
                    bw_str=$([ "${ch}" -le "94" ] && echo "HT40+" || echo "HT40-")
                elif [ ${ch} -lt "108" ]; then
                    ch=$([ "${ch}" -lt "104" ] && echo "100" || echo "104")
                    bw_str=$([ "${ch}" -le "102" ] && echo "HT40+" || echo "HT40-")
                elif [ ${ch} -lt "116" ]; then
                    ch=$([ "${ch}" -lt "112" ] && echo "108" || echo "112")
                    bw_str=$([ "${ch}" -le "110" ] && echo "HT40+" || echo "HT40-")
                elif [ ${ch} -lt "124" ]; then
                    ch=$([ "${ch}" -lt "120" ] && echo "116" || echo "120")
                    bw_str=$([ "${ch}" -le "118" ] && echo "HT40+" || echo "HT40-")
                elif [ ${ch} -lt "132" ]; then
                    ch=$([ "${ch}" -lt "128" ] && echo "124" || echo "128")
                    bw_str=$([ "${ch}" -le "126" ] && echo "HT40+" || echo "HT40-")
                elif [ ${ch} -lt "140" ]; then
                    ch=$([ "${ch}" -lt "136" ] && echo "132" || echo "136")
                    bw_str=$([ "${ch}" -le "134" ] && echo "HT40+" || echo "HT40-")
                elif [ ${ch} -lt "149" ]; then
                    ch=$([ "${ch}" -lt "144" ] && echo "140" || echo "144")
                    bw_str=$([ "${ch}" -le "142" ] && echo "HT40+" || echo "HT40-")
                elif [ ${ch} -lt "157" ]; then
                    ch=$([ "${ch}" -lt "153" ] && echo "149" || echo "153")
                    bw_str=$([ "${ch}" -le "151" ] && echo "HT40+" || echo "HT40-")
                elif [ ${ch} -lt "165" ]; then
                    ch=$([ "${ch}" -lt "161" ] && echo "157" || echo "161")
                    bw_str=$([ "${ch}" -le "159" ] && echo "HT40+" || echo "HT40-")
                elif [ ${ch} -lt "173" ]; then
                    ch=$([ "${ch}" -lt "169" ] && echo "165" || echo "169")
                    bw_str=$([ "${ch}" -le "167" ] && echo "HT40+" || echo "HT40-")
                elif [ ${ch} -lt "181" ]; then
                    ch=$([ "${ch}" -lt "177" ] && echo "173" || echo "177")
                    bw_str=$([ "${ch}" -le "175" ] && echo "HT40+" || echo "HT40-")
                fi
                ;;
            "0")
                local bw_str="HT20"
                ;;
        esac
    else
        echo "6G Todo"
    fi

    do_cmd "iw dev mon${phy_idx} set channel ${ch} ${bw_str}"
}

function convert_rxstat {
    local res=$(do_cmd "mt76-test ${interface} dump stats")
    local mdrdy=$(echo "${res}" | grep "rx_packets" | cut -d "=" -f 2)
    local fcs_error=$(echo "${res}" | grep "rx_fcs_error" | cut -d "=" -f 2)
    local rcpi=$(echo "${res}" | grep "last_rcpi" | cut -d "=" -f 2 | sed 's/,/ /g')
    local ib_rssi=$(echo "${res}" | grep "last_ib_rssi" | cut -d "=" -f 2 | sed 's/,/ /g')
    local wb_rssi=$(echo "${res}" | grep "last_wb_rssi" | cut -d "=" -f 2 | sed 's/,/ /g')
    local rx_ok=$(expr ${mdrdy} - ${fcs_error})

    write_dmesg "rcpi: ${rcpi}"
    write_dmesg "fagc rssi ib: ${ib_rssi}"
    write_dmesg "fagc rssi wb: ${wb_rssi}"
    write_dmesg "all_mac_rx_mdrdy_cnt: ${mdrdy}"
    write_dmesg "all_mac_rx_fcs_err_cnt: ${fcs_error}"
    write_dmesg "all_mac_rx_ok_cnt : ${rx_ok}"
}

function change_band_idx {
    local old_idx=$(get_config "ATECTRLBANDIDX")
    local new_idx=$1

    if [ -z "${old_idx}" ] && [ "${new_idx}" == "0" ]; then
        return
    fi

    if [ "${old_idx}" != "${new_idx}" ]; then
            if [ -z "${old_idx}" ]; then
                old_idx=0
            fi

            interface="phy${old_idx}"
            phy_idx=${old_idx}
            do_ate_work "ATESTOP"

            interface="phy${new_idx}"
            phy_idx=${new_idx}
            do_ate_work "ATESTART"

            record_config "ATECTRLBANDIDX" ${new_idx}
    fi
}

function set_mac_addr {
    record_config ${cmd} ${param}

    local addr1=$(get_config "ATEDA")
    local addr2=$(get_config "ATESA")
    local addr3=$(get_config "ATEBSSID")

    if [ -z "${addr1}" ]; then
        addr1="00:11:22:33:44:55"
    fi
    if [ -z "${addr2}" ]; then
        addr2="00:11:22:33:44:55"
    fi
    if [ -z "${addr3}" ]; then
        addr3="00:11:22:33:44:55"
    fi

    do_cmd "mt76-test phy${phy_idx} set mac_addrs=${addr1},${addr2},${addr3}"
}

function do_ate_work() {
    local ate_cmd=$1

    case ${ate_cmd} in
        "ATESTART")
            local if_str=$(ifconfig | grep mon${phy_idx})

            if [ ! -z "${if_str}" -a "${if_str}" != " " ]; then
                echo "ATE already starts."
            else
                do_cmd "iw phy ${interface} interface add mon${phy_idx} type monitor"
                do_cmd "iw dev wlan${phy_idx} del"
                do_cmd "ifconfig mon${phy_idx} up"
                do_cmd "iw reg set VV"
            fi
            ;;
        "ATESTOP")
            local if_str=$(ifconfig | grep mon${phy_idx})

            if [ -z "${if_str}" -a "${if_str}" != " " ]; then
                echo "ATE does not start."
            else
                do_cmd "mt76-test ${interface} set state=off"
                do_cmd "iw dev mon${phy_idx} del"
                do_cmd "iw phy ${interface} interface add wlan${phy_idx} type managed"
                do_cmd "mt76-test ${interface} set aid=0"
            fi

            rm ${tmp_file} > /dev/null 2>&1
            ;;
        "TXCOMMIT")
            do_cmd "mt76-test ${interface} set aid=1"
            ;;
        "TXFRAME")
            do_cmd "mt76-test ${interface} set state=tx_frames"
            ;;
        "TXSTOP"|"RXSTOP")
            do_cmd "mt76-test ${interface} set state=idle"
            ;;
        "TXREVERT")
            do_cmd "mt76-test ${interface} set aid=0"
            ;;
        "RXFRAME")
            do_cmd "mt76-test ${interface} set state=rx_frames"
            ;;
        "TXCONT")
            do_cmd "mt76-test ${interface} set state=tx_cont"
            ;;
        *)
            print_debug "skip ${ate_cmd}"
            ;;
    esac
}

# main start here

if [[ ${interface} == "ra"* ]]; then
    tmp=$(get_config "ATECTRLBANDIDX")
    if [ ! -z "${tmp}" ]; then
        interface="phy${tmp}"
        phy_idx=${tmp}
    else
        interface="phy0"
        phy_idx=0
    fi
fi

cmd=$(echo ${full_cmd} | sed s/=/' '/g | cut -d " " -f 1)
param=$(echo ${full_cmd} | sed s/=/' '/g | cut -d " " -f 2)

if [ "${cmd_type}" = "set" ]; then
    skip=0
    case ${cmd} in
        "ATE")
            do_ate_work ${param}

            skip=1
            ;;
        "ATETXCNT"|"ATETXLEN"|"ATETXMCS"|"ATEVHTNSS"|"ATETXLDPC"|"ATETXSTBC"| \
        "ATEPKTTXTIME"|"ATEIPG"|"ATEDUTYCYCLE"|"ATETXFREQOFFSET")
            cmd_new=$(simple_convert ${cmd})
            param_new=${param}
            ;;
        "ATETXANT"|"ATERXANT")
            cmd_new="tx_antenna"
            param_new=${param}
            ;;
        "ATETXGI")
            tx_mode=$(convert_tx_mode $(get_config "ATETXMODE"))
            convert_gi ${tx_mode} ${param}
            skip=1
            ;;
        "ATETXMODE")
            cmd_new="tx_rate_mode"
            param_new=$(convert_tx_mode ${param})
            record_config ${cmd} ${param}
            ;;
        "ATETXPOW0"|"ATETXPOW1"|"ATETXPOW2"|"ATETXPOW3")
            cmd_new="tx_power"
            param_new="${param},0,0,0"
            ;;
        "ATETXBW")
            record_config ${cmd} ${param}
            skip=1
            ;;
        "ATECHANNEL")
            convert_channel ${param}
            skip=1
            ;;
        "ATERXSTAT")
            convert_rxstat
            skip=1
            ;;
        "ATECTRLBANDIDX")
            change_band_idx ${param}
            skip=1
            ;;
        "ATEDA"|"ATESA"|"ATEBSSID")
            set_mac_addr ${cmd} ${param}
            skip=1
            ;;
        "bufferMode")
            if [ "${param}" = "2" ]; then
                do_cmd "atenl -i ${interface} -c \"eeprom update buffermode\""
            fi
            skip=1
            ;;
        "ResetCounter"|"ATERXSTATRESET")
            skip=1
            ;;
        *)
            print_debug "Unknown command to set: ${cmd}"
            skip=1
    esac

    if [ "${skip}" != "1" ]; then
        do_cmd "mt76-test ${interface} set ${cmd_new}=${param_new}"
    fi

elif [ "${cmd_type}" = "show" ]; then
    do_cmd "mt76-test ${interface} dump"
    do_cmd "mt76-test ${interface} dump stats"

elif [ "${cmd_type}" = "e2p" ]; then
    offset=$(printf "0x%s" ${cmd})
    val=$(printf "0x%s" ${param})

    # eeprom offset write
    if [[ ${full_cmd} == *"="* ]]; then
        tmp=$((${val} & 0xff))
        tmp=$(printf "0x%x" ${tmp})
        do_cmd "atenl -i ${interface} -c \"eeprom set ${offset}=${tmp}\""

        offset=$((${offset}))
        offset=$(expr ${offset} + "1")
        offset=$(printf "0x%x" ${offset})
        tmp=$(((${val} >> 8) & 0xff))
        tmp=$(printf "0x%x" ${tmp})
        do_cmd "atenl -i ${interface} -c \"eeprom set ${offset}=${tmp}\""
    else
        v1=$(do_cmd "atenl -i ${interface} -c \"eeprom read ${param}\"")
        v1=$(echo "${v1}" | grep "val =" | cut -d '(' -f 2 | grep -o -E '[0-9]+')

        tmp=$(printf "0x%s" ${param})
        tmp=$((${tmp}))
        param2=$(expr ${tmp} + "1")
        param2=$(printf "%x" ${param2})
        v2=$(do_cmd "atenl -i ${interface} -c \"eeprom read ${param2}\"")
        v2=$(echo "${v2}" | grep "val =" | cut -d '(' -f 2 | grep -o -E '[0-9]+')

        param=$(printf "0x%s" ${param})
        param=$(printf "%04x" ${param})
        param=$(echo $param | tr 'a-z' 'A-Z')
        printf "%s       e2p:\n" ${interface_ori}
        printf "[0x%s]:0x%02x%02x\n" ${param} ${v2} ${v1}
    fi

elif [ "${cmd_type}" = "mac" ]; then
    regidx=/sys/kernel/debug/ieee80211/phy${phy_idx}/mt76/regidx
    regval=/sys/kernel/debug/ieee80211/phy${phy_idx}/mt76/regval
    offset=$(printf "0x%s" ${cmd})
    val=$(printf "0x%s" ${param})

    echo ${offset} > ${regidx}
    # reg write
    if [[ ${full_cmd} == *"="* ]]; then
        echo ${val} > ${regval}
    fi

    res=$(cat ${regval} | cut -d 'x' -f 2)
    printf "%s       mac:[%s]:%s\n" ${interface_ori} ${offset} ${res}

else
    echo "Unknown command"
fi
