#!/bin/sh
. /lib/functions.sh

CURL_GROUP_CACHE="/etc/clash/clashbackup/clash_gorup.json"
CURL_NOW_CACHE="/etc/clash/clashbackup/clash_now.json"
CURL_CACHE="/etc/clash/clashbackup/clash_curl.json"
HISTORY_PATH="/etc/clash/clashbackup/history"
SECRET=$(uci get clash.config.dash_pass 2>/dev/null)
LAN_IP=$(uci get network.lan.ipaddr 2>/dev/null |awk -F '/' '{print $1}' 2>/dev/null)
PORT=$(uci get clash.config.dash_port 2>/dev/null)

curl -m 5 --retry 2 -w %{http_code}"\n" -H "Authorization: Bearer ${SECRET}" -H "Content-Type:application/json" -X GET http://"$LAN_IP":"$PORT"/proxies > "$CURL_CACHE" 2>/dev/null
if [ "$(sed -n '$p' "$CURL_CACHE" 2>/dev/null)" = "200" ]; then
	if [ ! -d /etc/clash/clashbackup ];then
		mkdir -p /etc/clash/clashbackup 2>/dev/null
	fi
	cat "$CURL_CACHE" |jsonfilter -e '@["proxies"][@.type="Selector"]["name"]' > "$CURL_GROUP_CACHE" 2>/dev/null
	cat "$CURL_CACHE" |jsonfilter -e '@["proxies"][@.type="Selector"]["now"]' > "$CURL_NOW_CACHE" 2>/dev/null
   awk 'NR==FNR{a[i]=$0;i++}NR>FNR{print a[j]"#*#"$0;j++}' "$CURL_GROUP_CACHE" "$CURL_NOW_CACHE" > "$HISTORY_PATH" 2>/dev/null
fi
rm -rf /etc/clash/clashbackup/clash_*.json  2>/dev/null 
