#!/bin/bash

getMyIP() {
    local _ip _myip _line _nl=$'\n'
    while IFS=$': \t' read -a _line ;do
        [ -z "${_line%inet}" ] &&
           _ip=${_line[${#_line[1]}>4?1:2]} &&
           [ "${_ip#127.0.0.1}" ] && _myip=$_ip
      done< <(LANG=C /sbin/ifconfig)
    printf ${1+-v} $1 "%s${_nl:0:$[${#1}>0?0:1]}" $_myip
}

echo url="https://www.duckdns.org/update?domains=tumgispinode.duckdns.org&token=XXXXXXXXXXXXXXXXXX&ip=$(getMyIP)&verbose=TRUE" | curl -k -o /home/pi/duckdns/duck.log -K -


