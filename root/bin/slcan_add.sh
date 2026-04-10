#!/bin/sh

set -eu

PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export PATH

TTY_INDEX="${1:?missing ttyCAN index}"
CAN_INDEX=""

i=0
while [ "$i" -le 99 ]; do
    if ! ip link show "can$i" >/dev/null 2>&1; then
        CAN_INDEX="$i"
        break
    fi
    i=$((i + 1))
done

if [ -z "$CAN_INDEX" ]; then
    echo "No available CAN index found" >&2
    exit 1
fi

slcand -o -c -f -s8 -S 3000000 "/dev/ttyCAN$TTY_INDEX" "can$CAN_INDEX"
sleep 1

ip link set up "can$CAN_INDEX"
ip link set "can$CAN_INDEX" txqueuelen 1000
