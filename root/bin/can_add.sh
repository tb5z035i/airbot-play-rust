#!/bin/sh

set -eu

PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export PATH

IFACE="${1:?missing CAN interface name}"
BITRATE="${AIRBOT_CAN_BITRATE:-1000000}"

ip link set "$IFACE" up type can bitrate "$BITRATE"
ip link set "$IFACE" txqueuelen 1000
