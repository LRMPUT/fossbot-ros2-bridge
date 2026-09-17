#!/bin/bash
# Set the system clock from an HTTP Date header.
#
# This robot has no RTC battery and the lab network blocks NTP (UDP 123 to
# ntp.ubuntu.com times out), so every cold boot comes up months out of date,
# which makes apt reject every repository with "Release file is not valid yet".
#
# Retries, because network-online.target fires before wifi has actually
# associated and got a DHCP lease: the first version of this ran one second
# into boot, failed every fetch, and left the clock wrong.
set -u
ATTEMPTS="${ATTEMPTS:-30}"
DELAY="${DELAY:-6}"

for i in $(seq 1 "$ATTEMPTS"); do
    for URL in http://ports.ubuntu.com http://archive.ubuntu.com http://deb.debian.org; do
        D=$(curl -sI --max-time 8 "$URL" 2>/dev/null | grep -i '^date:' | head -1 | cut -d' ' -f2-)
        if [ -n "$D" ] && date -u -s "$D" >/dev/null 2>&1; then
            echo "clock set from $URL on attempt $i: $(date -u)"
            hwclock -w 2>/dev/null || true
            exit 0
        fi
    done
    sleep "$DELAY"
done
echo "could not set clock from any HTTP source after $ATTEMPTS attempts" >&2
exit 1
