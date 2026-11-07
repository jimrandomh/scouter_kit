#!/bin/sh
# Builds g1v0 VERSION in four variants, each in its own g1-ncs container (no network):
#   g1v0-vXYZ         the G1 image to install (zephyr/app_update.bin)
#   g1v0-vXYZ'        the same source as version NEXT (default: patch + 1), the emulator's update test
#   g1v0-dk-vXYZ      the nRF5340 DK variant (no panel, 120 s pairing window after boot)
#   g1v0-vXYZ-emumic  the emulator test variant (panel timeout 2 s, MIC and ANCS on the emulator's unencrypted link, IN_CASE hold 40 s)
# into $G1_BUILD (default $G1_WORK/build), logs in its logs/. Needs scripts/setup.sh build first.
# Usage: scripts/build.sh VERSION [NEXT]
set -eu
FW=$(dirname "$(dirname "$(readlink -f "$0")")")
WORK=${G1_WORK:-$FW/work}
WORK=$(cd "$WORK" && pwd -P)
OUT=${G1_BUILD:-$WORK/build}
V=${1:?usage: scripts/build.sh VERSION [NEXT]}
N=${2:-$(echo "$V" | awk -F. '{print $1 "." $2 "." $3 + 1}')}
v=$(echo "$V" | tr -d .)
n=$(echo "$N" | tr -d .)
SRC=$FW/firmware
DK="-DEXTRA_DTC_OVERLAY_FILE=dk.overlay -Dmcuboot_EXTRA_DTC_OVERLAY_FILE=$FW/g1min/dk_flash.overlay -DPM_STATIC_YML_FILE=$SRC/pm_static_dk.yml -DCONFIG_G1_PAIRING_WINDOW_S=120 -DCONFIG_G1_NO_PANEL=y"
[ -f "$WORK/firmware/1.6.6/net_core_app_update.bin" ] || { echo "run scripts/fetch_stock.sh 1.6.6 first"; exit 1; }
mkdir -p "$OUT/logs"
b() {
	out=$1
	shift
	G1_WORK=$WORK BOXIMAGE=g1-ncs "$FW/scripts/box" sh "$FW/scripts/west_build.sh" "$SRC" "$OUT/$out" -- "$@" > "$OUT/logs/build-$out.log" 2>&1 \
		&& echo "$out ok" || echo "$out FAILED ($OUT/logs/build-$out.log)"
}
b "g1v0-v$v" "-DCONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION=\"$V+0\"" &
b "g1v0-v$n" "-DCONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION=\"$N+0\"" &
b "g1v0-dk-v$v" $DK "-DCONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION=\"$V+0\"" &
b "g1v0-v$v-emumic" -DCONFIG_G1_PANEL_IDLE_S=2 -DCONFIG_G1_MIC_ANY_LINK=y -DCONFIG_G1_ANCS_ANY_LINK=y -DCONFIG_G1_IN_CASE_HOLD_S=40 "-DCONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION=\"$V+0\"" &
wait
