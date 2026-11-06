#!/bin/sh
# The emulator gate for g1v0 VERSION as built by scripts/build.sh (the image, its "next update" NEXT and
# the emulator test variant): the standard tests of emulator/tests/v5_tests.py, "all" of them, or the
# tests named. The update chains (chain, chain_right) join the standard tests when the 0.4.10 build and
# the G1_FROM build (the version the arms run now) exist. Results: $G1_WORK/runs/<prefix>-<test>/result.json.
# Usage: [G1_FROM=0.5.12] [G1_RUN_PREFIX=gate] [JOBS=8] scripts/gate.sh VERSION [NEXT] [all | TEST...]
set -eu
FW=$(dirname "$(dirname "$(readlink -f "$0")")")
WORK=${G1_WORK:-$FW/work}
WORK=$(cd "$WORK" && pwd -P)
BUILD=${G1_BUILD:-$WORK/build}
PY=${PYTHON:-python3}
V=${1:?usage: scripts/gate.sh VERSION [NEXT] [all | TEST...]}
shift
N=$(echo "$V" | awk -F. '{print $1 "." $2 "." $3 + 1}')
case "${1:-}" in [0-9]*.[0-9]*) N=$1; shift ;; esac
img() { echo "$BUILD/g1v0-v$(echo "$1" | tr -d .)${2:-}/zephyr/app_update.bin"; }
# The asset store changes no flash chip state (docs/design-flash-assets.md item 30): no bank register, no 4-byte
# mode, nothing set at QSPI activation.
if grep -rniE 'ADDRCONF|WREAR|EN4B|0xB7|0xC5|0xE9' "$FW/firmware/src"; then
	echo "firmware/src names a flash addressing instruction (above): refused"
	exit 1
fi
for f in "$(img "$V")" "$(img "$N")" "$(img "$V" -emumic)"; do
	[ -f "$f" ] || { echo "missing $f: run scripts/build.sh $V $N"; exit 1; }
done
chains() { [ -n "${G1_FROM:-}" ] && [ -f "$(img 0.4.10)" ] && [ -f "$(img "$G1_FROM")" ]; }
NOCHAIN="chain, chain_right skipped: they need G1_FROM and the 0.4.10 and G1_FROM builds"
if [ $# -eq 0 ]; then
	TESTS="smoke power unattended security mic_capture mic_lease draw_bench panel_standby panel_prop panel_read video panel_guard in_case case_hold case_blank mic_popup_standby touch light gyro peer pair_link assets faceclaw"
	if chains; then TESTS="$TESTS chain chain_right"; else echo "$NOCHAIN"; fi
elif [ "$1" = all ]; then
	TESTS=$("$PY" -I -c "import sys; sys.path.insert(0, '$FW/emulator/tests'); import v5_tests; print(' '.join(v5_tests.TESTS))")
	chains || { TESTS=$(echo " $TESTS " | sed 's/ chain_right / /; s/ chain / /'); echo "$NOCHAIN"; }
else
	TESTS=$*
fi
# shellcheck disable=SC2086
G1_WORK=$WORK G1_VERSION=$V G1_NEXT=$N G1_RUN_PREFIX=${G1_RUN_PREFIX:-gate} exec "$PY" -I "$FW/emulator/tests/v5_tests.py" $TESTS --jobs "${JOBS:-8}"
