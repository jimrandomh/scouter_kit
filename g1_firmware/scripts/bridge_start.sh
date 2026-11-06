#!/bin/sh
# Starts one emulated arm (the replica MCUboot, APP_IMAGE and Even's 1.6.6 net core) with the virtual phone
# in bridge mode, in a background container named g1emu-<run dir name>. A host program then connects to
# RUN_DIR/vphone.sock (g1host.py --emu). Stop it with: podman kill g1emu-<run dir name>
# RUN_DIR and the images must lie in the work directory or scouter_kit/ (all the container sees).
# Usage: [G1_NET_RAW=net_raw.bin] scripts/bridge_start.sh RUN_DIR APP_IMAGE [EXTRA.resc run before the start]
set -eu
FW=$(dirname "$(dirname "$(readlink -f "$0")")")
WORK=${G1_WORK:-$FW/work}
WORK=$(cd "$WORK" && pwd -P)
mkdir -p "$1"
RUN=$(cd "$1" && pwd)
APP=$(readlink -f "$2")
EXTRA=${3:+$(readlink -f "$3")}
NET=$(readlink -f "${G1_NET_RAW:-$WORK/fw/1.6.6_net_raw.bin}")
NAME=g1emu-$(basename "$RUN")
rm -f "$RUN/vphone.sock"
cat > "$RUN/run.resc" <<RESC
path add @$WORK
logFile @$RUN/renode.log
i @$FW/emulator/harness/g1-board.resc
i @$FW/emulator/harness/vphone.resc
\$app_bin=@$APP
\$net_raw=@$NET
i @$FW/emulator/harness/load-mcuboot.resc
showAnalyzer sysbus.uart0 Antmicro.Renode.Analyzers.LoggingUartAnalyzer
showAnalyzer sysbus.uart_net Antmicro.Renode.Analyzers.LoggingUartAnalyzer
logLevel 3
logLevel 1 sysbus.uart0
logLevel 1 sysbus.uart_net
logLevel 1 sysbus.vphone
logLevel 2 sysbus.twim1.npm1300
esbslave Enabled true
esbslave AnnounceResponse true
sysbus.twim1.npm1300 VbusConnected true
${EXTRA:+i @$EXTRA}
vphone BridgePath @$RUN/vphone.sock
vphone Enabled true
emulation RunFor "5000"
quit
RESC
FIFO=$(mktemp -u)
mkfifo "$FIFO"
( exec 3<>"$FIFO"; rm -f "$FIFO"; G1_WORK=$WORK BOXNAME=$NAME BOXCWD=$WORK "$FW/scripts/box" "$WORK/renode/renode_1.16.1-dotnet_portable/renode" --disable-xwt --console --plain -e "i @$RUN/run.resc" <&3 > "$RUN/stdout.txt" 2>&1 ) >/dev/null 2>&1 &
echo "$NAME"
