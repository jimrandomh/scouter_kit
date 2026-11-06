#!/bin/sh
# Starts both arms of a pair in one emulation (machines "right" and "left": the replica MCUboot, APP_IMAGE and
# Even's 1.6.6 net core each), their radios on one wireless medium so the two stock net cores run their ESB link
# to each other, each arm with its own virtual phone in bridge mode (RUN_DIR/vphone-R.sock, vphone-L.sock), in a
# background container named g1pair-<run dir name>. Stop it with: podman kill g1pair-<run dir name>
# The global quantum (G1_QUANTUM, s) must be short enough for an ESB acknowledgement between the machines.
# Usage: [G1_QUANTUM=0.00002] [G1_NET_RAW=net_raw.bin] scripts/pair_start.sh RUN_DIR APP_IMAGE [EXTRA.resc]
set -eu
FW=$(dirname "$(dirname "$(readlink -f "$0")")")
WORK=${G1_WORK:-$FW/work}
WORK=$(cd "$WORK" && pwd -P)
mkdir -p "$1"
RUN=$(cd "$1" && pwd)
APP=$(readlink -f "$2")
EXTRA=${3:+$(readlink -f "$3")}
NET=$(readlink -f "${G1_NET_RAW:-$WORK/fw/1.6.6_net_raw.bin}")
Q=${G1_QUANTUM:-0.00002}
NAME=g1pair-$(basename "$RUN")
rm -f "$RUN/vphone-R.sock" "$RUN/vphone-L.sock"
arm() {
	cat <<RESC
\$g1_name="$1"
i @$FW/emulator/harness/g1-arm.resc
machine LoadPlatformDescriptionFromString "vphone: Wireless.G1_VirtualPhone @ sysbus 0x42002000 { radio: radio }"
\$app_bin=@$APP
\$net_raw=@$NET
i @$FW/emulator/harness/load-mcuboot.resc
connector Connect sysbus.radio wireless
showAnalyzer sysbus.uart0 Antmicro.Renode.Analyzers.LoggingUartAnalyzer
showAnalyzer sysbus.uart_net Antmicro.Renode.Analyzers.LoggingUartAnalyzer
logLevel 1 sysbus.uart0
logLevel 1 sysbus.uart_net
logLevel 1 sysbus.vphone
logLevel 2 sysbus.twim1.npm1300
esbslave Enabled false
sysbus.twim1.npm1300 VbusConnected true
RESC
}
{
	echo "path add @$WORK"
	echo "logFile @$RUN/renode.log"
	echo "i @$FW/emulator/harness/g1-models.resc"
	echo "i @$FW/emulator/models/G1_VirtualPhone.cs"
	echo "emulation CreateBLEMedium \"wireless\""
	echo "emulation SetGlobalQuantum \"$Q\""
	echo "logLevel 3"
	arm right
	echo "vphone BridgePath @$RUN/vphone-R.sock"
	echo "vphone Enabled true"
	arm left
	# the left arm: the P0.26 strap high (stock's role_init Slave, net role 2), set again by every machine reset;
	# no light sensor (as on the arms: stock sets one up only on the ESB master)
	cat <<'RESC'
sysbus.twim1.opt3001 FaultNak true
gpio0.strap_p0_26 SetHigh
macro reset
"""
    echo "G1RESET: machine reset, restarting in MCUboot"
    sysbus LoadBinary $mcuboot_bin 0x0
    runMacro $g1_vectors
    cpunet IsHalted true
    gpio0.strap_p0_26 SetHigh
"""
RESC
	echo "vphone BridgePath @$RUN/vphone-L.sock"
	echo "vphone Enabled true"
	[ -z "$EXTRA" ] || echo "i @$EXTRA"
	echo "emulation RunFor \"5000\""
	echo "quit"
} > "$RUN/run.resc"
FIFO=$(mktemp -u)
mkfifo "$FIFO"
( exec 3<>"$FIFO"; rm -f "$FIFO"; G1_WORK=$WORK BOXNAME=$NAME BOXCWD=$WORK "$FW/scripts/box" "$WORK/renode/renode_1.16.1-dotnet_portable/renode" --disable-xwt --console --plain -e "i @$RUN/run.resc" <&3 > "$RUN/stdout.txt" 2>&1 ) >/dev/null 2>&1 &
echo "$NAME"
