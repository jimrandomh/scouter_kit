"""The emulator harness of the g1v0 tests (v5_tests.py): one emulated arm per test in its own Renode
container (scripts/bridge_start.sh: a fresh, clean flash), driven by the protocol 5 client
(scouter_kit/tools/g1host.py) over the virtual phone's bridge; plus the fault tests (E3). Times are the
emulator's virtual time. The work directory is G1_WORK (default g1_firmware/work), builds G1_BUILD
(default work/build)."""
import json
import os
import subprocess
import sys
import threading
import traceback
from pathlib import Path

FW = Path(__file__).resolve().parents[2]
SK = FW.parent
WORK = Path(os.environ.get("G1_WORK", FW / "work")).resolve()
RUNS = WORK / "runs"
BUILD = Path(os.environ.get("G1_BUILD", WORK / "build")).resolve()
sys.path.insert(0, str(SK / "tools"))
import g1host as g  # noqa: E402

APP = None   # the image the fault tests boot (v5_tests.py sets it)


class Emu:
    """One emulated arm in a background container, plus the host session on its bridge."""

    def __init__(self, run, app, extra_resc=None):
        self.run = RUNS / run
        if self.run.exists():
            subprocess.run(["rm", "-rf", str(self.run)], check=True)
        self.run.mkdir(parents=True)
        cmd = [str(FW / "scripts/bridge_start.sh"), str(self.run), str(app)]
        if extra_resc:
            resc = self.run / "extra.resc"
            resc.write_text(extra_resc + "\n")
            cmd.append(str(resc))
        self.name = subprocess.run(cmd, check=True, capture_output=True, text=True).stdout.strip()
        self.lines = []
        self.arm = g.Arm(g.EmuTransport(str(self.run / "vphone.sock")), name=run, log=self.note)

    def note(self, s):
        self.lines.append(s)

    def stop(self):
        try:
            self.arm.tr.close()
        except Exception:
            pass
        subprocess.run(["podman", "kill", self.name], capture_output=True)

    def now(self):
        return self.arm.tr.now()


class Pair:
    """Both arms in one emulation (scripts/pair_start.sh), each driven by a host thread of its own: a phone steps
    in lock step with its host and a waiting phone stops the whole emulation, so each side keeps pumping its arm
    until both are done. go(right, left) runs right(arm) and left(arm) at once; ev(name) is a threading.Event the
    two sides share (wait with arm.run_until(ev.is_set, ...), which keeps pumping)."""

    def __init__(self, run, app, extra_resc=None, quantum="0.00002"):
        self.run = RUNS / run
        if self.run.exists():
            subprocess.run(["rm", "-rf", str(self.run)], check=True)
        self.run.mkdir(parents=True)
        cmd = [str(FW / "scripts/pair_start.sh"), str(self.run), str(app)]
        if extra_resc:
            resc = self.run / "extra.resc"
            resc.write_text(extra_resc + "\n")
            cmd.append(str(resc))
        env = dict(os.environ, G1_QUANTUM=quantum)
        self.name = subprocess.run(cmd, check=True, capture_output=True, text=True, env=env).stdout.strip()
        self.lines, self.arms, self.errors, self.done, self.events = [], {}, [], set(), {}

    def note(self, s):
        self.lines.append(s)

    def ev(self, name):
        return self.events.setdefault(name, threading.Event())

    def stop(self):
        for arm in self.arms.values():
            try:
                arm.tr.close()
            except Exception:
                pass
        subprocess.run(["podman", "kill", self.name], capture_output=True)

    def _side(self, side, fn):
        arm = None
        try:
            arm = g.Arm(g.EmuTransport(str(self.run / f"vphone-{side}.sock")), name=side, log=self.note)
            self.arms[side] = arm
            arm.setup()
            fn(arm)
        except Exception:
            self.errors.append(f"{side}: {traceback.format_exc()[-1500:]}")
            subprocess.run(["podman", "kill", self.name], capture_output=True)
            return
        finally:
            self.done.add(side)
        try:
            while len(self.done) < 2:
                arm.pump()
        except Exception:
            pass

    def go(self, right, left):
        ts = [threading.Thread(target=self._side, args=(s, f)) for s, f in (("R", right), ("L", left))]
        for t in ts:
            t.start()
        for t in ts:
            t.join()
        if self.errors:
            raise AssertionError("\n".join(self.errors))


def png(arm, idx, out, label):
    return g.emu_check(arm, idx, out, label)


def frame_ctx(arm):
    """A cached font + frame bitmap region in the arena (re-made when the epoch changed)."""
    if getattr(arm, "_fx_epoch", None) != arm.epoch:
        arm._fx_font = g.Fonts(arm).load(24)
        arm._fx_fr = arm.alloc(g.FRAME_H * g.FRAME_STRIDE)
        arm._fx_epoch = arm.epoch
    return arm._fx_font, arm._fx_fr


def wait_new_link(arm, losses_before, timeout=60.0):
    arm.run_until(lambda: arm.losses() > losses_before, timeout, "link loss")
    arm.setup(timeout)


def smp_reachable(arm, t_fault, timeout=60.0):
    """Wait until the arm answers an SMP image list; returns seconds since t_fault."""
    end = arm.tr.now() + timeout
    while arm.tr.now() < end:
        try:
            if not arm.tr.has("SMP"):
                arm.setup(timeout)
            arm.smp.image_list()
            return arm.tr.now() - t_fault
        except (TimeoutError, KeyError):
            continue
    raise TimeoutError("SMP not reachable")


def save_flash(arm, out):
    """Save internal flash (MCUboot, app slot, settings) and the QSPI NOR, for a 'power cycle'."""
    out.mkdir(parents=True, exist_ok=True)
    arm.tr.save_memory(0, 0x100000, str(out / "flash.bin"))
    arm.tr.call_model("sysbus.qspi.qspi_flash", "SaveContents", str(out / "qspi.bin"))
    arm.pump()
    arm.pump()
    # MCUboot is reloaded on top: its partition is write-protected on hardware (load-mcuboot.resc)
    return (f"sysbus LoadBinary @{out}/flash.bin 0x0\nsysbus LoadBinary $mcuboot_bin 0x0\n"
            f"sysbus.qspi.qspi_flash LoadContents @{out}/qspi.bin")


# Left arm: the P0.26 strap high (stock: role_init Slave, net role 2) and no virtual ESB slave. The strap
# is a board pin, so the reset macro (load-mcuboot.resc) sets it again after every machine reset.
LEFT_ARM = """gpio0.strap_p0_26 SetHigh
esbslave Enabled false
macro reset
\"\"\"
    echo "G1RESET: machine reset, restarting in MCUboot"
    sysbus LoadBinary $mcuboot_bin 0x0
    runMacro $g1_vectors
    cpunet IsHalted true
    gpio0.strap_p0_26 SetHigh
\"\"\"
"""


def boot_g1v0(name, extra=None):
    e = Emu(name, APP, extra)
    e.arm.setup()
    return e


def one_frame(arm, out, label, text="frame"):
    """Compose a text frame in the arena, draw it to the panel, check it pixel by pixel."""
    (fa, first), fr = frame_ctx(arm)
    arm.fill(fr, g.FRAME_STRIDE, 0, 0, g.FRAME_W, g.FRAME_H, 0)
    arm.text(fa, first, fr, g.FRAME_STRIDE, 20, 60, text)
    idx = arm.draw(fr, g.FRAME_STRIDE, 0, g.PANEL_Y, g.FRAME_W, g.FRAME_H)
    timing = arm.wait_draw(idx, 120)
    timing["pixel_mismatches"] = png(arm, idx, out, label)
    return timing


def fault_case(name, kind, expect_unhealthy=1, max_smp_s=5.0, reset_timeout=60):
    """Trigger a fault through the FAULT control request; the arm must reboot on its own and answer
    SMP again (criterion: within 5 s of the fault), with the boot counted as unhealthy."""
    e = boot_g1v0(name)
    res = {"test": "E3", "case": name, "fault": kind}
    try:
        arm = e.arm
        h0 = arm.hello()
        res["frame_before"] = one_frame(arm, e.run / "png", "before")
        before = arm.losses()
        t0 = arm.tr.now()
        arm.fault(kind)
        arm.run_until(lambda: arm.losses() > before, reset_timeout, "reset after the fault")
        res["fault_to_link_loss_s"] = round(arm.tr.now() - t0, 3)
        res["fault_to_smp_s"] = round(smp_reachable(arm, t0), 3)
        h = arm.hello()
        res["hello_after"] = {k: h[k] for k in ("boot_count", "unhealthy", "safe_level", "epoch")}
        assert h["epoch"] != h0["epoch"] and h["unhealthy"] == expect_unhealthy, h
        res["frame_after"] = one_frame(arm, e.run / "png", "after")
        res["log_tail"] = arm.read_log()[-1200:]
        res["pass"] = res["fault_to_smp_s"] <= max_smp_s and res["frame_after"]["pixel_mismatches"] == 0
    except Exception as ex:
        res.update(error=f"{type(ex).__name__}: {ex}", trace=traceback.format_exc()[-1200:])
        res["pass"] = False
    return finish(e, res)


def finish(e, res):
    res["virtual_s"] = round(e.now(), 1)
    res["stats"] = e.arm.stats
    res["arm_errors"] = e.arm.errors[-20:]
    res["host_log"] = e.lines[-30:]
    e.stop()
    (e.run / "result.json").write_text(json.dumps(res, indent=1, default=str))
    return res


def test_crash(name):
    return fault_case(name, 1)


WDT0_S = 120.0   # CONFIG_G1_WDT0_CRV (120 s: outlasts a whole MCUboot install, Kconfig)


def test_hang(name):   # interrupts off, PMIC watchdog reset off here: only WDT0 recovers it
    return fault_case(name, 2, max_smp_s=WDT0_S + 3.0, reset_timeout=WDT0_S + 30)


def test_thread_hang(name):
    return fault_case(name, 4, max_smp_s=6.5)   # render thread stuck: health thread reboots (~5 s)


def test_safe_mode(name):
    """Three unhealthy boots in a row -> safe mode: drawing refused (reported), SMP and control
    still work; REBOOT mode 1 clears the counter and returns to normal mode."""
    e = boot_g1v0(name)
    res = {"test": "E3", "case": name}
    try:
        arm = e.arm
        for i in range(3):
            before = arm.losses()
            arm.hello()
            arm.fault(1)
            arm.run_until(lambda: arm.losses() > before, 60, "reset")
            arm.setup()
        h = arm.hello()
        res["hello_safe"] = {k: h[k] for k in ("unhealthy", "safe_level", "safe_mode", "panel_on")}
        assert h["safe_mode"] and h["safe_level"] == 1, h
        assert not h["ancs"], h
        res["smp_in_safe_mode"] = arm.smp.image_list()
        n_err = len(arm.errors)
        fr = arm.alloc(g.FRAME_H * g.FRAME_STRIDE)
        arm.fill(fr, g.FRAME_STRIDE, 0, 0, 16, 16, 15)   # arena op: allowed
        arm.draw(fr, g.FRAME_STRIDE, 0, g.PANEL_Y, 16, 16)  # drawing: refused in safe mode
        arm.wait_idle(30)
        arm.run_until(lambda: len(arm.errors) > n_err, 10, "safe-mode error report")
        res["draw_in_safe_mode"] = arm.errors[-1]
        assert arm.errors[-1]["code"] == 7
        before = arm.losses()
        arm.reboot(1)
        wait_new_link(arm, before)
        h = arm.hello()
        res["hello_after_clear"] = {k: h[k] for k in ("unhealthy", "safe_level", "safe_mode")}
        assert not h["safe_mode"] and h["unhealthy"] == 0, h
        res["frame_after"] = one_frame(arm, e.run / "png", "after")
        res["pass"] = res["frame_after"]["pixel_mismatches"] == 0
    except Exception as ex:
        res.update(error=f"{type(ex).__name__}: {ex}", trace=traceback.format_exc()[-1200:])
        res["pass"] = False
    return finish(e, res)


def renode_log(e):
    f = e.run / "renode.log"
    return f.read_text(errors="replace") if f.exists() else ""


def test_no_pmic(name):
    """PMIC never answers on I2C: no charger writes, panel unavailable, the arm stays up and keeps
    advertising. Without a VBUS reading the arm counts as out of the case (fail closed): an unbonded
    central is asked to encrypt, dropped, and its writes are ignored."""
    e = Emu(name, APP, "sysbus.twim1.npm1300 FaultNak true")
    res = {"test": "E3", "case": name}
    try:
        arm = e.arm
        arm.run_until(lambda: arm.tr.connected, 30, "connect")
        before = arm.losses()
        arm.run_until(lambda: arm.losses() > before, 20, "unbonded drop")
        res["unbonded_dropped_s"] = round(arm.tr.now(), 1)
        try:
            arm.setup()
            arm.hello()
            res["hello_refused"] = False
        except Exception:
            res["hello_refused"] = True
        n0 = arm.losses()
        arm.wait(45)
        res["drops_in_45s"] = arm.losses() - n0      # it keeps advertising, accepting and dropping
        log = renode_log(e)
        res["boots"] = log.count(" booting: safe level")
        res["pmic_absent_logged"] = "PMIC not answering" in log
        res["fatal_errors"] = log.count("fatal error")
        res["charger_writes"] = log.count("charging enabled")
        res["pass"] = (res["hello_refused"] and res["drops_in_45s"] >= 2 and res["boots"] == 1
                       and res["pmic_absent_logged"] and res["fatal_errors"] == 0 and res["charger_writes"] == 0)
    except Exception as ex:
        res.update(error=f"{type(ex).__name__}: {ex}", trace=traceback.format_exc()[-1200:])
        res["pass"] = False
    return finish(e, res)


def test_i2c_errors(name):
    """I2C bus NACKs for 8 s while running, then recovers: the arm keeps running and counts errors."""
    e = boot_g1v0(name)
    res = {"test": "E3", "case": name}
    try:
        arm = e.arm
        h = arm.hello()
        arm.tr.set_model("sysbus.twim1", "FaultNak", "true")
        arm.wait(8)
        arm.tr.set_model("sysbus.twim1", "FaultNak", "false")
        arm.wait(25)
        st = arm.status()
        res["status"] = st
        h2 = arm.hello()
        res["same_boot"] = h2["boot_count"] == h["boot_count"]
        res["frame_after"] = one_frame(arm, e.run / "png", "after")
        res["pass"] = res["same_boot"] and st["i2c_errors"] > 0 and res["frame_after"]["pixel_mismatches"] == 0
    except Exception as ex:
        res.update(error=f"{type(ex).__name__}: {ex}", trace=traceback.format_exc()[-1200:])
        res["pass"] = False
    return finish(e, res)


def test_lost_packet(name):
    """The host 'loses' one packet of a frame (as a phone stack dropping a write): the arm reports
    the hole (selective ack), the host resends, the frame is complete and correct."""
    e = boot_g1v0(name)
    res = {"test": "E3", "case": name}
    try:
        arm = e.arm
        arm.hello()
        (fa, first), fr = frame_ctx(arm)   # uploads the font
        arm.wait_idle(60)
        drop_seq = arm.next_seq & 0xFFFF   # drop the first packet of the coming draw
        real_write = arm.tr.write
        dropped = []

        def lossy(name_, data):
            if name_ == "G1_CMD" and int.from_bytes(data[:2], "little") == drop_seq and not dropped:
                dropped.append(drop_seq)
                return
            real_write(name_, data)
        arm.tr.write = lossy
        arm.fill(fr, g.FRAME_STRIDE, 0, 0, g.FRAME_W, g.FRAME_H, 0)
        arm.text(fa, first, fr, g.FRAME_STRIDE, 20, 60, "after a lost packet")
        idx = arm.draw(fr, g.FRAME_STRIDE, 0, g.PANEL_Y, g.FRAME_W, g.FRAME_H)
        # the first draw packet (drop_seq) is dropped once; the arm's selective ack triggers a resend
        arm.run_until(lambda: bool(dropped), 30, "the packet to be dropped")
        arm.tr.write = real_write
        t = arm.wait_draw(idx, 60)
        t["pixel_mismatches"] = png(arm, idx, e.run / "png", "after")
        res.update(dropped=dropped, frame=t, stats=dict(arm.stats))
        res["pass"] = bool(dropped) and t["pixel_mismatches"] == 0 and arm.stats["fast_resent"] + arm.stats["resent"] > 0
    except Exception as ex:
        res.update(error=f"{type(ex).__name__}: {ex}", trace=traceback.format_exc()[-1200:])
        res["pass"] = False
    return finish(e, res)


def test_disconnect_mid_upload(name):
    """The phone goes silent for 3 s in the middle of a font upload (link supervision timeout).
    After the reconnect the epoch is unchanged and the host resends only what was missing."""
    e = boot_g1v0(name)
    res = {"test": "E3", "case": name}
    try:
        arm = e.arm
        h0 = arm.hello()
        (fa, first), fr = frame_ctx(arm)
        # fill a big arena region to generate many packets, so the drop lands mid-transfer
        big = arm.alloc(40000)
        arm.write_arena(big, bytes(40000))
        arm.flush()
        before = arm.losses()
        arm.run_until(lambda: arm.exec_next > 15, 120, "upload under way")
        res["silenced_at_exec_next"] = arm.exec_next
        arm.tr.go_silent(3000)
        arm.run_until(lambda: arm.losses() > before, 60, "link loss")
        arm.setup()
        h = arm.hello()
        res["same_epoch"] = h["epoch"] == h0["epoch"]
        res["resume_rx_next"] = h["rx_next"]
        arm.wait_idle(120)
        arm.fill(fr, g.FRAME_STRIDE, 0, 0, g.FRAME_W, g.FRAME_H, 0)
        arm.text(fa, first, fr, g.FRAME_STRIDE, 20, 60, "upload resumed")
        idx = arm.draw(fr, g.FRAME_STRIDE, 0, g.PANEL_Y, g.FRAME_W, g.FRAME_H)
        t = arm.wait_draw(idx, 60)
        t["pixel_mismatches"] = png(arm, idx, e.run / "png", "after")
        res["frame"] = t
        res["pass"] = res["same_epoch"] and t["pixel_mismatches"] == 0
    except Exception as ex:
        res.update(error=f"{type(ex).__name__}: {ex}", trace=traceback.format_exc()[-1200:])
        res["pass"] = False
    return finish(e, res)


def test_reset_mid_frame(name):
    """A reset (SoC reset of the emulated machine) in the middle of drawing: the arm comes back
    with a new epoch, SMP answers within 5 s, and the host's re-upload and redraw are correct."""
    e = boot_g1v0(name)
    res = {"test": "E3", "case": name}
    try:
        arm = e.arm
        h0 = arm.hello()
        big = arm.alloc(40000)
        arm.write_arena(big, bytes(40000))
        arm.flush()
        before = arm.losses()
        arm.run_until(lambda: arm.exec_next > 10, 120, "upload under way")
        t0 = arm.tr.now()
        arm.tr.reset_machine()
        arm.run_until(lambda: arm.losses() > before, 60, "link loss")
        res["reset_to_smp_s"] = round(smp_reachable(arm, t0), 3)
        h = arm.hello()
        res["new_epoch"] = h["epoch"] != h0["epoch"]
        res["frame_after"] = one_frame(arm, e.run / "png", "after", "redrawn")
        res["pass"] = res["new_epoch"] and res["reset_to_smp_s"] <= 5.0 and res["frame_after"]["pixel_mismatches"] == 0
    except Exception as ex:
        res.update(error=f"{type(ex).__name__}: {ex}", trace=traceback.format_exc()[-1200:])
        res["pass"] = False
    return finish(e, res)


def test_pmic_watchdog(name):
    """The PMIC thread stops kicking the nPM1300 watchdog (FAULT 5) while everything else runs:
    the PMIC must power-cycle the arm about 30 s after the last kick (emulator: machine reset;
    RAM is kept, unlike hardware)."""
    e = boot_g1v0(name, "sysbus.twim1.npm1300 ResetOnWatchdogExpiry true")
    res = {"test": "E3", "case": name}
    try:
        arm = e.arm
        arm.hello()
        before = arm.losses()
        t0 = arm.tr.now()
        arm.fault(5)
        arm.run_until(lambda: arm.losses() > before, 60, "PMIC power cycle")
        res["stop_to_reset_s"] = round(arm.tr.now() - t0, 2)
        res["smp_after_s"] = round(smp_reachable(arm, arm.tr.now()), 3)
        h = arm.hello()
        res["hello_after"] = {k: h[k] for k in ("boot_count", "unhealthy")}
        res["pass"] = 24 <= res["stop_to_reset_s"] <= 36
    except Exception as ex:
        res.update(error=f"{type(ex).__name__}: {ex}", trace=traceback.format_exc()[-1200:])
        res["pass"] = False
    return finish(e, res)


def test_security(name):
    """Bonding mirrors stock. In the case (VBUS from the NFC charger) the arm
    obeys every write bonded or not; out of the case an unbonded link is dropped and its writes are
    ignored. The virtual central cannot pair, so the bonded-out-of-case path is a documented gap."""
    e = Emu(name, APP, "sysbus.twim1.npm1300 VbusConnected false")  # boot out of the case
    res = {"test": "E3", "case": name}
    try:
        arm = e.arm
        arm.run_until(lambda: arm.tr.connected, 30, "connect")
        before = arm.losses()
        dropped = False
        try:
            arm.run_until(lambda: arm.losses() > before, 15, "unbonded drop")
            dropped = True
        except TimeoutError:
            pass
        res["unbonded_out_of_case_dropped"] = dropped
        try:
            arm.setup()
            arm.hello()
            res["hello_out_of_case"] = "WORKED (BAD)"
            ok_out = False
        except Exception:
            ok_out = True
        res["hello_out_of_case_rejected"] = ok_out
        # now put it in the case: writes must be obeyed without pairing
        arm.tr.set_model("sysbus.twim1.npm1300", "VbusConnected", "true")
        arm.wait(6)   # the PMIC thread polls VBUS every 5 s
        arm.setup()
        h = arm.hello()
        res["in_case_flags"] = {"in_case": bool(h["flags"] & 128), "encrypted": bool(h["flags"] & 64)}
        res["frame_in_case"] = one_frame(arm, e.run / "png", "in_case")
        # leave the case during this unbonded session: the arm asks the link to encrypt, drops it
        before = arm.losses()
        t0 = arm.tr.now()
        arm.tr.set_model("sysbus.twim1.npm1300", "VbusConnected", "false")
        lift(arm)   # 0.5.26+: IN_CASE is a state; a lift ends it 3 s after the last VBUS sample
        arm.run_until(lambda: arm.losses() > before, 30, "drop after leaving the case")
        res["left_case_to_drop_s"] = round(arm.tr.now() - t0, 1)
        try:
            arm.setup()
            arm.hello()
            res["hello_after_leaving_case"] = "WORKED (BAD)"
            ok_left = False
        except Exception:
            ok_left = True
        res["hello_after_leaving_case_rejected"] = ok_left
        res["pass"] = bool(dropped and ok_out and h["flags"] & 128 and not h["flags"] & 64
                           and res["frame_in_case"]["pixel_mismatches"] == 0 and ok_left)
    except Exception as ex:
        res.update(error=f"{type(ex).__name__}: {ex}", trace=traceback.format_exc()[-1200:])
        res["pass"] = False
    return finish(e, res)


SERIAL = "S110LTST000042"


SERIAL_SEED = WORK / "fw/seed/g1_extstore_seed_paired34_serial.bin"


G1_SVC = bytes.fromhex("f3da4d59277a4fc9a1ed41c9c22d4400")[::-1]


def scan_record(arm, timeout=60.0):
    arm.run_until(lambda: "scanrsp" in arm.adv and "adv" in arm.adv, timeout, "advertisement and scan response")
    adv, sd = g.ad_structures(arm.adv["adv"]), g.ad_structures(arm.adv["scanrsp"])
    mfr = sd.get(0xFF, b"")
    return adv, (g.pair_record(mfr[2:]) if mfr[:2] == b"\xff\xff" else None), len(arm.adv["adv"]), len(arm.adv["scanrsp"])


CASE_SETUP = "sysbus.saadc SetAnalogInput 3 575000\nsysbus.twim1.st25dv RfEnableMailbox\n"
LSM = "sysbus.twim2.lsm6dso"


def link_signal(arm, value):
    """The signal strength the emulated radios report (the net core's link value: 60 reads as worn, 30 as folded)."""
    arm.tr.set_model("sysbus.radio", "RssiSample", str(value))


def lift(arm):
    """The arm lifted out of the case, for its accelerometer: a 0.3 g step for half a second (VM.md §2.5, 0.5.26+)."""
    z = float(model_call(arm, LSM, "get_AccelerationZ"))
    arm.tr.set_model(LSM, "AccelerationZ", f"{z + 0.3:.6f}")
    arm.wait(0.5)
    arm.tr.set_model(LSM, "AccelerationZ", f"{z:.6f}")


def model_call(arm, path, method, arg=None, timeout=10.0):
    """Call a model method through the bridge and return its result text (the bridge's link event)."""
    got, old = [], arm.log

    def cap(s):
        key = f"link result {method}"
        if key in s:
            got.append(s.split(key, 1)[1].strip())
        old(s)
    arm.log = cap
    try:
        arm.tr.call_model(path, method, arg)
        arm.run_until(lambda: got, timeout, f"{method} result")
    finally:
        arm.log = old
    return got[0]


def case_request(arm, msg_hex, gpo=True, wait=0.8):
    """The case's side of the ST25DV mailbox: put a message over RF, pulse the GPO (P1.10 low), read the reply."""
    model_call(arm, "sysbus.twim1.st25dv", "RfPutMailboxMessage", msg_hex)
    if gpo:
        model_call(arm, "sysbus.gpio1.sw1", "Press")
        model_call(arm, "sysbus.gpio1.sw1", "Release")
    arm.wait(wait)
    r = model_call(arm, "sysbus.twim1.st25dv", "RfReadMailboxMessage")
    return bytes.fromhex(r.replace("-", "")) if r and "FAILED" not in r else b""


def sum8(b):
    return sum(b) & 0xFF
