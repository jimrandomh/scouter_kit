#!/usr/bin/env python3
"""Emulator tests of g1v0 0.5.x, protocol 5 (../../docs/VM.md), with the reference client
scouter_kit/tools/g1host.py. Each test starts its own emulated arm (common.Emu, scripts/bridge_start.sh),
drives it over the virtual phone bridge and writes work/runs/<prefix>-<test>/result.json. Times are emulated.
Usage: python3 -I emulator/tests/v5_tests.py <test>[:count] [<test> ...] [--jobs N]   (scripts/gate.sh)
G1_RUN_PREFIX=<prefix> (default v5); G1_VERSION / G1_NEXT the version under test and its "next update"
(default 0.5.2 / 0.5.3); G1_APP=<image> for the tests that boot one image (default G1_VERSION);
G1_FROM the protocol 5 image the update chain starts from (after 0.4.10).
"""
import hashlib
import json
import math
import os
import random
import struct
import subprocess
import sys
import time
import traceback
import zlib
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402
from common import BUILD, FW, RUNS, SK, WORK, g  # noqa: E402
import peer_check as pc  # noqa: E402

PREFIX = os.environ.get("G1_RUN_PREFIX", "v5")
VER = os.environ.get("G1_VERSION", "0.5.2")
NEXT = os.environ.get("G1_NEXT", "0.5.3")
FROM = os.environ.get("G1_FROM", "0.5.0")   # the protocol 5 image the update chain starts from (the arms' one)


def image(version, variant=""):
    return BUILD / f"g1v0-v{version.replace('.', '')}{variant}/zephyr/app_update.bin"


V410 = image("0.4.10")
V50 = image("0.5.0")
CUR = image(VER)
CUR_NEXT = image(NEXT)
CUR_MIC = image(VER, "-emumic")
APP = Path(os.environ.get("G1_APP", CUR))
VM_TEST = SK / "tests"
SEED = f"sysbus.qspi.qspi_flash LoadSegment 0x130000 @{common.SERIAL_SEED}\n"
TRIGGERS_M1 = 0xF063   # START, TIMER, BATTERY, CASE, USER0-3
TRIGGERS_M3 = 0xF06F   # and MOTION, TILT (the accelerometer, 0.5.2)
TRIGGERS_M8 = 0xF1FF   # and LIGHT, WEAR, TOUCH (the light sensor and the TouchBar controller, 0.5.18)
TRIGGERS_M10 = 0xF3FF  # and PEER (the arm-to-arm link, 0.5.24)


def triggers_for(version):
    v = vnum(version)
    return TRIGGERS_M10 if v >= (0, 5, 24) else TRIGGERS_M8 if v >= (0, 5, 18) else TRIGGERS_M3
TOUCH = "sysbus.twim2.touch"


Emu = common.Emu
common.APP = APP


def emu(name, app, extra=None):
    return Emu(name, app, extra)


def cycle(e, name, app, more="", left=False):
    """A power cycle: flash and QSPI saved, a fresh emulation booted from them."""
    extra = common.save_flash(e.arm, e.run / "flash")
    e.stop()
    e2 = emu(name, app, (common.LEFT_ARM if left else "") + extra + "\n" + more)
    e2.arm.setup()
    return e2


def finish(e, res):
    res["virtual_s"] = round(e.now(), 1)
    res["stats"] = e.arm.stats
    res["arm_errors"] = e.arm.errors[-20:]
    res["host_log"] = e.lines[-30:]
    e.stop()
    (e.run / "result.json").write_text(json.dumps(res, indent=1, default=lambda o: o.hex() if isinstance(o, bytes) else str(o)))
    return res


def run(name, app, body, extra=None, setup=True):
    """Boot `app`, run body(e, res), record the result."""
    e = emu(name, app, extra)
    res = {"test": name}
    try:
        if setup:
            e.arm.setup()
            res["hello"] = e.arm.hello()
        body(e, res)
        res.setdefault("pass", True)
    except Exception as ex:
        res.update(error=f"{type(ex).__name__}: {ex}", trace=traceback.format_exc()[-1500:], **{"pass": False})
    return finish(e, res)


def vnum(version):
    return tuple(int(x) for x in version.split("."))


def check_hello(h, version, role="R", mic=None):
    """0.5.0: the M1 triggers, HELLO.board bit 2 set (indicator only); from 0.5.2 MOTION and TILT, bit 2 clear
    (review-v5.md §10: the bit means capture); from 0.5.8 bit 2 on the right arm (capture, M5)."""
    old = version in ("0.5.0", "0.5.1")
    if mic is None:
        mic = old or (vnum(version) >= (0, 5, 8) and role == "R")
    assert (h["proto"] == 5 and h["version"] == version and h["role"] == role and h["panel"] == (640, 480)
            and h["arena"] == 262144 and h["queue"] == 32 and h["max_pkt"] == 244
            and h["triggers"] == (TRIGGERS_M1 if old else TRIGGERS_M10 if vnum(version) >= (0, 5, 24) else
                                  TRIGGERS_M8 if vnum(version) >= (0, 5, 18) else TRIGGERS_M3)
            and h["panel_ok"] and h["pmic_ok"] and not h["panel_on"] and h["safe_level"] == 0
            and h["mic"] == mic and not h["no_panel"]), h


def emu_panel(arm, out, label):
    """The emulated panel (JBD model dump) against the mirror's panel: number of differing pixels."""
    out = Path(out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    ppm = out / f"{label}.ppm"
    arm.tr.call_model("sysbus.spim_a.jbd", "DumpFrame", str(ppm))
    arm.pump()
    arm.pump()
    got = g.read_ppm_levels(ppm)
    ppm.unlink()
    want = bytes(arm.mirror.panel)
    bad = sum(1 for a, b in zip(got, want) if a != b)
    if bad:
        g.save_png(got, g.PANEL_W, g.PANEL_H, out / f"{label}_panel.png")
        g.save_png(want, g.PANEL_W, g.PANEL_H, out / f"{label}_mirror.png")
    return bad


# ---------------------------------------------------------------- smoke

def smoke_body(e, res):
    arm = e.arm
    check_hello(res["hello"], VER)
    res["props"] = arm.props("ARM", "LINK", "LINK_INTERVAL", "BATT_MV", "BATT_PCT", "CHARGE", "IN_CASE", "BATT_MA",
                             "BATT_TEMP", "BRIGHTNESS", "PANEL", "CASE_LID")
    p = res["props"]
    assert p["ARM"] == 0 and p["LINK"] == 1 and p["IN_CASE"] == 1 and p["BRIGHTNESS"] == 21 and p["PANEL"] == 0, p
    assert p["CASE_LID"] is None and p["BATT_MV"] and p["BATT_TEMP"] is not None, p
    res["vm_status"] = arm.vm_status()
    assert res["vm_status"]["state"] == 0 and res["vm_status"]["trigger"] == 0xFF, res["vm_status"]
    fr = arm.alloc(320 * 480)
    for k in range(48):
        arm.fill(fr, 320, 0, 10 * k, 640, 10, k % 16)
    idx = arm.draw(fr, 320, 0, 0, 640, 480)
    res["draw"] = arm.wait_draw(idx)
    arm.wait_idle(30)
    res["panel_read"] = arm.panel_read()
    res["arena_crc"] = arm.arena_crc()
    res["emu_panel_mismatches"] = emu_panel(arm, e.run / "png", "smoke")
    assert res["panel_read"]["match"] and res["arena_crc"]["match"] and res["emu_panel_mismatches"] == 0, res
    res["status"] = arm.status()
    if vnum(VER) >= (0, 5, 58):
        # DIAG BONDS (read only): the virtual central cannot pair, so whatever the seed's settings hold, well formed,
        # at most CONFIG_BT_MAX_PAIRED, none of them the connected peer
        res["bonds"] = arm.bonds()
        assert len(res["bonds"]) <= 3 and all(len(b["addr"]) == 17 and not b["connected"] for b in res["bonds"]), res


def test_smoke(name):
    return run(name, APP, smoke_body)


# ---------------------------------------------------------------- the VM core's test vectors through the firmware

def make_vectors(out, n=40, seed=0x5EED):
    """scouter_kit/tests/vm_test built here and run in vectors mode (its README): the cases with the core's
    own results."""
    exe = out / "vm_test"
    subprocess.run(["cc", "-std=c11", "-O1", f"-I{VM_TEST.parent / 'src/vm'}", str(VM_TEST / "vm_test.c"),
                    str(VM_TEST / "vm_v4_ref.c"), str(VM_TEST.parent / "src/vm/vm.c"),
                    str(VM_TEST.parent / "src/vm/vm_gfx.c"), "-o", str(exe)], check=True)
    path = out / "vectors.txt"
    r = subprocess.run([str(exe), "vectors", str(path), str(n), hex(seed)], check=True, capture_output=True, text=True)
    return path, r.stdout.strip().splitlines()[0]


def load_vectors(path):
    cases = []
    for line in open(path):
        t = line.split()
        if not t or t[0].startswith("#"):
            continue
        if t[0] == "case":
            cases.append({"name": t[1], "steps": [], "events": []})
        elif t[0] == "pkt":
            cases[-1]["steps"].append(("pkt", bytes.fromhex(t[1])))
        elif t[0] == "idle":
            cases[-1]["steps"].append(("idle", int(t[1]), bytes.fromhex(t[2])))
        elif t[0] == "start":
            cases[-1]["steps"].append(("start", int(t[1], 16), int(t[2])))
        elif t[0] == "stop":
            cases[-1]["steps"].append(("stop",))
        elif t[0] == "panel":
            cases[-1]["steps"].append(("panel", t[1]))
        elif t[0] == "event":
            cases[-1]["events"].append((int(t[1], 16), bytes.fromhex(t[2])))
        elif t[0] == "arena":
            cases[-1]["arena"] = t[1]
    return cases


def masked_status(raw):
    """VM_STATUS without what depends on time or on the BATTERY/CASE triggers the arm may see: pending bits
    5 and 6, NOTIFY drops, cap pauses, busy_ms (the vectors zero the last three)."""
    b = bytearray(raw[:32])
    b[2] &= ~0x60
    b[26:28] = bytes(2)
    b[29:32] = bytes(3)
    return bytes(b)


def replay(arm, case, timeout=60.0):
    """One vector case through the arm: its packets and control requests in order, VM_STATUS at each idle
    point, then the events, the arena and the panel against the core's results."""
    out = {"case": case["name"], "fails": []}
    arm.reset_session(clear_arena=True)
    n_ev = len(arm.events)
    for st in case["steps"]:
        if st[0] == "pkt":
            arm.raw_packet(st[1])
        elif st[0] == "start":
            got = arm.vm_start(st[1])
            if got != st[2]:
                out["fails"].append(f"VM_START {st[1]:#x}: status {got}, expected {st[2]}")
        elif st[0] == "stop":
            arm.vm_stop()
        elif st[0] == "idle":
            arm.flush()
            arm.run_until(lambda: arm.exec_next == st[1] and not arm.outq and not arm.unacked, timeout,
                          f"{case['name']}: exec_next {st[1]}")
            want, end, got = masked_status(st[2]), arm.tr.now() + timeout, None
            while arm.tr.now() < end:
                got = masked_status(arm.vm_status()["raw"])
                if got == want:
                    break
                arm.wait(0.05)
            if got != want:
                out["fails"].append(f"idle at {st[1]}: VM_STATUS {got.hex()}, expected {want.hex()}")
                break
        elif st[0] == "panel":
            rep = arm.ctrl("PANEL_READ", struct.pack("<HH", 0, 480), timeout=10.0, tries=1)
            got = "off" if rep["status"] == 11 else f"{struct.unpack_from('<I', rep['body'])[0]:08x}" if not rep["status"] else f"status {rep['status']}"
            if got != st[1]:
                out["fails"].append(f"panel {got}, expected {st[1]}")
    arm.wait(0.3)
    got = arm.events[n_ev:]
    i, dropped = 0, 0
    for ev in case["events"]:
        if i < len(got) and got[i] == ev:
            i += 1
        elif ev[0] == 0x04:
            dropped += 1
        else:
            out["fails"].append(f"event {ev[0]:02x} {ev[1].hex()} missing (next got: {got[i][0]:02x} {got[i][1].hex()})"
                                if i < len(got) else f"event {ev[0]:02x} {ev[1].hex()} missing")
            break
    if not out["fails"] and i < len(got):
        out["fails"].append(f"{len(got) - i} extra events, first {got[i][0]:02x} {got[i][1].hex()}")
    st = arm.vm_status()
    if dropped != st["notify_dropped"]:
        out["fails"].append(f"{dropped} NOTIFY not received, the arm dropped {st['notify_dropped']}")
    crc = f"{struct.unpack_from('<I', arm.ctrl_ok('ARENA_CRC', struct.pack('<II', 0, 0), timeout=10.0))[0]:08x}"
    if crc != case["arena"]:
        out["fails"].append(f"arena {crc}, expected {case['arena']}")
    out.update(events=len(case["events"]), notify_dropped=dropped)
    return out


def vectors_body(e, res, n=int(os.environ.get("G1_VECTORS", "40")), seed=int(os.environ.get("G1_VECTORS_SEED", "0x5EED"), 0)):
    arm = e.arm
    path, res["generator"] = make_vectors(e.run, n, seed)
    cases = load_vectors(path)
    res["cases"] = []
    for case in cases:
        r = replay(arm, case)
        res["cases"].append(r)
        if r["fails"]:
            arm.log(f"[{arm.name}] vector {case['name']} FAILED: {r['fails'][:3]}")
    res["failed"] = [c["case"] for c in res["cases"] if c["fails"]]
    res["summary"] = {"cases": len(cases), "packets": sum(1 for c in cases for s in c["steps"] if s[0] == "pkt"),
                      "events": sum(len(c["events"]) for c in cases), "failed": len(res["failed"])}
    assert not res["failed"], res["failed"]


def test_vectors(name):
    return run(name, APP, vectors_body)


# ---------------------------------------------------------------- the update chain

def install(e, img, res, label, app_for_cycle, left):
    """SMP upload + confirm + reset; the new image must come back with the g1v0 service. A power cycle
    follows (emulator artifact: the net core often stalls after an install, emulator.md §10.14)."""
    arm = e.arm
    data = img.read_bytes()
    res[f"{label}_upload"] = g.smp_upload_verified(arm, data)
    before = arm.losses()
    res[f"{label}_commit"] = g.smp_commit(arm, data)
    t0 = arm.tr.now()
    while True:
        common.wait_new_link(arm, before, timeout=240.0)
        if arm.tr.has("G1_CTRL"):
            break
        before = arm.losses()
    res[f"{label}_link_s"] = round(arm.tr.now() - t0, 2)
    return cycle(e, e.run.name + f"-{label}", app_for_cycle, SEED + common.CASE_SETUP, left)


def chain_body(name, left):
    role = "L" if left else "R"
    res = {"test": name, "role": role}
    e = emu(name + "-a", V410, (common.LEFT_ARM if left else "") + SEED + common.CASE_SETUP)
    try:
        arm = e.arm
        arm.setup()
        try:
            arm.hello()
            res["start"] = "protocol 5 (BAD)"
        except g.ProtocolMismatch as ex:
            res["start"] = str(ex)
        assert "protocol 4 (firmware 0.4.10)" in res["start"], res["start"]
        e = install(e, image(FROM), res, "from", V410, left)
        arm = e.arm
        h = arm.hello()
        res["hello_from"] = h
        check_hello(h, FROM, role)
        res["images_from"] = g.smp_images(arm.smp)
        assert res["images_from"][0]["version"] == FROM, res["images_from"]
        res["props"] = arm.props("ARM", "IN_CASE", "BATT_PCT", "PANEL")
        assert res["props"]["ARM"] == (1 if left else 0) and res["props"]["PANEL"] == 0, res["props"]
        if (0, 5, 32) <= vnum(FROM) < (0, 5, 60):   # its case hold (VM.md §6.6 until 0.5.58); the emulated arm lies in the case
            assert arm.ctrl(0x15, b"\x01")["status"] == 0   # CASE_PANEL 1
        fr = g.grid_frame(arm, "LEFT" if left else "RIGHT")
        idx = arm.draw(fr, g.PANEL_W // 2, 0, 0, g.PANEL_W, g.PANEL_H)
        res["grid_draw"] = arm.wait_draw(idx)
        res["grid_mismatches"] = g.emu_check(arm, idx, e.run / "png", "grid")
        res["grid_read"] = arm.panel_read()
        assert res["grid_mismatches"] == 0 and res["grid_read"]["match"], res["grid_read"]
        r = g.draw_test(arm, e.run / "png-drawtest")
        res["drawtest"] = {k: r[k] for k in ("checked", "mismatches", "bad_cases")}
        assert r["mismatches"] == 0 and r["checked"] == len(g.draw_cases()), res["drawtest"]
        res["arena_crc"] = arm.arena_crc()
        assert res["arena_crc"]["match"], res["arena_crc"]
        e = install(e, CUR, res, "cur", V410, left)
        arm = e.arm
        h = arm.hello()
        res["hello_cur"] = h
        check_hello(h, VER, role)
        res["images_cur"] = g.smp_images(arm.smp)
        assert res["images_cur"][0]["version"] == VER, res["images_cur"]
        res["imu_cur"] = arm.imu()
        assert res["imu_cur"]["IMU_RATE"] == 13 and res["imu_cur"]["PITCH"] is not None, res["imu_cur"]
        fr = g.grid_frame(arm, "LEFT" if left else "RIGHT")
        idx = arm.draw(fr, g.PANEL_W // 2, 0, 0, g.PANEL_W, g.PANEL_H)
        res["grid_draw_cur"] = arm.wait_draw(idx)
        res["grid_read_cur"] = arm.panel_read()
        assert res["grid_read_cur"]["match"], res["grid_read_cur"]
        r = g.draw_test(arm, e.run / "png-drawtest-cur")
        res["drawtest_cur"] = {k: r[k] for k in ("checked", "mismatches", "bad_cases")}
        assert r["mismatches"] == 0 and r["checked"] == len(g.draw_cases()), res["drawtest_cur"]
        e = install(e, CUR_NEXT, res, "next", V410, left)
        arm = e.arm
        h = arm.hello()
        res["hello_next"] = {k: h[k] for k in ("proto", "version", "role", "panel_ok", "safe_level", "epoch")}
        check_hello(h, NEXT, role)
        res["images_next"] = g.smp_images(arm.smp)
        res["pass"] = True
    except Exception as ex:
        res.update(error=f"{type(ex).__name__}: {ex}", trace=traceback.format_exc()[-1500:], **{"pass": False})
    return finish(e, res)


def test_chain(name):
    """0.4.10 (left arm) -> the v5 client refuses its HELLO -> SMP 0.5.0 (as on the arms) -> HELLO, grid,
    drawtest, ARENA_CRC -> SMP G1_VERSION -> HELLO, IMU, grid, drawtest -> SMP G1_NEXT."""
    return chain_body(name, True)


def test_chain_right(name):
    return chain_body(name, False)


# ---------------------------------------------------------------- 0.4.10's power behaviour on 0.5.0

NPM = "sysbus.twim1.npm1300"


def ack_repeat(arm, res):
    """The ACK that reports a drained queue is repeated once 100 ms later by the arm's clock (VM.md §6.2)."""
    acks, on_evt = [], arm.listeners["G1_EVT"]

    def spy(d, t):
        if d[0] == 0x01:
            acks.append((t, d[9:18].hex(), int.from_bytes(d[5:9], "little")))
        on_evt(d, t)

    arm.wait_idle(60)
    arm.wait(0.5)
    resent, t0 = arm.stats["resent"], arm.tr.now()
    arm.listeners["G1_EVT"] = spy
    arm.fence(0x42)
    arm.flush()
    arm.wait(0.6)
    arm.listeners["G1_EVT"] = on_evt
    res["acks"] = [(round(t - t0, 4), w, u) for t, w, u in acks]
    last = acks[-1][1]
    assert len(acks) >= 2 and acks[-2][1] == last and 99000 <= acks[-1][2] - acks[-2][2] <= 105000, res["acks"]
    assert last[0:4] == last[4:8] and arm.stats["resent"] == resent and sum(w == last for _, w, _ in acks) == 2, res["acks"]


def vterm(arm):
    return 3800 + 50 * arm.status()["vterm_code"]


def caselink_checks(arm, res):
    """The case's battery requests answered as stock 1.6.6 (power.md §4; bytes from stock in this emulator,
    runs/pw-stock166-*): charging (status 0), a request with no GPO pulse, full (status 1), 06 long trip
    (reply, no ship mode), an unknown command; then the "full" latch of 0.4.10 through a re-check that
    starts the charger, cleared 100 mV below VTERM."""
    out = {}
    arm.tr.set_model(NPM, "BatteryVoltageMilliVolts", "3900")
    arm.tr.set_model(NPM, "BatteryCurrentMilliAmps", "-30")
    arm.wait(2.5)
    vt, mv = vterm(arm), arm.props("BATT_MV")["BATT_MV"]
    level = min(99, (mv - 3500) * 100 // (vt - 3600))
    r = common.case_request(arm, "130001006407", wait=2.0)   # the emulator loses the first GPO pulse after a boot
    out.update(charging=r.hex(), expected_level=level)
    assert len(r) == 11 and r[:5] == bytes([0x13, 1, 2, 6, 0]) and r[10] == common.sum8(r[:10]), out
    assert int.from_bytes(r[5:7], "big") == 5740 and r[7] == level and r[8] == 1 and r[9] in (29, 30), out
    r = common.case_request(arm, "130001006407", gpo=False, wait=2.0)
    out["no_gpo"] = r.hex()
    assert len(r) == 11 and r[4] == 0, out
    arm.tr.set_model(NPM, "BatteryVoltageMilliVolts", "4400")
    arm.tr.set_model(NPM, "BatteryCurrentMilliAmps", "0")
    arm.wait(3)
    r = common.case_request(arm, "130001006408")
    out["full"] = r.hex()
    assert len(r) == 11 and r[4] == 1 and r[7] == 100 and r[8] == 0 and r[9] == 0 and r[10] == common.sum8(r[:10]), out
    p = arm.power()
    out["power_full"] = {k: p[k] for k in ("case_status", "case_level", "case_lid", "case_requests", "vrect_mv", "flags")}
    assert p["case_status"] == 1 and p["case_level"] == 100 and p["case_lid"] == 8 and p["flags"] & 0xA0 == 0xA0, out
    r = common.case_request(arm, "060003")
    out["long_trip"] = r.hex()
    assert r == bytes([6, 1, 3, 1, 0, 11]), out
    r = common.case_request(arm, "55000100")
    out["unknown"] = r.hex()
    assert r == bytes([0x55, 1, 3, 1, 2, common.sum8(bytes([0x55, 1, 3, 1, 2]))]), out
    arm.tr.set_model(NPM, "BatteryVoltageMilliVolts", str(vt))
    arm.wait(3)
    r = common.case_request(arm, "130001006407")
    out["latch_full"] = r.hex()
    assert len(r) == 11 and r[4] == 1 and r[7] == 100, out
    common.model_call(arm, NPM, "ClearCompletedLatch")
    arm.tr.set_model(NPM, "BatteryVoltageMilliVolts", str(vt - 20))
    arm.tr.set_model(NPM, "BatteryCurrentMilliAmps", "-10")
    arm.wait(3)
    out["charger_recheck"] = arm.power()["charge_status"]
    r = common.case_request(arm, "130001006407")
    out["latch_recheck"] = r.hex()
    assert out["charger_recheck"] & 0x1C and len(r) == 11 and r[4] == 1 and r[7] == 100, out
    common.model_call(arm, NPM, "InjectChargerErrorReason", "1")
    arm.tr.set_model(NPM, "BatteryCurrentMilliAmps", "1")
    if VER not in ("0.5.0", "0.5.1", "0.5.2", "0.5.3", "0.5.4", "0.5.5"):
        # 0.5.6+: a dip below VTERM - 100 mV shorter than 30 s (a boot's own load) keeps "full"
        arm.tr.set_model(NPM, "BatteryVoltageMilliVolts", str(vt - 150))
        arm.wait(8)
        arm.tr.set_model(NPM, "BatteryVoltageMilliVolts", str(vt - 20))
        arm.wait(3)
        r = common.case_request(arm, "130001006407")
        out["latch_short_dip"] = r.hex()
        assert len(r) == 11 and r[4] == 1 and r[7] == 100, out
    arm.tr.set_model(NPM, "BatteryVoltageMilliVolts", str(vt - 150))
    arm.wait(3 if VER in ("0.5.0", "0.5.1", "0.5.2", "0.5.3", "0.5.4", "0.5.5") else 33)
    r = common.case_request(arm, "130001006407")
    out["latch_cleared"] = r.hex()
    assert len(r) == 11 and r[4] == 0 and r[7] <= 99, out
    common.model_call(arm, NPM, "ClearChargerError")
    arm.tr.set_model(NPM, "BatteryVoltageMilliVolts", "4100")
    arm.tr.set_model(NPM, "BatteryCurrentMilliAmps", "0")
    res["caselink"] = out


def notify_program(arm, base=0xC000):
    """A program whose CASE handler sends NOTIFY (CASE_LID, CASE_BATT_PCT) and NOTIFY (CASE_CHARGING,
    IN_CASE), and whose BATTERY handler sends NOTIFY (BATT_PCT, CHARGE) (NOTIFY.source = the trigger)."""
    w = {n: arm.k(g.PROPS[n]) for n in ("CASE_LID", "CASE_BATT_PCT", "CASE_CHARGING", "IN_CASE", "BATT_PCT", "CHARGE")}
    c = g.Code()
    c.ins("SYS_READ", 0, w["CASE_LID"]).ins("SYS_READ", 1, w["CASE_BATT_PCT"]).ins("NOTIFY", 0, 1)
    c.ins("SYS_READ", 2, w["CASE_CHARGING"]).ins("SYS_READ", 3, w["IN_CASE"]).ins("NOTIFY", 2, 3).ins("END")
    batt = base + len(c)
    c.ins("SYS_READ", 4, w["BATT_PCT"]).ins("SYS_READ", 5, w["CHARGE"]).ins("NOTIFY", 4, 5).ins("END")
    arm.write(base, c.b)
    table = [0] * 16
    table[5], table[6] = batt, base
    arm.write(0x40, struct.pack("<16I", *table))
    arm.wait_idle(30)
    assert arm.vm_start(0x40) == 0


def triggers_checks(arm, res):
    """CASE_* from the case's messages and the CASE trigger (VM.md §6.8; UNKNOWN 3 s after the last
    message), the BATTERY trigger on a BATT_PCT change."""
    out = {}
    arm.wait(4)   # 3 s after the last case message the CASE_* values are UNKNOWN: the baseline at VM_START
    out["props_start"] = arm.props("CASE_LID", "CASE_CHARGING", "CASE_BATT_PCT")
    assert out["props_start"] == {"CASE_LID": None, "CASE_CHARGING": None, "CASE_BATT_PCT": None}, out
    notify_program(arm)
    U = g.UNKNOWN

    def case_notifies(n0):
        return [(x["a"], x["b"]) for x in arm.notifies[n0:] if x["source"] == 6]

    n0 = len(arm.notifies)
    common.case_request(arm, "130001006407")
    arm.wait(1.0)
    out["props_open"] = arm.props("CASE_LID", "CASE_CHARGING", "CASE_BATT_PCT", "IN_CASE")
    out["notify_open"] = case_notifies(n0)
    assert out["props_open"] == {"CASE_LID": 0, "CASE_CHARGING": 0, "CASE_BATT_PCT": 100, "IN_CASE": 1}, out
    assert out["notify_open"] == [(0, 100), (0, 1)], out
    n0 = len(arm.notifies)
    common.case_request(arm, "130001009e08")
    arm.wait(1.0)
    out["props_closed"] = arm.props("CASE_LID", "CASE_CHARGING", "CASE_BATT_PCT")
    out["notify_closed"] = case_notifies(n0)
    assert out["props_closed"] == {"CASE_LID": 1, "CASE_CHARGING": 1, "CASE_BATT_PCT": 30}, out
    assert out["notify_closed"] == [(1, 30), (1, 1)], out
    n0 = len(arm.notifies)
    arm.wait(4.5)
    out["props_quiet"] = arm.props("CASE_LID", "CASE_CHARGING", "CASE_BATT_PCT")
    out["notify_quiet"] = case_notifies(n0)
    assert out["props_quiet"] == {"CASE_LID": None, "CASE_CHARGING": None, "CASE_BATT_PCT": None}, out
    assert out["notify_quiet"] == [(U, U), (U, 1)], out
    pct0 = arm.props("BATT_PCT")["BATT_PCT"]
    n0 = len(arm.notifies)
    arm.tr.set_model(NPM, "BatteryVoltageMilliVolts", "3800")
    arm.wait(3)
    p = arm.props("BATT_PCT", "CHARGE")
    out["battery"] = {"before": pct0, "after": p, "notifies": [(x["a"], x["b"]) for x in arm.notifies[n0:] if x["source"] == 5]}
    assert p["BATT_PCT"] < pct0 - 1 and out["battery"]["notifies"][-1] == (p["BATT_PCT"], p["CHARGE"]), out
    arm.tr.set_model(NPM, "BatteryVoltageMilliVolts", "4100")
    arm.wait(2)
    assert arm.vm_stop() == 0
    res["triggers"] = out


def power_body(e, res):
    arm, h = e.arm, res["hello"]
    check_hello(h, VER)
    assert h["board"] == 2 | (4 if h["mic"] else 0) and h["in_case"], h
    adv, rec, n_adv, n_sd = common.scan_record(arm)
    res["adv"] = {"name": adv.get(9, b"").decode(), "uuid": adv.get(7) == common.G1_SVC, "pair": rec}
    assert adv.get(9) == b"G1v0_R" and adv.get(7) == common.G1_SVC, res["adv"]
    assert rec == {"role": "R", "no_panel": False, "from_serial": True, "pair_id": common.SERIAL}, rec
    ack_repeat(arm, res)
    p = arm.power()
    res["power_boot"] = {k: p[k] for k in ("batt_mv", "batt_temp_dc", "vbus_mv", "flags", "imu_id", "imu_ctrl1_xl",
                                           "imu_ctrl2_g", "state", "batt_ua")}
    assert not p["flags"] & 0x01 and p["imu_ctrl1_xl"] == 0x10 and p["imu_ctrl2_g"] == 0 and abs(p["batt_mv"] - 4100) <= 6, p
    pr = arm.props("BATT_MA", "BATT_TEMP", "BATT_MV")
    res["power_props"] = pr
    assert pr["BATT_TEMP"] == p["batt_temp_dc"] == 250 and abs(pr["BATT_MA"] * 1000 - arm.power()["batt_ua"]) <= 1500, pr
    res["log_net"] = [ln for ln in arm.read_log().splitlines() if "accepted role" in ln][-1:]
    assert res["log_net"] and "mode 0" in res["log_net"][0], res["log_net"]
    caselink_checks(arm, res)
    triggers_checks(arm, res)


def test_power(name):
    """0.5.0 keeps 0.4.10's power behaviour: panel off at boot, IMU low power, ESB mode 0 (power.md §5, ESB mode), the pair record,
    the ACK repeat, the case link and its "full" latch; plus the CASE and BATTERY triggers."""
    return run(name, APP, power_body, SEED + common.CASE_SETUP + "vphone ScanFirst true\n")


def unattended_body(e, res):
    """VM.md §6.6 with the test build's 2 s (the emulated net core stalls when the arm advertises much longer):
    connected, the panel stays on; disconnected, it goes off 2 s after the last DRAW; with a running program
    it stays on."""
    arm = e.arm
    fr = arm.alloc(320 * 20)
    arm.fill(fr, 320, 0, 0, 640, 20, 9)

    def draw():
        arm.wait_draw(arm.draw(fr, 320, 0, 0, 640, 20))

    def away(ms):
        before = arm.losses()
        arm.tr.go_silent(ms)
        arm.run_until(lambda: arm.losses() > before, 30, "link loss")
        arm.setup(60)
        return arm.hello()

    draw()
    arm.wait(7)
    res["connected_7s"] = arm.props("PANEL")["PANEL"]
    assert res["connected_7s"] == 1, res
    h = away(3500)
    res["away_no_program"] = h["panel_on"]
    assert not h["panel_on"], res
    arm.write(0xC000, g.Code().ins("END").b)
    arm.write(0x40, struct.pack("<16I", 0xC000, *([0] * 15)))
    arm.wait_idle(10)
    assert arm.vm_start(0x40) == 0
    draw()
    h = away(3500)
    res["away_program_running"] = h["panel_on"]
    assert h["panel_on"] and arm.vm_status()["state"] == 1, res
    arm.vm_stop()
    h = away(3500)
    res["away_program_stopped"] = h["panel_on"]
    assert not h["panel_on"], res


def test_unattended(name):
    return run(name, CUR_MIC, unattended_body)


# ---------------------------------------------------------------- the VM.md §9 examples, from its listings

VM_MD = FW / "docs/VM.md"


def listing(title):
    """The code bytes of the VM.md listing whose paragraph starts with `title`, checked against the
    addresses printed in it."""
    lines = VM_MD.read_text().splitlines()
    i = next(k for k, ln in enumerate(lines) if ln.startswith(title))
    i = next(k for k in range(i, len(lines)) if lines[k].startswith("```")) + 1
    code, base = bytearray(), None
    while not lines[i].startswith("```"):
        t = lines[i].split()
        if t and len(t[0]) == 5 and all(ch in "0123456789abcdef" for ch in t[0]):
            addr = int(t[0], 16)
            base = addr if base is None else base
            assert addr == base + len(code), (lines[i], hex(base + len(code)))
            for tok in t[1:]:
                if len(tok) != 2 or any(ch not in "0123456789abcdef" for ch in tok):
                    break
                code.append(int(tok, 16))
        i += 1
    return base, bytes(code)


EXAMPLE_POOL = [0, 1, 2, 3, 4, 5, 8, 10, 12, 15, 17, 20, 24, 32, 34, 40, 60, 80, 100, 160, 240, 256, 560, 1440, 16384,
                153600, 60000000]


def example_setup(arm, code, table):
    """VM.md §9.1: the constant pool, the code at 0xC000, the font at 0x25800 (glyph 62 space, 63 %, 64 :),
    the trigger table at 0x40, with the bytes printed there."""
    arm.reset_session(clear_arena=True)
    blob = g.build_font_blob(g.FONT_PATH, 20, g.FIRST_GLYPHS + " %:", kerning=False)
    f = g.Font(blob)
    assert f.indices(" %:") == [62, 63, 64], f.indices(" %:")
    arm.raw_packet(bytes.fromhex("018000006c") + struct.pack("<27I", *EXAMPLE_POOL))
    arm.raw_packet(bytes.fromhex("0100c000") + bytes([len(code)]) + code)
    arm.write(0x25800, blob)
    arm.flush()
    arm.raw_packet(bytes.fromhex("0140000040") + struct.pack("<16I", *table))
    arm.wait_idle(60)


def mirror_call(arm, addr):
    """What a handler does, on the mirror only: a packet that CALLs it (END there ends the packet)."""
    arm.mirror.packet(struct.pack("<H", 0xFFFF) + bytes(g.Code().ins("CALL", target=addr).b))


def panel_equal(arm, label, res, out=None):
    r = arm.panel_read()
    res[label] = {k: r.get(k) for k in ("crc", "mirror_crc", "match")}
    if not r["match"] and out is not None:
        res[label]["emu_mismatches"] = emu_panel(arm, out, label)
    assert r["match"], res[label]


def wait_tasks(arm, n, timeout):
    """Poll VM_STATUS until `n` tasks have started; returns the device time of the first reply that shows it."""
    end = arm.tr.now() + timeout
    while arm.tr.now() < end:
        rep = arm.ctrl("VM_STATUS")
        if struct.unpack_from("<I", rep["body"], 20)[0] >= n:
            return rep["t_dev"]
        arm.wait(0.02)
    raise TimeoutError(f"{n} tasks")


def examples_body(e, res):
    arm = e.arm
    arm.sync_clock()
    base, code = listing("Code (176 bytes at 0xC000")
    assert base == 0xC000 and len(code) == 176, len(code)
    table = [0] * 16
    table[0], table[1], table[12] = 0xC000, 0xC010, 0xC000
    example_setup(arm, code, table)
    arm.raw_packet(bytes.fromhex("01000100050000400000"))
    now = arm.device_now()
    n1 = (now + 2000000) & g.M32
    arm.raw_packet(bytes.fromhex("0108010008") + struct.pack("<II", 12 * 60 + 34, n1))
    arm.wait_idle(10)
    assert arm.vm_start(0x40) == 0
    wait_tasks(arm, 1, 5)
    arm.wait(0.3)
    mirror_call(arm, 0xC000)
    panel_equal(arm, "clock_1234", res, e.run / "png")
    st = arm.vm_status()
    assert st["timer"] == n1 and st["flags"] & 1 and st["state"] == 1, st
    t = wait_tasks(arm, 2, 5)
    res["tick_late_ms"] = round(g.signed32(t - n1) / 1000.0, 1)
    assert 0 <= res["tick_late_ms"] < 400, res
    arm.wait(0.3)
    mirror_call(arm, 0xC010)
    panel_equal(arm, "clock_1235", res)
    assert arm.vm_status()["timer"] == (n1 + 60000000) & g.M32
    n1 = (arm.device_now() + 1500000) & g.M32
    arm.raw_packet(bytes.fromhex("0108010008") + struct.pack("<II", 1439, n1) + bytes.fromhex("42f02800"))
    wait_tasks(arm, 3, 5)
    arm.wait(0.3)
    mirror_call(arm, 0xC000)
    panel_equal(arm, "clock_2359", res)
    wait_tasks(arm, 4, 5)
    arm.wait(0.3)
    mirror_call(arm, 0xC010)
    panel_equal(arm, "clock_0000", res)
    assert struct.unpack_from("<I", arm.mirror.arena, 0x110)[0] == 0 and not arm.halts, arm.halts

    base, code = listing("Code (113 bytes at 0xC000")
    assert base == 0xC000 and len(code) == 113, len(code)
    table = [0] * 16
    table[0], table[5] = 0xC000, 0xC000
    example_setup(arm, code, table)
    arm.raw_packet(bytes.fromhex("01000100040000003f"))
    arm.wait_idle(10)
    res["battery"] = []
    for mv in (4100, 3800, 3550):
        arm.tr.set_model(NPM, "BatteryVoltageMilliVolts", str(mv))
        if mv == 4100:
            arm.wait(2)
            assert arm.vm_start(0x40) == 0
        arm.wait(3)
        pct = arm.props("BATT_PCT")["BATT_PCT"]
        arm.mirror.set_prop(g.PROPS["BATT_PCT"], pct)
        mirror_call(arm, 0xC000)
        label = f"battery_{pct}"
        panel_equal(arm, label, res)
        res["battery"].append({"mv": mv, "pct": pct, "tasks": arm.vm_status()["tasks"]})
    assert len({b["pct"] for b in res["battery"]}) == 3 and not arm.halts, res["battery"]
    arm.tr.set_model(NPM, "BatteryVoltageMilliVolts", "4100")
    g.save_png(bytes(arm.mirror.panel), g.PANEL_W, g.PANEL_H, e.run / "examples_last.png")


def test_examples(name):
    """VM.md §9.2 (clock: start, minute tick at N', correction with RAISE C12, midnight) and §9.3 (battery:
    start, BATTERY on a voltage change) from the bytes printed in VM.md; the panel after each step equals
    the mirror running the same handler."""
    return run(name, APP, examples_body)


# ---------------------------------------------------------------- VM behaviour on the arm

def program(arm, handlers, base=0xC000):
    """handlers: {trigger: Code}; written at base..., the table at 0x40, then VM_START. Returns the addresses."""
    table, addr, blob = [0] * 16, base, bytearray()
    for trig, c in handlers.items():
        table[trig] = addr
        blob += c.b
        addr += len(c)
    arm.write(base, blob)
    arm.write(0x40, struct.pack("<16I", *table))
    arm.wait_idle(30)
    assert arm.vm_start(0x40) == 0
    return table


def packet_rtt(arm, n=5):
    """Send-to-NOTIFY time of one-packet host code (NOTIFY a = token), median of n, ms."""
    out = []
    for k in range(n):
        token = 0x7000 + len(arm.fences)
        arm.fences.pop(token, None)
        t0 = arm.tr.now()
        arm.fence(token)
        arm.flush()
        arm.run_until(lambda: token in arm.fences, 10, "fence")
        out.append((arm.fences[token]["t"] - t0) * 1000.0)
    return sorted(out)[len(out) // 2]


def vm_body(e, res):
    arm = e.arm
    out = res.setdefault("vm", {})
    k = arm.k
    # privacy and the M1 surface: MIC needs an encrypted link (the emulator's central cannot encrypt)
    out["mic_unencrypted"] = arm.mic(True)["status"]
    assert out["mic_unencrypted"] == 12 and arm.mic(False)["status"] == 0, out
    m8 = vnum(VER) >= (0, 5, 18)
    out["events_mask"] = arm.events_mask(0xFFFF)
    assert out["events_mask"] == triggers_for(VER), out
    assert arm.events_mask(0) == 0
    assert arm.ctrl("PROPS", bytes([0x3D]))["status"] == 5 and arm.ctrl("VM_START", struct.pack("<I", 0x3FFF0))["status"] == 3
    out["later_props"] = arm.props(0x30, 0x33, 0x34, 0x38, 0x39, 0x3B)
    # 0.5.18: GYR_X UNKNOWN with the gyroscope off, GYRO_RATE 0, no gesture yet, AUTO_BRIGHTNESS 0; 0.5.24: PEER_LINK 0
    # (off), PEER_MSG UNKNOWN (nothing received)
    want = [None, 0, 0, 0, 0 if vnum(VER) >= (0, 5, 24) else None, None] if m8 else [None] * 6
    assert list(out["later_props"].values()) == want, out
    # coalescing: three RAISEs of USER0 in one packet run its handler once and count 2 coalesced
    cnt, one = 300, k(1)
    h0 = g.Code().ins("ADD", cnt, cnt, one).ins("END")
    hw = g.Code().ins("WAIT_EVENT", 7, k(1 << 14)).ins("NOTIFY", 7, cnt).ins("END")
    loop = g.Code().ins("JMP", target=0xC000)
    halt = g.Code().ins("HALT")
    run_away = g.Code().ins("RAISE", k(15)).ins("END")
    program(arm, {12: h0, 13: hw, 15: run_away})
    arm.add(g.Code().ins("RAISE", k(12)).ins("RAISE", k(12)).ins("RAISE", k(12)))
    arm.wait_idle(10)
    arm.wait(0.3)
    st = arm.vm_status()
    out["coalesce"] = {"counter": arm.ctrl_ok("ARENA_CRC", struct.pack("<II", 4 * cnt, 4))[4:8].hex(),
                       "coalesced": st["coalesced"], "tasks": st["tasks"]}
    assert out["coalesce"]["counter"] == "01000000" and st["coalesced"] == 2 and st["tasks"] == 1, out
    # WAIT_EVENT: the USER1 task waits for USER2, then NOTIFYs
    arm.ins("RAISE", 13)
    arm.wait_idle(10)
    arm.wait(0.3)
    st = arm.vm_status()
    out["wait_event"] = {"task": st["task"], "trigger": st["trigger"]}
    assert st["task"] == 3 and st["trigger"] == 13, out
    n0 = len(arm.notifies)
    arm.ins("RAISE", 14)
    arm.wait_idle(10)
    arm.run_until(lambda: len(arm.notifies) > n0, 5, "WAIT_EVENT notify")
    out["wait_event"]["notify"] = {k_: arm.notifies[-1][k_] for k_ in ("source", "a", "b")}
    assert arm.notifies[-1]["source"] == 13 and arm.notifies[-1]["a"] == 14, out
    # the run-time cap: a handler that RAISEs itself runs 30 ms of every 100 ms; host packets come first
    rtt_idle = packet_rtt(arm)
    arm.ins("RAISE", 15)
    arm.wait_idle(10)
    arm.wait(2)
    st1 = arm.vm_status()
    rtt_busy = packet_rtt(arm)
    arm.wait(3)
    st2 = arm.vm_status()
    out["cap"] = {"busy_ms": [st1["busy_ms"], st2["busy_ms"]], "pauses": [st1["cap_pauses"], st2["cap_pauses"]],
                  "tasks_per_s": round((st2["tasks"] - st1["tasks"]) / 3.0), "rtt_idle_ms": round(rtt_idle, 1),
                  "rtt_busy_ms": round(rtt_busy, 1), "state": st2["state"]}
    assert st2["state"] == 1 and st2["cap_pauses"] >= st1["cap_pauses"] + 25 and 20 <= st2["busy_ms"] <= 40, out
    assert rtt_busy < rtt_idle + 40, out
    # RESET_SESSION stops it: new epoch, window and NOTIFY count from the start
    epoch = arm.epoch
    arm.reset_session()
    st = arm.vm_status()
    arm.fence(0x5151)
    arm.flush()
    arm.run_until(lambda: 0x5151 in arm.fences, 10, "fence after reset")
    out["reset"] = {"state": st["state"], "new_epoch": arm.epoch != epoch, "notify_count": arm.fences[0x5151]["count"]}
    assert st["state"] == 0 and arm.epoch != epoch and arm.fences[0x5151]["count"] == 1, out
    # budget and HALT: an endless JMP halts with error 10 at jump 1001; HALT halts with code 0
    n0 = len(arm.halts)
    loop_addr = program(arm, {0: loop})[0]
    arm.run_until(lambda: len(arm.halts) > n0, 10, "budget halt")
    out["budget_halt"] = {k_: arm.halts[-1][k_] for k_ in ("trigger", "address", "op", "code")}
    assert arm.halts[-1]["code"] == 10 and arm.halts[-1]["address"] == loop_addr and arm.halts[-1]["trigger"] == 0, out
    assert arm.vm_status()["state"] == 2
    n0 = len(arm.halts)
    program(arm, {0: halt})
    arm.run_until(lambda: len(arm.halts) > n0, 10, "HALT")
    out["halt"] = {k_: arm.halts[-1][k_] for k_ in ("trigger", "op", "code")}
    assert arm.halts[-1]["code"] == 0 and arm.halts[-1]["op"] == 0x34, out
    # host CALL into stored code (with a branch and RET) and SLEEP_UNTIL in host code
    arm.reset_session(clear_arena=True)
    sub = g.Code().ins("ADD", 400, 400, k(5)).ins("BLTU", 400, k(20), target=0xD000).ins("RET")
    arm.write(0xD000, sub.b)
    for _ in range(3):
        arm.add(g.Code().ins("CALL", target=0xD000))
    arm.wait_idle(10)
    out["host_call"] = arm.arena_crc(4 * 400, 4)
    assert out["host_call"]["match"] and arm.mirror.word(400) == 30, (out, arm.mirror.word(400))
    arm.sync_clock()
    late = []
    for _ in range(5):
        t = (arm.device_now() + 300000) & g.M32
        arm.add(g.Code().write(4 * 410, struct.pack("<I", t)).ins("SLEEP_UNTIL", 410).ins("NOTIFY", 410, k(0)))
        arm.flush()
        n0 = len(arm.notifies)
        arm.run_until(lambda: len(arm.notifies) > n0, 5, "sleep notify")
        late.append(round(g.signed32(arm.notifies[-1]["t_dev"] - t) / 1000.0, 3))
    out["sleep_until_late_ms"] = late
    assert all(0 <= x < 1.0 for x in late), out
    # a TIMER handler: the program arms TIMER = T in START, the TIMER handler NOTIFYs
    t = (arm.device_now() + 500000) & g.M32
    arm.write(4 * 420, struct.pack("<I", t))
    n0 = len(arm.notifies)
    program(arm, {0: g.Code().ins("SYS_WRITE", k(0x22), 420).ins("END"),
                  1: g.Code().ins("SYS_READ", 8, k(0)).ins("NOTIFY", 8, 420).ins("END")})
    arm.run_until(lambda: len(arm.notifies) > n0, 5, "TIMER notify")
    nt = arm.notifies[-1]
    out["timer_late_ms"] = round(g.signed32(nt["a"] - t) / 1000.0, 3)
    assert nt["source"] == 1 and 0 <= out["timer_late_ms"] < 2.0, out
    arm.vm_stop()


def test_vm(name):
    """MIC refused on the unencrypted link, EVENTS mask 0 in M1, coalescing, WAIT_EVENT, the run-time cap with
    host packets first, RESET_SESSION, the 1000-jump budget, HALT, host CALL, SLEEP_UNTIL and TIMER timing."""
    return run(name, APP, vm_body)


def mic_body(e, res):
    """The microphone indicator (VM.md §6.5; capture is M5), on the emulator test build that accepts MIC on
    the virtual central's unencrypted link: 16 x 16 at level 15 at (620, 4) on the right arm, redrawn over a
    DRAW, cleared at MIC 0, kept by PANEL = 0 (the panel blanked to it, the off applied at MIC 0) and stopped at
    a disconnection."""
    arm = e.arm
    out = res.setdefault("mic", {})

    def block(level, base):
        p = bytearray(base)
        for y in range(4, 20):
            p[y * 640 + 620:y * 640 + 636] = bytes([level]) * 16
        return bytes(p)

    def dump(label):
        ppm = e.run / f"{label}.ppm"
        arm.tr.call_model("sysbus.spim_a.jbd", "DumpFrame", str(ppm))
        arm.pump()
        arm.pump()
        got = g.read_ppm_levels(ppm)
        ppm.unlink()
        return got

    r = arm.mic(True)
    out["on"] = r
    assert r == {"status": 0, "on": True, "frame_bytes": 40, "frames_per_event": 5, "sample_rate": 16000, "frame_ms": 10,
                 "lease_ms": 2000}, r
    arm.wait(0.5)
    assert dump("on") == block(15, bytes(640 * 480)) and arm.props("PANEL")["PANEL"] == 1
    fr = arm.alloc(320 * 480)
    arm.fill(fr, 320, 0, 0, 640, 480, 3)
    arm.mic(True)
    arm.wait_draw(arm.draw(fr, 320, 0, 0, 640, 480))
    out["redrawn"] = dump("redraw") == block(15, bytes([3]) * (640 * 480))
    arm.mic(True)
    arm.wait_draw(arm.draw(fr, 320, 600, 0, 40, 30))
    out["redrawn_partial"] = dump("redraw2") == block(15, bytes([3]) * (640 * 480))
    assert arm.mic(False)["status"] == 0
    arm.wait(0.3)
    out["cleared"] = dump("off") == block(0, bytes([3]) * (640 * 480))
    assert out["redrawn"] and out["redrawn_partial"] and out["cleared"], out
    # PANEL = 0 while on (VM.md §6.5): the panel blanks to the indicator, PANEL reads 0, the microphone goes
    # on; at MIC 0 the pending off is applied
    assert arm.mic(True)["status"] == 0
    arm.wait(0.3)
    n0 = len(arm.notices)
    arm.set_panel(0)
    arm.wait_idle(10)
    arm.wait(0.3)
    out["panel0"] = {"blank_indicator": dump("panel0") == block(15, bytes(640 * 480)),
                     "panel_prop": arm.props("PANEL")["PANEL"], "mic_on": arm.mic_stats()["on"],
                     "notices": [x for x in arm.notices[n0:] if x["kind"] == 5]}
    assert out["panel0"] == {"blank_indicator": True, "panel_prop": 0, "mic_on": True, "notices": []}, out
    assert arm.mic(False)["status"] == 0
    arm.wait(0.3)
    out["after_mic_off"] = {"panel_read": arm.panel_read(0, 1)["status"], "panel_prop": arm.props("PANEL")["PANEL"]}
    assert out["after_mic_off"] == {"panel_read": 11, "panel_prop": 0}, out
    arm.set_panel(1)
    arm.wait_idle(10)
    arm.wait(0.3)
    out["after_panel_on"] = dump("panel") == bytes(640 * 480)
    assert out["after_panel_on"], out
    assert arm.mic(True)["status"] == 0
    arm.wait(0.3)
    before = arm.losses()
    arm.tr.go_silent(3000)
    arm.run_until(lambda: arm.losses() > before, 30, "link loss")
    arm.setup(60)
    arm.hello()
    arm.wait(0.3)
    out["after_disconnect"] = dump("disc") == bytes(640 * 480)
    assert out["after_disconnect"], out


def test_mic(name):
    return run(name, CUR_MIC, mic_body, extra=SINE)


SINE = "sysbus.pdm0 ConfigureSine 8000 1000"   # the PDM model's samples: amplitude 8000, 1 kHz


def tone_fit(x, hz=1000.0, rate=16000.0):
    """least-squares a sin + b cos at hz: (amplitude, correlation of x with the fit, rms of x)"""
    w = 2 * math.pi * hz / rate
    ss = sc = cc = xs = xc = 0.0
    for i, v in enumerate(x):
        si, ci = math.sin(w * i), math.cos(w * i)
        ss, sc, cc, xs, xc = ss + si * si, sc + si * ci, cc + ci * ci, xs + v * si, xc + v * ci
    det = ss * cc - sc * sc
    a, b = (xs * cc - xc * sc) / det, (xc * ss - xs * sc) / det
    fit = [a * math.sin(w * i) + b * math.cos(w * i) for i in range(len(x))]
    num = sum(u * v for u, v in zip(x, fit))
    den = math.sqrt(sum(u * u for u in x) * sum(v * v for v in fit)) or 1.0
    return math.hypot(a, b), num / den, math.sqrt(sum(u * u for u in x) / len(x))


def mic_collect(arm, seconds, rate=0):
    """MIC 1, then MIC_DATA for `seconds`, MIC 0; the events (first, n, frames, t_dev) and both replies"""
    got = []
    arm.on_mic = lambda first, n, frames, t_dev, t: got.append((first, n, bytes(frames), t_dev))
    on = arm.mic(True, rate)
    arm.mic_hold(seconds, rate)
    off = arm.ctrl("MIC", bytes([0, 0]))
    arm.wait(0.5)
    arm.on_mic = None
    return got, on, off, off["t_dev"]


def mic_analyse(events, frame_bytes, per_event, skip=10):
    """counter continuity, frame sizes, decode status, and the tone fit of the decoded audio"""
    dec = g.Lc3Decoder()
    pcm, status, gaps, expect = [], [], 0, None
    for first, n, frames, t_dev in events:
        assert n == per_event and len(frames) == n * frame_bytes, (first, n, len(frames))
        if expect is not None and first != expect:
            gaps += (first - expect) & 0xFFFF
        expect = (first + n) & 0xFFFF
        for k in range(n):
            x, st = dec.decode(frames[k * frame_bytes:(k + 1) * frame_bytes])
            pcm += x
            status.append(st)
    amp, corr, rms = tone_fit(pcm[skip * 160:])
    return dict(events=len(events), frames=len(status), bad_frames=sum(1 for v in status if v != 0), gaps=gaps,
                amplitude=round(amp), correlation=round(corr, 5), rms=round(rms),
                first_counter=events[0][0] if events else None)


def mic_capture_body(e, res):
    """M5 on the emulator test build (MIC on the virtual central's unencrypted link) with the PDM model's
    1 kHz tone at amplitude 8000: 32 kbps (5 x 40 B per event, ~20 events/s) and 16 kbps (10 x 20 B), each
    decoded on the host by the same liblc3: every frame valid, no counter gap, the tone back (amplitude
    within 1.5 dB, correlation with a fitted 1 kHz sine >= 0.99); no MIC_DATA made after the MIC 0 reply;
    the indicator while on; DIAG MIC counters; PANEL = 0 and a disconnection stop the capture."""
    arm = e.arm
    out = res.setdefault("capture", {})
    check_hello(res["hello"], VER, mic=True)
    for rate, fb, fpe, seconds in ((0, 40, 5, 3.0), (1, 20, 5, 1.5)):
        ev, on, off, off_t = mic_collect(arm, seconds, rate)
        assert on == {"status": 0, "on": True, "frame_bytes": fb, "frames_per_event": fpe, "sample_rate": 16000,
                      "frame_ms": 10, "lease_ms": 2000}, on
        assert off["status"] == 0, off
        a = mic_analyse(ev, fb, fpe)
        late = [t for *_, t in ev if g.signed32(t - off_t) > 20000]   # made > 20 ms after the MIC 0 reply
        a.update(events_per_s=round(a["events"] / seconds, 1), made_after_off=len(late), stats=arm.mic_stats())
        out[f"rate{rate}"] = a
        assert a["first_counter"] == 0 and a["gaps"] == 0 and a["bad_frames"] == 0, a
        assert 0.85 * seconds * 100 / fpe <= a["events"] <= 1.1 * seconds * 100 / fpe, a
        assert abs(20 * math.log10(a["amplitude"] / 8000)) <= 1.5 and a["correlation"] >= 0.99, a
        assert not late and not a["stats"]["on"] and a["stats"]["lost"] == 0 and a["stats"]["faults"] == 0, a
    # MIC 1 with the other format while on: the capture restarts in that format, its counter from 0
    sw = []
    arm.on_mic = lambda first, n, frames, t_dev, t: sw.append((first, n, len(frames), t_dev))
    assert arm.mic(True, 0)["status"] == 0
    arm.wait(0.5)
    k = len(sw)
    r2 = arm.ctrl("MIC", bytes([1, 1]))
    arm.wait(0.6)
    new = [x for x in sw[k:] if x[2] == 100]
    late_old = [x for x in sw[k:] if x[2] != 100 and g.signed32(x[3] - r2["t_dev"]) > 20000]
    out["format_switch"] = {"status": r2["status"], "body": r2["body"].hex(), "old_after": len(sw[k:]) - len(new),
                            "old_made_after": len(late_old), "new": [x[:3] for x in new[:3]]}
    assert r2["status"] == 0 and r2["body"][1:3] == bytes([20, 5]) and new and new[0][0] == 0, out["format_switch"]
    assert all(x[1] == 5 for x in new) and not late_old, out["format_switch"]
    assert arm.mic(False)["status"] == 0
    arm.wait(0.3)
    # the indicator while on, then PANEL = 0 stops the capture (NOTICE 5, reason 1)
    got = []
    arm.on_mic = lambda first, n, frames, t_dev, t: got.append(t_dev)
    assert arm.mic(True)["status"] == 0
    arm.wait(0.5)
    ppm = e.run / "ind.ppm"
    arm.tr.call_model("sysbus.spim_a.jbd", "DumpFrame", str(ppm))
    arm.pump()
    arm.pump()
    px = g.read_ppm_levels(ppm)
    ppm.unlink()
    out["indicator"] = all(px[y * 640 + x] == 15 for y in range(4, 20) for x in range(620, 636))
    assert out["indicator"], out
    # PANEL = 0 meanwhile does not stop the capture (VM.md §6.5): events go on, the panel stays on
    n0 = len(arm.notices)
    k = len(got)
    arm.set_panel(0)
    arm.wait_idle(10)
    arm.wait(0.6)
    out["panel0"] = {"events": len(got) - k, "notices": [x["body"] for x in arm.notices[n0:] if x["kind"] == 5],
                     "stats_on": arm.mic_stats()["on"], "panel_prop": arm.props("PANEL")["PANEL"]}
    assert out["panel0"]["events"] >= 10 and not out["panel0"]["notices"] and out["panel0"]["stats_on"], out
    assert out["panel0"]["panel_prop"] == 0, out
    assert arm.mic(False)["status"] == 0
    arm.wait(0.3)
    assert arm.panel_read(0, 1)["status"] == 11, out
    # a disconnection stops it: no MIC_DATA after the reconnect, DIAG MIC off
    arm.set_panel(1)
    arm.wait_idle(10)
    assert arm.mic(True)["status"] == 0
    arm.wait(0.5)
    before = arm.losses()
    arm.tr.go_silent(3000)
    arm.run_until(lambda: arm.losses() > before, 30, "link loss")
    arm.setup(60)
    arm.hello()
    got.clear()
    arm.wait(1.0)
    out["after_disconnect"] = {"events": len(got), "stats": arm.mic_stats()}
    arm.on_mic = None
    assert not got and not out["after_disconnect"]["stats"]["on"], out
    # DIAG ENCODE_LOAD (test builds): the encoder alone on a synthetic tone, 100 frames/s, at 64 and 128 MHz;
    # no MIC_DATA, and MIC refused meanwhile
    for fast in (0, 1):
        assert arm.ctrl("DIAG", struct.pack("<BHB", 10, 2, fast))["status"] == 0
        n0 = arm.mic_events
        refused = arm.mic(True)["status"]
        arm.wait(2.5)
        st = arm.mic_stats()
        out[f"encode_load_{64 << fast}MHz"] = {"encoded": st["encoded"], "avg_us": st["encode_avg_us"],
                                               "mic_refused": refused, "mic_data": arm.mic_events - n0}
        assert 180 <= st["encoded"] <= 205 and refused == 12 and arm.mic_events == n0, out


def test_mic_capture(name):
    return run(name, CUR_MIC, mic_capture_body, extra=SINE)


def mic_popup_body(e, res, hide_value=0):
    """the reviewer's case (VM.md §6.5): a resident program in the style of the g1 notification mirror pops up a
    band (USER0: FILL + DRAW, TIMER 1.5 s later) and hides it with PANEL = 0 (or 2, standby) from its TIMER while a
    host burst runs: the burst goes on without a gap or a NOTICE 5, the panel shows only the indicator and reads
    the value written, the program keeps running, and the panel goes off (or into standby) when the microphone
    stops"""
    arm = e.arm
    out = res.setdefault("popup", {})
    k = arm.k
    band = arm.alloc(320 * 100)
    popup = (g.Code().ins("FILL", k(4 * band), k(320), k(0), k(0), k(640), k(100), k(9))
             .ins("DRAW", k(4 * band), k(320), k(0), k(0), k(0), k(150), k(640), k(100))
             .ins("SYS_READ", 0, k(0x00)).ins("ADD", 0, 0, k(1500000)).ins("SYS_WRITE", k(0x22), 0).ins("END"))
    hide = g.Code().ins("SYS_WRITE", k(0x21), k(hide_value)).ins("NOTIFY", k(0x41DE), k(0)).ins("END")
    start = g.Code().ins("END")
    for addr, c in ((0xD000, popup), (0xD100, hide), (0xD200, start)):
        arm.write(addr, c.b)
    table = [0] * 16
    table[0], table[1], table[12] = 0xD200, 0xD100, 0xD000
    arm.write(0x80, struct.pack("<16I", *table))
    arm.wait_idle(30)
    assert arm.vm_start(0x80) == 0
    arm.add(g.Code().ins("RAISE", k(12)))
    arm.wait_idle(10)
    arm.wait(0.3)
    got = []
    arm.on_mic = lambda first, n, frames, t_dev, t: got.append(first)
    n0, h0 = len(arm.notices), len(arm.notifies)
    assert arm.mic(True)["status"] == 0
    arm.mic_hold(3.0)
    hidden = [x for x in arm.notifies[h0:] if x["a"] == 0x41DE]
    ppm = e.run / "popup.ppm"
    arm.tr.call_model("sysbus.spim_a.jbd", "DumpFrame", str(ppm))
    arm.pump()
    arm.pump()
    px = g.read_ppm_levels(ppm)
    ppm.unlink()
    want = bytearray(640 * 480)
    for y in range(4, 20):
        want[y * 640 + 620:y * 640 + 636] = bytes([15]) * 16
    st = arm.vm_status()
    out["during"] = {"hidden": len(hidden), "events": len(got), "gaps": sum(1 for a, b in zip(got, got[1:]) if b - a != 5),
                     "notices": [x["body"] for x in arm.notices[n0:] if x["kind"] == 5],
                     "blank_indicator": bytes(px) == bytes(want), "panel_prop": arm.props("PANEL")["PANEL"],
                     "vm": st["state_name"]}
    assert arm.mic(False)["status"] == 0
    arm.wait(0.3)
    arm.on_mic = None
    out["after"] = {"panel_read": arm.panel_read(0, 1)["status"], "vm": arm.vm_status()["state_name"],
                    "panel_prop": arm.props("PANEL")["PANEL"]}
    d = out["during"]
    assert d["hidden"] == 1 and d["events"] >= 55 and not d["gaps"] and not d["notices"], out
    assert d["blank_indicator"] and d["panel_prop"] == hide_value and d["vm"] == "running", out
    assert out["after"] == {"panel_read": 11, "vm": "running", "panel_prop": hide_value}, out


def test_mic_popup(name):
    return run(name, CUR_MIC, mic_popup_body, extra=SINE)


def test_mic_popup_standby(name):
    return run(name, CUR_MIC, lambda e, res: mic_popup_body(e, res, 2), extra=SINE)


def mic_lease_body(e, res):
    """VM.md §6.5's lease: MIC 1 holds the microphone 2 s and a MIC 1 with the same rate renews it; renewed every
    0.8 s it stays on (3 s, no gap); left alone with the host still connected it goes off at the lease's end with
    NOTICE 5 reason 3, the indicator cleared and no MIC_DATA made after"""
    arm = e.arm
    out = res.setdefault("lease", {})
    got = []
    arm.on_mic = lambda first, n, frames, t_dev, t: got.append((first, t_dev))
    r = arm.ctrl("MIC", bytes([1, 0]))
    arm.mic_hold(3.0)
    st = arm.mic_stats()
    out["renewed"] = {"on": st["on"], "events": len(got), "frames": st["frames"]}
    assert st["on"] and 55 <= len(got) <= 65 and all(b[0] - a[0] == 5 for a, b in zip(got, got[1:])), out
    n0 = len(arm.notices)
    t_last = arm.ctrl("MIC", bytes([1, 0]))["t_dev"]
    arm.run_until(lambda: any(x["kind"] == 5 for x in arm.notices[n0:]), 5, "NOTICE 5")
    note = [x for x in arm.notices[n0:] if x["kind"] == 5][0]
    arm.wait(0.5)
    after = [t for _, t in got if g.signed32(t - note["t_dev"]) > 20000]
    ppm = e.run / "lease.ppm"
    arm.tr.call_model("sysbus.spim_a.jbd", "DumpFrame", str(ppm))
    arm.pump()
    arm.pump()
    px = g.read_ppm_levels(ppm)
    ppm.unlink()
    out["expired"] = {"reason": note["body"], "after_last_renewal_s": round(g.signed32(note["t_dev"] - t_last) / 1e6, 3),
                      "made_after": len(after), "indicator": any(px[y * 640 + 620] for y in range(4, 20)),
                      "on": arm.mic_stats()["on"]}
    arm.on_mic = None
    x = out["expired"]
    assert r["status"] == 0 and x["reason"] == "03" and 1.95 <= x["after_last_renewal_s"] <= 2.15, out
    assert not x["made_after"] and not x["indicator"] and not x["on"], out


def test_mic_lease(name):
    return run(name, CUR_MIC, mic_lease_body, extra=SINE)


def mic_tool_body(e, res):
    """tools/mic_check.py (the on-arm procedure) against the emulated right arm: 2 rounds of 2 s"""
    import mic_check
    arm = e.arm
    out = mic_check.check(arm, 2, 2.0, 0, lambda: arm.tr.go_silent(3000), need_encrypted=False)
    res["mic_tool"] = out["summary"]
    sm = out["summary"]
    assert sm["rounds_ok"] == "2/2" and sm["after_disconnect_ok"], out
    assert all(abs(f - 100) <= 10 for f in sm["frames_per_s"]), sm
    assert all(r is not None and abs(r - 20 * math.log10(5657 / 32768)) <= 1.5 for r in sm["rms_dbfs"]), sm


def test_mic_tool(name):
    return run(name, CUR_MIC, mic_tool_body, extra=SINE)


def mic_tool_draw_body(e, res):
    """tools/mic_check.py with display traffic (arena writes and full-panel DRAWs during each burst): no frame lost,
    the indicator over the drawn frame while on and cleared to level 0 after"""
    import mic_check
    arm = e.arm
    out = mic_check.check(arm, 2, 2.0, 0, lambda: arm.tr.go_silent(3000), need_encrypted=False, draw=True)
    res["mic_tool_draw"] = out["summary"]
    sm = out["summary"]
    # the emulated link is slower than the arm's (the arm lost nothing, power.md §9): a dropped event is allowed
    # here as long as the counter shows it
    for r in out["rounds"]:
        assert r["indicator_on"]["block"] and r["indicator_off"]["clear"] and not r["events_after_off"], r
        assert r["burst"]["bad_frames"] == 0 and r["burst"]["gaps"] <= 5 * r["arm"]["event_drops"], r
    assert sm["after_disconnect_ok"] and not sm["lost"] and all(t["draws"] >= 2 for t in sm["traffic"]), sm


def draw_bench_body(e, res):
    """tools/draw_bench.py's rectangles on the emulated arm (timings are not the arm's): every one leaves the panel
    equal to the mirror"""
    import draw_bench
    arm = e.arm
    sizes = [(640, 480), (640, 9), (640, 17), (348, 197), (151, 150), (2, 200), (640, 1)]
    out = {}
    for w, h in sizes:
        draw_bench.bench(arm, 1, [(w, h)])
        arm.arena_top = 0x400
        arm.set_panel(1)
        arm.wait_idle(10)
        buf = arm.alloc(320 * 480)
        arm.fill(buf, 320, 0, 0, 640, 480, 7)
        arm.fill(buf, 320, 3, 5, 600, 470, 11)
        arm.wait_draw(arm.draw(buf, 320, 0, 0, w, h))
        bad = emu_panel(arm, e.run / "png", f"bench_{w}x{h}")
        out[f"{w}x{h}"] = bad
        arm.arena_top = 0x400
    res["draw_bench"] = out
    assert not any(out.values()), out


def test_draw_bench(name):
    return run(name, APP, draw_bench_body)


def panel_standby_body(e, res):
    """DIAG SET 6 (test builds): the panel into deep power-down with its rails on (1) or VCOM off too (2) instead of
    off; it reads as standby (PANEL 2, PANEL_READ 11); the next DRAW wakes it with the cache cleared (the frame equals the
    mirror's black panel plus the DRAW) much faster than a full power-on; 0 turns a panel in standby fully off"""
    arm = e.arm
    out = res.setdefault("standby", {})
    buf = arm.alloc(320 * 480)
    arm.fill(buf, 320, 0, 0, 640, 480, 9)
    arm.wait_draw(arm.draw(buf, 320, 0, 0, 640, 480))
    full = struct.unpack_from("<I", arm.ctrl_ok("DIAG", bytes([5, 6, 0xFF])))[0]
    out["full_power_on_ms"] = full
    for mode in (1, 2):
        rep = arm.ctrl("DIAG", bytes([5, 6, mode]))
        arm.mirror.set_panel(0)
        out[f"mode{mode}_status"] = rep["status"]
        assert rep["status"] == 0, rep
        assert arm.props("PANEL")["PANEL"] == 2 and arm.panel_read(0, 1)["status"] == 11, out
        arm.wait_draw(arm.draw(buf, 320, 100, 50, 151, 77))
        wake = struct.unpack_from("<I", arm.ctrl_ok("DIAG", bytes([5, 6, 0xFF])))[0]
        out[f"mode{mode}_wake_ms"] = wake
        out[f"mode{mode}_mismatch"] = emu_panel(arm, e.run / "png", f"standby{mode}")
        assert out[f"mode{mode}_mismatch"] == 0 and wake < full, out
    assert arm.ctrl("DIAG", bytes([5, 6, 1]))["status"] == 0
    arm.mirror.set_panel(0)
    assert arm.ctrl("DIAG", bytes([5, 6, 0]))["status"] == 0
    arm.wait_draw(arm.draw(buf, 320, 0, 0, 640, 480))
    out["after_off_power_on_ms"] = struct.unpack_from("<I", arm.ctrl_ok("DIAG", bytes([5, 6, 0xFF])))[0]
    out["after_off_mismatch"] = emu_panel(arm, e.run / "png", "after_off")
    assert out["after_off_mismatch"] == 0 and out["after_off_power_on_ms"] > out["mode1_wake_ms"], out


def test_panel_standby(name):
    return run(name, APP, panel_standby_body)


def touch_body(e, res):
    """M7, the TouchBar controller (g1-fw's model with this project's interrupt line, wear events and modes): set up
    as stock at boot (chip ID, handshake, the 44-entry table, commit, events 0x58F) and asleep while unused (touches
    not sensed); an EVENTS bit or a running program makes it scan; taps less than 305 ms apart give one TOUCH (1, 2,
    3) 305 ms after the last release, a press held 458 ms TOUCH 4 and its release 5 (INPUT events, a program's TOUCH
    handler, TOUCH_GESTURE); WORN reads 0 in the case, stays 0 after leaving it, then follows the controller's
    in/out-of-ear events; a
    controller reset loads the table again; the driver reads the controller only on its interrupt"""
    arm = e.arm
    out = res.setdefault("touch", {})
    injected = 0
    if vnum(VER) >= (0, 5, 26):   # the arms folded: the link value does not classify them as worn
        common.link_signal(arm, 30)
        arm.run_until(lambda: arm.peer_stats()["link_value"] == 30, 15, "link value 30")

    def m(method, arg=None):
        nonlocal injected
        injected += {"Tap": 2, "Press": 1, "Release": 1, "Worn": 1, "Removed": 1}.get(method, 0)
        return common.model_call(arm, TOUCH, method, arg)

    def model():
        return {k: m("get_" + k) for k in ("ConfigWriteCount", "CommitWritten", "EnableMask", "Mode")}

    def inputs(since, trig):
        return [x for x in arm.inputs[since:] if x["trigger"] == trig]

    t = arm.touch_stats()
    out["boot"] = {k: t[k] for k in ("present", "scanning", "chip_id", "wst", "irqen", "loads", "events")}
    out["model_boot"] = model()
    assert res["hello"]["triggers"] == triggers_for(VER), res["hello"]
    assert t["present"] and not t["scanning"] and t["chip_id"] == 0x20223203 and t["wst"] == 0x03000000, out
    assert t["irqen"] == 0x58F and t["loads"] == 1 and t["events"] == 0, out
    assert out["model_boot"] == {"ConfigWriteCount": "45", "CommitWritten": "True", "EnableMask": "1423", "Mode": "2"}, out
    m("Tap")
    injected -= 2   # not sensed: asleep
    assert arm.events_mask(1 << 7 | 1 << 8) == 0x180
    arm.wait(0.3)
    t = arm.touch_stats()
    out["scanning"] = {"diag": t["scanning"], "wst": t["wst"], "mode": m("get_Mode")}
    assert out["scanning"] == {"diag": True, "wst": 0x01000000, "mode": "1"}, out
    n0 = len(arm.inputs)
    t0 = arm.tr.now()
    m("Tap")
    arm.wait(0.8)
    tap = inputs(n0, 8)
    out["tap_after_s"] = round(tap[0]["t"] - t0, 3) if tap else None
    for k in (2, 3):
        for _ in range(k):
            m("Tap")
            arm.wait(0.15)
        arm.wait(0.6)
    t0 = arm.tr.now()
    m("Press")
    arm.wait(0.8)
    held = inputs(n0, 8)[-1]
    out["hold_after_s"] = round(held["t"] - t0, 3)
    m("Release")
    arm.wait(0.5)
    out["gestures"] = [x["value"] for x in inputs(n0, 8)]
    out["gesture_prop"] = arm.props("TOUCH_GESTURE")["TOUCH_GESTURE"]
    assert out["gestures"] == [1, 2, 3, 4, 5] and out["gesture_prop"] == 5, out
    assert 0.29 <= out["tap_after_s"] <= 0.45 and 0.44 <= out["hold_after_s"] <= 0.6, out
    n0 = len(arm.inputs)
    m("Worn")
    arm.wait(0.3)
    out["worn_in_case"] = arm.props("WORN")["WORN"]
    arm.tr.set_model(NPM, "VbusConnected", "false")
    if vnum(VER) >= (0, 5, 26):
        common.lift(arm)   # IN_CASE is a state: it goes 3 s after the last VBUS sample once the arm moved
    arm.wait(5.0)   # out of the case: WORN stays 0 until the controller says otherwise
    out["worn_left_case"] = [x["value"] for x in inputs(n0, 7)]
    m("Worn")
    arm.wait(0.3)
    m("Removed")
    arm.wait(0.3)
    m("Worn")
    arm.wait(0.3)
    arm.tr.set_model(NPM, "VbusConnected", "true")
    arm.run_until(lambda: len(inputs(n0, 7)) >= (2 if vnum(VER) >= (0, 5, 24) else 4), 5, "WEAR back in the case")
    out["wear"] = [x["value"] for x in inputs(n0, 7)]
    # 0.5.24+: once the net core reported its link value, the controller's out-of-ear event no longer clears WORN
    # (the link value does, VM.md §6.7); its in-ear event sets 1 at once. 0.5.26+: the in-ear event made in the case
    # counts when the arm leaves it (WORN 1 then; a classification as worn can end IN_CASE)
    want = [1, 0] if vnum(VER) >= (0, 5, 24) else [1, 0, 1, 0]
    left = [1] if vnum(VER) >= (0, 5, 26) else []
    assert out["worn_in_case"] == 0 and out["worn_left_case"] == left and out["wear"] == want, out
    arm.wait(1.5)
    assert arm.events_mask(0) == 0
    arm.wait(0.3)
    k = arm.k
    handler = g.Code().ins("SYS_READ", 0, k(0x34)).ins("NOTIFY", 0, k(0x70C4)).ins("END")
    arm.write(0xC000, handler.b)
    table = [0] * 16
    table[0], table[8] = 0xC100, 0xC000
    arm.write(0xC100, g.Code().ins("END").b)
    arm.write(0x80, struct.pack("<16I", *table))
    arm.wait_idle(10)
    out["asleep_again"] = arm.touch_stats()["scanning"]
    assert arm.vm_start(0x80) == 0
    arm.wait(0.3)
    out["program_scanning"] = arm.touch_stats()["scanning"]
    h0 = len(arm.notifies)
    m("Tap")
    m("Tap")
    arm.wait(0.8)
    out["program_notifies"] = [(x["a"], x["b"]) for x in arm.notifies[h0:]]
    arm.vm_stop()
    arm.wait(0.3)
    assert not out["asleep_again"] and out["program_scanning"] and out["program_notifies"] == [(2, 0x70C4)], out
    assert arm.events_mask(1 << 8) == 1 << 8
    arm.wait(0.3)
    m("ControllerReset")
    arm.wait(0.5)
    n0 = len(arm.inputs)
    m("Tap")
    arm.wait(0.8)
    t = arm.touch_stats()
    out["after_reset"] = {"loads": t["loads"], "model": model(), "gestures": [x["value"] for x in inputs(n0, 8)]}
    out["events"] = {"read": t["events"], "injected": injected + 1, "i2c_errors": t["i2c_errors"]}
    assert out["after_reset"] == {"loads": 2, "gestures": [1], "model": {"ConfigWriteCount": "43", "CommitWritten": "True",
                                                                          "EnableMask": "1423", "Mode": "1"}}, out
    assert t["events"] == injected + 1 and t["i2c_errors"] == 0, out


def test_touch(name):
    return run(name, APP, touch_body)


OPT = "sysbus.twim1.opt3001"


def light_body(e, res):
    """M8, the light sensor (g1-fw's OPT3001 model at TWIM1 0x45): shut down, LIGHT UNKNOWN while the panel is off or
    nothing uses it; with the panel on and the LIGHT EVENTS bit it converts every 800 ms (LIGHT in 0.01 lux) and
    INPUT events report leaving [LIGHT_LO, LIGHT_HI]; AUTO_BRIGHTNESS sets BRIGHTNESS by stock's table once two
    readings in a row ask for a level 2 or more away (VM.md §6.7); the panel off stops it; RESET_SESSION clears
    AUTO_BRIGHTNESS"""
    arm = e.arm
    k, M = arm.k, g.M32
    out = res.setdefault("light", {})

    def write(prop, value):
        arm.add(g.Code().ins("SYS_WRITE", k(prop), k(value & M)))
        arm.wait_idle(10)

    def lux(v, wait=1.9):
        arm.tr.set_model(OPT, "Lux", str(v))
        arm.wait(wait)
        return arm.props("LIGHT", "BRIGHTNESS")

    arm.tr.set_model(OPT, "Lux", "1250.5")
    arm.wait(1.0)
    out["off"] = arm.props("LIGHT")["LIGHT"]
    arm.set_panel(1)
    arm.wait_idle(10)
    arm.wait(1.2)
    out["on_unused"] = arm.props("LIGHT")["LIGHT"]
    assert arm.events_mask(1 << 4) == 1 << 4
    arm.wait(1.8)
    out["reading"] = arm.props("LIGHT")["LIGHT"]
    assert out["off"] is None and out["on_unused"] is None and abs(out["reading"] - 125050) <= 125050 * 0.002, out
    write(0x26, 100000)
    write(0x27, 150000)
    n0 = len(arm.inputs)
    lux(2000.0)
    out["range_left"] = [(x["trigger"], x["value"]) for x in arm.inputs[n0:]]
    assert out["range_left"] and out["range_left"][0][0] == 4 and abs(out["range_left"][0][1] - 200000) <= 400, out
    assert arm.events_mask(0) == 0
    write(0x38, 1)
    steps = [(10.0, 2), (100.0, 15), (600.0, 33), (5000.0, 42), (1500.0, 42), (119.0, 42), (115.0, 16), (125.0, 16),
             (135.0, 18)]   # 1500 lux asks 41, one level away: kept; 119 lux once: kept; then 16 at the second reading
    got = []
    for value, level in steps:
        if value == 119.0:
            arm.tr.set_model(OPT, "Lux", "119.0")
            arm.wait(0.85)   # one reading only
            got.append(arm.props("BRIGHTNESS")["BRIGHTNESS"])
            continue
        got.append(lux(value)["BRIGHTNESS"])
    out["auto"] = got
    out["auto_prop"] = arm.props("AUTO_BRIGHTNESS")["AUTO_BRIGHTNESS"]
    out["diag"] = arm.light_stats()
    assert out["diag"]["present"] and out["diag"]["converting"] and out["diag"]["readings"] >= 15, out
    assert out["diag"]["i2c_errors"] == 0, out
    assert got == [level for _, level in steps] and out["auto_prop"] == 1, out
    arm.set_panel(0)
    arm.wait_idle(10)
    arm.wait(1.0)
    out["panel_off"] = arm.props("LIGHT", "BRIGHTNESS")
    arm.reset_session()
    out["after_reset"] = arm.props("AUTO_BRIGHTNESS")["AUTO_BRIGHTNESS"]
    assert out["panel_off"] == {"LIGHT": None, "BRIGHTNESS": 18} and out["after_reset"] == 0, out


def test_light(name):
    return run(name, APP, light_body)


def gyro_body(e, res):
    """M8, the gyroscope: off at boot (GYR_* UNKNOWN, the model's gyroscope off); GYRO_RATE 52 turns it on, and GYR_*
    are the injected rates (sensor axes) through the arm's head map, in mdps, within 2 %; a rate the spec does not list
    is error 5; GYRO_RATE 0 and RESET_SESSION turn it off"""
    arm = e.arm
    k = arm.k
    out = res.setdefault("gyro", {})
    st = arm.imu_stats()
    m = st["head_map"]
    det = (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
           m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
    out["boot"] = {"props": arm.props("GYR_X", "GYRO_RATE"), "model": common.model_call(arm, LSM, "get_GyroscopeEnabled")}
    rates = (30.0, -45.0, 12.5)
    for axis, v in zip("XYZ", rates):
        arm.tr.set_model(LSM, f"AngularRate{axis}", str(v))
    arm.add(g.Code().ins("SYS_WRITE", k(0x33), k(52)))
    arm.wait_idle(10)
    arm.wait(0.3)
    got = arm.props("GYR_X", "GYR_Y", "GYR_Z", "GYRO_RATE")
    want = [round((1 if det > 0 else -1) * 1000 * sum(m[i][j] * rates[j] for j in range(3))) for i in range(3)]
    out["on"] = {"got": got, "want": want, "model": common.model_call(arm, LSM, "get_GyroscopeEnabled")}
    vals = [got["GYR_X"], got["GYR_Y"], got["GYR_Z"]]
    assert got["GYRO_RATE"] == 52 and out["on"]["model"] == "True", out
    assert all(abs(a - b) <= max(0.02 * abs(b), 20) for a, b in zip(vals, want)), out
    n0 = len(arm.errors)
    arm.add(g.Code().ins("SYS_WRITE", k(0x33), k(50)))
    arm.wait_idle(10)
    out["bad_rate"] = [(x["code"], x["detail"]) for x in arm.errors[n0:]]
    arm.add(g.Code().ins("SYS_WRITE", k(0x33), k(0)))
    arm.wait_idle(10)
    out["off"] = {"props": arm.props("GYR_X", "GYRO_RATE"), "model": common.model_call(arm, LSM, "get_GyroscopeEnabled")}
    arm.add(g.Code().ins("SYS_WRITE", k(0x33), k(104)))
    arm.wait_idle(10)
    arm.reset_session()
    out["after_reset"] = {"props": arm.props("GYRO_RATE"), "model": common.model_call(arm, LSM, "get_GyroscopeEnabled")}
    assert out["boot"] == {"props": {"GYR_X": None, "GYRO_RATE": 0}, "model": "False"}, out
    assert out["bad_rate"] == [(5, 50)] and out["off"] == {"props": {"GYR_X": None, "GYRO_RATE": 0}, "model": "False"}, out
    assert out["after_reset"] == {"props": {"GYRO_RATE": 0}, "model": "False"}, out


def test_gyro(name):
    return run(name, APP, gyro_body)


def image_hash(img):
    """an MCUboot image's SHA-256 TLV (what SMP lists)"""
    hs, size = struct.unpack_from("<H", img, 8)[0], struct.unpack_from("<I", img, 12)[0]
    p = hs + size
    magic, n = struct.unpack_from("<HH", img, p)
    if magic == 0x6908:
        p += n
        magic, n = struct.unpack_from("<HH", img, p)
    q = p + 4
    while q < p + n:
        t, ln = struct.unpack_from("<HH", img, q)
        if t == 0x10:
            return img[q + 4:q + 4 + ln].hex()
        q += 4 + ln
    return ""


def restore_body(e, res):
    """README "Restore Even's firmware", with flash.py itself on the emulated arm: from VERSION, Even's 1.6.6 app
    through g1v0's update service (upload, check, confirm, reset); MCUboot copies it, and Even's firmware boots and
    answers (its UART service and SMP list)"""
    check_hello(res["hello"], VER)
    e.arm.tr.close()
    r = subprocess.run([sys.executable, "-I", str(FW / "flash.py"), "--emu", str(e.run / "vphone.sock"), "--yes",
                        "restore", "1.6.6"], capture_output=True, text=True, timeout=1800,
                       env=dict(os.environ, G1_WORK=str(WORK)))
    res["flash_py"] = [ln.strip() for ln in r.stdout.replace("\r", "\n").splitlines() if ln.strip() and "upload " not in ln]
    res["flash_py_stderr"] = r.stderr.splitlines()[-6:]
    images = [ln for ln in res["flash_py"] if ln.startswith("image 0 slot 0:")]
    stock = image_hash((WORK / "firmware/1.6.6/app_update.bin").read_bytes())
    res["stock_hash"] = stock
    assert r.returncode == 0 and res["flash_py"][-1] == "done", res
    assert images and VER in images[0] and stock[:16] in images[-1], res   # Even's header says version 0.0.0


def test_restore(name):
    return run(name, APP, restore_body)


def last_power_on_ms(arm):
    return struct.unpack_from("<I", arm.ctrl_ok("DIAG", bytes([5, 6, 0xFF])))[0]


def panel_prop_body(e, res):
    """VM.md §2.5 PANEL = 2 (standby), from on and from off: it reads 2, PANEL_READ answers 11 and HELLO shows the
    panel off; a BRIGHTNESS written meanwhile is applied at the wake; the next DRAW wakes it much faster than a full
    power-on, with the cache cleared (the frame equals the mirror); with no host and no program the unattended rule
    turns standby fully off (2 s in the test build)"""
    arm = e.arm
    out = res.setdefault("standby", {})
    buf = arm.alloc(320 * 480)
    arm.fill(buf, 320, 0, 0, 640, 480, 9)
    arm.wait_draw(arm.draw(buf, 320, 0, 0, 640, 480))
    full = out["full_power_on_ms"] = last_power_on_ms(arm)
    for start in ("on", "off"):
        if start == "off":
            arm.set_panel(0)
        arm.set_panel(2)
        arm.wait_idle(10)
        st = {"panel": arm.props("PANEL")["PANEL"], "read": arm.panel_read(0, 1)["status"],
              "hello_on": arm.hello()["panel_on"]}
        arm.set_brightness(30)
        arm.wait_draw(arm.draw(buf, 320, 100, 50, 151, 77))
        st.update(wake_ms=last_power_on_ms(arm), after=arm.props("PANEL", "BRIGHTNESS"),
                  mismatch=emu_panel(arm, e.run / "png", f"standby_from_{start}"))
        out[start] = st
        arm.set_brightness(21)
        assert st["panel"] == 2 and st["read"] == 11 and not st["hello_on"] and st["mismatch"] == 0, out
        assert st["wake_ms"] < full / 4 and st["after"] == {"PANEL": 1, "BRIGHTNESS": 30}, out
    arm.set_panel(2)
    arm.wait_idle(10)
    before = arm.losses()
    arm.tr.go_silent(3500)
    arm.run_until(lambda: arm.losses() > before, 30, "link loss")
    arm.setup(60)
    out["after_away"] = arm.props("PANEL")["PANEL"]
    assert out["after_away"] == 0, out


def test_panel_prop(name):
    return run(name, CUR_MIC, panel_prop_body)


def in_case_body(e, res):
    """VM.md §2.5 IN_CASE: VBUS or a case message within the last 3 s, seen through CASE INPUT events (value bit
    0): a 1.5 s VBUS gap changes nothing; without VBUS it goes to 0 3-4 s later (the PMIC samples every second);
    case messages every 0.65 s without VBUS keep it at 1 until about 3 s after the last one"""
    arm = e.arm
    out = res.setdefault("in_case", {})
    vbus = lambda on: arm.tr.set_model(NPM, "VbusConnected", "true" if on else "false")

    def changes(since, last=1):
        got = []
        for x in arm.inputs[since:]:
            if x["trigger"] == 6 and (x["value"] & 1) != last:
                last = x["value"] & 1
                got.append((last, x["t"]))
        return got

    def wait_change(since, value, timeout):
        arm.run_until(lambda: any(v == value for v, _ in changes(since, 1 - value)), timeout, f"IN_CASE {value}")
        return next(t for v, t in changes(since, 1 - value) if v == value)

    assert arm.props("IN_CASE")["IN_CASE"] == 1 and arm.events_mask(1 << 6) == 1 << 6
    n0 = len(arm.inputs)
    vbus(False)
    arm.wait(1.5)
    vbus(True)
    arm.wait(4)
    out["gap_changes"] = changes(n0)
    n0 = len(arm.inputs)
    t_off = arm.tr.now()
    vbus(False)
    out["off_to_zero_s"] = round(wait_change(n0, 0, 8) - t_off, 2)
    n1 = len(arm.inputs)
    vbus(True)
    t_on = arm.tr.now()
    out["on_to_one_s"] = round(wait_change(n1, 1, 4) - t_on, 2)
    arm.wait(1)
    n0 = len(arm.inputs)
    vbus(False)
    for _ in range(10):
        common.case_request(arm, "130001006408", wait=0.65)
    t_last = arm.tr.now()
    out["during_messages"] = changes(n0)
    n1 = len(arm.inputs)
    out["last_message_to_zero_s"] = round(wait_change(n1, 0, 8) - t_last, 2)
    vbus(True)
    arm.wait(2)
    assert not out["gap_changes"] and not out["during_messages"], out
    assert 2.9 <= out["off_to_zero_s"] <= 5.2 and out["on_to_one_s"] <= 1.6, out
    assert 2.3 <= out["last_message_to_zero_s"] <= 4.6, out


def panel_guard_body(e, res):
    """VM.md §6.6 the panel guard on scripted battery values (0.5.54+; the currents in shares of the budget B, which
    DIAG GUARD reports from 0.5.56, else 120 mA): each 1 s round averages a burst of conversions, I_avg is their
    exponential average (1/8 from 0.5.56, 1/4 before). A current that alternates 0.33 / 1.17 B per round (mean 0.75 B)
    causes no step; 0.83 / 1.33 B (mean 1.08 B) one attack within 18 s (8 s at 1/4), straight to the table's level
    (33-39 from 42: 39 just over the budget) and no second one within 4 s; a single 2.4 B round among 0.5 B rounds (I_avg 0.975 B, not over) nothing. Release +1
    level per 5 s at 0.75 B, +3 at 0.3 B (under 50 %). VBAT under 3150 mV takes 2 levels, 41 C a step; BRIGHTNESS reads the written value; a
    BRIGHTNESS_MAX write sends no NOTICE; the start value is 42 at a rest voltage >= 3600 mV, else 24; BRIGHTNESS_CAP
    reads 42 while the panel is off or in standby; PANEL_GUARD 0 gives 42 and no NOTICE 7; VBUS gives 42. Times are
    the arm's (t_dev). Folded and still, the arm keeps IN_CASE for 40 s after the last VBUS sample (test build), so
    the unbonded link stays obeyed; a VBUS sample between the parts refreshes it."""
    arm = e.arm
    out = res.setdefault("guard", {})
    model = lambda k, v: arm.tr.set_model(NPM, k, str(v))
    cap = lambda: arm.props("BRIGHTNESS_CAP")["BRIGHTNESS_CAP"]
    secs = lambda t, t0: round(g.signed32(t - t0) / 1e6, 2)

    def notices(n0):
        return [(x["t_dev"], bytes.fromhex(x["body"])) for x in arm.notices[n0:] if x["kind"] == 7]

    def first_notice(timeout=6):
        n0, d0 = len(arm.notices), arm.device_now()
        arm.run_until(lambda: notices(n0), timeout, "NOTICE 7")
        t, b = notices(n0)[0]
        return secs(t, d0), b[0], b[1]

    def alternate(a, b, seconds, until_step=False):
        """the current a, b, a, ... one value per second (or until the first step down, at most 3 x as long: the
        emulator runs slower under load); the NOTICE 7s meanwhile as (s after the start on the arm's clock, cap)"""
        n0, d0 = len(arm.notices), arm.device_now()
        for k in range(int(seconds) * (3 if until_step else 1)):
            model("BatteryCurrentMilliAmps", a if k % 2 == 0 else b)
            arm.wait(1.0)
            if until_step and k + 1 >= seconds and notices(n0):
                break
        return [(secs(t, d0), x[0]) for t, x in notices(n0)]

    def vbus_sample():
        model("VbusConnected", "true")
        arm.run_until(lambda: cap() == 42, 6, "BRIGHTNESS_CAP 42 with VBUS")
        model("VbusConnected", "false")
        arm.wait(1.5)

    buf = arm.alloc(320 * 120)
    arm.fill(buf, 320, 0, 0, 640, 120, 15)
    lit = lambda: arm.wait_draw(arm.draw(buf, 320, 0, 0, 640, 120))
    arm.sync_clock()
    common.link_signal(arm, 30)   # folded: classified as not worn (3 s under 40), so IN_CASE stays 1 without VBUS
    arm.run_until(lambda: arm.case_stats()["wear"] == 0, 15, "classified as not worn")
    model("BatteryVoltageMilliVolts", 3800)
    model("BatteryCurrentMilliAmps", 60)
    model("BatteryTemperatureCelsius", 25)
    model("VbusConnected", "false")
    arm.wait(2.5)
    arm.set_brightness(42)
    lit()
    out["start"] = arm.props("BRIGHTNESS", "BRIGHTNESS_MAX", "BRIGHTNESS_CAP", "IN_CASE")
    assert out["start"] == {"BRIGHTNESS": 42, "BRIGHTNESS_MAX": 42, "BRIGHTNESS_CAP": 42, "IN_CASE": 1}, out
    arm.wait(1.2)
    out["burst"] = arm.guard_state()
    assert out["burst"]["burst_n"] >= 16 and 50 <= out["burst"]["burst_ma"] <= 70, out
    bud = out["burst"]["budget_ma"] or 120
    ma = lambda share: int(round(share * bud))
    # the noise of a pulsed current: no step at a mean of 0.75 B; one straight step at 1.08 B
    out["alternate_075"] = alternate(ma(0.33), ma(1.17), 10)
    assert not out["alternate_075"], out
    out["alternate_108"] = alternate(ma(0.83), ma(1.33), 20, True)
    model("BatteryCurrentMilliAmps", ma(0.5))
    steps = out["alternate_108"]
    assert steps and steps[0][0] <= 18.0 and 33 <= steps[0][1] <= 39, out
    assert len(steps) == 1 or steps[1][0] - steps[0][0] >= 3.9, out
    vbus_sample()
    # one 2.4 B round among 0.5 B rounds: nothing; the average starts at the 0.5 B rounds after the VBUS sample
    arm.wait(3)
    n0 = len(arm.notices)
    model("BatteryCurrentMilliAmps", ma(2.4))
    arm.run_until(lambda: arm.guard_state()["burst_ma"] >= ma(2.0), 3, "a round of 2.4 B")
    model("BatteryCurrentMilliAmps", ma(0.5))
    out["one_round_avg"] = arm.guard_state()["avg_ma"]
    arm.wait(4)
    out["one_round"] = [b.hex() for _, b in notices(n0)]
    assert not out["one_round"] and ma(0.65) <= out["one_round_avg"] <= bud, out
    model("BatteryCurrentMilliAmps", ma(0.83))   # the average restarts there after the VBUS sample
    vbus_sample()
    # releases: +1 level per 5 s at 0.75 B (under 85 %), +3 at 0.3 B (under 50 %)
    alternate(ma(0.83), ma(1.33), 12, True)
    model("BatteryCurrentMilliAmps", ma(0.75))
    arm.wait(2)
    n0 = len(arm.notices)
    held = cap()
    arm.run_until(lambda: len(notices(n0)) >= 2, 16, "two releases at 0.75 B")
    r = notices(n0)
    out["release_075"] = {"from": held, "caps": [b[0] for _, b in r], "gap_s": secs(r[1][0], r[0][0])}
    assert out["release_075"]["caps"][:2] == [held + 1, held + 2] and 4.5 <= out["release_075"]["gap_s"] <= 6.0, out
    model("BatteryCurrentMilliAmps", ma(2.0))   # the average restarts at 2 B: one step down, to about level 19
    vbus_sample()
    arm.run_until(lambda: cap() < 30, 8, "a step down at 2 B")
    n0 = len(arm.notices)
    held = cap()
    model("BatteryCurrentMilliAmps", ma(0.3))
    steps = lambda: [b[0] for _, b in notices(n0)]
    arm.run_until(lambda: any(b - a == 3 for a, b in zip([held] + steps(), steps())) or cap() == 42, 26,
                  "a release of 3 levels at 0.3 B")
    out["release_03"] = {"from": held, "caps": steps()}
    assert any(b - a == 3 for a, b in zip([held] + steps(), steps())), out
    model("BatteryCurrentMilliAmps", ma(0.6))
    vbus_sample()
    # VBAT under 3150 mV: 2 levels; 41 C: a step as over the budget; BRIGHTNESS_MAX: no NOTICE 7
    lit()
    arm.wait(1.5)
    level = cap()
    model("BatteryVoltageMilliVolts", 3100)
    s_, c, lv = first_notice()
    model("BatteryVoltageMilliVolts", 3800)
    out["vbat_low"] = {"after_s": s_, "from": level, "cap": c}
    assert s_ <= 2.2 and c == level - 2, out
    arm.wait(1.5)
    level = cap()
    model("BatteryTemperatureCelsius", 41)
    s_, c, lv = first_notice()
    out["hot"] = {"after_s": s_, "from": level, "cap": c, "batt_temp": arm.props("BATT_TEMP")["BATT_TEMP"]}
    model("BatteryTemperatureCelsius", 25)
    assert s_ <= 2.2 and c < level, out
    arm.wait(1.5)
    n0 = len(arm.notices)
    arm.ins("SYS_WRITE", g.PROPS["BRIGHTNESS_MAX"], 20)
    arm.flush()
    arm.wait_idle(10)
    arm.wait(2)
    out["max"] = {"props": arm.props("BRIGHTNESS", "BRIGHTNESS_MAX"), "notices": [b.hex() for _, b in notices(n0)]}
    assert out["max"]["props"] == {"BRIGHTNESS": 42, "BRIGHTNESS_MAX": 20} and not out["max"]["notices"], out
    arm.ins("SYS_WRITE", g.PROPS["BRIGHTNESS_MAX"], 42)
    vbus_sample()
    # off and standby read 42; the start value from the rest voltage
    out["off"] = {}
    for mode in (0, 2):
        arm.set_panel(mode)
        arm.wait_idle(10)
        arm.wait(1.5)
        out["off"][mode] = cap()
    assert out["off"] == {0: 42, 2: 42}, out
    out["start_values"] = {}
    for mv in (3550, 3650):
        arm.set_panel(0)
        arm.wait_idle(10)
        model("BatteryVoltageMilliVolts", mv)
        arm.wait(2.2)
        lit()
        out["start_values"][mv] = cap()
    assert out["start_values"] == {3550: 24, 3650: 42}, out
    # PANEL_GUARD 0: 42, no NOTICE 7 even over the budget; on again: an attack; VBUS: 42
    model("BatteryVoltageMilliVolts", 3800)
    model("BatteryCurrentMilliAmps", 200)
    out["guard_off"] = arm.panel_guard(False)
    arm.wait(0.5)
    n0 = len(arm.notices)
    arm.wait(4)
    out["guard_off_quiet"] = {"notices": [b.hex() for _, b in notices(n0)], "cap": cap()}
    assert out["guard_off"] is False and out["guard_off_quiet"] == {"notices": [], "cap": 42}, out
    out["guard_on"] = arm.panel_guard(True)
    out["on_again"] = first_notice(8)
    assert out["guard_on"] is True and out["on_again"][1] < 42, out
    model("BatteryCurrentMilliAmps", 50)
    model("VbusConnected", "true")
    arm.run_until(lambda: cap() == 42, 6, "BRIGHTNESS_CAP 42 with VBUS")
    out["in_case_end"] = arm.props("IN_CASE")["IN_CASE"]
    assert out["in_case_end"] == 1, out


def test_panel_guard(name):
    return run(name, CUR_MIC, panel_guard_body)


def in_case_state_body(e, res):
    """VM.md §2.5 IN_CASE as a state (0.5.26+; this test build holds it 40 s instead of 400), seen through CASE INPUT
    events (value bit 0): set by VBUS, a case message or the tag's field; with none of them it stays 1 while the
    arm lies still and folded (the link value 30); it goes to 0 3 s after the last of them once the arm moved more
    than 150 mg (a lift) or was classified as worn (the link value 60, WORN 1), or after the hold. The case link
    (M9): the charger enabled after a battery reply with status 0 and not after one with status 2 (stock), a restart
    request answered and logged but not obeyed, CASE_LID, CASE_CHARGING and CASE_BATT_PCT from the poll's bytes."""
    arm = e.arm
    out = res.setdefault("in_case", {})
    vbus = lambda on: arm.tr.set_model(NPM, "VbusConnected", "true" if on else "false")
    tag = "sysbus.twim1.st25dv"

    def changes(since, trig=6):
        return [(x["value"] & 1, x["t"]) for x in arm.inputs[since:] if x["trigger"] == trig]

    def wait_value(since, value, timeout, what, trig=6):
        arm.run_until(lambda: any(v == value for v, _ in changes(since, trig)), timeout, what)
        return next(t for v, t in changes(since, trig) if v == value)

    common.link_signal(arm, 30)
    arm.run_until(lambda: arm.peer_stats()["link_value"] == 30, 15, "link value 30 (folded)")
    assert arm.props("IN_CASE")["IN_CASE"] == 1 and arm.events_mask(1 << 6 | 1 << 7) == 1 << 6 | 1 << 7
    # a full case: no VBUS, no message, no field, the arm still
    n0 = len(arm.inputs)
    vbus(False)
    arm.wait(15)
    out["still"] = {"changes": changes(n0), "props": arm.props("IN_CASE", "WORN"), "case": arm.case_stats()}
    assert not out["still"]["changes"] and out["still"]["props"] == {"IN_CASE": 1, "WORN": 0}, out
    # the tag's field and a case message refresh it
    arm.tr.set_model(tag, "RfField", "true")
    arm.wait(2.5)
    field = arm.case_stats()
    arm.tr.set_model(tag, "RfField", "false")
    common.case_request(arm, "130001006407")
    msg = arm.case_stats()
    out["refresh"] = {"field": [field["field"], field["since_ms"]], "message_since_ms": msg["since_ms"]}
    assert field["field"] and field["since_ms"] < 1500 and msg["since_ms"] < 1500, out
    # a lift: 0 about 3 s after the last message; back in the case at once (before the unencrypted link's 10 s)
    n1 = len(arm.inputs)
    common.lift(arm)
    t0 = arm.tr.now()
    out["lift_to_zero_s"] = round(wait_value(n1, 0, 10, "IN_CASE 0 after a lift") - t0, 2)
    vbus(True)
    wait_value(n1, 1, 4, "IN_CASE 1 again")
    arm.wait(1.5)
    # the hold: nothing at all for 40 s (this build)
    n2 = len(arm.inputs)
    vbus(False)
    t0 = arm.tr.now()
    out["hold_to_zero_s"] = round(wait_value(n2, 0, 60, "IN_CASE 0 after the hold") - t0, 1)
    vbus(True)
    wait_value(n2, 1, 4, "IN_CASE 1 again")
    arm.wait(1.5)
    # worn: the link value 60 classifies it as worn; 0 3 s after the last VBUS sample, WORN 1
    n3 = len(arm.inputs)
    vbus(False)
    common.link_signal(arm, 60)
    t0 = arm.tr.now()
    out["worn_to_zero_s"] = round(wait_value(n3, 0, 15, "IN_CASE 0 when worn") - t0, 2)
    out["wear_after"] = [v for v, _ in changes(n3, 7)]
    vbus(True)
    common.link_signal(arm, 30)
    wait_value(n3, 1, 4, "IN_CASE 1 again")
    arm.run_until(lambda: arm.props("WORN")["WORN"] == 0, 5, "WORN 0 in the case")
    assert 2.0 <= out["lift_to_zero_s"] <= 6 and 38 <= out["hold_to_zero_s"] <= 46, out
    assert out["worn_to_zero_s"] <= 8 and out["wear_after"][:1] == [1], out
    # the case link: the charger enabled after a battery reply with status 0 (charging), not after 2 (too warm)
    arm.tr.set_model(NPM, "BatteryVoltageMilliVolts", "3900")
    arm.wait(6)
    c0 = arm.case_stats()["charger_enables"]
    r_low = common.case_request(arm, "130001006407")
    c1 = arm.case_stats()["charger_enables"]
    arm.tr.set_model(NPM, "BatteryTemperatureCelsius", "62")
    arm.wait(6)
    r_warm = common.case_request(arm, "130001006407")
    c2 = arm.case_stats()["charger_enables"]
    arm.tr.set_model(NPM, "BatteryTemperatureCelsius", "25")
    out["charger"] = {"low_status": r_low[4] if r_low else None, "warm_status": r_warm[4] if r_warm else None,
                      "enables": [c0, c1, c2]}
    assert out["charger"]["low_status"] == 0 and c1 == c0 + 1 and out["charger"]["warm_status"] == 2 and c2 == c1, out
    boots, epoch = arm.info["boot_count"], arm.epoch
    r = common.case_request(arm, "0B0103010000")
    arm.wait(1.0)
    h = arm.hello()
    out["restart"] = {"reply": r.hex(), "restarts": arm.case_stats()["restarts"], "boots": [boots, h["boot_count"]]}
    assert r[:1] == b"\x0b" and out["restart"]["restarts"] == 1 and h["boot_count"] == boots and arm.epoch == epoch, out
    common.case_request(arm, "13000100E408")
    out["case_props"] = arm.props("CASE_LID", "CASE_CHARGING", "CASE_BATT_PCT")
    assert out["case_props"] == {"CASE_LID": 1, "CASE_CHARGING": 1, "CASE_BATT_PCT": 100}, out


def case_hold_body(e, res):
    """VM.md §6.6 (0.5.60+): no case hold, display policy is the program's (the library's PanelPolicy). The emulated
    arm lies in the case: a DRAW shows (PANEL_READ matches), PANEL 2 is standby and a DRAW wakes it, the microphone
    starts, no NOTICE 6; CASE_PANEL (0x15) is an unknown request."""
    arm = e.arm
    out = res.setdefault("case_hold", {})
    (fa, first), fr = common.frame_ctx(arm)
    arm.fill(fr, g.FRAME_STRIDE, 0, 0, g.FRAME_W, g.FRAME_H, 0)
    arm.text(fa, first, fr, g.FRAME_STRIDE, 20, 60, "no case hold")
    band = lambda: arm.wait_draw(arm.draw(fr, g.FRAME_STRIDE, 0, g.PANEL_Y, g.FRAME_W, g.FRAME_H), 60)
    n0 = len(arm.notices)
    out["in_case"] = arm.props("IN_CASE")["IN_CASE"]
    out["case_panel_status"] = arm.ctrl(0x15, b"\x01")["status"]
    band()
    out["read"] = arm.panel_read(g.PANEL_Y, 40)["match"]
    arm.set_panel(2)
    arm.wait_idle(10)
    out["standby"] = {"PANEL": arm.props("PANEL")["PANEL"], "panel_on": arm.hello()["panel_on"]}
    band()
    out["read_after_standby"] = arm.panel_read(g.PANEL_Y, 40)["match"]
    out["mic"] = arm.mic(True)["status"]
    arm.mic(False)
    out["notice6"] = [x for x in arm.notices[n0:] if x["kind"] == 6]
    assert out["in_case"] == 1 and out["case_panel_status"] == 2 and out["read"], out
    assert out["standby"] == {"PANEL": 2, "panel_on": False} and out["read_after_standby"], out
    assert out["mic"] == 0 and not out["notice6"], out


def test_case_hold(name):
    return run(name, CUR_MIC, case_hold_body, common.CASE_SETUP)


def case_blank_body(e, res):
    """0.5.32+: in the case the LEDs are off (A9, the image kept: DRAW and PANEL_READ work on it) for 2 s after each
    of the case's RF pulses (a re-check of a full arm), lit again after, not with the microphone on and not for VBUS
    alone; since 0.5.60 (no case hold, no CASE_PANEL) also through a top-up: the case's polls every ~650 ms keep them
    off, with no NOTICE 6, until 2 s after the last. Each reply is in the mailbox when the case reads it 75 ms after
    its write. The panel model ignores A9/A3: the arm's DIAG CASE counters show them."""
    arm = e.arm
    out = res.setdefault("case_blank", {})
    st = "sysbus.twim1.st25dv"
    vbus = lambda on: arm.tr.set_model(NPM, "VbusConnected", "true" if on else "false")

    def pulse():
        common.model_call(arm, "sysbus.gpio1.sw1", "Press")
        common.model_call(arm, "sysbus.gpio1.sw1", "Release")

    def poll():
        t0 = arm.tr.now()
        common.model_call(arm, st, "RfPutMailboxMessage", "130001006407")
        pulse()
        arm.wait(0.075)
        r = common.model_call(arm, st, "RfReadMailboxMessage")
        arm.wait(max(0.0, 0.65 - (arm.tr.now() - t0)))
        return r.startswith("13")

    def state():
        c = arm.case_stats()
        return {"blanked": c["blanked"], "blanks": c["blanks"]}

    def notice6(since, value):
        return any(x["kind"] == 6 and x["body"] == "%02x" % value for x in arm.notices[since:])

    arm.tr.set_model(NPM, "BatteryCurrentMilliAmps", "-30")   # charging: no one-off 0x80 reply (and its 1 s)
    common.link_signal(arm, 30)   # folded, not worn: IN_CASE stays 1 without VBUS
    arm.run_until(lambda: arm.peer_stats()["link_value"] == 30, 15, "link value 30 (folded)")
    pulse()   # Renode's GPIOTE loses the first pulse after a boot
    arm.wait(2.5)
    (fa, first), fr = common.frame_ctx(arm)
    arm.fill(fr, g.FRAME_STRIDE, 0, 0, g.FRAME_W, g.FRAME_H, 0)
    arm.text(fa, first, fr, g.FRAME_STRIDE, 20, 60, "blank")
    arm.wait_draw(arm.draw(fr, g.FRAME_STRIDE, 0, g.PANEL_Y, g.FRAME_W, g.FRAME_H), 60)
    arm.wait(1.5)
    out["vbus"] = state()
    vbus(False)
    arm.wait(1.5)
    out["recheck_reply"] = poll()
    out["recheck"] = state()
    arm.text(fa, first, fr, g.FRAME_STRIDE, 20, 120, "drawn while dark")
    arm.wait_draw(arm.draw(fr, g.FRAME_STRIDE, 0, g.PANEL_Y, g.FRAME_W, g.FRAME_H), 60)
    out["dark_read"] = arm.panel_read(g.PANEL_Y, g.FRAME_H)["match"]
    arm.wait(2.5)
    out["after_recheck"] = state()
    n0 = len(arm.notices)
    out["run_replies"] = [poll() for _ in range(4)]
    out["run"] = dict(state(), notice=notice6(n0, 1))
    out["run_gap_replies"] = [poll() for _ in range(2)]
    arm.wait(1.0)   # 1.6 s after the last pulse
    out["run_gap"] = state()
    n1 = len(arm.notices)
    arm.wait(7)
    out["after_run"] = dict(state(), notice=notice6(n1, 0))
    out["mic"] = arm.mic(True)["status"]
    poll()
    out["mic_poll"] = state()
    arm.mic(False)
    vbus(True)
    assert not out["vbus"]["blanked"] and out["recheck_reply"] and out["recheck"]["blanked"], out
    assert out["dark_read"] and not out["after_recheck"]["blanked"], out
    assert all(out["run_replies"]) and out["run"]["blanked"] and not out["run"]["notice"], out
    assert all(out["run_gap_replies"]) and out["run_gap"]["blanked"], out
    assert not out["after_run"]["blanked"] and not out["after_run"]["notice"], out
    assert out["mic"] == 0 and not out["mic_poll"]["blanked"], out


def test_case_blank(name):
    return run(name, CUR_MIC, case_blank_body, common.CASE_SETUP)


def test_in_case(name):
    if vnum(VER) >= (0, 5, 26):
        return run(name, CUR_MIC, in_case_state_body, common.CASE_SETUP)
    return run(name, APP, in_case_body, common.CASE_SETUP)


def test_mic_tool_draw(name):
    return run(name, CUR_MIC, mic_tool_draw_body, extra=SINE)


def bad_packets_body(e, res):
    """Malformed and refused host code: each ERROR byte for byte as the mirror (the same core) predicts it,
    work after it goes on; a packet outside the window gets ERROR 6."""
    arm = e.arm
    k = arm.k
    one, big = k(1), k(0x3FFFC)
    arm.write(0xD000, g.Code().ins("CALL", target=0xD000).b)
    arm.write(0xD100, g.Code().ins("JMP", target=0xD100).b)
    arm.write(0xD200, g.Code().ins("WAIT_EVENT", 1, one).b)
    arm.wait_idle(10)
    cases = {
        "cut off": bytes([0x50]),
        "unknown op": bytes([0x77]),
        "program-only JMP": g.Code().ins("JMP", target=0).b,
        "padding nibble": bytes([0x42, 0x31]),
        "WRITE past the arena": g.Code().write(0x3FFFC, bytes(8)).b,
        "bad PackBits": bytes([0x02]) + struct.pack("<I", 0x9000)[:3] + struct.pack("<HB", 9, 2) + bytes([0xF9, 1]),
        "level 16": g.Code().ins("FILL", k(0x9000), k(320), one, one, k(8), k(8), k(16)).b,
        "bad font": g.Code().ins("GLYPHS", k(0x9000), k(0x9000), k(320), one, one, k(2), k(15), k(0x9000), one).b,
        "LD32 past the arena": g.Code().ins("LD32", 3, big, one).b,
        "CALL depth 17": g.Code().ins("CALL", target=0xD000).b,
        "budget": g.Code().ins("CALL", target=0xD100).b,
        "WAIT_EVENT in called code": g.Code().ins("CALL", target=0xD200).b,
        "error then nothing more": g.Code().ins("ADD", 500, 500, one).ins("FILL", k(0x9000), k(320), one, one, k(8), k(8), k(99)).ins("ADD", 500, 500, one).b,
    }
    arm.wait_idle(10)
    got = []
    for label, code in cases.items():
        n_arm, n_mir = len(arm.errors), len(arm.mirror.events)
        arm.raw_packet(code)
        arm.wait_idle(30)
        arm.run_until(lambda: len(arm.errors) > n_arm, 10, f"error for {label}")
        want = [b for t, b in arm.mirror.events[n_mir:] if t == 0x02]
        mine = arm.events[-1][1] if arm.events[-1][0] == 0x02 else None
        got.append({"case": label, "arm": arm.errors[-1], "same_as_mirror": bool(want) and mine == want[0]})
    res["errors"] = got
    assert all(x["same_as_mirror"] for x in got), [x for x in got if not x["same_as_mirror"]]
    assert arm.arena_crc()["match"] and arm.mirror.word(500) == 1
    n = len(arm.errors)
    seq = (arm.exec_next + 40) & 0xFFFF
    arm.tr.write("G1_CMD", struct.pack("<H", seq) + bytes([0x33]))
    arm.run_until(lambda: len(arm.errors) > n, 10, "window error")
    res["window"] = arm.errors[-1]
    assert arm.errors[-1]["code"] == 6 and arm.errors[-1]["seq"] == seq, res["window"]
    res["frame_after"] = common.one_frame(arm, e.run / "png", "after", "after bad packets")
    assert res["frame_after"]["pixel_mismatches"] == 0


def test_bad_packets(name):
    return run(name, APP, bad_packets_body)


def no_panel_body(e, res):
    """A panel with a wrong ID: HELLO panel not ok; a DRAW does nothing and is no error (VM.md §3.5); MIC and
    PANEL_READ give 11; the arm stays up."""
    arm, h = e.arm, res["hello"]
    assert not h["panel_ok"], h
    fr = arm.alloc(320 * 20)
    arm.fill(fr, 320, 0, 0, 100, 20, 15)
    n = len(arm.errors)
    idx = arm.draw(fr, 320, 0, 0, 100, 20)
    res["draw"] = arm.wait_draw(idx)
    assert len(arm.errors) == n, arm.errors[n:]
    res["panel_read"] = arm.ctrl("PANEL_READ", struct.pack("<HH", 0, 1))["status"]
    assert res["panel_read"] == 11, res
    arm.wait(45)
    res["still_up"] = arm.hello()["boot_count"] == h["boot_count"]
    assert res["still_up"]


def test_no_panel(name):
    return run(name, APP, no_panel_body, "spim_a.jbd FaultWrongId true")


def starve_body(e, res):
    """Host code is not capped: a 30 s burst of per-pixel BLITs (600 x 400 each) must leave the PMIC and
    case-link threads their 1 s cadence (POWER samples go on, a case request is answered meanwhile) and
    must not trip the health check (same epoch afterwards). 0.4.10 rebooted here (health reason 3)."""
    arm = e.arm
    epoch, a = arm.epoch, arm.alloc(320 * 480)
    p0 = arm.power()["samples"]
    t0, n, reply = arm.tr.now(), 0, None
    while arm.tr.now() - t0 < 30:
        if len(arm.outq) < 4:
            arm.blit(a, 320, 1, 0, a, 320, 0, 0, 600, 400, mode=g.MAX)
            arm.flush()
            n += 1
        if reply is None and arm.tr.now() - t0 > 15:
            reply = common.case_request(arm, "130001006407")
        arm.pump()
    arm.wait_idle(600)
    p1 = arm.power()["samples"]
    h = arm.hello()
    res["starve"] = {"blits": n, "busy_s": round(arm.tr.now() - t0, 1), "power_samples": p1 - p0,
                     "case_reply": reply.hex() if reply else None, "same_epoch": h["epoch"] == epoch}
    assert h["epoch"] == epoch and p1 - p0 >= 0.8 * (arm.tr.now() - t0) and reply and reply[0] == 0x13, res["starve"]


def test_starve(name):
    return run(name, APP, starve_body, common.CASE_SETUP)


def sessions_body(e, res):
    """A new client session on an arm that kept its epoch (a second client, as one CLI command after another):
    it does not know the arena, so its constants must not rely on words an earlier session left. The grid
    from session 1, then session 2 switches the panel off and draws a band, compared with its own mirror."""
    arm = e.arm
    fr = g.grid_frame(arm, "RIGHT")
    arm.wait_draw(arm.draw(fr, g.PANEL_W // 2, 0, 0, g.PANEL_W, g.PANEL_H))
    arm2 = g.Arm(arm.tr, name="second", log=e.note)
    h = arm2.hello()
    assert h["epoch"] == arm.epoch and h["panel_on"], h
    arm2.set_panel(0)
    arm2.wait_idle(10)
    res["panel_after_off"] = arm2.hello()["panel_on"]
    b = arm2.alloc(320 * 20)
    arm2.fill(b, 320, 0, 0, 640, 20, 0)
    arm2.fill(b, 320, 0, 0, 200, 20, 9)
    arm2.wait_draw(arm2.draw(b, 320, 0, 100, 640, 20))
    res["band"] = arm2.panel_read(100, 20)
    assert not res["panel_after_off"] and res["band"]["match"] and not arm2.errors, res


def test_sessions(name):
    return run(name, APP, sessions_body)


# ---------------------------------------------------------------- the accelerometer (M3, 0.5.2)

LSM = "sysbus.twim2.lsm6dso"


def gravity(arm, v):
    """the head-frame acceleration v (g) as the LSM6DSO model reports it in its own axes: through the arm's
    head map (DIAG IMU, 0.5.8+; rows = the head axes in sensor axes, so sensor = map^T v), else identity"""
    m = getattr(arm, "head_map", None) or ((1, 0, 0), (0, 1, 0), (0, 0, 1))
    s = [sum(m[i][j] * v[i] for i in range(3)) for j in range(3)]
    for axis, x in zip("XYZ", s):
        arm.tr.set_model(LSM, f"Acceleration{axis}", f"{x:.6f}")


def head(pitch, roll=0.0):
    """the reading of an arm at head pitch/roll (degrees) in the head frame: x right, y up, z toward the wearer
    (0.5.26+; before, z forward)"""
    p, r = math.radians(pitch), math.radians(roll)
    z = -math.sin(p) if vnum(VER) >= (0, 5, 26) else math.sin(p)
    return (-math.sin(r) * math.cos(p), math.cos(r) * math.cos(p), z)


def spec_angles(v):
    """VM.md §2.5: pitch = atan2(-acc_z, acc_y) (z toward the wearer; atan2(acc_z, acc_y) with z forward before
    0.5.26), roll = atan2(-acc_x, acc_y), degrees, -90..90"""
    x, y, z = (round(c * 1000) for c in v)
    deg = lambda a, b: max(-90, min(90, round(math.degrees(math.atan2(a, b)))))
    return deg(-z if vnum(VER) >= (0, 5, 26) else z, y), deg(-x, y)


def tilt_program(arm, notify_motion=True):
    """review-v5.md §2.4's head-up rule: TILT out of [-90, 20] (head up) -> NOTIFY 1, range [15, 90]; out of
    that (head down) -> NOTIFY 0, range [-90, 20]; MOTION -> NOTIFY 0x4D; START sets the first range"""
    k, M = arm.k, g.M32
    pitch = 1
    start = (g.Code().ins("SYS_WRITE", k(0x24), k(-90 & M)).ins("SYS_WRITE", k(0x25), k(20))
             .ins("SYS_WRITE", k(0x23), k(100)).ins("END"))
    up = (g.Code().ins("SYS_WRITE", k(0x24), k(15)).ins("SYS_WRITE", k(0x25), k(90))
          .ins("NOTIFY", k(1), pitch).ins("END"))
    down = (g.Code().ins("SYS_WRITE", k(0x24), k(-90 & M)).ins("SYS_WRITE", k(0x25), k(20))
            .ins("NOTIFY", k(0), pitch).ins("END"))
    tilt = (g.Code().ins("SYS_READ", pitch, k(0x1B)).ins("BLTS", pitch, k(18), target=0xC200)
            .ins("JMP", target=0xC100))
    motion = g.Code().ins("NOTIFY", k(0x4D), k(0)).ins("END")
    for addr, c in ((0xC000, tilt), (0xC100, up), (0xC200, down), (0xC300, start), (0xC400, motion)):
        arm.write(addr, c.b)
    table = [0] * 16
    table[0], table[3] = 0xC300, 0xC000
    if notify_motion:
        table[2] = 0xC400
    arm.write(0x40, struct.pack("<16I", *table))
    arm.wait_idle(30)
    assert arm.vm_start(0x40) == 0


def imu_body(e, res):
    arm, h = e.arm, res["hello"]
    out = res.setdefault("imu", {})
    check_hello(h, VER)
    st = arm.imu_stats()
    arm.head_map = st.get("head_map")
    out["head_map"], out["factory_pitch_base"] = arm.head_map, st.get("factory_pitch_base")
    out["mic"] = arm.mic(True)["status"]
    assert out["mic"] == 12, out   # no capture before 0.5.8; from 0.5.8 an unencrypted link
    # on demand: nothing reads the sensor while no MOTION or TILT can fire
    gravity(arm, head(0))
    arm.wait(0.3)
    s0 = arm.imu_stats()
    arm.wait(2.0)
    s1 = arm.imu_stats()
    out["idle_stats"] = [s0, s1]
    if vnum(VER) >= (0, 5, 26):
        # in the case every sample is read, for the lift that ends IN_CASE (VM.md §2.5): one FIFO read a second
        assert s0["present"] and s1["streaming"] and s1["irqs"] - s0["irqs"] <= 6, out
    else:
        assert s0["present"] and not s1["streaming"] and s1["samples"] == s0["samples"], out
    # injected gravity -> ACC_* (mg), PITCH and ROLL within 1 degree of the spec's formula (of the reported ACC_*
    # where a measured map's 1 mg matters, e.g. pitch at roll 89)
    out["angles"] = []
    for p, r in ((0, 0), (10, 0), (20, 0), (45, 0), (80, 0), (-10, 0), (-45, 0), (-80, 0), (10, 30),
                 (-20, -45), (0, 89), (100, 0), (-100, 0)):
        v = head(p, r)
        gravity(arm, v)
        arm.wait(0.2)
        got = arm.imu()
        want = spec_angles(v)
        out["angles"].append({"set": (p, r), "want": want, "got": (got["PITCH"], got["ROLL"]),
                              "acc": (got["ACC_X"], got["ACC_Y"], got["ACC_Z"])})
        near = lambda w: abs(got["PITCH"] - w[0]) <= 1 and abs(got["ROLL"] - w[1]) <= 1
        assert near(want) or near(spec_angles([a / 1000 for a in out["angles"][-1]["acc"]])), out["angles"][-1]
        assert all(abs(a - round(c * 1000)) <= 2 for a, c in zip((got["ACC_X"], got["ACC_Y"], got["ACC_Z"]), v)), got
    # IMU_RATE: 0 = off (UNKNOWN), the four rates, anything else error 5
    k = arm.k
    gravity(arm, head(0))
    arm.add(g.Code().ins("SYS_WRITE", k(0x28), k(0)))
    arm.wait_idle(10)
    arm.wait(0.2)
    out["rate0"] = arm.imu()
    assert out["rate0"]["IMU_RATE"] == 0 and out["rate0"]["PITCH"] is None and out["rate0"]["ACC_Y"] is None, out
    for hz in (104, 52, 26, 13):
        arm.add(g.Code().ins("SYS_WRITE", k(0x28), k(hz)))
        arm.wait_idle(10)
        arm.wait(0.25)
        got = arm.imu()
        assert got["IMU_RATE"] == hz and got["PITCH"] == 0 and got["ACC_Y"] == 1000, (hz, got)
    n_err = len(arm.errors)
    arm.add(g.Code().ins("SYS_WRITE", k(0x28), k(7)))
    arm.wait_idle(10)
    arm.wait(0.2)
    out["rate7"] = arm.errors[n_err:]
    assert len(out["rate7"]) == 1 and out["rate7"][0]["code"] == 5 and out["rate7"][0]["detail"] == 7, out["rate7"]
    # a program with MOTION and TILT handlers, INPUT reported too: every sample is read now
    out["events_mask"] = arm.events_mask(1 << 2 | 1 << 3)
    assert out["events_mask"] == 0x000C, out
    tilt_program(arm)
    arm.wait(0.5)
    out["running_stats"] = arm.imu_stats()
    assert out["running_stats"]["streaming"], out
    if out["running_stats"]["irqs"] is not None:
        # 0.5.4+ (INT1): at rest the samples wait in the chip's FIFO; the CPU takes them at the executor's
        # 500 ms beat or the 1 s watermark, never once per sample
        s0 = arm.imu_stats()
        arm.wait(4.0)
        s1 = arm.imu_stats()
        out["rest_irqs_4s"] = s1["irqs"] - s0["irqs"]
        out["rest_samples_4s"] = s1["samples"] - s0["samples"]
        assert out["rest_irqs_4s"] <= 6 and out["rest_samples_4s"] <= 12, out
    # MOTION: a 150 mg step fires once (INPUT value = the change), a 50 mg step does not
    n_ev, n_in = len(arm.notifies), len(arm.inputs)
    gravity(arm, (0, 0.85, 0))
    arm.wait(0.5)
    gravity(arm, (0, 0.90, 0))
    arm.wait(0.5)
    out["motion_notifies"] = [x["source"] for x in arm.notifies[n_ev:]]
    out["motion_inputs"] = [(x["trigger"], x["value"]) for x in arm.inputs[n_in:]]
    assert out["motion_notifies"] == [2] and out["motion_inputs"] == [(2, 150)], out
    gravity(arm, head(0))
    arm.wait(0.5)
    # TILT: once per crossing of the head-up range, the program's hysteresis holds against small steps
    n_ev, n_in = len(arm.notifies), len(arm.inputs)
    seq = (25, 22, 17, 19, 10, 18, 19, 30, 5, 0)
    for p in seq:
        gravity(arm, head(p))
        arm.wait(0.4)
    out["tilt_notifies"] = [(x["a"], x["b"]) for x in arm.notifies[n_ev:] if x["source"] == 3]
    out["tilt_inputs"] = [x["value"] for x in arm.inputs[n_in:] if x["trigger"] == 3]
    assert out["tilt_notifies"] == [(1, 25), (0, 10), (1, 30), (0, 5)], out
    assert out["tilt_inputs"] == [25, 10, 30, 5], out
    if out["running_stats"]["irqs"] is not None:
        # a slow tilt (1 degree per 100 ms, below the wake-up slope) still fires within about a second
        n_ev = len(arm.notifies)
        t_cross = None
        for p in range(1, 26):
            gravity(arm, head(p))
            if p == 21:
                t_cross = arm.tr.now()
            arm.wait(0.1)
        arm.run_until(lambda: any(x["source"] == 3 for x in arm.notifies[n_ev:]), 3, "slow TILT")
        hit = next(x for x in arm.notifies[n_ev:] if x["source"] == 3)
        out["slow_tilt"] = {"notify": (hit["a"], hit["b"]), "after_crossing_s": round(hit["t"] - t_cross, 3)}
        assert hit["a"] == 1 and 21 <= hit["b"] <= 25 and out["slow_tilt"]["after_crossing_s"] < 1.4, out
        # a quick head-down (one step) is seen at once through the wake-up line
        n_ev = len(arm.notifies)
        t0 = arm.tr.now()
        gravity(arm, head(0))
        arm.run_until(lambda: any(x["source"] == 3 for x in arm.notifies[n_ev:]), 3, "quick TILT")
        hit = next(x for x in arm.notifies[n_ev:] if x["source"] == 3)
        out["quick_tilt"] = {"notify": (hit["a"], hit["b"]), "after_s": round(hit["t"] - t0, 3)}
        assert hit["a"] == 0 and out["quick_tilt"]["after_s"] < 0.3, out
        arm.wait(0.5)
    # without a program the EVENTS mask alone reports them (the range the program left: [-90, 20])
    assert arm.vm_stop() == 0
    arm.wait(0.3)
    n_in = len(arm.inputs)
    gravity(arm, head(35))
    arm.wait(0.5)
    out["input_no_program"] = [(x["trigger"], x["value"]) for x in arm.inputs[n_in:]]
    assert (3, 35) in out["input_no_program"] and any(t == 2 for t, _ in out["input_no_program"]), out
    # mask 0: nothing to fire, the sensor is read on demand again
    assert arm.events_mask(0) == 0
    arm.wait(0.5)
    s0 = arm.imu_stats()
    arm.wait(1.0)
    s1 = arm.imu_stats()
    out["stopped_stats"] = [s0, s1]
    if vnum(VER) >= (0, 5, 26):   # in the case every sample is read for the lift (VM.md §2.5)
        assert s1["streaming"] and s1["irqs"] - s0["irqs"] <= 6, out
    else:
        assert not s1["streaming"] and s1["samples"] == s0["samples"], out
    # RESET_SESSION: MOTION_MG 0 and the range [-90, 90] again, so even the mask does not need every sample
    arm.reset_session()
    assert arm.events_mask(0xFFFF) == triggers_for(VER)
    arm.wait(0.5)
    out["reset_stats"] = arm.imu_stats()
    out["reset_props"] = arm.props("MOTION_MG", "PITCH_LO", "PITCH_HI", "IMU_RATE")
    streams = vnum(VER) >= (0, 5, 26)   # in the case for the lift (VM.md §2.5)
    assert out["reset_stats"]["streaming"] == streams and out["reset_props"] == {"MOTION_MG": 0, "PITCH_LO": -90, "PITCH_HI": 90, "IMU_RATE": 13}, out
    assert arm.events_mask(0) == 0
    assert not arm.halts, arm.halts


def test_imu(name):
    """The LSM6DSO driver: on-demand readings, angles, IMU_RATE, MOTION and the head-up TILT rule with a
    program and through the EVENTS mask alone; HELLO (triggers 0xF06F, no microphone bit, MIC 12)."""
    return run(name, APP, imu_body)


BASE_SEED = BUILD / "seed_imu_base_-1500.bin"


def base_seed():
    """the default settings seed with stock's IMU pitch base (+0x0C) = -1500 (X 15 degrees up when level)"""
    if not BASE_SEED.exists():
        b = bytearray((WORK / "fw/seed/g1_extstore_seed_paired34.bin").read_bytes())
        struct.pack_into("<i", b, 0x0C, -1500)
        BASE_SEED.write_bytes(bytes(b))
    return f"sysbus.qspi.qspi_flash LoadSegment 0x130000 @{BASE_SEED}"


def imu_base_body(e, res, role, record=-1500, used=-1500):
    """the head map from stock's factory pitch base (provisional, imu.c): the record's value is read (a record
    with 0, stock's "no base", takes G1_IMU_BASE_DEFAULT, -1987), and the sensor vector of that level pose
    (X up by -base/100 degrees, Y up on the right and down on the left) reads PITCH 0, ROLL 0; 20 degrees up from
    there reads PITCH 20, 30 down -30; with imu.c's measured map (the record still read) its level pose instead"""
    arm = e.arm
    out = res.setdefault("imu_base", {})
    st = arm.imu_stats()
    out["factory_pitch_base"], out["head_map"] = st["factory_pitch_base"], st["head_map"]
    assert st["factory_pitch_base"] == record, st
    u = 1 if role == "R" else -1
    e0 = math.radians(-used / 100)
    prov = [[0, 0, -u], [math.sin(e0), u * math.cos(e0), 0], [-math.cos(e0), u * math.sin(e0), 0]]
    out["measured"] = any(abs(a - b) > 0.001 for ra, rb in zip(st["head_map"], prov) for a, b in zip(ra, rb))
    for up, want in ((0, 0), (20, 20), (-30, -30)):
        if out["measured"]:
            arm.head_map = st["head_map"]
            gravity(arm, head(up))
        else:
            a = math.radians(-used / 100 + up)
            arm.head_map = None
            gravity(arm, (math.sin(a), u * math.cos(a), 0.0))
        arm.wait(0.25)
        got = arm.imu()
        out[f"up{up}"] = got
        assert abs(got["PITCH"] - want) <= 1 and abs(got["ROLL"]) <= 1, (up, got)


def test_imu_base(name):
    return run(name, APP, lambda e, res: imu_base_body(e, res, "R"), extra=base_seed())


def test_imu_base_left(name):
    return run(name, APP, lambda e, res: imu_base_body(e, res, "L"), extra=common.LEFT_ARM + "\n" + base_seed())


def test_imu_base_default_left(name):
    return run(name, APP, lambda e, res: imu_base_body(e, res, "L", record=0, used=-1987), extra=common.LEFT_ARM)


def diag_body(e, res):
    arm = e.arm
    out = res.setdefault("diag", {})
    out["regs"] = {p: arm.regs(p) for p in range(6)}
    r = out["regs"]
    assert int(r[1]["P0.30"], 16) & 1 and int(r[1]["P0.24"], 16) & 1, r[1]
    assert int(r[4]["RESET.NETWORK.FORCEOFF"], 16) == 0 and int(r[5]["LDSWSTATUS"], 16) & 0x04, (r[4], r[5])
    out["pin_probe"] = arm.pin_probe()
    # the emulated LSM6DSO drives P1.15 / P1.04 as the arms' INT1 / INT2 (g1-board.resc)
    assert out["pin_probe"]["found"] == {"INT1": "P1.15", "INT2": "P1.04"}, out["pin_probe"]
    out["imu_after_probe"] = arm.imu()
    assert out["imu_after_probe"]["IMU_RATE"] == 13, out
    out["touch_off"] = arm.diag_set(1, 0)
    out["touch_on"] = arm.diag_set(1, 1)
    assert out["touch_off"]["touch_rail"] == 0 and out["touch_on"]["touch_rail"] == 1, out
    out["ldsw2_off"] = arm.diag_set(2, 0)
    out["imu_ldsw2_off"] = arm.imu()
    out["ldsw2_on"] = arm.diag_set(2, 1)
    arm.wait(0.3)
    out["imu_ldsw2_on"] = arm.imu()
    assert not out["ldsw2_off"]["imu"] and out["imu_ldsw2_off"]["PITCH"] is None, out
    assert out["ldsw2_on"]["imu"] and out["imu_ldsw2_on"]["ACC_X"] is not None, out
    if VER not in ("0.5.2", "0.5.3"):
        assert arm.ctrl("DIAG", bytes([5, 4, 1]))["status"] == 0 and arm.ctrl("DIAG", bytes([5, 4, 0]))["status"] == 0
    rep = arm.ctrl("DIAG", bytes([5, 3, 0]))   # the touch controller's mode, read only: its chip ID and REG_WST
    out["touch_mode"] = {"status": rep["status"], "id": rep["body"][:4].hex() if rep["body"] else None}
    assert out["touch_mode"] == {"status": 0, "id": "03322220"}, out
    # the net-off measurement: a reboot without Bluetooth, then a normal boot with the result; the net-core
    # fallback's counter of boots without Bluetooth is the same before and after (0: Bluetooth is up)
    def back(before):
        t0 = arm.tr.now()
        while True:
            common.wait_new_link(arm, before, timeout=240.0)
            if arm.tr.has("G1_CTRL"):
                break
            before = arm.losses()
        arm.hello()
        return round(arm.tr.now() - t0, 1)

    out["fails_before"] = arm.net_off_result()["net_fails_now"]
    assert out["fails_before"] == 0, out
    assert arm.ctrl("DIAG", struct.pack("<BHB", 6, 1200, 3))["status"] == 5   # over the 2400 s cap
    out["net_off"] = arm.net_off(20, 1).hex()
    out["back_after_s"] = back(arm.losses())
    out["result"] = arm.net_off_result()
    ph = out["result"]["phases"]
    assert out["result"]["seconds"] == 20 and len(ph) == 1 and ph[0]["samples"] >= 18, out["result"]
    assert out["result"]["net_fails_at_test"] == 0 and out["result"]["net_fails_now"] == 0, out["result"]
    assert out["back_after_s"] >= 35, out
    log = arm.read_log()
    out["log"] = [x for x in log.splitlines() if "net-off" in x][-3:]
    assert any("net-off phase 0" in x for x in out["log"]), out["log"]
    # a reset in the middle of a run (the emulator's machine reset) gives a normal boot with Bluetooth at
    # once: the request was taken at the start, no result, the counter untouched
    arm.net_off(30, 2)
    arm.tr.go_silent(22000)   # the bridge then steps every 10 ms while the arm is away (its link is gone)
    arm.wait(20)              # emulated time: the arm is in its test boot (10 s settle, then phase 0)
    arm.tr.reset_machine()
    out["back_after_reset_s"] = back(arm.losses())
    out["result_reset"] = arm.net_off_result()
    assert out["result_reset"]["phases"] == [] and out["result_reset"]["net_fails_now"] == 0, out
    assert out["back_after_reset_s"] < 40, out
    out["hello_after_reset"] = {k: arm.info[k] for k in ("boot_count", "unhealthy", "safe_level")}
    assert arm.info["safe_level"] == 0, out
    if vnum(VER) >= (0, 5, 32):
        idle_switches_body(arm, out, back)


def idle_switches_body(arm, out, back):
    """0.5.32+: DIAG SET 7 sets exactly the idle bisect's switches of a mask (idle.c): the QSPI deep power-down is
    refused (the flash model stays awake, an SMP image list works), each other switch on and off; the net-off
    measurement with a mask per phase, all switches off afterwards."""
    flash = "sysbus.qspi.qspi_flash"
    dpd = lambda: common.model_call(arm, flash, "get_DeepPowerDown")
    out["idle"] = {"mask_dpd": arm.idle_switches(1), "dpd": dpd()}   # refused: the flash stays awake
    out["idle"]["each"] = [arm.idle_switches(1 << b) for b in range(1, 7)] + [arm.idle_switches(0)]
    out["idle"]["slots"] = len(arm.smp.image_list())
    assert out["idle"]["mask_dpd"] == 0 and out["idle"]["dpd"] == "False", out["idle"]
    assert out["idle"]["slots"] >= 1 and out["idle"]["each"] == [2, 4, 8, 16, 32, 64, 0], out["idle"]
    arm.net_off(20, 2, [0, 0x7E])
    out["idle"]["back_after_s"] = back(arm.losses())
    r = arm.net_off_result([0, 0x7E])
    out["idle"]["net_off"] = [(p["name"], p["samples"]) for p in r["phases"]]
    log = arm.read_log().splitlines()
    out["idle"]["switch_log"] = [x.split("idle: ")[-1] for x in log if "idle switch" in x][-12:]
    assert len(r["phases"]) == 2 and all(p["samples"] >= 18 for p in r["phases"]), out["idle"]
    assert r["phases"][1]["name"] == "qspi-pulls+uart-tx+lows+panel-en+sensors+twim2", out["idle"]
    assert sum("off (0)" in x for x in out["idle"]["switch_log"]) >= 6, out["idle"]


def test_diag(name):
    """DIAG REGS, PIN_PROBE (no INT line in the emulator), SET (touch rail, LDSW2, touch mode refused without a
    controller), NET_OFF (a reboot into the measurement and back) and NET_OFF_RESULT."""
    return run(name, APP, diag_body)


PEER = pc.PEER
PEER_LEAD_US = 200000
BAND_Y, BAND_H = pc.BAND_Y, pc.BAND_H


JBD = "sysbus.spim_a.jbd"


def band_end_us(arm):
    """The emulated panel's last write into the band: when it ended, in the virtual time both machines share (µs)."""
    windows = [w.split(":") for w in common.model_call(arm, JBD, "get_PixelWindows").split(";") if w]
    return max(int(t) for t, y, n in windows if BAND_Y <= int(y) < BAND_Y + BAND_H)


def peer_link_up(arm, out):
    arm.ins("SYS_WRITE", g.PROPS["PEER_LINK"], 1)
    arm.flush()
    arm.wait_idle(30)
    arm.run_until(lambda: arm.props("PEER_LINK")["PEER_LINK"] == 2, 10, "PEER_LINK 2")
    out["link_up_s"] = round(arm.tr.now(), 2)


def pair_link_body(p, res):
    """The arm-to-arm link between both emulated arms, their stock net cores on one radio medium (VM.md §6.9):
    PEER_LINK 2 on both; values right to left and back (each a NOTIFY on both arms, INPUT events, latency from the
    arms' clocks against the shared virtual time); the clock offset the link learns; two programs drawing the same
    band by the recipe without the host (the DRAW times and the panel read-back); the left arm following the
    right arm's brightness (AUTO_BRIGHTNESS = 1, refused before PEER_LINK); PEER_LINK = 0 on the left (the right
    reads 1, the left's AUTO_BRIGHTNESS 0, PEER_SEND refused)."""
    n_msgs = int(os.environ.get("G1_PAIR_MSGS", "20"))
    out = {"R": {}, "L": {}}
    res["sides"] = out

    def common_setup(arm, side):
        o = out[side]
        h = arm.hello()
        o["triggers"] = f"0x{h['triggers']:04x}"
        assert h["role"] == side and h["triggers"] >> PEER & 1 and h["triggers"] >> 7 & 1, h
        try:
            arm.ins("SYS_WRITE", g.PROPS["PEER_SEND"], 5)
            raise AssertionError("PEER_SEND accepted while PEER_LINK is 0")
        except g.Refused:
            pass
        if side == "L":
            try:
                arm.ins("SYS_WRITE", g.PROPS["AUTO_BRIGHTNESS"], 1)
                raise AssertionError("AUTO_BRIGHTNESS accepted on the left before PEER_LINK")
            except g.Refused:
                pass
        o["band"] = pc.band(arm, 9 if side == "R" else 6)
        pc.programs(arm, side, o["band"][0], PEER_LEAD_US)
        assert arm.events_mask(1 << PEER) == 1 << PEER
        peer_link_up(arm, o)
        p.ev(side + "ready").set()
        arm.run_until(lambda: p.ev("Rready").is_set() and p.ev("Lready").is_set(), 30, "both ready")

    def right(arm):
        o = out["R"]
        common_setup(arm, "R")
        for i in range(n_msgs):
            arm.add(g.Code().ins("RAISE", arm.k(12)))
            arm.flush()
            want = (i + 1) + 0x10000
            try:
                arm.run_until(lambda: any(n["source"] == PEER and n["b"] == want for n in arm.notifies), 5, f"echo {i}")
            except TimeoutError:
                o["stats_at_failure"] = arm.peer_stats()
                o["props_at_failure"] = arm.props("PEER_LINK", "PEER_MSG", "PEER_T_US", "TIME_US")
                p.ev("dump").set()
                arm.run_until(lambda: p.ev("dumped").is_set(), 10, "left's dump")
                raise
            arm.wait(0.03 + 0.011 * (i % 5))
        p.ev("msgs").set()
        arm.run_until(lambda: p.ev("Lsync").is_set(), 30, "left sync program")
        for i in range(5):
            before = len([n for n in arm.notifies if n["source"] == 13])
            arm.add(g.Code().ins("RAISE", arm.k(13)))
            arm.flush()
            arm.run_until(lambda: len([n for n in arm.notifies if n["source"] == 13]) > before, 5, "sync draw")
            o.setdefault("band_end_us", []).append(band_end_us(arm))
            arm.wait(0.3)
        o["panel"] = arm.panel_read(BAND_Y, BAND_H)
        p.ev("sync").set()
        arm.run_until(lambda: p.ev("Lfollow").is_set(), 30, "left follows")
        for level in (30, 12):
            arm.ins("SYS_WRITE", g.PROPS["BRIGHTNESS"], level)
            arm.flush()
            arm.wait_idle(10)
            o.setdefault("level_set_t", []).append(arm.tr.now())
            p.ev(f"level{level}").set()
            arm.run_until(lambda: p.ev(f"Lgot{level}").is_set(), 10, f"left at {level}")
        arm.run_until(lambda: p.ev("Loff").is_set(), 30, "left link off")
        t0 = arm.tr.now()
        arm.run_until(lambda: arm.props("PEER_LINK")["PEER_LINK"] == 1, 5, "right sees the left gone")
        o["gone_after_s"] = round(arm.tr.now() - t0, 3)
        o["stats"] = arm.peer_stats()
        o["inputs"] = len([x for x in arm.inputs if x["trigger"] == PEER])

    def left(arm):
        o = out["L"]
        common_setup(arm, "L")
        arm.run_until(lambda: p.ev("msgs").is_set() or p.ev("dump").is_set(), 300, "the right's messages")
        if p.ev("dump").is_set():
            o["stats_at_failure"] = arm.peer_stats()
            o["props_at_failure"] = arm.props("PEER_LINK", "PEER_MSG", "PEER_T_US", "TIME_US")
            o["notifies"] = arm.notifies[-6:]
            p.ev("dumped").set()
            return
        o["sync_mark"] = len(arm.notifies)
        assert arm.vm_start(0x80) == 0
        p.ev("Lsync").set()
        for i in range(5):
            arm.run_until(lambda: len([n for n in arm.notifies[o["sync_mark"]:] if n["source"] == PEER]) > i, 10,
                          f"sync draw {i}")
            o.setdefault("band_end_us", []).append(band_end_us(arm))
        arm.run_until(lambda: p.ev("sync").is_set(), 60, "sync draws")
        o["panel"] = arm.panel_read(BAND_Y, BAND_H)
        arm.ins("SYS_WRITE", g.PROPS["AUTO_BRIGHTNESS"], 1)
        arm.flush()
        arm.wait_idle(10)
        assert arm.props("AUTO_BRIGHTNESS")["AUTO_BRIGHTNESS"] == 1
        p.ev("Lfollow").set()
        for level in (30, 12):
            arm.run_until(lambda: p.ev(f"level{level}").is_set(), 30, f"right at {level}")
            arm.run_until(lambda: arm.props("BRIGHTNESS")["BRIGHTNESS"] == level, 5, f"follow {level}")
            o.setdefault("level_got_t", []).append(arm.tr.now())
            p.ev(f"Lgot{level}").set()
        arm.ins("SYS_WRITE", g.PROPS["PEER_LINK"], 0)
        arm.flush()
        arm.wait_idle(10)
        v = arm.props("PEER_LINK", "AUTO_BRIGHTNESS")
        assert v == {"PEER_LINK": 0, "AUTO_BRIGHTNESS": 0}, v
        errs = len(arm.errors)
        arm.raw_packet(g.Code().ins("SYS_WRITE", arm.k(g.PROPS["PEER_SEND"]), arm.k(1)).b)
        arm.run_until(lambda: len(arm.errors) > errs, 5, "PEER_SEND refused by the arm")
        assert arm.errors[-1]["code"] == 5, arm.errors[-1]
        o["stats"] = arm.peer_stats()
        o["inputs"] = len([x for x in arm.inputs if x["trigger"] == PEER])
        p.ev("Loff").set()

    p.go(right, left)
    R, L = p.arms["R"], p.arms["L"]
    mark = out["L"]["sync_mark"]
    r_draw = [n for n in R.notifies if n["source"] == 13]
    l_draw = [n for n in L.notifies[mark:] if n["source"] == PEER]
    assert len(r_draw) == 5 and len(l_draw) == 5, (r_draw, l_draw)
    # each arm's clock against the shared virtual time, at the end of its band DRAW (TIME_US read just after it)
    rel_r = [v - n["a"] for v, n in zip(out["R"]["band_end_us"], r_draw)]
    rel_l = [v - n["a"] for v, n in zip(out["L"]["band_end_us"], l_draw)]
    offsets = sorted(r - l for r, l in zip(rel_r, rel_l))
    true_offset = offsets[2]   # left clock minus right clock
    res["offset_true_us"] = true_offset
    res["offset_true_spread_us"] = offsets[-1] - offsets[0]
    res["offset_link_us"] = out["L"]["stats"]["offset_us"]
    assert res["offset_link_us"] is not None and abs(res["offset_link_us"] - true_offset) < 1000, res
    sent = {n["b"]: n["a"] for n in R.notifies if n["source"] == 12}
    got_l, t_l = pc.arrivals(L.notifies[:mark])
    echo_r, _ = pc.arrivals(R.notifies)
    echo_r = {v - 0x10000: t for v, t in echo_r.items() if v > 0x10000}
    assert len(sent) == n_msgs and set(got_l) == set(sent) and set(echo_r) == set(sent), (sent, got_l, echo_r)
    one_way = sorted(((got_l[v] - true_offset) - sent[v]) / 1000 for v in sent)
    res["peer_t_us_err_us"] = max(abs((t_l[v] - true_offset) - sent[v]) for v in sent)
    assert res["peer_t_us_err_us"] < 1000, res
    rtt = sorted((echo_r[v] - sent[v]) / 1000 for v in sent)
    res["right_to_left_ms"] = {"min": round(one_way[0], 2), "median": round(one_way[len(one_way) // 2], 2),
                               "max": round(one_way[-1], 2)}
    res["round_trip_ms"] = {"min": round(rtt[0], 2), "median": round(rtt[len(rtt) // 2], 2), "max": round(rtt[-1], 2)}
    assert one_way[-1] < 120 and rtt[-1] < 300, res
    res["sync_target_diff_us"] = [(l["b"] - true_offset) - r["b"] for r, l in zip(r_draw, l_draw)]
    res["sync_panel_diff_us"] = [l - r for r, l in zip(out["R"]["band_end_us"], out["L"]["band_end_us"])]
    assert all(abs(d) < 2000 for d in res["sync_target_diff_us"] + res["sync_panel_diff_us"]), res
    for side in ("R", "L"):
        o = out[side]
        assert o["panel"]["crc"] == f"{o['band'][1]:08x}", (side, o["panel"], o["band"])
        s = o["stats"]
        res.setdefault("report_timeouts", {})[side] = s["report_timeouts"]
        assert s["inbox_drops"] == 0 and s["send_errors"] == 0, (side, s)
    assert out["R"]["inputs"] >= n_msgs and out["L"]["inputs"] >= n_msgs + 5, out
    res["follow_ms"] = [round((g_ - s_) * 1000, 1) for s_, g_ in zip(out["R"]["level_set_t"], out["L"]["level_got_t"])]
    assert out["R"]["gone_after_s"] < 1.0, out["R"]


ESB = "sysbus.esbslave"


def peer_frame(flags, seq=0, ack=0, value=0, t_write=0, frame_id=1, nonce=0x33, ack_nonce=0, offset=0):
    """A left arm's frame as the right arm reads it in an acknowledgement (peer.c)."""
    f = bytearray(32)
    f[0], f[1], f[2], f[3] = 0x47, flags, seq, ack
    struct.pack_into("<II", f, 4, value, t_write)
    f[17], f[19], f[24] = frame_id, nonce, ack_nonce
    struct.pack_into("<i", f, 20, offset)
    return bytes(f)


def peer_body(e, res):
    """The arm-to-arm link on the right arm against the virtual ESB slave (g1-fw's model, with the master's last
    payload readable and a settable acknowledgement payload): PEER_SEND refused while PEER_LINK is 0 (here and on
    the arm); with PEER_LINK = 1 frames every 100 ms (marker, the right arm's BRIGHTNESS, at once when it changes),
    PEER_SEND's value in them until acknowledged; a left frame from the slave delivers PEER_MSG once (a PEER INPUT
    event and a program's PEER handler), with PEER_T_US the sender's time through the offset it carries, PEER_LINK 2,
    and its acknowledgement ends the repeats; PEER_LINK = 0 and RESET_SESSION stop the frames; WORN out of the case
    from the net core's link value (the radio's signal strength: 60 worn, 30 not worn after 3 s), as WEAR events."""
    arm = e.arm
    out = res.setdefault("peer", {})
    k = arm.k
    last = lambda: bytes.fromhex(common.model_call(arm, ESB, "get_LastPayloadHex"))
    assert res["hello"]["triggers"] == TRIGGERS_M10, res["hello"]
    try:
        arm.ins("SYS_WRITE", g.PROPS["PEER_SEND"], 1)
        raise AssertionError("PEER_SEND accepted while PEER_LINK is 0")
    except g.Refused:
        pass
    errs = len(arm.errors)
    arm.raw_packet(g.Code().ins("SYS_WRITE", k(g.PROPS["PEER_SEND"]), k(1)).b)
    arm.run_until(lambda: len(arm.errors) > errs, 5, "the arm refuses PEER_SEND")
    assert arm.errors[-1]["code"] == 5, arm.errors[-1]
    handler = g.Code().ins("SYS_READ", 0, k(g.PROPS["PEER_MSG"])).ins("SYS_READ", 1, k(g.PROPS["PEER_T_US"]))
    program(arm, {0: g.Code().ins("END"), PEER: handler.ins("NOTIFY", 0, 1).ins("END")})
    assert arm.events_mask(1 << PEER | 1 << 7) == 1 << PEER | 1 << 7
    arm.ins("SYS_WRITE", g.PROPS["PEER_LINK"], 1)
    arm.flush()
    arm.wait_idle(10)
    arm.wait(0.6)
    st0 = arm.peer_stats()
    f = last()
    out["poll"] = {"frames_out": st0["frames_out"], "payload": f.hex(), "link": arm.props("PEER_LINK")["PEER_LINK"]}
    assert f[0] == 0x47 and f[1] & 2 and not f[1] & 1 and f[18] == 21, f.hex()
    assert out["poll"]["link"] == 1 and 3 <= st0["frames_out"] <= 10 and st0["report_timeouts"] == 0, out
    nonce = f[19]
    arm.ins("SYS_WRITE", g.PROPS["PEER_SEND"], 0xABCD1234)
    arm.flush()
    arm.wait_idle(10)
    arm.run_until(lambda: last()[1] & 1, 2, "the value in a frame")
    f = last()
    assert struct.unpack_from("<I", f, 4)[0] == 0xABCD1234 and f[2] == 1, f.hex()
    arm.ins("SYS_WRITE", g.PROPS["BRIGHTNESS"], 33)
    arm.flush()
    arm.wait_idle(10)
    t0 = arm.tr.now()
    arm.run_until(lambda: last()[18] == 33, 2, "the new brightness in a frame")
    out["level_in_frame_ms"] = round((arm.tr.now() - t0) * 1000, 1)
    left = peer_frame(1 | 8 | 16, seq=1, ack=1, value=0x55AA, t_write=1000000, frame_id=9, ack_nonce=nonce, offset=250)
    arm.tr.set_model(ESB, "ResponsePayloadHex", left.hex())
    arm.run_until(lambda: any(n["source"] == PEER for n in arm.notifies), 3, "the PEER handler")
    v = arm.props("PEER_MSG", "PEER_T_US", "PEER_LINK")
    out["delivered"] = v
    assert v == {"PEER_MSG": 0x55AA, "PEER_T_US": 1000000 - 250, "PEER_LINK": 2}, v
    arm.run_until(lambda: not last()[1] & 1, 2, "the acknowledged value no longer sent")
    f = last()
    assert f[1] & 8 and f[3] == 1 and f[24] == 0x33, f.hex()
    arm.wait(0.5)
    left2 = peer_frame(1 | 8, seq=2, ack=1, value=0x77, frame_id=10, ack_nonce=nonce)
    arm.tr.set_model(ESB, "ResponsePayloadHex", left2.hex())
    arm.run_until(lambda: any(n["source"] == PEER and n["a"] == 0x77 for n in arm.notifies), 3, "the second value")
    arm.wait(0.5)
    st = arm.peer_stats()
    out["stats"] = st
    ins = [x["value"] for x in arm.inputs if x["trigger"] == PEER]
    assert ins == [0x55AA, 0x77] and st["delivered"] == 2 and len([n for n in arm.notifies if n["source"] == PEER]) == 2, (ins, st)
    arm.tr.set_model(ESB, "ResponsePayloadHex", "")
    arm.ins("SYS_WRITE", g.PROPS["PEER_LINK"], 0)
    arm.flush()
    arm.wait_idle(10)
    n0 = arm.peer_stats()["frames_out"]
    arm.wait(0.5)
    assert arm.peer_stats()["frames_out"] == n0 and arm.props("PEER_LINK")["PEER_LINK"] == 0
    arm.ins("SYS_WRITE", g.PROPS["PEER_LINK"], 1)
    arm.flush()
    arm.wait_idle(10)
    assert arm.props("PEER_LINK")["PEER_LINK"] == 1
    arm.reset_session()
    assert arm.props("PEER_LINK")["PEER_LINK"] == 0
    # WORN out of the case from the net core's link value (the signal strength the radio reports), seen through
    # INPUT events: out of the case the arm obeys no write on the emulator's unencrypted link and drops it 10 s
    # after IN_CASE went to 0. The value is first lowered in the case, so raising it after leaving is a fresh report;
    # from 0.5.26 the arm leaves the case when lifted (IN_CASE is a state).
    radio = lambda v: common.link_signal(arm, v)
    radio(30)
    arm.run_until(lambda: arm.peer_stats()["link_value"] == 30, 10, "link value 30")
    if vnum(VER) >= (0, 5, 26):   # the arm's own classification (3 s after the value) before it leaves the case
        arm.run_until(lambda: arm.case_stats()["wear"] == 0, 10, "classified as not worn")
    assert arm.props("WORN")["WORN"] == 0 and arm.events_mask(1 << 6 | 1 << 7) == 1 << 6 | 1 << 7
    n0 = len(arm.inputs)
    since = lambda trig, value: any(x["trigger"] == trig and (x["value"] & 1) == value for x in arm.inputs[n0:])
    arm.tr.set_model(NPM, "VbusConnected", "false")
    if vnum(VER) >= (0, 5, 26):
        common.lift(arm)
    arm.run_until(lambda: since(6, 0), 10, "out of the case")
    t0 = arm.tr.now()
    radio(60)
    arm.run_until(lambda: since(7, 1), 5, "WORN 1 (signal 60)")
    out["worn_after_s"] = round(arm.tr.now() - t0, 1)
    t1 = arm.tr.now()
    radio(30)
    arm.run_until(lambda: since(7, 0), 9, "WORN 0 (signal 30)")
    out["removed_after_s"] = round(arm.tr.now() - t1, 1)
    out["wear_inputs"] = [x["value"] for x in arm.inputs[n0:] if x["trigger"] == 7]
    arm.tr.set_model(NPM, "VbusConnected", "true")
    assert out["wear_inputs"] == [1, 0] and out["worn_after_s"] < 3 and 4 < out["removed_after_s"] < 9, out


def panel_read_body(e, res):
    """PANEL_READ verifies itself (0.5.28+): a band drawn by host code reads back equal to the mirror; with the panel
    model inverting a few bytes of the next reads (a transient on the read line, the cache unchanged, as the left
    arm's first check after an install, while the case pulsed its power) it still reads back equal, the row read a
    third time and logged; what every read agrees on (all bytes inverted on every read, as a panel that holds other
    content) still reads back different."""
    arm = e.arm
    out = res.setdefault("panel_read", {})
    w = arm.alloc(320 * 40)
    arm.write_arena(w, bytes(range(256)) * 50)
    arm.add(g.Code().ins("DRAW", arm.k(4 * w), arm.k(320), arm.k(0), arm.k(0), arm.k(0), arm.k(100), arm.k(640), arm.k(40)))
    arm.wait_idle(20)
    out["clean"] = arm.panel_read(100, 40)
    arm.tr.set_model(JBD, "FaultReadNoise", "3")
    out["noisy"] = arm.panel_read(100, 40)
    arm.tr.set_model(JBD, "FaultReadNoise", "0")
    log = arm.read_log()
    out["log"] = [x for x in log.splitlines() if "panel read-back" in x][-2:]
    assert out["clean"]["match"] and out["noisy"]["match"] and out["log"], out
    # what every read agrees on is reported as it is: all bytes inverted on every read (as a panel that holds other
    # content) reads back different from the mirror
    arm.tr.set_model(JBD, "FaultReadNoise", "1000000")
    out["consistent"] = arm.panel_read(100, 40)
    arm.tr.set_model(JBD, "FaultReadNoise", "0")
    assert not out["consistent"]["match"] and out["consistent"]["status"] == 0, out


def video_body(e, res):
    """VIDEO (VM.md §3.7) at its worst, five 640 x 480 records back to back (tools/draw_bench.py --video): the arm
    stays up (the core calls now every 16 rows, which feeds the health beat), VM_STATUS is answered while they run,
    and the panel and the arena end as the mirror's."""
    import draw_bench
    arm = e.arm
    out = res.setdefault("video", {})
    recs = draw_bench.video_records(e.run)
    out["record_bytes"] = [len(r) for r in recs]
    boots, epoch = arm.info["boot_count"], arm.epoch
    last = draw_bench.video_send(arm, recs)
    status = []
    arm.run_until(lambda: draw_bench.VIDEO_MARK in arm.fences, 60, "the first VIDEO")
    while last not in arm.fences and len(status) < 400:
        status.append(arm.ctrl("VM_STATUS", timeout=5.0, tries=1)["status"])
    arm.run_until(lambda: last in arm.fences, 120, "the last VIDEO")
    out["video_ms"] = draw_bench.video_ms(arm, len(recs))
    out["vm_status_answered"] = len(status)
    h = arm.hello()
    out["same_boot"] = h["boot_count"] == boots and arm.epoch == epoch
    out["panel"] = arm.panel_read(0, 480)
    out["arena"] = arm.arena_crc()
    assert out["same_boot"] and status and all(x == 0 for x in status), out
    assert out["panel"]["match"] and out["arena"]["match"] and not arm.errors and not arm.halts, out

def test_panel_read(name):
    return run(name, CUR_MIC, panel_read_body)


def test_video(name):
    return run(name, CUR_MIC, video_body)

def test_peer(name):
    return run(name, CUR_MIC, peer_body)


def stream_programs(arm, side, n):
    """pair_stream's programs, the host out of the loop. Right: USER0 sends 1; its PEER handler takes the echo of the
    value in flight (value + 0x10000) and sends the next one, counts other arrivals in r5, and after n values NOTIFYs
    (n, r5). Left: its PEER handler counts in r6 and echoes; USER1 NOTIFYs (r6, the last value)."""
    k = arm.k
    msg, send = k(g.PROPS["PEER_MSG"]), k(g.PROPS["PEER_SEND"])
    base = 0xC000
    if side == "L":
        echo = (g.Code().ins("ADD", 6, 6, k(1)).ins("SYS_READ", 3, msg).ins("ADD", 4, 3, k(0x10000))
                .ins("SYS_WRITE", send, 4).ins("END"))
        report = g.Code().ins("NOTIFY", 6, 3).ins("END")
        return program(arm, {0: g.Code().ins("END"), PEER: echo, 13: report})

    def handler(mismatch, done):
        c = g.Code().ins("SYS_READ", 3, msg).ins("SUB", 4, 3, k(0x10000)).ins("BNE", 4, 1, target=mismatch)
        c.ins("ADD", 1, 1, k(1)).ins("BGEU", 1, k(n + 1), target=done).ins("SYS_WRITE", send, 1).ins("END")
        return c

    start = g.Code().ins("MOV", 1, k(1)).ins("SYS_WRITE", send, 1).ins("END")
    peer_at = base + len(start)
    size = len(handler(0, 0))
    mismatch = peer_at + size
    tail = g.Code().ins("ADD", 5, 5, k(1)).ins("END")
    done = mismatch + len(tail)
    finish = g.Code().ins("SUB", 7, 1, k(1)).ins("NOTIFY", 7, 5).ins("END")
    code = start.b + handler(mismatch, done).b + tail.b + finish.b
    arm.write(base, code + g.Code().ins("END").b)
    table = [0] * 16
    table[0], table[PEER], table[12] = base + len(code), peer_at, base
    arm.write(0x40, struct.pack("<16I", *table))
    arm.wait_idle(30)
    assert arm.vm_start(0x40) == 0


def pair_stream_body(p, res):
    """M10's acceptance in the emulator: n values (G1_PAIR_STREAM, default 200; 1000 for the acceptance) from the
    right arm's program to the left arm's and each echoed back, the next one sent when its echo arrived: every value
    arrives once on the left, every echo on the right, nothing else arrives, no error or dropped frame."""
    n = int(os.environ.get("G1_PAIR_STREAM", "200"))
    out = {"R": {}, "L": {}}
    res["sides"] = out

    def setup(arm, side):
        arm.hello()
        stream_programs(arm, side, n)
        peer_link_up(arm, out[side])
        p.ev(side + "ready").set()
        arm.run_until(lambda: p.ev("Rready").is_set() and p.ev("Lready").is_set(), 30, "both ready")

    def right(arm):
        setup(arm, "R")
        t0 = arm.tr.now()
        arm.add(g.Code().ins("RAISE", arm.k(12)))
        arm.flush()
        arm.run_until(lambda: any(x["source"] == PEER for x in arm.notifies), n * 0.5 + 30, "the last echo")
        out["R"]["seconds"] = round(arm.tr.now() - t0, 1)
        out["R"]["result"] = next(x for x in arm.notifies if x["source"] == PEER)
        out["R"]["stats"] = arm.peer_stats()
        p.ev("done").set()

    def left(arm):
        setup(arm, "L")
        arm.run_until(lambda: p.ev("done").is_set(), n * 0.5 + 60, "the right's last echo")
        arm.add(g.Code().ins("RAISE", arm.k(13)))
        arm.flush()
        arm.run_until(lambda: any(x["source"] == 13 for x in arm.notifies), 10, "the left's count")
        out["L"]["result"] = next(x for x in arm.notifies if x["source"] == 13)
        out["L"]["stats"] = arm.peer_stats()

    p.go(right, left)
    r, l = out["R"]["result"], out["L"]["result"]
    res["exchanges"] = r["a"]
    res["round_trip_ms_avg"] = round(out["R"]["seconds"] * 1000 / n, 1)
    assert r["a"] == n and r["b"] == 0 and l["a"] == n and l["b"] == n, (r, l)
    # report timeouts are recorded only: the two emulated net cores' ESB stalls now and then (emulator.md)
    res["report_timeouts"] = {side: out[side]["stats"]["report_timeouts"] for side in ("R", "L")}
    for side in ("R", "L"):
        s = out[side]["stats"]
        assert s["inbox_drops"] == 0 and s["send_errors"] == 0, (side, s)


def test_pair_stream(name):
    return run_pair(name, CUR_MIC, pair_stream_body)


def run_pair(name, app, body, tries=2):
    """Both arms in one emulation; once more if the two emulated net cores never start their ESB exchanges (a
    known emulator flake: PEER_LINK never reaches 2)."""
    for attempt in range(tries):
        p = common.Pair(name, app)
        res = {"test": name, "attempt": attempt + 1}
        t0 = time.time()
        try:
            body(p, res)
            res.setdefault("pass", True)
        except Exception as ex:
            res.update(error=f"{type(ex).__name__}: {str(ex)[:3000]}", trace=traceback.format_exc()[-1500:],
                       **{"pass": False})
        res["wall_s"] = round(time.time() - t0)
        for side, arm in p.arms.items():
            res.setdefault("virtual_s", round(arm.tr.now(), 1))
            res[f"{side}_errors"] = arm.errors[-10:]
        res["host_log"] = p.lines[-30:]
        p.stop()
        (p.run / "result.json").write_text(json.dumps(res, indent=1, default=lambda o: o.hex() if isinstance(o, bytes) else str(o)))
        if res["pass"] or "PEER_LINK 2 not reached" not in res.get("error", ""):
            return res
    return res


def test_pair_link(name):
    return run_pair(name, CUR_MIC, pair_link_body)


# ---------------------------------------------------------------- the asset store (VM.md §6.10, 0.5.56)

FLASH_MODEL = "sysbus.qspi.qspi_flash"
STORE, STORE_SIZE = 0x140000, 0xEC0000       # firmware/pm_static.yml g1_assets
OUTSIDE = ((0x0, STORE), (STORE + STORE_SIZE, 0x2000000))
DATA_SECTORS = STORE_SIZE // 4096 - 2


def qspi_image(e, label):
    """The QSPI NOR as the flash model holds it now (a monitor save, removed after reading)."""
    path = e.run / f"qspi-{label}.bin"
    common.model_call(e.arm, FLASH_MODEL, "SaveContents", str(path), timeout=60.0)
    data = path.read_bytes()
    path.unlink()
    assert len(data) == 0x2000000, len(data)
    return data


def store_directory(img):
    """The directory the arm chooses from a flash image: the valid copy with the higher seq, (seq, IDs)."""
    best = (0, [])
    for c in (0, 1):
        b = img[STORE + 4096 * c:STORE + 4096 * (c + 1)]
        magic, fmt, count, seq, size = struct.unpack_from("<IHHII", b)
        if magic != 0x53414B53 or fmt != 1 or count > 120 or size != STORE_SIZE:
            continue
        n = 32 + 32 * count
        if zlib.crc32(b[:n]) != struct.unpack_from("<I", b, n)[0]:
            continue
        if seq > best[0] or not best[1] and seq >= best[0]:
            best = (seq, [b[32 + 32 * i:48 + 32 * i].hex() for i in range(count)])
    return best


def chip_state(arm, label):
    """The store sends no instruction that changes the chip's addressing: 3-byte mode, bank register 0."""
    four = common.model_call(arm, FLASH_MODEL, "get_FourByteMode")
    bank = common.model_call(arm, FLASH_MODEL, "get_ExtendedAddressRegister")
    assert four == "False" and bank == "0", f"{label}: 4-byte mode {four}, bank register {bank}"


def raw_put(arm, aid, addr, size, typ=4):
    rep = arm.ctrl("ASSET_PUT", bytes(aid) + struct.pack("<IIB", addr, size, typ))
    return rep["status"], rep["body"][:1]


def rebooted(arm, before, label):
    arm.run_until(lambda: arm.losses() > before, 120, label)
    arm.setup(120)
    h = arm.hello()
    arm.health_ok()
    return h


def fresh_boot(arm, label):
    """A clean REBOOT before a long phase (the fill and the wipe take minutes each): two runs of this test had the
    emulated arm reset by itself at 511 s of uptime, with no fault or health record (RTC1's 24-bit counter wraps
    at 512 s), and an idle 0.5.54 arm once at 165 s, so each phase starts from a fresh boot."""
    before = arm.losses()
    arm.reboot(0)
    rebooted(arm, before, f"the reboot before {label}")


def assets_body(e, res):
    arm = e.arm
    rng = random.Random(0xA55E7)
    flash0 = qspi_image(e, "start")
    fp0 = arm.fingerprint()
    res["fingerprint"] = fp0

    def unchanged(label):
        img = qspi_image(e, label)
        for a, b in OUTSIDE:
            assert img[a:b] == flash0[a:b], f"{label}: flash outside the store changed in 0x{a:x}..0x{b:x}"
        assert arm.fingerprint() == fp0, label
        chip_state(arm, label)
        return img

    info = arm.asset_info()
    assert info["flags"] & 1 and info["entries"] == 0 and info["seq"] == 0, info
    assert info["free"] == DATA_SECTORS * 4096 == info["largest"], info
    assert arm.asset_list() == []
    chip_state(arm, "start")

    # a 64 KB blob: stored, listed, loaded into another range
    blob = rng.randbytes(65536)
    r, aid = arm.asset_put(blob, 4, addr=0x10000)
    assert r == 0, r
    info = arm.asset_info()
    assert info["state"] == 4 and info["entries"] == 1 and info["data"] == 65536, info
    lst = arm.asset_list()
    assert lst[0]["id"] == aid.hex() and lst[0]["size"] == 65536 and lst[0]["crc"] == zlib.crc32(blob), lst
    res["put_64k"] = arm.asset_stats()
    arm.asset_load(aid, 0x28000, 0, 65536, data=blob)
    arm.wait_idle(60)
    c = arm.arena_crc(0x28000, 65536)
    assert c["match"] and int(c["crc"], 16) == zlib.crc32(blob), c
    erased = arm.asset_stats()["erased"]
    assert raw_put(arm, aid, 0x10000, 65536) == (0, b"\x01")
    assert arm.asset_stats()["erased"] == erased
    assert raw_put(arm, aid, 0x10000, 4096)[0] == 5   # the ID with another size

    # a reboot: still listed, the first load checks the body's crc
    before = arm.losses()
    arm.reboot(0)
    rebooted(arm, before, "reboot")
    assert [x["id"] for x in arm.asset_list()] == [aid.hex()]
    arm.asset_load(aid, 0x28000, 1000, 5000, data=blob)
    arm.wait_idle(60)
    c = arm.arena_crc(0x28000, 5000)
    assert c["match"] and int(c["crc"], 16) == zlib.crc32(blob[1000:6000]), c
    res["first_load_check_us"] = arm.asset_stats()["check_us"]
    assert res["first_load_check_us"] > 0

    # a wrong ID, and a staged range changed during the job: state 5, nothing new listed
    start = len(arm.notices)
    assert raw_put(arm, bytes(16), 0x10000, 8192) == (0, b"\x00")
    assert arm.asset_wait(1, start)[0] == 13
    assert arm.asset_info()["state"] == 5 and len(arm.asset_list()) == 1
    blob2 = rng.randbytes(131072)
    arm.write(0x10000, blob2)
    arm.flush()
    arm.wait_idle(120)
    start = len(arm.notices)
    assert raw_put(arm, hashlib.sha256(blob2).digest()[:16], 0x10000, len(blob2)) == (0, b"\x00")
    arm.write(0x10000 + 120000, b"changed")
    arm.flush()
    assert arm.asset_wait(1, start)[0] == 13
    assert len(arm.asset_list()) == 1

    # a program pages through the asset (USER0: ASSET_LOAD 256 bytes, NOTIFY off, off += 256)
    w = 0x30000 // 4
    arm.write(0x30100, aid)
    arm.write(0x30000, struct.pack("<5I", 0x30100, 0x31000, 0, 256, 256))
    code = g.Code().ins("ASSET_LOAD", w, w + 1, w + 2, w + 3).ins("NOTIFY", w + 2, w + 3).ins("ADD", w + 2, w + 2, w + 4)
    code.ins("END")
    program(arm, {12: code})
    for k in range(3):
        n0 = len(arm.notifies)
        arm.ins("RAISE", 12)
        arm.flush()
        arm.run_until(lambda: len(arm.notifies) > n0, 30, "the program's NOTIFY")
        assert arm.notifies[-1]["a"] == 256 * k, arm.notifies[-1]
        arm.wait_idle(30)
        rep = arm.ctrl_ok("ARENA_CRC", struct.pack("<II", 0x31000, 256))
        assert struct.unpack_from("<I", rep)[0] == zlib.crc32(blob[256 * k:256 * (k + 1)]), k
    arm.vm_stop()

    # delete and format
    assert arm.asset_delete(aid) == 0 and arm.asset_list() == []
    r, aid = arm.asset_put(rng.randbytes(5000), 4, addr=0x10000)
    assert r == 0 and len(arm.asset_list()) == 1
    assert arm.asset_format() == 0 and arm.asset_list() == [] and arm.asset_info()["seq"] == 0
    unchanged("format")

    # power fails at each checkpoint of a put: after the reset the directory is the old one, never a mix
    r, kept = arm.asset_put(rng.randbytes(3000), 4, addr=0x10000)
    assert r == 0
    res["checkpoints"] = {}
    for k in (1, 2, 3, 4):
        old = [x["id"] for x in arm.asset_list()]
        body = rng.randbytes(9000)
        arm.write(0x10000, body)
        arm.flush()
        arm.wait_idle(60)
        arm.asset_reset_at(k)
        before = arm.losses()
        assert raw_put(arm, hashlib.sha256(body).digest()[:16], 0x10000, len(body)) == (0, b"\x00")
        rebooted(arm, before, f"the reset at checkpoint {k}")
        now = [x["id"] for x in arm.asset_list()]
        img = qspi_image(e, f"checkpoint{k}")
        res["checkpoints"][k] = {"listed": len(now), "directory_seq": store_directory(img)[0]}
        assert now == old == store_directory(img)[1], (k, now, old)
        r, aid = arm.asset_put(body, 4, addr=0x10000)
        assert r == 0 and len(arm.asset_list()) == len(old) + 1, k
    unchanged("checkpoints")

    # safe mode: ASSET_INFO and ASSET_LIST work, the rest answers 7
    before = arm.losses()
    arm.reboot(2)
    h = rebooted(arm, before, "the reboot into safe mode")
    assert h["safe_mode"], h
    assert arm.ctrl("ASSET_INFO")["status"] == 0 and arm.ctrl("ASSET_LIST", b"\x00")["status"] == 0
    for op, body in (("ASSET_PUT", bytes(16) + struct.pack("<IIB", 0, 16, 4)), ("ASSET_DELETE", aid),
                     ("ASSET_FORMAT", struct.pack("<I", 0x534B4153)), ("FLASH_CRC", struct.pack("<II", 0, 16))):
        assert arm.ctrl(op, body)["status"] == 7, op
    before = arm.losses()
    arm.reboot(1)
    h = rebooted(arm, before, "the reboot out of safe mode")
    assert not h["safe_mode"], h

    # the acceptance run's single-stepped put: the fingerprints after every flash call (design item 28)
    r, _ = arm.asset_put_stepped(rng.randbytes(8192), 4, addr=0x10000, log=e.note)
    assert r == 0, r
    res["stepped_flash_calls"] = arm.asset_stats()["steps"]
    assert res["stepped_flash_calls"] == 6, res["stepped_flash_calls"]

    # fill until "no space" (13) with the free count exact: 62-sector assets at 0x1000 (the arena below the
    # client's pool), zeros but for a counter
    big = 62 * 4096
    assert arm.asset_format() == 0
    fresh_boot(arm, "the fill")   # the arena is zero again
    n, free = 0, DATA_SECTORS * 4096
    while free >= big:
        tag = struct.pack("<I", n) * 4
        arm.write(0x1000, tag)
        arm.flush()
        arm.wait_idle(60)
        data = tag + bytes(big - 16)
        start = len(arm.notices)
        assert raw_put(arm, hashlib.sha256(data).digest()[:16], 0x1000, big) == (0, b"\x00"), n
        assert arm.asset_wait(1, start, 120)[0] == 0, n
        n += 1
        free -= big
        assert arm.asset_info()["free"] == free, (n, free)
    arm.write(0x1000, struct.pack("<I", n) * 4)
    arm.flush()
    arm.wait_idle(60)
    assert raw_put(arm, bytes(16), 0x1000, big)[0] == 13          # no space
    last = struct.pack("<I", n) * 4 + bytes(free - 16)
    start = len(arm.notices)
    assert raw_put(arm, hashlib.sha256(last).digest()[:16], 0x1000, free) == (0, b"\x00")
    assert arm.asset_wait(1, start, 120)[0] == 0
    info = arm.asset_info()
    res["fill"] = {"assets": n + 1, "info": info}
    assert info["free"] == 0 and info["entries"] == n + 1 and info["largest"] == 0, info
    assert raw_put(arm, bytes(16), 0x1000, 4096)[0] == 13
    img = unchanged("fill")
    assert store_directory(img)[1] == [x["id"] for x in arm.asset_list()]
    assert img[STORE + STORE_SIZE - 4096:STORE + STORE_SIZE] != b"\xff" * 4096   # the last sector holds data

    # the wipe before a return to stock: the whole store erased, nothing else touched
    fresh_boot(arm, "the wipe")
    assert arm.asset_format(wipe=True) == 0
    res["wipe"] = arm.asset_stats()
    img = unchanged("wipe")
    assert img[STORE:STORE + STORE_SIZE] == b"\xff" * STORE_SIZE
    assert arm.asset_list() == []


def assets_logged(e, res):
    """assets_body, and on a failure the arm's log (after a reconnection if the arm reset)."""
    losses = e.arm.losses()
    try:
        assets_body(e, res)
    except Exception:
        for attempt in range(2):
            try:
                if attempt or e.arm.losses() > losses:
                    e.arm.run_until(lambda: e.arm.tr.connected, 120, "a link")
                    e.arm.setup(120)
                    e.arm.hello()
                res["arm_log"] = e.arm.read_log()[-3000:]
                break
            except Exception as ex:
                res["arm_log"] = f"unreadable: {ex}"
        raise


def test_assets(name):
    """The asset store (VM.md §6.10, docs/design-flash-assets.md item 23): put, list, load, reboot, the same put,
    wrong ID, staged range changed, a program paging, delete, format, power-fail checkpoints, safe mode, a
    single-stepped put, a fill until no space with exact counts, the wipe; the flash outside the store, the
    protected regions' crc and the chip's addressing state unchanged throughout."""
    return run(name, CUR, assets_logged)


def faceclaw_body(e, res):
    """Faceclaw's display path (VM.md §6.11): tools/g1fc.py's acceptance, by PANEL_READ."""
    import g1fc
    g1fc.acceptance(e.arm, res, emulated=True)


def test_faceclaw(name):
    # The test build: the microphone works on the emulator's unencrypted link.
    return run(name, CUR_MIC, faceclaw_body)


TESTS = {"smoke": test_smoke, "vectors": test_vectors, "chain": test_chain, "chain_right": test_chain_right,
         "power": test_power, "unattended": test_unattended, "examples": test_examples, "vm": test_vm, "mic": test_mic,
         "mic_capture": test_mic_capture, "mic_lease": test_mic_lease, "mic_popup": test_mic_popup, "imu_base": test_imu_base, "imu_base_left": test_imu_base_left,
         "mic_tool": test_mic_tool, "mic_tool_draw": test_mic_tool_draw, "draw_bench": test_draw_bench, "panel_standby": test_panel_standby, "imu_base_default_left": test_imu_base_default_left,
         "panel_prop": test_panel_prop, "in_case": test_in_case, "case_hold": test_case_hold, "case_blank": test_case_blank, "mic_popup_standby": test_mic_popup_standby,
         "touch": test_touch, "restore": test_restore, "light": test_light, "gyro": test_gyro,
         "bad_packets": test_bad_packets, "no_panel": test_no_panel, "starve": test_starve,
         "sessions": test_sessions, "imu": test_imu, "diag": test_diag, "pair_link": test_pair_link,
         "peer": test_peer, "pair_stream": test_pair_stream,
         "panel_read": test_panel_read, "video": test_video, "panel_guard": test_panel_guard,
         "assets": test_assets, "faceclaw": test_faceclaw}
for _t in ("crash", "hang", "thread_hang", "safe_mode", "lost_packet", "reset_mid_frame", "security", "pmic_watchdog",
           "i2c_errors", "no_pmic"):
    TESTS[_t] = getattr(common, "test_" + _t)
TESTS["disconnect"] = common.test_disconnect_mid_upload


def main():
    args = sys.argv[1:]
    jobs = 4
    if "--jobs" in args:
        i = args.index("--jobs")
        jobs = int(args[i + 1])
        del args[i:i + 2]
    work = []
    for a in args:
        if ":" in a:
            test, count = a.split(":")
            work += [(test, f"{PREFIX}-{test}-{k:02d}") for k in range(1, int(count) + 1)]
        else:
            work.append((a, f"{PREFIX}-{a}"))
    t0 = time.time()
    with ThreadPoolExecutor(jobs) as ex:
        results = list(ex.map(lambda w: TESTS[w[0]](w[1]), work))
    for (test, run_name), r in zip(work, results):
        print(f"{run_name:28s} {'PASS' if r['pass'] else 'FAIL'}  {r.get('error', '')}")
    print(f"{sum(r['pass'] for r in results)}/{len(results)} passed in {time.time() - t0:.0f} s real")
    sys.exit(0 if all(r["pass"] for r in results) else 1)


if __name__ == "__main__":
    main()
