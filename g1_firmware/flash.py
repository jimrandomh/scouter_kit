#!/usr/bin/env python3
"""Installs a firmware image on one G1 arm over Bluetooth through MCUmgr SMP, the update service of both
Even's firmware and g1v0 (the upload and confirm of ../tools/g1host.py smp-upload / smp-commit). Read
README.md first: installing over Even's firmware is a point of no return.

  flash.py --ble ADDR [--stock] list               the arm's image slots
  flash.py --ble ADDR [--stock] install IMAGE      upload to slot 1, check it, ask, confirm, reset, wait for the arm
  flash.py --ble ADDR restore [VERSION]            the same with Even's app image (scripts/fetch_stock.sh VERSION),
                                                   after erasing g1v0's asset store, where stock's fonts and
                                                   pictures were: stock then lacks them (they do not come back)

--stock: the arm runs Even's firmware (the link is kept alive with its heartbeat, no g1v0 checks);
--side L|R finds the arm by its name after an install that changes firmware (from before 0.5.66, also its address).
--emu SOCKET instead of --ble: an emulated arm (scripts/bridge_start.sh). --yes: do not ask.

-c CONN instead of --ble: both arms, the left one first. CONN is 'g1://local?left=ADDR&right=ADDR' (quote it) or the
path of a file holding that. Both arms must be advertising before either is touched, each install asks on its own, and
a failure stops the run. --side L|R: only that arm. On macOS, which hides MACs, the arms are found by their names.
"""
import argparse
import asyncio
import hashlib
import os
import re
import struct
import subprocess
import sys
import time
import urllib.parse
from pathlib import Path

FW = Path(__file__).resolve().parent
sys.path.insert(0, str(FW.parent / "tools"))
import g1host as g  # noqa: E402

SIDES = {"L": "left", "R": "right"}


def image_version(img):
    if img[:4] != bytes.fromhex("3db8f396"):
        sys.exit("not an MCUboot image")
    major, minor, rev = struct.unpack_from("<BBH", img, 20)
    return f"{major}.{minor}.{rev}"


def stock_image(version):
    want = {}
    for line in open(FW / "stock_images.txt"):
        t = line.split()
        if t and not t[0].startswith("#") and t[0] == version:
            want[t[1]] = t[2]
    if not want:
        sys.exit(f"unknown stock version {version} (stock_images.txt)")
    work = Path(os.environ.get("G1_WORK", FW / "work"))
    path = work / "firmware" / version / "app_update.bin"
    if not path.exists():
        sys.exit(f"{path} missing: run scripts/fetch_stock.sh {version}")
    img = path.read_bytes()
    if hashlib.sha256(img).hexdigest() != want["app_update.bin"]:
        sys.exit(f"{path}: SHA-256 does not match stock_images.txt")
    return img


def release(address):
    """BlueZ reconnects a bonded arm by itself and then holds the link (the arm stops advertising)."""
    try:
        subprocess.run(["bluetoothctl", "disconnect", address], capture_output=True, timeout=15)
    except (OSError, subprocess.SubprocessError):
        return
    time.sleep(1)


def connect(args, address=None, stock=False):
    if not args.emu:
        release(address or args.ble)
    tr = g.EmuTransport(args.emu) if args.emu else g.BleTransport(address or args.ble)
    arm = g.Arm(tr, name=args.side or "arm")
    arm.setup()
    arm.hb_every = 8.0 if stock else 0.0
    return arm


def show_images(arm):
    for e in g.smp_images(arm.smp):
        print(f"  image {e['image']} slot {e['slot']}: {e['version']} {e['hash'][:16]}… {' '.join(e['flags'])}")


def hello(args, arm):
    h = arm.hello()
    print(f"  g1v0 {h['version']} ({h['role']}), in the case {h['in_case']}, encrypted {h['encrypted']}, "
          f"safe level {h['safe_level']}, panel ok {h['panel_ok']}")
    if args.side not in (None, h["role"]):
        sys.exit(f"{args.ble or args.emu} is the {SIDES[h['role']]} arm, not the {SIDES[args.side]} one: stopped")
    args.side = h["role"]
    return h


def has_stock_address(version):
    """g1v0 from 0.5.66 on has the address Even's firmware has; before, one from the chip's device ID."""
    return tuple(int(n) for n in version.split(".")) >= (0, 5, 66)


def ask(args, text):
    if args.yes:
        return
    if input(f"{text}\nType yes to go on: ").strip() != "yes":
        sys.exit("stopped: nothing was installed on this arm (slot 1 holds the upload, unmarked)")


def scan(args, stock, timeout=240.0, skip=""):
    """After an install that changes the address: the arm's advertising by its name and side, not at skip (the old
    firmware may advertise there until its reset)."""
    from bleak import BleakScanner
    found = []

    def cb(dev, adv):
        name = adv.local_name or ""
        if stock:
            ok = name.startswith("Even G1_") and f"_{args.side}_" in name
        else:
            ok = re.fullmatch(rf"G1CFW_{args.side}_[0-9A-F]{{6}}|G1v0S?_{args.side}", name)
        if ok and dev.address.upper() != skip.upper():
            found.append((dev.address, name))

    async def run():
        async with BleakScanner(cb, scanning_mode="active"):
            t0 = time.monotonic()
            while not found and time.monotonic() - t0 < timeout:
                await asyncio.sleep(0.5)
    asyncio.run(run())
    return found[0] if found else None


def parse_connection(text):
    """g1://local?left=ADDR&right=ADDR, or the path of a file holding that: the addresses by side, left first."""
    if "://" not in text:
        try:
            text = Path(text).expanduser().read_text()
        except OSError as ex:
            sys.exit(f"--connection: {ex}")
    url = urllib.parse.urlparse(text.strip())
    query = urllib.parse.parse_qs(url.query)
    if (url.scheme, url.netloc) != ("g1", "local") or not {"left", "right"} <= query.keys():
        sys.exit(f"--connection {text.strip()!r}: expected 'g1://local?left=ADDR&right=ADDR' (quote it in a shell)")
    return {"L": query["left"][0], "R": query["right"][0]}


def locate(want, timeout=15.0):
    """The arms of want (side: address) by the addresses this host knows them by, once all of them were heard.
    CoreBluetooth shows a UUID per host and never a MAC, so there a MAC is matched by the arm's name: stock's and
    G1CFW_ names end in the address's low 3 bytes. G1v0_L (before 0.5.66) has no address in it: taken only if alone."""
    from bleak import BleakScanner
    hidden = sys.platform == "darwin"
    seen = {}

    def cb(dev, adv):
        seen[dev.address] = adv.local_name or seen.get(dev.address, "")

    def found(side):
        tail = want[side].replace(":", "")[-6:].upper()
        for address, name in seen.items():
            if address.upper() == want[side].upper():
                return address
            if hidden and re.fullmatch(rf"(Even G1_\d+|G1CFW)_{side}_{tail}", name):
                return address
        return None

    async def run():
        async with BleakScanner(cb, scanning_mode="active"):
            t0 = time.monotonic()
            while not all(found(side) for side in want) and time.monotonic() - t0 < timeout:
                await asyncio.sleep(0.5)

    for address in want.values():
        release(address)
    asyncio.run(run())
    where = {}
    for side, address in want.items():
        where[side] = found(side)
        old = [a for a, name in seen.items() if hidden and re.fullmatch(rf"G1v0S?_{side}", name)]
        if not where[side] and len(old) == 1:
            print(f"{SIDES[side]} arm: {seen[old[0]]!r} has no address in its name to check against {address}")
            where[side] = old[0]
        if not where[side]:
            others = [name for name in seen.values()
                      if re.fullmatch(rf"(Even G1_\d+|G1CFW)_{side}_[0-9A-F]{{6}}|G1v0S?_{side}", name)]
            hint = (f"Other arms on that side: {', '.join(others)} (if one is this arm, its address has changed: 0.5.66 "
                    "moved g1v0 to stock's address). " if others else "")
            sys.exit(f"the {SIDES[side]} arm ({address}) was not heard within {timeout:.0f} s. {hint}An arm stops "
                     "advertising while connected (a phone, for one). Nothing was done.")
    return where


def reconnect(args, address, timeout=240.0):
    t0 = time.monotonic()
    while True:
        try:
            return connect(args, address)
        except Exception as ex:
            if time.monotonic() - t0 > timeout:
                sys.exit(f"no connection to {address} within {timeout:.0f} s ({ex}): leave the arm in the case "
                         "on USB and see README.md, Recovery")
            time.sleep(5)


def wipe_store(arm):
    """Before a return to stock: g1v0's asset store erased (VM.md §6.10), so stock meets erased flash where
    its glyphs and pictures were. A firmware without the store (status 2) wrote nothing there."""
    if arm.ctrl("ASSET_INFO")["status"]:
        return
    print("  erasing the asset store first (minutes while stock's old data is still there)")
    result = arm.asset_format(wipe=True)
    if result:
        sys.exit(f"the asset store's wipe failed ({g.ERRORS.get(result, result)}): nothing was installed")


def install(args, img, stock_after):
    version = image_version(img)
    label = f"Even's {args.version}" if stock_after else f"g1v0 {version}"
    arm = connect(args, stock=args.stock)
    print(f"{args.ble or args.emu}: images now")
    show_images(arm)
    moves = False
    if not args.stock:
        h = hello(args, arm)
        moves = not stock_after and has_stock_address(h["version"]) != has_stock_address(version)
        if not h["in_case"]:
            ask(args, "The arm is not in the charging case: install only in the case, on USB, lid open.")
        if stock_after:
            wipe_store(arm)
    t0 = time.monotonic()
    up = g.smp_upload_verified(arm, img, progress=lambda off, n: print(f"\r  upload {off}/{n}", end="", flush=True))
    print(f"\n  slot 1 holds {label} ({up['kBps']} KB/s, {time.monotonic() - t0:.0f} s)")
    which = f"the {SIDES[args.side]} arm" if args.side else "this arm"
    ask(args, f"Install {label} on {which} now? MCUboot overwrites the running firmware for good (no revert).")
    before = arm.losses()
    g.smp_commit(arm, img)
    print("  confirmed, the arm resets: MCUboot copies the image (about 20 s), keep the arm powered")
    if args.emu:
        while True:
            arm.run_until(lambda: arm.losses() > before, 240.0, "the reset")
            arm.setup(240.0)
            if arm.tr.has("G1_CTRL" if not stock_after else "NUS_RX"):
                break
            before = arm.losses()
    else:
        arm.tr.close()
        address = args.ble
        if args.stock or stock_after or moves:
            seen = scan(args, stock_after, skip=address if moves else "")
            if not seen:
                sys.exit("the arm did not advertise within 240 s: leave it in the case on USB and see README.md, Recovery")
            print(f"  advertising again as {seen[1]!r} at {seen[0]}")
            address = args.ble = seen[0]
            if stock_after:
                return
        time.sleep(15)
        arm = reconnect(args, address)
    for attempt in range(5):
        try:
            show_images(arm)
            h = None if stock_after else hello(args, arm)
            break
        except (TimeoutError, KeyError):
            if attempt == 4:
                raise
            print("  the link dropped (the arm may have restarted once more): trying again")
            arm.run_until(lambda: arm.tr.connected, 240.0, "a new link")
            arm.setup(240.0)
    if h and h["version"] != version:
        sys.exit(f"the arm runs {h['version']}, not {version}")
    arm.tr.close()
    print("done")


def act(args, img):
    if args.cmd == "list":
        arm = connect(args, stock=args.stock)
        show_images(arm)
        if not args.stock:
            hello(args, arm)
        arm.tr.close()
    else:
        install(args, img, args.cmd == "restore")


def both_arms(args, img):
    """--connection: the left arm first (README.md), each link closed before the next, stopping at a failure."""
    conn = parse_connection(args.connection)
    changed = False
    where = locate({side: address for side, address in conn.items() if args.side in (None, side)})
    for n, (side, address) in enumerate(where.items()):
        args.side, args.ble = side, address
        print(f"{SIDES[side]} arm")
        try:
            act(args, img)
        except BaseException:
            if n and args.cmd != "list":
                print("the left arm is done, the right arm did not finish: --side R goes on with it",
                      file=sys.stderr)
            raise
        if args.ble != address:
            conn[side], changed = args.ble, True
    if changed:
        print(f"the address changed, now: g1://local?left={conn['L']}&right={conn['R']}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    where = ap.add_mutually_exclusive_group(required=True)
    where.add_argument("--ble", help="the arm's Bluetooth address")
    where.add_argument("-c", "--connection", help="both arms: g1://local?left=ADDR&right=ADDR, or a file holding that")
    where.add_argument("--emu", help="an emulated arm's bridge socket")
    ap.add_argument("--stock", action="store_true", help="the arm runs Even's firmware now")
    ap.add_argument("--side", choices=("L", "R"),
                    help="the arm's side (finds it after an address change); with -c, only this arm")
    ap.add_argument("--yes", action="store_true", help="do not ask before the point of no return")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list")
    p = sub.add_parser("install")
    p.add_argument("image")
    p = sub.add_parser("restore")
    p.add_argument("version", nargs="?", default="1.6.6")
    args = ap.parse_args()
    if args.ble and not args.side and (args.cmd == "restore" or args.cmd == "install" and args.stock):
        sys.exit(f"{args.cmd} needs --side L|R: the arm comes back under another address")
    img = None
    if args.cmd == "install":
        img = Path(args.image).read_bytes()
    elif args.cmd == "restore":
        img = stock_image(args.version)
    if args.connection:
        both_arms(args, img)
    else:
        act(args, img)


if __name__ == "__main__":
    main()
