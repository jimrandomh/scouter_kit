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
from pathlib import Path

FW = Path(__file__).resolve().parent
sys.path.insert(0, str(FW.parent / "tools"))
import g1host as g  # noqa: E402


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


def hello(arm):
    h = arm.hello()
    print(f"  g1v0 {h['version']} ({h['role']}), in the case {h['in_case']}, encrypted {h['encrypted']}, "
          f"safe level {h['safe_level']}, panel ok {h['panel_ok']}")
    return h


def ask(args, text):
    if args.yes:
        return
    if input(f"{text}\nType yes to go on: ").strip() != "yes":
        sys.exit("stopped: nothing was installed (slot 1 holds the upload, unmarked)")


def scan(args, stock, timeout=240.0):
    """After an install that changes the address: the arm's advertising by its name and side."""
    from bleak import BleakScanner
    found = []

    def cb(dev, adv):
        name = adv.local_name or ""
        if stock:
            ok = name.startswith("Even G1_") and f"_{args.side}_" in name
        else:
            ok = re.fullmatch(rf"G1CFW_{args.side}_[0-9A-F]{{6}}|G1v0S?_{args.side}", name)
        if ok:
            found.append((dev.address, name))

    async def run():
        async with BleakScanner(cb, scanning_mode="active"):
            t0 = time.monotonic()
            while not found and time.monotonic() - t0 < timeout:
                await asyncio.sleep(0.5)
    asyncio.run(run())
    return found[0] if found else None


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
    if not args.stock:
        h = hello(arm)
        if not h["in_case"]:
            ask(args, "The arm is not in the charging case: install only in the case, on USB, lid open.")
        if stock_after:
            wipe_store(arm)
    t0 = time.monotonic()
    up = g.smp_upload_verified(arm, img, progress=lambda off, n: print(f"\r  upload {off}/{n}", end="", flush=True))
    print(f"\n  slot 1 holds {label} ({up['kBps']} KB/s, {time.monotonic() - t0:.0f} s)")
    ask(args, f"Install {label} on this arm now? MCUboot overwrites the running firmware for good (no revert).")
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
        if args.stock or stock_after:
            seen = scan(args, stock_after)
            if not seen:
                sys.exit("the arm did not advertise within 240 s: leave it in the case on USB and see README.md, Recovery")
            print(f"  advertising again as {seen[1]!r} at {seen[0]}")
            if stock_after:
                return
            address = seen[0]
        time.sleep(15)
        arm = reconnect(args, address)
    for attempt in range(5):
        try:
            show_images(arm)
            h = None if stock_after else hello(arm)
            break
        except (TimeoutError, KeyError):
            if attempt == 4:
                raise
            print("  the link dropped (the arm may have restarted once more): trying again")
            arm.run_until(lambda: arm.tr.connected, 240.0, "a new link")
            arm.setup(240.0)
    if h and h["version"] != version:
        sys.exit(f"the arm runs {h['version']}, not {version}")
    print("done")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    where = ap.add_mutually_exclusive_group(required=True)
    where.add_argument("--ble", help="the arm's Bluetooth address")
    where.add_argument("--emu", help="an emulated arm's bridge socket")
    ap.add_argument("--stock", action="store_true", help="the arm runs Even's firmware now")
    ap.add_argument("--side", choices=("L", "R"), help="the arm's side (finds it after an address change)")
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
    if args.cmd == "list":
        arm = connect(args, stock=args.stock)
        show_images(arm)
        if not args.stock:
            hello(arm)
    elif args.cmd == "install":
        install(args, Path(args.image).read_bytes(), False)
    else:
        install(args, stock_image(args.version), True)


if __name__ == "__main__":
    main()
