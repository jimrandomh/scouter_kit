# ScouterKit firmware for the Even Realities G1

`g1v0` is an alternative firmware for the two arms of the Even Realities G1 glasses. It replaces Even's
firmware on each arm's application processor (an nRF5340) and keeps the parts that came from the
factory: the bootloader (MCUboot), the Bluetooth controller on the network core, and the factory
settings. A PC (or any other device) talks to it with ScouterKit's protocol 5 ([docs/VM.md](docs/VM.md))
through the libraries and tools of this repository (`../src`, `../go`, `../python`, `../tools`).

What it gives you:
- the full 640 × 480 panel of each eye, 16 grey levels, drawn by the host;
- uploads that do not lose data (about 10–13 KB/s per arm), both eyes drawing at the same moment;
- small programs that run on the glasses without a host (a clock, notifications, head-tilt pop-ups);
- the accelerometer (head pitch, roll, motion), the gyroscope, the TouchBar (tap, double and triple tap,
  long press), wear detection as Even's (from the arm-to-arm radio's signal), and on the right arm the light
  sensor with Even's automatic brightness (the left eye can follow it) and the microphone (LC3 audio, only while
  a host asks for it, with an indicator on the panel);
- the arm-to-arm radio as a message link between programs on the two arms, so they can draw together without a
  host (a message takes 6–150 ms; both eyes change within a fraction of a millisecond when the programs agree on
  a time);
- a panel standby for fast pop-ups (4 ms instead of 71 ms to light up, about 1 mA while waiting);
- a relay of an iPhone's notifications (ANCS) to an app on that iPhone, for a host that asks for it;
- charging, battery care and the charging case's link done exactly as Even's firmware does them; programs can read
  the case's lid, charger and battery, and whether the arm lies in the case. Lit pixels disturb the case's charging
  link, so the firmware keeps the panel's LEDs dark while the case talks to the arm, and the demos switch the panel
  off in the case and while the glasses are folded (the libraries' `PanelPolicy`; until 0.5.58 the firmware itself
  kept it off in the case).

What it does not do: anything of Even's app (dashboard, AI, translation, navigation, the app's
notifications). Wear detection has not been tried with the glasses on.

Status: developed on one pair of G1 glasses (both arms came with Even's firmware 1.6.3), always tested
first in an emulator and on an nRF5340 development kit. Most tests on the glasses ran with them in the
charging case; worn use has had little testing. It is a personal project without any support: you install
it at your own risk.

## Risks and points of no return

Read all of this before installing.

- **There is no way back to the firmware your arms run now.** The bootloader overwrites the running
  firmware when an update is confirmed, keeps no copy and cannot roll back. Even publishes no copy of
  most versions; what you can install back is Even's official 1.6.6 (or the older public 1.5.6 and
  1.3.6), see [Restore Even's firmware](#restore-evens-firmware).
- **An image that never brings Bluetooth up bricks the arm.** Updates (also back to Even's firmware)
  only go over Bluetooth. g1v0 starts Bluetooth and the update service before anything else, reboots
  itself out of faults (watchdogs), falls back to a safe mode after repeated bad boots, and is tested
  in the emulator before every release; still, the only recovery from a dead arm is opening it and
  erasing the chip through its debug port, which destroys the factory bootloader and settings. Use only
  images that passed the emulator tests (`scripts/gate.sh`).
- **Power loss while the bootloader copies the image** (the first ~20 s after the confirmation) leaves
  no bootable firmware until the copy is redone, which the bootloader does at the next boot. An empty
  battery at that moment is the real risk: install only with charged arms in the case, on USB.
- **Pairing the PC with an arm replaces the phone's pairing** with that arm (Even's firmware keeps one).
  Even's app does not work with g1v0 at all.
- **The network core.** g1v0 never updates the Bluetooth controller, except in one case: after three
  boots in a row without Bluetooth it reflashes Even's 1.6.6 network-core image (built into the image)
  as a last resort. That path has never run on an arm; the emulator follows it only up to the
  bootloader accepting the update (the transfer into the network core cannot be emulated).
- **In the charging case the arms accept anyone.** While an arm is in the case (from the case's power, field
  or messages until it is lifted out or worn; lying still in a full case it stays "in the case"), any
  nearby Bluetooth device can send it commands and install firmware, exactly as with Even's firmware.
  Out of the case only a bonded, encrypted host gets through, and new bonds are only made in the case.
- **Warranty.** A non-Even firmware most likely voids it (a guess; ask Even). Even still repairs G1.

The points of no return, in order: (1) pairing the PC with the arms; (2) confirming the first install
on the left arm (Even's firmware on it is gone); (3) the same on the right arm (no arm with Even's
firmware is left). Restoring Even's firmware later is an install like any other.

## What you need

- G1 glasses with both arms charged, the charging case on USB power, the lid open, the glasses inside.
- The phone's Bluetooth switched off while you work (Even's app would connect to the arms).
- A Linux PC with BlueZ (tested with 5.82) and a Bluetooth LE adapter; Python 3.10 or newer with the
  tools' requirements: `python3 -m venv .venv && .venv/bin/pip install -r ../tools/requirements.txt`
  (the commands below write `python3`: use the venv's).
- To build the image: rootless podman (Debian: `podman uidmap passt fuse-overlayfs`), git, curl and
  about 4 GB of disk space.

## Build the image

Images are not published: each one carries Even's network-core image (for the fallback above) and the
TouchBar controller's configuration table from Even's firmware, which the build downloads from Even's
servers and checks by their SHA-256 (`stock_images.txt`).

```
scripts/setup.sh build      # once: containers, NCS v2.5.1 and the Zephyr SDK, Even's 1.6.6 images
scripts/build.sh 0.5.40     # work/build/g1v0-v0540/zephyr/app_update.bin is the image to install
```

Before you install it, run the emulator tests (`scripts/setup.sh emulator` once, then
`scripts/gate.sh 0.5.40`; `scripts/gate.sh 0.5.40 all` runs all of them). Every test must pass.

## Install, step by step

Run the commands from this directory (`g1_firmware/`). One arm at a time, the left arm first (with Even's firmware the right arm decides when both eyes draw;
a right arm on g1v0 next to a left arm on Even's firmware would leave the left eye dark).

1. Put the glasses in the case, lid open, case on USB. Switch the phone's Bluetooth off.
2. Find the arms and pair the PC with the left one (point of no return 1: the phone loses this arm).
   In `bluetoothctl`: `agent NoInputNoOutput`, `default-agent`, `pairable on`, `scan le` until the arms
   are listed (`Even G1_NN_L_XXXXXX` is the left arm, `Even G1_NN_R_XXXXXX` the right one; note their
   addresses), `scan off`, `pair LEFT_ADDR`, `quit`.
3. Check the arm and its firmware slots: `python3 flash.py --ble LEFT_ADDR --stock list`.
4. Install (point of no return 2):
   ```
   python3 flash.py --ble LEFT_ADDR --stock --side L install work/build/g1v0-v0540/zephyr/app_update.bin
   ```
   It lists the arm's firmware slots, uploads the image to the second slot (about a minute; Even's
   firmware keeps running meanwhile, and nothing changes if you stop here), checks it, and asks you to
   type `yes`. Then it confirms the image and resets the arm: the bootloader copies the image (about
   20 s) and g1v0 starts. Do not take the arm out of the case meanwhile. The tool waits until the arm
   advertises as `G1v0_L` and checks its version. g1v0 uses the chip's own Bluetooth address, so the arm
   now has a **new address**, printed by the tool (`NEW_LEFT_ADDR` below).
5. Pair the PC with the new address (in the case): `bluetoothctl pairable on`, then
   `python3 ../tools/arm_pair.py NEW_LEFT_ADDR`.
6. Check it: `python3 ../tools/g1host.py --ble NEW_LEFT_ADDR hello` shows `"proto": 5`, the version and
   `"encrypted": true`; `python3 ../tools/g1host.py --ble NEW_LEFT_ADDR grid --label LEFT --verify`
   draws a test grid on the left panel and reads it back (`"match": true`); look through the left lens.
   `... panel off` switches the panel off again (it also goes off by itself 30 s after the last drawing
   once no host is connected).
7. The right arm: steps 2–6 with `RIGHT_ADDR`, `--side R` and `--label RIGHT` (point of no return 3).

Both arms now run g1v0. The libraries (`../src`, `../go`, `../python`) and `../tools` take the new
addresses.

Updating to a newer g1v0 later is the same command without `--stock` (the address stays):
`python3 flash.py --ble NEW_LEFT_ADDR install IMAGE`. g1v0 accepts an update out of the case only from a
bonded host; in the case from anyone.

`-c` does both arms in one run, in place of `--ble`. It takes a connection string, or the path of a file holding one:
```
python3 flash.py -c 'g1://local?left=LEFT_ADDR&right=RIGHT_ADDR' install IMAGE
```
Both arms must be advertising before either is touched. The left arm goes first, each arm asks for its own `yes`, and
a failure stops the run (`--side R` then goes on with the right arm alone). `list`, `restore` and `--stock` work the
same way. If an install moves an arm to another address (an update across 0.5.66, a restore), the tool prints the new
string. macOS hides Bluetooth addresses, so there a MAC is matched by the arm's name, which ends in the address's low
three bytes; `G1v0_L` and `G1v0_R` (before 0.5.66) have no address in the name, so they are taken only if a single
arm on that side is advertising, and the tool says so.

## Restore Even's firmware

```
scripts/fetch_stock.sh 1.6.6            # Even's official 1.6.6 from Even's server, checked by SHA-256
python3 flash.py --ble NEW_LEFT_ADDR --side L restore 1.6.6
```

This installs Even's application firmware 1.6.6 on the arm through g1v0's update service, the same way
as above (in the case, charged, typing `yes`); the network core is left as it is. The arm comes back
under its old address and name (`Even G1_NN_L_XXXXXX`). Do the right arm too, then remove the arms
from the PC (`bluetoothctl remove ADDR`) and pair the glasses with the phone again in Even's app, which
may then offer its own update. The restore was rehearsed in the emulator and on the nRF5340 DK (Even's
1.6.6 and 1.5.6 app images booted and answered), not on a G1 arm.

## Recovery

- **The arm does not show up after an install.** Leave it in the case, on USB, lid open, for a few
  minutes: the bootloader redoes an interrupted copy, a hung boot is reset by a watchdog within 2
  minutes, and after three boots without Bluetooth the network core is reflashed (see above). Then scan
  again: `bluetoothctl --timeout 20 scan on`.
- **It shows up as `G1v0S_L` or `G1v0S_R`** (safe mode, after three bad boots in a row): drawing and
  programs are off, the update service works. `python3 ../tools/g1host.py --ble ADDR reboot --mode 1`
  returns to normal mode; if it comes back, install another image (or Even's).
- **The PC cannot connect out of the case** (no bond, or a lost one): in the case, `bluetoothctl remove
  ADDR`, then pair again with `../tools/arm_pair.py ADDR`. Each arm keeps three bonds; a new one replaces
  the oldest.
- **A tool cannot connect to a bonded arm**: BlueZ reconnects bonded devices by itself and then holds
  the link (the arm stops advertising). `bluetoothctl disconnect ADDR` releases it.
- **One arm on g1v0, the other on Even's firmware** is not a supported state: install the second arm or
  restore the first.
- **Back to Even's firmware** (`flash.py restore`): from g1v0 0.5.56 the asset store takes the flash where
  Even's fonts and pictures were; `restore` erases it first, so Even's firmware then runs without its CJK
  text and demo pictures (they do not come back; settings and the factory record stay).
- **An arm that never advertises again, even after hours in the case**, needs a repair. Even repairs G1
  glasses.

## More

- [docs/VM.md](docs/VM.md): protocol 5 (the VM, its programs, the wire protocol), the source of truth for
  the byte layouts; [docs/PROTOCOL.md](docs/PROTOCOL.md): the older protocol v4 of g1v0 0.4.x.
- [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md): the source layout, building, the emulator, diagnostics and
  the change notes.
- `flash.py --help`; `flash.py --ble ADDR [--stock] list` shows an arm's firmware slots without changing
  anything.
