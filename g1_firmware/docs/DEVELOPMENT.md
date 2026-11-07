# g1v0 for developers

Internals and change notes of the ScouterKit firmware g1v0, the firmware of [../README.md](../README.md)
(that page is for owners: installing, risks, recovery). g1v0 0.5.x speaks protocol 5, the on-glasses VM of
[VM.md](VM.md) (byte layouts are the source of truth); 0.4.x spoke protocol v4 ([PROTOCOL.md](PROTOCOL.md)).

Paths on this page are relative to `g1_firmware/`.

| Path | What |
|---|---|
| `firmware/` | The firmware: a Zephyr application for the nRF5340 app core, NCS v2.5.1 (`src/`, `prj.conf`, `Kconfig`, the board in `app.overlay`, the DK in `dk.overlay`, the factory partition map in `pm_static.yml`, the MCUboot child configuration in `child_image/`) |
| `../src/vm` | The VM core (`vm.c`, `vm_gfx.c`), compiled into the firmware in place, unchanged: the firmware is its platform (`struct vm_host`) and makes its `INPUT` events. The same core runs in the host library's emulated arms and in the reference client's mirror |
| `../third_party/liblc3` | Google's LC3 codec (Apache-2.0), the microphone's encoder, compiled in place |
| `../tools/g1host.py` | The reference host client (Python): host code assembler, the window and RTO of VM.md §6.2, all control requests, SMP, and a mirror that is the VM core itself (`g1mirror.c`), so arena and panel compare byte for byte |
| `g1min/` | A minimal app whose build yields the replica of the G1's MCUboot (sdk-mcuboot of NCS v2.5.1, the G1's configuration: overwrite-only, RSA-2048 with MCUboot's public sample key, two images) for the emulator, and the DK's flash overlay |
| `container/`, `scripts/` | The build and emulator containers and scripts (`setup.sh`, `build.sh`, `gate.sh`, `bridge_start.sh`, `fetch_*.sh`, `box`) |
| `emulator/` | Renode harness (`harness/`), the virtual phone (`models/G1_VirtualPhone.cs`), this project's changes to seven of g1-fw's models (`models/patches/`, ed scripts), the tests (`tests/v5_tests.py`, `tests/common.py`) |
| `stock_images.txt` | Even's official images: URL and SHA-256, for `scripts/fetch_stock.sh` |
| `work/` | Everything fetched or built (NCS, Renode, g1-fw, Even's images, builds, emulator runs); `G1_WORK` moves it |

Firmware images are not distributed: each embeds Even's network-core image for the self-fallback
(`src/netimg.S`, `src/netfallback.c`), fetched by `scripts/fetch_stock.sh` from Even's CDN.

## Build and test

```
scripts/setup.sh all            # once: containers, NCS + SDK, Even's 1.6.6, Renode, g1-fw, seeds, replica MCUboot
scripts/build.sh 0.5.40 0.5.41  # work/build/g1v0-v0540, -v0541 (the "next update"), -dk-v0540, -v0540-emumic
scripts/gate.sh 0.5.40          # the standard emulator tests; `all` for all 49; G1_FROM=0.5.38 adds the update chains
```

`build.sh` builds the G1 image, the same source as the "next update" (the emulator's update tests), the
nRF5340 DK variant (no panel, a 120 s pairing window after boot) and the emulator test variant
(`G1_PANEL_IDLE_S=2`, `G1_MIC_ANY_LINK=y`), each in its own container without network. Builds are
reproducible: the same source gives the same `zephyr.bin` and image hash (the SMP list's); the signed file
differs only in its RSA-PSS signature (random salt). 0.5.40's image hash is `7c14dc03…`.

The emulator (Renode 1.16.1 with g1-fw's nRF5340 platform and models, commit `b8ec262`) runs one arm per
test in a rootless, network-less container: the replica MCUboot, the g1v0 image, Even's 1.6.6 network core
and a virtual phone that bridges the emulated BLE link to a host program (`g1host.py --emu SOCKET`).
`gate.sh`'s standard set: `smoke power unattended security mic_capture mic_lease draw_bench panel_standby
panel_prop panel_read video panel_guard in_case case_hold case_blank mic_popup_standby touch light gyro peer
pair_link assets`, plus `chain chain_right` (0.4.10 → G1_FROM → VERSION → NEXT
over SMP, both roles) when those builds exist; `all` adds the rest, among them `restore` (Even's 1.6.6 installed by
`flash.py` itself). The board harness has the TouchBar controller (g1-fw's model with this project's interrupt
line, wear events and modes) at TWIM2 0x12 with its interrupt on P1.09; the firmware arms that interrupt before the
controller's bring-up, because Renode's GPIOTE takes a pin's level when a channel is configured and only learns it
again from the pin's changes (the same reason the first GPO pulse after a boot is lost).
Emulator limits: no pairing or encryption (the tests run "in the case"), timing is not real (virtual time,
zero-time peripherals), and the emulated net core sometimes stalls after a disconnection or an install (a known
flake: rerun the test). One arm per test, its partner g1-fw's virtual ESB slave (it answers every frame at once,
and this project's patch shows the master's last frame and sends a frame given to it); the pair tests
(`pair_link`, `pair_stream`) run both arms in one emulation (`scripts/pair_start.sh`: machines `right` and
`left` on one radio medium, each with its own virtual phone, `common.Pair` drives them from two threads, since a
waiting phone stops the whole emulation). Their radios need a 20 µs global quantum (100 µs loses ESB
acknowledgements and the net cores stop exchanging), which makes them about 13 times slower than real time, and
even then the two emulated net cores' ESB stalls now and then for a second or more (with no app frames they
hardly exchange at all), so the pair tests check values and order, not timeouts. The left machine has no light
sensor, as the arms (its OPT3001 model NACKs); the single-arm left tests keep it. Panel times: the patched
panel model lists its last 32 pixel windows with their virtual end times (`PixelWindows`), the same clock on both
machines.

On-arm checks after an install (`../tools/`): `vm_check.py ADDR`, `imu_check.py --ble ADDR`,
`draw_bench.py --ble ADDR`, and on the right arm `mic_check.py --ble ADDR` (`--draw` adds display traffic,
`--clock 1` captures at 128 MHz on test builds). Tools that use the microphone must renew its lease
(`g1host.py` `mic_hold`; the library does it itself); Go binaries need `go build -a` after a change in `../src`.

## Firmware-specific control requests

VM.md §6.4 reserves ops 0x09 and 0x3F for diagnostics and test builds:
- **0x09 `DIAG`**: body 0: u8 sub-op. Sub-op 1 `POWER`, 1: u8 page (0 live values, 1…8 the per-state
  accumulators): the reply body of PROTOCOL.md §8.6 (48 or 40 bytes; flags bit 0 is now 0, ESB mode 0).
  Sub-op 2 `PANEL_PROBE`, 1: u8 MHz, 2: u8 flags, 3: u16 delay ms: the 0.3.2 probe record (138 bytes).
  From 0.5.2 (`src/diag.c`; client `tools/g1host.py` `pin-probe`, `regs`, `diag-set`, `net-off`, `imu`):
  - 3 `PIN_PROBE` (read only, a few ms): the unused pins (P0.02/03/06/12/25/27–29/31, P1.04–12/15) get
    pull-ups while the LSM6DSO drives INT1, then INT2 active low in open drain (data ready, latched).
    Reply: u32 P0 mask, u32 P1 mask, then 5 × (u32 P0.IN, u32 P1.IN) for the steps none, INT1, none, INT2,
    none; a pin reading 1 0 1 1 1 is INT1, 1 1 1 0 1 INT2.
  - 4 `REGS`, 1: u8 page (read only): reply u8 page, then 0–2: `PIN_CNF` of P0.00–15, P0.16–31, P1.00–15
    (16 × u32); 3: P0 and P1 `OUT`, `IN`, `DIR`, `GPIOTE.CONFIG[0..7]` (14 × u32); 4: 20 × u32 (UARTE0,
    TWIM1, TWIM2, SPIM4, SAADC, PDM0, QSPI `ENABLE`; `CLOCK` HFCLKSTAT, HFCLK192MSTAT, LFCLKSTAT, HFCLKCTRL;
    the three regulators' DCDCEN; `RESET.NETWORK.FORCEOFF`; WDT0 RUNSTATUS; GPIOTE0 INTENSET; P0/P1 LATCH,
    P0 DETECTMODE); 5: 41 nPM1300 registers (bucks, load switches, GPIOs, LED drivers, timer, POF, ship,
    charger status, VBUS status; names in the client).
  - 5 `SET`, 1: u8 item, 2: u8 value (test builds): 1 the touch rail P0.30 off/on, 2 LDSW2 (IMU and
    microphone) off/on (the IMU is initialised again), 3 the touch controller's mode (`0x4408` = 1 active,
    2 sleep, 3 deep sleep; 0 only reads) with u32 chip ID, `REG_WST` before and after, 4 (0.5.4) the tag's
    GPO pin out of the case: 1 input with pull-up as 0.5.2, 0 disconnected (the default), 5 (0.5.10) the capture
    clock: 1 the app core at 128 MHz while the microphone captures, 0 at 64 MHz (`G1_MIC_128MHZ`). Reply ends u8
    touch rail, u8 IMU present. For power measurements only: nothing sets them otherwise.
  - 6 `NET_OFF`, 1: u16 s, 3: u8 phases (test builds; s ≥ 20, phases 1–4, s × phases ≤ 2400): the arm
    replies and reboots into a boot with its network core held off (`RESET.NETWORK.FORCEOFF`, no Bluetooth
    or ESB) that measures the phases (0 net core off; 1 also the touch rail off; 2 also LDSW2 off; 3 all on
    again) from the PMIC's samples on battery, keeps the result in retained RAM and reboots normally. RAM
    only and one-shot: the request is cleared first, so any reset returns to a normal boot; both watchdogs
    stay armed; the health check reboots at a deadline (run time + 80 s); nothing is written to flash, and
    the net-core fallback's counter of boots without Bluetooth is not touched.
  - 7 `NET_OFF_RESULT`: u8 phases done, u16 s, u8 the fallback counter at the test, u8 the counter now, then
    per phase u32 samples, u32 samples on battery, i32 average IBAT µA, u16 VBAT first and last mV, i16 NTC
    0.1 °C, u16 app-core load ‰.
  - 8 `IMU` (read only): u32 samples read, u32 polls without a new sample, u32 I²C errors, u8 every sample
    being read (MOTION or TILT can fire), u8 IMU present, u32 INT1 wake-ups (0.5.4+); 0.5.8+: i32 stock's
    factory pitch base (settings +0x0C, centidegrees; 0x80000000 without a record), 9 × i16 the head map
    (Q14, row-major: the head axes x, y, z in sensor axes).
  - 9 `MIC` (0.5.8+, read only; counters and levels, never audio): u8 on, u8 rate, u8 frames per event, u8
    capture available (`HELLO.board` bit 2), then u32 starts, frames, lost (no free buffer), encoded, events,
    event drops (ring full), faults, total encode µs, then u16 longest encode µs, peak |sample| and RMS of the
    last second (both since the last `MIC` 1, after the DC blocker).
  - 11 `TOUCH` (0.5.18+, read only): u8 present, u8 scanning, u32 events read, u32 I²C errors, u32 table loads, u32
    chip ID, u32 `REG_WST` (0x01000000 scanning, 0x03000000 asleep), u32 event enable, u32 `TOUCH_GESTURE`, u32
    `WORN`, u32 interrupts taken (`g1host.py` `touch_stats`).
  - 12 `LIGHT` (0.5.18+, read only): u8 present, u8 converting, u8 `AUTO_BRIGHTNESS`, u32 readings, u32 I²C
    errors, u32 `LIGHT` (`g1host.py` `light_stats`).
  - 10 `ENCODE_LOAD`, 1: u16 s (≤ 900), 3: u8 128 MHz (test builds, 0.5.10+): the LC3 encoder alone on a synthetic
    tone, 100 frames a second for s seconds, with no PDM, no microphone and nothing sent (the encoder's energy,
    measured without audio; `tools/encode_load.py`); `MIC` is refused (12) meanwhile; DIAG `MIC` counts it.
  - 13 `PEER` (0.5.24+, read only): u8 the link available, u8 `PEER_LINK` as read (0/1/2), u8 the left's
    `AUTO_BRIGHTNESS` (following), u8 the net core's last link value; u32 its reports, u32 ESB acknowledged, u32
    failed, u32 received (from its last report), u32 that report's time; u32 frames queued, u32 delivery reports,
    u32 report timeouts, u32 send errors, u32 frames received (after duplicates), u32 values delivered, u32 inbox
    drops, u32 levels applied (left), u32 the last delivery's arrival time, u32 other net-core messages; i32 the
    clock offset (left minus right, µs; 0x80000000 unknown); u32 the last, longest and total time from queueing a
    frame to its delivery report (µs) (`g1host.py` `peer_stats`).
  - 14 `CASE` (0.5.26+, read only): u8 `IN_CASE`, u8 VBUS now, u8 the tag's field (last read), u8 moved since the last
    VBUS sample, field or message; u32 ms since that; u32 case messages, u32 battery requests, u32 replies; u16 busy
    mailbox, u16 unknown commands, u16 long-trip requests, u16 restart requests; u8 the last reply's status, u8 the
    case's battery byte, u8 its lid byte, u8 the tag's GPO configuration; u32 ms since the field was last seen
    (0xFFFFFFFF never); i8 the wear classification (−1 none), u8 in the case since it; u32 charger enables after a
    battery reply (`g1host.py` `case_stats`); 0.5.32+ appends the reply times, the tag's interrupt bits, the case hold
    (bytes 102–107, 0 since 0.5.60) and the LED blanking ("What 0.5.32 adds").
  - 15 `CASE_TRACE` (0.5.32+, read only): the case link's last 32 events (`case_trace`).
  - 18 `ASSETS` (0.5.56+, read only): the asset store's last put, u32 µs each: erase, program, verify (read
    back with SHA-256 and crc32), directory commit; u32 the longest sector erase µs, sectors erased, directory
    commits, `ASSET_LOAD`s, their bytes, the last first-load crc check µs, the last job's flash calls, the
    sectors the last wipe erased; u8 paused (`SET` 10), u8 step mode, u8 the armed checkpoint, u8 the directory
    copy in force (0 A, 1 B, 0xFF none); u32 the store thread's unused stack (0xFFFFFFFF: not measured without
    `CONFIG_INIT_STACKS`) (`g1host.py` `asset_stats`).
  - `SET` items 9 (0.5.56+) the next put resets the arm at checkpoint 1 after its erase, 2 in the middle of its
    program, 3 after its verification, 4 inside its directory write (32 bytes written), as a power failure would
    (`asset_reset_at`); 10 (0.5.56+) 1: the store pauses after every flash call that changes flash (each sector
    erase and program, the directory's erase and program) until 2 (one step), 3 aborts the job (error 13), 0 off;
    a `FLASH_CRC` runs while it pauses (`asset_step`, `asset_put_stepped`: the acceptance run's single-stepped put).
  - `SET` items 7 (0.5.32+) the idle bisect's switches (a mask, `idle_switches`; bit 0, QSPI deep power-down, is
    refused from 0.5.34) and 8 (0.5.34+) the "full" latch: 0 ends it now, so the case's next poll starts a top-up;
    `NET_OFF` takes one switch mask per phase (0.5.32+, up to 10 phases).
  - 19 `BONDS` (0.5.58+, read only, `g1host.py bonds`): the bonds with their aging counters (layout: "What 0.5.58
    adds"); the lowest counter not connected now is the one a new pairing replaces when all 3 are used.
  Status 5 for another sub-op or page.
- **0x3F `FAULT`** (test builds, `HELLO.flags` bit 5): 1 trap, 2 hang with interrupts off (WDT0), 3 panic,
  4 executor hang (health check), 5 stop the nPM1300 watchdog kicks (power cycle after 30 s).

Kconfig: `G1_PANEL_IDLE_S` (30), `G1_MIC_ANY_LINK` (n; the emulator test build: `MIC` on the virtual central's
unencrypted link), `G1_MIC_128MHZ` (n), `G1_IMU_BASE_DEFAULT` (−1987), `G1_NET_ESB_QUIET` (n), plus the options of
0.4.x. Devicetree (`app.overlay`):
`touch-power-gpios` (P0.30), `touch-int-gpios` (P1.09), `imu-int1-gpios` (P1.15), `nfc-gpo-gpios` (P1.10); PDM0's
pins are set by `mic.c`.

## Handover (2026-10-28, afternoon)

- **On the arms (2026-11-02):** 0.5.40 on both ("What 0.5.40 adds"); before that, from 2026-10-28:
  0.5.38 on both (left 14:11, right 14:12, with `flash.py`; image hash `f178d4cebb3f4c8c…`; the
  development tree's `emu/build/arm-rollout/g1v0-0.5.38-casepolls_g1_app_update.bin`), ESB mode 0, drawtest 40/40 on
  both (with the panel allowed in the case). Left running: the g1 notification program on both arms (restarted with
  `g1 mirror -check -for 2s`), the case hold in force (`CASE_PANEL` 0 on both), no host connected.
- **Gate of 0.5.38:** 49/49 of `all` with both update chains from 0.5.36 and `restore`. Known flakes, each passing on
  rerun: `unattended` (the emulated net core's stall), `pair_stream` (the emulated ESB's stall), `mic_tool_draw` (the
  emulated host misses the microphone's lease renewal under draw traffic, also on 0.5.30).
- **What this session found and changed** (0.5.32–0.5.38, notes below; `docs/power.md` §15 in the development tree):
  the case's lost polls came from the lit panel's LED current breaking the NFC link, not from the arm's timing (the arm
  answers within ~6 ms, the case looks after 75 ms); the case hold (in the case the panel is off as in stock) and
  `CTRL 0x15 CASE_PANEL` for development (not during a top-up); DIAG CASE timing and DIAG 15 `CASE_TRACE`; the idle
  bisect's switches (DIAG SET 7, the net-off masks) found nothing to save; QSPI deep power-down is refused (it stranded
  the left arm's flash until a PMIC power cycle); DIAG SET 8 ends the "full" latch for a top-up experiment.
- **Open for the firmware** (none blocks use):
  1. **A top-up warms the pack +7–11 °C at its peak** (~1 °C per minute of the case's power transfer, with or without
     the display, also in a short CV top-up at 6–13 mA): only an earlier "full" (e.g. at the CV taper) or shorter,
     more frequent top-ups would lower it; both change when charging ends, so they are the user's decision (charging
     is to stay as stock).
  2. The ~1.3 mA with the network core off: no firmware-switchable part draws it (power.md §15); a board-level probe.
  3. The right arm's NFC coupling is weaker (7× the left's field changes, 0–2 % of the case's polls lost in top-ups
     even with nothing drawn); the 2 ms wait after a pulse (0.5.36) raised the tag's NACKs (harmless retries) without
     lowering that loss; dropping the wait is a candidate.
  4. From the morning's handover: panel standby ≈ 1 mA; the run-time cap charges a DRAW's power-on to the program; the
     provisional IMU axis map; the `unattended` emulator flake.
- **To continue:** a change goes into the notes below; `scripts/build.sh NEW NEXT`, `G1_FROM=0.5.38
  scripts/gate.sh NEW all` (every test must pass, the known flakes on rerun), then `flash.py install` on one arm at a
  time, left first, `bluetoothctl disconnect ADDR`, a drawtest after each with `--case-panel` and the case
  at "full" (in a top-up the hold applies), `case-panel 0` after (it lasts until the arm leaves the case), the
  notification program restarted (`g1 mirror -case-panel -check -for 2s`, then `case-panel 0`), and the
  entry in the development tree's `docs/arm-rollout.md` §0.

## Change notes

The notes below were written during development. Their paths (`tools/…`, `emu/…`, `docs/…` other than VM.md
and PROTOCOL.md) are those of the development tree; `g1host.py`, `vm_check.py`, `imu_check.py`, `imu_cal.py`,
`draw_bench.py` and `mic_check.py` are in `../tools/`, the emulator tests in `emulator/tests/`, and
`emu/fwsrc/g1v0` is `firmware/`.

### What 0.5.66 adds (Even's address, names that tell pairs apart, version in the advertising)

- **Even's address.** `set_identity()` is gone, so Zephyr takes its default path as stock does: `settings_load()` reads
  a stored identity, else the net core's static address. Both firmwares live on the same settings partition with the
  same NCS 2.5.1, and `keys.c` reads stock's shorter key records, so a host's stock bond keeps working and the OS keeps one
  remembered device per arm. The way back: an image without `BT_KEYS_OVERWRITE_OLDEST` rejects this firmware's longer
  records. Host GATT caches now see two layouts under one address; `BT_GATT_SERVICE_CHANGED` and `BT_GATT_CACHING` are on.
- **Names:** `G1CFW_L_XXXXXX` (low three address bytes), so pairs can be told apart; the same digits as stock's names.
- **Advertising data:** the service UUID no longer fits (18 bytes, with a 16-byte name, in 31) and is gone: find arms by
  name or by the pair record. The record is format 2: safe mode (flags bit 3) and the firmware's version, so a host knows
  what an arm runs without connecting. Safe mode no longer changes the name.
- **Supervision timeout:** under 2 s the arm asks for 4 s (VM.md §6.1, `link_guard`), for 420 ms seen on a Pixel in
  crowded RF, where links died by supervision timeout.

### What 0.5.65 adds (the iPhone notification relay)

- **`ANCS`** (VM.md §6.12, `firmware/src/ancs.c`): a GATT client of the phone's ANCS on the link the phone made, relayed to a
  host app with the wire format of Faceclaw's custom firmware for the G2. `CONFIG_BT_GATT_CLIENT` is new; RX stack 2560.
- **Unlike the G2's** (which patches stock firmware and borrows its discovery): the relay finds ANCS itself, at every `START`,
  which saves following Service Changed. Output goes through a ring to the evt thread, because `bt_gatt_notify` can wait for
  a buffer and the BT thread must not; a frame is bound to its connection by a `bt_conn` reference, which also keeps the object
  from being reused. No lease, having no stock consumer to give ANCS back to.
- **The relay writes its CCCs with `bt_gatt_write`** and registers with `bt_gatt_resubscribe`, not `bt_gatt_subscribe` and
  `bt_gatt_unsubscribe`: if the phone refuses the unsubscribe of the last subscription, `gatt_write_ccc_rsp` finds no list and
  returns without calling back, which would leave the relay closing forever. The registrations are `VOLATILE`.
- **Attributes:** after `FC`, so `attr_g1_svc[6]` keeps its place (`ATTR_ANCS`). SMP, a dynamic service, moves up by 3 handles;
  the database hash changes, so bonded hosts get Service Changed, as for `PAIR` and `FC`. A host may not see the new
  characteristic on its first connection after the update.
- **Test:** `tests/ancs_test.c` (AGENTS.md) runs the state machine under ASan and UBSan against a scripted phone: each stage
  against errors, `STOP`, a new `START` and the link's end. The emulator tests `ancs` and `ancs_refused` run it on Zephyr's
  client against the virtual phone's ANCS (`A` in G1_VirtualPhone.cs): `CONFIG_G1_ANCS_ANY_LINK` in the emulator build, as the
  virtual central cannot encrypt. Not tested against a real iPhone by these.

### What 0.5.63 adds (properties without instructions; the display path and the microphone)

- **`PROP_SET`** (VM.md §6.4, `vm_prop_set`): a host on the display path has no arena words for `SYS_WRITE`'s operands.
- **A record left open by the next record's start is answered with a nack**, so the host need not wait for its ACK deadline
  (found on the arms: macOS drops writes without response it has no room for, bleak not waiting for
  `canSendWriteWithoutResponse`; `tools/g1fc.py` paces its packets since).
- **The microphone on the display path:** `FC` packets renew its lease; when it stops, `fc_indicator_cleared` repaints the
  indicator's corner, which `vm_set_mic` left black. `tools/g1fc.py` checks both by `PANEL_READ`.

### What 0.5.62 adds (Faceclaw's display path)

- **`src/fc.c`, `src/fc_inflate.c`** (VM.md §6.11): the records of Faceclaw's custom firmware for the G2, on a new `FC`
  characteristic, answered by a new `FC_ACK` event. The BT thread copies packets into a ring in the arena and publishes
  whole records; the executor runs one record per pass of its loop, so the panel is only touched from one thread and
  the health beat keeps going. The arena stays the only large buffer: +6.8 KB of flash, +1.6 KB of RAM.
- **The decoder** decodes one bit at a time, which is plenty at the link's rate, and rebuilds its tables per block. A whole
  record is in memory and ends in a sync flush, so it never suspends. `tests/fc_inflate_test.c` runs it against zlib
  under ASan and UBSan, damaged streams included.
- **`PAIR` takes writes too:** a write with response is what every host's stack can do without a new call.
- **Tests:** `faceclaw` (`tools/g1fc.py`, the same acceptance on an arm with `--ble`): every step checked by `PANEL_READ`,
  malformed messages, damaged records, recovery, `RESET_SESSION`. Not in the emulator: encryption.

### What 0.5.61 adds (a pairing trigger for CoreBluetooth)

- **`PAIR`** (VM.md §6.1, `src/proto.c`): a characteristic readable only on an encrypted link. CoreBluetooth has no
  pairing call and pairs only when an attribute needs it, and nothing did, so a Mac could not bond. `g1host.py pair`
  reads it, in the case. Who may bond is unchanged (`proto_update_bondable`): out of the case the attempt is refused.
  It follows `EVT`'s attributes, so `attr_g1_svc[6]` stays.
- **Not in the emulator:** its central cannot encrypt, so the gate only shows the table change does no harm.

### What 0.5.60 adds (no case hold: the display policy is the program's)

- **No case hold** (the user, 2026-11-05: display policy belongs in programs). In the case the panel works as
  anywhere: `src/exec.c` loses `case_hold`, `R_HOLD` and the hold's branches in `DRAW`, `PANEL` and `MIC` (the
  microphone starts in the case too); no `NOTICE` 6, no `NOTICE` 5 reason 4; `CTRL 0x15 CASE_PANEL` is gone (status
  2, `src/proto.c`) and with it the top-up exception (`caselink_charging`). `IN_CASE`, the `CASE` trigger and the
  case link are unchanged.
- **The LED blanking stays, now whenever the panel is on in the case** (`src/caselink.c` `blank_wanted` without
  `CASE_PANEL`): the LEDs dark (`A9`, the image kept) from the case's first RF command until 2 s after its last, at a
  full arm's re-checks and through a top-up, so a lit panel does not load the case's NFC link while it talks.
- **DIAG CASE** keeps its layout: bytes 102–107 (the hold, `CASE_PANEL`, its drops) read 0.
- **Libraries and tools:** `Connection::case_panel` (C `sk_g1_config.case_panel`, Go `Config.CasePanel`, Python
  `init(case_panel=)`), `--case-panel` and `g1host.py case-panel` are gone, and the library's emulated arm has no
  hold. New: `Program::PanelPolicy` (C `sk_g1_program_panel_policy`, Go `Program.PanelPolicy`, `../src/README.md`):
  the panel off while `IN_CASE` is 1 or `WORN` is 0, the program's `DRAW` and `VIDEO` aimed off the panel
  meanwhile; the Go demos with a program use it.
- **Tests:** `case_hold` checks that there is none (a `DRAW` shows in the case, `PANEL` 2 applies, `MIC` starts, no
  `NOTICE` 6, `CASE_PANEL` status 2); `case_blank` checks the LEDs dark through a run of polls instead of the hold;
  `chain` sends `CASE_PANEL` 1 to a `G1_FROM` firmware of 0.5.32–0.5.59 before it draws there.
- **Gate** `all` from 0.5.58 with both chains: 52/52. In the first run `unattended` (the emulated net core's HCI
  command timed out after a disconnection, the arm rebooted) and `pair_stream` (the pair emulation's slow ESB, the
  bridge closed) failed, the known emulator stalls; both passed when rerun (`logs/fwhw/22-…`, `23-…` in the
  development tree). Library: the C++, C, Go (with `-race`) and Python tests pass, and Go `TestPanelPolicy` is new.

### What 0.5.58 adds (bonds: least recently used, readable; no settings erase over SMP)

- **The bond that a new pairing replaces is now the least recently used one across reboots.** With 3 bonds used,
  Zephyr deletes the one with the lowest aging counter (`CONFIG_BT_KEYS_OVERWRITE_OLDEST`), even for a pairing
  attempt that is then refused. Until 0.5.57 the counter was stored only when a bond was made and raised in RAM
  by each encrypted link, so after a reboot the oldest *pairing* went: on the arms the PC's bond, inherited from
  stock (`docs/fps-link.md` §13.1 in the development tree). `CONFIG_BT_KEYS_SAVE_AGING_COUNTER_ON_PAIRING=y`
  stores the counter whenever another bond becomes the most recently used (one settings write per change of
  host). The record layout is unchanged (the counter was already in it), so 0.5.57's bonds load as they are,
  in their stored order; the first encrypted link of each host after the update makes that order persistent.
- **DIAG 19 `BONDS`** (read only; `src/bonds.c`, `g1host.py bonds`): u8 count, u32 the highest counter, then per
  bond 16 bytes: u8 address type, 6 address bytes (LSB first), u16 key types, u8 flags (1 authenticated, 0x10
  LE Secure Connections), u8 key size, u32 aging counter, u8 1 when that peer is connected now. It reads the
  host's internal `struct bt_keys` (`subsys/bluetooth/host/keys.h`, NCS 2.5.1; the include path is set for this
  file only) and writes nothing. No keys leave the arm.
- **MCUmgr's zephyr-basic group is off** (`CONFIG_MCUMGR_GRP_ZBASIC=n`): NCS's OTA sample enables it with NVS
  settings, and its only command erases the settings partition (every bond and setting). Nothing uses it:
  `flash.py` install and restore use the image and OS groups, the asset store's format and wipe are its own
  `ASSET_FORMAT` job.

### What 0.5.56 adds (the asset store)

Host-uploaded assets in each arm's QSPI flash (`docs/design-flash-assets.md` revision 1, VM.md §6.10, `ASSET_LOAD`
§2.4); `src/assets.c`:
- **Where.** Partition `g1_assets` 0x140000–0x1000000 (`pm_static.yml`; the DK 0x140000–0x800000), stock's glyph
  and asset area: the arm and stock address the chip with 24 bits, so the upper 16 MiB is out of reach without
  chip state. `stock_settings` (0x130000, 64 KiB, read only) and `external_flash` (the unused upper 16 MiB) tile
  the rest. Every store access is a `flash_area_*` call with a partition-relative offset; the store sends no
  instruction of its own (no bank register, no 4-byte mode, no power-down, no XIP). Checks: `BUILD_ASSERT` (base
  0x140000, end ≤ 0x1000000, 64 KiB multiples, no `address-size-32`), at boot QSPI `IFCONFIG0` 24-bit and the flash
  ready (else the store is unavailable), and `gate.sh` refuses a source tree that names `ADDRCONF`, `WREAR`,
  `EN4B`, 0xB7, 0xC5 or 0xE9. `FLASH_CRC` fingerprints 0x0–0x140000, which nothing of the store writes.
- **On flash.** Two directory copies (sectors 0 and 1): `"SKAS"`, u16 format 1, u16 count ≤ 120, u32 seq, u32
  partition size, 16 bytes 0, count × 32-byte entries (id[16], u32 offset, u32 size, u32 crc32, u8 type, u8 0, u16
  0), u32 crc32; valid = magic, format, count, this build's size, extents in the data area and disjoint, crc; the
  valid copy with the higher seq is the directory. Data from 0x2000 in whole 4 KiB sectors, first fit, erased when
  reallocated; a wipe skips sectors that are already erased.
- **Jobs** run on the store thread (priority 11, below the executor; 2 KiB stack) one flash call at a time: put
  = erase each sector, program 4 KiB at a time from the arena, read back with TinyCrypt SHA-256 and crc32 (the
  first 16 bytes must be the ID), then the other directory copy (erase, program, read back) is the commit; a reset
  before leaves the old directory. Two mutexes: the RAM directory (the control thread reads only it), and the
  flash calls, which the executor's `ASSET_LOAD` holds for its lookup, first-load crc check and read, so a load
  waits at most one flash call of a job and never reads an extent being erased. `NOTICE` 8 ends every job;
  `ASSET_INFO` state 4/5 tells the same when the notice is lost. The control request buffer is 32 bytes (was 24).
- **Cost.** RAM +10 720 bytes (2 KiB stack, 4 KiB sector buffer, 3.75 KiB RAM directory, TinyCrypt's state,
  the thread, the larger request queue; the design's budget was 10 KB), code +6.2 KB. Idle the store costs
  nothing (the driver leaves the QSPI off between calls).
- **Driver facts (NCS 2.5.1 `nrf_qspi_nor.c`):** a call takes the driver's semaphore and sleeps on the transfer's
  completion semaphore (no busy wait); after an erase, `qspi_device_uninit` polls the chip's busy bit with
  `k_msleep(50)` while it is still busy, so a sector erase may cost up to 50 ms more than the chip needs (the DK's
  89.7 ms per sector fits that). Job times on the arm: `docs/assets.md`.
- **Emulator.** g1-fw's `Macronix_MX25U.cs` gains a read-only `ExtendedAddressRegister` (patch
  `emulator/models/patches/Macronix_MX25U.cs.ed`; seven patched models now). Test `assets` (standard set): put,
  list, load, reboot, the same put (result 1, no erase), a wrong ID, a staged range changed during the job, a
  program paging with `ASSET_LOAD`, delete, format, the four power-fail checkpoints (the directory is the old
  one after each reset, checked in the flash image too), safe mode, a single-stepped put with the fingerprints
  after each flash call, a fill to exactly 0 free bytes, the wipe; the flash outside the store byte-identical,
  the fingerprints unchanged and the model's 4-byte mode off and bank register 0 throughout. Renode's reboot
  resets the flash chip, the arm's does not: the store leaves no chip state, so nothing depends on that. The
  test reboots before the fill and the wipe: an emulated arm sometimes resets by itself after minutes of
  uptime with no fault or health record (also 0.5.54 idle, once at 165 s under host load; twice at 511 s).

### What 0.5.56 adds (the panel guard's budget at 100 mA)

- `CONFIG_G1_PANEL_BUDGET_MA` 120 → 100 after the 10 min acceptance run of 0.5.54 (2026-11-04 afternoon, charged
  arms; `docs/power.md` §17 in the development tree): the cap held steady (one change after the first minute) with
  the mean current at 94 % of the budget, but the right arm's pack rose 6.7 °C in 12 min of white (limit 5 °C);
  `docs/review-dimming.md` points 9 and 14: lower to 100 on a fail. DIAG 17 `GUARD` also reports the budget, and the
  emulator test scales its currents to it.

### What 0.5.54 adds (the panel guard on the mean current)

- The 0.5.52 acceptance run found the cap sawing ±1 level every ~5 s: the loop acted on one IBAT conversion per
  second, and the panel's current is pulsed (single conversions 2–140 mA at one level). Follow-up review
  (`docs/review-dimming.md` points 12–17 in the development tree): while the panel is on, each PMIC round adds a
  burst of 16 VBAT+IBAT conversions at unequal spacings over `CONFIG_G1_GUARD_BURST_MS` (40) or more (`pmic.c`
  `burst`, the bus held meanwhile); the round's current is their mean, its voltage their minimum. `guard.c`: I_avg,
  an exponential average of the rounds' means (α = 1/4); an attack at most once per 4 s, in one step to the highest
  level whose full-white current (the measured table) times the content factor I_avg / I_white(level shown) is within
  95 % of the budget, after which I_avg is rescaled to the new level; release +1 per 5 s at ≤ 85 %, +3 at ≤ 50 %; the
  start value 42 at a rest voltage ≥ 3600 mV, else 24; the cap 42 while the panel is off or in standby. The `power:`
  line has the lowest burst VBAT, the burst mean and I_avg.
- DIAG 16 `ADC_BURST` (n conversions with their times, for the pulse record) and 17 `GUARD` (the guard's state);
  `g1host.py adc-record`, `Arm.guard_state()`.
- Tests: emulator `panel_guard` rewritten (40/140 mA per round: no step; 100/160: one table step within 8 s and none
  within 4 s after; one 300 mA round among 60 mA ones: nothing; +1 and +3 releases; VBAT, temperature, the start
  values, off and standby reading 42, `BRIGHTNESS_MAX`, `PANEL_GUARD` 0, VBUS).
- Gate `all` with both update chains from 0.5.52: 51/51. On the arms since 2026-11-04 04:18 (left) and 04:20 (right).
  The panel's current pulse measured with DIAG 16: a period of ~10.4 ms (`docs/power.md` §17 in the development tree),
  so the 40 ms burst covers about four. The 10 min acceptance rerun waits for charged arms.

### What 0.5.52 adds (the panel guard)

- `src/guard.c`, run by the PMIC thread once per second after its sample: while the panel is on and there is no
  VBUS, `BRIGHTNESS_CAP` keeps IBAT under `CONFIG_G1_PANEL_BUDGET_MA` (Kconfig, 80…150, default 120) and VBAT off the
  nPM1300's 2.8 V POF: over the budget the panel's current register (level − 6) is scaled by budget / IBAT, VBAT
  < 3150 mV takes 2 levels, < 3000 mV halves the register, `BATT_TEMP` ≥ 38.0 °C counts as over the budget; one step
  per second at most, floor level 6; one level up per 5 s when the whole 5 s had IBAT ≤ 85 % of the budget, VBAT ≥
  3400 mV and < 36 °C. The cap at each panel power-on comes from the last VBAT sample with the panel off or in standby
  (≥ 3650 mV 42, ≥ 3550 mV 30, else 21); 42 while charging. VM.md §6.6; `docs/review-dimming.md` in the development
  tree (the design review, 11 points).
- `panel.c` keeps `BRIGHTNESS` (as written), `BRIGHTNESS_MAX` and the cap and drives the lowest; the PMIC thread sets
  the cap under the panel lock with a 200 ms timeout (else the next second). `AUTO_BRIGHTNESS` and the arm-to-arm
  link use the written level.
- Protocol: `BRIGHTNESS_MAX` (0x29, R/W 0…42, a sensor setting), `BRIGHTNESS_CAP` (0x2A, R), `NOTICE` 7 (cap, level
  shown) on each cap change, `CTRL` 0x16 `PANEL_GUARD` (hosts only). The `power:` line has the cap and the level
  shown; each step down is logged with VBAT, IBAT, `BATT_TEMP`, the cap and the written level (the first 8, then
  every 100th). The library matches the eyes (`SCOUTER_G1_MATCH_BRIGHTNESS`, `src/README.md`).
- Tests: emulator `panel_guard` (scripted VBAT, IBAT and temperature: attack within 2 s, hold, release at 5 s,
  the 85 % rule, the VBAT and temperature steps, start values, `BRIGHTNESS_MAX`, `PANEL_GUARD` 0, VBUS); `g1_test`
  `TestBrightnessMatch` (two emulated arms through the library, also a link drop); `vm_test` (the core's range
  check, 0x2A read-only). The library gives an eye 42 again when the capped eye's link drops and at `Deinit`.
- Gate `all` with both update chains from 0.5.50: 50/51, `disconnect` passed on rerun. On the arms since
  2026-11-04 03:07 (left) and 03:08 (right). Acceptance (`docs/power.md` §17 in the development tree): no reset,
  VBAT ≥ 3452 mV, pack +2.4–2.8 °C in 4–4.5 min of white at level 42, the eyes matched; the cap is not stable
  (±1 level every ~5 s): the loop's one IBAT sample per second scatters 2–140 mA on the pulsed panel current, and
  the mean current stays ~60–70 mA. Budget kept at 120; a mean of several samples is proposed to the reviewer.

### What 0.5.50 adds (a kick counts once the PMIC answers; the panel read-back's slow retry)

- A watchdog kick counts only when NTCCOLD (charger page 0x03, offset 0x10, a plain register `configure()` writes)
  reads back as it read right after `configure()` (0 there: no check). While a device holds SDA low, the TWIM sees every address and byte ACKed and reads 0, so a kick could
  succeed without reaching the PMIC, and 0.5.48 would not notice before the PMIC's power cycle; now that is a failed
  kick (logged with the value read, "check 0x00"), recovered and, after 20 s, a warm reboot. The bus recovery holds
  the PMIC's mutex: Zephyr's TWIM recovery does not wait for a transfer in progress.
- PANEL_READ: when a row's three reads all differ, two more at 8 MHz instead of 32 decide (logged). The left arm's
  first read-back after a cold power-on failed so twice on 2026-11-04 (row 44 read three ways, two attempts 50 ms
  apart); `docs/fps-link.md` §10 in the development tree.
- Gate `all` with both update chains from 0.5.48: 49/50, `unattended` passed on rerun. On the arms since 2026-11-04
  01:54 (left) and 01:55 (right).

### What 0.5.48 adds (the PMIC watchdog kick checked; battery voltage while the panel is on)

- The kick of the nPM1300 watchdog runs first in the PMIC loop (`watchdog_tick` in `src/pmic.c`, every 5 s). A
  failed kick is logged with the age of the last good one, the PMIC's I2C error count, the panel's state and the light
  sensor's (the right arm's OPT3007 shares TWIM1 and is powered with the panel's rails); a second failure in a row
  runs `i2c_recover_bus` first; 20 s without a good kick end in a warm reboot with health reason 7, which keeps the
  retained log (the PMIC's power cycle after 30 s would lose it). The PMIC's failed transfers are logged (the first
  8, then every 100th), `sample()` stops at its first failed read, and the bus is recovered while the PMIC does not
  answer at boot.
- While the panel is on, a `power:` line every 10 s: VBAT, its lowest 1 s sample since the last line, IBAT, and the
  light sensor's state, readings and errors.
- Why: the right arm reset twice under VIDEO on 0.5.44 with `RESETREAS` 0 and its retained RAM lost (a power cycle);
  `docs/fps-link.md` §9–10 in the development tree. On the arms since 2026-11-04 00:51 (left) and 00:54 (right).

### What 0.5.46 adds (the nPM1300's reset cause in the log)

- At boot `pmic.c` reads the nPM1300's error log (product specification, "Reset and error registers", base
  0x0E00): `RSTCAUSE` (0x03: bit 0 ship mode exit, 1 boot monitor, 2 watchdog, 3 long press, 4 thermal shutdown,
  5 VSYS low, 6 software reset), `CHARGERERRREASON` (0x04) and `CHARGERERRSENSOR` (0x05), logs them ("PMIC reset
  cause …") and, when any is set, clears them with `TASKCLRERRLOG` (0x00 = 1). No other PMIC register is written.
  Why: on 0.5.44 the right arm reset abruptly under VIDEO with `RESETREAS` 0 and its retained RAM gone (a power-on
  or brown-out reset), which the SoC alone cannot tell apart; the PMIC keeps its record across the SoC's power cycle.
  Register sources: Nordic's online nPM1300 specification (offsets) and Nordic's npmx `adk/npm1300.h` at commit
  4da2753 (bit fields); `docs/fps-link.md` §9 in the development tree.

### What 0.5.44 adds (the Bluetooth-liveness check reads in order; its diagnostics)

- Health check 4 (Bluetooth liveness, `src/health.c`) rebooted the left arm twice on 0.5.42 during VIDEO over hci1
  while its link carried traffic up to the reboot (btmon: CMD writes accepted and executed, NOTIFYs, until the
  reset; `docs/fps-link.md` §9 in the development tree): a false positive. The check read `now` before `bt_ok_ms`,
  so a CMD write handled in between by a higher-priority thread could store a later time and the unsigned
  `now - bt_ok_ms` wrapped. `bt_ok_ms` is now read first, in the check, the HCI probe and the boot-counter clear.
- The check logs its time and both readings when it fires; and it logs, at most 8 times each, a `bt_ok_ms`
  later than the check's `now` and an uptime that went back or moved more than 3 s between two checks.
- On the arms since 2026-11-03 14:29 (left) and 14:31 (right); gate `all` with both chains from 0.5.42: 50/50.

### What 0.5.42 adds (a faster interpreter, VIDEO; the DK bench)

- **The VM runs host code and programs ~2.2× faster**: 5.5 µs per simple instruction instead of 12.0 on the
  nRF5340 DK at 64 MHz (`go/examples/linkbench -mode speed`, `docs/fps-link.md` in the development tree), a FILL of
  8 pixels 12.8 µs instead of ~20. DWT cycle counts and J-Link PC samples showed the cost: ~830–1000 cycles per
  instruction, a quarter of it the clock read before every instruction (`now_us()`: the kernel's timer lock and
  64-bit RTC read, ~160 cycles), 14 % the 16-trigger scan, the rest decode and dispatch. Changes, none of them
  visible to a host: `h_now` (`src/exec.c`) reuses its last value while RTC1's counter (the kernel's 30.5 µs tick)
  has not moved, which yields the same value (exec thread only, cleared after each `vm_run`); the VM core
  (`../src/vm/vm.c`) reads the clock in `vm_run` only where it is used (a program's armed timer, a sleeping or
  missing packet, the program task) and `SLEEP_UNTIL` reads it itself, and skips the trigger scan when `poll`
  returned none; `vm.c` and `vm_gfx.c` are compiled with `-O2` in the firmware (+2.3 KB).
  The core's tests pass and its 400 replayable vectors (`vm_test vectors`, seed 11) are byte-identical to 0.5.40's.
- The exec thread's health beat (3 s limit) is fed on one RTC1 compare (every 250 ms) from `h_poll` (every 16th
  instruction) and from `h_now` on the exec thread (a long instruction that reads the clock), no longer from
  `now_us()` alone: host code may now run for seconds without reading the clock. Found on the DK: 999 loops of
  1000 ADDs (5.6 s, no sleep, no clock read) rebooted it (health check 1) when only `h_now` fed the beat.
- Design review (`docs/review-fps-apps.md` §3 in the development tree): accepted with these two changes (the beat
  in `h_now`; the VM README's note that `poll` and `now` together, not `now` alone, can be a liveness signal).
- **`CONFIG_G1_DK_NCS_NET`** (DK builds only): the DK's net core runs NCS's `hci_rpmsg` (the child image of the DK
  build, `hci_rpmsg/zephyr/merged_CPUNET.hex`), so g1v0 skips the stock handshake and has no ESB. With Even's net
  core and no ESB partner a DK drops 7.5 ms links within seconds (0x28, 0x08); with NCS's controller (the same SDC
  build) they hold, which makes the DK a throughput test bed. Never flash a DK build without this option onto a DK
  whose net core runs `hci_rpmsg`: its handshake fails and the net-core fallback would follow.
- **`VIDEO`** (0x55, VM.md §3.7; the development tree's `docs/fps-codec.md`): decodes a frame record into a 1-bpp
  arena bitmap and writes only the changed part of each changed row to the panel, with one `97`. The decoder is the
  shared VM's (`src/vm/vm_gfx.c`), the encoder the library's (`VideoEncoder`). Bad Apple at 480 × 360: 261 bytes
  per frame (row runs: 1666), ~9 ms of panel per frame instead of a 27 ms DRAW. Design review
  (`docs/review-fps-apps.md` §1 in the development tree): accepted with its changes 1.1–1.4 (below).
- `panel_rect_span`: a part of a row as its own window inside a begun rectangle, pipelined like the rows of a
  narrow `DRAW` (an even width takes the extra one-pixel window, so the VM sends odd widths); the executor sets
  `vm_host.panel_span` to it. The rows of a `DRAW` and the spans go into the stage buffers with word copies
  (picolibc's `memcpy` is a byte loop at `-Os`).
- Review changes: the core calls `now` every 16 rows inside a `VIDEO` (the health beat of `h_now` above; a busy
  640 × 480 record takes ~0.4 s in the emulator, 0.9 s on the arms), out-of-range sizes are errors in the encoder's four APIs, VM.md
  §3.7 states the bound (w + 1 symbols per row, 7 decisions per pixel) and that nothing outside the arena keeps state.
  Emulator test `video` (in the standard set): five worst-case 640 × 480 records back to back (`tools/draw_bench.py
  --video`, also for the arms), `VM_STATUS` answered meanwhile, no reboot, panel and arena as the mirror's.
- 0.5.42 against 0.5.40: RAM +5.2 KB (`struct vm`'s rows for `VIDEO`), flash +7.8 KB (with the `-O2` above). Gate
  `all` with both update chains from 0.5.40: 49/50, `restore` passed on rerun (it timed out under 4 parallel jobs).

### What 0.5.40 adds (the worn head map)

- `measured_map` in `imu.c` holds the worn two-pose result of `../tools/imu_cal.py` (2026-11-01: level, then 23°
  down; `logs/imu-cal/1/` in the development tree): the provisional map's x axis was mirrored on both arms
  (determinant −1) and each arm sits splayed ~16°, so looking down leaked into roll (±6–7° per 23°) and the level
  pose read roll 7° (left) and −5° (right). With the measured map both arms read level 0/0 and the same angles worn.
  The factory pitch base is still read (DIAG IMU) but no longer used; an all-zero table brings the provisional map
  back. Gyroscope rates follow the map's determinant, now +1.
- Tests: `imu_base`, `imu_base_left`, `imu_base_default_left` check the measured map's level and pitched poses when
  it is in force; `imu` accepts PITCH and ROLL from the spec's formula of the reported `ACC_*` where 1 mg matters
  (pitch at roll 89° from `ACC_Y` = 17 mg).
- On the arms since 2026-11-02 07:49 (left) and 07:51 (right); gate `all` with both chains from 0.5.38.

### What 0.5.38 adds (a top-up survives lost polls)

- **`caselink_charging` tolerates lost polls** (`src/caselink.c`): a poll continues the run if it comes within 4.5 s
  of the last one (a lost poll leaves ~2 s, two ~4 s), and the case counts as charging until 6 s after the last poll.
  On 0.5.36 the right arm's hold (with `CASE_PANEL` 1 and gifplay drawing) ended and started again 8 times in one
  top-up: each lost poll's gap (> 1.5 s) restarted the run, the panel lit up again (the kept `PANEL` 1) and the next
  polls failed too (~5 % lost against ~1.5 % on 0.5.34).
- **Test:** `case_blank` checks the hold 1.4 s after a run's last poll and its end 7 s later.

### What 0.5.36 adds (fewer reads of the tag)

- **The case link reads the ST25DV less** (`src/caselink.c` `service`, `mailbox`, `field`): after a GPO pulse it waits
  2 ms (the pulse marks the start of the case's RF command, during which the tag NACKs I2C) and reads only `IT_STS_Dyn`;
  the mailbox only when `RF_PUT_MSG` says a message came, the field only when a field bit changed; everything at the
  1 s look (a lost pulse). Before, every pulse read IT_STS, MB_CTRL and EH_CTRL at once. Why: on 0.5.34 the right arm
  saw 3444 field changes and retried 133 tag reads in 40 min against the left's 464 and 2, and it loses 1–3 % of the
  case's polls in a top-up even with its panel off (the left 0.1 %); a read that overlaps the case's next RF command
  makes that command fail.
- **The LED blanking only around the case's RF pulses** (2 s after the last), no longer for VBUS alone: on the arm a
  top-up brings polls and with them the hold, a re-check brings pulses; in the emulator VBUS is always present, so the
  blanking toggled at every microphone start and stop there.
- **Tests:** `in_case`'s lift bound 2.0–6 s (the last field read now comes from the 1 s look); `case_blank` for the
  pulse-only blanking (none with VBUS alone). `mic_tool_draw` fails now and then on every version (1 of 6 runs on
  0.5.30, 1–2 of 5 on 0.5.34–0.5.36): the emulated host misses the microphone's lease renewal under the draw traffic
  (the arm stops it, reason 3); it passes on rerun.

### What 0.5.34 adds (the case hold also during a top-up; no QSPI deep power-down)

- **`CASE_PANEL` no longer covers a top-up** (`src/exec.c` `case_hold`, `src/caselink.c` `caselink_charging`; VM.md
  §6.6): on 0.5.32 the right arm, with `CASE_PANEL` 1 and gifplay drawing, lost 6 % of the case's polls during its
  top-up (VRECT normal: the LEDs were off) while the left, held by the policy, lost 0.1 %; the panel's SPI traffic and
  rails still disturbed the link. Now the hold applies with `CASE_PANEL` too while the case charges the arm: its polls
  come every ~650 ms (a full arm's re-check is a single one), so from the third poll in a row (less than 1.5 s apart)
  until 3 s after the last. The LED blanking stays for the re-checks (2 s after any RF pulse).
- **The QSPI deep power-down switch is refused** (`src/idle.c`, power.md §15): on the left arm the driver's resume
  failed (−EIO), the flash stayed asleep and the next boot read no factory record (VTERM 4.20 V, the default ESB
  channel) until a PMIC power cycle (FAULT 5).
- The net-off measurement's health deadline allows 10 s per phase and 2 min more (the masked run of 0.5.32 ended on it).
- **Tests:** `case_blank` (the hold during a run of polls with `CASE_PANEL` 1, its `NOTICE` 6 both ways, the arm kept
  folded so `IN_CASE` stays 1 without VBUS), `diag` (the deep power-down refused, the flash model awake).

### What 0.5.32 adds (the case link under load; the case hold; the idle bisect's switches)

- **The cause of the lost case polls** (`docs/power.md` §15): not the arm's latency. In the final regression's case log
  (`logs/final/case.jsonl`, 6886 polls) no message ever waited unread: at the case's first look, 75 ms after its write,
  the reply was there every time (`tools/case_polls.py`: reply 103 ms after the write, the case's own schedule). The
  lost polls are failures of the NFC link (the case's write of its message, its reads of the dynamic register or of the
  reply, "card lost"), and they come from the lit pixels: on the left arm in a top-up, 60 s each (`tools/case_load.py`,
  `logs/caseheat/01-…`): panel off 0 % lost, Bluetooth uploads at full speed with the panel off 0 %, the panel on and
  black 0 %, one bright full-frame image left standing 52 %, a changing 348 × 197 GIF-like DRAW at 10 fps 23 %. The
  arm-side counters agreed (every request it received answered). The case reported VRECT 4.2–4.7 V with the bright image
  instead of 5.4–6.0 V: the LED current loads the case's power transfer until the tag's RF side fails. So the polled
  SPI, the executor's priority, the I²C and GPO latency and critical sections are not it, and no priority change fixes
  it.
- **The case hold** (VM.md §6.6, §10 decision 12; `src/exec.c` `case_hold`): while `IN_CASE` is 1 and no host sent
  `CASE_PANEL` 1, the panel is off; `h_panel_begin` returns "no panel" (the VM core then draws nothing, no error) and
  sets the kept `PANEL` to 1; `PANEL` writes are kept (`held`, shared with the microphone's hold, which cannot be active
  at the same time); the end applies the kept value; `NOTICE` 6 (1/0) at both ends; the microphone stops at the start
  (`NOTICE` 5 reason 4) and `MIC` answers 11 meanwhile. The PMIC thread (an `IN_CASE` change) and the control thread
  (`CASE_PANEL`, a disconnection) post `R_HOLD` to the executor, which also checks it once per loop.
- **`CTRL 0x15 CASE_PANEL`** (`src/proto.c`): 0 at boot and when the arm leaves the case (`proto_vbus_changed`); a
  disconnection and `RESET_SESSION` keep it (so a program left running in the case stays visible, and the unattended
  rule's test still holds; clearing it at every disconnection held a running program's panel off). `g1host.py` sends
  1 after every HELLO to an emulated arm (always in the case, no pairing there) or with `--case-panel` (until
  2026-10-28 the environment variable `SCOUTER_G1_CASE_PANEL=1`, for the library too);
  command `case-panel [0|1]`; it clears its mirror's panel at `NOTICE` 6 1 (a hold's start), not at its end (that
  raced with a test's first DRAW). The library (`src/g1`) sends 1 with `Connection::case_panel` and 0 without it at
  every session setup (no app inherits a development setting), and redraws an arm's panel from its mirror at
  `NOTICE` 6 0 and when a kept session finds the panel off while the mirror shows it (`Core::RedrawPanel`, with the
  panel half of `Restore`, now `PanelFromMirror`); a `NOTICE` 5 with reason 4 is reported as `MIC_STOP_PANEL_OFF`.
- **The LEDs off while the case powers the arm and `CASE_PANEL` is 1** (`src/caselink.c` `window_step`,
  `src/panel.c` `panel_blank`; datasheet 7.3.7–7.3.8: `A9` drives no pixel and keeps the cache, `A3` + `97` shows it
  again; the blank state is kept over power-ons, `brightness()` leaves the display disabled): blanked while VBUS is
  present (a top-up) and for 2 s after any GPO pulse (the case's re-check of a full arm, before its VBUS sample);
  not while the microphone is on. The mailbox is serviced before the blanking takes the panel's lock, so a reply
  never waits for a DRAW. A first version blanked only around each poll (predicted from the 650 ms period, emulator
  test passing): it fixed the polls but left the panel's load on the case during the power transfer, which is what
  made the left arm send 0x80 and the case restart. The caselink thread no longer reads the tag at every wake: only
  at a GPO pulse, a VBUS change and the 1 s (in the case) / 30 s look.
- **DIAG 14 `CASE`** grows (offsets after the 42 bytes of 0.5.26): 42 u32 GPO pulses, 46 u32 the tag's I²C retries, 50
  u32 messages found by the 1 s look (no pulse), 54 u32 battery replies timed, 58 u32 of them later than 50 ms after the
  pulse, 62 u32 the longest µs, 66 u32 the average µs, 70 8 × u32 `IT_STS_Dyn` bit counts (RF_USER, RF_ACTIVITY,
  RF_INTERRUPT, FIELD_FALLING, FIELD_RISING, RF_PUT_MSG, RF_GET_MSG, RF_WRITE), 102 u8 the case hold, 103 u8
  `CASE_PANEL`, 104 u32 DRAWs not drawn in the hold, 108 u16 the last VRECT mV, 110 u32 blanks, 114 u32 of them begun by
  an RF pulse without VBUS, 118 u32 seconds blanked, 122 u8 blanked now (`g1host.py` `case_stats`). **DIAG 15
  `CASE_TRACE`** (read only): the last 32 case-link events, 6 bytes each (u32 ms of uptime, u8 kind: 1 GPO pulse, 2
  `IT_STS_Dyn`, 3 reply (its status), 4 blank, 5 unblank (value: VBUS); u8 value) (`case_trace`).
- **The idle bisect's switches** (`src/idle.c`, test builds, RAM only, all off at boot; `docs/power.md` §15): DIAG SET 7
  value = a mask (exactly those on; reply byte: the mask in force; `g1host.py` `idle_switches`): bit 0 the QSPI flash in
  deep power-down (`pm_device_action_run` SUSPEND/RESUME of the flash, hence `CONFIG_PM_DEVICE=y`; nothing suspends a
  device otherwise), 1 pulls on the idle QSPI lines (SCK, IO0, IO1 down, IO2, IO3 up), 2 the console's TX not driven
  (log output lost meanwhile), 3 stock's P0.19/21/23 not driven low, 4 the panel's −2 V enable P0.24 not driven (panel
  off only), 5 the IMU powered down and the TouchBar controller in deep sleep. DIAG 6 `NET_OFF` takes an optional mask per
  phase (up to 10 phases, `net_off(s, phases, masks)`), all switches off at its end.
- **Tests:** `case_hold`, `case_blank` (both in the standard gate), `diag` with the switches (the flash model's
  deep power-down, an SMP image list after it, a masked net-off run).

### What 0.5.30 adds (panel standby without the floating MISO input)

- **`PANEL` = 2 cost about 1 mA, not 0.2** (M11, `docs/power.md` §14): paired against off on 0.5.28, the left arm
  read +0.99 ± 0.26 mA (4 pairs) and the right +0.85 ± 0.27 (as read), single pairs from +0.36 to +3.0 mA; DIAG SET 6 2,
  0.5.14's measured standby, read the same (+0.51, +0.77), so the 0.2 mA of 0.5.14 (two unpaired rounds, +0.42 and
  −0.01) was low. DIAG REGS showed the same pins in all three standby paths: SCK, MOSI and CS driven, **MISO an input
  with its buffer connected and no pull**, while the panel leaves it floating with CS high (the probe of
  `arm-rollout.md` §10). A floating CMOS input draws a current that depends on where the line drifts, which fits the
  scatter.
- `src/panel.c`: the standby disconnects MISO's input buffer after the SPIM is disabled and connects it again before
  the wake's first transfer (`miso_connect`); nothing else changes (the panel's commands, rails, VCOM, the wake time).
- **Tests:** the standard gate and `all` (47/47, both update chains from 0.5.28, `restore`); the emulator's panel model
  does not see pin configurations. On the arms: P0.10 reads `0x302` in standby (buffer disconnected), the wake from
  standby still takes 2–3 ms and from off 71–72 ms (`draw_bench.py`), drawtest 40/40.
- **Result: the floating input was not the cause.** On 0.5.30 standby still costs +0.95 ± 0.47 mA (4 pairs) and
  +0.98 / +1.70 mA in two 5 min blocks against off (left; steady within a block, different between entries), so the
  extra current is the panel's own deep power-down with its rails on (or the rails), which g1v0 cannot change without
  the panel's datasheet. The change stays (an input left floating is wrong anyway). Consequences, decided here: the
  documented standby cost is ≈ 1 mA (VM.md §2.5, §6.6), and the g1 notification app hides with `PANEL` = 0 by
  default (`-standby` opts in; popups light up about 70 ms later from off, still within A1's 300 ms) — for the
  reviewer: review-v5.md §10.7 chose standby for the app on the strength of 0.2 mA.

### What 0.5.28 adds (a self-verifying `PANEL_READ`)

- **The first `g1 mirror -check` after an install failed one left-panel read-back** (rows 30–129, the first popup), and
  the second run passed: after the 0.5.14, 0.5.24 and 0.5.26 installs, each time while the case was charging or
  re-checking the arms (VBUS pulses). Found on the arms (`logs/m9/03-first-draw-left.*`, `logs/standby/11-…`, `12-…`):
  the left panel's content is right on every path a first popup takes (a fresh boot and `PANEL` = 2 from off, standby
  entered from on, a fresh boot and `PANEL` = 1; each followed by a DRAW: rows 30–129 read back equal as a whole and row
  by row, twice), and the app's first check passed after a plain left reboot and right after a drawtest, with the case
  quiet. The app's program reports the popup only after its DRAW, its band is filled anew each time and the library
  clears the arena at every session, so both runs draw the same bytes. What the failures share is the case's charging
  pulses, and the left panel's SPI reads are known to fail during them (its ID read 0x0000 twice after a reboot,
  0.5.4's retry): the read-back, not the panel, was disturbed (likely; the read and the write cannot be told apart
  after the fact).
- So `PANEL_READ` reads each row twice, and a third time when the two differ, and returns the bytes two reads agree
  on, with a log line ("panel read-back: row …"); what every read agrees on (a panel that really shows other content)
  is still reported as it is. 100 rows take about 16 ms instead of 8. `demo.CheckPanel` (the Go examples' read-back
  check) now names the rows that differ when it fails, so a recurrence shows whether whole rows are wrong.
- **Test:** `panel_read` (in the standard gate): a band drawn by host code reads back equal; with the panel model
  inverting 3 bytes of the next read (`FaultReadNoise`, this project's patch) it still does, the row reread and logged;
  with every read inverted it reads back different. 0.5.26 fails it. `peer` waits for the arm's own classification
  (DIAG `CASE` wear 0) before it takes the arm out of the case (it had left with the "below 40" timer still running).
- **The `unattended` flake** (the emulated net core stops answering after the test's second link drop: the virtual
  phone cannot reconnect, and 10 s later `bt_probe`'s HCI round trip times out and Zephyr's BT assert reboots the arm,
  `logs`: "Kernel oops … (bt_probe)" at 30 or 40 s of uptime) came in 1 of 4 runs on 0.5.26 and 2 of 6 on 0.5.28,
  against 0 of 4 on 0.5.24; it passes on rerun. Possibly the in-case accelerometer stream (0.5.26) shifts the
  emulator's interleaving; the arms show no unhealthy boot.

### What 0.5.26 adds (M9: the case link both ways, `IN_CASE` as a state; the right-handed head frame)

- **`IN_CASE` is a state** (`src/pmic.c`, VM.md §2.5, review-v5.md §10.10): set by VBUS (sampled every second), a case
  message or the tag's RF field (`EH_CTRL_Dyn` FIELD_ON, read by the case-link thread every second while in the case,
  every 30 s otherwise); cleared 3 s after the last of them when the arm moved more than 150 mg between two samples
  (every accelerometer sample is read while in the case: the FIFO path `MOTION` uses, one read a second) or was
  classified as worn since entering the case, or `G1_IN_CASE_HOLD_S` (400; 40 in the emulator test build) after the
  last of them. So a full arm lying in the case stays in it, and the case's VBUS pulses no longer flicker it.
- **`WORN`** (`src/touch.c`): the classification (link value, controller) is kept apart from the property: `WORN` reads 0
  in the case and after it until the next classification, and a classification as worn made since entering the case
  lets `IN_CASE` go.
- **The case link** (`src/caselink.c`, VM.md §6.8): the charger is enabled after each battery reply with status 0 or 4,
  as stock's `process_pt_data` does, instead of every 5 s; the case's restart request (`0B`) is answered as stock and
  logged, not obeyed (before, g1v0 rebooted as stock); the field and every message feed `IN_CASE`; the poll's bytes are
  read as stock (`get_battery_info`: bits 0–6 of byte 4 the case battery, bit 7 its charger, as stock forwards it to
  the phone in `F5 0E`/`F5 0F`; byte 5 the lid, 7 open, 8 or 6 closed). DIAG 14 `CASE`.
- **Head frame** (`src/imu.c`, VM.md §2.5, review-v5.md §10.9): x right, y up, z toward the wearer (right-handed); the
  provisional map's z row is negated and pitch = atan2(−acc_z, acc_y), so `PITCH` and `ROLL` read as before and
  `ACC_Z` changes sign; `GYR_*` keep the det(map) rule (the provisional map is not a rotation in the new frame:
  determinant −1). `tools/imu_cal.py` builds the frame the same way (z from the pose looking down, x = y × z).
- **Tests:** `in_case` (0.5.26: the state, the field, a lift, wear, the hold, the charger rule, the restart request,
  `CASE_*`), `touch`, `peer`, `security` and `imu` adapted (a lift to leave the case; streaming in the case), the ST25DV
  model's `RfField`; `common.lift`, `common.link_signal`.

### What 0.5.24 adds (M10: the arm-to-arm link; `WORN` from the link value)

- **The net core's IPC multiplexer** (`src/netcore.c`; review-v5.md §8.7, g1-fw `g1-netcore-ipc-protocol.md`): besides
  the role handshake, the `ipc0` endpoint now takes the stock net core's link report (`01 07`: ESB acknowledged,
  failed and received counts and the signal strength of the other arm's last frame), its delivery report (`01 0d
  02`) and the other arm's frames (`06` on the right arm, `04` on the left, 251 bytes), and sends frames for its
  single ESB slot (`02` + 32 bytes on the right, `04` + 32 on the left). What the net core does with them was read
  from the decompiled stock net core of the arms' build (2025-06-28) and checked in the emulator against Even's
  1.6.6 net core: one exchange every 50 ms (its 0x11/0x10 announcements), an app frame sent instead of the next
  announcement or returned in the next acknowledgements, frames starting 0x10/0x11 not forwarded, a left net core
  dropping a frame whose byte 17 repeats the previous one's. No net-core change.
- **The message link** (`src/peer.c`, VM.md §6.9): `PEER_LINK`, `PEER_SEND`, `PEER_MSG`, `PEER_T_US`, the `PEER`
  trigger (`HELLO.triggers` bit 9, 0xF3FF on the right arm). Frame: 0 marker 0x47, 1 flags (value, brightness, clock
  reference, acknowledgement, offset), 2 value seq, 3 acknowledged seq, 4 u32 value, 8 u32 write time, 12 u32 the
  right arm's report time of frame `ref`, 16 `ref`, 17 frame id, 18 the right arm's `BRIGHTNESS`, 19 session nonce,
  20 i32 the left's clock offset, 24 the nonce the acknowledgement refers to. One frame in flight per arm (the next
  waits for the delivery report, 300 ms at most: a lost report is retried, the net core refuses a frame while its
  slot is full); the right arm polls every 100 ms and sends at once on news, the left keeps alive every 200 ms and
  sends at once on its `PEER_SEND`; a value is repeated until acknowledged and delivered once; the clock offset is
  the median of the last 9 (left arrival − right report) pairs of the same frames; `PEER_T_US` = the sender's write
  time in the receiver's clock. `PEER_LINK` = 0 at boot and `RESET_SESSION`; a left arm's `AUTO_BRIGHTNESS` = 1
  follows the right arm's level (only while `PEER_LINK` is 1; `PEER_LINK` = 0 sets it back to 0).
- **`WORN` from the link value** (`src/touch.c`, VM.md §6.7), as stock's `check_work_mode`: 51 and above worn at once,
  below 40 for 3 s not worn, 0 no reading; the TouchBar controller's worn event sets 1 at once, its out-of-ear event
  counts only before the first report; 0 in the case as before. `WEAR` is in `HELLO.triggers` with the link alone.
- **DIAG 13 `PEER`** (above); `g1host.py` `peer_stats`; `tools/peer_check.py` (the on-arm check, the same programs
  as the emulator test).
- **Hosts:** the reference client's mirror (`tools/g1mirror.c`) and the library's take the arm's settings from
  `HELLO.triggers` (`GYRO_RATE` without the IMU, `AUTO_BRIGHTNESS` without the light sensor unless a left arm with the
  link while `PEER_LINK` is 1, `PEER_*` without the link, `PEER_SEND` while `PEER_LINK` is 0); `g1host.py` `Arm.ins`
  raises `Refused` and sends nothing for a write the arm would refuse (before, the client sent the left arm's
  `AUTO_BRIGHTNESS` and the arm ended the packet with error 5), the library already refused through its mirror
  (`g1_test` `TestArmSettings`; its emulated right arm now has the light sensor, `HELLO.triggers` 0xF073).
- **Tests:** `peer` (one arm against the virtual slave: frames, the brightness in them within 8 ms of a change, a
  value delivered once with `PEER_T_US` through the carried offset, acknowledgements, `PEER_LINK` 0/1/2, error 5
  for `PEER_SEND` while off, `RESET_SESSION`; `WORN` out of the case from the radio's signal strength: 60 worn 1.1 s
  after the report, 30 not worn 6.1 s later), `pair_link` (both arms: 20 values and echoes, the eye-sync recipe
  read from both panels' times, brightness following, link off), `pair_stream` (values echoed one after another;
  M10's acceptance with 1000: every value and echo once, no report timeout, 150 ms average round trip, 153 s
  virtual, 15 min); `touch` (the controller's out-of-ear event no longer clears `WORN`), `vm`
  (`PEER_LINK` reads 0).
- Emulator results (two machines, 20 µs quantum): right to left 6–89 ms (median 24–41), round trip 61–237 ms
  (median 136–171), the link's clock offset within 120–150 µs of the panels' (the true offset −2 µs), `PEER_T_US`
  within 149–215 µs, both panels changed within 60–260 µs of each other by the recipe (lead 200 ms), the left at
  the right's brightness 113–142 ms after the change. On the arms (in the case, `docs/power.md` §12, arm-rollout.md
  §0): 20/20 values, round trip 60–196 ms (median 110), right to left 10–93 ms, the two bands within 0.27 ms,
  the left at the right's brightness within 60 ms, `PEER_LINK` +0.09 ± 0.06 mA (left).

### What 0.5.20–0.5.22 add (found on the arms)

- **0.5.20, the light sensor on the panel's rails:** on the arms the OPT3007 answers only while the panel's rails are
  on (stock's `power_on_opt3007` powers the panel), so 0.5.18 never found it. It is now probed during the boot's
  panel check (`light_probe`), set up again whenever it starts converting, and set up again if a reading finds it
  reset (the rails went off and on in between). Only the right arm has one (stock sets it up only on the ESB master;
  the left NACKs at 0x45): the left reports no `LIGHT` trigger and refuses `AUTO_BRIGHTNESS` (VM.md §6.7).
- **0.5.22, `WORN`:** 0 from the arm's time in the case on, until the controller classifies it as worn; 0.5.18–0.5.20
  went back to `UNKNOWN` whenever `IN_CASE` dropped, so `WEAR` fired every few seconds while the case charged the arms
  (~50 changes in 10 min, `logs/m7/07-…`).
- **Hosts:** the reference client's mirror (`tools/g1host.py`, `g1mirror.c`) and the library's take an arm's missing
  IMU or light sensor from `HELLO.triggers` and refuse `GYRO_RATE` / `AUTO_BRIGHTNESS` there as the arm does; a mirror
  that accepted them diverged from the left arm after its error 5 (an error ends the packet, so the mirror ran pool
  writes the arm skipped).

### What 0.5.18 adds (M7: the TouchBar and wear; M8: light, auto-brightness, gyroscope)

- **TouchBar** (`src/touch.c`, VM.md §6.7, review-v5.md §8.1), written from the register protocol (16-bit
  register addresses and 32-bit values, big endian; no vendor code): at boot, as stock 1.6.6, the chip ID (0xFF00
  ∈ 0x2022320{3,5,7}), the wake writes (0xFF20, 0x4700, 0x4444, 0xFF18, 0x4700), the ready handshake (0x4410 bit 0
  within 100 ms), the controller's firmware version (0x1BA8 = 0x06000007, else it stays unconfigured: g1v0 never
  updates the controller's firmware), stock's 44-entry table, the commit (0x4408 = 1) and the event enable 0x4414 =
  stock's 0x40F plus the wear bits 7 and 8 (0x58F). The table is not in this tree: `scripts/fetch_stock.sh 1.6.6`
  takes it from Even's 1.6.6 image (app_update.bin offset 0x81B58 = flash 0x8DB58, 352 bytes, checked by its own
  SHA-256 in `stock_images.txt`) into `work/fw/1.6.6_aw9320x_table.bin`, and `src/touchcfg.S` embeds it.
  Interrupt P1.09 (active low, pull-up, an edge interrupt through a GPIOTE channel, armed only while the controller
  scans); each interrupt reads 0x4410 (read clears it) while the line stays low: bit 0 loads the table again, bits
  1/2 (touch down/up) feed the gesture classifier with stock's timings (taps closer than 305 ms, reported 305 ms
  after the last release, up to 3; a hold of 458 ms is a long press start, its release the end; a missing release
  ends it after 32 s), bits 8/7 set `WORN`. The controller scans (0x4408 = 1) only while a running program has a
  `TOUCH` or `WEAR` handler or waits for one (`vm_wants`, new in the VM core) or the `EVENTS` mask has them, and
  sleeps (0x4408 = 2) otherwise; `WORN` is 0 while `IN_CASE`, `UNKNOWN` while asleep. `HELLO.triggers` gains
  `WEAR` and `TOUCH` (0xF1FF with the light sensor).
- **Light** (`src/light.c`, VM.md §6.7, review §8.4): the OPT3007 on TWIM1 at 0x45 (manufacturer 0x5449, device
  0x3001), shut down at boot; continuous 800 ms conversions with the automatic range (stock's 0xCE10) only while the
  panel is on and a running program, `AUTO_BRIGHTNESS` or the `LIGHT` `EVENTS` bit uses it; a thread reads each
  conversion (the OPT3007 has no interrupt line) into `LIGHT` (0.01 lux = the result's mantissa << exponent) and
  wakes the executor, which evaluates the `LIGHT` range (trigger and `INPUT`). `AUTO_BRIGHTNESS` = 1 applies stock's
  table when two readings in a row ask for a level 2 or more away from the current one.
- **Gyroscope** (`src/imu.c`): `GYRO_RATE` 13/26/52/104 Hz, ±500 dps (17.5 mdps per LSB), low-power mode below 104
  Hz, read on demand (at most one period old, `UNKNOWN` for 70 ms + 2 periods after it starts); `GYR_X/Y/Z` in mdps
  through the same head map as `ACC_*` (with the map's determinant, so a mirroring map keeps the right-hand rule).
- **VM core** (`../src/vm`): `struct vm_host` gains `prop_write` (optional; `NULL` = error 5) for the properties the
  platform owns: `GYRO_RATE` and `AUTO_BRIGHTNESS` (the core checks the spec's values), later `PEER_LINK`,
  `PEER_SEND`; `vm_wants()`. The library's mirror and emulated arms, `tools/g1mirror.c` keep both settings.
  `GYRO_RATE` and `AUTO_BRIGHTNESS` go back to 0 at `RESET_SESSION` (sensor settings, VM.md §5.2).
- **Tests:** `touch` (boot state, asleep while unused, gestures 1–5 with their timing, `TOUCH_GESTURE`, a program's
  `TOUCH` handler, `WORN` in and out of the case, a controller reset, every event read once), `light` (off while the
  panel is off or unused, readings, the range `INPUT`, the auto-brightness table and hysteresis), `gyro` (rates
  through the head map within 2 %, error 5 for a rate not listed, off at 0 and `RESET_SESSION`); `vm_test` for
  `prop_write` and `vm_wants`.

### What 0.5.16 adds (the reviewer's decisions after 0.5.14: panel standby, the held `IN_CASE`)

- **`PANEL` = 2, standby** (VM.md §2.5, §6.6; review-v5.md §10.7): deep power-down with VCOM off and the rails on,
  the cache cleared first (`panel_power_standby`), ≈ +0.2 mA over off; the next `DRAW` wakes it in ≈ 4 ms
  (release, brightness registers, ID check) instead of the 71 ms power-on. From off it powers on first, so a later
  `DRAW` is still the fast wake. It reads 2; `PANEL_READ` answers 11 and `HELLO` shows the panel off. No automatic
  timer: the 30 s unattended rule turns standby fully off. While the microphone holds the panel, 2 blanks it to the
  indicator like 0 and is applied (standby) when the microphone stops. The VM core accepts 2 (`vm.c`), and the
  platforms model it: the library's mirror and emulated arms, `tools/g1mirror.c`, `g1host.py panel standby`.
- **Library** (`../src`): an eye whose whole panel is black after a `Flush` gets `PANEL` = 2 at the present's time T
  instead of black `DRAW`s; `MicStop` puts the panel back in standby if it was there before `MicStart`.
- **The g1 notification app** (`../go/examples/g1`): its program hides with `PANEL` = 2, so a popup wakes the panel
  in ≈ 4 ms: popups now show 65–167 ms after the bus delivered them (138 ms from standby on the 50 ms idle link),
  against 306–327 ms from off on 0.5.14 (`docs/power.md` §10, arm-rollout.md §0); the panel's ≈ 0.2 mA stays while
  the program runs.
- **`IN_CASE`** (VM.md §2.5, §6.1; review-v5.md §10.8): VBUS (sampled every second, held 3 s from the first sample
  without it) or a case message within 3 s (`g1_in_case()` in `pmic.c`), for the property, `HELLO` flags bit 7, the
  in-case security rules, bonding, the advertising interval and the case link's GPO interrupt. So the in-case rules
  last 3–4 s after the arm leaves the case. At "full" the case sends neither, so an arm lying in the full case reads 0.
- **Tests:** `panel_prop`, `in_case`, `mic_popup_standby` (in the standard gate), `restore` (Even's 1.6.6 installed by
  `flash.py` itself on the emulated arm; in `all`); the library's `g1_test` (standby at an all-black `Flush`).

### What 0.5.12–0.5.14 add (the microphone lease, a faster panel, an audit log)

- **MIC lease** (VM.md §6.5): a `MIC` 1 holds the microphone 2 s (`lease_ms` in the reply); a `MIC` 1 with the rate in
  force renews it. Without a renewal the arm switches the microphone off, clears the indicator and sends `NOTICE` 5
  reason 3, so a host killed with its link kept up (BlueZ holds it) cannot leave the microphone on. The library
  renews it whenever a `MIC_DATA` arrives and 0.8 s have passed (a host that stops reading lets it lapse);
  `tools/g1host.py` `mic_hold`. Tests: emulator `mic_lease` (off 2.007 s after the last renewal, reason 3, no event
  after, indicator cleared), the library's `tests/mic_test.cpp` `TestLease`, on the arm `tools/mic_lease_check.py`
  (a host killed by SIGKILL).
- **PANEL while the microphone is on** (0.5.14, the reviewer's decision, VM.md §6.5): a `PANEL` = 0 from a program
  or host code blanks the panel to the indicator (all pixels 0, the indicator drawn by the firmware, one latch)
  and is remembered: `PANEL` reads 0, a `DRAW` or `PANEL` = 1 cancels it, and the panel goes off when the
  microphone stops (`MIC` 0, lease, fault, link). It no longer stops the microphone; `NOTICE` 5 reason 1 means a
  panel fault. The VM core no longer clears its indicator state at `PANEL` = 0 (the platform decides).
- **Panel writes** (`src/panel.c`): a full-width `DRAW` goes out as windows of 8 rows (one transfer each, 2.6 KB)
  instead of a window per row, and transfers up to a row long are polled to their end instead of sleeping on the
  interrupt (no thread switch per row; 0.5.14: the SPIM's END event itself, with its interrupt off, and CS high
  without the 1 µs hold that only single commands need); the power-on clears the panel in the same 8-row windows, sends the
  current/display registers once instead of twice, and waits 6 ms instead of 15 after the panel's enable (the
  datasheet's power-to-SPI minimum is 5 ms) and nothing before its reset commands (the datasheet asks for none).
  The log reports each power-on's time (`panel on in N ms (rails, init, clear)`); DIAG SET 6 0xFF reads the last.
  Before/after on the arms: `docs/power.md` §10 and VM.md §3.5.
- **Panel standby** (DIAG SET 6, test builds, an experiment): deep power-down (B9) with the rails on, the cache
  cleared first, so the next power-on is a release, the brightness registers and a sync (~1–3 ms instead of a full
  init); `tools/panel_power.py` measures its current against off.
- **Audit log:** `VM_START`, `VM_STOP` (with the state, table and task count it stopped) and `RESET_SESSION` go to the
  arm's text log, and so does every `HALT` (trigger, address, op, code, detail), so a stopped or halted program
  leaves a readable reason (`LOG_READ`) even when no host heard the event.

### What 0.5.8–0.5.10 add (M5 firmware part: the microphone; the IMU axis map)

- **Microphone** (`src/mic.c`, VM.md §6.5, review-v5.md §4), on the right arm as stock: PDM0 with CLK P1.13 and DIN
  P1.14, 1.28 MHz with ratio 80 = 16 kHz mono, sampled on the falling edge, default gain, clocked from the HFXO (the
  values stock's Zephyr dmic driver computes for its 16 kHz, 1.0–3.5 MHz request; the firmware writes the
  registers itself). 10 ms DMA buffers (6 in a ring, a spare takes the samples while all are busy and counts them
  lost); a thread above the executor encodes each buffer with liblc3 into one 10 ms frame, 40 B (32 kbps, rate 0)
  or 20 B (16 kbps, rate 1), and `frames_per_event` consecutive frames make one `MIC_DATA` event (5 or 10 at MTU
  247). The events wait in a ring of 4 that the event thread sends; when it is full the oldest is dropped and the
  frame counter shows the gap. 0.5.10, after the codec review (`docs/audio-codec.md`: LC3 stays): at most 5 frames
  per event at both rates (50 ms of bundling; VM.md §6.5 changed accordingly), and the first 40 ms after each start
  are dropped (on the arm the first 30 ms
  were at full scale, then a DC step of about −22 dBFS decayed over ~1 s: an audible pop) and a first-order DC
  blocker (pole 0.995, 13 Hz) precedes the encoder; the counter of the first frame sent is 0. `MIC` 1 while on
  with another rate restarts the capture in that format (the reply's), counter from 0. Encoding: liblc3 1.1.3 (Google, Apache-2.0) vendored in `scouter_kit/third_party/liblc3`
  with its licence (from `github.com/google/liblc3` tag v1.1.3, tarball SHA-256 `276752ff…b8af`; only `src/`,
  `include/` and `LICENSE`), built `-O3 -ffast-math` with `LC3_PLUS=0`, `LC3_PLUS_HR=0`; the build now has
  `CONFIG_FPU` (hard float, shared). The image grows by ~100 KB (532 KB). Measured on the right arm (0.5.8, in the
  case): 100.0 frames/s by the arm's clock, no gap, no frame the host's decoder rejects; the encoder takes 2.5 ms
  per frame at 32 kbps (2.27 ms at 16 kbps; longest 5.5 ms), i.e. ~25 % of the 64 MHz app core; the codec review
  replaced review §4.1's 15 % rule by an energy budget (the encoder thread starves nothing: with display traffic on
  the same arm no frame was lost). `G1_MIC_128MHZ` (n) runs the core at 128 MHz while capturing: 1.23 ms per frame,
  but the encoder alone costs +1.03 mA there against +0.28 mA at 64 MHz (left arm), so it stays off. Current:
  about +3.5 mA with the microphone on (`docs/power.md` §9).
- **Privacy rules** (VM.md §6.5): only the `MIC` control request starts it (the executor applies it; no instruction,
  property or program can, and audio never reaches the arena); off at boot, at a disconnection and when the link
  stops being encrypted; `MIC` needs an encrypted link in the case too (the emulator build `G1_MIC_ANY_LINK` alone
  skips that). `MIC` 1 draws the indicator first (it powers the panel on) and starts the capture only if the panel is
  on; `MIC` 0 stops the capture first, then clears the indicator. A stop disables PDM0 at once, leaves P1.13 low and
  DIN disconnected, releases the HFXO and, under the encoder's lock, drops the events not yet sent, so no
  `MIC_DATA` is made after it. `PANEL` = 0 stops it before the panel goes off (`NOTICE` 5, reason 1); an encoder
  error, no samples for 250 ms or an HFXO that does not start stop it with `NOTICE` 5 reason 2 and clear `HELLO.board`
  bit 2 until the next boot. Bit 2 is set on the right arm only; the left arm refuses `MIC` with 12 (stock's firmware
  has no role check, but only the right arm's microphone was ever used; the boards are identical).
- **IMU axis map** (`src/imu.c` `head_map_init`, provisional): stock computes the head pitch from the sensor's X axis
  alone (a Mahony filter's pitch = asin(−a_x/|a|), in centidegrees; the 1.6.6 binary calls `asinf`, the decomp's
  label `acosf` is wrong), lower when looking up, against a per-unit "IMU pitch base" that its factory test stored
  at +0x0C of the read-only settings record (QSPI 0x130000; `set_imu_base`, `load_sys_setting`) and against which
  the head-up window is set (`imu_fusion_thread`'s thresholds). So X points forward along the arm. Folded in the
  case, both arms read the same magnitude on Y with opposite signs (the hinge axis: 337 / −335 mg), which with the
  same board on both sides makes Y up on the right and down on the left and Z inward on both; the other sign
  choice would need the temples folded past closed (~112°) to fit both arms' readings, this one fits with
  ~68° folds. The map: x = −Z (right) / +Z (left), y = sin e·X ± cos e·Y, z = cos e·X ∓ sin e·Y, with e = −base/100°,
  so stock's level pose reads `PITCH` 0. The right arm's record holds −1987 (X 19.9° above the level forward axis);
  the left arm's holds 0, stock's "no base" (its IMU was never used), and the glasses are rigid, so an arm with 0
  (or a value beyond ±45°) uses `G1_IMU_BASE_DEFAULT` = −1987 (0.5.10; 0.5.8 used 0 there). A measured map
  (`measured_map`, the rows `tools/imu_cal.py` prints from the worn two-pose check) replaces it per arm. DIAG
  `IMU` reports the record's base and the map in use.

### What 0.5.2–0.5.6 added (M3 firmware part, power work)

- **Accelerometer** (`src/imu.c`, LSM6DSO on TWIM2): ±2 g, low-power mode at 13, 26 and 52 Hz, normal mode at
  104 Hz, gyroscope off; `IMU_RATE` (13 at boot; 0 = off, values `UNKNOWN`); `ACC_X/Y/Z` (mg), `PITCH` and
  `ROLL` (VM.md §2.5's formulas, clamped to −90…90) at most one sample period old, read on demand. While
  `MOTION` or `TILT` can fire (a threshold or a range that can be left, and a running program or the
  trigger's `EVENTS` bit), every sample is handled: the chip batches them in its FIFO (BDR = `IMU_RATE`,
  watermark one second) and raises INT1 (P1.15 on both arms, found by `PIN_PROBE`) at the watermark or at
  once when its wake-up function sees a slope above min(2 LSB = 15.6 mg, `MOTION_MG`/(2√3)) — so any pair of
  samples further apart than `MOTION_MG` wakes the CPU, and the CPU sleeps while the head is still. Every
  sample is handled in order: `MOTION` (`vm_moved`, value = the distance) from consecutive samples, and each
  sample outside the `TILT` range (and the first back inside) is shown to the executor on its own. Reads of
  the properties while batching take the FIFO's newest sample, so a reading never runs ahead of the samples
  still to be handled. A board without INT1 polls `STATUS_REG` a little faster than the ODR. The head-frame
  axis map was the identity until 0.5.8 (above).
- **Triggers and `INPUT`:** `HELLO.triggers` 0xF06F (adds `MOTION`, `TILT`); `EVENTS` answers the request ∧
  `HELLO.triggers` and the core reports the masked triggers as `INPUT` events whether or not a program runs
  (`scouter_kit/src/vm` README); the mask is 0 at boot and after a disconnection.
- **Microphone surface** (review-v5.md §10): `HELLO.board` bit 2 clear and `MIC` 1 refused with 12 until capture
  existed (0.5.8).
- **Power:** the case link's GPO pin (P1.10) only in the case: out of it the pin is disconnected (the tag
  holds its open-drain GPO low on both arms, so the pull-up drew ~0.14 mA, and the edge interrupt held a
  GPIOTE IN channel); the "full" latch survives soft resets (retained RAM) and ends only after 30 s in a row
  below VTERM − 100 mV, so a reboot in the full case costs no top-up; the touch rail and LDSW2 stay on as
  stock (switching them off makes the idle current higher: the unpowered parts are fed through TWIM2).
- **Robustness:** a second panel power-on attempt after a failed ID check (the first DRAW after a long
  panel-off failed twice on the left arm during the case's short VBUS pulses).
- **Diagnostics** (DIAG 3–8, below): `PIN_PROBE`, `REGS`, `SET`, the net-off measurement, `IMU` statistics.

### What 0.5.0 implements (M1)

- **Executor** (`src/exec.c`): one thread, below Bluetooth, the control thread, the PMIC and the case-link
  threads (priority 8; host code is not capped and must not stop their 1 s cadence); host packets first,
  program instructions between them (VM.md §5.4). It blocks when nothing can run, with the 500 ms health
  beat. Control actions (`VM_START`, `VM_STOP`, `RESET_SESSION`, `MIC`) are posted to it and applied before
  its next instruction.
- **Wire** (`src/proto.c`): the window, ACKs and their repeat, events with the 9-byte header, `ERROR` 6 for
  a packet outside the window, every control request of VM.md §6.4. `EVENTS` answers mask 0: no trigger is
  reported as `INPUT` before M7 (the mask in force).
- **Panel** (`src/panel.c`): the core packs each `DRAW` row; the rows stream to the JBD panel with the
  next one staged while the current one is on the wire, one `97` latch per `DRAW`.
- **Properties and triggers from the hardware driven so far:** `BATT_MV`, `BATT_PCT` (the voltage estimate;
  a 1 % step is taken after 30 s so ADC noise does not raise `BATTERY` every second), `CHARGE`, `IN_CASE`
  (VBUS), `BATT_MA`, `BATT_TEMP` (the `POWER` telemetry), `CASE_LID`, `CASE_BATT_PCT`, `CASE_CHARGING` from
  the case's battery requests (UNKNOWN 3 s after the last message; `CASE_CHARGING` = bit 7 of the case's
  battery byte, seen set while the case charged on USB, to be confirmed in M9), `ARM`, `LINK`,
  `LINK_INTERVAL`, `BRIGHTNESS`, `PANEL`. Triggers: `START`, `TIMER`, `BATTERY`, `CASE`, `USER0`–`3`
  (`HELLO.triggers` 0xF063). The IMU, light, wear, touch and peer properties read UNKNOWN; writes to
  `IMU_RATE` are kept (the accelerometer stays at 12.5 Hz low power until M3), writes to `GYRO_RATE`,
  `AUTO_BRIGHTNESS`, `PEER_LINK`, `PEER_SEND` are error 5 until their milestones. (0.5.2+: above.)
- **Microphone surface** (VM.md §6.5): `MIC` with its checks (safe mode 7, no microphone or an unencrypted
  link 12, no panel 11), the indicator (16 × 16 at level 15, (620, 4) right, (4, 4) left), redrawn over
  `DRAW`s, cleared at `MIC` 0, stopped by `PANEL` = 0 (`NOTICE` 5, reason 1) and at a disconnection or a link
  that is not encrypted. `HELLO.board` bit 2 is set on the G1 (the hardware is there); capture and `MIC_DATA`
  come with M5, so 0.5.x turns on the indicator only and nothing is ever clocked or recorded.
- **Power rules kept** (docs/power.md): panel off at boot and 30 s after the last `DRAW` with no host and
  no running program, advertising 100–150 ms for 60 s then 1 s, the case's battery replies and the "full"
  latch of 0.4.10, UART TX only, the LSM6DSO at 12.5 Hz low power, the 30 ms / 100 ms run-time cap.
- **ESB mode 0 again** (net-core work mode 0 in the role reply, as stock and 0.4.6 and older): with
  mode 1 ("quiet", 0.4.8–0.4.10) the right arm, the ESB master, missed most BLE connection events under
  traffic (power.md §5, ESB mode). Both arms must always run the same mode.

## Notes for the design review

### For the reviewer (VIDEO, 0.5.42)

The proposal, its measurements and the wire format: the development tree's `docs/fps-codec.md` §3–§4; the spec:
VM.md §3.7. Points to decide: a codec-specific instruction in the VM (one opcode, 10 operands, a 1596-byte state
in the arena); the vm_host's optional `panel_span`; the library's `VideoEncoder` and the C and Go API.

### For the reviewer (the case hold, 0.5.32–0.5.38)

1. **The policy** (VM.md §6.6, §10 decision 12): in the case the panel is off as in stock. `DRAW` keeps its checks and
   errors but draws nothing (the VM core's "no panel" path, no new error code), so programs and hosts run unchanged;
   `PANEL` keeps the value written, as the microphone's hold does the other way round, and it is applied when the
   hold ends. Rejected: keeping the panel powered and dark to receive the draws (≈ 1 mA in standby, ≈ 4.6 mA lit
   black, the panel's rails and SPI still disturb the case link: 6 % of the polls lost) and a frame buffer in the arm
   (no 150 KB of RAM to spare).
2. **Restoring the image** is the host's or the program's: `NOTICE` 6 0 (the library redraws from its mirror, also
   when a kept session finds the panel off) and `CASE` (a program's handler). A program without a `CASE` handler shows
   nothing until it draws again after the glasses left the case: acceptable? (gifplay and the notification app redraw
   by themselves.)
3. **`CASE_PANEL`** (`CTRL 0x15`): host only (a program cannot set it), lasts until the arm leaves the case (it
   survived a disconnection by design: programs left running in the case for tests, and the emulator, which is
   always "in the case", needs it across reconnections), never during a top-up. The library sends it at every
   session setup (1 with `Connection::case_panel`, else 0).
4. **"Charging" = the case's run of polls** (three within 4.5 s of each other, until 6 s after the last), not VBUS:
   the emulator has VBUS all the time, and a full arm's re-check (one poll, VBUS for ~1 s) must not take the panel
   away. The LEDs go dark (`A9`) for 2 s around such a re-check.
5. **The microphone in the case** is refused (11) and stops at the hold's start (`NOTICE` 5 reason 4), since its
   indicator needs the panel; with `CASE_PANEL` it works outside top-ups (no blanking while it is on).
6. **Not done, for the user:** lowering a top-up's +7–11 °C peak needs an earlier "full" or shorter top-ups (power.md
   §15); both change when charging ends.

### For the reviewer (M9 and the head frame, 0.5.26)

VM.md changed where it did not say how (§2.5 `IN_CASE`; §6.6; §6.8). Points to review:
1. **Wear ends `IN_CASE` only 3 s after the last VBUS sample, field or message**, as motion does, not at once as
   decision 10 reads: a classification as worn while the case still powers or talks to the arm would otherwise let
   `IN_CASE` drop and come back with the next message (flicker). Taking the glasses out stops the case's evidence
   within about a second, so wear still ends it within about 4 s.
2. A classification as worn made while in the case counts once the arm leaves it (`WORN` 1 at once), so that wear can
   end `IN_CASE`; a classification as not worn there is kept too.
3. While `IN_CASE` is 1 every accelerometer sample is read (one FIFO read a second at 13 Hz); with `IMU_RATE` 0 there
   is no lift detection and only wear or the 400 s end the state.
4. **The charger rule** is stock's: enable after each battery reply with status 0 or 4 (stock has no disable
   command); g1v0 had rewritten the enable every 5 s, a deviation power.md §3 noted.
5. **The restart request** (`0B`) is now logged and not obeyed, as VM.md §6.8 says; g1v0 had rebooted as stock.
6. **Head frame:** decision 9's right-handed frame is applied by negating the provisional map's z row (pitch =
   atan2(−acc_z, acc_y)), so `PITCH` and `ROLL` keep their values on the arms and `ACC_Z` changes sign. The provisional
   map then has determinant −1: one of its axes (from stock's axes: X forward, Y up on the right, Z inward) is mirrored.
   If stock's pitch, asin(−ax/|a|), is positive looking up, X points toward the wearer and the old map read with the
   new labels is the rotation, which would flip `PITCH`'s sign instead. The worn two-pose check decides (STATUS.md).
   The cube and the dashboard use `PITCH`/`ROLL` (unchanged); the gestures demo records `ACC_Z` (its sign changed).

### For the reviewer (M10, 0.5.24)

VM.md changed where it did not match the stock net core or left a rule open (§2.5 `WORN`, `AUTO_BRIGHTNESS`,
`PEER_LINK`, `PEER_T_US`; §5.2; §6.7; §6.9; §8). Points to review:
1. **The link is not fast.** The stock net core exchanges once every 50 ms and holds one app frame at a time; the
   left arm's frames ride in acknowledgements one exchange behind. So one way takes 6–90 ms right to left and
   50–150 ms left to right, a round trip 60–240 ms (emulator; 60–196 ms, median 110, on the arms), not VM.md's
   "under 2 ms" or review §5's "< 5 ms".
   Only a net-core change could do better (not planned). The 100 ms poll adds no radio traffic: app frames take
   the place of the net core's own announcements.
2. **Eye sync without the host still works, with a larger lead:** `PEER_T_US` became the sender's `PEER_SEND`
   time in the receiver's clock (the link learns the clock offset from the exchanges both arms see), so the recipe
   is `SLEEP_UNTIL` write time + lead on the sender and `PEER_T_US` + lead on the receiver, lead 200 ms. Measured in
   the emulator from both panels' times: within 0.3 ms. The arrival time is no longer readable (DIAG `PEER` keeps
   the last one).
3. `PEER_LINK` is a sensor setting (0 at boot and `RESET_SESSION`, kept across `VM_START`/`VM_STOP`), as
   `GYRO_RATE`; a left arm's `AUTO_BRIGHTNESS` returns to 0 when its `PEER_LINK` goes to 0. `PEER_LINK` reads 2 while
   a frame of the other arm arrived within 0.5 s (the left arm keeps alive every 200 ms).
4. **`WORN` from the link value**: the value is the received signal strength of the other arm's last frame (ESB
   RSSI, −dBm), probably strong with the temples folded (below 40: not worn) and weaker when worn (51 and above).
   The net core reports it about every 10 s and at once on a threshold crossing; 0 (no frame yet) is taken as no
   reading, where stock's app would count it as "removed". Whether the arms read ≥ 51 when worn is for the
   user's return (STATUS.md).
5. The emulator's two-machine ESB is not faithful (stalls; next to no exchanges without app frames), so the
   pair tests check delivery and order, and the timing above needs the arms (power.md §12).

### For the reviewer (M7, M8 and 0.5.16)

VM.md changed where it did not match stock or the decisions (§2.5 `WORN`, `TOUCH_GESTURE`, `GYRO_RATE`,
`AUTO_BRIGHTNESS`, `LIGHT`; §5.2 the sensor settings; §6.6; §6.7). Points to review:
1. **TouchBar classification.** VM.md §6.7 and review §8.1 had the controller classify gestures itself with event
   enable 0xFDF (the Awinic SDK's table). Stock 1.6.6's own table (flash 0x8DB58; review §8.1's 0x8A0A8 is g1-fw's
   older dump, byte-identical) enables 0x40F, and stock classifies in its firmware from the touch down/up events
   (0x4410 bits 1/2) with timers (decomp `touch_key_thread`, g1-fw `g1-button-semantics.md` §4). g1v0 does the same
   with stock's 305/458 ms; the controller has no slide configured, so `TOUCH_GESTURE` 6/7 never occur; stock's 5-tap
   reboot, 10-tap wipe and 15-tap ship mode are not reproduced (more than 3 taps are ignored).
2. **Wear.** Stock's worn/removed (`F5 06/07`, `EVENT_ENTER_WARED`/`EVENT_UNWARED_OUTBOX` in `check_work_mode`) do not
   come from the TouchBar controller: they compare a value the network core sends with its ESB packet counts (IPC
   message 7, fourth field; below 40 for some rounds → removed, 51 and above → worn), probably the arm-to-arm link's
   quality with the temples folded or open. The controller's in-ear/out-of-ear events (bits 8/7) are disabled in
   stock's table; g1v0 enables them for `WORN` as review §8.2 decided, but whether they ever fire on a worn G1 is
   unknown (STATUS.md, for the user's return). Proposal: M10 (the IPC multiplexer) derives `WORN` from that ESB value
   as stock does.
3. **The controller scans only when wanted** (a program with a `TOUCH`/`WEAR` handler or waiting for one, or the
   `EVENTS` bits), else sleeps; `WORN` is `UNKNOWN` while it sleeps. A program that reads `WORN` or `TOUCH_GESTURE`
   without a handler gets `UNKNOWN`/the last gesture.
4. **`IN_CASE` at "full", and while charging.** Between the case's re-checks (every ~300 s) a full arm gets neither
   VBUS nor case messages, so it reads `IN_CASE` 0 in the case: the in-case rules and bonding do not apply then. While
   the case charged the arms after their reboots, VBUS came and went every ~6 s, so even with the 3 s hold `IN_CASE`
   (and `CASE`) changed about every 6 s. A hold of ≥ 10 s, or the ST25DV's RF field detection (M9), would steady it.
   `WORN` no longer follows these changes (0.5.22: 0 from the case on until the controller says worn).
5. **Light only with the panel on, on the right arm only** (as stock, which starts the OPT3007 in `panel_on`, on
   the ESB master only; on the arms it is powered with the panel's rails): VM.md §6.6 had "while something uses it"
   alone, §6.7 assumed a sensor per arm. The left eye follows the light through the host or the arm-to-arm link. The
   OPT3007 has no interrupt line; the "threshold interrupt" is the VM's `LIGHT` range, checked at each 800 ms reading.
6. **`GYRO_RATE` and `AUTO_BRIGHTNESS` reset at `RESET_SESSION`** like `IMU_RATE` (VM.md §5.2), so a dead host's
   session cannot leave the gyroscope on.
7. **Head frame handedness.** VM.md's head frame (x right, y up, z forward) is left-handed; the LSM6DSO's frame is
   right-handed and the provisional map is a proper rotation (determinant +1), so one of its axes is mirrored against
   the spec's frame (the worn two-pose check will tell which). `GYR_*` = det(map) · map · rate keeps the right-hand rule
   in the map's frame; a measured map with determinant −1 flips the rates accordingly.
8. **0.5.16:** `PANEL` = 2 from off powers the panel on first (71 ms), so the next `DRAW` is the 4 ms wake; while the
   microphone holds the panel, 2 is applied when it stops (standby, not off). The library's all-black `Flush` →
   standby also blanks what a program drew on that eye.

### For the reviewer (M5, firmware part; the IMU map)

VM.md changed once, by the codec review (`docs/audio-codec.md`): frames_per_event = min(5, …) instead of min(10, …)
(§6.5 and §8). Points it leaves open, decided in the firmware:
1. `MIC` 0: no `MIC_DATA` is made after the stop is applied (the encoder's lock), but events made just before it may
   still arrive after the reply (they were in the outgoing ring or the Bluetooth queue).
2. `MIC` 1 while on with the other rate restarts the capture in that format, counter from 0 (the reply then tells
   the truth); with the same rate it changes nothing.
3. The first 40 ms of each start are dropped and a 13 Hz DC blocker precedes the encoder (a start pop on the arm);
   the spec says nothing about processing before the encoder.
4. The encoder costs ~25 % of the 64 MHz app core (2.5 ms per 10 ms frame); 16 kbps barely changes it (2.27 ms).
   As the codec review decided, the capture stays at 64 MHz; `G1_MIC_128MHZ` (or DIAG SET 5) halves the time but
   measured +1.03 mA against +0.28 mA for the encoder alone (the busy-wait loops follow `SystemCoreClock`, so the
   switch itself is safe).
5. The left arm reports no microphone and refuses `MIC` (12): stock's firmware has no role check (the phone app only
   ever opened the right arm's), the boards are the same; whether the left one is populated is untested.
6. The capture holds the HFXO (an exact 16 kHz); `CONFIG_FPU` is on for the whole image (hard float, shared).
7. The IMU map is provisional (stock's axes, the right arm's factory pitch base, the case readings for the Y sign; the
   left arm borrows the right's base); VM.md §2.5's "measured once" still needs the worn two-pose check.
8. The lease (0.5.12) is renewed by `MIC` 1 itself rather than by acknowledging frames: no new request, and the
   library ties it to frames that arrived (it renews from `MIC_DATA`), so a host that stops reading lets it lapse
   as well as one that died. VM.md §6.4's `MIC` reply gained `lease_ms` at offset 6.
9. Open for the reviewer: (a) a panel standby policy — deep power-down with VCOM off costs +0.20 mA and makes the
   next power-on ~4 ms instead of ~73 (`docs/power.md` §10); a program's `PANEL` = 0 could enter it for a while
   before the rails go off. (b) `IN_CASE` follows the raw VBUS, which the case pulses during its polls; a 3 s hold
   would steady it but also stretches the in-case security rules by 3 s after leaving the case.

### For the reviewer (M3, firmware part)

1. VM.md §2.5 says `PITCH` = atan2(acc_z, acc_y) and `ROLL` = atan2(−acc_x, acc_y), each −90…90: atan2 gives
   ±180 when acc_y < 0 (head beyond vertical), so the firmware clamps; with roll the pitch reads high
   (atan2(z, y) with y = cos r · cos p). atan2(z, √(x² + y²)) would be roll-independent and −90…90 by itself.
2. §2.5's "at most one sample period old" holds; while the FIFO batches, reading a property drains it (in
   order), so a reading is the chip's newest sample.
3. §5.2 `MOTION`: exact as specified (consecutive samples, Euclidean); the chip only decides when to look.
   `TILT` is evaluated per sample when outside the range; a sample inside it is not shown to the executor
   on its own unless it is the first one back inside.
4. Review §1.3 said the INT wiring was unknown: INT1 is P1.15 on both arms, INT2 P1.04 on the left (the
   right arm's I²C stops answering while P1.04 has a pull-up, so its INT2 was not probed).
5. The axis map per arm still needs the two-pose calibration with the glasses worn.
6. The `INPUT` count restarts at `RESET_SESSION` (as `NOTIFY`); `RAISE` reports only while a program runs.

### For the reviewer (M1)

VM.md needed no correction. Open points:
1. `HELLO.board` bit 2 says the microphone exists while 0.5.x has no capture (indicator only until M5).
2. `PANEL_READ` with the panel off answers 11 (nothing to read back); VM.md says "error 11 without a panel".
3. §5.4 places the executor below Bluetooth and the control requests; since host code is uncapped it must
   also be below the firmware's periodic threads (0.4.10's render thread at a higher priority starved the
   PMIC thread for 20 s under a stream of large `BLIT`s and the health check rebooted the arm).
4. Review §3.4's condition held: mode 1 changed BLE behaviour on the right arm, so mode 0 stays.
5. `BATT_PCT`'s 1 %-after-30 s hysteresis is a firmware choice the spec does not mention.
