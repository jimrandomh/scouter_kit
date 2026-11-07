# ScouterKit G1 protocol, version 5 (draft)

Draft of 2026-10-23, reviewed 2026-10-26: the decisions of `docs/review-v5.md` (development tree) are
applied below. Implemented by g1v0 0.5.x (`../README.md`); corrections found while implementing are listed for
the reviewer in `DEVELOPMENT.md`.

Each arm of the G1 (one eye) is a small programmable machine: a 256 KiB **arena** that is its only
memory, a 640 × 480 panel with 16 grey levels, a few sensors and a Bluetooth LE link. The host sends
**host code**: instructions in numbered packets, run at once and in order. The same instructions stored
in the arena form **programs**, which the arm runs by itself when **triggers** fire (a timer, motion,
ambient light, battery, ...). The two arms are independent; the host keeps them in step.

## 1. Conventions
- Little-endian. `u8`…`u32` are unsigned, `i32` is two's complement, and a `u24` is 3 bytes, low byte
  first.
- **Device time**: u32 µs since boot. It wraps after 71.6 min, so two times are compared by their signed
  32-bit difference.
- **Arena**: 262 144 bytes, zero at boot. An **address** is a byte address in it (0…262 143). A **word**
  is 4 bytes read as a u32; word i is at address 4·i.
- Integer division truncates toward zero; ceil(a/b) = (a + b − 1)/b.

Error codes, used by `ERROR`, `HALT` and control replies:

| Code | Name | Meaning |
|---|---|---|
| 1 | malformed | an instruction cut off, a bad varint, a bad packet length |
| 2 | invalid op | an unknown opcode, a program-only instruction in host code, an unknown control request |
| 3 | arena | an address, source or range outside the arena or outside its bitmap |
| 4 | format | invalid PackBits data |
| 5 | argument | a value out of range: level, mode, property, trigger, mask |
| 6 | window | a packet outside the receive window (§6.2) |
| 7 | safe mode | drawing and programs are disabled (§6.4) |
| 8 | font | an invalid font blob or glyph index |
| 9 | stack | `CALL` deeper than 16, or `RET` with an empty stack |
| 10 | budget | more than 1000 jumps in one activation (§5.4) |
| 11 | panel | no panel available (`PANEL_READ`, `MIC`) |
| 12 | mic | microphone unavailable: no microphone on this board, or the link is not encrypted (§6.5) |
| 13 | asset | `ASSET_LOAD`: no asset with that ID, or a corrupt one; a store request: a job runs, no space, no store (§6.10) |

## 2. Instructions

### 2.1 Encoding
An instruction is its opcode byte, then k **operand nibbles** (k is fixed by the opcode; two per byte,
the first in the high nibble; an odd k is padded with a 0 nibble), then its literal fields, if any, and
last its variable part: one u16 per escape nibble, in operand order, or the data bytes of `WRITE` and
`WRITE_RLE`. The opcode and the nibbles fix the size, so every instruction is self-delimiting, and its
fixed fields come before its variable part. Checks at a fetch, in this order: an unknown opcode, or one
not allowed where it is fetched, is error 2 (before any length check); an instruction cut off by its
packet's or the arena's end, or a nonzero padding nibble, is error 1; a fetch at or past the arena's end
(a handler entry, code running off the end) is error 3 with that address as detail.

- An operand names an arena word, and its value is that word. Nibble 0–14 is **register r0–r14 = words
  0–14** (addresses 0–59). Nibble 15 is an **escape**: a u16 word index (0…65 535, the whole arena)
  follows in the variable part.
- **No immediate values.** Constants are arena words (a constant pool that the host writes). The only
  literal fields are code addresses (u24) in `JMP`, `CALL` and branches, and the address, length and data of `WRITE` and
  `WRITE_RLE`.
- Values are 32-bit and arithmetic wraps. A destination operand d is written after all sources are read.

In listings, `rN` is a register, `[w]` the word w, and `Cv` the pool word that holds the value v.
Example: `ADD r3, r1, [0x0120]` = `11 31 f0 20 01`.

### 2.2 Table
k = operand count, e = escape count. H marks instructions that are also valid in host code; all are valid
in programs.

| Op | Name | Operands | Literal fields | Size | H | Effect |
|---|---|---|---|---|---|---|
| 0x01 | `WRITE` | | u24 addr, u8 n, n bytes | 5 + n | H | the n bytes to addr |
| 0x02 | `WRITE_RLE` | | u24 addr, u16 len, u8 n, n bytes | 7 + n | H | PackBits-decoded bytes to addr (§2.4) |
| 0x03 | `COPY` | dst, src, n | | 3 + 2e | H | memmove of n bytes |
| 0x04 | `UNPACK` | dst, src, n, len | | 3 + 2e | H | PackBits-decode the n bytes at src into the len bytes at dst (§2.4) |
| 0x10 | `MOV` | d, a | | 2 + 2e | H | d = a |
| 0x11–0x1D | arithmetic | d, a, b | | 3 + 2e | H | §2.3 |
| 0x20–0x23 | `LD32`, `LD16`, `LD8`, `LD4` | d, base, i | | 3 + 2e | H | d = element i (§2.4) |
| 0x24–0x27 | `ST32`, `ST16`, `ST8`, `ST4` | base, i, a | | 3 + 2e | H | element i = a |
| 0x30 | `JMP` | | u24 target | 4 | | go to target |
| 0x31 | `CALL` | | u24 target | 4 | H | push the next address, go to target (in host code: §6.2) |
| 0x32 | `RET` | | | 1 | | pop the return address |
| 0x33 | `END` | | | 1 | H | end the task (in host code: the packet) |
| 0x34 | `HALT` | | | 1 | | stop the program (§5.5) |
| 0x35–0x3A | `BEQ`, `BNE`, `BLTU`, `BGEU`, `BLTS`, `BGES` | a, b | u24 target | 5 + 2e | | go to target if a = b, a ≠ b, a < b, a ≥ b (U unsigned, S signed) |
| 0x40 | `SLEEP_UNTIL` | t | | 2 + 2e | H | wait until device time t (§2.6) |
| 0x41 | `WAIT_EVENT` | d, m | | 2 + 2e | | wait for a trigger in mask m, d = its ID (§5.3) |
| 0x42 | `RAISE` | t | | 2 + 2e | H | set trigger t pending (§2.6) |
| 0x43 | `NOTIFY` | a, b | | 2 + 2e | H | send a `NOTIFY` event with a and b (§2.6) |
| 0x44 | `SYS_READ` | d, p | | 2 + 2e | H | d = property p (§2.5) |
| 0x45 | `SYS_WRITE` | p, a | | 2 + 2e | H | property p = a |
| 0x50 | `FILL` | 7 | | 5 + 2e | H | §3.2 |
| 0x51 | `BLIT` | 12 | | 7 + 2e | H | §3.3 |
| 0x52 | `GLYPHS` | 9 | | 6 + 2e | H | §3.4 |
| 0x53 | `DRAW` | 8 | | 5 + 2e | H | §3.5 |
| 0x54 | `LINE` | 7 | | 5 + 2e | H | §3.6 |
| 0x55 | `VIDEO` | 10 | | 6 + 2e | H | §3.7 |
| 0x56 | `ASSET_LOAD` | id, dst, off, n | | 3 + 2e | H | bytes off…off + n − 1 of the stored asset whose ID is at id to dst (§2.4) |

Every other opcode is invalid (error 2). 0x00 is never an instruction, so running into zeroed memory
stops at once. Host code (the bytes of a packet) uses the instructions marked H; stored code reached from
a packet by `CALL` may use every instruction except `WAIT_EVENT` and `HALT` (§6.2).

### 2.3 Arithmetic and logic
0x11 `ADD`, 0x12 `SUB`, 0x13 `MUL` (low 32 bits), 0x14 `DIVU`, 0x15 `MODU`, 0x16 `DIVS`, 0x17 `MODS`,
0x18 `AND`, 0x19 `OR`, 0x1A `XOR`, 0x1B `SHL`, 0x1C `SHR` (zeros in), 0x1D `SAR` (sign in); d = a op b.
`DIVS` and `MODS` are signed and truncate; the remainder has the sign of a. Shifts use b mod 32.
Division by zero gives quotient 0 and remainder a, and `DIVS` of 0x80000000 by −1 gives 0x80000000 with
remainder 0, so a = (a/b)·b + a%b always holds and arithmetic never fails.

### 2.4 Memory
- `WRITE` copies its n data bytes to addr. Error 3 if addr + n > 262 144.
- `WRITE_RLE` decodes PackBits into the len bytes at addr. Each run starts with a header byte h read as
  i8: 0…127 copies the next h + 1 bytes, −127…−1 repeats the next byte 1 − h times, −128 does nothing.
  Error 3 if addr + len > 262 144; error 4 unless the runs use exactly the n bytes and produce exactly
  len bytes. Both are checked before anything is written.
- `COPY dst, src, n` moves n bytes as if the source were read first. Error 3 if either range ends past
  the arena.
- `UNPACK dst, src, n, len` decodes the PackBits stream of n bytes at src into the len bytes at dst, with
  the rules of `WRITE_RLE`: error 3 if either range ends past the arena or the two overlap, error 4 unless
  the runs use exactly n bytes and produce exactly len bytes; both checked before anything is written.
- `LDw d, base, i` and `STw base, i, a`: base is an address and i counts elements of the width w. For
  w = 32, 16 and 8 the element is at base + i·w/8 (little-endian, any alignment). For w = 4 it is nibble
  2·base + i: in byte base + i/2, the high nibble when i is even. That is the pixel order of §3.1, so
  pixel (x, y) of a 4-bpp bitmap at B with stride s is `LD4`/`ST4` element 2·s·y + x of base B. Loads
  zero-extend; stores write the low w bits of a. Error 3 if the element is outside the arena. Address
  arithmetic is exact for every range in this section (a wrapping index is error 3, never an alias). A
  `WRITE` or `WRITE_RLE` stored in the arena may overwrite its own bytes: the data are read first.
- `ASSET_LOAD id, dst, off, n` copies bytes off…off + n − 1 of the stored asset (§6.10) whose 16-byte ID lies
  at address id into dst. Checked before anything is written: error 3 if id + 16 or dst + n ends past the arena
  (detail id or dst), error 13 if the arm holds no asset with that ID (or has no store), error 3 with detail off if
  off + n passes the asset's end. The ID is read first, so dst may overlap it. The first load of an asset after a
  boot checks the crc32 of its whole body (about 10 ms per 64 KB); a body that fails is error 13 until the asset is
  deleted. The bytes come from the flash at about 4 MB/s; while a store job runs, a load also waits for the job's
  current flash call (one sector erase at most). In a program it counts toward the run-time cap like `DRAW` (§5.4).

### 2.5 Properties
`SYS_READ d, p` and `SYS_WRITE p, a` read and write the arm's state; p is a value, normally a pool word.
An unknown p, or a write to a read-only property, is error 5. **`UNKNOWN` = 0x80000000** is read for a
value the arm does not have: no driver for the sensor, or no reading yet.

| ID | Name | Access | Unit and values |
|---|---|---|---|
| 0x00 | `TIME_US` | R | device time, µs |
| 0x01 | `TRIGGER` | R | ID of the trigger that started the current task; 0xFF in host code |
| 0x02 | `PENDING` | R | pending trigger bits (bit i = trigger i) |
| 0x03 | `ARM` | R | 0 right arm, 1 left arm |
| 0x04 | `LINK` | R | 1 while a host is connected and its writes are obeyed (§6.1), else 0 |
| 0x05 | `LINK_INTERVAL` | R | connection interval, µs; 0 without a link |
| 0x10 | `BATT_MV` | R | battery voltage, mV |
| 0x11 | `BATT_PCT` | R | charge estimate from the voltage, 0…100 (reads high while charging); a 1 % change is taken only after it has held for 30 s, so ADC noise does not raise `BATTERY` every second |
| 0x12 | `CHARGE` | R | 0 not charging, 1 charging, 2 charge complete |
| 0x13 | `IN_CASE` | R | 1 in the charging case, a state: **set** by VBUS, a case message (§6.8) or the tag's RF field; **cleared** when 3 s have passed without any of them and since the last one the accelerometer saw motion (a change above 150 mg between samples, a firmware constant independent of `MOTION_MG`: the arm was lifted out; every sample is read while `IN_CASE` is 1) or the arm was classified as worn (§6.7), or after 400 s without any of them (the case re-checks a full arm about every 300 s, so an arm lying still in a full case stays 1). The raw VBUS (in `STATUS`) pulses and this does not. It also gates the in-case rules of §6.1 |
| 0x14 | `WORN` | R | 1 worn, 0 not worn: 0 from the arm's time in the case (`IN_CASE` = 1) on, until the network core's link value or the TouchBar controller says worn (§6.7); then the link value as stock reads it (51 and above worn, below 40 for 3 s not worn), the controller's worn event setting 1 at once; `UNKNOWN` before either |
| 0x15 | `BATT_MA` | R | battery current, mA (i32), positive while discharging; a 64-sample average |
| 0x16 | `BATT_TEMP` | R | battery temperature from the pack's NTC, 0.1 °C (i32) |
| 0x18–0x1A | `ACC_X`, `ACC_Y`, `ACC_Z` | R | acceleration, mg (i32), head frame: x right, y up, z toward the wearer (right-handed, so the right-hand rule holds for `GYR_*`); about (0, 1000, 0) upright and still, acc_z below 0 while looking up |
| 0x1B | `PITCH` | R | head pitch, degrees (i32), −90…90, positive looking up |
| 0x1C | `LIGHT` | R | ambient light, 0.01 lux |
| 0x1D | `ROLL` | R | head roll, degrees (i32), −90…90, positive with the head tilted to the right (right ear down) |
| 0x20 | `BRIGHTNESS` | R/W | panel brightness level 0…42 as written (21 at boot; applied at power-on while the panel is off); the panel shows the lowest of `BRIGHTNESS`, `BRIGHTNESS_MAX` and `BRIGHTNESS_CAP` (§6.6) |
| 0x21 | `PANEL` | R/W | panel power: 0 off (the image is lost; a `DRAW` brings it back in ≈ 71 ms), 1 on (all pixels 0; no error without a panel), 2 standby (deep power-down with VCOM off: the image is lost, ≈ 1 mA on the G1 (0.3–1.7 mA), a `DRAW` brings it back in ≈ 4 ms; for a program or host that will draw again soon, §6.6); while the microphone is on, 0 and 2 blank the panel to the indicator and are applied when the microphone stops (§6.5); other values are error 5 |
| 0x22 | `TIMER` | R/W | write t: trigger `TIMER` fires at device time t, due when (i32)(t − now) ≤ 0, the rule of `SLEEP_UNTIL`: at once if t is not in the future, and a time 2^31 µs or more ahead counts as past (no error); a write replaces the previous time; read: the last time written |
| 0x23 | `MOTION_MG` | R/W | `MOTION` threshold, mg; 0 off |
| 0x24, 0x25 | `PITCH_LO`, `PITCH_HI` | R/W | `TILT` range, degrees, signed |
| 0x26, 0x27 | `LIGHT_LO`, `LIGHT_HI` | R/W | `LIGHT` range, 0.01 lux, unsigned |
| 0x28 | `IMU_RATE` | R/W | accelerometer sample rate, Hz: 0 (off), 13, 26, 52 or 104 (the LSM6DSO rates 12.5…104 Hz); 13 at boot; other values are error 5 |
| 0x29 | `BRIGHTNESS_MAX` | R/W | the highest level the panel shows, 0…42; 42 at boot and after `RESET_SESSION`, kept across `VM_START` (a sensor setting, §5.2); programs and hosts may write it (the last write counts, as for `BRIGHTNESS`); the library writes the other eye's `BRIGHTNESS_CAP` here (§6.6) |
| 0x2A | `BRIGHTNESS_CAP` | R | the panel guard's limit, 6…42 (42: none, and while the panel is off or in standby), §6.6 |
| 0x30–0x32 | `GYR_X`, `GYR_Y`, `GYR_Z` | R | angular rate, mdps (i32), head frame (right-hand rule about x, y, z); `UNKNOWN` while `GYRO_RATE` = 0 |
| 0x33 | `GYRO_RATE` | R/W | gyroscope sample rate, Hz: 0 (off, at boot and after `RESET_SESSION`), 13, 26, 52 or 104; other values are error 5 |
| 0x34 | `TOUCH_GESTURE` | R | the last TouchBar gesture (§6.7): 1 tap, 2 double tap, 3 triple tap, 4 long press start, 5 long press end, 6 slide forward, 7 slide back (6 and 7 not produced with stock's controller configuration); 0 none yet |
| 0x35 | `CASE_LID` | R | 0 open, 1 closed, as the case reports it (§6.8); `UNKNOWN` out of the case |
| 0x36 | `CASE_CHARGING` | R | 1 while the case reports its own charger connected; `UNKNOWN` out of the case |
| 0x37 | `CASE_BATT_PCT` | R | the case's battery, 0…100; `UNKNOWN` out of the case |
| 0x38 | `AUTO_BRIGHTNESS` | R/W | 0 (at boot and after `RESET_SESSION`): `BRIGHTNESS` stays as written; 1: the firmware sets it from `LIGHT` with stock's table (§6.7) on the right arm, and on the left from the right arm's level over the arm-to-arm link (§6.9: error 5 there while `PEER_LINK` is 0, and `PEER_LINK` = 0 sets it back to 0); other values are error 5 |
| 0x39 | `PEER_LINK` | R/W | arm-to-arm link (§6.9): write 0 off (at boot and after `RESET_SESSION`) or 1 on; reads 2 while on and a frame of the other arm arrived within the last 0.5 s, else 1 while on |
| 0x3A | `PEER_SEND` | W | the value to send to the other arm; one slot, a write replaces an unsent value; error 5 while `PEER_LINK` is 0 |
| 0x3B | `PEER_MSG` | R | the last value received from the other arm; `UNKNOWN` before the first |
| 0x3C | `PEER_T_US` | R | the sender's time of the `PEER_SEND` write that `PEER_MSG` came from, in this arm's clock (the link learns the offset between the two clocks, §6.9); the arrival time until the offset is known; `UNKNOWN` before the first value |

Sensor properties hold the latest reading: battery values at most 5 s old, `ACC_*`, `PITCH` and `ROLL` at
most one sample period old (`IMU_RATE`; `UNKNOWN` at rate 0), `LIGHT` one sensor conversion (≤ 800 ms;
`UNKNOWN` while the panel is off or nothing uses the sensor, §6.6). The firmware maps each arm's sensor axes to the head frame, so
both arms report the same values; pitch = atan2(−acc_z, acc_y) and roll = atan2(−acc_x, acc_y) in that
frame (the axis map per arm is measured once, review §1.3). The gyroscope runs only at a `GYRO_RATE` above 0 (stock never
turns it on). `WORN` comes from the TouchBar controller (§6.7).

### 2.6 Waiting, raising and notifying
- `SLEEP_UNTIL t`: with d = (i32)(t − now), d ≤ 0 goes on at once; otherwise it waits min(d, 2 s). In
  host code the rest of the packet waits, and no program instruction runs meanwhile. In a program the
  task suspends. Longer waits use the `TIMER` trigger.
- `RAISE t` sets trigger t (0…15) pending, as if it had fired (§5.2); t > 15 is error 5. Host code uses
  it to hand work to a program, after writing the data the program needs, and to simulate sensor
  triggers in tests.
- `NOTIFY a, b` sends a `NOTIFY` event (§6.3) with the two values. If the event cannot be queued (no host
  subscribed, or the outgoing queue is full), it is dropped; the event's count shows the gap. Host code
  uses it as a fence; programs use it to report.

## 3. Drawing

### 3.1 Bitmaps, clipping and modes
A bitmap is a base address B and a stride s (bytes per row). It reaches down to the end of the arena.
- **4 bpp**, the panel's depth (levels 0…15; 0 is off, 15 brightest): pixel (x, y) is in byte
  B + y·s + x/2, the high nibble for even x. The bitmap is 2s pixels wide.
- **1 bpp** (fonts, masks): pixel (x, y) is bit 7 − x%8 of byte B + y·s + x/8, and 1 is ink. The bitmap
  is 8s pixels wide.

Coordinates are i32, widths and heights u32; w = 0 or h = 0 draws nothing (only the argument checks apply). **Destinations clip**:
destination pixels outside the bitmap (x < 0, y < 0, x ≥ its width, or past the end of the arena) or,
for `DRAW`, outside the panel are skipped; rectangles are clipped as a whole before drawing, so the work
is bounded by the bitmap. **Sources must exist**: a source pixel outside its bitmap or
the arena is error 3.

For a destination pixel d, a source coverage a and a level L (0…15):

| Mode | Name | d' | Use |
|---|---|---|---|
| 0 | `COPY` | (a·L + 7)/15 | replace |
| 1 | `MAX` | max(d, (a·L + 7)/15) | lighten |
| 2 | `MASK` | (d·(15 − a) + L·a + 7)/15 | draw L where the source covers (text) |
| 3 | `ERASE` | (d·(15 − a) + 7)/15 | darken toward 0 |

A 4-bpp source gives a = its level; a 1-bpp source gives a = 15 for ink, else 0. A mode above 3 (for
`BLIT` see §3.3) or a level above 15 is error 5.

### 3.2 FILL dst, stride, x, y, w, h, level
Sets the pixels of the rectangle (x, y, w, h) of the 4-bpp bitmap (dst, stride) to level.

### 3.3 BLIT src, src_stride, src_x, src_y, dst, dst_stride, dst_x, dst_y, w, h, mode, level
Combines the source rectangle (src_x, src_y, w, h) with the 4-bpp destination rectangle at (dst_x,
dst_y), pixel by pixel. `mode` bits 0–1 select the mode of §3.1, bit 2 set means a 1-bpp source (else
4 bpp), and other bits are error 5. For overlapping bitmaps: if the destination's first drawn pixel lies after
the source's in the arena (after clipping; comparing bit addresses 8·B + 8·y·s + x·depth), rows run from the last to the
first and pixels from right to left, else from the first and from the left, and each pixel is read,
combined and written before the next. With equal depth and stride this is the same as reading the whole
source first, so a region scrolls in any direction with one `BLIT`. The firmware moves whole bytes when
the source is 4 bpp, the mode is `COPY`, the level is 15 and src_x, dst_x and w are even (row memmove: scrolling a
600 × 400 text area takes a few milliseconds, not a pixel loop).

### 3.4 GLYPHS font, dst, stride, x, y, mode, level, text, n
Draws the glyphs whose indices are the n bytes at address text, read as unsigned LEB128 varints (7 bits
per byte, low group first, bit 7 set on every byte but the last of an index; at most 3 bytes; indices
below 128 take one byte). The pen starts at (x, y), y being the baseline. For each glyph g: pen_x +=
kern(previous, g) (nothing for the first); g's 1-bpp bitmap is combined into the destination with its
top-left pixel at (pen_x + x_off, y + y_off); pen_x += advance. Checked before drawing: the font is
valid and every index < glyph_count (error 8); no varint is cut off at the end of the n bytes or longer
than 3 bytes (error 1); text + n ≤ 262 144 (error 3). Check order: mode and level (5), the text range
(3), the font (8), then per index in text order: varint (1), index (8), bitmap range (8). Drawing into a
bitmap that overlaps the text or the font changes them under way, so the drawing pass checks each glyph
again: a damaged varint or index ends the run, a glyph whose bitmap has become invalid is skipped (its
advance still applies), neither is an error. The pen does not wrap. `MASK` is the text mode.

### 3.5 DRAW src, src_stride, src_x, src_y, dst_x, dst_y, w, h
Copies the 4-bpp source rectangle (src_x, src_y, w, h) to the panel with its top-left pixel at (dst_x,
dst_y), then latches the panel, so a `DRAW` is never seen half done. The panel is 640 × 480, (0, 0)
top-left as the wearer sees it; its own memory holds the image (the arena is not a framebuffer), and
the levels are roughly linear in luminance. A full-width `DRAW` takes about 87 µs per row (≈ 42 ms for the
full panel, ≈ 2.6 ms for a 30-row band), a narrower one about 23 µs per row plus 0.22 µs per byte written
(g1v0 0.5.14, `docs/power.md` §10). The panel is off at boot; if it is off or in standby, `DRAW` powers it on first
(≈ 71 ms from off, ≈ 4 ms from standby); if no panel is available, it does nothing (no error). A `DRAW` with nothing visible (empty, or
entirely off the panel) does nothing: no power-on, no latch. Error 7 in safe mode. While the microphone is on, the
firmware redraws its indicator after every `DRAW` that covers it (§6.5); with no host connected and no
running program, the panel switches off 30 s after the last `DRAW` (§6.6). `VIDEO` (§3.7) draws the same way.

### 3.6 LINE dst, stride, x0, y0, x1, y1, level
Sets the pixels of the straight line from (x0, y0) to (x1, y1), both ends included, in the 4-bpp bitmap
(dst, stride) to level: one pixel per step along the longer axis (x when |dx| = |dy|), each at the exact
line rounded to the nearest pixel, ties toward the end with the smaller major coordinate, so a line and
its reverse set the same pixels. Pixels outside the bitmap are skipped. Error 5 if level > 15 or a coordinate is outside −32768…32767 (so a line is at most 65 536
steps).

### 3.7 VIDEO bitmap, stride, w, h, x, y, level, src, n, state
Decodes the frame record of n bytes at src (below) into the 1-bpp bitmap (bitmap, stride) of w × h pixels,
which holds the frame the panel shows, and draws what changed: each row whose pixels changed goes to the panel
at (x, y) plus its position, `level` for ink and 0 elsewhere (clipped as `DRAW`), at least from its first to
its last changed pixel (the core widens it to whole bitmap bytes and to an odd width: one panel window per row);
then the panel is latched once, so a frame is never seen half done. Unchanged rows are not sent: the arena
needs no 4-bpp frame, and the panel time follows the change, not the frame size. As for `DRAW`: if nothing
visible changes, the panel is not touched (no power-on, no latch), else it is powered on first; error 7 in
safe mode; the microphone indicator stays (§6.5); it counts as a `DRAW` for §6.6. `state` is the decoder's
adaptive model, VIDEO_STATE = 1596 bytes (798 u16), kept from record to record: one per stream, reset by
the record's flag. Checked before anything changes: `level` ≤ 15, w = 1…640, h = 1…480 and 8·stride ≥ w
(error 5, detail the value); the bitmap (up to bitmap + (h − 1)·stride + ceil(w/8)), the record and the
state inside the arena and not overlapping (error 3, detail the address); n ≥ 1 and flags ≤ 3 (error 4).
Any bytes decode to some frame (bad data gives wrong pixels, never an error): at most w + 1 symbols per row
and 7 coded decisions per pixel. The decoder keeps no state outside the arena between instructions: the
bitmap and the state are ordinary arena ranges that the host allocates, mirrors and restores like any bitmap,
and the panel's memory stays the only image store. Every 16 rows the core calls the platform's `now`, which
g1v0 uses as a sign of life. Time on the arms (g1v0 0.5.42, 2026-11-03): 18–19 ms per frame on average for Bad
Apple at 480 × 360 (the first, with the panel's power-on and every row, ~100 ms); 0.86–0.94 s for 640 × 480
one-pixel stripes with the redraw flag, 0.42–0.43 s for 12-pixel stripes whose every transition is V(±5)
(`tools/draw_bench.py --video`); the bound is about 1 s.

**Record.** Byte 0 is the flags: bit 0 *reset* (every probability is set to 2048 first), bit 1 *redraw*
(every row is drawn whole, changed or not: after the panel lost its image). Then binary decisions coded
with LZMA's range coder (without its first byte, always 0):
- At the start range = 0xFFFFFFFF and code = the next 4 bytes, big-endian. Bytes past the record read as 0.
- A bit with probability i: p = u16 i of the state & 0xFFF (the probability of a 0, in 1/4096), bound =
  (range >> 12)·p. If code < bound: the bit is 0, range = bound, p += (4096 − p) >> 5; else it is 1, code −=
  bound, range −= bound, p −= p >> 5. p is stored back; then while range < 2^24: range <<= 8, code = code
  << 8 | the next byte.
- A run r of colour c: j bits 1 with probabilities RUNLEN + 20c + min(k, 19) for k = 0, 1, …, ended by a 0
  or after 20 ones; then j bits with RUNBIT + 80c + 4·min(j, 19) + min(i, 3), i = 0…j − 1, are the binary
  digits of m after its leading 1; r = m − 1.

Probability bases: SAME 0 (3 of them), V0 3 (294), ISV 297 (84), SIGN 381 (42), MAG 423 (168), PASS 591 (7),
RUNLEN 598 (40), RUNBIT 638 (160).

**Rows**, y = 0…h − 1. A row is its *transitions*: each x (0…w − 1) whose pixel differs from the one on its
left, pixel −1 being 0; transition i (from 0) switches ink on if i is even. The reference of row y is the
new row y − 1 (none for row 0), and each of its transitions has a code: d + 8 if V(d) coded it, else 0.
Row y first reads *same* = bit(SAME + s), s = 2 for row 0, 1 if row y − 1 was same, else 0: a same row stays
as the bitmap holds it (codes 0). Otherwise its transitions are decoded with the modes of ITU-T T.6, with
a0 = −1 and colour = 0 at the start, until the row ends:
- b1 = the first reference transition after a0 whose index has the parity of colour, b2 = the next
  reference transition after b1 (w where there is none).
- Contexts: pc = the previous mode in this row (0 none yet, 1 V0, 2 V+1, 3 V−1, 4 other V, 5 pass, 6
  horizontal); ac = 6 if b1 = w, else by b1's code: 0 → 0, V0 → 1, V+1 → 2, V−1 → 3, V+2…+5 → 4,
  V−2…−5 → 5; tc = 0 if b1 = w, else 3 + t − b1 for the leftmost t in b1 − 2…b1 + 2 where the previous
  frame's row y (the bitmap before the row is written; pixels outside 0…w − 1 count as 0) has pixel t − 1 =
  colour and pixel t ≠ colour, or 0 if there is none.
- bit(V0 + 42pc + 6ac + tc) = 0: mode V0 (d = 0). Else bit(ISV + 12pc + 6a + tc) with a = 1 if 1 ≤ ac ≤ 5,
  else 0, = 1: mode V(d), negative if bit(SIGN + 6ac + tc) = 1, |d| = m: m = 1, then m += 1 while m < 5
  and bit(MAG + 42(m − 1) + 6ac + tc) = 1. Else bit(PASS + pc) = 1: pass. Else horizontal.
- V(d): a1 = max(b1 + d, a0 + 1). If a1 ≥ w the row ends; else a1 is a transition, a0 = a1 and colour
  flips.
- Pass: if b2 ≥ w the row ends; else a0 = b2.
- Horizontal: r1 = run(colour), r2 = run(1 − colour), t1 = max(max(a0, 0) + r1, a0 + 1), t2 = t1 + max(r2,
  1). If t1 ≥ w the row ends; else t1 is a transition; then if t2 ≥ w the row ends; else t2 is a
  transition and a0 = t2.

The library's `VideoEncoder` makes the records (`scouter_kit_g1.hpp`); at 480 × 360 the user's Bad Apple
clip takes 261 bytes per frame on average at 30 frames/s, 6.4 times less than row runs
(`docs/fps-codec.md`, development tree).

## 4. Fonts

### 4.1 Glyph order
Glyph index i < 62 is the i-th character of `0`–`9`, `A`–`Z`, `a`–`z`; a font may end before 62.
Indices 62 and up are the other characters, in ascending codepoint order. A font without some of the 62
characters keeps empty glyphs in their places. So a digit's value is its glyph index, in any base up to
36 (hex digit 0xB is glyph 11, `B`), and a program turns a number into text with `DIVU` and `MODU`.

### 4.2 Font blob
A blob sits at any address in the arena and is used in place.

| Offset | Size | Field |
|---|---|---|
| 0 | 3 | magic `"SKF"` (`53 4B 46`) |
| 3 | 1 | format = 1 (the format of PROTOCOL.md §5; a format 2 without the first 62 codepoints was dropped) |
| 4 | 4 | `size`: blob length in bytes |
| 8 | 2 | `glyph_count` n (1…65 535) |
| 10 | 2 | `kern_count` k |
| 12 | 1 | `ascent`: pixels above the baseline |
| 13 | 1 | `descent`: pixels below the baseline |
| 14 | 1 | `line_spacing`: baseline to baseline |
| 15 | 1 | reserved = 0 |
| 16 | 12·n | glyph table |
| 16 + 12n | 4·n | u32 codepoints of glyphs 0…n − 1 (host only; the firmware ignores them) |
| 16 + 16n | 4·k | kerning keys (u32), strictly ascending: left index · 65 536 + right index |
| 16 + 16n + 4k | k | kerning adjustments (i8), in key order |
| 16 + 16n + 5k | | glyph bitmaps, up to `size` |

Glyph entry (12 bytes): 0: u32 `bits` (bit offset of the bitmap from the blob's start), 4: u8 `w`,
5: u8 `h`, 6: u8 `advance`, 7: reserved 0, 8: i16 `x_off` (left edge from the pen), 10: i16 `y_off` (top
edge from the baseline, negative above). The bitmap is w·h bits, w bits per row, tightly packed: pixel
(c, r) is blob bit `bits` + r·w + c, MSB first. A glyph with w = 0 or h = 0 has no bitmap (`bits` = 0).
kern(p, g) is the adjustment of key p·65 536 + g, or 0 without one.

Valid (else error 8): magic and format; n ≥ 1; 16 + 16n + 5k ≤ `size`; the blob ends inside the
arena; every drawn glyph has `bits` + w·h ≤ 8·`size`. The host maps a character to its index by
position (`0`–`9`, `A`–`Z`, `a`–`z`) or by its own lookup over the codepoint table (not sorted as a
whole), and draws U+FFFD (if present) for a missing one. A line is ascent + descent tall and the next baseline is line_spacing lower;
a run advances by the sum of its advances and kerning.

### 4.3 Drawing a number
A u32 has at most 10 decimal digits. This loop writes them from the end of a 10-byte buffer at address B
(pool word `CB`; `CFONT`, `CDST`, ... name further pool words), so the most significant digit comes
first, and draws 0 as `0`:
```
        MOV    r1, C10        ; i = 10, one past the last digit
next:   SUB    r1, r1, C1     ; i -= 1
        MODU   r2, r0, C10    ; digit = value % 10 = its glyph index
        ST8    CB, r1, r2     ; buffer[i] = digit
        DIVU   r0, r0, C10    ; value /= 10
        BNE    r0, C0, next   ; until value = 0
        ADD    r3, r1, CB     ; address of the first digit
        SUB    r4, C10, r1    ; n = 10 − i
        GLYPHS CFONT, CDST, CSTRIDE, CX, CY, C2, C15, r3, r4    ; MASK, level 15
```
It takes at most 9 jumps. To right-align with a monospaced font, set the pen x to the right edge minus
n·advance; to append a unit, end the buffer with its glyph (§9.3).

## 5. Programs

### 5.1 Trigger table
A program is code in the arena plus a **trigger table**: 16 words at the address the host gives to
`VM_START`. Word i is the address of the handler for trigger i, or 0 for none (address 0 is r0, never
code). The table and the code are ordinary arena bytes: the host or the program may change them at any
time, and a change takes effect the next time the entry or instruction is used.

### 5.2 Triggers
| ID | Trigger | Fires when |
|---|---|---|
| 0 | `START` | once, at `VM_START` |
| 1 | `TIMER` | device time reaches `TIMER` (one-shot) |
| 2 | `MOTION` | the Euclidean distance between two consecutive acceleration samples exceeds `MOTION_MG` (computed by the firmware at `IMU_RATE`) |
| 3 | `TILT` | `PITCH` leaves [`PITCH_LO`, `PITCH_HI`] |
| 4 | `LIGHT` | `LIGHT` leaves [`LIGHT_LO`, `LIGHT_HI`] (OPT3007, §6.7) |
| 5 | `BATTERY` | `BATT_PCT` or `CHARGE` changes |
| 6 | `CASE` | `IN_CASE`, `CASE_LID`, `CASE_CHARGING` or `CASE_BATT_PCT` changes (§6.8) |
| 7 | `WEAR` | `WORN` changes |
| 8 | `TOUCH` | a TouchBar gesture; `TOUCH_GESTURE` holds it (§6.7) |
| 9 | `PEER` | a message from the other arm arrived; `PEER_MSG` and `PEER_T_US` hold it (§6.9) |
| 10–11 | | reserved |
| 12–15 | `USER0`–`USER3` | only by `RAISE` |

- Each trigger has **one pending bit**, as with Unix signals: a trigger that fires while its bit is set
  merges with it (`VM_STATUS` counts these). Triggers fire only while a program runs; while none runs,
  every trigger, `RAISE` included, is ignored. Independently of programs, the firmware reports the
  triggers in the `EVENTS` mask (§6.4) to the host as `INPUT` events (§6.3).
- The ranges fire on the change from inside to outside. A program usually sets a new range around the
  current value in its handler. A `SYS_WRITE` applies at once, from host code or a program; whether a
  range has been left is evaluated when the executor goes idle (end of a packet, `END`, `SLEEP_UNTIL`,
  `WAIT_EVENT`; a task paused by the cap is not idle), so both bounds can be set first. An `UNKNOWN`
  reading changes nothing; LO > HI is an empty range (always outside); the inside/outside state is
  sampled at `VM_START`, so a range already left does not fire then. `BATTERY`, `CASE` and `WEAR` fire on
  any change of the raw value (to or from `UNKNOWN` too) against a baseline taken at `VM_START`.
- **Settings and program state.** `MOTION_MG` (0, off), the ranges ([−90, 90] and [0, 0xFFFFFFFF], never
  left), `IMU_RATE` (13), `GYRO_RATE` (0), `AUTO_BRIGHTNESS` (0), `PEER_LINK` (0) and `BRIGHTNESS_MAX` (42) are sensor settings: reset only at boot and `RESET_SESSION`, kept across
  `VM_START` and `VM_STOP`, so host code can prepare them before `VM_START`. The timer and the pending
  bits are program state: cleared by `VM_START`, `VM_STOP` and a halt, so a program arms its timer in its
  `START` handler. `HELLO.triggers` shows which triggers this firmware can raise; one without a sensor
  driver never fires, and its properties read `UNKNOWN`.

### 5.3 Tasks
- **One task at a time.** When no task is current, the arm starts the handler of the lowest-numbered
  pending trigger that has one and clears that bit. The task runs until `END`, `HALT` or an error.
- `SLEEP_UNTIL` and `WAIT_EVENT` suspend the task. It stays the current task, so other handlers wait.
- `WAIT_EVENT d, m`: m is a trigger mask (bits 0–15). If a trigger in m is pending, lowest ID first,
  `WAIT_EVENT` clears its bit and writes its ID to d at once; otherwise the task waits until one in m
  fires, which then goes to the task instead of starting a handler. m = 0 or a bit above 15 is error 5.
  A pending trigger without a handler stays pending until a `WAIT_EVENT` takes it; `PENDING` reads the
  bits.
- A program can thus be short handlers, or one task that loops on `WAIT_EVENT`.
- Registers and the whole arena keep their values between tasks; each task starts with an empty call
  stack.

### 5.4 Execution and budget
One executor thread runs host code and programs, at a lower priority than Bluetooth, the control
requests and the firmware's periodic threads (PMIC, case link, health): host code is not capped, so a
long burst of packets must not stop their 1 s cadence. **Host code always comes first**: before each program instruction the executor runs any host
packet that is ready, and while a packet runs or sleeps no program instruction runs. Each instruction is
atomic and host code and programs interleave only between instructions, so a program never blocks the
host. With nothing to run, the executor blocks until a packet, a trigger, a deadline or a control
request arrives; it never polls.

**Budget:** each activation of a task may make **1000 jumps**: `JMP`, `CALL` and taken branches. The
count starts again when the task starts and each time it resumes from a real suspension in `SLEEP_UNTIL`
or `WAIT_EVENT` (not after a `SLEEP_UNTIL` already past, a `WAIT_EVENT` satisfied at once or a cap
pause; a packet's budget follows the same rule); jump 1001 is error 10. Without jumps, code only runs forward, so every activation ends. `CALL` nests up to
**16** deep (error 9 beyond, or for `RET` with an empty stack). The firmware keeps the return addresses
outside the arena, so a program cannot forge them.

**Run-time cap:** program instructions may take at most **30 ms of every 100 ms** period (a fixed grid
from boot; wall time, `DRAW`, `VIDEO` and `ASSET_LOAD` included; host code is not counted). A program instruction or a new task
starts only while the time charged to the current period is below 30 ms; time beyond that consumed
inside one instruction (one long `DRAW`) is charged to the following periods, so the bound holds for
back-to-back draws too. A paused task waits for the next period; this is not an error, `VM_STATUS`
counts it. It bounds the CPU and panel time of a
runaway program, for instance a handler that `RAISE`s its own trigger or writes `TIMER` = now, to about a
third of one core.

**Host `CALL`:** a packet may `CALL` stored code (§6.2). The packet then has a call stack of its own,
empty at its start, and the jump budget above; the called code runs as part of the packet, so no program
instruction runs meanwhile.

### 5.5 Errors, states and lifetime
A program is stopped (at boot, after `VM_STOP` and `RESET_SESSION`), running, or halted. An error in a
program **halts** it: the failing instruction changes nothing, the pending bits and the timer are
cleared, and the arm sends `HALT` (§6.3) with the trigger, the instruction's address, its opcode, the
code and a detail word. The `HALT` instruction does the same with code 0. The arm never reboots because
of a program, and host code goes on.

Programs live in RAM only. After a reboot the arena is zero and the VM is stopped, so a program never
starts by itself, and one that hits a firmware bug is gone after the reboot that bug causes.

### 5.6 Sharing with the host
- Host code and programs share the arena, the panel, `BRIGHTNESS` and `PANEL`; the last writer wins. A
  host that needs the display stops the program first.
- While a program runs, r0–r14 are its registers; host code uses other words.
- A program may write anywhere in the arena. The host library mirrors the arena, so it treats whatever
  its programs write, and the panel areas they draw, as unknown, and leaves them out of `ARENA_CRC` and
  `PANEL_READ` checks.
- To hand data to a program, host code writes it and then `RAISE`s a `USER` trigger in the same packet:
  the handler runs after the packet and sees the data.
- A program instruction in progress delays a host packet by at most that instruction (one `DRAW` of the
  program's area at worst). For exact eye sync, keep program draws small or stop the program.
- The microphone is outside the VM: no instruction or property reaches it, and audio never enters the
  arena (§6.5).

## 6. The link

### 6.1 Transport and security
One GATT service; MCUmgr SMP (firmware update) runs beside it.

| Item | UUID | Properties | Content |
|---|---|---|---|
| service | `f3da4d59-277a-4fc9-a1ed-41c9c22d4400` | | |
| `CMD` | `f3da4d59-277a-4fc9-a1ed-41c9c22d4401` | write, write without response | packets of host code (§6.2) |
| `CTRL` | `f3da4d59-277a-4fc9-a1ed-41c9c22d4402` | write, write without response | control requests (§6.4) |
| `EVT` | `f3da4d59-277a-4fc9-a1ed-41c9c22d4403` | notify | events (§6.3) |
| `PAIR` | `f3da4d59-277a-4fc9-a1ed-41c9c22d4404` | read, write, encrypted link only (0.5.61+, write 0.5.62+) | read: u8 1; a write is ignored. Either makes a host whose stack pairs on demand (CoreBluetooth, Android) pair; the bondable rule below still decides |
| `FC` | `f3da4d59-277a-4fc9-a1ed-41c9c22d4405` | write, write without response (0.5.62+) | Faceclaw's records (§6.11) |
| `ANCS` | `f3da4d59-277a-4fc9-a1ed-41c9c22d4406` | write, notify (0.5.65+) | the iPhone notification relay (§6.12) |

- The host subscribes to `EVT`, exchanges the MTU (ATT MTU ≥ 65), then sends `HELLO`. A `CMD` packet is
  at most `max_packet` = min(MTU − 3, 244) bytes and a `CTRL` request at most 24 bytes. Write without
  response is the normal case; the protocol's acks make it reliable.
- **Advertising** (no host connected): connectable, every 100–150 ms, with the name `G1CFW_L_XXXXXX` or
  `G1CFW_R_XXXXXX`, its digits the low three bytes of the arm's address (0.5.66+; before: `G1v0_L`, `G1v0_R`,
  `G1v0S_L`, `G1v0S_R` in safe mode, and the service UUID, which no longer fits). The scan response carries
  manufacturer data for company 0xFFFF: u8 format = 2, u8 flags (bit 0 left arm, bit 1 board without a panel,
  bit 2 the pair ID is the frame serial, bit 3 safe mode), three u8 the firmware's version as in `HELLO`, then the
  pair ID in ASCII (1–16 bytes): the glasses' frame serial, the same on both arms. A board without a serial sends
  its 64-bit device ID in hex, with bit 2 clear. A host finds a pair by scanning actively and grouping arms by pair
  ID. Format 1, before 0.5.66, has no version and the pair ID right after the flags.
- **Address** (0.5.66+): the one Even's firmware has, Zephyr's default identity. A bond made on either firmware
  still works after an install in either direction, as far as that firmware kept its keys (Even's keeps one bond,
  and may not read the records this one writes). Before 0.5.66: an address from the chip's device ID.
- **Supervision timeout** (0.5.66+): under 2 s, the arm asks the central for 4 s at the same interval and latency, at
  most 5 times per connection, 5 s apart; the central decides. `NOTICE` `LINK` reports what it does.
- **Security**: in the charging case (`IN_CASE` = 1, §2.5: set by the case's power, field or messages and
  cleared by motion, wear or a 400 s silence, so a full arm lying in the case stays bondable and the rules
  end within 3 s of lifting the arm out; a case carried in a pocket may read 0 until its next re-check) the arm is bondable and obeys every write. Out of the case only a
  bonded, encrypted link is obeyed: writes on an unencrypted link are ignored, and a peer that does not
  encrypt within 10 s is disconnected. Pairing: LE Secure Connections, Just Works, up to 3 bonds; the
  oldest is replaced, also by a refused pairing attempt. One connection per arm; the host chooses the
  link parameters (`LINK`). The microphone (`MIC`, §6.5) needs an encrypted link, in the case too.

### 6.2 Packets and reliability
- A `CMD` packet is `u16 seq` followed by host code. Packets run in seq order and their instructions in
  order; an instruction never spans packets. **Host code** is the instructions marked H in §2.2: it has
  no jumps of its own, but `CALL` runs stored code as part of the packet (a call stack of its own, empty
  at the packet's start, and the jump budget of §5.4): `RET` at depth 1 returns to the packet, `END` ends
  the packet (the rest is skipped, no error), and `WAIT_EVENT` or `HALT` there is error 2. An error ends
  the packet: `ERROR` (§6.3) gives seq, the offset of the packet instruction (the `CALL`, for an error in
  called code) and the arena address of a failing called instruction; the instructions before it stay
  done.
- **Window**: Q = 32 packets. The arm stores packet s if exec_next ≤ s < exec_next + Q (16-bit
  arithmetic) and runs packets from exec_next while the next one is there. rx_next is the lowest seq not
  yet received; credits = exec_next + Q − rx_next.
- **ACK** reports rx_next, exec_next, credits, `sack` (bit i set: packet rx_next + 1 + i is held) and
  the device time. It is sent within 2 ms after a burst, at once on a hole or a duplicate, after each
  executed packet (coalesced), and repeated once 100 ms after one that reports a drained queue.
  Duplicates and out-of-window packets are acked but not stored; out-of-window ones also get `ERROR` 6.
- **Host**: it sends within the credits; resends a hole that the sack shows at once (at most once per
  round trip); and resends the oldest unacked packet when no ACK has moved rx_next for one RTO. The RTO
  follows RFC 6298: from samples R of packets never resent, SRTT = R and RTTVAR = R/2 at first, then
  RTTVAR = 3/4·RTTVAR + 1/4·|SRTT − R| and SRTT = 7/8·SRTT + 1/8·R; RTO = max(SRTT + 4·RTTVAR,
  max(4 connection intervals, 100 ms)), 1 s before the first sample, doubled per resend up to ×8, at
  most a few seconds. A fixed short timeout causes congestion collapse, because written packets can wait
  in the host's own Bluetooth stack for many intervals.
- A host that still waits for the arm with nothing unacked, and has heard nothing for about 300 ms,
  sends `HELLO` and applies its window fields.
- **Epoch**: a random non-zero u32, new at boot and at `RESET_SESSION`, in every event. Same epoch after
  a reconnect: the arena and the window are as the arm left them, and the host resends from rx_next. New
  epoch: the arena and the VM are not what the host thinks; it resets its mirror, uploads again and
  redraws.
- Control requests use no window: the control thread checks a request's arguments and replies on
  arrival, also while host code sleeps or draws. An action on the executor (`VM_START`, `VM_STOP`,
  `RESET_SESSION`, the `MIC` indicator) is applied by the executor before its next instruction, within
  one instruction's time (a program `DRAW` at worst; a sleeping packet does not delay it). `VM_STATUS`
  and `PROPS` read the executor's state without stopping it, so a body may mix the state before and
  after one instruction. A host retries a lost request or reply after a timeout with a new tag
  (`RESET_SESSION` and `REBOOT` may then run twice).

### 6.3 Events (`EVT`)
Every event starts with `u8 type`, `u32 epoch` and `u32 t_us` (the device time it was made). Events
made while no host is subscribed are lost, and an event longer than MTU − 3 is not sent.

| Type | Name | Fields after offset 9 | Size |
|---|---|---|---|
| 0x01 | `ACK` | 9: u16 rx_next, 11: u16 exec_next, 13: u8 credits, 14: u32 sack | 18 |
| 0x02 | `ERROR` | 9: u16 seq, 11: u8 offset (of the instruction in the packet's code), 12: u8 op, 13: u8 code, 14: u32 detail, 18: u32 address (of the failing instruction in the arena when it was reached by `CALL`, else 0) | 22 |
| 0x03 | `HALT` | 9: u8 trigger, 10: u32 address (of the instruction), 14: u8 op, 15: u8 code, 16: u32 detail | 20 |
| 0x04 | `NOTIFY` | 9: u16 count, 11: u8 source (trigger of the task; 0xFF host code), 12: u32 a, 16: u32 b | 20 |
| 0x05 | `NOTICE` | 9: u8 kind, 10: body | 10 + body |
| 0x06 | `MIC_DATA` | 9: u16 first (counter of the first frame), 11: u8 n (frames), 12: n × frame_bytes LC3 frames (§6.5) | 12 + n·frame_bytes |
| 0x07 | `INPUT` | 9: u16 count, 11: u8 trigger (ID of §5.2), 12: u32 value | 16 |
| 0x08 | `FC_ACK` | 9: u8 kind (1 ack, 3 nack), 10: u8 stream, 11: u16 0, 13: u8 lens (1 left, 2 right), 14: u16 decoded size, 16: u16 CRC (§6.11) | 18 |
| 0x80 + op | control reply | 9: u8 tag, 10: u8 status (0 ok, else an error code), 11: body (may be empty on an error) | 11 + body |

- `detail` for error 3 is the start of the failing range (`WRITE`/`WRITE_RLE` addr, `COPY`/`UNPACK` src or
  dst, the source bitmap of `BLIT`/`DRAW`, the `GLYPHS` text, `ASSET_LOAD` id or dst), the element address of
  an `LD`/`ST`, a jump target or fetch address, or `ASSET_LOAD`'s off when off + n passes the asset's end; for
  error 5 the offending value (property ID, mode, level, mask, trigger,
  coordinate, rate); else 0. For an error in code called from a packet, `op` is the failing instruction's.
  `NOTIFY.count` increases with every `NOTIFY` executed, sent or dropped; the first after boot or
  `RESET_SESSION` has count 1.
- `NOTICE` kinds: 1 link parameters (u16 interval ×1.25 ms, u16 latency, u16 timeout ×10 ms), 2 PHY
  (u8 tx, u8 rx; 1 = 1M, 2 = 2M), 3 panel fault at boot (u8 1), 4 link encrypted (u8 1), 5 microphone
  stopped by the firmware (u8 reason: 1 panel fault, 2 capture fault, 3 lease not renewed, 4 the case hold of 0.5.32–0.5.58),
  6 the case hold of 0.5.32–0.5.58 (u8 1: the arm lies in the case and holds its panel off, 0: it ended; §6.6), 7 the panel
  guard's cap changed (u8 `BRIGHTNESS_CAP`, u8 the level the panel shows; at most once per second, never for a
  `BRIGHTNESS` or `BRIGHTNESS_MAX` write, none while `PANEL_GUARD` is 0; §6.6), 8 a store job ended (§6.10: u8
  job, 1 put, 2 delete, 3 format, 4 crc; u8 result, 0 or the error code; then 16 bytes: the asset's ID, zeros for a
  format, or for a crc job u32 address, u32 n, u32 crc32 and u32 the bytes that are not 0xFF).
- `INPUT`: one event per firing of a trigger whose bit is set in the `EVENTS` mask (§6.4), sent whether
  or not a program runs (the firmware evaluates those triggers itself); `count` increases per `INPUT`
  sent or dropped. `value`: `TOUCH` the gesture, `WEAR` `WORN`, `TILT` `PITCH`, `LIGHT` `LIGHT`,
  `BATTERY` `BATT_PCT`, `MOTION` the change in mg, `CASE` bits 0 `IN_CASE`, 1 `CASE_LID`, 2
  `CASE_CHARGING` and bits 8–15 `CASE_BATT_PCT` (0xFF unknown), `PEER` `PEER_MSG`; 0 for `TIMER` and
  `USER`.

### 6.4 Control requests (`CTRL`)
Request: 0: u8 op, 1: u8 tag, 2: body. Body offsets below count from the body's first byte.

| Op | Name | Request body | Reply body |
|---|---|---|---|
| 0x01 | `HELLO` | | below (35 bytes) |
| 0x02 | `PING` | 0: u32 host_time | 0: u32 host_time (echo), 4: u32 t_rx, 8: u32 t_tx (§7) |
| 0x03 | `STATUS` | | diagnostics (below, 48 bytes) |
| 0x04 | `RESET_SESSION` | 0: u8 flags (bit 0: clear the arena) | 0: u16 packets dropped, 2: the `HELLO` body |
| 0x05 | `LINK` | 0: u16 interval_min, 2: u16 interval_max (×1.25 ms), 4: u16 latency, 6: u16 timeout (×10 ms), 8: u8 phy (0 keep, 1 1M, 2 2M) | 0: i8 param_result, 1: i8 phy_result (0 = requested; the central decides, `NOTICE` reports changes) |
| 0x06 | `LOG_READ` | 0: u16 offset | 0: u16 total, 2: u16 offset, 4: bytes of the arm's text log |
| 0x07 | `HEALTH_OK` | | 0: u8 unhealthy boots counted so far (then reset) |
| 0x08 | `REBOOT` | 0: u8 mode (0 normal, 1 clear the boot counter, 2 into safe mode) | empty, then the arm reboots |
| 0x0A | `PROPS` | 0: up to 12 u8 property IDs | 0: one u32 value per ID (§2.5) |
| 0x0B | `PANEL_READ` | 0: u16 y0, 2: u16 rows | 0: u32 crc32 of the rows as read back from the panel (320 bytes per row, left pixel in the high nibble), 4: u16 rows read, 6: their first 16 bytes; error 11 without a panel or while it is off (nothing to read back) |
| 0x0C | `ARENA_CRC` | 0: u32 address, 4: u32 length (0: to the end) | 0: u32 crc32 (zlib's) of the bytes, 4: their first 16 bytes (zero-padded); error 3 past the arena |
| 0x10 | `VM_START` | 0: u32 trigger table address | empty |
| 0x11 | `VM_STOP` | | empty |
| 0x12 | `VM_STATUS` | | below (32 bytes) |
| 0x13 | `MIC` | 0: u8 on (0 off, 1 on), 1: u8 rate (0 = 32 kbps, 1 = 16 kbps) | 0: u8 on, 1: u8 frame_bytes (40 or 20), 2: u8 frames_per_event, 3: u16 sample_rate (16000), 5: u8 frame_ms (10), 6: u16 lease_ms (2000); §6.5 |
| 0x14 | `EVENTS` | 0: u16 mask (bit i: report trigger i as `INPUT` events, §6.3); no body = query | 0: u16 mask in force = the request ∧ `HELLO.triggers` (0 at boot and after a disconnection) |
| 0x15 | `CASE_PANEL` | 0.5.32–0.5.58 only: u8 allow (1: the panel works in the case, 0: the case hold); no body = query | 0: u8 in force; since 0.5.60 (no case hold, §6.6) and before 0.5.32 status 2 |
| 0x16 | `PANEL_GUARD` | 0: u8 on (1 the panel guard works, 0 off: `BRIGHTNESS_CAP` 42 and no `NOTICE` 7); no body = query | 0: u8 in force (1 at boot; a disconnection and `RESET_SESSION` keep it); hosts only, for measurements (§6.6): no property, so a program cannot switch it off; a firmware before 0.5.52 answers 2 |
| 0x17 | `ASSET_INFO` | | 20 bytes (§6.10) |
| 0x18 | `ASSET_LIST` | 0: u8 first | 0: u8 total, 1: u8 first, 2: whole 32-byte directory entries (§6.10) |
| 0x19 | `ASSET_PUT` | 0: id[16], 16: u32 address, 20: u32 size, 24: u8 type | 0: u8 result: 0 job started, 1 already stored (§6.10) |
| 0x1A | `ASSET_DELETE` | 0: id[16] | empty: the job started (§6.10) |
| 0x1B | `ASSET_FORMAT` | 0: u32 0x534B4153; 4: u32 with 0x5057 in its high half: also wipe | empty: the job started (§6.10) |
| 0x1C | `FLASH_CRC` | 0: u32 address, 4: u32 n | empty: the job started; the crc comes in `NOTICE` 8 (§6.10) |
| 0x1D | `FC_INFO` | | 0.5.62+: 26 bytes (§6.11); status 7 in safe mode |
| 0x1E | `PROP_SET` | 0.5.63+: (u8 id, u32 value) up to 4 times | empty: what `SYS_WRITE` does for each, between two instructions, in order; an error goes only to the log. For a host that runs no instructions (§6.11) |

Ops 0x09 and 0x3F are reserved for diagnostics and test builds. A firmware before 0.5.56 answers 2 to ops 0x17–0x1C.

- **Safe mode**: an arm that boots repeatedly without a `HEALTH_OK` from the host starts in safe mode:
  `DRAW` fails with error 7, and `VM_START` and `MIC` are refused. `HEALTH_OK` confirms that the arm works.
- `ARENA_CRC`, `VM_START`, `PROPS` and `MIC` act on arrival, outside the packet order: send them once the
  packets they depend on have run (exec_next has passed them).
- `VM_START` (error 3 if the table does not lie in the arena, 7 in safe mode) drops any current task,
  clears the pending bits and the timer (the sensor settings stay, §5.2), sets `START` pending and runs
  the program; it also restarts a running one. `VM_STOP` drops the current task and stops the program; the panel,
  brightness and arena stay as they are. Both are applied by the executor between two instructions
  (§6.2): the reply follows the checks, the action comes within one instruction.

`HELLO` body (35 bytes):

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | protocol = 5 |
| 1 | u8 ×3 | firmware version: major, minor, patch |
| 4 | u8 | role: 0 right arm, 1 left arm |
| 5 | u8 | flags: bit 0 safe mode, 1 panel ok, 2 PMIC ok, 3 panel on, 4 charging enabled, 5 test build, 6 link encrypted, 7 in the case (`IN_CASE`, §2.5) |
| 6 | u8 | board: bit 0 no panel (development board), bit 1 bondable now, bit 2 microphone: `MIC` 1 captures and streams `MIC_DATA` (a build without capture clears the bit and refuses `MIC` with 12; the indicator never shows without capture, except in a test build, flags bit 5), bit 3 the iPhone notification relay answers (§6.12; not in safe mode) |
| 7 | u16 | triggers this firmware can raise (bit i = trigger i) |
| 9 | u16 | panel width = 640 |
| 11 | u16 | panel height = 480 |
| 13 | u32 | arena size = 262 144 |
| 17 | u8 | window Q = 32 |
| 18 | u16 | max_packet |
| 20 | u16 | rx_next |
| 22 | u16 | exec_next |
| 24 | u8 | credits |
| 25 | u32 | boot_count |
| 29 | u32 | reset_reason |
| 33 | u8 | safe_level |
| 34 | u8 | unhealthy boots |

`VM_STATUS` body (32 bytes): 0: u8 state (0 stopped, 1 running, 2 halted), 1: u8 trigger of the current
task (0xFF none), 2: u16 pending bits, 4: u32 trigger table address, 8: u32 address of the current task's
next instruction (when halted: of the instruction that halted), 12: u8 task (0 none, 1 running, 2 in
`SLEEP_UNTIL`, 3 in `WAIT_EVENT`), 13: u8 jumps left (exact below 255, else 255), 14: u8 call depth, 15: u8 halt code, 16: u32
`TIMER`, 20: u32 tasks started, 24: u16 coalesced triggers, 26: u16 `NOTIFY` events dropped, 28: u8
flags (bit 0 timer armed), 29: u16 run-time cap pauses (§5.4), 31: u8 busy_ms (program run time in the
last complete 100 ms period, at most 255). Without a task the next-instruction address, jumps left and
call depth are 0; after a halt the trigger is 0xFF and the halt code and address stay until the next
`VM_START`. Tasks started, coalesced triggers and cap pauses restart at `VM_START`; the `NOTIFY` counts
restart at `RESET_SESSION` only.

`STATUS` body (48 bytes): 0: u16 vbat_mv, 2: u16 ntc_code, 4: u8 charge_status, 5: u8 vbus_status, 6: u8
vterm_code, 7: u8 pmic_flags, 8: u8 charge_refused, 9: u32 packets, 13: u32 instructions, 17: u16 errors,
19: u32 draws, 23: u16 late (`SLEEP_UNTIL` of host code already past), 25: u16 duplicates, 27: u16
window_errors, 29: u16 event_drops, 31: u16 interval, 33: u16 latency, 35: u16 timeout, 37: u8 phy, 38:
u16 mtu, 40: u32 uptime_ms, 44: u32 i2c_errors.

### 6.5 Microphone
The arm's PDM microphone is a host device, not a VM device.
- **Only `MIC`** (§6.4) starts it; no instruction or property can, and audio never enters the arena.
  `MIC` needs an encrypted link, in the case too (status 12 otherwise), a microphone with capture
  (`HELLO.board` bit 2; status 12 without it: the indicator is lit only while audio is captured and sent) and a panel that is on or can be powered on (status 11;
  in the case also with 0.5.32–0.5.58, whose case hold keeps the panel off, §6.6); status 7 in safe mode.
- **Off** at boot, at every disconnection and when the link stops being encrypted; off at once on
  `MIC` 0. A stop stops the PDM clock, disables the peripheral, leaves the clock pin low and clears the
  indicator (level 0) if the panel is on.
- **Lease:** a `MIC` 1 holds the microphone for lease_ms (2 s, in the reply); a `MIC` 1 with the rate in
  force renews it and changes nothing else (the same reply), one with the other rate restarts the capture in
  that format (counter from 0). The host renews at least every second while it reads the frames (the
  library: when a `MIC_DATA` arrives and 0.8 s have passed). Without a renewal the arm switches the
  microphone off at the lease's end, clears the indicator and sends `NOTICE` 5 (reason 3), so a host that
  died with its link kept up (a killed process) does not leave the microphone on.
- **Indicator:** while the microphone is on the firmware draws a 16 × 16 block at level 15 in the panel's
  outer top corner, (620, 4) on the right arm and (4, 4) on the left, and redraws it after every `DRAW`
  that covers it. `MIC` 1 powers the panel on and **holds it on** until the microphone stops: a `PANEL` = 0 or 2
  from a program or host code meanwhile blanks the panel (all pixels 0, the indicator redrawn) and is
  remembered; `PANEL` reads the value written, a `DRAW` cancels it as usual, and the rails go off when the microphone
  stops if it is still pending. So a resident program (a notification mirror hiding its popup) cannot end
  a microphone session; `NOTICE` 5 reason 1 is left for a panel fault. The panel's ≈ 5 mA last only as
  long as the lease the host keeps renewing.
- **Format:** 16 kHz mono, LC3 frames of 10 ms: 40 bytes at 32 kbps (rate 0, the default) or 20 bytes at
  16 kbps (rate 1). `MIC_DATA` (§6.3) carries frames_per_event = min(5, floor((MTU − 3 − 12) /
  frame_bytes)) consecutive frames (at least 1) with the counter of the first one; the counter increases
  by one per frame, so the host sees a gap when the arm dropped an event (outgoing queue full) and
  conceals it (PLC). At MTU 247: 5 frames per event every 50 ms at both rates (≈ 4.2 KB/s at 32 kbps,
  ≈ 2.2 KB/s at 16 kbps), which keeps the bundling delay at 50 ms (`docs/audio-codec.md`, 2026-10-27). The
  host keeps the connection interval at 15 ms or less while the microphone is on.
- One arm at a time: the host picks one (stock used the right arm).

### 6.6 Power rules
- The panel is off at boot; the first `DRAW` or `PANEL` = 1 powers it on. `PANEL` = 2 (standby, ≈ 1 mA
  above off on the G1) is for a program or host that will draw again soon and wants the 4 ms wake instead of 71 ms
  (a notification popup); there is no automatic standby, a program chooses 0 or 2. With no host connected
  and no running program, the panel, on or in standby, switches off 30 s after the last `DRAW`. A running
  program manages the panel itself (`PANEL`). A lit panel showing only black still costs ≈ 4.6 mA: a host
  whose whole display is black puts the panel in standby (the library does this at `Flush`).
- Advertising: 100–150 ms for 60 s after boot or a disconnection and whenever in the case, then 1 s.
- The accelerometer runs at `IMU_RATE` (13 Hz at boot, low-power mode; while `IN_CASE` is 1 the firmware reads every
  sample, in batches of one second, to see a lift, §2.5); the gyroscope only at a
  `GYRO_RATE` above 0; the light sensor converts (800 ms) only while the panel is on, as stock runs it, and
  something uses it (a running program, the `LIGHT` bit in `EVENTS`, `AUTO_BRIGHTNESS`), else it is shut down
  and `LIGHT` is `UNKNOWN`; the TouchBar controller stays powered (its own rail, as stock) and scans only while
  something uses it (a running program with a `TOUCH` or `WEAR` handler or waiting for one, those bits in
  `EVENTS`), else it sleeps; it wakes the CPU through its interrupt line, nothing polls it; the case link is
  interrupt-driven and silent out of the case; the arm-to-arm link is off at
  boot (§6.9); the microphone is powered but clockless when off.
- The executor and the other firmware threads block; nothing polls faster than the PMIC's 1 s VBUS check (while the
  panel is on, its round adds the panel guard's burst of 16 conversions over ≥ 40 ms, below).
- Program run time is capped (§5.4). Host code is paced by the link and not capped.
- **In the case the panel works as anywhere** (0.5.60+): whether it is on there is the program's choice, as it is
  out of the case. The library's `PanelPolicy` (`../../src/README.md`) is the shared one: the panel off while
  `IN_CASE` is 1 or the glasses are folded (`WORN` 0), the program's `DRAW` and `VIDEO` showing nothing meanwhile.
  Why it matters: lit pixels load the case's NFC power so much that the case's messages fail and the charging
  falters (§6.8), and the panel's own power warms the pack during a top-up. 0.5.32–0.5.58 had a **case hold**
  instead: while `IN_CASE` was 1 the firmware kept the panel off (`DRAW` drew nothing, `PANEL` kept the value
  written and applied it when the hold ended, `PANEL_READ` and `MIC` answered 11, `NOTICE` 6 at both ends), and
  `CASE_PANEL` 1 (§6.4, for development) let the panel work there except during a top-up.
  While the panel is on in the case its LEDs are off (`A9`: no pixel driven, the image kept, `DRAW` and `PANEL_READ`
  work on it) from the case's first RF command until 2 s after its last, at a full arm's re-checks (one poll every
  ~300 s) and through a top-up (its polls every ~650 ms); then on again (`A3`, sync). Not while the microphone is on
  (its indicator stays lit). Until 0.5.58 only with `CASE_PANEL` 1.
- **The panel guard (0.5.52+, `docs/review-dimming.md`; this form 0.5.54+):** while the panel is on and the arm runs
  on its battery (no VBUS), the firmware keeps the mean discharge current under a budget (`CONFIG_G1_PANEL_BUDGET_MA`,
  100 mA = 1.25 C of the 80 mAh cell since 0.5.56: at 120 mA a pack rose 6–7 °C in 12 min of white; IBAT is the
  whole arm's) and VBAT away from the PMIC's 2.8 V power-fail cut (a
  reset that loses the retained log), by `BRIGHTNESS_CAP`: the panel shows min(`BRIGHTNESS`, `BRIGHTNESS_MAX`,
  `BRIGHTNESS_CAP`). The panel's current is pulsed, so while it is on each 1 s PMIC round takes a burst of 16 VBAT+IBAT
  conversions over 40 ms or more at unequal spacings (invalid ones discarded): its mean IBAT is the round's current,
  its lowest VBAT the round's voltage. The budget rules use I_avg, an exponential average of the rounds' currents
  (α = 1/4, τ ≈ 4 s). Over the budget (or `BATT_TEMP` ≥ 38.0 °C) the cap goes, at most once per 4 s, in one step to the
  highest level whose full-white current (15, 80, 190, 275, 355 mA at levels 6, 12, 21, 30, 42, linear between) times
  the content factor I_avg / I_white(level shown) is within 95 % of the budget, never below 6; VBAT under 3150 mV
  takes 2 levels and under 3000 mV halves the current register (level − 6), once per round. Up: +1 level per 5 s when
  the whole 5 s had I_avg ≤ 85 % of the budget, VBAT ≥ 3400 mV and `BATT_TEMP` < 36 °C, +3 per 5 s when I_avg stayed
  ≤ 50 %; between those thresholds the cap holds. At each power-on (from off or standby) the cap starts from the last
  VBAT sample taken with the panel off or in standby: ≥ 3600 mV 42, else 24; while charging it is 42; while the panel
  is off or in standby `BRIGHTNESS_CAP` reads 42. A change is one short register write between frames. `NOTICE` 7
  reports each change of the cap (§6.3); `BRIGHTNESS` reads the written value, so `AUTO_BRIGHTNESS`'s hysteresis and
  the level the right arm sends to the left use it too. **The eyes:** each arm guards its own cell (the right trips
  first); the library writes one eye's cap as the other's `BRIGHTNESS_MAX` (`SCOUTER_G1_MATCH_BRIGHTNESS`, default 1),
  so both show the lower level, and 42 again when that eye's link drops or the session ends; without a host the eyes
  may differ while one is capped. **Off** for measurements: `PANEL_GUARD` 0 (§6.4), until 1 or a reboot. A program
  reads `BRIGHTNESS_CAP`; there is no trigger for it.
  **Brightness advice:** the whole panel white draws about 15, 80, 190, 275 and 355 mA at levels 6, 12, 21, 30 and
  42 (both arms, `docs/power.md` §17; with the arm's own current a white screen holds ~113 mA at level 12–13), so
  a mostly white screen is held near level 10–12 and capped even at the default 21; the current scales with the
  lit pixels, so text and lines on black stay far below it.
- Charging is configured exactly as stock and not changed by this protocol. `BATT_MA` and `BATT_TEMP`
  (§2.5) make the arm's own current and battery temperature readable through `PROPS`.

### 6.7 TouchBar, wear, light and gyroscope
Firmware drivers; the VM and the host see them through the properties, triggers and `INPUT` events
above. How stock drives each part and the sources to port from: `docs/review-v5.md` §8.
- **TouchBar.** The AW9320x capacitive controller on the arm, configured with stock's table, signals touch
  down and up on its interrupt line (its proximity enter and exit events, which stock reads); the firmware
  classifies them with stock's timings and raises `TOUCH` with `TOUCH_GESTURE`: presses less than 305 ms
  apart count together and are reported 305 ms after the last release as 1 tap, 2 double tap or 3 triple
  tap (more are ignored); a press held 458 ms reports 4 long press start, its release 5 long press end. A
  tap is reported once it is classified (a double tap is not preceded by a tap). Stock's configuration has
  no slide, so 6 slide forward (toward the lens) and 7 slide back are not produced. The configuration table
  comes from the stock image through the fetch script; the firmware loads it again when the controller
  reports a reset.
- **Wear.** `WORN` is 0 in the case, and a change raises `WEAR`. Stock's worn and removed reports do not come
  from the TouchBar controller (whose in-ear and out-of-ear events stock's table leaves disabled and this
  firmware enables, as a confirmation): they follow a value the network core
  reports with its arm-to-arm packet counts (decomp `check_work_mode`: worn at 51 and above, removed below
  40 for several rounds: the received signal strength of the other arm's last frame, in −dBm, so a small value
  is a strong signal; probably strong with the temples folded and weaker with them open). That value, from the network core's report (§6.9; about every 10 s, at once when it
  crosses a threshold), decides `WORN` as in stock: 51 and above worn at once, below 40 for 3 s not worn, 0 no
  reading yet; the controller's worn event sets 1 at once, its out-of-ear event counts only before the first
  report.
- **Light.** The OPT3007 sits on the right arm: stock sets it up only there (decomp `panel_on`: the ESB
  master), and the left arm of this pair has none at its address, so the left reports no `LIGHT` trigger in
  `HELLO.triggers`, reads `LIGHT` `UNKNOWN` and takes `AUTO_BRIGHTNESS` = 1 only while `PEER_LINK` is 1. Its eye follows the
  light through the host, which reads the right arm's `BRIGHTNESS` (or `LIGHT`) and writes the left's, or
  through the arm-to-arm link (§6.9: the firmware carries the right arm's level and the left applies it with `AUTO_BRIGHTNESS` = 1). The
  sensor is powered with the panel's rails. It converts continuously (800 ms) while in use (§6.6); `LIGHT` is
  in 0.01 lux.
  With `AUTO_BRIGHTNESS` = 1 the firmware sets `BRIGHTNESS` (the written level, which the panel guard may cap, §6.6)
  from each reading with stock's table:
  lux < 50 → lux/5; < 200 → 10 + (lux − 50)/10; < 1000 → 25 + (lux − 200)/50; < 2000 → 41; else 42. A
  new level is applied when two consecutive readings ask for one that differs from the current level by
  2 or more (hysteresis). Writing `BRIGHTNESS` while `AUTO_BRIGHTNESS` = 1 holds until the next reading;
  a program with its own curve keeps `AUTO_BRIGHTNESS` = 0 and uses `LIGHT`.
- **Gyroscope.** `GYRO_RATE` turns the LSM6DSO's gyroscope on at that rate (±500 dps, low-power mode below
  104 Hz); `GYR_X/Y/Z` hold the latest sample in mdps, mapped to the head frame like `ACC_*`. Off at boot.

### 6.8 Case link (NFC)
The charging case talks to each arm through the ST25DV tag's mailbox (the 13.56 MHz charging link): it
polls every ~650 ms for the arm's state and sends commands. The firmware
- answers every poll as stock does, so the case charges and shows the arms as with stock. The poll is
  `13 00 nn 00 bb ll` (bb: bits 0–6 the case's battery in %, bit 7 its own charger connected; ll: 7 lid open, 8
  lid closed, 6 lid closed for a long trip); the reply `13 01 02 06 st vv vv pp cc mm` + sum8: st 0 charging, 1
  full, 2 battery too warm, 4 powered but not charging; the rectifier voltage in mV (big endian), the battery in
  %, the charging flag, the current in mA (0x80 asks the case to look again);
- feeds the `IN_CASE` state (§2.5) with every message and with the tag's RF field (`EH_CTRL_Dyn` FIELD_ON, read
  every second while `IN_CASE` is 1 and every 30 s otherwise); takes `CASE_LID`, `CASE_CHARGING` and
  `CASE_BATT_PCT` from the poll's bytes as stock does; 3 s without a message clears those three to `UNKNOWN`;
  every change raises `CASE`;
- enables the charger after each reply with status 0 or 4, as stock (the case has no disable command), and
  obeys nothing else: the case's long-trip (ship mode, `06 01 03 01 00`) and restart (`0B 01 03 01 00`)
  commands are answered as stock and logged, not obeyed (ship mode is a user decision).
- **What breaks the link** (`docs/power.md` §15): the case writes its poll into the tag's mailbox and reads the
  arm's reply 75 ms later; the arm's reply is always there by then. What fails is the 13.56 MHz link itself
  (the case's writes and reads of the tag, "card lost") when the arm's lit pixels load the case's power
  transfer: the rectifier voltage falls from about 5.8 V to 4.3 V. A black but powered panel, Bluetooth traffic
  and the CPU do not. A lost poll makes the case keep its power transfer on, which warms the packs (it stops
  only at a reply "full"); and an arm powered but not charging for several polls sends stock's one-off 0x80, at
  which the case restarts its own controller. Hence the LED blanking of §6.6 and the programs' `PanelPolicy` (until
  0.5.58 the case hold).

### 6.9 Arm-to-arm link (ESB)
The arms share a private 2.4 GHz link (Nordic ESB in the stock network core: the right arm transmits and
the left answers inside the acknowledgement; channel and addresses come from the factory record). Stock
drives the left display through it; this protocol uses it only as a **message link**: the host keeps the
eyes in step, and programs may exchange one 32-bit word per frame.
- The network core makes one exchange every 50 ms on its own (its announcements) and holds one frame of the
  app core at a time, which it sends instead of the next announcement (right arm) or returns in its next
  acknowledgements (left arm), and reports when the frame went out. `PEER_LINK` = 1 starts the message link:
  the right arm sends a frame at once when there is news (`PEER_SEND` written, its `BRIGHTNESS` changed) and
  otherwise every 100 ms (a poll); the left arm sends one at once when its `PEER_SEND` was written and
  otherwise every 200 ms. A frame carries the sender's value until the other arm acknowledges it, so a value
  arrives once, even if a frame or a report is lost; a write replaces a value that has not arrived yet. A
  delivered value sets `PEER_MSG` and `PEER_T_US` on the receiver and raises `PEER` (pending bits coalesce: a
  burst keeps the last value). `PEER_LINK` reads 2 while the other arm's frames arrive, 1 while they do not, 0
  when off. Both arms must have `PEER_LINK` = 1; `RESET_SESSION` sets it to 0.
- **Timing** (two emulated arms and the arms in the case, `docs/power.md` §12): right to left takes 6–93 ms (a
  value waits for the next exchange, and for a poll already in the network core), left to right 50–150 ms (its
  frame rides in the acknowledgement of the exchange after the next), a round trip 60–240 ms (110 ms median on
  the arms). Both arms see the same
  exchange (the right as its report, the left as the frame's arrival), so the link learns the offset between
  the two clocks (within about 0.2 ms), and `PEER_T_US` is the sender's write time in the receiver's clock.
- **Drawing together without the host**: the sender reads t = `TIME_US`, writes `PEER_SEND`, then
  `SLEEP_UNTIL` t + lead and draws; the receiver's `PEER` handler does `SLEEP_UNTIL` `PEER_T_US` + lead and
  draws. The lead must exceed the one-way time: 200 ms works both ways (`SLEEP_UNTIL` allows 2 s). Both
  images change within about 0.3 ms of each other (emulator, panel times). The host follows the traffic
  through `INPUT` events.
- The network core runs ESB in stock's mode 0 on both arms (its quiet mode breaks the master's BLE,
  `docs/power.md` §5): its own announcements run whatever `PEER_LINK` says, and the app core's frames take
  their place, so `PEER_LINK` adds no radio traffic, only the app core's work per frame. A firmware without
  the message link reports `HELLO.triggers` bit 9 clear and refuses `PEER_LINK` writes with error 5.
- Besides program messages the link carries firmware data: every frame of the right arm carries its
  `BRIGHTNESS`, and a left arm with `AUTO_BRIGHTNESS` = 1 applies it (the left has no light sensor); the
  network core's link value (the signal strength of the other arm's last frame, in its periodic report)
  feeds `WORN` (§6.7) whatever `PEER_LINK` says.

### 6.10 Asset store (0.5.56)
Each arm keeps **assets**, data blobs that the host names, in its QSPI flash: fonts, bitmaps, anything reused
across sessions. An asset's **ID** is the first 16 bytes of the SHA-256 of its body, so a host sees what an arm
holds and uploads a body once; after a reboot or a new session, `ASSET_LOAD` (§2.4) brings it back into the arena
in milliseconds. A body is 1…262 144 bytes (it is staged in the arena) and carries a u8 **type** that the arm
keeps but ignores: 1 a font blob (§4.2), 2 a 4-bpp bitmap (`"SKB1"`, u16 w, u16 h, u16 stride, u16 0, then the
rows), 3 a 1-bpp bitmap (the same header), 4 raw. Nothing runs or draws from flash: programs stay in RAM, the store
never starts anything, and no program writes or erases flash. The store holds up to 120 assets in whole 4 KiB
sectors of the partition 0x140000–0x1000000 (14.75 MiB; the DK 0x140000–0x800000), which takes the place of
stock's glyphs and pictures; the store never writes below 0x140000 (the update slots, the factory record).
`RESET_SESSION`, `VM_START`, a reboot and an epoch change leave the store as it is.

**Jobs.** `ASSET_PUT`, `ASSET_DELETE`, `ASSET_FORMAT` and `FLASH_CRC` start a job and reply at once; the arm runs
one job at a time beside the executor (packets and programs go on) and ends each with `NOTICE` 8 (§6.3) and the
state 4 (done) or 5 (failed) of `ASSET_INFO`. A put erases the sectors it needs one at a time, copies the staged
range to flash, reads it back computing SHA-256 and crc32, and commits a new directory: the asset exists only once
`NOTICE` 8 reports 0, and a reset before that leaves the store as it was (a job never survives a reboot). The arm
reads the staged range during the job: the host leaves it unchanged until the job ends; a change fails the
verification (error 13), never the store. Times: about 2 s for 64 KB in the emulator (each 4 KiB erase and
program costs the driver's 50 ms busy poll); the arm's in DEVELOPMENT.md.

`ASSET_INFO` body (20 bytes): 0: u32 data bytes (the assets' sizes), 4: u32 free bytes (whole free sectors), 8:
u32 largest free extent, 12: u8 assets, 13: u8 job state (0 idle, 1 erasing, 2 writing, 3 verifying, 4 done, 5
failed), 14: u8 the last job's error (0 ok), 15: u8 flags (bit 0: the store is available), 16: u32 seq (the
directory's commit count, 0 for an empty store).

`ASSET_LIST` 0: u8 first → 0: u8 total, 1: u8 first, 2: the entries from index first on, as many whole entries as
fit the reply (at most 7; an MTU of 185 allows 5); the host pages by first + the number received. Entry (32 bytes):
0: id[16], 16: u32 offset in the partition, 20: u32 size, 24: u32 crc32 (zlib's) of the body, 28: u8 type, 29: u8
flags (0), 30: u16 0.

`ASSET_PUT` 0: id[16], 16: u32 address, 20: u32 size, 24: u8 type → status 0 with the result 0 (the job started)
or 1 (already stored: the same ID and size; nothing written, no job); refused with an empty body: 13 (a job runs,
no space or no free entry, no store), 3 (the range is not in the arena), 5 (size 0 or above 262 144, or the ID is
stored with another size), 7 (safe mode). The job fails with 13 when the read-back's SHA-256 does not begin with the
ID (a wrong ID, or the range changed).

`ASSET_DELETE` 0: id[16] → status 0 (the job started), 13 (no such asset, a job runs, no store) or 7.
`ASSET_FORMAT` 0: u32 0x534B4153 (else 5) frees everything (seq 0 again); with a second u32 whose high half is
0x5057 it also **wipes**: it erases every sector of the partition that is not already erased, minutes when stock's
data is still there. A host that installs stock's firmware wipes first, so stock meets erased flash where its
assets were (they do not come back). Status 0, 13 or 7.

`FLASH_CRC` 0: u32 address, 4: u32 n (n ≥ 1, address + n ≤ 0x1000000 and inside the chip, else 5) → status 0 or
13 (a job runs) or 7; a read-only job whose `NOTICE` 8 gives the crc32 (zlib's) of those bytes and how many are not
0xFF. It fingerprints the flash outside the store, which the store never writes.

In safe mode `ASSET_INFO` and `ASSET_LIST` work and the others answer 7. Without a usable flash the store is
unavailable: `ASSET_INFO` flags bit 0 clear, the jobs refused with 13, every `ASSET_LOAD` error 13.

### 6.11 Faceclaw's display path (0.5.62)
A second way to draw, beside the VM, for a host that composes each eye's frame itself and sends what changed. No
instruction runs: the first record stops any program and takes the arena, until `RESET_SESSION`:

| Arena bytes | Content |
|---|---|
| 0x00000–0x003FF | left to the VM (registers, table) |
| 0x00400–0x25BFF | the frame, 153 600 bytes: 640 × 480, 4 bpp, as a bitmap of §3.1 |
| 0x25C00–0x31BFF | the inflater: 32 KiB of history and a decoded message of up to 16 384 bytes |
| 0x31C00–0x3FFFF | the receive ring, 58 368 bytes |

**Packets.** A write to `FC`: u8 opts (0x80 first of the record, 0x40 last), u8 stream (the host's ID of this attempt,
echoed in `FC_ACK`), then the next bytes of the record. Writes are obeyed under the rules of §6.1.

**Record:** u8 flags, u16 wire (bytes of the body), u16 crc, body. Flags: bits 0–1 the lens, bit 2 (4) the body is
compressed, bit 3 (8) a new stream starts here. The compressed bodies are one zlib stream (RFC 1950, a window up to
32 KiB), each ending in `Z_SYNC_FLUSH`, which is never finished; a body that isn't compressed adds nothing to its
history. crc is CRC-16/CCITT-FALSE of the decoded message. A record's body is at most 16 640 bytes on the wire.

**`FC_ACK`** answers each record once it has been applied. A nack (crc, a bad stream, a refused message, no room in the
ring) means the host goes back: after any nack, or a record dropped for any reason, compressed records are refused
until one with bit 3 arrives, so a host repeats everything unanswered from a new stream. Repeating a message is safe:
draws overwrite and `PRESENT` is idempotent. Up to three records of 16 640 bytes fit the ring.

**Messages** (the decoded record; byte 0 & 127 is the type, others are acknowledged and ignored):
- 26, draw: u16 count, then count × (u16 length, call). A call is u8 opcode, u8 flags = 0, arguments. Opcode 1, bbox: u8 0
  and u8 x/4, y/2, w/4, h/2, or u8 1 and u16 x, y, w, h; then runs covering exactly w × h pixels left to right, top to
  bottom, across rows: [n: high nibble, 1…15 | colour] or [colour][n8] or [colour][0][n16, little-endian]. Opcode 9, clear:
  u8 colour. Other opcodes and flags are refused. Draws change the frame only.
- 28, present: what changed in the frame since the last present goes to the panel, latched at once (a rectangle's x
  rounded down to a whole byte), powering it on first, the whole frame if it had lost its image. As `DRAW`, for §6.6.
- 30, brightness: u8 1, u8 target (0…100, the level is `BRIGHTNESS` = max(1, target · 42 / 100) rounded), u8 visible,
  u16 fade (ignored). Not visible, the panel goes to standby and presents wait in the frame; visible again, it comes
  back with the next present, which the host sends with it.

While the path is open the arena belongs to it, so a host reads properties with `PROPS`, writes them with `PROP_SET`, and
hears triggers as `INPUT` (`EVENTS`). Packets also renew the microphone's lease (§6.5), and when the microphone stops its
indicator's corner is repainted from the frame.

`FC_INFO` body: 0: u8 version (1), 1: u8 flags (bit 0 the path is open), 2: u16 max message, 4: u16 max wire, 6: u32 ring
size, 10: u32 history, 14: u32 records, 18: u32 nacks, 22: u32 presents.

### 6.12 The iPhone notification relay (0.5.65)
A host on an iPhone gets the phone's own notifications through the arm: iOS offers ANCS (Apple Notification Center Service)
only to the peripheral it is connected to, as a GATT server, and has no API for an app to read them itself. So the arm is
the ANCS consumer and passes everything on, over the same link. The host reads, keeps and draws the notifications; the arm
doesn't look inside them and keeps no list.

The wire format is that of Faceclaw's custom firmware for the G2 (one relay client serves both), on a characteristic of its
own. All integers little-endian.

**Commands**, a write with response to `ANCS`: `41 4E 01 op token:u32 [body]`. token is nonzero and chosen by the host.
- 0 `START`, 8 bytes: end any session of this link, then look for ANCS (primary service, its characteristics, their CCCs),
  subscribe to Data Source, and write Notification Source's CCC 0, then 1, which makes iOS list what it has. A session that
  was running is replaced, without a `STOP`. Refused while an earlier session's requests are still out.
- 1 `STOP`, 8 bytes, with the session's token: both CCCs to 0. The frames not yet sent are erased.
- 2 `CP`, at most 80 bytes, with the session's token and once it is ready: the body is written to the Control Point. Only
  ANCS's commands: 0 (get notification attributes, ≥ 6 bytes), 1 (get app attributes, ≥ 4 bytes, ending in the identifier's
  terminator and an attribute ID of 0) and 2 (perform action, 6 bytes, action 0 or 1). One at a time.

The answer to a write is an ATT error when it is refused: 0x08 not allowed now (below), or no session with this token, or
`START` while requests are out; 0x0D malformed, or `CP` before ready; 0x11 no room (a `CP` still out).

Who may write: an encrypted link, in the case too (the case's rule of §6.1 lets anyone write, and this reads the phone's
notifications), a host subscribed to `ANCS`, and no safe mode. Nothing else grants anything: the token only lets the host
tell this session's frames from an older one's. One link per arm, so whoever may write is the phone the frames came from.

**Frames**, notifications on `ANCS`: `41 4E 01 kind token:u32 sequence:u16 flags [body]`, at most min(MTU − 3, 244) bytes.
Flags: bit 0 the first frame of a value, bit 1 its last. sequence counts every frame of the session from 0 (`START`'s status
is 0) and wraps; a frame the arm has no room for still takes its number, so a gap means something was lost and the host
starts over with a new `START`.

| kind | body |
|---|---|
| 0 | u8 status: 0 `START` taken, 1 ready, 2 a failure (of a CCC write, a `CP` the phone refused, or discovery), 3 this phone isn't offering ANCS (Share System Notifications is off, or it isn't an iPhone) |
| 1 | a Notification Source value, 8 bytes. May come before status 1, as iOS lists what it has |
| 2 | a Data Source value, in as many frames as it takes |
| 3 | none: the phone accepted the last `CP` |

After status 3, or 2 from a `START`, there is no session; the host may `START` again at any time, and each `START` looks for
ANCS again, so that iOS withdrawing and offering it needs no other handling. A `CP` the phone refuses (status 2) leaves the
session running.

**Lifetime.** No GATT request goes to the phone before a `START`, so a host that isn't a relay client sees no difference. The
session ends with the link: its subscriptions aren't restored at a later connection and nothing goes to another connection.
Frames wait in an 8 KiB RAM ring until the link can take them, erased as they leave and whenever a session or the link ends;
the host stack's own buffers hold the last ones until reused. Nothing is written to flash, and nothing about notifications
goes to the log. There is no lease: g1v0 has no consumer of its own to return ANCS to, and CoreBluetooth ends the link when
the last app using it goes. Any app on the phone that may use this link can do what the host does, which the arm cannot tell
apart.

## 7. Time and eye sync
- **Clock model** (`PING`): the host sends its time s; the arm replies with t_rx (receive) and t_tx
  (send); the host notes the reply's arrival r. Offset = t_rx − s + ((t_tx − r) − (t_rx − s))/2 (mod
  2^32), delay = (r − s) − (t_tx − t_rx). Keep the sample with the smallest delay of the last ~16 (its
  error is at most delay/2) and repeat every few minutes (drift ≤ ~130 ppm).
- **Both eyes at once**: compose on both arms; once both have acked everything, choose T = now + lead
  (lead ≈ 2 connection intervals + clock error + 15 ms) and send each arm a packet that writes T, in that
  arm's clock, to a word, then `SLEEP_UNTIL` it and `DRAW`s. Both images change at about T plus the
  DRAW's own time, about 1 ms apart. To measure it, end the packet with `SYS_READ` of `TIME_US` and a
  `NOTIFY`.
- **Animation**: a packet or a program that alternates `SLEEP_UNTIL` and `DRAW` from frames already in
  the arena plays at exact times, independent of the link.
- **Programs on both arms** run independently. The host keeps them in step through their data: it
  writes times in each arm's own clock, as the clock of §9.2 does, and refreshes them every few minutes.

## 8. Limits
| Item | Value |
|---|---|
| arena | 262 144 bytes |
| panel | 640 × 480, 4 bpp, brightness 0…42 |
| `CMD` packet | ≤ max_packet ≤ 244 bytes, seq included |
| receive window | 32 packets |
| `CTRL` request | ≤ 32 bytes (24 before 0.5.56) |
| jumps per activation | 1000 |
| call depth | 16 |
| program run time | 30 ms per 100 ms |
| `SLEEP_UNTIL` | ≤ 2 s |
| `TIMER` | < 2^31 µs ahead (more counts as past) |
| font | ≤ 65 535 glyphs, each ≤ 255 × 255 |
| `LINE` coordinates | −32768…32767 |
| `VIDEO` frame | 1…640 × 1…480 pixels, state 1596 bytes |
| microphone | 16 kHz mono, LC3 10 ms frames of 40 or 20 bytes, ≤ 5 frames per event |
| `EVENTS` mask | u16, triggers 0–15 |
| asset store | ≤ 120 assets of 1…262 144 bytes in 14.75 MiB (the DK: 6.75 MiB), whole 4 KiB sectors |
| arm-to-arm link | one u32 per frame; one frame in flight per arm; the network core's exchange every 50 ms; the right arm's poll every 100 ms, the left's keep-alive every 200 ms |

## 9. Examples

### 9.1 Layout
Both examples use this arena layout. The font at 0x25800 has glyph 62 = space, 63 = `%`, 64 = `:`.

| Address | Content |
|---|---|
| 0x00000–0x0003B | r0–r14 |
| 0x00040–0x0007F | trigger table |
| 0x00080–0x000EB | constant pool (27 words, below) |
| 0x00100 | text buffer: glyph indices, one per byte |
| 0x00108 | clock mailbox: words 66 (`M'`), 67 (`N'`) |
| 0x00110 | clock state: words 68 (`M`), 69 (`N`) |
| 0x04000 | band bitmap, 4 bpp |
| 0x0C000 | code |
| 0x25800 | font blob |

Constant pool, uploaded with `01 80 00 00 6c` and 108 bytes: word 32 + j holds the j-th value of 0, 1,
2, 3, 4, 5, 8, 10, 12, 15, 17, 20, 24, 32, 34, 40, 60, 80, 100, 160, 240, 256, 560, 1440, 16384,
153600, 60000000. For example C0 is word 32 (0x20), C10 word 39 (0x27), C256 word 53 (0x35), C16384
word 56 (0x38), C153600 word 57 (0x39). Property IDs among them: 17 `BATT_PCT`, 34 `TIMER`.

Each program is uploaded with `WRITE` (code at 0xC000, then its table at 0x40: `01 40 00 00 40` and 16
words); once these packets have run, `VM_START` = `10 <tag> 40 00 00 00`.

### 9.2 A clock that redraws once a minute
`HH:MM` in a 160 × 32 band (stride 80) shown at panel (240, 8). The program keeps the minute of the day M
and the device time N of the next minute. The host does not run a clock on the arm: it writes `M'` (the
current minute) and `N'` (the next minute's start in this arm's clock) into the mailbox, and the `START`
or `USER0` handler copies them. Since tasks never overlap, the copy cannot mix with a tick in progress.
To correct the clock later, the host sends one packet: `WRITE` of `M'` and `N'`, then `RAISE C12`
(`42 f0 28 00`).

Setup: the text buffer `01 00 01 00 05 00 00 40 00 00` (glyphs 0 0 : 0 0); the mailbox `01 08 01 00 08`
+ `M'` + `N'`; the trigger table with word 0 (`START`) = 0xC000, word 1 (`TIMER`) = 0xC010, word 12
(`USER0`) = 0xC000, the others 0.

Code (176 bytes at 0xC000; at most 1 jump per activation):
```
0c000  10 ff 44 00 42 00             sync: MOV [M], [M']        ; START, USER0: take the host's time
0c006  10 ff 45 00 43 00                   MOV [N], [N']
0c00c  30 2b c0 00                         JMP show
0c010  11 ff f0 44 00 44 00 21 00    tick: ADD [M], [M], C1     ; TIMER: next minute
0c019  15 ff f0 44 00 44 00 37 00          MODU [M], [M], C1440 ; midnight
0c022  11 ff f0 45 00 45 00 3a 00          ADD [N], [N], C60000000
0c02b  14 4f f0 44 00 30 00          show: DIVU r4, [M], C60    ; hour
0c032  15 5f f0 44 00 30 00                MODU r5, [M], C60    ; minute
0c039  14 64 f0 27 00                      DIVU r6, r4, C10
0c03e  26 ff 60 35 00 20 00                ST8 C256, C0, r6     ; text[0] = tens of hours
0c045  15 64 f0 27 00                      MODU r6, r4, C10
0c04a  26 ff 60 35 00 21 00                ST8 C256, C1, r6     ; text[1]
0c051  14 65 f0 27 00                      DIVU r6, r5, C10
0c056  26 ff 60 35 00 23 00                ST8 C256, C3, r6     ; text[3]
0c05d  15 65 f0 27 00                      MODU r6, r5, C10
0c062  26 ff 60 35 00 24 00                ST8 C256, C4, r6     ; text[4]
0c069  50 ff ff ff f0 38 00 31 00 20 00 20 00 33 00 2d 00 20 00
                                           FILL C16384, C80, C0, C0, C160, C32, C0
0c07c  52 ff ff ff ff f0 39 00 38 00 31 00 24 00 2c 00 22 00 29 00 35 00 25 00
                                           GLYPHS C153600, C16384, C80, C4, C24, C2, C15, C256, C5
0c094  53 ff ff ff ff 38 00 31 00 20 00 20 00 34 00 26 00 33 00 2d 00
                                           DRAW C16384, C80, C0, C0, C240, C8, C160, C32
0c0a9  45 ff 2e 00 45 00                   SYS_WRITE C34, [N]   ; TIMER = N
0c0af  33                                  END
```
`FILL` clears the band, `GLYPHS` draws the 5 glyphs at 0x100 with the pen at (4, 24), `MASK`, level 15,
and `DRAW` shows the band at (240, 8). If `N'` has already passed when the program starts, `TIMER` fires
at once and the tick catches up.

### 9.3 A battery percentage that redraws on change
The percentage and `%` in an 80 × 24 band (stride 40) shown at panel (560, 8). One handler serves `START`
and `BATTERY`: the loop of §4.3 with a 3-digit buffer whose byte 3 holds `%`. An `UNKNOWN` reading (above
100 when unsigned) leaves the display as it is.

Setup: the text buffer `01 00 01 00 04 00 00 00 3f` (byte 3 = glyph 63, `%`); the trigger table with
words 0 (`START`) and 5 (`BATTERY`) = 0xC000, the others 0.

Code (113 bytes at 0xC000; at most 2 jumps):
```
0c000  44 0f 2a 00                         SYS_READ r0, C17     ; BATT_PCT
0c004  37 f0 70 c0 00 32 00                BLTU C100, r0, done  ; above 100: unknown
0c00b  10 1f 23 00                         MOV r1, C3           ; i = 3, the '%' byte
0c00f  12 11 f0 21 00                next: SUB r1, r1, C1       ; i -= 1
0c014  15 20 f0 27 00                      MODU r2, r0, C10     ; digit
0c019  26 f1 20 35 00                      ST8 C256, r1, r2     ; text[i] = digit
0c01e  14 00 f0 27 00                      DIVU r0, r0, C10
0c023  36 0f 0f c0 00 20 00                BNE r0, C0, next     ; until 0
0c02a  11 31 f0 35 00                      ADD r3, r1, C256     ; address of the first digit
0c02f  12 4f 10 24 00                      SUB r4, C4, r1       ; n = digits + 1
0c034  50 ff ff ff f0 38 00 2f 00 20 00 20 00 31 00 2c 00 20 00
                                           FILL C16384, C40, C0, C0, C80, C24, C0
0c047  52 ff ff ff f3 40 39 00 38 00 2f 00 22 00 2b 00 22 00 29 00
                                           GLYPHS C153600, C16384, C40, C2, C20, C2, C15, r3, r4
0c05b  53 ff ff ff ff 38 00 2f 00 20 00 20 00 36 00 26 00 31 00 2c 00
                                           DRAW C16384, C40, C0, C0, C560, C8, C80, C24
0c070  33                            done: END
```
For 42 % the loop ends with i = 1, so `GLYPHS` draws 3 glyphs (4, 2, `%`) from 0x101; for 100 % it draws
4 from 0x100, and for 0 % `0%` from 0x102.

## 10. Decisions (2026-10-26, `docs/review-v5.md`)
1. Host code may `CALL` stored code (§6.2): a packet `WRITE frame; CALL draw` keeps the packet pipeline and
   reuses stored code, where `RAISE` coalesces repeats and costs a round trip per frame.
2. A program may write and draw anywhere: it is the host's own code; the sandbox protects the arm, and
   audio never enters the arena. No program writes or erases flash; programs may read stored assets
   (`ASSET_LOAD`, §6.10, 2026-11-04).
3. Byte addresses in values, word indices only in operand escapes: pointer arithmetic and text buffers
   stay natural, and the u16 escape covers the whole arena.
4. An error in host code ends its packet: with `CALL` and data-dependent control the next instruction
   would act on garbage; `ERROR` says exactly what ran.
5. Also decided there: `VM_START` keeps the sensor settings (§5.2), "range" instead of "window", font
   format 1 kept, jump budget 1000 and the run-time cap (§5.4), `UNPACK` and `LINE`, `ROLL`, `IMU_RATE`,
   `BATT_MA` and `BATT_TEMP`, the microphone (§6.5) and the power rules (§6.6).
6. Hardware parity (user, 2026-10-26): every block stock drives is driven here too: TouchBar and wear,
   light with auto-brightness and the gyroscope (§6.7), the microphone (§6.5), the case link (§6.8) and
   the arm-to-arm link (§6.9), with the minimal VM surface above and `INPUT` events for the host (§6.3).
   The power rules (§6.6) and the microphone rules (§6.5) are unchanged.
7. After M1 (2026-10-27, `docs/review-v5.md` §10): `HELLO.board` bit 2 means capture, `PANEL_READ` answers 11
   while the panel is off, the executor sits below the periodic threads (§5.4), `BATT_PCT` keeps a 30 s
   hysteresis (§2.5), the network core runs ESB mode 0 on both arms (§6.9).
8. Panel standby (2026-10-27, measured on 0.5.14): an explicit `PANEL` = 2, no automatic standby timer:
   predictable power, one value, and the apps that need a 4 ms popup choose it; an unattended arm still
   goes fully off (§2.5, §3.5, §6.6).
9. `IN_CASE` = VBUS or a case message within 3 s (§2.5, §6.1): the pulsed charging of a full arm no longer
   flickers the in-case rules, at the price of those rules lasting up to 3 s after leaving the case.
10. After M7/M8 (2026-10-27, `docs/review-v5.md` §10): taps are classified in the firmware with stock's
    305/458 ms timers (the controller's table classifies nothing, no slides); stock's wear comes from the
    network core's link-quality value, which M10 adopts (§6.7); the light sensor exists on the right arm only
    and the left follows it over the link from M10 (§6.7, §6.9); `GYRO_RATE` and `AUTO_BRIGHTNESS` reset at
    `RESET_SESSION`; the head frame is right-handed with z toward the wearer (§2.5).
11. `IN_CASE` is a state (§2.5): set by the case's power, field or messages, cleared by motion after 3 s of
    silence, by wear, or by 400 s of silence, so a full arm lying in the case stays in the case (the 3 s hold
    of decision 9 read 0 there, the case re-checking it only every ~300 s).
12. The case hold (2026-10-28, firmware agent, 0.5.32; for the reviewer): in the case the panel is off as in
    stock, `DRAW` draws nothing and `PANEL` keeps the value written (as the microphone's hold of §6.5 does the
    other way round), `NOTICE` 6 marks the hold, and `CASE_PANEL` lets a development host use the panel there
    except during a top-up (§6.6, §6.8). Chosen over drawing into a panel kept powered but
    dark: the panel's memory needs its rails (≈ 1 mA in standby, ≈ 4.6 mA lit black) and the arm has no frame
    buffer of its own; a host's mirror and a program's `CASE` handler restore the image instead. Removed in 0.5.60
    (the user, 2026-11-05: display policy belongs in programs; the library's `PanelPolicy` keeps the panel off in
    the case and while folded, the LED blanking of §6.6 stays).
13. The panel guard (2026-11-04, `docs/review-dimming.md`, 0.5.52): the firmware caps the panel's level on the
    measured discharge current and voltage, not on `BATT_PCT` (voltage-based: it reads 0–1 % under a white panel and
    would oscillate) and not on the content (a lit-pixel estimate would act a second earlier, which matters only
    below ~15 % charge). A cap, not a change of `BRIGHTNESS`: the written level stays the application's, and the
    library matches the eyes through `BRIGHTNESS_MAX`. Not the fix for the right arm's two resets of 2026-11-03
    (their PMIC record shows no VSYSLOW): it keeps bright content at low charge away from the 2.8 V POF and the pack
    from heating under ~0.35 A. `PANEL_GUARD` is a host control so a program cannot switch it off.
