/* Faceclaw's display path (VM.md §6.11): the phone composes each eye's frame and sends what changed, as draw calls in
 * the records of Faceclaw's custom firmware for the G2 (the same record format, compression and calls), on the FC
 * characteristic. No VM instruction runs: the path takes the arena for its memory and stops any program.
 *
 *   BT thread:  fc_rx() copies packets into the receive ring and publishes whole records
 *   executor:   fc_run() inflates one record, checks its CRC, applies it and answers with EV_FC_ACK
 *
 * Nothing here trusts a record. Out of the case only a bonded host gets this far, but in the case anyone does. */
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/crc.h>
#include "fc_inflate.h"
#include "g1.h"

LOG_MODULE_REGISTER(fc, LOG_LEVEL_INF);

#define FC_VERSION 1

/* The arena: 1 KiB left to the VM's registers and table, the frame, the inflater, the receive ring. */
#define FRAME_OFF 0x400
#define FRAME_SIZE (PANEL_ROW_BYTES * PANEL_H)
#define MAX_MESSAGE 16384u
#define Z_OFF (FRAME_OFF + FRAME_SIZE)
#define Z_SIZE (FC_Z_HISTORY + MAX_MESSAGE)
#define RX_OFF (Z_OFF + Z_SIZE)
#define RX_SIZE (ARENA_SIZE - RX_OFF)
#define REC_HDR 5
/* deflate can grow a message by little: 5 bytes per stored block, and the flush's own */
#define MAX_WIRE (MAX_MESSAGE + 256u)
BUILD_ASSERT(RX_OFF < ARENA_SIZE && RX_SIZE >= 3 * (REC_HDR + MAX_WIRE));

#define MAX_RECORDS 12
#define MAX_DIRTY 24

enum { OPT_START = 0x80, OPT_END = 0x40 };
enum { REC_LENSES = 3, REC_COMPRESSED = 4, REC_RESET = 8 };
enum { MSG_DRAW = 26, MSG_PRESENT = 28, MSG_BRIGHTNESS = 30, MSG_TYPE_MASK = 127 };
enum { OP_BBOX = 1, OP_CLEAR = 9 };
enum { KIND_ACK = 1, KIND_NACK = 3 };

struct record {
	uint32_t off;
	uint16_t len;
	uint8_t stream;
};

static uint8_t *arena;
static uint8_t *frame;
static uint8_t *rx_mem;

/* ring and record queue, shared by the BT thread and the executor */
static struct k_spinlock rx_lock;
static struct record records[MAX_RECORDS];
static uint8_t rec_tail, rec_count;
static uint32_t rx_wr, rx_rd;
static atomic_t rx_lost;   /* a record was dropped: the inflater's history no longer matches the sender's */

/* the record being received (BT thread only) */
static uint32_t cur_off, cur_fill, cur_need;
static uint8_t cur_stream;
static bool cur_open, cur_reserved;

/* executor only */
static struct fc_inflate z;
static bool ctx_ok, active;
static bool visible = true;
static struct fc_rect {
	uint16_t x, y, w, h;
} dirty[MAX_DIRTY];
static uint8_t dirty_n;
static bool dirty_all;

static uint32_t stats_records, stats_nacks, stats_unknown, stats_presents;

static uint8_t lens_mask(void)
{
	return board_role() == 1 ? 1 : 2;
}

static void answer(uint8_t kind, uint8_t stream, uint32_t size, uint16_t crc, bool block)
{
	uint8_t b[9] = {kind, stream, 0, 0, lens_mask()};

	put16(&b[5], (uint16_t)size);
	put16(&b[7], crc);
	proto_event(EV_FC_ACK, b, sizeof(b), block);
}

void fc_init(void)
{
	arena = exec_arena();
	frame = arena + FRAME_OFF;
	rx_mem = arena + RX_OFF;
	fc_inflate_init(&z, arena + Z_OFF, Z_SIZE);
}

/* ---------------------------------------------------------------- receiving (BT thread) */

static bool rx_reserve(uint32_t need, uint32_t *off)
{
	k_spinlock_key_t key = k_spin_lock(&rx_lock);
	bool ok = false;

	if (rec_count == 0) {
		rx_wr = 0;
		rx_rd = 0;
	}
	if (rx_wr >= rx_rd) {
		if (RX_SIZE - rx_wr >= need) {
			*off = rx_wr;
			ok = true;
		} else if (rx_rd > need) {
			*off = 0;
			ok = true;
		}
	} else if (rx_rd - rx_wr > need) {
		*off = rx_wr;
		ok = true;
	}
	if (ok) {
		rx_wr = *off + need;
	}
	k_spin_unlock(&rx_lock, key);
	return ok;
}

static void rx_abandon(void)
{
	if (cur_reserved) {
		k_spinlock_key_t key = k_spin_lock(&rx_lock);

		rx_wr = cur_off;
		k_spin_unlock(&rx_lock, key);
		cur_reserved = false;
	}
	cur_open = false;
}

static bool rx_publish(void)
{
	k_spinlock_key_t key = k_spin_lock(&rx_lock);
	bool ok = rec_count < MAX_RECORDS;

	if (ok) {
		struct record *r = &records[(rec_tail + rec_count) % MAX_RECORDS];

		r->off = cur_off;
		r->len = (uint16_t)cur_need;
		r->stream = cur_stream;
		if (rec_count == 0) {
			rx_rd = cur_off;
		}
		rec_count++;
	}
	k_spin_unlock(&rx_lock, key);
	return ok;
}

static void rx_drop(uint8_t stream)
{
	rx_abandon();
	atomic_set(&rx_lost, 1);
	answer(KIND_NACK, stream, 0, 0, false);
}

/* One packet: [opts: 0x80 first, 0x40 last][stream][the next bytes of the record] */
void fc_rx(const uint8_t *d, size_t len)
{
	const uint8_t *p = d + 2;
	size_t n;
	uint8_t opts, stream;

	if (len < 3) {
		return;
	}
	if (!arena || safe_level() != 0) {
		answer(KIND_NACK, d[1], 0, 0, false);
		return;
	}
	opts = d[0];
	stream = d[1];
	n = len - 2;
	if (opts & OPT_START) {
		uint32_t need;

		if (cur_open) {
			/* the last one never ended: it is lost */
			rx_abandon();
			atomic_set(&rx_lost, 1);
		}
		if (n < REC_HDR) {
			rx_drop(stream);
			return;
		}
		need = REC_HDR + (uint32_t)get16(p + 1);
		if (need > REC_HDR + MAX_WIRE || !rx_reserve(need, &cur_off)) {
			LOG_WRN("record of %u bytes refused", need);
			rx_drop(stream);
			return;
		}
		cur_reserved = true;
		cur_open = true;
		cur_stream = stream;
		cur_need = need;
		cur_fill = 0;
	} else if (!cur_open || stream != cur_stream) {
		/* the rest of a record already dropped: its first packet was answered */
		if (cur_open) {
			rx_drop(stream);
		}
		return;
	}
	if (n > cur_need - cur_fill) {
		rx_drop(stream);
		return;
	}
	memcpy(rx_mem + cur_off + cur_fill, p, n);
	cur_fill += n;
	if (opts & OPT_END) {
		if (cur_fill != cur_need || !rx_publish()) {
			rx_drop(stream);
			return;
		}
		cur_open = false;
		cur_reserved = false;
		exec_wake();
	}
}

void fc_link_down(void)
{
	rx_abandon();
	atomic_set(&rx_lost, 1);
}

/* ---------------------------------------------------------------- the frame */

static void add_dirty(int x, int y, int w, int h)
{
	if (dirty_all) {
		return;
	}
	if (dirty_n == MAX_DIRTY) {
		dirty_all = true;
		return;
	}
	dirty[dirty_n++] = (struct fc_rect){.x = x, .y = y, .w = w, .h = h};
}

/* n pixels of one colour from (x, y) in one row */
static void fill_span(int x, int y, int n, uint8_t color)
{
	uint8_t *row = frame + y * PANEL_ROW_BYTES;

	if (x & 1) {
		row[x / 2] = (row[x / 2] & 0xF0) | color;
		x++;
		n--;
	}
	if (n >= 2) {
		memset(row + x / 2, color * 0x11, n / 2);
		x += n & ~1;
		n &= 1;
	}
	if (n) {
		row[x / 2] = (row[x / 2] & 0x0F) | (uint8_t)(color << 4);
	}
}

/* A bbox's pixels: runs over exactly w x h pixels, left to right and top to bottom. One token is [count4|color4], for a
 * count of 1…15; [0|color4][count8]; or [0|color4][0][count16], little-endian. */
static int bbox_pixels(int x, int y, int w, int h, const uint8_t *p, uint32_t n)
{
	uint32_t left = (uint32_t)w * (uint32_t)h, i = 0;
	int cx = x, cy = y;

	while (i < n) {
		uint8_t color = p[i] & 0x0F;
		uint32_t count = p[i++] >> 4;

		if (!count) {
			if (i >= n) {
				return -EINVAL;
			}
			count = p[i++];
			if (!count) {
				if (n - i < 2) {
					return -EINVAL;
				}
				count = get16(p + i);
				i += 2;
				if (!count) {
					return -EINVAL;
				}
			}
		}
		if (count > left) {
			return -EINVAL;
		}
		left -= count;
		while (count) {
			int span = MIN((int)count, x + w - cx);

			fill_span(cx, cy, span, color);
			count -= span;
			cx += span;
			if (cx == x + w) {
				cx = x;
				cy++;
			}
		}
	}
	return left ? -EINVAL : 0;
}

static int call_bbox(const uint8_t *a, uint32_t n)
{
	int x, y, w, h;

	if (n < 1) {
		return -EINVAL;
	}
	if (a[0] == 0) {
		if (n < 5) {
			return -EINVAL;
		}
		x = a[1] * 4;
		y = a[2] * 2;
		w = a[3] * 4;
		h = a[4] * 2;
		a += 5;
		n -= 5;
	} else if (a[0] == 1) {
		if (n < 9) {
			return -EINVAL;
		}
		x = get16(a + 1);
		y = get16(a + 3);
		w = get16(a + 5);
		h = get16(a + 7);
		a += 9;
		n -= 9;
	} else {
		return -EINVAL;
	}
	if (w <= 0 || h <= 0 || x + w > PANEL_W || y + h > PANEL_H) {
		return -EINVAL;
	}
	if (bbox_pixels(x, y, w, h, a, n)) {
		/* a part of the rectangle has changed: show it right once the phone repeats the record */
		add_dirty(x, y, w, h);
		return -EINVAL;
	}
	add_dirty(x, y, w, h);
	return 0;
}

/* [count:u16]{length:u16, call}; a call is [op][flags: only 0, this path has no targets, depth or clips][args] */
static int draw_calls(const uint8_t *p, uint32_t n)
{
	uint32_t count, i = 2;

	if (n < 2) {
		return -EINVAL;
	}
	count = get16(p);
	for (uint32_t c = 0; c < count; c++) {
		uint32_t len;
		const uint8_t *call;
		int err = 0;

		if (n - i < 2) {
			return -EINVAL;
		}
		len = get16(p + i);
		i += 2;
		if (len < 2 || len > n - i) {
			return -EINVAL;
		}
		call = p + i;
		i += len;
		if (call[1] != 0) {
			return -EINVAL;
		}
		switch (call[0]) {
		case OP_BBOX:
			err = call_bbox(call + 2, len - 2);
			break;
		case OP_CLEAR:
			if (len < 3 || call[2] > 15) {
				return -EINVAL;
			}
			memset(frame, call[2] * 0x11, FRAME_SIZE);
			dirty_all = true;
			break;
		default:
			return -EINVAL;
		}
		if (err) {
			return err;
		}
		atomic_inc(&hb_exec);
	}
	return i == n ? 0 : -EINVAL;
}

/* The frame's changes to the panel, latched at once. A rectangle's x is rounded down to a whole byte of the frame. */
static void present(void)
{
	int x0 = PANEL_W, y0 = PANEL_H, x1 = 0, y1 = 0;
	uint32_t area = 0;

	stats_presents++;
	if (!visible) {
		return;   /* kept dirty until the panel may show it */
	}
	if (!panel_is_on()) {
		dirty_all = true;   /* off or in standby, the panel has lost its image */
	}
	if (dirty_all) {
		x0 = 0;
		y0 = 0;
		x1 = PANEL_W;
		y1 = PANEL_H;
		area = PANEL_W * PANEL_H;
	} else {
		for (int i = 0; i < dirty_n; i++) {
			x0 = MIN(x0, dirty[i].x & ~1);
			y0 = MIN(y0, dirty[i].y);
			x1 = MAX(x1, dirty[i].x + dirty[i].w);
			y1 = MAX(y1, dirty[i].y + dirty[i].h);
			area += (uint32_t)dirty[i].w * dirty[i].h;
		}
		if (!dirty_n) {
			return;
		}
	}
	atomic_inc(&hb_exec);
	if (dirty_all || area >= PANEL_W * PANEL_H / 2) {
		/* whole rows go out in bursts, faster than a window per row */
		if (exec_fc_panel_begin(0, y0, PANEL_W, y1 - y0)) {
			LOG_WRN("present: no panel");
			return;
		}
		for (int y = y0; y < y1; y++) {
			panel_rect_row(frame + y * PANEL_ROW_BYTES);
		}
	} else {
		if (exec_fc_panel_begin(x0, y0, x1 - x0, y1 - y0)) {
			LOG_WRN("present: no panel");
			return;
		}
		for (int i = 0; i < dirty_n; i++) {
			int x = dirty[i].x & ~1;
			int w = dirty[i].x + dirty[i].w - x;

			for (int y = dirty[i].y; y < dirty[i].y + dirty[i].h; y++) {
				panel_rect_span(x, y, w, frame + y * PANEL_ROW_BYTES + x / 2);
			}
			atomic_inc(&hb_exec);
		}
	}
	exec_fc_panel_end();
	dirty_n = 0;
	dirty_all = false;
}

/* [30][version = 1][target 0…100][visible][fade ms: u16]: no fade yet. Hidden, the panel waits in standby; shown again,
 * it comes back with the next PRESENT, which the phone sends with it. */
static int brightness(const uint8_t *m, uint32_t n)
{
	if (n < 4 || m[1] != 1 || m[2] > 100) {
		return -EINVAL;
	}
	exec_fc_brightness(MAX(1, (m[2] * PANEL_LEVEL_MAX + 50) / 100));
	if (visible && !m[3]) {
		exec_fc_panel(2);
		dirty_all = true;
	}
	visible = m[3] != 0;
	return 0;
}

static int apply(const uint8_t *m, uint32_t n)
{
	if (!n) {
		return 0;
	}
	switch (m[0] & MSG_TYPE_MASK) {
	case MSG_DRAW:
		return draw_calls(m + 1, n - 1);
	case MSG_PRESENT:
		present();
		return 0;
	case MSG_BRIGHTNESS:
		return brightness(m, n);
	default:
		/* a later phone's message: nothing here depends on it */
		stats_unknown++;
		return 0;
	}
}

/* ---------------------------------------------------------------- records (executor) */

static void open_session(void)
{
	if (active) {
		return;
	}
	exec_fc_stop_vm();
	memset(frame, 0, FRAME_SIZE);
	dirty_n = 0;
	dirty_all = true;
	visible = true;
	ctx_ok = false;
	active = true;
	LOG_INF("display path open");
}

static bool handle(const uint8_t *r, uint32_t n, uint32_t *size, uint16_t *crc)
{
	uint8_t flags = r[0];
	uint32_t wire = get16(r + 1);
	const uint8_t *msg = r + REC_HDR;
	uint32_t len = wire;
	bool ok;

	*size = 0;
	*crc = get16(r + 3);
	if (REC_HDR + wire != n) {
		return false;
	}
	open_session();
	if (flags & REC_RESET) {
		fc_inflate_reset(&z);
		ctx_ok = true;
		atomic_set(&rx_lost, 0);
	} else if (atomic_get(&rx_lost)) {
		ctx_ok = false;
	}
	if (flags & REC_COMPRESSED) {
		int err;

		if (!ctx_ok) {
			return false;
		}
		err = fc_inflate_record(&z, r + REC_HDR, wire, &msg, &len);
		if (err) {
			LOG_WRN("inflate: %d", err);
			ctx_ok = false;
			return false;
		}
	} else if (len > MAX_MESSAGE) {
		return false;
	}
	*size = len;
	if (crc16_itu_t(0xFFFF, msg, len) != *crc) {
		LOG_WRN("record CRC: size %u", len);
		ctx_ok = false;
		return false;
	}
	ok = apply(msg, len) == 0;
	if (!ok) {
		LOG_WRN("message %u of %u bytes refused", len ? msg[0] : 0, len);
	}
	if (flags & REC_COMPRESSED) {
		fc_inflate_commit(&z);
	}
	return ok;
}

bool fc_pending(void)
{
	return rec_count != 0;
}

/* One record per call, so the executor's loop keeps its cadence. */
void fc_run(void)
{
	struct record r;
	k_spinlock_key_t key = k_spin_lock(&rx_lock);
	uint32_t size = 0;
	uint16_t crc = 0;
	bool ok;

	if (!rec_count) {
		k_spin_unlock(&rx_lock, key);
		return;
	}
	r = records[rec_tail];
	k_spin_unlock(&rx_lock, key);

	ok = safe_level() == 0 && handle(rx_mem + r.off, r.len, &size, &crc);
	stats_records++;
	stats_nacks += !ok;

	key = k_spin_lock(&rx_lock);
	rec_tail = (rec_tail + 1) % MAX_RECORDS;
	rec_count--;
	rx_rd = rec_count ? records[rec_tail].off : rx_wr;
	k_spin_unlock(&rx_lock, key);

	atomic_inc(&hb_exec);
	answer(ok ? KIND_ACK : KIND_NACK, r.stream, ok ? size : 0, crc, true);
}

/* RESET_SESSION: the arena is the VM's again, whatever the path held there is gone. Executor. */
void fc_reset(void)
{
	k_spinlock_key_t key = k_spin_lock(&rx_lock);

	rec_tail = 0;
	rec_count = 0;
	rx_wr = 0;
	rx_rd = 0;
	k_spin_unlock(&rx_lock, key);
	atomic_set(&rx_lost, 1);
	active = false;
	ctx_ok = false;
}

/* FC_INFO's body (VM.md §6.11): u8 version, u8 flags (bit 0 the path is open), u16 max_message, u16 max_wire, u32 rx_size,
 * u32 history */
size_t fc_info(uint8_t *b)
{
	b[0] = FC_VERSION;
	b[1] = active ? 1 : 0;
	put16(&b[2], MAX_MESSAGE);
	put16(&b[4], MAX_WIRE);
	put32(&b[6], RX_SIZE);
	put32(&b[10], FC_Z_HISTORY);
	put32(&b[14], stats_records);
	put32(&b[18], stats_nacks);
	put32(&b[22], stats_presents);
	return 26;
}
