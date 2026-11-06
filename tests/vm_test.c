/* Tests for the protocol 5 VM (src/vm): every instruction, the VM.md §9 examples byte for byte,
 * triggers, jump budget, run-time cap and errors through a fake platform, and drawing pixel for pixel
 * against g1v0's protocol 4 code (vm_v4_ref.c) and a per-pixel reading of VM.md §3.
 * Build and run: src/vm/README.md. "vm_test fuzz N [seed]" fuzzes the VM for N seconds after the tests
 * (default 2 s); VM_FUZZ_HIST=1 prints the most frequent HALT causes of the fuzz programs. */
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "vm.h"

uint8_t *v4_arena(void);
int v4_write_rle(const uint8_t *body);
int v4_fill(const uint8_t *body);
int v4_blit(const uint8_t *body, int compressed);
int v4_glyph_run(const uint8_t *body);

#define ARENA ((int64_t)VM_ARENA_SIZE)
#define WIN 65536u
#define WIN0 (VM_ARENA_SIZE - WIN)
#define POOL 256

enum {
	WRITE = 0x01, WRITE_RLE = 0x02, COPY = 0x03, UNPACK = 0x04, MOV = 0x10, ADD = 0x11, SUB = 0x12,
	MUL = 0x13, DIVU = 0x14, MODU = 0x15, DIVS = 0x16, MODS = 0x17, AND = 0x18, OR = 0x19, XOR = 0x1A,
	SHL = 0x1B, SHR = 0x1C, SAR = 0x1D, LD32 = 0x20, LD16 = 0x21, LD8 = 0x22, LD4 = 0x23, ST32 = 0x24,
	ST16 = 0x25, ST8 = 0x26, ST4 = 0x27, JMP = 0x30, CALL = 0x31, RET = 0x32, END = 0x33, HALT = 0x34,
	BEQ = 0x35, BNE = 0x36, BLTU = 0x37, BGEU = 0x38, BLTS = 0x39, BGES = 0x3A, SLEEP_UNTIL = 0x40,
	WAIT_EVENT = 0x41, RAISE = 0x42, NOTIFY = 0x43, SYS_READ = 0x44, SYS_WRITE = 0x45, FILL = 0x50,
	BLIT = 0x51, GLYPHS = 0x52, DRAW = 0x53, LINE = 0x54, VIDEO = 0x55, ASSET_LOAD = 0x56
};

static void spec_shape(uint8_t op, int *k, int *lit);
static uint32_t crc32z(uint32_t crc, const uint8_t *p, uint32_t n);

static int checks, failures;

static void check_eq(uint64_t a, uint64_t b, const char *x, const char *y, int line)
{
	checks++;
	if (a != b) {
		failures++;
		if (failures <= 40) {
			printf("FAIL line %d: %s = 0x%llx, expected %s = 0x%llx\n", line, x, (unsigned long long)a, y,
			       (unsigned long long)b);
		}
	}
}

#define CHECK(c) check_eq((c) != 0, 1, #c, "true", __LINE__)
#define CHECK_EQ(a, b) check_eq((uint64_t)(a), (uint64_t)(b), #a, #b, __LINE__)

static uint64_t seed = 0x9E3779B97F4A7C15ull;

static uint32_t rnd(void)
{
	seed ^= seed << 13;
	seed ^= seed >> 7;
	seed ^= seed << 17;
	return (uint32_t)(seed >> 16);
}

static uint32_t rndn(uint32_t n)
{
	return n ? rnd() % n : 0;
}

static uint32_t get16(const uint8_t *p)
{
	return (uint32_t)p[0] | (uint32_t)p[1] << 8;
}

static uint32_t get32(const uint8_t *p)
{
	return get16(p) | get16(p + 2) << 16;
}

static void put16(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
}

static void put32(uint8_t *p, uint32_t v)
{
	put16(p, v);
	put16(p + 2, v >> 16);
}

/* ---------------------------------------------------------------- fake platform */

struct event {
	uint8_t type;
	uint8_t body[16];
	uint32_t len, t;
};

#define MAXEV 4096

static struct {
	uint32_t t, tick, row_cost;
	uint8_t pk[128][256];
	uint32_t pk_len[128], head, tail;
	uint16_t seq;
	uint32_t fired, yield_at;
	uint32_t props[256];
	struct event ev[MAXEV];
	uint32_t nev;
	int drop;
	uint8_t panel[VM_PANEL_H][VM_PANEL_W];
	int panel_on, standby, has_panel, begins, ends, rows, next_row, spans;
	int32_t bx, by, bw, bh;
	uint32_t brightness, imu_rate, imu_calls;
	uint32_t pw_id, pw_value, pw_calls;
	int pw_err;
	struct {
		uint8_t id[16];
		const uint8_t *body;
		uint32_t len;
	} asset[2];
	int nassets, asset_calls;
} f;

static uint8_t arena[VM_ARENA_SIZE], ref[VM_ARENA_SIZE];
static uint8_t ref_panel[VM_PANEL_H][VM_PANEL_W];
static struct vm vm;
static int npool;

static uint32_t h_now(void *c)
{
	(void)c;
	f.t += f.tick;
	return f.t;
}

static const uint8_t *h_packet(void *c, uint32_t *len)
{
	(void)c;
	if (f.head == f.tail) {
		return NULL;
	}
	*len = f.pk_len[f.head % 128];
	return f.pk[f.head % 128];
}

static void h_packet_done(void *c)
{
	(void)c;
	f.head++;
}

static uint32_t h_poll(void *c)
{
	uint32_t r = f.fired;

	(void)c;
	f.fired = 0;
	if (f.yield_at && vm.stats.instructions >= f.yield_at) {
		r |= VM_YIELD;
	}
	return r;
}

static uint32_t h_prop(void *c, uint32_t id)
{
	(void)c;
	if (id == VM_P_PANEL) {
		return f.panel_on ? 1 : f.standby ? 2 : 0;
	}
	if (id == VM_P_BRIGHTNESS) {
		return f.brightness;
	}
	return f.props[id & 255];
}

static int h_event(void *c, uint8_t type, const uint8_t *body, uint32_t len)
{
	(void)c;
	if (f.nev < MAXEV && len <= 16) {
		f.ev[f.nev].type = type;
		memcpy(f.ev[f.nev].body, body, len);
		f.ev[f.nev].len = len;
		f.ev[f.nev].t = f.t;
		f.nev++;
	}
	return f.drop;
}

static int h_panel_begin(void *c, int32_t x, int32_t y, int32_t w, int32_t h)
{
	(void)c;
	CHECK(w > 0 && h > 0 && x >= 0 && y >= 0 && x + w <= VM_PANEL_W && y + h <= VM_PANEL_H);
	if (!f.has_panel) {
		return 1;
	}
	if (!f.panel_on) {
		f.panel_on = 1;
		f.standby = 0;
		memset(f.panel, 0, sizeof(f.panel));
	}
	f.begins++;
	f.bx = x;
	f.by = y;
	f.bw = w;
	f.bh = h;
	f.next_row = y;
	return 0;
}

static void h_panel_row(void *c, int32_t x, int32_t y, int32_t w, const uint8_t *px)
{
	(void)c;
	if (f.spans) {
		CHECK(x >= f.bx && x + w <= f.bx + f.bw && y >= f.next_row && y < f.by + f.bh && w > 0);
	} else {
		CHECK(x == f.bx && w == f.bw && y == f.next_row && y < f.by + f.bh);
	}
	f.next_row = y + 1;
	for (int32_t i = 0; i < w; i++) {
		f.panel[y][x + i] = (i & 1) ? (px[i / 2] & 15) : (px[i / 2] >> 4);
	}
	if (w & 1) {
		CHECK((px[w / 2] & 15) == 0);
	}
	f.rows++;
	f.t += f.row_cost;
}

static void h_panel_end(void *c)
{
	(void)c;
	CHECK(f.spans || f.next_row == f.by + f.bh);
	f.ends++;
}

static void h_panel_power(void *c, uint32_t on)
{
	(void)c;
	if (!on || !f.panel_on) {
		memset(f.panel, 0, sizeof(f.panel));
	}
	f.panel_on = on == 1;
	f.standby = on == 2;
}

static void h_brightness(void *c, uint32_t level)
{
	(void)c;
	f.brightness = level;
}

static void h_imu_rate(void *c, uint32_t hz)
{
	(void)c;
	f.imu_rate = hz;
	f.imu_calls++;
}

static int h_prop_write(void *c, uint32_t id, uint32_t value)
{
	(void)c;
	f.pw_calls++;
	if (!f.pw_err) {
		f.pw_id = id;
		f.pw_value = value;
	}
	return f.pw_err;
}

/* the platform's asset store: f.asset[0..nassets-1] */
static int h_asset_read(void *c, const uint8_t *id, uint32_t off, uint8_t *dst, uint32_t n)
{
	(void)c;
	f.asset_calls++;
	for (int i = 0; i < f.nassets; i++) {
		if (memcmp(id, f.asset[i].id, 16) == 0) {
			if ((uint64_t)off + n > f.asset[i].len) {
				return VM_E_ARENA;
			}
			memcpy(dst, f.asset[i].body + off, n);
			return 0;
		}
	}
	return VM_E_ASSET;
}

static const struct vm_host host = {
	NULL, h_now, h_packet, h_packet_done, h_poll, h_prop, h_event, h_panel_begin, h_panel_row,
	h_panel_end, h_panel_power, h_brightness, h_imu_rate, h_prop_write, NULL, h_asset_read
};

static void setup(int safe)
{
	memset(&f, 0, sizeof(f));
	f.pw_err = VM_E_ARG;
	f.t = 1000000;
	f.has_panel = 1;
	f.brightness = 21;
	f.imu_rate = 13;
	for (int i = 0; i < 256; i++) {
		f.props[i] = VM_UNKNOWN;
	}
	f.props[VM_P_ARM] = 0;
	f.props[VM_P_LINK] = 1;
	f.props[VM_P_LINK_INTERVAL] = 7500;
	f.props[VM_P_BATT_MV] = 3900;
	f.props[VM_P_BATT_PCT] = 50;
	f.props[VM_P_CHARGE] = 0;
	f.props[VM_P_IN_CASE] = 0;
	f.props[VM_P_PITCH] = 0;
	f.props[VM_P_LIGHT] = 500;
	vm_init(&vm, arena, &host, safe);
	npool = 0;
}

static uint32_t pool_getw(uint32_t i)
{
	return get32(arena + 4 * i);
}

static void setw(uint32_t i, uint32_t v)
{
	put32(arena + 4 * i, v);
}

/* the pool word that holds v */
static int K(uint32_t v)
{
	for (int i = 0; i < npool; i++) {
		if (pool_getw(POOL + (uint32_t)i) == v) {
			return POOL + i;
		}
	}
	setw(POOL + (uint32_t)npool, v);
	return POOL + npool++;
}

static void send(const uint8_t *code, uint32_t n)
{
	uint8_t *p = f.pk[f.tail % 128];

	put16(p, f.seq++);
	memcpy(p + 2, code, n);
	f.pk_len[f.tail % 128] = n + 2;
	f.tail++;
}

/* drive the executor as a platform would, advancing virtual time by at most us */
static void run_for(uint32_t us)
{
	uint32_t end = f.t + us;

	for (int guard = 0; guard < 20000000; guard++) {
		uint32_t w = vm_run(&vm);
		int32_t left = (int32_t)(end - f.t);

		if (w == 0 || left <= 0) {
			return;
		}
		f.t += w < (uint32_t)left ? w : (uint32_t)left;
	}
	CHECK(0);
}

static void run(void)
{
	run_for(0);
}

static struct event *last(uint8_t type)
{
	for (int i = (int)f.nev - 1; i >= 0; i--) {
		if (f.ev[i].type == type) {
			return &f.ev[i];
		}
	}
	return NULL;
}

static int count(uint8_t type)
{
	int n = 0;

	for (uint32_t i = 0; i < f.nev; i++) {
		n += f.ev[i].type == type;
	}
	return n;
}

static void expect_error(int line, int code, int op, int off, uint32_t detail, uint32_t addr)
{
	struct event *e = last(VM_EV_ERROR);

	if (!e) {
		check_eq(0, 1, "ERROR event", "present", line);
		return;
	}
	check_eq(e->len, 13, "ERROR length", "13", line);
	check_eq(get16(e->body), (uint16_t)(f.seq - 1), "ERROR seq", "last seq", line);
	check_eq(e->body[2], (uint64_t)off, "ERROR offset", "offset", line);
	check_eq(e->body[3], (uint64_t)op, "ERROR op", "op", line);
	check_eq(e->body[4], (uint64_t)code, "ERROR code", "code", line);
	check_eq(get32(e->body + 5), detail, "ERROR detail", "detail", line);
	check_eq(get32(e->body + 9), addr, "ERROR address", "address", line);
}

static void expect_halt(int line, int trigger, uint32_t addr, int op, int code, uint32_t detail)
{
	struct event *e = last(VM_EV_HALT);

	if (!e) {
		check_eq(0, 1, "HALT event", "present", line);
		return;
	}
	check_eq(e->len, 11, "HALT length", "11", line);
	check_eq(e->body[0], (uint64_t)trigger, "HALT trigger", "trigger", line);
	check_eq(get32(e->body + 1), addr, "HALT address", "address", line);
	check_eq(e->body[5], (uint64_t)op, "HALT op", "op", line);
	check_eq(e->body[6], (uint64_t)code, "HALT code", "code", line);
	check_eq(get32(e->body + 7), detail, "HALT detail", "detail", line);
}

#define EXPECT_ERROR(...) expect_error(__LINE__, __VA_ARGS__)
#define EXPECT_HALT(...) expect_halt(__LINE__, __VA_ARGS__)

/* ---------------------------------------------------------------- assembler */

struct code {
	uint8_t b[8192];
	uint32_t n;
};

static void emit(struct code *c, uint8_t op, const int *w, int k, int lit_bytes, uint32_t lit)
{
	c->b[c->n++] = op;
	for (int i = 0; i < k; i += 2) {
		int hi = w[i] < 15 ? w[i] : 15;
		int lo = i + 1 < k ? (w[i + 1] < 15 ? w[i + 1] : 15) : 0;

		c->b[c->n++] = (uint8_t)(hi << 4 | lo);
	}
	for (int i = 0; i < lit_bytes; i++) {
		c->b[c->n++] = (uint8_t)(lit >> (8 * i));
	}
	for (int i = 0; i < k; i++) {
		if (w[i] >= 15) {
			put16(c->b + c->n, (uint32_t)w[i]);
			c->n += 2;
		}
	}
}

#define NARGS(...) (int)(sizeof((int[]){__VA_ARGS__}) / sizeof(int))
#define I(c, op, ...) emit(c, op, (const int[]){__VA_ARGS__}, NARGS(__VA_ARGS__), 0, 0)
#define B(c, op, target, x, y) emit(c, op, (const int[]){x, y}, 2, 3, target)

static void J(struct code *c, uint8_t op, uint32_t target)
{
	emit(c, op, NULL, 0, 3, target);
}

static void O(struct code *c, uint8_t op)
{
	emit(c, op, NULL, 0, 0, 0);
}

static void W(struct code *c, uint32_t addr, const uint8_t *d, uint32_t n)
{
	c->b[c->n++] = WRITE;
	c->b[c->n++] = (uint8_t)addr;
	c->b[c->n++] = (uint8_t)(addr >> 8);
	c->b[c->n++] = (uint8_t)(addr >> 16);
	c->b[c->n++] = (uint8_t)n;
	memcpy(c->b + c->n, d, n);
	c->n += n;
}

static void send_code(const struct code *c)
{
	send(c->b, c->n);
}

static void place(uint32_t addr, const struct code *c)
{
	memcpy(arena + addr, c->b, c->n);
}

static void table(uint32_t id, uint32_t addr)
{
	put32(arena + 0x40 + 4 * id, addr);
}

/* hex listing "addr  bytes..." lines as printed in VM.md; checks each address */
static uint32_t listing(const char *s, uint8_t *out, uint32_t base)
{
	uint32_t n = 0;
	int first = 1;

	while (*s) {
		char *end;
		unsigned long v;

		if (*s == '\n') {
			first = 1;
			s++;
			continue;
		}
		if (*s == ' ') {
			s++;
			continue;
		}
		v = strtoul(s, &end, 16);
		if (first) {
			CHECK_EQ(v, base + n);
			first = 0;
		} else {
			out[n++] = (uint8_t)v;
		}
		s = end;
	}
	return n;
}

static uint32_t hex(const char *s, uint8_t *out)
{
	uint32_t n = 0;

	while (*s) {
		char *end;
		unsigned long v = strtoul(s, &end, 16);

		if (end == s) {
			s++;
			continue;
		}
		out[n++] = (uint8_t)v;
		s = end;
	}
	return n;
}

static uint32_t packbits(const uint8_t *in, uint32_t n, uint8_t *out)
{
	uint32_t i = 0, o = 0;

	while (i < n) {
		uint32_t run = 1;

		while (i + run < n && run < 128 && in[i + run] == in[i]) {
			run++;
		}
		if (run >= 2) {
			out[o++] = (uint8_t)(1 - (int)run);
			out[o++] = in[i];
			i += run;
		} else {
			uint32_t lit = 1;

			while (i + lit < n && lit < 128 && !(i + lit + 1 < n && in[i + lit] == in[i + lit + 1])) {
				lit++;
			}
			out[o++] = (uint8_t)(lit - 1);
			memcpy(out + o, in + i, lit);
			o += lit;
			i += lit;
		}
	}
	return o;
}

/* a random font blob (format 1) with n glyphs, glyph `space` empty, up to k kerning pairs */
static uint32_t make_font(uint8_t *b, uint32_t n, uint32_t k, int space)
{
	uint32_t keys[64], nk = 0, base = 16 + 16 * n, bit, size;

	memset(b, 0, 16384);
	for (uint32_t i = 0; i < k && i < 64; i++) {
		uint32_t key = rndn(n) << 16 | rndn(n), j = nk;

		while (j > 0 && keys[j - 1] > key) {
			j--;
		}
		if (j > 0 && keys[j - 1] == key) {
			continue;
		}
		memmove(keys + j + 1, keys + j, 4 * (nk - j));
		keys[j] = key;
		nk++;
	}
	b[0] = 'S';
	b[1] = 'K';
	b[2] = 'F';
	b[3] = 1;
	put16(b + 8, n);
	put16(b + 10, nk);
	b[12] = 14;
	b[13] = 4;
	b[14] = 20;
	bit = 8 * (base + 5 * nk);
	for (uint32_t g = 0; g < n; g++) {
		uint8_t *e = b + 16 + 12 * g;
		uint32_t w = rndn(11), h = rndn(15);

		if ((int)g == space || rndn(10) == 0) {
			w = h = 0;
		}
		put32(e, w && h ? bit : 0);
		e[4] = (uint8_t)w;
		e[5] = (uint8_t)h;
		e[6] = (uint8_t)(1 + rndn(11));
		put16(e + 8, rndn(5) - 2);
		put16(e + 10, (uint32_t)(-(int32_t)h + (int32_t)rndn(4)));
		put32(b + 16 + 12 * n + 4 * g, 0x21 + g);
		for (uint32_t j = 0; j < w * h; j++, bit++) {
			if (rnd() & 1) {
				b[bit / 8] |= (uint8_t)(0x80 >> (bit % 8));
			}
		}
	}
	for (uint32_t i = 0; i < nk; i++) {
		put32(b + base + 4 * i, keys[i]);
		b[base + 4 * nk + i] = (uint8_t)(rndn(9) - 4);
	}
	size = (bit + 7) / 8;
	put32(b + 4, size);
	return size;
}

/* ---------------------------------------------------------------- per-pixel reading of VM.md §3 */

static int ex(uint32_t base, uint32_t stride, int64_t x, int64_t y, int bpp)
{
	if (x < 0 || y < 0 || x >= (int64_t)stride * (bpp == 1 ? 8 : 2) || y >= ARENA) {
		return 0;
	}
	return base + y * (int64_t)stride + x * bpp / 8 < ARENA;
}

static uint32_t rd(const uint8_t *a, uint32_t base, uint32_t stride, int64_t x, int64_t y, int bpp)
{
	uint8_t b = a[base + y * (int64_t)stride + x * bpp / 8];

	if (bpp == 1) {
		return (b >> (7 - x % 8)) & 1;
	}
	return x % 2 ? b & 15u : b >> 4u;
}

static void wr4(uint8_t *a, uint32_t base, uint32_t stride, int64_t x, int64_t y, uint32_t v)
{
	uint8_t *p = a + base + y * (int64_t)stride + x / 2;

	*p = x % 2 ? (uint8_t)((*p & 0xF0) | v) : (uint8_t)((*p & 0x0F) | v << 4);
}

static uint32_t mix(uint32_t d, uint32_t a, uint32_t mode, uint32_t level)
{
	uint32_t s = (a * level + 7) / 15;

	if (mode == 0) {
		return s;
	}
	if (mode == 1) {
		return d > s ? d : s;
	}
	if (mode == 2) {
		return (d * (15 - a) + level * a + 7) / 15;
	}
	return (d * (15 - a) + 7) / 15;
}

static int64_t lo64(int64_t a, int64_t b)
{
	return a < b ? a : b;
}

static int64_t hi64(int64_t a, int64_t b)
{
	return a > b ? a : b;
}

static int ref_fill(uint8_t *a, const uint32_t *v)
{
	uint32_t dst = v[0], stride = v[1], w = v[4], h = v[5], level = v[6];
	int64_t x0 = (int32_t)v[2], y0 = (int32_t)v[3];

	if (level > 15) {
		return 5;
	}
	if (dst >= VM_ARENA_SIZE) {
		return 0;
	}
	for (int64_t y = hi64(y0, 0); y < lo64(y0 + h, ARENA); y++) {
		if (stride && dst + y * (int64_t)stride >= ARENA) {
			break;
		}
		for (int64_t x = hi64(x0, 0); x < lo64(lo64(x0 + w, 2 * (int64_t)stride), 2 * (ARENA - dst)); x++) {
			if (ex(dst, stride, x, y, 4)) {
				wr4(a, dst, stride, x, y, level);
			}
		}
	}
	return 0;
}

static int ref_blit(uint8_t *a, const uint32_t *v)
{
	uint32_t src = v[0], ss = v[1], dst = v[4], ds = v[5], w = v[8], h = v[9], mode = v[10], level = v[11];
	int64_t sx = (int32_t)v[2], sy = (int32_t)v[3], dx = (int32_t)v[6], dy = (int32_t)v[7];
	int bpp = mode & 4 ? 1 : 4, back;
	int64_t x0, x1, y0, y1;

	if (mode > 7 || level > 15) {
		return 5;
	}
	if (w == 0 || h == 0) {
		return 0;
	}
	if (!ex(src, ss, sx, sy, bpp) || !ex(src, ss, sx + w - 1, sy + h - 1, bpp)) {
		return 3;
	}
	if (dst >= VM_ARENA_SIZE) {
		return 0;
	}
	x0 = hi64(dx, 0);
	y0 = hi64(dy, 0);
	x1 = lo64(lo64(dx + w, 2 * (int64_t)ds), 2 * (ARENA - dst));
	y1 = lo64(dy + h, ARENA);
	if (x0 >= x1 || y0 >= y1 || !ex(dst, ds, x0, y0, 4)) {
		return 0;
	}
	back = 8 * (src + (y0 + sy - dy) * (int64_t)ss) + (x0 + sx - dx) * bpp < 8 * (dst + y0 * (int64_t)ds) + 4 * x0;
	for (int64_t i = 0; i < y1 - y0; i++) {
		int64_t y = back ? y1 - 1 - i : y0 + i;

		if (!ex(dst, ds, x0, y, 4)) {
			continue;
		}
		for (int64_t j = 0; j < x1 - x0; j++) {
			int64_t x = back ? x1 - 1 - j : x0 + j;
			uint32_t s;

			if (!ex(dst, ds, x, y, 4)) {
				continue;
			}
			s = rd(a, src, ss, x + sx - dx, y + sy - dy, bpp) * (bpp == 1 ? 15 : 1);
			wr4(a, dst, ds, x, y, mix(rd(a, dst, ds, x, y, 4), s, mode & 3, level));
		}
	}
	return 0;
}

static int ref_glyphs(uint8_t *a, const uint32_t *v)
{
	uint32_t font = v[0], dst = v[1], stride = v[2], mode = v[5], level = v[6], text = v[7], n = v[8];
	uint32_t size, gn, nk, i = 0, cnt = 0, idx[512];
	int64_t pen = (int32_t)v[3], y = (int32_t)v[4];

	if (mode > 3 || level > 15) {
		return 5;
	}
	if ((uint64_t)text + n > VM_ARENA_SIZE) {
		return 3;
	}
	if ((uint64_t)font + 16 > VM_ARENA_SIZE || memcmp(a + font, "SKF\1", 4) != 0) {
		return 8;
	}
	size = get32(a + font + 4);
	gn = get16(a + font + 8);
	nk = get16(a + font + 10);
	if (gn == 0 || 16 + 16 * (uint64_t)gn + 5 * (uint64_t)nk > size || (uint64_t)font + size > VM_ARENA_SIZE) {
		return 8;
	}
	while (i < n) {
		uint32_t val = 0;
		const uint8_t *e;

		for (int len = 0;; len++) {
			uint8_t b;

			if (len == 3 || i >= n) {
				return 1;
			}
			b = a[text + i++];
			val |= (uint32_t)(b & 127) << (7 * len);
			if (!(b & 128)) {
				break;
			}
		}
		if (val >= gn) {
			return 8;
		}
		e = a + font + 16 + 12 * val;
		if (get32(e) + (uint64_t)e[4] * e[5] > 8 * (uint64_t)size) {
			return 8;
		}
		if (cnt < 512) {
			idx[cnt++] = val;
		}
	}
	for (uint32_t j = 0; j < cnt; j++) {
		const uint8_t *e = a + font + 16 + 12 * idx[j];
		int64_t gx, gy;

		if (j > 0) {
			uint32_t key = idx[j - 1] << 16 | idx[j];

			for (uint32_t q = 0; q < nk; q++) {
				if (get32(a + font + 16 + 16 * gn + 4 * q) == key) {
					pen += (int8_t)a[font + 16 + 16 * gn + 4 * nk + q];
				}
			}
		}
		gx = pen + (int16_t)get16(e + 8);
		gy = y + (int16_t)get16(e + 10);
		for (int64_t r = 0; r < e[5]; r++) {
			for (int64_t c = 0; c < e[4]; c++) {
				uint64_t bit = 8 * (uint64_t)font + get32(e) + (uint64_t)(r * e[4] + c);
				uint32_t ink = (a[bit / 8] >> (7 - bit % 8)) & 1;

				if (ex(dst, stride, gx + c, gy + r, 4)) {
					wr4(a, dst, stride, gx + c, gy + r, mix(rd(a, dst, stride, gx + c, gy + r, 4), 15 * ink, mode, level));
				}
			}
		}
		pen += e[6];
	}
	return 0;
}

static int ref_draw(const uint8_t *a, const uint32_t *v, uint8_t panel[VM_PANEL_H][VM_PANEL_W])
{
	uint32_t src = v[0], ss = v[1], w = v[6], h = v[7];
	int64_t sx = (int32_t)v[2], sy = (int32_t)v[3], dx = (int32_t)v[4], dy = (int32_t)v[5];

	if (w == 0 || h == 0) {
		return 0;
	}
	if (!ex(src, ss, sx, sy, 4) || !ex(src, ss, sx + w - 1, sy + h - 1, 4)) {
		return 3;
	}
	for (int64_t y = hi64(0, -dy); y < lo64(h, VM_PANEL_H - dy); y++) {
		for (int64_t x = hi64(0, -dx); x < lo64(w, VM_PANEL_W - dx); x++) {
			panel[dy + y][dx + x] = (uint8_t)rd(a, src, ss, sx + x, sy + y, 4);
		}
	}
	return 0;
}

/* pixel j of a line, ordered so that its longer axis increases: the exact line rounded to the
 * nearest pixel, a tie toward the first end */
static void line_pixel(int64_t x0, int64_t y0, int64_t x1, int64_t y1, int64_t j, int64_t *x, int64_t *y)
{
	int64_t dx = x1 > x0 ? x1 - x0 : x0 - x1, dy = y1 > y0 ? y1 - y0 : y0 - y1, t;

	if (dx >= dy) {
		if (x0 > x1) {
			t = x0, x0 = x1, x1 = t;
			t = y0, y0 = y1, y1 = t;
		}
		*x = x0 + j;
		*y = y0 + (dx ? (y1 > y0 ? 1 : -1) * ((2 * j * dy + dx - 1) / (2 * dx)) : 0);
	} else {
		if (y0 > y1) {
			t = x0, x0 = x1, x1 = t;
			t = y0, y0 = y1, y1 = t;
		}
		*y = y0 + j;
		*x = x0 + (x1 > x0 ? 1 : -1) * ((2 * j * dx + dy - 1) / (2 * dy));
	}
}

/* ---------------------------------------------------------------- tests */

static void test_encoding(void)
{
	static const uint8_t add[] = {0x11, 0x31, 0xf0, 0x20, 0x01};
	static const uint8_t add_esc[] = {0x11, 0xff, 0xf0, 0x03, 0x00, 0x01, 0x00, 0xff, 0xff};

	setup(0);
	setw(1, 5);
	setw(0x120, 7);
	send(add, sizeof(add));
	run();
	CHECK_EQ(pool_getw(3), 12);
	setw(0xFFFF, 0x10);
	send(add_esc, sizeof(add_esc));
	run();
	CHECK_EQ(pool_getw(3), 0x15);
	CHECK_EQ(f.nev, 0);
	CHECK_EQ(f.head, 2);
	CHECK_EQ(vm.stats.packets, 2);
}

static uint32_t ref_alu(int op, uint32_t a, uint32_t b)
{
	int64_t sa = (int32_t)a, sb = (int32_t)b;

	switch (op) {
	case ADD:
		return a + b;
	case SUB:
		return a - b;
	case MUL:
		return (uint32_t)((uint64_t)a * b);
	case DIVU:
		return b ? a / b : 0;
	case MODU:
		return b ? a % b : a;
	case DIVS:
		return sb ? (uint32_t)(sa / sb) : 0;
	case MODS:
		return sb ? (uint32_t)(sa % sb) : a;
	case AND:
		return a & b;
	case OR:
		return a | b;
	case XOR:
		return a ^ b;
	case SHL:
		return a << (b % 32);
	case SHR:
		return a >> (b % 32);
	default:
		return (uint32_t)(sa >> (b % 32));
	}
}

static void test_alu(void)
{
	static const uint32_t vals[] = {0, 1, 2, 3, 7, 31, 32, 33, 1000, 0x7FFFFFFF, 0x80000000, 0x80000001,
					0xFFFFFFFF, 0xFFFFFFF9, 12345678};
	const int n = (int)(sizeof(vals) / sizeof(vals[0]));
	struct code c;

	setup(0);
	for (int op = ADD; op <= SAR; op++) {
		for (int i = 0; i < n; i++) {
			for (int j = 0; j < n; j++) {
				c.n = 0;
				I(&c, (uint8_t)op, 300, K(vals[i]), K(vals[j]));
				send_code(&c);
				run();
				CHECK_EQ(pool_getw(300), ref_alu(op, vals[i], vals[j]));
			}
		}
	}
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			c.n = 0;
			I(&c, DIVU, 301, K(vals[i]), K(vals[j]));
			I(&c, MODU, 302, K(vals[i]), K(vals[j]));
			I(&c, DIVS, 303, K(vals[i]), K(vals[j]));
			I(&c, MODS, 304, K(vals[i]), K(vals[j]));
			send_code(&c);
			run();
			CHECK_EQ(pool_getw(301) * vals[j] + pool_getw(302), vals[i]);
			CHECK_EQ(pool_getw(303) * vals[j] + pool_getw(304), vals[i]);
		}
	}
	/* the destination is written after the sources are read */
	setw(1, 21);
	setw(2, 5);
	setw(3, 7);
	c.n = 0;
	I(&c, ADD, 1, 1, 1);
	I(&c, SUB, 2, 3, 2);
	I(&c, MOV, 4, 3);
	send_code(&c);
	run();
	CHECK_EQ(pool_getw(1), 42);
	CHECK_EQ(pool_getw(2), 2);
	CHECK_EQ(pool_getw(4), 7);
	CHECK_EQ(f.nev, 0);
}

static void test_branches(void)
{
	static const uint32_t vals[] = {0, 1, 2, 0x7FFFFFFF, 0x80000000, 0xFFFFFFFF};
	static const uint8_t ops[] = {BEQ, BNE, BLTU, BGEU, BLTS, BGES};
	struct code c, p;

	setup(0);
	for (unsigned o = 0; o < 6; o++) {
		for (unsigned i = 0; i < 6; i++) {
			for (unsigned j = 0; j < 6; j++) {
				uint32_t a = vals[i], b = vals[j], taken;
				int want;

				c.n = 0;
				B(&c, ops[o], 0, K(a), K(b));
				I(&c, MOV, 400, K(0));
				O(&c, RET);
				taken = 0x8000 + c.n;
				I(&c, MOV, 400, K(1));
				O(&c, RET);
				c.b[2] = (uint8_t)taken;
				c.b[3] = (uint8_t)(taken >> 8);
				c.b[4] = (uint8_t)(taken >> 16);
				place(0x8000, &c);
				p.n = 0;
				J(&p, CALL, 0x8000);
				send_code(&p);
				run();
				switch (ops[o]) {
				case BEQ:
					want = a == b;
					break;
				case BNE:
					want = a != b;
					break;
				case BLTU:
					want = a < b;
					break;
				case BGEU:
					want = a >= b;
					break;
				case BLTS:
					want = (int32_t)a < (int32_t)b;
					break;
				default:
					want = (int32_t)a >= (int32_t)b;
					break;
				}
				CHECK_EQ(pool_getw(400), want);
			}
		}
	}
	CHECK_EQ(f.nev, 0);
}

static void test_load_store(void)
{
	static const uint8_t bytes[] = {0x11, 0x22, 0x33, 0x44};
	uint32_t v[7] = {0x4000, 10, 3, 2, 1, 1, 9}, detail = 0;
	struct code c = {{0}, 0};

	setup(0);
	memcpy(arena + 0x2001, bytes, 4);
	I(&c, LD32, 1, K(0x2001), K(0));
	I(&c, LD16, 2, K(0x2001), K(1));
	I(&c, LD8, 3, K(0x2001), K(3));
	I(&c, ST16, K(0x3000), K(2), K(0xABCD1234));
	I(&c, ST8, K(0x3010), K(0x10), K(0x1FF));
	I(&c, ST32, K(0x3021), K(1), K(0xDEADBEEF));
	I(&c, ST4, K(0x3040), K(0), K(0xA));
	I(&c, ST4, K(0x3040), K(1), K(0xFB));
	I(&c, ST4, K(0x3040), K(3), K(0x7));
	I(&c, LD4, 4, K(0x3040), K(1));
	I(&c, LD4, 5, K(0x3040), K(2));
	send_code(&c);
	run();
	CHECK_EQ(pool_getw(1), 0x44332211);
	CHECK_EQ(pool_getw(2), 0x4433);
	CHECK_EQ(pool_getw(3), 0x44);
	CHECK_EQ(get32(arena + 0x3004), 0x1234);
	CHECK_EQ(arena[0x3020], 0xFF);
	CHECK_EQ(arena[0x3021], 0);
	CHECK_EQ(get32(arena + 0x3025), 0xDEADBEEF);
	CHECK_EQ(arena[0x3040], 0xAB);
	CHECK_EQ(arena[0x3041], 0x07);
	CHECK_EQ(pool_getw(4), 0xB);
	CHECK_EQ(pool_getw(5), 0);
	CHECK_EQ(f.nev, 0);
	/* pixel (x, y) of a 4-bpp bitmap at B, stride s, is LD4 element 2·s·y + x of B */
	CHECK_EQ(vm_fill(arena, v, &detail), 0);
	c.n = 0;
	I(&c, LD4, 6, K(0x4000), K(2 * 10 * 2 + 3));
	I(&c, LD4, 7, K(0x4000), K(2 * 10 * 2 + 4));
	send_code(&c);
	run();
	CHECK_EQ(pool_getw(6), 9);
	CHECK_EQ(pool_getw(7), 0);
	/* the last element fits; one more is error 3 with its address, and changes nothing */
	put32(arena + VM_ARENA_SIZE - 4, 0x5A5A5A5A);
	c.n = 0;
	I(&c, LD32, 8, K(VM_ARENA_SIZE - 4), K(0));
	I(&c, ST4, K(VM_ARENA_SIZE - 1), K(1), K(3));
	send_code(&c);
	run();
	CHECK_EQ(pool_getw(8), 0x5A5A5A5A);
	CHECK_EQ(arena[VM_ARENA_SIZE - 1], 0x53);
	CHECK_EQ(f.nev, 0);
	setw(8, 77);
	c.n = 0;
	I(&c, LD32, 8, K(VM_ARENA_SIZE - 4), K(1));
	send_code(&c);
	run();
	EXPECT_ERROR(3, LD32, 0, VM_ARENA_SIZE, 0);
	CHECK_EQ(pool_getw(8), 77);
	c.n = 0;
	I(&c, LD32, 8, K(0xFFFFFFF0), K(4));
	send_code(&c);
	run();
	EXPECT_ERROR(3, LD32, 0, 0, 0);
	c.n = 0;
	I(&c, ST4, K(VM_ARENA_SIZE - 1), K(2), K(3));
	send_code(&c);
	run();
	EXPECT_ERROR(3, ST4, 0, VM_ARENA_SIZE, 0);
	c.n = 0;
	I(&c, LD16, 8, K(VM_ARENA_SIZE - 1), K(0));
	send_code(&c);
	run();
	EXPECT_ERROR(3, LD16, 0, VM_ARENA_SIZE - 1, 0);
	c.n = 0;
	I(&c, ST8, K(0x10), K(0xFFFFFFFF), K(1));
	send_code(&c);
	run();
	EXPECT_ERROR(3, ST8, 0, 0xF, 0);
	CHECK_EQ(arena[0xF], 0);
	CHECK_EQ(pool_getw(8), 77);
}

static void test_memory(void)
{
	static const uint8_t src[] = "AAAAAAAABCDEFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF xyz";
	uint8_t pb[256], buf[300], x[4] = {1, 2, 3, 4};
	uint32_t n = packbits(src, sizeof(src), pb);
	struct code c = {{0}, 0};

	setup(0);
	W(&c, VM_ARENA_SIZE - 3, x, 3);
	send_code(&c);
	run();
	CHECK_EQ(arena[VM_ARENA_SIZE - 1], 3);
	CHECK_EQ(f.nev, 0);
	c.n = 0;
	W(&c, VM_ARENA_SIZE - 3, x, 4);
	send_code(&c);
	run();
	EXPECT_ERROR(3, WRITE, 0, VM_ARENA_SIZE - 3, 0);
	CHECK_EQ(arena[VM_ARENA_SIZE - 3], 1);
	/* WRITE_RLE: u24 addr, u16 len, u8 n, PackBits; a 0x80 header does nothing */
	buf[0] = WRITE_RLE;
	put32(buf + 1, 0x5000);
	put16(buf + 4, sizeof(src));
	buf[6] = (uint8_t)(n + 1);
	buf[7] = 0x80;
	memcpy(buf + 8, pb, n);
	f.nev = 0;
	send(buf, 8 + n);
	run();
	CHECK_EQ(memcmp(arena + 0x5000, src, sizeof(src)), 0);
	CHECK_EQ(f.nev, 0);
	put16(buf + 4, sizeof(src) + 1);
	send(buf, 8 + n);
	run();
	EXPECT_ERROR(4, WRITE_RLE, 0, 0, 0);
	put16(buf + 4, sizeof(src) - 1);
	send(buf, 8 + n);
	run();
	EXPECT_ERROR(4, WRITE_RLE, 0, 0, 0);
	put16(buf + 4, sizeof(src));
	buf[6] = (uint8_t)(n - 1);
	send(buf, 6 + n);
	run();
	EXPECT_ERROR(4, WRITE_RLE, 0, 0, 0);
	put32(buf + 1, VM_ARENA_SIZE - 10);
	put16(buf + 4, sizeof(src));
	buf[6] = (uint8_t)(n - 1);
	send(buf, 6 + n);
	run();
	EXPECT_ERROR(3, WRITE_RLE, 0, VM_ARENA_SIZE - 10, 0);
	CHECK_EQ(arena[VM_ARENA_SIZE - 10], 0);
	/* COPY is a memmove; ranges must end inside the arena */
	for (int i = 0; i < 16; i++) {
		arena[0x6000 + i] = (uint8_t)i;
	}
	c.n = 0;
	I(&c, COPY, K(0x6004), K(0x6000), K(8));
	send_code(&c);
	run();
	for (int i = 0; i < 8; i++) {
		CHECK_EQ(arena[0x6004 + i], i);
	}
	c.n = 0;
	I(&c, COPY, K(0x6000), K(0x6002), K(8));
	I(&c, COPY, K(0x7000), K(VM_ARENA_SIZE), K(0));
	f.nev = 0;
	send_code(&c);
	run();
	CHECK_EQ(arena[0x6000], 2);
	CHECK_EQ(arena[0x6001], 3);
	CHECK_EQ(arena[0x6002], 0);
	CHECK_EQ(arena[0x6007], 5);
	CHECK_EQ(arena[0x6008], 4);
	CHECK_EQ(f.nev, 0);
	c.n = 0;
	I(&c, COPY, K(0x7000), K(VM_ARENA_SIZE - 4), K(5));
	send_code(&c);
	run();
	EXPECT_ERROR(3, COPY, 0, VM_ARENA_SIZE - 4, 0);
	c.n = 0;
	I(&c, COPY, K(VM_ARENA_SIZE - 4), K(0x7000), K(5));
	send_code(&c);
	run();
	EXPECT_ERROR(3, COPY, 0, VM_ARENA_SIZE - 4, 0);
	/* UNPACK dst, src, n, len */
	memcpy(arena + 0x8000, pb, n);
	f.nev = 0;
	c.n = 0;
	I(&c, UNPACK, K(0x8100), K(0x8000), K(n), K(sizeof(src)));
	I(&c, UNPACK, K(0x8300), K(0x8000), K(0), K(0));
	send_code(&c);
	run();
	CHECK_EQ(memcmp(arena + 0x8100, src, sizeof(src)), 0);
	CHECK_EQ(f.nev, 0);
	c.n = 0;
	I(&c, UNPACK, K(0x8002), K(0x8000), K(n), K(sizeof(src)));
	send_code(&c);
	run();
	EXPECT_ERROR(3, UNPACK, 0, 0x8002, 0);
	c.n = 0;
	I(&c, UNPACK, K(0x7FF6), K(0x8000), K(n), K(sizeof(src)));
	send_code(&c);
	run();
	EXPECT_ERROR(3, UNPACK, 0, 0x7FF6, 0);
	c.n = 0;
	I(&c, UNPACK, K(0x8400), K(0x8000), K(n), K(sizeof(src) + 1));
	send_code(&c);
	run();
	EXPECT_ERROR(4, UNPACK, 0, 0, 0);
	c.n = 0;
	I(&c, UNPACK, K(0x8400), K(0x8000), K(0), K(1));
	send_code(&c);
	run();
	EXPECT_ERROR(4, UNPACK, 0, 0, 0);
	c.n = 0;
	I(&c, UNPACK, K(0x8400), K(VM_ARENA_SIZE - 2), K(3), K(1));
	send_code(&c);
	run();
	EXPECT_ERROR(3, UNPACK, 0, VM_ARENA_SIZE - 2, 0);
	c.n = 0;
	I(&c, UNPACK, K(VM_ARENA_SIZE - 2), K(0x8000), K(n), K(3));
	send_code(&c);
	run();
	EXPECT_ERROR(3, UNPACK, 0, VM_ARENA_SIZE - 2, 0);
	CHECK_EQ(arena[0x8400], 0);
}

static void test_host_rules(void)
{
	static const uint8_t program_only[] = {JMP, RET, HALT, BEQ, BNE, BLTU, BGEU, BLTS, BGES, WAIT_EVENT};
	static const uint8_t invalid[] = {0x00, 0x05, 0x0F, 0x1E, 0x1F, 0x28, 0x2F, 0x3B, 0x3F, 0x46, 0x4F, 0x57, 0x60, 0x80, 0xFF};
	static const char *cut[] = {"11 31", "11 3f f0 20 01 30", "01 00 50 00 05 aa aa", "02 00 50 00 04 00 03 aa",
				    "50 ff ff ff f0 38 00", "31 00 80"};
	static const char *pad[] = {"42 f1 28 00", "40 01", "50 00 00 00 01"};
	uint8_t aa = 0xAA, bb = 0xBB, buf[64];
	struct code c;
	struct event *e;

	setup(0);
	for (unsigned i = 0; i < sizeof(program_only); i++) {
		uint8_t op = program_only[i];
		int k, lit;

		spec_shape(op, &k, &lit);
		arena[0x500] = arena[0x501] = 0;
		c.n = 0;
		W(&c, 0x500, &aa, 1);
		emit(&c, op, (const int[]){1, 2}, k, lit, 0x8000);
		W(&c, 0x501, &bb, 1);
		send_code(&c);
		run();
		EXPECT_ERROR(2, op, 6, 0, 0);
		CHECK_EQ(arena[0x500], 0xAA);
		CHECK_EQ(arena[0x501], 0);
	}
	for (unsigned i = 0; i < sizeof(invalid); i++) {
		buf[0] = invalid[i];
		memset(buf + 1, 0, 8);
		send(buf, 9);
		run();
		EXPECT_ERROR(2, invalid[i], 0, 0, 0);
	}
	for (unsigned i = 0; i < sizeof(cut) / sizeof(cut[0]); i++) {
		uint32_t n = hex(cut[i], buf + 4);

		buf[0] = MOV;
		buf[1] = 0x1f;
		buf[2] = 0;
		buf[3] = 0;
		send(buf, 4 + n);
		run();
		EXPECT_ERROR(1, buf[4], 4, 0, 0);
	}
	for (unsigned i = 0; i < sizeof(pad) / sizeof(pad[0]); i++) {
		uint32_t n = hex(pad[i], buf);

		send(buf, n);
		run();
		EXPECT_ERROR(1, buf[0], 0, 0, 0);
	}
	/* the packet after a failing one runs */
	c.n = 0;
	W(&c, 0x502, &bb, 1);
	send_code(&c);
	run();
	CHECK_EQ(arena[0x502], 0xBB);
	/* END ends the packet: nothing after it runs, no error */
	f.nev = 0;
	c.n = 0;
	O(&c, END);
	W(&c, 0x504, &bb, 1);
	send_code(&c);
	run();
	CHECK_EQ(arena[0x504], 0);
	CHECK_EQ(f.nev, 0);
	CHECK_EQ(f.head, f.tail);
	/* the ERROR body byte for byte: seq, offset, op, code, detail, address */
	f.nev = 0;
	c.n = 0;
	W(&c, 0x503, &bb, 1);
	I(&c, SYS_READ, 1, K(0x77));
	send_code(&c);
	run();
	e = last(VM_EV_ERROR);
	CHECK(e != NULL);
	if (e) {
		uint8_t want[13] = {0, 0, 6, SYS_READ, 5, 0x77, 0, 0, 0, 0, 0, 0, 0};

		put16(want, (uint16_t)(f.seq - 1));
		CHECK_EQ(e->len, 13);
		CHECK_EQ(memcmp(e->body, want, 13), 0);
	}
}

static void test_host_call(void)
{
	struct code c = {{0}, 0}, p = {{0}, 0};
	uint8_t one = 0x5A;
	uint32_t j0;

	setup(0);
	I(&c, ADD, 1, 1, K(1));
	O(&c, RET);
	place(0x8000, &c);
	J(&p, CALL, 0x8000);
	J(&p, CALL, 0x8000);
	W(&p, 0x600, &one, 1);
	send_code(&p);
	run();
	CHECK_EQ(pool_getw(1), 2);
	CHECK_EQ(arena[0x600], 0x5A);
	CHECK_EQ(f.nev, 0);
	/* END in called code ends the packet without an error */
	c.n = 0;
	O(&c, END);
	place(0x8100, &c);
	p.n = 0;
	J(&p, CALL, 0x8100);
	W(&p, 0x601, &one, 1);
	send_code(&p);
	run();
	CHECK_EQ(arena[0x601], 0);
	CHECK_EQ(f.nev, 0);
	CHECK_EQ(f.head, f.tail);
	/* WAIT_EVENT and HALT are error 2 there; the offset is the CALL's, the address the instruction's */
	c.n = 0;
	I(&c, ADD, 1, 1, K(1));
	I(&c, WAIT_EVENT, 2, K(1));
	place(0x8200, &c);
	c.n = 0;
	O(&c, HALT);
	place(0x8280, &c);
	p.n = 0;
	W(&p, 0x602, &one, 1);
	J(&p, CALL, 0x8200);
	send_code(&p);
	run();
	EXPECT_ERROR(2, WAIT_EVENT, 6, 0, 0x8205);
	CHECK_EQ(pool_getw(1), 3);
	p.n = 0;
	J(&p, CALL, 0x8280);
	send_code(&p);
	run();
	EXPECT_ERROR(2, HALT, 0, 0, 0x8280);
	/* an error in called code */
	c.n = 0;
	I(&c, ST32, K(VM_ARENA_SIZE), K(0), K(1));
	place(0x8300, &c);
	p.n = 0;
	W(&p, 0x603, &one, 1);
	W(&p, 0x604, &one, 1);
	J(&p, CALL, 0x8300);
	send_code(&p);
	run();
	EXPECT_ERROR(3, ST32, 12, VM_ARENA_SIZE, 0x8300);
	/* depth: the packet's CALL is depth 1, so 15 more fit and the 16th fails */
	c.n = 0;
	J(&c, CALL, 0x8400);
	place(0x8400, &c);
	j0 = vm.stats.jumps;
	p.n = 0;
	J(&p, CALL, 0x8400);
	send_code(&p);
	run();
	EXPECT_ERROR(9, CALL, 0, 0, 0x8400);
	CHECK_EQ(vm.stats.jumps - j0, 16);
	/* budget: the packet's CALL is jump 1, jump 1001 fails */
	c.n = 0;
	J(&c, JMP, 0x8500);
	place(0x8500, &c);
	j0 = vm.stats.jumps;
	p.n = 0;
	J(&p, CALL, 0x8500);
	send_code(&p);
	run();
	EXPECT_ERROR(10, JMP, 0, 0, 0x8500);
	CHECK_EQ(vm.stats.jumps - j0, 1000);
	/* SLEEP_UNTIL in called code holds the packet */
	c.n = 0;
	I(&c, SLEEP_UNTIL, K(f.t + 5000));
	O(&c, RET);
	place(0x8600, &c);
	p.n = 0;
	J(&p, CALL, 0x8600);
	W(&p, 0x605, &one, 1);
	send_code(&p);
	f.nev = 0;
	run();
	CHECK_EQ(arena[0x605], 0);
	run_for(4999);
	CHECK_EQ(arena[0x605], 0);
	run_for(1);
	CHECK_EQ(arena[0x605], 0x5A);
	/* called code is host code: TRIGGER reads 0xFF; jumps and branches are allowed */
	c.n = 0;
	J(&c, JMP, 0x8704);
	I(&c, SYS_READ, 310, K(VM_P_TRIGGER));
	B(&c, BEQ, 0x8718, 0, 0);
	I(&c, ADD, 311, 311, K(100));
	I(&c, ADD, 311, 311, K(1));
	O(&c, RET);
	place(0x8700, &c);
	p.n = 0;
	J(&p, CALL, 0x8700);
	send_code(&p);
	run();
	CHECK_EQ(pool_getw(310), 0xFF);
	CHECK_EQ(pool_getw(311), 1);
	CHECK_EQ(f.nev, 0);
	/* a CALL target outside the arena fails in the packet */
	p.n = 0;
	J(&p, CALL, VM_ARENA_SIZE);
	send_code(&p);
	run();
	EXPECT_ERROR(3, CALL, 0, VM_ARENA_SIZE, 0);
}

static void test_program(void)
{
	struct code c = {{0}, 0}, p = {{0}, 0};
	struct event *e;
	uint8_t s[32], one = 1;
	uint32_t j0, loop, br;

	setup(0);
	I(&c, NOTIFY, K(7), K(8));
	I(&c, SYS_READ, 320, K(VM_P_TRIGGER));
	O(&c, END);
	place(0x9000, &c);
	table(VM_T_START, 0x9000);
	CHECK_EQ(vm_start(&vm, 0x40), 0);
	run();
	e = last(VM_EV_NOTIFY);
	CHECK(e != NULL);
	if (e) {
		CHECK_EQ(e->len, 11);
		CHECK_EQ(get16(e->body), 1);
		CHECK_EQ(e->body[2], 0);
		CHECK_EQ(get32(e->body + 3), 7);
		CHECK_EQ(get32(e->body + 7), 8);
	}
	CHECK_EQ(pool_getw(320), 0);
	I(&p, NOTIFY, K(1), K(2));
	send_code(&p);
	run();
	e = last(VM_EV_NOTIFY);
	CHECK(e && get16(e->body) == 2 && e->body[2] == 0xFF);
	f.drop = 1;
	send_code(&p);
	run();
	f.drop = 0;
	send_code(&p);
	run();
	e = last(VM_EV_NOTIFY);
	CHECK(e && get16(e->body) == 4);
	vm_status(&vm, s);
	CHECK_EQ(get16(s + 26), 1);
	CHECK_EQ(get32(s + 20), 1);
	/* HALT */
	c.n = 0;
	I(&c, ADD, 321, 321, K(1));
	O(&c, HALT);
	place(0x9100, &c);
	table(VM_T_USER0, 0x9100);
	p.n = 0;
	I(&p, RAISE, K(12));
	send_code(&p);
	run();
	EXPECT_HALT(12, 0x9109, HALT, 0, 0);
	CHECK_EQ(pool_getw(321), 1);
	vm_status(&vm, s);
	CHECK_EQ(s[0], VM_HALTED);
	CHECK_EQ(get32(s + 8), 0x9109);
	CHECK_EQ(s[15], 0);
	CHECK_EQ(s[1], 0xFF);
	/* an error halts: pending bits and timer cleared, host code goes on */
	c.n = 0;
	I(&c, ST32, K(VM_ARENA_SIZE), K(0), K(1));
	place(0x9200, &c);
	table(13, 0x9200);
	CHECK_EQ(vm_start(&vm, 0x40), 0);
	run();
	p.n = 0;
	I(&p, SYS_WRITE, K(VM_P_TIMER), K(f.t + 1000000));
	I(&p, RAISE, K(14));
	I(&p, RAISE, K(13));
	send_code(&p);
	run();
	EXPECT_HALT(13, 0x9200, ST32, 3, VM_ARENA_SIZE);
	vm_status(&vm, s);
	CHECK_EQ(s[0], VM_HALTED);
	CHECK_EQ(s[15], 3);
	CHECK_EQ(get16(s + 2), 0);
	CHECK_EQ(get32(s + 16), 0);
	CHECK_EQ(s[28], 0);
	p.n = 0;
	W(&p, 0x606, &one, 1);
	send_code(&p);
	run();
	CHECK_EQ(arena[0x606], 1);
	/* the trigger table must lie in the arena */
	CHECK_EQ(vm_start(&vm, VM_ARENA_SIZE - 63), VM_E_ARENA);
	CHECK_EQ(vm_start(&vm, 0xFFFFFFFF), VM_E_ARENA);
	CHECK_EQ(vm_start(&vm, VM_ARENA_SIZE - 64), 0);
	run();
	vm_status(&vm, s);
	CHECK_EQ(s[0], VM_RUNNING);
	CHECK_EQ(get16(s + 2), 1);
	/* RET with an empty stack; 16 CALLs fit, the 17th fails */
	c.n = 0;
	O(&c, RET);
	place(0x9300, &c);
	table(VM_T_START, 0x9300);
	vm_start(&vm, 0x40);
	run();
	EXPECT_HALT(0, 0x9300, RET, 9, 0);
	c.n = 0;
	J(&c, CALL, 0x9310);
	place(0x9310, &c);
	table(VM_T_START, 0x9310);
	j0 = vm.stats.jumps;
	vm_start(&vm, 0x40);
	run();
	EXPECT_HALT(0, 0x9310, CALL, 9, 0);
	CHECK_EQ(vm.stats.jumps - j0, 16);
	/* budget: 1000 jumps run, jump 1001 is error 10 and changes nothing */
	for (uint32_t limit = 1001; limit <= 1002; limit++) {
		c.n = 0;
		I(&c, MOV, 1, K(0));
		loop = 0x9400 + c.n;
		I(&c, ADD, 1, 1, K(1));
		br = 0x9400 + c.n;
		B(&c, BNE, loop, 1, K(limit));
		O(&c, END);
		place(0x9400, &c);
		table(VM_T_START, 0x9400);
		f.nev = 0;
		vm_start(&vm, 0x40);
		run();
		CHECK_EQ(pool_getw(1), 1001);
		if (limit == 1001) {
			CHECK_EQ(count(VM_EV_HALT), 0);
		} else {
			EXPECT_HALT(0, br, BNE, 10, 0);
		}
	}
	/* the budget starts again after a SLEEP_UNTIL that waited, not after one already past */
	for (int past = 0; past < 2; past++) {
		c.n = 0;
		I(&c, MOV, 1, K(0));
		loop = 0x9500 + c.n;
		I(&c, ADD, 1, 1, K(1));
		B(&c, BNE, loop, 1, K(600));
		I(&c, SYS_READ, 2, K(VM_P_TIME_US));
		I(&c, ADD, 2, 2, K(10));
		I(&c, SLEEP_UNTIL, past ? K(0) : 2);
		loop = 0x9500 + c.n;
		I(&c, ADD, 1, 1, K(1));
		br = 0x9500 + c.n;
		B(&c, BNE, loop, 1, K(1200));
		O(&c, END);
		place(0x9500, &c);
		table(VM_T_START, 0x9500);
		f.nev = 0;
		vm_start(&vm, 0x40);
		run_for(100);
		if (past) {
			EXPECT_HALT(0, br, BNE, 10, 0);
			CHECK_EQ(pool_getw(1), 1002);
		} else {
			CHECK_EQ(count(VM_EV_HALT), 0);
			CHECK_EQ(pool_getw(1), 1200);
		}
	}
	/* a WAIT_EVENT that is satisfied at once does not start the budget again */
	c.n = 0;
	I(&c, MOV, 1, K(0));
	loop = 0x9600 + c.n;
	I(&c, ADD, 1, 1, K(1));
	B(&c, BNE, loop, 1, K(600));
	I(&c, RAISE, K(14));
	I(&c, WAIT_EVENT, 3, K(1 << 14));
	loop = 0x9600 + c.n;
	I(&c, ADD, 1, 1, K(1));
	br = 0x9600 + c.n;
	B(&c, BNE, loop, 1, K(1200));
	O(&c, END);
	place(0x9600, &c);
	table(VM_T_START, 0x9600);
	vm_start(&vm, 0x40);
	run();
	EXPECT_HALT(0, br, BNE, 10, 0);
	CHECK_EQ(pool_getw(3), 14);
	/* jump targets and handlers outside the arena, the arena's end, zeroed memory */
	c.n = 0;
	I(&c, ADD, 1, 1, K(1));
	J(&c, JMP, VM_ARENA_SIZE);
	place(0x9700, &c);
	table(VM_T_START, 0x9700);
	vm_start(&vm, 0x40);
	run();
	EXPECT_HALT(0, 0x9705, JMP, 3, VM_ARENA_SIZE);
	table(VM_T_START, VM_ARENA_SIZE);
	vm_start(&vm, 0x40);
	run();
	EXPECT_HALT(0, VM_ARENA_SIZE, 0, 3, VM_ARENA_SIZE);
	arena[VM_ARENA_SIZE - 2] = ADD;
	arena[VM_ARENA_SIZE - 1] = 0x31;
	table(VM_T_START, VM_ARENA_SIZE - 2);
	vm_start(&vm, 0x40);
	run();
	EXPECT_HALT(0, VM_ARENA_SIZE - 2, ADD, 1, 0);
	table(VM_T_START, 0x9F00);
	vm_start(&vm, 0x40);
	run();
	EXPECT_HALT(0, 0x9F00, 0, 2, 0);
	CHECK_EQ(count(VM_EV_ERROR), 0);
}

static void counter_handler(uint32_t addr, uint32_t word)
{
	struct code c = {{0}, 0};

	I(&c, ADD, (int)word, (int)word, K(1));
	O(&c, END);
	place(addr, &c);
}

/* handler that appends its trigger ID to the log at 0x700 (count in word 344) */
static void log_handler(uint32_t addr)
{
	struct code c = {{0}, 0};

	I(&c, SYS_READ, 5, K(VM_P_TRIGGER));
	I(&c, ST8, K(0x700), 344, 5);
	I(&c, ADD, 344, 344, K(1));
	O(&c, END);
	place(addr, &c);
}

static void test_triggers(void)
{
	struct code c = {{0}, 0}, p = {{0}, 0};
	uint8_t s[32];
	uint32_t v, t0;
	int32_t prev[3] = {0, 1000, 0}, cur[3];

	setup(0);
	/* TIMER: one-shot at the time written, a write replaces, a time not ahead fires at once */
	I(&c, SYS_WRITE, K(VM_P_TIMER), 340);
	O(&c, END);
	place(0xA000, &c);
	counter_handler(0xA100, 341);
	table(VM_T_START, 0xA000);
	table(VM_T_TIMER, 0xA100);
	t0 = f.t;
	setw(340, t0 + 1000);
	vm_start(&vm, 0x40);
	run();
	vm_status(&vm, s);
	CHECK_EQ(s[28], 1);
	CHECK_EQ(get32(s + 16), t0 + 1000);
	run_for(999);
	CHECK_EQ(pool_getw(341), 0);
	run_for(1);
	CHECK_EQ(pool_getw(341), 1);
	run_for(100000);
	CHECK_EQ(pool_getw(341), 1);
	vm_status(&vm, s);
	CHECK_EQ(s[28], 0);
	CHECK(vm_prop(&vm, VM_P_TIMER, &v) == 0 && v == t0 + 1000);
	p.n = 0;
	I(&p, SYS_WRITE, K(VM_P_TIMER), K(f.t + 500));
	I(&p, SYS_WRITE, K(VM_P_TIMER), K(f.t + 2000));
	send_code(&p);
	run_for(1000);
	CHECK_EQ(pool_getw(341), 1);
	run_for(1000);
	CHECK_EQ(pool_getw(341), 2);
	p.n = 0;
	I(&p, SYS_WRITE, K(VM_P_TIMER), K(f.t - 5));
	send_code(&p);
	run();
	CHECK_EQ(pool_getw(341), 3);
	p.n = 0;
	I(&p, SYS_WRITE, K(VM_P_TIMER), K(f.t + 0x80000000u));
	send_code(&p);
	run();
	CHECK_EQ(pool_getw(341), 4);
	p.n = 0;
	I(&p, SYS_WRITE, K(VM_P_TIMER), K(f.t + 0x7FFFFFFFu));
	send_code(&p);
	run_for(100000);
	CHECK_EQ(pool_getw(341), 4);
	/* one pending bit per trigger, repeats coalesce, the lowest ID runs first */
	log_handler(0xA200);
	table(12, 0xA200);
	table(13, 0xA200);
	setw(344, 0);
	vm_start(&vm, 0x40);
	run();
	p.n = 0;
	I(&p, RAISE, K(13));
	I(&p, RAISE, K(12));
	I(&p, RAISE, K(12));
	I(&p, RAISE, K(12));
	I(&p, SYS_READ, 345, K(VM_P_PENDING));
	send_code(&p);
	run();
	CHECK_EQ(pool_getw(345), 0x3000);
	CHECK_EQ(pool_getw(344), 2);
	CHECK_EQ(arena[0x700], 12);
	CHECK_EQ(arena[0x701], 13);
	vm_status(&vm, s);
	CHECK_EQ(get16(s + 24), 2);
	/* a pending trigger without a handler stays pending until a WAIT_EVENT takes it */
	p.n = 0;
	I(&p, RAISE, K(14));
	send_code(&p);
	run();
	CHECK(vm_prop(&vm, VM_P_PENDING, &v) == 0 && v == 0x4000);
	c.n = 0;
	I(&c, WAIT_EVENT, 346, K(1 << 14 | 1 << 15));
	I(&c, ADD, 347, 347, K(1));
	O(&c, END);
	place(0xA300, &c);
	table(12, 0xA300);
	p.n = 0;
	I(&p, RAISE, K(12));
	send_code(&p);
	run();
	CHECK_EQ(pool_getw(346), 14);
	CHECK_EQ(pool_getw(347), 1);
	vm_status(&vm, s);
	CHECK_EQ(get16(s + 2), 0);
	/* a waiting task gets a trigger of its mask; other triggers wait for their handlers */
	setw(344, 0);
	send_code(&p);
	run();
	vm_status(&vm, s);
	CHECK_EQ(s[12], VM_TASK_WAIT);
	CHECK_EQ(s[1], 12);
	p.n = 0;
	I(&p, RAISE, K(13));
	send_code(&p);
	run();
	CHECK_EQ(pool_getw(344), 0);
	vm_status(&vm, s);
	CHECK_EQ(get16(s + 2), 0x2000);
	p.n = 0;
	I(&p, RAISE, K(15));
	send_code(&p);
	run();
	CHECK_EQ(pool_getw(346), 15);
	CHECK_EQ(pool_getw(347), 2);
	CHECK_EQ(pool_getw(344), 1);
	CHECK_EQ(arena[0x700], 13);
	/* WAIT_EVENT masks 0 and above bit 15 are error 5 */
	c.n = 0;
	I(&c, WAIT_EVENT, 346, K(0));
	place(0xA400, &c);
	table(12, 0xA400);
	p.n = 0;
	I(&p, RAISE, K(12));
	send_code(&p);
	run();
	EXPECT_HALT(12, 0xA400, WAIT_EVENT, 5, 0);
	c.n = 0;
	I(&c, WAIT_EVENT, 346, K(0x10000));
	place(0xA400, &c);
	vm_start(&vm, 0x40);
	run();
	send_code(&p);
	run();
	EXPECT_HALT(12, 0xA400, WAIT_EVENT, 5, 0x10000);
	p.n = 0;
	I(&p, RAISE, K(16));
	vm_start(&vm, 0x40);
	run();
	send_code(&p);
	run();
	EXPECT_ERROR(5, RAISE, 0, 16, 0);
	/* triggers fire only while a program runs; VM_START clears them and keeps the settings */
	vm_stop(&vm);
	p.n = 0;
	I(&p, RAISE, K(12));
	I(&p, SYS_WRITE, K(VM_P_TIMER), K(f.t + 10));
	I(&p, SYS_WRITE, K(VM_P_MOTION_MG), K(50));
	I(&p, SYS_WRITE, K(VM_P_PITCH_LO), K((uint32_t)-10));
	I(&p, SYS_WRITE, K(VM_P_IMU_RATE), K(52));
	send_code(&p);
	run();
	vm_status(&vm, s);
	CHECK_EQ(s[0], VM_STOPPED);
	CHECK_EQ(get16(s + 2), 0);
	table(VM_T_START, 0);
	vm_start(&vm, 0x40);
	vm_status(&vm, s);
	CHECK_EQ(get16(s + 2), 1);
	CHECK_EQ(s[28], 0);
	CHECK_EQ(get32(s + 16), 0);
	CHECK(vm_prop(&vm, VM_P_MOTION_MG, &v) == 0 && v == 50);
	CHECK(vm_prop(&vm, VM_P_PITCH_LO, &v) == 0 && v == (uint32_t)-10);
	CHECK(vm_prop(&vm, VM_P_IMU_RATE, &v) == 0 && v == 52);
	CHECK_EQ(f.imu_rate, 52);
	/* triggers from the platform (poll) */
	counter_handler(0xA500, 348);
	table(VM_T_MOTION, 0xA500);
	run();
	f.fired = 1 << VM_T_MOTION;
	run();
	CHECK_EQ(pool_getw(348), 1);
	cur[0] = 6, cur[1] = 1008, cur[2] = 0;
	CHECK_EQ(vm_moved(prev, cur, 10), 0);
	cur[0] = 7;
	CHECK_EQ(vm_moved(prev, cur, 10), 1);
	CHECK_EQ(vm_moved(prev, cur, 0), 0);
	CHECK_EQ(vm_moved(prev, prev, 1), 0);
	prev[0] = INT32_MIN, cur[0] = INT32_MAX, prev[1] = cur[1] = prev[2] = cur[2] = 0;
	CHECK_EQ(vm_moved(prev, cur, 0xFFFFFFFF), 0);
	cur[1] = 1;
	CHECK_EQ(vm_moved(prev, cur, 0xFFFFFFFF), 1);
	CHECK_EQ(vm_moved(prev, cur, 0xFFFFFFFE), 1);
	/* TILT: PITCH leaves [PITCH_LO, PITCH_HI]; the handler sets a new range around the value */
	vm_stop(&vm);
	p.n = 0;
	I(&p, SYS_WRITE, K(VM_P_PITCH_LO), K((uint32_t)-90));
	I(&p, SYS_WRITE, K(VM_P_PITCH_HI), K(20));
	send_code(&p);
	run();
	c.n = 0;
	I(&c, ADD, 350, 350, K(1));
	I(&c, SYS_WRITE, K(VM_P_PITCH_LO), K(15));
	I(&c, SYS_WRITE, K(VM_P_PITCH_HI), K(90));
	O(&c, END);
	place(0xA600, &c);
	table(VM_T_TILT, 0xA600);
	f.props[VM_P_PITCH] = 0;
	vm_start(&vm, 0x40);
	run();
	f.props[VM_P_PITCH] = 30;
	run();
	CHECK_EQ(pool_getw(350), 1);
	run();
	CHECK_EQ(pool_getw(350), 1);
	f.props[VM_P_PITCH] = 10;
	run();
	CHECK_EQ(pool_getw(350), 2);
	f.props[VM_P_PITCH] = VM_UNKNOWN;
	run();
	f.props[VM_P_PITCH] = 10;
	run();
	CHECK_EQ(pool_getw(350), 2);
	/* a range is evaluated when the executor goes idle, not between instructions */
	counter_handler(0xA700, 351);
	table(VM_T_TILT, 0xA700);
	p.n = 0;
	I(&p, SYS_WRITE, K(VM_P_PITCH_LO), K(0));
	I(&p, SYS_WRITE, K(VM_P_PITCH_HI), K(20));
	send_code(&p);
	run();
	CHECK_EQ(pool_getw(351), 0);
	p.n = 0;
	I(&p, SYS_WRITE, K(VM_P_PITCH_LO), K(12));
	I(&p, SYS_WRITE, K(VM_P_PITCH_LO), K(3));
	send_code(&p);
	run();
	CHECK_EQ(pool_getw(351), 0);
	p.n = 0;
	I(&p, SYS_WRITE, K(VM_P_PITCH_LO), K(12));
	send_code(&p);
	p.n = 0;
	I(&p, SYS_WRITE, K(VM_P_PITCH_LO), K(3));
	send_code(&p);
	run();
	CHECK_EQ(pool_getw(351), 1);
	/* LIGHT: unsigned range */
	counter_handler(0xA800, 352);
	table(VM_T_LIGHT, 0xA800);
	p.n = 0;
	I(&p, SYS_WRITE, K(VM_P_LIGHT_LO), K(100));
	send_code(&p);
	run();
	f.props[VM_P_LIGHT] = 50;
	run();
	CHECK_EQ(pool_getw(352), 1);
	f.props[VM_P_LIGHT] = 500;
	run();
	f.props[VM_P_LIGHT] = 0x90000000;
	run();
	CHECK_EQ(pool_getw(352), 1);
	f.props[VM_P_LIGHT] = 50;
	run();
	CHECK_EQ(pool_getw(352), 2);
	/* BATTERY, CASE, WEAR fire on changes while the program runs, from a baseline at VM_START */
	counter_handler(0xA900, 353);
	counter_handler(0xA980, 354);
	counter_handler(0xAA00, 355);
	table(VM_T_BATTERY, 0xA900);
	table(VM_T_CASE, 0xA980);
	table(VM_T_WEAR, 0xAA00);
	f.props[VM_P_BATT_PCT] = 49;
	run();
	CHECK_EQ(pool_getw(353), 1);
	f.props[VM_P_CHARGE] = 1;
	run();
	run();
	CHECK_EQ(pool_getw(353), 2);
	f.props[VM_P_IN_CASE] = 1;
	f.props[VM_P_WORN] = 1;
	run();
	CHECK_EQ(pool_getw(354), 1);
	CHECK_EQ(pool_getw(355), 1);
	f.props[VM_P_CASE_LID] = 0;
	run();
	CHECK_EQ(pool_getw(354), 2);
	f.props[VM_P_CASE_CHARGING] = 1;
	f.props[VM_P_CASE_BATT_PCT] = 80;
	run();
	CHECK_EQ(pool_getw(354), 3);
	f.props[VM_P_CASE_BATT_PCT] = 79;
	run();
	CHECK_EQ(pool_getw(354), 4);
	f.props[VM_P_CASE_LID] = VM_UNKNOWN;
	f.props[VM_P_CASE_CHARGING] = VM_UNKNOWN;
	f.props[VM_P_CASE_BATT_PCT] = VM_UNKNOWN;
	run();
	run();
	CHECK_EQ(pool_getw(354), 5);
	CHECK_EQ(pool_getw(353), 2);
	CHECK_EQ(pool_getw(355), 1);
	vm_stop(&vm);
	f.props[VM_P_BATT_PCT] = 30;
	run();
	vm_start(&vm, 0x40);
	run();
	CHECK_EQ(pool_getw(353), 2);
	CHECK_EQ(count(VM_EV_HALT), 2);
}

/* INPUT events (§6.3): the EVENTS mask, sent whether or not a program runs */
static struct event *nth_input(int n)
{
	for (uint32_t i = 0; i < f.nev; i++) {
		if (f.ev[i].type == VM_EV_INPUT && n-- == 0) {
			return &f.ev[i];
		}
	}
	return NULL;
}

static void expect_input(int line, int n, uint32_t count_, uint32_t trigger, uint32_t value)
{
	struct event *e = nth_input(n);

	if (!e || e->len != 7 || get16(e->body) != count_ || e->body[2] != trigger || get32(e->body + 3) != value) {
		fprintf(stderr, "line %d: INPUT %d: want count %u trigger %u value 0x%x, got %s count %u trigger %u value 0x%x\n",
			line, n, count_, trigger, value, e ? "" : "none", e ? get16(e->body) : 0, e ? e->body[2] : 0,
			e ? get32(e->body + 3) : 0);
		failures++;
	}
	checks++;
}
#define EXPECT_INPUT(n, c, t, v) expect_input(__LINE__, n, c, t, v)

/* vm_wants: the triggers with a handler and those the task waits for, while a program runs */
static void test_wants(void)
{
	struct code c = {{0}, 0}, w = {{0}, 0};

	setup(0);
	CHECK_EQ(vm_wants(&vm), 0);
	O(&c, END);
	place(0xA000, &c);
	I(&w, WAIT_EVENT, 300, K(1u << VM_T_WEAR));
	O(&w, END);
	place(0xA100, &w);
	table(VM_T_START, 0xA000);
	table(VM_T_TOUCH, 0xA000);
	table(VM_T_USER0, 0xA100);
	vm_start(&vm, 0x40);
	run();
	CHECK_EQ(vm_wants(&vm), (1u << VM_T_START) | (1u << VM_T_TOUCH) | (1u << VM_T_USER0));
	vm_trigger(&vm, VM_T_USER0, 0);
	run();
	CHECK_EQ(vm_wants(&vm), (1u << VM_T_START) | (1u << VM_T_TOUCH) | (1u << VM_T_USER0) | (1u << VM_T_WEAR));
	vm_stop(&vm);
	CHECK_EQ(vm_wants(&vm), 0);
}

static void test_input(void)
{
	struct code c = {{0}, 0}, p = {{0}, 0};
	int32_t a[3] = {0, 1000, 0}, b[3] = {3, 1004, 0};

	setup(0);
	f.props[VM_P_PITCH] = 0;
	f.props[VM_P_CASE_BATT_PCT] = VM_UNKNOWN;
	/* mask 0: nothing is reported */
	f.props[VM_P_BATT_PCT] = 51;
	run();
	vm_trigger(&vm, VM_T_MOTION, 7);
	run();
	CHECK_EQ(count(VM_EV_INPUT), 0);
	/* the mask samples the current values: only later changes are reported, no program running */
	vm_set_events(&vm, 1 << VM_T_MOTION | 1 << VM_T_TILT | 1 << VM_T_LIGHT | 1 << VM_T_BATTERY |
			   1 << VM_T_CASE | 1 << VM_T_WEAR | 1 << VM_T_TOUCH | 1 << VM_T_TIMER | 1 << 12);
	run();
	CHECK_EQ(count(VM_EV_INPUT), 0);
	f.props[VM_P_BATT_PCT] = 52;
	run();
	run();
	EXPECT_INPUT(0, 1, VM_T_BATTERY, 52);
	f.props[VM_P_CHARGE] = 1;
	run();
	EXPECT_INPUT(1, 2, VM_T_BATTERY, 52);
	f.props[VM_P_IN_CASE] = 1;
	f.props[VM_P_CASE_LID] = 1;
	f.props[VM_P_CASE_CHARGING] = 1;
	f.props[VM_P_CASE_BATT_PCT] = 80;
	run();
	EXPECT_INPUT(2, 3, VM_T_CASE, 0x5007);
	f.props[VM_P_CASE_LID] = VM_UNKNOWN;
	f.props[VM_P_CASE_BATT_PCT] = VM_UNKNOWN;
	run();
	EXPECT_INPUT(3, 4, VM_T_CASE, 0xFF05);
	f.props[VM_P_WORN] = 1;
	run();
	EXPECT_INPUT(4, 5, VM_T_WEAR, 1);
	/* TILT and LIGHT: once per crossing out of the range; UNKNOWN changes nothing */
	p.n = 0;
	I(&p, SYS_WRITE, K(VM_P_PITCH_LO), K((uint32_t)-90));
	I(&p, SYS_WRITE, K(VM_P_PITCH_HI), K(20));
	send_code(&p);
	run();
	f.props[VM_P_PITCH] = 25;
	run();
	run();
	EXPECT_INPUT(5, 6, VM_T_TILT, 25);
	f.props[VM_P_PITCH] = 30;
	run();
	f.props[VM_P_PITCH] = VM_UNKNOWN;
	run();
	f.props[VM_P_PITCH] = 10;
	run();
	f.props[VM_P_PITCH] = VM_UNKNOWN;
	run();
	f.props[VM_P_PITCH] = (uint32_t)-30;
	run();
	CHECK_EQ(count(VM_EV_INPUT), 6);
	f.props[VM_P_PITCH] = 21;
	run();
	EXPECT_INPUT(6, 7, VM_T_TILT, 21);
	f.props[VM_P_LIGHT] = 900;
	p.n = 0;
	I(&p, SYS_WRITE, K(VM_P_LIGHT_HI), K(1000));
	send_code(&p);
	run();
	f.props[VM_P_LIGHT] = 1001;
	run();
	EXPECT_INPUT(7, 8, VM_T_LIGHT, 1001);
	/* platform triggers: vm_trigger carries its value, poll bits read the property */
	vm_trigger(&vm, VM_T_MOTION, 12);
	EXPECT_INPUT(8, 9, VM_T_MOTION, 12);
	CHECK_EQ(vm_distance(a, b), 5);
	f.props[VM_P_TOUCH_GESTURE] = 2;
	f.fired = 1 << VM_T_TOUCH;
	run();
	EXPECT_INPUT(9, 10, VM_T_TOUCH, 2);
	/* RAISE and TIMER only while a program runs; START is reported too when in the mask */
	p.n = 0;
	I(&p, RAISE, K(12));
	send_code(&p);
	run();
	CHECK_EQ(count(VM_EV_INPUT), 10);
	counter_handler(0xA000, 360);
	counter_handler(0xA100, 361);
	c.n = 0;
	I(&c, SYS_WRITE, K(VM_P_TIMER), 362);
	O(&c, END);
	place(0xA200, &c);
	table(VM_T_START, 0xA200);
	table(VM_T_TIMER, 0xA000);
	table(12, 0xA100);
	setw(362, f.t + 5000);
	vm_start(&vm, 0x40);
	run();
	CHECK_EQ(count(VM_EV_INPUT), 10);
	run_for(6000);
	CHECK_EQ(pool_getw(360), 1);
	EXPECT_INPUT(10, 11, VM_T_TIMER, 0);
	p.n = 0;
	I(&p, RAISE, K(12));
	send_code(&p);
	run();
	CHECK_EQ(pool_getw(361), 1);
	EXPECT_INPUT(11, 12, 12, 0);
	/* with a program, a sensor change is reported once, and the program's trigger fires too */
	counter_handler(0xA300, 363);
	table(VM_T_BATTERY, 0xA300);
	f.props[VM_P_BATT_PCT] = 60;
	run();
	CHECK_EQ(pool_getw(363), 1);
	EXPECT_INPUT(12, 13, VM_T_BATTERY, 60);
	CHECK_EQ(count(VM_EV_INPUT), 13);
	/* a dropped event still counts; RESET_SESSION restarts the count; mask 0 stops reports */
	f.drop = 1;
	vm_trigger(&vm, VM_T_MOTION, 1);
	f.drop = 0;
	vm_trigger(&vm, VM_T_MOTION, 2);
	EXPECT_INPUT(14, 15, VM_T_MOTION, 2);
	vm_reset_session(&vm, 0);
	vm_trigger(&vm, VM_T_MOTION, 3);
	EXPECT_INPUT(15, 1, VM_T_MOTION, 3);
	vm_set_events(&vm, 0);
	vm_trigger(&vm, VM_T_MOTION, 4);
	f.props[VM_P_BATT_PCT] = 61;
	run();
	CHECK_EQ(count(VM_EV_INPUT), 16);
	/* vm_distance: rounded down, saturating */
	b[0] = 1, b[1] = 1001, b[2] = 1;
	CHECK_EQ(vm_distance(a, b), 1);
	a[0] = INT32_MIN, b[0] = INT32_MAX;
	CHECK_EQ(vm_distance(a, b), 0xFFFFFFFFu);
	a[0] = -1000000, b[0] = 1000000, a[1] = b[1] = 0, a[2] = b[2] = 0;
	CHECK_EQ(vm_distance(a, b), 2000000);
	CHECK_EQ(count(VM_EV_HALT), 0);
}

static void test_cap(void)
{
	struct code c = {{0}, 0}, p = {{0}, 0};
	uint8_t s[32], one = 1;
	uint32_t n0, t0, draws;

	setup(0);
	I(&c, RAISE, K(12));
	O(&c, END);
	place(0xB000, &c);
	table(VM_T_START, 0xB000);
	table(VM_T_USER0, 0xB000);
	f.tick = 1;
	vm_start(&vm, 0x40);
	n0 = (uint32_t)vm.stats.instructions;
	t0 = f.t;
	run_for(1000000);
	vm_status(&vm, s);
	/* 1 µs of measured run time per instruction: 30 000 instructions per 100 ms period */
	CHECK((uint32_t)vm.stats.instructions - n0 >= 300000 - 30000);
	CHECK((uint32_t)vm.stats.instructions - n0 <= 300000 + 30000 + 10);
	CHECK(get16(s + 29) >= 9 && get16(s + 29) <= 11);
	CHECK_EQ(s[31], 30);
	CHECK(f.t - t0 >= 1000000);
	/* host code still runs at once */
	W(&p, 0x607, &one, 1);
	send_code(&p);
	run();
	CHECK_EQ(arena[0x607], 1);
	CHECK_EQ(count(VM_EV_HALT), 0);
	/* host code is not counted */
	setup(0);
	counter_handler(0xB100, 360);
	table(VM_T_USER0, 0xB100);
	vm_start(&vm, 0x40);
	f.row_cost = 110;
	f.panel_on = 1;
	p.n = 0;
	I(&p, DRAW, K(0x10000), K(320), K(0), K(0), K(0), K(0), K(640), K(480));
	I(&p, RAISE, K(12));
	send_code(&p);
	run();
	CHECK_EQ(pool_getw(360), 1);
	vm_status(&vm, s);
	CHECK_EQ(get16(s + 29), 0);
	/* a DRAW's time beyond the cap is charged to the next periods: ~30 % of the time drawing */
	setup(0);
	c.n = 0;
	I(&c, DRAW, K(0x10000), K(320), K(0), K(0), K(0), K(0), K(640), K(480));
	I(&c, RAISE, K(12));
	O(&c, END);
	place(0xB200, &c);
	table(VM_T_START, 0xB200);
	table(VM_T_USER0, 0xB200);
	f.row_cost = 110;
	vm_start(&vm, 0x40);
	run_for(2000000);
	draws = (uint32_t)f.ends;
	CHECK(draws >= 11 && draws <= 13);
	vm_status(&vm, s);
	CHECK(get16(s + 29) >= 5);
}

static void test_host_first(void)
{
	struct code c = {{0}, 0}, p = {{0}, 0};
	uint32_t k;

	setup(0);
	for (int i = 0; i < 100; i++) {
		I(&c, ADD, 1, 1, K(1));
	}
	O(&c, END);
	place(0xB300, &c);
	table(VM_T_START, 0xB300);
	vm_start(&vm, 0x40);
	f.yield_at = (uint32_t)vm.stats.instructions + 10;
	run();
	k = pool_getw(1);
	CHECK(k > 0 && k < 100);
	f.yield_at = 0;
	I(&p, MOV, 360, 1);
	send_code(&p);
	run();
	CHECK_EQ(pool_getw(360), k);
	CHECK_EQ(pool_getw(1), 100);
	/* while a packet sleeps no program instruction runs */
	setw(1, 0);
	vm_start(&vm, 0x40);
	f.yield_at = (uint32_t)vm.stats.instructions + 20;
	run();
	k = pool_getw(1);
	f.yield_at = 0;
	p.n = 0;
	I(&p, SLEEP_UNTIL, K(f.t + 5000));
	I(&p, MOV, 361, 1);
	send_code(&p);
	run();
	CHECK_EQ(pool_getw(1), k);
	run_for(4999);
	CHECK_EQ(pool_getw(1), k);
	run_for(1);
	CHECK_EQ(pool_getw(361), k);
	CHECK_EQ(pool_getw(1), 100);
	/* a host SLEEP_UNTIL already past counts as late */
	p.n = 0;
	I(&p, SLEEP_UNTIL, K(f.t - 1));
	I(&p, SLEEP_UNTIL, K(f.t));
	send_code(&p);
	run();
	CHECK_EQ(vm.stats.late, 2);
	/* SLEEP_UNTIL waits at most 2 s */
	p.n = 0;
	I(&p, SLEEP_UNTIL, K(f.t + 10000000));
	I(&p, MOV, 362, K(5));
	send_code(&p);
	run_for(1999999);
	CHECK_EQ(pool_getw(362), 0);
	run_for(1);
	CHECK_EQ(pool_getw(362), 5);
}

static void test_status(void)
{
	struct code c = {{0}, 0}, d = {{0}, 0};
	uint8_t s[32], want[32];
	uint32_t loop;

	setup(0);
	vm_status(&vm, s);
	memset(want, 0, 32);
	want[1] = 0xFF;
	CHECK_EQ(memcmp(s, want, 32), 0);
	J(&c, CALL, 0xB410);
	O(&c, END);
	place(0xB400, &c);
	I(&d, SYS_READ, 2, K(VM_P_TIME_US));
	I(&d, ADD, 2, 2, K(5000));
	I(&d, SLEEP_UNTIL, 2);
	O(&d, RET);
	place(0xB410, &d);
	table(VM_T_START, 0xB400);
	vm_start(&vm, 0x40);
	run();
	vm_status(&vm, s);
	CHECK_EQ(s[0], VM_RUNNING);
	CHECK_EQ(s[1], 0);
	CHECK_EQ(get32(s + 4), 0x40);
	CHECK_EQ(get32(s + 8), 0xB410 + d.n - 1);
	CHECK_EQ(s[12], VM_TASK_SLEEP);
	CHECK_EQ(s[13], 255);
	CHECK_EQ(s[14], 1);
	CHECK_EQ(get32(s + 20), 1);
	run_for(5000);
	vm_status(&vm, s);
	CHECK_EQ(s[12], VM_TASK_NONE);
	CHECK_EQ(s[1], 0xFF);
	CHECK_EQ(get32(s + 8), 0);
	/* jumps left below 255 */
	c.n = 0;
	I(&c, MOV, 1, K(0));
	loop = 0xB500 + c.n;
	I(&c, ADD, 1, 1, K(1));
	B(&c, BNE, loop, 1, K(901));
	I(&c, WAIT_EVENT, 3, K(1 << 15));
	O(&c, END);
	place(0xB500, &c);
	table(VM_T_START, 0xB500);
	vm_start(&vm, 0x40);
	run();
	vm_status(&vm, s);
	CHECK_EQ(s[12], VM_TASK_WAIT);
	CHECK_EQ(s[13], 100);
	CHECK_EQ(s[14], 0);
}

static void test_props(void)
{
	static const uint32_t unknown[] = {0x06, 0x0F, 0x17, 0x1E, 0x1F, 0x2B, 0x2F, 0x3D, 0xFF, 0x100, 0xFFFFFFFF};
	static const uint32_t readonly[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x10, 0x11, 0x12, 0x13, 0x14,
					    0x15, 0x16, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x2A, 0x30, 0x31, 0x32,
					    0x34, 0x35, 0x36, 0x37, 0x3B, 0x3C};
	/* the hardware of later milestones: readable (UNKNOWN without a driver), not writable yet */
	static const uint32_t later[] = {0x33, 0x38, 0x39};
	static const uint32_t rates[] = {0, 13, 26, 52, 104};
	struct code c = {{0}, 0};
	uint32_t v;

	setup(0);
	f.props[VM_P_ROLL] = (uint32_t)-7;
	f.props[VM_P_BATT_TEMP] = 251;
	f.props[VM_P_CASE_BATT_PCT] = 64;
	I(&c, SYS_READ, 370, K(VM_P_TIME_US));
	I(&c, SYS_READ, 371, K(VM_P_TRIGGER));
	I(&c, SYS_READ, 372, K(VM_P_ARM));
	I(&c, SYS_READ, 373, K(VM_P_LINK_INTERVAL));
	I(&c, SYS_READ, 374, K(VM_P_ROLL));
	I(&c, SYS_READ, 375, K(VM_P_BATT_TEMP));
	I(&c, SYS_READ, 376, K(VM_P_ACC_X));
	I(&c, SYS_READ, 377, K(VM_P_BRIGHTNESS));
	I(&c, SYS_READ, 378, K(VM_P_PITCH_LO));
	I(&c, SYS_READ, 379, K(VM_P_LIGHT_HI));
	I(&c, SYS_READ, 380, K(VM_P_IMU_RATE));
	I(&c, SYS_READ, 381, K(VM_P_PANEL));
	I(&c, SYS_READ, 387, K(VM_P_CASE_BATT_PCT));
	I(&c, SYS_READ, 388, K(VM_P_CASE_LID));
	send_code(&c);
	run();
	CHECK_EQ(pool_getw(387), 64);
	CHECK_EQ(pool_getw(388), VM_UNKNOWN);
	CHECK_EQ(pool_getw(370), f.t);
	CHECK_EQ(pool_getw(371), 0xFF);
	CHECK_EQ(pool_getw(372), 0);
	CHECK_EQ(pool_getw(373), 7500);
	CHECK_EQ(pool_getw(374), (uint32_t)-7);
	CHECK_EQ(pool_getw(375), 251);
	CHECK_EQ(pool_getw(376), VM_UNKNOWN);
	CHECK_EQ(pool_getw(377), 21);
	CHECK_EQ(pool_getw(378), (uint32_t)-90);
	CHECK_EQ(pool_getw(379), 0xFFFFFFFF);
	CHECK_EQ(pool_getw(380), 13);
	CHECK_EQ(pool_getw(381), 0);
	CHECK_EQ(f.nev, 0);
	for (unsigned i = 0; i < sizeof(unknown) / 4; i++) {
		c.n = 0;
		I(&c, SYS_READ, 382, K(unknown[i]));
		send_code(&c);
		run();
		EXPECT_ERROR(5, SYS_READ, 0, unknown[i], 0);
		CHECK_EQ(vm_prop(&vm, unknown[i], &v), VM_E_ARG);
		c.n = 0;
		I(&c, SYS_WRITE, K(unknown[i]), K(1));
		send_code(&c);
		run();
		EXPECT_ERROR(5, SYS_WRITE, 0, unknown[i], 0);
	}
	for (unsigned i = 0; i < sizeof(readonly) / 4; i++) {
		c.n = 0;
		I(&c, SYS_WRITE, K(readonly[i]), K(1));
		send_code(&c);
		run();
		EXPECT_ERROR(5, SYS_WRITE, 0, readonly[i], 0);
		CHECK_EQ(vm_prop(&vm, readonly[i], &v), 0);
	}
	for (unsigned i = 0; i < sizeof(later) / 4; i++) {
		c.n = 0;
		I(&c, SYS_READ, 389, K(later[i]));
		I(&c, SYS_WRITE, K(later[i]), K(0));
		send_code(&c);
		run();
		CHECK_EQ(pool_getw(389), VM_UNKNOWN);
		EXPECT_ERROR(5, SYS_WRITE, 6, later[i], 0);
	}
	c.n = 0;
	I(&c, SYS_READ, 390, K(VM_P_PEER_SEND));
	send_code(&c);
	run();
	EXPECT_ERROR(5, SYS_READ, 0, VM_P_PEER_SEND, 0);
	/* the platform's writable properties: the core checks the spec's values, the platform the rest */
	{
		static const uint32_t ok[][2] = {{VM_P_GYRO_RATE, 52}, {VM_P_GYRO_RATE, 0}, {VM_P_AUTO_BRIGHTNESS, 1},
						 {VM_P_PEER_LINK, 1}, {VM_P_PEER_SEND, 7}, {VM_P_BRIGHTNESS_MAX, 42},
						 {VM_P_BRIGHTNESS_MAX, 0}};
		static const uint32_t bad[][2] = {{VM_P_GYRO_RATE, 50}, {VM_P_AUTO_BRIGHTNESS, 2},
						  {VM_P_BRIGHTNESS_MAX, 43}};

		f.pw_err = 0;
		for (unsigned i = 0; i < 7; i++) {
			c.n = 0;
			I(&c, SYS_WRITE, K(ok[i][0]), K(ok[i][1]));
			send_code(&c);
			run();
			CHECK_EQ(f.pw_id, ok[i][0]);
			CHECK_EQ(f.pw_value, ok[i][1]);
		}
		for (unsigned i = 0; i < 3; i++) {
			uint32_t calls = f.pw_calls;

			c.n = 0;
			I(&c, SYS_WRITE, K(bad[i][0]), K(bad[i][1]));
			send_code(&c);
			run();
			EXPECT_ERROR(5, SYS_WRITE, 0, bad[i][1], 0);
			CHECK_EQ(f.pw_calls, calls);
		}
		f.pw_err = VM_E_ARG;
	}
	c.n = 0;
	I(&c, SYS_WRITE, K(VM_P_BRIGHTNESS), K(42));
	I(&c, SYS_WRITE, K(VM_P_BRIGHTNESS), K(43));
	send_code(&c);
	run();
	EXPECT_ERROR(5, SYS_WRITE, 6, 43, 0);
	CHECK_EQ(f.brightness, 42);
	c.n = 0;
	I(&c, SYS_WRITE, K(VM_P_PANEL), K(1));
	send_code(&c);
	run();
	CHECK_EQ(f.panel_on, 1);
	c.n = 0;
	I(&c, SYS_WRITE, K(VM_P_PANEL), K(2));
	send_code(&c);
	run();
	CHECK_EQ(f.panel_on, 0);
	CHECK_EQ(f.standby, 1);
	c.n = 0;
	I(&c, SYS_WRITE, K(VM_P_PANEL), K(3));
	send_code(&c);
	run();
	EXPECT_ERROR(5, SYS_WRITE, 0, 3, 0);
	CHECK_EQ(f.standby, 1);
	c.n = 0;
	I(&c, SYS_WRITE, K(VM_P_PANEL), K(1));
	send_code(&c);
	run();
	CHECK_EQ(f.panel_on, 1);
	c.n = 0;
	I(&c, SYS_WRITE, K(VM_P_PANEL), K(0));
	send_code(&c);
	run();
	CHECK_EQ(f.panel_on, 0);
	for (unsigned i = 0; i < 5; i++) {
		c.n = 0;
		I(&c, SYS_WRITE, K(VM_P_IMU_RATE), K(rates[i]));
		send_code(&c);
		run();
		CHECK_EQ(f.imu_rate, rates[i]);
	}
	c.n = 0;
	I(&c, SYS_WRITE, K(VM_P_IMU_RATE), K(12));
	send_code(&c);
	run();
	EXPECT_ERROR(5, SYS_WRITE, 0, 12, 0);
	CHECK_EQ(f.imu_rate, 104);
	c.n = 0;
	I(&c, SYS_WRITE, K(VM_P_MOTION_MG), K(123));
	I(&c, SYS_WRITE, K(VM_P_PITCH_HI), K(45));
	I(&c, SYS_WRITE, K(VM_P_LIGHT_LO), K(7));
	I(&c, SYS_WRITE, K(VM_P_LIGHT_HI), K(8));
	I(&c, SYS_READ, 383, K(VM_P_MOTION_MG));
	I(&c, SYS_READ, 384, K(VM_P_PITCH_HI));
	I(&c, SYS_READ, 385, K(VM_P_LIGHT_LO));
	I(&c, SYS_READ, 386, K(VM_P_LIGHT_HI));
	send_code(&c);
	run();
	CHECK_EQ(pool_getw(383), 123);
	CHECK_EQ(pool_getw(384), 45);
	CHECK_EQ(pool_getw(385), 7);
	CHECK_EQ(pool_getw(386), 8);
	CHECK(vm_prop(&vm, VM_P_TRIGGER, &v) == 0 && v == 0xFF);
	CHECK(vm_prop(&vm, VM_P_BATT_MV, &v) == 0 && v == 3900);
}

/* random parameters: mostly ordinary, sometimes extreme */
static int32_t rcoord(void)
{
	switch (rndn(10)) {
	case 0:
		return (int32_t)rnd();
	case 1:
		return -(int32_t)rndn(300);
	default:
		return (int32_t)rndn(700) - 100;
	}
}

static uint32_t rlen(void)
{
	switch (rndn(10)) {
	case 0:
		return rnd();
	case 1:
		return 0;
	default:
		return rndn(400);
	}
}

static uint32_t rstride(void)
{
	switch (rndn(12)) {
	case 0:
		return rnd();
	case 1:
		return rndn(4);
	default:
		return rndn(500);
	}
}

static uint32_t rbase(void)
{
	switch (rndn(12)) {
	case 0:
		return VM_ARENA_SIZE + rndn(100);
	case 1:
		return VM_ARENA_SIZE - rndn(64);
	default:
		return WIN0 + rndn(WIN);
	}
}

static void randomize_window(void)
{
	for (uint32_t i = WIN0; i < VM_ARENA_SIZE; i++) {
		arena[i] = (uint8_t)rnd();
	}
}

static int same_window(const uint8_t *a, const uint8_t *b)
{
	return memcmp(a + WIN0, b + WIN0, WIN) == 0;
}

static void copy_window(uint8_t *to, const uint8_t *from)
{
	memcpy(to + WIN0, from + WIN0, WIN);
}

/* FILL, BLIT, GLYPH_RUN and WRITE_RLE against the protocol 4 code wherever protocol 4 accepts the
 * operands; clipped cases against the per-pixel reading. Everything happens in the arena's last
 * 64 KiB, where all destinations lie. */
static void test_gfx_v4(void)
{
	uint8_t *a4 = v4_arena(), b[64], font[16384], text[256];
	int n4[4] = {0, 0, 0, 0}, n_clip = 0;

	setup(0);
	randomize_window();
	copy_window(a4, arena);
	for (int i = 0; i < 4000; i++) {
		uint32_t aw = (WIN0 + rndn(WIN)) / 4, stride = rndn(8) ? rndn(300) : rndn(5000);
		uint32_t x = rndn(8) ? rndn(400) : rndn(65536), y = rndn(8) ? rndn(300) : rndn(65536);
		uint32_t w = rndn(8) ? rndn(300) : rndn(65536), h = rndn(8) ? rndn(200) : rndn(65536);
		uint32_t level = rndn(17), v[7], detail = 0;
		int e4, e5, er;

		if (rndn(3)) {
			stride = 1 + rndn(200);
			x = rndn(2 * stride);
			w = rndn(2 * stride - x + 1);
			y = rndn(60);
			h = rndn(60);
			aw = (WIN0 + rndn(WIN - (y + h + 1) * stride)) / 4;
		}
		v[0] = 4 * aw, v[1] = stride, v[2] = x, v[3] = y, v[4] = w, v[5] = h, v[6] = level;
		put16(b, aw);
		put16(b + 2, stride);
		put16(b + 4, x);
		put16(b + 6, y);
		put16(b + 8, w);
		put16(b + 10, h);
		b[12] = (uint8_t)level;
		copy_window(ref, arena);
		/* v4's FILL with w = 0 walks its rows anyway and forms pointers past the arena (UB, no write) */
		e4 = w == 0 ? (level > 15 ? 5 : 0) : v4_fill(b);
		e5 = vm_fill(arena, v, &detail);
		er = ref_fill(ref, v);
		CHECK_EQ(e5, er);
		CHECK(same_window(arena, ref));
		if (e4 == 0) {
			CHECK(same_window(arena, a4));
			n4[0]++;
		} else if (e4 == 3) {
			n_clip++;
		} else {
			CHECK_EQ(e5, e4);
		}
		copy_window(a4, arena);
	}
	for (int i = 0; i < 4000; i++) {
		int compressed = (int)rndn(2), same = rndn(3) == 0;
		uint32_t mode = rndn(5), level = compressed ? rndn(17) : 15;
		uint32_t saw = (WIN0 + rndn(WIN)) / 4, daw = same ? saw : (WIN0 + rndn(WIN)) / 4;
		uint32_t ss = rndn(8) ? 1 + rndn(200) : rndn(3000), ds = same ? ss : 1 + rndn(200);
		uint32_t sx = rndn(300), sy = rndn(200), dx = rndn(300), dy = rndn(200);
		uint32_t w = rndn(8) ? rndn(150) : rndn(2000), h = rndn(8) ? rndn(100) : rndn(2000);
		uint32_t v[12] = {4 * saw, ss, sx, sy, 4 * daw, ds, dx, dy, w, h, mode > 3 ? 8 : mode | (compressed ? 4u : 0u), level};
		uint32_t detail = 0;
		int e4, e5, er;

		if (rndn(3)) {
			uint32_t sw, dw;

			ss = 1 + rndn(150);
			ds = same ? ss : 1 + rndn(150);
			sw = compressed ? 8 * ss : 2 * ss;
			dw = 2 * ds;
			w = rndn((sw < dw ? sw : dw) + 1);
			sx = rndn(sw - w + 1);
			dx = rndn(dw - w + 1);
			h = rndn(50);
			sy = rndn(50);
			dy = rndn(50);
			if (same && rndn(2)) {
				uint32_t ox = rndn(5), oy = rndn(5);

				dx = sx + ox >= 2 && sx + ox - 2 <= dw - w ? sx + ox - 2 : sx;
				dy = sy + oy >= 2 ? sy + oy - 2 : sy;
			}
			saw = (WIN0 + rndn(WIN - (sy + h + 1) * ss)) / 4;
			daw = same ? saw : (WIN0 + rndn(WIN - (dy + h + 1) * ds)) / 4;
		} else if (same && rndn(2)) {
			sx = (dx + rndn(5) - 2) & 0xFFFF;
			sy = (dy + rndn(5) - 2) & 0xFFFF;
		}
		v[0] = 4 * saw, v[1] = ss, v[2] = sx, v[3] = sy, v[4] = 4 * daw, v[5] = ds, v[6] = dx, v[7] = dy;
		v[8] = w, v[9] = h;
		put16(b, saw);
		put16(b + 2, ss);
		put16(b + 4, sx);
		put16(b + 6, sy);
		put16(b + 8, daw);
		put16(b + 10, ds);
		put16(b + 12, dx);
		put16(b + 14, dy);
		put16(b + 16, w);
		put16(b + 18, h);
		b[20] = (uint8_t)mode;
		b[21] = (uint8_t)level;
		copy_window(ref, arena);
		e4 = v4_blit(b, compressed);
		e5 = vm_blit(arena, v, &detail);
		er = ref_blit(ref, v);
		CHECK_EQ(e5, er);
		CHECK(same_window(arena, ref));
		if (e4 == 0) {
			CHECK(same_window(arena, a4));
			n4[1]++;
		} else if (e4 != 3) {
			CHECK_EQ(e5, e4);
		}
		copy_window(a4, arena);
	}
	for (int i = 0; i < 3000; i++) {
		uint32_t ng = 1 + rndn(300), size = make_font(font, ng, rndn(3) ? 0 : rndn(40), -1);
		uint32_t n = rndn(40), t = 0, faw = 0x2000 / 4, daw = (WIN0 + rndn(WIN)) / 4, ds = rndn(8) ? rndn(300) : rndn(3000);
		int32_t x = (int32_t)rndn(700) - 100, y = (int32_t)rndn(300) - 20;
		uint32_t mode = rndn(5), level = rndn(17), v[9], detail = 0;
		int e4, e5, er;

		if (rndn(20) == 0) {
			font[rndn(16)] ^= (uint8_t)(1 + rndn(255));
		}
		memcpy(arena + 0x2000, font, size);
		memcpy(a4 + 0x2000, font, size);
		while (t < n) {
			uint32_t g = rndn(30) ? rndn(ng) : rndn(ng + 20);

			if (rndn(50) == 0) {
				text[t++] = (uint8_t)(0x80 | rnd());
			} else if (g < 128 && rndn(4)) {
				text[t++] = (uint8_t)g;
			} else {
				text[t++] = (uint8_t)(0x80 | (g & 127));
				if (t < sizeof(text)) {
					text[t++] = (uint8_t)(g >> 7);
				}
			}
		}
		n = t;
		memcpy(arena + 0x1000, text, n);
		put16(b, faw);
		put16(b + 2, daw);
		put16(b + 4, ds);
		put16(b + 6, (uint32_t)x);
		put16(b + 8, (uint32_t)y);
		b[10] = (uint8_t)mode;
		b[11] = (uint8_t)level;
		b[12] = (uint8_t)n;
		uint8_t body[300];
		memcpy(body, b, 13);
		memcpy(body + 13, text, n);
		v[0] = 0x2000, v[1] = 4 * daw, v[2] = ds, v[3] = (uint32_t)x, v[4] = (uint32_t)y;
		v[5] = mode, v[6] = level, v[7] = 0x1000, v[8] = n;
		copy_window(ref, arena);
		memcpy(ref + 0x1000, text, n);
		memcpy(ref + 0x2000, font, size);
		e4 = v4_glyph_run(body);
		e5 = vm_glyphs(arena, v, &detail);
		er = ref_glyphs(ref, v);
		CHECK_EQ(e5, er);
		CHECK_EQ(e5, e4 == 9 ? 8 : e4);
		CHECK(same_window(arena, ref));
		CHECK(same_window(arena, a4));
		n4[2] += e4 == 0;
		copy_window(a4, arena);
	}
	for (int i = 0; i < 2000; i++) {
		uint8_t raw[300], pk[300], cmd[300];
		uint32_t len = rndn(250), n, aw = (WIN0 + rndn(WIN)) / 4, e4;
		struct event *e;

		for (uint32_t j = 0; j < len; j++) {
			raw[j] = rndn(3) ? (uint8_t)(j / 7) : (uint8_t)rnd();
		}
		n = packbits(raw, len, pk);
		if (n > 200) {
			continue;
		}
		if (rndn(5) == 0 && n) {
			pk[rndn(n)] = (uint8_t)rnd();
		}
		if (rndn(10) == 0) {
			aw = 0xFFFF - rndn(30);
		}
		put16(cmd, aw);
		put16(cmd + 2, len);
		cmd[4] = (uint8_t)n;
		memcpy(cmd + 5, pk, n);
		e4 = (uint32_t)v4_write_rle(cmd);
		cmd[0] = WRITE_RLE;
		put32(cmd + 1, 4 * aw);
		put16(cmd + 4, len);
		cmd[6] = (uint8_t)n;
		memcpy(cmd + 7, pk, n);
		f.nev = 0;
		send(cmd, 7 + n);
		run();
		e = last(VM_EV_ERROR);
		CHECK_EQ(e ? e->body[4] : 0, e4);
		n4[3] += e4 == 0;
		CHECK(same_window(arena, a4));
		copy_window(a4, arena);
	}
	printf("v4-exact cases: FILL %d, BLIT %d, GLYPH_RUN %d, WRITE_RLE %d; clipped FILL %d\n", n4[0], n4[1], n4[2], n4[3], n_clip);
	CHECK(n4[0] > 500 && n4[1] > 500 && n4[2] > 1500 && n4[3] > 500 && n_clip > 500);
}

/* the same primitives with protocol 5 operands (byte addresses, negative and extreme values) */
static void test_gfx_clip(void)
{
	uint8_t font[16384], text[64];

	setup(0);
	randomize_window();
	for (int i = 0; i < 3000; i++) {
		uint32_t v[7] = {rbase(), rstride(), (uint32_t)rcoord(), (uint32_t)rcoord(), rlen(), rlen(), rndn(17)}, d = 0;
		int e5, er;

		copy_window(ref, arena);
		e5 = vm_fill(arena, v, &d);
		er = ref_fill(ref, v);
		CHECK_EQ(e5, er);
		CHECK(same_window(arena, ref));
		if (e5 == 5) {
			CHECK_EQ(d, v[6]);
		}
	}
	for (int i = 0; i < 3000; i++) {
		uint32_t v[12], d = 0;
		int e5, er;

		v[0] = rndn(10) ? WIN0 + rndn(WIN) : rbase();
		v[1] = rstride();
		v[2] = (uint32_t)rcoord();
		v[3] = (uint32_t)rcoord();
		v[4] = rndn(3) ? v[0] + rndn(64) - 32 : rbase();
		v[5] = rndn(2) ? v[1] : rstride();
		v[6] = rndn(3) ? v[2] + rndn(9) - 4 : (uint32_t)rcoord();
		v[7] = rndn(3) ? v[3] + rndn(9) - 4 : (uint32_t)rcoord();
		v[8] = rndn(10) ? rndn(200) : rlen();
		v[9] = rndn(10) ? rndn(120) : rlen();
		v[10] = rndn(12);
		v[11] = rndn(17);
		copy_window(ref, arena);
		e5 = vm_blit(arena, v, &d);
		er = ref_blit(ref, v);
		CHECK_EQ(e5, er);
		CHECK(same_window(arena, ref));
		if (e5 == 3) {
			CHECK_EQ(d, v[0]);
		}
	}
	for (int i = 0; i < 2000; i++) {
		uint32_t ng = 1 + rndn(200), size = make_font(font, ng, rndn(30), 0), n = rndn(30), v[9], d = 0;
		int e5, er;

		for (uint32_t j = 0; j < n; j++) {
			text[j] = (uint8_t)rndn(ng < 128 ? ng : 128);
		}
		memcpy(arena + 0x2000, font, size);
		memcpy(arena + 0x1000, text, n);
		memcpy(ref + 0x2000, font, size);
		memcpy(ref + 0x1000, text, n);
		v[0] = 0x2000;
		v[1] = rbase();
		v[2] = rstride();
		v[3] = (uint32_t)rcoord();
		v[4] = (uint32_t)rcoord();
		v[5] = rndn(5);
		v[6] = rndn(17);
		v[7] = rndn(20) ? 0x1000 : VM_ARENA_SIZE - rndn(10);
		v[8] = n;
		copy_window(ref, arena);
		e5 = vm_glyphs(arena, v, &d);
		er = ref_glyphs(ref, v);
		CHECK_EQ(e5, er);
		CHECK(same_window(arena, ref));
	}
}

static void test_line(void)
{
	uint32_t v[7], d = 0;
	int64_t px, py;

	setup(0);
	for (int i = 0; i < 3000; i++) {
		int64_t x0 = (int32_t)rndn(900) - 300, y0 = (int32_t)rndn(900) - 300, x1, y1, n, dx, dy;

		x1 = rndn(4) ? x0 + (int32_t)rndn(81) - 40 : (int32_t)rndn(900) - 300;
		y1 = rndn(4) ? y0 + (int32_t)rndn(81) - 40 : (int32_t)rndn(900) - 300;
		memset(arena + WIN0, 0, WIN);
		memset(ref + WIN0, 0, WIN);
		v[0] = WIN0 + rndn(64);
		v[1] = rndn(10) ? 50 + rndn(150) : rstride();
		v[2] = (uint32_t)x0, v[3] = (uint32_t)y0, v[4] = (uint32_t)x1, v[5] = (uint32_t)y1;
		v[6] = 1 + rndn(15);
		if (rndn(2)) {
			uint32_t t = v[2];

			v[2] = v[4], v[4] = t, t = v[3], v[3] = v[5], v[5] = t;
		}
		CHECK_EQ(vm_line(arena, v, &d), 0);
		dx = x1 > x0 ? x1 - x0 : x0 - x1;
		dy = y1 > y0 ? y1 - y0 : y0 - y1;
		n = dx > dy ? dx : dy;
		for (int64_t j = 0; j <= n; j++) {
			line_pixel(x0, y0, x1, y1, j, &px, &py);
			if (ex(v[0], v[1], px, py, 4)) {
				wr4(ref, v[0], v[1], px, py, v[6]);
			}
		}
		CHECK(same_window(arena, ref));
	}
	/* a short diagonal-ish line, pixel by pixel */
	memset(arena + WIN0, 0, WIN);
	v[0] = WIN0, v[1] = 8, v[2] = 0, v[3] = 0, v[4] = 4, v[5] = 1, v[6] = 15;
	vm_line(arena, v, &d);
	CHECK_EQ(arena[WIN0], 0xFF);
	CHECK_EQ(arena[WIN0 + 1], 0xF0);
	CHECK_EQ(arena[WIN0 + 8 + 1], 0x0F);
	CHECK_EQ(arena[WIN0 + 8 + 2], 0xF0);
	v[2] = 32768;
	CHECK_EQ(vm_line(arena, v, &d), VM_E_ARG);
	CHECK_EQ(d, 32768);
	v[2] = (uint32_t)-32769;
	CHECK_EQ(vm_line(arena, v, &d), VM_E_ARG);
	v[2] = (uint32_t)-32768, v[4] = 32767, v[6] = 16;
	CHECK_EQ(vm_line(arena, v, &d), VM_E_ARG);
	CHECK_EQ(d, 16);
	v[6] = 3;
	CHECK_EQ(vm_line(arena, v, &d), 0);
	v[0] = VM_ARENA_SIZE;
	CHECK_EQ(vm_line(arena, v, &d), 0);
}

static void test_draw(void)
{
	struct code c = {{0}, 0};
	struct event *e;

	setup(0);
	randomize_window();
	vm_set_mic(&vm, 0);
	f.panel_on = 1;
	for (int i = 0; i < 600; i++) {
		uint32_t v[8];
		int er, begins = f.begins;

		v[0] = rndn(10) ? WIN0 + rndn(WIN) : rbase();
		v[1] = rndn(10) ? rndn(400) : rstride();
		v[2] = rndn(10) ? rndn(300) : (uint32_t)rcoord();
		v[3] = rndn(10) ? rndn(100) : (uint32_t)rcoord();
		v[4] = (uint32_t)((int32_t)rndn(1400) - 400);
		v[5] = (uint32_t)((int32_t)rndn(1000) - 300);
		v[6] = rndn(10) ? rndn(700) : rlen();
		v[7] = rndn(10) ? rndn(500) : rlen();
		memcpy(ref_panel, f.panel, sizeof(ref_panel));
		er = ref_draw(arena, v, ref_panel);
		npool = 0;
		c.n = 0;
		I(&c, DRAW, K(v[0]), K(v[1]), K(v[2]), K(v[3]), K(v[4]), K(v[5]), K(v[6]), K(v[7]));
		f.nev = 0;
		send_code(&c);
		run();
		e = last(VM_EV_ERROR);
		CHECK_EQ(e ? e->body[4] : 0, er);
		if (er == 3 && e) {
			CHECK_EQ(get32(e->body + 5), v[0]);
		}
		CHECK_EQ(memcmp(ref_panel, f.panel, sizeof(ref_panel)), 0);
		if (er == 0 && v[6] && v[7] && (int32_t)v[4] < VM_PANEL_W && (int32_t)v[5] < VM_PANEL_H &&
		    (int64_t)(int32_t)v[4] + v[6] > 0 && (int64_t)(int32_t)v[5] + v[7] > 0) {
			CHECK_EQ(f.begins, begins + 1);
			CHECK_EQ(f.bx, (int32_t)v[4] > 0 ? (int32_t)v[4] : 0);
			CHECK_EQ(f.by, (int32_t)v[5] > 0 ? (int32_t)v[5] : 0);
		} else {
			CHECK_EQ(f.begins, begins);
		}
	}
	/* the first DRAW powers the panel on; without a panel a DRAW does nothing */
	setup(0);
	CHECK_EQ(f.panel_on, 0);
	c.n = 0;
	I(&c, DRAW, K(0x1000), K(4), K(0), K(0), K(10), K(10), K(8), K(2));
	send_code(&c);
	run();
	CHECK_EQ(f.panel_on, 1);
	CHECK_EQ(vm.stats.draws, 1);
	f.has_panel = 0;
	send_code(&c);
	run();
	CHECK_EQ(f.nev, 0);
	CHECK_EQ(vm.stats.draws, 1);
	/* safe mode: DRAW is error 7, VM_START is refused, the rest works */
	setup(1);
	c.n = 0;
	I(&c, FILL, K(0x1000), K(4), K(0), K(0), K(8), K(2), K(5));
	I(&c, DRAW, K(0x1000), K(4), K(0), K(0), K(10), K(10), K(8), K(2));
	send_code(&c);
	run();
	EXPECT_ERROR(7, DRAW, 19, 0, 0);
	CHECK_EQ(arena[0x1000], 0x55);
	CHECK_EQ(vm_start(&vm, 0x40), VM_E_SAFE);
}

static void test_draw_instructions(void)
{
	struct code c = {{0}, 0};
	uint32_t vb[12] = {0x3000, 16, 1, 2, 0x5000, 20, 3, 1, 9, 7, 6, 11}, vl[7] = {0x5000, 20, -3, 2, 30, 9, 13}, d;

	setup(0);
	for (uint32_t i = 0x3000; i < 0x6000; i++) {
		arena[i] = (uint8_t)rnd();
	}
	memcpy(ref, arena, VM_ARENA_SIZE);
	CHECK_EQ(vm_blit(ref, vb, &d), 0);
	CHECK_EQ(vm_line(ref, vl, &d), 0);
	I(&c, BLIT, K(vb[0]), K(vb[1]), K(vb[2]), K(vb[3]), K(vb[4]), K(vb[5]), K(vb[6]), K(vb[7]), K(vb[8]), K(vb[9]), K(vb[10]), K(vb[11]));
	I(&c, LINE, K(vl[0]), K(vl[1]), K(vl[2]), K(vl[3]), K(vl[4]), K(vl[5]), K(vl[6]));
	send_code(&c);
	run();
	CHECK_EQ(f.nev, 0);
	CHECK_EQ(memcmp(ref + 0x3000, arena + 0x3000, 0x3000), 0);
	c.n = 0;
	I(&c, BLIT, K(vb[0]), K(vb[1]), K(vb[2]), K(vb[3]), K(vb[4]), K(vb[5]), K(vb[6]), K(vb[7]), K(vb[8]), K(vb[9]), K(8), K(vb[11]));
	I(&c, BLIT, K(vb[0]), K(vb[1]), K(vb[2]), K(vb[3]), K(vb[4]), K(vb[5]), K(vb[6]), K(vb[7]), K(vb[8]), K(vb[9]), K(0), K(16));
	send_code(&c);
	run();
	EXPECT_ERROR(5, BLIT, 0, 8, 0);
	c.n = 0;
	I(&c, BLIT, K(vb[0]), K(vb[1]), K(vb[2]), K(vb[3]), K(vb[4]), K(vb[5]), K(vb[6]), K(vb[7]), K(vb[8]), K(vb[9]), K(0), K(16));
	send_code(&c);
	run();
	EXPECT_ERROR(5, BLIT, 0, 16, 0);
	c.n = 0;
	I(&c, BLIT, K(vb[0]), K(vb[1]), K(-1), K(vb[3]), K(vb[4]), K(vb[5]), K(vb[6]), K(vb[7]), K(vb[8]), K(vb[9]), K(0), K(15));
	send_code(&c);
	run();
	EXPECT_ERROR(3, BLIT, 0, vb[0], 0);
	c.n = 0;
	I(&c, FILL, K(0x5000), K(20), K(0), K(0), K(8), K(2), K(16));
	send_code(&c);
	run();
	EXPECT_ERROR(5, FILL, 0, 16, 0);
	c.n = 0;
	I(&c, GLYPHS, K(0x9000), K(0x5000), K(20), K(0), K(0), K(2), K(15), K(0x1000), K(1));
	send_code(&c);
	run();
	EXPECT_ERROR(8, GLYPHS, 0, 0, 0);
	c.n = 0;
	I(&c, GLYPHS, K(0x9000), K(0x5000), K(20), K(0), K(0), K(4), K(15), K(0x1000), K(1));
	send_code(&c);
	run();
	EXPECT_ERROR(5, GLYPHS, 0, 4, 0);
	c.n = 0;
	I(&c, GLYPHS, K(0x9000), K(0x5000), K(20), K(0), K(0), K(2), K(15), K(VM_ARENA_SIZE - 1), K(2));
	send_code(&c);
	run();
	EXPECT_ERROR(3, GLYPHS, 0, VM_ARENA_SIZE - 1, 0);
}

/* VIDEO records from the library's encoder (scouter_kit's fps-codec eval/golden.c): a 37 x 23 disc that moves
 * 4 pixels right; the CRC-32 of the frames' rows: 0xffc20531, 0xe1b8dc7f */
static const uint8_t video_rec0[] = {0x01, 0x46, 0xb9, 0x9e, 0x06, 0x91, 0x9e, 0x71, 0x81, 0x07, 0xc9, 0x42, 0x8c, 0x4e,
				     0xc1, 0x58, 0xad, 0x55, 0xd2, 0x4f, 0x70, 0xaa, 0x42, 0x6f, 0x9a, 0x81, 0xd4, 0xc2};
static const uint8_t video_rec1[] = {0x00, 0x44, 0x42, 0xd0, 0xce, 0x1c, 0xde, 0x76, 0xbd, 0xfb, 0xc1, 0x96, 0x02, 0x22,
				     0x57, 0xd8, 0x2b, 0x18, 0x98, 0x6a, 0x72, 0x24, 0x3d, 0x1e, 0x97};

/* VIDEO: the frame as the panel should show it (the bitmap at (x, y), level for ink, clipped) */
static void video_ref(uint32_t bmp, uint32_t stride, uint32_t w, uint32_t h, int32_t x, int32_t y, uint32_t level)
{
	for (uint32_t r = 0; r < h; r++) {
		for (uint32_t c = 0; c < w; c++) {
			int64_t px = (int64_t)x + c, py = (int64_t)y + r;

			if (px >= 0 && px < VM_PANEL_W && py >= 0 && py < VM_PANEL_H) {
				ref_panel[py][px] = (uint8_t)((arena[bmp + r * stride + c / 8] >> (7 - c % 8) & 1) ? level : 0);
			}
		}
	}
}

static void video_send(uint32_t bmp, uint32_t stride, uint32_t w, uint32_t h, int32_t x, int32_t y,
		       uint32_t level, uint32_t src, uint32_t n, uint32_t st)
{
	struct code c = {{0}, 0};

	npool = 0;
	I(&c, VIDEO, K(bmp), K(stride), K(w), K(h), K((uint32_t)x), K((uint32_t)y), K(level), K(src), K(n), K(st));
	send_code(&c);
	run();
}

static uint32_t video_rows_crc(uint32_t bmp, uint32_t stride, uint32_t nb, uint32_t h)
{
	uint32_t crc = 0;

	for (uint32_t r = 0; r < h; r++) {
		crc = crc32z(crc, arena + bmp + r * stride, nb);
	}
	return crc;
}

static void test_video(void)
{
	const uint8_t *rec0 = video_rec0, *rec1 = video_rec1;
	const uint32_t bmp = 0x2000, stride = 7, st = 0x3001, src = 0x4003;
	uint8_t prev[23][5];
	int rows = 0, begins;
	struct code c = {{0}, 0};

	setup(0);
	f.spans = 1;
	for (uint32_t r = 0; r < 23; r++) {
		memset(arena + bmp + r * stride + 5, 0xA5, 2);
	}
	I(&c, SYS_WRITE, K(VM_P_PANEL), K(1));
	send_code(&c);
	run();
	memcpy(ref_panel, f.panel, sizeof(ref_panel));
	memcpy(arena + src, rec0, sizeof(video_rec0));
	video_send(bmp, stride, 37, 23, 100, 50, 9, src, sizeof(video_rec0), st);
	CHECK_EQ(f.nev, 0);
	CHECK_EQ(video_rows_crc(bmp, stride, 5, 23), 0xffc20531u);
	video_ref(bmp, stride, 37, 23, 100, 50, 9);
	CHECK_EQ(memcmp(ref_panel, f.panel, sizeof(ref_panel)), 0);
	CHECK_EQ(f.ends, 1);
	for (uint32_t r = 0; r < 23; r++) {
		memcpy(prev[r], arena + bmp + r * stride, 5);
		CHECK_EQ(arena[bmp + r * stride + 5], 0xA5);
	}
	f.rows = 0;
	memcpy(arena + src, rec1, sizeof(video_rec1));
	video_send(bmp, stride, 37, 23, 100, 50, 9, src, sizeof(video_rec1), st);
	CHECK_EQ(f.nev, 0);
	CHECK_EQ(video_rows_crc(bmp, stride, 5, 23), 0xe1b8dc7fu);
	video_ref(bmp, stride, 37, 23, 100, 50, 9);
	CHECK_EQ(memcmp(ref_panel, f.panel, sizeof(ref_panel)), 0);
	for (uint32_t r = 0; r < 23; r++) {
		rows += memcmp(prev[r], arena + bmp + r * stride, 5) != 0;
	}
	CHECK_EQ(f.rows, rows); /* only the rows that changed */
	CHECK(rows > 0 && rows < 23);
	for (uint32_t i = 0; i < VM_VIDEO_STATE / 2; i++) {
		CHECK(get16(arena + st + 2 * i) < 4096);
	}

	/* nothing visible: the bitmap changes, the panel is not touched */
	memset(arena + bmp, 0, 23 * stride);
	begins = f.begins;
	memcpy(arena + src, rec0, sizeof(video_rec0));
	video_send(bmp, stride, 37, 23, 640, 50, 9, src, sizeof(video_rec0), st);
	CHECK_EQ(f.nev, 0);
	CHECK_EQ(video_rows_crc(bmp, stride, 5, 23), 0xffc20531u);
	CHECK_EQ(f.begins, begins);

	/* errors, checked before anything changes */
	memcpy(ref, arena, VM_ARENA_SIZE);
	video_send(bmp, stride, 37, 23, 0, 0, 16, src, sizeof(video_rec0), st);
	EXPECT_ERROR(5, VIDEO, 0, 16, 0);
	video_send(bmp, stride, 0, 23, 0, 0, 9, src, sizeof(video_rec0), st);
	EXPECT_ERROR(5, VIDEO, 0, 0, 0);
	video_send(bmp, 81, 641, 23, 0, 0, 9, src, sizeof(video_rec0), st);
	EXPECT_ERROR(5, VIDEO, 0, 641, 0);
	video_send(bmp, stride, 37, 481, 0, 0, 9, src, sizeof(video_rec0), st);
	EXPECT_ERROR(5, VIDEO, 0, 481, 0);
	video_send(bmp, 4, 37, 23, 0, 0, 9, src, sizeof(video_rec0), st);
	EXPECT_ERROR(5, VIDEO, 0, 4, 0);
	video_send(VM_ARENA_SIZE - 100, stride, 37, 23, 0, 0, 9, src, sizeof(video_rec0), st);
	EXPECT_ERROR(3, VIDEO, 0, VM_ARENA_SIZE - 100, 0);
	video_send(bmp, stride, 37, 23, 0, 0, 9, VM_ARENA_SIZE - 3, 4, st);
	EXPECT_ERROR(3, VIDEO, 0, VM_ARENA_SIZE - 3, 0);
	video_send(bmp, stride, 37, 23, 0, 0, 9, src, sizeof(video_rec0), VM_ARENA_SIZE - 1000);
	EXPECT_ERROR(3, VIDEO, 0, VM_ARENA_SIZE - 1000, 0);
	video_send(bmp, stride, 37, 23, 0, 0, 9, bmp + 150, 20, st);
	EXPECT_ERROR(3, VIDEO, 0, bmp + 150, 0);
	video_send(bmp, stride, 37, 23, 0, 0, 9, src, sizeof(video_rec0), bmp - 1000);
	EXPECT_ERROR(3, VIDEO, 0, bmp - 1000, 0);
	video_send(bmp, stride, 37, 23, 0, 0, 9, src, sizeof(video_rec0), src - 1595);
	EXPECT_ERROR(3, VIDEO, 0, src - 1595, 0);
	video_send(bmp, stride, 37, 23, 0, 0, 9, src, 0, st);
	EXPECT_ERROR(4, VIDEO, 0, 0, 0);
	arena[src] = 4;
	ref[src] = 4;
	video_send(bmp, stride, 37, 23, 0, 0, 9, src, sizeof(video_rec0), st);
	EXPECT_ERROR(4, VIDEO, 0, 0, 0);
	CHECK_EQ(memcmp(ref + 0x2000, arena + 0x2000, VM_ARENA_SIZE - 0x2000), 0);

	/* any record keeps the panel showing the bitmap; the redraw flag draws every row */
	for (int it = 0; it < 400; it++) {
		uint32_t w = rndn(8) ? 1 + rndn(200) : 1 + rndn(640), h = rndn(8) ? 1 + rndn(60) : 1 + rndn(480);
		uint32_t sb = 0x8000 + rndn(16), sw = (w + 7) / 8 + rndn(3), level = rndn(16);
		int32_t x = (int32_t)rndn(800) - 150, y = (int32_t)rndn(600) - 60;
		uint32_t rs = 0x20000 + rndn(16), ss = 0x30000 + rndn(16);

		for (uint32_t i = 0; i < sw * h; i++) {
			arena[sb + i] = (uint8_t)rnd();
		}
		for (int k = 0; k < 4; k++) {
			uint32_t n = 1 + rndn(rndn(4) ? 64 : 1500);

			for (uint32_t i = 0; i < n; i++) {
				arena[rs + i] = (uint8_t)rnd();
			}
			arena[rs] = (uint8_t)(k == 0 ? 2 | rndn(2) : rndn(4));
			memcpy(ref_panel, f.panel, sizeof(ref_panel));
			f.nev = 0;
			video_send(sb, sw, w, h, x, y, level, rs, n, ss);
			CHECK_EQ(f.nev, 0);
			video_ref(sb, sw, w, h, x, y, level);
			CHECK_EQ(memcmp(ref_panel, f.panel, sizeof(ref_panel)), 0);
		}
	}

	/* the microphone indicator stays over a VIDEO; safe mode */
	vm_set_mic(&vm, 1);
	arena[src] = 3;
	memcpy(ref_panel, f.panel, sizeof(ref_panel));
	video_send(bmp, stride, 37, 23, 600, 0, 9, src, 40, st);
	video_ref(bmp, stride, 37, 23, 600, 0, 9);
	for (int32_t yy = 4; yy < 20; yy++) {
		for (int32_t xx = 620; xx < 636; xx++) {
			ref_panel[yy][xx] = 15;
		}
	}
	CHECK_EQ(memcmp(ref_panel, f.panel, sizeof(ref_panel)), 0);
	setup(1);
	f.spans = 1;
	memcpy(arena + src, rec0, sizeof(video_rec0));
	video_send(bmp, stride, 37, 23, 0, 0, 9, src, sizeof(video_rec0), st);
	EXPECT_ERROR(7, VIDEO, 0, 0, 0);
	CHECK_EQ(arena[bmp], 0);
}

static void test_mic(void)
{
	struct code c = {{0}, 0};
	int ok = 1;

	setup(0);
	vm_set_mic(&vm, 1);
	CHECK_EQ(f.panel_on, 1);
	for (int y = 0; y < VM_PANEL_H; y++) {
		for (int x = 0; x < VM_PANEL_W; x++) {
			ok &= f.panel[y][x] == (x >= 620 && x < 636 && y >= 4 && y < 20 ? 15 : 0);
		}
	}
	CHECK(ok);
	/* a DRAW covering the indicator keeps it; one covering part of it keeps that part */
	memset(arena + 0x10000, 0x33, 320 * 480);
	I(&c, DRAW, K(0x10000), K(320), K(0), K(0), K(0), K(0), K(640), K(480));
	send_code(&c);
	run();
	ok = 1;
	for (int y = 0; y < VM_PANEL_H; y++) {
		for (int x = 0; x < VM_PANEL_W; x++) {
			ok &= f.panel[y][x] == (x >= 620 && x < 636 && y >= 4 && y < 20 ? 15 : 3);
		}
	}
	CHECK(ok);
	memset(arena + 0x10000, 0x55, 320 * 480);
	c.n = 0;
	I(&c, DRAW, K(0x10000), K(320), K(1), K(0), K(630), K(15), K(10), K(10));
	send_code(&c);
	run();
	CHECK_EQ(f.panel[15][619], 3);
	CHECK_EQ(f.panel[15][629], 15);
	CHECK_EQ(f.panel[15][630], 15);
	CHECK_EQ(f.panel[19][635], 15);
	CHECK_EQ(f.panel[19][636], 5);
	CHECK_EQ(f.panel[20][630], 5);
	vm_set_mic(&vm, 0);
	CHECK_EQ(f.panel[4][620], 0);
	CHECK_EQ(f.panel[19][635], 0);
	CHECK_EQ(f.panel[20][630], 5);
	/* the left arm's indicator is at (4, 4) */
	f.props[VM_P_ARM] = 1;
	vm_set_mic(&vm, 1);
	CHECK_EQ(f.panel[4][4], 15);
	CHECK_EQ(f.panel[19][19], 15);
	CHECK_EQ(f.panel[3][4], 3);
	/* PANEL = 0 goes to the platform, which keeps the microphone and its indicator (VM.md §6.5; this
	 * fixture's panel goes off); a later stop draws nothing on a panel that is off */
	c.n = 0;
	I(&c, SYS_WRITE, K(VM_P_PANEL), K(0));
	send_code(&c);
	run();
	CHECK_EQ(f.panel_on, 0);
	CHECK_EQ(vm.mic, 1);
	vm_set_mic(&vm, 0);
	CHECK_EQ(f.panel_on, 0);
}

static const char *clock_listing =
	"0c000  10 ff 44 00 42 00\n"
	"0c006  10 ff 45 00 43 00\n"
	"0c00c  30 2b c0 00\n"
	"0c010  11 ff f0 44 00 44 00 21 00\n"
	"0c019  15 ff f0 44 00 44 00 37 00\n"
	"0c022  11 ff f0 45 00 45 00 3a 00\n"
	"0c02b  14 4f f0 44 00 30 00\n"
	"0c032  15 5f f0 44 00 30 00\n"
	"0c039  14 64 f0 27 00\n"
	"0c03e  26 ff 60 35 00 20 00\n"
	"0c045  15 64 f0 27 00\n"
	"0c04a  26 ff 60 35 00 21 00\n"
	"0c051  14 65 f0 27 00\n"
	"0c056  26 ff 60 35 00 23 00\n"
	"0c05d  15 65 f0 27 00\n"
	"0c062  26 ff 60 35 00 24 00\n"
	"0c069  50 ff ff ff f0 38 00 31 00 20 00 20 00 33 00 2d 00 20 00\n"
	"0c07c  52 ff ff ff ff f0 39 00 38 00 31 00 24 00 2c 00 22 00 29 00 35 00 25 00\n"
	"0c094  53 ff ff ff ff 38 00 31 00 20 00 20 00 34 00 26 00 33 00 2d 00\n"
	"0c0a9  45 ff 2e 00 45 00\n"
	"0c0af  33\n";

static const char *battery_listing =
	"0c000  44 0f 2a 00\n"
	"0c004  37 f0 70 c0 00 32 00\n"
	"0c00b  10 1f 23 00\n"
	"0c00f  12 11 f0 21 00\n"
	"0c014  15 20 f0 27 00\n"
	"0c019  26 f1 20 35 00\n"
	"0c01e  14 00 f0 27 00\n"
	"0c023  36 0f 0f c0 00 20 00\n"
	"0c02a  11 31 f0 35 00\n"
	"0c02f  12 4f 10 24 00\n"
	"0c034  50 ff ff ff f0 38 00 2f 00 20 00 20 00 31 00 2c 00 20 00\n"
	"0c047  52 ff ff ff f3 40 39 00 38 00 2f 00 22 00 2b 00 22 00 29 00\n"
	"0c05b  53 ff ff ff ff 38 00 2f 00 20 00 20 00 36 00 26 00 31 00 2c 00\n"
	"0c070  33\n";

static uint8_t example_font[16384];
static uint32_t example_font_size;

/* the §9.1 setup packets: pool, the program's code at 0xC000, the font at 0x25800 */
static void example_setup(const uint8_t *code, uint32_t n)
{
	static const uint32_t pool[27] = {0, 1, 2, 3, 4, 5, 8, 10, 12, 15, 17, 20, 24, 32, 34, 40, 60, 80, 100,
					  160, 240, 256, 560, 1440, 16384, 153600, 60000000};
	uint8_t pk[240];
	uint32_t len = hex("01 80 00 00 6c", pk);

	setup(0);
	for (int i = 0; i < 27; i++) {
		put32(pk + len + 4 * i, pool[i]);
	}
	send(pk, len + 108);
	len = hex("01 00 c0 00", pk);
	pk[len++] = (uint8_t)n;
	memcpy(pk + len, code, n);
	send(pk, len + n);
	for (uint32_t off = 0; off < example_font_size; off += 200) {
		struct code c = {{0}, 0};
		uint32_t m = example_font_size - off < 200 ? example_font_size - off : 200;

		W(&c, 0x25800 + off, example_font + off, m);
		send_code(&c);
	}
}

static void example_table(const uint32_t *words)
{
	uint8_t pk[80];
	uint32_t len = hex("01 40 00 00 40", pk);

	for (int i = 0; i < 16; i++) {
		put32(pk + len + 4 * i, words[i]);
	}
	send(pk, len + 64);
}

/* the band as the protocol 4 code draws it: FILL, then GLYPH_RUN of the text */
static void v4_band(uint32_t stride, uint32_t w, uint32_t h, int32_t x, int32_t y, const uint8_t *text, uint32_t n)
{
	uint8_t *a4 = v4_arena(), b[64];

	memcpy(a4 + 0x25800, example_font, example_font_size);
	put16(b, 0x4000 / 4);
	put16(b + 2, stride);
	put16(b + 4, 0);
	put16(b + 6, 0);
	put16(b + 8, w);
	put16(b + 10, h);
	b[12] = 0;
	CHECK_EQ(v4_fill(b), 0);
	put16(b, 0x25800 / 4);
	put16(b + 2, 0x4000 / 4);
	put16(b + 4, stride);
	put16(b + 6, (uint32_t)x);
	put16(b + 8, (uint32_t)y);
	b[10] = 2;
	b[11] = 15;
	b[12] = (uint8_t)n;
	memcpy(b + 13, text, n);
	CHECK_EQ(v4_glyph_run(b), 0);
}

static int panel_shows_band(int32_t px, int32_t py, uint32_t stride, int32_t w, int32_t h)
{
	const uint8_t *a4 = v4_arena();
	int ok = 1;

	for (int32_t y = 0; y < h; y++) {
		for (int32_t x = 0; x < w; x++) {
			uint8_t b = a4[0x4000 + (uint32_t)y * stride + (uint32_t)x / 2];

			ok &= f.panel[py + y][px + x] == (x & 1 ? (b & 15) : (b >> 4));
		}
	}
	return ok && memcmp(a4 + 0x4000, arena + 0x4000, stride * (uint32_t)h) == 0;
}

static void test_clock(void)
{
	uint8_t code[256], pk[64], digits[5];
	uint32_t n = listing(clock_listing, code, 0xC000), len, n1, j0, begins, words[16] = {0};

	CHECK_EQ(n, 176);
	example_font_size = make_font(example_font, 65, 0, 62);
	example_setup(code, n);
	len = hex("01 00 01 00 05 00 00 40 00 00", pk);
	send(pk, len);
	n1 = f.t + 1000000;
	len = hex("01 08 01 00 08", pk);
	put32(pk + len, 12 * 60 + 34);
	put32(pk + len + 4, n1);
	send(pk, len + 8);
	words[0] = 0xC000;
	words[1] = 0xC010;
	words[12] = 0xC000;
	example_table(words);
	run();
	CHECK_EQ(f.nev, 0);
	CHECK_EQ(f.head, f.tail);
	CHECK_EQ(vm_start(&vm, 0x40), 0);
	j0 = vm.stats.jumps;
	run();
	CHECK_EQ(f.nev, 0);
	CHECK_EQ(vm.stats.jumps - j0, 1);
	CHECK_EQ(memcmp(arena + 0x100, "\x01\x02\x40\x03\x04", 5), 0);
	CHECK_EQ(pool_getw(68), 754);
	CHECK_EQ(pool_getw(69), n1);
	CHECK_EQ(f.begins, 1);
	CHECK(f.bx == 240 && f.by == 8 && f.bw == 160 && f.bh == 32);
	CHECK(vm_prop(&vm, VM_P_TIMER, &len) == 0 && len == n1);
	v4_band(80, 160, 32, 4, 24, arena + 0x100, 5);
	CHECK(panel_shows_band(240, 8, 80, 160, 32));
	/* the minute tick, with no jump */
	j0 = vm.stats.jumps;
	run_for(n1 - f.t - 1);
	CHECK_EQ(f.begins, 1);
	run_for(1);
	CHECK_EQ(f.begins, 2);
	CHECK_EQ(vm.stats.jumps - j0, 0);
	CHECK_EQ(memcmp(arena + 0x100, "\x01\x02\x40\x03\x05", 5), 0);
	CHECK(vm_prop(&vm, VM_P_TIMER, &len) == 0 && len == n1 + 60000000);
	v4_band(80, 160, 32, 4, 24, arena + 0x100, 5);
	CHECK(panel_shows_band(240, 8, 80, 160, 32));
	/* a correction: WRITE M', N' and RAISE C12 = 42 f0 28 00; then midnight */
	n1 = f.t + 1000;
	len = hex("01 08 01 00 08", pk);
	put32(pk + len, 1439);
	put32(pk + len + 4, n1);
	len += 8;
	len += hex("42 f0 28 00", pk + len);
	send(pk, len);
	run();
	CHECK_EQ(memcmp(arena + 0x100, "\x02\x03\x40\x05\x09", 5), 0);
	run_for(1000);
	CHECK_EQ(memcmp(arena + 0x100, "\x00\x00\x40\x00\x00", 5), 0);
	v4_band(80, 160, 32, 4, 24, arena + 0x100, 5);
	CHECK(panel_shows_band(240, 8, 80, 160, 32));
	/* an N' already past: TIMER fires at once and the tick catches up */
	begins = (uint32_t)f.begins;
	len = hex("01 08 01 00 08", pk);
	put32(pk + len, 600);
	put32(pk + len + 4, f.t - 30000000);
	len += 8;
	len += hex("42 f0 28 00", pk + len);
	send(pk, len);
	run();
	CHECK_EQ((uint32_t)f.begins, begins + 2);
	digits[0] = 1, digits[1] = 0, digits[2] = 64, digits[3] = 0, digits[4] = 1;
	CHECK_EQ(memcmp(arena + 0x100, digits, 5), 0);
	CHECK_EQ(f.nev, 0);
}

static void test_battery(void)
{
	uint8_t code[256], pk[64];
	uint32_t n = listing(battery_listing, code, 0xC000), len, j0, begins, words[16] = {0};

	CHECK_EQ(n, 113);
	example_font_size = make_font(example_font, 65, 0, 62);
	example_setup(code, n);
	len = hex("01 00 01 00 04 00 00 00 3f", pk);
	send(pk, len);
	words[0] = 0xC000;
	words[5] = 0xC000;
	example_table(words);
	f.props[VM_P_BATT_PCT] = 42;
	run();
	CHECK_EQ(f.nev, 0);
	CHECK_EQ(vm_start(&vm, 0x40), 0);
	j0 = vm.stats.jumps;
	run();
	CHECK_EQ(vm.stats.jumps - j0, 1);
	CHECK_EQ(memcmp(arena + 0x101, "\x04\x02\x3f", 3), 0);
	CHECK(f.begins == 1 && f.bx == 560 && f.by == 8 && f.bw == 80 && f.bh == 24);
	v4_band(40, 80, 24, 2, 20, arena + 0x101, 3);
	CHECK(panel_shows_band(560, 8, 40, 80, 24));
	f.props[VM_P_BATT_PCT] = 100;
	j0 = vm.stats.jumps;
	run();
	CHECK_EQ(vm.stats.jumps - j0, 2);
	CHECK_EQ(memcmp(arena + 0x100, "\x01\x00\x00\x3f", 4), 0);
	v4_band(40, 80, 24, 2, 20, arena + 0x100, 4);
	CHECK(panel_shows_band(560, 8, 40, 80, 24));
	f.props[VM_P_BATT_PCT] = 0;
	j0 = vm.stats.jumps;
	run();
	CHECK_EQ(vm.stats.jumps - j0, 0);
	CHECK_EQ(memcmp(arena + 0x102, "\x00\x3f", 2), 0);
	v4_band(40, 80, 24, 2, 20, arena + 0x102, 2);
	CHECK(panel_shows_band(560, 8, 40, 80, 24));
	begins = (uint32_t)f.begins;
	f.props[VM_P_BATT_PCT] = VM_UNKNOWN;
	run();
	CHECK_EQ((uint32_t)f.begins, begins);
	CHECK(panel_shows_band(560, 8, 40, 80, 24));
	CHECK_EQ(vm.tasks, 4);
	CHECK_EQ(f.nev, 0);
}

/* VM.md §4.3: the digits of a u32 from the end of a 10-byte buffer, at most 9 jumps, then GLYPHS */
static void test_number(void)
{
	static const uint32_t values[] = {0, 7, 10, 1234, 4294967295u};
	uint8_t font[16384], b[64], *a4 = v4_arena();
	struct code c = {{0}, 0}, p = {{0}, 0};
	uint32_t size, next, j0;

	setup(0);
	size = make_font(font, 62, 0, -1);
	memcpy(arena + 0x20000, font, size);
	memcpy(a4 + 0x20000, font, size);
	I(&c, MOV, 1, K(10));
	next = 0x8000 + c.n;
	I(&c, SUB, 1, 1, K(1));
	I(&c, MODU, 2, 0, K(10));
	I(&c, ST8, K(0x6000), 1, 2);
	I(&c, DIVU, 0, 0, K(10));
	B(&c, BNE, next, 0, K(0));
	I(&c, ADD, 3, 1, K(0x6000));
	I(&c, SUB, 4, K(10), 1);
	I(&c, GLYPHS, K(0x20000), K(0x7000), K(100), K(3), K(20), K(2), K(15), 3, 4);
	O(&c, END);
	place(0x8000, &c);
	table(VM_T_USER0, 0x8000);
	vm_start(&vm, 0x40);
	run();
	I(&p, RAISE, K(12));
	for (unsigned i = 0; i < sizeof(values) / 4; i++) {
		char txt[16];
		uint32_t digits = (uint32_t)snprintf(txt, sizeof(txt), "%u", values[i]);

		memset(arena + 0x7000, 0, 100 * 30);
		memset(a4 + 0x7000, 0, 100 * 30);
		setw(0, values[i]);
		j0 = vm.stats.jumps;
		send_code(&p);
		run();
		CHECK_EQ(vm.stats.jumps - j0, digits - 1);
		CHECK_EQ(pool_getw(4), digits);
		CHECK_EQ(pool_getw(3), 0x6000 + 10 - digits);
		for (uint32_t k = 0; k < digits; k++) {
			CHECK_EQ(arena[0x6000 + 10 - digits + k], txt[k] - '0');
		}
		put16(b, 0x20000 / 4);
		put16(b + 2, 0x7000 / 4);
		put16(b + 4, 100);
		put16(b + 6, 3);
		put16(b + 8, 20);
		b[10] = 2;
		b[11] = 15;
		b[12] = (uint8_t)digits;
		memcpy(b + 13, arena + pool_getw(3), digits);
		CHECK_EQ(v4_glyph_run(b), 0);
		CHECK_EQ(memcmp(arena + 0x7000, a4 + 0x7000, 100 * 30), 0);
	}
	CHECK_EQ(f.nev, 0);
}

static void test_reset(void)
{
	struct code c = {{0}, 0};
	uint8_t s[32];
	uint32_t v;

	setup(0);
	I(&c, SYS_WRITE, K(VM_P_MOTION_MG), K(5));
	I(&c, SYS_WRITE, K(VM_P_PITCH_LO), K(1));
	I(&c, SYS_WRITE, K(VM_P_LIGHT_HI), K(9));
	I(&c, SYS_WRITE, K(VM_P_IMU_RATE), K(52));
	I(&c, NOTIFY, K(0), K(0));
	I(&c, SLEEP_UNTIL, K(f.t + 1000000));
	I(&c, MOV, 390, K(1));
	send_code(&c);
	table(VM_T_START, 0);
	vm_start(&vm, 0x40);
	run();
	CHECK_EQ(f.imu_rate, 52);
	arena[0x9000] = 1;
	vm_reset_session(&vm, 1);
	CHECK_EQ(f.imu_rate, 13);
	CHECK(vm_prop(&vm, VM_P_MOTION_MG, &v) == 0 && v == 0);
	CHECK(vm_prop(&vm, VM_P_PITCH_LO, &v) == 0 && v == (uint32_t)-90);
	CHECK(vm_prop(&vm, VM_P_LIGHT_HI, &v) == 0 && v == 0xFFFFFFFF);
	vm_status(&vm, s);
	CHECK_EQ(s[0], VM_STOPPED);
	CHECK_EQ(arena[0x9000], 0);
	CHECK(f.head != f.tail);
	f.head = f.tail;
	run_for(2000000);
	CHECK_EQ(pool_getw(390), 0);
	c.n = 0;
	I(&c, NOTIFY, K(0), K(0));
	send_code(&c);
	run();
	CHECK(last(VM_EV_NOTIFY) && get16(last(VM_EV_NOTIFY)->body) == 1);
	arena[0x9000] = 1;
	vm_reset_session(&vm, 0);
	CHECK_EQ(arena[0x9000], 1);
}

/* ASSET_LOAD (§2.4): the ranges, the platform's answers, the ID read before dst is written, NULL asset_read */
static void test_assets(void)
{
	static uint8_t body[5000], zero[64];
	static const struct vm_host bare = {
		NULL, h_now, h_packet, h_packet_done, h_poll, h_prop, h_event, h_panel_begin, h_panel_row,
		h_panel_end, h_panel_power, h_brightness, h_imu_rate, h_prop_write, NULL, NULL
	};
	struct code c = {{0}, 0};
	uint8_t id[16];

	for (uint32_t i = 0; i < sizeof(body); i++) {
		body[i] = (uint8_t)(i * 7 + (i >> 8));
	}
	for (int i = 0; i < 16; i++) {
		id[i] = (uint8_t)(0xA0 + i);
	}
	setup(0);
	memcpy(f.asset[0].id, id, 16);
	f.asset[0].body = body;
	f.asset[0].len = sizeof(body);
	f.nassets = 1;
	memcpy(arena + 0x1000, id, 16);
	I(&c, ASSET_LOAD, K(0x1000), K(0x2000), K(10), K(4000));
	send_code(&c);
	run();
	CHECK_EQ(f.nev, 0);
	CHECK_EQ(memcmp(arena + 0x2000, body + 10, 4000), 0);
	CHECK_EQ(arena[0x2000 + 4000], 0);
	/* the size: 3 bytes and one u16 per escape */
	CHECK_EQ(c.n, 11);
	c.n = 0;
	I(&c, ASSET_LOAD, 1, 2, 3, 4);
	CHECK_EQ(c.n, 3);
	/* off + n past the asset's end: error 3 with off as detail, nothing written */
	c.n = 0;
	I(&c, ASSET_LOAD, K(0x1000), K(0x8000), K(4990), K(11));
	send_code(&c);
	run();
	EXPECT_ERROR(3, ASSET_LOAD, 0, 4990, 0);
	CHECK_EQ(memcmp(arena + 0x8000, zero, 11), 0);
	/* n = 0 at the end is fine */
	c.n = 0;
	I(&c, ASSET_LOAD, K(0x1000), K(0x8000), K(sizeof(body)), K(0));
	f.nev = 0;
	send_code(&c);
	run();
	CHECK_EQ(f.nev, 0);
	/* an unknown ID: 13, detail 0 */
	arena[0x1000] ^= 1;
	c.n = 0;
	I(&c, ASSET_LOAD, K(0x1000), K(0x8000), K(0), K(16));
	send_code(&c);
	run();
	EXPECT_ERROR(13, ASSET_LOAD, 0, 0, 0);
	CHECK_EQ(memcmp(arena + 0x8000, zero, 16), 0);
	arena[0x1000] ^= 1;
	/* the ranges are checked before the platform is asked: id + 16, then dst + n */
	f.asset_calls = 0;
	c.n = 0;
	I(&c, ASSET_LOAD, K(VM_ARENA_SIZE - 15), K(0x8000), K(0), K(1));
	send_code(&c);
	run();
	EXPECT_ERROR(3, ASSET_LOAD, 0, VM_ARENA_SIZE - 15, 0);
	c.n = 0;
	I(&c, ASSET_LOAD, K(0x1000), K(VM_ARENA_SIZE - 100), K(0), K(101));
	send_code(&c);
	run();
	EXPECT_ERROR(3, ASSET_LOAD, 0, VM_ARENA_SIZE - 100, 0);
	c.n = 0;
	I(&c, ASSET_LOAD, K(0x1000), K(0xFFFFFFF0u), K(0), K(0x20));
	send_code(&c);
	run();
	EXPECT_ERROR(3, ASSET_LOAD, 0, 0xFFFFFFF0u, 0);
	CHECK_EQ(f.asset_calls, 0);
	/* dst over the ID itself: the ID is read first */
	memcpy(arena + 0x3000, id, 16);
	c.n = 0;
	I(&c, ASSET_LOAD, K(0x3000), K(0x2FF8), K(100), K(64));
	f.nev = 0;
	send_code(&c);
	run();
	CHECK_EQ(f.nev, 0);
	CHECK_EQ(memcmp(arena + 0x2FF8, body + 100, 64), 0);
	/* in a program: the same load, then an unknown ID halts with 13 */
	c.n = 0;
	I(&c, ASSET_LOAD, K(0x1000), K(0x9000), K(0), K(sizeof(body)));
	I(&c, ASSET_LOAD, K(0x1010), K(0x9000), K(0), K(1));
	O(&c, END);
	place(0xC000, &c);
	table(VM_T_START, 0xC000);
	f.nev = 0;
	CHECK_EQ(vm_start(&vm, 0x40), 0);
	run();
	CHECK_EQ(memcmp(arena + 0x9000, body, sizeof(body)), 0);
	EXPECT_HALT(VM_T_START, 0xC000 + 11, ASSET_LOAD, 13, 0);
	/* without asset_read every ASSET_LOAD is 13 */
	memset(&f, 0, sizeof(f));
	f.has_panel = 1;
	vm_init(&vm, arena, &bare, 0);
	npool = 0;
	memcpy(arena + 0x1000, id, 16);
	c.n = 0;
	I(&c, ASSET_LOAD, K(0x1000), K(0x2000), K(0), K(1));
	send_code(&c);
	run();
	EXPECT_ERROR(13, ASSET_LOAD, 0, 0, 0);
}

/* ---------------------------------------------------------------- fuzz */

static const uint8_t fuzz_ops[] = {WRITE, WRITE_RLE, COPY, UNPACK, MOV, ADD, SUB, MUL, DIVU, MODU, DIVS, MODS,
				   AND, OR, XOR, SHL, SHR, SAR, LD32, LD16, LD8, LD4, ST32, ST16, ST8, ST4, JMP,
				   CALL, END, BEQ, BNE, BLTU, BGEU, BLTS, BGES, SLEEP_UNTIL, WAIT_EVENT, RAISE,
				   NOTIFY, SYS_READ, SYS_WRITE, FILL, BLIT, GLYPHS, DRAW, LINE, FILL, BLIT, GLYPHS,
				   DRAW, LINE, VIDEO, ASSET_LOAD};

static const uint8_t host_ops[] = {WRITE, WRITE_RLE, COPY, UNPACK, MOV, ADD, SUB, MUL, DIVU, MODU, DIVS, MODS,
				   AND, OR, XOR, SHL, SHR, SAR, LD32, LD16, LD8, LD4, ST32, ST16, ST8, ST4, CALL,
				   END, SLEEP_UNTIL, RAISE, NOTIFY, SYS_READ, SYS_WRITE, FILL, BLIT, GLYPHS, DRAW, LINE,
				   VIDEO, ASSET_LOAD};

static void spec_shape(uint8_t op, int *k, int *lit)
{
	*k = 0;
	*lit = 0;
	if (op == COPY || (op >= ADD && op <= ST4)) {
		*k = 3;
	} else if (op == UNPACK) {
		*k = 4;
	} else if (op == MOV || op == WAIT_EVENT || op == NOTIFY || op == SYS_READ || op == SYS_WRITE) {
		*k = 2;
	} else if (op == SLEEP_UNTIL || op == RAISE) {
		*k = 1;
	} else if (op == FILL || op == LINE) {
		*k = 7;
	} else if (op == BLIT) {
		*k = 12;
	} else if (op == GLYPHS) {
		*k = 9;
	} else if (op == DRAW) {
		*k = 8;
	} else if (op == VIDEO) {
		*k = 10;
	} else if (op == ASSET_LOAD) {
		*k = 4;
	} else if (op >= BEQ && op <= BGES) {
		*k = 2;
		*lit = 3;
	} else if (op == JMP || op == CALL) {
		*lit = 3;
	}
}

static const uint32_t fuzz_values[] = {0, 1, 2, 3, 4, 5, 7, 8, 12, 13, 15, 16, 31, 32, 100, 255, 256, 640, 480,
				       0x7FFF, 0x8000, 0xFFFF, 0x10000, 0x1000, 0x1800, 0x20000, 0x3FFC0,
				       0x3FFFF, 0x40000, 0x7FFFFFFF, 0x80000000, 0xFFFFFFFF, 0xFFFFFF80,
				       0x22, 0x21, 0x20, 0x28};

static uint32_t fuzz_value(void)
{
	switch (rndn(5)) {
	case 0:
		return rnd();
	case 1:
		return (uint32_t)((int32_t)rndn(1400) - 700);
	default:
		return fuzz_values[rndn(sizeof(fuzz_values) / 4)];
	}
}

/* operand roles, one letter per operand: x anything, a address, l length, s small, c coordinate,
 * w stride, p property, q writable property, m trigger mask, t time, f font, u source coordinate,
 * o BLIT mode, g GLYPHS mode, e text address, n glyph count, b source bitmap address, v property value,
 * r trigger to raise */
static const char *fuzz_groups = "slcwapqmtfuogenbvr";

/* vectors mode (below): no time, sensor or BATTERY/CASE dependence in what the generator makes */
static int vmode;

static const char *roles(uint8_t op)
{
	if (op == COPY) {
		return "aal";
	}
	if (op == UNPACK) {
		return "aall";
	}
	if (op >= LD32 && op <= LD4) {
		return "xas";
	}
	if (op >= ST32 && op <= ST4) {
		return "asx";
	}
	if (op == SLEEP_UNTIL) {
		return "t";
	}
	if (op == WAIT_EVENT) {
		return "xm";
	}
	if (op == RAISE) {
		return "r";
	}
	if (op == SYS_READ) {
		return "xp";
	}
	if (op == SYS_WRITE) {
		return "qv";
	}
	if (op == FILL) {
		return "awcclls";
	}
	if (op == BLIT) {
		return "bwuuawccllos";
	}
	if (op == GLYPHS) {
		return "fawccgsen";
	}
	if (op == DRAW) {
		return "bwuuccll";
	}
	if (op == LINE) {
		return "awccccs";
	}
	if (op == VIDEO) {
		return "awllccsbna";
	}
	return "xxx";
}

/* the pool holds 4 words per role from word 32 on (words 32-99) */
static int role_word(char r)
{
	const char *g = strchr(fuzz_groups, r);

	if (r == 'x' || g == NULL || rndn(64) == 0) {
		if (rndn(16) == 0) {
			return 32 + (int)rndn(4 * (uint32_t)strlen(fuzz_groups));
		}
		return rndn(2) ? (int)rndn(15) : 160 + (int)rndn(32);
	}
	return 32 + 4 * (int)(g - fuzz_groups) + (int)rndn(4);
}

static uint32_t role_value(char r)
{
	static const uint32_t bases[] = {0x600, 0x700, 0x10000, 0x18000, 0x30000, 0x30800, 0x3F000, VM_ARENA_SIZE - 16};
	static const uint32_t rare[] = {0x80, 0x400, 0x8000, 0x20000};
	static const uint32_t readable[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
					    0x16, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x20, 0x21, 0x22, 0x23, 0x24,
					    0x25, 0x26, 0x27, 0x28, 0x35, 0x36, 0x37};
	/* properties that hold the same value on the firmware in the emulator and here (vectors) */
	static const uint32_t steady[] = {0x01, 0x02, 0x03, 0x04, 0x13, 0x14, 0x18, 0x19, 0x1A, 0x1B, 0x1D,
					  0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x35, 0x36, 0x37};

	switch (r) {
	case 's':
		return rndn(16);
	case 'l':
		return rndn(32) ? rndn(120) : fuzz_value();
	case 'c':
		return rndn(32) ? (uint32_t)((int32_t)rndn(800) - 100) : fuzz_value();
	case 'w':
		return rndn(32) ? 64 + rndn(200) : fuzz_value();
	case 'a':
		if (rndn(32) == 0) {
			return rare[rndn(4)] + rndn(64);
		}
		return rndn(32) ? bases[rndn(8)] + rndn(64) : fuzz_value();
	case 'p':
		if (vmode) {
			return rndn(8) ? steady[rndn(sizeof(steady) / 4)] : 0x29 + rndn(12);
		}
		return rndn(8) ? readable[rndn(sizeof(readable) / 4)] : rndn(0x30);
	case 'q':
		return 0x20 + rndn(vmode ? 8 : 9);   /* vectors: IMU_RATE 0 would make the arm's IMU values UNKNOWN */
	case 'm': {
		uint32_t m = rndn(8) ? 1u << rndn(16) | (rndn(2) ? 1u << rndn(16) : 0) : fuzz_value();

		return vmode ? m & ~0x60u : m;
	}
	case 'r': {
		uint32_t t = rndn(16);

		return vmode && (t == VM_T_BATTERY || t == VM_T_CASE) ? VM_T_USER0 : t;
	}
	case 't':
		return (vmode ? 0x40000000u : f.t) + rndn(8000) - 2000;
	case 'f':
		return rndn(32) ? 0x20000 : fuzz_value();
	case 'u':
		return rndn(32) ? rndn(64) : fuzz_value();
	case 'o':
		return rndn(32) ? rndn(8) : fuzz_value();
	case 'g':
		return rndn(32) ? rndn(4) : fuzz_value();
	case 'e':
		return rndn(32) ? 0x400 + rndn(32) : fuzz_value();
	case 'n':
		return rndn(32) ? rndn(20) : fuzz_value();
	case 'b':
		return rndn(32) ? 0x10000 + 0x4000 * rndn(4) + rndn(64) : fuzz_value();
	case 'v': {
		static const uint32_t vals[] = {0, 1, 13, 21, 26, 42, 52, 104};

		return rndn(4) ? vals[rndn(8)] : (vmode ? 0x40000000u : f.t) + rndn(5000);
	}
	}
	return fuzz_value();
}

struct gen {
	uint32_t starts[2048], nstarts, fix[2048], nfix;
};

/* one instruction, mostly with operands that suit their roles; jump targets are patched afterwards */
static void fuzz_ins(struct code *c, uint32_t base, int host, struct gen *g)
{
	uint8_t op = host ? host_ops[rndn(sizeof(host_ops))] : fuzz_ops[rndn(sizeof(fuzz_ops))];
	const char *r;
	int k, lit, w[12];

	switch (rndn(256)) {
	case 0:
		op = (uint8_t)rnd();
		break;
	case 1:
		op = RET;
		break;
	case 2:
		op = HALT;
		break;
	}
	if (vmode && op == SLEEP_UNTIL) {
		op = NOTIFY;
	}
	r = roles(op);
	if (!host && g->nstarts < 2048) {
		g->starts[g->nstarts++] = base + c->n;
	}
	if (op == WRITE || op == WRITE_RLE) {
		uint8_t d[40];
		uint32_t n = rndn(40), addr = role_value('a') & 0xFFFFFF;

		for (uint32_t i = 0; i < n; i++) {
			d[i] = (uint8_t)rnd();
		}
		if (op == WRITE) {
			W(c, addr, d, n);
			return;
		}
		c->b[c->n++] = WRITE_RLE;
		c->b[c->n++] = (uint8_t)addr;
		c->b[c->n++] = (uint8_t)(addr >> 8);
		c->b[c->n++] = (uint8_t)(addr >> 16);
		put16(c->b + c->n, rndn(200));
		c->n += 2;
		c->b[c->n++] = (uint8_t)n;
		memcpy(c->b + c->n, d, n);
		c->n += n;
		return;
	}
	spec_shape(op, &k, &lit);
	for (int i = 0; i < k; i++) {
		w[i] = role_word(r[i]);
	}
	if (lit && g->nfix < 2048) {
		g->fix[g->nfix++] = c->n + 1 + (uint32_t)(k + 1) / 2;
	}
	emit(c, op, w, k, lit, rnd() & 0xFFFFFF);
}

static void patch_targets(struct code *c, const struct gen *g, const struct gen *targets)
{
	for (uint32_t i = 0; i < g->nfix; i++) {
		uint32_t t = rndn(16) && targets->nstarts ? targets->starts[rndn(targets->nstarts)] : rnd() & 0xFFFFFF;

		c->b[g->fix[i]] = (uint8_t)t;
		c->b[g->fix[i] + 1] = (uint8_t)(t >> 8);
		c->b[g->fix[i] + 2] = (uint8_t)(t >> 16);
	}
}

static void fuzz(double seconds)
{
	static struct gen g, gp;
	uint8_t font[16384], s[32];
	clock_t end = clock() + (clock_t)(seconds * CLOCKS_PER_SEC);
	unsigned long runs = 0, ins = 0, draws = 0, tasks = 0, halts = 0, errors = 0;
	static unsigned long hist[256][16];

	while (clock() < end) {
		struct code c = {{0}, 0};

		setup(rndn(20) == 0);
		f.spans = 1;
		f.tick = rndn(3);
		f.row_cost = rndn(4);
		f.has_panel = rndn(8) != 0;
		f.drop = rndn(4) == 0;
		f.nassets = (int)rndn(2);   /* the all-zero ID, so zeroed words name it */
		f.asset[0].body = font;
		f.asset[0].len = 1 + rndn(sizeof(font));
		f.props[VM_P_ARM] = rndn(2);
		for (int i = 0; i < 0x30; i++) {
			if (rndn(4) == 0) {
				f.props[i] = fuzz_value();
			}
		}
		for (uint32_t i = 0; i < 32; i++) {
			setw(i, fuzz_value());
		}
		for (uint32_t i = 0; i < 4 * strlen(fuzz_groups); i++) {
			setw(32 + i, role_value(fuzz_groups[i / 4]));
		}
		for (uint32_t i = 160; i < 192; i++) {
			setw(i, fuzz_value());
		}
		for (uint32_t i = 0x30000; i < 0x31000; i++) {
			arena[i] = (uint8_t)rnd();
		}
		if (rndn(32)) {
			uint32_t ng = 1 + rndn(200), size = make_font(font, ng, rndn(20), (int)rndn(200));

			memcpy(arena + 0x20000, font, size);
			for (uint32_t i = 0; i < 64; i++) {
				arena[0x400 + i] = (uint8_t)rndn(ng < 128 ? ng : 128);
			}
		}
		g.nstarts = g.nfix = 0;
		while (c.n < 0x1800 - 64) {
			fuzz_ins(&c, 0x8000, 0, &g);
		}
		patch_targets(&c, &g, &g);
		place(0x8000, &c);
		for (uint32_t i = 0; i < 16; i++) {
			table(i, rndn(3) ? g.starts[rndn(g.nstarts)] : rndn(4) ? 0 : fuzz_value());
		}
		if (rndn(8)) {
			vm_start(&vm, rndn(16) ? 0x40 : fuzz_value());
		}
		for (int round = 0; round < 40; round++) {
			uint32_t w;

			if (rndn(2) && f.tail - f.head < 100) {
				c.n = 0;
				gp.nfix = 0;
				for (int i = 0, n = 1 + (int)rndn(6); i < n && c.n < 160; i++) {
					fuzz_ins(&c, 0, 1, &gp);
				}
				patch_targets(&c, &gp, &g);
				if (c.n <= 242) {
					send_code(&c);
				}
			}
			if (rndn(4) == 0) {
				f.fired = rnd() & 0xFFFF;
			}
			switch (rndn(60)) {
			case 0:
				vm_stop(&vm);
				break;
			case 1:
				vm_start(&vm, rndn(4) ? 0x40 : fuzz_value());
				break;
			case 2:
				vm_reset_session(&vm, (int)rndn(2));
				f.head = f.tail;
				break;
			case 3:
				vm_set_mic(&vm, (int)rndn(2));
				break;
			case 4:
				f.props[rndn(0x30)] = fuzz_value();
				break;
			}
			vm_status(&vm, s);
			if (s[0] == VM_HALTED && rndn(3) == 0) {
				vm_start(&vm, 0x40);
			}
			CHECK(s[0] <= 2 && s[12] <= 3 && s[14] <= 16);
			f.yield_at = (uint32_t)vm.stats.instructions + 1 + rndn(3000);
			w = vm_run(&vm);
			f.t += w != 0 && w != VM_FOREVER ? (rndn(2) ? w : rndn(w + 1)) : rndn(1000);
		}
		runs++;
		ins += vm.stats.instructions;
		draws += vm.stats.draws;
		tasks += vm.tasks;
		halts += (unsigned long)count(VM_EV_HALT);
		errors += (unsigned long)count(VM_EV_ERROR);
		if (getenv("VM_FUZZ_HIST")) {
			for (uint32_t i = 0; i < f.nev; i++) {
				if (f.ev[i].type == VM_EV_HALT) {
					hist[f.ev[i].body[5]][f.ev[i].body[6] & 15]++;
				}
			}
		}
	}
	if (getenv("VM_FUZZ_HIST")) {
		for (int op = 0; op < 256; op++) {
			for (int code = 0; code < 16; code++) {
				if (hist[op][code] > runs / 50) {
					printf("halt op 0x%02x code %d: %lu\n", op, code, hist[op][code]);
				}
			}
		}
	}
	if (runs == 0) {
		return;
	}
	printf("fuzz: %lu runs in %.0f s; per run %.0f instructions, %.2f draws, %.1f tasks, %.2f halts, %.2f errors\n",
	       runs, seconds, (double)ins / runs, (double)draws / runs, (double)tasks / runs, (double)halts / runs,
	       (double)errors / runs);
}

/* ---------------------------------------------------------------- vectors for the firmware */

/* "vm_test vectors FILE [N [SEED]]" writes test cases for an arm made only of host traffic (packets,
 * VM_START, VM_STOP): a directed DRAW case and N fuzz cases. Each case runs here twice, with different
 * device times and battery values, and is kept only if both runs agree, so its results do not depend on
 * time or sensors. A case lists its packets, VM_STATUS at each point where nothing can run (time fields
 * zeroed), then its ERROR, HALT and NOTIFY events, the arena's CRC-32 and the panel's (PANEL_READ rows).
 * g1_firmware/emulator/tests/v5_tests.py replays them through the firmware. */

static char vtext[1 << 21];
static size_t vlen;
static int vbad;

static void vput(const char *fmt, ...)
{
	va_list ap;
	int n;

	va_start(ap, fmt);
	n = vsnprintf(vtext + vlen, sizeof(vtext) - vlen, fmt, ap);
	va_end(ap);
	if (n < 0 || (size_t)n >= sizeof(vtext) - vlen) {
		vbad = 1;
		return;
	}
	vlen += (size_t)n;
}

static void vhex(const uint8_t *b, uint32_t n)
{
	static const char digits[] = "0123456789abcdef";

	if (vlen + 2 * n + 2 > sizeof(vtext)) {
		vbad = 1;
		return;
	}
	for (uint32_t i = 0; i < n; i++) {
		vtext[vlen++] = digits[b[i] >> 4];
		vtext[vlen++] = digits[b[i] & 15];
	}
	vtext[vlen++] = '\n';
}

static uint32_t crc32z(uint32_t crc, const uint8_t *p, uint32_t n)
{
	crc = ~crc;
	while (n--) {
		crc ^= *p++;
		for (int k = 0; k < 8; k++) {
			crc = crc >> 1 ^ (0xEDB88320u & (0u - (crc & 1)));
		}
	}
	return ~crc;
}

/* the platform as the firmware shows it in the emulator (g1v0 0.5.18+): right arm, a host connected, in the
 * case (VBUS, so WORN 0), the IMU model at rest with all axes 0 (ACC_*, PITCH, ROLL 0), the gyroscope off, no
 * gesture yet, no case messages; LIGHT depends on the panel, so the cases do not read it; env 1 changes the
 * time and the battery values */
static void venv(int env)
{
	setup(0);
	f.spans = 1;
	f.t = env ? 2000000000u : 10000000u;
	f.props[VM_P_LINK_INTERVAL] = env ? 15000 : 7500;
	f.props[VM_P_BATT_MV] = env ? 4100 : 3900;
	f.props[VM_P_BATT_PCT] = env ? 80 : 50;
	f.props[VM_P_CHARGE] = env ? 1 : 0;
	f.props[VM_P_BATT_MA] = env ? (uint32_t)-30 : 5;
	f.props[VM_P_BATT_TEMP] = env ? 300 : 250;
	f.props[VM_P_IN_CASE] = 1;
	f.props[VM_P_LIGHT] = env ? 500 : 0;   /* the emulated right arm's OPT3001 reads 0 lux: LIGHT cases drop out */
	f.props[VM_P_ACC_X] = 0;
	f.props[VM_P_ACC_Y] = 0;
	f.props[VM_P_ACC_Z] = 0;
	f.props[VM_P_PITCH] = 0;
	f.props[VM_P_ROLL] = 0;
	f.props[VM_P_WORN] = 0;
	f.props[VM_P_GYRO_RATE] = 0;
	f.props[VM_P_TOUCH_GESTURE] = 0;
	vm_init(&vm, arena, &host, 0);
	vlen = 0;
	vbad = 0;
}

static void vsend(const uint8_t *code, uint32_t n)
{
	if (f.tail - f.head >= 100) {
		run();
	}
	send(code, n);
	vput("pkt ");
	vhex(code, n);
}

static void vupload(uint32_t addr, const uint8_t *d, uint32_t n)
{
	for (uint32_t off = 0; off < n; off += 200) {
		struct code c = {{0}, 0};

		W(&c, addr + off, d + off, n - off < 200 ? n - off : 200);
		vsend(c.b, c.n);
	}
}

/* one instruction in a packet of its own, its operand values written just before it (words 4096...) */
static void vins(uint8_t op, const uint32_t *v, int k)
{
	struct code c = {{0}, 0};
	uint8_t b[48];
	int w[12];

	for (int i = 0; i < k; i++) {
		put32(b + 4 * i, v[i]);
		w[i] = 4096 + i;
	}
	W(&c, 4 * 4096, b, (uint32_t)(4 * k));
	emit(&c, op, w, k, 0, 0);
	vsend(c.b, c.n);
}

/* run until nothing can run now; a case whose program never settles or that draws too much is dropped */
static void vidle(void)
{
	uint8_t st[32];

	f.yield_at = (uint32_t)vm.stats.instructions + 200000;
	if (vm_run(&vm) == 0 || f.rows > 4000 || f.nev >= MAXEV) {
		vbad = 1;
	}
	f.yield_at = 0;
	vm_status(&vm, st);
	memset(st + 26, 0, 2);
	memset(st + 29, 0, 3);
	vput("idle %u ", (unsigned)f.seq);
	vhex(st, 32);
}

static void vstart(uint32_t table)
{
	vput("start %08x %d\n", (unsigned)table, vm_start(&vm, table));
	vidle();
}

/* every case starts with the panel off and brightness 21, as after a boot */
static void vprologue(void)
{
	static const uint32_t v[2][2] = {{VM_P_PANEL, 0}, {VM_P_BRIGHTNESS, 21}};

	vins(SYS_WRITE, v[0], 2);
	vins(SYS_WRITE, v[1], 2);
}

static void vpanel(void)
{
	uint8_t row[VM_PANEL_W / 2];
	uint32_t crc = 0;

	for (int y = 0; y < VM_PANEL_H; y++) {
		for (int x = 0; x < VM_PANEL_W; x += 2) {
			row[x / 2] = (uint8_t)(f.panel[y][x] << 4 | f.panel[y][x + 1]);
		}
		crc = crc32z(crc, row, sizeof(row));
	}
	vput(f.panel_on ? "panel %08x\n" : "panel off\n", (unsigned)crc);
}

static void vend(void)
{
	for (uint32_t i = 0; i < f.nev; i++) {
		vput("event %02x ", f.ev[i].type);
		vhex(f.ev[i].body, f.ev[i].len);
	}
	vput("arena %08x\n", (unsigned)crc32z(0, arena, VM_ARENA_SIZE));
	vpanel();
	vput("end\n");
}

/* DRAW at every alignment and edge: a full frame, full-width bands, odd x, w and source x, clipping */
static void vdraw_case(int env)
{
	static const int32_t draws[][6] = {
		{0, 0, 0, 0, 640, 480}, {0, 100, 0, 7, 640, 1}, {1, 3, 0, 20, 639, 30}, {0, 0, 1, 60, 639, 2},
		{5, 9, 3, 70, 1, 40}, {6, 9, 4, 70, 2, 40}, {7, 9, 6, 70, 3, 40}, {11, 0, 101, 300, 77, 33},
		{12, 0, 202, 300, 78, 33}, {0, 0, -5, 120, 50, 10}, {3, 0, -6, 140, 51, 10}, {0, 0, 600, 150, 100, 10},
		{1, 0, 601, 170, 100, 10}, {0, 0, 300, -9, 40, 20}, {0, 0, 300, 470, 40, 20}, {0, 0, -700, 0, 10, 10},
		{0, 0, 0, 480, 10, 10}, {0, 0, 0, 0, 0, 10}, {9, 77, 639, 479, 1, 1}, {0, 0, -1, -1, 642, 482},
		{2, 2, 320, 240, 319, 239}, {1, 1, 10, 400, 1, 1}, {0, 5, 0, 250, 640, 16},
	};
	uint32_t v[8];

	venv(env);
	vput("case draw\n");
	vprologue();
	for (uint32_t i = 0; i < 30; i++) {
		uint32_t fill[7] = {0x10000, 320, 0, 16 * i, 640, 16, i % 16};

		vins(FILL, fill, 7);
	}
	for (uint32_t j = 0; j < 12; j++) {
		uint32_t fill[7] = {0x10000, 320, 7 + 53 * j, 0, 1 + 2 * j, 480, 15 - j};

		vins(FILL, fill, 7);
	}
	vidle();
	for (uint32_t i = 0; i < sizeof(draws) / sizeof(draws[0]); i++) {
		v[0] = 0x10000;
		v[1] = 320;
		for (int k = 0; k < 6; k++) {
			v[2 + k] = (uint32_t)draws[i][k];
		}
		vins(DRAW, v, 8);
		vidle();
		if (i == 0 || i == 2 || i == 10 || i == 16) {
			vpanel();
		}
	}
	v[0] = VM_P_PANEL;
	v[1] = 0;
	vins(SYS_WRITE, v, 2);
	vidle();
	vpanel();
	v[0] = 0x10000;
	v[1] = 320;
	v[2] = 100;
	v[3] = 100;
	v[4] = 33;
	v[5] = 44;
	v[6] = 55;
	v[7] = 66;
	vins(DRAW, v, 8);
	vidle();
	vend();
}

/* VIDEO: the two golden records, then random bitmaps and records with every flag, a full-width frame, at the
 * panel's edges and off it */
static void vvideo_case(int env)
{
	static uint8_t buf[6000];
	uint32_t v[10] = {0x2000, 7, 37, 23, 100, 50, 9, 0x5000, sizeof(video_rec0), 0x3000};

	venv(env);
	seed = 0x51DE0;
	vput("case video\n");
	vprologue();
	memset(buf, 0, 7 * 23);
	vupload(0x2000, buf, 7 * 23);
	vupload(0x5000, video_rec0, sizeof(video_rec0));
	vins(VIDEO, v, 10);
	vidle();
	vpanel();
	vupload(0x5000, video_rec1, sizeof(video_rec1));
	v[8] = sizeof(video_rec1);
	vins(VIDEO, v, 10);
	vidle();
	vpanel();
	for (int i = 0; i < 14; i++) {
		uint32_t w = i == 3 ? 640 : 1 + rndn(300), h = i == 3 ? 9 : 1 + rndn(40), sw = (w + 7) / 8 + rndn(2);
		uint32_t n = 1 + rndn(400);

		for (uint32_t k = 0; k < sw * h; k++) {
			buf[k] = (uint8_t)rnd();
		}
		vupload(0x8000, buf, sw * h);
		for (uint32_t k = 0; k < n; k++) {
			buf[k] = (uint8_t)rnd();
		}
		buf[0] = (uint8_t)(i % 4 == 0 ? 3 : rndn(4));
		vupload(0x10000, buf, n);
		v[0] = 0x8000;
		v[1] = sw;
		v[2] = w;
		v[3] = h;
		v[4] = i == 3 ? 0 : (uint32_t)((int32_t)rndn(760) - 60);
		v[5] = (uint32_t)((int32_t)rndn(520) - 20);
		v[6] = rndn(16);
		v[7] = 0x10000;
		v[8] = n;
		v[9] = 0x3000;
		vins(VIDEO, v, 10);
		vidle();
		if (i % 3 == 2) {
			vpanel();
		}
	}
	vend();
}

static void vfuzz_case(uint64_t cseed, int env)
{
	static struct gen g, gp;
	static uint8_t blob[16384];
	uint8_t buf[1024];
	struct code c = {{0}, 0};
	uint32_t ng = 4 * (uint32_t)strlen(fuzz_groups);

	venv(env);
	seed = cseed;
	vput("case fuzz-%016llx\n", (unsigned long long)cseed);
	vprologue();
	for (uint32_t i = 0; i < 32; i++) {
		put32(buf + 4 * i, fuzz_value());
	}
	vupload(0, buf, 128);
	for (uint32_t i = 0; i < ng; i++) {
		put32(buf + 4 * i, role_value(fuzz_groups[i / 4]));
	}
	vupload(128, buf, 4 * ng);
	for (uint32_t i = 0; i < 32; i++) {
		put32(buf + 4 * i, fuzz_value());
	}
	vupload(640, buf, 128);
	for (uint32_t i = 0; i < 1024; i++) {
		buf[i] = (uint8_t)rnd();
	}
	vupload(0x30000, buf, 1024);
	if (rndn(32)) {
		uint32_t n = 1 + rndn(64), size = make_font(blob, n, rndn(20), (int)rndn(70));

		vupload(0x20000, blob, size);
		for (uint32_t i = 0; i < 64; i++) {
			buf[i] = (uint8_t)rndn(n);
		}
		vupload(0x400, buf, 64);
	}
	g.nstarts = g.nfix = 0;
	while (c.n < 0x800 - 64) {
		fuzz_ins(&c, 0x8000, 0, &g);
	}
	patch_targets(&c, &g, &g);
	vupload(0x8000, c.b, c.n);
	for (uint32_t i = 0; i < 16; i++) {
		uint32_t h = rndn(3) ? g.starts[rndn(g.nstarts)] : rndn(4) ? 0 : fuzz_value();

		put32(buf + 4 * i, i == VM_T_BATTERY || i == VM_T_CASE ? 0 : h);
	}
	vupload(0x40, buf, 64);
	vidle();
	if (rndn(8)) {
		vstart(rndn(16) ? 0x40 : fuzz_value());
	}
	for (int round = 0; round < 12 && !vbad; round++) {
		switch (rndn(12)) {
		case 0:
			vput("stop\n");
			vm_stop(&vm);
			vidle();
			break;
		case 1:
			vstart(rndn(4) ? 0x40 : fuzz_value());
			break;
		default:
			c.n = 0;
			gp.nfix = 0;
			for (int i = 0, n = 1 + (int)rndn(6); i < n && c.n < 160; i++) {
				fuzz_ins(&c, 0, 1, &gp);
			}
			patch_targets(&c, &gp, &g);
			if (c.n <= 242) {
				vsend(c.b, c.n);
			}
			vidle();
			break;
		}
	}
	vend();
}

static void vectors(const char *path, uint32_t n, uint64_t s0)
{
	static char first[sizeof(vtext)];
	FILE *out = fopen(path, "w");
	size_t first_len;
	uint32_t kept = 0, tried = 0;

	if (!out) {
		perror(path);
		exit(1);
	}
	vmode = 1;
	fprintf(out, "# scouter_kit/tests/vm_test vectors, seed 0x%llx\n", (unsigned long long)s0);
	for (uint32_t i = 0; i <= n + 1 && tried < 50 * (n + 2); tried++) {
		uint64_t cs = (s0 + tried * 0x9E3779B97F4A7C15ull) | 1;

		i == 0 ? vdraw_case(0) : i == 1 ? vvideo_case(0) : vfuzz_case(cs, 0);
		if (vbad) {
			CHECK(i > 1);
			continue;
		}
		memcpy(first, vtext, vlen);
		first_len = vlen;
		i == 0 ? vdraw_case(1) : i == 1 ? vvideo_case(1) : vfuzz_case(cs, 1);
		if (vbad || vlen != first_len || memcmp(first, vtext, vlen) != 0) {
			CHECK(i > 1);
			continue;
		}
		fwrite(vtext, 1, vlen, out);
		kept += i > 1;
		i++;
	}
	fclose(out);
	vmode = 0;
	printf("vectors: draw and video cases + %u fuzz cases (%u tried) in %s\n", (unsigned)kept, (unsigned)tried, path);
}

int main(int argc, char **argv)
{
	if (argc > 2 && strcmp(argv[1], "vectors") == 0) {
		vectors(argv[2], argc > 3 ? (uint32_t)atoi(argv[3]) : 40, argc > 4 ? strtoull(argv[4], NULL, 0) : 0x5EED);
		printf("%d checks, %d failures\n", checks, failures);
		return failures != 0;
	}
	test_encoding();
	test_alu();
	test_branches();
	test_load_store();
	test_memory();
	test_host_rules();
	test_host_call();
	test_program();
	test_triggers();
	test_wants();
	test_input();
	test_cap();
	test_host_first();
	test_status();
	test_props();
	test_gfx_v4();
	test_gfx_clip();
	test_line();
	test_draw();
	test_draw_instructions();
	test_video();
	test_mic();
	test_clock();
	test_battery();
	test_number();
	test_reset();
	test_assets();
	if (argc > 3) {
		seed = strtoull(argv[3], NULL, 0) | 1;
	}
	fuzz(argc > 2 && strcmp(argv[1], "fuzz") == 0 ? atof(argv[2]) : 2.0);
	printf("%d checks, %d failures\n", checks, failures);
	return failures != 0;
}
