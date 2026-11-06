/* The VM interpreter and executor (vm.h, g1_firmware/docs/VM.md §2, §5, §6.2-§6.5). */
#include <string.h>
#include "vm.h"

enum op {
	OP_WRITE = 0x01, OP_WRITE_RLE = 0x02, OP_COPY = 0x03, OP_UNPACK = 0x04,
	OP_MOV = 0x10, OP_ADD = 0x11, OP_SUB = 0x12, OP_MUL = 0x13, OP_DIVU = 0x14, OP_MODU = 0x15,
	OP_DIVS = 0x16, OP_MODS = 0x17, OP_AND = 0x18, OP_OR = 0x19, OP_XOR = 0x1A, OP_SHL = 0x1B,
	OP_SHR = 0x1C, OP_SAR = 0x1D,
	OP_LD32 = 0x20, OP_LD4 = 0x23, OP_ST32 = 0x24, OP_ST4 = 0x27,
	OP_JMP = 0x30, OP_CALL = 0x31, OP_RET = 0x32, OP_END = 0x33, OP_HALT = 0x34,
	OP_BEQ = 0x35, OP_BNE = 0x36, OP_BLTU = 0x37, OP_BGEU = 0x38, OP_BLTS = 0x39, OP_BGES = 0x3A,
	OP_SLEEP_UNTIL = 0x40, OP_WAIT_EVENT = 0x41, OP_RAISE = 0x42, OP_NOTIFY = 0x43,
	OP_SYS_READ = 0x44, OP_SYS_WRITE = 0x45,
	OP_FILL = 0x50, OP_BLIT = 0x51, OP_GLYPHS = 0x52, OP_DRAW = 0x53, OP_LINE = 0x54, OP_VIDEO = 0x55,
	OP_ASSET_LOAD = 0x56
};

/* where an instruction comes from */
enum where { IN_PACKET, IN_CALL, IN_PROGRAM };

/* instruction flags: valid in host code; invalid in stored code called by host code */
#define F_HOST 1
#define F_NOT_CALLED 2

struct shape {
	uint32_t k, lit, flags;
};

struct ins {
	uint8_t op;
	uint32_t size, lit, n, len;
	const uint8_t *data;
	uint32_t w[12]; /* operand word addresses */
	uint32_t v[12]; /* operand values */
};

static uint32_t get16(const uint8_t *p)
{
	return (uint32_t)p[0] | (uint32_t)p[1] << 8;
}

static uint32_t get24(const uint8_t *p)
{
	return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16;
}

static uint32_t get32(const uint8_t *p)
{
	return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static void put16(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
}

static void put32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
	p[2] = (uint8_t)(v >> 16);
	p[3] = (uint8_t)(v >> 24);
}

static int32_t s32(uint32_t v)
{
	return v < 0x80000000u ? (int32_t)v : -(int32_t)~v - 1;
}

static uint32_t lowest(uint32_t m)
{
	uint32_t i = 0;

	while (!(m & 1)) {
		m >>= 1;
		i++;
	}
	return i;
}

/* operand count k, literal bytes and flags of an opcode; 0 if invalid */
static int shape(uint8_t op, struct shape *s)
{
	s->k = 0;
	s->lit = 0;
	s->flags = F_HOST;
	switch (op) {
	case OP_WRITE:
		s->lit = 4;
		return 1;
	case OP_WRITE_RLE:
		s->lit = 6;
		return 1;
	case OP_COPY:
		s->k = 3;
		return 1;
	case OP_UNPACK:
		s->k = 4;
		return 1;
	case OP_MOV:
		s->k = 2;
		return 1;
	case OP_JMP:
		s->lit = 3;
		s->flags = 0;
		return 1;
	case OP_CALL:
		s->lit = 3;
		return 1;
	case OP_RET:
		s->flags = 0;
		return 1;
	case OP_END:
		return 1;
	case OP_HALT:
		s->flags = F_NOT_CALLED;
		return 1;
	case OP_SLEEP_UNTIL:
	case OP_RAISE:
		s->k = 1;
		return 1;
	case OP_WAIT_EVENT:
		s->k = 2;
		s->flags = F_NOT_CALLED;
		return 1;
	case OP_NOTIFY:
	case OP_SYS_READ:
	case OP_SYS_WRITE:
		s->k = 2;
		return 1;
	case OP_FILL:
	case OP_LINE:
		s->k = 7;
		return 1;
	case OP_BLIT:
		s->k = 12;
		return 1;
	case OP_GLYPHS:
		s->k = 9;
		return 1;
	case OP_DRAW:
		s->k = 8;
		return 1;
	case OP_VIDEO:
		s->k = 10;
		return 1;
	case OP_ASSET_LOAD:
		s->k = 4;
		return 1;
	}
	if ((op >= OP_ADD && op <= OP_SAR) || (op >= OP_LD32 && op <= OP_ST4)) {
		s->k = 3;
		return 1;
	}
	if (op >= OP_BEQ && op <= OP_BGES) {
		s->k = 2;
		s->lit = 3;
		s->flags = 0;
		return 1;
	}
	return 0;
}

/* §2.1: opcode, k operand nibbles (odd k padded with 0), literal fields, one u16 word index per
 * escape nibble (or the data bytes of WRITE and WRITE_RLE) */
static int decode(const struct vm *vm, const uint8_t *c, uint32_t avail, int where, struct ins *in)
{
	struct shape s;
	uint32_t nb, fixed, pos, e = 0;
	uint8_t nib[12];

	in->op = c[0];
	if (!shape(c[0], &s)) {
		return VM_E_OP;
	}
	if ((where == IN_PACKET && !(s.flags & F_HOST)) || (where == IN_CALL && (s.flags & F_NOT_CALLED))) {
		return VM_E_OP;
	}
	nb = (s.k + 1) / 2;
	fixed = 1 + nb + s.lit;
	if (avail < fixed) {
		return VM_E_MALFORMED;
	}
	for (uint32_t i = 0; i < s.k; i++) {
		nib[i] = (i & 1) ? (uint8_t)(c[1 + i / 2] & 15) : (uint8_t)(c[1 + i / 2] >> 4);
		e += nib[i] == 15;
	}
	if ((s.k & 1) && (c[nb] & 15)) {
		return VM_E_MALFORMED;
	}
	in->size = fixed + 2 * e;
	if (c[0] == OP_WRITE) {
		in->lit = get24(c + 1);
		in->n = c[4];
		in->data = c + 5;
		in->size = 5 + in->n;
	} else if (c[0] == OP_WRITE_RLE) {
		in->lit = get24(c + 1);
		in->len = get16(c + 4);
		in->n = c[6];
		in->data = c + 7;
		in->size = 7 + in->n;
	} else if (s.lit) {
		in->lit = get24(c + 1 + nb);
	}
	if (avail < in->size) {
		return VM_E_MALFORMED;
	}
	pos = fixed;
	for (uint32_t i = 0; i < s.k; i++) {
		if (nib[i] == 15) {
			in->w[i] = 4 * get16(c + pos);
			pos += 2;
		} else {
			in->w[i] = 4u * nib[i];
		}
		in->v[i] = get32(vm->arena + in->w[i]);
	}
	return 0;
}

static uint32_t alu(uint8_t op, uint32_t a, uint32_t b)
{
	switch (op) {
	case OP_ADD:
		return a + b;
	case OP_SUB:
		return a - b;
	case OP_MUL:
		return a * b;
	case OP_DIVU:
		return b ? a / b : 0;
	case OP_MODU:
		return b ? a % b : a;
	case OP_DIVS:
		if (b == 0) {
			return 0;
		}
		if (a == 0x80000000u && b == 0xFFFFFFFFu) {
			return a;
		}
		return (uint32_t)(s32(a) / s32(b));
	case OP_MODS:
		if (b == 0) {
			return a;
		}
		if (a == 0x80000000u && b == 0xFFFFFFFFu) {
			return 0;
		}
		return (uint32_t)(s32(a) % s32(b));
	case OP_AND:
		return a & b;
	case OP_OR:
		return a | b;
	case OP_XOR:
		return a ^ b;
	case OP_SHL:
		return a << (b & 31);
	case OP_SHR:
		return a >> (b & 31);
	default:
		return a < 0x80000000u ? a >> (b & 31) : ~(~a >> (b & 31));
	}
}

/* LDw d, base, i and STw base, i, a (§2.4), addresses computed exactly (no 32-bit wrap) */
static int load_store(struct vm *vm, const struct ins *in, uint32_t *detail)
{
	uint8_t *a = vm->arena;
	int store = in->op >= OP_ST32;
	uint32_t bytes = 4u >> ((in->op - OP_LD32) & 3);
	uint32_t base = store ? in->v[0] : in->v[1], i = store ? in->v[1] : in->v[2], val = in->v[2];
	uint64_t at;

	if (bytes == 0) {
		uint64_t nib = 2 * (uint64_t)base + i;

		at = nib / 2;
		if (at >= VM_ARENA_SIZE) {
			*detail = (uint32_t)at;
			return VM_E_ARENA;
		}
		if (!store) {
			put32(a + in->w[0], (nib & 1) ? (uint32_t)(a[at] & 15) : (uint32_t)(a[at] >> 4));
		} else if (nib & 1) {
			a[at] = (uint8_t)((a[at] & 0xF0) | (val & 15));
		} else {
			a[at] = (uint8_t)((a[at] & 0x0F) | (val & 15) << 4);
		}
		return 0;
	}
	at = base + (uint64_t)i * bytes;
	if (at + bytes > VM_ARENA_SIZE) {
		*detail = (uint32_t)at;
		return VM_E_ARENA;
	}
	if (store) {
		for (uint32_t k = 0; k < bytes; k++) {
			a[at + k] = (uint8_t)(val >> (8 * k));
		}
	} else {
		uint32_t r = 0;

		for (uint32_t k = 0; k < bytes; k++) {
			r |= (uint32_t)a[at + k] << (8 * k);
		}
		put32(a + in->w[0], r);
	}
	return 0;
}

static int unpack(uint8_t *a, const uint32_t *v, uint32_t *detail)
{
	uint32_t dst = v[0], src = v[1], n = v[2], len = v[3];

	if ((uint64_t)src + n > VM_ARENA_SIZE) {
		*detail = src;
		return VM_E_ARENA;
	}
	if ((uint64_t)dst + len > VM_ARENA_SIZE) {
		*detail = dst;
		return VM_E_ARENA;
	}
	if (n && len && (uint64_t)src < (uint64_t)dst + len && (uint64_t)dst < (uint64_t)src + n) {
		*detail = dst;
		return VM_E_ARENA;
	}
	if (!vm_packbits_ok(a + src, n, len)) {
		return VM_E_FORMAT;
	}
	vm_packbits_decode(a + src, n, a + dst);
	return 0;
}

static void fire(struct vm *vm, uint32_t id)
{
	uint16_t bit = (uint16_t)(1u << id);

	if (vm->state != VM_RUNNING) {
		return;
	}
	if (vm->pending & bit) {
		vm->coalesced++;
	} else {
		vm->pending |= bit;
	}
}

/* INPUT (§6.3): one event per firing of a trigger in the EVENTS mask; the count includes drops */
static void input(struct vm *vm, uint32_t id, uint32_t value)
{
	uint8_t body[7];

	if (!(vm->events >> id & 1)) {
		return;
	}
	vm->input_count++;
	put16(body, vm->input_count);
	body[2] = (uint8_t)id;
	put32(body + 3, value);
	vm->host->event(vm->host->ctx, VM_EV_INPUT, body, 7);
}

static uint32_t case_bits(const uint32_t *cs)
{
	return (cs[0] == 1 ? 1u : 0) | (cs[1] == 1 ? 2u : 0) | (cs[2] == 1 ? 4u : 0) |
	       (cs[3] <= 100 ? cs[3] : 0xFFu) << 8;
}

static const uint8_t case_ids[4] = {VM_P_IN_CASE, VM_P_CASE_LID, VM_P_CASE_CHARGING, VM_P_CASE_BATT_PCT};

/* the INPUT value of a trigger that fired without one of its own (RAISE, poll) */
static uint32_t input_value(struct vm *vm, uint32_t id)
{
	const struct vm_host *h = vm->host;
	uint32_t cs[4];

	switch (id) {
	case VM_T_TILT:
		return h->prop(h->ctx, VM_P_PITCH);
	case VM_T_LIGHT:
		return h->prop(h->ctx, VM_P_LIGHT);
	case VM_T_BATTERY:
		return h->prop(h->ctx, VM_P_BATT_PCT);
	case VM_T_CASE:
		for (int i = 0; i < 4; i++) {
			cs[i] = h->prop(h->ctx, case_ids[i]);
		}
		return case_bits(cs);
	case VM_T_WEAR:
		return h->prop(h->ctx, VM_P_WORN);
	case VM_T_TOUCH:
		return h->prop(h->ctx, VM_P_TOUCH_GESTURE);
	case VM_T_PEER:
		return h->prop(h->ctx, VM_P_PEER_MSG);
	}
	return 0;
}

static void fire_input(struct vm *vm, uint32_t id, uint32_t value)
{
	if (vm->state == VM_RUNNING) {
		fire(vm, id);
		input(vm, id, value);
	}
}

static void drop_program(struct vm *vm)
{
	vm->task.state = VM_TASK_NONE;
	vm->pending = 0;
	vm->timer = 0;
	vm->timer_armed = 0;
}

static void halt(struct vm *vm, int code, uint32_t detail, uint8_t op, uint32_t addr)
{
	uint8_t b[11];

	b[0] = vm->task.trigger;
	put32(b + 1, addr);
	b[5] = op;
	b[6] = (uint8_t)code;
	put32(b + 7, detail);
	if (code) {
		vm->stats.errors++;
	}
	vm->state = VM_HALTED;
	vm->halt_code = (uint8_t)code;
	vm->halt_addr = addr;
	drop_program(vm);
	vm->host->event(vm->host->ctx, VM_EV_HALT, b, 11);
}

static void end_packet(struct vm *vm)
{
	vm->pkt.state = VM_TASK_NONE;
	vm->code = NULL;
	vm->stats.packets++;
	vm->host->packet_done(vm->host->ctx);
}

/* An error ends host code's packet with ERROR and halts a program with HALT. */
static void fail(struct vm *vm, struct vm_thread *t, uint8_t op, int code, uint32_t detail)
{
	uint8_t b[13];

	if (t != &vm->pkt) {
		halt(vm, code, detail, op, t->pc);
		return;
	}
	put16(b, vm->seq);
	b[2] = (uint8_t)vm->pkt_off;
	b[3] = op;
	b[4] = (uint8_t)code;
	put32(b + 5, detail);
	put32(b + 9, t->in_packet ? 0 : t->pc);
	vm->stats.errors++;
	vm->host->event(vm->host->ctx, VM_EV_ERROR, b, 13);
	end_packet(vm);
}

static int jump(struct vm *vm, struct vm_thread *t, uint32_t target, uint32_t *detail)
{
	if (t->jumps >= VM_JUMPS) {
		return VM_E_BUDGET;
	}
	if (target >= VM_ARENA_SIZE) {
		*detail = target;
		return VM_E_ARENA;
	}
	t->jumps++;
	vm->stats.jumps++;
	t->pc = target;
	return 0;
}

static int call(struct vm *vm, struct vm_thread *t, uint32_t target, uint32_t next, uint32_t *detail)
{
	if (t->jumps >= VM_JUMPS) {
		return VM_E_BUDGET;
	}
	if (t->depth >= VM_DEPTH) {
		return VM_E_STACK;
	}
	if (target >= VM_ARENA_SIZE) {
		*detail = target;
		return VM_E_ARENA;
	}
	t->jumps++;
	vm->stats.jumps++;
	t->ret[t->depth++] = next;
	t->pc = target;
	t->in_packet = 0;
	return 0;
}

static int ret(struct vm *vm, struct vm_thread *t)
{
	if (t->depth == 0) {
		return VM_E_STACK;
	}
	t->depth--;
	t->pc = t->ret[t->depth];
	if (t == &vm->pkt && t->depth == 0) {
		t->in_packet = 1;
	}
	return 0;
}

static int branch(uint8_t op, uint32_t a, uint32_t b)
{
	switch (op) {
	case OP_BEQ:
		return a == b;
	case OP_BNE:
		return a != b;
	case OP_BLTU:
		return a < b;
	case OP_BGEU:
		return a >= b;
	case OP_BLTS:
		return s32(a) < s32(b);
	default:
		return s32(a) >= s32(b);
	}
}

static int read_prop(const struct vm *vm, uint32_t id, uint32_t trigger, uint32_t *out)
{
	const struct vm_host *h = vm->host;

	switch (id) {
	case VM_P_TIME_US:
		*out = h->now(h->ctx);
		return 0;
	case VM_P_TRIGGER:
		*out = trigger;
		return 0;
	case VM_P_PENDING:
		*out = vm->pending;
		return 0;
	case VM_P_TIMER:
		*out = vm->timer;
		return 0;
	case VM_P_MOTION_MG:
		*out = vm->motion_mg;
		return 0;
	case VM_P_PITCH_LO:
		*out = vm->pitch_lo;
		return 0;
	case VM_P_PITCH_HI:
		*out = vm->pitch_hi;
		return 0;
	case VM_P_LIGHT_LO:
		*out = vm->light_lo;
		return 0;
	case VM_P_LIGHT_HI:
		*out = vm->light_hi;
		return 0;
	case VM_P_IMU_RATE:
		*out = vm->imu_rate;
		return 0;
	case VM_P_ARM:
	case VM_P_LINK:
	case VM_P_LINK_INTERVAL:
	case VM_P_BATT_MV:
	case VM_P_BATT_PCT:
	case VM_P_CHARGE:
	case VM_P_IN_CASE:
	case VM_P_WORN:
	case VM_P_BATT_MA:
	case VM_P_BATT_TEMP:
	case VM_P_ACC_X:
	case VM_P_ACC_Y:
	case VM_P_ACC_Z:
	case VM_P_PITCH:
	case VM_P_LIGHT:
	case VM_P_ROLL:
	case VM_P_BRIGHTNESS:
	case VM_P_BRIGHTNESS_MAX:
	case VM_P_BRIGHTNESS_CAP:
	case VM_P_PANEL:
	case VM_P_CASE_LID:
	case VM_P_CASE_CHARGING:
	case VM_P_CASE_BATT_PCT:
	case VM_P_GYR_X:
	case VM_P_GYR_Y:
	case VM_P_GYR_Z:
	case VM_P_GYRO_RATE:
	case VM_P_TOUCH_GESTURE:
	case VM_P_AUTO_BRIGHTNESS:
	case VM_P_PEER_LINK:
	case VM_P_PEER_MSG:
	case VM_P_PEER_T_US:
		*out = h->prop(h->ctx, id);
		return 0;
	}
	return VM_E_ARG;
}

static int write_prop(struct vm *vm, uint32_t id, uint32_t val, uint32_t *detail)
{
	const struct vm_host *h = vm->host;

	switch (id) {
	case VM_P_BRIGHTNESS:
		if (val > 42) {
			*detail = val;
			return VM_E_ARG;
		}
		h->brightness(h->ctx, val);
		return 0;
	case VM_P_PANEL:
		if (val > 2) {
			*detail = val;
			return VM_E_ARG;
		}
		h->panel_power(h->ctx, val);
		return 0;
	case VM_P_TIMER:
		vm->timer = val;
		vm->timer_armed = 1;
		return 0;
	case VM_P_MOTION_MG:
		vm->motion_mg = val;
		return 0;
	case VM_P_PITCH_LO:
		vm->pitch_lo = val;
		return 0;
	case VM_P_PITCH_HI:
		vm->pitch_hi = val;
		return 0;
	case VM_P_LIGHT_LO:
		vm->light_lo = val;
		return 0;
	case VM_P_LIGHT_HI:
		vm->light_hi = val;
		return 0;
	case VM_P_IMU_RATE:
		if (val != 0 && val != 13 && val != 26 && val != 52 && val != 104) {
			*detail = val;
			return VM_E_ARG;
		}
		vm->imu_rate = val;
		h->imu_rate(h->ctx, val);
		return 0;
	case VM_P_GYRO_RATE:
	case VM_P_AUTO_BRIGHTNESS:
	case VM_P_BRIGHTNESS_MAX:
		if (id == VM_P_GYRO_RATE ? val != 0 && val != 13 && val != 26 && val != 52 && val != 104
		    : val > (id == VM_P_BRIGHTNESS_MAX ? 42u : 1u)) {
			*detail = val;
			return VM_E_ARG;
		}
		/* fall through */
	case VM_P_PEER_LINK:
	case VM_P_PEER_SEND: {
		int err = h->prop_write ? h->prop_write(h->ctx, id, val) : VM_E_ARG;

		if (err) {
			*detail = id;
		}
		return err;
	}
	}
	*detail = id;
	return VM_E_ARG;
}

/* x of the microphone indicator: the outer top corner, (620, 4) right arm, (4, 4) left arm */
static int32_t mic_x(const struct vm *vm)
{
	return vm->host->prop(vm->host->ctx, VM_P_ARM) == 1 ? 4 : 620;
}

/* the microphone indicator over a panel row of w pixels from (x, y) */
static void indicator(const struct vm *vm, int32_t x, int32_t y, int32_t w, uint8_t *row)
{
	int32_t ix;

	if (!vm->mic || y < 4 || y >= 20) {
		return;
	}
	ix = mic_x(vm);
	for (int32_t i = ix; i < ix + 16; i++) {
		if (i >= x && i < x + w) {
			row[(i - x) / 2] |= (uint8_t)(((i - x) & 1) ? 0x0F : 0xF0);
		}
	}
}

static int draw(struct vm *vm, const uint32_t *v, uint32_t *detail)
{
	const struct vm_host *h = vm->host;
	struct vm_draw d;
	int err = vm_draw_clip(v, &d, detail);

	if (err) {
		return err;
	}
	if (vm->safe) {
		return VM_E_SAFE;
	}
	if (d.w == 0 || h->panel_begin(h->ctx, d.x, d.y, d.w, d.h) != 0) {
		return 0;
	}
	for (int32_t r = 0; r < d.h; r++) {
		vm_draw_row(vm->arena, &d, r, vm->row);
		indicator(vm, d.x, d.y + r, d.w, vm->row);
		h->panel_row(h->ctx, d.x, d.y + r, d.w, vm->row);
	}
	h->panel_end(h->ctx);
	vm->stats.draws++;
	return 0;
}

/* VIDEO: a changed span of a frame row to the panel, which is prepared at the first visible one */
struct video_out {
	struct vm *vm;
	int64_t x, y;
	uint32_t level;
	int32_t bx, by, bw, bh;
	int state; /* 0 not begun, 1 begun, -1 no panel */
};

static void video_span(void *c, uint32_t r, uint32_t x0, uint32_t x1, const uint8_t *row)
{
	struct video_out *o = (struct video_out *)c;
	struct vm *vm = o->vm;
	const struct vm_host *h = vm->host;
	int64_t y = o->y + r, a = o->x + x0, b = o->x + x1;

	if (x0 == x1) {
		h->now(h->ctx); /* every 16 rows: g1v0 feeds its health beat from now */
		return;
	}
	if (y < 0 || y >= VM_PANEL_H || o->state < 0) {
		return;
	}
	a = a < 0 ? 0 : a;
	b = b > VM_PANEL_W ? VM_PANEL_W : b;
	if (a >= b) {
		return;
	}
	if (o->state == 0) {
		o->state = h->panel_begin(h->ctx, o->bx, o->by, o->bw, o->bh) == 0 ? 1 : -1;
		if (o->state < 0) {
			return;
		}
	}
	vm_video_expand(row, (uint32_t)(a - o->x), (uint32_t)(b - o->x), o->level, vm->row);
	indicator(vm, (int32_t)a, (int32_t)y, (int32_t)(b - a), vm->row);
	(h->panel_span ? h->panel_span : h->panel_row)(h->ctx, (int32_t)a, (int32_t)y, (int32_t)(b - a), vm->row);
}

static int video(struct vm *vm, const uint32_t *v, uint32_t *detail)
{
	struct video_out o;
	int64_t x0, y0, x1, y1;
	int err = vm_video_check(vm->arena, v, detail);

	if (err) {
		return err;
	}
	if (vm->safe) {
		return VM_E_SAFE;
	}
	o.vm = vm;
	o.x = s32(v[4]);
	o.y = s32(v[5]);
	o.level = v[6];
	x0 = o.x < 0 ? 0 : o.x;
	y0 = o.y < 0 ? 0 : o.y;
	x1 = o.x + v[2] > VM_PANEL_W ? VM_PANEL_W : o.x + v[2];
	y1 = o.y + v[3] > VM_PANEL_H ? VM_PANEL_H : o.y + v[3];
	o.state = x0 < x1 && y0 < y1 ? 0 : -1;
	o.bx = (int32_t)x0;
	o.by = (int32_t)y0;
	o.bw = (int32_t)(x1 - x0);
	o.bh = (int32_t)(y1 - y0);
	vm_video_decode(vm->arena, v, &vm->video, video_span, &o);
	if (o.state == 1) {
		vm->host->panel_end(vm->host->ctx);
		vm->stats.draws++;
	}
	return 0;
}

/* ASSET_LOAD id, dst, off, n: bytes off..off+n-1 of the stored asset whose 16-byte ID is at id to dst; the
 * platform reads it (asset_read) after the arena ranges are checked */
static int asset_load(struct vm *vm, const uint32_t *v, uint32_t *detail)
{
	uint8_t id[16];
	int err;

	if ((uint64_t)v[0] + 16 > VM_ARENA_SIZE) {
		*detail = v[0];
		return VM_E_ARENA;
	}
	if ((uint64_t)v[1] + v[3] > VM_ARENA_SIZE) {
		*detail = v[1];
		return VM_E_ARENA;
	}
	if (!vm->host->asset_read) {
		return VM_E_ASSET;
	}
	memcpy(id, vm->arena + v[0], 16);
	err = vm->host->asset_read(vm->host->ctx, id, v[2], vm->arena + v[1], v[3]);
	if (err == VM_E_ARENA) {
		*detail = v[2];
	}
	return err == 0 || err == VM_E_ARENA ? err : VM_E_ASSET;
}

static void notify(struct vm *vm, uint8_t source, uint32_t a, uint32_t b)
{
	uint8_t body[11];

	vm->notify_count++;
	put16(body, vm->notify_count);
	body[2] = source;
	put32(body + 3, a);
	put32(body + 7, b);
	if (vm->host->event(vm->host->ctx, VM_EV_NOTIFY, body, 11) != 0) {
		vm->notify_dropped++;
	}
}

/* Runs the decoded instruction of thread t. On an error nothing has changed and t->pc still points
 * at the instruction. */
static int exec(struct vm *vm, struct vm_thread *t, int where, const struct ins *in, uint32_t *detail)
{
	uint8_t *a = vm->arena;
	const uint32_t *v = in->v;
	uint32_t next = t->pc + in->size, r;
	uint8_t source = where == IN_PROGRAM ? t->trigger : 0xFF;
	int err = 0;

	switch (in->op) {
	case OP_WRITE:
		if ((uint64_t)in->lit + in->n > VM_ARENA_SIZE) {
			*detail = in->lit;
			return VM_E_ARENA;
		}
		memmove(a + in->lit, in->data, in->n);
		break;
	case OP_WRITE_RLE: {
		uint8_t buf[255];

		if ((uint64_t)in->lit + in->len > VM_ARENA_SIZE) {
			*detail = in->lit;
			return VM_E_ARENA;
		}
		if (!vm_packbits_ok(in->data, in->n, in->len)) {
			return VM_E_FORMAT;
		}
		memcpy(buf, in->data, in->n);
		vm_packbits_decode(buf, in->n, a + in->lit);
		break;
	}
	case OP_COPY:
		if ((uint64_t)v[1] + v[2] > VM_ARENA_SIZE) {
			*detail = v[1];
			return VM_E_ARENA;
		}
		if ((uint64_t)v[0] + v[2] > VM_ARENA_SIZE) {
			*detail = v[0];
			return VM_E_ARENA;
		}
		memmove(a + v[0], a + v[1], v[2]);
		break;
	case OP_UNPACK:
		err = unpack(a, v, detail);
		break;
	case OP_MOV:
		put32(a + in->w[0], v[1]);
		break;
	case OP_JMP:
		return jump(vm, t, in->lit, detail);
	case OP_CALL:
		return call(vm, t, in->lit, next, detail);
	case OP_RET:
		return ret(vm, t);
	case OP_END:
		if (t == &vm->pkt) {
			end_packet(vm);
		} else {
			t->state = VM_TASK_NONE;
		}
		return 0;
	case OP_HALT:
		halt(vm, 0, 0, OP_HALT, t->pc);
		return 0;
	case OP_SLEEP_UNTIL: {
		uint32_t now = vm->host->now(vm->host->ctx);
		int32_t d = s32(v[0] - now);

		if (d > 0) {
			t->wake = now + (uint32_t)(d < VM_SLEEP_MAX_US ? d : VM_SLEEP_MAX_US);
			t->state = VM_TASK_SLEEP;
		} else if (t == &vm->pkt) {
			vm->stats.late++;
		}
		break;
	}
	case OP_WAIT_EVENT:
		if (v[1] == 0 || v[1] > 0xFFFF) {
			*detail = v[1];
			return VM_E_ARG;
		}
		if (vm->pending & v[1]) {
			r = lowest(vm->pending & v[1]);
			vm->pending &= (uint16_t)~(1u << r);
			put32(a + in->w[0], r);
		} else {
			t->state = VM_TASK_WAIT;
			t->wait_mask = (uint16_t)v[1];
			t->wait_word = in->w[0];
		}
		break;
	case OP_RAISE:
		if (v[0] > 15) {
			*detail = v[0];
			return VM_E_ARG;
		}
		fire_input(vm, v[0], input_value(vm, v[0]));
		break;
	case OP_NOTIFY:
		notify(vm, source, v[0], v[1]);
		break;
	case OP_SYS_READ:
		err = read_prop(vm, v[1], source, &r);
		if (err) {
			*detail = v[1];
			return err;
		}
		put32(a + in->w[0], r);
		break;
	case OP_SYS_WRITE:
		err = write_prop(vm, v[0], v[1], detail);
		break;
	case OP_FILL:
		err = vm_fill(a, v, detail);
		break;
	case OP_BLIT:
		err = vm_blit(a, v, detail);
		break;
	case OP_GLYPHS:
		err = vm_glyphs(a, v, detail);
		break;
	case OP_DRAW:
		err = draw(vm, v, detail);
		break;
	case OP_VIDEO:
		err = video(vm, v, detail);
		break;
	case OP_ASSET_LOAD:
		err = asset_load(vm, v, detail);
		break;
	case OP_LINE:
		err = vm_line(a, v, detail);
		break;
	default:
		if (in->op >= OP_BEQ && in->op <= OP_BGES) {
			if (branch(in->op, v[0], v[1])) {
				return jump(vm, t, in->lit, detail);
			}
		} else if (in->op >= OP_LD32 && in->op <= OP_ST4) {
			err = load_store(vm, in, detail);
		} else {
			put32(a + in->w[0], alu(in->op, v[1], v[2]));
		}
		break;
	}
	if (!err) {
		t->pc = next;
	}
	return err;
}

/* one instruction of the current packet (t = &vm->pkt) or of the program's task */
static void step(struct vm *vm, struct vm_thread *t)
{
	struct ins in;
	const uint8_t *c;
	uint32_t avail, detail = 0;
	int where, err;

	if (t == &vm->pkt && t->in_packet) {
		if (t->pc >= vm->code_len) {
			end_packet(vm);
			return;
		}
		where = IN_PACKET;
		c = vm->code + t->pc;
		avail = vm->code_len - t->pc;
		vm->pkt_off = t->pc;
	} else {
		where = t == &vm->pkt ? IN_CALL : IN_PROGRAM;
		if (t->pc >= VM_ARENA_SIZE) {
			vm->stats.instructions++;
			fail(vm, t, 0, VM_E_ARENA, t->pc);
			return;
		}
		c = vm->arena + t->pc;
		avail = VM_ARENA_SIZE - t->pc;
	}
	vm->stats.instructions++;
	err = decode(vm, c, avail, where, &in);
	if (!err) {
		err = exec(vm, t, where, &in, &detail);
	}
	if (err) {
		fail(vm, t, in.op, err, detail);
	}
}

static int range_state(uint32_t v, uint32_t lo, uint32_t hi, int is_signed)
{
	if (v == VM_UNKNOWN) {
		return 0;
	}
	if (is_signed) {
		return s32(lo) <= s32(v) && s32(v) <= s32(hi) ? 1 : 2;
	}
	return lo <= v && v <= hi ? 1 : 2;
}

/* 0 unknown, 1 inside, 2 outside: TILT and LIGHT fire on a change from 1 to 2 */
static void range_check(struct vm *vm, uint8_t *state, uint32_t trigger, uint32_t v, uint32_t lo, uint32_t hi,
			int is_signed)
{
	int s = range_state(v, lo, hi, is_signed);

	if (s == 0) {
		return;
	}
	if (*state == 1 && s == 2) {
		fire(vm, trigger);
	}
	*state = (uint8_t)s;
}

/* The sensor triggers the VM derives from property values, evaluated when the executor goes idle. */
static void sense(struct vm *vm, int fire_changes)
{
	const struct vm_host *h = vm->host;
	uint32_t pct = h->prop(h->ctx, VM_P_BATT_PCT), charge = h->prop(h->ctx, VM_P_CHARGE);
	uint32_t worn = h->prop(h->ctx, VM_P_WORN), cs[4];
	uint32_t pitch = h->prop(h->ctx, VM_P_PITCH), light = h->prop(h->ctx, VM_P_LIGHT);
	int case_changed = 0;

	for (int i = 0; i < 4; i++) {
		cs[i] = h->prop(h->ctx, case_ids[i]);
		case_changed |= cs[i] != vm->last_case[i];
		vm->last_case[i] = cs[i];
	}
	if (!fire_changes) {
		vm->tilt_in = (uint8_t)range_state(pitch, vm->pitch_lo, vm->pitch_hi, 1);
		vm->light_in = (uint8_t)range_state(light, vm->light_lo, vm->light_hi, 0);
	} else {
		if (pct != vm->last_pct || charge != vm->last_charge) {
			fire(vm, VM_T_BATTERY);
		}
		if (case_changed) {
			fire(vm, VM_T_CASE);
		}
		if (worn != vm->last_worn) {
			fire(vm, VM_T_WEAR);
		}
		range_check(vm, &vm->tilt_in, VM_T_TILT, pitch, vm->pitch_lo, vm->pitch_hi, 1);
		range_check(vm, &vm->light_in, VM_T_LIGHT, light, vm->light_lo, vm->light_hi, 0);
	}
	vm->last_pct = pct;
	vm->last_charge = charge;
	vm->last_worn = worn;
}

/* The same checks for the INPUT events, with baselines of their own taken at vm_set_events, so they
 * are reported whether or not a program runs. */
static void sense_input(struct vm *vm, int report)
{
	const struct vm_host *h = vm->host;
	uint32_t pct = h->prop(h->ctx, VM_P_BATT_PCT), charge = h->prop(h->ctx, VM_P_CHARGE);
	uint32_t worn = h->prop(h->ctx, VM_P_WORN), cs[4];
	uint32_t pitch = h->prop(h->ctx, VM_P_PITCH), light = h->prop(h->ctx, VM_P_LIGHT);
	int case_changed = 0, s;

	for (int i = 0; i < 4; i++) {
		cs[i] = h->prop(h->ctx, case_ids[i]);
		case_changed |= cs[i] != vm->ev_case[i];
		vm->ev_case[i] = cs[i];
	}
	if (report) {
		if (pct != vm->ev_pct || charge != vm->ev_charge) {
			input(vm, VM_T_BATTERY, pct);
		}
		if (case_changed) {
			input(vm, VM_T_CASE, case_bits(cs));
		}
		if (worn != vm->ev_worn) {
			input(vm, VM_T_WEAR, worn);
		}
	}
	s = range_state(pitch, vm->pitch_lo, vm->pitch_hi, 1);
	if (s) {
		if (report && vm->ev_tilt_in == 1 && s == 2) {
			input(vm, VM_T_TILT, pitch);
		}
		vm->ev_tilt_in = (uint8_t)s;
	}
	s = range_state(light, vm->light_lo, vm->light_hi, 0);
	if (s) {
		if (report && vm->ev_light_in == 1 && s == 2) {
			input(vm, VM_T_LIGHT, light);
		}
		vm->ev_light_in = (uint8_t)s;
	}
	vm->ev_pct = pct;
	vm->ev_charge = charge;
	vm->ev_worn = worn;
}

static void cap_roll(struct vm *vm, uint32_t now)
{
	uint32_t elapsed = now - vm->period_start, k;
	uint64_t credit;

	if (elapsed < VM_PERIOD_US) {
		return;
	}
	k = elapsed / VM_PERIOD_US;
	vm->period_start += k * VM_PERIOD_US;
	vm->last_run_us = k == 1 ? vm->run_us : 0;
	vm->run_us = 0;
	credit = (uint64_t)k * VM_CAP_US;
	vm->charged_us = vm->charged_us > credit ? (uint32_t)(vm->charged_us - credit) : 0;
	vm->capped = 0;
}

/* Run-time cap (§5.4): 0 if a program instruction may run now, else the wait for the next period.
 * Time beyond 30 ms (a long DRAW) is charged to the following periods. */
static uint32_t cap_wait(struct vm *vm, uint32_t now)
{
	cap_roll(vm, now);
	if (vm->charged_us < VM_CAP_US) {
		return 0;
	}
	if (!vm->capped) {
		vm->capped = 1;
		vm->cap_pauses++;
	}
	return vm->period_start + VM_PERIOD_US - now;
}

static uint32_t timer_wait(const struct vm *vm, uint32_t now)
{
	int32_t d;

	if (vm->state != VM_RUNNING || !vm->timer_armed) {
		return VM_FOREVER;
	}
	d = s32(vm->timer - now);
	return d > 0 ? (uint32_t)d : 1;
}

static uint32_t handler(const struct vm *vm, uint32_t id)
{
	return get32(vm->arena + vm->table + 4 * id);
}

/* 0 if the program's task can run an instruction now, else how long to wait */
static uint32_t task_ready(struct vm *vm, uint32_t now)
{
	struct vm_thread *t = &vm->task;
	uint32_t wait, id;

	if (t->state == VM_TASK_NONE) {
		for (id = 0; id < 16; id++) {
			if ((vm->pending >> id & 1) && handler(vm, id) != 0) {
				break;
			}
		}
		if (id == 16) {
			return timer_wait(vm, now);
		}
		wait = cap_wait(vm, now);
		if (wait) {
			return wait;
		}
		vm->pending &= (uint16_t)~(1u << id);
		t->pc = handler(vm, id);
		t->state = VM_TASK_RUN;
		t->depth = 0;
		t->jumps = 0;
		t->trigger = (uint8_t)id;
		t->in_packet = 0;
		vm->tasks++;
		return 0;
	}
	if (t->state == VM_TASK_SLEEP) {
		int32_t d = s32(t->wake - now);

		if (d > 0) {
			wait = timer_wait(vm, now);
			return (uint32_t)d < wait ? (uint32_t)d : wait;
		}
		t->state = VM_TASK_RUN;
		t->jumps = 0;
	} else if (t->state == VM_TASK_WAIT) {
		uint32_t got = vm->pending & t->wait_mask;

		if (!got) {
			return timer_wait(vm, now);
		}
		id = lowest(got);
		vm->pending &= (uint16_t)~(1u << id);
		put32(vm->arena + t->wait_word, id);
		t->state = VM_TASK_RUN;
		t->jumps = 0;
	}
	return cap_wait(vm, now);
}

/* Host code first: before each program instruction any ready packet runs; while a packet runs or
 * sleeps no program instruction runs (§5.4). */
uint32_t vm_run(struct vm *vm)
{
	const struct vm_host *h = vm->host;

	for (;;) {
		uint32_t news = h->poll(h->ctx), now, wait, len, end;
		const uint8_t *p;

		for (uint32_t id = 0; id < 16 && (news & 0xFFFFu); id++) {
			if (news >> id & 1) {
				fire(vm, id);
				input(vm, id, input_value(vm, id));
			}
		}
		if (news & VM_YIELD) {
			return 0;
		}
		/* the clock only where it is used: host code reads it itself (SLEEP_UNTIL, TIME_US) */
		now = 0;
		if ((vm->state == VM_RUNNING && vm->timer_armed) || vm->pkt.state != VM_TASK_RUN) {
			now = h->now(h->ctx);
		}
		if (vm->state == VM_RUNNING && vm->timer_armed && s32(vm->timer - now) <= 0) {
			vm->timer_armed = 0;
			fire_input(vm, VM_T_TIMER, 0);
		}
		if (vm->pkt.state != VM_TASK_RUN && vm->task.state != VM_TASK_RUN) {
			if (vm->state == VM_RUNNING) {
				sense(vm, 1);
			}
			if (vm->events) {
				sense_input(vm, 1);
			}
		}
		if (vm->pkt.state == VM_TASK_NONE) {
			p = h->packet(h->ctx, &len);
			if (p) {
				vm->seq = (uint16_t)(len >= 2 ? get16(p) : 0);
				vm->code = len >= 2 ? p + 2 : p;
				vm->code_len = len >= 2 ? len - 2 : 0;
				vm->pkt_off = 0;
				vm->pkt.pc = 0;
				vm->pkt.in_packet = 1;
				vm->pkt.depth = 0;
				vm->pkt.jumps = 0;
				vm->pkt.state = VM_TASK_RUN;
			}
		}
		if (vm->pkt.state == VM_TASK_SLEEP) {
			int32_t d = s32(vm->pkt.wake - now);

			if (d > 0) {
				return (uint32_t)d;
			}
			vm->pkt.state = VM_TASK_RUN;
			vm->pkt.jumps = 0;
		}
		if (vm->pkt.state == VM_TASK_RUN) {
			step(vm, &vm->pkt);
			continue;
		}
		if (vm->state != VM_RUNNING) {
			return VM_FOREVER;
		}
		wait = task_ready(vm, now);
		if (wait) {
			return wait;
		}
		step(vm, &vm->task);
		end = h->now(h->ctx);
		vm->run_us += end - now;
		vm->charged_us += end - now;
	}
}

static void default_settings(struct vm *vm)
{
	vm->motion_mg = 0;
	vm->pitch_lo = (uint32_t)-90;
	vm->pitch_hi = 90;
	vm->light_lo = 0;
	vm->light_hi = 0xFFFFFFFFu;
	vm->imu_rate = 13;
}

void vm_init(struct vm *vm, uint8_t *arena, const struct vm_host *host, int safe)
{
	memset(vm, 0, sizeof(*vm));
	vm->arena = arena;
	vm->host = host;
	vm->safe = safe != 0;
	vm->task.trigger = 0xFF;
	vm->pkt.trigger = 0xFF;
	default_settings(vm);
	vm->period_start = host->now(host->ctx);
	memset(arena, 0, VM_ARENA_SIZE);
}

int vm_start(struct vm *vm, uint32_t table)
{
	if (vm->safe) {
		return VM_E_SAFE;
	}
	if ((uint64_t)table + 64 > VM_ARENA_SIZE) {
		return VM_E_ARENA;
	}
	drop_program(vm);
	vm->state = VM_RUNNING;
	vm->table = table;
	vm->pending = 1u << VM_T_START;
	vm->halt_code = 0;
	vm->halt_addr = 0;
	vm->tasks = 0;
	vm->coalesced = 0;
	vm->cap_pauses = 0;
	sense(vm, 0);
	input(vm, VM_T_START, 0);
	return 0;
}

void vm_stop(struct vm *vm)
{
	vm->state = VM_STOPPED;
	drop_program(vm);
}

void vm_reset_session(struct vm *vm, int clear_arena)
{
	vm_stop(vm);
	vm->table = 0;
	vm->halt_code = 0;
	vm->halt_addr = 0;
	vm->tasks = 0;
	vm->coalesced = 0;
	vm->cap_pauses = 0;
	vm->notify_count = 0;
	vm->notify_dropped = 0;
	vm->input_count = 0;
	vm->pkt.state = VM_TASK_NONE;
	vm->code = NULL;
	default_settings(vm);
	vm->host->imu_rate(vm->host->ctx, vm->imu_rate);
	if (clear_arena) {
		memset(vm->arena, 0, VM_ARENA_SIZE);
	}
}

void vm_set_mic(struct vm *vm, int on)
{
	const struct vm_host *h = vm->host;
	int32_t x = mic_x(vm);
	uint8_t px[8];

	vm->mic = on != 0;
	if (!on && h->prop(h->ctx, VM_P_PANEL) != 1) {
		return;
	}
	if (h->panel_begin(h->ctx, x, 4, 16, 16) != 0) {
		return;
	}
	memset(px, on ? 0xFF : 0, sizeof(px));
	for (int32_t y = 4; y < 20; y++) {
		h->panel_row(h->ctx, x, y, 16, px);
	}
	h->panel_end(h->ctx);
}

int vm_prop_set(struct vm *vm, uint32_t id, uint32_t value)
{
	uint32_t detail;

	return write_prop(vm, id, value, &detail);
}

int vm_prop(const struct vm *vm, uint32_t id, uint32_t *value)
{
	return read_prop(vm, id, vm->task.state != VM_TASK_NONE ? vm->task.trigger : 0xFF, value);
}

void vm_status(const struct vm *vm, uint8_t *out)
{
	const struct vm_thread *t = &vm->task;
	uint32_t elapsed = vm->host->now(vm->host->ctx) - vm->period_start, busy = 0, left = 0;
	int task = t->state != VM_TASK_NONE;

	if (elapsed < VM_PERIOD_US) {
		busy = vm->last_run_us;
	} else if (elapsed < 2 * VM_PERIOD_US) {
		busy = vm->run_us;
	}
	if (task) {
		left = VM_JUMPS - t->jumps;
	}
	memset(out, 0, 32);
	out[0] = vm->state;
	out[1] = task ? t->trigger : 0xFF;
	put16(out + 2, vm->pending);
	put32(out + 4, vm->table);
	put32(out + 8, vm->state == VM_HALTED ? vm->halt_addr : task ? t->pc : 0);
	out[12] = t->state;
	out[13] = (uint8_t)(left > 255 ? 255 : left);
	out[14] = task ? t->depth : 0;
	out[15] = vm->halt_code;
	put32(out + 16, vm->timer);
	put32(out + 20, vm->tasks);
	put16(out + 24, vm->coalesced);
	put16(out + 26, vm->notify_dropped);
	out[28] = vm->timer_armed;
	put16(out + 29, vm->cap_pauses);
	out[31] = (uint8_t)(busy / 1000 > 255 ? 255 : busy / 1000);
}

uint16_t vm_wants(const struct vm *vm)
{
	uint16_t bits = 0;

	if (vm->state != VM_RUNNING) {
		return 0;
	}
	for (uint32_t i = 0; i < 16; i++) {
		if (vm->table + 4 * i + 4 <= VM_ARENA_SIZE && get32(vm->arena + vm->table + 4 * i)) {
			bits |= (uint16_t)(1u << i);
		}
	}
	if (vm->task.state == VM_TASK_WAIT) {
		bits |= vm->task.wait_mask;
	}
	return bits;
}

void vm_set_events(struct vm *vm, uint16_t mask)
{
	vm->events = mask;
	sense_input(vm, 0);
}

void vm_trigger(struct vm *vm, uint32_t id, uint32_t value)
{
	if (id < 16) {
		fire(vm, id);
		input(vm, id, value);
	}
}

uint32_t vm_distance(const int32_t *prev, const int32_t *cur)
{
	uint64_t sum = 0, r = 0, bit = (uint64_t)1 << 62;

	for (int i = 0; i < 3; i++) {
		int64_t d = (int64_t)cur[i] - prev[i];
		uint64_t m = (uint64_t)(d < 0 ? -d : d);

		if (m >= (1u << 31)) {
			return 0xFFFFFFFFu;
		}
		sum += m * m;
	}
	while (bit > sum) {
		bit >>= 2;
	}
	while (bit) {
		if (sum >= r + bit) {
			sum -= r + bit;
			r = (r >> 1) + bit;
		} else {
			r >>= 1;
		}
		bit >>= 2;
	}
	return (uint32_t)r;
}

int vm_moved(const int32_t *prev, const int32_t *cur, uint32_t mg)
{
	uint64_t lim = (uint64_t)mg * mg, sum = 0;

	if (mg == 0) {
		return 0;
	}
	for (int i = 0; i < 3; i++) {
		int64_t d = (int64_t)cur[i] - prev[i];
		uint64_t m = (uint64_t)(d < 0 ? -d : d);

		if (m > mg || m * m > lim - sum) {
			return 1;
		}
		sum += m * m;
	}
	return 0;
}
