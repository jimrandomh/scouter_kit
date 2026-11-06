/* The executor (VM.md §5.4): one thread runs the host packets and the program on the arena with the
 * portable VM core (scouter_kit/src/vm), host packets first and program instructions between them.
 * The control thread (proto.c) checks a request and posts its action here (VM_START, VM_STOP,
 * RESET_SESSION, MIC); the core yields within one instruction and the action is applied before the
 * next. Power rules (VM.md §6.6): the panel is off at boot and, with no host connected and no running
 * program, 30 s after the last DRAW; with nothing to run the thread blocks (500 ms health beat). */
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/crc.h>
#include <hal/nrf_rtc.h>
#include "g1.h"
#include "vm.h"

LOG_MODULE_REGISTER(exec, LOG_LEVEL_INF);

#define PANEL_IDLE_MS (CONFIG_G1_PANEL_IDLE_S * 1000)
#define BEAT_TICKS 8192   /* 250 ms of RTC1 */

enum req_op { R_START, R_STOP, R_RESET, R_MIC, R_MIC_FAULT, R_MIC_LEASE, R_HANG, R_EVENTS };

struct req {
	uint8_t op;
	uint32_t arg;
};

static uint8_t arena[VM_ARENA_SIZE] __aligned(4);
static struct vm vm;
static k_tid_t self;
static K_SEM_DEFINE(wake, 0, 1);
static K_SEM_DEFINE(reset_done, 0, 1);
K_MSGQ_DEFINE(req_q, sizeof(struct req), 8, 4);
static atomic_t reqs;
static volatile uint16_t reset_flushed;
static volatile bool mic;
static int8_t held = -1;   /* PANEL written while the microphone holds the panel on */
static volatile uint32_t last_draw_us;
static int64_t panel_used_ms;
static uint32_t beat_cnt, polls;
static uint32_t pct = VM_UNKNOWN, pct_stale, sensed[7];
static uint16_t events;
static struct k_spinlock tlock;
static uint16_t trig_bits;
static uint32_t trig_value[16];

static void post(uint8_t op, uint32_t arg)
{
	struct req r = {.op = op, .arg = arg};

	while (k_msgq_put(&req_q, &r, K_NO_WAIT) != 0) {
		k_msleep(1);
	}
	atomic_set(&reqs, 1);
	k_sem_give(&wake);
}

void exec_wake(void)
{
	k_sem_give(&wake);
}

/* BATT_PCT: the voltage estimate of PROTOCOL v4's BATTERY (100 once the charger completed); a change of
 * 1 % is taken after 30 s, so ADC noise at a step does not raise BATTERY every second */
static uint32_t pct_raw(void)
{
	uint8_t cs = pmic_status.charge_status;
	uint16_t mv = pmic_status.vbat_mv;
	uint16_t vterm = pmic_status.vterm_code ? 3800 + 50 * pmic_status.vterm_code : 0;

	if (!pmic_status.present || !mv || vterm <= 3600) {
		return VM_UNKNOWN;
	}
	if (cs & 0x02) {
		return 100;
	}
	return CLAMP(((int)mv - 3500) * 100 / (vterm - 3600), 0, 100);
}

static uint32_t charge(void)
{
	uint8_t cs = pmic_status.charge_status;

	if (!pmic_status.present) {
		return VM_UNKNOWN;
	}
	return (cs & 0x02) ? 2 : (cs & 0x1C) ? 1 : 0;
}

static uint32_t h_prop(void *ctx, uint32_t id)
{
	switch (id) {
	case VM_P_ARM:
		return board_role();
	case VM_P_LINK:
		return proto_link_obeyed() ? 1 : 0;
	case VM_P_LINK_INTERVAL:
		return proto_link_interval_us();
	case VM_P_BATT_MV:
		return pmic_status.present && pmic_status.vbat_mv ? pmic_status.vbat_mv : VM_UNKNOWN;
	case VM_P_BATT_PCT:
		return pct;
	case VM_P_CHARGE:
		return charge();
	case VM_P_IN_CASE:
		return g1_in_case() ? 1 : 0;
	case VM_P_BATT_MA:
		return (uint32_t)power_batt_ma();
	case VM_P_BATT_TEMP: {
		int16_t t = power_batt_temp();

		return t == INT16_MIN ? VM_UNKNOWN : (uint32_t)(int32_t)t;
	}
	case VM_P_BRIGHTNESS:
		return panel_level();
	case VM_P_BRIGHTNESS_MAX:
		return panel_level_max();
	case VM_P_BRIGHTNESS_CAP:
		return panel_cap();
	case VM_P_PANEL:
		return held >= 0 ? held : panel_is_on() ? 1 : panel_in_standby() ? 2 : 0;
	case VM_P_CASE_LID:
	case VM_P_CASE_CHARGING:
	case VM_P_CASE_BATT_PCT:
		return caselink_prop(id);
	case VM_P_ACC_X:
	case VM_P_ACC_Y:
	case VM_P_ACC_Z:
	case VM_P_PITCH:
	case VM_P_ROLL:
		return imu_prop(id);
	case VM_P_WORN:
	case VM_P_TOUCH_GESTURE:
		return touch_prop(id);
	case VM_P_LIGHT:
		return light_prop();
	case VM_P_AUTO_BRIGHTNESS:
		return light_present() ? light_auto() : peer_available() && board_role() ? peer_follow() : VM_UNKNOWN;
	case VM_P_PEER_LINK:
		return peer_available() ? peer_link() : VM_UNKNOWN;
	case VM_P_PEER_MSG:
		return peer_msg(false);
	case VM_P_PEER_T_US:
		return peer_msg(true);
	case VM_P_GYR_X:
	case VM_P_GYR_Y:
	case VM_P_GYR_Z:
		return imu_gyro_prop(id);
	case VM_P_GYRO_RATE:
		return imu_gyro_rate();
	}
	return VM_UNKNOWN;
}

/* PMIC thread, every second; case link after a message: the core evaluates its
 * sensor triggers when the executor goes idle, so the executor is woken when a watched value changed */
void exec_sensors(void)
{
	static const uint8_t ids[7] = {VM_P_BATT_PCT, VM_P_CHARGE, VM_P_IN_CASE, VM_P_CASE_LID,
				       VM_P_CASE_CHARGING, VM_P_CASE_BATT_PCT, VM_P_WORN};
	uint32_t raw = pct_raw();
	bool changed = false;

	touch_wear_tick();
	if (raw == VM_UNKNOWN || pct == VM_UNKNOWN || raw > pct + 1 || raw + 1 < pct) {
		pct = raw;
		pct_stale = 0;
	} else if (raw == pct) {
		pct_stale = 0;
	} else if (++pct_stale >= 30) {
		pct = raw;
		pct_stale = 0;
	}
	for (int i = 0; i < ARRAY_SIZE(ids); i++) {
		uint32_t v = h_prop(NULL, ids[i]);

		changed |= v != sensed[i];
		sensed[i] = v;
	}
	if (changed) {
		k_sem_give(&wake);
	}
}

/* the exec thread's health beat (3 s limit), every 250 ms of RTC1 */
static void beat(uint32_t c)
{
	if (((c - beat_cnt) & 0xFFFFFF) >= BEAT_TICKS) {
		beat_cnt = c;
		atomic_inc(&hb_exec);
	}
}

/* now_us() costs ~150 cycles (the kernel's timer lock and 64-bit RTC read) and a program reads the clock before
 * and after each instruction: within one vm_run the executor reuses the last value while RTC1 (the kernel's
 * 30.5 µs tick) has not moved, which gives the same value. */
static uint32_t clk_cnt = UINT32_MAX, clk_us;

static uint32_t h_now(void *ctx)
{
	uint32_t c, t;

	if (k_current_get() != self) {
		return now_us();
	}
	c = nrf_rtc_counter_get(NRF_RTC1);
	beat(c);
	if (c == clk_cnt) {
		return clk_us;
	}
	t = now_us();
	clk_cnt = c;
	clk_us = t;
	return t;
}

static const uint8_t *h_packet(void *ctx, uint32_t *len)
{
	struct pkt_slot *s = &qslots[q_exec_next % QSLOTS];
	k_spinlock_key_t key = k_spin_lock(&qlock);
	bool full = s->full;

	k_spin_unlock(&qlock, key);
	if (!full) {
		return NULL;
	}
	*len = s->len;
	return s->data;
}

static void h_packet_done(void *ctx)
{
	k_spinlock_key_t key = k_spin_lock(&qlock);

	qslots[q_exec_next % QSLOTS].full = false;
	q_exec_next++;
	k_spin_unlock(&qlock, key);
	proto_ack(false);
}

/* before every instruction; host code may run for seconds without reading the clock, so the health beat is
 * also fed here, every 16th instruction */
static uint32_t h_poll(void *ctx)
{
	if (!(++polls & 15)) {
		beat(nrf_rtc_counter_get(NRF_RTC1));
	}
	return atomic_get(&reqs) ? VM_YIELD : 0;
}

/* ERROR and HALT wait for room in the queue; NOTIFY and INPUT are dropped (and counted) instead */
/* a HALT also goes to the log, so its reason stays readable (LOG_READ) when no host heard the event */
static int h_event(void *ctx, uint8_t type, const uint8_t *body, uint32_t len)
{
	if (type == VM_EV_HALT && len >= 11) {
		LOG_WRN("program halted: trigger %u, address 0x%x, op 0x%02x, code %u, detail 0x%x", body[0],
			get32(&body[1]), body[5], body[6], get32(&body[7]));
	}
	return proto_event(type, body, len, type == VM_EV_ERROR || type == VM_EV_HALT);
}

static int h_panel_begin(void *ctx, int32_t x, int32_t y, int32_t w, int32_t h)
{
	panel_used_ms = k_uptime_get();
	held = -1;
	if (!panel_is_on() && panel_power_on() != 0) {
		return 1;
	}
	return panel_rect_begin(x, y, w, h) == 0 ? 0 : 1;
}

static void h_panel_row(void *ctx, int32_t x, int32_t y, int32_t w, const uint8_t *px)
{
	panel_rect_row(px);
}

static void h_panel_span(void *ctx, int32_t x, int32_t y, int32_t w, const uint8_t *px)
{
	panel_rect_span(x, y, w, px);
}

static void h_panel_end(void *ctx)
{
	panel_rect_end();
	last_draw_us = now_us();
	panel_used_ms = k_uptime_get();
}

/* a PANEL = 0 or 2 that waited for the microphone to stop (VM.md §6.5) */
static void pending_off(void)
{
	if (held == 0) {
		panel_power_off();
	} else if (held == 2) {
		panel_power_standby();
	}
	held = -1;
}

/* the firmware stopped the microphone (VM.md §6.5): NOTICE 5 with the reason (1 panel fault, 2 capture,
 * 3 lease) */
static void mic_stopped(uint8_t reason)
{
	if (mic) {
		mic_stop();
		mic = false;
		LOG_INF("microphone off (reason %u)", reason);
		proto_notice(N_MIC, &reason, 1);
	}
}

/* the indicator block (VM.md §6.5) drawn by the firmware itself, over a blanked panel */
static void indicator(void)
{
	static const uint8_t px[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	int x = board_role() ? 4 : 620;

	if (panel_rect_begin(x, 4, 16, 16) == 0) {
		for (int y = 0; y < 16; y++) {
			panel_rect_row(px);
		}
		panel_rect_end();
	}
}

/* MIC 1: the indicator first (it powers the panel on), then the capture; MIC 0: the capture stops first.
 * MIC 1 while on with another format restarts the capture in that format (the reply's). */
static void mic_set(bool on, uint8_t rate, uint8_t per_event)
{
	static uint8_t cur_rate, cur_per_event;
	bool other = rate != cur_rate || per_event != cur_per_event;

	cur_rate = rate;
	cur_per_event = per_event;
	if (on && mic && other) {
		mic_stop();
		if (mic_start(rate, per_event) != 0) {
			mic_stopped(2);
			vm_set_mic(&vm, 0);
		}
		return;
	}
	if (on == mic) {
		if (on) {
			mic_renew();
		}
		return;
	}
	if (!on) {
		mic_stop();
		mic = false;
		vm_set_mic(&vm, 0);
		pending_off();
		return;
	}
	vm_set_mic(&vm, 1);
	mic = true;
	if (!panel_is_on()) {
		vm_set_mic(&vm, 0);
		mic_stopped(1);
	} else if (mic_start(rate, per_event) != 0) {
		vm_set_mic(&vm, 0);
		mic_stopped(2);
	}
}

/* PANEL: 0 off, 1 on, 2 standby; while the microphone is on, 0 and 2 blank the panel to its indicator and
 * wait for the microphone to stop (VM.md §6.5) */
static void h_panel_power(void *ctx, uint32_t v)
{
	panel_used_ms = k_uptime_get();
	if (v == 1) {
		held = -1;
		panel_power_on();
		return;
	}
	if (mic && panel_is_on()) {
		held = v;
		panel_clear_all();
		indicator();
		return;
	}
	if (v == 2) {
		panel_power_standby();
	} else {
		panel_power_off();
	}
}

static void h_brightness(void *ctx, uint32_t level)
{
	uint8_t out[PANEL_LEVEL_LEN];

	panel_set_level(level, out);
	peer_kick();
}

/* Faceclaw's display path (fc.c) runs on this thread, and goes through the same panel calls as DRAW and PANEL. */
uint8_t *exec_arena(void)
{
	return arena;
}

void exec_fc_stop_vm(void)
{
	vm_stop(&vm);
}

int exec_fc_panel_begin(int x, int y, int w, int h)
{
	return h_panel_begin(NULL, x, y, w, h);
}

void exec_fc_panel_end(void)
{
	h_panel_end(NULL);
	if (mic) {
		vm_set_mic(&vm, 1);   /* the frame just went over the indicator */
	}
}

void exec_fc_panel(uint32_t v)
{
	h_panel_power(NULL, v);
}

void exec_fc_brightness(uint32_t level)
{
	h_brightness(NULL, level);
}

static void h_imu_rate(void *ctx, uint32_t hz)
{
	imu_set_rate(hz);
}

/* GYRO_RATE, AUTO_BRIGHTNESS (the core checked their values; on the left it follows the right arm over the
 * arm-to-arm link), BRIGHTNESS_MAX, PEER_LINK and PEER_SEND */
static int h_prop_write(void *ctx, uint32_t id, uint32_t v)
{
	switch (id) {
	case VM_P_BRIGHTNESS_MAX:
		panel_set_level_max(v);
		return 0;
	case VM_P_GYRO_RATE:
		return imu_set_gyro_rate(v) == 0 ? 0 : VM_E_ARG;
	case VM_P_AUTO_BRIGHTNESS:
		if (light_present()) {
			light_set_auto(v);
			return 0;
		}
		return board_role() && peer_available() ? peer_set_follow(v) : VM_E_ARG;
	case VM_P_PEER_LINK:
		return peer_set_link(v);
	case VM_P_PEER_SEND:
		return peer_send(v);
	}
	return VM_E_ARG;
}

static int h_asset_read(void *c, const uint8_t *id, uint32_t off, uint8_t *dst, uint32_t n)
{
	return assets_read(id, off, dst, n);
}

static const struct vm_host host = {
	NULL, h_now, h_packet, h_packet_done, h_poll, h_prop, h_event, h_panel_begin, h_panel_row,
	h_panel_end, h_panel_power, h_brightness, h_imu_rate, h_prop_write, h_panel_span, h_asset_read,
};

/* RESET_SESSION: the window flushed (seq 0 again), a new epoch, the VM's session state reset */
static void reset_session(bool clear_arena)
{
	k_spinlock_key_t key = k_spin_lock(&qlock);
	uint16_t flushed = 0;

	for (int i = 0; i < QSLOTS; i++) {
		if (qslots[i].full) {
			flushed++;
			qslots[i].full = false;
		}
	}
	q_rx_next = 0;
	q_exec_next = 0;
	proto_new_epoch();
	k_spin_unlock(&qlock, key);
	fc_reset();
	vm_reset_session(&vm, clear_arena);
	if (imu_present() && imu_gyro_rate()) {
		imu_set_gyro_rate(0);
	}
	light_set_auto(false);
	peer_reset();
	panel_set_level_max(PANEL_LEVEL_MAX);
	reset_flushed = flushed;
}

/* the platform's triggers (MOTION from the imu thread, TOUCH): repeats before the executor takes them merge,
 * as a trigger's pending bit does, keeping the largest MOTION and the last TOUCH */
void exec_trigger(uint32_t id, uint32_t value)
{
	k_spinlock_key_t key = k_spin_lock(&tlock);

	if (id != VM_T_MOTION || !(trig_bits & BIT(id)) || value > trig_value[id]) {
		trig_value[id] = value;
	}
	trig_bits |= BIT(id);
	k_spin_unlock(&tlock, key);
	atomic_set(&reqs, 1);
	k_sem_give(&wake);
}

static void apply_triggers(void)
{
	k_spinlock_key_t key = k_spin_lock(&tlock);
	uint16_t bits = trig_bits;
	uint32_t value[16];

	memcpy(value, trig_value, sizeof(value));
	trig_bits = 0;
	k_spin_unlock(&tlock, key);
	for (int id = 0; bits; id++, bits >>= 1) {
		if (bits & 1) {
			vm_trigger(&vm, id, value[id]);
		}
	}
}

static void apply(void)
{
	struct req r;
	uint8_t st[32];

	atomic_set(&reqs, 0);
	apply_triggers();
	while (k_msgq_get(&req_q, &r, K_NO_WAIT) == 0) {
		switch (r.op) {
		case R_START:
			LOG_INF("VM_START table 0x%x", r.arg);
			vm_start(&vm, r.arg);
			break;
		case R_STOP:
			vm_status(&vm, st);
			LOG_INF("VM_STOP: state %u, table 0x%x, %u tasks", st[0], get32(&st[4]), get32(&st[20]));
			vm_stop(&vm);
			break;
		case R_RESET:
			LOG_INF("RESET_SESSION (clear arena %u)", r.arg & 1);
			reset_session(r.arg & 1);
			k_sem_give(&reset_done);
			break;
		case R_MIC:
			mic_set(r.arg & 1, (r.arg >> 1) & 1, r.arg >> 8);
			panel_used_ms = k_uptime_get();
			break;
		case R_MIC_FAULT:
		case R_MIC_LEASE:
			if (mic) {
				mic_stopped(r.op == R_MIC_FAULT ? 2 : 3);
				vm_set_mic(&vm, 0);
				pending_off();
			}
			break;
		case R_EVENTS:
			events = r.arg;
			vm_set_events(&vm, events);
			break;
		case R_HANG:
			for (;;) {
				k_busy_wait(1000);
			}
		}
	}
}

static bool running(void)
{
	uint8_t st[32];

	vm_status(&vm, st);
	return st[0] == VM_RUNNING;
}

/* every sample is read only while MOTION or TILT can fire: a threshold or a range that can be left,
 * and a running program or the trigger's EVENTS bit; the TouchBar controller scans only while a running
 * program has a TOUCH or WEAR handler (or waits for one) or the EVENTS mask has them; the light sensor
 * converts only while the panel is on and a program, AUTO_BRIGHTNESS or the LIGHT EVENTS bit uses it */
static void imu_need(void)
{
	uint32_t mg = 0, lo = 0, hi = 0;
	bool run = running();

	vm_prop(&vm, VM_P_MOTION_MG, &mg);
	vm_prop(&vm, VM_P_PITCH_LO, &lo);
	vm_prop(&vm, VM_P_PITCH_HI, &hi);
	bool motion = mg && (run || (events & BIT(VM_T_MOTION)));
	bool tilt = ((int32_t)lo > -90 || (int32_t)hi < 90) && (run || (events & BIT(VM_T_TILT)));

	bool lift = g1_in_case();   /* a lift out of the case clears IN_CASE (VM.md §2.5): every sample while in it */

	imu_stream(motion || tilt || lift, lift ? MIN(motion ? mg : LIFT_MG, LIFT_MG) : motion ? mg : 0);
	touch_use(((vm_wants(&vm) | events) & (BIT(VM_T_TOUCH) | BIT(VM_T_WEAR))) != 0);
	light_use(panel_is_on() && (run || light_auto() || (events & BIT(VM_T_LIGHT))));
}

/* VM.md §6.6: the panel, on or in standby, off 30 s after the last DRAW with no host and no program */
static void unattended(void)
{
	if ((panel_is_on() || panel_in_standby()) && !proto_connected() && !running() &&
	    k_uptime_get() - panel_used_ms >= PANEL_IDLE_MS && !qslots[q_exec_next % QSLOTS].full) {
		panel_power_off();
		LOG_INF("panel off: no host, no program and no DRAW for %u s", CONFIG_G1_PANEL_IDLE_S);
	}
}

static void exec_thread(void *a, void *b, void *c)
{
	self = k_current_get();
	vm_init(&vm, arena, &host, safe_level() != 0);
	fc_init();
	/* the panel is off at boot (VM.md §6.6): powered once to check its ID (HELLO panel ok, NOTICE 3) and to
	 * find the light sensor on its rails, then off until the first DRAW or PANEL = 1 */
	if (pmic_wait_ready(K_SECONDS(5)) && safe_level() == 0) {
		if (panel_power_on() != 0) {
			proto_notice(N_PANEL, (const uint8_t[]){1}, 1);
		}
		light_probe();
		panel_power_off();
	}
	for (;;) {
		uint32_t wait;

		apply();
		fc_run();
		wait = vm_run(&vm);
		clk_cnt = UINT32_MAX;
		atomic_inc(&hb_exec);
		imu_need();
		unattended();
		if (wait && !fc_pending()) {
			k_sem_take(&wake, wait >= 500000 ? K_MSEC(500) : K_USEC(wait));
		}
	}
}

/* below the PMIC (6) and case-link (7) threads: host code is not capped, and a long burst of it must not
 * stop their 1 s cadence (VBUS, the nPM1300 watchdog kick, the case's replies) */
K_THREAD_DEFINE(exec, 3072, exec_thread, NULL, NULL, NULL, 8, 0, K_TICKS_FOREVER);

void exec_start(void)
{
	k_thread_start(exec);
}

/* control requests (proto.c, the evt thread) */

int exec_vm_start(uint32_t table)
{
	if (safe_level()) {
		return VM_E_SAFE;
	}
	if ((uint64_t)table + 64 > VM_ARENA_SIZE) {
		return VM_E_ARENA;
	}
	post(R_START, table);
	return 0;
}

void exec_vm_stop(void)
{
	post(R_STOP, 0);
}

uint16_t exec_reset_session(bool clear_arena)
{
	k_sem_reset(&reset_done);
	post(R_RESET, clear_arena);
	k_sem_take(&reset_done, K_SECONDS(5));
	return reset_flushed;
}

void exec_mic(bool on, uint8_t rate, uint8_t per_event)
{
	post(R_MIC, (on ? 1 : 0) | (rate ? 2 : 0) | per_event << 8);
}

void exec_mic_fault(void)
{
	post(R_MIC_FAULT, 0);
}

void exec_mic_lease_end(void)
{
	post(R_MIC_LEASE, 0);
}

bool exec_mic_on(void)
{
	return mic;
}

/* the link dropped or is not encrypted: the microphone goes off (no NOTICE: no host to tell) */
void exec_link_down(void)
{
	if (mic) {
		post(R_MIC, 0);
	}
}

void exec_hang_for_test(void)
{
	post(R_HANG, 0);
}

void exec_vm_status(uint8_t *out)
{
	vm_status(&vm, out);
}

int exec_prop(uint32_t id, uint32_t *value)
{
	return vm_prop(&vm, id, value);
}

uint32_t exec_triggers(void)
{
	return BIT(VM_T_START) | BIT(VM_T_TIMER) | BIT(VM_T_BATTERY) | BIT(VM_T_CASE) | (0xFu << VM_T_USER0) |
	       (imu_present() ? BIT(VM_T_MOTION) | BIT(VM_T_TILT) : 0) |
	       (touch_present() ? BIT(VM_T_TOUCH) : 0) | (touch_present() || peer_available() ? BIT(VM_T_WEAR) : 0) |
	       (light_present() ? BIT(VM_T_LIGHT) : 0) | (peer_available() ? BIT(VM_T_PEER) : 0);
}

/* EVENTS (control thread): the mask in force, applied by the executor between two instructions */
uint16_t exec_events(uint16_t mask)
{
	mask &= exec_triggers();
	post(R_EVENTS, mask);
	return mask;
}

uint32_t exec_motion_mg(void)
{
	uint32_t mg = 0;

	vm_prop(&vm, VM_P_MOTION_MG, &mg);
	return mg;
}

/* the TILT range (imu.c shows samples outside it to the executor one by one) */
void exec_pitch_range(int32_t *lo, int32_t *hi)
{
	uint32_t l = (uint32_t)-90, h = 90;

	vm_prop(&vm, VM_P_PITCH_LO, &l);
	vm_prop(&vm, VM_P_PITCH_HI, &h);
	*lo = (int32_t)l;
	*hi = (int32_t)h;
}

const struct vm_stats *exec_stats(void)
{
	return &vm.stats;
}

uint32_t exec_last_draw_us(void)
{
	return last_draw_us;
}

/* ASSET_PUT's job reads the staged range while host code and programs run: a change fails its verification */
void exec_arena_copy(uint32_t addr, uint8_t *dst, uint32_t n)
{
	memcpy(dst, &arena[addr], n);
}

/* ARENA_CRC: zlib's CRC-32 of arena bytes addr..addr+len-1 and the first 16 of them (zero-padded) */
uint32_t exec_arena_crc(uint32_t addr, uint32_t len, uint8_t *first16)
{
	memset(first16, 0, 16);
	memcpy(first16, &arena[addr], MIN(len, 16));
	return crc32_ieee(&arena[addr], len);
}
