/* The ScouterKit G1 VM, protocol 5 (g1_firmware/docs/VM.md): host code and programs on one arena.
 * Freestanding C99, also valid C++. The firmware and the host library's emulator embed the same
 * interpreter and give it a struct vm_host; README.md describes the call pattern. */
#ifndef SCOUTER_VM_H
#define SCOUTER_VM_H

#include <stdint.h>
#include "vm_gfx.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VM_FOREVER 0xFFFFFFFFu
#define VM_UNKNOWN 0x80000000u
#define VM_YIELD 0x80000000u
#define VM_JUMPS 1000
#define VM_DEPTH 16
#define VM_CAP_US 30000u
#define VM_PERIOD_US 100000u
#define VM_SLEEP_MAX_US 2000000

enum vm_trigger_id {
	VM_T_START = 0,
	VM_T_TIMER = 1,
	VM_T_MOTION = 2,
	VM_T_TILT = 3,
	VM_T_LIGHT = 4,
	VM_T_BATTERY = 5,
	VM_T_CASE = 6,
	VM_T_WEAR = 7,
	VM_T_TOUCH = 8,
	VM_T_PEER = 9,
	VM_T_USER0 = 12
};

enum vm_prop_id {
	VM_P_TIME_US = 0x00,
	VM_P_TRIGGER = 0x01,
	VM_P_PENDING = 0x02,
	VM_P_ARM = 0x03,
	VM_P_LINK = 0x04,
	VM_P_LINK_INTERVAL = 0x05,
	VM_P_BATT_MV = 0x10,
	VM_P_BATT_PCT = 0x11,
	VM_P_CHARGE = 0x12,
	VM_P_IN_CASE = 0x13,
	VM_P_WORN = 0x14,
	VM_P_BATT_MA = 0x15,
	VM_P_BATT_TEMP = 0x16,
	VM_P_ACC_X = 0x18,
	VM_P_ACC_Y = 0x19,
	VM_P_ACC_Z = 0x1A,
	VM_P_PITCH = 0x1B,
	VM_P_LIGHT = 0x1C,
	VM_P_ROLL = 0x1D,
	VM_P_BRIGHTNESS = 0x20,
	VM_P_PANEL = 0x21,
	VM_P_TIMER = 0x22,
	VM_P_MOTION_MG = 0x23,
	VM_P_PITCH_LO = 0x24,
	VM_P_PITCH_HI = 0x25,
	VM_P_LIGHT_LO = 0x26,
	VM_P_LIGHT_HI = 0x27,
	VM_P_IMU_RATE = 0x28,
	VM_P_BRIGHTNESS_MAX = 0x29,
	VM_P_BRIGHTNESS_CAP = 0x2A,
	VM_P_GYR_X = 0x30,
	VM_P_GYR_Y = 0x31,
	VM_P_GYR_Z = 0x32,
	VM_P_GYRO_RATE = 0x33,
	VM_P_TOUCH_GESTURE = 0x34,
	VM_P_CASE_LID = 0x35,
	VM_P_CASE_CHARGING = 0x36,
	VM_P_CASE_BATT_PCT = 0x37,
	VM_P_AUTO_BRIGHTNESS = 0x38,
	VM_P_PEER_LINK = 0x39,
	VM_P_PEER_SEND = 0x3A,
	VM_P_PEER_MSG = 0x3B,
	VM_P_PEER_T_US = 0x3C
};

enum vm_event_type { VM_EV_ERROR = 0x02, VM_EV_HALT = 0x03, VM_EV_NOTIFY = 0x04, VM_EV_INPUT = 0x07 };

enum vm_state { VM_STOPPED = 0, VM_RUNNING = 1, VM_HALTED = 2 };

enum vm_task_state { VM_TASK_NONE = 0, VM_TASK_RUN = 1, VM_TASK_SLEEP = 2, VM_TASK_WAIT = 3 };

/* What the VM needs from its platform. All members are required except prop_write, panel_span and
 * asset_read. The VM calls them only from inside the vm_* functions, on the thread that calls those. */
struct vm_host {
	void *ctx;
	/* device time, µs since boot (wraps) */
	uint32_t (*now)(void *ctx);
	/* the packet at exec_next if it has arrived (u16 seq, then host code), else NULL; it stays valid
	 * until packet_done */
	const uint8_t *(*packet)(void *ctx, uint32_t *len);
	/* that packet has run or ended on an error: free its slot, advance exec_next, ack */
	void (*packet_done)(void *ctx);
	/* called before every instruction: bits 0-15 are triggers fired by the platform since the last
	 * call (TOUCH, tests; a trigger with a value of its own goes through vm_trigger), VM_YIELD makes
	 * vm_run return at once */
	uint32_t (*poll)(void *ctx);
	/* a property the platform owns (§2.5): ARM, LINK, LINK_INTERVAL, battery, sensor and case values,
	 * BRIGHTNESS, PANEL, GYR_*, GYRO_RATE, TOUCH_GESTURE, AUTO_BRIGHTNESS, PEER_LINK, PEER_MSG, PEER_T_US;
	 * VM_UNKNOWN when it has no value (no driver yet) */
	uint32_t (*prop)(void *ctx, uint32_t id);
	/* an event (§6.3: ERROR, HALT, NOTIFY, INPUT): body is everything after offset 9 (the host adds
	 * type, epoch and time); returns 0 if it was queued */
	int (*event)(void *ctx, uint8_t type, const uint8_t *body, uint32_t len);
	/* DRAW: power the panel on if it is off, prepare the rectangle (w, h > 0); nonzero = no panel */
	int (*panel_begin)(void *ctx, int32_t x, int32_t y, int32_t w, int32_t h);
	/* the rows of that rectangle, top to bottom: w pixels in (w + 1) / 2 bytes, left pixel high */
	void (*panel_row)(void *ctx, int32_t x, int32_t y, int32_t w, const uint8_t *px);
	/* latch: everything written since panel_begin becomes visible at once */
	void (*panel_end)(void *ctx);
	/* PANEL written: 0 off (image lost), 1 on, 2 standby (image lost, a fast wake at the next DRAW); while
	 * the microphone is on, 0 and 2 blank the panel and wait for the microphone to stop (VM.md §6.5) */
	void (*panel_power)(void *ctx, uint32_t on);
	/* BRIGHTNESS written, 0..42 */
	void (*brightness)(void *ctx, uint32_t level);
	/* IMU_RATE written or reset: 0, 13, 26, 52 or 104 Hz */
	void (*imu_rate)(void *ctx, uint32_t hz);
	/* a write to a property the platform owns: GYRO_RATE (0, 13, 26, 52 or 104), AUTO_BRIGHTNESS (0 or 1),
	 * PEER_LINK, PEER_SEND; 0 or an error code (5: not available). NULL: none of them (error 5) */
	int (*prop_write)(void *ctx, uint32_t id, uint32_t value);
	/* VIDEO's rows: part of a row inside the rectangle (w pixels from x, as panel_row), rows top to
	 * bottom, some skipped. NULL: panel_row is called instead */
	void (*panel_span)(void *ctx, int32_t x, int32_t y, int32_t w, const uint8_t *px);
	/* ASSET_LOAD: bytes off..off+n-1 of the stored asset with this ID to dst (n bytes in the arena); 0, 13
	 * (unknown ID, a corrupt body, no store) or 3 (off + n past its end), and nothing written unless 0.
	 * NULL: there is no store (error 13) */
	int (*asset_read)(void *ctx, const uint8_t *id, uint32_t off, uint8_t *dst, uint32_t n);
};

/* the current host packet or the current program task */
struct vm_thread {
	uint32_t pc; /* arena address, or offset in the packet's code while in_packet */
	uint32_t wake;
	uint32_t wait_word;
	uint32_t ret[VM_DEPTH];
	uint16_t wait_mask;
	uint16_t jumps;
	uint8_t state;
	uint8_t depth;
	uint8_t trigger;
	uint8_t in_packet;
};

/* counters since vm_init, for the STATUS reply */
struct vm_stats {
	uint32_t packets, instructions, draws, jumps;
	uint16_t errors, late;
};

/* The members after stats are private. */
struct vm {
	uint8_t *arena;
	const struct vm_host *host;
	struct vm_stats stats;

	uint8_t safe, mic, state, halt_code, timer_armed, capped, tilt_in, light_in;
	uint16_t pending, coalesced, notify_count, notify_dropped, cap_pauses, seq;
	uint16_t events, input_count;
	uint8_t ev_tilt_in, ev_light_in;
	uint32_t ev_pct, ev_charge, ev_worn, ev_case[4];
	uint32_t table, halt_addr, timer, tasks;
	struct vm_thread task, pkt;
	const uint8_t *code;
	uint32_t code_len, pkt_off;
	uint32_t motion_mg, pitch_lo, pitch_hi, light_lo, light_hi, imu_rate;
	uint32_t last_pct, last_charge, last_worn, last_case[4];
	uint32_t period_start, run_us, last_run_us, charged_us;
	uint8_t row[VM_PANEL_W / 2];
	struct vm_video video;
};

/* Boot: zero the arena (VM_ARENA_SIZE bytes, owned by the caller), program stopped, default
 * settings. safe: the arm is in safe mode (DRAW is error 7, VM_START is refused). */
void vm_init(struct vm *vm, uint8_t *arena, const struct vm_host *host, int safe);

/* Run host packets (first) and the program until nothing can run now. Returns how long the caller
 * may block in µs: VM_FOREVER until news (a packet, a trigger, a sensor change, a control request),
 * 0 after VM_YIELD. Call it again when that time is up or news arrives. */
uint32_t vm_run(struct vm *vm);

/* Control requests (§6.4), called between two vm_run calls on the same thread. */
int vm_start(struct vm *vm, uint32_t table); /* VM_START: 0, or error 3 or 7 */
void vm_stop(struct vm *vm);
/* RESET_SESSION: program stopped, settings reset, the current packet dropped without packet_done
 * (the caller flushes its receive window) */
void vm_reset_session(struct vm *vm, int clear_arena);
/* the microphone went on or off: draw or clear its indicator (§6.5) */
void vm_set_mic(struct vm *vm, int on);
/* PROPS: one property; 0, or 5 for an unknown ID */
int vm_prop(const struct vm *vm, uint32_t id, uint32_t *value);
/* What SYS_WRITE does, for a host that runs no instructions: 0 or an error code. */
int vm_prop_set(struct vm *vm, uint32_t id, uint32_t value);
/* VM_STATUS reply body, 32 bytes */
void vm_status(const struct vm *vm, uint8_t *out);

/* The triggers a running program can take now: those with a handler in its table and those its task waits
 * for; 0 while no program runs. A platform switches a sensor off that nothing wants. */
uint16_t vm_wants(const struct vm *vm);

/* EVENTS (§6.4): the triggers reported as INPUT events (§6.3), whether or not a program runs. The
 * platform passes the mask in force (the request AND HELLO.triggers; 0 at boot and when the link
 * drops); the sensor values are sampled now, so only later changes are reported. */
void vm_set_events(struct vm *vm, uint16_t mask);
/* A trigger the platform detected (MOTION with the change in mg, TOUCH, PEER): fired for a running
 * program and reported as INPUT with this value when its EVENTS bit is set. */
void vm_trigger(struct vm *vm, uint32_t id, uint32_t value);

/* MOTION (§5.2): 1 if two consecutive samples (mg) are more than mg apart (Euclidean); 0 if mg = 0.
 * The IMU code calls it with the MOTION_MG setting (vm_prop) and fires VM_T_MOTION (vm_trigger). */
int vm_moved(const int32_t *prev, const int32_t *cur, uint32_t mg);
/* the Euclidean distance of two samples, mg (rounded down) */
uint32_t vm_distance(const int32_t *prev, const int32_t *cur);

#ifdef __cplusplus
}
#endif

#endif
