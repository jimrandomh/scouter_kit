#ifndef G1_H_
#define G1_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>

#define PANEL_W 640
#define PANEL_H 480
#define PANEL_ROW_BYTES (PANEL_W / 2)   /* 4 bpp */
#define ARENA_SIZE (256 * 1024)         /* u16 word addresses cover exactly 256 KiB */
#define ARENA_WORDS (ARENA_SIZE / 4)
#define QSLOTS 32
#define MAX_PKT 244
#define PROTO_VERSION 5

/* retained.c: boot accounting and the RAM log that survive soft resets */
enum clean_kind { CLEAN_NONE = 0, CLEAN_SMP = 1, CLEAN_HOST = 2 };
void retained_boot(void);
void retained_mark_clean(enum clean_kind kind, bool clear_counter, bool force_safe);
void retained_clear_unhealthy(void);
void retained_record_fault(uint32_t reason, uint32_t pc, uint32_t lr);
void retained_set_health_reason(uint32_t reason);
uint32_t retained_boot_count(void);
uint8_t retained_net_fails(void);
void retained_set_net_fails(uint8_t v);
bool retained_net_fallback_done(void);
void retained_set_net_fallback_done(void);
uint32_t retained_resetreas(void);
uint8_t retained_unhealthy(void);
uint8_t safe_level(void);
size_t retained_log_read(uint32_t off, uint8_t *out, size_t max, uint32_t *total);
/* the net-off measurement (diag.c): per phase, battery samples only (no VBUS) */
#define NETOFF_PHASES 10
struct netoff_phase {
	uint32_t samples, battery_samples;
	int32_t avg_ua;
	uint16_t vbat_first, vbat_last;
	int16_t ntc_dc;
	uint16_t cpu_permille;
};
bool retained_case_full(void);
void retained_set_case_full(bool full);
void retained_netoff_request(uint16_t seconds, uint8_t phases, const uint8_t *masks);
bool retained_netoff_take(uint16_t *seconds, uint8_t *phases, uint8_t *masks, bool *masked);
void retained_netoff_store(const struct netoff_phase *p, uint8_t phases);
uint8_t retained_netoff_result(struct netoff_phase *p, uint16_t *seconds, uint8_t *net_fails_at_test);

/* health.c */
extern atomic_t hb_exec, hb_pmic;
extern volatile uint32_t bt_ok_ms;
void health_start(void);
bool health_good(void);
void health_host_ok(void);
void health_bt_traffic(void);
void health_set_ble_up(void);
void health_no_bt_checks(uint32_t until_ms);
void wdt0_feed(void);
void g1_reboot(enum clean_kind kind, bool clear_counter, bool force_safe);
bool g1_install_pending(void);
void g1_prepare_reboot(void);

/* board.c */
struct g1_settings {
	uint8_t esb_channel, esb_master, esb_slave;
	uint8_t sn[16];
	char serial[17];             /* +0x15: the glasses' frame serial, ASCII ("" if none) */
	bool read_ok, initialised;   /* record readable; stock's first_enter_flag (+0x00) == 1 */
	uint8_t panel_v;             /* +0x14: stock powers the panel's BUCK2 at 1.1 V when 1, else 1.2 V */
	uint8_t lum_base;            /* +0x78: stock's luminance base, 1..63 (else 63) */
	int32_t imu_base;            /* +0x0C: stock's IMU pitch base, centidegrees (IMU_BASE_NONE: no record) */
};
#define IMU_BASE_NONE INT32_MIN
extern struct g1_settings g1_settings;
int board_init(void);
uint8_t board_role(void); /* 0 = right (ESB master), 1 = left */
void board_panel_enable(bool on);
void board_touch_power(bool on);
bool board_touch_powered(void);
uint32_t now_us(void);

/* netcore.c */
int g1_netcore_handshake(uint8_t role, uint8_t esb_channel, uint8_t esb_master, uint8_t esb_slave);
int g1_netcore_send(const uint8_t *msg, size_t len);
extern uint32_t netcore_other_msgs;

/* peer.c: the arm-to-arm message link (VM.md §6.9) */
struct peer_stats {
	uint8_t available, link, follow, link_value;
	uint32_t reports, net_acked, net_failed, net_received, report_t;
	uint32_t frames_out, reports_sent, report_timeouts, send_errors, frames_in, delivered, inbox_drops;
	uint32_t levels_applied, last_arrival_t, other_msgs;
	int32_t offset;
	uint32_t delay_last_us, delay_max_us, delay_total_us;   /* frame queued to its delivery report */
};
void peer_start(bool master);
bool peer_available(void);
void peer_net_frame(const uint8_t *frame);
void peer_net_sent(uint8_t v);
void peer_net_report(uint32_t acked, uint32_t failed, uint32_t received, uint8_t value);
uint32_t peer_link(void);
int peer_set_link(uint32_t v);
int peer_send(uint32_t v);
uint32_t peer_msg(bool time);
int peer_set_follow(uint32_t v);
bool peer_follow(void);
void peer_kick(void);
void peer_reset(void);
void peer_get_stats(struct peer_stats *s);

/* pmic.c */
struct pmic_status {
	bool present, configured, charging_enabled, wdt_armed;
	uint8_t vterm_code, charge_status, vbus_status, refuse_reason;
	uint16_t vbat_mv, ntc_code;
	uint32_t i2c_errors;
};
extern struct pmic_status pmic_status;
/* One nPM1300 ADC sample (every second, pmic.c), as read: results 0x05 0x10..0x1A, charger status
 * 0x03 0x34, charger error 0x03 0x36, VBUS status 0x02 0x07, LDSW status 0x08 0x04. */
struct pmic_sample {
	uint16_t vbat_code, ntc_code, die_code, vsys_code, vbus_code, ibat_code;
	uint8_t ibat_stat, charge_status, charge_error, vbus_status, ldsw_status;
};
int pmic_i2c_read(uint8_t page, uint8_t off, uint8_t *out, size_t n);
void pmic_start(void);
bool pmic_wait_ready(k_timeout_t timeout);
int pmic_panel_rails(bool on);
int pmic_ldsw2_on(void);
int pmic_ldsw2_off(void);
void pmic_stop_kicking(void);
void pmic_kick_now(void);
void pmic_watchdog_stop_for_install(void);
bool pmic_vbus_present(void);
bool g1_in_case(void);
void pmic_charger_enable(void);
uint32_t g1_case_since_ms(bool *moved_since);
void g1_case_seen(void);    /* VBUS, the tag's field or a case message: IN_CASE set (VM.md §2.5) */
void g1_case_moved(void);   /* a sample more than LIFT_MG from the one before, while IN_CASE */
#define LIFT_MG 150

/* power.c: power telemetry (CTRL POWER, PROTOCOL.md §8.6). BATT_MA and BATT_TEMP as the v5 properties
 * (VM.md §2.5): mA positive while discharging (64-sample average), 0.1 degC from the pack's NTC. */
void power_sample(const struct pmic_sample *s);
size_t power_reply(uint8_t page, uint8_t *b, size_t n);
int32_t power_batt_ua(void);    /* 64-sample average, uA, positive while discharging */
int32_t power_batt_ma(void);    /* the same in mA, rounded; INT32_MIN (VM UNKNOWN) before a sample */
int32_t power_ibat_ua(void);    /* latest sample */
bool power_ibat_ok(void);       /* the latest sample's IBAT was a valid measurement */
bool power_ibat_code_ua(uint8_t stat, uint16_t code, int32_t *ua);
/* the PMIC round's burst while the panel is on (pmic.c, VM.md §6.6): the mean IBAT and the lowest VBAT of its
 * conversions; false when this round had none (the panel off, or no valid conversion) */
bool pmic_burst(int32_t *ua, uint16_t *mv_min, uint8_t *n);
/* DIAG ADC_BURST: n VBAT+IBAT conversions wait_us apart as fast as the bus allows, for the pulse record */
struct adc_conv {
	uint32_t t_us;
	int32_t ua;          /* INT32_MIN: not a valid measurement */
	uint16_t mv;
};
int pmic_adc_record(uint8_t n, uint16_t wait_us, struct adc_conv *out);
int16_t power_batt_temp(void);  /* 0.1 degC, INT16_MIN unknown */
uint16_t power_vbus_mv(void);
void power_window_start(void);
void power_window_get(struct netoff_phase *p);
extern volatile uint8_t power_imu_regs[3];  /* LSM6DSO WHO_AM_I, CTRL1_XL, CTRL2_G as read (0xFF unknown) */
extern volatile bool power_imu_configured;

/* imu.c: LSM6DSO accelerometer (VM.md §2.5): ACC_X/Y/Z, PITCH, ROLL; IMU_RATE */
void imu_init(void);
bool imu_present(void);
int imu_set_rate(uint32_t hz);
uint32_t imu_prop(uint32_t id);
int imu_set_gyro_rate(uint32_t hz);
uint32_t imu_gyro_rate(void);
uint32_t imu_gyro_prop(uint32_t id);
void imu_stream(bool on, uint32_t motion_mg);
uint32_t imu_irqs(void);
void imu_stats(uint32_t *samples, uint32_t *misses, uint32_t *i2c_errors, bool *streaming);
void imu_off(void);
void imu_map(int16_t out[9], int32_t *base);
/* DIAG PIN_PROBE: INT1/INT2 open drain, active low; 1 INT1 active, 2 INT2 active, 0 none; 0xFF restores */
int imu_int_drive(uint8_t state);

/* touch.c: the TouchBar controller (VM.md §6.7): TOUCH, TOUCH_GESTURE, WORN */
int touch_reg_read(uint16_t reg, uint32_t *v);
int touch_reg_write(uint16_t reg, uint32_t v);
void touch_start(void);
void touch_use(bool on);
bool touch_present(void);
uint32_t touch_prop(uint32_t id);
void touch_link_value(uint8_t v);
void touch_wear_tick(void);
void touch_case_entered(void);
bool touch_worn(void);   /* classified as worn since the arm was last put in the case */
int touch_wear_raw(bool *since_case);
void touch_stats(uint32_t *events, uint32_t *errors, uint32_t *loads, uint32_t *irqs, bool *scanning);

/* light.c: the OPT3007 (VM.md §6.7): LIGHT, AUTO_BRIGHTNESS */
void light_probe(void);
void light_use(bool on);
bool light_present(void);
uint32_t light_prop(void);
void light_set_auto(bool on);
bool light_auto(void);
void light_stats(uint32_t *readings, uint32_t *errors, bool *converting);

/* caselink.c: the case's NFC requests over the ST25DV mailbox, answered as stock (power.md §4) */
struct caselink_state {
	uint32_t requests, messages, replies, gpo_events, last_ms, msg_ms, i2c_errors, field_ms, restarts, charger_enables;
	uint32_t by_look, reply_n, reply_late, reply_max_us, it_count[8], blanks, blank_pulses;
	uint64_t blanked_ms;
	uint64_t reply_sum_us;
	uint16_t vrect_mv, busy, unknown, long_trip;
	uint8_t status, level, lid, box_battery, it_sts, gpo_config;
	bool lid_closed, full_latched, charging, lid_known, box_known, field, blanked;
};
extern struct caselink_state caselink;
enum { CE_EDGE = 1, CE_IT_STS = 2, CE_REPLY = 3, CE_BLANK = 4, CE_UNBLANK = 5 };
struct caselink_event {
	uint32_t ms;
	uint8_t kind, value;
};
size_t caselink_trace(uint8_t *out, size_t max);   /* DIAG CASE_TRACE: the last events, 6 bytes each */
uint32_t caselink_prop(uint32_t id);   /* CASE_LID, CASE_CHARGING, CASE_BATT_PCT (VM.md §6.8) */
bool caselink_heard(uint32_t ms);      /* a case message within the last ms */
void caselink_start(void);
void caselink_vbus_changed(void);
void caselink_battery_tick(void);
void caselink_gpo_pullup_out(bool on);
void caselink_set_full(bool on);

/* panel.c */
int panel_power_on(void);
void panel_power_off(void);
bool panel_is_on(void);
bool panel_in_standby(void);
int panel_power_standby(void);
int panel_blank(bool blank);
int panel_standby(uint8_t mode);
uint32_t panel_power_on_ms(void);
bool panel_ok(void);
int panel_clear_all(void);
/* A DRAW of the rectangle (x, y, w, h), inside the panel: begin (-ENODEV without a panel that is on),
 * then h rows of (w + 1) / 2 bytes (left pixel in the high nibble), then end, which latches it with one
 * 97 and returns the rows written or <0. The panel is locked from begin to end. */
int panel_rect_begin(int x, int y, int w, int h);
void panel_rect_row(const uint8_t *px);
/* instead of the rows, for VIDEO: parts of rows inside the rectangle (w pixels from x), top to bottom */
void panel_rect_span(int x, int y, int w, const uint8_t *px);
int panel_rect_end(void);
uint16_t panel_luminance(void);
uint8_t panel_level(void);             /* BRIGHTNESS as written */
uint8_t panel_level_max(void);         /* BRIGHTNESS_MAX */
uint8_t panel_cap(void);               /* BRIGHTNESS_CAP, the panel guard's */
uint8_t panel_effective_level(void);   /* what the panel shows: min(BRIGHTNESS, BRIGHTNESS_MAX, BRIGHTNESS_CAP) */
void panel_set_level_max(uint8_t max);
int panel_set_cap(uint8_t cap, k_timeout_t wait);   /* -EAGAIN when the panel lock is busy */
/* BRIGHTNESS (CTRL 0x0A): stock's user levels 0..42 (0xFF = query), default 21 as stock's default
 * user setting; out = level, u16 lum, current, u16 lum read back, current read back, base, applied. */
#define PANEL_LEVEL_MAX 42
#define PANEL_LEVEL_DEFAULT 21
#define PANEL_LEVEL_LEN 9
int panel_set_level(uint8_t level, uint8_t *out);
/* PANEL_READ (CTRL 0x0B): rows read back from the panel cache; out = u32 crc32, u16 rows, 16 bytes */
#define PANEL_READ_LEN 22
int panel_read_rows(uint16_t y0, uint16_t rows, uint8_t *out);
/* PANEL_PROBE (diagnostic, CTRL 0x09): rails on as stock, then stock's ID read `9F 00 00 00` in every
 * SPI mode and MISO sample delay at one clock rate; record = PANEL_PROBE_LEN bytes into out. */
#define PROBE_CS_HIGH 0x01   /* CS active high (stock: active low) */
#define PROBE_EN_FIRST 0x02  /* P0.24 before the rails (stock's boot order) */
#define PROBE_LDSW2 0x04     /* LDSW2 on first (stock's boot; stays on) */
#define PROBE_RESET 0x08     /* stock's software reset 66, 99 before the reads */
#define PROBE_AB 0x10        /* release from deep power-down (AB) before the reads */
#define PANEL_PROBE_LEN (10 + 32 * 4)
int panel_probe(uint8_t mhz, uint8_t flags, uint16_t delay_ms, uint8_t *out);

/* idle.c: the idle-current bisect's switches (test builds, RAM only; DIAG SET 7, the net-off phases) */
enum { IDLE_QSPI_DPD, IDLE_QSPI_PULLS, IDLE_UART_TX, IDLE_LOWS, IDLE_PANEL_EN, IDLE_SENSORS, IDLE_TWIM2, IDLE_BITS };
uint8_t idle_set(uint8_t mask);

/* diag.c: DIAG sub-ops 3-7 (g1_firmware/README.md) and the net-off measurement at boot */
size_t diag_request(const uint8_t *a, size_t alen, uint8_t *b, size_t n, uint8_t *st, bool *reboot);
bool diag_netoff_boot(void);

/* exec.c: the executor thread owns the arena and the VM core (scouter_kit/src/vm) */
struct vm_stats;
void exec_start(void);
void exec_wake(void);
void exec_sensors(void);
int exec_vm_start(uint32_t table);
void exec_vm_stop(void);
uint16_t exec_reset_session(bool clear_arena);
void exec_mic(bool on, uint8_t rate, uint8_t per_event);
void exec_mic_fault(void);
void exec_mic_lease_end(void);
void exec_link_down(void);
bool exec_mic_on(void);
void exec_hang_for_test(void);
void exec_vm_status(uint8_t *out);
int exec_prop(uint32_t id, uint32_t *value);
uint32_t exec_triggers(void);
uint16_t exec_events(uint16_t mask);
void exec_trigger(uint32_t id, uint32_t value);
uint32_t exec_motion_mg(void);
void exec_pitch_range(int32_t *lo, int32_t *hi);
const struct vm_stats *exec_stats(void);
uint32_t exec_last_draw_us(void);
uint32_t exec_arena_crc(uint32_t addr, uint32_t len, uint8_t *first16);
void exec_arena_copy(uint32_t addr, uint8_t *dst, uint32_t n);
/* fc.c's side of the executor */
uint8_t *exec_arena(void);
void exec_fc_stop_vm(void);
int exec_fc_panel_begin(int x, int y, int w, int h);
void exec_fc_panel_end(void);
void exec_fc_panel(uint32_t v);
void exec_fc_brightness(uint32_t level);

/* fc.c: Faceclaw's display path (VM.md §6.11) */
#define EV_FC_ACK 0x08
void fc_init(void);
void fc_rx(const uint8_t *d, size_t len);
void fc_link_down(void);
bool fc_pending(void);
void fc_run(void);
void fc_reset(void);
size_t fc_info(uint8_t *b);

/* assets.c: the asset store (VM.md §6.10); control requests return 0 or an error code */
void assets_info(uint8_t *out);   /* 20 bytes */
size_t assets_list(uint8_t first, uint8_t *out, size_t room);
int assets_put(const uint8_t *id, uint32_t addr, uint32_t size, uint8_t type, uint8_t *result);
int assets_delete(const uint8_t *id);
int assets_format(const uint8_t *a, size_t alen);
int assets_crc(uint32_t addr, uint32_t n);
int assets_read(const uint8_t *id, uint32_t off, uint8_t *dst, uint32_t n);   /* ASSET_LOAD: 0, 13 or 3 */
void assets_set(uint8_t item, uint8_t value);   /* DIAG SET 9 and 10 */
size_t assets_diag(uint8_t *out);               /* DIAG 18 */
size_t bonds_diag(uint8_t *out, size_t max);    /* DIAG 19 BONDS (bonds.c), read only */

/* proto.c */
struct pkt_slot {
	uint16_t len;
	bool full;
	uint8_t data[MAX_PKT];
};
extern struct pkt_slot qslots[QSLOTS];
extern struct k_spinlock qlock;
extern uint16_t q_rx_next, q_exec_next;
extern uint32_t session_epoch;
int proto_init(void);
void proto_ack(bool now);
/* an event from the VM (ERROR, HALT, NOTIFY): the header is added; 0 if queued. block: wait up to 1 s
 * for room in the queue */
int proto_event(uint8_t type, const uint8_t *body, size_t len, bool block);
void proto_notice(uint8_t kind, const uint8_t *body, size_t len);
void proto_mic_data(uint16_t first, uint8_t n, const uint8_t *frames, size_t len);
void proto_mic_flush(void);
uint32_t proto_mic_drops(void);

/* mic.c (VM.md §6.5): capture on the right arm; started and stopped by the executor only */
#define MIC_MAX_FRAMES_BYTES 232   /* MTU 247 - 3 - the 12-byte MIC_DATA head */
#define MIC_MAX_PER_EVENT 5        /* 50 ms per event at both rates (docs/audio-codec.md) */
#define MIC_LEASE_MS 2000          /* MIC 1 holds the microphone this long unless renewed */
struct mic_stats {
	uint8_t on, rate, per_event;
	uint32_t starts, frames, lost, encoded, events, faults, encode_us;
	uint16_t encode_max_us, peak, rms;
};
bool mic_available(void);
int mic_start(uint8_t rate, uint8_t per_event);
void mic_stop(void);
void mic_get_stats(struct mic_stats *out);
int mic_encode_load(uint16_t seconds, uint8_t clock_fast);
void mic_set_fast(bool fast);
void mic_renew(void);
void proto_new_epoch(void);
uint8_t proto_credits(void);
bool proto_connected(void);
bool proto_link_obeyed(void);
uint32_t proto_link_interval_us(void);
bool proto_writes_allowed(void);
void proto_update_bondable(void);
bool proto_bondable(void);
void adv_kick(void);
bool adv_slow(void);   /* main.c: advertising at the slow (1 s) interval */
void adv_vbus_changed(void);
void proto_vbus_changed(void);

/* VM.md §1 */
enum err_code {
	E_MALFORMED = 1, E_UNKNOWN_OP = 2, E_ARENA = 3, E_FORMAT = 4, E_ARG = 5, E_WINDOW = 6,
	E_SAFE_MODE = 7, E_FONT = 8, E_STACK = 9, E_BUDGET = 10, E_PANEL = 11, E_MIC = 12, E_ASSET = 13,
};

/* VM.md §6.3 */
enum notice_kind { N_LINK = 1, N_PHY = 2, N_PANEL = 3, N_SEC = 4, N_MIC = 5, N_CASE = 6, N_GUARD = 7, N_ASSET = 8 };

/* the panel guard (guard.c, VM.md §6.6): the PMIC thread's loop, the cap at a panel power-on, PANEL_GUARD */
void guard_tick(bool fresh);
uint8_t guard_start_cap(void);
void guard_set(bool on);
bool guard_on(void);
int32_t guard_avg_ua(void);   /* I_avg, -1000 while the panel is off */
void guard_state(uint8_t *out);   /* DIAG GUARD, GUARD_STATE_LEN bytes */
#define GUARD_STATE_LEN 18

static inline void put16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
static inline void put32(uint8_t *p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }
static inline uint16_t get16(const uint8_t *p) { return p[0] | (p[1] << 8); }
static inline uint32_t get32(const uint8_t *p) { return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24); }
static inline uint32_t get24(const uint8_t *p) { return p[0] | (p[1] << 8) | (p[2] << 16); }

#endif
