/* GATT transport of protocol 5 (scouter_kit/g1_firmware/docs/VM.md §6): CMD packets go into the
 * sequence-numbered window (run by the executor, exec.c), CTRL requests and all notifications are
 * handled by the evt thread, so control traffic never waits behind host code or a program. */
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/random/rand32.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/settings/settings.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>
#include "g1.h"
#include "vm.h"

LOG_MODULE_REGISTER(proto, LOG_LEVEL_INF);

#define G1_UUID(n) BT_UUID_128_ENCODE(0xf3da4d59, 0x277a, 0x4fc9, 0xa1ed, 0x41c9c22d4400 + (n))

enum ctrl_op {
	C_HELLO = 0x01, C_PING = 0x02, C_STATUS = 0x03, C_RESET_SESSION = 0x04, C_LINK = 0x05,
	C_LOG_READ = 0x06, C_HEALTH_OK = 0x07, C_REBOOT = 0x08, C_DIAG = 0x09, C_PROPS = 0x0A,
	C_PANEL_READ = 0x0B, C_ARENA_CRC = 0x0C, C_VM_START = 0x10, C_VM_STOP = 0x11, C_VM_STATUS = 0x12,
	C_MIC = 0x13, C_EVENTS = 0x14, C_PANEL_GUARD = 0x16, C_ASSET_INFO = 0x17,
	C_ASSET_LIST = 0x18, C_ASSET_PUT = 0x19, C_ASSET_DELETE = 0x1A, C_ASSET_FORMAT = 0x1B, C_FLASH_CRC = 0x1C,
	C_FC_INFO = 0x1D, C_PROP_SET = 0x1E,
	C_FAULT = 0x3F,
};

/* DIAG (0x09, diagnostics; g1_firmware/README.md): sub-op 1 POWER telemetry, 2 PANEL_PROBE, 3-7 diag.c */
enum diag_op { D_POWER = 1, D_PANEL_PROBE = 2 };

enum evt_type { EV_ACK = 0x01, EV_ERROR = 0x02, EV_NOTICE = 0x05, EV_MIC_DATA = 0x06 };

#define HDR_LEN 9

struct ctrl_req {
	uint8_t len;
	uint32_t t_rx;
	uint8_t data[32];
};

struct evt_msg {
	uint8_t len;
	uint8_t data[32];
};

extern const struct bt_gatt_attr attr_g1_svc[];

struct pkt_slot qslots[QSLOTS];
struct k_spinlock qlock;
uint16_t q_rx_next, q_exec_next;
uint32_t session_epoch;

K_MSGQ_DEFINE(ctrl_q, sizeof(struct ctrl_req), 8, 4);
K_MSGQ_DEFINE(evt_q, sizeof(struct evt_msg), 16, 4);
static K_SEM_DEFINE(evt_sem, 0, 1);
static atomic_t ack_pending, ack_now;
static int64_t ack_due;
static struct bt_conn *conn;
static volatile bool conn_encrypted;
static uint16_t conn_interval, conn_latency, conn_timeout, events_mask;
static uint8_t conn_phy;
static uint32_t pkts_rx;
static uint16_t dups, window_errs, evt_drops;

static struct bt_uuid_128 svc_uuid = BT_UUID_INIT_128(G1_UUID(0));
static struct bt_uuid_128 cmd_uuid = BT_UUID_INIT_128(G1_UUID(1));
static struct bt_uuid_128 ctrl_uuid = BT_UUID_INIT_128(G1_UUID(2));
static struct bt_uuid_128 evt_uuid = BT_UUID_INIT_128(G1_UUID(3));
static struct bt_uuid_128 pair_uuid = BT_UUID_INIT_128(G1_UUID(4));
static struct bt_uuid_128 fc_uuid = BT_UUID_INIT_128(G1_UUID(5));
static struct bt_uuid_128 ancs_uuid = BT_UUID_INIT_128(G1_UUID(6));

/* ANCS's value in the service below: attr_g1_svc[6], EVT's, must keep its place, so new characteristics go last. */
#define ATTR_ANCS 13

bool proto_connected(void) { return conn != NULL; }

/* Out of the case a write needs an encrypted link, which (with CONFIG_BT_BONDING_REQUIRED and
 * bondable only in the case) means a bonded peer; in the case (IN_CASE: VBUS from the NFC charger or a
 * case message within 3 s) every write is obeyed, bonded or not, mirroring the stock arms
 * (docs/protocol.md §8). An unreadable PMIC and a silent tag count as out of the case (fail closed). */
bool proto_writes_allowed(void)
{
	return !IS_ENABLED(CONFIG_G1_REQUIRE_SECURITY) || g1_in_case() || conn_encrypted;
}

/* LINK (VM.md §2.5): a host is connected and its writes are obeyed */
bool proto_link_obeyed(void)
{
	return conn != NULL && proto_writes_allowed();
}

uint32_t proto_link_interval_us(void)
{
	return conn ? conn_interval * 1250u : 0;
}

static void sec_timeout(struct k_work *w);
static K_WORK_DELAYABLE_DEFINE(sec_work, sec_timeout);

static void sec_timeout(struct k_work *w)
{
	if (conn && !conn_encrypted && !g1_in_case()) {
		LOG_WRN("no encrypted link within the deadline, disconnecting");
		bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
	}
}

/* Out of the case: ask the peer to encrypt; a bonded peer does at once, any other is dropped
 * (security_changed) or, if it does not answer, after the deadline (sec_work). */
static void require_encryption(struct bt_conn *c)
{
	if (IS_ENABLED(CONFIG_G1_REQUIRE_SECURITY) && !g1_in_case() && !conn_encrypted) {
		bt_conn_set_security(c, BT_SECURITY_L2);
		k_work_schedule(&sec_work, K_SECONDS(10));
	}
}

static void vbus_work_fn(struct k_work *w)
{
	proto_update_bondable();
	adv_vbus_changed();
	if (conn) {
		require_encryption(conn);
	}
}

static K_WORK_DEFINE(vbus_work, vbus_work_fn);

/* PMIC thread: IN_CASE changed. Leaving the case re-checks a live unencrypted link. */
void proto_vbus_changed(void)
{
	k_work_submit(&vbus_work);
}

static uint16_t cur_mtu(void)
{
	struct bt_conn *c = conn;

	return c ? bt_gatt_get_mtu(c) : 23;
}

void proto_new_epoch(void)
{
	uint32_t old = session_epoch;

	/* The boot counter and the cycle counter make the epoch differ across boots even where the
	 * random number generator repeats (the emulator's does). */
	do {
		session_epoch = sys_rand32_get() ^ (retained_boot_count() * 0x9E3779B9u) ^ k_cycle_get_32();
	} while (session_epoch == old || session_epoch == 0);
}

uint8_t proto_credits(void)
{
	return (uint16_t)(q_exec_next + QSLOTS - q_rx_next);
}

void proto_ack(bool now)
{
	if (now) {
		atomic_set(&ack_now, 1);
	} else if (!atomic_get(&ack_pending)) {
		ack_due = k_uptime_get() + 2;
	}
	atomic_set(&ack_pending, 1);
	k_sem_give(&evt_sem);
}

/* Bluetooth callbacks never wait; with block the executor waits up to 1 s for room (ERROR, HALT) */
static int evt_put(const uint8_t *d, size_t n, bool block)
{
	struct evt_msg m = {.len = n};
	int err;

	memcpy(m.data, d, n);
	err = k_msgq_put(&evt_q, &m, block ? K_MSEC(1000) : K_NO_WAIT);
	if (err) {
		evt_drops++;
	}
	k_sem_give(&evt_sem);
	return err;
}

static size_t evt_head(uint8_t *b, uint8_t type)
{
	b[0] = type;
	put32(&b[1], session_epoch);
	put32(&b[5], now_us());
	return HDR_LEN;
}

static bool subscribed(void)
{
	struct bt_conn *c = conn;

	/* Ask the stack, not a flag: for a bonded peer Zephyr restores the stored CCC before our
	 * connected() runs, and the peer's identical re-write then fires no ccc callback. */
	return c && bt_gatt_is_subscribed(c, &attr_g1_svc[6], BT_GATT_CCC_NOTIFY);
}

/* ERROR for a packet the window refuses (VM.md §6.3): offset 0, the first opcode, no detail */
static void window_error(uint16_t seq, uint8_t op, uint8_t code)
{
	uint8_t b[22] = {0};
	size_t n = evt_head(b, EV_ERROR);

	put16(&b[n], seq);
	b[n + 3] = op;
	b[n + 4] = code;
	evt_put(b, sizeof(b), false);
}

int proto_event(uint8_t type, const uint8_t *body, size_t len, bool block)
{
	uint8_t b[sizeof(((struct evt_msg *)0)->data)];
	size_t n = evt_head(b, type);

	if (!subscribed() || n + len > sizeof(b)) {
		return -ENOTCONN;
	}
	memcpy(&b[n], body, len);
	return evt_put(b, n + len, block);
}

void proto_notice(uint8_t kind, const uint8_t *body, size_t len)
{
	uint8_t b[32];
	size_t n = evt_head(b, EV_NOTICE);

	b[n] = kind;
	len = MIN(len, sizeof(b) - n - 1);
	memcpy(&b[n + 1], body, len);
	evt_put(b, n + 1 + len, false);
}

/* MIC_DATA (VM.md §6.3, §6.5): whole events in a small ring that the evt thread sends; when it is full
 * the oldest is dropped (the frame counter shows the gap) */
#define MIC_SLOTS 4
#define MIC_EVT_LEN (HDR_LEN + 3 + MIC_MAX_FRAMES_BYTES)

static struct {
	uint8_t len;
	uint8_t d[MIC_EVT_LEN];
} mic_ring[MIC_SLOTS];
static uint8_t mic_head, mic_count;
static uint32_t mic_drops;
static struct k_spinlock mic_lock;

void proto_mic_data(uint16_t first, uint8_t n, const uint8_t *frames, size_t len)
{
	if (!subscribed() || len > MIC_MAX_FRAMES_BYTES) {
		return;
	}
	k_spinlock_key_t key = k_spin_lock(&mic_lock);

	if (mic_count == MIC_SLOTS) {
		mic_head = (mic_head + 1) % MIC_SLOTS;
		mic_count--;
		mic_drops++;
	}
	uint8_t *b = mic_ring[(mic_head + mic_count) % MIC_SLOTS].d;
	size_t h = evt_head(b, EV_MIC_DATA);

	put16(&b[h], first);
	b[h + 2] = n;
	memcpy(&b[h + 3], frames, len);
	mic_ring[(mic_head + mic_count) % MIC_SLOTS].len = h + 3 + len;
	mic_count++;
	k_spin_unlock(&mic_lock, key);
	k_sem_give(&evt_sem);
}

void proto_mic_flush(void)
{
	k_spinlock_key_t key = k_spin_lock(&mic_lock);

	mic_count = 0;
	k_spin_unlock(&mic_lock, key);
}

uint32_t proto_mic_drops(void)
{
	return mic_drops;
}

static bool mic_take(uint8_t *d, uint8_t *len)
{
	k_spinlock_key_t key = k_spin_lock(&mic_lock);
	bool any = mic_count > 0;

	if (any) {
		*len = mic_ring[mic_head].len;
		memcpy(d, mic_ring[mic_head].d, *len);
		mic_head = (mic_head + 1) % MIC_SLOTS;
		mic_count--;
	}
	k_spin_unlock(&mic_lock, key);
	return any;
}

static void send(const uint8_t *d, size_t n)
{
	struct bt_conn *c = conn;
	bool sub = subscribed();
	int err;

	if (!sub || n > cur_mtu() - 3) {
		LOG_WRN("event 0x%02x not sent: conn %d notify %d mtu %u len %u", d[0], c != NULL, sub, cur_mtu(),
			(unsigned)n);
		return;
	}
	err = bt_gatt_notify(c, &attr_g1_svc[6], d, n);
	if (err) {
		LOG_WRN("notify 0x%02x failed: %d", d[0], err);
	}
}

/* Returns true when the queue is drained (every received packet has run). */
static bool send_ack(void)
{
	uint8_t b[18];
	uint32_t sack = 0;
	uint16_t rx, ex;
	size_t n = evt_head(b, EV_ACK);
	k_spinlock_key_t key = k_spin_lock(&qlock);

	rx = q_rx_next;
	ex = q_exec_next;
	for (int i = 0; i < 32; i++) {
		uint16_t s = rx + 1 + i;

		if ((uint16_t)(s - ex) < QSLOTS && qslots[s % QSLOTS].full) {
			sack |= 1u << i;
		}
	}
	k_spin_unlock(&qlock, key);
	put16(&b[n], rx);
	put16(&b[n + 2], ex);
	b[n + 4] = (uint16_t)(ex + QSLOTS - rx);
	put32(&b[n + 5], sack);
	send(b, sizeof(b));
	return rx == ex;
}

static ssize_t on_cmd(struct bt_conn *c, const struct bt_gatt_attr *attr, const void *buf, uint16_t len,
		      uint16_t offset, uint8_t flags)
{
	const uint8_t *d = buf;
	struct pkt_slot *s;
	uint16_t seq;
	int16_t diff;
	bool now = false;
	k_spinlock_key_t key;

	if (!proto_writes_allowed()) {
		return len;   /* out of case and not bonded/encrypted: ignore drawing writes */
	}
	if (offset || len < 2 || len > MAX_PKT) {
		window_error(len >= 2 ? get16(d) : 0, len > 2 ? d[2] : 0, E_MALFORMED);
		return len;
	}
	seq = get16(d);
	health_bt_traffic();
	key = k_spin_lock(&qlock);
	diff = (int16_t)(seq - q_exec_next);
	if (diff >= QSLOTS) {
		window_errs++;
		k_spin_unlock(&qlock, key);
		window_error(seq, len > 2 ? d[2] : 0, E_WINDOW);
		proto_ack(true);
		return len;
	}
	s = &qslots[seq % QSLOTS];
	if (diff < 0 || s->full) {
		dups++;
		now = true;
	} else {
		memcpy(s->data, d, len);
		s->len = len;
		s->full = true;
		pkts_rx++;
		if (seq != q_rx_next) {
			now = true;
		}
		while ((uint16_t)(q_rx_next - q_exec_next) < QSLOTS && qslots[q_rx_next % QSLOTS].full) {
			q_rx_next++;
		}
	}
	k_spin_unlock(&qlock, key);
	exec_wake();
	proto_ack(now);
	return len;
}

static ssize_t on_ctrl(struct bt_conn *c, const struct bt_gatt_attr *attr, const void *buf, uint16_t len,
		       uint16_t offset, uint8_t flags)
{
	struct ctrl_req r = {.len = MIN(len, sizeof(r.data)), .t_rx = now_us()};

	if (offset || len < 2) {
		return len;
	}
	if (!proto_writes_allowed()) {
		return len;   /* out of case and not bonded/encrypted: ignore control requests too */
	}
	memcpy(r.data, buf, r.len);
	health_bt_traffic();
	if (k_msgq_put(&ctrl_q, &r, K_NO_WAIT)) {
		evt_drops++;
	}
	k_sem_give(&evt_sem);
	return len;
}

/* PAIR (VM.md §6.1): readable and writable only on an encrypted link, so that a host whose stack pairs on demand
 * (CoreBluetooth has no pairing call) pairs by reading it or writing to it. It only triggers pairing: whether the arm
 * bonds is still proto_update_bondable's rule. */
static ssize_t on_pair_read(struct bt_conn *c, const struct bt_gatt_attr *a, void *buf, uint16_t len, uint16_t off)
{
	const uint8_t encrypted = 1;

	return bt_gatt_attr_read(c, a, buf, len, off, &encrypted, sizeof(encrypted));
}

static ssize_t on_pair_write(struct bt_conn *c, const struct bt_gatt_attr *attr, const void *buf, uint16_t len,
			     uint16_t offset, uint8_t flags)
{
	return len;
}

/* FC (VM.md §6.11): a packet of Faceclaw's records */
static ssize_t on_fc(struct bt_conn *c, const struct bt_gatt_attr *attr, const void *buf, uint16_t len,
		     uint16_t offset, uint8_t flags)
{
	if (!offset && proto_writes_allowed()) {
		health_bt_traffic();
		mic_renew();   /* the host is here, and may be busy writing a big record: not a reason to lose the mic */
		fc_rx(buf, len);
	}
	return len;
}

/* ANCS (VM.md §6.12): the host's commands to the iPhone notification relay. Like MIC, it needs an encrypted link in the
 * case too: the case's rule lets anyone write, and this reads the phone's notifications. */
static ssize_t on_ancs(struct bt_conn *c, const struct bt_gatt_attr *attr, const void *buf, uint16_t len,
		       uint16_t offset, uint8_t flags)
{
	uint8_t err;

	if (offset) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if (safe_level() || (!conn_encrypted && !IS_ENABLED(CONFIG_G1_ANCS_ANY_LINK)) ||
	    !bt_gatt_is_subscribed(c, &attr_g1_svc[ATTR_ANCS], BT_GATT_CCC_NOTIFY)) {
		return BT_GATT_ERR(BT_ATT_ERR_AUTHORIZATION);
	}
	health_bt_traffic();
	err = ancs_command(c, buf, len);
	return err ? BT_GATT_ERR(err) : len;
}

BT_GATT_SERVICE_DEFINE(g1_svc,
	BT_GATT_PRIMARY_SERVICE(&svc_uuid),
	BT_GATT_CHARACTERISTIC(&cmd_uuid.uuid, BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
			       BT_GATT_PERM_WRITE, NULL, on_cmd, NULL),
	BT_GATT_CHARACTERISTIC(&ctrl_uuid.uuid, BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
			       BT_GATT_PERM_WRITE, NULL, on_ctrl, NULL),
	BT_GATT_CHARACTERISTIC(&evt_uuid.uuid, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(&pair_uuid.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT, on_pair_read, on_pair_write, NULL),
	BT_GATT_CHARACTERISTIC(&fc_uuid.uuid, BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
			       BT_GATT_PERM_WRITE, NULL, on_fc, NULL),
	BT_GATT_CHARACTERISTIC(&ancs_uuid.uuid, BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_WRITE, NULL, on_ancs, NULL),
	BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);
BUILD_ASSERT(ATTR_ANCS + 2 == 15, "ATTR_ANCS: the value of the service's last characteristic, then its CCC");

static size_t reply_head(uint8_t *b, uint8_t op, uint8_t tag, uint8_t status)
{
	size_t n = evt_head(b, 0x80 | op);

	b[n] = tag;
	b[n + 1] = status;
	return n + 2;
}

void proto_version(uint8_t *v)
{
	const char *s = CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION;
	int i = 0;

	memset(v, 0, 3);
	for (; *s && i < 3; s++) {
		if (*s == '.' || *s == '+') {
			i++;
		} else if (*s >= '0' && *s <= '9') {
			v[i] = v[i] * 10 + (*s - '0');
		}
	}
}

static uint8_t state_flags(void)
{
	return (safe_level() ? 1 : 0) | (panel_ok() ? 2 : 0) | (pmic_status.present ? 4 : 0) |
	       (panel_is_on() ? 8 : 0) | (pmic_status.charging_enabled ? 16 : 0) |
	       (IS_ENABLED(CONFIG_G1_FAULT_INJECTION) ? 32 : 0) | (conn_encrypted ? 64 : 0) |
	       (g1_in_case() ? 128 : 0);
}

/* HELLO.board bit 2 means MIC 1 captures and streams (VM.md §6.4): the right arm, as stock, with a panel
 * for the indicator; cleared after a capture fault */
static bool has_mic(void)
{
	return !IS_ENABLED(CONFIG_G1_NO_PANEL) && mic_available();
}

/* HELLO.board bit 3 means the ANCS relay answers (VM.md §6.12). */
static bool has_ancs(void)
{
	return !safe_level();
}

/* HELLO body, 35 bytes (VM.md §6.4) */
static size_t hello(uint8_t *b, size_t n)
{
	k_spinlock_key_t key = k_spin_lock(&qlock);
	uint16_t rx = q_rx_next, ex = q_exec_next;

	k_spin_unlock(&qlock, key);
	b[n] = PROTO_VERSION;
	proto_version(&b[n + 1]);
	b[n + 4] = board_role();
	b[n + 5] = state_flags();
	b[n + 6] = (IS_ENABLED(CONFIG_G1_NO_PANEL) ? 1 : 0) | (proto_bondable() ? 2 : 0) | (has_mic() ? 4 : 0) |
		   (has_ancs() ? 8 : 0);
	put16(&b[n + 7], exec_triggers());
	put16(&b[n + 9], PANEL_W);
	put16(&b[n + 11], PANEL_H);
	put32(&b[n + 13], ARENA_SIZE);
	b[n + 17] = QSLOTS;
	put16(&b[n + 18], MIN(cur_mtu() - 3, MAX_PKT));
	put16(&b[n + 20], rx);
	put16(&b[n + 22], ex);
	b[n + 24] = (uint16_t)(ex + QSLOTS - rx);
	put32(&b[n + 25], retained_boot_count());
	put32(&b[n + 29], retained_resetreas());
	b[n + 33] = safe_level();
	b[n + 34] = retained_unhealthy();
	return n + 35;
}

/* STATUS body, 48 bytes (VM.md §6.4) */
static size_t status(uint8_t *b, size_t n)
{
	const struct vm_stats *st = exec_stats();

	put16(&b[n], pmic_status.vbat_mv);
	put16(&b[n + 2], pmic_status.ntc_code);
	b[n + 4] = pmic_status.charge_status;
	b[n + 5] = pmic_status.vbus_status;
	b[n + 6] = pmic_status.vterm_code;
	b[n + 7] = (pmic_status.present ? 1 : 0) | (pmic_status.configured ? 2 : 0) |
		   (pmic_status.charging_enabled ? 4 : 0) | (pmic_status.wdt_armed ? 8 : 0);
	b[n + 8] = pmic_status.refuse_reason;
	put32(&b[n + 9], pkts_rx);
	put32(&b[n + 13], st->instructions);
	put16(&b[n + 17], st->errors);
	put32(&b[n + 19], st->draws);
	put16(&b[n + 23], st->late);
	put16(&b[n + 25], dups);
	put16(&b[n + 27], window_errs);
	put16(&b[n + 29], evt_drops);
	put16(&b[n + 31], conn_interval);
	put16(&b[n + 33], conn_latency);
	put16(&b[n + 35], conn_timeout);
	b[n + 37] = conn_phy;
	put16(&b[n + 38], cur_mtu());
	put32(&b[n + 40], k_uptime_get_32());
	put32(&b[n + 44], pmic_status.i2c_errors);
	return n + 48;
}

static uint8_t mic_per_event(uint8_t frame_bytes)
{
	size_t room = cur_mtu() > 18 ? cur_mtu() - 3 - 12 : 0;

	return CLAMP(MIN(room, MIC_MAX_FRAMES_BYTES) / frame_bytes, 1, MIC_MAX_PER_EVENT);
}

/* MIC (VM.md §6.5): on needs a microphone with capture, an encrypted link (in the case too) and a
 * panel; the executor then shows the indicator and starts the capture. */
static int mic_request(bool on, uint8_t rate)
{
	if (rate > 1) {
		return E_ARG;
	}
	if (!on) {
		exec_mic(false, 0, 0);
		return 0;
	}
	if (safe_level()) {
		return E_SAFE_MODE;
	}
	if (!has_mic() || (!conn_encrypted && !IS_ENABLED(CONFIG_G1_MIC_ANY_LINK))) {
		return E_MIC;
	}
	if (!panel_ok()) {
		return E_PANEL;
	}
	exec_mic(true, rate, mic_per_event(rate ? 20 : 40));
	return 0;
}

static void handle_ctrl(struct ctrl_req *r)
{
	static uint8_t b[MAX_PKT];
	uint8_t op = r->data[0], tag = r->data[1];
	const uint8_t *a = &r->data[2];
	size_t alen = r->len - 2;
	size_t n = reply_head(b, op, tag, 0);
	uint8_t *st = &b[n - 1];

	LOG_DBG("ctrl 0x%02x tag %u", op, tag);

	switch (op) {
	case C_HELLO:
		n = hello(b, n);
		break;
	case C_PING:
		if (alen < 4) {
			*st = E_MALFORMED;
			break;
		}
		memcpy(&b[n], a, 4);
		put32(&b[n + 4], r->t_rx);
		n += 12;
		break;
	case C_STATUS:
		n = status(b, n);
		break;
	case C_RESET_SESSION: {
		uint16_t flushed = exec_reset_session(alen > 0 && (a[0] & 1));

		n = reply_head(b, op, tag, 0);
		put16(&b[n], flushed);
		n = hello(b, n + 2);
		break;
	}
	case C_LINK: {
		struct bt_le_conn_param p = {
			.interval_min = get16(a), .interval_max = get16(a + 2),
			.latency = get16(a + 4), .timeout = get16(a + 6),
		};
		int e1 = -ENOTCONN, e2 = 0;

		if (alen < 9) {
			*st = E_MALFORMED;
			break;
		}
		if (conn) {
			e1 = bt_conn_le_param_update(conn, &p);
			if (a[8]) {
				struct bt_conn_le_phy_param phy = {.pref_tx_phy = a[8], .pref_rx_phy = a[8]};

				e2 = bt_conn_le_phy_update(conn, &phy);
			}
		}
		b[n++] = (int8_t)e1;
		b[n++] = (int8_t)e2;
		break;
	}
	case C_LOG_READ: {
		uint32_t total;
		size_t max = MIN(cur_mtu() - 3, MAX_PKT) - n - 4;
		size_t got = retained_log_read(alen >= 2 ? get16(a) : 0, &b[n + 4], max, &total);

		put16(&b[n], total);
		put16(&b[n + 2], alen >= 2 ? get16(a) : 0);
		n += 4 + got;
		break;
	}
	case C_HEALTH_OK:
		health_host_ok();
		b[n++] = retained_unhealthy();
		break;
	case C_REBOOT:
		send(b, n);
		k_msleep(200);
		g1_reboot(CLEAN_HOST, alen > 0 && a[0] == 1, alen > 0 && a[0] == 2);
		return;
	case C_DIAG:
		if (alen >= 1 && a[0] == D_POWER) {   /* u8 page (0 live, 1..8 state accumulators) */
			uint8_t page = alen > 1 ? a[1] : 0;

			if (page > 8) {
				*st = E_ARG;
				break;
			}
			n = power_reply(page, b, n);
		} else if (alen >= 5 && a[0] == D_PANEL_PROBE) {   /* u8 MHz, u8 flags, u16 delay ms */
			int err = safe_level() ? -EPERM : panel_probe(a[1], a[2], get16(&a[3]), &b[n]);

			*st = !err ? 0 : err == -EINVAL ? E_ARG : err == -EPERM ? E_SAFE_MODE : E_PANEL;
			if (!err || err == -EIO) {
				n += PANEL_PROBE_LEN;
			}
		} else {
			bool reboot = false;

			n = diag_request(a, alen, b, n, st, &reboot);
			if (reboot && *st == 0) {
				send(b, n);
				k_msleep(200);
				g1_reboot(CLEAN_HOST, false, false);
				return;
			}
		}
		break;
	case C_PROPS:
		if (alen > 12) {
			*st = E_MALFORMED;
			break;
		}
		for (size_t i = 0; i < alen; i++) {
			uint32_t v;

			if (exec_prop(a[i], &v) != 0) {
				*st = E_ARG;
				n = reply_head(b, op, tag, E_ARG);
				break;
			}
			put32(&b[n], v);
			n += 4;
		}
		break;
	case C_PANEL_READ: {   /* u16 y0, u16 rows -> u32 crc32, u16 rows read, 16 bytes of the first row */
		int err = alen < 4 ? -EINVAL : panel_read_rows(get16(a), get16(&a[2]), &b[n]);

		*st = !err ? 0 : err == -EINVAL ? E_ARG : E_PANEL;
		if (!err || err == -EIO) {
			n += PANEL_READ_LEN;
		}
		break;
	}
	case C_ARENA_CRC: {   /* u32 address, u32 length (0 = to the end) -> u32 crc32, 16 bytes */
		uint32_t addr = alen >= 8 ? get32(a) : 0, len = alen >= 8 ? get32(&a[4]) : 0;

		if (alen < 8) {
			*st = E_MALFORMED;
			break;
		}
		if (addr > ARENA_SIZE || (len == 0 && addr == ARENA_SIZE)) {
			*st = E_ARENA;
			break;
		}
		if (len == 0) {
			len = ARENA_SIZE - addr;
		}
		if ((uint64_t)addr + len > ARENA_SIZE) {
			*st = E_ARENA;
			break;
		}
		put32(&b[n], exec_arena_crc(addr, len, &b[n + 4]));
		n += 20;
		break;
	}
	case C_VM_START:
		*st = alen < 4 ? E_MALFORMED : exec_vm_start(get32(a));
		break;
	case C_VM_STOP:
		exec_vm_stop();
		break;
	case C_VM_STATUS:
		exec_vm_status(&b[n]);
		n += 32;
		break;
	case C_MIC: {
		int err = mic_request(alen > 0 && a[0], alen > 1 ? a[1] : 0);

		*st = err;
		if (!err) {
			uint8_t fb = alen > 1 && a[1] ? 20 : 40;

			b[n] = alen > 0 && a[0];
			b[n + 1] = fb;
			b[n + 2] = mic_per_event(fb);
			put16(&b[n + 3], 16000);
			b[n + 5] = 10;
			put16(&b[n + 6], MIC_LEASE_MS);
			n += 8;
		}
		break;
	}
	case C_EVENTS:
		if (alen >= 2) {
			events_mask = exec_events(get16(a));
		}
		put16(&b[n], events_mask);
		n += 2;
		break;
	case C_PANEL_GUARD:   /* u8 on; no body = query; 1 at boot, kept across disconnection and RESET_SESSION */
		if (alen >= 1 && a[0] > 1) {
			*st = E_ARG;
			break;
		}
		if (alen >= 1) {
			guard_set(a[0]);
		}
		b[n++] = guard_on();
		break;
	case C_ASSET_INFO:
		assets_info(&b[n]);
		n += 20;
		break;
	case C_ASSET_LIST:
		n += assets_list(alen ? a[0] : 0, &b[n], MIN(cur_mtu() - 3, MAX_PKT) - n);
		break;
	case C_ASSET_PUT: {   /* id[16], u32 arena address, u32 size, u8 type -> u8 result (0 started, 1 stored) */
		uint8_t result = 0;

		*st = alen < 25 ? E_MALFORMED : assets_put(a, get32(&a[16]), get32(&a[20]), a[24], &result);
		if (!*st) {
			b[n++] = result;
		}
		break;
	}
	case C_ASSET_DELETE:
		*st = alen < 16 ? E_MALFORMED : assets_delete(a);
		break;
	case C_ASSET_FORMAT:
		*st = assets_format(a, alen);
		break;
	case C_FLASH_CRC:
		*st = alen < 8 ? E_MALFORMED : assets_crc(get32(a), get32(&a[4]));
		break;
	case C_FAULT:
		if (!IS_ENABLED(CONFIG_G1_FAULT_INJECTION)) {
			*st = E_UNKNOWN_OP;
			break;
		}
		send(b, n);
		k_msleep(50);
		LOG_WRN("FAULT %u requested", a[0]);
		switch (a[0]) {
		case 1:
			__builtin_trap();
			break;
		case 2:
			irq_lock();
			for (;;) {
			}
		case 3:
			k_panic();
			break;
		case 4:
			exec_hang_for_test();
			break;
		case 5:
			pmic_stop_kicking();
			break;
		}
		return;
	case C_PROP_SET:
		/* (u8 id, u32 value) pairs; each is what SYS_WRITE does, in order, and its error only goes to the log */
		if (alen == 0 || alen % 5) {
			*st = E_MALFORMED;
			break;
		}
		for (size_t i = 0; i < alen; i += 5) {
			exec_prop_set(a[i], get32(a + i + 1));
		}
		break;
	case C_FC_INFO:
		if (safe_level() != 0) {
			*st = E_SAFE_MODE;
			break;
		}
		n += fc_info(&b[n]);
		break;
	default:
		*st = E_UNKNOWN_OP;
		break;
	}
	if (op == C_PING && *st == 0) {
		put32(&b[n - 4], now_us());
	}
	send(b, n);
}

/* The ACK that reports a drained queue is sent once more ACK_REPEAT_MS later, unless another ACK
 * went out meanwhile: a lost last ACK then does not leave the host waiting (VM.md §6.2). */
#define ACK_REPEAT_MS 100
/* Frames of the ANCS relay sent per pass of the thread, so that ACKs and control replies get their turn. */
#define ANCS_PER_PASS 8

void proto_ancs_kick(void)
{
	k_sem_give(&evt_sem);
}

static void send_ancs(void)
{
	static uint8_t d[MAX_PKT];
	struct bt_conn *c;
	uint8_t len;
	int sent = 0;

	while (sent < ANCS_PER_PASS && (c = ancs_take(d, &len)) != NULL) {
		if (bt_gatt_is_subscribed(c, &attr_g1_svc[ATTR_ANCS], BT_GATT_CCC_NOTIFY)) {
			int err = bt_gatt_notify(c, &attr_g1_svc[ATTR_ANCS], d, len);

			if (err) {
				LOG_WRN("ANCS notify failed: %d", err);
			}
		}
		bt_conn_unref(c);
		sent++;
	}
	if (sent) {
		memset(d, 0, sizeof(d));
	}
	if (sent == ANCS_PER_PASS) {
		k_sem_give(&evt_sem);
	}
}

/* A central can settle on a supervision timeout of a few hundred ms (420 ms on a Pixel, while it juggled links), which a
 * burst of 2.4 GHz interference outlasts. Ask for one that rides it out, with the same interval and latency, a few
 * times per connection: the central decides. */
#define LINK_MIN_TIMEOUT 200 /* x 10 ms */
#define LINK_ASK_TIMEOUT 400
#define LINK_ASK_GAP_MS 5000
#define LINK_ASKS 5

static uint8_t link_asks;
static int64_t link_ask_at;

static void link_guard(int64_t now)
{
	if (!conn || !conn_timeout || conn_timeout >= LINK_MIN_TIMEOUT || link_asks >= LINK_ASKS || now < link_ask_at) {
		return;
	}
	struct bt_le_conn_param p = {
		.interval_min = conn_interval, .interval_max = conn_interval,
		.latency = conn_latency, .timeout = LINK_ASK_TIMEOUT,
	};
	int err = bt_conn_le_param_update(conn, &p);

	link_asks++;
	link_ask_at = now + LINK_ASK_GAP_MS;
	LOG_INF("link: timeout %u x 10 ms is short, asked for %u: %d", conn_timeout, LINK_ASK_TIMEOUT, err);
}

static void evt_thread(void *p1, void *p2, void *p3)
{
	static uint8_t mic_buf[MIC_EVT_LEN];
	int64_t repeat_due = 0;
	uint8_t mic_len;

	for (;;) {
		struct ctrl_req r;
		struct evt_msg m;
		int64_t now = k_uptime_get();
		int64_t due = repeat_due ? repeat_due : now + 1000;

		if (atomic_get(&ack_now)) {
			due = now;
		} else if (atomic_get(&ack_pending)) {
			due = MIN(due, ack_due);
		}
		k_sem_take(&evt_sem, due > now ? K_MSEC(due - now) : K_NO_WAIT);
		while (k_msgq_get(&ctrl_q, &r, K_NO_WAIT) == 0) {
			handle_ctrl(&r);
		}
		while (k_msgq_get(&evt_q, &m, K_NO_WAIT) == 0) {
			send(m.data, m.len);
		}
		while (mic_take(mic_buf, &mic_len)) {
			send(mic_buf, mic_len);
		}
		send_ancs();
		now = k_uptime_get();
		link_guard(now);
		if (atomic_get(&ack_pending) && (atomic_get(&ack_now) || now >= ack_due)) {
			atomic_set(&ack_pending, 0);
			atomic_set(&ack_now, 0);
			repeat_due = send_ack() ? now + ACK_REPEAT_MS : 0;
		} else if (repeat_due && now >= repeat_due) {
			repeat_due = 0;
			if (conn) {
				send_ack();
			}
		}
	}
}

K_THREAD_DEFINE(evt, 2048, evt_thread, NULL, NULL, NULL, 3, 0, K_TICKS_FOREVER);

static void connected(struct bt_conn *c, uint8_t err)
{
	struct bt_conn_info info;

	if (err) {
		return;
	}
	conn = bt_conn_ref(c);
	conn_encrypted = false;
	link_asks = 0;
	link_ask_at = k_uptime_get() + LINK_ASK_GAP_MS;
	if (bt_conn_get_info(c, &info) == 0) {
		conn_interval = info.le.interval;
		conn_latency = info.le.latency;
		conn_timeout = info.le.timeout;
	}
	bt_conn_le_data_len_update(c, BT_LE_DATA_LEN_PARAM_MAX);
	LOG_INF("connected, interval %u x 1.25 ms", conn_interval);
	proto_update_bondable();
	require_encryption(c);
}

static void disconnected(struct bt_conn *c, uint8_t reason)
{
	if (conn == c) {
		bt_conn_unref(conn);
		conn = NULL;
	}
	conn_encrypted = false;
	events_mask = exec_events(0);
	exec_link_down();
	fc_link_down();
	ancs_link_down();
	LOG_INF("disconnected (reason 0x%02x)", reason);
	adv_kick();
}

static void param_updated(struct bt_conn *c, uint16_t interval, uint16_t latency, uint16_t timeout)
{
	uint8_t body[6];

	conn_interval = interval;
	conn_latency = latency;
	conn_timeout = timeout;
	put16(body, interval);
	put16(body + 2, latency);
	put16(body + 4, timeout);
	LOG_INF("link: interval %u x 1.25 ms, latency %u, timeout %u x 10 ms", interval, latency, timeout);
	proto_notice(N_LINK, body, sizeof(body));
}

static void phy_updated(struct bt_conn *c, struct bt_conn_le_phy_info *p)
{
	conn_phy = p->tx_phy | (p->rx_phy << 4);
	proto_notice(N_PHY, (const uint8_t[]){p->tx_phy, p->rx_phy}, 2);
}

static void security_changed(struct bt_conn *c, bt_security_t level, enum bt_security_err err)
{
	conn_encrypted = (err == BT_SECURITY_ERR_SUCCESS && level >= BT_SECURITY_L2);
	LOG_INF("security level %u err %u (encrypted=%d)", level, err, conn_encrypted);
	if (!conn_encrypted) {
		exec_link_down();
	}
	if (conn_encrypted) {
		k_work_cancel_delayable(&sec_work);
		proto_notice(N_SEC, (const uint8_t[]){1}, 1);
	} else if (IS_ENABLED(CONFIG_G1_REQUIRE_SECURITY) && !g1_in_case() && conn) {
		LOG_WRN("pairing/encryption failed (err %u), disconnecting", err);
		bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
	}
}

BT_CONN_CB_DEFINE(proto_conn_cb) = {
	.connected = connected,
	.disconnected = disconnected,
	.le_param_updated = param_updated,
	.le_phy_updated = phy_updated,
	.security_changed = security_changed,
};

static bool bondable;

static void window_end(struct k_work *w)
{
	proto_update_bondable();
}

static K_WORK_DELAYABLE_DEFINE(window_work, window_end);

bool proto_bondable(void)
{
	return bondable;
}

/* Bondable only in the case (user's rule). CONFIG_G1_PAIRING_WINDOW_S > 0 adds a window after boot
 * (used by the nRF5340 DK build, which has no PMIC and so is never "in the case"); 0 on the arm. */
void proto_update_bondable(void)
{
	int64_t left = (int64_t)CONFIG_G1_PAIRING_WINDOW_S * 1000 - k_uptime_get();

	bondable = g1_in_case() || left > 0;
	bt_set_bondable(bondable);
	if (left > 0) {
		k_work_schedule(&window_work, K_MSEC(left + 1));
	}
}

int proto_init(void)
{
	proto_new_epoch();
	k_thread_start(evt);
	return 0;
}
