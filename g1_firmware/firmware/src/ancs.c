/* The iPhone notification relay (VM.md §6.12): the arm is the ANCS consumer, as a GATT client on its one link, and hands
 * what it hears to the app at the other end of that same link. The app owns everything else: it asks for attributes,
 * keeps the notifications and draws them. Nothing here reads a notification, and nothing is kept past the link.
 *
 *   BT thread:  ancs_command(), the GATT callbacks and ancs_link_down() run the state machine and fill the ring
 *   evt thread: ancs_take() empties the ring, because a notification may wait for a buffer and the BT thread must not
 *
 * Without a START from the host this file does nothing, so a host that isn't an iPhone never sees a GATT request.
 *
 * The authority is the connection, not the token: a frame in the ring goes to the connection that asked for it, through
 * a reference that also keeps the connection object from being reused. The token only lets the host reject old frames.
 * There is no lease and no way back to a stock consumer, because g1v0 has none: the link's end is what ends the relay. */
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>
#include "g1.h"

LOG_MODULE_REGISTER(ancs, LOG_LEVEL_INF);

#define CMD_HEAD 8
#define MAX_CMD 80
#define FRAME_HEAD 11
#define MAX_VALUE 512
/* About 400 Notification Source frames, or several attribute responses, behind the link. */
#define RING_SIZE 8192

enum { CMD_START = 0, CMD_STOP = 1, CMD_CP = 2 };
enum { KIND_STATUS = 0, KIND_NS = 1, KIND_DS = 2, KIND_CP_DONE = 3 };
enum { STATUS_RESETTING = 0, STATUS_READY = 1, STATUS_FAILED = 2, STATUS_NO_ANCS = 3 };
enum { FLAG_FIRST = 1, FLAG_LAST = 2 };

enum stage {
	S_IDLE,
	S_CLOSE_NS, S_CLOSE_DS,                                     /* a CCC = 0 is out */
	S_FIND_SERVICE, S_FIND_CHARS, S_FIND_NS_CCC, S_FIND_DS_CCC,
	S_SUB_DS, S_NS_OFF, S_SUB_NS,                               /* a CCC write is out */
	S_ACTIVE,
};

static struct bt_uuid_128 ancs_uuid = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x7905f431, 0xb5ce, 0x4e99, 0xa40f, 0x4b1e122d00d0));
static struct bt_uuid_128 ns_uuid = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x9fbf120d, 0x6301, 0x42d9, 0x8c58, 0x25e699a21dbd));
static struct bt_uuid_128 cp_uuid = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x69d1d8f3, 0x45e1, 0x49a8, 0x9821, 0x9bbdfdaad9d9));
static struct bt_uuid_128 ds_uuid = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x22eac6e9, 0x24d6, 0x4bb5, 0xbe44, 0xb36ace7c7bfb));
static struct bt_uuid_16 ccc_uuid = BT_UUID_INIT_16(BT_UUID_GATT_CCC_VAL);

/* the ring and the connection it sends on; the BT thread alone writes relay_conn, under the lock */
static struct k_spinlock ring_lock;
static uint8_t ring[RING_SIZE];
static uint32_t ring_head, ring_used, ring_drops;
static struct bt_conn *relay_conn;

/* BT thread only */
static enum stage stage;
static uint32_t token;
static uint16_t sequence;
static bool stop_pending;   /* STOP came while a request was out: it takes effect at the answer */
static bool ns_on, ds_on;   /* this link's CCC was or may have been set */
static bool subs_added;     /* ns_sub and ds_sub are in Zephyr's list, which they leave when the link ends */
static bool cp_busy;
static uint16_t service_end, ns_value, ns_ccc, cp_value, ds_value, ds_ccc;
static struct bt_gatt_discover_params discover;
static struct bt_gatt_subscribe_params ns_sub, ds_sub;
static struct bt_gatt_write_params ccc_write, cp_write;
static uint8_t cp_buf[MAX_CMD - CMD_HEAD];
static const uint8_t ccc_off[2], ccc_on[2] = {BT_GATT_CCC_NOTIFY, 0};

static void find_service(void);
static void close_relay(void);

static void ring_flush(void)
{
	k_spinlock_key_t key = k_spin_lock(&ring_lock);

	memset(ring, 0, sizeof(ring));
	ring_head = ring_used = 0;
	k_spin_unlock(&ring_lock, key);
}

/* A frame in the ring: u8 length, the head, the body. */
static bool ring_put(const uint8_t *head, const uint8_t *body, size_t n)
{
	size_t total = 1 + FRAME_HEAD + n;
	k_spinlock_key_t key = k_spin_lock(&ring_lock);
	bool room = ring_used + total <= RING_SIZE;

	if (room) {
		uint32_t at = (ring_head + ring_used) % RING_SIZE;

		ring[at] = FRAME_HEAD + n;
		for (size_t i = 0; i < FRAME_HEAD + n; i++) {
			ring[(at + 1 + i) % RING_SIZE] = i < FRAME_HEAD ? head[i] : body[i - FRAME_HEAD];
		}
		ring_used += total;
	} else {
		ring_drops++;
	}
	k_spin_unlock(&ring_lock, key);
	return room;
}

/* The next frame and a reference to the connection it belongs to, which the caller drops; NULL when there is none.
 * What the host has been sent is erased from the ring. */
struct bt_conn *ancs_take(uint8_t *out, uint8_t *len)
{
	struct bt_conn *c = NULL;
	k_spinlock_key_t key = k_spin_lock(&ring_lock);

	if (ring_used && relay_conn) {
		uint8_t n = ring[ring_head];

		ring[ring_head] = 0;
		for (uint8_t i = 0; i < n; i++) {
			uint32_t at = (ring_head + 1 + i) % RING_SIZE;

			out[i] = ring[at];
			ring[at] = 0;
		}
		ring_head = (ring_head + 1 + n) % RING_SIZE;
		ring_used -= 1 + n;
		*len = n;
		c = bt_conn_ref(relay_conn);
	}
	k_spin_unlock(&ring_lock, key);
	return c;
}

/* One value as frames that fit the MTU. A frame the ring has no room for still takes its sequence number, so the host
 * sees the gap and starts over. */
static void emit(uint8_t kind, const uint8_t *data, uint16_t len)
{
	uint16_t mtu = relay_conn ? bt_gatt_get_mtu(relay_conn) : 0;
	uint16_t room, off = 0;

	if (mtu < 23) {
		return;
	}
	room = MIN(mtu - 3, MAX_PKT) - FRAME_HEAD;
	do {
		uint16_t count = MIN(len - off, room);
		uint8_t head[FRAME_HEAD] = {'A', 'N', 1, kind};

		put32(&head[4], token);
		put16(&head[8], sequence++);
		head[10] = (off == 0 ? FLAG_FIRST : 0) | (off + count == len ? FLAG_LAST : 0);
		if (!ring_put(head, data ? data + off : NULL, count)) {
			LOG_WRN("ring full, frame dropped");
		}
		off += count;
	} while (off < len);
	proto_ancs_kick();
}

static void emit_status(uint8_t status)
{
	emit(KIND_STATUS, &status, 1);
}

static void fail(uint8_t status)
{
	LOG_WRN("relay failed in stage %d: status %u", stage, status);
	emit_status(status);
	close_relay();
}

static void ccc_done(struct bt_conn *c, uint8_t err, struct bt_gatt_write_params *p);

/* The relay writes its CCCs itself. bt_gatt_subscribe's and bt_gatt_unsubscribe's writes report to no one when the phone
 * refuses the unsubscribe of the last subscription (gatt_write_ccc_rsp finds no list and returns), and iOS can refuse
 * once it has withdrawn ANCS; bt_gatt_write always answers, also when the link ends. */
static int write_ccc(uint16_t handle, const uint8_t *value)
{
	ccc_write = (struct bt_gatt_write_params){.func = ccc_done, .handle = handle, .data = value, .length = 2};
	return bt_gatt_write(relay_conn, &ccc_write);
}

/* CCC = 0 for what was set, one write at a time, then idle. A refused or impossible write doesn't matter: what still
 * arrives is ignored, and the link's end clears it. */
static void close_next(void)
{
	while (ns_on || ds_on) {
		uint16_t handle = ns_on ? ns_ccc : ds_ccc;

		stage = ns_on ? S_CLOSE_NS : S_CLOSE_DS;
		if (ns_on) {
			ns_on = false;
		} else {
			ds_on = false;
		}
		if (handle && !write_ccc(handle, ccc_off)) {
			return;
		}
	}
	stage = S_IDLE;
	token = 0;
}

static void close_relay(void)
{
	stop_pending = false;
	close_next();
}

/* True when a STOP that waited for this answer took over. */
static bool stop_taken(void)
{
	if (!stop_pending) {
		return false;
	}
	close_relay();
	return true;
}

static uint8_t received(struct bt_conn *c, struct bt_gatt_subscribe_params *p, const void *data, uint16_t len)
{
	if (!data) {
		/* Zephyr took the subscriptions off its list: they go together. */
		subs_added = false;
		return BT_GATT_ITER_STOP;
	}
	if (stop_pending) {
		return BT_GATT_ITER_CONTINUE;
	}
	/* iOS may send before it answers the CCC write; the host holds these until it is told the relay is ready. */
	if (p == &ns_sub && (stage == S_SUB_NS || stage == S_ACTIVE) && len == 8) {
		emit(KIND_NS, data, len);
	} else if (p == &ds_sub && stage == S_ACTIVE && len && len <= MAX_VALUE) {
		emit(KIND_DS, data, len);
	}
	return BT_GATT_ITER_CONTINUE;
}

/* Where Zephyr delivers the two characteristics. No CCC write here; never restored at a later connection. */
static int add_sub(const struct bt_conn_info *info, struct bt_gatt_subscribe_params *p)
{
	memset(p, 0, sizeof(*p));
	p->notify = received;
	p->value = BT_GATT_CCC_NOTIFY;
	p->ccc_handle = p == &ns_sub ? ns_ccc : ds_ccc;
	p->value_handle = p == &ns_sub ? ns_value : ds_value;
	atomic_set_bit(p->flags, BT_GATT_SUBSCRIBE_FLAG_VOLATILE);
	return bt_gatt_resubscribe(info->id, info->le.dst, p);
}

/* The handles are known. A subscription from an earlier session of this link only needs them updated. */
static void subscribe(void)
{
	struct bt_conn_info info;

	if (subs_added) {
		ns_sub.value_handle = ns_value;
		ns_sub.ccc_handle = ns_ccc;
		ds_sub.value_handle = ds_value;
		ds_sub.ccc_handle = ds_ccc;
	} else {
		if (bt_conn_get_info(relay_conn, &info) || add_sub(&info, &ns_sub)) {
			fail(STATUS_FAILED);
			return;
		}
		subs_added = true;
		if (add_sub(&info, &ds_sub)) {
			fail(STATUS_FAILED);
			return;
		}
	}
	stage = S_SUB_DS;
	ds_on = true;
	if (write_ccc(ds_ccc, ccc_on)) {
		fail(STATUS_FAILED);
	}
}

static void ccc_done(struct bt_conn *c, uint8_t err, struct bt_gatt_write_params *p)
{
	if (stage == S_CLOSE_NS || stage == S_CLOSE_DS) {
		close_next();
		return;
	}
	if (stop_taken()) {
		return;
	}
	if (err) {
		fail(STATUS_FAILED);
		return;
	}
	switch (stage) {
	case S_SUB_DS:
		/* Set from an earlier session or not, only a new subscription makes iOS list what it has. */
		stage = S_NS_OFF;
		if (write_ccc(ns_ccc, ccc_off)) {
			fail(STATUS_FAILED);
		}
		break;
	case S_NS_OFF:
		stage = S_SUB_NS;
		ns_on = true;
		if (write_ccc(ns_ccc, ccc_on)) {
			fail(STATUS_FAILED);
		}
		break;
	case S_SUB_NS:
		stage = S_ACTIVE;
		LOG_INF("relay ready");
		emit_status(STATUS_READY);
		break;
	default:
		break;
	}
}

static uint8_t found(struct bt_conn *c, const struct bt_gatt_attr *attr, struct bt_gatt_discover_params *p);

static void find(enum stage next, uint8_t type, const struct bt_uuid *uuid, uint16_t start, uint16_t end)
{
	stage = next;
	discover = (struct bt_gatt_discover_params){.uuid = uuid, .func = found, .type = type, .end_handle = end};
	discover.start_handle = start;
	if (start > end || bt_gatt_discover(relay_conn, &discover)) {
		fail(STATUS_FAILED);
	}
}

/* iOS adds and removes ANCS as it pleases (Share System Notifications), so each session looks for it again. */
static void find_service(void)
{
	service_end = ns_value = ns_ccc = cp_value = ds_value = ds_ccc = 0;
	find(S_FIND_SERVICE, BT_GATT_DISCOVER_PRIMARY, &ancs_uuid.uuid, BT_ATT_FIRST_ATTRIBUTE_HANDLE,
	     BT_ATT_LAST_ATTRIBUTE_HANDLE);
}

static uint8_t found(struct bt_conn *c, const struct bt_gatt_attr *attr, struct bt_gatt_discover_params *p)
{
	if (stop_taken()) {
		return BT_GATT_ITER_STOP;
	}
	switch (stage) {
	case S_FIND_SERVICE:
		if (!attr) {
			fail(STATUS_NO_ANCS);
			break;
		}
		service_end = ((struct bt_gatt_service_val *)attr->user_data)->end_handle;
		find(S_FIND_CHARS, BT_GATT_DISCOVER_CHARACTERISTIC, NULL, attr->handle + 1, service_end);
		break;
	case S_FIND_CHARS:
		if (attr) {
			const struct bt_gatt_chrc *chrc = attr->user_data;

			if (!bt_uuid_cmp(chrc->uuid, &ns_uuid.uuid)) {
				ns_value = chrc->value_handle;
			} else if (!bt_uuid_cmp(chrc->uuid, &cp_uuid.uuid)) {
				cp_value = chrc->value_handle;
			} else if (!bt_uuid_cmp(chrc->uuid, &ds_uuid.uuid)) {
				ds_value = chrc->value_handle;
			}
			return BT_GATT_ITER_CONTINUE;
		}
		if (!ns_value || !cp_value || !ds_value) {
			fail(STATUS_FAILED);
			break;
		}
		find(S_FIND_NS_CCC, BT_GATT_DISCOVER_DESCRIPTOR, &ccc_uuid.uuid, ns_value + 1, service_end);
		break;
	case S_FIND_NS_CCC:
		if (!attr) {
			fail(STATUS_FAILED);
			break;
		}
		ns_ccc = attr->handle;
		find(S_FIND_DS_CCC, BT_GATT_DISCOVER_DESCRIPTOR, &ccc_uuid.uuid, ds_value + 1, service_end);
		break;
	case S_FIND_DS_CCC:
		if (!attr) {
			fail(STATUS_FAILED);
			break;
		}
		ds_ccc = attr->handle;
		subscribe();
		break;
	default:
		break;
	}
	return BT_GATT_ITER_STOP;
}

static void cp_done(struct bt_conn *c, uint8_t err, struct bt_gatt_write_params *p)
{
	cp_busy = false;
	if (stage != S_ACTIVE) {
		return;
	}
	if (err) {
		emit_status(STATUS_FAILED);
	} else {
		emit(KIND_CP_DONE, NULL, 0);
	}
}

/* Only ANCS's own commands: this must never become a way to write any handle on the phone. */
static bool cp_valid(const uint8_t *cp, uint16_t size)
{
	return (cp[0] == 0 && size >= 6) ||
	       (cp[0] == 1 && size >= 4 && cp[size - 2] == 0 && cp[size - 1] == 0) ||
	       (cp[0] == 2 && size == 6 && cp[5] <= 1);
}

/* A write to ANCS from the link's host: 0 or an ATT error. Who may write was checked by the caller. */
uint8_t ancs_command(struct bt_conn *c, const uint8_t *v, uint16_t len)
{
	uint32_t t;

	if (len < CMD_HEAD || v[0] != 'A' || v[1] != 'N' || v[2] != 1 || v[3] > CMD_CP) {
		return BT_ATT_ERR_INVALID_ATTRIBUTE_LEN;
	}
	t = get32(&v[4]);
	if (!t) {
		return BT_ATT_ERR_INVALID_ATTRIBUTE_LEN;
	}
	switch (v[3]) {
	case CMD_START:
		if (len != CMD_HEAD) {
			return BT_ATT_ERR_INVALID_ATTRIBUTE_LEN;
		}
		/* An answer outstanding from before could otherwise be taken for this session's. */
		if ((stage != S_IDLE && stage != S_ACTIVE) || (relay_conn && relay_conn != c)) {
			return BT_ATT_ERR_AUTHORIZATION;
		}
		ring_flush();
		if (!relay_conn) {
			k_spinlock_key_t key = k_spin_lock(&ring_lock);

			relay_conn = bt_conn_ref(c);
			k_spin_unlock(&ring_lock, key);
		}
		token = t;
		sequence = 0;
		emit_status(STATUS_RESETTING);
		find_service();
		return 0;
	case CMD_STOP:
		if (len != CMD_HEAD) {
			return BT_ATT_ERR_INVALID_ATTRIBUTE_LEN;
		}
		if (c != relay_conn || t != token) {
			return BT_ATT_ERR_AUTHORIZATION;
		}
		ring_flush();
		if (stage == S_ACTIVE) {
			close_relay();
		} else if (stage != S_CLOSE_NS && stage != S_CLOSE_DS) {
			stop_pending = true;
		}
		return 0;
	default:
		if (c != relay_conn || t != token) {
			return BT_ATT_ERR_AUTHORIZATION;
		}
		if (stage != S_ACTIVE || len <= CMD_HEAD || len > MAX_CMD || !cp_valid(&v[CMD_HEAD], len - CMD_HEAD)) {
			return BT_ATT_ERR_INVALID_ATTRIBUTE_LEN;
		}
		if (cp_busy) {
			return BT_ATT_ERR_INSUFFICIENT_RESOURCES;
		}
		memcpy(cp_buf, &v[CMD_HEAD], len - CMD_HEAD);
		cp_write = (struct bt_gatt_write_params){
			.func = cp_done, .handle = cp_value, .data = cp_buf, .length = len - CMD_HEAD,
		};
		if (bt_gatt_write(c, &cp_write)) {
			return BT_ATT_ERR_INSUFFICIENT_RESOURCES;
		}
		cp_busy = true;
		return 0;
	}
}

/* The link is gone: Zephyr has failed every request and dropped the subscriptions by now. */
void ancs_link_down(void)
{
	k_spinlock_key_t key = k_spin_lock(&ring_lock);
	struct bt_conn *c = relay_conn;

	relay_conn = NULL;
	memset(ring, 0, sizeof(ring));
	ring_head = ring_used = 0;
	k_spin_unlock(&ring_lock, key);
	if (c) {
		bt_conn_unref(c);
	}
	memset(cp_buf, 0, sizeof(cp_buf));
	stage = S_IDLE;
	token = 0;
	stop_pending = ns_on = ds_on = subs_added = cp_busy = false;
}
