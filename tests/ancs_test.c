/* Tests for the ANCS relay's state machine (g1_firmware/firmware/src/ancs.c, VM.md §6.12), compiled with its Zephyr calls
 * replaced by a scripted phone: every stage against errors, STOP, a new START and the link's end, what reaches the ring,
 * and the connection references. The emulator test `ancs` (emulator/tests/v5_tests.py) runs the same code on Zephyr's GATT
 * client.
 *
 *   cc -std=c11 -O1 -g -fsanitize=address,undefined -fno-sanitize-recover=all -Itests/ancs_stubs -Ig1_firmware/firmware/src \
 *       tests/ancs_test.c -o ancs_test && ./ancs_test
 */
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include "../g1_firmware/firmware/src/ancs.c"

#define MAX_FRAMES 2048

static int failures, checks;
#define CHECK(c) do { checks++; if (!(c)) { failures++; printf("%s:%d: %s failed\n", __func__, __LINE__, #c); } } while (0)

/* The phone's GATT database. */
enum { H_SVC = 0x30, H_NS_DECL, H_NS, H_NS_CCC, H_CP_DECL, H_CP, H_DS_DECL, H_DS, H_DS_CCC, H_END = H_DS_CCC };

/* The scripted link. */
static struct bt_conn link_a = {0}, link_b = {0};
static struct bt_conn *live;
static uint16_t mtu = 247;
static bool no_service, no_ds_char, no_ccc;
static int discover_err, write_err, resub_err;
static struct bt_gatt_discover_params *pending_discover;
static struct bt_gatt_write_params *pending_ccc, *pending_cp;
static uint8_t ccc_state[0x100];
static uint32_t write_log[64];
static int write_count, resubs;
static struct bt_gatt_subscribe_params *listed[2];
static int listed_n;
static uint8_t cp_seen[MAX_CMD];
static uint16_t cp_seen_len;
static const bt_addr_le_t peer;

/* Zephyr's side. */
struct bt_conn *bt_conn_ref(struct bt_conn *c) { c->refs++; return c; }
void bt_conn_unref(struct bt_conn *c) { CHECK(c->refs > 0); c->refs--; }
int bt_conn_get_info(const struct bt_conn *c, struct bt_conn_info *info) { (void)c; info->id = 0; info->le.dst = &peer; return 0; }
uint16_t bt_gatt_get_mtu(struct bt_conn *c) { return c == live ? mtu : 0; }

int bt_gatt_discover(struct bt_conn *c, struct bt_gatt_discover_params *p)
{
	if (c != live) {
		return -ENOTCONN;
	}
	if (discover_err) {
		return discover_err;
	}
	CHECK(!pending_discover);   /* one request at a time */
	CHECK(p->start_handle && p->start_handle <= p->end_handle);
	pending_discover = p;
	return 0;
}

int bt_gatt_write(struct bt_conn *c, struct bt_gatt_write_params *p)
{
	if (c != live) {
		return -ENOTCONN;
	}
	if (write_err) {
		return write_err;
	}
	if (p->handle == H_CP) {
		CHECK(!pending_cp);
		pending_cp = p;
		CHECK(p->length <= sizeof(cp_seen));
		memcpy(cp_seen, p->data, p->length);
		cp_seen_len = p->length;
		return 0;
	}
	CHECK(p->handle == H_NS_CCC || p->handle == H_DS_CCC);   /* never any other handle of the phone */
	CHECK(p->length == 2);
	CHECK(!pending_ccc);
	pending_ccc = p;
	ccc_state[p->handle] = ((const uint8_t *)p->data)[0];
	CHECK(write_count < 64);
	write_log[write_count++] = p->handle << 8 | ((const uint8_t *)p->data)[0];
	return 0;
}

int bt_gatt_resubscribe(uint8_t id, const bt_addr_le_t *addr, struct bt_gatt_subscribe_params *p)
{
	(void)id; (void)addr;
	if (resub_err) {
		return resub_err;
	}
	for (int i = 0; i < listed_n; i++) {
		if (listed[i] == p) {
			return -EALREADY;
		}
	}
	CHECK(listed_n < 2);
	CHECK(p->notify && p->ccc_handle && p->value_handle && p->flags[0]);
	listed[listed_n++] = p;
	resubs++;
	return 0;
}

/* proto.c's side. */
void proto_ancs_kick(void) {}
static uint8_t safe;
uint8_t safe_level(void) { return safe; }

/* The phone's answers. */
static bool answer_ccc(uint8_t err)
{
	struct bt_gatt_write_params *p = pending_ccc;

	if (!p) {
		return false;
	}
	pending_ccc = NULL;
	p->func(live, err, p);
	return true;
}

static void answer_cp(uint8_t err)
{
	struct bt_gatt_write_params *p = pending_cp;

	CHECK(p);
	pending_cp = NULL;
	p->func(live, err, p);
}

/* One round of the current discovery: what the database holds in its range, then the end. */
static void answer_discover(void)
{
	struct bt_gatt_discover_params *p = pending_discover;
	static struct bt_uuid_16 ccc16 = BT_UUID_INIT_16(BT_UUID_GATT_CCC_VAL);

	CHECK(p);
	pending_discover = NULL;
	switch (p->type) {
	case BT_GATT_DISCOVER_PRIMARY: {
		struct bt_gatt_service_val v = {&ancs_uuid.uuid, H_END};
		struct bt_gatt_attr a = {&ancs_uuid.uuid, &v, H_SVC};

		CHECK(!bt_uuid_cmp(p->uuid, &ancs_uuid.uuid));
		if (no_service) {
			p->func(live, NULL, p);
		} else if (p->func(live, &a, p) != BT_GATT_ITER_STOP) {
			CHECK(0);
		}
		break;
	}
	case BT_GATT_DISCOVER_CHARACTERISTIC: {
		struct bt_gatt_chrc c[3] = {{&ns_uuid.uuid, H_NS}, {&cp_uuid.uuid, H_CP}, {&ds_uuid.uuid, H_DS}};

		for (int i = 0; i < (no_ds_char ? 2 : 3); i++) {
			struct bt_gatt_attr a = {NULL, &c[i], (uint16_t)(c[i].value_handle - 1)};

			if (p->func(live, &a, p) != BT_GATT_ITER_CONTINUE) {
				return;
			}
		}
		p->func(live, NULL, p);
		break;
	}
	default: {
		uint16_t h = p->start_handle <= H_NS_CCC ? H_NS_CCC : H_DS_CCC;
		struct bt_gatt_attr a = {&ccc16.uuid, NULL, h};

		CHECK(p->uuid && !bt_uuid_cmp(p->uuid, &ccc16.uuid));
		p->func(live, no_ccc ? NULL : &a, p);
		break;
	}
	}
}

static void notify_from_phone(struct bt_gatt_subscribe_params *p, const uint8_t *d, uint16_t n)
{
	for (int i = 0; i < listed_n; i++) {
		if (listed[i] == p && p->notify(live, p, d, n) != BT_GATT_ITER_CONTINUE) {
			CHECK(0);   /* a stop would make Zephyr unsubscribe */
		}
	}
}

static void link_up(struct bt_conn *c)
{
	live = c;
	mtu = 247;
	no_service = no_ds_char = no_ccc = false;
	discover_err = write_err = resub_err = 0;
	write_count = 0;
	listed_n = 0;
	memset(ccc_state, 0, sizeof(ccc_state));
}

/* As Zephyr ends a link: the outstanding requests fail, the subscriptions go, then the connection's callback. */
static void link_down(void)
{
	struct bt_conn *c = live;

	live = NULL;
	if (pending_cp) {
		struct bt_gatt_write_params *p = pending_cp;

		pending_cp = NULL;
		p->func(c, 0x0e, p);
	}
	if (pending_ccc) {
		struct bt_gatt_write_params *p = pending_ccc;

		pending_ccc = NULL;
		p->func(c, 0x0e, p);
	}
	if (pending_discover) {
		struct bt_gatt_discover_params *p = pending_discover;

		pending_discover = NULL;
		p->func(c, NULL, p);
	}
	for (int i = 0; i < listed_n; i++) {
		listed[i]->notify(c, listed[i], NULL, 0);
	}
	listed_n = 0;
	ancs_link_down();
}

/* The host's side: commands, and frames as the evt thread would send them. */
struct frame {
	uint8_t kind, flags;
	uint32_t token;
	uint16_t seq, n;
	uint8_t body[MAX_PKT];
};
static struct frame frames[MAX_FRAMES];
static int frame_n;

static void drain(void)
{
	uint8_t d[MAX_PKT], len;
	struct bt_conn *c;

	frame_n = 0;
	while ((c = ancs_take(d, &len)) != NULL) {
		struct frame *f = &frames[frame_n++];

		CHECK(frame_n < MAX_FRAMES);
		CHECK(c == live || !live);
		CHECK(len >= FRAME_HEAD && len <= MIN(mtu - 3, MAX_PKT));
		CHECK(d[0] == 'A' && d[1] == 'N' && d[2] == 1);
		f->kind = d[3];
		f->token = get32(&d[4]);
		f->seq = get16(&d[8]);
		f->flags = d[10];
		f->n = len - FRAME_HEAD;
		memcpy(f->body, &d[FRAME_HEAD], f->n);
		bt_conn_unref(c);
	}
}

static uint8_t command(uint8_t op, uint32_t t, const uint8_t *body, uint16_t n)
{
	uint8_t v[256] = {'A', 'N', 1, op};

	put32(&v[4], t);
	if (n) {
		memcpy(&v[8], body, n);
	}
	return ancs_command(live, v, 8 + n);
}

static uint8_t start(uint32_t t) { return command(CMD_START, t, NULL, 0); }
static uint8_t stop(uint32_t t) { return command(CMD_STOP, t, NULL, 0); }

/* From START to the relay ready, stopping once the stage is reached (S_ACTIVE: all the way). */
static void run_to(enum stage target)
{
	while (stage != target) {
		if (pending_discover) {
			answer_discover();
		} else if (!answer_ccc(0)) {
			CHECK(0);
			return;
		}
	}
}

static bool is_idle(void) { return stage == S_IDLE && !token && !pending_ccc && !pending_discover; }

static void settle(void)
{
	while (pending_discover || pending_ccc) {
		if (pending_discover) {
			answer_discover();
		} else {
			answer_ccc(0);
		}
	}
}

static bool ring_is_clear(void)
{
	for (size_t i = 0; i < sizeof(ring); i++) {
		if (ring[i]) {
			return false;
		}
	}
	return ring_used == 0;
}

static const uint8_t ns_added[8] = {0, 0, 1, 1, 7, 0, 0, 0};

static void test_normal_session(void)
{
	link_up(&link_a);
	CHECK(start(42) == 0);
	CHECK(stage == S_FIND_SERVICE);
	run_to(S_ACTIVE);
	/* DS on, NS off and on again (iOS lists what it has at a new subscription), and no other write */
	CHECK(write_count == 3);
	CHECK(write_log[0] == (H_DS_CCC << 8 | 1));
	CHECK(write_log[1] == (H_NS_CCC << 8 | 0));
	CHECK(write_log[2] == (H_NS_CCC << 8 | 1));
	CHECK(resubs >= 2 && listed_n == 2);
	CHECK(ns_sub.value_handle == H_NS && ds_sub.value_handle == H_DS);
	drain();
	CHECK(frame_n == 2);
	CHECK(frames[0].kind == KIND_STATUS && frames[0].body[0] == STATUS_RESETTING && frames[0].seq == 0);
	CHECK(frames[1].kind == KIND_STATUS && frames[1].body[0] == STATUS_READY && frames[1].seq == 1);
	CHECK(frames[0].token == 42 && frames[0].flags == 3);

	notify_from_phone(&ns_sub, ns_added, 8);
	drain();
	CHECK(frame_n == 1 && frames[0].kind == KIND_NS && frames[0].seq == 2 && frames[0].n == 8);
	CHECK(!memcmp(frames[0].body, ns_added, 8));

	/* a CP command, its answer, and its own frame */
	const uint8_t cp[] = {0, 7, 0, 0, 0, 0, 1, 128, 0};

	CHECK(command(CMD_CP, 42, cp, sizeof(cp)) == 0);
	CHECK(cp_seen_len == sizeof(cp) && !memcmp(cp_seen, cp, sizeof(cp)));
	CHECK(command(CMD_CP, 42, cp, sizeof(cp)) == BT_ATT_ERR_INSUFFICIENT_RESOURCES);   /* one at a time */
	answer_cp(0);
	drain();
	CHECK(frame_n == 1 && frames[0].kind == KIND_CP_DONE && frames[0].n == 0 && frames[0].flags == 3);
	/* ANCS's own error (0xA3: unknown notification) goes to the host as a failure */
	CHECK(command(CMD_CP, 42, cp, sizeof(cp)) == 0);
	answer_cp(0xa3);
	drain();
	CHECK(frame_n == 1 && frames[0].kind == KIND_STATUS && frames[0].body[0] == STATUS_FAILED);
	CHECK(stage == S_ACTIVE);

	CHECK(stop(42) == 0);
	CHECK(stage == S_CLOSE_NS);
	CHECK(answer_ccc(0));
	CHECK(stage == S_CLOSE_DS);
	CHECK(answer_ccc(0));
	CHECK(is_idle());
	CHECK(!ccc_state[H_NS_CCC] && !ccc_state[H_DS_CCC]);
	drain();
	CHECK(frame_n == 0);
	/* the subscriptions stay listed but pass nothing on */
	notify_from_phone(&ns_sub, ns_added, 8);
	notify_from_phone(&ds_sub, ns_added, 8);
	drain();
	CHECK(frame_n == 0);
	link_down();
	CHECK(link_a.refs == 0);
}

static void test_fragments(uint16_t the_mtu)
{
	uint8_t value[500];
	size_t got = 0;
	uint8_t back[sizeof(value)];

	link_up(&link_a);
	mtu = the_mtu;
	CHECK(start(9) == 0);
	run_to(S_ACTIVE);
	drain();
	for (size_t i = 0; i < sizeof(value); i++) {
		value[i] = i * 7 + 1;
	}
	notify_from_phone(&ds_sub, value, sizeof(value));
	drain();
	for (int i = 0; i < frame_n; i++) {
		struct frame *f = &frames[i];

		CHECK(f->kind == KIND_DS);
		CHECK(f->seq == i + 2);
		CHECK(!!(f->flags & FLAG_FIRST) == (i == 0));
		CHECK(!!(f->flags & FLAG_LAST) == (i == frame_n - 1));
		CHECK(f->n + FRAME_HEAD <= the_mtu - 3 && f->n + FRAME_HEAD <= MAX_PKT);
		memcpy(&back[got], f->body, f->n);
		got += f->n;
	}
	CHECK(got == sizeof(value) && !memcmp(back, value, got));
	/* too long for ATT, or nothing at all: not passed on */
	notify_from_phone(&ds_sub, value, 0);
	drain();
	CHECK(frame_n == 0);
	/* a Notification Source value that isn't one */
	notify_from_phone(&ns_sub, value, 7);
	drain();
	CHECK(frame_n == 0);
	link_down();
	CHECK(link_a.refs == 0);
}

/* A full ring: frames dropped still take their numbers, so the host sees a gap and starts over. */
static void test_overflow(void)
{
	int total = 0, missing = 0;
	uint16_t expect = 2;

	link_up(&link_a);
	CHECK(start(5) == 0);
	run_to(S_ACTIVE);
	drain();
	while (ring_drops == 0 || total < 2000) {
		notify_from_phone(&ns_sub, ns_added, 8);
		total++;
		if (total > 5000) {
			CHECK(0);
			break;
		}
		if (ring_drops > 40) {
			break;
		}
	}
	drain();
	CHECK(frame_n > 300 && frame_n < total);
	for (int i = 0; i < frame_n; i++) {
		if (frames[i].seq != expect) {
			missing++;
		}
		expect = frames[i].seq + 1;
	}
	CHECK(missing == 0);   /* nothing lost before the ring filled */
	notify_from_phone(&ns_sub, ns_added, 8);
	drain();
	CHECK(frame_n == 1 && frames[0].seq != expect);   /* the gap */
	CHECK(frames[0].seq == (uint16_t)(2 + total));
	link_down();
	CHECK(link_a.refs == 0);
	ring_drops = 0;
}

static void test_commands_refused(void)
{
	static const uint8_t cp[] = {0, 7, 0, 0, 0, 0, 1, 128, 0};
	uint8_t big[100] = {0};

	link_up(&link_a);
	/* nothing to stop, nothing to forward: no owner yet */
	CHECK(stop(42) == BT_ATT_ERR_AUTHORIZATION);
	CHECK(command(CMD_CP, 42, cp, sizeof(cp)) == BT_ATT_ERR_AUTHORIZATION);
	CHECK(link_a.refs == 0);
	/* malformed */
	CHECK(start(0) == BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	CHECK(command(3, 42, NULL, 0) == BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	CHECK(command(CMD_START, 42, cp, 1) == BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	CHECK(ancs_command(live, (const uint8_t *)"AN\1", 3) == BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	CHECK(ancs_command(live, (const uint8_t *)"XN\1\0\1\0\0\0", 8) == BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	CHECK(is_idle());

	CHECK(start(42) == 0);
	/* CP before ready, and a second START while the first's requests are out */
	CHECK(command(CMD_CP, 42, cp, sizeof(cp)) == BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	CHECK(start(43) == BT_ATT_ERR_AUTHORIZATION);
	run_to(S_ACTIVE);
	drain();
	/* a stale token, a wrong connection */
	CHECK(stop(41) == BT_ATT_ERR_AUTHORIZATION);
	CHECK(command(CMD_CP, 41, cp, sizeof(cp)) == BT_ATT_ERR_AUTHORIZATION);
	CHECK(ancs_command(&link_b, (const uint8_t *)"AN\1\1\52\0\0\0", 8) == BT_ATT_ERR_AUTHORIZATION);
	CHECK(ancs_command(&link_b, (const uint8_t *)"AN\1\0\52\0\0\0", 8) == BT_ATT_ERR_AUTHORIZATION);
	CHECK(link_b.refs == 0);
	/* only ANCS's commands, bounded: no arbitrary write */
	cp_seen_len = 0;
	CHECK(command(CMD_CP, 42, NULL, 0) == BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	CHECK(command(CMD_CP, 42, (const uint8_t *)"\3\0\0\0\0\0", 6) == BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	CHECK(command(CMD_CP, 42, (const uint8_t *)"\0\0\0\0\0", 5) == BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	CHECK(command(CMD_CP, 42, (const uint8_t *)"\1a\1", 3) == BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	CHECK(command(CMD_CP, 42, (const uint8_t *)"\1abc\0\1", 6) == BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	CHECK(command(CMD_CP, 42, (const uint8_t *)"\2\7\0\0\0\2", 6) == BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	CHECK(command(CMD_CP, 42, (const uint8_t *)"\2\7\0\0\0", 5) == BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	CHECK(command(CMD_CP, 42, big, sizeof(big)) == BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	CHECK(cp_seen_len == 0 && !pending_cp);
	/* the largest allowed, and the app-attributes and action forms */
	big[0] = 1;
	memset(&big[1], 'a', 69);
	CHECK(command(CMD_CP, 42, big, 72) == 0);
	CHECK(cp_seen_len == 72);
	answer_cp(0);
	CHECK(command(CMD_CP, 42, (const uint8_t *)"\2\7\0\0\0\1", 6) == 0);
	answer_cp(0);
	link_down();
	CHECK(link_a.refs == 0);
}

static void start_and_reach(enum stage s)
{
	CHECK(start(77) == 0);
	run_to(s);
}

static const enum stage stages[] = {S_FIND_SERVICE, S_FIND_CHARS, S_FIND_NS_CCC, S_FIND_DS_CCC, S_SUB_DS, S_NS_OFF, S_SUB_NS};

/* STOP while a request is out takes effect at its answer; nothing is reported after it. */
static void test_stop_in_each_stage(void)
{
	for (size_t i = 0; i < sizeof(stages) / sizeof(stages[0]); i++) {
		link_up(&link_a);
		start_and_reach(stages[i]);
		drain();
		CHECK(stop(77) == 0);
		CHECK(start(78) == BT_ATT_ERR_AUTHORIZATION);
		if (i >= 6) {
			notify_from_phone(&ns_sub, ns_added, 8);   /* iOS can send before it answers */
		}
		settle();
		CHECK(is_idle());
		CHECK(!ccc_state[H_NS_CCC] && !ccc_state[H_DS_CCC]);
		drain();
		CHECK(frame_n == 0);
		CHECK(start(79) == 0);   /* and it can start again */
		run_to(S_ACTIVE);
		drain();
		CHECK(frame_n == 2 && frames[0].token == 79 && frames[0].seq == 0);
		link_down();
		CHECK(link_a.refs == 0);
		CHECK(is_idle());
	}
}

/* The link's end in each stage, with what Zephyr calls back on the way: the next link starts from nothing. */
static void test_link_down_in_each_stage(void)
{
	for (size_t i = 0; i < sizeof(stages) / sizeof(stages[0]); i++) {
		link_up(&link_a);
		start_and_reach(stages[i]);
		link_down();
		CHECK(link_a.refs == 0);
		CHECK(is_idle());
		CHECK(ring_is_clear());
		CHECK(!relay_conn);

		link_up(&link_b);
		/* nothing of the old session is accepted, or sent */
		CHECK(stop(77) == BT_ATT_ERR_AUTHORIZATION);
		CHECK(!ancs_take((uint8_t[MAX_PKT]){0}, (uint8_t[1]){0}));
		CHECK(start(90) == 0);
		run_to(S_ACTIVE);
		drain();
		CHECK(frame_n == 2 && frames[0].token == 90 && frames[0].seq == 0 && frames[1].seq == 1);
		link_down();
		CHECK(link_b.refs == 0);
	}
	/* with everything up and a command out */
	link_up(&link_a);
	start_and_reach(S_ACTIVE);
	CHECK(command(CMD_CP, 77, (const uint8_t *)"\2\7\0\0\0\1", 6) == 0);
	notify_from_phone(&ns_sub, ns_added, 8);   /* sits in the ring */
	link_down();
	CHECK(link_a.refs == 0 && ring_is_clear() && is_idle() && !cp_busy);
}

/* A frame on its way when the link ended goes nowhere new: the reference holds the old connection. */
static void test_frame_stays_with_its_link(void)
{
	uint8_t d[MAX_PKT], len;
	struct bt_conn *c;

	link_up(&link_a);
	start_and_reach(S_ACTIVE);
	notify_from_phone(&ns_sub, ns_added, 8);
	drain();
	notify_from_phone(&ns_sub, ns_added, 8);
	c = ancs_take(d, &len);
	CHECK(c == &link_a && link_a.refs == 2);
	link_down();
	CHECK(link_a.refs == 1);   /* the object can't be reused while the sender holds it */
	bt_conn_unref(c);
	CHECK(link_a.refs == 0);
}

static void test_failures(void)
{
	/* no ANCS on this phone, or not shared with this device */
	link_up(&link_a);
	no_service = true;
	CHECK(start(1) == 0);
	answer_discover();
	CHECK(is_idle());
	drain();
	CHECK(frame_n == 2 && frames[1].kind == KIND_STATUS && frames[1].body[0] == STATUS_NO_ANCS && frames[1].token == 1);
	CHECK(write_count == 0);
	/* shared later: the next START finds it */
	no_service = false;
	CHECK(start(2) == 0);
	run_to(S_ACTIVE);
	drain();
	CHECK(frame_n == 2 && frames[1].body[0] == STATUS_READY);
	link_down();

	/* a characteristic or CCC missing */
	link_up(&link_a);
	no_ds_char = true;
	CHECK(start(3) == 0);
	settle();
	CHECK(is_idle());
	drain();
	CHECK(frames[frame_n - 1].body[0] == STATUS_FAILED);
	link_down();
	link_up(&link_a);
	no_ccc = true;
	CHECK(start(3) == 0);
	settle();
	CHECK(is_idle());
	CHECK(write_count == 0);
	drain();
	CHECK(frames[frame_n - 1].body[0] == STATUS_FAILED);
	link_down();

	/* no room for a request at each step */
	link_up(&link_a);
	discover_err = -ENOMEM;
	CHECK(start(4) == 0);
	CHECK(is_idle());
	drain();
	CHECK(frame_n == 2 && frames[1].body[0] == STATUS_FAILED);
	link_down();

	/* a phone that refuses each CCC write: reported, what was set is cleared, a later START works */
	for (int refuse = 0; refuse < 3; refuse++) {
		link_up(&link_a);
		CHECK(start(6) == 0);
		run_to(S_SUB_DS);
		for (int i = 0; i < refuse; i++) {
			CHECK(answer_ccc(0));
		}
		CHECK(answer_ccc(0x05));
		settle();
		CHECK(is_idle());
		drain();
		CHECK(frames[frame_n - 1].kind == KIND_STATUS && frames[frame_n - 1].body[0] == STATUS_FAILED);
		CHECK(!ccc_state[H_DS_CCC] && !ccc_state[H_NS_CCC]);
		CHECK(start(7) == 0);
		run_to(S_ACTIVE);
		link_down();
		CHECK(link_a.refs == 0);
	}

	/* a phone that refuses CCC = 0 at the end (ANCS withdrawn): still idle, and the relay isn't stuck */
	link_up(&link_a);
	start_and_reach(S_ACTIVE);
	CHECK(stop(77) == 0);
	CHECK(answer_ccc(0x0a));
	CHECK(answer_ccc(0x0a));
	CHECK(is_idle());
	CHECK(start(8) == 0);
	run_to(S_ACTIVE);
	link_down();

	/* no room for the CCC = 0 writes: idle at once */
	link_up(&link_a);
	start_and_reach(S_ACTIVE);
	write_err = -ENOMEM;
	CHECK(stop(77) == 0);
	CHECK(is_idle());
	write_err = 0;
	link_down();
	CHECK(link_a.refs == 0);

	/* no room for the subscription */
	link_up(&link_a);
	resub_err = -ENOMEM;
	CHECK(start(9) == 0);
	settle();
	CHECK(is_idle());
	CHECK(write_count == 0);
	link_down();
}

/* The app's recovery: a new START on a live relay, nothing in between. */
static void test_restart(void)
{
	link_up(&link_a);
	start_and_reach(S_ACTIVE);
	CHECK(command(CMD_CP, 77, (const uint8_t *)"\2\7\0\0\0\1", 6) == 0);
	notify_from_phone(&ns_sub, ns_added, 8);   /* never reaches the new session */
	write_count = 0;
	resubs = 0;
	CHECK(start(78) == 0);
	CHECK(stage == S_FIND_SERVICE);
	/* the old session's answer comes first, and is no one's now */
	answer_cp(0);
	notify_from_phone(&ns_sub, ns_added, 8);   /* before the new subscription: dropped */
	run_to(S_ACTIVE);
	CHECK(resubs == 0 && listed_n == 2);       /* still listed */
	CHECK(write_count == 3 && write_log[1] == (H_NS_CCC << 8 | 0) && write_log[2] == (H_NS_CCC << 8 | 1));
	drain();
	CHECK(frame_n == 2);
	CHECK(frames[0].token == 78 && frames[0].seq == 0 && frames[0].body[0] == STATUS_RESETTING);
	CHECK(frames[1].token == 78 && frames[1].seq == 1 && frames[1].body[0] == STATUS_READY);
	/* iOS's list, while the host waits for ready */
	CHECK(start(80) == 0);
	run_to(S_SUB_NS);
	notify_from_phone(&ns_sub, ns_added, 8);
	notify_from_phone(&ds_sub, ns_added, 8);   /* no attributes asked for yet: dropped */
	CHECK(answer_ccc(0));
	drain();
	CHECK(frame_n == 3 && frames[1].kind == KIND_NS && frames[2].body[0] == STATUS_READY);
	link_down();
	CHECK(link_a.refs == 0);
}

int main(void)
{
	test_normal_session();
	test_fragments(247);
	test_fragments(185);
	test_fragments(23);
	test_overflow();
	test_commands_refused();
	test_stop_in_each_stage();
	test_link_down_in_each_stage();
	test_frame_stays_with_its_link();
	test_failures();
	test_restart();
	printf("%d checks, %d failed\n", checks, failures);
	return failures ? 1 : 0;
}
