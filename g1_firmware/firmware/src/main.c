/* g1v0 boot order (firmware-plan.md §6): retained-RAM accounting, watchdog feeding, then BLE + SMP
 * first; PMIC and panel run in their own threads so they can never hold up DFU. */
#include <stdio.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/mgmt/mcumgr/mgmt/callbacks.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <nrf.h>
#include "g1.h"
#include "netfallback.h"

LOG_MODULE_REGISTER(g1v0, LOG_LEVEL_INF);

void retained_log_boot_banner(void);

static char name[16];

/* The arm keeps the Bluetooth address Even's firmware has (Zephyr's default: the net core's static address, or what
 * the settings hold), so a host's bond and what it remembers about the arm survive an install in either direction.
 * Called once the identity is known, after settings_load(). The name tells the pairs in a room apart: stock's names
 * end in the same six hex digits. */
static void set_name(void)
{
	bt_addr_le_t id = {0};
	size_t count = 1;

	bt_id_get(&id, &count);
	snprintf(name, sizeof(name), "G1CFW_%c_%02X%02X%02X", board_role() ? 'L' : 'R', id.a.val[2], id.a.val[1], id.a.val[0]);
}

static void adv_restart(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(adv_work, adv_restart);

/* Advertising (VM.md §6.6): 100-150 ms for 60 s after boot or a disconnection and whenever in the case
 * (IN_CASE), then 1 s (1000-1100 ms). The switch is a stop and a start with the same data. */
#define ADV_FAST_S 60
static const struct bt_le_adv_param adv_fast = BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_CONNECTABLE,
	BT_GAP_ADV_FAST_INT_MIN_2, BT_GAP_ADV_FAST_INT_MAX_2, NULL);
static const struct bt_le_adv_param adv_slow_param = BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_CONNECTABLE,
	0x0640, 0x06E0, NULL);
static volatile bool adv_on, adv_is_slow, adv_ready;
static int64_t fast_until;

/* The pair record (VM.md §6.1): manufacturer data without a company ID (0xFFFF), format 2, flags, the firmware's
 * version, then the frame serial, or the chip's device ID in hex on a board without one. Fixed for the whole boot,
 * so the advertising data is never updated while advertising. */
static size_t pair_record(uint8_t *b)
{
	size_t n = strlen(g1_settings.serial);

	b[0] = 0xFF;
	b[1] = 0xFF;
	b[2] = 2;
	b[3] = board_role() | (IS_ENABLED(CONFIG_G1_NO_PANEL) ? 2 : 0) | (n ? 4 : 0) | (safe_level() ? 8 : 0);
	proto_version(&b[4]);
	if (n) {
		memcpy(&b[7], g1_settings.serial, n);
	} else {
		n = snprintf((char *)&b[7], 17, "%08X%08X", NRF_FICR_S->INFO.DEVICEID[1], NRF_FICR_S->INFO.DEVICEID[0]);
	}
	return 7 + n;
}

static void adv_restart(struct k_work *work)
{
	uint8_t rec[7 + 17];
	const struct bt_data ad[] = {
		BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
		BT_DATA(BT_DATA_NAME_COMPLETE, name, strlen(name)),
	};
	const struct bt_data sd[] = {
		BT_DATA(BT_DATA_MANUFACTURER_DATA, rec, pair_record(rec)),
	};
	int err;

	bool slow = !g1_in_case() && k_uptime_get() >= fast_until;

	if (proto_connected()) {
		adv_on = false;
		return;
	}
	if (adv_on && slow == adv_is_slow) {
		return;
	}
	if (adv_on) {
		bt_le_adv_stop();
		adv_on = false;
	}
	err = bt_le_adv_start(slow ? &adv_slow_param : &adv_fast, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err && err != -EALREADY) {
		LOG_ERR("advertising failed: %d, retrying", err);
		k_work_schedule(&adv_work, K_SECONDS(1));
		return;
	}
	adv_on = true;
	adv_is_slow = slow;
	if (!slow && !g1_in_case()) {
		k_work_schedule(&adv_work, K_TIMEOUT_ABS_MS(fast_until));
	}
}

bool adv_slow(void)
{
	return adv_on && adv_is_slow;
}

/* after a disconnection: fast again for 60 s */
void adv_kick(void)
{
	adv_on = false;
	fast_until = k_uptime_get() + ADV_FAST_S * 1000;
	k_work_schedule(&adv_work, K_MSEC(200));
}

/* IN_CASE changed: in the case fast, out of it the 60 s rule */
void adv_vbus_changed(void)
{
	if (!adv_ready) {
		return;   /* before bt_enable: main starts advertising */
	}
	if (!g1_in_case() && fast_until < k_uptime_get() + 1000) {
		fast_until = k_uptime_get();
	}
	k_work_schedule(&adv_work, K_NO_WAIT);
}

static enum mgmt_cb_return smp_reset_hook(uint32_t event, enum mgmt_cb_return prev_status, int32_t *rc,
					  uint16_t *group, bool *abort_more, void *data, size_t data_size)
{
	retained_mark_clean(CLEAN_SMP, true, false);
	LOG_INF("SMP reset requested");
	g1_prepare_reboot();
	return MGMT_CB_OK;
}

static struct mgmt_callback smp_reset_cb = {
	.callback = smp_reset_hook,
	.event_id = MGMT_EVT_OP_OS_MGMT_RESET,
};

static enum mgmt_cb_return smp_rx_hook(uint32_t event, enum mgmt_cb_return prev_status, int32_t *rc,
				       uint16_t *group, bool *abort_more, void *data, size_t data_size)
{
	if (!proto_writes_allowed()) {
		*rc = MGMT_ERR_EACCESSDENIED;
		*abort_more = true;
		return MGMT_CB_ERROR_RC;
	}
	health_bt_traffic();
	return MGMT_CB_OK;
}

static struct mgmt_callback smp_rx_cb = {
	.callback = smp_rx_hook,
	.event_id = MGMT_EVT_OP_CMD_RECV,
};

int main(void)
{
	int err;
	bool ble_up;

	retained_boot();
	retained_log_boot_banner();
	health_start();
	LOG_INF("g1v0 %s booting: safe level %u, unhealthy boots %u, boot %u, RESETREAS 0x%08x",
		CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION, safe_level(), retained_unhealthy(), retained_boot_count(),
		retained_resetreas());
	mgmt_callback_register(&smp_reset_cb);
	mgmt_callback_register(&smp_rx_cb);
	board_init();
	if (safe_level() < 2) {
		pmic_start();
	}
	caselink_start();
	touch_start();
	proto_init();
	exec_start();
	diag_netoff_boot();
	if (netfallback_boot()) {
		g1_reboot(CLEAN_HOST, false, false);
	}
	/* The net core must give us HCI. If the handshake does not complete we must NOT call bt_enable:
	 * with a dead net core its first HCI command would time out and Zephyr's BT_ASSERT would reset
	 * the arm (the same 10 s timeout as the SMP-upload risk). A failed handshake instead goes to the
	 * net-core fallback (netfallback.c). */
	err = IS_ENABLED(CONFIG_G1_DK_NCS_NET) ? 0 :
	      g1_netcore_handshake(board_role() ? 2 : 1, g1_settings.esb_channel, g1_settings.esb_master,
				   g1_settings.esb_slave);
	ble_up = false;
	if (err) {
		LOG_ERR("net core handshake failed: %d; not enabling Bluetooth this boot", err);
	} else {
		for (int i = 0; i < 5; i++) {
			k_sleep(K_MSEC(i == 0 ? 100 : 800));
			err = bt_enable(NULL);
			if (!err || err == -EALREADY) {
				ble_up = true;
				break;
			}
			LOG_WRN("bt_enable: %d, retrying", err);
		}
	}
	if (ble_up) {
		if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
			settings_load();
		}
		proto_update_bondable();
		health_set_ble_up();
		netfallback_ble_up();
	}
	if (!ble_up) {
		LOG_ERR("no Bluetooth this boot (%u consecutive)", retained_net_fails());
		if (!retained_net_fallback_done()) {
			k_sleep(K_SECONDS(2));
			g1_reboot(CLEAN_HOST, false, false);
		}
		LOG_WRN("net fallback already used; staying up without BLE (panel/charging/health) for recovery");
		return 0;
	}
	set_name();
	bt_set_name(name);
	fast_until = k_uptime_get() + ADV_FAST_S * 1000;
	adv_ready = true;
	k_work_schedule(&adv_work, K_NO_WAIT);
	LOG_INF("advertising as %s, SMP ready", name);
	if (safe_level() == 1 && pmic_wait_ready(K_SECONDS(5))) {
		pmic_panel_rails(false);
		LOG_WRN("safe mode: panel rails off, drawing disabled");
	}
	return 0;
}
