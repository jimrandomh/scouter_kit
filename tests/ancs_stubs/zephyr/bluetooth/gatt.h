#ifndef ANCS_STUB_GATT_H_
#define ANCS_STUB_GATT_H_
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#define BT_ATT_ERR_AUTHORIZATION 0x08
#define BT_ATT_ERR_INVALID_ATTRIBUTE_LEN 0x0d
#define BT_ATT_ERR_INSUFFICIENT_RESOURCES 0x11
#define BT_ATT_FIRST_ATTRIBUTE_HANDLE 1
#define BT_ATT_LAST_ATTRIBUTE_HANDLE 0xffff
enum { BT_GATT_ITER_STOP = 0, BT_GATT_ITER_CONTINUE };
#define BT_GATT_CCC_NOTIFY 1
struct bt_gatt_attr { const struct bt_uuid *uuid; void *user_data; uint16_t handle; };
struct bt_gatt_chrc { const struct bt_uuid *uuid; uint16_t value_handle; };
struct bt_gatt_service_val { const struct bt_uuid *uuid; uint16_t end_handle; };
enum { BT_GATT_DISCOVER_PRIMARY, BT_GATT_DISCOVER_CHARACTERISTIC, BT_GATT_DISCOVER_DESCRIPTOR };
struct bt_gatt_discover_params;
typedef uint8_t (*bt_gatt_discover_func_t)(struct bt_conn *c, const struct bt_gatt_attr *attr,
					   struct bt_gatt_discover_params *p);
struct bt_gatt_discover_params {
	const struct bt_uuid *uuid;
	bt_gatt_discover_func_t func;
	uint16_t start_handle, end_handle;
	uint8_t type;
};
struct bt_gatt_write_params;
typedef void (*bt_gatt_write_func_t)(struct bt_conn *c, uint8_t err, struct bt_gatt_write_params *p);
struct bt_gatt_write_params {
	bt_gatt_write_func_t func;
	uint16_t handle, offset;
	const void *data;
	uint16_t length;
};
enum { BT_GATT_SUBSCRIBE_FLAG_VOLATILE };
struct bt_gatt_subscribe_params;
typedef uint8_t (*bt_gatt_notify_func_t)(struct bt_conn *c, struct bt_gatt_subscribe_params *p, const void *data,
					 uint16_t len);
struct bt_gatt_subscribe_params {
	bt_gatt_notify_func_t notify;
	uint16_t value_handle, ccc_handle, value;
	ATOMIC_DEFINE(flags, 4);
};
uint16_t bt_gatt_get_mtu(struct bt_conn *c);
int bt_gatt_discover(struct bt_conn *c, struct bt_gatt_discover_params *p);
int bt_gatt_write(struct bt_conn *c, struct bt_gatt_write_params *p);
int bt_gatt_resubscribe(uint8_t id, const bt_addr_le_t *peer, struct bt_gatt_subscribe_params *p);
#endif
