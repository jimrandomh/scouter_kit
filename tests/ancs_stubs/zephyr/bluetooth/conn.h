#ifndef ANCS_STUB_CONN_H_
#define ANCS_STUB_CONN_H_
#include <stdint.h>
typedef struct { uint8_t type; uint8_t a[6]; } bt_addr_le_t;
struct bt_conn { int refs; };
struct bt_conn_info {
	uint8_t id;
	struct { const bt_addr_le_t *dst; } le;
};
struct bt_conn *bt_conn_ref(struct bt_conn *c);
void bt_conn_unref(struct bt_conn *c);
int bt_conn_get_info(const struct bt_conn *c, struct bt_conn_info *info);
#endif
