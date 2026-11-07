#ifndef ANCS_STUB_UUID_H_
#define ANCS_STUB_UUID_H_
#include <stdint.h>
#include <string.h>
struct bt_uuid { uint8_t type; };
struct bt_uuid_16 { struct bt_uuid uuid; uint16_t val; };
struct bt_uuid_128 { struct bt_uuid uuid; uint8_t val[16]; };
#define BT_UUID_GATT_CCC_VAL 0x2902
#define BT_UUID_INIT_16(v) { .uuid = { 16 }, .val = (v) }
#define BT_UUID_INIT_128(...) { .uuid = { 128 }, .val = { __VA_ARGS__ } }
#define BT_UUID_128_ENCODE(w32, w1, w2, w3, w48) \
	(uint8_t)(w48), (uint8_t)((w48) >> 8), (uint8_t)((w48) >> 16), (uint8_t)((w48) >> 24), (uint8_t)((uint64_t)(w48) >> 32), \
	(uint8_t)((uint64_t)(w48) >> 40), (uint8_t)(w3), (uint8_t)((w3) >> 8), (uint8_t)(w2), (uint8_t)((w2) >> 8), \
	(uint8_t)(w1), (uint8_t)((w1) >> 8), (uint8_t)(w32), (uint8_t)((w32) >> 8), (uint8_t)((w32) >> 16), (uint8_t)((w32) >> 24)
static inline int bt_uuid_cmp(const struct bt_uuid *a, const struct bt_uuid *b)
{
	if (a->type != b->type) {
		return 1;
	}
	return a->type == 16 ? ((const struct bt_uuid_16 *)a)->val != ((const struct bt_uuid_16 *)b)->val
			     : memcmp(((const struct bt_uuid_128 *)a)->val, ((const struct bt_uuid_128 *)b)->val, 16);
}
#endif
