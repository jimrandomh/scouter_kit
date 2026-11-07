/* What ancs.c and g1.h use of Zephyr, for tests/ancs_test.c. Single threaded: locks do nothing. */
#ifndef ANCS_STUB_KERNEL_H_
#define ANCS_STUB_KERNEL_H_
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
typedef long atomic_t;
typedef struct { int ms; } k_timeout_t;
struct k_spinlock { int unused; };
typedef int k_spinlock_key_t;
static inline k_spinlock_key_t k_spin_lock(struct k_spinlock *l) { (void)l; return 0; }
static inline void k_spin_unlock(struct k_spinlock *l, k_spinlock_key_t k) { (void)l; (void)k; }
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define ATOMIC_DEFINE(name, n) atomic_t name[1]
static inline void atomic_set_bit(atomic_t *t, int bit) { *t |= 1L << bit; }
#endif
