/* Tests of the decoder of Faceclaw's records against zlib, which is what the phone runs, under the sanitizers:
 *   cc -std=c11 -O1 -g -fsanitize=address,undefined -fno-sanitize-recover=all -Ig1_firmware/firmware/src \
 *      tests/fc_inflate_test.c g1_firmware/firmware/src/fc_inflate.c -lz -o fc_inflate_test && ./fc_inflate_test
 * Streams of sync-flushed records, of every kind of content and level, across the history's wrap; then the same streams
 * damaged, where any result but a memory fault or a wrong "ok" is right. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zlib.h>
#include "fc_inflate.h"

#define MAX_RECORD 16384u
#define CAP (FC_Z_HISTORY + MAX_RECORD)

static uint32_t seed = 0x1234567;   /* xorshift32: no zero state */
static uint32_t rnd(void)
{
	seed ^= seed << 13;
	seed ^= seed >> 17;
	seed ^= seed << 5;
	return seed;
}

/* RLE-like, repetitive, noisy or empty content: what the phone's messages look like, and what they do not */
static uint32_t content(uint8_t *d, uint32_t max, const uint8_t *old, uint32_t old_n)
{
	uint32_t kind = rnd() % 6, n = rnd() % (max + 1);

	switch (kind) {
	case 0:
		n %= 64;
		for (uint32_t i = 0; i < n; i++) {
			d[i] = (uint8_t)rnd();
		}
		break;
	case 1:
		for (uint32_t i = 0; i < n; i++) {
			d[i] = (uint8_t)rnd();
		}
		break;
	case 2:
		memset(d, (int)(rnd() & 0xFF), n);
		break;
	case 3:
		for (uint32_t i = 0; i < n; i++) {
			d[i] = (uint8_t)(i / 7);
		}
		break;
	case 4:   /* reaches back into the history */
		for (uint32_t i = 0; i < n; i++) {
			d[i] = old_n ? old[(i * 3 + 11) % old_n] : (uint8_t)i;
		}
		break;
	default:
		for (uint32_t i = 0; i < n; i++) {
			d[i] = (rnd() % 8) ? 0x10 : (uint8_t)(rnd() & 0x1F);
		}
		break;
	}
	if (n == 0) {
		/* zlib refuses a repeated empty flush (Z_BUF_ERROR); the phone sends an empty record raw */
		d[0] = 0;
		n = 1;
	}
	return n;
}

/* a zlib stream as the phone keeps one; returns the record's length */
static uint32_t flush_record(z_stream *s, const uint8_t *d, uint32_t n, uint8_t *out, uint32_t cap)
{
	s->next_in = (Bytef *)d;
	s->avail_in = n;
	s->next_out = out;
	s->avail_out = cap;
	if (deflate(s, Z_SYNC_FLUSH) != Z_OK || s->avail_in || !s->avail_out) {
		fprintf(stderr, "deflate failed\n");
		exit(2);
	}
	return cap - s->avail_out;
}

static void init_stream(z_stream *s, int level, int strategy)
{
	memset(s, 0, sizeof(*s));
	if (deflateInit2(s, level, Z_DEFLATED, 15, 8, strategy) != Z_OK) {
		exit(2);
	}
}

static int clean_streams(void)
{
	static uint8_t buf[CAP], plain[MAX_RECORD], packed[2 * MAX_RECORD + 256];
	static const int levels[] = {0, 1, 6, 9};
	static const int strategies[] = {Z_DEFAULT_STRATEGY, Z_FIXED, Z_HUFFMAN_ONLY, Z_RLE, Z_FILTERED};
	struct fc_inflate z;
	unsigned long records = 0, bytes = 0;

	fc_inflate_init(&z, buf, CAP);
	for (int round = 0; round < 200; round++) {
		z_stream s;
		uint32_t n = 0;

		init_stream(&s, levels[round % 4], strategies[round % 5]);
		fc_inflate_reset(&z);
		for (int i = 0; i < 40; i++) {
			const uint8_t *out;
			uint32_t out_n, w;
			int err;
			static uint8_t last[MAX_RECORD];

			memcpy(last, plain, n);
			n = content(plain, MAX_RECORD, last, n);
			w = flush_record(&s, plain, n, packed, sizeof(packed));
			err = fc_inflate_record(&z, packed, w, &out, &out_n);
			if (err || out_n != n || memcmp(out, plain, n)) {
				fprintf(stderr, "round %d record %d: err %d, %u bytes of %u\n", round, i, err, out_n, n);
				return 1;
			}
			fc_inflate_commit(&z);
			records++;
			bytes += n;
		}
		deflateEnd(&s);
	}
	printf("clean: %lu records, %lu bytes\n", records, bytes);
	return 0;
}

/* The history's limits, exactly: a match of distance 32768 right after a full history, and output to the last byte. */
static int limits(void)
{
	static uint8_t buf[CAP], plain[MAX_RECORD], packed[2 * MAX_RECORD + 256];
	struct fc_inflate z;
	z_stream s;
	const uint8_t *out;
	uint32_t out_n, w;
	int err;

	fc_inflate_init(&z, buf, CAP);
	init_stream(&s, 9, Z_DEFAULT_STRATEGY);
	for (int i = 0; i < 4; i++) {
		for (uint32_t k = 0; k < MAX_RECORD; k++) {
			plain[k] = (uint8_t)rnd();
		}
		w = flush_record(&s, plain, MAX_RECORD, packed, sizeof(packed));
		err = fc_inflate_record(&z, packed, w, &out, &out_n);
		if (err || out_n != MAX_RECORD || memcmp(out, plain, MAX_RECORD)) {
			fprintf(stderr, "full record %d: err %d\n", i, err);
			return 1;
		}
		fc_inflate_commit(&z);
	}
	if (z.hist != FC_Z_HISTORY) {
		fprintf(stderr, "history %u\n", z.hist);
		return 1;
	}
	/* one byte over the limit is refused */
	{
		static uint8_t big[MAX_RECORD + 1];

		for (uint32_t k = 0; k <= MAX_RECORD; k++) {
			big[k] = (uint8_t)rnd();
		}
		w = flush_record(&s, big, MAX_RECORD + 1, packed, sizeof(packed));
		err = fc_inflate_record(&z, packed, w, &out, &out_n);
		if (err != FC_Z_OUTPUT) {
			fprintf(stderr, "oversized record: err %d\n", err);
			return 1;
		}
	}
	deflateEnd(&s);
	/* handmade: a stored block's complement wrong; a final block; a reserved type; a match before the start */
	{
		static const struct {
			uint8_t d[12];
			uint32_t n;
			int want;
		} cases[] = {
			{{0x78, 0x9C, 0x00, 0x01, 0x00, 0xFE, 0xFF, 'a'}, 8, FC_Z_TRUNCATED},       /* no sync flush after it */
			{{0x78, 0x9C, 0x00, 0x01, 0x00, 0x00, 0x00, 'a'}, 8, FC_Z_BLOCK},           /* NLEN wrong */
			{{0x78, 0x9C, 0x01, 0x00, 0x00, 0xFF, 0xFF}, 7, FC_Z_BLOCK},                /* final */
			{{0x78, 0x9C, 0x06}, 3, FC_Z_BLOCK},                                        /* type 3 */
			{{0x79, 0x9C, 0x00, 0x00, 0x00, 0xFF, 0xFF}, 7, FC_Z_HEADER},               /* FCHECK */
			{{0x88, 0x1C, 0x00, 0x00, 0x00, 0xFF, 0xFF}, 7, FC_Z_HEADER},               /* a 64 KiB window */
			{{0x78, 0x20 | 0x1F, 0x00, 0x00, 0x00, 0xFF, 0xFF}, 7, FC_Z_HEADER},        /* FDICT */
			{{0x78, 0x9C, 0x00, 0x00, 0x00, 0xFF, 0xFF}, 7, FC_Z_OK},                   /* only the flush */
			{{0x78, 0x9C, 0x00, 0x00, 0x00, 0xFF}, 6, FC_Z_TRUNCATED},
			{{0x78}, 1, FC_Z_TRUNCATED},
			{{0x78, 0x9C}, 2, FC_Z_TRUNCATED},
			/* fixed block: literal-free match of length 3, distance 1, with nothing before it */
			{{0x78, 0x9C, 0x02, 0x02, 0x00, 0x00, 0x00, 0xFF, 0xFF}, 9, FC_Z_DISTANCE},
		};

		for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
			fc_inflate_reset(&z);
			err = fc_inflate_record(&z, cases[i].d, cases[i].n, &out, &out_n);
			if (err != cases[i].want) {
				fprintf(stderr, "handmade %zu: err %d, want %d\n", i, err, cases[i].want);
				return 1;
			}
		}
	}
	printf("limits: ok\n");
	return 0;
}

/* Damage: flipped bits, cut records, random bytes. The sanitizers catch a bad access; an "ok" must still stay in bounds,
 * and is then checked by the record's CRC in the firmware, which a test here cannot replace. */
static int damaged(void)
{
	static uint8_t buf[CAP + 64], plain[MAX_RECORD], packed[2 * MAX_RECORD + 256], bad[2 * MAX_RECORD + 256];
	struct fc_inflate z;
	unsigned long runs = 0, oks = 0, errs[8] = {0};

	for (int i = 0; i < 64; i++) {
		buf[CAP + i] = 0xA5;
	}
	fc_inflate_init(&z, buf, CAP);
	for (int round = 0; round < 600; round++) {
		z_stream s;
		uint32_t n = 0;

		init_stream(&s, round % 2 ? 6 : 1, round % 3 ? Z_DEFAULT_STRATEGY : Z_FIXED);
		fc_inflate_reset(&z);
		for (int i = 0; i < 12; i++) {
			const uint8_t *out;
			uint32_t out_n, w, m;
			int err;

			n = content(plain, MAX_RECORD, plain, n);
			w = flush_record(&s, plain, n, packed, sizeof(packed));
			m = w;
			memcpy(bad, packed, w);
			switch (rnd() % 5) {
			case 0:
				bad[rnd() % w] ^= (uint8_t)(1u << (rnd() % 8));
				break;
			case 1:
				for (int k = 0; k < 8; k++) {
					bad[rnd() % w] = (uint8_t)rnd();
				}
				break;
			case 2:
				m = rnd() % (w + 1);
				break;
			case 3:
				m = rnd() % 512;
				for (uint32_t k = 0; k < m; k++) {
					bad[k] = (uint8_t)rnd();
				}
				break;
			default:
				break;
			}
			err = fc_inflate_record(&z, bad, m, &out, &out_n);
			runs++;
			if (err) {
				errs[-err]++;
				break;   /* the stream is lost, as in the firmware */
			}
			oks++;
			if (out_n > MAX_RECORD || out != buf + z.hist) {
				fprintf(stderr, "ok out of bounds: %u\n", out_n);
				return 1;
			}
			fc_inflate_commit(&z);
		}
		deflateEnd(&s);
	}
	for (int i = 0; i < 64; i++) {
		if (buf[CAP + i] != 0xA5) {
			fprintf(stderr, "wrote past the buffer\n");
			return 1;
		}
	}
	printf("damaged: %lu runs, %lu ok; truncated %lu header %lu block %lu table %lu symbol %lu distance %lu output %lu\n",
	       runs, oks, errs[1], errs[2], errs[3], errs[4], errs[5], errs[6], errs[7]);
	return 0;
}

int main(void)
{
	return clean_streams() || limits() || damaged();
}
