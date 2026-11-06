/* See fc_inflate.h. RFC 1951 canonical Huffman decoding, one bit at a time: the input is at most what the link
 * carries (about 13 KB/s), so the few KB of tables a faster decoder needs are better left to the frame. Every read and
 * every write is checked here: the input comes off the air. */
#include <string.h>
#include "fc_inflate.h"

#define MAX_BITS 15
#define LIT_SYMS 288
#define DIST_SYMS 30
#define CL_SYMS 19

struct huff {
	uint16_t count[MAX_BITS + 1];   /* codes of each length */
	uint16_t symbol[LIT_SYMS];      /* symbols in canonical order */
};

/* One decoder runs at a time (the executor thread); the tables are rebuilt per block. */
static struct huff lit, dist;

struct reader {
	const uint8_t *in;
	uint32_t n, pos;
	uint32_t bitbuf;
	int bitcnt;
	bool err;
};

static inline uint32_t get_bits(struct reader *r, int need)
{
	uint32_t v = r->bitbuf;

	while (r->bitcnt < need) {
		if (r->pos == r->n) {
			r->err = true;
			return 0;
		}
		v |= (uint32_t)r->in[r->pos++] << r->bitcnt;
		r->bitcnt += 8;
	}
	r->bitbuf = v >> need;
	r->bitcnt -= need;
	return v & ((1u << need) - 1);
}

/* the next symbol, or -1 */
static inline int decode(struct reader *r, const struct huff *h)
{
	int code = 0, first = 0, index = 0;

	for (int len = 1; len <= MAX_BITS; len++) {
		int count = h->count[len];

		code |= (int)get_bits(r, 1);
		if (r->err) {
			return -1;
		}
		if (code - count < first) {
			return h->symbol[index + (code - first)];
		}
		index += count;
		first += count;
		first <<= 1;
		code <<= 1;
	}
	return -1;
}

/* From the code lengths of n symbols. Returns the codes left unused (0: complete), < 0 if over-subscribed. */
static int build(struct huff *h, const uint8_t *lengths, int n)
{
	uint16_t offs[MAX_BITS + 1];
	int left = 1;

	memset(h->count, 0, sizeof(h->count));
	for (int i = 0; i < n; i++) {
		h->count[lengths[i]]++;
	}
	if (h->count[0] == n) {
		return 0;
	}
	for (int len = 1; len <= MAX_BITS; len++) {
		left <<= 1;
		left -= h->count[len];
		if (left < 0) {
			return left;
		}
	}
	offs[1] = 0;
	for (int len = 1; len < MAX_BITS; len++) {
		offs[len + 1] = offs[len] + h->count[len];
	}
	for (int i = 0; i < n; i++) {
		if (lengths[i]) {
			h->symbol[offs[lengths[i]]++] = (uint16_t)i;
		}
	}
	return left;
}

static const uint16_t len_base[29] = {3,  4,  5,  6,  7,  8,  9,  10, 11,  13,  15,  17,  19,  23, 27,
				      31, 35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258};
static const uint8_t len_extra[29] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0};
static const uint16_t dist_base[30] = {1,   2,   3,   4,   5,   7,    9,    13,   17,   25,   33,   49,   65,    97,    129,
				       193, 257, 385, 513, 769, 1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577};
static const uint8_t dist_extra[30] = {0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13};

static int block_codes(struct reader *r, uint8_t *o, uint32_t cap, uint32_t *pos, uint32_t avail)
{
	uint32_t p = *pos;

	for (;;) {
		int sym = decode(r, &lit);

		if (sym < 0) {
			return r->err ? FC_Z_TRUNCATED : FC_Z_SYMBOL;
		}
		if (sym < 256) {
			if (p >= cap) {
				return FC_Z_OUTPUT;
			}
			o[p++] = (uint8_t)sym;
		} else if (sym == 256) {
			*pos = p;
			return FC_Z_OK;
		} else {
			uint32_t len, d;

			sym -= 257;
			if (sym >= 29) {
				return FC_Z_SYMBOL;
			}
			len = len_base[sym] + get_bits(r, len_extra[sym]);
			sym = decode(r, &dist);
			if (sym < 0) {
				return r->err ? FC_Z_TRUNCATED : FC_Z_SYMBOL;
			}
			if (sym >= DIST_SYMS) {
				return FC_Z_SYMBOL;
			}
			d = dist_base[sym] + get_bits(r, dist_extra[sym]);
			if (r->err) {
				return FC_Z_TRUNCATED;
			}
			/* avail: the history before o[0]; o is buf + hist, so o - d stays in buf */
			if (d > p + avail) {
				return FC_Z_DISTANCE;
			}
			if (len > cap - p) {
				return FC_Z_OUTPUT;
			}
			for (uint32_t i = 0; i < len; i++) {
				o[p + i] = o[(int32_t)(p + i) - (int32_t)d];
			}
			p += len;
		}
	}
}

static int fixed_tables(void)
{
	uint8_t lengths[LIT_SYMS];
	int i = 0;

	for (; i < 144; i++) {
		lengths[i] = 8;
	}
	for (; i < 256; i++) {
		lengths[i] = 9;
	}
	for (; i < 280; i++) {
		lengths[i] = 7;
	}
	for (; i < LIT_SYMS; i++) {
		lengths[i] = 8;
	}
	build(&lit, lengths, LIT_SYMS);
	memset(lengths, 5, DIST_SYMS + 2);
	build(&dist, lengths, DIST_SYMS + 2);
	return FC_Z_OK;
}

static int dynamic_tables(struct reader *r)
{
	static const uint8_t order[CL_SYMS] = {16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15};
	uint8_t lengths[LIT_SYMS + 32];
	int nlen = (int)get_bits(r, 5) + 257;
	int ndist = (int)get_bits(r, 5) + 1;
	int ncode = (int)get_bits(r, 4) + 4;
	int i, err;

	if (r->err) {
		return FC_Z_TRUNCATED;
	}
	if (nlen > 286 || ndist > 30) {
		return FC_Z_TABLE;
	}
	memset(lengths, 0, CL_SYMS);
	for (i = 0; i < ncode; i++) {
		lengths[order[i]] = (uint8_t)get_bits(r, 3);
	}
	if (r->err) {
		return FC_Z_TRUNCATED;
	}
	if (build(&lit, lengths, CL_SYMS) != 0) {
		return FC_Z_TABLE;
	}
	i = 0;
	while (i < nlen + ndist) {
		int sym = decode(r, &lit);

		if (sym < 0) {
			return r->err ? FC_Z_TRUNCATED : FC_Z_TABLE;
		}
		if (sym < 16) {
			lengths[i++] = (uint8_t)sym;
		} else {
			int prev = 0, rep;

			if (sym == 16) {
				if (i == 0) {
					return FC_Z_TABLE;
				}
				prev = lengths[i - 1];
				rep = 3 + (int)get_bits(r, 2);
			} else if (sym == 17) {
				rep = 3 + (int)get_bits(r, 3);
			} else {
				rep = 11 + (int)get_bits(r, 7);
			}
			if (r->err) {
				return FC_Z_TRUNCATED;
			}
			if (i + rep > nlen + ndist) {
				return FC_Z_TABLE;
			}
			while (rep--) {
				lengths[i++] = (uint8_t)prev;
			}
		}
	}
	if (lengths[256] == 0) {
		return FC_Z_TABLE;   /* no end-of-block code */
	}
	/* a table may be incomplete only with a single code */
	err = build(&lit, lengths, nlen);
	if (err < 0 || (err > 0 && nlen - lit.count[0] != 1)) {
		return FC_Z_TABLE;
	}
	/* nlen + ndist is below sizeof(lengths), and build only reads ndist of them */
	err = build(&dist, lengths + nlen, ndist);
	if (err < 0 || (err > 0 && ndist - dist.count[0] != 1)) {
		return FC_Z_TABLE;
	}
	return FC_Z_OK;
}

void fc_inflate_reset(struct fc_inflate *z)
{
	z->hist = 0;
	z->out = 0;
	z->header = true;
}

void fc_inflate_init(struct fc_inflate *z, uint8_t *buf, uint32_t cap)
{
	z->buf = buf;
	z->cap = cap;
	fc_inflate_reset(z);
}

int fc_inflate_record(struct fc_inflate *z, const uint8_t *in, uint32_t n, const uint8_t **out, uint32_t *out_len)
{
	struct reader r = {.in = in, .n = n};
	uint8_t *o = z->buf + z->hist;
	uint32_t room = z->cap - z->hist, p = 0;
	bool flushed = false;

	z->out = 0;
	if (z->header) {
		if (n < 2) {
			return FC_Z_TRUNCATED;
		}
		if ((in[0] & 0x0F) != 8 || (in[0] >> 4) > 7 || (in[1] & 0x20) || ((in[0] << 8) | in[1]) % 31) {
			return FC_Z_HEADER;
		}
		r.pos = 2;
		z->header = false;
	}
	while (!(flushed && r.pos == r.n)) {
		uint32_t last = get_bits(&r, 1);
		uint32_t type = get_bits(&r, 2);
		int err;

		if (r.err) {
			return FC_Z_TRUNCATED;
		}
		if (last) {
			return FC_Z_BLOCK;   /* the sender never finishes the stream */
		}
		flushed = false;
		if (type == 0) {
			uint32_t len;

			r.bitbuf = 0;
			r.bitcnt = 0;
			if (r.n - r.pos < 4) {
				return FC_Z_TRUNCATED;
			}
			len = in[r.pos] | ((uint32_t)in[r.pos + 1] << 8);
			if ((len ^ 0xFFFF) != (in[r.pos + 2] | ((uint32_t)in[r.pos + 3] << 8))) {
				return FC_Z_BLOCK;
			}
			r.pos += 4;
			if (len > r.n - r.pos) {
				return FC_Z_TRUNCATED;
			}
			if (len > room - p) {
				return FC_Z_OUTPUT;
			}
			memcpy(o + p, in + r.pos, len);
			r.pos += len;
			p += len;
			flushed = len == 0;
			continue;
		}
		if (type == 3) {
			return FC_Z_BLOCK;
		}
		err = type == 1 ? fixed_tables() : dynamic_tables(&r);
		if (err) {
			return err;
		}
		err = block_codes(&r, o, room, &p, z->hist);
		if (err) {
			return err;
		}
	}
	z->out = p;
	*out = o;
	*out_len = p;
	return FC_Z_OK;
}

void fc_inflate_commit(struct fc_inflate *z)
{
	uint32_t total = z->hist + z->out;

	if (total > FC_Z_HISTORY) {
		memmove(z->buf, z->buf + total - FC_Z_HISTORY, FC_Z_HISTORY);
		total = FC_Z_HISTORY;
	}
	z->hist = total;
	z->out = 0;
}
