/* The decoder of Faceclaw's records (VM.md §6.11): one persistent zlib stream, RFC 1950 and 1951, cut into records
 * by Z_SYNC_FLUSH, so that each record ends on a byte boundary with an empty stored block and the stream never
 * finishes. A whole record is in memory when it is decoded, so the decoder never suspends. Pure C with no
 * platform calls: tests/fc_inflate_test.c runs it on a PC. */
#ifndef FC_INFLATE_H_
#define FC_INFLATE_H_

#include <stdbool.h>
#include <stdint.h>

#define FC_Z_HISTORY 32768u

enum fc_inflate_err {
	FC_Z_OK = 0,
	FC_Z_TRUNCATED = -1,  /* input ended inside a block, or without the sync flush's empty stored block */
	FC_Z_HEADER = -2,     /* not a zlib header this decoder takes */
	FC_Z_BLOCK = -3,      /* reserved block type, a final block, a stored block's lengths disagree */
	FC_Z_TABLE = -4,      /* a bad Huffman table */
	FC_Z_SYMBOL = -5,     /* a code outside the table, a length or distance symbol out of range */
	FC_Z_DISTANCE = -6,   /* a match before the start of the history */
	FC_Z_OUTPUT = -7,     /* the record decodes to more than the room after the history */
};

struct fc_inflate {
	uint8_t *buf;     /* the history, then the record being decoded */
	uint32_t cap;
	uint32_t hist;    /* bytes of history at buf, at most FC_Z_HISTORY */
	uint32_t out;     /* bytes of the last decoded record, at buf + hist */
	bool header;      /* the zlib header is still to come */
};

/* buf holds FC_Z_HISTORY bytes of history plus the largest decoded record. */
void fc_inflate_init(struct fc_inflate *z, uint8_t *buf, uint32_t cap);
/* A new stream: no history, a zlib header expected. */
void fc_inflate_reset(struct fc_inflate *z);
/* Decodes one record. On FC_Z_OK *out and *out_len give the bytes, valid until commit. After an error the stream is
 * lost: nothing but fc_inflate_reset makes it usable again (its history is not what the sender's is). */
int fc_inflate_record(struct fc_inflate *z, const uint8_t *in, uint32_t n, const uint8_t **out, uint32_t *out_len);
/* The decoded record becomes history. */
void fc_inflate_commit(struct fc_inflate *z);

#endif
