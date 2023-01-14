#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#define assert(x) do { if (!(x)) ++*((char *)0); } while (0)
//#define bch_assert assert
//#define DPRINTF(x...) printf(x)
//#define USE_DPRINTF 1
//#define assert(x) (void) 0
#define DPRINTF(x...) (void) 0

#define printk(format...) (void) 0 //printf(format)
#define CONFIG_YAFFS_DIRECT

#include "yaffs_ecc_mlc.c"

static void rnd(char *xd, unsigned n) {
    char *d;
    static unsigned char s[256];
    static unsigned char i, j;
    unsigned char t = 0;
    if (!xd) {
	i = j = 0;
	do { s[i] = i; } while (++i);
	return;
    }
    for (d = xd; d != xd + n; ++d) {
	j += s[i];
        t = s[i];
        s[i] = s[j];
        s[j] = t;
	*d = s[s[i]+s[j]];
        ++i;
    }
}

static int fast_verify(unsigned char *data) {
    unsigned char r[CHECK_BYTES];
    encode(data, r);
    return !memcmp(data + DATA_BYTES, r, CHECK_BYTES);
}

static void logictest() {
     int i, i2, e;
     unsigned char data[CODE_BYTES];
     memset(data, 0, sizeof(data));
     for (i = 0; i != 100; ++i) {
	 tpnum_t difl[LPOLY_SIZE];
	 tpnum_t roots[LPOLY_SIZE];
	 tpnum_t sy[POLY_SIZE];
	 int e1 = i % CODE_BITS;
	 int e2 = (i + 1) % CODE_BITS;
	 yaffs_ecc_calc_mlc_other(data);
	 assert(fast_verify(data));
	 syndrome(data, sy);
	 for (i2 = 0; i2 != POLY_SIZE; ++i2) assert(!pow[sy[i2]]);
	 int npoly = bm(sy, difl);
	 int nroots = chien(difl, roots);
	 assert(nroots == 0);
	 data[e1 / 8] ^= 1 << (e1 % 8);
	 assert(!fast_verify(data));
	 syndrome(data, sy);
	 e = 0;
	 for (i2 = 0; i2 != POLY_SIZE; ++i2) if (pow[sy[i2]]) e = 1;
	 assert(e);
	 npoly = bm(sy, difl);
	 nroots = chien(difl, roots);
	 assert(nroots == 1);
	 assert(fix_single(data, sy));
	 assert(fast_verify(data));
	 data[e1 / 8] ^= 1 << (e1 % 8);
	 data[e2 / 8] ^= 1 << (e2 % 8);
	 assert(!fast_verify(data));
	 syndrome(data, sy);
	 assert(!fix_single(data, sy));
	 npoly = bm(sy, difl);
	 nroots = chien(difl, roots);
	 assert(nroots == 2);
	 assert(yaffs_ecc_correct_mlc_other(data) > 0);
	 assert(fast_verify(data));
	 data[3] ^= 0xf7;
	 syndrome(data, sy);
	 npoly = bm(sy, difl);
	 nroots = chien(difl, roots);
	 assert(yaffs_ecc_correct_mlc_other(data) < 0);
	 rnd(data, CODE_BYTES);
     }
     printf("test %s ok\n", __PRETTY_FUNCTION__);
}

static void chbit(unsigned char *data, unsigned bit, int mode) {
    data += bit / 8;
    bit = 1 << (bit % 8);
    if (!mode) *data &= ~bit;
    else if (mode > 0) *data |= bit;
    else *data ^= bit;
}

//begin: first bit to set
//end: first bit not to set
//mode: 0 - clear, >0 - set, <0 - flip
static void chbits(unsigned char *data, unsigned begin,
	unsigned end, int mode) {
    if (end <= begin || !data) return;
    while (begin != end && (begin & 7)) chbit(data, begin++, mode);
    while (begin != end && (end & 7)) chbit(data, --end, mode);
    if (begin == end) return;
    begin /= 8;
    end /= 8;
    if (mode >= 0) {
	memset(data + begin, mode ? 0xff : 0x00, end - begin);
	return;
    }
    unsigned char *i;
    for (i = data + begin; i != data + end; ++i) *i ^= 0xff;
}

static void regrtest() {
    unsigned i, i2;
    unsigned char data1[CODE_BYTES];
    unsigned char data2[CODE_BYTES];
    tpnum_t sy[POLY_SIZE];
    memset(data1, 0, sizeof(data1));
    memset(data2, 0, sizeof(data2));

    assert(fast_verify(data1)); //all zeros are valid data

    //test root->position conversion in the slow path
    chbit(data1, 0, 1);
    chbit(data1, CODE_BITS - 1, 1);
    assert(yaffs_ecc_correct_mlc_other(data1) > 0);
    assert(!memcmp(data1, data2, sizeof(data1)));

    if (CODE_BYTES == 528) {
	//this case tests inversion of a^0 in bm() function
	chbits(data1, 3461, 3467, 1);
	assert(yaffs_ecc_correct_mlc_other(data1) > 0);
	assert(!memcmp(data1, data2, sizeof(data1)));
    }

    for (i = 0; i != CODE_BITS; ++i) {
	chbits(data1, i, i + 1, -1);
	assert(yaffs_ecc_correct_mlc_other(data1) > 0);
	assert(!memcmp(data1, data2, sizeof(data1)));
    }
    printf("test %s at 1 bit ok\n", __PRETTY_FUNCTION__);

    for (i = 1; i != CODE_BITS; ++i) {
	int i2;
	for (i2 = 0; i2 < i; i2 += 1) {
	    chbit(data1, i, -1);
	    chbit(data1, i2, -1);
	    assert(yaffs_ecc_correct_mlc_other(data1) > 0);
	    assert(!memcmp(data1, data2, sizeof(data1)));
	}
    }
    printf("test %s at 2 bits ok\n", __PRETTY_FUNCTION__);
    unsigned n;
    for (n = (POLY_SIZE - 1) / 2; n > 2; --n) {
	for (i = CODE_BITS; i != n - 1; --i) {
	    chbits(data1, i - n, i, -1);
	    assert(yaffs_ecc_correct_mlc_other(data1) > 0);
	    assert(!memcmp(data1, data2, sizeof(data1)));
	}
	printf("test %s positive at n %d ok\n", __PRETTY_FUNCTION__, n);
    }
    for (n = POLY_SIZE - 1; n != (POLY_SIZE - 1) / 2; --n) {
	for (i = CODE_BITS; i != n - 1; --i) {
	    chbits(data1, i - n, i, -1);
	    assert(yaffs_ecc_correct_mlc_other(data1) < 0);
	    memset(data1, 0, sizeof(data1));
	}
	printf("test %s negative at n %d ok\n", __PRETTY_FUNCTION__, n);
    }
    printf("test %s ok\n", __PRETTY_FUNCTION__);
}

static unsigned char bits_bad1[CODE_BYTES] = {
    0x00, 0x00, 0x10, 0x04,
    0x10, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x10,
    0x00, 0x00, 0x00, 0x00
};
static unsigned char data_orig1[CODE_BYTES] = {
    0x00, 0x00, 0x10, 0x38,
    0x00, 0x00, 0x02, 0xf9,
    0x00, 0x00, 0x00, 0x01,
    0x00, 0x00, 0x05, 0x5a,
    0x73, 0x65, 0xb9, 0xe4
};
static unsigned char bits_bad2[CODE_BYTES] = {
    0, 0, 0, 0,
    0, 0x14, 0, 0,
    0, 0, 0, 0x10,
    0, 0, 0, 0,
    0, 0, 0, 0
};
static unsigned char data_orig2[CODE_BYTES] = {
    0, 0, 0, 0x21,
    0, 0, 0, 0x9a,
    0, 0, 0, 0x49,
    0, 0, 0x08, 0,
    0xd3, 0x62, 0xf2, 0x55
};

static int test_specific(unsigned char *bits_bad, unsigned char *data_orig) {
	unsigned char buf[CODE_BYTES];
	memcpy(buf, data_orig, DATA_BYTES);
	yaffs_ecc_calc_mlc_other(buf);
	if (memcmp(buf, data_orig, CODE_BYTES) != 0) {
		printf("bad orig data!\n");
		return 1;
	}
	unsigned i;
	int errors_total = 0;
	for (i = 0; i < CODE_BYTES; i++) {
		unsigned char bad = bits_bad[i];
		buf[i] ^= bad;
		for ( ; bad != 0; bad >>= 1) {
			errors_total += (bad & 1);
		}
	}
	if (errors_total > 4) {
		printf("too many errors %d!\n", errors_total);
		return 1;
	}
	if (errors_total == 0) {
		printf("no errors!\n");
		return 0;
	}
	if (yaffs_ecc_correct_mlc_other(buf) != 1) {
		printf("failed to fix data!\n");
		return 1;
	}
	if (memcmp(buf, data_orig, CODE_BYTES) != 0) {
		printf("bad fixed data!\n");
		return 1;
	}
	return 0;
}

int main() {
    if (test_specific(bits_bad1, data_orig1)) return 1;
    if (test_specific(bits_bad2, data_orig2)) return 1;
    rnd(0, 0);
    precalc();
    logictest();
    regrtest();
}

