#include "ubx.h"

void ubx_calc_checksum(
		unsigned char *ck_a,
		unsigned char *ck_b,
		unsigned char *buf,
		int len
		) {
	*ck_a = 0;
	*ck_b = 0;

	for (int i = 0; i < len; i++) {
		*ck_a += buf[i];
		*ck_b += *ck_a;
	}
}
