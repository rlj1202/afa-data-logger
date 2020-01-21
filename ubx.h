#ifndef H_UBX
#define H_UBX

/*******************************************************************************
 * Constants
 ******************************************************************************/

const unsigned char UBX_SYNC_CHAR[2] = {0xb5, 0x62}; // 181, 98

enum UBX_CLASS {
	UBX_CLS_NAV = 0x01, // navigation results: position, speed, time, acc, heading, DOP, SVs used
	UBX_CLS_RXM = 0x02, // receiver manager messages: satellite status, RTC status
	UBX_CLS_INF = 0x04, // information messages: printf-style messages, with IDs such as error, warning, notice
	UBX_CLS_ACK = 0x05, // ack, nack messages: as replices to CFG input messages
	UBX_CLS_CFG = 0x06, // configuration input messages: set dynamic model, set DOP mask, set baud rate, etc
	UBX_CLS_MON = 0x0A, // monitoring messages: communication status, CPU load, stack usage, task status
	UBX_CLS_AID = 0x0B, // AssisNow Aiding Messages: ephemeris, almanac, other A-GPS data input
	UBX_CLS_TIM = 0x0D  // timing messages: timepulse output, timemark results
};

enum GNSS_ID {
	GNSS_ID_GPS     = 0,
	GNSS_ID_SBAS    = 1,
	GNSS_ID_GALILEO = 2,
	GNSS_ID_BEIDOU  = 3,
	GNSS_ID_IMES    = 4,
	GNSS_ID_QZSS    = 5,
	GNSS_ID_GLONASS = 6
};

/*******************************************************************************
 * Data types
 ******************************************************************************/

// little endian ordered, unless indicate
typedef unsigned char ubx_u1;
typedef unsigned char ubx_ru1_3; // float 3bit exp, eeebbbbb, (value & 0x1f) << (value >> 5)
typedef signed char ubx_l1;
typedef unsigned char ubx_x1;

typedef unsigned short ubx_u2;
typedef signed short ubx_i2;
typedef unsigned short ubx_x2;

typedef unsigned long ubx_u4;
typedef signed long ubx_i4;
typedef unsigned long ubx_x4;

typedef float ubx_r4;
typedef double ubx_r8;

typedef unsigned char ubx_ch;

/*******************************************************************************
 * Structures
 ******************************************************************************/

#pragma pack(push, 1)

struct ubx_packet {
	unsigned char sync[2];
	unsigned char cls;
	unsigned char id;
	unsigned short len;
	unsigned char payload[0];
};

template<typename T, unsigned char cls, unsigned char id, bool pool = false>
struct ubx_msg {
	T payload;
};

// UBX_NAV_POSLLH
// description: Geodetic Position Solution
// version: 15~22
// type: periodic/polled
struct ubx_nav_posllh_payload {
	ubx_u4 iTOW;   // ms; GPS time of week of the navigation epoch
	ubx_i4 lon;    // deg; longitude
	ubx_i4 lat;    // deg; latitude
	ubx_i4 height; // mm; height above ellipsoid
	ubx_i4 hMSL;   // mm; height above mean sea level
	ubx_u4 hAcc;   // mm; horizontal accuracy estimate
	ubx_u4 vAcc;   // mm; vertical accuracy estimate
};
typedef ubx_msg<ubx_nav_posllh_payload, UBX_CLS_NAV, 0x02, true> ubx_nav_posllh;

// UBX_CFG_PRT
// description: Polls the configuration for one I/O Port
// version: 15~22
// type: Poll Request
struct ubx_cfg_prt_payload {
	ubx_u1 portID;
};
typedef ubx_msg<ubx_cfg_prt_payload, UBX_CLS_CFG, 0x00> ubx_cfg_prt;

#pragma pack(pop)

/*******************************************************************************
 * Functions
 ******************************************************************************/

// ubx_calc_checksum(ck_a, ck_b, buffer, len)
//   calculates checksum
//   8-bit fletcher algorithm is used (TPC standard, RFC 1145)
//   range over from cls before ck_a
void ubx_calc_checksum(
		unsigned char *ck_a,
		unsigned char *ck_b,
		unsigned char *buf,
		int len
		);

template<typename T, unsigned char cls, unsigned char id, bool poll>
// ubx_make_packet
//   returns length of packet
int ubx_make_packet(unsigned char *dst, ubx_msg<T, cls, id, poll> msg);

/*******************************************************************************
 * Template Implementations
 ******************************************************************************/

template<typename T, unsigned char cls, unsigned char id, bool poll>
int ubx_make_packet(unsigned char *dst, ubx_msg<T, cls, id, poll> msg) {
	ubx_packet *packet = (ubx_packet*) dst;

	packet->sync[0] = UBX_SYNC_CHAR[0];
	packet->sync[1] = UBX_SYNC_CHAR[1];

	packet->cls = cls;
	packet->id = id;

	if (!poll) {
		packet->len = sizeof(msg.payload);

		for (int i = 0; i < packet->len; i++)
			packet->payload[i] = ((unsigned char *) &msg.payload)[i];
	} else packet->len = 0;

	unsigned char *ck_a = ((unsigned char *) packet->payload) + packet->len;
	unsigned char *ck_b = ck_a + 1;

	ubx_calc_checksum(ck_a, ck_b, &packet->cls,
			sizeof(packet->cls) + sizeof(packet->id) + sizeof(packet->len) + packet->len);

	return sizeof(ubx_packet) + packet->len + sizeof(unsigned char) * 2;
}

#endif
