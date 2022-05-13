#ifndef __MAXZENO_SLIP_H__
#define __MAXZENO_SLIP_H__

#include <zephyr/sys/util.h>
#include <errno.h>
#include <string.h>
#include <zephyr/drivers/uart_pipe.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>

/************************************************************
 * Predefined slip messages
 ************************************************************/
extern const unsigned char cmd_reset[11];
extern const unsigned char cmd_setPsk[35];
extern const unsigned char cmd_discovery[13];
extern const unsigned char cmd_setGmk[35];
extern const unsigned char cmd_activateGmk[19];
extern const unsigned char cmd_myst[19]; /* bypass the authentication */
extern const unsigned char mzIPv6Header[16]; /* bypass the authentication */

/************************************************************
 * Slip constants
 ************************************************************/
/* location in buffer */
/* NOTE: this assumes that the frame delimeter has been trimmed from the beginning and end */ 
#define SLIP_DEVICE_TYPE_BUFFER_IDX 0
#define SLIP_DEVICE_TYPE_VAL 0x02

#define SLIP_DEVICE_ADDRESS1_BUFFER_IDX 1
#define SLIP_DEVICE_ADDRESS1_VAL 0x00

#define SLIP_DEVICE_ADDRESS2_BUFFER_IDX 2
#define SLIP_DEVICE_ADDRESS2_VAL 0x01

#define SLIP_LAYER_BUFFER_IDX 3
#define SLIP_PRIMITIVE_BUFFER_IDX 4

#define SLIP_MAX_HEADER_LEN	 5

/* layer type */
#define SLIP_LAYER_MAC 0x02
#define SLIP_LAYER_ADP 0x03

/* Primitive codes */
#define SLIP_PRIMITIVE_NULL 0x00
#define SLIP_PRIMITIVE_ARRAY 0x01
#define SLIP_PRIMITIVE_STRUCT 0x02
#define SLIP_PRIMITIVE_BOOL 0x03
#define SLIP_PRIMITIVE_BIT 0x04
#define SLIP_PRIMITIVE_INT32 0x05
#define SLIP_PRIMITIVE_UINT32 0x06
#define SLIP_PRIMITIVE_INT16 0x10
#define SLIP_PRIMITIVE_ASCII 0x0A
#define SLIP_PRIMITIVE_UINT16 0x12
#define SLIP_PRIMITIVE_ENUM 0x16
#define SLIP_PRIMITIVE_FLOAT32 0x17
#define SLIP_PRIMITIVE_TIME 0x1B
#define SLIP_PRIMITIVE_BCD 0x0D
#define SLIP_PRIMITIVE_STRING 0x09
#define SLIP_PRIMITIVE_INT8 0x0F
#define SLIP_PRIMITIVE_UINT8 0x11
#define SLIP_PRIMITIVE_DATE 0x20
#define SLIP_PRIMITIVE_DATETIME 0x19
#define SLIP_PRIMITIVE_CHARARRAY 0x13
#define SLIP_PRIMITIVE_FLOAT64 0x18
#define SLIP_PRIMITIVE_RAWDATA 0x30
#define SLIP_PRIMITIVE_UNKNOWN 0xFF

/* command primitive codes */
#define SLIP_PRIMITIVE_CONFIRM 0x80
#define SLIP_PRIMITIVE_GET_REQUEST 0x01
#define SLIP_PRIMITIVE_GET_CONFIRM SLIP_PRIMITIVE_CONFIRM & SLIP_PRIMITIVE_GET
#define SLIP_PRIMITIVE_SET_REQUEST 0x02
#define SLIP_PRIMITIVE_SET_CONFIRM SLIP_PRIMITIVE_CONFIRM & SLIP_PRIMITIVE_SET
#define SLIP_PRIMITIVE_MLME_GET_REQUEST 0x01
#define SLIP_PRIMITIVE_MLME_GET_REQUEST_SIZE 0x02

#define SLIP_PRIMITIVE_MLME_GET_CONFIRM 0x81
#define SLIP_PRIMITIVE_MLME_SET_REQUEST 0x02
#define SLIP_PRIMITIVE_MLME_SET_CONFIRM 0x82

#define SLIP_PRIMITIVE_ADPM_GET_REQUEST 0x01
#define SLIP_PRIMITIVE_ADPM_GET_REQUEST_SIZE 0x02

#define SLIP_PRIMITIVE_ADPM_GET_CONFIRM 0x81
#define SLIP_PRIMITIVE_ADPM_SET_REQUEST 0x02
#define SLIP_PRIMITIVE_ADPM_SET_CONFIRM 0x82
#define SLIP_PRIMITIVE_ADPM_DATA_REQUEST 0x03
#define SLIP_PRIMITIVE_ADPM_DATA_CONFIRM 0x83
#define SLIP_PRIMITIVE_ADPM_DATA_INDICATION 0x84
#define SLIP_PRIMITIVE_ADPM_DISCOVERY_REQUEST 0x05
#define SLIP_PRIMITIVE_ADPM_DISCOVERY_CONFIRM 0x85
#define SLIP_PRIMITIVE_ADPM_NETWORK_START_REQUEST 0x06
#define SLIP_PRIMITIVE_ADPM_NETWORK_START_CONFIRM 0x86
#define SLIP_PRIMITIVE_ADPM_NETWORK_JOIN_REQUEST 0x07
#define SLIP_PRIMITIVE_ADPM_NETWORK_JOIN_CONFIRM 0x87
#define SLIP_PRIMITIVE_ADPM_NETWORK_LEAVE_REQUEST 0x09
#define SLIP_PRIMITIVE_ADPM_NETWORK_LEAVE_INDICATION 0x8A
#define SLIP_PRIMITIVE_ADPM_NETWORK_LEAVE_CONFIRM 0x89
#define SLIP_PRIMITIVE_ADPM_RESET_REQUEST 0x0B
#define SLIP_PRIMITIVE_ADPM_RESET_CONFIRM 0x8B
#define SLIP_PRIMITIVE_ADPM_NETWORK_STATUS_INDICATION 0x8C
#define SLIP_PRIMITIVE_ADPM_ROUTE_DISCOVERY_REQUEST 0x0D
#define SLIP_PRIMITIVE_ADPM_ROUTE_DISCOVERY_CONFIRM 0x8D
#define SLIP_PRIMITIVE_ADPM_PATH_DISCOVERY_REQUEST 0x10
#define SLIP_PRIMITIVE_ADPM_PATH_DISCOVERY_CONFIRM 0x91
#define SLIP_PRIMITIVE_ADPM_BUFFER_INDICATION 0x90
#define SLIP_PRIMITIVE_ADPM_LBP_REQUEST 0x0E
#define SLIP_PRIMITIVE_ADPM_LBP_CONFIRM 0x8E
#define SLIP_PRIMITIVE_ADPM_LBP_INDICATOIN 0x8F

/* command primitive sizes */
#define SLIP_PRIMITIVE_MLME_SET_REQUEST_SIZE 0x03
#define SLIP_PRIMITIVE_ADPM_SET_REQUEST_SIZE 0x03

/* strings are handled differently because they can have a variable length */
#define SLIP_PRIMITIVE_STRING_MORE_128BYTES_2BYTE_KEY 0x82
#define SLIP_PRIMITIVE_STRING_MORE_128BYTES_4BYTE_KEY 0x84

/* error/status codes */
#define SLIP_STATUS_SUCCESS 0x00
#define SLIP_STATUS_ALTERNATE_PANID_DETECTION 0x03
#define SLIP_STATUS_INVALID_REQUEST 0x05
#define SLIP_STATUS_NOT_PERMITTED 0x06
#define SLIP_STATUS_TX_RX_OFF 0x08
#define SLIP_STATUS_PREP_GENERATED 0x09
#define SLIP_STATUS_ROUTE_ERROR 0x0C
#define SLIP_STATUS_INVALID_IPV6_ADDRESS 0x0D
#define SLIP_STATUS_INVALID_IPV6_HOP_MAX 0x0E
#define SLIP_STATUS_BT_TABLE_FULL 0x0F
#define SLIP_STATUS_FRAME_NOT_BUFFERED 0x10
#define SLIP_STATUS_FATAL_ERROR 0x1E
#define SLIP_STATUS_STARTUP_FAILURE 0x1F
#define SLIP_STATUS_LOAD_OVERFLOW 0x20
#define SLIP_STATUS_MEMORY_ERROR 0x21
#define SLIP_STATUS_WRITE_ONLY 0x22
#define SLIP_STATUS_NT_FULL 0x23
#define SLIP_STATUS_RT_FULL 0x24
#define SLIP_STATUS_INCOMPLETE_PATH 0x25
#define SLIP_STATUS_LINEDRIVER_SHUTDOWN 0x30
#define SLIP_STATUS_LINEDRIVER_HOT_GAIN_REDUCED 0x31
#define SLIP_STATUS_LINEDRIVER_OVERCURRENT 0x32
#define SLIP_STATUS_LINEDRIVER_NORMAL 0x33
#define SLIP_STATUS_PHY_NOT_RESPONSIVE 0x34
#define SLIP_STATUS_TMAP_TX 0xA0
#define SLIP_STATUS_TMAP_RX 0xA1
#define SLIP_STATUS_ROUTING_TABLE_UPDATED 0xA2
#define SLIP_STATUS_MAC_FRAME_COUNTER_REPORTED 0xA4
#define SLIP_STATUS_LOADNG_SEQ_NUM_REPORTED 0xA5
#define SLIP_STATUS_FCS_ERROR 0xA6
#define SLIP_STATUS_BAND_CHANGED 0xBA
#define SLIP_STATUS_COUNTER_ERROR 0xDB
#define SLIP_STATUS_IMPROPER_KEY_TYPE 0xDC
#define SLIP_STATUS_IMPROPER_SECURITY_LEVEL 0xDD
#define SLIP_STATUS_UNSUPPORTED_LEGACY 0xDE
#define SLIP_STATUS_UNSUPPORTED_SECURITY 0xDF
#define SLIP_STATUS_BEACON_LOSS 0xE0
#define SLIP_STATUS_CHANNEL_ACCESS_FAILURE 0xE1
#define SLIP_STATUS_DENIED 0xE2
#define SLIP_STATUS_DISABLE_TRX_FAILURE 0xE3
#define SLIP_STATUS_SECURITY_ERROR 0xE4
#define SLIP_STATUS_FRAME_TOO_LONG 0xE5
#define SLIP_STATUS_INVALID_GTS 0xE6
#define SLIP_STATUS_INVALID_HANDLE 0xE7
#define SLIP_STATUS_INVALID_PARAMETER 0xE8
#define SLIP_STATUS_NO_ACK 0xE9
#define SLIP_STATUS_NO_BEACON 0xEA
#define SLIP_STATUS_NO_DATA 0xEB
#define SLIP_STATUS_NO_SHORT_ADDRESS 0xEC
#define SLIP_STATUS_OUT_OFF_CAP 0xED
#define SLIP_STATUS_REALIGNMENT 0xEF
#define SLIP_STATUS_TRANSACTION_EXPIRED 0xF0
#define SLIP_STATUS_TRANSACTION_OVERFLOW 0xF1
#define SLIP_STATUS_TX_ACTIVE 0xF2
#define SLIP_STATUS_UNAVAILABLE_KEY 0xF3
#define SLIP_STATUS_UNSUPPORTED_ATTRIBUTE 0xF4
#define SLIP_STATUS_INVALID_ADDRESS 0xF5
#define SLIP_STATUS_ON_TIME_TOO_LONG 0xF6
#define SLIP_STATUS_PAST_TIME 0xF7
#define SLIP_STATUS_TRACKING_OFF 0xF8
#define SLIP_STATUS_INVALID_INDEX 0xF9
#define SLIP_STATUS_LIMIT_REACHED 0xFA
#define SLIP_STATUS_READ_ONLY 0xFB
#define SLIP_STATUS_SCAN_IN_PROGRESS 0xFC
#define SLIP_STATUS_SUPERFRAME_OVERLAP 0xFD

/* ADPM attribute codes */
#define SLIP_ADPM_ATTRID_DEVICE_TYPE 0x10
#define SLIP_ADPM_ATTRID_SECURITY_LEVEL 0x00
#define SLIP_ADPM_ATTRID_GROUP_TABLE 0x0E
#define SLIP_ADPM_ATTRID_PSK 0xEF
#define SLIP_ADPM_ATTRID_ACTIVATE_KEY_INDEX 0x22
#define SLIP_ADPM_ATTRID_BYPASS_AUTH 0x93
#define SLIP_ADPM_ATTRID_PREFIX_TABLE 0x01
#define SLIP_ADPM_ATTRID_MAX_HOPS 0x0F
#define SLIP_ADPM_ATTRID_BROADCAST_LOG_TABLE_ENTRY 0x02
#define SLIP_ADPM_ATTRID_ROUTING_TABLE 0x0c

/* MLME attribute coes */ 
#define SLIP_MLME_ATTRID_MAC_BAND_SELECT 0x0225 
#define SLIP_MLME_ATTRID_MAC_EXTENDED_ADDRESS 0x01A3
#define SLIP_MLME_ATTRID_MAC_SECURITY_ENABLED 0x005D
#define SLIP_MLME_ATTRID_MAC_TX_ANALOG_GAIN 0x01AF
#define SLIP_MLME_ATTRID_MAC_SHORT_ADDRESS 0x0053
#define SLIP_MLME_ATTRID_MAC_HIGH_PRIORITY_WINDOW_SIZE 0x0100
#define SLIP_MLME_ATTRID_MAC_BROADCAST_MAX_CW_ENABLE 0x011E
#define SLIP_MLME_ATTRID_MAC_MIN_CW_ATTEMPTS 0x0114
#define SLIP_MLME_ATTRID_MAC_FORCE_MODULATION 0x01ad
#define SLIP_MLME_ATTRID_MAC_ENABLE_ROLLBACK 0x0193
#define SLIP_MLME_ATTRID_MAC_KEY_TABLE 0x0071
#define SLIP_MLME_ATTRID_MAC_NEIGHBOR_TABLE 0x010a
#define SLIP_MLME_ATTRID_MAC_MIN_BE 0x004f

/* Modulation codes, used with SLIP_MLME_ATTID_MAC_FORCE_MODULATION attribute */
#define MODULATION_ROBO 	0x00
#define MODULATION_BPSK 	0x01
#define MODULATION_QPSK 	0x02
#define MODULATION_8PSK 	0x03
#define MODULATION_DISABLE 	0xff

/* slip message consts */
#define CHAR_END     0xC0
#define CHAR_ESC     0xDB
#define CHAR_ESC_END 0xDC
#define CHAR_ESC_ESC 0xDD

/* IPv6 Constants */
#define ADPM_IPv6_HEADER_START_IDX 12
#define ADPM_IPv6_HEADER_PAYLOAD_IDX (ADPM_IPv6_HEADER_START_IDX + 8 + 16 + 16)
#define IPv6_HEADER_SIZE 40 // in bytes

#define MAX_PLC_PACKET 	1500
struct PLCPacket {
	uint8_t buf[MAX_PLC_PACKET];
	int len;
	uint8_t state;
	struct k_work work;
	const struct device *dev;
	struct k_mem_slab *slab;
	struct maxzeno_data *zeno_ctx;
};

enum slip_layer
{
	SLIP_MAC,
	SLIP_ADPM
};

#define STATE_READING_PKT 	(1 << 0)
#define STATE_ESC 			(1 << 1)
#define STATE_INVALID_PKT	(1 << 2)

struct slip_message_response
{
	/* data that should exist for every message */
	enum slip_layer layer;
	uint8_t primitive;

	/* data that depends on the type */
	uint8_t status;
	uint16_t attribute_id;
	uint16_t attribute_idx;
};

int mlme_set_msg(uint8_t *buf, int buf_len, uint16_t attr_id, uint16_t attr_index, const void *data, int data_size);

int mlme_get_msg(uint8_t *msg_buffer, int buf_len, uint16_t attr_id, uint16_t attr_index);

int adpm_set_msg(uint8_t *buf, int buf_len, uint8_t attr_id, uint16_t attr_index, const void *data, int data_size);

int adpm_get_msg(uint8_t *msg_buffer, int buf_len, uint8_t attr_id, uint16_t attr_index);

unsigned int fcs(unsigned char bytes[], int off, int length);

/* read buffer and signal that a new message has been received */
int slip_input_byte(struct PLCPacket *pkt, unsigned char c);

int set_network_start(uint8_t *msg_buffer, const uint8_t panid[2]);

int network_discovery(uint8_t *msg_buffer, int discovery_time);

#endif /* __MAXZENO_SLIP_H__ */
