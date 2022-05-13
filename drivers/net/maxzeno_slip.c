#include "maxzeno_slip.h"
#include <zephyr/sys/crc.h>

#define SLIP_LOG_MODULE_NAME maxzeno_slip

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(SLIP_LOG_MODULE_NAME, CONFIG_MAXZENO_LOG_LEVEL);

const unsigned char cmd_reset[]= {
	0xC0,0x02,0x00,0x01,0x03,0x0B,0x02,0x00,0x36,0x33,0xC0};

const unsigned char cmd_setPsk[]= {
	0xC0,0x02,0x00,0x01,0x03,0x02,0x02,0x03,0x11,0xEF,0x12,
	0x00,0x00,0x09,0x10,0x00,0x01,0x02,0x03,0x04,0x05,0x06,
	0x07,0x08,0x09,0xA0,0xB0,0xDB,0xDC,0xD0,0xE0,0xF0,0x8E,
	0xC9,0xC0};

const unsigned char cmd_setGmk[]= {
	0xC0,0x02,0x00,0x01,0x02,0x02,0x02,0x03,0x12,0x00,0x71,
	0x12,0x00,0x00,0x09,0x10,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xB2,
	0x05,0xC0};

const unsigned char cmd_activateGmk[]= {
	0xC0,0x02,0x00,0x01,0x03,0x02,0x02,0x03,0x11,0x22,0x12,
	0x00,0x00,0x09,0x01,0x00,0x58,0x8D,0xC0};

/* bypass the authentication */
const unsigned char cmd_myst[]= {
			0xC0,
			0x02,0x00,0x01,0x03, 			// Primitive ADPM-SET
			0x02,0x02,
			0x03,
			0x11,0x93, 						// Uint8: 0x93 - adpBypassAuth
			0x12, 0x00,0x00, 				// Uint16: 0x00,0x00
			0x09,0x01,0x01, 				// String: 1 byte: 0x01
			0x02,0xB9,
			0xC0};

const unsigned char mzIPv6Header[]= {
				0x02,
				0x00,0x01,0x03,
				0x03, 								// ADPD-DATA.request
				0x02,0x05,
				0x12,0x00,0x37, 					// Uint16_t NSDU Length
				0x09,0x84,0x00,0x00,0x00,0x37};     // String with 4 byte length


int mlme_get_msg(uint8_t *msg_buffer, int buf_len, uint16_t attr_id, uint16_t attr_index)
{
	int index = 0;
	uint16_t crc;

	if (buf_len < 15) {
		return -EINVAL;
	}

	msg_buffer[index++] = SLIP_DEVICE_TYPE_VAL; 					// Device type: 02
	msg_buffer[index++] = SLIP_DEVICE_ADDRESS1_VAL; 				// Device address (0): 00
	msg_buffer[index++] = SLIP_DEVICE_ADDRESS2_VAL; 				// Device address (1): 01
	msg_buffer[index++] = SLIP_LAYER_MAC; 							// Layer: 02 (MAC)

	msg_buffer[index++] = SLIP_PRIMITIVE_MLME_GET_REQUEST; 			// Primitive type: MLME-GET.request
	msg_buffer[index++] = SLIP_PRIMITIVE_STRUCT; 					// XDR tag: structure
	msg_buffer[index++] = SLIP_PRIMITIVE_MLME_GET_REQUEST_SIZE; 	// Number of parameters: 2

	msg_buffer[index++] = SLIP_PRIMITIVE_UINT16; 					// XDR tag: Uint16
	msg_buffer[index++] = attr_id >> 8; 							// Attribute ID
	msg_buffer[index++] = attr_id; 									// Attribute ID
	
	msg_buffer[index++] = SLIP_PRIMITIVE_UINT16; 					// XDR tag: Uint16
	msg_buffer[index++] = attr_index >> 8; 							// Attribute Index
	msg_buffer[index++] = attr_index; 								// Attribute Index

	crc = crc16_itu_t(0xffff, &msg_buffer[0], index);
	msg_buffer[index++] = crc >> 8;
	msg_buffer[index++] = crc;

	return index;
}

int adpm_get_msg(uint8_t *msg_buffer, int buf_len, uint8_t attr_id, uint16_t attr_index)
{
	int index = 0;
	uint16_t crc;

	if (buf_len < 15) {
		return -EINVAL;
	}

	msg_buffer[index++] = SLIP_DEVICE_TYPE_VAL; 					// Device type: 02
	msg_buffer[index++] = SLIP_DEVICE_ADDRESS1_VAL; 				// Device address (0): 00
	msg_buffer[index++] = SLIP_DEVICE_ADDRESS2_VAL; 				// Device address (1): 01
	msg_buffer[index++] = SLIP_LAYER_ADP; 							// Layer: 03 (ADPM)

	msg_buffer[index++] = SLIP_PRIMITIVE_ADPM_GET_REQUEST; 			// Primitive type: ADPM-GET.request
	msg_buffer[index++] = SLIP_PRIMITIVE_STRUCT; 					// XDR tag: structure
	msg_buffer[index++] = SLIP_PRIMITIVE_ADPM_GET_REQUEST_SIZE; 	// Number of parameters: 2

	msg_buffer[index++] = SLIP_PRIMITIVE_UINT8; 					// XDR tag: Uint16
	msg_buffer[index++] = attr_id; 									// Attribute ID

	msg_buffer[index++] = SLIP_PRIMITIVE_UINT16; 					// XDR tag: Uint16
	msg_buffer[index++] = attr_index >> 8; 							// Attribute Index
	msg_buffer[index++] = attr_index; 								// Attribute Index

	crc = crc16_itu_t(0xffff, &msg_buffer[0], index);
	msg_buffer[index++] = crc >> 8;
	msg_buffer[index++] = crc;

	return index;
}

int mlme_set_msg(uint8_t *msg_buffer, int buf_len, uint16_t attr_id, uint16_t attr_index, const void *data, int data_size)
{
	int index = 0;
	uint16_t crc;
	msg_buffer[index++] = SLIP_DEVICE_TYPE_VAL; 	// Device type: 02
	msg_buffer[index++] = SLIP_DEVICE_ADDRESS1_VAL; 	// Device address (0): 00
	msg_buffer[index++] = SLIP_DEVICE_ADDRESS2_VAL; 	// Device address (1): 01
	msg_buffer[index++] = SLIP_LAYER_MAC; 	// Layer: 02 (MAC)

	msg_buffer[index++] = SLIP_PRIMITIVE_MLME_SET_REQUEST; 	// Primitive type: ADPM-SET.request
	msg_buffer[index++] = SLIP_PRIMITIVE_STRUCT; 	// XDR tag: structure
	msg_buffer[index++] = SLIP_PRIMITIVE_MLME_SET_REQUEST_SIZE; 	// Number of parameters: 3

	msg_buffer[index++] = SLIP_PRIMITIVE_UINT16; 	// XDR tag: Uint16
	msg_buffer[index++] = attr_id >> 8; 	// Attribute ID
	msg_buffer[index++] = attr_id; 	// Attribute ID

	msg_buffer[index++] = SLIP_PRIMITIVE_UINT16; 	// XDR tag: Uint16
	msg_buffer[index++] = attr_index >> 8; 	// Attribute ID
	msg_buffer[index++] = attr_index; 	// Attribute ID

	msg_buffer[index++] = SLIP_PRIMITIVE_STRING; 	// XDR tag: String
	if (data_size < 128) {
		msg_buffer[index++] = data_size;				
	} else if (data_size <= UINT16_MAX) {
		msg_buffer[index++] = 0x82;
		msg_buffer[index++] = data_size >> 8;
		msg_buffer[index++] = data_size;
	} else {
		return -EINVAL;
	}

	if (index + data_size > buf_len) {
		return -EINVAL;
	}

	memcpy(&msg_buffer[index], data, data_size);
	index += data_size;

	crc = crc16_itu_t(0xffff, &msg_buffer[0], index);
	msg_buffer[index++] = crc >> 8;
	msg_buffer[index++] = crc;

	return index;
}

int adpm_set_msg(uint8_t *msg_buffer, int buf_len, uint8_t attr_id, uint16_t attr_index, const void *data, int data_size)
{
	int index = 0;
	uint16_t crc;
	msg_buffer[index++] = SLIP_DEVICE_TYPE_VAL; 		// Device type: 02
	msg_buffer[index++] = SLIP_DEVICE_ADDRESS1_VAL; 	// Device address (0): 00
	msg_buffer[index++] = SLIP_DEVICE_ADDRESS2_VAL; 	// Device address (1): 01
	msg_buffer[index++] = SLIP_LAYER_ADP; 				// Layer: 03 (ADP)

	msg_buffer[index++] = SLIP_PRIMITIVE_ADPM_SET_REQUEST; 	// Primitive type: ADPM-SET.request
	msg_buffer[index++] = SLIP_PRIMITIVE_STRUCT; 	// XDR tag: structure
	msg_buffer[index++] = SLIP_PRIMITIVE_ADPM_SET_REQUEST_SIZE; 	// Number of parameters: 3

	msg_buffer[index++] = SLIP_PRIMITIVE_UINT8; 	// XDR tag: Uint8
	msg_buffer[index++] = attr_id; 	// Attribute ID

	msg_buffer[index++] = SLIP_PRIMITIVE_UINT16; 	// XDR tag: Uint16
	msg_buffer[index++] = attr_index >> 8; // first byte
	msg_buffer[index++] = attr_index & 0xff; // second byte

	msg_buffer[index++] = SLIP_PRIMITIVE_STRING; 	// XDR tag: String
	if (data_size < 128) {
		msg_buffer[index++] = data_size;				
	} else if (data_size <= UINT16_MAX) {
		msg_buffer[index++] = SLIP_PRIMITIVE_STRING_MORE_128BYTES_2BYTE_KEY;
		msg_buffer[index++] = data_size >> 8;
		msg_buffer[index++] = data_size;
	} else {
		return -EINVAL;
	}

	if (index + data_size > buf_len) {
		return -EINVAL;
	}

	memcpy(&msg_buffer[index], data, data_size);
	index += data_size;

	crc = crc16_itu_t(0xffff, &msg_buffer[0], index);
	msg_buffer[index++] = crc >> 8;
	msg_buffer[index++] = crc;

	return index;
}

int set_network_start(uint8_t *msg_buffer, const uint8_t panid[2])
{
	int index = 0;
	msg_buffer[index++] = SLIP_DEVICE_TYPE_VAL; 		// Device type: 02
	msg_buffer[index++] = SLIP_DEVICE_ADDRESS1_VAL; 	// Device address (0): 00
	msg_buffer[index++] = SLIP_DEVICE_ADDRESS2_VAL; 	// Device address (1): 01
	msg_buffer[index++] = SLIP_LAYER_ADP; 				// Layer: 03 (ADP)

	msg_buffer[index++] = SLIP_PRIMITIVE_ADPM_NETWORK_START_REQUEST;
	msg_buffer[index++] = SLIP_PRIMITIVE_STRUCT;
	msg_buffer[index++] = 0x01; // only one parameter
	msg_buffer[index++] = SLIP_PRIMITIVE_UINT16;
	
	// write pan-id
	msg_buffer[index++] = panid[0];
	msg_buffer[index++] = panid[1];

	// generate CRC
	uint16_t crc = crc16_itu_t(0xffff, &msg_buffer[0], index);
	msg_buffer[index++] = crc >> 8;
	msg_buffer[index++] = crc;

	return index;
}

int network_discovery(uint8_t *msg_buffer, int discovery_time)
{
	int index = 0;
	msg_buffer[index++] = SLIP_DEVICE_TYPE_VAL; 		// Device type: 02
	msg_buffer[index++] = SLIP_DEVICE_ADDRESS1_VAL; 	// Device address (0): 00
	msg_buffer[index++] = SLIP_DEVICE_ADDRESS2_VAL; 	// Device address (1): 01
	msg_buffer[index++] = SLIP_LAYER_ADP; 				// Layer: 03 (ADP)
														//
	msg_buffer[index++] = SLIP_PRIMITIVE_ADPM_DISCOVERY_REQUEST;
	msg_buffer[index++] = SLIP_PRIMITIVE_STRUCT;
	msg_buffer[index++] = 0x01; // only one parameter
	msg_buffer[index++] = SLIP_PRIMITIVE_UINT8;
	msg_buffer[index++] = discovery_time;

	// generate CRC
	uint16_t crc = crc16_itu_t(0xffff, &msg_buffer[0], index);
	msg_buffer[index++] = crc >> 8;
	msg_buffer[index++] = crc;

	return index;
}

int slip_input_byte(struct PLCPacket *pkt, unsigned char c)
{
	if (pkt->state & STATE_ESC) {
		/* clear the state */
		pkt->state &= ~STATE_ESC;
		switch (c) {
		case CHAR_ESC_END:
			c = CHAR_END;
			break;
		case CHAR_ESC_ESC:
			c = CHAR_ESC;
			break;
		}
		/* just append the byte if there's a protocol violation */
		if (pkt->len < MAX_PLC_PACKET) {
			pkt->buf[pkt->len++] = c;
		}
		return 0;
	}

	if (c == CHAR_END) {
		if (pkt->len > 0) {
			LOG_DBG("Found packet of length %d", pkt->len);
			return 1;
		} else {
			return 0;
		}
	}

	if (c == CHAR_ESC) {
		pkt->state |= STATE_ESC;
		return 0;
	}

	/* if we fill up the full wait for the end character to flush */
	if (pkt->len < MAX_PLC_PACKET) {
		pkt->buf[pkt->len++] = c;
	}
	return 0;
}
