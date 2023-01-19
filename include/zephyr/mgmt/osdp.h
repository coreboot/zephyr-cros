/*
 * Copyright (c) 2020 Siddharth Chandrasekaran <sidcha.dev@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _OSDP_H_
#define _OSDP_H_

#include <zephyr/kernel.h>
#include <stdint.h>

#include <zephyr/sys/slist.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OSDP_CMD_TEXT_MAX_LEN 32
#define OSDP_CMD_KEYSET_KEY_MAX_LEN 32
#define OSDP_CMD_MFG_MAX_DATALEN 64
#define OSDP_EVENT_MAX_DATALEN 64
#define OSDP_IO_STATUS_MAX_LEN 32
#define OSDP_RTMAPER_STATUS_MAX_LEN 8
#define OSDP_CMD_CHLNG_RND_NUM_LEN 8
#define OSDP_CMD_SCRYPT_CRYPT_LEN 16

/**
 * @brief Various PD capability function codes.
 */
enum osdp_pd_cap_function_code_e {
	/**
	 * @brief Dummy.
	 */
	OSDP_PD_CAP_UNUSED,

	/**
	 * @brief This function indicates the ability to monitor the status of a
	 * switch using a two-wire electrical connection between the PD and the
	 * switch. The on/off position of the switch indicates the state of an
	 * external device.
	 *
	 * The PD may simply resolve all circuit states to an open/closed
	 * status, or it may implement supervision of the monitoring circuit.
	 * A supervised circuit is able to indicate circuit fault status in
	 * addition to open/closed status.
	 */
	OSDP_PD_CAP_CONTACT_STATUS_MONITORING,

	/**
	 * @brief This function provides a switched output, typically in the
	 * form of a relay. The Output has two states: active or inactive. The
	 * Control Panel (CP) can directly set the Output's state, or, if the PD
	 * supports timed operations, the CP can specify a time period for the
	 * activation of the Output.
	 */
	OSDP_PD_CAP_OUTPUT_CONTROL,

	/**
	 * @brief This capability indicates the form of the card data is
	 * presented to the Control Panel.
	 */
	OSDP_PD_CAP_CARD_DATA_FORMAT,

	/**
	 * @brief This capability indicates the presence of and type of LEDs.
	 */
	OSDP_PD_CAP_READER_LED_CONTROL,

	/**
	 * @brief This capability indicates the presence of and type of an
	 * Audible Annunciator (buzzer or similar tone generator)
	 */
	OSDP_PD_CAP_READER_AUDIBLE_OUTPUT,

	/**
	 * @brief This capability indicates that the PD supports a text display
	 * emulating character-based display terminals.
	 */
	OSDP_PD_CAP_READER_TEXT_OUTPUT,

	/**
	 * @brief This capability indicates that the type of date and time
	 * awareness or time keeping ability of the PD.
	 */
	OSDP_PD_CAP_TIME_KEEPING,

	/**
	 * @brief All PDs must be able to support the checksum mode. This
	 * capability indicates if the PD is capable of supporting CRC mode.
	 */
	OSDP_PD_CAP_CHECK_CHARACTER_SUPPORT,

	/**
	 * @brief This capability indicates the extent to which the PD supports
	 * communication security (Secure Channel Communication)
	 */
	OSDP_PD_CAP_COMMUNICATION_SECURITY,

	/**
	 * @brief This capability indicates the maximum size single message the
	 * PD can receive.
	 */
	OSDP_PD_CAP_RECEIVE_BUFFERSIZE,

	/**
	 * @brief This capability indicates the maximum size multi-part message
	 * which the PD can handle.
	 */
	OSDP_PD_CAP_LARGEST_COMBINED_MESSAGE_SIZE,

	/**
	 * @brief This capability indicates whether the PD supports the
	 * transparent mode used for communicating directly with a smart card.
	 */
	OSDP_PD_CAP_SMART_CARD_SUPPORT,

	/**
	 * @brief This capability indicates the number of credential reader
	 * devices present. Compliance levels are bit fields to be assigned as
	 * needed.
	 */
	OSDP_PD_CAP_READERS,

	/**
	 * @brief This capability indicates the ability of the reader to handle
	 * biometric input
	 */
	OSDP_PD_CAP_BIOMETRICS,

	/**
	 * @brief This capability indicates if the reader is capable of supporting 
	 * Secure PIN Entry (SPE) 
	 */
	OSDP_PD_CAP_SECURE_PIN_ENTRY_SUPPORT,

	/**
	 * @brief This capability indicates the version of OSDP this PD supports.
	 */
	OSDP_PD_CAP_OSDP_VERSION,

	/**
	 * @brief Capability Sentinel
	 */
	OSDP_PD_CAP_SENTINEL
};

/**
 * @brief PD capability structure. Each PD capability has a 3 byte
 * representation.
 *
 * @param function_code One of enum osdp_pd_cap_function_code_e.
 * @param compliance_level A function_code dependent number that indicates what
 *                         the PD can do with this capability.
 * @param num_items Number of such capability entities in PD.
 */
struct osdp_pd_cap {
	uint8_t function_code;
	uint8_t compliance_level;
	uint8_t num_items;
};

/**
 * @brief PD ID information advertised by the PD.
 *
 * @param version 3-bytes IEEE assigned OUI
 * @param model 1-byte Manufacturer's model number
 * @param vendor_code 1-Byte Manufacturer's version number
 * @param serial_number 4-byte serial number for the PD
 * @param firmware_version 3-byte version (major, minor, build)
 */
struct osdp_pd_id {
	int version;
	int model;
	uint32_t vendor_code;
	uint32_t serial_number;
	uint32_t firmware_version;
};

/**
 * @brief PD status.
 *
 * @param inputs Inputs status 
 * @param outputs Outputs status
 * @param rtampers Connected reader tamper status
 * @param power Power status
 * @param tamper Tamper status
 */
struct osdp_pd_status {
	uint32_t inputs;
	uint32_t outputs;
	uint16_t rtampers;
	uint8_t power;
	uint8_t tamper;
};

struct osdp_channel {
	/**
	 * @brief pointer to a block of memory that will be passed to the
	 * send/receive method. This is optional and can be left empty.
	 */
	void *data;

	/**
	 * @brief pointer to function that copies received bytes into buffer
	 * @param data for use by underlying layers. channel_s::data is passed
	 * @param buf byte array copy incoming data
	 * @param len sizeof `buf`. Can copy utmost `len` bytes into `buf`
	 *
	 * @retval +ve: number of bytes copied on to `bug`. Must be <= `len`
	 * @retval -ve on errors
	 */
	int (*recv)(void *data, uint8_t *buf, int maxlen);

	/**
	 * @brief pointer to function that sends byte array into some channel
	 * @param data for use by underlying layers. channel_s::data is passed
	 * @param buf byte array to be sent
	 * @param len number of bytes in `buf`
	 *
	 * @retval +ve: number of bytes sent. must be <= `len`
	 * @retval -ve on errors
	 */
	int (*send)(void *data, uint8_t *buf, int len);

	/**
	 * @brief pointer to function that drops all bytes in TX/RX fifo
	 * @param data for use by underlying layers. channel_s::data is passed
	 */
	void (*flush)(void *data);
};

/**
 * @brief OSDP CP Configuration.
 *
 * @param connected_readers_num Specifies number of connected readers
 * @param connected_readers_addresses Pointer to array with addresses of connected readers
 */
struct osdp_cp_cfg {
	int connected_readers_num;
	uint8_t *connected_readers_addresses;
};

/**
 * @brief OSDP PD Configuration.
 * @param reader_address Reader address.
 * @param id Pointer to static information that the PD reports to the CP when 
 *        it received a `CMD_ID`. These information must be populated by a PD 
 *        appliication.
 * @param cap This is a pointer to an array of structures containing the PD'
 *        capabilities. Use { -1, 0, 0 } to terminate the array. This is used
 *        only PD mode of operation
 * @param status Pointer to struct with information on reader status
 */
struct osdp_pd_cfg {
	uint8_t reader_address;
	struct osdp_pd_id *id;
	struct osdp_pd_cap *cap;
	struct osdp_pd_status *status;
};

/**
 * @brief OSDP Configuration. This struct is used to configure OSDP on runtime.
 *
 * @param baud_rate Can be one of 9600/19200/38400/115200/230400
 * @param skip_mark_byte Set to ignore mark byte in packet
 * @param cp_cfg Pointer to CP configuration. If not-null it is used to configure 
 *        CP device on runtime. Otherwise the default build time configuration is used.
 * @param pd_cfg Pointer to PD configuration. If not-null it is used to configure 
 *        PD device on runtime. Otherwise the default build time configuration is used. 
 * @param key Pointer to 16 bytes of Secure Channel Base Key for the PD. If
 *        non-null, this is used to set-up the secure channel instead of using
 *        the Master Key (in case of CP).
 */
struct osdp_cfg {
	int baud_rate;
	bool skip_mark_byte;
	struct osdp_cp_cfg *cp_cfg;
	struct osdp_pd_cfg *pd_cfg;
	uint8_t *key;
};

/* ------------------------------- */
/*         OSDP Commands           */
/* ------------------------------- */

/**
 * @brief Command sent from CP to Control digital output of PD.
 *
 * @param output_no 0 = First Output, 1 = Second Output, etc.
 * @param control_code One of the following:
 *        0 - NOP – do not alter this output
 *        1 - set the permanent state to OFF, abort timed operation (if any)
 *        2 - set the permanent state to ON, abort timed operation (if any)
 *        3 - set the permanent state to OFF, allow timed operation to complete
 *        4 - set the permanent state to ON, allow timed operation to complete
 *        5 - set the temporary state to ON, resume perm state on timeout
 *        6 - set the temporary state to OFF, resume permanent state on timeout
 * @param timer_count Time in units of 100 ms
 */
struct osdp_cmd_output {
	uint8_t output_no;
	uint8_t control_code;
	uint16_t timer_count;
};

/**
 * @brief LED Colors as specified in OSDP for the on_color/off_color parameters.
 */
enum osdp_led_color_e {
	OSDP_LED_COLOR_NONE,
	OSDP_LED_COLOR_RED,
	OSDP_LED_COLOR_GREEN,
	OSDP_LED_COLOR_AMBER,
	OSDP_LED_COLOR_BLUE,
    OSDP_LED_COLOR_MAGENTA,
    OSDP_LED_COLOR_CYAN,
    OSDP_LED_COLOR_WHITE,
	OSDP_LED_COLOR_SENTINEL
};

/**
 * @brief LED params sub-structure. Part of LED command. See struct osdp_cmd_led
 *
 * @param control_code One of the following:
 *        Temporary Control Code:
 *           0 - NOP - do not alter this LED's temporary settings
 *           1 - Cancel any temporary operation and display this LED's permanent
 *               state immediately
 *           2 - Set the temporary state as given and start timer immediately
 *        Permanent Control Code:
 *           0 - NOP - do not alter this LED's permanent settings
 *           1 - Set the permanent state as given
 * @param on_count The ON duration of the flash, in units of 100 ms
 * @param off_count The OFF duration of the flash, in units of 100 ms
 * @param on_color Color to set during the ON timer (enum osdp_led_color_e)
 * @param off_color Color to set during the OFF timer (enum osdp_led_color_e)
 * @param timer_count Time in units of 100 ms (only for temporary mode)
 */
struct osdp_cmd_led_params {
	uint8_t control_code;
	uint8_t on_count;
	uint8_t off_count;
	uint8_t on_color;
	uint8_t off_color;
	uint16_t timer_count;
};

/**
 * @brief Sent from CP to PD to control the behaviour of it's on-board LEDs
 *
 * @param reader 0 = First Reader, 1 = Second Reader, etc.
 * @param led_number 0 = first LED, 1 = second LED, etc.
 * @param temporary ephemeral LED status descriptor
 * @param permanent permanent LED status descriptor
 */
struct osdp_cmd_led {
	uint8_t reader;
	uint8_t led_number;
	struct osdp_cmd_led_params temporary;
	struct osdp_cmd_led_params permanent;
};

/**
 * @brief Sent from CP to control the behaviour of a buzzer in the PD.
 *
 * @param reader 0 = First Reader, 1 = Second Reader, etc.
 * @param control_code 0: no tone, 1: off, 2: default tone, 3+ is TBD.
 * @param on_count The ON duration of the flash, in units of 100 ms
 * @param off_count The OFF duration of the flash, in units of 100 ms
 * @param rep_count The number of times to repeat the ON/OFF cycle; 0: forever
 */
struct osdp_cmd_buzzer {
	uint8_t reader;
	uint8_t control_code;
	uint8_t on_count;
	uint8_t off_count;
	uint8_t rep_count;
};

/**
 * @brief Command to manipulate any display units that the PD supports.
 *
 * @param reader 0 = First Reader, 1 = Second Reader, etc.
 * @param control_code One of the following:
 *        1 - permanent text, no wrap
 *        2 - permanent text, with wrap
 *        3 - temp text, no wrap
 *        4 - temp text, with wrap
 * @param temp_time duration to display temporary text, in seconds
 * @param offset_row row to display the first character (1 indexed)
 * @param offset_col column to display the first character (1 indexed)
 * @param length Number of characters in the string
 * @param data The string to display
 */
struct osdp_cmd_text {
	uint8_t reader;
	uint8_t control_code;
	uint8_t temp_time;
	uint8_t offset_row;
	uint8_t offset_col;
	uint8_t length;
	uint8_t data[OSDP_CMD_TEXT_MAX_LEN];
};

/**
 * @brief Sent in response to a COMSET command. Set communication parameters to
 * PD. Must be stored in PD non-volatile memory.
 *
 * @param address Unit ID to which this PD will respond after the change takes
 *        effect.
 * @param baud_rate baud rate value 9600/19200/38400/115200/230400
 */
struct osdp_cmd_comset {
	uint8_t address;
	uint32_t baud_rate;
};

/**
 * @brief This command transfers an encryption key from the CP to a PD.
 *
 * @param type Type of keys:
 *   - 0x01 – Secure Channel Base Key
 * @param length Number of bytes of key data - (Key Length in bits + 7) / 8
 * @param data Key data
 */
struct osdp_cmd_keyset {
	uint8_t type;
	uint8_t length;
	uint8_t data[OSDP_CMD_KEYSET_KEY_MAX_LEN];
};

/**
 * @brief This command delivers a random challenge to the PD and it requests 
 * the PD to initialize for the secure session.
 *
 * @param key_type 		Used key type
 * @param random_number Random number genrated by the ACU (RND.A)* 
 */
struct osdp_cmd_chlng {
	uint8_t key_type;
	uint8_t random_number[OSDP_CMD_CHLNG_RND_NUM_LEN];
};

/**
 * @brief This command transfers a block of data used for encryption 
 * synchronization 
 *
 * @param crypotogram 16-byte Server Cryptogram array
 */
struct osdp_cmd_scrypt {
	uint8_t crypotogram[OSDP_CMD_SCRYPT_CRYPT_LEN];
};

/**
 * @brief Manufacturer Specific Commands
 *
 * @param vendor_code 3-byte IEEE assigned OUI. Most Significat 8-bits are
 *        unused.
 * @param command 1-byte manufacturer defined osdp command
 * @param length Length of command data (optional)
 * @param data Command data (optional)
 */
struct osdp_cmd_mfg {
	uint32_t vendor_code;
	uint8_t command;
	uint8_t length;
	uint8_t data[OSDP_CMD_MFG_MAX_DATALEN];
};

/**
 * @brief ACU receive size
 *
 * @param max_rx_size ACU max receiver buffer size
 */
struct osdp_cmd_acurxsize {
	uint16_t max_rx_size;
};

/**
 * @brief OSDP application exposed commands
 */
enum osdp_cmd_e {
	OSDP_CMD_OUTPUT = 1,
	OSDP_CMD_LED,
	OSDP_CMD_BUZZER,
	OSDP_CMD_TEXT,
	OSDP_CMD_CHLNG,
	OSDP_CMD_SCRYPT,
	OSDP_CMD_KEYSET,
	OSDP_CMD_COMSET,
	OSDP_CMD_MFG,
	OSDP_CMD_ACURXSIZE,
	OSDP_CMD_ABORT,
	OSDP_CMD_SENTINEL
};

/**
 * @brief OSDP Command Structure. This is a wrapper for all individual OSDP
 * commands.
 *
 * @param id used to select specific commands in union. Type: enum osdp_cmd_e
 * @param led LED command structure
 * @param buzzer buzzer command structure
 * @param text text command structure
 * @param output output command structure
 * @param comset comset command structure
 * @param keyset keyset command structure
 * @param chlng chlng command structure
 * @param scrypt scrypt command structure

 */
struct osdp_cmd {
	sys_snode_t node;
	enum osdp_cmd_e id;
	union {
		struct osdp_cmd_led led;
		struct osdp_cmd_buzzer buzzer;
		struct osdp_cmd_text text;
		struct osdp_cmd_output output;
		struct osdp_cmd_comset comset;
		struct osdp_cmd_keyset keyset;
		struct osdp_cmd_chlng chlng;
		struct osdp_cmd_scrypt scrypt;
		struct osdp_cmd_mfg mfg;
		struct osdp_cmd_acurxsize acurxsize;
		uint8_t data[0];
	};
};

/* ------------------------------- */
/*          OSDP Events            */
/* ------------------------------- */

/**
 * @brief PD tamper status.
 */
enum osdp_tamper_status_e { 
	TAMPER_STATUS_NORMAL, 
	TAMPER_STATUS_TAMPER, 
	TAMPER_STATUS_SENTINEL 
};

/**
 * @brief PD power status.
 */
enum osdp_power_status_e { 
	POWER_STATUS_NORMAL, 
	POWER_STATUS_FAILURE, 
	POWER_STATUS_SENTINEL
};

/**
 * @brief PD local status report.
 *
 * @param tamper_status Tamper status (enum osdp_tamper_status_e)
 * @param power_status Power status (enum osdp_power_status_e)
 */
struct osdp_event_local_status {
	uint8_t tamper_status;
	uint8_t power_status;
};

/**
 * @brief PD input/output state
 */
enum osdp_io_status_e { 
	IO_STATUS_INACTIVE, 
	IO_STATUS_ACTIVE, 
	IO_STATUS_SENTINEL
};

/**
 * @brief PD inputs/outputs status.
 *
 * @param io_num Number of inputs/outputs
 * @param io_statuses Array with statuses of input/output  (enum osdp_io_status_e)
 */
struct osdp_event_io_status {
	uint8_t io_num;
	uint8_t io_statuses[OSDP_IO_STATUS_MAX_LEN];
};

/**
 * @brief PD tamper staus of connected reader. 
 */
enum osdp_reader_tamper_status_e {
	RTAMPER_STATUS_NORMAL,
	RTAMPER_STATUS_NOT_CONNECTED,
	RTAMPER_STATUS_TAMPER,
	RTAMPER_STATUS_SENTINEL
};
/**
 * @brief PD tamper status of connected readers.
 *
 * @param readers_num Number of connected readers
 * @param rtamper_status Array with statuses of connected reader tamper (enum osdp_reader_tamper_status_e)
 */
struct osdp_event_reader_tamper_status {
	uint8_t readers_num;
	uint8_t rtamper_statuses[OSDP_RTMAPER_STATUS_MAX_LEN];
};
/**
 * @brief Various card formats that a PD can support. This is sent to CP
 * when a PD must report a card read.
 */
enum osdp_event_cardread_format_e {
	OSDP_CARD_FMT_RAW_UNSPECIFIED,
	OSDP_CARD_FMT_RAW_WIEGAND,
	OSDP_CARD_FMT_ASCII,
	OSDP_CARD_FMT_SENTINEL
};

/**
 * @brief OSDP event cardread
 *
 * @param reader_no In context of readers attached to current PD, this number
 *        indicated this number. This is not supported by LibOSDP.
 * @param format Format of the card being read.
 *        see `enum osdp_event_cardread_format_e`
 * @param direction Card read direction of PD 0 - Forward; 1 - Backward
 * @param length Length of card data in bytes or bits depending on `format`
 *        (see note).
 * @param data Card data of `length` bytes or bits bits depending on `format`
 *        (see note).
 *
 * @note When `format` is set to OSDP_CARD_FMT_RAW_UNSPECIFIED or
 * OSDP_CARD_FMT_RAW_WIEGAND, the length is expressed in bits. OTOH, when it is
 * set to OSDP_CARD_FMT_ASCII, the length is in bytes. The number of bytes to
 * read from the `data` field must be interpreted accordingly.
 */
struct osdp_event_cardread {
	int reader_no;
	enum osdp_event_cardread_format_e format;
	int direction;
	int length;
	uint8_t data[OSDP_EVENT_MAX_DATALEN];
};

/**
 * @brief OSDP Event Keypad
 *
 * @param reader_no In context of readers attached to current PD, this number
 *                  indicated this number. This is not supported by LibOSDP.
 * @param length Length of keypress data in bytes
 * @param data keypress data of `length` bytes
 */
struct osdp_event_keypress {
	int reader_no;
	int length;
	uint8_t data[OSDP_EVENT_MAX_DATALEN];
};

/**
 * @brief OSDP Event Manufacturer Specific Command
 *
 * Note: OSDP spec v2.2 makes this structure fixed at 4 bytes. LibOSDP allows
 * for some additional data to be passed in this command using the data and
 * length fields. To be fully compliant with the specification, set the length
 * field to 0.
 *
 * @param vendor_code 3-bytes IEEE assigned OUI of manufacturer
 * @param command 1-byte reply code
 * @param length Length of manufacturer data in bytes (optional)
 * @param data manufacturer data of `length` bytes (optional)
 */
struct osdp_event_mfgrep {
	uint32_t vendor_code;
	uint8_t command;
	uint8_t length;
	uint8_t data[OSDP_EVENT_MAX_DATALEN];
};

/**
 * @brief OSDP PD Events
 */
enum osdp_event_type {
	OSDP_EVENT_CARDREAD,
	OSDP_EVENT_KEYPRESS,
	OSDP_EVENT_MFGREP,
	OSDP_EVENT_ISTAT,
	OSDP_EVENT_RSTAT,
	OSDP_EVENT_OSTAT,
	OSDP_EVENT_LSTAT,
	OSDP_EVENT_SENTINEL
};

/**
 * @brief OSDP Event structure.
 *
 * @param id used to select specific event in union. See: enum osdp_event_type
 * @param keypress keypress event structure
 * @param cardread cardread event structure
 * @param mfgrep mfgrep event structure
 * @param localstatus local status event structure
 * @param iostatus IO status event structure
 * @param rtamperstatus reader tamper status event structure
 */
struct osdp_event {
	sys_snode_t node;
	enum osdp_event_type type;
	union {
		struct osdp_event_keypress keypress;
		struct osdp_event_cardread cardread;
		struct osdp_event_mfgrep mfgrep;
		struct osdp_event_local_status localstatus;
		struct osdp_event_io_status iostatus;
		struct osdp_event_reader_tamper_status rtamperstatus;
	};
};

/**
 * @brief Callback for PD command notifications. After it has been registered
 * with `osdp_pd_set_command_callback`, this method is invoked when the PD
 * receives a command from the CP.
 *
 * @param arg pointer that will was passed to the arg param of
 * `osdp_pd_set_command_callback`.
 * @param cmd pointer to the received command.
 *
 * @retval 0 if LibOSDP must send a `osdp_ACK` response
 * @retval -ve if LibOSDP must send a `osdp_NAK` response
 * @retval +ve and modify the passed `struct osdp_cmd *cmd` if LibOSDP must send
 * a specific response. This is useful for sending manufacturer specific
 * reply ``osdp_MFGREP``.
 */
typedef int (*pd_command_callback_t)(void *arg, struct osdp_cmd *cmd);

/**
 * @brief Callback for CP event notifications. After is has been registered with
 * `osdp_cp_set_event_callback`, this method is invoked when the CP receives an
 * event from the PD.
 *
 * @param arg pointer that will was passed to the arg param of
 * `osdp_cp_set_event_callback`.
 * @param pd PD offset number as in `pd_info_t *`.
 * @param ev pointer to osdp_event struct (filled by libosdp).
 *
 * @retval 0 on handling the event successfully.
 * @retval -ve on errors.
 */
typedef int (*cp_event_callback_t)(void *arg, int pd, struct osdp_event *ev);

/**
 * @brief Callback for for command completion event callbacks. After is has
 * been registered with `osdp_set_command_complete_callback()` this method is
 * invoked after a command has been processed successfully in CP and PD sides.
 *
 * @param id OSDP command ID (Note: this is not `enum osdp_cmd_e`)
 */
typedef void (*osdp_command_complete_callback_t)(int id);

/**
 * @brief API to initialize OSDP module.
 * 
 * @retval 0 on success
 * @retval -1 on failure
 */
int osdp_init(const struct osdp_cfg *info);

/* ------------------------------- */
/*            PD Methods           */
/* ------------------------------- */

#ifdef CONFIG_OSDP_MODE_PD

/**
 * @brief Set callback method for PD command notification. This callback is
 * invoked when the PD receives a command from the CP.
 *
 * @param cb The callback function's pointer
 * @param arg A pointer that will be passed as the first argument of `cb`
 */
void osdp_pd_set_command_callback(pd_command_callback_t cb, void *arg);

/**
 * @brief API to notify PD events to CP. These events are sent to the CP as an
 * alternate response to a POLL command.
 *
 * @param event pointer to event struct. Must be filled by application.
 *
 * @retval 0 on success
 * @retval -1 on failure
 */
int osdp_pd_notify_event(const struct osdp_event *event);

#else /* CONFIG_OSDP_MODE_PD */

/* ------------------------------- */
/*            CP Methods           */
/* ------------------------------- */

/**
 * @brief Generic command enqueue API.
 *
 * @param pd PD offset number as in `pd_info_t *`.
 * @param cmd command pointer. Must be filled by application.
 *
 * @retval 0 on success
 * @retval -1 on failure
 *
 * @note This method only adds the command on to a particular PD's command
 * queue. The command itself can fail due to various reasons.
 */
int osdp_cp_send_command(int pd, struct osdp_cmd *cmd);

/**
 * @brief Set callback method for CP event notification. This callback is
 * invoked when the CP receives an event from the PD.
 *
 * @param cb The callback function's pointer
 * @param arg A pointer that will be passed as the first argument of `cb`
 */
void osdp_cp_set_event_callback(cp_event_callback_t cb, void *arg);

#endif /* CONFIG_OSDP_MODE_PD */

/* ------------------------------- */
/*          Common Methods         */
/* ------------------------------- */

/**
 * @brief Get a bit mask of number of PD that are online currently.
 *
 * @param bitmask pointer to an array of bytes. must be as large as
 *                (num_pds + 7 / 8).
 */
void osdp_get_status_mask(uint8_t *bitmask);

/**
 * @brief Get a bit mask of number of PD that are online and have an active
 * secure channel currently.
 *
 * @param bitmask pointer to an array of bytes. must be as large as
 *                (num_pds + 7 / 8).
 */
void osdp_get_sc_status_mask(uint8_t *bitmask);

/**
 * @brief Set osdp_command_complete_callback_t to subscribe to osdp command or
 * event completion events. This can be used to perform post command actions
 * such as changing the baud rate of the underlying channel after a COMSET
 * command was acknowledged/issued by a peer.
 *
 * @param cb Callback to be invoked when a command is completed.
 */
void osdp_set_command_complete_callback(osdp_command_complete_callback_t cb);

#ifdef __cplusplus
}
#endif

#endif /* _OSDP_H_ */
