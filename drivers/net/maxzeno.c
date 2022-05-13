#include <string.h>

#define DT_DRV_COMPAT   maxim_max79356_plc

#define LOG_MODULE_NAME maxzeno
#define LOG_LEVEL CONFIG_MAXZENO_LOG_LEVEL

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

#include <stdbool.h>
#include <errno.h>
#include <stddef.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/util.h>
#include <zephyr/net/buf.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_core.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/uart_pipe.h>

#include <zephyr/net/plc.h>

#include <zephyr/drivers/gpio.h>

#include "maxzeno_slip.h"

static const struct device *const uart_dev = DEVICE_DT_GET(DT_ALIAS(plc_uart));
static const struct device *const plc_dev = DEVICE_DT_GET(DT_NODELABEL(plc));

static const unsigned char packet_trailer[4]= {0x03,0x01,0x11,0x00};

#define MAXZENO_MTU_SIZE	(CONFIG_MAXZENO_MAX_PAYLOAD + NET_IPV6UDPH_LEN)
#define MAXZENO_MAX_PACKET_UNCODED (MAXZENO_MTU_SIZE + sizeof(mzIPv6Header) + 2 + sizeof(packet_trailer) + 2)

/* In the worst case all the bytes are escape characters +2 for delimitter */
#define MAXZENO_MAX_PACKET_CODED (MAXZENO_MAX_PACKET_UNCODED * 2 + 2)

#define PLC_MSG_QUEUE_SIZE      4
K_MEM_SLAB_DEFINE(plc_slab, sizeof(struct PLCPacket), PLC_MSG_QUEUE_SIZE, 4);

#define RX_BUFFER_SIZE CONFIG_MAXZENO_RX_BUFFER_SIZE
#define RX_BUFFER_NUM CONFIG_MAXZENO_RX_NUM_BUFFERS
#define RX_RB_BUF_LEN RX_BUFFER_SIZE * 2

#define TX_QUEUE_SIZE 4

#ifdef CONFIG_MAXZENO_USE_ASYNC_UART
K_MEM_SLAB_DEFINE(uart_async_rx_slab, RX_BUFFER_SIZE, RX_BUFFER_NUM, 1);
K_KERNEL_STACK_DEFINE(maxzeno_rx_stack, CONFIG_MAXZENO_RX_STACK_SIZE);
static bool uart_rx_disabled_signal;
#endif

K_KERNEL_STACK_DEFINE(init_stack_area, 1024);
static struct k_thread init_thread;

const struct slip_message_response expected_initialization_response[] = {
	{ .layer=SLIP_ADPM, .primitive=SLIP_PRIMITIVE_ADPM_RESET_CONFIRM, .status=SLIP_STATUS_SUCCESS, .attribute_id=0 }, // INIT_CONFIRM_RESET
	{ .layer=SLIP_MAC,  .primitive=SLIP_PRIMITIVE_MLME_SET_CONFIRM, .status=SLIP_STATUS_SUCCESS, .attribute_id=SLIP_MLME_ATTRID_MAC_BAND_SELECT }, // INIT_SET_MACBANDSELECT
	{ .layer=SLIP_ADPM, .primitive=SLIP_PRIMITIVE_ADPM_SET_CONFIRM, .status=SLIP_STATUS_SUCCESS, .attribute_id=SLIP_ADPM_ATTRID_DEVICE_TYPE }, // INIT_SET_SERVER
	{ .layer=SLIP_ADPM, .primitive=SLIP_PRIMITIVE_ADPM_SET_CONFIRM, .status=SLIP_STATUS_SUCCESS, .attribute_id=SLIP_ADPM_ATTRID_PSK }, // INIT_SET_PSK,
	{ .layer=SLIP_MAC, 	.primitive=SLIP_PRIMITIVE_MLME_SET_CONFIRM, .status=SLIP_STATUS_SUCCESS, .attribute_id=SLIP_MLME_ATTRID_MAC_EXTENDED_ADDRESS }, // INIT_SET_EXTSERVER,
	{ .layer=SLIP_MAC, 	.primitive=SLIP_PRIMITIVE_MLME_SET_CONFIRM, .status=SLIP_STATUS_SUCCESS, .attribute_id=SLIP_MLME_ATTRID_MAC_SECURITY_ENABLED }, // INIT_DISABLE_MACSECURITY,
	{ .layer=SLIP_ADPM, .primitive=SLIP_PRIMITIVE_ADPM_SET_CONFIRM, .status=SLIP_STATUS_SUCCESS, .attribute_id=SLIP_ADPM_ATTRID_SECURITY_LEVEL }, // INIT_DISABLE_ADPSECURITY,
	{ .layer=SLIP_MAC,  .primitive=SLIP_PRIMITIVE_MLME_SET_CONFIRM, .status=SLIP_STATUS_SUCCESS, .attribute_id=SLIP_MLME_ATTRID_MAC_TX_ANALOG_GAIN }, // INIT_SET_GAIN,
	{ .layer=SLIP_ADPM, .primitive=SLIP_PRIMITIVE_ADPM_DISCOVERY_CONFIRM, .status=SLIP_STATUS_SUCCESS, .attribute_id=0 }, // INIT_SEND_DISCOVERY,
	{ .layer=SLIP_ADPM, .primitive=SLIP_PRIMITIVE_ADPM_NETWORK_START_CONFIRM, .status=SLIP_STATUS_SUCCESS, .attribute_id=0 }, // INIT_SEND_NETWORKSTART,
	{ .layer=SLIP_MAC,  .primitive=SLIP_PRIMITIVE_MLME_SET_CONFIRM, .status=SLIP_STATUS_SUCCESS, .attribute_id=SLIP_MLME_ATTRID_MAC_SHORT_ADDRESS }, // INIT_SEND_SETSHORTSERVER,
	{ .layer=SLIP_ADPM, .primitive=SLIP_PRIMITIVE_ADPM_SET_CONFIRM, .status=SLIP_STATUS_SUCCESS, .attribute_id=SLIP_ADPM_ATTRID_GROUP_TABLE }, // INIT_SUBSCRIBE_MULTICAST,
	{ .layer=SLIP_ADPM, .primitive=SLIP_PRIMITIVE_ADPM_SET_CONFIRM, .status=SLIP_STATUS_SUCCESS, .attribute_id=SLIP_ADPM_ATTRID_GROUP_TABLE }, // INIT_SUBSCRIBE_MULTICAST,
	{ .layer=SLIP_MAC,  .primitive=SLIP_PRIMITIVE_MLME_SET_CONFIRM, .status=SLIP_STATUS_SUCCESS, .attribute_id=SLIP_MLME_ATTRID_MAC_KEY_TABLE }, // INIT_SEND_SETGMK,
	{ .layer=SLIP_ADPM, .primitive=SLIP_PRIMITIVE_ADPM_SET_CONFIRM, .status=SLIP_STATUS_SUCCESS, .attribute_id=SLIP_ADPM_ATTRID_ACTIVATE_KEY_INDEX }, // INIT_SEND_ACTIVATEGMK,
	{ .layer=SLIP_ADPM, .primitive=SLIP_PRIMITIVE_ADPM_SET_CONFIRM, .status=SLIP_STATUS_SUCCESS, .attribute_id=SLIP_ADPM_ATTRID_BYPASS_AUTH }, // INIT_SEND_MYST,
	{ .layer=SLIP_ADPM, .primitive=SLIP_PRIMITIVE_ADPM_SET_CONFIRM, .status=SLIP_STATUS_SUCCESS, .attribute_id=SLIP_ADPM_ATTRID_PREFIX_TABLE }, // INIT_ADPM_SET_PREFIX,
	{ .layer=SLIP_ADPM, .primitive=SLIP_PRIMITIVE_ADPM_SET_CONFIRM, .status=SLIP_STATUS_SUCCESS, .attribute_id=SLIP_ADPM_ATTRID_MAX_HOPS }, // INIT_ADPM_SET_MAX_HOPS,
	{ .layer=SLIP_MAC,  .primitive=SLIP_PRIMITIVE_MLME_SET_CONFIRM, .status=SLIP_STATUS_SUCCESS, .attribute_id=SLIP_MLME_ATTRID_MAC_HIGH_PRIORITY_WINDOW_SIZE }, // INIT_SET_HIGH_PRIORITY_WINDOW_SIZE,
	{ .layer=SLIP_MAC,  .primitive=SLIP_PRIMITIVE_MLME_SET_CONFIRM, .status=SLIP_STATUS_SUCCESS, .attribute_id=SLIP_MLME_ATTRID_MAC_BROADCAST_MAX_CW_ENABLE }, // INIT_SET_MAC_BROADCAST_MAX_CW_ENABLE,
	{ .layer=SLIP_MAC,  .primitive=SLIP_PRIMITIVE_MLME_SET_CONFIRM, .status=SLIP_STATUS_SUCCESS, .attribute_id=SLIP_MLME_ATTRID_MAC_MIN_CW_ATTEMPTS }, // INIT_SET_MAC_MIN_CW_ATTEMPTS,
	{ .layer=SLIP_MAC,  .primitive=SLIP_PRIMITIVE_MLME_SET_CONFIRM, .status=SLIP_STATUS_SUCCESS, .attribute_id=SLIP_MLME_ATTRID_MAC_MIN_BE }, // INIT_SET_MAC_MIN_CW_ATTEMPTS,
	{ .layer=SLIP_MAC,  .primitive=SLIP_PRIMITIVE_MLME_SET_CONFIRM, .status=SLIP_STATUS_SUCCESS, .attribute_id=SLIP_MLME_ATTRID_MAC_FORCE_MODULATION }, // INIT_MAC_FORCE_MODULATION,
	{ .layer=SLIP_ADPM, .primitive=SLIP_PRIMITIVE_ADPM_SET_CONFIRM, .status=SLIP_STATUS_SUCCESS, .attribute_id=SLIP_ADPM_ATTRID_BROADCAST_LOG_TABLE_ENTRY }, // INIT_ADPM_BROADCAST_LOG_TABLE_ENTRY_TTL,
	{ .layer=SLIP_MAC,  .primitive=SLIP_PRIMITIVE_MLME_SET_CONFIRM, .status=SLIP_STATUS_SUCCESS, .attribute_id=SLIP_MLME_ATTRID_MAC_ENABLE_ROLLBACK } // INIT_MAC_MODULATION_ENABLE_ROLLBACK,
};

const uint8_t prefix_buf_out[]  = { 0x40, 												/* prefix length */
									0x00, 												/* L,A, padding */
									0xff, 0xff, 0xff, 0xff, 							/* valid lifetime */
									0xff, 0xff, 0xff, 0xff, 							/* preferred lifetime */
									0x20, 0x01, 0x0b, 0xd8, 0x00, 0x00, 0x00, 0x00, 	/* 64 bit prefix */
									0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; 	/* remaining 64 bit */

const uint8_t nat_src[]  = 		  { 0x20, 0x01, 0x0b, 0xd8, 0x00, 0x00, 0x00, 0x00,
									0x40, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

#define PACKET_TIMEOUT_MSEC 500
struct transmit_packet_timeout {
		int nsdu_handle;
		const struct device *dev;
		struct k_work_delayable timeout_work;
};

struct maxzeno_data
{
	/* for tracking parsing of input data and formatting output data correctly */
	struct PLCPacket *pkt;

	/* reset GPIO */
	const struct device *reset_gpio_port;

	struct k_sem buffer_indication_semaphore;

#ifdef CONFIG_MAXZENO_USE_ASYNC_UART
	struct k_sem tx_sem;
	struct k_sem rx_sem;
	struct ring_buf rx_rb;
	struct k_thread rx_thread;
#endif
	char __aligned(4) rx_rb_buf[RX_RB_BUF_LEN];

	bool init_done;
	struct net_if *iface;

	uint16_t multicast_group;
	uint16_t broadcast_group;
	uint16_t short_address;

	uint8_t link_address[4];
	struct in6_addr mcast_addr;
	struct in6_addr bcast_addr;
	struct in6_addr addr;

	uint8_t __aligned(4) tx_packet[MAXZENO_MTU_SIZE];
	uint8_t __aligned(4) tx_uncoded[MAXZENO_MAX_PACKET_UNCODED];
	uint8_t __aligned(4) uart_buf[MAXZENO_MAX_PACKET_CODED];

	void(*user_receive_cb)(struct net_pkt *pkt);
	void(*user_error_cb)(enum PLCError);

	uint32_t nsdu_handle_usage;
	struct k_mutex nsdu_handle_mutex;

	struct transmit_packet_timeout packet_timeout[TX_QUEUE_SIZE];
};

struct maxzeno_cfg
{
	uint8_t ext_mac_address[8];
	int ext_mac_address_len;
	uint8_t pan_id[2];
	const struct gpio_dt_spec hw_reset_gpio;
};

#define PRIMITIVE_IN_USE 			(1 << 0)
#define PRIMITIVE_ACTIVE 			(1 << 1)

#define MSG_BUF_SIZE 200
uint8_t msg_buffer[MSG_BUF_SIZE];

struct primitive_ctx
{
	uint8_t status; 							/* see PRIMITIVE_xx flags */
	struct k_sem sem_response;
	struct k_sem sem_ctx; 						/* the ctx could be used from different threads */
	const struct slip_message_response *resp; 	/* expected message response	 */
	int(*send_func)(const struct device*, const uint8_t*, int); 		/* function pointer for sending primitive */
	int(*callback_func)(const uint8_t*, int); 	/* callback function that on the received reply */
	const uint8_t *buf; 						/* pointer to buffer for sending data */
	int len; 									/* length of the data in msg_buf */
	struct k_work work; 						/* work struct used to schedule slip send */
};

struct primitive_ctx prim_ctx;

static void transmit_packet_timeout(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct transmit_packet_timeout *packet_timeout = CONTAINER_OF(dwork, struct transmit_packet_timeout, timeout_work);
	struct maxzeno_data *data = packet_timeout->dev->data;
	int nsdu_handle = packet_timeout->nsdu_handle;

	if (nsdu_handle >= TX_QUEUE_SIZE) {
			LOG_ERR("Invalid nsdu handle");
			return;
	}

	if ((data->nsdu_handle_usage & BIT(nsdu_handle)) == 0) {
			LOG_ERR("nsdu handle 0x%x is unused 0x%x", (uint32_t)BIT(nsdu_handle),
				data->nsdu_handle_usage);
			return;
	}

	LOG_ERR("packet timeout with nsdu handle %d", nsdu_handle);

	k_mutex_lock(&data->nsdu_handle_mutex, K_FOREVER);
	data->nsdu_handle_usage &= ~BIT(nsdu_handle);
	k_mutex_unlock(&data->nsdu_handle_mutex);
}

static int primitive_init(const struct slip_message_response *resp, int(send_func)(const struct device*,
						  const uint8_t*, int), int(callback_func)(const uint8_t*, int))
{
	int ret;
	LOG_DBG("Initializing primitive");
	ret = k_sem_take(&prim_ctx.sem_ctx, K_SECONDS(1));
	if (ret) {
		return ret;
	}
	prim_ctx.callback_func 	= callback_func;
	prim_ctx.send_func 	   	= send_func;
	prim_ctx.resp 		   	= resp;
	prim_ctx.len 		   	= 0;
	prim_ctx.status 	   	= PRIMITIVE_IN_USE;
	return ret;
}

static void primitive_set_buffer(const uint8_t* buf, int len)
{
	prim_ctx.buf = buf;
	prim_ctx.len = len;
}

static void msg_send_work_queue(struct k_work *item)
{
	struct primitive_ctx *ctx = CONTAINER_OF(item, struct primitive_ctx, work);
	if (ctx->send_func) {
		ctx->send_func(plc_dev, ctx->buf, ctx->len);
	}
}

static int send_primitive(k_timeout_t timeout)
{
	int ret;
	if ((prim_ctx.status & PRIMITIVE_IN_USE) == 0) {
		LOG_ERR("Primitive ctx not initialized");
		k_sem_give(&prim_ctx.sem_ctx);
		return -EINVAL;
	}
	prim_ctx.status |= PRIMITIVE_ACTIVE;
	if (prim_ctx.buf && prim_ctx.len > 0 && prim_ctx.send_func) {
		k_work_init(&prim_ctx.work, msg_send_work_queue);
		k_work_submit(&prim_ctx.work);
	}
	ret = k_sem_take(&prim_ctx.sem_response, timeout);
	prim_ctx.buf 	= NULL;
	prim_ctx.len   	= 0;
	prim_ctx.status = 0;
	k_sem_give(&prim_ctx.sem_ctx);
	return ret;
}

/* defined at EOF */
static int reset(const struct device *dev);

static inline int slip_writeb_esc_unchecked(uint8_t *buf, unsigned char c)
{
	switch (c) {
	case CHAR_END:
		buf[0] = CHAR_ESC;
		buf[1] = CHAR_ESC_END;
		return 2;
	case CHAR_ESC:
		buf[0] = CHAR_ESC;
		buf[1] = CHAR_ESC_ESC;
		return 2;
	default:
		buf[0] = c;
		return 1;
	}
}

int send_slip_msg_no_escape(const struct device *dev, const uint8_t *buffer, int size)
{
#ifdef CONFIG_MAXZENO_USE_ASYNC_UART
	int ret;
	struct maxzeno_data *data = dev->data;

	ret = uart_tx(uart_dev, buffer, size, SYS_FOREVER_US);
	if (ret) {
		LOG_ERR("Error sending message %d", ret);
		return ret;
	}

	return k_sem_take(&data->tx_sem, K_FOREVER);
#else
	uart_pipe_send(buffer, size);
	return 0;
#endif
}

int send_slip_msg(const struct device *dev, const uint8_t *buffer, int size)
{
	struct maxzeno_data *data = dev->data;
	volatile int plc_index = 0;

#ifdef CONFIG_MAXZENO_USE_ASYNC_UART
	int ret;

	ret = k_sem_take(&data->tx_sem, K_FOREVER);
	if (ret) {
		return ret;
	}

	data->uart_buf[plc_index++] = CHAR_END;
	for (int i = 0; i < size; i++) {
		plc_index += slip_writeb_esc_unchecked(&data->uart_buf[plc_index], buffer[i]);
	}
	data->uart_buf[plc_index++] = CHAR_END;

	ret = uart_tx(uart_dev, data->uart_buf, plc_index, SYS_FOREVER_US);
	if (ret) {
		k_sem_give(&data->tx_sem);
	}
	return ret;
#else
	data->uart_buf[plc_index++] = CHAR_END;
	for (int i = 0; i < size; i++) {
		plc_index += slip_writeb_esc_unchecked(&data->uart_buf[plc_index], buffer[i]);
	}
	data->uart_buf[plc_index++] = CHAR_END;

	uart_pipe_send(data->uart_buf, plc_index);
	return 0;
#endif
}

static int maxzeno_send(const struct device *dev, struct net_pkt *pkt)
{
	// NOTE: this function is primarily for sending an IPv6 packet
	int length, i, x, ret;
	int nsdu_handle;

	// Ignore packet if maxim zeno not initialized yet
	struct maxzeno_data *data = dev->data;
	struct in6_addr *dst = (struct in6_addr *)&NET_IPV6_HDR(pkt)->dst;
	struct in6_addr *src = (struct in6_addr *)&NET_IPV6_HDR(pkt)->src;

	/* get the interface ipv6 address. I'm not sure if addr can be cached as */
	/* new addresses can be added or removed by user */
	struct net_if_ipv6 *ipv6;
	struct in6_addr *addr = NULL;
	net_if_config_ipv6_get(data->iface, &ipv6);
	for (i = 0; i < NET_IF_MAX_IPV6_ADDR; i++) {
		if (!ipv6->unicast[i].is_used ||
		    ipv6->unicast[i].address.family != AF_INET6) {
			continue;
		}
		addr = &ipv6->unicast[i].address.in6_addr;
	}

	if (!addr) {
		LOG_DBG("Coult not find unicast address");
		/* todo return proper error? */
		return 0;
	}

	k_mutex_lock(&data->nsdu_handle_mutex, K_FOREVER);
	for (nsdu_handle = 0; nsdu_handle < TX_QUEUE_SIZE; nsdu_handle++) {
		if ((BIT(nsdu_handle) & data->nsdu_handle_usage) == 0) {
			data->nsdu_handle_usage |= BIT(nsdu_handle);
			break;
		}
	}
	k_mutex_unlock(&data->nsdu_handle_mutex);
	LOG_INF("Sending packet with nsdu handle %d", nsdu_handle);

	if (nsdu_handle >= TX_QUEUE_SIZE) {
		return -EBUSY;
	}

	data->packet_timeout[nsdu_handle].nsdu_handle = nsdu_handle;
	k_work_reschedule(&data->packet_timeout[nsdu_handle].timeout_work, K_MSEC(PACKET_TIMEOUT_MSEC));

	/* nat the 64 bit source address by the maxzeno address. */
	/* I haven't figured how to get plc to forward packets with source address != plc address */
	memcpy((uint8_t*)src + 8, (uint8_t*)addr + 8, 8);

	/* nat the dst address */
	if (dst->s6_addr[0] == 0xff && dst->s6_addr[1] == 0x08) {
		uint16_t node_id = dst->s6_addr16[7] & htons(0x7fff);
		uint16_t pan_id  = addr->s6_addr16[4];
		memcpy(&dst->s6_addr[0], &prefix_buf_out[10], 8);
		memcpy(&dst->s6_addr[8], &pan_id, 2);
		dst->s6_addr[10] = 0;
		dst->s6_addr[11] = 0xff;
		dst->s6_addr[12] = 0xfe;
		dst->s6_addr[13] = 0x00;
		memcpy(&dst->s6_addr[14], &node_id, 2);
	}

	length = net_pkt_get_len(pkt);
	LOG_DBG("maxzeno_send called, length=%d", length);
	if (length == 0 || length > MAXZENO_MTU_SIZE) {
		LOG_ERR("Invalid packet length %d", length);
		/* todo return proper error? */
		return 0;
	}

	// Copy packet to tx_packet buffer
	net_pkt_cursor_init(pkt);
	net_pkt_read(pkt, data->tx_packet, length);

	// Build txEncoded
	// First 16 bytes copied from template
	for (i = 0; i < 16; i++) {
		data->tx_uncoded[i] = mzIPv6Header[i];
	}

	// Adapt packet length
	data->tx_uncoded[8]  = length >> 8;
	data->tx_uncoded[9]  = length & 0xff;

	data->tx_uncoded[14] = length >> 8;
	data->tx_uncoded[15] = length & 0xff;

	int plc_index = 16;
	for (i = 0; i < 8; i++) {
		data->tx_uncoded[plc_index++] = data->tx_packet[i];
	}

	// Copy IPv6 addresses
	for (i = 0; i < 16; i++) {
		data->tx_uncoded[plc_index++] = src->s6_addr[i];
	}
	for (i = 0; i < 16; i++) {
		data->tx_uncoded[plc_index++] = dst->s6_addr[i];
	}

	// Copy IPv6 payload - ipv6 header is 40 bytes
	for (i = 40; i < length; i++) {
		data->tx_uncoded[plc_index++] = data->tx_packet[i];
	}

	/* cdr uint8_t */
	data->tx_uncoded[plc_index++] = 0x11;
	data->tx_uncoded[plc_index++] = nsdu_handle;

	// Copy Maxim Zeno trailer
	for (i = 0; i < sizeof(packet_trailer); i++) {
		data->tx_uncoded[plc_index++] = packet_trailer[i];
	}

	// Compute Frame Check
	x =  crc16_itu_t(0xffff, data->tx_uncoded, plc_index);
	data->tx_uncoded[plc_index++] = x >> 8; // upper byte
	data->tx_uncoded[plc_index++] = x & 0xff; // lower byte

	ret = send_slip_msg(dev, data->tx_uncoded, plc_index);

	if (ret != 0) {
		LOG_ERR("Failed to send message");
		k_work_cancel_delayable(&data->packet_timeout[nsdu_handle].timeout_work);

		k_mutex_lock(&data->nsdu_handle_mutex, K_FOREVER);
		data->nsdu_handle_usage &= ~BIT(nsdu_handle);
		k_mutex_unlock(&data->nsdu_handle_mutex);
	}
	return ret;
}

struct reset_work {
	const struct device *dev;
	struct k_work work;
};

static void reset_work_queue_func(struct k_work *item)
{
	LOG_WRN("Resetting device");
	struct reset_work *ctx = CONTAINER_OF(item, struct reset_work, work);
	reset(ctx->dev);
}

static void maxzeno_uart_abort(const struct device *uart_dev)
{
#ifdef CONFIG_MAXZENO_USE_ASYNC_UART
	uart_tx_abort(uart_dev);
#endif
}

#define SIMPLE_SET(log_msg, serialize_func, attr_id, attr_indx, dat_ptr, dat_size)		\
	do { 																				\
	LOG_DBG(log_msg); 																	\
	primitive_init(&expected_initialization_response[inc++], send_slip_msg, NULL); 		\
	ret = serialize_func(msg_buffer, sizeof(msg_buffer), attr_id, attr_indx, 			\
						 dat_ptr, dat_size);											\
	primitive_set_buffer(msg_buffer, ret); 												\
	ret = send_primitive(default_timeout); 												\
	if (ret < 0) { 																		\
		maxzeno_uart_abort(uart_dev); 													\
		LOG_ERR(log_msg "-failed"); 													\
		goto init;																		\
	} 																					\
	} while (0)

#define SIMPLE_SET_CONST_BUF(log_msg, buf_send) 										\
	do { 																				\
	LOG_DBG(log_msg); 																	\
	primitive_init(&expected_initialization_response[inc++], send_slip_msg_no_escape, NULL); 	\
	primitive_set_buffer(buf_send, sizeof(buf_send)); 									\
	ret = send_primitive(default_timeout); 												\
	if (ret < 0) { 																		\
		maxzeno_uart_abort(uart_dev); 													\
		LOG_ERR(log_msg "-failed"); 													\
		goto init;																		\
	} 																					\
	} while (0)


static void process_initialization(const struct device *dev)
{
	struct maxzeno_data *data = dev->data;
	const struct maxzeno_cfg *cfg = dev->config;
	struct net_if_mcast_addr *maddr;
	struct net_if_addr *ifaddr;

	uint8_t buf_out;
	uint16_t buf16_out;
	int ret;

	struct reset_work reset_work_queue;
	reset_work_queue.dev = dev;
	int inc = 0;
	const k_timeout_t default_timeout = K_SECONDS(CONFIG_MAXZENO_INIT_TIMEOUT);

init:
	/* send hw and sw reset to initiate sequence */
	inc = 0;

	/* todo: fix k_sem_init is only allowed to be called once  */
	k_sem_init(&data->buffer_indication_semaphore, 0, 1);

	k_work_init(&reset_work_queue.work, reset_work_queue_func);
	k_work_submit(&reset_work_queue.work);
	primitive_init(&expected_initialization_response[inc++], NULL, NULL);
	ret = send_primitive(default_timeout);
	if (ret) {
		maxzeno_uart_abort(uart_dev);
		LOG_ERR("Error resetting device");
		goto init;
	}
	LOG_DBG("Received reset confirm");

	/* we need a separate semaphore to handle buffer indication */
	ret = k_sem_take(&data->buffer_indication_semaphore, default_timeout);
	if (ret) {
		LOG_ERR("No buffer indication received");
		goto init;
	}
	LOG_DBG("Received buffer indication");

	buf_out = 0x00;
	SIMPLE_SET("Mac band", mlme_set_msg, SLIP_MLME_ATTRID_MAC_BAND_SELECT, 0, &buf_out, 1);
	buf_out = 0x01;
	SIMPLE_SET("Set server", adpm_set_msg, SLIP_ADPM_ATTRID_DEVICE_TYPE, 0, &buf_out, 1);

	SIMPLE_SET_CONST_BUF("Send setPsk", cmd_setPsk);

	SIMPLE_SET("Send mac extended addr", mlme_set_msg, SLIP_MLME_ATTRID_MAC_EXTENDED_ADDRESS, 0,
				cfg->ext_mac_address, sizeof(cfg->ext_mac_address));

	buf_out = 0x00;
	SIMPLE_SET("Disable Mac security", mlme_set_msg, SLIP_MLME_ATTRID_MAC_SECURITY_ENABLED, 0, &buf_out, 1);
	SIMPLE_SET("Disable ADP security", adpm_set_msg, SLIP_ADPM_ATTRID_SECURITY_LEVEL, 0, &buf_out, 1);
	buf_out = CONFIG_MAXZENO_TX_GAIN;
	SIMPLE_SET("Set gain", mlme_set_msg, SLIP_MLME_ATTRID_MAC_TX_ANALOG_GAIN, 0, &buf_out, 1);

	LOG_DBG("Network discovery");
	primitive_init(&expected_initialization_response[inc++], send_slip_msg, NULL);
	ret = network_discovery(msg_buffer, 0);
	primitive_set_buffer(msg_buffer, ret);
	ret = send_primitive(default_timeout);
	if (ret < 0) {
		maxzeno_uart_abort(uart_dev);
		LOG_ERR("Network discovery-failed");
		goto init;
	}

	LOG_DBG("Send networkStart");
	primitive_init(&expected_initialization_response[inc++], send_slip_msg, NULL);
	ret = set_network_start(msg_buffer, cfg->pan_id);
	primitive_set_buffer(msg_buffer, ret);
	ret = send_primitive(default_timeout);
	if (ret < 0) {
		maxzeno_uart_abort(uart_dev);
		LOG_ERR("Send network start-failed");
		goto init;
	}

	data->link_address[0] = cfg->pan_id[0];
	data->link_address[1] = cfg->pan_id[1];

	uint16_t short_address_be = sys_cpu_to_be16(data->short_address);
	memcpy(&data->link_address[2], &short_address_be, sizeof(short_address_be));

	SIMPLE_SET("Set addr", mlme_set_msg, SLIP_MLME_ATTRID_MAC_SHORT_ADDRESS, 0, &short_address_be, sizeof(short_address_be));

	net_if_set_link_addr(data->iface, &data->link_address[0], 4, NET_LINK_PLC);

	/* returns true on success */
	ret = !net_if_ipv6_addr_rm(data->iface, &data->addr);
	if (!ret) {
		printk("No ipv6 address to remove\n");
	}

	net_ipv6_addr_create_iid(&data->addr, net_if_get_link_addr(data->iface));
	ifaddr = net_if_ipv6_addr_add(data->iface, &data->addr, NET_ADDR_AUTOCONF, 0);
	if (!ifaddr) {
		LOG_ERR("Failed to add IP address");
		goto init;
	}

	uint16_t multicast_group_be = sys_cpu_to_be16(data->multicast_group);
	SIMPLE_SET("Set group table", adpm_set_msg, SLIP_ADPM_ATTRID_GROUP_TABLE, 0, &multicast_group_be, sizeof(multicast_group_be));

	/* set multicast address */
	UNALIGNED_PUT(htonl(0xff020000), &data->mcast_addr.s6_addr32[0]);
	UNALIGNED_PUT(0, &data->mcast_addr.s6_addr32[1]);
	UNALIGNED_PUT(0, &data->mcast_addr.s6_addr32[2]);
	data->mcast_addr.s6_addr[11] = 0x01;
	data->mcast_addr.s6_addr[12] = 0xff;
	data->mcast_addr.s6_addr[13] = 0x00;
	UNALIGNED_PUT(multicast_group_be, &data->mcast_addr.s6_addr16[7]);

	net_if_ipv6_maddr_rm(data->iface, (struct in6_addr*)data->mcast_addr.s6_addr);
	maddr = net_if_ipv6_maddr_add(data->iface, &data->mcast_addr);
	if (!maddr) {
		LOG_ERR("Failed to add mcast address");
		goto init;
	}
	net_if_ipv6_maddr_join(data->iface, maddr);

	LOG_DBG("Broadcast address 0x%x", data->broadcast_group);
	uint16_t broadcast_group_be = sys_cpu_to_be16(data->broadcast_group);
	SIMPLE_SET("Set group table", adpm_set_msg, SLIP_ADPM_ATTRID_GROUP_TABLE, 1, &broadcast_group_be, sizeof(broadcast_group_be));
	UNALIGNED_PUT(htonl(0xff020000), &data->bcast_addr.s6_addr32[0]);
	UNALIGNED_PUT(0, &data->bcast_addr.s6_addr32[1]);
	UNALIGNED_PUT(0, &data->bcast_addr.s6_addr32[2]);
	data->bcast_addr.s6_addr[11] = 0x01;
	data->bcast_addr.s6_addr[12] = 0xff;
	data->bcast_addr.s6_addr[13] = 0x00;
	UNALIGNED_PUT(broadcast_group_be, &data->bcast_addr.s6_addr16[7]);

	net_if_ipv6_maddr_rm(data->iface, (struct in6_addr*)data->bcast_addr.s6_addr);
	maddr = net_if_ipv6_maddr_add(data->iface, &data->bcast_addr);
	if (!maddr) {
		LOG_ERR("Failed to add broadcast address");
		goto init;
	}
	net_if_ipv6_maddr_join(data->iface, maddr);

	SIMPLE_SET_CONST_BUF("Send setGmk", cmd_setGmk);
	SIMPLE_SET_CONST_BUF("Send activateGmk", cmd_activateGmk);
	SIMPLE_SET_CONST_BUF("Send cmd_myst", cmd_myst);

	SIMPLE_SET("Set prefix", adpm_set_msg, SLIP_ADPM_ATTRID_PREFIX_TABLE, 0, prefix_buf_out, sizeof(prefix_buf_out));

	buf_out = 0x01;
	SIMPLE_SET("Set max hops", adpm_set_msg, SLIP_ADPM_ATTRID_MAX_HOPS, 0, &buf_out, 1);
	SIMPLE_SET("Set high priority window size", mlme_set_msg, SLIP_MLME_ATTRID_MAC_HIGH_PRIORITY_WINDOW_SIZE, 0, &buf_out, 1);
	buf_out = 0x00;
	SIMPLE_SET("Set broadcast CW enable", mlme_set_msg, SLIP_MLME_ATTRID_MAC_BROADCAST_MAX_CW_ENABLE, 0, &buf_out, 1);
	buf_out = 0xff;
	SIMPLE_SET("Set min CW attempts", mlme_set_msg, SLIP_MLME_ATTRID_MAC_MIN_CW_ATTEMPTS, 0, &buf_out, 1);
	buf_out = 0x0;
	SIMPLE_SET("Set macMinBE ", mlme_set_msg, SLIP_MLME_ATTRID_MAC_MIN_BE, 0, &buf_out, 1);

 	buf_out = CONFIG_MAXZENO_FORCE_MODULATION;
	SIMPLE_SET("Force the modulation", mlme_set_msg, SLIP_MLME_ATTRID_MAC_FORCE_MODULATION, 0, &buf_out, 1);

	buf16_out = 0;
	SIMPLE_SET("Set broadcast ttl", adpm_set_msg, SLIP_ADPM_ATTRID_BROADCAST_LOG_TABLE_ENTRY, 0, &buf16_out, 2);

	buf_out = 0x00;
	SIMPLE_SET("Disable modulation rollback", mlme_set_msg, SLIP_MLME_ATTRID_MAC_ENABLE_ROLLBACK, 0, &buf_out, 1);

	net_if_up(data->iface);
}

static int parse_packet(struct PLCPacket *pkt, struct slip_message_response *msg_response)
{
	uint8_t *recv_buf = pkt->buf;

	// make sure to 0 our all data members
	msg_response->primitive = 0;
	msg_response->layer = 0;
	msg_response->status = 0;
	msg_response->attribute_id = 0;
	msg_response->attribute_idx = 0;

	if (pkt->len < SLIP_MAX_HEADER_LEN) {
		return -EINVAL;
	}

	/* validate slip message header */
	if (recv_buf[SLIP_DEVICE_TYPE_BUFFER_IDX] != SLIP_DEVICE_TYPE_VAL ||
		recv_buf[SLIP_DEVICE_ADDRESS1_BUFFER_IDX] != SLIP_DEVICE_ADDRESS1_VAL||
		recv_buf[SLIP_DEVICE_ADDRESS2_BUFFER_IDX] != SLIP_DEVICE_ADDRESS2_VAL) {
		return -EINVAL;
	}

	/* pull out message specific information */
	msg_response->primitive = recv_buf[SLIP_PRIMITIVE_BUFFER_IDX];
	msg_response->layer = (recv_buf[SLIP_LAYER_BUFFER_IDX]==SLIP_LAYER_MAC)? SLIP_MAC : SLIP_ADPM;

	if (recv_buf[SLIP_PRIMITIVE_BUFFER_IDX] & SLIP_PRIMITIVE_CONFIRM) {
		if (pkt->len < 9) {
			return -EINVAL;
		}
		msg_response->status = recv_buf[8]; // TODO: is this the same for all?
		LOG_DBG("Status 0x%x", msg_response->status);
		if (msg_response->status) {
			LOG_ERR("error status %d", msg_response->status);
		}
	}

	if (msg_response->primitive == SLIP_PRIMITIVE_ADPM_SET_CONFIRM ||
		msg_response->primitive == SLIP_PRIMITIVE_MLME_SET_CONFIRM) {
		// pull out attribute id and index
		LOG_DBG("Received adpm/mlme set confirmation %d", pkt->len);
		int buf_idx;
		if (pkt->len < 15) {
			LOG_ERR("Length error");
			return -EINVAL;
		}
		if (msg_response->layer == SLIP_ADPM) {
			buf_idx = 10;
			msg_response->attribute_id = recv_buf[buf_idx]; // for ADPM this is one byte, for MLME this is two bytes
		} else {
			msg_response->attribute_id = (recv_buf[10] << 8) | recv_buf[11];
			buf_idx = 11;
		}

		msg_response->attribute_idx = (recv_buf[buf_idx + 2] << 8) | recv_buf[buf_idx + 3];
		LOG_DBG("Attribute idx 0x%x", msg_response->attribute_idx);
	}

	if (msg_response->layer == SLIP_MAC && msg_response->primitive == SLIP_PRIMITIVE_MLME_GET_CONFIRM) {
		msg_response->attribute_id  = recv_buf[10] << 8;
		msg_response->attribute_id += recv_buf[11];

		msg_response->attribute_idx  = recv_buf[13] << 8;
		msg_response->attribute_idx += recv_buf[14];
	}

	if (msg_response->layer == SLIP_ADPM && msg_response->primitive == SLIP_PRIMITIVE_ADPM_GET_CONFIRM) {
		msg_response->attribute_id  = recv_buf[10];

		msg_response->attribute_idx  = recv_buf[12] << 8;
		msg_response->attribute_idx += recv_buf[13];
	}
	return 0;
}


static void process_plc_packet(struct k_work *item)
{
	int ret;
	struct PLCPacket *plc_pkt = CONTAINER_OF(item, struct PLCPacket, work);
	struct maxzeno_data *data = plc_pkt->zeno_ctx;
	uint8_t *recv_buf = plc_pkt->buf;
	int recv_buf_size = plc_pkt->len;
	struct slip_message_response msg_response;
	uint16_t crc_calc;
	enum PLCError err = PLC_ERROR_NO_ERROR;

	/* validate checksum */
	if (recv_buf_size < 3) {
		LOG_ERR("Invalid size");
		err = PLC_ERROR_RX_INVALID_LEN;
		goto finish;
	}

	crc_calc = crc16_itu_t(0xffff, recv_buf, recv_buf_size);
	if (crc_calc != 0) {
		LOG_ERR("Checksum failed");
		LOG_HEXDUMP_WRN(recv_buf, recv_buf_size, "plc packet");
		err = PLC_ERROR_RX_CRC_ERROR;
		goto finish;
	}

	ret = parse_packet(plc_pkt, &msg_response);
	if (ret < 0) {
		LOG_ERR("Error parsing packet");
		err = PLC_ERROR_RX_PARSE_ERROR;
		goto finish;
	}

	if (msg_response.status == 0xa6) {
		LOG_ERR("UART FCS error");
		err = PLC_ERROR_UART_FCS_ERROR;
	}

	if (msg_response.primitive == SLIP_PRIMITIVE_ADPM_DATA_INDICATION) {
		int off, ret;
		struct in6_addr *dst;
		struct in6_addr *src;
		struct net_pkt *pkt;

		if (recv_buf[11] == 0x82) {
			off = 14;
		} else if (recv_buf[11] == 0x84) {
			off = 16;
		} else {
			off = 12;
		}

		/* + 8 is for link layer overhead */
		pkt = net_pkt_rx_alloc_with_buffer(data->iface, CONFIG_MAXZENO_MAX_PAYLOAD + 8, AF_INET6, IPPROTO_UDP, K_NO_WAIT);
		if (!pkt) {
			LOG_ERR("Failed to allocate packet");
			err = PLC_ERROR_RX_ALLOCATION_ERROR;
			goto finish;
		}
		net_pkt_cursor_init(pkt);

		uint16_t packet_size;
		packet_size  = recv_buf[off + 5];
		packet_size |= recv_buf[off + 4] << 8;
		LOG_DBG("Received packet of size %d", packet_size);

		/* nat the rx address */
		dst = (struct in6_addr *)&recv_buf[off + 8 + 16];
		if (memcmp(&dst->s6_addr[0], &prefix_buf_out[10], 8) == 0) {
			uint16_t node_id = dst->s6_addr16[7];
			memset(&dst->s6_addr[0], 0, 16);
			dst->s6_addr[0] = 0xff;
			dst->s6_addr[1] = 0x08;
			dst->s6_addr[11] = 0x01;
			dst->s6_addr[12] = 0xff;
			dst->s6_addr16[7] = htons(0x8000) | node_id;
		}

		src = (struct in6_addr *)&recv_buf[off + 8];
		memcpy(&src->s6_addr[0], nat_src, 14);

		/* link layer address  - [panid (16 bits), short address (16 bits)]   */
		/* link layer - ethernet / can / plc (l2 layer) */
		/* ipv6 layer */

		/* write the link layer addresses */
		/* link layer address source */
		net_pkt_write_u8(pkt, recv_buf[off + 8 + 8]);
		net_pkt_write_u8(pkt, recv_buf[off + 8 + 8 + 1]);
		net_pkt_write_u8(pkt, recv_buf[off + 8 + 14]);
		net_pkt_write_u8(pkt, recv_buf[off + 8 + 15]);

		/* link layer address dst */
		net_pkt_write_u8(pkt, recv_buf[off + 8 + 16 + 8]);
		net_pkt_write_u8(pkt, recv_buf[off + 8 + 16 + 8 + 1]);
		net_pkt_write_u8(pkt, recv_buf[off + 8 + 16 + 14]);
		net_pkt_write_u8(pkt, recv_buf[off + 8 + 16 + 15]);

		// Copy IPv6 first 8 bytes of header and paylaod
		ret = net_pkt_write(pkt, &recv_buf[off], 8 + 32 + packet_size);
		if (ret) {
			net_pkt_unref(pkt);
			LOG_ERR("Payload write overflow");
			err = PLC_ERROR_RX_ALLOCATION_ERROR;
			goto finish;
		}

		if (data->user_receive_cb) {
			/* bypass networking stack and forward directly to the user */
			net_pkt_cursor_init(pkt);
			net_pkt_set_overwrite(pkt, true);
			/* skip the l2 address */
			net_pkt_skip(pkt, 8);
			data->user_receive_cb(pkt);
		}

		// Send to IP layer
		ret = net_recv_data(data->iface, pkt);
		if (ret < 0) {
			LOG_ERR("Error forwarding to ip stack");
			net_pkt_unref(pkt);
		}
		goto finish;
	}

	if (msg_response.primitive == SLIP_PRIMITIVE_ADPM_DATA_CONFIRM) {
		uint8_t nsdu_handle;
		LOG_DBG("ADPM-DATA.confirm - status=%d", msg_response.status);

		if (recv_buf_size < 11) {
				LOG_ERR("Invalid packet size (%d) in ADPM.DATA-CONFIRM", recv_buf_size);
				goto finish;
		}

		nsdu_handle = recv_buf[10];
		if (nsdu_handle >= TX_QUEUE_SIZE) {
				LOG_ERR("Invalid NSDU handle (%d)", nsdu_handle);
				goto finish;
		}

		LOG_INF("Confirmation for packet %d", nsdu_handle);

		k_work_cancel_delayable(&data->packet_timeout[nsdu_handle].timeout_work);

		k_mutex_lock(&data->nsdu_handle_mutex, K_FOREVER);
		data->nsdu_handle_usage &= ~BIT(nsdu_handle);
		k_mutex_unlock(&data->nsdu_handle_mutex);

		goto finish;
	}

	/* special case for buffer indication */
	if (msg_response.layer == SLIP_ADPM &&
		msg_response.primitive == SLIP_PRIMITIVE_ADPM_BUFFER_INDICATION &&
		msg_response.status == 0x1 &&
		msg_response.attribute_id == 0)
	{
		k_sem_give(&data->buffer_indication_semaphore);
		goto finish;
	}

	if (!(prim_ctx.status == (PRIMITIVE_ACTIVE | PRIMITIVE_IN_USE))) {
		goto finish;
	}

	LOG_DBG("layer %d - %d", msg_response.layer, (prim_ctx.resp)->layer);
	LOG_DBG("primitive %d - %d", msg_response.primitive, (prim_ctx.resp)->primitive);
	LOG_DBG("status %d - %d", msg_response.status, (prim_ctx.resp)->status);
	LOG_DBG("layer %d - %d", msg_response.attribute_id, (prim_ctx.resp)->attribute_id);

	if (msg_response.layer 			== (prim_ctx.resp)->layer 		&&
		msg_response.primitive 		== (prim_ctx.resp)->primitive 	&&
		msg_response.status 		== (prim_ctx.resp)->status 		&&
		msg_response.attribute_id 	== (prim_ctx.resp)->attribute_id)
	{
		prim_ctx.status = 0;
		if (prim_ctx.callback_func) {
			prim_ctx.callback_func(recv_buf, recv_buf_size);
		}
		k_sem_give(&prim_ctx.sem_response);
	}

finish:
	k_mem_slab_free(plc_pkt->slab, (void *)plc_pkt);

	if (err != PLC_ERROR_NO_ERROR && data->user_error_cb) {
		data->user_error_cb(err);
	}

	return;
}

static int recv_cb(struct maxzeno_data *data, uint8_t *buf, int len)
{
	int i;
	if (!data->init_done) {
		return len;
	}

	if (!data->pkt) {
		int ret;
		ret = k_mem_slab_alloc(&plc_slab, (void**)&data->pkt, K_NO_WAIT);
		if (ret) {
			/* TODO: On allocation failure we probably need to handle device reset */
			LOG_ERR("Allocation error");
			return len;
		}
		data->pkt->len 		= 0;
		data->pkt->slab 	= &plc_slab;
		data->pkt->state 	= 0;
		data->pkt->zeno_ctx = data;
	}

	for (i = 0; i < len; i++) {
		if (slip_input_byte(data->pkt, buf[i])) {
			struct PLCPacket *pkt = data->pkt;
			data->pkt = NULL;

			k_work_init(&pkt->work, process_plc_packet);
			k_work_submit(&pkt->work);
			break;
		}
	}

	return i;
}

static int reset(const struct device *dev)
{
	const struct maxzeno_cfg *cfg = dev->config;
	int ret;

	ret = gpio_pin_set_dt(&cfg->hw_reset_gpio, 1);
	if (ret) {
		return ret;
	}
	k_busy_wait(1000);

	ret = gpio_pin_set_dt(&cfg->hw_reset_gpio, 0);
	if (ret) {
		return ret;
	}

	k_busy_wait(1000);
	ret = gpio_pin_set_dt(&cfg->hw_reset_gpio, 1);
	if (ret) {
		return ret;
	}

#ifdef CONFIG_MAXZENO_USE_ASYNC_UART
	struct maxzeno_data *data = dev->data;
	ret = uart_tx(uart_dev, cmd_reset, sizeof(cmd_reset), SYS_FOREVER_US);
	if (ret) {
		LOG_ERR("Error sending message %d", ret);
		return ret;
	}
	return k_sem_take(&data->tx_sem, K_FOREVER);
#else
	uart_pipe_send(cmd_reset, sizeof(cmd_reset));
	return 0;
#endif
}

#ifdef CONFIG_MAXZENO_USE_ASYNC_UART
static void uart_async_callback(const struct device *dev,
			       struct uart_event *evt, void *user_data)
{
	const struct device *dev_zeno = user_data;
	struct maxzeno_data *data = dev_zeno->data;
	uint32_t written;
	uint8_t *buf;
	int ret;

	switch (evt->type) {
	case UART_TX_DONE:
		k_sem_give(&data->tx_sem);
		break;
	case UART_RX_RDY:
		written = ring_buf_put(&data->rx_rb,
				       evt->data.rx.buf + evt->data.rx.offset,
				       evt->data.rx.len);
		if (written != evt->data.rx.len) {
			LOG_WRN("Received bytes dropped from ring buf");
		}
		k_sem_give(&data->rx_sem);
		break;
	case UART_TX_ABORTED:
		k_sem_give(&data->tx_sem);
		break;
	case UART_RX_BUF_REQUEST:
		ret = k_mem_slab_alloc(&uart_async_rx_slab, (void **)&buf, K_NO_WAIT);
		if (ret < 0) {
			LOG_ERR("RX buffer starvation");
			break;
		}
		uart_rx_buf_rsp(dev, buf, RX_BUFFER_SIZE);
		break;
	case UART_RX_BUF_RELEASED:
		k_mem_slab_free(&uart_async_rx_slab, (void *)evt->data.rx_buf.buf);
		break;
	case UART_RX_DISABLED:
		uart_rx_disabled_signal = true;
		k_sem_give(&data->rx_sem);
		break;
	default:
		break;
	}
}

static void maxzeno_rx(struct maxzeno_data *data)
{
	uint8_t *buf;
	while (1) {
		k_sem_take(&data->rx_sem, K_FOREVER);

		if (uart_rx_disabled_signal) {
			int ret;
			uint8_t *buf;

			ret = k_mem_slab_alloc(&uart_async_rx_slab, (void **)&buf, K_NO_WAIT);
			if (ret) {
				LOG_ERR("Error allocating PLC UART buf");
				k_oops();
			}

			ret = uart_rx_enable(uart_dev, buf, RX_BUFFER_SIZE, 100);
			if (ret) {
				LOG_ERR("Error enabled PLC UART");
				k_oops();
			}

			uart_rx_disabled_signal = false;
		}

		while (1) {
			int read;
			int len = ring_buf_get_claim(&data->rx_rb, &buf, RX_RB_BUF_LEN);

			if (len == 0) {
				break;
			}
			read = recv_cb(data, buf, len);
			ring_buf_get_finish(&data->rx_rb, read);
		}
	}
}

#else

static uint8_t *uart_pipe_handler(uint8_t *buf, size_t *off)
{
	struct maxzeno_data *data = CONTAINER_OF(buf, struct maxzeno_data, rx_rb_buf);

	if (!data->init_done) {
		*off = 0;
		return buf;
	}

	while (*off > 0) {
		*off -= recv_cb(data, buf, *off);
	}

	return buf;
}

#endif

static int slip_init(const struct device *dev)
{
	struct maxzeno_data *data = dev->data;
	const struct maxzeno_cfg *cfg = dev->config;

	if (!device_is_ready(cfg->hw_reset_gpio.port)) {
		LOG_DBG("Failed to configure pin\n");
		return -1;
	}

	if (gpio_pin_configure_dt(&cfg->hw_reset_gpio, GPIO_OUTPUT) < 0) {
		LOG_DBG("Failed to configure pin\n");
		return -1;
	}

	for (int i = 0; i < TX_QUEUE_SIZE; i++) {
			k_work_init_delayable(&data->packet_timeout[i].timeout_work, transmit_packet_timeout);
			data->packet_timeout[i].dev = dev;
	}

#ifdef CONFIG_MAXZENO_USE_ASYNC_UART
	int ret;
	uint8_t *buf;

	ring_buf_init(&data->rx_rb, RX_RB_BUF_LEN, data->rx_rb_buf);
	k_sem_init(&data->tx_sem, 1, 1);
	k_sem_init(&data->rx_sem, 0, 1);

	k_thread_create(&data->rx_thread, maxzeno_rx_stack, K_KERNEL_STACK_SIZEOF(maxzeno_rx_stack),
			       (k_thread_entry_t)maxzeno_rx, data, NULL, NULL,
				    K_PRIO_COOP(CONFIG_MAXZENO_RX_THREAD_PRIORITY), 0, K_NO_WAIT);

	uart_callback_set(uart_dev, uart_async_callback, (void*)dev);
	ret = k_mem_slab_alloc(&uart_async_rx_slab, (void **)&buf, K_NO_WAIT);
	if (ret) {
		return ret;
	}
	return uart_rx_enable(uart_dev, buf, RX_BUFFER_SIZE, 100);
#else
	uart_pipe_register(data->rx_rb_buf, sizeof(data->rx_rb_buf), uart_pipe_handler);
	return 0;
#endif
}

static int set_node_address(const struct device *dev, uint16_t node_address)
{
	struct maxzeno_data *data = dev->data;
	const struct maxzeno_cfg *cfg = dev->config;
	struct net_if_addr *ifaddr;
	const k_timeout_t default_timeout = K_SECONDS(CONFIG_MAXZENO_INIT_TIMEOUT);
	int ret;
	const uint16_t attr_id = SLIP_MLME_ATTRID_MAC_SHORT_ADDRESS;

	const struct slip_message_response resp = {
		.layer 			= SLIP_MAC,
		.primitive 		= SLIP_PRIMITIVE_MLME_SET_CONFIRM,
		.attribute_id 	= attr_id,
		.status 		= SLIP_STATUS_SUCCESS,
	};

	printk("Current address 0x%x New address 0x%x\n", data->short_address, node_address);
	data->short_address = node_address;

	uint16_t short_address_be = sys_cpu_to_be16(data->short_address);

	data->link_address[0] = cfg->pan_id[0];
	data->link_address[1] = cfg->pan_id[1];
	memcpy(&data->link_address[2], &short_address_be, sizeof(short_address_be));

	ret = primitive_init(&resp, send_slip_msg, NULL);
	if (ret < 0) {
		printk("Error initializing primitive");
		return ret;
	}
	ret = mlme_set_msg(msg_buffer, sizeof(msg_buffer), attr_id, 0, &short_address_be, sizeof(short_address_be));

	primitive_set_buffer(msg_buffer, ret);
	ret = send_primitive(default_timeout);
	if (ret < 0) {
		maxzeno_uart_abort(uart_dev);
		printk("Error initializing primitive\n");
		return ret;
	}

	printk("Finished setting node address\n");

	net_if_set_link_addr(data->iface, &data->link_address[0], 4, NET_LINK_PLC);

	ret = !net_if_ipv6_addr_rm(data->iface, &data->addr);
	if (!ret) {
		printk("No ipv6 address to remove\n");
	}

	net_ipv6_addr_create_iid(&data->addr, net_if_get_link_addr(data->iface));
	ifaddr = net_if_ipv6_addr_add(data->iface, &data->addr, NET_ADDR_AUTOCONF, 0);
	if (!ifaddr) {
		printk("Failed to add IP address\n");
		ret = -ENOMEM;
	}

	return ret;
}

static int get_queue_space(const struct device *dev, int *queue_space)
{
	struct maxzeno_data *data = dev->data;

	*queue_space = 0;
	for (int i = 0; i < TX_QUEUE_SIZE; i++) {
		if ((data->nsdu_handle_usage & BIT(i)) == 0) {
			(*queue_space)++;
		}
	}

	return 0;
}

static int set_gain(const struct device *dev, uint8_t gain)
{
	int ret;
	struct slip_message_response resp = { 0 };
	resp.layer = SLIP_MAC;
	resp.primitive = SLIP_PRIMITIVE_MLME_SET_CONFIRM;
	resp.attribute_id = SLIP_MLME_ATTRID_MAC_TX_ANALOG_GAIN;
	resp.status = SLIP_STATUS_SUCCESS;

	gain = gain & 0x33;
	primitive_init(&resp, send_slip_msg, NULL);
	ret = mlme_set_msg(msg_buffer, sizeof(msg_buffer),
					   SLIP_MLME_ATTRID_MAC_TX_ANALOG_GAIN, 0,
					   &gain, 1);
	primitive_set_buffer(msg_buffer, ret);
	ret = send_primitive(K_SECONDS(1));
	if (ret < 0) {
		maxzeno_uart_abort(uart_dev);
		LOG_ERR("Set tx analog gain-failed");
	}
	return ret;
}

static int set_modulation(const struct device *dev, uint8_t modulation)
{
	int ret;
	struct slip_message_response resp = { 0 };
	resp.layer = SLIP_MAC;
	resp.primitive = SLIP_PRIMITIVE_MLME_SET_CONFIRM;
	resp.attribute_id = SLIP_MLME_ATTRID_MAC_FORCE_MODULATION;
	resp.status = SLIP_STATUS_SUCCESS;

	modulation = modulation & 0x33;
	primitive_init(&resp, send_slip_msg, NULL);
	ret = mlme_set_msg(msg_buffer, sizeof(msg_buffer),
					   SLIP_MLME_ATTRID_MAC_FORCE_MODULATION, 0,
					   &modulation, 1);
	primitive_set_buffer(msg_buffer, ret);
	ret = send_primitive(K_SECONDS(1));
	if (ret < 0) {
		maxzeno_uart_abort(uart_dev);
		LOG_ERR("Set tx analog gain-failed");
	}
	return ret;
}

static int set_multicast_address(const struct device *dev, uint16_t multicast_address)
{
	int ret;
	struct maxzeno_data *data = dev->data;
	const k_timeout_t default_timeout = K_SECONDS(CONFIG_MAXZENO_INIT_TIMEOUT);
	const uint16_t attr_id = SLIP_ADPM_ATTRID_GROUP_TABLE;

	const struct slip_message_response resp = {
		.layer 			= SLIP_ADPM,
		.primitive 		= SLIP_PRIMITIVE_ADPM_SET_CONFIRM,
		.attribute_id 	= attr_id,
		.status 		= SLIP_STATUS_SUCCESS,
	};

	struct net_if_mcast_addr *maddr;
	struct net_if *iface = data->iface;
	uint16_t multicast_group_be;

	/* remove the current mcast address */
	net_if_ipv6_maddr_rm(iface, &data->mcast_addr);

	printk("Current multicast address 0x%x New mcast address 0x%x\n", data->multicast_group, multicast_address);
	data->multicast_group = multicast_address;

	multicast_group_be = sys_cpu_to_be16(data->multicast_group);

	ret = primitive_init(&resp, send_slip_msg, NULL);
	if (ret < 0) {
		printk("Failed to initialize primitive");
		return ret;
	}
	ret = adpm_set_msg(msg_buffer, sizeof(msg_buffer), attr_id, 0, &multicast_group_be, sizeof(multicast_group_be));
	primitive_set_buffer(msg_buffer, ret);
	ret = send_primitive(default_timeout);
	if (ret < 0) {
		maxzeno_uart_abort(uart_dev);
		printk("Failed to send primitive\n");
		return ret;
	}

	UNALIGNED_PUT(htonl(0xff020000), &data->mcast_addr.s6_addr32[0]);
	UNALIGNED_PUT(0, &data->mcast_addr.s6_addr32[1]);
	UNALIGNED_PUT(0, &data->mcast_addr.s6_addr32[2]);
	data->mcast_addr.s6_addr[11] = 0x01;
	data->mcast_addr.s6_addr[12] = 0xff;
	data->mcast_addr.s6_addr[13] = 0x00;
	UNALIGNED_PUT(multicast_group_be, &data->mcast_addr.s6_addr16[7]);

	maddr = net_if_ipv6_maddr_add(iface, &data->mcast_addr);
	if (!maddr) {
		printk("Failed to add mcast address\n");
		return -1;
	}
	net_if_ipv6_maddr_join(data->iface, maddr);
	return 0;
}

static void slip_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct maxzeno_data *data = dev->data;

	/* Do not start the interface until PLC link is up */
	net_if_flag_set(iface, NET_IF_NO_AUTO_START);

	data->iface = iface;

	k_mutex_init(&data->nsdu_handle_mutex);

	/* initialize data structures for initialization thread */
	k_sem_init(&prim_ctx.sem_ctx, 1, 1);
	k_sem_init(&prim_ctx.sem_response, 0, 1);
	k_thread_create(&init_thread,
					init_stack_area,
					K_KERNEL_STACK_SIZEOF(init_stack_area),
					(k_thread_entry_t)process_initialization,
					(void*)dev, NULL, NULL,
					K_PRIO_COOP(CONFIG_MAXZENO_INIT_THREAD_PRIORITY),
					0, K_NO_WAIT);
	data->init_done = true;
}

static struct maxzeno_data maxzeno_data_0 = {
	.multicast_group = DT_PROP(DT_NODELABEL(plc), multicast_group),
	.broadcast_group = DT_PROP(DT_NODELABEL(plc), broadcast_group),
	.short_address = DT_PROP(DT_NODELABEL(plc), short_address),
};

static int add_receive_callback(const struct device *dev, void(*cb)(struct net_pkt *pkt))
{
	struct maxzeno_data *data = dev->data;
	data->user_receive_cb = cb;
	return 0;
}

static int add_error_callback(const struct device *dev, void(*cb)(enum PLCError))
{
	struct maxzeno_data *data = dev->data;
	data->user_error_cb = cb;
	return 0;
}

static bool is_tx_busy(const struct device *dev)
{
	struct maxzeno_data *data = dev->data;

	return data->nsdu_handle_usage > 0;
}

/* const definitions for device */
static const struct maxzeno_cfg maxzeno_cfg_0 =
{
	/* network identifiers */
	.ext_mac_address = DT_PROP(DT_NODELABEL(plc), ext_mac_address),
	.ext_mac_address_len = 8,
	.pan_id = DT_PROP(DT_NODELABEL(plc), pan_id),
	.hw_reset_gpio = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(plc), hw_reset_gpios, 0),
};

static const struct plc_api slip_if_api =
{
	.iface_api.init = slip_iface_init,
	.send = maxzeno_send,
	.add_receive_callback = add_receive_callback,
	.add_error_callback = add_error_callback,
	.set_node_address = set_node_address,
	.set_multicast_address = set_multicast_address,
	.set_gain = set_gain,
	.set_modulation = set_modulation,
	.get_queue_space = get_queue_space,
	.is_tx_busy = is_tx_busy,
};

NET_DEVICE_DT_DEFINE(DT_NODELABEL(plc), slip_init, device_pm_control_nop, &maxzeno_data_0,
				  &maxzeno_cfg_0, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
				  &slip_if_api, PLC_L2, NET_L2_GET_CTX_TYPE(PLC_L2), MAXZENO_MTU_SIZE);
