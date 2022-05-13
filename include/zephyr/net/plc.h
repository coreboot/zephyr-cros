#ifndef ZEPHYR_INCLUDE_NET_PLC_H_
#define ZEPHYR_INCLUDE_NET_PLC_H_

#include <limits.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/net_if.h>

#ifdef __cplusplus
extern "C" {
#endif

enum PLCError {
	PLC_ERROR_NO_ERROR,
	PLC_ERROR_RX_ALLOCATION_ERROR,
	PLC_ERROR_RX_CRC_ERROR,
	PLC_ERROR_RX_INVALID_LEN,
	PLC_ERROR_RX_PARSE_ERROR,
	PLC_ERROR_UART_FCS_ERROR,
};

struct net_plc_addr {
	uint8_t addr[4];
};

struct net_plc_hdr {
	struct net_plc_addr src;
	struct net_plc_addr dst;
} __packed;

#define NET_PLC_HDR(pkt) ((struct net_plc_hdr *)net_pkt_data(pkt))

struct plc_api {
	/**
	 * The net_if_api must be placed in first position in this
	 * struct so that we are compatible with network interface API.
	 */
	struct net_if_api iface_api;

	/** Send a single CAN frame */
	int (*send)(const struct device *dev, struct net_pkt *pkt);
	int (*add_receive_callback)(const struct device *dev, void(*cb)(struct net_pkt *pkt));
	int (*add_error_callback)(const struct device *dev, void(*cb)(enum PLCError));
	int (*set_node_address)(const struct device *dev, uint16_t node_address);
	int (*set_multicast_address)(const struct device *dev, uint16_t node_address);
	int (*set_gain)(const struct device *dev, uint8_t gain);
	int (*set_modulation)(const struct device *dev, uint8_t modulation);
	int (*get_queue_space)(const struct device *dev, int *queue_space);
	bool (*is_tx_busy)(const struct device *dev);
};


#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_NET_PLC_H_ */
