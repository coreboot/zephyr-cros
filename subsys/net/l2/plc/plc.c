#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(net_plc, LOG_LEVEL_NONE);

#include <zephyr/net/net_core.h>
#include <zephyr/net/net_l2.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>

#include <zephyr/net/plc.h>

static inline enum net_verdict plc_recv(struct net_if *iface,
					  struct net_pkt *pkt)
{
	struct net_plc_hdr *hdr = NET_PLC_HDR(pkt);
	uint8_t hdr_len 		= sizeof(*hdr);

	net_pkt_lladdr_src(pkt)->addr = hdr->src.addr;
	net_pkt_lladdr_src(pkt)->len = 4U;
	net_pkt_lladdr_src(pkt)->type = NET_LINK_PLC;
	net_pkt_lladdr_dst(pkt)->addr = hdr->dst.addr;
	net_pkt_lladdr_dst(pkt)->len = 4U;
	net_pkt_lladdr_dst(pkt)->type = NET_LINK_PLC;

	net_buf_pull(pkt->frags, hdr_len);

	return NET_CONTINUE;
}

static inline int plc_send(struct net_if *iface, struct net_pkt *pkt)
{
	const struct plc_api *api = net_if_get_device(iface)->api;
	int ret;

	if (!api) {
		return -ENOENT;
	}

	ret = api->send(net_if_get_device(iface), pkt);
	if (!ret) {
		ret = net_pkt_get_len(pkt);
		net_pkt_unref(pkt);
	}

	return ret;
}

static enum net_l2_flags plc_flags(struct net_if *iface)
{
	return 0;
}

NET_L2_INIT(PLC_L2, plc_recv, plc_send, NULL, plc_flags);
