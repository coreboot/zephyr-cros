/*  Media proxy */

/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <sys/check.h>

#include <bluetooth/services/ots.h>
#include <bluetooth/audio/media_proxy.h>
#include <bluetooth/audio/mcc.h>

#include "media_proxy_internal.h"

#define BT_DBG_ENABLED IS_ENABLED(CONFIG_BT_DEBUG_MEDIA_PROXY)
#define LOG_MODULE_NAME media_proxy
#include "common/log.h"


/* Media player */
struct media_player {
	struct media_proxy_pl_calls *calls;
	struct bt_conn *conn;  /* TODO: Treat local and remote player differently */
	bool   registered;
};

/* Synchronous controller - controller using the synchronous API */
struct scontroller {
	struct media_proxy_sctrl_cbs *cbs;
};

/* Aynchronous controller - controller using the asynchronous API */
struct controller {
	struct media_proxy_ctrl_cbs *cbs;
};
/* Media proxy */
struct mprx {
	/* Controller using the synchronous interface - i.e. a remote controller via
	 * the media control service
	 */
	struct scontroller   sctrlr;

	/* Controller using the callback interface - i.e. upper layer */
	struct controller    ctrlr;

	/* The local media player */
	struct media_player  local_player;

	/* Remote media player - to have a player instance for the user, accessed via MCC */
	struct media_player  remote_player;
};

static struct mprx mprx = { 0 };
#ifdef CONFIG_BT_MCC
static struct bt_mcc_cb mcc_cbs;
#endif /* CONFIG_BT_MCC */

/* Synchronous controller calls ***********************************************/

/* The synchronous controller is the media control service, representing a remote controller.
 * It only gets access to the local player, not remote players.
 * I.e., the calls from the synchronous controller are always routed to the local_player.
 */

int media_proxy_sctrl_register(struct media_proxy_sctrl_cbs *sctrl_cbs)
{
	mprx.sctrlr.cbs = sctrl_cbs;
	return 0;
};

const char *media_proxy_sctrl_get_player_name(void)
{
	/* TODO: Add check for whether function pointer is non-NULL everywhere */
	return mprx.local_player.calls->get_player_name();
}

#ifdef CONFIG_BT_OTS
uint64_t media_proxy_sctrl_get_icon_id(void)
{
	return mprx.local_player.calls->get_icon_id();
}
#endif /* CONFIG_BT_OTS */

const char *media_proxy_sctrl_get_icon_url(void)
{
	return mprx.local_player.calls->get_icon_url();
}

const char *media_proxy_sctrl_get_track_title(void)
{
	return mprx.local_player.calls->get_track_title();
}

int32_t media_proxy_sctrl_get_track_duration(void)
{
	return mprx.local_player.calls->get_track_duration();
}

int32_t media_proxy_sctrl_get_track_position(void)
{
	return mprx.local_player.calls->get_track_position();
}

void media_proxy_sctrl_set_track_position(int32_t position)
{
	mprx.local_player.calls->set_track_position(position);
}

int8_t media_proxy_sctrl_get_playback_speed(void)
{
	return mprx.local_player.calls->get_playback_speed();
}

void media_proxy_sctrl_set_playback_speed(int8_t speed)
{
	mprx.local_player.calls->set_playback_speed(speed);
}

int8_t media_proxy_sctrl_get_seeking_speed(void)
{
	return mprx.local_player.calls->get_seeking_speed();
}

#ifdef CONFIG_BT_OTS
uint64_t media_proxy_sctrl_get_track_segments_id(void)
{
	return mprx.local_player.calls->get_track_segments_id();
}

uint64_t media_proxy_sctrl_get_current_track_id(void)
{
	return mprx.local_player.calls->get_current_track_id();
}

void media_proxy_sctrl_set_current_track_id(uint64_t id)
{
	mprx.local_player.calls->set_current_track_id(id);
}

uint64_t media_proxy_sctrl_get_next_track_id(void)
{
	return mprx.local_player.calls->get_next_track_id();
}

void media_proxy_sctrl_set_next_track_id(uint64_t id)
{
	mprx.local_player.calls->set_next_track_id(id);
}

uint64_t media_proxy_sctrl_get_parent_group_id(void)
{
	return mprx.local_player.calls->get_parent_group_id();
}

uint64_t media_proxy_sctrl_get_current_group_id(void)
{
	return mprx.local_player.calls->get_current_group_id();
}

void media_proxy_sctrl_set_current_group_id(uint64_t id)
{
	mprx.local_player.calls->set_current_group_id(id);
}
#endif /* CONFIG_BT_OTS */

uint8_t media_proxy_sctrl_get_playing_order(void)
{
	return mprx.local_player.calls->get_playing_order();
}

void media_proxy_sctrl_set_playing_order(uint8_t order)
{
	mprx.local_player.calls->set_playing_order(order);
}

uint16_t media_proxy_sctrl_get_playing_orders_supported(void)
{
	return mprx.local_player.calls->get_playing_orders_supported();
}

uint8_t media_proxy_sctrl_get_media_state(void)
{
	return mprx.local_player.calls->get_media_state();
}

void media_proxy_sctrl_send_command(struct mpl_cmd cmd)
{
	mprx.local_player.calls->send_command(cmd);
}

uint32_t media_proxy_sctrl_get_commands_supported(void)
{
	return mprx.local_player.calls->get_commands_supported();
}

#ifdef CONFIG_BT_OTS
void media_proxy_sctrl_send_search(struct mpl_search search)
{
	mprx.local_player.calls->send_search(search);
}

uint64_t media_proxy_sctrl_get_search_results_id(void)
{
	return mprx.local_player.calls->get_search_results_id();
}
void media_proxy_sctrl_search_results_id_cb(uint64_t id);
#endif /* CONFIG_BT_OTS */

uint8_t media_proxy_sctrl_get_content_ctrl_id(void)
{
	return mprx.local_player.calls->get_content_ctrl_id();
}


/* Media control client callbacks *********************************************/

#ifdef CONFIG_BT_MCC
static void mcc_discover_mcs_cb(struct bt_conn *conn, int err)
{
	if (err) {
		BT_ERR("Discovery failed (%d)", err);
	}

	BT_DBG("Disovered player");

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->discover_player) {
		mprx.ctrlr.cbs->discover_player(&mprx.remote_player, err);
	} else {
		BT_DBG("No callback");
	}
}

static void mcc_read_player_name_cb(struct bt_conn *conn, int err, const char *name)
{
	/* Debug statements for at least a couple of the callbacks, to show flow */
	BT_DBG("MCC player name callback");

	if (err) {
		BT_ERR("Player name failed");
	}

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->player_name_recv) {
		mprx.ctrlr.cbs->player_name_recv(&mprx.remote_player, err, name);
	} else {
		BT_DBG("No callback");
	}
}

#ifdef CONFIG_BT_MCC_OTS
static void mcc_read_icon_obj_id_cb(struct bt_conn *conn, int err, uint64_t id)
{
	BT_DBG("Icon Object ID callback");

	if (err) {
		BT_ERR("Icon Object ID read failed (%d)", err);
	}

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->icon_id_recv) {
		mprx.ctrlr.cbs->icon_id_recv(&mprx.remote_player, err, id);
	} else {
		BT_DBG("No callback");
	}
}
#endif /* CONFIG_BT_MCC_OTS */

static void mcc_read_icon_url_cb(struct bt_conn *conn, int err, const char *url)
{
	if (err) {
		BT_ERR("Icon URL read failed (%d)", err);
	}

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->icon_url_recv) {
		mprx.ctrlr.cbs->icon_url_recv(&mprx.remote_player, err, url);
	} else {
		BT_DBG("No callback");
	}
}

static void mcc_track_changed_ntf_cb(struct bt_conn *conn, int err)
{
	if (err) {
		BT_ERR("Track change notification failed (%d)", err);
		return;
	}

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->track_changed_recv) {
		mprx.ctrlr.cbs->track_changed_recv(&mprx.remote_player, err);
	} else {
		BT_DBG("No callback");
	}
}

static void mcc_read_track_title_cb(struct bt_conn *conn, int err, const char *title)
{
	if (err) {
		BT_ERR("Track title read failed (%d)", err);
	}

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->track_title_recv) {
		mprx.ctrlr.cbs->track_title_recv(&mprx.remote_player, err, title);
	} else {
		BT_DBG("No callback");
	}
}

static void mcc_read_track_duration_cb(struct bt_conn *conn, int err, int32_t dur)
{
	if (err) {
		BT_ERR("Track duration read failed (%d)", err);
	}

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->track_duration_recv) {
		mprx.ctrlr.cbs->track_duration_recv(&mprx.remote_player, err, dur);
	} else {
		BT_DBG("No callback");
	}
}

static void mcc_read_track_position_cb(struct bt_conn *conn, int err, int32_t pos)
{
	if (err) {
		BT_ERR("Track position read failed (%d)", err);
	}

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->track_position_recv) {
		mprx.ctrlr.cbs->track_position_recv(&mprx.remote_player, err, pos);
	} else {
		BT_DBG("No callback");
	}
}

static void mcc_set_track_position_cb(struct bt_conn *conn, int err, int32_t pos)
{
	if (err) {
		BT_ERR("Track Position set failed (%d)", err);
	}

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->track_position_write) {
		mprx.ctrlr.cbs->track_position_write(&mprx.remote_player, err, pos);
	} else {
		BT_DBG("No callback");
	}
}

static void mcc_read_playback_speed_cb(struct bt_conn *conn, int err, int8_t speed)
{
	if (err) {
		BT_ERR("Playback speed read failed (%d)", err);
	}

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->playback_speed_recv) {
		mprx.ctrlr.cbs->playback_speed_recv(&mprx.remote_player, err, speed);
	} else {
		BT_DBG("No callback");
	}
}

static void mcc_set_playback_speed_cb(struct bt_conn *conn, int err, int8_t speed)
{
	if (err) {
		BT_ERR("Playback speed set failed (%d)", err);
	}

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->playback_speed_write) {
		mprx.ctrlr.cbs->playback_speed_write(&mprx.remote_player, err, speed);
	} else {
		BT_DBG("No callback");
	}
}

static void mcc_read_seeking_speed_cb(struct bt_conn *conn, int err, int8_t speed)
{
	if (err) {
		BT_ERR("Seeking speed read failed (%d)", err);
	}

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->seeking_speed_recv) {
		mprx.ctrlr.cbs->seeking_speed_recv(&mprx.remote_player, err, speed);
	} else {
		BT_DBG("No callback");
	}
}

#ifdef CONFIG_BT_MCC_OTS
static void mcc_read_segments_obj_id_cb(struct bt_conn *conn, int err, uint64_t id)
{
	if (err) {
		BT_ERR("Track Segments Object ID read failed (%d)", err);
	}

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->track_segments_id_recv) {
		mprx.ctrlr.cbs->track_segments_id_recv(&mprx.remote_player, err, id);
	} else {
		BT_DBG("No callback");
	}
}

static void mcc_read_current_track_obj_id_cb(struct bt_conn *conn, int err, uint64_t id)
{
	if (err) {
		BT_ERR("Current Track Object ID read failed (%d)", err);
	}

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->current_track_id_recv) {
		mprx.ctrlr.cbs->current_track_id_recv(&mprx.remote_player, err, id);
	} else {
		BT_DBG("No callback");
	}
}

/* TODO: current track set callback - must be added to MCC first */

static void mcc_read_next_track_obj_id_cb(struct bt_conn *conn, int err, uint64_t id)
{
	if (err) {
		BT_ERR("Next Track Object ID read failed (%d)", err);
	}

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->next_track_id_recv) {
		mprx.ctrlr.cbs->next_track_id_recv(&mprx.remote_player, err, id);
	} else {
		BT_DBG("No callback");
	}
}

/* TODO: next track set callback - must be added to MCC first */

static void mcc_read_parent_group_obj_id_cb(struct bt_conn *conn, int err, uint64_t id)
{
	if (err) {
		BT_ERR("Parent Group Object ID read failed (%d)", err);
	}

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->parent_group_id_recv) {
		mprx.ctrlr.cbs->parent_group_id_recv(&mprx.remote_player, err, id);
	} else {
		BT_DBG("No callback");
	}
}

static void mcc_read_current_group_obj_id_cb(struct bt_conn *conn, int err, uint64_t id)
{
	if (err) {
		BT_ERR("Current Group Object ID read failed (%d)", err);
	}

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->current_group_id_recv) {
		mprx.ctrlr.cbs->current_group_id_recv(&mprx.remote_player, err, id);
	} else {
		BT_DBG("No callback");
	}
}

/* TODO: current group set callback - must be added to MCC first */

#endif /* CONFIG_BT_MCC_OTS */

static void mcc_read_playing_order_cb(struct bt_conn *conn, int err, uint8_t order)
{
	if (err) {
		BT_ERR("Playing order read failed (%d)", err);
	}

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->playing_order_recv) {
		mprx.ctrlr.cbs->playing_order_recv(&mprx.remote_player, err, order);
	} else {
		BT_DBG("No callback");
	}
}

static void mcc_set_playing_order_cb(struct bt_conn *conn, int err, uint8_t order)
{
	if (err) {
		BT_ERR("Playing order set failed (%d)", err);
	}

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->playing_order_write) {
		mprx.ctrlr.cbs->playing_order_write(&mprx.remote_player, err, order);
	} else {
		BT_DBG("No callback");
	}
}

static void mcc_read_playing_orders_supported_cb(struct bt_conn *conn, int err, uint16_t orders)
{
	if (err) {
		BT_ERR("Playing orders supported read failed (%d)", err);
	}

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->playing_orders_supported_recv) {
		mprx.ctrlr.cbs->playing_orders_supported_recv(&mprx.remote_player, err, orders);
	} else {
		BT_DBG("No callback");
	}
}

static void mcc_read_media_state_cb(struct bt_conn *conn, int err, uint8_t state)
{
	if (err) {
		BT_ERR("Media State read failed (%d)", err);
	}

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->media_state_recv) {
		mprx.ctrlr.cbs->media_state_recv(&mprx.remote_player, err, state);
	} else {
		BT_DBG("No callback");
	}
}

static void mcc_send_cmd_cb(struct bt_conn *conn, int err, struct mpl_cmd cmd)
{
	if (err) {
		BT_ERR("Command send failed (%d) - opcode: %d, param: %d",
		       err, cmd.opcode, cmd.param);
	}

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->command_send) {
		mprx.ctrlr.cbs->command_send(&mprx.remote_player, err, cmd);
	} else {
		BT_DBG("No callback");
	}
}

static void mcc_cmd_ntf_cb(struct bt_conn *conn, int err,
			   struct mpl_cmd_ntf ntf)
{
	if (err) {
		BT_ERR("Command notification error (%d) - command opcode: %d, result: %d",
		       err, ntf.requested_opcode, ntf.result_code);
	}

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->command_recv) {
		mprx.ctrlr.cbs->command_recv(&mprx.remote_player, err, ntf);
	} else {
		BT_DBG("No callback");
	}
}

static void mcc_read_opcodes_supported_cb(struct bt_conn *conn, int err, uint32_t opcodes)
{
	if (err) {
		BT_ERR("Opcodes supported read failed (%d)", err);
	}

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->commands_supported_recv) {
		mprx.ctrlr.cbs->commands_supported_recv(&mprx.remote_player, err, opcodes);
	} else {
		BT_DBG("No callback");
	}
}

#ifdef CONFIG_BT_MCC_OTS
static void mcc_send_search_cb(struct bt_conn *conn, int err, struct mpl_search search)
{
	if (err) {
		BT_ERR("Search send failed (%d)", err);
	}

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->search_send) {
		mprx.ctrlr.cbs->search_send(&mprx.remote_player, err, search);
	} else {
		BT_DBG("No callback");
	}
}

static void mcc_search_ntf_cb(struct bt_conn *conn, int err, uint8_t result_code)
{
	if (err) {
		BT_ERR("Search notification error (%d), result code: %d",
		       err, result_code);
	}

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->search_recv) {
		mprx.ctrlr.cbs->search_recv(&mprx.remote_player, err, result_code);
	} else {
		BT_DBG("No callback");
	}
}

static void mcc_read_search_results_obj_id_cb(struct bt_conn *conn, int err, uint64_t id)
{
	if (err) {
		BT_ERR("Search Results Object ID read failed (%d)", err);
	}

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->search_results_id_recv) {
		mprx.ctrlr.cbs->search_results_id_recv(&mprx.remote_player, err, id);
	} else {
		BT_DBG("No callback");
	}
}
#endif /* CONFIG_BT_MCC_OTS */

static void mcc_read_content_control_id_cb(struct bt_conn *conn, int err, uint8_t ccid)
{
	if (err) {
		BT_ERR("Content Control ID read failed (%d)", err);
	}

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->content_ctrl_id_recv) {
		mprx.ctrlr.cbs->content_ctrl_id_recv(&mprx.remote_player, err, ccid);
	} else {
		BT_DBG("No callback");
	}
}

#endif /* CONFIG_BT_MCC */


/* Asynchronous controller calls **********************************************/

int media_proxy_ctrl_register(struct media_proxy_ctrl_cbs *ctrl_cbs)
{
	CHECKIF(ctrl_cbs == NULL) {
		BT_DBG("NULL callback pointer");
		return -EINVAL;
	}

	mprx.ctrlr.cbs = ctrl_cbs;

	if (mprx.local_player.registered) {
		if (mprx.ctrlr.cbs->local_player_instance) {
			mprx.ctrlr.cbs->local_player_instance(&mprx.local_player, 0);
		}
	}

	/* TODO: Return error code if too many controllers registered */
	return 0;
};

#ifdef CONFIG_BT_MCC
int media_proxy_ctrl_discover_player(struct bt_conn *conn)
{
	int err;

	CHECKIF(!conn) {
		BT_DBG("NUll conn pointer");
		return -EINVAL;
	}

	/* Initialize MCC */
	mcc_cbs.discover_mcs                  = mcc_discover_mcs_cb;
	mcc_cbs.read_player_name              = mcc_read_player_name_cb;
#ifdef CONFIG_BT_MCC_OTS
	mcc_cbs.read_icon_obj_id              = mcc_read_icon_obj_id_cb;
#endif /* CONFIG_BT_MCC_OTS */
	mcc_cbs.read_icon_url                 = mcc_read_icon_url_cb;
	mcc_cbs.track_changed_ntf             = mcc_track_changed_ntf_cb;
	mcc_cbs.read_track_title              = mcc_read_track_title_cb;
	mcc_cbs.read_track_duration           = mcc_read_track_duration_cb;
	mcc_cbs.read_track_position           = mcc_read_track_position_cb;
	mcc_cbs.set_track_position            = mcc_set_track_position_cb;
	mcc_cbs.read_playback_speed           = mcc_read_playback_speed_cb;
	mcc_cbs.set_playback_speed            = mcc_set_playback_speed_cb;
	mcc_cbs.read_seeking_speed            = mcc_read_seeking_speed_cb;
#ifdef CONFIG_BT_MCC_OTS
	mcc_cbs.read_segments_obj_id          = mcc_read_segments_obj_id_cb;
	mcc_cbs.read_current_track_obj_id     = mcc_read_current_track_obj_id_cb;
	mcc_cbs.read_next_track_obj_id        = mcc_read_next_track_obj_id_cb;
	mcc_cbs.read_parent_group_obj_id      = mcc_read_parent_group_obj_id_cb;
	mcc_cbs.read_current_group_obj_id     = mcc_read_current_group_obj_id_cb;
#endif /* CONFIG_BT_MCC_OTS */
	mcc_cbs.read_playing_order	      = mcc_read_playing_order_cb;
	mcc_cbs.set_playing_order             = mcc_set_playing_order_cb;
	mcc_cbs.read_playing_orders_supported = mcc_read_playing_orders_supported_cb;
	mcc_cbs.read_media_state              = mcc_read_media_state_cb;
	mcc_cbs.send_cmd                      = mcc_send_cmd_cb;
	mcc_cbs.cmd_ntf                       = mcc_cmd_ntf_cb;
	mcc_cbs.read_opcodes_supported        = mcc_read_opcodes_supported_cb;
#ifdef CONFIG_BT_MCC_OTS
	mcc_cbs.send_search                   = mcc_send_search_cb;
	mcc_cbs.search_ntf                    = mcc_search_ntf_cb;
	mcc_cbs.read_search_results_obj_id    = mcc_read_search_results_obj_id_cb;
#endif /* CONFIG_BT_MCC_OTS */
	mcc_cbs.read_content_control_id       = mcc_read_content_control_id_cb;

	err = bt_mcc_init(&mcc_cbs);
	if (err) {
		BT_ERR("Failed to initialize MCC");
		return err;
	}

	/* Start discovery of remote MCS, subscribe to notifications */
	err = bt_mcc_discover_mcs(conn, 1);
	if (err) {
		BT_ERR("Discovery failed");
		return err;
	}

	mprx.remote_player.conn = conn;
	mprx.remote_player.registered = true;  /* TODO: Do MCC init and "registration" at startup */

	return 0;
}
#endif /* CONFIG_BT_MCC	*/

int media_proxy_ctrl_get_player_name(struct media_player *player)
{
	CHECKIF(player == NULL) {
		BT_DBG("player is NULL");
		return -EINVAL;
	}

	if (mprx.local_player.registered && player == &mprx.local_player) {
		BT_DBG("Local player");
		if (mprx.local_player.calls->get_player_name) {
			const char *name = mprx.local_player.calls->get_player_name();

			if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->player_name_recv) {
				mprx.ctrlr.cbs->player_name_recv(&mprx.local_player, 0, name);
			} else {
				BT_DBG("No callback");
			}

			return 0;
		}

		BT_DBG("No call");
		return -EOPNOTSUPP;
	}

	if (IS_ENABLED(CONFIG_BT_MCC) &&
	    mprx.remote_player.registered && player == &mprx.remote_player) {
		BT_DBG("Remote player");
		return bt_mcc_read_player_name(mprx.remote_player.conn);
	}

	return -EOPNOTSUPP;
}

int media_proxy_ctrl_get_icon_id(struct media_player *player)
{
	CHECKIF(player == NULL) {
		BT_DBG("player is NULL");
		return -EINVAL;
	}

	if (mprx.local_player.registered && player == &mprx.local_player) {
		if (mprx.local_player.calls->get_icon_id) {
			uint64_t id = mprx.local_player.calls->get_icon_id();

			if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->icon_id_recv) {
				mprx.ctrlr.cbs->icon_id_recv(&mprx.local_player, 0, id);
			} else {
				BT_DBG("No callback");
			}

			return 0;
		}

		BT_DBG("No call");
		return -EOPNOTSUPP;
	}

	if (IS_ENABLED(CONFIG_BT_MCC_OTS) &&
	    mprx.remote_player.registered && player == &mprx.remote_player) {
		return bt_mcc_read_icon_obj_id(mprx.remote_player.conn);
	}

	return -EOPNOTSUPP;
}

int media_proxy_ctrl_get_icon_url(struct media_player *player)
{
	CHECKIF(player == NULL) {
		BT_DBG("player is NULL");
		return -EINVAL;
	}

	if (mprx.local_player.registered && player == &mprx.local_player) {
		if (mprx.local_player.calls->get_icon_url) {
			const char *url = mprx.local_player.calls->get_icon_url();

			if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->icon_url_recv) {
				mprx.ctrlr.cbs->icon_url_recv(player, 0, url);
			} else {
				BT_DBG("No callback");
			}

			return 0;
		}

		BT_DBG("No call");
		return -EOPNOTSUPP;
	}

	if (IS_ENABLED(CONFIG_BT_MCC) &&
	    mprx.remote_player.registered && player == &mprx.remote_player) {
		return bt_mcc_read_icon_url(mprx.remote_player.conn);
	}

	return -EOPNOTSUPP;
}

int media_proxy_ctrl_get_track_title(struct media_player *player)
{
	CHECKIF(player == NULL) {
		BT_DBG("player is NULL");
		return -EINVAL;
	}

	if (mprx.local_player.registered && player == &mprx.local_player) {
		if (mprx.local_player.calls->get_track_title) {
			const char *title = mprx.local_player.calls->get_track_title();

			if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->track_title_recv) {
				mprx.ctrlr.cbs->track_title_recv(player, 0, title);
			} else {
				BT_DBG("No callback");
			}

			return 0;
		}

		BT_DBG("No call");
		return -EOPNOTSUPP;
	}

	if (IS_ENABLED(CONFIG_BT_MCC) &&
	    mprx.remote_player.registered && player == &mprx.remote_player) {
		return bt_mcc_read_track_title(mprx.remote_player.conn);
	}

	return -EOPNOTSUPP;
}

int media_proxy_ctrl_get_track_duration(struct media_player *player)
{
	CHECKIF(player == NULL) {
		BT_DBG("player is NULL");
		return -EINVAL;
	}

	if (mprx.local_player.registered && player == &mprx.local_player) {
		if (mprx.local_player.calls->get_track_duration) {
			int32_t duration = mprx.local_player.calls->get_track_duration();

			if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->track_duration_recv) {
				mprx.ctrlr.cbs->track_duration_recv(player, 0, duration);
			} else {
				BT_DBG("No callback");
			}

			return 0;
		}

		BT_DBG("No call");
		return -EOPNOTSUPP;
	}
	if (IS_ENABLED(CONFIG_BT_MCC) &&
	    mprx.remote_player.registered && player == &mprx.remote_player) {
		return bt_mcc_read_track_duration(mprx.remote_player.conn);
	}

	return -EOPNOTSUPP;
}

int media_proxy_ctrl_get_track_position(struct media_player *player)
{
	CHECKIF(player == NULL) {
		BT_DBG("player is NULL");
		return -EINVAL;
	}

	if (mprx.local_player.registered && player == &mprx.local_player) {
		if (mprx.local_player.calls->get_track_position) {
			int32_t position = mprx.local_player.calls->get_track_position();

			if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->track_position_recv) {
				mprx.ctrlr.cbs->track_position_recv(player, 0, position);
			} else {
				BT_DBG("No callback");
			}

			return 0;
		}

		BT_DBG("No call");
		return -EOPNOTSUPP;
	}

	if (IS_ENABLED(CONFIG_BT_MCC) &&
	    mprx.remote_player.registered && player == &mprx.remote_player) {
		return bt_mcc_read_track_position(mprx.remote_player.conn);
	}

	return -EOPNOTSUPP;
}

int media_proxy_ctrl_set_track_position(struct media_player *player, int32_t position)
{
	CHECKIF(player == NULL) {
		BT_DBG("player is NULL");
		return -EINVAL;
	}

	if (mprx.local_player.registered && player == &mprx.local_player) {
		if (mprx.local_player.calls->set_track_position) {
			mprx.local_player.calls->set_track_position(position);

			if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->track_position_write) {
				mprx.ctrlr.cbs->track_position_write(player, 0, position);
			} else {
				BT_DBG("No callback");
			}

			return 0;
		}

		BT_DBG("No call");
		return -EOPNOTSUPP;
	}

	if (IS_ENABLED(CONFIG_BT_MCC) &&
	    mprx.remote_player.registered && player == &mprx.remote_player) {
		return bt_mcc_set_track_position(mprx.remote_player.conn, position);
	}

	return -EOPNOTSUPP;
}

int media_proxy_ctrl_get_playback_speed(struct media_player *player)
{
	CHECKIF(player == NULL) {
		BT_DBG("player is NULL");
		return -EINVAL;
	}

	if (mprx.local_player.registered && player == &mprx.local_player) {
		if (mprx.local_player.calls->get_playback_speed) {
			int8_t speed = mprx.local_player.calls->get_playback_speed();

			if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->playback_speed_recv) {
				mprx.ctrlr.cbs->playback_speed_recv(player, 0, speed);
			} else {
				BT_DBG("No callback");
			}

			return 0;
		}

		BT_DBG("No call");
		return -EOPNOTSUPP;
	}

	if (IS_ENABLED(CONFIG_BT_MCC) &&
	    mprx.remote_player.registered && player == &mprx.remote_player) {
		return bt_mcc_read_playback_speed(mprx.remote_player.conn);
	}

	return -EOPNOTSUPP;
}

int media_proxy_ctrl_set_playback_speed(struct media_player *player, int8_t speed)
{
	CHECKIF(player == NULL) {
		BT_DBG("player is NULL");
		return -EINVAL;
	}

	if (mprx.local_player.registered && player == &mprx.local_player) {
		if (mprx.local_player.calls->set_playback_speed) {
			mprx.local_player.calls->set_playback_speed(speed);

			if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->playback_speed_write) {
				mprx.ctrlr.cbs->playback_speed_write(player, 0, speed);
			} else {
				BT_DBG("No callback");
			}

			return 0;
		}

		BT_DBG("No call");
		return -EOPNOTSUPP;
	}

	if (IS_ENABLED(CONFIG_BT_MCC) &&
	    mprx.remote_player.registered && player == &mprx.remote_player) {
		return bt_mcc_set_playback_speed(mprx.remote_player.conn, speed);
	}

	return -EOPNOTSUPP;
}

int media_proxy_ctrl_get_seeking_speed(struct media_player *player)
{
	CHECKIF(player == NULL) {
		BT_DBG("player is NULL");
		return -EINVAL;
	}

	if (mprx.local_player.registered && player == &mprx.local_player) {
		if (mprx.local_player.calls->get_seeking_speed) {
			int8_t speed = mprx.local_player.calls->get_seeking_speed();

			if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->seeking_speed_recv) {
				mprx.ctrlr.cbs->seeking_speed_recv(player, 0, speed);
			} else {
				BT_DBG("No callback");
			}

			return 0;
		}

		BT_DBG("No call");
		return -EOPNOTSUPP;
	}

	if (IS_ENABLED(CONFIG_BT_MCC) &&
	    mprx.remote_player.registered && player == &mprx.remote_player) {
		return bt_mcc_read_seeking_speed(mprx.remote_player.conn);
	}

	return -EOPNOTSUPP;
}

int media_proxy_ctrl_get_track_segments_id(struct media_player *player)
{
	CHECKIF(player == NULL) {
		BT_DBG("player is NULL");
		return -EINVAL;
	}

	if (mprx.local_player.registered && player == &mprx.local_player) {
		if (mprx.local_player.calls->get_track_segments_id) {
			uint64_t id = mprx.local_player.calls->get_track_segments_id();

			if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->track_segments_id_recv) {
				mprx.ctrlr.cbs->track_segments_id_recv(player, 0, id);
			} else {
				BT_DBG("No callback");
			}

			return 0;
		}

		BT_DBG("No call");
		return -EOPNOTSUPP;
	}

	if (IS_ENABLED(CONFIG_BT_MCC_OTS) &&
	    mprx.remote_player.registered && player == &mprx.remote_player) {
		return bt_mcc_read_segments_obj_id(mprx.remote_player.conn);
	}

	return -EOPNOTSUPP;
}

int media_proxy_ctrl_get_current_track_id(struct media_player *player)
{
	CHECKIF(player == NULL) {
		BT_DBG("player is NULL");
		return -EINVAL;
	}

	if (mprx.local_player.registered && player == &mprx.local_player) {
		if (mprx.local_player.calls->get_current_track_id) {
			uint64_t id = mprx.local_player.calls->get_current_track_id();

			if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->current_track_id_recv) {
				mprx.ctrlr.cbs->current_track_id_recv(player, 0, id);
			} else {
				BT_DBG("No callback");
			}

			return 0;
		}

		BT_DBG("No call");
		return -EOPNOTSUPP;
	}

	if (IS_ENABLED(CONFIG_BT_MCC_OTS) &&
	    mprx.remote_player.registered && player == &mprx.remote_player) {
		return bt_mcc_read_current_track_obj_id(mprx.remote_player.conn);
	}

	return -EOPNOTSUPP;
}

int media_proxy_ctrl_set_current_track_id(struct media_player *player, uint64_t id)
{
	CHECKIF(player == NULL) {
		BT_DBG("player is NULL");
		return -EINVAL;
	}

	CHECKIF(id < BT_OTS_OBJ_ID_MIN || id > BT_OTS_OBJ_ID_MAX) {
		BT_DBG("Object ID invalid");
		return -EINVAL;
	}

	if (mprx.local_player.registered && player == &mprx.local_player) {
		if (mprx.local_player.calls->set_current_track_id) {
			mprx.local_player.calls->set_current_track_id(id);

			if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->current_track_id_write) {
				mprx.ctrlr.cbs->current_track_id_write(player, 0, id);
			} else {
				BT_DBG("No callback");
			}

			return 0;
		}

		BT_DBG("No call");
		return -EOPNOTSUPP;
	}

	if (IS_ENABLED(CONFIG_BT_MCC_OTS) &&
	    mprx.remote_player.registered && player == &mprx.remote_player) {
		/* TODO: Uncomment when function is implemented */
		/* return bt_mcc_set_current_track_obj_id(mprx.remote_player.conn, position); */
	}

	return -EOPNOTSUPP;
}

int media_proxy_ctrl_get_next_track_id(struct media_player *player)
{
	CHECKIF(player == NULL) {
		BT_DBG("player is NULL");
		return -EINVAL;
	}

	if (mprx.local_player.registered && player == &mprx.local_player) {
		if (mprx.local_player.calls->get_next_track_id) {
			uint64_t id = mprx.local_player.calls->get_next_track_id();

			if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->next_track_id_recv) {
				mprx.ctrlr.cbs->next_track_id_recv(player, 0, id);
			} else {
				BT_DBG("No callback");
			}

			return 0;
		}

		BT_DBG("No call");
		return -EOPNOTSUPP;
	}

	if (IS_ENABLED(CONFIG_BT_MCC_OTS) &&
	    mprx.remote_player.registered && player == &mprx.remote_player) {
		return bt_mcc_read_next_track_obj_id(mprx.remote_player.conn);
	}

	return -EOPNOTSUPP;
}

int media_proxy_ctrl_set_next_track_id(struct media_player *player, uint64_t id)
{
	CHECKIF(player == NULL) {
		BT_DBG("player is NULL");
		return -EINVAL;
	}

	CHECKIF(id < BT_OTS_OBJ_ID_MIN || id > BT_OTS_OBJ_ID_MAX) {
		BT_DBG("Object ID invalid");
		return -EINVAL;
	}

	if (mprx.local_player.registered && player == &mprx.local_player) {
		if (mprx.local_player.calls->set_next_track_id) {
			mprx.local_player.calls->set_next_track_id(id);

			if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->next_track_id_write) {
				mprx.ctrlr.cbs->next_track_id_write(player, 0, id);
			} else {
				BT_DBG("No callback");
			}

			return 0;
		}

		BT_DBG("No call");
		return -EOPNOTSUPP;
	}

	if (IS_ENABLED(CONFIG_BT_MCC_OTS) &&
	    mprx.remote_player.registered && player == &mprx.remote_player) {
		/* TODO: Uncomment when function is implemented */
		/* return bt_mcc_set_next_track_obj_id(mprx.remote_player.conn, position); */
	}

	return -EOPNOTSUPP;
}

int media_proxy_ctrl_get_parent_group_id(struct media_player *player)
{
	CHECKIF(player == NULL) {
		BT_DBG("player is NULL");
		return -EINVAL;
	}

	if (mprx.local_player.registered && player == &mprx.local_player) {
		if (mprx.local_player.calls->get_parent_group_id) {
			uint64_t id = mprx.local_player.calls->get_parent_group_id();

			if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->parent_group_id_recv) {
				mprx.ctrlr.cbs->parent_group_id_recv(player, 0, id);
			} else {
				BT_DBG("No callback");
			}

			return 0;
		}

		BT_DBG("No call");
		return -EOPNOTSUPP;
	}

	if (IS_ENABLED(CONFIG_BT_MCC_OTS) &&
	    mprx.remote_player.registered && player == &mprx.remote_player) {
		return bt_mcc_read_parent_group_obj_id(mprx.remote_player.conn);
	}

	return -EOPNOTSUPP;
}

int media_proxy_ctrl_get_current_group_id(struct media_player *player)
{
	CHECKIF(player == NULL) {
		BT_DBG("player is NULL");
		return -EINVAL;
	}

	if (mprx.local_player.registered && player == &mprx.local_player) {
		if (mprx.local_player.calls->get_current_group_id) {
			uint64_t id = mprx.local_player.calls->get_current_group_id();

			if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->current_group_id_recv) {
				mprx.ctrlr.cbs->current_group_id_recv(player, 0, id);
			} else {
				BT_DBG("No callback");
			}

			return 0;
		}

		BT_DBG("No call");
		return -EOPNOTSUPP;
	}

	if (IS_ENABLED(CONFIG_BT_MCC_OTS) &&
	    mprx.remote_player.registered && player == &mprx.remote_player) {
		return bt_mcc_read_current_group_obj_id(mprx.remote_player.conn);
	}

	return -EOPNOTSUPP;
}

int media_proxy_ctrl_set_current_group_id(struct media_player *player, uint64_t id)
{
	CHECKIF(player == NULL) {
		BT_DBG("player is NULL");
		return -EINVAL;
	}

	CHECKIF(id < BT_OTS_OBJ_ID_MIN || id > BT_OTS_OBJ_ID_MAX) {
		BT_DBG("Object ID invalid");
		return -EINVAL;
	}

	if (mprx.local_player.registered && player == &mprx.local_player) {
		if (mprx.local_player.calls->set_current_group_id) {
			mprx.local_player.calls->set_current_group_id(id);

			if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->current_group_id_write) {
				mprx.ctrlr.cbs->current_group_id_write(player, 0, id);
			} else {
				BT_DBG("No callback");
			}

			return 0;
		}

		BT_DBG("No call");
		return -EOPNOTSUPP;
	}

	if (IS_ENABLED(CONFIG_BT_MCC_OTS) &&
	    mprx.remote_player.registered && player == &mprx.remote_player) {
		/* TODO: Uncomment when function is implemented */
		/* return bt_mcc_set_current_group_obj_id(mprx.remote_player.conn, position); */
	}

	return -EOPNOTSUPP;
}

int media_proxy_ctrl_get_playing_order(struct media_player *player)
{
	CHECKIF(player == NULL) {
		BT_DBG("player is NULL");
		return -EINVAL;
	}

	if (mprx.local_player.registered && player == &mprx.local_player) {
		if (mprx.local_player.calls->get_playing_order) {
			uint8_t order = mprx.local_player.calls->get_playing_order();

			if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->playing_order_recv) {
				mprx.ctrlr.cbs->playing_order_recv(player, 0, order);
			} else {
				BT_DBG("No callback");
			}

			return 0;
		}

		BT_DBG("No call");
		return -EOPNOTSUPP;
	}

	if (IS_ENABLED(CONFIG_BT_MCC) &&
	    mprx.remote_player.registered && player == &mprx.remote_player) {
		BT_DBG("Remote player");
		return bt_mcc_read_playing_order(mprx.remote_player.conn);
	}

	return -EOPNOTSUPP;
}

int media_proxy_ctrl_set_playing_order(struct media_player *player, uint8_t order)
{
	CHECKIF(player == NULL) {
		BT_DBG("player is NULL");
		return -EINVAL;
	}

	if (mprx.local_player.registered && player == &mprx.local_player) {
		if (mprx.local_player.calls->set_playing_order) {
			mprx.local_player.calls->set_playing_order(order);

			if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->playing_order_write) {
				mprx.ctrlr.cbs->playing_order_write(player, 0, order);
			} else {
				BT_DBG("No callback");
			}

			return 0;
		}

		BT_DBG("No call");
		return -EOPNOTSUPP;
	}

	if (IS_ENABLED(CONFIG_BT_MCC) &&
	    mprx.remote_player.registered && player == &mprx.remote_player) {
		return bt_mcc_set_playing_order(mprx.remote_player.conn, order);
	}

	return -EOPNOTSUPP;
}

int media_proxy_ctrl_get_playing_orders_supported(struct media_player *player)
{
	CHECKIF(player == NULL) {
		BT_DBG("player is NULL");
		return -EINVAL;
	}

	if (mprx.local_player.registered && player == &mprx.local_player) {
		if (mprx.local_player.calls->get_playing_orders_supported) {
			uint16_t orders = mprx.local_player.calls->get_playing_orders_supported();

			if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->playing_orders_supported_recv) {
				mprx.ctrlr.cbs->playing_orders_supported_recv(player, 0, orders);
			} else {
				BT_DBG("No callback");
			}

			return 0;
		}

		BT_DBG("No call");
		return -EOPNOTSUPP;
	}

	if (IS_ENABLED(CONFIG_BT_MCC) &&
	    mprx.remote_player.registered && player == &mprx.remote_player) {
		return bt_mcc_read_playing_orders_supported(mprx.remote_player.conn);
	}

	return -EOPNOTSUPP;
}

int media_proxy_ctrl_get_media_state(struct media_player *player)
{
	CHECKIF(player == NULL) {
		BT_DBG("player is NULL");
		return -EINVAL;
	}

	if (mprx.local_player.registered && player == &mprx.local_player) {
		if (mprx.local_player.calls->get_media_state) {
			uint8_t state = mprx.local_player.calls->get_media_state();

			if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->media_state_recv) {
				mprx.ctrlr.cbs->media_state_recv(player, 0, state);
			} else {
				BT_DBG("No callback");
			}

			return 0;
		}

		BT_DBG("No call");
		return -EOPNOTSUPP;
	}

	if (IS_ENABLED(CONFIG_BT_MCC) &&
	    mprx.remote_player.registered && player == &mprx.remote_player) {
		return bt_mcc_read_media_state(mprx.remote_player.conn);
	}

	return -EOPNOTSUPP;
}

int media_proxy_ctrl_send_command(struct media_player *player, struct mpl_cmd cmd)
{
	CHECKIF(player == NULL) {
		BT_DBG("player is NULL");
		return -EINVAL;
	}

	if (mprx.local_player.registered && player == &mprx.local_player) {
		if (mprx.local_player.calls->send_command) {
			mprx.local_player.calls->send_command(cmd);

			if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->command_send) {
				mprx.ctrlr.cbs->command_send(player, 0, cmd);
			} else {
				BT_DBG("No callback");
			}

			return 0;
		}

		BT_DBG("No call");
		return -EOPNOTSUPP;
	}

	if (IS_ENABLED(CONFIG_BT_MCC) &&
	    mprx.remote_player.registered && player == &mprx.remote_player) {
		return bt_mcc_send_cmd(mprx.remote_player.conn, cmd);
	}

	return -EOPNOTSUPP;
}

int media_proxy_ctrl_get_commands_supported(struct media_player *player)
{
	CHECKIF(player == NULL) {
		BT_DBG("player is NULL");
		return -EINVAL;
	}

	if (mprx.local_player.registered && player == &mprx.local_player) {
		if (mprx.local_player.calls->get_commands_supported) {
			uint32_t opcodes = mprx.local_player.calls->get_commands_supported();

			if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->commands_supported_recv) {
				mprx.ctrlr.cbs->commands_supported_recv(player, 0, opcodes);
			} else {
				BT_DBG("No callback");
			}

			return 0;
		}

		BT_DBG("No call");
		return -EOPNOTSUPP;
	}

	if (IS_ENABLED(CONFIG_BT_MCC) &&
	    mprx.remote_player.registered && player == &mprx.remote_player) {
		return bt_mcc_read_opcodes_supported(mprx.remote_player.conn);
	}

	return -EOPNOTSUPP;
}

int media_proxy_ctrl_send_search(struct media_player *player, struct mpl_search search)
{
	CHECKIF(player == NULL) {
		BT_DBG("player is NULL");
		return -EINVAL;
	}

	if (mprx.local_player.registered && player == &mprx.local_player) {
		if (mprx.local_player.calls->send_search) {
			mprx.local_player.calls->send_search(search);

			if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->search_send) {
				mprx.ctrlr.cbs->search_send(player, 0, search);
			} else {
				BT_DBG("No callback");
			}

			return 0;
		}

		BT_DBG("No call");
		return -EOPNOTSUPP;
	}

	if (IS_ENABLED(CONFIG_BT_MCC_OTS) &&
	    mprx.remote_player.registered && player == &mprx.remote_player) {
		return bt_mcc_send_search(mprx.remote_player.conn, search);
	}

	return -EOPNOTSUPP;
}

int media_proxy_ctrl_get_search_results_id(struct media_player *player)
{
	CHECKIF(player == NULL) {
		BT_DBG("player is NULL");
		return -EINVAL;
	}

	if (mprx.local_player.registered && player == &mprx.local_player) {
		if (mprx.local_player.calls->get_search_results_id) {
			uint64_t id = mprx.local_player.calls->get_search_results_id();

			if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->search_results_id_recv) {
				mprx.ctrlr.cbs->search_results_id_recv(player, 0, id);
			} else {
				BT_DBG("No callback");
			}

			return 0;
		}

		BT_DBG("No call");
		return -EOPNOTSUPP;
	}

	if (IS_ENABLED(CONFIG_BT_MCC_OTS) &&
	    mprx.remote_player.registered && player == &mprx.remote_player) {
		return bt_mcc_read_search_results_obj_id(mprx.remote_player.conn);
	}

	return -EOPNOTSUPP;
}

uint8_t media_proxy_ctrl_get_content_ctrl_id(struct media_player *player)
{
	CHECKIF(player == NULL) {
		BT_DBG("player is NULL");
		return -EINVAL;
	}

	if (mprx.local_player.registered && player == &mprx.local_player) {
		if (mprx.local_player.calls->get_content_ctrl_id) {
			uint8_t ccid = mprx.local_player.calls->get_content_ctrl_id();

			if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->content_ctrl_id_recv) {
				mprx.ctrlr.cbs->content_ctrl_id_recv(player, 0, ccid);
			} else {
				BT_DBG("No callback");
			}

			return 0;
		}

		BT_DBG("No call");
		return -EOPNOTSUPP;
	}

	if (IS_ENABLED(CONFIG_BT_MCC) &&
	    mprx.remote_player.registered && player == &mprx.remote_player) {
		return bt_mcc_read_content_control_id(mprx.remote_player.conn);
	}

	return -EOPNOTSUPP;
}

/* Player calls *******************************************/

int media_proxy_pl_register(struct media_proxy_pl_calls *pl_calls)
{
	CHECKIF(pl_calls == NULL) {
		BT_DBG("NULL calls");
		return -EINVAL;
	}

	if (mprx.local_player.registered) {
		BT_DBG("Player already registered");
		return -EALREADY;
	}

	mprx.local_player.calls = pl_calls;
	mprx.local_player.registered = true;

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->local_player_instance) {
		mprx.ctrlr.cbs->local_player_instance(&mprx.local_player, 0);
	}

	return 0;
};

/* Player callbacks ********************************/

/* All callbacks here must come from the local player - mprx.local_player */

void media_proxy_pl_track_changed_cb(void)
{
	mprx.sctrlr.cbs->track_changed();

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->track_changed_recv) {
		mprx.ctrlr.cbs->track_changed_recv(&mprx.local_player, 0);
	} else {
		BT_DBG("No ctrlr track changed callback");
	}
}

void media_proxy_pl_track_title_cb(char *title)
{
	mprx.sctrlr.cbs->track_title(title);

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->track_title_recv) {
		mprx.ctrlr.cbs->track_title_recv(&mprx.local_player, 0, title);
	} else {
		BT_DBG("No ctrlr track title callback");
	}
}

void media_proxy_pl_track_duration_cb(int32_t duration)
{
	mprx.sctrlr.cbs->track_duration(duration);

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->track_duration_recv) {
		mprx.ctrlr.cbs->track_duration_recv(&mprx.local_player, 0, duration);
	} else {
		BT_DBG("No ctrlr track duration callback");
	}
}

void media_proxy_pl_track_position_cb(int32_t position)
{
	mprx.sctrlr.cbs->track_position(position);

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->track_position_recv) {
		mprx.ctrlr.cbs->track_position_recv(&mprx.local_player, 0, position);
	} else {
		BT_DBG("No ctrlr track position callback");
	}
}

void media_proxy_pl_playback_speed_cb(int8_t speed)
{
	mprx.sctrlr.cbs->playback_speed(speed);

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->playback_speed_recv) {
		mprx.ctrlr.cbs->playback_speed_recv(&mprx.local_player, 0, speed);
	} else {
		BT_DBG("No ctrlr playback speed callback");
	}
}

void media_proxy_pl_seeking_speed_cb(int8_t speed)
{
	mprx.sctrlr.cbs->seeking_speed(speed);

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->seeking_speed_recv) {
		mprx.ctrlr.cbs->seeking_speed_recv(&mprx.local_player, 0, speed);
	} else {
		BT_DBG("No ctrlr seeking speed callback");
	}
}

#ifdef CONFIG_BT_OTS
void media_proxy_pl_current_track_id_cb(uint64_t id)
{
	mprx.sctrlr.cbs->current_track_id(id);

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->current_track_id_recv) {
		mprx.ctrlr.cbs->current_track_id_recv(&mprx.local_player, 0, id);
	} else {
		BT_DBG("No ctrlr current track id callback");
	}
}

void media_proxy_pl_next_track_id_cb(uint64_t id)
{
	mprx.sctrlr.cbs->next_track_id(id);

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->next_track_id_recv) {
		mprx.ctrlr.cbs->next_track_id_recv(&mprx.local_player, 0, id);
	} else {
		BT_DBG("No ctrlr next track id callback");
	}
}

void media_proxy_pl_parent_group_id_cb(uint64_t id)
{
	mprx.sctrlr.cbs->parent_group_id(id);

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->parent_group_id_recv) {
		mprx.ctrlr.cbs->parent_group_id_recv(&mprx.local_player, 0, id);
	} else {
		BT_DBG("No ctrlr parent group id callback");
	}
}

void media_proxy_pl_current_group_id_cb(uint64_t id)
{
	mprx.sctrlr.cbs->current_group_id(id);

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->current_group_id_recv) {
		mprx.ctrlr.cbs->current_group_id_recv(&mprx.local_player, 0, id);
	} else {
		BT_DBG("No ctrlr current group id callback");
	}
}
#endif /* CONFIG_BT_OTS */

void media_proxy_pl_playing_order_cb(uint8_t order)
{
	mprx.sctrlr.cbs->playing_order(order);

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->playing_order_recv) {
		mprx.ctrlr.cbs->playing_order_recv(&mprx.local_player, 0, order);
	} else {
		BT_DBG("No ctrlr playing order callback");
	}
}

void media_proxy_pl_media_state_cb(uint8_t state)
{
	mprx.sctrlr.cbs->media_state(state);

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->media_state_recv) {
		mprx.ctrlr.cbs->media_state_recv(&mprx.local_player, 0, state);
	} else {
		BT_DBG("No ctrlr media state callback");
	}
}

void media_proxy_pl_command_cb(struct mpl_cmd_ntf cmd_ntf)
{
	mprx.sctrlr.cbs->command(cmd_ntf);

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->command_recv) {
		mprx.ctrlr.cbs->command_recv(&mprx.local_player, 0, cmd_ntf);
	} else {
		BT_DBG("No ctrlr command callback");
	}
}

void media_proxy_pl_commands_supported_cb(uint32_t opcodes)
{
	mprx.sctrlr.cbs->commands_supported(opcodes);

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->commands_supported_recv) {
		mprx.ctrlr.cbs->commands_supported_recv(&mprx.local_player, 0, opcodes);
	} else {
		BT_DBG("No ctrlr commands supported callback");
	}
}

#ifdef CONFIG_BT_OTS
void media_proxy_pl_search_cb(uint8_t result_code)
{
	mprx.sctrlr.cbs->search(result_code);

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->search_recv) {
		mprx.ctrlr.cbs->search_recv(&mprx.local_player, 0, result_code);
	} else {
		BT_DBG("No ctrlr search callback");
	}
}

void media_proxy_pl_search_results_id_cb(uint64_t id)
{
	mprx.sctrlr.cbs->search_results_id(id);

	if (mprx.ctrlr.cbs && mprx.ctrlr.cbs->search_results_id_recv) {
		mprx.ctrlr.cbs->search_results_id_recv(&mprx.local_player, 0, id);
	} else {
		BT_DBG("No ctrlr search results id callback");
	}
}
#endif /* CONFIG_BT_OTS */
