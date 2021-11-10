/**
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SUBSYS_BLUETOOTH_AUDIO_CSIS_H_
#define ZEPHYR_SUBSYS_BLUETOOTH_AUDIO_CSIS_H_

/**
 * @brief Coordinated Set Identification Service (CSIS)
 *
 * @defgroup bt_gatt_csis Coordinated Set Identification Service  (CSIS)
 *
 * @ingroup bluetooth
 * @{
 * *
 * [Experimental] Users should note that the APIs can change as a part of ongoing development.
 */


#include <zephyr/types.h>
#include <stdbool.h>
#include <bluetooth/conn.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Recommended timer for member discovery */
#define CSIS_CLIENT_DISCOVER_TIMER_VALUE               K_SECONDS(10)

#if defined(CONFIG_BT_CSIS_CLIENT)
#define BT_CSIS_CLIENT_MAX_CSIS_INSTANCES CONFIG_BT_CSIS_CLIENT_MAX_CSIS_INSTANCES
#else
#define BT_CSIS_CLIENT_MAX_CSIS_INSTANCES 0
#endif /* CONFIG_BT_CSIS_CLIENT */

#define BT_CSIS_MINIMUM_SET_SIZE                2
#define BT_CSIS_PSRI_SIZE                       6

/** Accept the request to read the SIRK as plaintext */
#define BT_CSIS_READ_SIRK_REQ_RSP_ACCEPT        0x00
/** Accept the request to read the SIRK, but return encrypted SIRK */
#define BT_CSIS_READ_SIRK_REQ_RSP_ACCEPT_ENC    0x01
/** Reject the request to read the SIRK */
#define BT_CSIS_READ_SIRK_REQ_RSP_REJECT        0x02
/** SIRK is available only via an OOB procedure */
#define BT_CSIS_READ_SIRK_REQ_RSP_OOB_ONLY      0x03

#define BT_CSIS_SET_SIRK_SIZE 16

#define BT_CSIS_ERROR_LOCK_DENIED               0x80
#define BT_CSIS_ERROR_LOCK_RELEASE_DENIED       0x81
#define BT_CSIS_ERROR_LOCK_INVAL_VALUE          0x82
#define BT_CSIS_ERROR_SIRK_ACCESS_REJECTED      0x83
#define BT_CSIS_ERROR_SIRK_OOB_ONLY             0x84
#define BT_CSIS_ERROR_LOCK_ALREADY_GRANTED      0x85

#define BT_CSIS_RELEASE_VALUE                   0x01
#define BT_CSIS_LOCK_VALUE                      0x02

#define BT_CSIS_SIRK_TYPE_ENCRYPTED             0x00
#define BT_CSIS_SIRK_TYPE_PLAIN                 0x01

/** @brief Opaque Coordinated Set Identification Service instance. */
struct bt_csis;

struct bt_csis_cb {
	/**
	 * @brief Callback whenever the lock changes on the server.
	 *
	 * @param conn    The connection to the client that changed the lock.
	 *                NULL if server changed it, either by calling
	 *                bt_csis_lock() or by timeout.
	 * @param csis    Pointer to the Coordinated Set Identification Service.
	 * @param locked  Whether the lock was locked or released.
	 *
	 */
	void (*lock_changed)(struct bt_conn *conn, struct bt_csis *csis,
			     bool locked);

	/**
	 * @brief Request from a peer device to read the sirk.
	 *
	 * If this callback is not set, all clients will be allowed to read
	 * the SIRK unencrypted.
	 *
	 * @param conn The connection to the client that requested to read the
	 *             SIRK.
	 * @param csis Pointer to the Coordinated Set Identification Service.
	 *
	 * @return A BT_CSIS_READ_SIRK_REQ_RSP_* response code.
	 */
	uint8_t (*sirk_read_req)(struct bt_conn *conn, struct bt_csis *csis);
};

/** Register structure for Coordinated Set Identification Service */
struct bt_csis_register_param {
	/**
	 * @brief Size of the set.
	 *
	 * If set to 0, the set size characteric won't be initialized.
	 * Otherwise shall be set to minimum 2.
	 */
	uint8_t set_size;

	/**
	 * @brief The unique Set Identity Resolving Key (SIRK)
	 *
	 * This shall be unique between different sets, and shall be the same
	 * for each set member for each set.
	 */
	uint8_t set_sirk[BT_CSIS_SET_SIRK_SIZE];

	/**
	 * @brief Boolean to set whether the set is lockable by clients
	 *
	 * Setting this to false will disable the lock characteristic.
	 */
	bool lockable;

	/**
	 * @brief Rank of this device in this set.
	 *
	 * If the lockable parameter is set to true, this shall be > 0 and
	 * <= to the set_size. If the lockable parameter is set to false, this
	 * may be set to 0 to disable the rank characteristic.
	 */
	uint8_t rank;

	/** Pointer to the callback structure. */
	struct bt_csis_cb *cb;
};

/**
 * @brief Get the service declaration attribute.
 *
 * The first service attribute can be included in any other GATT service.
 *
 * @param csis   Pointer to the Coordinated Set Identification Service.
 *
 * @return The first CSIS attribute instance.
 */
void *bt_csis_svc_decl_get(const struct bt_csis *csis);

/**
 * @brief Register the Coordinated Set Identification Service.
 *
 * This will register and enable the service and make it discoverable by
 * clients.
 *
 * This shall only be done as a server.
 *
 * @param param      Coordinated Set Identification Service register parameters.
 * @param[out] csis  Pointer to the registered Coordinated Set Identification
 *                   Service.
 *
 * @return 0 if success, errno on failure.
 */
int bt_csis_register(const struct bt_csis_register_param *param,
		     struct bt_csis **csis);

/**
 * @brief Print the sirk to the debug output
 *
 * @param csis   Pointer to the Coordinated Set Identification Service.
 */
void bt_csis_print_sirk(const struct bt_csis *csis);

/**
 * @brief Starts advertising the PRSI value.
 *
 * This cannot be used with other connectable advertising sets.
 *
 * @param csis          Pointer to the Coordinated Set Identification Service.
 * @param enable	If true start advertising, if false stop advertising
 *
 * @return int		0 if on success, ERRNO on error.
 */
int bt_csis_advertise(struct bt_csis *csis, bool enable);

/**
 * @brief Locks the sets on the server.
 *
 * @param csis    Pointer to the Coordinated Set Identification Service.
 * @param lock    If true lock the set, if false release the set.
 * @param force   This argument only have meaning when @p lock is false
 *                (release) and will force release the lock, regardless of who
 *                took the lock.
 *
 * @return 0 on success, GATT error on error.
 */
int bt_csis_lock(struct bt_csis *csis, bool lock, bool force);

struct bt_csis_set_sirk {
	uint8_t type;
	uint8_t value[BT_CSIS_SET_SIRK_SIZE];
} __packed;

struct bt_csis_client_set {
	struct bt_csis_set_sirk set_sirk;
	uint8_t set_size;
	uint8_t rank;
};

struct bt_csis_client_set_member {
	struct bt_conn *conn;
	bt_addr_le_t addr;
	struct bt_csis_client_set sets[BT_CSIS_CLIENT_MAX_CSIS_INSTANCES];
};

typedef void (*bt_csis_client_discover_cb)(struct bt_conn *conn, int err,
					   uint8_t set_count);

/**
 * @brief Initialise the csis_client instance for a connection. This will do a
 * discovery on the device and prepare the instance for following commands.
 *
 * @param member Pointer to a set member struct to store discovery results in
 *
 * @return int Return 0 on success, or an errno value on error.
 */
int bt_csis_client_discover(struct bt_csis_client_set_member *member);

typedef void (*bt_csis_client_discover_sets_cb)(struct bt_conn *conn,
						int err, uint8_t set_count,
						struct bt_csis_client_set *sets);

/**
 * @brief Reads CSIS characteristics from a device, to find more information
 * about the set(s) that the device is part of.
 *
 * @param conn The connection to the device to read CSIS characteristics
 *
 * @return int Return 0 on success, or an errno value on error.
 */
int bt_csis_client_discover_sets(struct bt_conn *conn);

typedef void (*bt_csis_client_discover_members_cb)(int err, uint8_t set_size,
						   uint8_t members_found);

/**
 * @brief Start scanning for all devices that are part of a set.
 *
 * @param set The set to find devices for
 *
 * @return int Return 0 on success, or an errno value on error.
 */
int bt_csis_client_discover_members(struct bt_csis_client_set *set);

typedef void (*bt_csis_client_lock_set_cb)(int err);

/**
 * @brief Callback when the lock value on a set of a connected device changes.
 *
 * @param conn    Connection of the CSIS server.
 * @param set     The set that was changed.
 * @param locked  Whether the lock is locked or release.
 *
 * @return int Return 0 on success, or an errno value on error.
 */
typedef void (*bt_csis_client_lock_changed_cb)(struct bt_conn *conn,
					       struct bt_csis_client_set *set,
					       bool locked);

/**
 * @brief Callback when the lock value is read on a device.
 *
 * @param conn      Connection of the CSIS server.
 * @param err       Error value. 0 on success, GATT error or errno on fail.
 * @param inst_idx  The index of the CSIS service.
 * @param locked    Whether the lock is locked or release.
 */
typedef void (*bt_csis_client_lock_read_cb)(struct bt_conn *conn, int err,
					    uint8_t inst_idx, bool locked);

/**
 * @brief Callback when the lock value is written to a device.
 *
 * @param conn      Connection of the CSIS server.
 * @param err       Error value. 0 on success, GATT error or errno on fail.
 * @param inst_idx  The index of the CSIS service.
 */
typedef void (*bt_csis_client_lock_cb)(struct bt_conn *conn, int err,
				       uint8_t inst_idx);

/**
 * @brief Callback when the release value is written to a device.
 *
 * @param conn      Connection of the CSIS server.
 * @param err       Error value. 0 on success, GATT error or errno on fail.
 * @param inst_idx  The index of the CSIS service.
 */
typedef void (*bt_csis_client_release_cb)(struct bt_conn *conn, int err,
					  uint8_t inst_idx);

struct bt_csis_client_cb {
	/* Set callbacks */
	bt_csis_client_lock_set_cb             lock_set;
	bt_csis_client_lock_set_cb             release_set;
	bt_csis_client_discover_members_cb     members;
	bt_csis_client_discover_sets_cb        sets;
	bt_csis_client_lock_changed_cb         lock_changed;

	/* Device specific callbacks */
	bt_csis_client_discover_cb             discover;
	bt_csis_client_lock_read_cb            lock_read;
	bt_csis_client_lock_cb                 lock;
	bt_csis_client_release_cb              release;
};

/**
 * @brief Check if advertising data indicates a set member
 *
 * @param set_sirk The SIRK of the set to check against
 * @param data     The advertising data
 *
 * @return true if the advertising data indicates a set member, false otherwise
 */
bool bt_csis_client_is_set_member(uint8_t set_sirk[BT_CSIS_SET_SIRK_SIZE],
				  struct bt_data *data);

/**
 * @brief Lock the set
 *
 * Connect to and set the lock for all devices in a set.
 *
 * @return Return 0 on success, or an errno value on error.
 */
int bt_csis_client_lock_set(void);

/**
 * @brief Connect to and release the lock for all devices in a set
 *
 * @return int Return 0 on success, or an errno value on error.
 */
int bt_csis_client_release_set(void);

/**
 * @brief Registers callbacks for csis_client.
 *
 * @param cb   Pointer to the callback structure.
 */
void bt_csis_client_register_cb(struct bt_csis_client_cb *cb);

/**
 * @brief Read the lock value of a specific device and instance.
 *
 * @param conn      Pointer to the connection to the device.
 * @param inst_idx  Index of the CSIS index of the peer device (as it may have
 *                  multiple CSIS instances).
 *
 * @return Return 0 on success, or an errno value on error.
 */
int bt_csis_client_lock_get(struct bt_conn *conn, uint8_t inst_idx);

/**
 * @brief Lock an array of set members
 *
 * The members will be locked starting from lowest rank going up.
 *
 * TODO: If locking fails, the already locked members will not be unlocked.
 *
 * @param members   Array of set members to lock.
 * @param count     Number of set members in @p members.
 * @param set       Pointer to the specified set, as a member may be part of
 *                  multiple sets.
 *
 * @return Return 0 on success, or an errno value on error.
 */
int bt_csis_client_lock(const struct bt_csis_client_set_member **members,
			uint8_t count, const struct bt_csis_client_set *set);

/**
 * @brief Release an array of set members
 *
 * The members will be released starting from highest rank going down.
 *
 * @param members   Array of set members to lock.
 * @param count     Number of set members in @p members.
 * @param set       Pointer to the specified set, as a member may be part of
 *                  multiple sets.
 *
 * @return Return 0 on success, or an errno value on error.
 */
int bt_csis_client_release(const struct bt_csis_client_set_member **members,
			   uint8_t count, const struct bt_csis_client_set *set);


#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_SUBSYS_BLUETOOTH_AUDIO_CSIS_H_ */
