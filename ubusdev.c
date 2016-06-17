/*
 * netifd - network interface daemon
 * Copyright (C) 2015 Arne Kappen <akappen@inet.tu-berlin.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <libubox/blobmsg.h>
#include <libubox/list.h>
#include <libubus.h>

#include "netifd.h"
#include "handler.h"
#include "device.h"
#include "ubus.h"
#include "ubusdev.h"
#include "interface.h"
#include "system.h"

#define UBUSDEV_UBUSOBJ_NAME_PREFIX "network.device.ubus."

extern struct ubus_context *ubus_ctx;
static struct blob_buf blob_buffer;
static int confdir_fd = -1;

enum external_sync_state {
	// device handler interface
	EXTERNAL_STATE_SYNC_OK,
	EXTERNAL_STATE_SYNC_PENDING_CREATE,
	EXTERNAL_STATE_SYNC_PENDING_RELOAD,
	EXTERNAL_STATE_SYNC_PENDING_FREE,

	// hotplug ops
	EXTERNAL_STATE_SYNC_PENDING_ADD,
	EXTERNAL_STATE_SYNC_PENDING_REMOVE,
	EXTERNAL_STATE_SYNC_PENDING_PREPARE,
};

// wrapper for ubus device type structs
struct ubusdev_type {
	struct device_type handler;

	const char *ext_dev_handler_name;
	uint32_t ubus_peer_id;
	struct ubus_subscriber ubus_sub;
	bool subscribed;

	// for parsing device configs
	struct uci_blob_param_list *config_params;
	char *config_strbuf;

	// for parsing 'dump info' replies
	struct uci_blob_param_list *info_params;
	char *info_strbuf;

	// for parsing 'dump stats' replies
	struct uci_blob_param_list *stats_params;
	char *stats_strbuf;
};

// wrapper for device structs with bridge capability
struct ubusdev_bridge {
	struct device dev;
	device_state_cb set_state;

	struct ubusdev_type *utype;

	struct blob_attr *config;
	// the members' names
	bool empty;
	struct blob_attr *ifnames;
	bool active;
	bool force_active;

	struct vlist_tree members;
	int n_present;
	int n_failed;

	enum external_sync_state sync;
	// retry for tasks that need to be executed by external device handler
	struct uloop_timeout retry;
};

struct ubusdev_bridge_member {
	struct vlist_node node;
	struct ubusdev_bridge *parent_br;
	struct device_user dev_usr;
	bool present;
	bool hotplug;
	char *name;

	enum external_sync_state sync;
};

enum {
	// device handler interface
	UBUSDEV_METHOD_CREATE,
	UBUSDEV_METHOD_CONFIG_INIT,
	UBUSDEV_METHOD_RELOAD,
	UBUSDEV_METHOD_DUMP_INFO,
	UBUSDEV_METHOD_DUMP_STATS,
	UBUSDEV_METHOD_CHECK_STATE,
	UBUSDEV_METHOD_FREE,

	// hotplug ops
	UBUSDEV_METHOD_HOTPLUG_ADD,
	UBUSDEV_METHOD_HOTPLUG_REMOVE,
	UBUSDEV_METHOD_HOTPLUG_PREPARE,
	__UBUSDEV_METHODS_MAX
};

static const char *__ubusdev_methods[__UBUSDEV_METHODS_MAX] = {
	// device handler interface
	[UBUSDEV_METHOD_CREATE] = "create",
	[UBUSDEV_METHOD_CONFIG_INIT] = "config_init",
	[UBUSDEV_METHOD_RELOAD] = "reload",
	[UBUSDEV_METHOD_DUMP_INFO] = "dump_info",
	[UBUSDEV_METHOD_DUMP_STATS] = "dump_stats",
	[UBUSDEV_METHOD_CHECK_STATE] = "check_state",
	[UBUSDEV_METHOD_FREE] = "free",

	// hotplug ops
	[UBUSDEV_METHOD_HOTPLUG_ADD] = "add",
	[UBUSDEV_METHOD_HOTPLUG_REMOVE] = "remove",
	[UBUSDEV_METHOD_HOTPLUG_PREPARE] = "prepare",
};

static void
ubusdev_invocation_error(int error, const char *method, const char *devname)
{
	fprintf(stderr, "invocation of method '%s' failed for device '%s': %s\n",
		method, devname, ubus_strerror(error));
	netifd_log_message(L_CRIT, "invocation of method '%s' failed for device "
		"'%s': %s\n", method, devname, ubus_strerror(error));
}


static struct ubus_method ubusdev_object_methods[] = {/*
	{ .name = "create", 		.handler = ubusdev_handle_create },
	{ .name = "config_init", 	.handler = ubusdev_handle_config_init },
	{ .name = "reload", 		.handler = ubusdev_handle_reload },
	{ .name = "dump_info", 		.handler = ubusdev_handle_dump_info },
	{ .name = "dump_stats", 	.handler = ubusdev_handle_dump_stats },
	{ .name = "check_state", 	.handler = ubusdev_handle_check_state },
	{ .name = "free", 			.handler = ubusdev_handle_free }*/
};

static struct ubus_object_type ubusdev_ubus_object_type =
	UBUS_OBJECT_TYPE("netifd_ubusdev", ubusdev_object_methods);

/* find out the ubus ID of the peer to a given ubus device type wrapper
 */
static int
ubusdev_lookup_id(struct ubusdev_type *utype)
{
	int ret;

	if (!utype || !utype->ext_dev_handler_name)
		return 1;

	ret = ubus_lookup_id(ubus_ctx, utype->ext_dev_handler_name,
		&utype->ubus_peer_id);

	if (ret) {
		fprintf(stderr, "could not find ubus ID for object '%s'\n",
			utype->ext_dev_handler_name);
		return ret;
	}

	return 0;
}

static void
ubusdev_bridge_set_sync(struct ubusdev_bridge *ubr,
	enum external_sync_state status)
{
	ubr->sync = status;
}

static void
ubusdev_bridge_set_timeout(enum external_sync_state status,
	struct ubusdev_bridge *ubr, unsigned long ms)
{
	ubusdev_bridge_set_sync(ubr, status);
	uloop_timeout_set(&ubr->retry, ms);
}

 /* Enable the bridge. This means creating the bridge device which is done by
  * the external device handler.
  * The bridge's sync status is set to inidcate pending creation and the ubus
  * call to the external device handler is issued.
  * Return values:
  * 	0 	- the bridge exists and is operational
  *		<0 	- the ubus call was made and netifd has to wait for the notification
  *			  from the external device handler
  *		>0	- something went wrong with the ubus call. return the ubus status
  */
static int
ubusdev_bridge_enable_interface(struct ubusdev_bridge *ubr)
{
	int ret;
	struct ubusdev_type *utype;

	if (ubr->active)
		return 0;

	if (ubr->sync == EXTERNAL_STATE_SYNC_PENDING_CREATE)
		return -1;

	// tell external device handler to create bridge with stored config
	ubusdev_bridge_set_sync(ubr, EXTERNAL_STATE_SYNC_PENDING_CREATE);
	utype = container_of(ubr->dev.type, struct ubusdev_type, handler);
	ret = netifd_ubusdev_invoke(utype->ubus_peer_id,
		__ubusdev_methods[UBUSDEV_METHOD_CREATE], ubr->config, NULL, NULL);

	if (ret)
		goto error;

	// the ubus call was made successfully -> start notification timer
	ubusdev_bridge_set_timeout(EXTERNAL_STATE_SYNC_PENDING_CREATE, ubr, 100);
	return -1;

error:
	ubusdev_invocation_error(ret, __ubusdev_methods[UBUSDEV_METHOD_CREATE],
		ubr->dev.ifname);
	return ret;
}

static int
ubusdev_bridge_disable_interface(struct ubusdev_bridge *ubr)
{
	ubr->active = false;
	return 0;
}

static int
ubusdev_hotplug_add(struct device *dev, struct device *member)
{
	int ret = 0;
	struct ubusdev_type *utype = container_of(dev->type, struct ubusdev_type,
		handler);

	if (!dev->type->bridge_capability)
		return UBUS_STATUS_NOT_SUPPORTED;

	blob_buf_init(&blob_buffer, 0);
	blobmsg_add_string(&blob_buffer, "bridge", dev->ifname);
	blobmsg_add_string(&blob_buffer, "member", member->ifname);

	ret = netifd_ubusdev_invoke(utype->ubus_peer_id,
		__ubusdev_methods[UBUSDEV_METHOD_HOTPLUG_ADD], blob_buffer.head, NULL,
		NULL);

	if (ret) {
		ubusdev_invocation_error(ret,
			__ubusdev_methods[UBUSDEV_METHOD_HOTPLUG_ADD], member->ifname);
		return ret;
	}

	// calling device_lock() ensures that the simple device created for a
	// bridge member interface is not freed too early by device_free_unused().
	// (i.e. before the external device handler has signalled back
	// successful device creation)
	device_lock();

	return ret;
}

static int
ubusdev_hotplug_del(struct device *dev, struct device *member)
{
	struct ubusdev_bridge *ubr = container_of(dev, struct ubusdev_bridge, dev);
	struct ubusdev_bridge_member *ubm;

	ubm = vlist_find(&ubr->members, member->ifname, ubm, node);
	if (!ubm)
		return UBUS_STATUS_NOT_FOUND;

	vlist_delete(&ubr->members, &ubm->node);
	return 0;
}

/* Prepare bridge for new member by bringing it up if necessary.
 */
static int
ubusdev_hotplug_prepare(struct device *dev)
{
 	struct ubusdev_bridge *ubr;

 	if (!dev->type->bridge_capability)
 		return UBUS_STATUS_NOT_SUPPORTED;

	ubr = container_of(dev, struct ubusdev_bridge, dev);
	ubr->force_active = true;
	device_set_present(&ubr->dev, true);

	return 0;
}

static int
ubusdev_bridge_enable_member(struct ubusdev_bridge_member *member)
{
	int ret;
	struct ubusdev_bridge *ubr = member->parent_br;
	struct ubusdev_type *utype;

	if (!member->present)
		return 0;

	ret = ubusdev_bridge_enable_interface(ubr);
	if (ret)
		goto error;

	if (ubr->sync != EXTERNAL_STATE_SYNC_OK)
		goto error;

	ret = device_claim(&member->dev_usr);
	if (ret < 0)
		goto error;

	// don't make the ubus call if either
	// 1) the member has been added via a hotplug call to 
	//    'network.interface add_device' (i.e. member->hotplug == true)
	// 2) the member has not been added via a call to
	//    'network.interface add_device' AND its state is in sync with the
	//     external device handler
	if (member->hotplug)
		return 0;
	else if (member->sync != EXTERNAL_STATE_SYNC_PENDING_ADD)
		return 0;

	utype = container_of(ubr->dev.type, struct ubusdev_type, handler);
	blob_buf_init(&blob_buffer, 0);
	blobmsg_add_string(&blob_buffer, "bridge", ubr->dev.ifname);
	blobmsg_add_string(&blob_buffer, "member", member->dev_usr.dev->ifname);

	// abuse hotplug add as addif equivalent. Maybe we need a dedicated ubus
	// method on the external handler for this sort of operation.
	ret = netifd_ubusdev_invoke(utype->ubus_peer_id,
		__ubusdev_methods[UBUSDEV_METHOD_HOTPLUG_ADD], blob_buffer.head, NULL,
		NULL);

	if (ret)
		goto inv_error;

	return 0;

inv_error:
	ubusdev_invocation_error(ret,
		__ubusdev_methods[UBUSDEV_METHOD_HOTPLUG_ADD],
		member->dev_usr.dev->ifname);

error:
	ubr->n_failed++;
	member->present = false;
	member->sync = EXTERNAL_STATE_SYNC_PENDING_ADD;
	ubr->n_present--;
	device_release(&member->dev_usr);

	return ret;
}

/* Disable a bridge member.
 * At the moment, hotplug remove is used to remove the member at the external
 * device handler. If that clashes with future requirements, maybe we'll have to
 * add methods for disabling members.
 */
static int
ubusdev_bridge_disable_member(struct ubusdev_bridge_member *member)
{
	int ret;
	struct ubusdev_bridge *ubr = member->parent_br;
	struct ubusdev_type *utype;

	if (!member->present)
		return 0;

	utype = container_of(ubr->dev.type, struct ubusdev_type, handler);
	blob_buf_init(&blob_buffer, 0);
	blobmsg_add_string(&blob_buffer, "bridge", ubr->dev.ifname);
	blobmsg_add_string(&blob_buffer, "member", member->dev_usr.dev->ifname);

	// abuse hotplug remove as delif equivalent. Maybe we need a dedicated ubus
	// method on the external handler for this sort of operation.
	ret = netifd_ubusdev_invoke(utype->ubus_peer_id,
		__ubusdev_methods[UBUSDEV_METHOD_HOTPLUG_REMOVE], blob_buffer.head,
		NULL, NULL);

	if (ret)
		goto error;

	member->sync = EXTERNAL_STATE_SYNC_PENDING_REMOVE;
	return 0;

error:
	ubusdev_invocation_error(ret,
		__ubusdev_methods[UBUSDEV_METHOD_HOTPLUG_REMOVE],
		member->dev_usr.dev->ifname);

	return ret;
}

static void
ubusdev_bridge_check_members(struct ubusdev_bridge *ubr)
{
	if (!ubr->n_failed)
		return;

	uloop_timeout_set(&ubr->retry, 100);
}

static int
ubusdev_bridge_set_down(struct ubusdev_bridge *ubr)
{
	struct ubusdev_bridge_member *ubm;

	ubr->set_state(&ubr->dev, false);

	vlist_for_each_element(&ubr->members, ubm, node)
		ubusdev_bridge_disable_member(ubm);

	ubusdev_bridge_disable_interface(ubr);

	return 0;
}

static int
ubusdev_bridge_set_up(struct ubusdev_bridge *ubr)
{
	struct ubusdev_bridge_member *member;

	if (!ubr->n_present)
		if (!ubr->force_active)
			return -ENOENT;

	ubr->n_failed = 0;
	vlist_for_each_element(&ubr->members, member, node)
		ubusdev_bridge_enable_member(member);
	ubusdev_bridge_check_members(ubr);


	if (!ubr->force_active && !ubr->n_present) {
		// initialization of all member interfaces failed
		ubusdev_bridge_disable_interface(ubr);
		device_set_present(&ubr->dev, false);
		return -ENOENT;
	}

	return 0;
}

static int
ubusdev_bridge_set_state(struct device *dev, bool up)
{
	struct ubusdev_bridge *ubr;

	if (!dev->type->bridge_capability)
		return -1;

	 ubr = container_of(dev, struct ubusdev_bridge, dev);

	if (up)
		return ubusdev_bridge_set_up(ubr);
	else
		return ubusdev_bridge_set_down(ubr);
}

static void
ubusdev_bridge_remove_member(struct ubusdev_bridge_member *member)
{
	struct ubusdev_bridge *ubr = member->parent_br;

	if (!member->present)
		return;

	// TODO: in case of primary port: reset
	if (ubr->dev.active)
		ubusdev_bridge_disable_member(member);

	member->present = false;
	ubr->n_present--;

	if (ubr->n_present == 0)
		device_set_present(&ubr->dev, false);
}

static void
ubusdev_bridge_member_cb(struct device_user *usr, enum device_event event)
{
	struct ubusdev_bridge_member *member =
		container_of(usr, struct ubusdev_bridge_member, dev_usr);
	struct ubusdev_bridge *ubr = member->parent_br;

	switch (event) {
		case DEV_EVENT_ADD:
			member->present = true;
			ubr->n_present++;

			// set bridge present when first memeber is enabled
			if (ubr->n_present == 1)
				device_set_present(&ubr->dev, true);

			ubusdev_bridge_enable_member(member);
			break;

		case DEV_EVENT_REMOVE:
			if (usr->hotplug) {
				vlist_delete(&ubr->members, &member->node);
				return;
			}

			if (member->present)
				ubusdev_bridge_remove_member(member);
			break;
		default:
			break;
	}
	return;
}

/* Add member 'dev' to bridge.
 */
static struct ubusdev_bridge_member *
ubusdev_bridge_create_member(struct ubusdev_bridge *ubr, struct device *dev,
	bool hotplug)
{
	struct ubusdev_bridge_member *member;
	char *name;

	member = calloc_a(sizeof(*member), &name, strlen(dev->ifname) + 1);
	if (!member)
		return NULL;

	member->parent_br = ubr;
	member->name = name;
	member->hotplug = hotplug;
	strcpy(name, dev->ifname);
	member->dev_usr.dev = dev;
	member->dev_usr.cb = ubusdev_bridge_member_cb;
	vlist_add(&ubr->members, &member->node, member->name);
	// Need to look up the bridge member again as the above
	// created pointer will be freed in case the bridge member
	// already existed
	member = vlist_find(&ubr->members, dev->ifname, member, node);
	if (!member)
		return NULL;

	// hotplug devices have been added to the bridge already
	if (hotplug) {
		member->sync = EXTERNAL_STATE_SYNC_OK;
		member->node.version = -1;
		return member;
	}

	// non-hotplug devices need creation at the external device handler
	member->sync = EXTERNAL_STATE_SYNC_PENDING_ADD;
	return member;
}

/* Add member to bridge. A device for the member is created if it doesn't exist.
 */
static void
ubusdev_bridge_add_member(struct ubusdev_bridge *ubr, const char *name)
{
	struct device *dev;

	dev = device_get(name, true);
	if (!dev)
		return;

	ubusdev_bridge_create_member(ubr, dev, false);
}

static void
ubusdev_bridge_free_member(struct ubusdev_bridge_member *member)
{
	struct device *dev = member->dev_usr.dev;

	ubusdev_bridge_remove_member(member);
	device_remove_user(&member->dev_usr);

	/*
	 * When reloading the config and moving a device from one bridge to
	 * another, the other bridge may have tried to claim this device
	 * before it was removed here.
	 * Ensure that claiming the device is retried by toggling its present
	 * state
	 */
	if (dev->present) {
		device_set_present(dev, false);
		device_set_present(dev, true);
	}

	free(member);
}

/* Called whenever a node is inserted into or removed from the members vlist of
 * ubusdev_bridge structs.
 */
static void
ubusdev_bridge_member_update(struct vlist_tree *tree,
	struct vlist_node *node_new, struct vlist_node *node_old)
{
	struct ubusdev_bridge_member *member;
	struct device *dev;

	if (node_new) {
		member = container_of(node_new, struct ubusdev_bridge_member, node);

		// don't allow replacements
		if (node_old) {
			free(member);
			return;
		}

		// clear device_user fields and set new member
		dev = member->dev_usr.dev;
		member->dev_usr.dev = NULL;
		device_add_user(&member->dev_usr, dev);
	}

	if (node_old) {
		member = container_of(node_old, struct ubusdev_bridge_member, node);
		ubusdev_bridge_free_member(member);
	}
}

/* Parse the config for a device. If the ubusdev_bridge struct already has a
 * configuration, compare them and send it to the external device hanlder with a
 * 'reload' call.
 * If the bridge struct does not have a config yet, this means it has just been
 * created. In this case we simply store the configuration.
 */
static enum dev_change_type
ubusdev_bridge_reload(struct device *dev, struct blob_attr *config)
{
	enum {
		UBUSDEV_BRIDGE_ATTR_EMPTY,
		UBUSDEV_BRIDGE_ATTR_IFNAMES,
		__UBUSDEV_BRIDGE_ATTR_MAX
	};

	static
	struct blobmsg_policy ubusdev_bridge_policy[__UBUSDEV_BRIDGE_ATTR_MAX] = {
		[UBUSDEV_BRIDGE_ATTR_EMPTY] = {
			.name = "empty",
			.type = BLOBMSG_TYPE_BOOL,
		},
		[UBUSDEV_BRIDGE_ATTR_IFNAMES] = {
			.name = "ifname",
			.type = BLOBMSG_TYPE_ARRAY,
		},
	};

	struct blob_attr *tb[__UBUSDEV_BRIDGE_ATTR_MAX];
	enum dev_change_type ret = DEV_CONFIG_APPLIED;
	unsigned long diff;
	struct ubusdev_bridge *ubr;
	int inv_ret;

	BUILD_BUG_ON(sizeof(diff) < __UBUSDEV_BRIDGE_ATTR_MAX / 8);

	ubr = container_of(dev, struct ubusdev_bridge, dev);
	config = blob_memdup(config);

	blobmsg_parse(ubusdev_bridge_policy, __UBUSDEV_BRIDGE_ATTR_MAX, tb,
		blobmsg_data(config), blobmsg_len(config));

	// ignore interface names if the empty flag is set
	if (tb[UBUSDEV_BRIDGE_ATTR_EMPTY] &&
		blobmsg_get_bool(tb[UBUSDEV_BRIDGE_ATTR_EMPTY])) {
		ubr->empty = true;
	} else {
		ubr->ifnames = tb[UBUSDEV_BRIDGE_ATTR_IFNAMES];
	}

	// check if we're reloading a config for an existing device
	// or setting one for the first time
	if (ubr->config) {
		struct blob_attr *old_tb[__UBUSDEV_BRIDGE_ATTR_MAX];

		blobmsg_parse(ubusdev_bridge_policy, __UBUSDEV_BRIDGE_ATTR_MAX,
			old_tb, blobmsg_data(ubr->config), blobmsg_len(ubr->config));

		diff = 0;
		uci_blob_diff(tb, old_tb, dev->type->config_params, &diff);
		if (diff)
			ret = DEV_CONFIG_RESTART;

		struct ubusdev_type *utype = container_of(dev->type,
			struct ubusdev_type, handler);

		inv_ret = netifd_ubusdev_invoke(utype->ubus_peer_id,
			__ubusdev_methods[UBUSDEV_METHOD_RELOAD], config, NULL, NULL);

		if (inv_ret) {
			fprintf(stderr, "Failed to finish config reload for device '%s'."
				"Ubus call to external device handler failed: %s\n",
				dev->ifname, ubus_strerror(inv_ret));
			return DEV_CONFIG_NO_CHANGE;
		}

		// schedule retry
		ubusdev_bridge_set_timeout(EXTERNAL_STATE_SYNC_PENDING_RELOAD,
			ubr, 100);

		free(ubr->config);
	}

	ubr->config = config;
	return ret;
}

static enum dev_change_type
ubusdev_reload(struct device *dev, struct blob_attr *config)
{
	if (dev->type->bridge_capability)
		return ubusdev_bridge_reload(dev, config);

	// TODO: compare configs and if different:
	// 1. set device not present
	// 2. contact external device handler
	return DEV_CONFIG_NO_CHANGE;
}

static void
ubusdev_bridge_retry_enable_members(struct ubusdev_bridge *ubr)
{
	struct ubusdev_bridge_member *cur;

	ubr->n_failed = 0;
	vlist_for_each_element(&ubr->members, cur, node) {
		if (cur->present)
			continue;

		if (!cur->dev_usr.dev->present)
			continue;
		}

		// don't make the call to the external device handler if
		// state is sync'ed
		if (cur->sync == EXTERNAL_STATE_SYNC_OK)
			continue;

		cur->present = true;
		ubr->n_present++;
		ubusdev_bridge_enable_member(cur);
	}
}

/* Callback for timeout on external device handler task.
 * Initiate retry.
 */
static void
ubusdev_bridge_timeout_cb(struct uloop_timeout *timeout)
{
	int ret;
	const char *method;
	struct blob_attr *attr;
	struct ubusdev_bridge *ubr = container_of(timeout, struct ubusdev_bridge,
		retry);

	// In case the external device handler has failed to notify us of anything,
	// retry the call.
	// If the external device handler has notified us of success, however, we
	// re-initiate the setup of the bridge members for active bridges.
	switch (ubr->sync) {
		case EXTERNAL_STATE_SYNC_PENDING_CREATE:
			method = __ubusdev_methods[UBUSDEV_METHOD_CREATE];
			attr = ubr->config;
			break;
		case EXTERNAL_STATE_SYNC_PENDING_RELOAD:
			method = __ubusdev_methods[UBUSDEV_METHOD_RELOAD];
			attr = ubr->config;
			break;
		case EXTERNAL_STATE_SYNC_PENDING_FREE:
			method = __ubusdev_methods[UBUSDEV_METHOD_FREE];
			blob_buf_init(&blob_buffer, 0);
			blobmsg_add_string(&blob_buffer, "bridge", ubr->dev.ifname);
			attr = blob_buffer.head;
			break;
		case EXTERNAL_STATE_SYNC_OK:
			if (!ubr->active)
				return;

			ubusdev_bridge_retry_enable_members(ubr);
			return;
		default:
			return;
	}

	ret = netifd_ubusdev_invoke(ubr->utype->ubus_peer_id, method, attr, NULL,
		NULL);

	if (ret)
		ubusdev_invocation_error(ret, method, ubr->dev.ifname);

	ubusdev_bridge_set_timeout(ubr->sync, ubr, 100);
}

static struct device*
_ubusdev_create(const char *name, struct device_type *type,
	struct blob_attr *config)
{
	struct device *dev;
	struct ubusdev_type *utype;
	int ret;

	utype = container_of(type, struct ubusdev_type, handler);

	dev = calloc(1, sizeof(struct device));

	if (!dev)
		return NULL;

	if (!utype->ubus_peer_id && ubusdev_lookup_id(utype)) {
		ret = UBUS_STATUS_NOT_FOUND;
		goto error;
	}

	ret = device_init(dev, type, name);
	if (ret)
		goto error;

	// let the external device handler set up the device
	ret = netifd_ubusdev_invoke(utype->ubus_peer_id,
		__ubusdev_methods[UBUSDEV_METHOD_CREATE], config, NULL, NULL);
	if (ret)
		goto inv_error;

	// setting this to false avoids calling config_init automatically.
	// config_init is instead called when the external device handler notifies
	// us of successful device creation
	dev->config_pending = false;

	return dev;

inv_error:
	ubusdev_invocation_error(ret, __ubusdev_methods[UBUSDEV_METHOD_CREATE],
		name);
error:
	device_free(dev);
	fprintf(stderr, "Creating %s %s failed: %s\n", type->name, name,
		ubus_strerror(ret));
	return NULL;
}

static const struct device_hotplug_ops ubusdev_ops = {
	.prepare = ubusdev_hotplug_prepare,
	.add = ubusdev_hotplug_add,
	.del = ubusdev_hotplug_del
};

/* Initialize a bridge device stub in netifd.
 */
static struct device*
_ubusdev_bridge_create(const char *name, struct device_type *devtype,
	struct blob_attr *config)
{
	struct ubusdev_bridge *ubr;

	ubr = calloc(1, sizeof(*ubr));
	if (!ubr)
		return NULL;

	device_init(&ubr->dev, devtype, name);
	ubr->dev.config_pending = true;
	ubr->utype = container_of(devtype, struct ubusdev_type, handler);
	ubr->retry.cb = ubusdev_bridge_timeout_cb;

	// for bridge types, the default device state callback is replaced
	// in the device struct but kept in the ubusdev_bridge wrapper struct
	// much like what bridge.c is doing.
	// Also, a copy of the config is stored with the wrapper in case the
	// bridge gets disabled and re-enabled
	ubr->set_state = ubr->dev.set_state;
	ubr->dev.set_state = ubusdev_bridge_set_state;

	ubr->dev.hotplug_ops = &ubusdev_ops;

	vlist_init(&ubr->members, avl_strcmp, ubusdev_bridge_member_update);
	ubr->members.keep_old = true;
	ubusdev_bridge_reload(&ubr->dev, config);

	return &ubr->dev;
}

/* Device creation process with ubus devices:
 * TODO
 */
static struct device *
ubusdev_create(const char *name, struct device_type *devtype,
	struct blob_attr *config)
{
	if (devtype->bridge_capability)
		return _ubusdev_bridge_create(name, devtype, config);
	else
		return _ubusdev_create(name, devtype, config);
}

static void
ubusdev_bridge_free(struct ubusdev_bridge *ubr)
{
	struct ubusdev_type *utype;
	int ret;

	utype = container_of(ubr->dev.type, struct ubusdev_type, handler);

	ret = netifd_ubusdev_invoke(utype->ubus_peer_id,
		__ubusdev_methods[UBUSDEV_METHOD_FREE], ubr->config, NULL, NULL);

	if (ret)
		goto error;

	ubusdev_bridge_set_timeout(EXTERNAL_STATE_SYNC_PENDING_FREE, ubr, 100);

	return;

error:
	ubusdev_invocation_error(ret, __ubusdev_methods[UBUSDEV_METHOD_FREE],
		ubr->dev.ifname);
}

/* Free a device both locally wihtin netifd and externally by invoking
 * 'free' on the external device handler
 */
static void
ubusdev_free(struct device *dev)
{
	struct ubusdev_type *utype;
	struct ubusdev_bridge *ubr;
	int ret;

	if (dev->type->bridge_capability) {
		ubr = container_of(dev, struct ubusdev_bridge, dev);
		ubusdev_bridge_free(ubr);
		return;
	}

	utype = container_of(dev->type, struct ubusdev_type, handler);

	blob_buf_init(&blob_buffer, 0);
	blobmsg_add_string(&blob_buffer, "name", dev->ifname);

	ret = netifd_ubusdev_invoke(utype->ubus_peer_id,
		__ubusdev_methods[UBUSDEV_METHOD_FREE], blob_buffer.head, NULL, NULL);

	if (ret) {
		ubusdev_invocation_error(ret, __ubusdev_methods[UBUSDEV_METHOD_FREE],
			dev->ifname);
		return;
	}
}

/* Set bridge present if it's marked empty or initialize bridge members.
 * TODO: when is ubus create call made for empty bridges?
 */
static void
_ubusdev_bridge_config_init(struct device *dev)
{
	int rem;
	struct blob_attr *cur;
	struct ubusdev_bridge *ubr = container_of(dev, struct ubusdev_bridge, dev);

	if (ubr->empty) {
		ubr->force_active = true;
		device_set_present(dev, true);
	}

	ubr->n_failed = 0;
	vlist_update(&ubr->members);
	if (ubr->ifnames) {
		blobmsg_for_each_attr(cur, ubr->ifnames, rem)
			ubusdev_bridge_add_member(ubr, blobmsg_data(cur));
	}
	vlist_flush(&ubr->members);
}

/* Set a device present. If the device is a bridge type device, allocate
 * state for the mmebers. The create calls for the members will not be issued
 * immediately, though. That is done when the bridge is brought up
 * (ubusdev_bridge_enable_interface).
 */
static void
ubusdev_config_init(struct device *dev)
{
	if (dev->type->bridge_capability)
		_ubusdev_bridge_config_init(dev);
	else
		device_set_present(dev, true);
}

static void
ubusdev_info_buf_add_array(struct blob_attr *array, const char *name,
	struct blob_buf *buf)
{
	int rem;
	struct blob_attr *cur;
	void *list = blobmsg_open_array(buf, name);

	blobmsg_for_each_attr(cur, array, rem)
		blobmsg_add_string(buf, NULL, blobmsg_get_string(cur));
	blobmsg_close_array(buf, list);
}

struct ubusdev_dump_data {
	const struct device *dev;
	struct blob_buf *buf;
};

static void
ubusdev_info_data_cb(struct ubus_request *req, int type, struct blob_attr
		*reply)
{
	struct ubusdev_dump_data *data = req->priv;
	struct ubusdev_type *utype = container_of(data->dev->type, struct
			ubusdev_type, handler);
	const struct blobmsg_policy *info_policy = utype->info_params->params;
	int n_params = utype->info_params->n_params;
	struct blob_buf *buf = data->buf;

	struct blob_attr *tb[n_params];

	blobmsg_parse(info_policy, n_params, tb, blobmsg_data(reply),
			blobmsg_len(reply));

	for (int i = 0; i < n_params; i++) {
		if (!tb[i])
			continue;

		switch (info_policy[i].type) {
			case BLOBMSG_TYPE_STRING:
				blobmsg_add_string(buf, info_policy[i].name,
					blobmsg_get_string(tb[i]));
				break;
			case BLOBMSG_TYPE_ARRAY:
				ubusdev_info_buf_add_array(tb[i], info_policy[i].name, buf);
				break;
			default:
				continue;
		}
	}
}

static void
ubusdev_dump_info(struct device *dev, struct blob_buf *buf)
{
	struct ubusdev_type *utype = container_of(dev->type, struct ubusdev_type,
			handler);
	struct ubusdev_dump_data *data;
	data = malloc(sizeof(struct ubusdev_dump_data));
	if (!data)
		return;

	data->dev = dev;
	data->buf = buf;

	blob_buf_init(&blob_buffer, 0);
	if (dev->type->bridge_capability)
		blobmsg_add_string(&blob_buffer, "bridge", dev->ifname);
	else
		blobmsg_add_string(&blob_buffer, "device", dev->ifname);

	netifd_ubusdev_invoke(utype->ubus_peer_id,
		__ubusdev_methods[UBUSDEV_METHOD_DUMP_INFO], blob_buffer.head,
		ubusdev_info_data_cb, data);

	system_if_dump_info(dev, buf);
}

static void
ubusdev_stats_data_cb(struct ubus_request *req, int type, struct blob_attr
*reply)
{
	struct ubusdev_dump_data *data = req->priv;
	struct ubusdev_type *utype = container_of(data->dev->type, struct
			ubusdev_type, handler);
	const struct blobmsg_policy *stats_policy = utype->stats_params->params;
	int n_params = utype->stats_params->n_params;
	struct blob_buf *buf = data->buf;

	struct blob_attr *tb[n_params];

	blobmsg_parse(stats_policy, n_params, tb, blobmsg_data(reply),
			blobmsg_len(reply));

	for (int i = 0; i < n_params; i++) {
		if (!tb[i])
			continue;

		switch (stats_policy[i].type) {
			case BLOBMSG_TYPE_STRING:
				blobmsg_add_string(buf, stats_policy[i].name,
						blobmsg_get_string(tb[i]));
				break;
			case BLOBMSG_TYPE_ARRAY:
				ubusdev_info_buf_add_array(tb[i], stats_policy[i].name, buf);
				break;
			default:
				continue;
		}
	}
}

static void
ubusdev_dump_stats(struct device *dev, struct blob_buf *buf)
{
	struct ubusdev_type *utype = container_of(dev->type, struct ubusdev_type,
			handler);
	struct ubusdev_dump_data *data;
	data = malloc(sizeof(struct ubusdev_dump_data));
	if (!data)
		return;

	data->dev = dev;
	data->buf = buf;

	blob_buf_init(&blob_buffer, 0);
	if (dev->type->bridge_capability)
		blobmsg_add_string(&blob_buffer, "bridge", dev->ifname);
	else
		blobmsg_add_string(&blob_buffer, "device", dev->ifname);

	netifd_ubusdev_invoke(utype->ubus_peer_id,
			__ubusdev_methods[UBUSDEV_METHOD_DUMP_STATS], blob_buffer.head,
			ubusdev_stats_data_cb, data);
}

/* Called when external device handler signals successful device creation.
 * Mark devices as synced and ready for use.
 */
static int
ubusdev_handle_create_notify(const char **devices, int n_devices)
{
	const char *cur;
	int i, ret;
	struct device *dev;
	struct ubusdev_bridge *ubr;

	for (i = 0, cur = devices[i]; i < n_devices; cur = devices[++i]) {
		dev = device_get(cur, 0);

		if (!dev || dev->type == &simple_device_type)
			continue;

		if (dev->type->bridge_capability) {
			ubr = container_of(dev, struct ubusdev_bridge, dev);

			if (ubr->sync != EXTERNAL_STATE_SYNC_PENDING_CREATE)
				return 0;

			// call preserved set_state callback to bring bridge up
			ret = ubr->set_state(&ubr->dev, true);
			if (ret < 0) {
				ubusdev_bridge_set_down(ubr);
				return ret;
			}

			ubr->active = true;
			ubusdev_bridge_set_sync(ubr, EXTERNAL_STATE_SYNC_OK);
		}
	}

	return 0;
}

static int
ubusdev_handle_update_notify(const char **devices, int n_dev)
{
	// TODO
	return 0;
}

static int
ubusdev_handle_delete_notify(const char **devices, int n_dev)
{
	const char *cur;
	int i;
	struct device *dev;
	struct ubusdev_bridge *ubr;

	for (i = 0, cur = devices[i]; i < n_dev; cur = devices[++i]) {

		dev = device_get(cur, 0);
		if (!dev)
			continue;

		if (dev->type->bridge_capability) {
			ubr = container_of(dev, struct ubusdev_bridge, dev);

			// do not delete device that is not marked for deletion.
			// This means that bridges merely get 'disabled' while their
			// devices and configs are still available.
			if (ubr->sync != EXTERNAL_STATE_SYNC_PENDING_FREE) {
				ubr->active = false;
				ubusdev_bridge_set_sync(ubr, EXTERNAL_STATE_SYNC_OK);
				continue;
			}

			if (ubr->ifnames)
				free(ubr->ifnames);

			if (ubr->config)
				free(ubr->config);

			vlist_flush_all(&ubr->members);
			free(ubr);
		}
	}
	return 0;
}

/* Called as part of the subscription to the external device handler's
 * ubus object.
 * TODO description (distinguish real hotplug and add-port)
 */
static int
ubusdev_handle_hotplug_add_notify(const char **devices, int n_dev)
{
	struct device *bridge, *member_dev;
	struct ubusdev_bridge_member *member;
	struct ubusdev_bridge *ubr;

	if (n_dev != 2)
		return UBUS_STATUS_INVALID_ARGUMENT;

	bridge = device_get(devices[0], 0);
	if (!bridge)
		return UBUS_STATUS_INVALID_ARGUMENT;

	member_dev = device_get(devices[1], 0);
	if (!member_dev)
		return UBUS_STATUS_NOT_FOUND;

	ubr = container_of(bridge, struct ubusdev_bridge, dev);

	// If the member is already present in the members list of the bridge,
	// it means that this notification is happening because the member has
	// not been added via hotplug add. This member has to be activated rather
	// than created.
	// Correspondingly, if the member does not exist in the bridge, create it
	// and unlock the devices list for device_free_unused(), again.
	member = vlist_find(&ubr->members, devices[1], member, node);
	if (!member) {
		ubusdev_bridge_create_member(ubr, member_dev, true);
		device_unlock();
		return 0;
	}

	member->sync = EXTERNAL_STATE_SYNC_OK;

	device_set_present(&ubr->dev, true);
	device_broadcast_event(&ubr->dev, DEV_EVENT_TOPO_CHANGE);

	return 0;
}

static int
ubusdev_handle_hotplug_remove_notify(const char **devices, int n_dev)
{
	struct device *bridge;
	struct ubusdev_bridge_member *member;
	struct ubusdev_bridge *ubr;

	if (n_dev != 2)
		return UBUS_STATUS_INVALID_ARGUMENT;

	bridge = device_get(devices[0], 0);
	if (!bridge)
		return UBUS_STATUS_INVALID_ARGUMENT;

	ubr = container_of(bridge, struct ubusdev_bridge, dev);

	member = vlist_find(&ubr->members, devices[1], member, node);
	if (!member)
		return UBUS_STATUS_INVALID_ARGUMENT;

	member->sync = EXTERNAL_STATE_SYNC_OK;

	device_release(&member->dev_usr);
	device_broadcast_event(&ubr->dev, DEV_EVENT_TOPO_CHANGE);

	return 0;
}

/* Called as part of the subscription to the external device handler's
 * ubus object.
 * Dispatch appropriate handler for specific event.
 */
static int
ubusdev_handle_notify(struct ubus_context *ctx, struct ubus_object *obj,
	struct ubus_request_data *req, const char *type, struct blob_attr *msg)
{
	static struct blobmsg_policy pol = {
		.name = "devices",
		.type = BLOBMSG_TYPE_ARRAY
	};

	const char **devices;
	struct blob_attr *cur, *tb[1];
	int cnt, rem, i = 0, ret = 0;

	// parse devices
	blobmsg_parse(&pol, 1, tb, blobmsg_data(msg), blobmsg_len(msg));
	if (!tb[0]) {
		ret = UBUS_STATUS_INVALID_ARGUMENT;
		goto done;
	}

	cnt = blobmsg_check_array(tb[0], BLOBMSG_TYPE_STRING);
	if (cnt == -1) {
		ret = UBUS_STATUS_INVALID_ARGUMENT;
		goto done;
	}

	devices = calloc(cnt, sizeof(char*));
	if (!devices) {
		free(msg);
		free(req);
		return -ENOMEM;
	}

	blobmsg_for_each_attr(cur, tb[0], rem)
		devices[i++] = blobmsg_get_string(cur);

	if (!strcmp(type, "create"))
		ret = ubusdev_handle_create_notify(devices, cnt);
	else if (!strcmp(type, "update"))
		ret = ubusdev_handle_update_notify(devices, cnt);
	else if (!strcmp(type, "delete"))
		ret = ubusdev_handle_delete_notify(devices, cnt);
	else if (!strcmp(type, "add"))
		ret = ubusdev_handle_hotplug_add_notify(devices, cnt);
	else if (!strcmp(type, "remove"))
		ret = ubusdev_handle_hotplug_remove_notify(devices, cnt);

	free(devices);

done:
	if (ret)
		fprintf(stderr, "%s notification failed: %s\n", type,
			ubus_strerror(ret));

	free(req);
	return ret;
}

static void
ubusdev_ext_handler_remove_cb(struct ubus_context *ctx,
	struct ubus_subscriber *obj, uint32_t id)
{
	struct ubusdev_type *utype;
	utype = container_of(obj, struct ubusdev_type, ubus_sub);

	utype->ubus_peer_id = 0;
	utype->subscribed = false;

	return;
}

/* Add a new device type struct for a ubus device class.
 */
static void
ubusdev_add_devtype(const char *cfg_file, const char *tname,
	const char *ubus_name, bool bridge_capability,
	const char *br_prefix, json_object *obj)
{
	struct ubusdev_type *utype;
	struct device_type *devtype;
	json_object *cfg;
	char *ubus_obj_name, *devtype_name, *ext_dev_handler_name, *name_prefix;
	struct uci_blob_param_list *config_params, *info_params, *stats_params;
	int ret;

	utype = calloc_a(sizeof(*utype),
		&ubus_obj_name, strlen(UBUSDEV_UBUSOBJ_NAME_PREFIX) +
			strlen(ubus_name) + 1,
		&devtype_name, strlen(tname) + 1,
		&ext_dev_handler_name, strlen(ubus_name) + 1,
		&config_params, sizeof(struct uci_blob_param_list),
		&info_params, sizeof(struct uci_blob_param_list),
		&stats_params, sizeof(struct uci_blob_param_list));
	if (!utype)
		return;

	utype->config_params = config_params;
	utype->info_params = info_params;
	utype->ext_dev_handler_name = strcpy(ext_dev_handler_name, ubus_name);

	devtype = &utype->handler;
	devtype->name = strcpy(devtype_name, tname);
	
	devtype->create = ubusdev_create;
	devtype->free = ubusdev_free;
	devtype->config_init = ubusdev_config_init;
	devtype->reload = ubusdev_reload;
	devtype->dump_info = ubusdev_dump_info;
	devtype->dump_stats = ubusdev_dump_stats;

	devtype->bridge_capability = bridge_capability;
	devtype->config_params = utype->config_params;

	if (bridge_capability) {
		name_prefix = malloc(strlen(br_prefix) + 1);
		if (!name_prefix)
			goto error;

		strcpy(name_prefix, br_prefix);
		devtype->name_prefix = name_prefix;
	}

	// prepare and register ubus object
	sprintf(ubus_obj_name, UBUSDEV_UBUSOBJ_NAME_PREFIX "%s", ubus_name);
	utype->ubus_sub.obj.name = ubus_obj_name;
	utype->ubus_sub.obj.type = &ubusdev_ubus_object_type;
	utype->ubus_sub.obj.n_methods = ARRAY_SIZE(ubusdev_object_methods);
	//netifd_add_object(&utype->ubus_sub.obj);
	ret = ubus_register_subscriber(ubus_ctx, &utype->ubus_sub);
	if (ret)
		fprintf(stderr, "Failed to register subscriber object '%s'\n",
			utype->ubus_sub.obj.name);

	// look for remote ubus object
	ubusdev_lookup_id(utype);

	// prepare subscription and subscribe to peer object
	utype->ubus_sub.cb = ubusdev_handle_notify;
	utype->ubus_sub.remove_cb = ubusdev_ext_handler_remove_cb;
	ret = ubus_subscribe(ubus_ctx, &utype->ubus_sub,
		utype->ubus_peer_id);
	if (ret)
		utype->subscribed = false;
	else
		utype->subscribed = true;

	// parse and store config format description
	cfg = json_get_field(obj, "config", json_type_array);
	if (!cfg)
		goto error;

	utype->config_strbuf = netifd_handler_parse_config(
		utype->config_params, cfg);
	if (!utype->config_strbuf)
		goto error;

	// Parse and store info dump format description.
	// If none is given, do not set 'dump_info' handler
	cfg = json_get_field(obj, "info", json_type_array);
	if (!cfg) {
		devtype->dump_info = NULL;
	} else {
		utype->info_strbuf = netifd_handler_parse_config(utype->info_params,
				cfg);

		if (!utype->info_strbuf)
			devtype->dump_info = NULL;
	}

	// Parse and store statistics dump format description.
	// If none is present, do not set 'dump_statistics' handler.
	cfg = json_get_field(obj, "stats", json_type_array);
	if (!cfg) {
		devtype->dump_stats = NULL;
	} else {
		utype->stats_strbuf = netifd_handler_parse_config(utype->stats_params,
				cfg);

		if (!utype->stats_strbuf)
			devtype->dump_stats = NULL;
	}

	// add device type to device type list
	ret = device_type_add(devtype);
	if (ret)
		goto config_error;

	return;

config_error:
	free(utype->config_strbuf);
	free(utype->info_strbuf);
	free(utype->stats_strbuf);

error:
	fprintf(stderr, "Failed to create device handler for device"
		"type '%s' from file '%s'\n", tname, cfg_file);
	free(ubus_obj_name);
	free(devtype_name);
	free(utype);
}

/* parse JSON metadata for ubusdev types
 */
void
ubusdev_init(void)
{
	confdir_fd = netifd_open_subdir("ubusdev-config");
	if (confdir_fd < 0)
		return;
	netifd_init_ubusdev_handlers(confdir_fd, ubusdev_add_devtype);
}
