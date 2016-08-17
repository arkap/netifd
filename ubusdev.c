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
#define UBUSDEV_MAX_RETRY_CNT 3
#define UBUSDEV_TIMEOUT 1000

extern struct ubus_context *ubus_ctx;
static struct blob_buf blob_buffer;
static int confdir_fd = -1;

enum state_sync {
	// device handler interface
	STATE_SYNCHRONIZED,
	STATE_PENDING_CREATE,
	STATE_PENDING_RELOAD,
	STATE_PENDING_DISABLE,
	STATE_PENDING_FREE,
	STATE_PENDING_CONFIG_INIT,

	// hotplug ops
	STATE_PENDING_PREPARE,
	STATE_PENDING_ADD,
	STATE_PENDING_REMOVE,
};

// wrapper for ubus device type structs
struct ubusdev_type {
	struct device_type handler;

	const char *ext_dev_handler_name;
	uint32_t ubus_peer_id;
	struct ubus_subscriber ubus_sub;
	bool subscribed;
	struct ubus_event_handler obj_wait;

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

// wrapper for ubus devices
struct ubusdev_device {
	struct device dev;

	struct ubusdev_type *utype;

	struct ubus_request req;

	// synchronization fields
	enum state_sync sync;
	struct uloop_timeout retry;
	int retry_cnt;
};

// wrapper for device structs with bridge capability
struct ubusdev_bridge {
	struct ubusdev_device udev;
	device_state_cb set_state;

	struct blob_attr *config;
	// the members' names
	bool empty;
	struct blob_attr *ifnames;
	bool active;
	bool force_active;

	struct vlist_tree members;
	int n_present;
	int n_failed;
};

struct ubusdev_bridge_member {
	struct vlist_node node;
	struct ubusdev_bridge *parent_br;
	struct device_user dev_usr;
	bool present;
	bool hotplug;
	char *name;

	struct ubus_request req;

	enum state_sync sync;
	struct uloop_timeout retry;
	int retry_cnt;
};

static void
ubusdev_bridge_retry_enable_members(struct ubusdev_bridge *ubr);

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
	UBUSDEV_METHOD_HOTPLUG_PREPARE,
	UBUSDEV_METHOD_HOTPLUG_ADD,
	UBUSDEV_METHOD_HOTPLUG_REMOVE,
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
	[UBUSDEV_METHOD_HOTPLUG_PREPARE] = "prepare",
	[UBUSDEV_METHOD_HOTPLUG_ADD] = "add",
	[UBUSDEV_METHOD_HOTPLUG_REMOVE] = "remove",
};

static void
ubusdev_invocation_error(int error, const char *method, const char *devname)
{
	fprintf(stderr, "invocation of method '%s' failed for device '%s': %s\n",
		method, devname, ubus_strerror(error));
	netifd_log_message(L_CRIT, "invocation of method '%s' failed for device "
		"'%s': %s\n", method, devname, ubus_strerror(error));
}

static struct ubus_method ubusdev_ubus_obj_methods[] = {};

static struct ubus_object_type ubusdev_ubus_object_type =
	UBUS_OBJECT_TYPE("netifd_ubusdev", ubusdev_ubus_obj_methods);

/* Lookup the ubus ID of the external device handler's ubus object for a ubus
 * device type.
 */
static int
ubusdev_lookup_id(struct ubusdev_type *utype)
{
	int ret = UBUS_STATUS_UNKNOWN_ERROR;

	if (!utype || !utype->ext_dev_handler_name)
		goto error;

	ret = ubus_lookup_id(ubus_ctx, utype->ext_dev_handler_name,
		&utype->ubus_peer_id);

	if (ret)
		goto error;

	return 0;

error:
	fprintf(stderr, "Could not find ubus ID for object '%s': %s\n",
		utype->ext_dev_handler_name, ubus_strerror(ret));
	return ret;
}

static int
ubusdev_ext_ubus_obj_wait(struct ubus_event_handler *handler)
{
	return ubus_register_event_handler(ubus_ctx, handler, "ubus.object.add");
}

static int
ubusdev_subscribe(struct ubusdev_type *utype)
{
	int ret;

	// look for remote ubus object
	ret = ubusdev_lookup_id(utype);
	if (ret) {
		utype->subscribed = false;
		return ret;
	}

	ret = ubus_subscribe(ubus_ctx, &utype->ubus_sub, utype->ubus_peer_id);
	if (ret) {
		utype->subscribed = false;
		ubusdev_ext_ubus_obj_wait(&utype->obj_wait);
	} else {
		netifd_log_message(L_NOTICE, "subscribed to external device handler "
			"at '%s'\n", utype->ext_dev_handler_name);
		utype->subscribed = true;
	}

	return ret;
}

static void
ubusdev_wait_ev_cb(struct ubus_context *ctx,
	struct ubus_event_handler *ev_handler, const char *type,
	struct blob_attr *msg)
{
	static const struct blobmsg_policy wait_policy = {
		"path", BLOBMSG_TYPE_STRING
	};

	struct blob_attr *attr;
	const char *path;
	struct ubusdev_type *utype = container_of(ev_handler, struct ubusdev_type,
		obj_wait);

	if (strcmp(type, "ubus.object.add"))
		return;

	blobmsg_parse(&wait_policy, 1, &attr, blob_data(msg), blob_len(msg));
	if (!attr)
		return;

	path = blobmsg_data(attr);
	if (strcmp(utype->ext_dev_handler_name, path))
		return;

	ubusdev_subscribe(utype);
}

static bool
ubusdev_check_subscribed(const struct ubusdev_type *utype, const char *action)
{
	if (!utype->subscribed) {
		fprintf(stderr, "%s: Not subscribed to external device handler. Cannot "
			"execute action '%s' until it re-appears and subscription is "
			"renewed.\n", utype->handler.name, action);

		netifd_log_message(L_WARNING, "%s: Not subscribed to external device"
			" handler. Cannot execute action '%s' until it re-appears and "
			"subscription is renewed.\n", utype->handler.name, action);
	}

	return utype->subscribed;
}

static void
ubusdev_set_sync(struct ubusdev_device *udev, enum state_sync status)
{
	struct ubusdev_bridge *ubr;

	udev->sync = status;
	if (status == STATE_SYNCHRONIZED)
		uloop_timeout_cancel(&udev->retry);

	// when setting bridge synced, cancel retry timers and bring
	// members up
	if (udev->dev.type->bridge_capability) {
		ubr = container_of(udev, struct ubusdev_bridge, udev);
		if (ubr->n_failed)
			ubusdev_bridge_retry_enable_members(ubr);
	}

}

static void
ubusdev_set_timeout(struct ubusdev_device *udev, enum state_sync state,
	int ms)
{
	ubusdev_set_sync(udev, state);
	uloop_timeout_set(&udev->retry, ms);
}

static void
ubusdev_bridge_member_set_sync(struct ubusdev_bridge_member *ubm,
	enum state_sync status)
{
	ubm->sync = status;

	if (status == STATE_SYNCHRONIZED)
		uloop_timeout_cancel(&ubm->retry);
}

static void
ubusdev_bridge_member_set_timeout(struct ubusdev_bridge_member *ubm,
	enum state_sync status, int ms)
{
	ubusdev_bridge_member_set_sync(ubm, status);
	uloop_timeout_set(&ubm->retry, ms);
}

static void
ubusdev_member_req_complete_cb(struct ubus_request *req, int ret)
{
	struct ubusdev_bridge_member *ubm;

	if (!ret)
		return;

	ubm = container_of(req, struct ubusdev_bridge_member, req);

	netifd_log_message(L_CRIT, "External device handler returned error "
		"concerning bridge member %s: %s\n", ubm->name, ubus_strerror(ret));
}

/* Data callback for messages from external device handler.
 * These can provide valuable information for the user e.g. about errors.
 */
static void
ubusdev_req_data_cb(struct ubus_request *req, int type, struct blob_attr *msg)
{
	static struct blobmsg_policy policy = {
		.name = "message",
		.type = BLOBMSG_TYPE_STRING,
	};

	struct blob_attr *tb;
	struct ubusdev_device *udev;

	blobmsg_parse(&policy, 1, &tb, blobmsg_data(msg), blobmsg_len(msg));

	if (!tb)
		return;

	udev = container_of(req, struct ubusdev_device, req);

	netifd_log_message(L_NOTICE, "Message from external device handler of %s "
		"%s: [%s]\n", udev->dev.type->name, udev->dev.ifname,
		blobmsg_get_string(tb));
}

static void
ubusdev_req_complete_cb(struct ubus_request *req, int ret)
{
	struct ubusdev_device *udev;

	if (!ret)
		return;

	udev = container_of(req, struct ubusdev_device, req);

	netifd_log_message(L_CRIT, "External device handler returned error "
		"concerning %s %s: %s\n", udev->dev.type->name, udev->dev.ifname,
		ubus_strerror(ret));
}

/* Delte bridge at external device handler but keep state in netifd.
 */
static int
ubusdev_bridge_disable_interface(struct ubusdev_bridge *ubr)
{
	int ret;

	blob_buf_init(&blob_buffer, 0);
	blobmsg_add_string(&blob_buffer, "name", ubr->udev.dev.ifname);

	ret = netifd_ubusdev_invoke_async(&ubr->udev.req,
		ubr->udev.utype->ubus_peer_id, __ubusdev_methods[UBUSDEV_METHOD_FREE],
		blob_buffer.head, ubusdev_req_data_cb, ubusdev_req_complete_cb, NULL);

	if (ret)
		goto error;

	ubusdev_set_timeout(&ubr->udev, STATE_PENDING_DISABLE, UBUSDEV_TIMEOUT);

	return 0;

error:
	ubusdev_invocation_error(ret, __ubusdev_methods[UBUSDEV_METHOD_FREE],
		ubr->udev.dev.ifname);
	return ret;
}

static int
ubusdev_bridge_enable_member(struct ubusdev_bridge_member *ubm)
{
	int ret;
	struct ubusdev_bridge *ubr = ubm->parent_br;

	if (!ubm->present)
		return 0;

	ret = device_claim(&ubm->dev_usr);
	if (ret < 0)
		goto error;

	if (!ubr->udev.dev.present || ubr->udev.sync != STATE_SYNCHRONIZED)
		goto error;

	blob_buf_init(&blob_buffer, 0);
	blobmsg_add_string(&blob_buffer, "bridge", ubr->udev.dev.ifname);
	blobmsg_add_string(&blob_buffer, "member", ubm->dev_usr.dev->ifname);

	// abuse hotplug add as addif equivalent. Maybe we need a dedicated ubus
	// method on the external handler for this sort of operation.
	ret = netifd_ubusdev_invoke_async(&ubm->req, ubr->udev.utype->ubus_peer_id,
		__ubusdev_methods[UBUSDEV_METHOD_HOTPLUG_ADD], blob_buffer.head, NULL,
		ubusdev_member_req_complete_cb, NULL);

	ubusdev_bridge_member_set_timeout(ubm, STATE_PENDING_ADD, UBUSDEV_TIMEOUT);

	if (ret)
		goto inv_error;

	return 0;

inv_error:
	ubusdev_invocation_error(ret,
		__ubusdev_methods[UBUSDEV_METHOD_HOTPLUG_ADD],
		ubm->dev_usr.dev->ifname);

error:
	netifd_log_message(L_DEBUG, "\terror!\n");
	ubr->n_failed++;
	ubm->present = false;
	ubm->sync = STATE_PENDING_ADD;
	ubr->n_present--;

	return ret;
}

/* Disable a bridge member.
 * At the moment, hotplug remove is used to remove the member at the external
 * device handler. If that clashes with future requirements, maybe we'll have to
 * add methods for disabling members.
 */
static int
ubusdev_bridge_disable_member(struct ubusdev_bridge_member *ubm)
{
	int ret;
	struct ubusdev_bridge *ubr = ubm->parent_br;

	if (!ubm->present)
		return 0;

	blob_buf_init(&blob_buffer, 0);
	blobmsg_add_string(&blob_buffer, "bridge", ubr->udev.dev.ifname);
	blobmsg_add_string(&blob_buffer, "member", ubm->dev_usr.dev->ifname);

	// abuse hotplug remove as delif equivalent. Maybe we need a dedicated ubus
	// method on the external handler for this sort of operation.
	ret = netifd_ubusdev_invoke_async(&ubm->req, ubr->udev.utype->ubus_peer_id,
		__ubusdev_methods[UBUSDEV_METHOD_HOTPLUG_REMOVE], blob_buffer.head,
		NULL, ubusdev_member_req_complete_cb, NULL);

	if (ret)
		goto error;

	ubusdev_bridge_member_set_timeout(ubm, STATE_PENDING_REMOVE,
		UBUSDEV_TIMEOUT);

	return 0;

error:
	ubusdev_invocation_error(ret,
		__ubusdev_methods[UBUSDEV_METHOD_HOTPLUG_REMOVE],
		ubm->dev_usr.dev->ifname);

	return ret;
}

static int
ubusdev_bridge_set_down(struct ubusdev_bridge *ubr)
{
	struct ubusdev_bridge_member *ubm;

	ubr->set_state(&ubr->udev.dev, false);

	vlist_for_each_element(&ubr->members, ubm, node)
		ubusdev_bridge_disable_member(ubm);

	ubusdev_bridge_disable_interface(ubr);

	return 0;
}

static int
ubusdev_bridge_set_up(struct ubusdev_bridge *ubr)
{
	struct ubusdev_bridge_member *ubm;

	if (!ubr->n_present)
		if (!ubr->force_active)
			return -ENOENT;

	ubr->n_failed = 0;
	vlist_for_each_element(&ubr->members, ubm, node)
		ubusdev_bridge_enable_member(ubm);

	if (!ubr->force_active && !ubr->n_present) {
		// initialization of all member interfaces failed
		ubusdev_bridge_disable_interface(ubr);
		device_set_present(&ubr->udev.dev, false);
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

	 ubr = container_of(dev, struct ubusdev_bridge, udev.dev);

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

	if (ubr->udev.dev.active)
		ubusdev_bridge_disable_member(member);

	member->present = false;
	ubr->n_present--;

	if (ubr->n_present == 0)
		device_set_present(&ubr->udev.dev, false);
}

static void
ubusdev_bridge_member_cb(struct device_user *usr, enum device_event event)
{
	struct ubusdev_bridge_member *ubm =
		container_of(usr, struct ubusdev_bridge_member, dev_usr);
	struct ubusdev_bridge *ubr = ubm->parent_br;


	switch (event) {
		case DEV_EVENT_ADD:

			ubm->present = true;
			ubr->n_present++;

			// if this member is the first one that is brought up, create the
			// bridge at the external device handler
			if (ubr->n_present == 1) {
				netifd_ubusdev_invoke_async(&ubr->udev.req,
					ubr->udev.utype->ubus_peer_id,
					__ubusdev_methods[UBUSDEV_METHOD_CREATE], ubr->config,
					NULL, ubusdev_req_complete_cb, NULL);

				ubusdev_set_timeout(&ubr->udev, STATE_PENDING_CREATE,
					UBUSDEV_TIMEOUT);
			} else {
				ubusdev_bridge_enable_member(ubm);
			}
			break;

		case DEV_EVENT_REMOVE:
			if (usr->hotplug) {
				vlist_delete(&ubr->members, &ubm->node);
				return;
			}

			if (ubm->present)
				ubusdev_bridge_remove_member(ubm);
			break;
		default:
			break;
	}
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

		// don't make the call to the external device handler if
		// state is sync'ed
		if (cur->sync == STATE_SYNCHRONIZED)
			continue;

		cur->present = true;
		ubr->n_present++;
		ubusdev_bridge_enable_member(cur);
	}
}

static void
ubusdev_bridge_member_timeout_cb(struct uloop_timeout *timeout)
{
	int ret;
	struct ubusdev_bridge_member *ubm = container_of(timeout,
		struct ubusdev_bridge_member, retry);

	if ((ubm->retry_cnt++) == UBUSDEV_MAX_RETRY_CNT) {
		netifd_log_message(L_CRIT, "%s: no state sync with external "
				"device handler after %d retries. Giving up.\n",
			ubm->dev_usr.dev->ifname, UBUSDEV_MAX_RETRY_CNT);

		device_release(&ubm->dev_usr);
		return;
	}

	switch (ubm->sync) {
		case STATE_PENDING_ADD:
			ubusdev_bridge_retry_enable_members(ubm->parent_br);
			break;
		case STATE_PENDING_REMOVE:
			blob_buf_init(&blob_buffer, 0);
			blobmsg_add_string(&blob_buffer, "bridge",
				ubm->parent_br->udev.dev.ifname);
			blobmsg_add_string(&blob_buffer, "member", ubm->name);

			ret = netifd_ubusdev_invoke_async(&ubm->req,
				ubm->parent_br->udev.utype->ubus_peer_id,
				__ubusdev_methods[UBUSDEV_METHOD_HOTPLUG_REMOVE],
				blob_buffer.head, NULL, ubusdev_member_req_complete_cb, NULL);

			if (ret) {
				ubusdev_invocation_error(ret,
					__ubusdev_methods[UBUSDEV_METHOD_HOTPLUG_REMOVE],
					ubm->name);
			} else {
				uloop_timeout_set(timeout, UBUSDEV_TIMEOUT);
			}
			break;
		default:
			break;
	}
}

/* Add member 'dev' to bridge.
 */
static struct ubusdev_bridge_member *
ubusdev_bridge_create_member(struct ubusdev_bridge *ubr, struct device *dev,
	bool hotplug)
{
	struct ubusdev_bridge_member *ubm;
	char *name;

	ubm = calloc_a(sizeof(*ubm), &name, strlen(dev->ifname) + 1);
	if (!ubm)
		return NULL;

	ubm->parent_br = ubr;
	ubm->name = name;
	ubm->hotplug = hotplug;
	strcpy(name, dev->ifname);
	ubm->retry.cb = ubusdev_bridge_member_timeout_cb;
	ubm->dev_usr.dev = dev;
	ubm->dev_usr.cb = ubusdev_bridge_member_cb;
	ubm->sync = STATE_PENDING_ADD;
	vlist_add(&ubr->members, &ubm->node, ubm->name);
	// Need to look up the bridge member again as the above
	// created pointer will be freed in case the bridge member
	// already existed
	ubm = vlist_find(&ubr->members, dev->ifname, ubm, node);
	if (!ubm)
		return NULL;

	return ubm;
}

static void
ubusdev_bridge_add_member(struct ubusdev_bridge *ubr, const char *name)
{
	struct device *dev;

	dev = device_get(name, 1);
	if (!dev)
		return;

	ubusdev_bridge_create_member(ubr, dev, false);
}

static int
ubusdev_hotplug_add(struct device *ubr_dev, struct device *ubm_dev)
{
	struct ubusdev_bridge *ubr;
	struct ubusdev_bridge_member *ubm;

	if (!ubr_dev->type->bridge_capability)
		return UBUS_STATUS_NOT_SUPPORTED;

	ubr = container_of(ubr_dev, struct ubusdev_bridge, udev.dev);

	if (!ubusdev_check_subscribed(ubr->udev.utype,
		__ubusdev_methods[UBUSDEV_METHOD_HOTPLUG_ADD]))
		return UBUS_STATUS_NOT_FOUND;

	// create bridge member structure
	ubm = ubusdev_bridge_create_member(ubr, ubm_dev, true);
	if (!ubm)
		return UBUS_STATUS_UNKNOWN_ERROR;

	return 0;
}

static int
ubusdev_hotplug_remove(struct device *dev, struct device *member)
{
	struct ubusdev_bridge *ubr;
	struct ubusdev_bridge_member *ubm;

	if (!dev->type->bridge_capability)
		return UBUS_STATUS_NOT_SUPPORTED;

	ubr = container_of(dev, struct ubusdev_bridge, udev.dev);

	if (!ubusdev_check_subscribed(ubr->udev.utype,
		__ubusdev_methods[UBUSDEV_METHOD_HOTPLUG_REMOVE]))
		return UBUS_STATUS_NOT_FOUND;

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
	int ret;

	if (!dev->type->bridge_capability)
		return UBUS_STATUS_NOT_SUPPORTED;

	ubr = container_of(dev, struct ubusdev_bridge, udev.dev);

	blob_buf_init(&blob_buffer, 0);
	blobmsg_add_string(&blob_buffer, "bridge", dev->ifname);

	ret = netifd_ubusdev_invoke_async(&ubr->udev.req,
		ubr->udev.utype->ubus_peer_id,
		__ubusdev_methods[UBUSDEV_METHOD_HOTPLUG_PREPARE], blob_buffer.head,
		ubusdev_req_data_cb, ubusdev_req_complete_cb, NULL);
	if (ret)
		goto error;

	ubusdev_set_timeout(&ubr->udev, STATE_PENDING_PREPARE,
		UBUSDEV_TIMEOUT);

	return 0;

	error:
	ubusdev_invocation_error(ret,
		__ubusdev_methods[UBUSDEV_METHOD_HOTPLUG_PREPARE], dev->ifname);
	return ret;
}

static void
ubusdev_bridge_free_member(struct ubusdev_bridge_member *ubm)
{
	struct device *dev = ubm->dev_usr.dev;

	ubusdev_bridge_remove_member(ubm);
	device_remove_user(&ubm->dev_usr);

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

	free(ubm);
}

/* Called whenever a node is inserted into or removed from the members vlist of
 * ubusdev_bridge structs.
 */
static void
ubusdev_bridge_member_update(struct vlist_tree *tree,
	struct vlist_node *node_new, struct vlist_node *node_old)
{
	struct ubusdev_bridge_member *ubm;
	struct device *dev;

	if (node_new) {
		ubm = container_of(node_new, struct ubusdev_bridge_member, node);

		// don't allow replacements
		if (node_old) {
			free(ubm);
			return;
		}

		// clear device_user fields and set new member
		dev = ubm->dev_usr.dev;
		ubm->dev_usr.dev = NULL;
		device_add_user(&ubm->dev_usr, dev);
	}

	if (node_old) {
		ubm = container_of(node_old, struct ubusdev_bridge_member, node);
		ubusdev_bridge_free_member(ubm);
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

	ubr = container_of(dev, struct ubusdev_bridge, udev.dev);
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

		inv_ret = netifd_ubusdev_invoke_async(&ubr->udev.req,
			utype->ubus_peer_id, __ubusdev_methods[UBUSDEV_METHOD_RELOAD],
			config, ubusdev_req_data_cb, ubusdev_req_complete_cb, NULL);

		if (inv_ret) {
			fprintf(stderr, "Failed to finish config reload for device '%s'."
				"Ubus call to external device handler failed: %s\n",
				dev->ifname, ubus_strerror(inv_ret));

			free(config);
			return DEV_CONFIG_NO_CHANGE;
		}

		ubusdev_set_timeout(&ubr->udev, STATE_PENDING_RELOAD, UBUSDEV_TIMEOUT);

		free(ubr->config);
	}

	ubr->config = config;
	return ret;
}

static enum dev_change_type
_ubusdev_reload(struct device *dev, struct blob_attr *config)
{
	int inv_ret;
	enum dev_change_type ret = DEV_CONFIG_NO_CHANGE;
	unsigned long diff = 0;
	struct ubusdev_device *udev = container_of(dev, struct ubusdev_device, dev);

	struct blob_attr *tb[udev->utype->config_params->n_params];
	struct blob_attr *old_tb[udev->utype->config_params->n_params];

	blobmsg_parse(udev->utype->config_params->params,
		udev->utype->config_params->n_params,
		tb, blobmsg_data(config), blobmsg_len(config));
	blobmsg_parse(udev->utype->config_params->params,
		udev->utype->config_params->n_params,
		old_tb, blobmsg_data(dev->config), blobmsg_len(dev->config));

	uci_blob_diff(tb, old_tb, udev->utype->config_params, &diff);
	if (!diff)
		return DEV_CONFIG_NO_CHANGE;

	device_set_present(dev, false);

	ret = DEV_CONFIG_RESTART;
	inv_ret = netifd_ubusdev_invoke_async(&udev->req, udev->utype->ubus_peer_id,
		__ubusdev_methods[UBUSDEV_METHOD_RELOAD], config, ubusdev_req_data_cb,
		ubusdev_req_complete_cb, NULL);

	if (inv_ret) {
		fprintf(stderr, "Failed to reload config for '%s': %s\n", dev->ifname,
			ubus_strerror(inv_ret));
		return DEV_CONFIG_NO_CHANGE;
	}

	ubusdev_set_timeout(udev, STATE_PENDING_RELOAD, UBUSDEV_TIMEOUT);

	return ret;
}

static enum dev_change_type
ubusdev_reload(struct device *dev, struct blob_attr *config)
{
	struct ubusdev_type *utype = container_of(dev->type, struct ubusdev_type,
		handler);

	if (!ubusdev_check_subscribed(utype,
		__ubusdev_methods[UBUSDEV_METHOD_RELOAD]))
		return DEV_CONFIG_NO_CHANGE;

	if (dev->type->bridge_capability)
		return ubusdev_bridge_reload(dev, config);
	else
		return _ubusdev_reload(dev, config);
}

static void
ubusdev_timeout_cb(struct uloop_timeout *timeout)
{
	int ret;
	const char *method;
	struct blob_attr *attr;
	struct ubusdev_device *udev = container_of(timeout, struct ubusdev_device,
		retry);

	if ((udev->retry_cnt++) == UBUSDEV_MAX_RETRY_CNT) {
		netifd_log_message(L_CRIT, "%s: no state sync with external "
				"device handler after %d retries. Giving up.\n",
			udev->dev.ifname, UBUSDEV_MAX_RETRY_CNT);
		return;
	}

	switch (udev->sync) {
		case STATE_PENDING_CREATE:
			method = __ubusdev_methods[UBUSDEV_METHOD_CREATE];
			attr = udev->dev.config;
			break;
		case STATE_PENDING_RELOAD:
			method = __ubusdev_methods[STATE_PENDING_RELOAD];
			attr = udev->dev.config;
			break;
		case STATE_PENDING_FREE:
			method = __ubusdev_methods[UBUSDEV_METHOD_FREE];
			blob_buf_init(&blob_buffer, 0);
			blobmsg_add_string(&blob_buffer, "name", udev->dev.ifname);
			attr = blob_buffer.head;
			break;
		default:
			return;
	}

	ret = netifd_ubusdev_invoke_async(&udev->req, udev->utype->ubus_peer_id,
		method, attr, ubusdev_req_data_cb, ubusdev_req_complete_cb, NULL);

	if (ret)
		ubusdev_invocation_error(ret, method, udev->dev.ifname);

	uloop_timeout_set(timeout, UBUSDEV_TIMEOUT);
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
		udev.retry);

	if ((ubr->udev.retry_cnt++) == UBUSDEV_MAX_RETRY_CNT) {
		netifd_log_message(L_CRIT, "%s: no state sync with external "
			"device handler after %d retries. Giving up.\n",
			ubr->udev.dev.ifname, UBUSDEV_MAX_RETRY_CNT);
		return;
	}

	// In case the external device handler has failed to notify us
	// after UBUSDEV_TIMEOUT ms, retry the call.
	// If the external device handler has notified us of success, however, we
	// re-initiate the setup of the bridge members for active bridges.
	switch (ubr->udev.sync) {
		case STATE_PENDING_CREATE:
			method = __ubusdev_methods[UBUSDEV_METHOD_CREATE];
			attr = ubr->config;
			break;
		case STATE_PENDING_RELOAD:
			method = __ubusdev_methods[UBUSDEV_METHOD_RELOAD];
			attr = ubr->config;
			break;
		case STATE_PENDING_DISABLE:
		case STATE_PENDING_FREE:
			method = __ubusdev_methods[UBUSDEV_METHOD_FREE];
			blob_buf_init(&blob_buffer, 0);
			blobmsg_add_string(&blob_buffer, "name", ubr->udev.dev.ifname);
			attr = blob_buffer.head;
			break;
		case STATE_PENDING_PREPARE:
			method = __ubusdev_methods[UBUSDEV_METHOD_HOTPLUG_PREPARE];
			blob_buf_init(&blob_buffer, 0);
			blobmsg_add_string(&blob_buffer, "bridge", ubr->udev.dev.ifname);
			attr = blob_buffer.head;
			break;
		default:
			return;
	}

	ret = netifd_ubusdev_invoke_async(&ubr->udev.req,
		ubr->udev.utype->ubus_peer_id, method, attr, ubusdev_req_data_cb,
		ubusdev_req_complete_cb, NULL);

	if (ret)
		goto error;

	uloop_timeout_set(timeout, UBUSDEV_TIMEOUT);
	return;

error:
	ubusdev_invocation_error(ret, method, ubr->udev.dev.ifname);
}

static struct device*
_ubusdev_create(const char *name, struct device_type *type,
	struct blob_attr *config)
{
	struct ubusdev_device *udev;
	struct ubusdev_type *utype;
	int ret;

	utype = container_of(type, struct ubusdev_type, handler);

	udev = calloc(1, sizeof(struct ubusdev_device));

	if (!udev)
		return NULL;

	ret = device_init(&udev->dev, type, name);
	if (ret)
		goto error;

	udev->utype = utype;
	udev->retry.cb = ubusdev_timeout_cb;

	// let the external device handler set up the device
	ret = netifd_ubusdev_invoke_async(&udev->req, utype->ubus_peer_id,
		__ubusdev_methods[UBUSDEV_METHOD_CREATE], config, ubusdev_req_data_cb,
		ubusdev_req_complete_cb, NULL);
	if (ret)
		goto inv_error;

	// Don't call config_init automatically but only after the external 
	// device handler has notified us of successful device creation.
	udev->dev.config_pending = false;

	ubusdev_set_timeout(udev, STATE_PENDING_CREATE, UBUSDEV_TIMEOUT);

	return &udev->dev;

inv_error:
	ubusdev_invocation_error(ret, __ubusdev_methods[UBUSDEV_METHOD_CREATE],
		name);
error:
	device_free(&udev->dev);
	fprintf(stderr, "Creating %s %s failed: %s\n", type->name, name,
		ubus_strerror(ret));
	return NULL;
}

static const struct device_hotplug_ops ubusdev_ops = {
	.prepare = ubusdev_hotplug_prepare,
	.add = ubusdev_hotplug_add,
	.del = ubusdev_hotplug_remove
};

static struct device*
_ubusdev_bridge_create(const char *name, struct device_type *devtype,
	struct blob_attr *config)
{
	struct ubusdev_bridge *ubr;

	ubr = calloc(1, sizeof(*ubr));
	if (!ubr)
		return NULL;

	device_init(&ubr->udev.dev, devtype, name);
	ubr->udev.dev.config_pending = true;
	ubr->udev.utype = container_of(devtype, struct ubusdev_type, handler);
	ubr->udev.retry.cb = ubusdev_bridge_timeout_cb;

	// for bridge types, the default device state callback is replaced
	// in the device struct but kept in the ubusdev_bridge wrapper struct
	// much like what bridge.c is doing.
	// Also, a copy of the config is stored with the wrapper in case the
	// bridge gets disabled and re-enabled
	ubr->set_state = ubr->udev.dev.set_state;
	ubr->udev.dev.set_state = ubusdev_bridge_set_state;

	ubr->udev.dev.hotplug_ops = &ubusdev_ops;

	vlist_init(&ubr->members, avl_strcmp, ubusdev_bridge_member_update);
	ubr->members.keep_old = true;
	ubusdev_bridge_reload(&ubr->udev.dev, config);

	return &ubr->udev.dev;
}

/* Device creation process with ubus devices:
 * TODO: check if still accurate!
 * For bridges:
 *  1) The bridge state is initialized in netifd. Devices for the members are
 *     created and added to the members vlist by config_init automatically.
 *  2) When the first bridge member device is brought up (ubusdev_bridge_enable_member)
 *     the asynchronous 'create' call to the external device handler is issued.
 *  3) The creation happens asynchronously. The device is marked PENDING_CREATE
 *     and a timer is started to regularly check if creation has completed.
 *  4) After successful device creation, the external device handler notifies
 *     netifd via the ubus subscription mechanism. The bridge is then marked "present"
 *     and a new attempt at adding the member is made.
 * For regular devices:
 *  1) The device structure is created in netifd.
 *  2) config_init is called automatically which issues the 'create' call to the
 *     external device handler.
 *  3) Device creation happens asynchronously and the external device handler notifies
 *     netifd when it is done.
 */
static struct device *
ubusdev_create(const char *name, struct device_type *devtype,
	struct blob_attr *config)
{
	struct ubusdev_type *utype = container_of(devtype, struct ubusdev_type,
			handler);

	// abort if external device handler not present
	if (!ubusdev_check_subscribed(utype,
		__ubusdev_methods[UBUSDEV_METHOD_CREATE]))
		return NULL;

	if (devtype->bridge_capability)
		return _ubusdev_bridge_create(name, devtype, config);
	else
		return _ubusdev_create(name, devtype, config);
}

/* Free a device both locally wihtin netifd and externally by invoking
 * 'free' on the external device handler
 */
static void
ubusdev_free(struct device *dev)
{
	struct ubusdev_type *utype;
	struct ubusdev_device *udev;
	int ret;

	utype = container_of(dev->type, struct ubusdev_type, handler);

	if (!ubusdev_check_subscribed(utype,
		__ubusdev_methods[UBUSDEV_METHOD_FREE]))
		return;

	udev = container_of(dev, struct ubusdev_device, dev);

	blob_buf_init(&blob_buffer, 0);
	blobmsg_add_string(&blob_buffer, "name", dev->ifname);

	ret = netifd_ubusdev_invoke_async(&udev->req, utype->ubus_peer_id,
		__ubusdev_methods[UBUSDEV_METHOD_FREE], blob_buffer.head,
		ubusdev_req_data_cb, ubusdev_req_complete_cb, NULL);

	if (ret)
		goto error;

	ubusdev_set_timeout(udev, STATE_PENDING_FREE, UBUSDEV_TIMEOUT);

error:
	ubusdev_invocation_error(ret, __ubusdev_methods[UBUSDEV_METHOD_FREE],
			dev->ifname);
}

/* Set bridge present if it is empty or initialize bridge members.
 */
static void
_ubusdev_bridge_config_init(struct device *dev)
{
	int rem, ret;
	struct blob_attr *cur;
	struct ubusdev_bridge *ubr = container_of(dev, struct ubusdev_bridge,
		udev.dev);

	if (ubr->empty)
		ubr->force_active = true;

	ubr->n_failed = 0;
	vlist_update(&ubr->members);
	if (ubr->ifnames) {
		blobmsg_for_each_attr(cur, ubr->ifnames, rem)
			ubusdev_bridge_add_member(ubr, blobmsg_data(cur));
	} else if (ubr->empty) {
		// in case of empty bridge, create it immediately instead of waiting
		// for members to be brought up first.
		ret = netifd_ubusdev_invoke_async(&ubr->udev.req,
			ubr->udev.utype->ubus_peer_id,
			__ubusdev_methods[UBUSDEV_METHOD_CREATE], ubr->config,
			ubusdev_req_data_cb, ubusdev_req_complete_cb, NULL);

		if (ret)
			goto error;

		ubusdev_set_timeout(&ubr->udev, STATE_PENDING_CREATE, UBUSDEV_TIMEOUT);
	}

	vlist_flush(&ubr->members);
	return;

error:
	fprintf(stderr, "Failed to init config for '%s': %s\n", dev->ifname,
		ubus_strerror(ret));
}

static void
ubusdev_config_init(struct device *dev)
{
	struct ubusdev_type *utype = container_of(dev->type, struct ubusdev_type,
			handler);

	if (!ubusdev_check_subscribed(utype,
		__ubusdev_methods[UBUSDEV_METHOD_CONFIG_INIT]))
		return;

	if (dev->type->bridge_capability)
		_ubusdev_bridge_config_init(dev);
}

static void
ubusdev_buf_add_list(struct blob_attr *attr, int len, const char *name,
		struct blob_buf *buf, bool array)
{
	struct blob_attr *cur;
	struct blobmsg_hdr *hdr;
	void *list;
	int type;

	if (array)
		list = blobmsg_open_array(buf, name);
	else
		list = blobmsg_open_table(buf, name);

	__blob_for_each_attr(cur, attr, len) {
		hdr = blob_data(cur);
		type = blobmsg_type(cur);
		switch (type) {
			case BLOBMSG_TYPE_STRING:
				blobmsg_add_string(buf, (char *) hdr->name,
						blobmsg_get_string(cur));
				break;
			case BLOBMSG_TYPE_TABLE:
			case BLOBMSG_TYPE_ARRAY:
				ubusdev_buf_add_list(blobmsg_data(cur),
					blobmsg_data_len(cur), (char *) hdr->name, buf,
					type == BLOBMSG_TYPE_ARRAY);
				break;
			case BLOBMSG_TYPE_INT64:
				blobmsg_add_u64(buf, (char *) hdr->name,
					blobmsg_get_u64(cur));
				break;
			case BLOBMSG_TYPE_INT32:
				blobmsg_add_u32(buf, (char *) hdr->name,
					blobmsg_get_u32(cur));
				break;
			case BLOBMSG_TYPE_INT16:
				blobmsg_add_u16(buf, (char *) hdr->name,
					blobmsg_get_u16(cur));
				break;
			case BLOBMSG_TYPE_INT8:
				blobmsg_add_u8(buf, (char *) hdr->name,
					blobmsg_get_u8(cur));
				break;
			default:
				break;
		}
	}

	if (array)
		blobmsg_close_array(buf, list);
	else
		blobmsg_close_table(buf, list);
}

static void
_add_parsed_data(struct blob_attr **tb, const struct blobmsg_policy *policy,
	int n_params, struct blob_buf *buf)
{
	for (int i = 0; i < n_params; i++) {
		if (!tb[i])
			continue;

		switch (policy[i].type) {
			case BLOBMSG_TYPE_STRING:
				blobmsg_add_string(buf, policy[i].name,
						blobmsg_get_string(tb[i]));
				break;
			case BLOBMSG_TYPE_ARRAY:
			case BLOBMSG_TYPE_TABLE:
				ubusdev_buf_add_list(blobmsg_data(tb[i]),
						blobmsg_data_len(tb[i]), policy[i].name, buf,
						policy[i].type == BLOBMSG_TYPE_ARRAY);
				break;
			case BLOBMSG_TYPE_INT64:
				blobmsg_add_u64(buf, policy[i].name, blobmsg_get_u64(tb[i]));
				break;
			case BLOBMSG_TYPE_INT32:
				blobmsg_add_u32(buf, policy[i].name, blobmsg_get_u32(tb[i]));
				break;
			case BLOBMSG_TYPE_INT16:
				blobmsg_add_u16(buf, policy[i].name, blobmsg_get_u16(tb[i]));
				break;
			case BLOBMSG_TYPE_INT8:
				blobmsg_add_u8(buf, policy[i].name, blobmsg_get_u8(tb[i]));
				break;
			default:
				break;
		}
	}
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

	_add_parsed_data(tb, info_policy, n_params, buf);

	free(req->priv);
}

static void
ubusdev_dump_info(struct device *dev, struct blob_buf *buf)
{
	struct ubusdev_type *utype = container_of(dev->type, struct ubusdev_type,
			handler);
	struct ubusdev_dump_data *data;

	if (!ubusdev_check_subscribed(utype,
		__ubusdev_methods[UBUSDEV_METHOD_DUMP_INFO]))
		return;

	data = calloc(1, sizeof(struct ubusdev_dump_data));
	if (!data)
		return;

	data->dev = dev;
	data->buf = buf;

	blob_buf_init(&blob_buffer, 0);
	blobmsg_add_string(&blob_buffer, "name", dev->ifname);

	netifd_ubusdev_invoke_sync(utype->ubus_peer_id,
		__ubusdev_methods[UBUSDEV_METHOD_DUMP_INFO], blob_buffer.head,
		ubusdev_info_data_cb, data);

	system_if_dump_info(dev, buf);
}

static void
ubusdev_stats_data_cb(struct ubus_request *req, int type,
	struct blob_attr *reply)
{
	struct ubusdev_dump_data *data = req->priv;
	struct ubusdev_type *utype = container_of(data->dev->type, struct
		ubusdev_type, handler);
	const struct blobmsg_policy *stats_policy = utype->stats_params->params;
	int n_params = utype->stats_params->n_params;

	struct blob_attr *tb[n_params];

	blobmsg_parse(stats_policy, n_params, tb, blobmsg_data(reply),
		blobmsg_len(reply));

	_add_parsed_data(tb, stats_policy, n_params, data->buf);

	free(req->priv);
}

static void
ubusdev_dump_stats(struct device *dev, struct blob_buf *buf)
{
	struct ubusdev_type *utype = container_of(dev->type, struct ubusdev_type,
			handler);
	struct ubusdev_dump_data *data;

	if (!ubusdev_check_subscribed(utype,
		__ubusdev_methods[UBUSDEV_METHOD_DUMP_STATS]))
		return;

	data = calloc(1, sizeof(struct ubusdev_dump_data));
	if (!data)
		return;

	data->dev = dev;
	data->buf = buf;

	blob_buf_init(&blob_buffer, 0);
	blobmsg_add_string(&blob_buffer, "name", dev->ifname);

	netifd_ubusdev_invoke_sync(utype->ubus_peer_id,
		__ubusdev_methods[UBUSDEV_METHOD_DUMP_STATS], blob_buffer.head,
		ubusdev_stats_data_cb, data);
}

static char *
_ubusdev_parse_dev_notification(struct blob_attr *attr)
{
	static struct blobmsg_policy policy = {
		.name = "name",
		.type = BLOBMSG_TYPE_STRING
	};

	struct blob_attr *tb;

	blobmsg_parse(&policy, 1, &tb, blobmsg_data(attr), blobmsg_len(attr));

	if (!tb)
		return NULL;

	return blobmsg_get_string(tb);
}
static int
_ubusdev_parse_hotplug_notification(struct blob_attr *attr, char **bridge_name,
	char **member_name)
{
	static struct blobmsg_policy policy[2] = {
		{"bridge", BLOBMSG_TYPE_STRING},
		{"member", BLOBMSG_TYPE_STRING}
	};

	struct blob_attr *tb[2];

	blobmsg_parse(policy, 2, tb, blobmsg_data(attr), blobmsg_len(attr));
	if (!tb[0] || !tb[1])
		return UBUS_STATUS_INVALID_ARGUMENT;

	*bridge_name = blobmsg_get_string(tb[0]);
	*member_name = blobmsg_get_string(tb[1]);

	return 0;
}
static int
_ubusdev_bridge_handle_create_notification(struct ubusdev_bridge *ubr)
{
	int ret;

	if (ubr->udev.sync != STATE_PENDING_CREATE)
		return 0;

	ubr->active = true;
	ubusdev_set_sync(&ubr->udev, STATE_SYNCHRONIZED);

	// call preserved set_state callback to bring bridge up (system_if_up)
	ret = ubr->set_state(&ubr->udev.dev, true);
	if (ret < 0)
		ubusdev_bridge_set_down(ubr);

	device_set_present(&ubr->udev.dev, true);
	return ret;
}

static int
_ubusdev_handle_create_notification(struct ubusdev_device *udev)
{
	if (udev->sync != STATE_PENDING_CREATE)
		return 0;

	ubusdev_set_sync(udev, STATE_SYNCHRONIZED);
	device_set_present(&udev->dev, true);

	return 0;
}

/* Called when external device handler signals successful device creation.
 * Mark devices as synced and ready for use.
 */
static int
ubusdev_handle_create_notification(const char *name)
{
	struct device *dev;
	struct ubusdev_bridge *ubr;
	struct ubusdev_device *udev;

	dev = device_get(name, 0);

	if (!dev)
		return 0;

	if (dev->type->bridge_capability) {
		ubr = container_of(dev, struct ubusdev_bridge, udev.dev);
		return _ubusdev_bridge_handle_create_notification(ubr);
	} else {
		udev = container_of(dev, struct ubusdev_device, dev);
		return _ubusdev_handle_create_notification(udev);
	}

	return 0;
}

static int
ubusdev_handle_reload_notification(const char *name)
{
	struct device *dev;
	struct ubusdev_device *udev;

	dev = device_get(name, 0);
	if (!dev)
		return UBUS_STATUS_NOT_FOUND;

	udev = container_of(dev, struct ubusdev_device, dev);

	if (udev->sync == STATE_PENDING_RELOAD) {
		ubusdev_set_sync(udev, STATE_SYNCHRONIZED);
		device_set_present(dev, true);
	}

	return 0;
}

static int
ubusdev_handle_free_notification(const char *name)
{
	struct device *dev;
	struct ubusdev_bridge *ubr;

	dev = device_get(name, 0);
	if (!dev)
		return UBUS_STATUS_INVALID_ARGUMENT;

	if (dev->type->bridge_capability) {
		ubr = container_of(dev, struct ubusdev_bridge, udev.dev);

		// Do not delete devices that are not marked for deletion.
		// This means that bridges merely get 'disabled' while their
		// devices and configs are still available.
		if (ubr->udev.sync == STATE_PENDING_DISABLE) {
			ubr->active = false;
			ubusdev_set_sync(&ubr->udev, STATE_SYNCHRONIZED);
			return 0;
		}

		if (ubr->ifnames)
			free(ubr->ifnames);

		if (ubr->config)
			free(ubr->config);

		vlist_flush_all(&ubr->members);
		free(ubr);
	}

	return 0;
}

static int
ubusdev_handle_hotplug_prepare_notification(const char *name)
{
	struct device *dev;
	struct ubusdev_bridge *ubr;
	dev = device_get(name, 0);
	if (!dev)
		return UBUS_STATUS_INVALID_ARGUMENT;

	if (!dev->type->bridge_capability)
		return UBUS_STATUS_NOT_SUPPORTED;

	ubr = container_of(dev, struct ubusdev_bridge, udev.dev);

	if (ubr->udev.sync != STATE_PENDING_PREPARE)
		return 0;

	ubusdev_set_sync(&ubr->udev, STATE_SYNCHRONIZED);
	ubr->force_active = true;
	device_set_present(&ubr->udev.dev, true);

	return 0;
}

static int
ubusdev_handle_hotplug_add_notification(const char *bridge_name,
	const char *member_name)
{
	struct device *bridge, *member_dev;
	struct ubusdev_bridge_member *ubm;
	struct ubusdev_bridge *ubr;

	bridge = device_get(bridge_name, 0);
	if (!bridge)
		return UBUS_STATUS_INVALID_ARGUMENT;

	member_dev = device_get(member_name, 0);
	if (!member_dev)
		return UBUS_STATUS_NOT_FOUND;

	ubr = container_of(bridge, struct ubusdev_bridge, udev.dev);

	// If the member is already present in the members list of the bridge,
	// it means that this notification is happening because the member has
	// not been added via hotplug add. This member has to be activated rather
	// than created.
	// Correspondingly, if the member does not exist in the bridge, create it.
	ubm = vlist_find(&ubr->members, member_name, ubm, node);
	if (!ubm) {
		ubusdev_bridge_create_member(ubr, member_dev, true);
		return 0;
	}

	if (ubm->sync != STATE_PENDING_ADD)
		return 0;

	ubusdev_bridge_member_set_sync(ubm, STATE_SYNCHRONIZED);
	device_broadcast_event(&ubr->udev.dev, DEV_EVENT_TOPO_CHANGE);

	return 0;
}

static int
ubusdev_handle_hotplug_remove_notification(const char *bridge_name,
	const char *member_name)
{
	struct device *bridge;
	struct ubusdev_bridge_member *ubm;
	struct ubusdev_bridge *ubr;

	bridge = device_get(bridge_name, 0);
	if (!bridge)
		return UBUS_STATUS_INVALID_ARGUMENT;

	ubr = container_of(bridge, struct ubusdev_bridge, udev.dev);

	ubm = vlist_find(&ubr->members, member_name, ubm, node);
	if (!ubm)
		return UBUS_STATUS_INVALID_ARGUMENT;

	if (ubm->sync != STATE_PENDING_REMOVE)
		return 0;

	ubusdev_bridge_member_set_sync(ubm, STATE_SYNCHRONIZED);

	device_release(&ubm->dev_usr);
	device_broadcast_event(&ubr->udev.dev, DEV_EVENT_TOPO_CHANGE);

	return 0;
}

/* Called as part of the subscription to the external device handler's
 * ubus object.
 * Dispatch appropriate handler for specific event.
 */
static int
ubusdev_handle_notification(struct ubus_context *ctx, struct ubus_object *obj,
	struct ubus_request_data *req, const char *type, struct blob_attr *msg)
{
	char *dev, *bridge, *member;
	int (*dev_handler)(const char *) = NULL;
	int (*hotplug_handler)(const char *, const char *) = NULL;

	if (!strcmp(type, "create"))
		dev_handler = ubusdev_handle_create_notification;
	else if (!strcmp(type, "reload"))
		dev_handler = ubusdev_handle_reload_notification;
	else if (!strcmp(type, "free"))
		dev_handler = ubusdev_handle_free_notification;
	else if (!strcmp(type, "prepare"))
		dev_handler = ubusdev_handle_hotplug_prepare_notification;
	else if (!strcmp(type, "add"))
		hotplug_handler = ubusdev_handle_hotplug_add_notification;
	else if (!strcmp(type, "remove"))
		hotplug_handler = ubusdev_handle_hotplug_remove_notification;
	else
		return UBUS_STATUS_NOT_SUPPORTED;

	if (dev_handler) {
		dev = _ubusdev_parse_dev_notification(msg);
		if (!dev)
			return UBUS_STATUS_INVALID_ARGUMENT;

		return dev_handler(dev);
	} else {
		if (_ubusdev_parse_hotplug_notification(msg, &bridge, &member))
			return UBUS_STATUS_INVALID_ARGUMENT;

		return hotplug_handler(bridge, member);
	}
}

static void
ubusdev_ext_handler_remove_cb(struct ubus_context *ctx,
	struct ubus_subscriber *obj, uint32_t id)
{
	struct ubusdev_type *utype;
	utype = container_of(obj, struct ubusdev_type, ubus_sub);

	netifd_log_message(L_NOTICE, "%s: connection to external device handler at "
		"'%s' lost. Waiting for it to re-appear.\n", utype->handler.name,
		utype->ext_dev_handler_name);

	utype->ubus_peer_id = 0;
	utype->subscribed = false;

	ubusdev_ext_ubus_obj_wait(&utype->obj_wait);
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

	// prepare and register ubus object for subscription
	sprintf(ubus_obj_name, UBUSDEV_UBUSOBJ_NAME_PREFIX "%s", ubus_name);
	utype->ubus_sub.obj.name = ubus_obj_name;
	utype->ubus_sub.obj.type = &ubusdev_ubus_object_type;
	ret = ubus_register_subscriber(ubus_ctx, &utype->ubus_sub);
	if (ret) {
		fprintf(stderr, "Failed to register subscriber object '%s'\n",
			utype->ubus_sub.obj.name);
		goto error;
	}

	// set up event handler for external device handler's ubus object add
	utype->obj_wait.cb = ubusdev_wait_ev_cb;

	// subscribe to peer object
	utype->ubus_sub.cb = ubusdev_handle_notification;
	utype->ubus_sub.remove_cb = ubusdev_ext_handler_remove_cb;
	ubusdev_subscribe(utype);

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
