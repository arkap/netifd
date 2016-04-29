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
#include "utils.h"
#include "ubus.h"
#include "ubusdev.h"

#define UBUSDEV_UBUSOBJ_NAME_PREFIX "network.device.ubus."

extern struct ubus_context *ubus_ctx;
static struct blob_buf blob_buffer;
static int confdir_fd = -1;

static void _ubusdev_free_local(struct device *dev);

// wrapper for ubus device type structs
struct ubusdev_type {
	struct device_type handler;

	uint32_t ubus_peer_id;
	struct ubus_subscriber ubus_sub;
	bool subscribed;

	struct uci_blob_param_list *config_params;
	char *config_strbuf;
};

// wrapper for device structs supporting bridge functionality
struct ubusdev_bridge {
	struct device dev;
	device_state_cb set_state;

	// the members' names
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
};

enum {
	// device handler interface
	UBUSDEV_METHOD_CREATE,
	UBUSDEV_METHOD_CONFIG_INIT,
	UBUSDEV_METHOD_RELOAD,
	UBUSDEV_METHOD_DUMP_INFO,
	UBUSDEV_METHOD_DUMP_STATS,
	UBUSDEV_METHOD_CHECK_STATE,
	UBUSDEV_METHOD_DELETE,

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
	[UBUSDEV_METHOD_DELETE] = "free",

	// hotplug ops
	[UBUSDEV_METHOD_HOTPLUG_ADD] = "add",
	[UBUSDEV_METHOD_HOTPLUG_REMOVE] = "remove",
	[UBUSDEV_METHOD_HOTPLUG_PREPARE] = "prepare",
};

static void
ubusdev_invokation_error(int error, int method, const char *devname)
{
	fprintf(stderr, "invokation of method '%s' failed for device '%s': %s\n",
		__ubusdev_methods[method], devname, ubus_strerror(error));
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
ubusdev_lookup_id(struct ubusdev_type *dtype_wrap)
{
	int ret;

	if (!dtype_wrap || !dtype_wrap->handler.name)
		return 1;

	ret = ubus_lookup_id(ubus_ctx, dtype_wrap->handler.name,
		&dtype_wrap->ubus_peer_id);

	if (ret) {
		fprintf(stderr, "could not find ubus ID for object '%s'\n",
			dtype_wrap->handler.name);
		return ret;
	}

	return 0;
}

static int
ubusdev_hotplug_add(struct device *dev, struct device *member)
{
	netifd_log_message(L_DEBUG, "call :: %s (bridge='%s', member='%s')\n", __func__, dev->ifname, member->ifname);
	int ret = 0;
	struct ubusdev_type *utype = container_of(dev->type, struct ubusdev_type,
		handler);

	if (dev->type->bridge_support) {

		blob_buf_init(&blob_buffer, 0);
		blobmsg_add_string(&blob_buffer, "name", dev->ifname);
		blobmsg_add_string(&blob_buffer, "member", member->ifname);

		ret = netifd_ubusdev_invoke(utype->ubus_peer_id,
			__ubusdev_methods[UBUSDEV_METHOD_HOTPLUG_ADD], blob_buffer.head);
	} else {
		// TODO
	}

	if (ret)
		ubusdev_invokation_error(ret, UBUSDEV_METHOD_HOTPLUG_ADD, NULL);

	return ret;
}

static int
ubusdev_hotplug_del(struct device *dev, struct device *member)
{
	netifd_log_message(L_DEBUG, "call :: %s (dev=%s, member=%s)\n", __func__, dev->ifname, member->ifname);
	netifd_log_message(L_DEBUG, "call :: %s (dev=%s, member=%s)\n", __func__, dev->ifname, member->ifname);
	// TODO
	return 0;
}

static int
ubusdev_hotplug_prepare(struct device *dev)
{
	netifd_log_message(L_DEBUG, "call :: %s (dev=%s)\n", __func__, dev->ifname);
 	// TODO
	return 0;
}

static const struct device_hotplug_ops ubusdev_ops = {
	.prepare = ubusdev_hotplug_prepare,
	.add = ubusdev_hotplug_add,
	.del = ubusdev_hotplug_del
};

static int
ubusdev_bridge_enable_member(struct ubusdev_bridge_member *member)
{
	netifd_log_message(L_DEBUG, "call :: %s\n", __func__);

	int ret;
	struct ubusdev_bridge *ubr = member->parent_br;

	if (!member->present)
		return 0;

	netifd_log_message(L_DEBUG, "member '%s' claiming device '%s'\n", member->name, member->dev_usr.dev->ifname);

	ret = device_claim(&member->dev_usr);
	if (ret < 0)
		goto error;

	device_set_present(&ubr->dev, true);
	device_broadcast_event(&ubr->dev, DEV_EVENT_TOPO_CHANGE);

	return 0;

error:
	ubr->n_failed++;
	member->present = false;
	ubr->n_present--;
	device_release(&member->dev_usr);

	return ret;
}

static int
ubusdev_bridge_set_down(struct ubusdev_bridge *ubr)
{
	netifd_log_message(L_DEBUG, "call :: %s\n", __func__);

	// TODO
	return 0;
}

static int
ubusdev_bridge_set_up(struct ubusdev_bridge *ubr)
{
	netifd_log_message(L_DEBUG, "call :: %s (dev->ifname='%s')\n", __func__, ubr->dev.ifname);

	int ret;
	struct ubusdev_bridge_member *member;

	netifd_log_message(L_DEBUG, "present: %d, force_active: %d\n", ubr->n_present, ubr->force_active);

	if (!ubr->n_present)
		if (!ubr->force_active)
			return -ENOENT;

	ubr->n_failed = 0;
	vlist_for_each_element(&ubr->members, member, node)
		ubusdev_bridge_enable_member(member);
	// TODO: need retry?

	if (!ubr->force_active && !ubr->n_present) {
		netifd_log_message(L_DEBUG, "init of all members for bridge '%s' failed\n", ubr->dev.ifname);

		// initialization of all member interfaces failed
		device_set_present(&ubr->dev, false);
		return -ENOENT;
	}

	// call preserved set_state callback to bring bridge up
	ret = ubr->set_state(&ubr->dev, true);
	if (ret < 0)
		ubusdev_bridge_set_down(ubr);

	return ret;
}

static int
ubusdev_bridge_set_state(struct device *dev, bool up)
{
	struct ubusdev_bridge *ubr;

	if (!dev->type->bridge_support)
		return -1;

	 ubr = container_of(dev, struct ubusdev_bridge, dev);

	if (up)
		return ubusdev_bridge_set_up(ubr);
	else
		return ubusdev_bridge_set_down(ubr);
}

static int
ubusdev_bridge_disable_member(struct ubusdev_bridge_member *member)
{
	netifd_log_message(L_DEBUG, "call :: %s (member='%s')\n", __func__, member->dev_usr.dev->ifname);

	if (!member->present)
		return 0;

	// TODO: the bridge code broadcasts a TOPOLOGY CHANGE event here. do we need that?
	device_release(&member->dev_usr);
	return 0;
}

static void
ubusdev_bridge_remove_member(struct ubusdev_bridge_member *member)
{
	netifd_log_message(L_DEBUG, "call :: %s (member='%s')\n", __func__, member->dev_usr.dev->ifname);

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
	struct ubusdev_bridge_member *member = container_of(usr, struct ubusdev_bridge_member, dev_usr);
	struct ubusdev_bridge *ubr = member->parent_br;

	netifd_log_message(L_DEBUG, "call :: %s (name='%s', event=%d)\n", __func__, member->name, event);

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
	netifd_log_message(L_DEBUG, "call :: %s (ubr->dev.ifname='%s', dev->ifname='%s')\n", __func__, ubr->dev.ifname, dev->ifname);

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
	if (hotplug && member)
		member->node.version = -1;

	return member;
}

/* Add member with given name to bridge. A device for the member is created
 * if it does not exist.
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
	netifd_log_message(L_DEBUG, "call :: %s (member='%s')\n", __func__, member->dev_usr.dev->ifname);

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
	// TODO: ubusdev_bridge_member_update
}

static void
ubusdev_bridge_member_update(struct vlist_tree *tree,
	struct vlist_node *node_new, struct vlist_node *node_old)
{
	netifd_log_message(L_DEBUG, "call :: %s\n", __func__);

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

enum {
	UBUSDEV_BRIDGE_EMPTY,
	UBUSDEV_BRIDGE_IFNAMES,
	__UBUSDEV_BRIDGE_MAX
};

static struct blobmsg_policy ubusdev_bridge_policy[__UBUSDEV_BRIDGE_MAX] = {
	[UBUSDEV_BRIDGE_IFNAMES] = {
		.name = "ifname",
		.type = BLOBMSG_TYPE_ARRAY,
	},
};

static struct device*
_ubusdev_bridge_create(struct blob_attr *config)
{
	netifd_log_message(L_DEBUG, "call :: %s\n", __func__);

	struct blob_attr *tb[__UBUSDEV_BRIDGE_MAX];
	struct ubusdev_bridge *ubr;

	ubr = calloc(1, sizeof(*ubr));
	if (!ubr)
		return NULL;

	blobmsg_parse(ubusdev_bridge_policy, __UBUSDEV_BRIDGE_MAX, tb,
	blobmsg_data(config), blobmsg_len(config));

	// make sure the empty flag is set when no interface names are given and
	// vice versa
	if (!tb[UBUSDEV_BRIDGE_EMPTY] || !blobmsg_get_bool(tb[UBUSDEV_BRIDGE_EMPTY])) {
		ubr->force_active = false;

		if (!tb[UBUSDEV_BRIDGE_IFNAMES])
			goto error;
	} else {
		ubr->force_active = true;
	}

	ubr->ifnames = tb[UBUSDEV_BRIDGE_IFNAMES];

	vlist_init(&ubr->members, avl_strcmp, ubusdev_bridge_member_update);
	ubr->members.keep_old = true;

	return &ubr->dev;

error:
	free(ubr);
	return NULL;
}

/* Device creation process with ubus devices:
 * 1. create() called on the device type struct with a config for the new device
 * 2. the config blob is sent asynchronously to the external device handler for
 *    processing.
 */
static struct device *
ubusdev_create(const char *name, struct device_type *devtype,
	struct blob_attr *config)
{
	netifd_log_message(L_DEBUG, "call :: %s (name=%s, devtype=%s)\n", __func__, name, devtype->name);

	struct device *dev;
	struct ubusdev_type *utype;
	int ret;
	bool bridge_type = devtype->bridge_support;
	struct ubusdev_bridge *ubr;

	utype = container_of(devtype, struct ubusdev_type, handler);

	if (bridge_type)
		dev = _ubusdev_bridge_create(config);
	else
		dev = calloc(1, sizeof(struct device));

	if (!dev)
		return NULL;

	if (!utype->ubus_peer_id && ubusdev_lookup_id(utype)) {
		ret = UBUS_STATUS_NOT_FOUND;
		goto error;
	}

	// create local device representation
	ret = device_init(dev, devtype, name);
	if (ret)
		goto error;

	// let the external device handler set up the device
	ret = netifd_ubusdev_invoke(utype->ubus_peer_id,
		__ubusdev_methods[UBUSDEV_METHOD_CREATE], config);
	if (ret)
		goto error;

	// setting this to false avoids calling config_init automatically.
	// config_init is instead called when the external device handler notifies
	// us of successful device creation
	dev->config_pending = false;

	// for bridge types, the default device state callback is overriden
	// in the device struct but kept in the ubusdev_bridge wrapper struct
	// much like what bridge.c is doing
	if (bridge_type) {
		ubr = container_of(dev, struct ubusdev_bridge, dev);
		ubr->set_state = dev->set_state;
		dev->set_state = ubusdev_bridge_set_state;
	}
	dev->hotplug_ops = &ubusdev_ops;

	return dev;

error:
	ubusdev_invokation_error(ret, UBUSDEV_METHOD_CREATE, name);
	_ubusdev_free_local(dev);
	return NULL;
}

/* Free all state for 'dev' within netifd
 */
static void
_ubusdev_free_local(struct device *dev)
{
	netifd_log_message(L_DEBUG, "call :: %s (dev->ifname='%s')\n", __func__, dev->ifname);

	struct ubusdev_bridge *ubr;

	// if device is a bridge take care of the wrapper
	if (dev->type->bridge_support) {
		ubr = container_of(dev, struct ubusdev_bridge, dev);
		if (ubr->ifnames)
			free(ubr->ifnames);

		vlist_flush_all(&ubr->members);
		free(ubr);
	} else {
		device_free(dev);
	}
}

/* Free a given device both locally wihtin netifd and externally by invoking
 * 'free' on the external device handler
 */
static void
ubusdev_free(struct device *dev)
{
	netifd_log_message(L_DEBUG, "call :: %s (dev->ifname='%s')\n", __func__, dev->ifname);

	struct ubusdev_type *utype;
	int ret;

	utype = container_of(dev->type, struct ubusdev_type, handler);

	blob_buf_init(&blob_buffer, 0);
	blobmsg_add_string(&blob_buffer, "name", dev->ifname);

	ret = netifd_ubusdev_invoke(utype->ubus_peer_id,
		__ubusdev_methods[UBUSDEV_METHOD_DELETE], blob_buffer.head);

	if (ret) {
		ubusdev_invokation_error(ret, UBUSDEV_METHOD_DELETE, dev->ifname);
		return;
	}

	_ubusdev_free_local(dev);
}

/* Bridge type flavor of config_init. Set device present if necessary and
 * potentially initialize bridge members.
 */
static void
_ubusdev_bridge_config_init(struct device *dev)
{
	int rem;
	struct blob_attr *cur;
	struct ubusdev_bridge *ubr = container_of(dev, struct ubusdev_bridge, dev);

	if (ubr->force_active)
		device_set_present(dev, true);

	ubr->n_failed = 0;
	vlist_update(&ubr->members);
	if (ubr->ifnames) {
		blobmsg_for_each_attr(cur, ubr->ifnames, rem)
			ubusdev_bridge_add_member(ubr, blobmsg_data(cur));
	}
	vlist_flush(&ubr->members);
	// TODO: need timed retry like in bridge.c?
}

/* Set a device present. If the device is a bridge type device, allocate
 * state for the mmebers and set them up as well.
 */
static void
ubusdev_config_init(struct device *dev)
{
	netifd_log_message(L_DEBUG, "call :: %s (dev->ifname='%s', bridge=%d)\n", __func__, dev->ifname, dev->type->bridge_support);

	if (dev->type->bridge_support)
		_ubusdev_bridge_config_init(dev);
	else
		device_set_present(dev, true);
}

static enum dev_change_type
ubusdev_reload(struct device *dev, struct blob_attr *config)
{
	netifd_log_message(L_DEBUG, "call :: %s (dev->ifname='%s')\n", __func__, dev->ifname);
	// TODO: compare configs and if different:
	// 1. set device not present
	// 2. contact external device handler
	return DEV_CONFIG_NO_CHANGE;
}

/* Called when external device handler signals successful device creation.
 * Initialize devices on our side.
 */
static int
ubusdev_handle_create_notify(const char **devices, int n_devices)
{
	const char *cur;
	int i;
	struct device *dev;

	for (i = 0, cur = devices[i]; i < n_devices; cur = devices[++i]) {
		netifd_log_message(L_DEBUG, "handling create notification for device '%s'\n", cur);

		dev = device_get(cur, 0);
		if (!dev)
			continue;

		dev->type->config_init(dev);
	}

	return 0;
}

static int
ubusdev_handle_update_notify(const char **devices, int n_dev)
{
	netifd_log_message(L_DEBUG, "call :: %s\n", __func__);
	// TODO
	return 0;
}

static int
ubusdev_handle_delete_notify(const char **devices, int n_dev)
{
	const char *cur;
	int i;
	struct device *dev;

	for (i = 0, cur = devices[i]; i < n_dev; cur = devices[++i]) {
		netifd_log_message(L_DEBUG, "handling 'delete' notification for device '%s'\n", cur);

		dev = device_get(cur, 0);
		if (!dev)
			continue;

		device_free(dev);
	}
	return 0;
}

static int
ubusdev_handle_hotplug_add_notify(const char **devices, int n_dev)
{
	netifd_log_message(L_DEBUG, "call :: %s (n_dev=%d)\n", __func__, n_dev);

	struct device *bridge, *member;
	struct ubusdev_bridge *ubr;

	if (n_dev != 2)
		return UBUS_STATUS_INVALID_ARGUMENT;

	bridge = device_get(devices[0], 0);
	if (!bridge)
		goto error;

	member = device_get(devices[1], 0);
	if (!member)
		goto error;

	ubr = container_of(bridge, struct ubusdev_bridge, dev);
	ubusdev_bridge_create_member(ubr, member, true);
	return 0;

error:
	netifd_log_message(L_DEBUG, "an error occurred in %s\n", __func__);
	// TODO: cleanup
	return -1;
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

	free(devices);

done:
	if (ret)
		fprintf(stderr, "%s notification failed: %s\n", type,
			ubus_strerror(ret));

	free(req);
	return ret;
}

static void
ubusdev_handle_remove(struct ubus_context *ctx, struct ubus_subscriber *obj,
	uint32_t id)
{
	netifd_log_message(L_DEBUG, "call :: %s, (id=%d)", __func__, id);

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
	bool bridge_support, const char *br_prefix, json_object *obj)
{
	struct ubusdev_type *utype;
	struct device_type *devtype;
	json_object *cfg;
	char *ubus_obj_name, *devtype_name, *name_prefix;
	struct uci_blob_param_list *config_params;
	int ret;

	utype = calloc_a(sizeof(*utype),
		&ubus_obj_name, strlen(UBUSDEV_UBUSOBJ_NAME_PREFIX) + strlen(tname) + 1,
		&devtype_name, strlen(tname) + 1,
		&config_params, sizeof(struct uci_blob_param_list));
	if (!utype)
		return;

	utype->config_params = config_params;

	devtype = &utype->handler;
	devtype->name = strcpy(devtype_name, tname);
	devtype->create = ubusdev_create;
	devtype->free = ubusdev_free;
	devtype->config_init = ubusdev_config_init;
	devtype->reload = ubusdev_reload;
	devtype->bridge_support = bridge_support;
	devtype->config_params = utype->config_params;

	if (bridge_support) {
		name_prefix = malloc(strlen(br_prefix) + 1);
		if (!name_prefix)
			goto error;

		strcpy(name_prefix, br_prefix);
		devtype->name_prefix = name_prefix;
	}

	// prepare and register ubus object
	sprintf(ubus_obj_name, UBUSDEV_UBUSOBJ_NAME_PREFIX "%s", tname);
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
	utype->ubus_sub.remove_cb = ubusdev_handle_remove;
	ret = ubus_subscribe(ubus_ctx, &utype->ubus_sub,
		utype->ubus_peer_id);
	if (ret)
		utype->subscribed = false;
	else
		utype->subscribed = true;

	if (!ret)
		netifd_log_message(L_DEBUG, "subscribed to ubus object '%s' at %08x\n", utype->ubus_sub.obj.name, utype->ubus_peer_id);

	// parse and store config format description
	cfg = json_get_field(obj, "config", json_type_array);
	if (!cfg)
		goto error;

	utype->config_strbuf = netifd_handler_parse_config(
		utype->config_params, cfg);
	if (!utype->config_strbuf)
		goto error;

	// add device type to device type list
	ret = device_type_add(devtype);
	if (ret)
		goto config_error;

	return;

config_error:
	free(utype->config_strbuf);

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
