#include "messages.h"
#include "peer.h"

#include <uapi/linux/wireguard.h>

#include <linux/netdevice.h>
#include <net/genetlink.h>

spinlock_t lock;

static const struct genl_multicast_group wglog_mcgrps[] = {
		{ .name = WGLOG_GENL_NAME, },
};

static struct genl_family wglog_genl_family = {
		.hdrsize = 0,
		.name = WGLOG_GENL_NAME,
		.version = WGLOG_GENL_VERSION,
		.maxattr = WGLOG_A_MAX,
		.mcgrps = wglog_mcgrps,
		.n_mcgrps = ARRAY_SIZE(wglog_mcgrps),
};

void wg_log(int level, const struct net_device *dev, const struct wg_peer *peer, const char *fmt, ...) {
	struct sk_buff *skb;
	void *hdr;
	int ret;
	char msg[512];
	va_list args;

	spin_lock_bh(&lock);

	va_start(args, fmt);
	vsnprintf(msg, sizeof(msg), fmt, args);
	va_end(args);

	skb = genlmsg_new(NLMSG_GOODSIZE, GFP_ATOMIC);
	if (skb == NULL) {
		printk(KERN_INFO "error: genlmsg_new\n");
		goto end;
	}

	hdr = genlmsg_put(skb, 0, 0, &wglog_genl_family, 0, 0);
	if (hdr == NULL) {
		printk(KERN_INFO "error: genlmsg_put\n");
		goto end;
	}

	ret = nla_put_s32(skb, WGLOG_A_LEVEL, level);
	if (ret != 0) {
		printk(KERN_INFO "error: nla_put_s32\n");
	}

	ret = nla_put_string(skb, WGLOG_A_MESSAGE, msg);
	if (ret != 0) {
		printk(KERN_INFO "error: nla_put_string\n");
	}

	if (dev) {
		ret = nla_put_string(skb, WGLOG_A_DEV_IFNAME, dev->name);
		if (ret != 0) {
			printk(KERN_INFO "error: nla_put_string\n");
		}

		ret = nla_put_s32(skb, WGLOG_A_DEV_IFINDEX, dev->ifindex);
		if (ret != 0) {
			printk(KERN_INFO "error: nla_put_u32\n");
		}
	}

	if (peer) {
		ret = nla_put(skb, WGLOG_A_PEER_PUBLIC_KEY, NOISE_PUBLIC_KEY_LEN, peer->handshake.remote_static);
		if (ret != 0) {
			printk(KERN_INFO "error: nla_put\n");
		}
	}

	genlmsg_end(skb, hdr);

	ret = genlmsg_multicast(&wglog_genl_family, skb, 0, 0, GFP_ATOMIC);
	if (ret != 0) {
		printk(KERN_INFO "error: genlmsg_multicast\n");
	}

end:
	spin_unlock_bh(&lock);
}

int __init wg_logger_init(void) {
	spin_lock_init(&lock);
	return genl_register_family(&wglog_genl_family);
}

void __exit wg_logger_uninit(void) {
	genl_unregister_family(&wglog_genl_family);
}
