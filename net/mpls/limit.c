#include <linux/types.h>
#include <linux/skbuff.h>
#include <linux/net.h>
#include <linux/module.h>
#include <linux/mpls.h>
#include <linux/vmalloc.h>
#include <net/ip.h>
#include <net/dst.h>
#include <net/lwtunnel.h>
#include <net/netevent.h>
#include <net/netns/generic.h>
#include <net/ip6_fib.h>
#include <net/route.h>
#include <net/mpls_iptunnel.h>
#include <linux/mpls_iptunnel.h>
#include "internal.h"

#define LIMIT_BURSTTIME (HZ / 10)

struct mpls_limit {
	struct rcu_head rcu;
	atomic64_t tried_bytes;
	atomic64_t admitted_bytes;
	int limit;
	spinlock_t lock;
	u64 bw;
	u64 bytes_per_jiffy;
	u64 toks;
	u64 toks_max;
	unsigned long toks_lastupdate;
};

struct mpls_limit_table {
	struct rcu_head rcu;
	unsigned count;
	struct mpls_limit *limits[0];
} *mpls_limits;

int mpls_limit_query(u32 id, struct sk_buff *skb) {
	struct mpls_limit_table *table;
	struct mpls_limit *l;
	int ret = 0;
	
	table = rcu_dereference(mpls_limits);
	if (!table || id >= table->count) return 0;

	l = rcu_dereference(table->limits[id]);
	if (!l) return 0;

	if (l->limit) {
		unsigned long now;

		spin_lock_bh(&l->lock);

		now = jiffies;
		if (now != l->toks_lastupdate) {
			unsigned long delta = now - l->toks_lastupdate;
			if (delta > LIMIT_BURSTTIME) delta = LIMIT_BURSTTIME;
			l->toks += l->bytes_per_jiffy * (u64)delta;
			if (l->toks > l->toks_max) l->toks = l->toks_max;
			l->toks_lastupdate = now;
		}

		if ((u64)skb->len > l->toks) ret = -1;
		else l->toks -= (u64)skb->len;
		
		spin_unlock_bh(&l->lock);
	}

	atomic64_add(skb->len, &l->tried_bytes);
	if (ret == 0) atomic64_add(skb->len, &l->admitted_bytes);

	return ret;
}
EXPORT_SYMBOL_GPL(mpls_limit_query);

static void mpls_limit_table_free(struct mpls_limit_table *t, int limits) {
	if (!t) return;

	if (limits) {
		unsigned i;
		for (i = 0; i < t->count; ++i) {
			if (t->limits[i]) kfree_rcu(t->limits[i], rcu);
		}
	}
	kfree_rcu(t, rcu);
}

static const struct nla_policy mpls_limit_policy[LIMIT_ATTR_MAX + 1] = {
	[LIMIT_ATTR_BW] = {.type = NLA_U64 },
};

static int mpls_rtm_newlimit(struct sk_buff *skb, struct nlmsghdr *nlh,
				struct netlink_ext_ack *extack) {
	int err;
	struct limitmsg *lm;
	struct nlattr *tb[LIMIT_ATTR_MAX+1];
	struct mpls_limit_table *table;
	struct mpls_limit *l;
	int created = 0;

	err = nlmsg_parse(nlh, sizeof(*lm), tb, LIMIT_ATTR_MAX,
			mpls_limit_policy, extack);
	if (err < 0) return err;

	lm = nlmsg_data(nlh);

	table = rtnl_dereference(mpls_limits);
	if (!table || lm->limit_id >= table->count) {
		struct mpls_limit_table *newt;
		unsigned sz;

		if (!(nlh->nlmsg_flags & NLM_F_CREATE)) return -ENOENT;

		sz = sizeof(*newt) + (lm->limit_id + 1) * sizeof(newt->limits[0]);
		sz = 2 << fls(sz - 1);
		if (sz < 1024) sz = 1024;
		newt = kzalloc(sz, GFP_KERNEL);
		if (!newt) {
			NL_SET_ERR_MSG(extack, "Failed to allocate table");
			return -ENOMEM;
		}

		newt->count = (sz - sizeof(*newt)) / sizeof(newt->limits[0]);
		BUG_ON(newt->count <= lm->limit_id);

		printk("mpls_limit: new table, sz %u, count %u, id %u\n",
			sz, newt->count, lm->limit_id);

		if (table) {
			memcpy(newt->limits, table->limits,
				table->count * sizeof(table->limits[0]));
		}
		rcu_assign_pointer(mpls_limits, newt);

		mpls_limit_table_free(table, 0);
		table = newt;
	}

	l = rtnl_dereference(table->limits[lm->limit_id]);
	if (!l) {
		if (!(nlh->nlmsg_flags & NLM_F_CREATE)) return -ENOENT;

		l = kzalloc(sizeof(*l), GFP_KERNEL);
		if (!l) {
			NL_SET_ERR_MSG(extack, "Failed to allocate limit");
			return -ENOMEM;
		}
		spin_lock_init(&l->lock);
		created = 1;
	}
	else {
		if ((nlh->nlmsg_flags & NLM_F_EXCL)) return -EEXIST;
		spin_lock_bh(&l->lock);
	}

	if (tb[LIMIT_ATTR_BW]) l->bw = nla_get_u64(tb[LIMIT_ATTR_BW]);
	else l->bw = 0;
	l->limit = l->bw ? 1 : 0;
	l->bytes_per_jiffy = div64_u64(l->bw, HZ);
	if (!l->bytes_per_jiffy && l->bw) l->bytes_per_jiffy = 1;

	printk("mpls_rtm_newlimit: id %u created %d bw %llu "
		"bytes_per_jiffy %llu\n",
		lm->limit_id, created, l->bw, l->bytes_per_jiffy);

	l->toks_max = l->bytes_per_jiffy * (u64)LIMIT_BURSTTIME;
	// must allow at least one max size frame, for now 16k
	if (l->toks_max < 16384) l->toks_max = 16384;
	l->toks = l->toks_max;
	l->toks_lastupdate = jiffies;

	if (created) rcu_assign_pointer(table->limits[lm->limit_id], l);
	else spin_unlock_bh(&l->lock);

	return 0;
}

static int mpls_rtm_dellimit(struct sk_buff *skb, struct nlmsghdr *nlh,
				struct netlink_ext_ack *extack) {
	int err;
	struct limitmsg *lm;
	struct nlattr *tb[LIMIT_ATTR_MAX+1];
	struct mpls_limit_table *table;
	struct mpls_limit *l;

	err = nlmsg_parse(nlh, sizeof(*lm), tb, LIMIT_ATTR_MAX, NULL, extack);
	if (err < 0) return err;

	lm = nlmsg_data(nlh);

	table = rtnl_dereference(mpls_limits);
	if (!table || lm->limit_id >= table->count) return 0;

	l = rtnl_dereference(table->limits[lm->limit_id]);
	if (!l) return 0;

	printk("mpls_rtm_dellimit: remove id %u\n", lm->limit_id);

	rcu_assign_pointer(table->limits[lm->limit_id], NULL);
	kfree_rcu(l, rcu);

	return 0;
}

static size_t mpls_limit_nlmsgsize(struct mpls_limit *l) {
	return NLMSG_ALIGN(sizeof(struct limitmsg))
		+ nla_total_size_64bit(8)
		+ nla_total_size_64bit(sizeof(struct limitstats));
}

static int mpls_limit_dump(struct sk_buff *skb, unsigned id,
		struct mpls_limit *l, struct sk_buff *req,
		const struct nlmsghdr *reqh, int multi) {
	struct nlmsghdr *nlh;
	struct limitmsg *lm;
	struct limitstats s;

	nlh = nlmsg_put(skb, NETLINK_CB(skb).portid, reqh->nlmsg_seq,
		RTM_NEWLIMIT, sizeof(*lm), multi ? NLM_F_MULTI : 0);
	if (!nlh) return -EMSGSIZE;

	lm = nlmsg_data(nlh);
	lm->limit_family = AF_MPLS;
	lm->limit_id = id;

	if (l->limit && nla_put_u64_64bit(skb, LIMIT_ATTR_BW,
		l->bw, LIMIT_ATTR_PAD)) goto nla_put_failure;

	s.tried_bytes = atomic64_read(&l->tried_bytes);
	s.admitted_bytes = atomic64_read(&l->admitted_bytes);
	s.timestamp = (u32)jiffies;
	if (nla_put_64bit(skb, LIMIT_ATTR_STATS, sizeof(s), &s,
		LIMIT_ATTR_PAD)) goto nla_put_failure;

	nlmsg_end(skb, nlh);
	return 0;

nla_put_failure:
	nlmsg_cancel(skb, nlh);
	return -EMSGSIZE;
}

static int mpls_rtm_getlimit(struct sk_buff *skb, struct nlmsghdr *nlh,
				struct netlink_ext_ack *extack) {
	int err;
	struct limitmsg *lm;
	struct nlattr *tb[LIMIT_ATTR_MAX+1];
	struct mpls_limit_table *table;
	struct mpls_limit *l;
	struct sk_buff *resp;

	err = nlmsg_parse(nlh, sizeof(*lm), tb, LIMIT_ATTR_MAX, NULL, extack);
	if (err < 0) return err;

	lm = nlmsg_data(nlh);

	table = rtnl_dereference(mpls_limits);
	if (!table || lm->limit_id >= table->count) return -ENOENT;

	l = rtnl_dereference(table->limits[lm->limit_id]);
	if (!l) return -ENOENT;

	resp = nlmsg_new(mpls_limit_nlmsgsize(l), GFP_KERNEL);
	if (!resp) return -ENOBUFS;

	err = mpls_limit_dump(resp, lm->limit_id, l, skb, nlh, 0);
	if (err < 0) {
		kfree_skb(resp);
		return err;
	}

	return rtnl_unicast(resp, sock_net(skb->sk), NETLINK_CB(skb).portid);
}

static int mpls_rtm_dumplimits(struct sk_buff *skb, struct netlink_callback *cb) {
	struct mpls_limit_table *table;
	struct mpls_limit *l;
	unsigned id;

	table = rtnl_dereference(mpls_limits);
	for (id = cb->args[0]; table && id < table->count; ++id) {
		l = rtnl_dereference(table->limits[id]);
		if (!l) continue;

		if (mpls_limit_dump(skb, id, l, cb->skb, cb->nlh, 1) < 0)
			break;
	}

	cb->args[0] = id;

	return skb->len;
}

void mpls_limit_init(void) {
	rtnl_register_module(THIS_MODULE, PF_MPLS, RTM_NEWLIMIT,
			     mpls_rtm_newlimit, NULL, 0);
	rtnl_register_module(THIS_MODULE, PF_MPLS, RTM_DELLIMIT,
			     mpls_rtm_dellimit, NULL, 0);
	rtnl_register_module(THIS_MODULE, PF_MPLS, RTM_GETLIMIT,
			     mpls_rtm_getlimit, mpls_rtm_dumplimits, 0);

	rcu_assign_pointer(mpls_limits, NULL);
}

void mpls_limit_exit() {
	struct mpls_limit_table *old;

	// RTM_ handlers are unregistered by rtnl_unregister_all in af_mpls.c

	rtnl_lock();
	old = rtnl_dereference(mpls_limits);
	rcu_assign_pointer(mpls_limits, NULL);
	rtnl_unlock();

	mpls_limit_table_free(old, 1);
}
