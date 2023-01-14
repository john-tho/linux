#include <linux/types.h>
#include <linux/skbuff.h>
#include <linux/socket.h>
#include <linux/sysctl.h>
#include <linux/net.h>
#include <linux/module.h>
#include <linux/if_arp.h>
#include <linux/ipv6.h>
#include <linux/mpls.h>
#include <linux/netconf.h>
#include <linux/vmalloc.h>
#include <linux/percpu.h>
#include <net/ip.h>
#include <net/dst.h>
#include <net/sock.h>
#include <net/arp.h>
#include <net/ip_fib.h>
#include <net/netevent.h>
#include <net/netns/generic.h>
#include <net/ip_tunnels.h>
#if IS_ENABLED(CONFIG_IPV6)
#include <net/ipv6.h>
#endif
#include <net/addrconf.h>
#include <net/nexthop.h>
#include <linux/etherdevice.h>
#include "internal.h"

#define PW_FRAG_B  0x00800000
#define PW_FRAG_E  0x00400000
#define PW_FRAG_MASK (PW_FRAG_B | PW_FRAG_E)

#define PW_FRAG_NONE (0)
#define PW_FRAG_FIRST (PW_FRAG_E)
#define PW_FRAG_LAST (PW_FRAG_B)
#define PW_FRAG_MID (PW_FRAG_B | PW_FRAG_E)

#define PW_FRAG_ISNONE(x) (((x) & PW_FRAG_MASK) == PW_FRAG_NONE)
#define PW_FRAG_ISFIRST(x) (((x) & PW_FRAG_MASK) == PW_FRAG_FIRST)
#define PW_FRAG_ISLAST(x) (((x) & PW_FRAG_MASK) == PW_FRAG_LAST)
#define PW_FRAG_ISMID(x) (((x) & PW_FRAG_MASK) == PW_FRAG_MID)

#define PW_FRAG_ISMIDORLAST(x) (((x) & PW_FRAG_B) == PW_FRAG_B)

#define PW_TTL 64

struct pw_rxskb_cb {
	u32 seq;
	u32 cw;
	unsigned long timestamp;
};

#define PW_RXSKB_MAXAGE (HZ / 10)
#define PW_RXSKB_MAXQUEUE 128
#define PW_RXSKB_CB(x) ((struct pw_rxskb_cb *)((x)->cb))
#define PW_RXSKB_SEQ(x) PW_RXSKB_CB(x)->seq
#define PW_RXSKB_ISNONE(x) PW_FRAG_ISNONE(PW_RXSKB_CB(x)->cw)
#define PW_RXSKB_ISFIRST(x) PW_FRAG_ISFIRST(PW_RXSKB_CB(x)->cw)
#define PW_RXSKB_ISLAST(x) PW_FRAG_ISLAST(PW_RXSKB_CB(x)->cw)
#define PW_RXSKB_ISMID(x) PW_FRAG_ISMID(PW_RXSKB_CB(x)->cw)
#define PW_RXSKB_ISOLD(x, now) \
		(((now) - PW_RXSKB_CB(x)->timestamp) >= PW_RXSKB_MAXAGE)

static inline u32 pw_seq_next(u32 seq) {
	if (seq == 0xffff) return 1;
	return seq + 1;
}

static inline u32 pw_seq_add(u32 seq, u32 v) {
	seq += v;
	if (seq <= 0xffff) return seq;
	return seq - 0xffff;
}

static inline u32 pw_seq_sub(u32 seq, u32 v) {
	if (seq > v) return seq - v;
	return seq + 0xffff - v;
}

static inline int pw_seq_delta(u32 s1, u32 s2) {
    unsigned d;
    if (s1 < s2) d = s2 - s1;
    else d = 0xffff + s2 - s1;
    if (d > 32767) return (int)d - 0xffff;
    else return (int)d;
}

struct pw_priv {
	struct net *net;
	struct net_device *netdev;
	u32 label;
	u32 flags;

	atomic_t txnextseq;
	spinlock_t rxlock;
	unsigned rxlastseq;
	struct sk_buff_head rxq;
};

static void pw_recv_flush(struct pw_priv *pw) {
	struct sk_buff *x;
	while ((x = __skb_dequeue(&pw->rxq)) != NULL) kfree_skb(x);
	pw->rxlastseq = 0;
}

static int pw_recv_one(struct pw_priv *pw, struct sk_buff *skb) {
       struct pcpu_sw_netstats *stats;

	if (unlikely(!pskb_may_pull(skb, 14))) {
		pw->netdev->stats.rx_errors++;
		kfree_skb(skb);
		return NET_RX_DROP;
	}
	skb->dev = pw->netdev;
	skb_reset_mac_header(skb);
	skb->protocol = eth_type_trans(skb, pw->netdev);
	skb_scrub_packet(skb, true);

	stats = this_cpu_ptr(pw->netdev->tstats);
	u64_stats_update_begin(&stats->syncp);
	stats->rx_packets++;
	stats->rx_bytes += skb->len;
	u64_stats_update_end(&stats->syncp);

	return netif_receive_any(skb);
}

static void pw_recv_dropupto(struct pw_priv *pw, struct sk_buff *u) {
	while (1) {
		struct sk_buff *h = skb_peek(&pw->rxq);
		__skb_unlink(h, &pw->rxq);
		kfree_skb(h);
		if (h == u) break;
	}
}

static void pw_recv_dropuptoseq(struct pw_priv *pw, unsigned seq) {
	while (1) {
		struct sk_buff *h = skb_peek(&pw->rxq);
		if (!h) break;
		if (pw_seq_delta(seq, PW_RXSKB_SEQ(h)) > 0) break;
		__skb_unlink(h, &pw->rxq);
		kfree_skb(h);
	}
}

static struct sk_buff *pw_recv_join(struct pw_priv *pw, struct sk_buff *h,
			struct sk_buff *t, unsigned bytes) {
	struct sk_buff *r;

	if (h == t) {
		__skb_unlink(h, &pw->rxq);
		return h;
	}

	// it is quite likely that there will be enough tailroom in first
	// fragment to place whole frame
	// otherwise allocate fresh skb to copy all skbs there
	if (unlikely((h->len + skb_tailroom(h)) < bytes || skb_cloned(h))) {
		r = dev_alloc_skb(bytes + NET_IP_ALIGN + 32);
		if (unlikely(!r)) {
			while (1) {
				struct sk_buff *n = h->next;
				skb_unlink(h, &pw->rxq);
				kfree_skb(h);
				if (h == t) break;
				h = n;
			}
			return NULL;
		}

		r->ingress_priority = h->ingress_priority;
		skb_reserve(r, NET_IP_ALIGN);
	}
	else {
		r = h;
		h = h->next;
		__skb_unlink(r, &pw->rxq);
	}

	while (1) {
		struct sk_buff *n = h->next;
		skb_copy_from_linear_data(h, skb_put(r, h->len), h->len);
		__skb_unlink(h, &pw->rxq);
		kfree_skb(h);
		if (h == t) break;
		h = n;
	}

	return r;
}

static int pw_recv(struct pw_priv *pw, struct sk_buff *skb,
			u32 cw, u32 seq) {
	int ret = NET_RX_SUCCESS;
	struct sk_buff_head recvq;
	struct sk_buff *b;
	int ordered = (pw->flags & PW_FLAGS_RXORD);
	unsigned long now = jiffies;

	skb = skb_share_check(skb, GFP_ATOMIC);
	if (!skb) return NET_RX_DROP;

	if (skb_linearize(skb) != 0) {
		kfree_skb(skb);
		return NET_RX_DROP;
	}

	__skb_queue_head_init(&recvq);

	spin_lock(&pw->rxlock);

	if (ordered && pw->rxlastseq && pw_seq_delta(pw->rxlastseq, seq) <= 0)
		goto drop;

	// if there is stuff in window, and this buffer is after tail,
	// drop old stuff
	b = skb_peek_tail(&pw->rxq);
	if (b && pw_seq_delta(PW_RXSKB_SEQ(b), seq) > 0) {
		while (1) {
			struct sk_buff *h = skb_peek(&pw->rxq);
			if (pw_seq_delta(PW_RXSKB_SEQ(h), seq) > 0) break;
			__skb_unlink(h, &pw->rxq);
			kfree_skb(h);
		}
	}

	if (PW_FRAG_ISNONE(cw)) {
		// not fragment, we must be ordering, otherwise pw_recv does not
		// get called, can receive if this is next expected
		if (pw->rxlastseq && pw_seq_next(pw->rxlastseq) == seq) {
			pw->rxlastseq = seq;
			__skb_queue_tail(&recvq, skb);
			goto out;
		}

		// if nothing received yet, receive this, but must clean older
		// stuff from win
		if (!pw->rxlastseq) {
			pw->rxlastseq = seq;
			__skb_queue_tail(&recvq, skb);
			pw_recv_dropuptoseq(pw, seq);
			goto out;
		}
		// not fragment, but must place in win
	}

	// find place for this buffer in win, start from the tail
	b = (struct sk_buff *)&pw->rxq;
	while (b->prev != (struct sk_buff *)&pw->rxq) {
		int d = pw_seq_delta(PW_RXSKB_SEQ(b->prev), seq);
		if (d == 0) goto drop;
		if (d > 0) break;
		b = b->prev;
	}
	PW_RXSKB_CB(skb)->cw = cw;
	PW_RXSKB_CB(skb)->seq = seq;
	PW_RXSKB_CB(skb)->timestamp = now;
	__skb_insert(skb, b->prev, b, &pw->rxq);

	// now can walk list and see if can receive something or drop something
	// old
out:
	b = skb_peek(&pw->rxq);
	while (b) {
		struct sk_buff *n;
		struct sk_buff *t = b;
		unsigned bytes = b->len;
		int complete = 0;
		int old = PW_RXSKB_ISOLD(b, now);

		if (PW_RXSKB_ISNONE(b)) {
			complete = 2;
		}
		else if ((PW_RXSKB_ISFIRST(b) || PW_RXSKB_ISMID(b))
			&& !skb_queue_is_last(&pw->rxq, t)) {
			unsigned prevseq = PW_RXSKB_SEQ(t);
			if (PW_RXSKB_ISFIRST(b)) ++complete;
			t = t->next;
			while (1) {
				if (PW_RXSKB_SEQ(t) != pw_seq_next(prevseq)) break;
				if (PW_RXSKB_ISNONE(t) || PW_RXSKB_ISFIRST(t)) break;
				bytes += t->len;
				old |= PW_RXSKB_ISOLD(t, now);
				if (PW_RXSKB_ISLAST(t)) {
					++complete;
					break;
				}
				if (skb_queue_is_last(&pw->rxq, t)) break;
				t = t->next;
				prevseq = pw_seq_next(prevseq);
			}
		}

		n = skb_queue_is_last(&pw->rxq, t) ? NULL : t->next;

		if (complete == 2) {
			// buffer complete, can receive if not ordered or if
			// it is next
			if (!ordered
				|| !pw->rxlastseq
				|| PW_RXSKB_SEQ(b) == pw_seq_next(pw->rxlastseq)) {
				struct sk_buff *x;
				pw->rxlastseq = PW_RXSKB_SEQ(t);
				x = pw_recv_join(pw, b, t, bytes);
				if (x) __skb_queue_tail(&recvq, x);
			}
			else {
				// ordered, but something before this is missing
				// if some part is old, drop frame and everything before it
				if (old) {
					pw->rxlastseq = PW_RXSKB_SEQ(t);
					pw_recv_dropupto(pw, t);
				}
			}
		}
		else {
			// if some part is old, drop frame and everything before it
			if (old) {
				pw->rxlastseq = PW_RXSKB_SEQ(t);
				pw_recv_dropupto(pw, t);
			}
		}

		b = n;
	}

	// and last - do not allow fragment queue to grow too big
	while (skb_queue_len(&pw->rxq) > PW_RXSKB_MAXQUEUE) {
		struct sk_buff *x = __skb_dequeue(&pw->rxq);
		kfree_skb(x);
	}

	spin_unlock(&pw->rxlock);

	while (!skb_queue_empty(&recvq)) {
		skb = __skb_dequeue(&recvq);
		pw_recv_one(pw, skb);
	}

	return ret;
drop:
	spin_unlock(&pw->rxlock);
	kfree_skb(skb);
	return NET_RX_DROP;
}

int mpls_pw_recv(struct mpls_route *mr, struct sk_buff *skb) {
	struct net_device *dev = mr->rt_pw;
	struct pw_priv *pw = netdev_priv(dev);
	struct mpls_entry_decoded dec;
	void *p;

	if (unlikely(!netif_running(dev))) {
		kfree_skb(skb);
		return NET_RX_DROP;
	}

	dec = mpls_entry_decode(mpls_hdr(skb));
	if (!dec.bos) goto drop;

	// pull off last tag
	p = __skb_pull(skb, 4);
	skb_reset_network_header(skb);

	if ((pw->flags & PW_FLAGS_RXCW) != 0) {
		u32 cw, len, seq;

		// pull off control word, just make sure there is one
		if (unlikely(!pskb_may_pull(skb, 4))) goto drop;

		cw = ntohl(get_unaligned((u32 *)p));
		seq = cw & 0xffff;
		len = (cw >> 16) & 0x3f;

		// as per RFC4385, seq 0 means in order and no defrag possible
		if (seq == 0) {
			__skb_pull(skb, 4);
			goto dorx;
		}

		if (len) {
			// length specified, makes sure we have at least that
			// and trim skb to that
			if (skb->len < len) goto drop;
			pskb_trim(skb, len);
		}
		else {
			// no length specified, this means that frame len should
			// be at least 64 bytes
			if (skb->len < 64) {
				goto drop;
			}
		}

		__skb_pull(skb, 4);

		if (PW_FRAG_ISNONE(cw)) {
			if ((pw->flags & PW_FLAGS_RXORD) == 0) goto dorx;
		}
		else {
			if ((pw->flags & PW_FLAGS_RXDEFRAG) == 0) goto drop;
		}

		return pw_recv(pw, skb, cw, seq);
	}

dorx:
	// skb now has frame with correct encap to receive over pw iface,
	// account it and receive
	return pw_recv_one(pw, skb);

drop:
	dev->stats.rx_errors++;
	kfree_skb(skb);
	return NET_RX_DROP;
}

void mpls_pw_set_label(struct net_device *dev, u32 label) {
	struct pw_priv *pw = netdev_priv(dev);
	pw->label = label;
}

static int pw_init(struct net_device *dev) {
	dev->tstats = netdev_alloc_pcpu_stats(struct pcpu_sw_netstats);
	if (!dev->tstats) return -ENOMEM;
	return 0;
}

static void pw_free(struct net_device *dev) {
        free_percpu(dev->tstats);
}

static int pw_open(struct net_device *dev) {
	return 0;
}

static int pw_close(struct net_device *dev) {
	struct pw_priv *pw = netdev_priv(dev);

	atomic_set(&pw->txnextseq, 1);

	spin_lock_bh(&pw->rxlock);
	pw_recv_flush(pw);
	spin_unlock_bh(&pw->rxlock);

	return 0;
}

static int pw_change_mtu(struct net_device *dev, int new_mtu) {
	if (new_mtu < 32 || new_mtu > dev->l2mtu) return -EINVAL;
	dev->mtu = new_mtu;
	return 0;
}

static int pw_xmit_one(struct pw_priv *pw, struct sk_buff *skb,
			struct mpls_route *rt, struct mpls_nh *nh,
			struct net_device *out_dev) {
	struct mpls_shim_hdr *hdr;
	bool bos = true;
	int i;

	skb->dev = out_dev;
	skb->protocol = htons(ETH_P_MPLS_UC);

	skb_push(skb, mpls_nh_header_size(nh));
	skb_reset_network_header(skb);
	/* Push the new labels */
	hdr = mpls_hdr(skb);
	for (i = nh->nh_labels - 1; i >= 0; i--) {
		hdr[i] = mpls_entry_encode(nh->nh_label[i],
					   255, 0, bos);
		bos = false;
	}

	if (nh->nh_limit && mpls_limit_query(nh->nh_limit, skb) != 0) goto drop;

	mpls_stats_inc_outucastpkts(out_dev, skb);

	/* If via wasn't specified then send out using device address */
	if (nh->nh_via_table == MPLS_NEIGH_TABLE_UNSPEC)
		return neigh_xmit(NEIGH_LINK_TABLE, out_dev,
				 out_dev->dev_addr, skb);
	else
		return neigh_xmit(nh->nh_via_table, out_dev,
				 mpls_nh_via(rt, nh), skb);

drop:
	kfree_skb(skb);
	return -1;
}

static void pw_putcw(struct pw_priv *pw, struct sk_buff *skb,
			u32 txseq, u32 frag) {
	u32 cw = frag;
	if (skb->len + 4 < 64) cw |= (skb->len + 4) << 16; // length field
	if ((pw->flags & PW_FLAGS_TXSEQ) != 0 || frag != PW_FRAG_NONE) {
		cw |= txseq;
	}
	put_unaligned(htonl(cw), (u32 *)skb_push(skb, 4));
}

static u32 pw_alloc_seq(struct pw_priv *pw, unsigned cnt) {
	unsigned v = atomic_add_return(cnt, &pw->txnextseq);
	if (v <= 0xffff) return v - cnt;
	if ((v - cnt) <= 0xffff) {
		// we crossed the line, fixup counter
		atomic_sub(0xffff, &pw->txnextseq);
		return v - cnt;
	}
	return (v - cnt) - 0xffff;
}

static void pw_xmit_frag(struct pw_priv *pw, struct sk_buff *skb,
			struct mpls_route *rt, struct mpls_nh *nh,
			struct net_device *out_dev, unsigned datamtu) {
	// we have to send fragments in correct order therefore create queue
	// of skbuffs and then send it out
	unsigned fragc, txseq;
	unsigned remlen, hroom;
	unsigned char *remdata;
	struct sk_buff_head q;

	fragc = (skb->len + (datamtu - 1)) / datamtu;
	txseq = pw_alloc_seq(pw, fragc);

	__skb_queue_head_init(&q);

	remlen = skb->len - datamtu;
	remdata = skb->data + datamtu;

	hroom = 4 + LL_RESERVED_SPACE(out_dev) + mpls_nh_header_size(nh);

	while (remlen) {
		unsigned fl = remlen > datamtu ? datamtu : remlen;
		struct sk_buff *b = dev_alloc_skb(fl + hroom);
		if (unlikely(!b)) goto fail;

		b->priority = skb->priority;
		b->mark = skb->mark;
		skb_reserve(b, hroom);
		memcpy(skb_put(b, fl), remdata, fl);

		__skb_queue_tail(&q, b);

		remlen -= fl;
		remdata += fl;
	}

	// now deal with orig buffer - unshare it, trim to size we left in it
	// and make sure there is enough headroom
	skb = skb_share_check(skb, GFP_ATOMIC);
	if (unlikely(!skb)) goto fail;

	skb_trim(skb, datamtu);

	if (unlikely(skb_cow(skb, hroom) != 0)) goto fail;

	// all went fine, skb is first fragment, the rest of fragments are in q
	pw_putcw(pw, skb, txseq, PW_FRAG_FIRST);
	pw_xmit_one(pw, skb, rt, nh, out_dev);
	txseq = pw_seq_next(txseq);
	while (1) {
		int last;
		skb = __skb_dequeue(&q);
		last = skb_queue_empty(&q);
		pw_putcw(pw, skb, txseq, last ? PW_FRAG_LAST : PW_FRAG_MID);
		pw_xmit_one(pw, skb, rt, nh, out_dev);
		if (last) break;
		txseq = pw_seq_next(txseq);
	}

	return;

fail:
	kfree_skb(skb);
	while ((skb = __skb_dequeue(&q)) != 0) kfree_skb(skb);
}

static int pw_xmit(struct sk_buff *skb, struct net_device *dev) {
	struct pw_priv *pw = netdev_priv(dev);
	struct net_device *out_dev;
	struct mpls_route *rt = NULL;
	struct mpls_nh *nh = NULL;
	struct pcpu_sw_netstats *stats;
	unsigned len = skb->len;
	unsigned hlen;
	unsigned mtu;
	bool txcw;

	rcu_read_lock();

	if (pw->label != ~0U) rt = mpls_route_input_rcu(pw->net, pw->label);
	if (!rt) goto drop;

	nh = mpls_select_multipath(rt, skb);
	if (!nh) goto drop;

	out_dev = rcu_dereference(nh->nh_dev);
	if (!mpls_output_possible(out_dev)) goto drop;

	txcw = pw->flags & PW_FLAGS_TXCW;

	hlen = mpls_nh_header_size(nh);
	mtu = mpls_dev_mtu(out_dev) - hlen;
	if (mtu < skb->len + (txcw ? 4 : 0)) {
		if ((pw->flags & PW_FLAGS_TXFRAG) == 0) goto drop;

		mtu -= 4; // will need 4 bytes for control word
		pw_xmit_frag(pw, skb, rt, nh, out_dev, mtu);
	}
	else {
		unsigned hroom;

		skb = skb_share_check(skb, GFP_ATOMIC);
		if (!skb) goto drop;

		hroom = LL_RESERVED_SPACE(out_dev) + hlen + (txcw ? 4 : 0);
		if (unlikely(skb_cow(skb, hroom) != 0)) goto drop;

		if (txcw) {
			u32 txseq = (pw->flags & PW_FLAGS_TXSEQ)
				? pw_alloc_seq(pw, 1) : 0;
			pw_putcw(pw, skb, txseq, PW_FRAG_NONE);
		}

		pw_xmit_one(pw, skb, rt, nh, out_dev);
	}

	stats = this_cpu_ptr(dev->tstats);
	u64_stats_update_begin(&stats->syncp);
	stats->tx_packets++;
	stats->tx_bytes += len;
	u64_stats_update_end(&stats->syncp);

	skb_scrub_packet(skb, true);

	rcu_read_unlock();
	return 0;

drop:
	rcu_read_unlock();
	++dev->stats.tx_errors;
	kfree_skb(skb);
	return 0;
}

static const struct net_device_ops pw_ops = {
	.ndo_init               = pw_init,
	.ndo_open               = pw_open,
	.ndo_stop               = pw_close,
	.ndo_start_xmit         = pw_xmit,
	.ndo_change_mtu         = pw_change_mtu,
	.ndo_get_stats64        = ip_tunnel_get_stats64,
	.ndo_set_mac_address    = eth_mac_addr,
	.ndo_features_check     = passthru_features_check,
};

int is_mpls_pw(struct net_device *dev) {
	return dev->netdev_ops == &pw_ops;
}

static void pw_link_setup(struct net_device *dev) {
	ether_setup(dev);

	dev->netdev_ops = &pw_ops;
	dev->needs_free_netdev = true;
	dev->features |= NETIF_F_LLTX;
	dev->priv_destructor = &pw_free;

	dev->priv_flags |= IFF_NO_QUEUE;
	dev->tx_queue_len = 0;
}

static int pw_link_new(struct net *src_net, struct net_device *dev,
			struct nlattr *tb[], struct nlattr *data[],
			struct netlink_ext_ack *extack) {
	struct pw_priv *pw = netdev_priv(dev);

	if (!data || !data[IFLA_PW_FLAGS]) {
		NL_SET_ERR_MSG(extack, "PW flags missing");
		return -EINVAL;
	}

	pw->net = src_net;
	pw->netdev = dev;
	pw->flags = nla_get_u32(data[IFLA_PW_FLAGS]);
	pw->label = ~0U;

	atomic_set(&pw->txnextseq, 1);
	spin_lock_init(&pw->rxlock);
	pw->rxlastseq = 0;
	__skb_queue_head_init(&pw->rxq);

	return register_netdevice(dev);
}

static size_t pw_link_getsize(const struct net_device *dev) {
	return nla_total_size(sizeof(u32));
}

static const struct nla_policy pw_link_policy[IFLA_PW_MAX + 1] = {
	[IFLA_PW_FLAGS] = { .type = NLA_U32 },
};

static int pw_link_fillinfo(struct sk_buff *skb, const struct net_device *dev) {
	struct pw_priv *pw = netdev_priv(dev);
        return nla_put_u32(skb, IFLA_PW_FLAGS, pw->flags);
}

static struct rtnl_link_ops pw_link_ops = {
	.kind		= "pw",
	.priv_size	= sizeof(struct pw_priv),
	.setup		= pw_link_setup,
	.newlink	= pw_link_new,
	.dellink	= unregister_netdevice_queue,

	.maxtype	= IFLA_PW_MAX,
	.get_size	= pw_link_getsize,
	.policy		= pw_link_policy,
	.fill_info	= pw_link_fillinfo,
};

int mpls_pw_init(void) {
	return rtnl_link_register(&pw_link_ops);
}

void mpls_pw_exit(void) {
	rtnl_link_unregister(&pw_link_ops);
}
