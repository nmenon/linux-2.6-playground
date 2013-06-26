/*
 * Low Latency Sockets
 * Copyright(c) 2013 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Author: Eliezer Tamir
 *
 * Contact Information:
 * e1000-devel Mailing List <e1000-devel@lists.sourceforge.net>
 */

#ifndef _LINUX_NET_LL_POLL_H
#define _LINUX_NET_LL_POLL_H

#include <linux/netdevice.h>
#include <net/ip.h>

#ifdef CONFIG_NET_LL_RX_POLL

struct napi_struct;
extern unsigned int sysctl_net_ll_read __read_mostly;
extern unsigned int sysctl_net_ll_poll __read_mostly;

/* return values from ndo_ll_poll */
#define LL_FLUSH_FAILED		-1
#define LL_FLUSH_BUSY		-2

/* we can use sched_clock() because we don't care much about precision
 * we only care that the average is bounded
 * we don't mind a ~2.5% imprecision so <<10 instead of *1000
 * sk->sk_ll_usec is a u_int so this can't overflow
 */
static inline u64 ll_sk_end_time(struct sock *sk)
{
	return ((u64)ACCESS_ONCE(sk->sk_ll_usec) << 10) + sched_clock();
}

/* in poll/select we use the global sysctl_net_ll_poll value */
static inline u64 ll_end_time(void)
{
	return ((u64)ACCESS_ONCE(sysctl_net_ll_poll) << 10) + sched_clock();
}

static inline bool sk_valid_ll(struct sock *sk)
{
	return sk->sk_ll_usec && sk->sk_napi_id &&
	       !need_resched() && !signal_pending(current);
}

static inline bool can_poll_ll(u64 end_time)
{
	return !time_after64(sched_clock(), end_time);
}

/* when used in sock_poll() nonblock is known at compile time to be true
 * so the loop and end_time will be optimized out
 */
static inline bool sk_poll_ll(struct sock *sk, int nonblock)
{
	u64 end_time = nonblock ? 0 : ll_sk_end_time(sk);
	const struct net_device_ops *ops;
	struct napi_struct *napi;
	int rc = false;

	/*
	 * rcu read lock for napi hash
	 * bh so we don't race with net_rx_action
	 */
	rcu_read_lock_bh();

	napi = napi_by_id(sk->sk_napi_id);
	if (!napi)
		goto out;

	ops = napi->dev->netdev_ops;
	if (!ops->ndo_ll_poll)
		goto out;

	do {
		rc = ops->ndo_ll_poll(napi);

		if (rc == LL_FLUSH_FAILED)
			break; /* permanent failure */

		if (rc > 0)
			/* local bh are disabled so it is ok to use _BH */
			NET_ADD_STATS_BH(sock_net(sk),
					 LINUX_MIB_LOWLATENCYRXPACKETS, rc);

	} while (!nonblock && skb_queue_empty(&sk->sk_receive_queue) &&
		 can_poll_ll(end_time));

	rc = !skb_queue_empty(&sk->sk_receive_queue);
out:
	rcu_read_unlock_bh();
	return rc;
}

/* used in the NIC receive handler to mark the skb */
static inline void skb_mark_ll(struct sk_buff *skb, struct napi_struct *napi)
{
	skb->napi_id = napi->napi_id;
}

/* used in the protocol hanlder to propagate the napi_id to the socket */
static inline void sk_mark_ll(struct sock *sk, struct sk_buff *skb)
{
	sk->sk_napi_id = skb->napi_id;
}

#else /* CONFIG_NET_LL_RX_POLL */

static inline u64 sk_ll_end_time(struct sock *sk)
{
	return 0;
}

static inline u64 ll_end_time(void)
{
	return 0;
}

static inline bool sk_valid_ll(struct sock *sk)
{
	return false;
}

static inline bool sk_poll_ll(struct sock *sk, int nonblock)
{
	return false;
}

static inline void skb_mark_ll(struct sk_buff *skb, struct napi_struct *napi)
{
}

static inline void sk_mark_ll(struct sock *sk, struct sk_buff *skb)
{
}

static inline bool can_poll_ll(u64 end_time)
{
	return false;
}

#endif /* CONFIG_NET_LL_RX_POLL */
#endif /* _LINUX_NET_LL_POLL_H */
