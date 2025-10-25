// SPDX-License-Identifier: GPL-2.0
// controller_kernel.c — Node B with ISR-like RX using can_rx_register() + bottom half
// Build:  make -C /lib/modules/$(uname -r)/build M=$PWD modules
// Load:   sudo insmod controller_kernel.ko ifname=vcan0 period_ms=100 idle_ms=1500
// Unload: sudo rmmod controller_kernel
// Show:   dmesg -w | grep -E '^\[B\]| nodeb'

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/version.h>

#include <linux/netdevice.h>
#include <linux/net.h>
#include <linux/socket.h>
#include <linux/sockptr.h>
#include <net/sock.h>

#include <linux/spinlock.h>
#include <linux/kfifo.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>   /* s64/u64 */

#include <linux/can.h>
#include <linux/can/core.h>  /* can_rx_register / unregister */
#include <linux/can/raw.h>

#if IS_ENABLED(CONFIG_KUNIT)
#include <kunit/test.h>
#endif


/* Module parameters */
static char ifname[IFNAMSIZ] = "vcan0";
module_param_string(ifname, ifname, sizeof(ifname), 0644);
MODULE_PARM_DESC(ifname, "CAN interface name (e.g., vcan0, can0)");

static int period_ms = 1000;
module_param(period_ms, int, 0644);
MODULE_PARM_DESC(period_ms, "TX period (ms) for CAN ID 0x201");

/* NEW: idle window — stop TX if no 0x202 arrives for this long */
static int idle_ms = 3000;
module_param(idle_ms, int, 0644);
MODULE_PARM_DESC(idle_ms, "Idle window (ms) without 0x202 before stopping TX");

#if IS_ENABLED(CONFIG_KUNIT)
/* When true, skip netdev hooks/sockets/timers to allow pure-logic KUnit runs */
static bool kunit_no_hw = true;
module_param(kunit_no_hw, bool, 0644);
MODULE_PARM_DESC(kunit_no_hw, "Skip HW/net/timers for KUnit (pure logic only)");
#endif

/* If your tree added can_rx_register() flags in a different minor, adjust here */
#ifndef CAN_RX_REG_NEEDS_FLAGS
# if LINUX_VERSION_CODE >= KERNEL_VERSION(6,12,0)
#  define CAN_RX_REG_NEEDS_FLAGS 1
# else
#  define CAN_RX_REG_NEEDS_FLAGS 0
# endif
#endif

#define RX_FIFO_ELEMS 128

struct rx_item {
	struct can_frame cf;
};

/* -------------------------- Q16.16 fixed-point helpers ----------------- */
typedef s64 q16_16;
#define Q_ONE          ((q16_16)1 << 16)
#define Q_FROM_INT(x)  ((q16_16)(x) << 16)
#define Q_TO_INT(x)    ((int)((x) >> 16))
#define Q_MUL(a,b)     ((q16_16)(((s64)(a) * (s64)(b)) >> 16))
#define Q_DIV(a,b)     ((q16_16)(((s64)(a) << 16) / (s64)(b)))
static inline q16_16 q_sat(q16_16 x, q16_16 lo, q16_16 hi)
{ return x < lo ? lo : (x > hi ? hi : x); }

/* LE helpers + temp converter */
static inline s16 le_to_s16(const u8 *d){ return (s16)((u16)d[0] | ((u16)d[1] << 8)); }
static inline u16 le_to_u16(const u8 *d){ return (u16)d[0] | ((u16)d[1] << 8); }
static inline void u16_to_le(u8 *dst, u16 v){ dst[0]=(u8)(v & 0xFF); dst[1]=(u8)(v>>8); }
/* 0.1°C -> Q16.16 °C */
static inline q16_16 q_from_q01_temp(s16 t_q01)
{ return (q16_16)(((s64)t_q01 * (s64)Q_ONE) / 10); }

/* -------------------------- Controller config/state -------------------- */
struct ctrl_cfg {
	q16_16 Ts_sp;           /* °C (Q16.16); default 25 */

	/* Gains in Q16.16 */
	q16_16 KpT, KiT, KdT;   /* temperature loop */
	q16_16 Kpm, Kim;        /* flow loop       */
	q16_16 kawT, kawm;      /* anti-windup back-calculation */
	q16_16 kvw,  kwv;       /* decoupling */

	/* Integer domains */
	u16 omega0_rpm, v0_rpm; /* feedforward baselines */
	u16 omega_max_rpm, v_max_rpm;
	u16 v_cut_rpm;          /* fan cut-in (e.g., 700 rpm) */

	q16_16 tau_d_min_s;     /* min derivative filter time constant (>= 1e-3 s) */
};

struct ctrl_state {
	q16_16 eta_T, eta_m;    /* integrators */
	q16_16 dTh_f;           /* derivative filter state (°C in Q16.16) */
	q16_16 tau_d;           /* current tau_d (s in Q16.16) */
};

/* -------------------------- Node-B context ----------------------------- */
struct nodeb_ctx {
	/* TX */
	struct socket *tx_sock;
	int ifindex;
	struct hrtimer tx_timer;
	ktime_t period;
	u8 seq;

	/* NEW: inactivity guard to stop TX when node C is silent */
	struct hrtimer rx_guard;
	ktime_t        idle_period;

	/* RX (ISR-like + BH) */
	DECLARE_KFIFO(rx_fifo, struct rx_item, RX_FIFO_ELEMS);
	spinlock_t rx_lock;
	struct workqueue_struct *wq;
	struct work_struct rx_work;

	/* simple state */
	int state;

	/* Controller additions */
	struct ctrl_cfg   cfg;
	struct ctrl_state st;

	/* Latest plant feedback */
	q16_16 Ts, Th, Tc;          /* °C (Q16.16) */
	u16    v_prev_rpm;          /* rpm */
	u8     dt_ms;               /* 1..255 ms */
	bool   have_feedback;

	/* Last computed command */
	u16 omega_cmd_rpm;
	u16 v_cmd_rpm;
};

static struct nodeb_ctx *g;

static void nodeb_print_cf(const char *tag, const struct can_frame *cf)
{
	char buf[3 * 8 + 1];
	int i, pos = 0;
	for (i = 0; i < cf->len && i < 8; i++)
		pos += scnprintf(buf + pos, sizeof(buf) - pos, "%02X ", cf->data[i]);
	buf[sizeof(buf) - 1] = '\0';

	pr_info("[B] %s 0x%03X [%d] %s\n",
		tag, cf->can_id & CAN_SFF_MASK, cf->len, buf);
}

/* -------------------------- Defaults ----------------------------------- */
static void ctrl_defaults(struct nodeb_ctx *ctx)
{
	ctx->cfg.Ts_sp = Q_FROM_INT(25);

	/* Python: KpT=100.6, KiT=0.10, KdT=4.0 */
	ctx->cfg.KpT = Q_FROM_INT(100) + (Q_ONE/5 + Q_ONE/10); /* ≈100.6 */
	ctx->cfg.KiT = Q_ONE/10;     /* 0.10 */
	ctx->cfg.KdT = Q_FROM_INT(4);

	ctx->cfg.Kpm = Q_FROM_INT(130);
	ctx->cfg.Kim = Q_ONE/100;    /* 0.01 */

	ctx->cfg.kawT = Q_FROM_INT(5);
	ctx->cfg.kawm = Q_FROM_INT(10);

	/* kvw≈-0.15, kwv≈-0.02 */
	ctx->cfg.kvw = -(Q_ONE/6 + Q_ONE/30);
	ctx->cfg.kwv = -(Q_ONE/50);

	ctx->cfg.omega0_rpm = 100;
	ctx->cfg.v0_rpm     = 100;

	ctx->cfg.omega_max_rpm = 4000;
	ctx->cfg.v_max_rpm     = 2800;
	ctx->cfg.v_cut_rpm     = 700;

	ctx->cfg.tau_d_min_s   = Q_ONE/1000; /* 0.001 s minimum */
}

/* -------------------------- Controller core ---------------------------- */
static void controller_step(struct nodeb_ctx *ctx)
{
	if (!ctx->have_feedback || ctx->dt_ms == 0) {
		ctx->omega_cmd_rpm = 0;
		ctx->v_cmd_rpm     = 0;
		return;
	}

	/* dt seconds in Q16.16 */
	q16_16 dt = Q_DIV(Q_FROM_INT(ctx->dt_ms), Q_FROM_INT(1000));

	/* ----- Flow loop (pump) ----- */
	q16_16 e_m = ctx->cfg.Ts_sp - ctx->Ts; /* Ts_sp - Ts */

	q16_16 omega0_q = Q_FROM_INT(ctx->cfg.omega0_rpm);
	q16_16 omega_raw_q = -(omega0_q
		+ Q_MUL(ctx->cfg.Kpm, e_m)
		+ Q_MUL(ctx->cfg.Kim, ctx->st.eta_m));

	q16_16 v_prev_q = Q_FROM_INT(ctx->v_prev_rpm);
	q16_16 v_ff_q   = Q_FROM_INT(ctx->cfg.v0_rpm);
	q16_16 omega_cmd_q = omega_raw_q + Q_MUL(ctx->cfg.kwv, (v_prev_q - v_ff_q));

	int omega_cmd_i = Q_TO_INT(omega_cmd_q);
	if (omega_cmd_i < 0) omega_cmd_i = 0;
	if (omega_cmd_i > ctx->cfg.omega_max_rpm) omega_cmd_i = ctx->cfg.omega_max_rpm;

	q16_16 omega_cmd_q16 = Q_FROM_INT(omega_cmd_i);
	q16_16 omega_err_q   = omega_cmd_q16 - omega_raw_q;
	ctx->st.eta_m += Q_MUL( (e_m + Q_MUL(ctx->cfg.kawm, omega_err_q)), dt );
	ctx->st.eta_m  = q_sat(ctx->st.eta_m, Q_FROM_INT(-200), Q_FROM_INT(200));

	/* ----- Temperature loop (fan) ----- */
	q16_16 e_T = ctx->cfg.Ts_sp - ctx->Ts;

	if (ctx->st.tau_d < ctx->cfg.tau_d_min_s) ctx->st.tau_d = ctx->cfg.tau_d_min_s;

	q16_16 Th_minus = ctx->Th - ctx->st.dTh_f;
	q16_16 term1 = Q_DIV(Th_minus, dt);                 /* (Th - dTh_f)/dt */
	q16_16 term2 = Q_DIV(ctx->st.dTh_f, ctx->st.tau_d); /* dTh_f / tau_d */
	ctx->st.dTh_f += Q_MUL((term1 - term2), dt);

	q16_16 v0_q = Q_FROM_INT(ctx->cfg.v0_rpm);
	q16_16 v_raw_q = -(v0_q
		+ Q_MUL(ctx->cfg.KpT, e_T)
		+ Q_MUL(ctx->cfg.KiT, ctx->st.eta_T)
		- Q_MUL(ctx->cfg.KdT, ctx->st.dTh_f));

	q16_16 omega_ff_q = Q_FROM_INT(ctx->cfg.omega0_rpm);
	q16_16 v_cmd_q = v_raw_q + Q_MUL(ctx->cfg.kvw, (omega_cmd_q16 - omega_ff_q));

	int v_cmd_i = Q_TO_INT(v_cmd_q);
	if (v_cmd_i < 0) v_cmd_i = 0;
	if (v_cmd_i > ctx->cfg.v_max_rpm) v_cmd_i = ctx->cfg.v_max_rpm;
	if (v_cmd_i < ctx->cfg.v_cut_rpm) v_cmd_i = 0;

	q16_16 v_cmd_q16 = Q_FROM_INT(v_cmd_i);
	q16_16 v_err_q   = v_cmd_q16 - v_raw_q;
	ctx->st.eta_T += Q_MUL( (e_T + Q_MUL(ctx->cfg.kawT, v_err_q)), dt );
	ctx->st.eta_T  = q_sat(ctx->st.eta_T, Q_FROM_INT(-500), Q_FROM_INT(500));

	ctx->omega_cmd_rpm = (u16)omega_cmd_i;
	ctx->v_cmd_rpm     = (u16)v_cmd_i;
}

/* -------------------------- RX bottom-half ----------------------------- */
static void nodeb_rx_work(struct work_struct *work)
{
	unsigned long flags;
	struct rx_item item;

	for (;;) {
		int copied;

		spin_lock_irqsave(&g->rx_lock, flags);
		copied = kfifo_out(&g->rx_fifo, &item, 1);
		spin_unlock_irqrestore(&g->rx_lock, flags);

		if (copied != 1)
			break;

		nodeb_print_cf("RX", &item.cf);
		switch (item.cf.can_id & CAN_SFF_MASK) {
		case 0x101:
			g->state = 1;
			break;

		case 0x202: { /* Plant feedback: Ts,Th,Tc,v_prev,dt */
			if (item.cf.len == 8) {
				s16 Ts_q01 = le_to_s16(&item.cf.data[0]);
				s16 Th_q01 = le_to_s16(&item.cf.data[2]);
				s16 Tc_q01 = le_to_s16(&item.cf.data[4]);
				u8  vprev_q10 = item.cf.data[6];
				u8  dt_ms     = item.cf.data[7] ? item.cf.data[7] : 1;

				g->Ts = q_from_q01_temp(Ts_q01);
				g->Th = q_from_q01_temp(Th_q01);
				g->Tc = q_from_q01_temp(Tc_q01);
				g->v_prev_rpm = (u16)(vprev_q10 * 10u);
				g->dt_ms = dt_ms;
				g->have_feedback = true;

				controller_step(g);   /* compute omega_cmd/v_cmd now */

				/* NEW: start TX timer on demand after 0x202 */
				if (!hrtimer_active(&g->tx_timer)) {
					hrtimer_start(&g->tx_timer, g->period,
					              HRTIMER_MODE_REL_PINNED);
					pr_info("[B] TX timer started after 0x202\n");
				}
				/* NEW: (re)arm inactivity guard */
				hrtimer_start(&g->rx_guard, g->idle_period,
				              HRTIMER_MODE_REL_PINNED);
			}
			g->state = 2;
			break;
		}

		case 0x301: { /* setpoint from node A */
			if (item.cf.len >= 2) {
				s16 Ts_sp_q01 = le_to_s16(&item.cf.data[0]);
				g->cfg.Ts_sp = q_from_q01_temp(Ts_sp_q01);
				pr_info("[B] Ts_sp set to %d.%01d C\n",
				        Ts_sp_q01/10, abs(Ts_sp_q01%10));
			}
			break;
		}

		case 0x300: { /* optional hyperparameters */
			/* [0..1] KpT q8.8, [2..3] KiT q8.8, [4..5] KdT q8.8, [6] kawT q4.4 */
			if (item.cf.len >= 7) {
				u16 kp = le_to_u16(&item.cf.data[0]);
				u16 ki = le_to_u16(&item.cf.data[2]);
				u16 kd = le_to_u16(&item.cf.data[4]);
				u8  kaw= item.cf.data[6];
				g->cfg.KpT  = (q16_16)kp << 8;   /* q8.8 -> Q16.16 */
				g->cfg.KiT  = (q16_16)ki << 8;
				g->cfg.KdT  = (q16_16)kd << 8;
				g->cfg.kawT = (q16_16)kaw << 12; /* q4.4 -> Q16.16 */
				pr_info("[B] Gains updated via 0x300\n");
			}
			break;
		}

		case 0x302: { /* Kpm/Kim (q8.8), kawm/kvw/kwv (q4.4) */
			if (item.cf.len >= 7) {
				u16 kpm = le_to_u16(&item.cf.data[0]);
				u16 kim = le_to_u16(&item.cf.data[2]);
				u8  kawm= item.cf.data[4];
				u8  kvw = item.cf.data[5];
				u8  kwv = item.cf.data[6];
				g->cfg.Kpm = (q16_16)kpm << 8;   /* q8.8 -> Q16.16 */
				g->cfg.Kim = (q16_16)kim << 8;
				g->cfg.kawm= (q16_16)kawm << 12; /* q4.4 -> Q16.16 */
				g->cfg.kvw = (q16_16)kvw  << 12;
				g->cfg.kwv = (q16_16)kwv  << 12;
				pr_info("[B] Flow/decouple gains updated via 0x302\n");
			}
			break;
		}

		default:
			break;
		}
	}
}

/* -------------------------- RX "ISR-like" callback --------------------- */
static void nodeb_can_rx_cb(struct sk_buff *skb, void *data)
{
	struct nodeb_ctx *ctx = data;
	const struct can_frame *cf;
	struct rx_item it;
	unsigned long flags;

	if (unlikely(!skb))
		return;

	if (skb->len < sizeof(struct can_frame))
		return;

	cf = (const struct can_frame *)skb->data;
	memcpy(&it.cf, cf, sizeof(*cf));

	spin_lock_irqsave(&ctx->rx_lock, flags);
	if (!kfifo_is_full(&ctx->rx_fifo))
		kfifo_in(&ctx->rx_fifo, &it, 1);
	else
		pr_warn("[B] RX FIFO overflow; dropping\n");
	spin_unlock_irqrestore(&ctx->rx_lock, flags);

	queue_work(ctx->wq, &ctx->rx_work);
}

/* -------------------------- Register/unregister RX --------------------- */
static int nodeb_register_rx(struct nodeb_ctx *ctx)
{
	int ret = 0;
	struct net_device *dev;

	rcu_read_lock();
	dev = dev_get_by_name_rcu(&init_net, ifname);
	if (!dev) {
		rcu_read_unlock();
		pr_err("[B] netdev %s not found\n", ifname);
		return -ENODEV;
	}
	ctx->ifindex = dev->ifindex;
	rcu_read_unlock();

	dev = dev_get_by_index(&init_net, ctx->ifindex);
	if (!dev)
		return -ENODEV;

#if CAN_RX_REG_NEEDS_FLAGS
	ret = can_rx_register(&init_net, dev, 0x101, CAN_SFF_MASK,
	                      nodeb_can_rx_cb, ctx, "nodeb-0x101", 0);
#else
	ret = can_rx_register(&init_net, dev, 0x101, CAN_SFF_MASK,
	                      nodeb_can_rx_cb, ctx, "nodeb-0x101");
#endif
	if (ret) {
		pr_err("[B] can_rx_register 0x101 failed: %d\n", ret);
		dev_put(dev);
		return ret;
	}

#if CAN_RX_REG_NEEDS_FLAGS
	ret = can_rx_register(&init_net, dev, 0x202, CAN_SFF_MASK,
	                      nodeb_can_rx_cb, ctx, "nodeb-0x202", 0);
#else
	ret = can_rx_register(&init_net, dev, 0x202, CAN_SFF_MASK,
	                      nodeb_can_rx_cb, ctx, "nodeb-0x202");
#endif
	if (ret) {
		struct net_device *d2 = dev_get_by_index(&init_net, ctx->ifindex);
		if (d2) {
			can_rx_unregister(&init_net, d2, 0x101, CAN_SFF_MASK,
			                  nodeb_can_rx_cb, ctx);
			dev_put(d2);
		}
		pr_err("[B] can_rx_register 0x202 failed: %d\n", ret);
		dev_put(dev);
		return ret;
	}

#if CAN_RX_REG_NEEDS_FLAGS
	ret = can_rx_register(&init_net, dev, 0x301, CAN_SFF_MASK,
	                      nodeb_can_rx_cb, ctx, "nodeb-0x301", 0);
#else
	ret = can_rx_register(&init_net, dev, 0x301, CAN_SFF_MASK,
	                      nodeb_can_rx_cb, ctx, "nodeb-0x301");
#endif
	if (ret) {
		struct net_device *d2 = dev_get_by_index(&init_net, ctx->ifindex);
		if (d2) {
			can_rx_unregister(&init_net, d2, 0x202, CAN_SFF_MASK, nodeb_can_rx_cb, ctx);
			can_rx_unregister(&init_net, d2, 0x101, CAN_SFF_MASK, nodeb_can_rx_cb, ctx);
			dev_put(d2);
		}
		pr_err("[B] can_rx_register 0x301 failed: %d\n", ret);
		dev_put(dev);
		return ret;
	}

#if CAN_RX_REG_NEEDS_FLAGS
	ret = can_rx_register(&init_net, dev, 0x300, CAN_SFF_MASK,
	                      nodeb_can_rx_cb, ctx, "nodeb-0x300", 0);
#else
	ret = can_rx_register(&init_net, dev, 0x300, CAN_SFF_MASK,
	                      nodeb_can_rx_cb, ctx, "nodeb-0x300");
#endif
	dev_put(dev);

	if (ret) {
		struct net_device *d2 = dev_get_by_index(&init_net, ctx->ifindex);
		if (d2) {
			can_rx_unregister(&init_net, d2, 0x301, CAN_SFF_MASK, nodeb_can_rx_cb, ctx);
			can_rx_unregister(&init_net, d2, 0x202, CAN_SFF_MASK, nodeb_can_rx_cb, ctx);
			can_rx_unregister(&init_net, d2, 0x101, CAN_SFF_MASK, nodeb_can_rx_cb, ctx);
			dev_put(d2);
		}
		pr_err("[B] can_rx_register 0x300 failed: %d\n", ret);
		return ret;
	}


#if CAN_RX_REG_NEEDS_FLAGS
	ret = can_rx_register(&init_net, dev, 0x302, CAN_SFF_MASK,
	                      nodeb_can_rx_cb, ctx, "nodeb-0x302", 0);
#else
	ret = can_rx_register(&init_net, dev, 0x302, CAN_SFF_MASK,
	                      nodeb_can_rx_cb, ctx, "nodeb-0x302");
#endif
	dev_put(dev);

	if (ret) {
		struct net_device *d2 = dev_get_by_index(&init_net, ctx->ifindex);
		if (d2) {
			can_rx_unregister(&init_net, d2, 0x300, CAN_SFF_MASK, nodeb_can_rx_cb, ctx);
			can_rx_unregister(&init_net, d2, 0x301, CAN_SFF_MASK, nodeb_can_rx_cb, ctx);
			can_rx_unregister(&init_net, d2, 0x202, CAN_SFF_MASK, nodeb_can_rx_cb, ctx);
			can_rx_unregister(&init_net, d2, 0x101, CAN_SFF_MASK, nodeb_can_rx_cb, ctx);
			dev_put(d2);
		}
		pr_err("[B] can_rx_register 0x302 failed: %d\n", ret);
		return ret;
	}

	pr_info("[B] RX hooks registered on %s (ifindex=%d)\n", ifname, ctx->ifindex);
	return 0;
}

static void nodeb_unregister_rx(struct nodeb_ctx *ctx)
{
	struct net_device *dev = dev_get_by_index(&init_net, ctx->ifindex);
	if (!dev)
		return;

	can_rx_unregister(&init_net, dev, 0x101, CAN_SFF_MASK, nodeb_can_rx_cb, ctx);
	can_rx_unregister(&init_net, dev, 0x202, CAN_SFF_MASK, nodeb_can_rx_cb, ctx);
	can_rx_unregister(&init_net, dev, 0x301, CAN_SFF_MASK, nodeb_can_rx_cb, ctx);
	can_rx_unregister(&init_net, dev, 0x300, CAN_SFF_MASK, nodeb_can_rx_cb, ctx);
	can_rx_unregister(&init_net, dev, 0x302, CAN_SFF_MASK, nodeb_can_rx_cb, ctx);
	dev_put(dev);
}

/* -------------------------- TX timer ----------------------------------- */
static enum hrtimer_restart nodeb_tx_timer_fn(struct hrtimer *t)
{
	struct can_frame cf = {0};
	struct msghdr msg = {0};
	struct kvec iov;
	int ret;

	cf.can_id = 0x201;
	cf.len    = 8;

	/* Payload: controller outputs to plant */
	u16_to_le(&cf.data[0], g->omega_cmd_rpm);  /* bytes 0..1: omega_cmd rpm LE */
	u16_to_le(&cf.data[2], g->v_cmd_rpm);      /* bytes 2..3: v_cmd rpm LE */

	iov.iov_base = &cf;
	iov.iov_len  = sizeof(cf);

	ret = kernel_sendmsg(g->tx_sock, &msg, &iov, 1, sizeof(cf));
	if (ret >= 0) {
		nodeb_print_cf("TX", &cf);
		g->seq++;
	} else {
		pr_warn("[B] kernel_sendmsg() failed: %d\n", ret);
	}

	hrtimer_forward_now(&g->tx_timer, g->period);
	return HRTIMER_RESTART;
}

/* NEW: Inactivity guard callback — fires when no 0x202 within idle_period */
static enum hrtimer_restart nodeb_rx_guard_fn(struct hrtimer *t)
{
	if (hrtimer_active(&g->tx_timer)) {
		hrtimer_cancel(&g->tx_timer);
		pr_info("[B] TX timer stopped due to 0x202 inactivity\n");
	}
	/* one-shot guard; rearmed on next 0x202 */
	return HRTIMER_NORESTART;
}

/* -------------------------- TX socket helpers -------------------------- */
static int nodeb_open_tx_socket(struct nodeb_ctx *ctx)
{
	int ret;
	struct sockaddr_can addr = {0};
	struct net_device *dev;

	ret = sock_create_kern(&init_net, AF_CAN, SOCK_RAW, CAN_RAW, &ctx->tx_sock);
	if (ret) {
		pr_err("[B] sock_create_kern failed: %d\n", ret);
		return ret;
	}

	rcu_read_lock();
	dev = dev_get_by_name_rcu(&init_net, ifname);
	if (!dev) {
		rcu_read_unlock();
		pr_err("[B] no such netdev: %s\n", ifname);
		return -ENODEV;
	}
	ctx->ifindex = dev->ifindex;
	rcu_read_unlock();

	addr.can_family  = AF_CAN;
	addr.can_ifindex = ctx->ifindex;

	ret = kernel_bind(ctx->tx_sock, (struct sockaddr *)&addr, sizeof(addr));
	if (ret) {
		pr_err("[B] bind failed: %d\n", ret);
		return ret;
	}
	return 0;
}

/* -------------------------- Module init/exit --------------------------- */

static int __init nodeb_init(void)
{
	int ret;

	g = kzalloc(sizeof(*g), GFP_KERNEL);
	if (!g)
		return -ENOMEM;

	ctrl_defaults(g);
	g->st.eta_T = 0; g->st.eta_m = 0;
	g->st.dTh_f = 0; g->st.tau_d = Q_FROM_INT(1); /* start tau_d=1s; clamped by tau_d_min_s */
	g->omega_cmd_rpm = 0; g->v_cmd_rpm = 0; g->have_feedback = false;

	INIT_KFIFO(g->rx_fifo);
	spin_lock_init(&g->rx_lock);
	INIT_WORK(&g->rx_work, nodeb_rx_work);

	#if IS_ENABLED(CONFIG_KUNIT)
	if (kunit_no_hw) {
		/* Don’t touch netdev / sockets / timers / wq start – keep module loadable */
		pr_info("[B] KUnit: loaded with kunit_no_hw=1 (logic-only)\n");
		return 0;
	}
	#endif

	ret = nodeb_register_rx(g);
	if (ret)
		goto err_free;

	ret = nodeb_open_tx_socket(g);
	if (ret)
		goto err_unreg;

	g->seq = 0;
	g->period = ktime_set(0, (s64)period_ms * 1000000LL);
	hrtimer_init(&g->tx_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_PINNED);
	g->tx_timer.function = nodeb_tx_timer_fn;
	/* NOTE: do NOT start TX timer here; it starts after first 0x202 */

	/* NEW: initialize inactivity guard (one-shot) */
	g->idle_period = ktime_set(0, (s64)idle_ms * 1000000LL);
	hrtimer_init(&g->rx_guard, CLOCK_MONOTONIC, HRTIMER_MODE_REL_PINNED);
	g->rx_guard.function = nodeb_rx_guard_fn;

	/* Ordered BH workqueue */
	g->wq = alloc_ordered_workqueue("nodeb_wq", 0);
	if (!g->wq) {
		ret = -ENOMEM;
		pr_err("[B] alloc_ordered_workqueue failed\n");
		goto err_timer;
	}

	pr_info("[B] started on %s: RX via can_rx_register(0x101/0x202/0x302/0x301/0x300), TX 0x201 period %d ms (armed on 0x202, idle %d ms)\n",
	        ifname, period_ms, idle_ms);
	return 0;

err_timer:
	hrtimer_cancel(&g->tx_timer);
	if (g->tx_sock) {
		kernel_sock_shutdown(g->tx_sock, SHUT_RDWR);
		sock_release(g->tx_sock);
	}
err_unreg:
	nodeb_unregister_rx(g);
err_free:
	kfree(g);
	return ret;
}

static void __exit nodeb_exit(void)
{
	if (!g)
		return;

	hrtimer_cancel(&g->rx_guard);   /* NEW */
	hrtimer_cancel(&g->tx_timer);

	if (g->wq) {
		flush_workqueue(g->wq);
		destroy_workqueue(g->wq);
	}

	if (g->tx_sock) {
		kernel_sock_shutdown(g->tx_sock, SHUT_RDWR);
		sock_release(g->tx_sock);
	}

	nodeb_unregister_rx(g);

	kfree(g);
	pr_info("[B] stopped\n");
}

/* ======================= KUnit test hooks (no impact to prod) ======================= */
#if IS_ENABLED(CONFIG_KUNIT)
#include "tests/nodeb_test_hooks.h"

#ifndef __visible_for_testing
#define __visible_for_testing
#endif

__visible_for_testing struct nodeb_ctx *nodeb_alloc_ctx_for_test(void)
{
	struct nodeb_ctx *ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return NULL;
	INIT_KFIFO(ctx->rx_fifo);
	spin_lock_init(&ctx->rx_lock);
	INIT_WORK(&ctx->rx_work, nodeb_rx_work);
	ctrl_defaults(ctx);
	ctx->st.eta_T = 0; ctx->st.eta_m = 0;
	ctx->st.dTh_f = 0; ctx->st.tau_d = Q_FROM_INT(1);
	ctx->omega_cmd_rpm = 0; ctx->v_cmd_rpm = 0;
	ctx->have_feedback = false;
	return ctx;
}
EXPORT_SYMBOL_GPL(nodeb_alloc_ctx_for_test);

__visible_for_testing void nodeb_free_ctx_for_test(struct nodeb_ctx *ctx)
{
	kfree(ctx);
}
EXPORT_SYMBOL_GPL(nodeb_free_ctx_for_test);

__visible_for_testing void nodeb_test_ctrl_defaults(struct nodeb_ctx *ctx)
{
	ctrl_defaults(ctx);
}
EXPORT_SYMBOL_GPL(nodeb_test_ctrl_defaults);

__visible_for_testing void nodeb_test_controller_step(struct nodeb_ctx *ctx)
{
	controller_step(ctx);
}
EXPORT_SYMBOL_GPL(nodeb_test_controller_step);

__visible_for_testing void nodeb_test_inject_0x202(struct nodeb_ctx *ctx,
						   s16 Ts_q01, s16 Th_q01, s16 Tc_q01,
						   u8 vprev_q10, u8 dt_ms)
{
	if (!dt_ms) dt_ms = 1;
	ctx->Ts = q_from_q01_temp(Ts_q01);
	ctx->Th = q_from_q01_temp(Th_q01);
	ctx->Tc = q_from_q01_temp(Tc_q01);
	ctx->v_prev_rpm = (u16)(vprev_q10 * 10u);
	ctx->dt_ms = dt_ms;
	ctx->have_feedback = true;
	controller_step(ctx);
}
EXPORT_SYMBOL_GPL(nodeb_test_inject_0x202);
#endif /* CONFIG_KUNIT */
/* ===================== end KUnit test hooks ======================================== */


module_init(nodeb_init);
module_exit(nodeb_exit);

MODULE_AUTHOR("Alireza");
MODULE_DESCRIPTION("Kernel-space node B with ISR-like RX via can_rx_register (Controller) + 0x202-gated TX timer");
MODULE_LICENSE("GPL");
