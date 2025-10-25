// SPDX-License-Identifier: GPL-2.0
// tests/nodeb_kunit_test.c — KUnit tests for controller_kernel.c core logic (OOT)

#include <linux/module.h>
#include <kunit/test.h>
#include <linux/types.h>
#include "nodeb_test_hooks.h"

/* Minimal Q16.16 helpers for assertions */
typedef s64 q16_16;
#define Q_ONE          ((q16_16)1 << 16)
#define Q_FROM_INT(x)  ((q16_16)(x) << 16)
#define Q_TO_INT(x)    ((int)((x) >> 16))

/* Shallow peek; we only rely on fields used in assertions */
struct ctrl_cfg_peek {
	q16_16 Ts_sp, KpT, KiT, KdT, Kpm, Kim, kawT, kawm, kvw, kwv, tau_d_min_s;
	u16 omega0_rpm, v0_rpm, omega_max_rpm, v_max_rpm, v_cut_rpm;
};
struct ctrl_state_peek { q16_16 eta_T, eta_m, dTh_f, tau_d; };

struct nodeb_ctx_peek {
	struct { struct ctrl_cfg_peek cfg; struct ctrl_state_peek st; } _;
	u16 omega_cmd_rpm, v_cmd_rpm;
	q16_16 Ts, Th, Tc;
	u16 v_prev_rpm; u8 dt_ms; bool have_feedback;
};

/* ---- Test 1: defaults ---- */
static void nodeb_defaults_populates_expected(struct kunit *test)
{
	struct nodeb_ctx *ctx = nodeb_alloc_ctx_for_test();
	struct nodeb_ctx_peek *p = (void *)ctx;

	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, ctx);

	nodeb_test_ctrl_defaults(ctx);
	KUNIT_EXPECT_EQ(test, Q_TO_INT(p->_.cfg.Ts_sp), 25);
	KUNIT_EXPECT_LE(test, abs(Q_TO_INT(p->_.cfg.KpT) - 101), 1);
	KUNIT_EXPECT_EQ(test, p->_.cfg.KiT, Q_ONE/10);
	KUNIT_EXPECT_EQ(test, Q_TO_INT(p->_.cfg.KdT), 4);
	KUNIT_EXPECT_EQ(test, Q_TO_INT(p->_.cfg.Kpm), 130);
	KUNIT_EXPECT_EQ(test, p->_.cfg.Kim, Q_ONE/100);
	KUNIT_EXPECT_EQ(test, Q_TO_INT(p->_.cfg.kawT), 5);
	KUNIT_EXPECT_EQ(test, Q_TO_INT(p->_.cfg.kawm), 10);
	KUNIT_EXPECT_EQ(test, p->_.cfg.omega0_rpm, 100);
	KUNIT_EXPECT_EQ(test, p->_.cfg.v0_rpm, 100);
	KUNIT_EXPECT_EQ(test, p->_.cfg.omega_max_rpm, 4000);
	KUNIT_EXPECT_EQ(test, p->_.cfg.v_max_rpm, 2800);
	KUNIT_EXPECT_EQ(test, p->_.cfg.v_cut_rpm, 700);
	KUNIT_EXPECT_EQ(test, p->_.cfg.tau_d_min_s, Q_ONE/1000);

	nodeb_free_ctx_for_test(ctx);
}

/* ---- Test 2: inject a normal 0x202 and check outputs/clamps ---- */
static void nodeb_step_basic_behavior(struct kunit *test)
{
	struct nodeb_ctx *ctx = nodeb_alloc_ctx_for_test();
	struct nodeb_ctx_peek *p = (void *)ctx;
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, ctx);

	/* Ts=20.0C, Th=25.0C, Tc=22.5C, v_prev=1200 rpm, dt=10 ms */
	nodeb_test_inject_0x202(ctx, 200, 250, 225, 120, 10);

	KUNIT_EXPECT_TRUE(test, p->have_feedback);
	KUNIT_EXPECT_EQ(test, p->dt_ms, 10);
	KUNIT_EXPECT_EQ(test, p->v_prev_rpm, 1200);
	KUNIT_EXPECT_GE(test, p->omega_cmd_rpm, 0);
	KUNIT_EXPECT_GE(test, p->v_cmd_rpm, 0);
	KUNIT_EXPECT_LE(test, p->omega_cmd_rpm, 4000);
	KUNIT_EXPECT_LE(test, p->v_cmd_rpm, 2800);

	nodeb_free_ctx_for_test(ctx);
}

/* ---- Test 3: dt=0 coerced to 1; large errors clamp ---- */
static void nodeb_ingest_edge_cases(struct kunit *test)
{
	struct nodeb_ctx *ctx = nodeb_alloc_ctx_for_test();
	struct nodeb_ctx_peek *p = (void *)ctx;
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, ctx);

	nodeb_test_inject_0x202(ctx, 250, 250, 250, 0, 0); /* dt=0 -> 1 */
	KUNIT_EXPECT_TRUE(test, p->have_feedback);
	KUNIT_EXPECT_EQ(test, p->dt_ms, 1);

	nodeb_test_inject_0x202(ctx, 100, 50, 50, 10, 10); /* big error */
	KUNIT_EXPECT_LE(test, p->omega_cmd_rpm, 4000);
	KUNIT_EXPECT_LE(test, p->v_cmd_rpm, 2800);

	nodeb_free_ctx_for_test(ctx);
}

static struct kunit_case nodeb_kunit_cases[] = {
	KUNIT_CASE(nodeb_defaults_populates_expected),
	KUNIT_CASE(nodeb_step_basic_behavior),
	KUNIT_CASE(nodeb_ingest_edge_cases),
	{}
};

static struct kunit_suite nodeb_kunit_test_suite = {
	.name = "nodeb-controller-kunit-oot",
	.test_cases = nodeb_kunit_cases,
};
kunit_test_suites(&nodeb_kunit_test_suite);
MODULE_AUTHOR("Alireza");
MODULE_DESCRIPTION("KUnit tests for NodeB controller (out-of-tree)");
MODULE_LICENSE("GPL");