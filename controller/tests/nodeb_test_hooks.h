#pragma once
#include <linux/types.h>

struct nodeb_ctx;

#if IS_ENABLED(CONFIG_KUNIT)
struct nodeb_ctx *nodeb_alloc_ctx_for_test(void);
void nodeb_free_ctx_for_test(struct nodeb_ctx *ctx);
void nodeb_test_ctrl_defaults(struct nodeb_ctx *ctx);
void nodeb_test_controller_step(struct nodeb_ctx *ctx);
void nodeb_test_inject_0x202(struct nodeb_ctx *ctx,
			     s16 Ts_q01, s16 Th_q01, s16 Tc_q01,
			     u8 vprev_q10, u8 dt_ms);
#endif