#include <linux/module.h>
#include <linux/export-internal.h>
#include <linux/compiler.h>

MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};



static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0x01ca826f, "can_rx_unregister" },
	{ 0xe1e1f979, "_raw_spin_lock_irqsave" },
	{ 0x81a1a811, "_raw_spin_unlock_irqrestore" },
	{ 0x49733ad6, "queue_work_on" },
	{ 0x562e3aaa, "__kfifo_in" },
	{ 0x36a36ab1, "hrtimer_cancel" },
	{ 0xbeb1d261, "__flush_workqueue" },
	{ 0xbeb1d261, "destroy_workqueue" },
	{ 0xae1e4074, "kernel_sock_shutdown" },
	{ 0xdc89bc96, "sock_release" },
	{ 0xcb8b6ec6, "kfree" },
	{ 0x42244bac, "hrtimer_active" },
	{ 0xd1ea1c88, "__kfifo_out" },
	{ 0x5fa07cc0, "hrtimer_start_range_ns" },
	{ 0xbd03ed67, "random_kmalloc_seed" },
	{ 0xc2fefbb5, "kmalloc_caches" },
	{ 0x38395bf3, "__kmalloc_cache_noprof" },
	{ 0xd272d446, "__rcu_read_lock" },
	{ 0x3b9eb4f2, "dev_get_by_name_rcu" },
	{ 0xd272d446, "__rcu_read_unlock" },
	{ 0xe74df256, "can_rx_register" },
	{ 0xab571ac8, "sock_create_kern" },
	{ 0xb89975c9, "kernel_bind" },
	{ 0x535f4f5f, "hrtimer_init" },
	{ 0xdf4bee3d, "alloc_workqueue" },
	{ 0x9e7ac839, "kernel_sendmsg" },
	{ 0x5a844b26, "__x86_indirect_thunk_rax" },
	{ 0x49fc4616, "hrtimer_forward" },
	{ 0x1b3db703, "param_ops_int" },
	{ 0x1b3db703, "param_ops_string" },
	{ 0xd272d446, "__fentry__" },
	{ 0x40a621c5, "scnprintf" },
	{ 0xe8213e80, "_printk" },
	{ 0xd272d446, "__x86_return_thunk" },
	{ 0x90a48d82, "__ubsan_handle_out_of_bounds" },
	{ 0xd272d446, "__stack_chk_fail" },
	{ 0x340bcc69, "init_net" },
	{ 0xda249131, "dev_get_by_index" },
	{ 0x70eca2ca, "module_layout" },
};

static const u32 ____version_ext_crcs[]
__used __section("__version_ext_crcs") = {
	0x01ca826f,
	0xe1e1f979,
	0x81a1a811,
	0x49733ad6,
	0x562e3aaa,
	0x36a36ab1,
	0xbeb1d261,
	0xbeb1d261,
	0xae1e4074,
	0xdc89bc96,
	0xcb8b6ec6,
	0x42244bac,
	0xd1ea1c88,
	0x5fa07cc0,
	0xbd03ed67,
	0xc2fefbb5,
	0x38395bf3,
	0xd272d446,
	0x3b9eb4f2,
	0xd272d446,
	0xe74df256,
	0xab571ac8,
	0xb89975c9,
	0x535f4f5f,
	0xdf4bee3d,
	0x9e7ac839,
	0x5a844b26,
	0x49fc4616,
	0x1b3db703,
	0x1b3db703,
	0xd272d446,
	0x40a621c5,
	0xe8213e80,
	0xd272d446,
	0x90a48d82,
	0xd272d446,
	0x340bcc69,
	0xda249131,
	0x70eca2ca,
};
static const char ____version_ext_names[]
__used __section("__version_ext_names") =
	"can_rx_unregister\0"
	"_raw_spin_lock_irqsave\0"
	"_raw_spin_unlock_irqrestore\0"
	"queue_work_on\0"
	"__kfifo_in\0"
	"hrtimer_cancel\0"
	"__flush_workqueue\0"
	"destroy_workqueue\0"
	"kernel_sock_shutdown\0"
	"sock_release\0"
	"kfree\0"
	"hrtimer_active\0"
	"__kfifo_out\0"
	"hrtimer_start_range_ns\0"
	"random_kmalloc_seed\0"
	"kmalloc_caches\0"
	"__kmalloc_cache_noprof\0"
	"__rcu_read_lock\0"
	"dev_get_by_name_rcu\0"
	"__rcu_read_unlock\0"
	"can_rx_register\0"
	"sock_create_kern\0"
	"kernel_bind\0"
	"hrtimer_init\0"
	"alloc_workqueue\0"
	"kernel_sendmsg\0"
	"__x86_indirect_thunk_rax\0"
	"hrtimer_forward\0"
	"param_ops_int\0"
	"param_ops_string\0"
	"__fentry__\0"
	"scnprintf\0"
	"_printk\0"
	"__x86_return_thunk\0"
	"__ubsan_handle_out_of_bounds\0"
	"__stack_chk_fail\0"
	"init_net\0"
	"dev_get_by_index\0"
	"module_layout\0"
;

MODULE_INFO(depends, "can");


MODULE_INFO(srcversion, "3AB18CE41A815D61CBC54B6");
