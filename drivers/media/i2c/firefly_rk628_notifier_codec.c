/*
* Because different Firefly boards use different audio codecs, the characteristics of these codecs are also different. 
* In order to facilitate RK628D to adapt these audio codecs, we provide a notification chain to notify codecs. 
* When using RK628D module, different audio codecs are adjusted according to their own characteristics under this mechanism.  
*/

#include <asm/uaccess.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/notifier.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/module.h>

static RAW_NOTIFIER_HEAD(firefly_rk628_chain);

int register_firefly_rk628_notifier(struct notifier_block *nb)
{
    if(nb == NULL)
        return -EINVAL;

    return raw_notifier_chain_register(&firefly_rk628_chain, nb);
}
EXPORT_SYMBOL(register_firefly_rk628_notifier);

int unregister_firefly_rk628_notifier(struct notifier_block *nb)
{
    if(nb == NULL)
        return -EINVAL;

    return raw_notifier_chain_unregister(&firefly_rk628_chain, nb);
}
EXPORT_SYMBOL(unregister_firefly_rk628_notifier);

int firefly_rk628_notifier_call_chain(unsigned long val, void *ptr)
{
    return raw_notifier_call_chain(&firefly_rk628_chain, val ,ptr);
}
EXPORT_SYMBOL(firefly_rk628_notifier_call_chain);

// static int __init firefly_rk628_notifier_init(void)
// {
//     printk("Firefly RK628D notifier init\n");
//     return 0;
// } 

// static void __exit firefly_rk628_notifier_exit(void)
// {
//     printk("Firefly RK628D notifier exit\n");
// } 

// module_init(firefly_rk628_notifier_init);
// module_exit(firefly_rk628_notifier_exit);

// MODULE_DESCRIPTION("Firefly RK628D for audio codec");
// MODULE_AUTHOR("zyk@t-chip.com.cn");
// MODULE_LICENSE("GPL v2");