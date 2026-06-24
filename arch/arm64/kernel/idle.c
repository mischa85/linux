// SPDX-License-Identifier: GPL-2.0-only
/*
 * Low-level idle sequences
 */

#include <linux/cpu.h>
#include <linux/irqflags.h>

#include <asm/barrier.h>
#include <asm/cpuidle.h>
#include <asm/cpufeature.h>
#include <asm/sysreg.h>

enum { WFI, NOP } idle = WFI;

/* User can over-ride above with "idle=<wfi|nop>" in cmdline */
static int __init setup_idle(char *s)
{
	if (!s)
		return -1;
        else if (!strcmp(s, "wfi"))
                idle = WFI;
        else if (!strcmp(s, "nop"))
                idle = NOP;
	else
		return -1;

	return 0;
}
early_param("idle", setup_idle);

/*
 *	cpu_do_idle()
 *
 *	Idle the processor (wait for interrupt).
 *
 *	If the CPU supports priority masking we must do additional work to
 *	ensure that interrupts are not masked at the PMR (because the core will
 *	not wake up if we block the wake up signal in the interrupt controller).
 */
void __cpuidle cpu_do_idle(void)
{
	struct arm_cpuidle_irq_context context;

	arm_cpuidle_save_irq_context(&context);

	if (likely(idle == WFI)) {
		dsb(sy);
		wfi();
	}

	arm_cpuidle_restore_irq_context(&context);
}

/*
 * This is our default idle handler.
 */
void __cpuidle arch_cpu_idle(void)
{
	/*
	 * This should do all the clock switching and wait for interrupt
	 * tricks
	 */
	cpu_do_idle();
}
