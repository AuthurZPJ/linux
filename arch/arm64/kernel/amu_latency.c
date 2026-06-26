// SPDX-License-Identifier: GPL-2.0-only
/*
 * AMU counter read / this_cpu_has_cap() latency benchmark module.
 *
 * Measures the per-operation cost of:
 *   - read_sysreg_s(SYS_AMEVCNTR0_CORE_EL0)   (read_corecnt)
 *   - read_sysreg_s(SYS_AMEVCNTR0_CONST_EL0)  (read_constcnt)
 *   - this_cpu_has_cap(ARM64_WORKAROUND_2457168)
 *
 * The hot paths in topology.c (sched tick counter refresh, FFH reads) are not
 * touched; this module only calls the same primitives, gated behind
 * CONFIG_ARM64_AMU_LATENCY (default n).
 *
 * Method: each op is inlined in a tight loop bracketed by cntvct_el0 reads,
 * paired with an empty baseline loop in the same run to cancel loop overhead.
 * The minimum delta over RUNS runs is taken to reject interrupt jitter.
 * Ticks are converted to nanoseconds via cntfrq_el0.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/irqflags.h>
#include <linux/compiler.h>
#include <linux/smp.h>
#include <linux/cpumask.h>
#include <linux/printk.h>
#include <asm/sysreg.h>
#include <asm/cpufeature.h>

#define RUNS	64
#define ITERS	20000

static u64 cntfrq;

static inline u64 cntvct(void)
{
	isb();
	return read_sysreg_s(SYS_CNTVCT_EL0);
}

/*
 * Loop body shared by all measurements. op_expr is evaluated ITERS times,
 * with its result sinked to the compiler via an empty asm so the read cannot
 * be hoisted out of the loop. An identical empty loop is timed right after to
 * subtract loop-control overhead from the measurement.
 */
#define MEASURE_LOOP(op_expr)						\
	do {								\
		t0 = cntvct();						\
		for (i = 0; i < ITERS; i++) {				\
			u64 v = (op_expr);				\
			asm volatile("" :: "r" (v));			\
		}							\
		t1 = cntvct();						\
		for (i = 0; i < ITERS; i++)				\
			barrier();					\
		t2 = cntvct();						\
		delta = (t1 - t0) - (t2 - t1);				\
		if (delta < best)					\
			best = delta;					\
	} while (0)

static noinline u64 measure_corecnt(void)
{
	u64 best = ~0ULL, t0, t1, t2, delta;
	unsigned int i, run;

	for (run = 0; run < RUNS; run++)
		MEASURE_LOOP(read_sysreg_s(SYS_AMEVCNTR0_CORE_EL0));
	return best;
}

static noinline u64 measure_constcnt(void)
{
	u64 best = ~0ULL, t0, t1, t2, delta;
	unsigned int i, run;

	for (run = 0; run < RUNS; run++)
		MEASURE_LOOP(read_sysreg_s(SYS_AMEVCNTR0_CONST_EL0));
	return best;
}

static noinline u64 measure_has_cap(void)
{
	u64 best = ~0ULL, t0, t1, t2, delta;
	unsigned int i, run;

	for (run = 0; run < RUNS; run++)
		MEASURE_LOOP((u64)this_cpu_has_cap(ARM64_WORKAROUND_2457168));
	return best;
}

static void report(const char *name, u64 best_ticks)
{
	u64 per_iter_ticks, ns, rem;

	if (best_ticks == ~0ULL) {
		pr_info("amu_latency: %-16s (not measured)\n", name);
		return;
	}

	/* per-op ns = best_ticks / ITERS * 1e9 / cntfrq, with sub-ns remainder */
	per_iter_ticks = best_ticks;
	ns = per_iter_ticks * 1000000000ULL / ITERS / cntfrq;
	rem = (per_iter_ticks * 1000000000ULL / ITERS) % cntfrq;
	rem = rem * 1000ULL / cntfrq;
	pr_info("amu_latency: %-16s best=%6llu ticks / %u iters  =>  %4llu.%03llu ns/op\n",
		name, best_ticks, ITERS, ns, rem);
}

static int __init amu_latency_init(void)
{
	unsigned long flags;
	int cpu;
	bool amu;

	cntfrq = read_sysreg_s(SYS_CNTFRQ_EL0);
	if (!cntfrq) {
		pr_err("amu_latency: cntfrq_el0 == 0, aborting\n");
		return -ENODEV;
	}

	/*
	 * Pin to one CPU and disable preemption + IRQs for a stable
	 * measurement. this_cpu_has_cap() has WARN_ON(preemptible()), so
	 * preemption must be off when it is invoked.
	 */
	cpu = get_cpu();
	amu = cpu_has_amu_feat(cpu);
	pr_info("amu_latency: cpu=%d cntfrq=%llu Hz AMU=%d RUNS=%d ITERS=%d\n",
		cpu, cntfrq, amu, RUNS, ITERS);

	local_irq_save(flags);

	if (amu) {
		report("read_corecnt", measure_corecnt());
		report("read_constcnt", measure_constcnt());
	} else {
		/* Reading AMEVCNTR0_* on AMU-less silicon may UNDEF. */
		report("read_corecnt", ~0ULL);
		report("read_constcnt", ~0ULL);
	}
	/* this_cpu_has_cap() is independent of AMU support; always measured. */
	report("this_cpu_has_cap", measure_has_cap());

	local_irq_restore(flags);
	put_cpu();

	pr_info("amu_latency: done (per-op cost, baseline-subtracted)\n");
	return 0;
}

module_init(amu_latency_init);
MODULE_DESCRIPTION("AMU counter read / this_cpu_has_cap latency benchmark");
MODULE_LICENSE("GPL");
