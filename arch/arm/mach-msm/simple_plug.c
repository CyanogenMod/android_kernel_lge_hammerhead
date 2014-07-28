/*
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Author: Christopher R. Palmer <crpalmer@gmail.com>
 */

#define PR_NAME "simple_plug: "

#include <linux/workqueue.h>
#include <linux/cpu.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/cpufreq.h>

#define DEF_SAMPLING_MS			10
#define DEF_VERIFY_MS			(120*1000)
#define HISTORY_SIZE			20

static struct {
	bool	verify_cores_needed;
	bool	up_down_cores_needed;
	unsigned int desired_n_online;
} work_data;

static struct work_struct work;
static struct timer_list update_timer;

/* Zero disables the hotplug driver and 1 enables it.  Normally it is enabled
 * once the boot has completed.
 */
static unsigned int active;

/* Min/max number of cores that will be online.  If an invalid values is
 * specified the driver will silently sanitize them to values in the range
 * 1 .. num_possible_cpus().
 */
static unsigned int min_cores = 1;
static unsigned int max_cores;

/* Ms period to schedule the timer to collect samples of system load.  This is
 * not a strict time period and merely represents the rough desired sample rate.
 */
static unsigned int sampling_ms = DEF_SAMPLING_MS;

/* Approximate ms time to wait after discovering that our cores have been
 * plugged or unplugged by some other process until we take corrective action.
 *
 * It is unfortunately true that kernel tuning apps often start / stop cores
 * and we need to make sure we give it some time to finish whatever it is
 * doing before we take control over again.
 */
static unsigned int verify_ms = DEF_VERIFY_MS;

#ifdef CONFIG_SIMPLE_PLUG_STATS

#define MAX_CORES 16
static u64 last_stat_us;

/* Non-zero will cause the stats to be reset on the next plug/unplug operation.
 */
static unsigned int reset_stats;

/* Records the us (displayed as sec.ms) that we consider each core to have
 * been plugged.  This does not take into account any external actions and
 * includes any time the core is actually in deep sleep and not really
 * running.
 */
static u64 time_cores_running[MAX_CORES];

/* Records the number of times each core brought up/down. */
static unsigned int times_core_up[MAX_CORES];
static unsigned int times_core_down[MAX_CORES];

#define STATS(expr) (expr)

#else

#define STATS(expr) /* nop */

#endif

#define DUMP_HISTORY	0
#define HISTORY_MULT	100
#define HISTORY_DECIMAL_DIGITS	2

static unsigned int nr_avg;
static unsigned int nr_run_history[HISTORY_SIZE];
static unsigned int nr_last_i;

static unsigned int n_until_verify = HISTORY_SIZE;
static bool verify_cores_needed = 1;
static unsigned int n_online;

static void dump_history(unsigned int desired_n_online)
{
#if DUMP_HISTORY
	int i;
	char buf[HISTORY_SIZE*10+1];
	size_t n_buf = 0;

	for (i = 0; i < HISTORY_SIZE; i++) {
		unsigned int nr = nr_run_history[(nr_last_i + i) % HISTORY_SIZE];
		n_buf += sprintf(&buf[n_buf], " %u.%0*u", nr / HISTORY_MULT,
				HISTORY_DECIMAL_DIGITS, nr % HISTORY_MULT);
	}

	pr_info(PR_NAME ": desired=%u online=%u avg=%u history%s", desired_n_online, n_online, nr_avg / HISTORY_SIZE, buf);
#endif
}

static unsigned int get_scaled_avg_nr_running(void)
{
	unsigned int nr = avg_nr_running();

	return (nr * HISTORY_MULT) >> FSHIFT;
}

static void update_history(void)
{
	unsigned int value = get_scaled_avg_nr_running();

	nr_avg -= nr_run_history[nr_last_i];
	nr_avg += nr_run_history[nr_last_i] = value;
	nr_last_i = (nr_last_i + 1) % HISTORY_SIZE;
	if (n_until_verify)
		n_until_verify--;
}

static unsigned desired_number_of_cores(void)
{
	unsigned int up_cores, down_cores;
	int avg;

	/* Compute number of cores of average active work (runnable tasks).
	 * To add core N we must have a load of at least N+0.5 runnables.
	 * To remove core N we must have load of N-0.5 or fewer runnables.
	 */

	avg = nr_avg / HISTORY_SIZE;
	up_cores = (avg - HISTORY_MULT/2) / HISTORY_MULT;
	down_cores = (avg + HISTORY_MULT/2) / HISTORY_MULT;

	up_cores = max(min(up_cores, max_cores), min_cores);
	down_cores = max(min(down_cores, max_cores), min_cores);

	if (up_cores > n_online)
		return up_cores;
	else if (down_cores < n_online)
		return down_cores;
	else
		return n_online;
}

static bool __cpuinit cpu_state_is_not_valid(int cpu)
{
	return (cpu < n_online && ! cpu_online(cpu)) ||
	       (cpu >= n_online && cpu_online(cpu));
}

static void simple_plug_stats_update_running(void)
{
#ifdef CONFIG_SIMPLE_PLUG_STATS
	int cpu;
	u64 now = ktime_to_us(ktime_get());
	u64 delta = now - last_stat_us;

	for (cpu = 0; cpu < n_online; cpu++)
		time_cores_running[cpu] += delta;
	last_stat_us = now;
#endif
}

static void __cpuinit verify_cores(unsigned int desired_n_online)
{
	int cpu;

	for (cpu = 0; cpu < max_cores; cpu++) {
		if (cpu < desired_n_online && ! cpu_online(cpu)) {
			pr_info(PR_NAME "start cpu%d that someone took offline.\n", cpu);
			cpu_up(cpu);
		} else if (cpu >= desired_n_online && cpu_online(cpu)) {
			pr_info(PR_NAME "stop cpu%d that we want offline\n", cpu);
			cpu_down(cpu);
		}
	}

	verify_cores_needed = false;
}

static void __cpuinit up_down_cores(unsigned int desired_n_online)
{
	int cpu;

	BUG_ON(desired_n_online < 1 || desired_n_online > max_cores);

	for (cpu = max_cores-1; cpu >= 0; cpu--) {
		if (cpu_state_is_not_valid(cpu)) {
			pr_info(PR_NAME "cpu%d was externall modified, scheduling verify", cpu);
			verify_cores_needed = true;
			n_until_verify = (verify_ms+sampling_ms-1)/sampling_ms;
			BUG_ON(n_until_verify == 0);
			return;
		}

		if (cpu >= n_online && cpu < desired_n_online) {
			pr_debug(PR_NAME "start cpu%d, want %d online\n", cpu, desired_n_online);
			STATS(times_core_up[cpu]++);
			cpu_up(cpu);
		} else if (cpu >= desired_n_online && cpu < n_online) {
			pr_debug(PR_NAME "stop  cpu%d, want %d online\n", cpu, desired_n_online);
			STATS(times_core_down[cpu]++);
			cpu_down(cpu);
		}
	}

	n_online = desired_n_online;
}

static void __cpuinit work_fn(struct work_struct *work)
{
#ifdef CONFIG_SIMPLE_PLUG_STATS
	if (reset_stats) {
		reset_stats = 0;
		memset(time_cores_running, 0, sizeof(time_cores_running));
		memset(times_core_up, 0, sizeof(times_core_up));
		memset(times_core_down, 0, sizeof(times_core_down));
	}
#endif

	simple_plug_stats_update_running();

	if (work_data.verify_cores_needed)
		verify_cores(work_data.desired_n_online);

	if (work_data.up_down_cores_needed)
		up_down_cores(work_data.desired_n_online);

	n_online = work_data.desired_n_online;

	work_data.verify_cores_needed = false;
	work_data.up_down_cores_needed = false;
	work_data.desired_n_online = 0;
}

static unsigned long expires(unsigned int ms)
{
	return jiffies + msecs_to_jiffies(ms);
}

static void start_timer(unsigned int ms)
{
	pr_info(PR_NAME "starting update_timer");
#ifdef CONFIG_SIMPLE_PLUG_STATS
	last_stat_us = ktime_to_us(ktime_get());
#endif
	update_timer.expires = expires(ms);
	add_timer_on(&update_timer, 0);
}

static void schedule_next_timer(void)
{
	mod_timer_pinned(&update_timer, expires(sampling_ms));
}

static bool consider_scheduling_verify_cores(void)
{
	if (! verify_cores_needed || n_until_verify)
		return false;

	work_data.verify_cores_needed = true;
	return true;
}

static bool consider_scheduling_up_down_cores(unsigned int desired_n_online)
{
	if (verify_cores_needed)
		return false;

	if (desired_n_online == n_online)
		return false;

	work_data.up_down_cores_needed = true;
	return true;
}

static void simple_plug_timer_work(void)
{
	bool launch_work = false;
	unsigned int desired_n_online;

	update_history();

	if (work_data.desired_n_online)
		return;

	desired_n_online = desired_number_of_cores();

#ifdef CONFIG_SIMPLE_PLUG_STATS
	launch_work |= reset_stats;
#endif

	launch_work |= consider_scheduling_verify_cores();
	launch_work |= consider_scheduling_up_down_cores(desired_n_online);

	if (launch_work) {
		dump_history(desired_n_online);
		work_data.desired_n_online = desired_n_online;
		schedule_work(&work);
	}
}

static void simple_plug_update_fn(unsigned long data_unused)
{
	if (active == 1)
		simple_plug_timer_work();

	schedule_next_timer();
}

/* ---------------------------------------------------------------------- */

static void noop_hook(void)
{
}

#define PARAM_UINT_HOOK(name, mode, on_set_hook) \
\
static ssize_t show_##name(struct kobject *kobj, struct attribute *attr, char *buf) \
{ \
	return snprintf(buf, PAGE_SIZE, "%u\n", name); \
} \
\
static ssize_t store_##name(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count) \
{ \
	unsigned long val; \
	int ret = kstrtoul(buf, 0, &val); \
	if (ret < 0) \
		return ret; \
	name = val; \
	on_set_hook(); \
	return count; \
} \
\
static struct global_attr global_attr_##name = __ATTR(name, mode, show_##name, store_##name);

#define PARAM_UINT(name, mode) \
	PARAM_UINT_HOOK(name, mode, noop_hook)

/* ---------------------------------------------------------------------- */

#define PARAM_UINT_ARRAY_RO(name) \
\
static ssize_t show_##name(struct kobject *kobj, struct attribute *attr, char *buf) \
{ \
	size_t i; \
	ssize_t ret = 0; \
	\
	for (i = 0; i < max_cores && ret < PAGE_SIZE; i++) \
		ret += snprintf(&buf[ret], PAGE_SIZE - ret, "%u%c", name[i], i<max_cores-1 ? ' ' : '\n'); \
	return ret; \
} \
\
static ssize_t store_##name(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count) \
{ \
	return -1; \
} \
\
static struct global_attr global_attr_##name = __ATTR(name, 0444, show_##name, store_##name);

/* ---------------------------------------------------------------------- */

static void active_set_hook(void)
{
	del_timer_sync(&update_timer);
	if (active) {
		pr_info(PR_NAME "scheduling initial verify in %d samples\n", n_until_verify);
		verify_cores_needed = true;
		n_until_verify = HISTORY_SIZE;
		start_timer(sampling_ms);
	} else
		pr_info(PR_NAME "stopping update_timer\n");
}

static void min_max_cores_set_hook(void)
{
	min_cores = max(1u, min_cores);
	max_cores = max(1u, max_cores);
	min_cores = min(min_cores, num_possible_cpus());
	max_cores = min(max_cores, num_possible_cpus());
	min_cores = min(min_cores, max_cores);
	max_cores = max(min_cores, max_cores);
}

PARAM_UINT_HOOK(active, 0644, active_set_hook);
PARAM_UINT_HOOK(min_cores, 0644, min_max_cores_set_hook);
PARAM_UINT_HOOK(max_cores, 0644, min_max_cores_set_hook);
PARAM_UINT(sampling_ms, 0644);
PARAM_UINT(verify_ms, 0644);

PARAM_UINT(nr_avg, 0444);
PARAM_UINT(n_online, 0444);
PARAM_UINT_ARRAY_RO(nr_run_history);

#ifdef CONFIG_SIMPLE_PLUG_STATS

static ssize_t show_time_cores_running(struct kobject *kobj, struct attribute *attr, char *buf)
{
	size_t i;
	ssize_t ret = 0;

	for (i = 0; i < max_cores && ret < PAGE_SIZE; i++) {
		unsigned int usec;
		unsigned int sec = div_u64_rem(time_cores_running[i], 1000000, &usec);

		ret += snprintf(&buf[ret], PAGE_SIZE - ret, "%u.%03u%c",
			sec, usec / 1000, i < max_cores-1 ? ' ' : '\n');
	}
	return ret;
}

static ssize_t store_time_cores_running(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	return -1;
}

static struct global_attr global_attr_time_cores_running = __ATTR(time_cores_running, 0444, show_time_cores_running, store_time_cores_running);

PARAM_UINT(reset_stats, 0644);
PARAM_UINT_ARRAY_RO(times_core_up);
PARAM_UINT_ARRAY_RO(times_core_down);
#endif

static struct attribute *simple_plug_attributes[] = {
	&global_attr_active.attr,
	&global_attr_min_cores.attr,
	&global_attr_max_cores.attr,
	&global_attr_sampling_ms.attr,
	&global_attr_verify_ms.attr,
	&global_attr_nr_avg.attr,
	&global_attr_n_online.attr,
	&global_attr_nr_run_history.attr,
#ifdef CONFIG_SIMPLE_PLUG_STATS
	&global_attr_reset_stats.attr,
	&global_attr_time_cores_running.attr,
	&global_attr_times_core_up.attr,
	&global_attr_times_core_down.attr,
#endif
	NULL,
};

static struct attribute_group simple_plug_attr_group = {
	.attrs = simple_plug_attributes,
	.name = "simple_plug",
};

/* ---------------------------------------------------------------------- */

static int __init simple_plug_init(void)
{
	int rc;

	max_cores = num_possible_cpus();

	pr_info(PR_NAME "max_cores=%d\n", max_cores);

	INIT_WORK(&work, work_fn);
	init_timer_deferrable(&update_timer);
	update_timer.function = simple_plug_update_fn;

	rc = sysfs_create_group(&cpu_subsys.dev_root->kobj, &simple_plug_attr_group);
	BUG_ON(rc);

	return 0;
}

MODULE_AUTHOR("Christopher R. Palmer <crpalmer@gmail.com>");
MODULE_DESCRIPTION("'simple_plug' - An simple cpu hotplug driver for "
	"Low Latency Frequency Transition capable processors");
MODULE_LICENSE("GPL");

fs_initcall(simple_plug_init);
