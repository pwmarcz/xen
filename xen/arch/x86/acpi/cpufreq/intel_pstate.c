#include <xen/kernel.h>
#include <xen/types.h>
#include <xen/init.h>
#include <xen/bitmap.h>
#include <xen/cpumask.h>
#include <xen/param.h>
#include <xen/timer.h>
#include <asm/msr.h>
#include <asm/msr-index.h>
#include <asm/processor.h>
#include <asm/div64.h>
#include <asm/cpufreq.h>
#include <acpi/cpufreq/cpufreq.h>

#define BYT_RATIOS       0x66a
#define BYT_VIDS         0x66b
#define BYT_TURBO_RATIOS 0x66c
#define BYT_TURBO_VIDS   0x66d

#define FRAC_BITS 8
#define int_tofp(X) ((uint64_t)(X) << FRAC_BITS)
#define fp_toint(X) ((X) >> FRAC_BITS)
#define clamp(val, lo, hi) min(max(val, lo), hi)
#define clamp_t(type, val, lo, hi) min_t(type, max_t(type, val, lo), hi)

static inline uint32_t mul_fp(uint32_t x, uint32_t y)
{
    return ((uint64_t)x * (uint64_t)y) >> FRAC_BITS;
}

static inline uint32_t div_fp(uint32_t x, uint32_t y)
{
    return div_s64((uint64_t)x << FRAC_BITS, y);
}

static inline uint32_t ceiling_fp(uint32_t x)
{
    uint32_t mask, ret;

    ret = fp_toint(x);
    mask = (1 << FRAC_BITS) - 1;
    if ( x & mask )
        ret += 1;
    return ret;
}

struct sample {
    uint32_t core_pct_busy;
    uint64_t aperf;
    uint64_t mperf;
    uint32_t freq;
    s_time_t time;
};

struct pstate_data {
    uint32_t    current_pstate;
    uint32_t    min_pstate;
    uint32_t    max_pstate;
    uint32_t    scaling;
    uint32_t    turbo_pstate;
};

struct vid_data {
    uint32_t min;
    uint32_t max;
    uint32_t turbo;
    uint32_t ratio;
};

struct _pid {
    uint32_t setpoint;
    uint32_t integral;
    uint32_t p_gain;
    uint32_t i_gain;
    uint32_t d_gain;
    uint32_t deadband;
    int32_t last_err;
};

struct cpudata {
    int cpu;

    struct timer timer;

    struct pstate_data pstate;
    struct vid_data vid;
    struct _pid pid;

    s_time_t last_sample_time;
    uint64_t prev_aperf;
    uint64_t prev_mperf;
    struct sample sample;
};

static struct cpudata **all_cpu_data;

struct pstate_adjust_policy {
    uint32_t sample_rate_ms;
    uint32_t deadband;
    uint32_t setpoint;
    uint32_t p_gain_pct;
    uint32_t d_gain_pct;
    uint32_t i_gain_pct;
};

struct pstate_funcs {
    uint32_t (*get_max)(void);
    uint32_t (*get_min)(void);
    uint32_t (*get_turbo)(void);
    uint32_t (*get_scaling)(void);
    void (*set)(struct perf_limits *, struct cpudata *, uint32_t pstate);
    void (*get_vid)(struct cpudata *);
};

struct cpu_defaults {
    struct pstate_adjust_policy pid_policy;
    struct pstate_funcs funcs;
};

static struct pstate_adjust_policy pid_params;
static struct pstate_funcs pstate_funcs;

static inline void pid_reset(struct _pid *pid, uint32_t setpoint,
                             uint32_t busy, uint32_t deadband,
                             uint32_t integral)
{
    pid->setpoint = setpoint;
    pid->deadband = deadband;
    pid->integral = int_tofp(integral);
    pid->last_err = int_tofp(setpoint) - int_tofp(busy);
}

static inline void pid_p_gain_set(struct _pid *pid, uint32_t percent)
{
    pid->p_gain = div_fp(int_tofp(percent), int_tofp(100));
}

static inline void pid_i_gain_set(struct _pid *pid, uint32_t percent)
{
    pid->i_gain = div_fp(int_tofp(percent), int_tofp(100));
}

static inline void pid_d_gain_set(struct _pid *pid, uint32_t percent)
{
    pid->d_gain = div_fp(int_tofp(percent), int_tofp(100));
}

static signed int pid_calc(struct _pid *pid, uint32_t busy)
{
    signed int result;
    int32_t pterm, dterm, fp_error;
    int32_t integral_limit;

    fp_error = int_tofp(pid->setpoint) - busy;

    if ( ABS(fp_error) <= int_tofp(pid->deadband) )
        return 0;

    pterm = mul_fp(pid->p_gain, fp_error);

    pid->integral += fp_error;

    /*
     * We limit the integral here so that it will never
     * get higher than 30.  This prevents it from becoming
     * too large an input over long periods of time and allows
     * it to get factored out sooner.
     * The value of 30 was chosen through experimentation.
     */
    integral_limit = int_tofp(30);
    if ( pid->integral > integral_limit )
        pid->integral = integral_limit;
    if ( pid->integral < -integral_limit )
        pid->integral = -integral_limit;

    dterm = mul_fp(pid->d_gain, fp_error - pid->last_err);
    pid->last_err = fp_error;

    result = pterm + mul_fp(pid->integral, pid->i_gain) + dterm;
    result = result + (1 << (FRAC_BITS-1));
    return (signed int)fp_toint(result);
}

static inline void intel_pstate_busy_pid_reset(struct cpudata *cpu)
{
    pid_p_gain_set(&cpu->pid, pid_params.p_gain_pct);
    pid_d_gain_set(&cpu->pid, pid_params.d_gain_pct);
    pid_i_gain_set(&cpu->pid, pid_params.i_gain_pct);

    pid_reset(&cpu->pid, pid_params.setpoint, 100, pid_params.deadband, 0);
}

static inline void intel_pstate_reset_all_pid(void)
{
    uint32_t cpu;

    for_each_online_cpu(cpu)
    {
        if ( all_cpu_data[cpu] )
            intel_pstate_busy_pid_reset(all_cpu_data[cpu]);
    }
}

static inline void update_turbo_state(struct cpufreq_policy *policy)
{
    uint64_t misc_en;
    struct cpudata *cpu;

    cpu = all_cpu_data[policy->cpu];
    rdmsrl(MSR_IA32_MISC_ENABLE, misc_en);
    policy->limits.turbo_disabled =
        (misc_en & MSR_IA32_MISC_ENABLE_TURBO_DISABLE ||
            cpu->pstate.max_pstate == cpu->pstate.turbo_pstate);
}

#define BYT_TURBO_CONTROL_BIT 32
#define BYT_MIN_PSTATE(val) (((value) >> 8) & 0x7f)
#define BYT_MAX_PSTATE(val) (((value) >> 16) & 0x7f)
#define BYT_TURBO_PSTATE(value) ((value) & 0x7f)
static uint32_t byt_get_min_pstate(void)
{
    uint64_t value;

    rdmsrl(BYT_RATIOS, value);
    return BYT_MIN_PSTATE(val);
}

static uint32_t byt_get_max_pstate(void)
{
    uint64_t value;

    rdmsrl(BYT_RATIOS, value);
    return BYT_MAX_PSTATE(val);
}

static uint32_t byt_get_turbo_pstate(void)
{
    uint64_t value;

    rdmsrl(BYT_TURBO_RATIOS, value);
    return BYT_TURBO_PSTATE(value);
}

static void byt_set_pstate(struct perf_limits *limits,
                struct cpudata *cpudata, uint32_t pstate)
{
    uint64_t val;
    uint32_t vid_fp;
    uint32_t vid;

    val = pstate << 8;
    if ( limits->no_turbo && !limits->turbo_disabled )
        val |= (uint64_t)1 << BYT_TURBO_CONTROL_BIT;

    vid_fp = cpudata->vid.min + mul_fp(
        int_tofp(pstate - cpudata->pstate.min_pstate),
        cpudata->vid.ratio);

    vid_fp = clamp(vid_fp, cpudata->vid.min, cpudata->vid.max);
    vid = ceiling_fp(vid_fp);

    if ( pstate > cpudata->pstate.max_pstate )
        vid = cpudata->vid.turbo;

    val |= vid;

    wrmsrl(MSR_IA32_PERF_CTL, val);
}

#define BYT_BCLK_FREQS 5
#define TO_FREQ_TABLE_IDX_MASK 0x7
static uint32_t byt_get_scaling(void)
{
    const uint32_t byt_freq_table[BYT_BCLK_FREQS] =
                   {833, 1000, 1333, 1167, 800};
    uint64_t value;
    int i;

    rdmsrl(MSR_FSB_FREQ, value);
    i = value & TO_FREQ_TABLE_IDX_MASK;

    BUG_ON(i > BYT_BCLK_FREQS);

    return byt_freq_table[i] * 100;
}

static void byt_get_vid(struct cpudata *cpudata)
{
    uint64_t value;

    rdmsrl(BYT_VIDS, value);
    cpudata->vid.min = int_tofp(BYT_MIN_PSTATE(val));
    cpudata->vid.max = int_tofp(BYT_MAX_PSTATE(val));
    cpudata->vid.ratio = div_fp(cpudata->vid.max -
                                cpudata->vid.min,
                                int_tofp(cpudata->pstate.max_pstate -
                                         cpudata->pstate.min_pstate));
    rdmsrl(BYT_TURBO_VIDS, value);
    cpudata->vid.turbo = BYT_TURBO_PSTATE(value);
}

#define SCALING_FACTOR 100000
#define CORE_TURBO_CONTROL_BIT 32
#define CORE_MIN_PSTATE(val) (((value) >> 40) & 0xff)
#define CORE_MAX_PSTATE(val) (((value) >> 8) & 0xff)
#define CORE_TURBO_PSTATE(value) ((value) & 0xff)
static uint32_t core_get_min_pstate(void)
{
    uint64_t value;

    rdmsrl(MSR_INTEL_PLATFORM_INFO, value);
    return CORE_MIN_PSTATE(val);
}

static uint32_t core_get_max_pstate(void)
{
    uint64_t value;

    rdmsrl(MSR_INTEL_PLATFORM_INFO, value);
    return CORE_MAX_PSTATE(val);
}

static uint32_t core_get_turbo_pstate(void)
{
    uint64_t value;
    uint32_t nont, ret;

    rdmsrl(MSR_NHM_TURBO_RATIO_LIMIT, value);
    nont = core_get_max_pstate();
    ret = CORE_TURBO_PSTATE(value);
    if ( ret <= nont )
        ret = nont;
    return ret;
}

static inline uint32_t core_get_scaling(void)
{
    return SCALING_FACTOR;
}

static void core_set_pstate(struct perf_limits *limits,
                            struct cpudata *cpudata, uint32_t pstate)
{
    uint64_t val;

    val = pstate << 8;
    if ( limits->no_turbo && !limits->turbo_disabled )
        val |= (uint64_t)1 << CORE_TURBO_CONTROL_BIT;

    wrmsrl(MSR_IA32_PERF_CTL, val);
}

static struct cpu_defaults core_params = {
    .pid_policy = {
        .sample_rate_ms = 10,
        .deadband = 0,
        .setpoint = 97,
        .p_gain_pct = 20,
        .d_gain_pct = 0,
        .i_gain_pct = 0,
    },
    .funcs = {
        .get_max = core_get_max_pstate,
        .get_min = core_get_min_pstate,
        .get_turbo = core_get_turbo_pstate,
        .get_scaling = core_get_scaling,
        .set = core_set_pstate,
    },
};

static struct cpu_defaults byt_params = {
    .pid_policy = {
        .sample_rate_ms = 10,
        .deadband = 0,
        .setpoint = 97,
        .p_gain_pct = 14,
        .d_gain_pct = 0,
        .i_gain_pct = 4,
    },
    .funcs = {
        .get_max = byt_get_max_pstate,
        .get_min = byt_get_min_pstate,
        .get_turbo = byt_get_turbo_pstate,
        .set = byt_set_pstate,
        .get_scaling = byt_get_scaling,
        .get_vid = byt_get_vid,
    },
};

static void intel_pstate_get_min_max(struct perf_limits *limits,
                                     struct cpudata *cpu, uint32_t *min,
                                     uint32_t *max)
{
    uint32_t max_perf = cpu->pstate.turbo_pstate;
    uint32_t max_perf_adj;
    uint32_t min_perf;

    if ( limits->no_turbo || limits->turbo_disabled )
        max_perf = cpu->pstate.max_pstate;

    /* performance can be limited by user through xenpm */
    max_perf_adj = fp_toint(mul_fp(int_tofp(max_perf), limits->max_perf));
    *max = clamp(max_perf_adj, cpu->pstate.min_pstate,
                 cpu->pstate.turbo_pstate);
    min_perf = fp_toint(mul_fp(int_tofp(max_perf), limits->min_perf));
    *min = clamp(min_perf, cpu->pstate.min_pstate, max_perf);
}

static void intel_pstate_set_pstate(struct cpufreq_policy *policy,
                                    struct cpudata *cpu, uint32_t pstate)
{
    uint32_t max_perf, min_perf;
    struct perf_limits *limits;

    limits = &policy->limits;

    update_turbo_state(policy);

    if ( limits->turbo_disabled )
        policy->turbo = CPUFREQ_TURBO_UNSUPPORTED;
    else if ( limits->no_turbo )
        policy->turbo = CPUFREQ_TURBO_DISABLED;
    else
        policy->turbo = CPUFREQ_TURBO_ENABLED;

    intel_pstate_get_min_max(limits, cpu, &min_perf, &max_perf);

    pstate = clamp(pstate, min_perf, max_perf);

    if ( pstate == cpu->pstate.current_pstate )
        return;

    cpu->pstate.current_pstate = pstate;
    policy->cur = pstate * SCALING_FACTOR;

    pstate_funcs.set(limits, cpu, pstate);
}

static void intel_pstate_get_cpu_pstates(struct cpudata *cpu)
{
    struct cpufreq_policy *policy = per_cpu(cpufreq_cpu_policy, cpu->cpu);

    cpu->pstate.min_pstate = pstate_funcs.get_min();
    cpu->pstate.max_pstate = pstate_funcs.get_max();
    cpu->pstate.turbo_pstate = pstate_funcs.get_turbo();
    cpu->pstate.scaling = pstate_funcs.get_scaling();

    if ( pstate_funcs.get_vid )
        pstate_funcs.get_vid(cpu);
    intel_pstate_set_pstate(policy, cpu, cpu->pstate.min_pstate);
}

static inline void intel_pstate_calc_busy(struct cpudata *cpu)
{
    struct sample *sample = &cpu->sample;
    uint64_t core_pct;

    core_pct = int_tofp(sample->aperf) * int_tofp(100);
    core_pct = div64_u64(core_pct, int_tofp(sample->mperf));

    sample->freq = fp_toint(mul_fp(int_tofp(cpu->pstate.max_pstate *
                                   cpu->pstate.scaling / 100), core_pct));

    sample->core_pct_busy = (int32_t)core_pct;
}

static inline void intel_pstate_sample(struct cpudata *cpu)
{
    uint64_t aperf, mperf;
    unsigned long flags;

    local_irq_save(flags);
    rdmsrl(MSR_IA32_APERF, aperf);
    rdmsrl(MSR_IA32_MPERF, mperf);
    local_irq_restore(flags);

    cpu->last_sample_time = cpu->sample.time;
    cpu->sample.time = get_s_time();
    cpu->sample.aperf = aperf;
    cpu->sample.mperf = mperf;
    cpu->sample.aperf -= cpu->prev_aperf;
    cpu->sample.mperf -= cpu->prev_mperf;

    intel_pstate_calc_busy(cpu);

    cpu->prev_aperf = aperf;
    cpu->prev_mperf = mperf;
}

static inline void intel_pstate_set_sample_time(struct cpudata *cpu)
{
    set_timer(&cpu->timer, NOW() + MILLISECS(pid_params.sample_rate_ms));
}

static inline int32_t intel_pstate_get_scaled_busy(struct cpudata *cpu)
{
    uint32_t core_busy, max_pstate, current_pstate, sample_ratio;
    uint32_t duration_us;
    uint32_t sample_time_us;

    /*
     * core_busy is the ratio of actual performance to max
     * max_pstate is the max non turbo pstate available
     * current_pstate was the pstate that was requested during
     * the last sample period.
     *
     * We normalize core_busy, which was our actual percent
     * performance to what we requested during the last sample
     * period. The result will be a percentage of busy at a
     * specified pstate.
     */
    core_busy = cpu->sample.core_pct_busy;
    max_pstate = int_tofp(cpu->pstate.max_pstate);
    current_pstate = int_tofp(cpu->pstate.current_pstate);
    core_busy = mul_fp(core_busy, div_fp(max_pstate, current_pstate));

    /*
     * Since we have a deferred timer, it will not fire unless
     * we are in C0.  So, determine if the actual elapsed time
     * is significantly greater (3x) than our sample interval. If it
     * is, then we were idle for a long enough period of time
     * to adjust our busyness.
     */
    sample_time_us = pid_params.sample_rate_ms  * 1000ULL;
    duration_us = (uint32_t)((s_time_t)(cpu->sample.time -
                              cpu->last_sample_time) / 1000);
    if ( duration_us > sample_time_us * 3 )
    {
        sample_ratio = div_fp(int_tofp(sample_time_us),
                              int_tofp(duration_us));
        core_busy = mul_fp(core_busy, sample_ratio);
    }

    return core_busy;
}

static inline void intel_pstate_adjust_busy_pstate(struct cpudata *cpu)
{
    int32_t busy_scaled;
    struct _pid *pid;
    signed int ctl;
    struct cpufreq_policy *policy = per_cpu(cpufreq_cpu_policy, cpu->cpu);

    pid = &cpu->pid;
    busy_scaled = intel_pstate_get_scaled_busy(cpu);

    ctl = pid_calc(pid, busy_scaled);

    /* Negative values of ctl increase the pstate and vice versa */
    intel_pstate_set_pstate(policy, cpu, cpu->pstate.current_pstate - ctl);
}

static void intel_pstate_timer_func(void *data)
{
    struct cpudata *cpu = (struct cpudata *) data;

    intel_pstate_sample(cpu);

    intel_pstate_adjust_busy_pstate(cpu);

    intel_pstate_set_sample_time(cpu);
}

#define ICPU(model, policy) \
    { X86_VENDOR_INTEL, 6, model, X86_FEATURE_APERFMPERF,\
            &policy##_params }

static struct x86_cpu_id intel_pstate_cpu_ids[] = {
    ICPU(0x2a, core),
    ICPU(0x2d, core),
    ICPU(0x37, byt),
    ICPU(0x3a, core),
    ICPU(0x3c, core),
    ICPU(0x3d, core),
    ICPU(0x3e, core),
    ICPU(0x3f, core),
    ICPU(0x45, core),
    ICPU(0x46, core),
    ICPU(0x47, core),
    ICPU(0x4c, byt),
    ICPU(0x4e, core),
    ICPU(0x4f, core),
    ICPU(0x56, core),
    ICPU(0x8e, core),
    {}
};

static int intel_pstate_init_cpu(unsigned int cpunum)
{
    struct cpudata *cpu;
    s_time_t expires;

    if ( !all_cpu_data[cpunum] )
        all_cpu_data[cpunum] = xzalloc(struct cpudata);
    if ( !all_cpu_data[cpunum] )
        return -ENOMEM;

    cpu = all_cpu_data[cpunum];

    cpu->cpu = cpunum;
    intel_pstate_get_cpu_pstates(cpu);

    init_timer(&cpu->timer, intel_pstate_timer_func, cpu, cpunum);
    expires = NOW() + MILLISECS(10);

    intel_pstate_busy_pid_reset(cpu);
    intel_pstate_sample(cpu);

    set_timer(&cpu->timer, expires);

    return 0;
}

static int intel_pstate_set_policy(struct cpufreq_policy *policy)
{
    struct perf_limits *limits = &policy->limits;
    uint32_t cur_gov = policy->internal_gov->cur_gov;

    if ( !policy->cpuinfo.max_freq )
        return -ENODEV;

    switch ( cur_gov )
    {
    case INTERNAL_GOV_PERFORMANCE:
        limits->no_turbo = 0;
        limits->max_perf_pct = 100;
        limits->max_perf = int_tofp(1);
        limits->min_perf_pct = 100;
        limits->min_perf = int_tofp(1);
        break;
    case INTERNAL_GOV_POWERSAVE:
        limits->min_perf = div_fp(int_tofp(limits->min_policy_pct),
                                  int_tofp(100));
        limits->max_perf = limits->min_perf;
        limits->min_perf_pct = limits->min_policy_pct;
        limits->max_perf_pct = limits->min_perf_pct;
        break;
    case INTERNAL_GOV_USERSPACE:
        limits->max_perf = div_fp(int_tofp(limits->max_perf_pct),
                                  int_tofp(100));
        limits->min_perf = limits->max_perf;
        limits->min_perf_pct = limits->max_perf_pct;
        break;
    case INTERNAL_GOV_ONDEMAND:
    default:
        limits->min_perf = div_fp(int_tofp(limits->min_perf_pct),
                                  int_tofp(100));
        limits->max_perf = div_fp(int_tofp(limits->max_perf_pct),
                                  int_tofp(100));
        cur_gov = INTERNAL_GOV_ONDEMAND;
        break;
    }

    return 0;
}

static int intel_pstate_verify_policy(struct cpufreq_policy *policy)
{
    uint32_t cur_gov = policy->internal_gov->cur_gov;

    cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
                                 policy->cpuinfo.max_freq);

    switch( cur_gov )
    {
    case INTERNAL_GOV_PERFORMANCE:
    case INTERNAL_GOV_POWERSAVE:
    case INTERNAL_GOV_USERSPACE:
    case INTERNAL_GOV_ONDEMAND:
        return 0;
    default:
        return -EINVAL;
    }
}

static void intel_pstate_internal_gov_release(struct internal_governor *gov)
{
    xfree(gov->avail_gov);
    xfree(gov);
}

static int intel_pstate_cpu_exit(struct cpufreq_policy *policy)
{
    int cpu_num = policy->cpu;
    struct cpudata *cpu = all_cpu_data[cpu_num];

    kill_timer(&all_cpu_data[cpu_num]->timer);

    intel_pstate_set_pstate(policy, cpu, cpu->pstate.min_pstate);

    intel_pstate_internal_gov_release(policy->internal_gov);

    return 0;
}

static int intel_pstate_turbo_update(int cpuid, struct cpufreq_policy *policy)
{
    struct cpudata *cpu = all_cpu_data[policy->cpu];
    struct perf_limits *limits = &policy->limits;

    update_turbo_state(policy);
    if ( limits->turbo_disabled )
    {
        printk("Turbo disabled by BIOS or not supported on CPU\n");
        return -EINVAL;
    }
    limits->no_turbo = policy->turbo == CPUFREQ_TURBO_ENABLED ? 0 : 1;

    if ( limits->no_turbo )
        policy->cpuinfo.max_freq = cpu->pstate.max_pstate *
                                   cpu->pstate.scaling;
    else
        policy->cpuinfo.max_freq = cpu->pstate.turbo_pstate *
                                   cpu->pstate.scaling;

    policy->max = clamp(policy->max, policy->cpuinfo.min_freq,
                        policy->cpuinfo.max_freq);

    return 0;
}

static uint32_t get_turbo_pct(struct cpudata *cpu)
{
    uint32_t total, no_turbo, turbo_pct;
    uint32_t turbo_fp;

    total = cpu->pstate.turbo_pstate - cpu->pstate.min_pstate + 1;
    no_turbo = cpu->pstate.max_pstate - cpu->pstate.min_pstate + 1;
    turbo_fp = div_fp(int_tofp(no_turbo), int_tofp(total));
    turbo_pct = 100 - fp_toint(mul_fp(turbo_fp, int_tofp(100)));
    return turbo_pct;
}

#define INTEL_PSTATE_GOV_NUM 4
static struct internal_governor* intel_pstate_internal_gov_init(void)
{
    unsigned int i = 0;
    struct internal_governor *gov;
    char *avail_gov;

    gov = xzalloc(struct internal_governor);
    if ( !gov )
        return NULL;
    avail_gov = xzalloc_array(char,
            INTEL_PSTATE_GOV_NUM * CPUFREQ_NAME_LEN);
    if ( !avail_gov )
        return NULL;

    gov->avail_gov = avail_gov;

    i += scnprintf(&avail_gov[0], CPUFREQ_NAME_LEN, "%s ", "performance");
    i += scnprintf(&avail_gov[i], CPUFREQ_NAME_LEN, "%s ", "powersave");
    i += scnprintf(&avail_gov[i], CPUFREQ_NAME_LEN, "%s ", "userspace");
    i += scnprintf(&avail_gov[i], CPUFREQ_NAME_LEN, "%s ", "ondemand");
    avail_gov[i-1] = '\0';
    gov->gov_num = INTEL_PSTATE_GOV_NUM;
    gov->cur_gov = INTERNAL_GOV_ONDEMAND;
    return gov;
}

static int intel_pstate_cpu_setup(struct cpufreq_policy *policy)
{
    struct cpudata *cpu;
    struct perf_limits *limits = &policy->limits;
    int rc;

    rc = intel_pstate_init_cpu(policy->cpu);
    if ( rc )
        return rc;

    policy->internal_gov = intel_pstate_internal_gov_init();
    if ( !policy->internal_gov )
        return -ENOMEM;

    cpu = all_cpu_data[policy->cpu];
    policy->min = cpu->pstate.min_pstate * cpu->pstate.scaling;
    policy->max = cpu->pstate.turbo_pstate * cpu->pstate.scaling;

    /* cpuinfo and default policy values */
    policy->cpuinfo.min_freq = cpu->pstate.min_pstate *
                               cpu->pstate.scaling;
    policy->cpuinfo.max_freq = cpu->pstate.turbo_pstate *
                               cpu->pstate.scaling;
    policy->cpuinfo.transition_latency = CPUFREQ_ETERNAL;
    cpumask_set_cpu(policy->cpu, policy->cpus);

    limits->no_turbo = 0;
    limits->turbo_disabled = 0;
    limits->turbo_pct = get_turbo_pct(cpu);
    limits->min_policy_pct = (policy->min * 100) /
                             policy->cpuinfo.max_freq;
    limits->min_policy_pct = clamp_t(uint32_t,
                                     limits->min_policy_pct, 0, 100);
    limits->max_policy_pct = (policy->max * 100) /
                             policy->cpuinfo.max_freq;
    limits->max_policy_pct = clamp_t(uint32_t,
                                     limits->max_policy_pct, 0, 100);
    limits->max_perf_pct   = limits->max_policy_pct;
    limits->min_perf_pct   = limits->min_policy_pct;

    return 0;
}

static struct cpufreq_driver intel_pstate_driver = {
    .verify       = intel_pstate_verify_policy,
    .setpolicy    = intel_pstate_set_policy,
    .init         = intel_pstate_cpu_setup,
    .exit         = intel_pstate_cpu_exit,
    .update       = intel_pstate_turbo_update,
    .name         = "intel_pstate",
};

static int intel_pstate_msrs_not_valid(void)
{
    if ( !pstate_funcs.get_max() ||
         !pstate_funcs.get_min() ||
         !pstate_funcs.get_turbo() )
        return -ENODEV;

    return 0;
}

static void __init copy_pid_params(struct pstate_adjust_policy *policy)
{
    pid_params.sample_rate_ms = policy->sample_rate_ms;
    pid_params.p_gain_pct = policy->p_gain_pct;
    pid_params.i_gain_pct = policy->i_gain_pct;
    pid_params.d_gain_pct = policy->d_gain_pct;
    pid_params.deadband = policy->deadband;
    pid_params.setpoint = policy->setpoint;
}

static void __init copy_cpu_funcs(struct pstate_funcs *funcs)
{
    pstate_funcs.get_max   = funcs->get_max;
    pstate_funcs.get_min   = funcs->get_min;
    pstate_funcs.get_turbo = funcs->get_turbo;
    pstate_funcs.get_scaling = funcs->get_scaling;
    pstate_funcs.set       = funcs->set;
    pstate_funcs.get_vid   = funcs->get_vid;
}

int __init intel_pstate_init(void)
{
    int cpu, rc = 0;
    const struct x86_cpu_id *id;
    struct cpu_defaults *cpu_info;
    static bool_t load;
    boolean_param("intel_pstate", load);

    if ( !load )
        return -ENODEV;

    id = x86_match_cpu(intel_pstate_cpu_ids);
    if ( !id )
        return -ENODEV;

    cpu_info = (struct cpu_defaults *)id->driver_data;

    copy_pid_params(&cpu_info->pid_policy);
    copy_cpu_funcs(&cpu_info->funcs);

    if ( intel_pstate_msrs_not_valid() )
        return -ENODEV;

    all_cpu_data = xzalloc_array(struct cpudata *, NR_CPUS);
    if ( !all_cpu_data )
        return -ENOMEM;

    rc = cpufreq_register_driver(&intel_pstate_driver);
    if ( rc )
        goto out;

    return rc;
out:
    for_each_online_cpu(cpu)
    {
        if ( all_cpu_data[cpu] )
        {
            kill_timer(&all_cpu_data[cpu]->timer);
            xfree(all_cpu_data[cpu]);
        }
    }
    xfree(all_cpu_data);
    return -ENODEV;
}
