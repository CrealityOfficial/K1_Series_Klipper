// Load Cell based end stops.
//
// Copyright (C) 2023  Gareth Farrington <gareth@waves.ky>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "basecmd.h" // oid_alloc
#include "board/irq.h" // irq_disable
#include "command.h" // DECL_COMMAND
#include "sched.h" // shutdown
#include "trsync.h" // trsync_do_trigger
#include "board/misc.h" // timer_read_time
#include "load_cell_endstop.h" //load_cell_endstop_report_sample
#include <stdint.h>

/*
 * Fix Point Support
 */
// https://en.wikipedia.org/wiki/Q_(number_format)
// Q12.19 / ARM Q13.19
// 12 because: 2^11 = 2048 >= 2Kg in grams
// +1 additional bit to allow 2 such numbers to be added without overflow
typedef int32_t fixedQ12_t;
#define FIXEDQ12 12
#define FIXEDQ12_FRAC_BITS (31 - FIXEDQ12)
#define FIXEDQ12_ROUNDING (1 << (FIXEDQ12_FRAC_BITS - 1))

// filter strucutre sizes
#define MAX_SECTIONS 4
#define SECTION_WIDTH 5
#define STATE_WIDTH 2

const uint8_t DEFAULT_SAMPLE_COUNT = 2;

// Flags
enum {FLAG_IS_HOMING = 1 << 0
    , FLAG_IS_TRIGGERED = 1 << 1
    , FLAG_IS_HOMING_TRIGGER = 1 << 2
    , FLAG_IS_FILTER_READY = 1 << 3
    };

// Endstop Structure
struct load_cell_endstop {
    struct timer time;
    uint32_t trigger_ticks, last_sample_ticks, rest_ticks;
    struct trsync *ts;
    int32_t last_sample, safety_counts_min, safety_counts_max
            , filter_counts_min, filter_counts_max
            , trigger_counts_min, trigger_counts_max, tare_counts;
    uint8_t flags, sample_count, trigger_count, trigger_reason, watchdog_max
            , watchdog_count, n_sections, last_section, round_shift;
    fixedQ12_t trigger_grams, grams_per_count;
    // filter composed of second order sections
    fixedQ12_t filter[MAX_SECTIONS][SECTION_WIDTH];     // aka sos
    fixedQ12_t filter_state[MAX_SECTIONS][STATE_WIDTH]; // aka zi
};

// Multiply two fixedQ12_t numbers into fixedQ12_t
static inline fixedQ12_t
fixedQ12_mul(const fixedQ12_t fixed_a, const fixedQ12_t fixed_b) {
    int64_t temp = (int64_t)fixed_a * (int64_t)fixed_b;
    temp += FIXEDQ12_ROUNDING;  // Round up
    return (fixedQ12_t)(temp >> FIXEDQ12_FRAC_BITS);
}

// absolute value of a fixedQ12_t
static inline fixedQ12_t
fixedQ12_abs(const fixedQ12_t fixed_a) {
    return (fixed_a < 0 ? -fixed_a : fixed_a);
}

// Convert sensor counts to grams
static inline fixedQ12_t
counts_to_grams(struct load_cell_endstop *lce, const int32_t counts) {
    // tearing ensures readings are referenced to 0.0g
    const int32_t tare =  counts - lce->tare_counts;
    const uint8_t is_negative = tare < 0;
    uint32_t tare_abs = (uint32_t)(is_negative ? -tare : tare);
    // round_shift truncates large counts per gram values
    tare_abs = (tare_abs >> lce->round_shift);
    // convert sensor counts to grams by multiplication: 124 * 0.051 = 6.324
    // Host guarantees 0 < grams_per_count < 1
    const fixedQ12_t grams = tare_abs * lce->grams_per_count;
    return is_negative ? -grams : grams;
}

/*
 *  Second Order Sections Filter Implementaiton
 */
// filter Q12 grams input value, return Q12 grams filtered
fixedQ12_t
sosfilt(struct load_cell_endstop *lce, const fixedQ12_t grams_in) {
    fixedQ12_t cur_val = grams_in;
    for (int section = 0; section < lce->n_sections; section++) {
        fixedQ12_t next_val = fixedQ12_mul(lce->filter[section][0], cur_val)
                              + lce->filter_state[section][0];
        lce->filter_state[section][0] =
            fixedQ12_mul((lce->filter[section][1]), cur_val)
            - fixedQ12_mul((lce->filter[section][3]), next_val)
            + (lce->filter_state[section][1]);
        lce->filter_state[section][1] =
            fixedQ12_mul((lce->filter[section][2]), cur_val)
            - fixedQ12_mul((lce->filter[section][4]), next_val);
        cur_val = next_val;
    }
    return cur_val;
}

// resets the filter state (zi) to 0 to start a new homing event
// because of taring to 0g, this means the settling time is also 0.
void
reset_filter_state(struct load_cell_endstop *lce) {
    for (uint8_t i = 0; i < MAX_SECTIONS; i++){
        lce->filter_state[i][0] = 0;
        lce->filter_state[i][1] = 0;
    }
}

static inline uint8_t
is_flag_set(const uint8_t mask, struct load_cell_endstop *lce)
{
    return !!(mask & lce->flags);
}

static inline void
set_flag(uint8_t mask, struct load_cell_endstop *lce)
{
    lce->flags |= mask;
}

static inline void
clear_flag(uint8_t mask, struct load_cell_endstop *lce)
{
    lce->flags &= ~mask;
}

// mark filter as not ready for use, ready to be updated
void reset_filter(struct load_cell_endstop *lce, uint8_t n_sections) {
    if (n_sections > MAX_SECTIONS) {
        shutdown("Filter section count too large");
    }
    lce->n_sections = n_sections;
    lce->last_section = -1;
    clear_flag(FLAG_IS_FILTER_READY, lce);
}

void
try_trigger(struct load_cell_endstop *lce)
{
    // set live trigger flag
    set_flag(FLAG_IS_TRIGGERED, lce);

    uint8_t is_homing = is_flag_set(FLAG_IS_HOMING, lce);
    uint8_t is_homing_triggered = is_flag_set(FLAG_IS_HOMING_TRIGGER, lce);
    if (is_homing && !is_homing_triggered) {
        // this flag latches until a reset, disabling further triggering
        set_flag(FLAG_IS_HOMING_TRIGGER, lce);
        trsync_do_trigger(lce->ts, lce->trigger_reason);
    }
}

// Used by Sensors to report new raw ADC sample
void
load_cell_endstop_report_sample(struct load_cell_endstop *lce
                                , const int32_t sample, const uint32_t ticks)
{
    // save new sample
    lce->last_sample = sample;
    lce->last_sample_ticks = ticks;
    lce->watchdog_count = 0;
    const uint8_t is_homing = is_flag_set(FLAG_IS_HOMING, lce);

    // check for safety limit violations first
    const uint8_t is_safety_trigger = sample <= lce->safety_counts_min
                                        || sample >= lce->safety_counts_max;
    if (is_homing && is_safety_trigger) {
        shutdown("Load cell endstop: too much force!");
    }

    // Use filter if filter is ready and trigger_grams configured
    const uint8_t use_filter = is_flag_set(FLAG_IS_FILTER_READY, lce)
                                && lce->trigger_grams != 0;
    uint8_t is_trigger;
    if (is_homing && use_filter) {
        if (sample < lce->filter_counts_min
                || sample > lce->filter_counts_max) {
            shutdown("Continuous tare drift limit exceeded while homing");
        }
        const fixedQ12_t grams = counts_to_grams(lce, sample);
        const fixedQ12_t filtered_grams = sosfilt(lce, grams);
        const fixedQ12_t abs_grams = fixedQ12_abs(filtered_grams);
        is_trigger = abs_grams >= lce->trigger_grams;
        /*// DEBUG: uncomment to log filter trigger in grams
        if (is_trigger) {
            output("Filter Trigger at %i grams"
                    , (uint32_t)(abs_grams >> FIXEDQ12_FRAC_BITS));
        }*/
    } else {
        // static threashold triggering for QUERY_ENDSTOPS and non-filter use
        is_trigger = sample <= lce->trigger_counts_min
                        || sample >= lce->trigger_counts_max;
    }

    const uint8_t is_homing_triggered = is_flag_set(FLAG_IS_HOMING_TRIGGER,
                                                lce);
    // update trigger state
    if (is_trigger && lce->trigger_count > 0) {
        // the first triggering sample when homing sets the trigger time
        if (is_homing && !is_homing_triggered
                && lce->trigger_count == lce->sample_count) {
            lce->trigger_ticks = ticks;
        }

        lce->trigger_count -= 1;
        // when the trigger count hits zero, trigger the trsync
        if (lce->trigger_count == 0) {
            try_trigger(lce);
        }
    }
    else if (!is_trigger && lce->trigger_count < lce->sample_count) {
        // any sample thats not a trigger resets the trigger count
        lce->trigger_count = lce->sample_count;

        // clear "live" trigger view
        clear_flag(FLAG_IS_TRIGGERED, lce);

        // if homing, but not yet triggered, clear the trigger time
        if (is_homing && !is_homing_triggered) {
            lce->trigger_ticks = 0;
        }
    }
}

// Timer callback that monitors for timeouts
static uint_fast8_t
watchdog_event(struct timer *t)
{
    struct load_cell_endstop *lce = container_of(t, struct load_cell_endstop
                                        , time);
    uint8_t is_homing = is_flag_set(FLAG_IS_HOMING, lce);
    uint8_t is_homing_trigger = is_flag_set(FLAG_IS_HOMING_TRIGGER, lce);
    // the watchdog stops when not homing or when trsync becomes triggered
    if (!is_homing || is_homing_trigger) {
        return SF_DONE;
    }

    irq_disable();
    if (lce->watchdog_count > lce->watchdog_max) {
        shutdown("LoadCell Endstop timed out waiting on ADC data");
    }
    lce->watchdog_count += 1;
    irq_enable();

    // A sample was recently delivered, continue monitoring
    lce->time.waketime += lce->rest_ticks;
    return SF_RESCHEDULE;
}

static void
set_endstop_range(struct load_cell_endstop *lce
                , int32_t safety_counts_min, int32_t safety_counts_max
                , int32_t filter_counts_min, int32_t filter_counts_max
                , int32_t trigger_counts_min, int32_t trigger_counts_max
                , int32_t tare_counts, fixedQ12_t trigger_grams
                , uint8_t round_shift, fixedQ12_t grams_per_count)
{
    if (!(safety_counts_max >= safety_counts_min)) {
        shutdown("Safety range reversed");
    }
    // may all be 0, but must be an orderly range: max >= tare >= min
    if (!(trigger_counts_max >= tare_counts
            && tare_counts >= trigger_counts_min)) {
        shutdown("Trigger range reversed");
    }
    // may all be 0, but must be an orderly range: max >= tare >= min
    if (safety_counts_max < trigger_counts_max
            || safety_counts_min > trigger_counts_min) {
        shutdown("Trigger range outside safety range");
    }
    // may all be 0, but must be an orderly range: max >= tare >= min
    if (safety_counts_max < filter_counts_max
            || safety_counts_min > filter_counts_min) {
        shutdown("Filter range outside safety range");
    }
    // grams_per_count must be a positive fraction
    const fixedQ12_t one = 1 << FIXEDQ12_FRAC_BITS;
    if (grams_per_count < 0 || grams_per_count > one) {
        shutdown("grams_per_count is invalid");
    }
    lce->safety_counts_min = safety_counts_min;
    lce->safety_counts_max = safety_counts_max;
    lce->filter_counts_min = filter_counts_min;
    lce->filter_counts_max = filter_counts_max;
    lce->trigger_counts_min = trigger_counts_min;
    lce->trigger_counts_max = trigger_counts_max;
    lce->tare_counts = tare_counts;
    lce->trigger_grams = trigger_grams;
    lce->round_shift = round_shift;
    lce->grams_per_count = grams_per_count;
    reset_filter_state(lce);
}

// Create a load_cell_endstop
void
command_config_load_cell_endstop(uint32_t *args)
{
    struct load_cell_endstop *lce = oid_alloc(args[0]
                            , command_config_load_cell_endstop, sizeof(*lce));
    lce->flags = 0;
    lce->trigger_count = lce->sample_count = DEFAULT_SAMPLE_COUNT;
    lce->trigger_ticks = 0;
    lce->watchdog_max = 0;
    lce->watchdog_count = 0;
    set_endstop_range(lce, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
}
DECL_COMMAND(command_config_load_cell_endstop, "config_load_cell_endstop"
                                               " oid=%c");

// Lookup a load_cell_endstop
struct load_cell_endstop *
load_cell_endstop_oid_lookup(uint8_t oid)
{
    return oid_lookup(oid, command_config_load_cell_endstop);
}

// Set one section of the filter
void
command_config_filter_section(uint32_t *args)
{
    struct load_cell_endstop *lce = load_cell_endstop_oid_lookup(args[0]);
    uint8_t n_sections = args[1];
    uint8_t section_idx = args[2];
    // if this is the first filter section, initalize
    if (section_idx == 0)  {
        reset_filter(lce, n_sections);
    } else {
        // validate that this new section is in order
        if (n_sections != lce->n_sections) {
            shutdown("Filter size mismatch");
        }
        if (section_idx != lce->last_section + 1) {
            shutdown("Filter section out of order");
        }
    }
    // copy section data
    const uint8_t arg_base = 3;
    for (uint8_t i = 0; i < SECTION_WIDTH; i++) {
        lce->filter[section_idx][i] = args[i + arg_base];
    }

    /*
    // DEBUG: uncomment to log filter section contents
    for (uint8_t i = 0; i < SECTION_WIDTH; i++) {
        output("section[%c][%c]=%i", section_idx, i,
                                    (int32_t)lce->sos[section_idx][i]);
    }*/

    // update counter & check for last section
    lce->last_section += 1;
    if (lce->last_section == lce->n_sections - 1) {
        set_flag(FLAG_IS_FILTER_READY, lce);
    }
}
DECL_COMMAND(command_config_filter_section
    , "config_filter_section_load_cell_endstop oid=%c n_sections=%c"
    " section_idx=%c sos0=%i sos1=%i sos2=%i sos3=%i sos4=%i");

// Set the triggering range and tare value
void
command_set_range_load_cell_endstop(uint32_t *args)
{
    struct load_cell_endstop *lce = load_cell_endstop_oid_lookup(args[0]);
    set_endstop_range(lce, args[1], args[2], args[3], args[4], args[5]
                , args[6], args[7], args[8], args[9], (fixedQ12_t)args[10]);
}
DECL_COMMAND(command_set_range_load_cell_endstop, "set_range_load_cell_endstop"
    " oid=%c safety_counts_min=%i safety_counts_max=%i"
    " filter_counts_min=%i filter_counts_max=%i"
    " trigger_counts_min=%i trigger_counts_max=%i tare_counts=%i"
    " trigger_grams=%i round_shift=%c grams_per_count=%i");

// Home an axis
void
command_load_cell_endstop_home(uint32_t *args)
{
    struct load_cell_endstop *lce = load_cell_endstop_oid_lookup(args[0]);
    sched_del_timer(&lce->time);
    // clear the homing trigger flag
    clear_flag(FLAG_IS_HOMING_TRIGGER, lce);
    clear_flag(FLAG_IS_HOMING, lce);
    lce->trigger_ticks = 0;
    lce->ts = NULL;
    // 0 samples indicates homing is finished
    if (args[3] == 0) {
        // Disable end stop checking
        return;
    }
    lce->ts = trsync_oid_lookup(args[1]);
    lce->trigger_reason = args[2];
    lce->time.waketime = args[3];
    lce->sample_count = args[4];
    lce->rest_ticks = args[5];
    lce->watchdog_max = args[6];
    lce->watchdog_count = 0;
    reset_filter_state(lce);
    lce->time.func = watchdog_event;
    sched_add_timer(&lce->time);
    set_flag(FLAG_IS_HOMING, lce);
}
DECL_COMMAND(command_load_cell_endstop_home,
             "load_cell_endstop_home oid=%c trsync_oid=%c trigger_reason=%c"
             " clock=%u sample_count=%c rest_ticks=%u timeout=%u");

void
command_load_cell_endstop_query_state(uint32_t *args)
{
    uint8_t oid = args[0];
    struct load_cell_endstop *lce = load_cell_endstop_oid_lookup(args[0]);
    sendf("load_cell_endstop_state oid=%c homing=%c homing_triggered=%c"
        " is_triggered=%c trigger_ticks=%u sample=%i sample_ticks=%u"
            , oid, is_flag_set(FLAG_IS_HOMING, lce)
            , is_flag_set(FLAG_IS_HOMING_TRIGGER, lce)
            , is_flag_set(FLAG_IS_TRIGGERED, lce), lce->trigger_ticks
            , lce->last_sample, lce->last_sample_ticks);
}
DECL_COMMAND(command_load_cell_endstop_query_state
                , "load_cell_endstop_query_state oid=%c");
