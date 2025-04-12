// Support for bit-banging commands to HX711 and HX717 ADC chips
//
// Copyright (C) 2024 Gareth Farrington <gareth@waves.ky>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_MACH_AVR
#include "board/gpio.h" // gpio_out_write
#include "board/irq.h" // irq_poll
#include "board/misc.h" // timer_read_time
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // sched_add_timer
#include "sensor_bulk.h" // sensor_bulk_report
#include "load_cell_endstop.h" // load_cell_endstop_report_sample
#include <stdint.h>

struct hx71x_adc {
    struct timer timer;
    uint32_t rest_ticks;
    struct gpio_in dout; // pins used to receive data from the hx71x 0
    struct gpio_out sclk; // pins used to generate clock for the hx71x 0
    uint8_t gain_channel;   // the gain+channel selection (1-4)
    uint8_t flags;
    struct sensor_bulk sb;
    struct load_cell_endstop *lce;
};

// Flag types
enum {
    FLAG_PENDING = 1 << 0
};

#define BYTES_PER_SAMPLE 4

static struct task_wake wake_hx71x;

/****************************************************************
 * Timing
 ****************************************************************/
static uint32_t
nsecs_to_ticks(uint32_t ns)
{
    return timer_from_us(ns * 1000) / 1000000;
}

static inline void
hx71x_delay(uint32_t ticks)
{
    if (CONFIG_MACH_AVR)
        // Slower MCUs don't require a delay
        return;
    uint32_t end = timer_read_time() + ticks;
    while (timer_is_before(timer_read_time(), end))
        irq_poll();
}

/****************************************************************
 * HX711 and HX717 Sensor Support
 ****************************************************************/
// both HX717 and HX711 have 200ns min pulse time for clock pin on/off
#define MIN_PULSE_TIME nsecs_to_ticks(200)
// wakeup delay for HX717 is 80us and for HX711 is 60us 
#define WAKEUP_TIME timer_from_us(100)

// Event handler that wakes wake_hx71x() periodically
static uint_fast8_t
hx71x_event(struct timer *timer)
{
    struct hx71x_adc *hx71x = container_of(timer, struct hx71x_adc, timer);
    hx71x->flags |= FLAG_PENDING;
    sched_wake_task(&wake_hx71x);
    return SF_DONE;
}

// Helper code to reschedule the hx71x_event() timer
static void
hx71x_reschedule_timer(struct hx71x_adc *hx71x)
{
    irq_disable();
    hx71x->flags |= FLAG_PENDING;
    hx71x->timer.waketime = timer_read_time() + hx71x->rest_ticks;
    sched_add_timer(&hx71x->timer);
    irq_enable();
}

// Add a measurement to the buffer
static void
add_sample(struct hx71x_adc *hx71x, uint32_t counts)
{
    hx71x->sb.data[hx71x->sb.data_count] = counts;
    hx71x->sb.data[hx71x->sb.data_count + 1] = counts >> 8;
    hx71x->sb.data[hx71x->sb.data_count + 2] = counts >> 16;
    hx71x->sb.data[hx71x->sb.data_count + 3] = counts >> 24;
    hx71x->sb.data_count += BYTES_PER_SAMPLE;
}

static void
flush_samples(struct hx71x_adc *hx71x, uint8_t oid)
{
    if (hx71x->sb.data_count + BYTES_PER_SAMPLE > ARRAY_SIZE(hx71x->sb.data))
        sensor_bulk_report(&hx71x->sb, oid);
}

// Pulse all clock pins to move to the next bit
inline static void
hx71x_pulse_clocks(struct hx71x_adc *hx71x) {
    irq_disable();
    gpio_out_write(hx71x->sclk, 1);
    hx71x_delay(MIN_PULSE_TIME);
    gpio_out_write(hx71x->sclk, 0);
    irq_enable();
}

// hx71x ADC query
void
hx71x_query(struct hx71x_adc *hx71x, uint8_t oid)
{
    uint32_t start_time = timer_read_time();
    // reschedule timer if ADC is not ready
    if (gpio_in_read(hx71x->dout)) {
        hx71x_reschedule_timer(hx71x);
        return;
    }
    // some data is ready
    int32_t counts = 0;
    for (uint8_t sample_idx = 0; sample_idx < 24; sample_idx++) {
        hx71x_pulse_clocks(hx71x);
        hx71x_delay(MIN_PULSE_TIME);
        // read 2's compliment int bits
        counts = (counts << 1) | gpio_in_read(hx71x->dout);
    }
    // bit bang 1 to 4 more bits to configure gain & channel for the next sample
    for (uint8_t gain_idx = 0; gain_idx < hx71x->gain_channel; gain_idx++) {
        hx71x_pulse_clocks(hx71x);
        hx71x_delay(MIN_PULSE_TIME);
    }
    // 
    uint32_t time_diff = timer_read_time() - start_time;
    if (time_diff >= hx71x->rest_ticks) {
        // some IRQ delayed this read so much that the chips must be reset
        shutdown("HX71x read took too long");
    }
    // dout should be 1, which indacates sample not ready
    // if its 0, that probably means the chip got hit with ESD
    if (!gpio_in_read(hx71x->dout)) {
        output("HX71x dout pin is 0");
        hx71x_reschedule_timer(hx71x);
        return; // so if we see this we dont return the error to host
    }
    // extend 2's complement 24 bits to 32bits
    counts = (counts ^ 0x800000) - 0x800000;
    if (counts < -0x7FFFFF || counts > 0x7FFFFF) {
        shutdown("HX71x value out of 24 bit range");
    }
    add_sample(hx71x, counts);

    // endstop is optional, report if enabled
    if (hx71x->lce) {
        load_cell_endstop_report_sample(hx71x->lce, counts, start_time);
    }

    flush_samples(hx71x, oid);
    hx71x_reschedule_timer(hx71x);
}

// Create a hx71x sensor
void
command_config_hx71x(uint32_t *args)
{
    struct hx71x_adc *hx71x = oid_alloc(args[0]
                , command_config_hx71x, sizeof(*hx71x));
    hx71x->timer.func = hx71x_event;
    hx71x->flags = 0;
    // 
    uint8_t gain_channel = args[1];
    if (gain_channel < 1 || gain_channel > 4) {
        shutdown("HX71x gain/channel out of range 1-4");
    }
    hx71x->gain_channel = gain_channel;
    hx71x->dout = gpio_in_setup(args[2], -1);
    hx71x->sclk = gpio_out_setup(args[3], 0);
    // Wakeup ADC sensor
    gpio_out_write(hx71x->sclk, 0);
    hx71x_delay(WAKEUP_TIME);
}
DECL_COMMAND(command_config_hx71x, "config_hx71x oid=%c gain_channel=%c"
    " dout_pin=%u sclk_pin=%u");

void
command_attach_endstop_hx71x(uint32_t *args) {
    uint8_t oid = args[0];
    struct hx71x_adc *hx71x = oid_lookup(oid, command_config_hx71x);
    hx71x->lce = load_cell_endstop_oid_lookup(args[1]);
}
DECL_COMMAND(command_attach_endstop_hx71x, "attach_endstop_hx71x oid=%c"
    " load_cell_endstop_oid=%c");

// start/stop capturing ADC data
void
command_query_hx71x(uint32_t *args)
{
    uint8_t oid = args[0];
    struct hx71x_adc *hx71x = oid_lookup(oid, command_config_hx71x);
    sched_del_timer(&hx71x->timer);
    hx71x->flags = 0;
    hx71x->rest_ticks = args[1];
    if (!hx71x->rest_ticks) {
        // End measurements
        return;
    }
    // Start new measurements
    sensor_bulk_reset(&hx71x->sb);
    hx71x_reschedule_timer(hx71x);
}
DECL_COMMAND(command_query_hx71x,
             "query_hx71x oid=%c rest_ticks=%u");

void
command_query_hx71x_status(const uint32_t *args)
{
    uint8_t oid = args[0];
    struct hx71x_adc *hx71x = oid_lookup(oid, command_config_hx71x);
    const uint32_t t1 = timer_read_time();
    uint8_t pending_bytes = !gpio_in_read(hx71x->dout) * BYTES_PER_SAMPLE;
    const uint32_t t2 = timer_read_time();
    sensor_bulk_status(&hx71x->sb, oid, t1, (t2 - t1)
                      , pending_bytes);
}
DECL_COMMAND(command_query_hx71x_status, "query_hx71x_status oid=%c");

// Background task that performs measurements
void
hx71x_capture_task(void)
{
    if (!sched_check_wake(&wake_hx71x))
        return;
    uint8_t oid;
    struct hx71x_adc *hx71x;
    foreach_oid(oid, hx71x, command_config_hx71x) {
        if (hx71x->flags & FLAG_PENDING) {
            hx71x_query(hx71x, oid);
        }
    }
}
DECL_TASK(hx71x_capture_task);