/*
 * ZMK LED Indicators — battery level on all underglow LEDs
 *
 * On boot: all LEDs show battery charge as a green-to-red color for 60s.
 * Below critical threshold: dim red on all LEDs, always on.
 *
 * Color mapping (smooth gradient):
 *   100% = pure green
 *    50% = yellow
 *     0% = pure red
 *
 * Both halves of a split keyboard run independently using their own
 * battery gauge and LED strip.
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/logging/log.h>

#include <zmk/event_manager.h>
#include <zmk/events/battery_state_changed.h>

#if IS_ENABLED(CONFIG_ZMK_EXT_POWER)
#include <drivers/ext_power.h>
#endif

LOG_MODULE_REGISTER(led_indicators, CONFIG_ZMK_LOG_LEVEL);

#define LED_NODE   DT_CHOSEN(zmk_underglow)
#define NUM_PIXELS DT_PROP(LED_NODE, chain_length)

#define BOOT_BRIGHT  CONFIG_ZMK_LED_INDICATORS_BOOT_BRIGHTNESS
#define CRIT_BRIGHT  CONFIG_ZMK_LED_INDICATORS_CRITICAL_BRIGHTNESS
#define BOOT_DUR_S   CONFIG_ZMK_LED_INDICATORS_BOOT_DURATION_S
#define CRIT_PCT     CONFIG_ZMK_LED_INDICATORS_CRITICAL_PCT

/* --- state --- */

enum ind_mode {
    MODE_OFF,
    MODE_BOOT,     /* showing battery colour, temporary */
    MODE_CRITICAL, /* <10%, dim red, always on */
};

static const struct device *strip = DEVICE_DT_GET(LED_NODE);
static struct led_rgb pixels[NUM_PIXELS];

static enum ind_mode mode = MODE_OFF;
static uint8_t battery_level = 255; /* 255 = not yet reported */

/* --- colour math --- */

/*
 * Smooth green-to-red via two linear segments:
 *   100% → (0, bright, 0)       pure green
 *    50% → (bright, bright, 0)  yellow
 *     0% → (bright, 0, 0)       pure red
 */
static struct led_rgb battery_color(uint8_t level, uint8_t bright) {
    struct led_rgb c = {.r = 0, .g = 0, .b = 0};

    if (level > 100) {
        level = 100;
    }

    if (level >= 50) {
        /* green to yellow: red ramps up, green stays full */
        c.r = (uint8_t)((uint16_t)bright * (100 - level) / 50);
        c.g = bright;
    } else {
        /* yellow to red: red stays full, green ramps down */
        c.r = bright;
        c.g = (uint8_t)((uint16_t)bright * level / 50);
    }

    return c;
}

/* --- LED helpers --- */

static void set_all(struct led_rgb color) {
    for (int i = 0; i < NUM_PIXELS; i++) {
        pixels[i] = color;
    }
}

static void push(void) {
    led_strip_update_rgb(strip, pixels, NUM_PIXELS);
}

static void leds_off(void) {
    struct led_rgb off = {.r = 0, .g = 0, .b = 0};
    set_all(off);
    push();
}

/* --- update the LEDs based on current mode + battery --- */

static void refresh_leds(void) {
    switch (mode) {
    case MODE_BOOT:
        if (battery_level <= 100) {
            set_all(battery_color(battery_level, BOOT_BRIGHT));
        } else {
            /* battery not yet reported — dim yellow as placeholder */
            struct led_rgb unknown = {.r = BOOT_BRIGHT / 4,
                                      .g = BOOT_BRIGHT / 8, .b = 0};
            set_all(unknown);
        }
        push();
        break;

    case MODE_CRITICAL: {
        struct led_rgb dim_red = {.r = CRIT_BRIGHT, .g = 0, .b = 0};
        set_all(dim_red);
        push();
        break;
    }

    case MODE_OFF:
        leds_off();
        break;
    }
}

/* --- timers / work --- */

static void boot_expire_work_handler(struct k_work *work);
static K_WORK_DEFINE(boot_expire_work, boot_expire_work_handler);

static void boot_timer_handler(struct k_timer *timer) {
    k_work_submit(&boot_expire_work);
}
static K_TIMER_DEFINE(boot_timer, boot_timer_handler, NULL);

static void boot_expire_work_handler(struct k_work *work) {
    if (mode != MODE_BOOT) {
        return;
    }

    /* Boot display time is up — go critical or off */
    if (battery_level <= CRIT_PCT) {
        mode = MODE_CRITICAL;
    } else {
        mode = MODE_OFF;
    }
    refresh_leds();
}

/* --- ZMK battery event --- */

static void battery_work_handler(struct k_work *work);
static K_WORK_DEFINE(battery_work, battery_work_handler);

static uint8_t pending_battery_level;

static void battery_work_handler(struct k_work *work) {
    battery_level = pending_battery_level;
    LOG_DBG("Battery: %u%%", battery_level);

    if (battery_level <= CRIT_PCT) {
        /* Enter critical mode regardless of current mode */
        mode = MODE_CRITICAL;
        refresh_leds();
    } else if (mode == MODE_CRITICAL) {
        /* Was critical, now recovered (charging?) — turn off */
        mode = MODE_OFF;
        refresh_leds();
    } else if (mode == MODE_BOOT) {
        /* Still in boot display — update the colour */
        refresh_leds();
    }
    /* If MODE_OFF and not critical, stay off */
}

static int on_battery_changed(const zmk_event_t *eh) {
    const struct zmk_battery_state_changed *ev =
        as_zmk_battery_state_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    pending_battery_level = ev->state_of_charge;
    k_work_submit(&battery_work);

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(led_ind_batt, on_battery_changed);
ZMK_SUBSCRIPTION(led_ind_batt, zmk_battery_state_changed);

/* --- boot --- */

static void boot_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(boot_work, boot_work_handler);

static void boot_work_handler(struct k_work *work) {
    mode = MODE_BOOT;
    refresh_leds();

    /* Schedule transition out of boot display */
    k_timer_start(&boot_timer, K_SECONDS(BOOT_DUR_S), K_NO_WAIT);

    LOG_INF("LED indicators: boot display active");
}

static int led_indicators_init(void) {
    if (!device_is_ready(strip)) {
        LOG_ERR("LED strip device not ready");
        return -ENODEV;
    }

#if IS_ENABLED(CONFIG_ZMK_EXT_POWER)
    /* Ensure external power is on — LEDs need it */
    const struct device *ext_power = device_get_binding("EXT_POWER");
    if (ext_power != NULL) {
        ext_power_enable(ext_power);
    }
#endif

    /* Wait for battery gauge to report before showing anything */
    k_work_schedule(&boot_work, K_SECONDS(2));
    return 0;
}

SYS_INIT(led_indicators_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
