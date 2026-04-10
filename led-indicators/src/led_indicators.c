/*
 * ZMK LED Indicators
 *
 * Battery: all LEDs show charge as green-to-red on boot (60s), dim red
 *          always on below critical threshold.
 *
 * BT (central/left only):
 *   - Profile selected:  1 blue blink on the BT_SEL key
 *   - Profile connected: 2 blue blinks on the BT_SEL key
 *   - Profile cleared:   1 red blink on the BT_SEL key
 *   Only profiles 0-2 (left-side keys).
 *
 * Both halves run battery indicators independently.
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

#define IS_BT_CENTRAL                                                          \
    (IS_ENABLED(CONFIG_ZMK_BLE) &&                                             \
     (!IS_ENABLED(CONFIG_ZMK_SPLIT) ||                                         \
      IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)))

#if IS_BT_CENTRAL
#include <zmk/ble.h>
#include <zmk/events/ble_active_profile_changed.h>
#endif

LOG_MODULE_REGISTER(led_indicators, CONFIG_ZMK_LOG_LEVEL);

#define LED_NODE   DT_CHOSEN(zmk_underglow)
#define NUM_PIXELS DT_PROP(LED_NODE, chain_length)

#define BOOT_BRIGHT  CONFIG_ZMK_LED_INDICATORS_BOOT_BRIGHTNESS
#define CRIT_BRIGHT  CONFIG_ZMK_LED_INDICATORS_CRITICAL_BRIGHTNESS
#define BOOT_DUR_S   CONFIG_ZMK_LED_INDICATORS_BOOT_DURATION_S
#define CRIT_PCT     CONFIG_ZMK_LED_INDICATORS_CRITICAL_PCT

/*
 * LED indices for BT_SEL 0/1/2 on the left half.
 * From Keebart QMK keyboard.json — chain snakes column-by-column.
 * Upper layer row 2: col 3 = BT_SEL 0, col 4 = BT_SEL 1, col 5 = BT_SEL 2
 */
#define BT_LED_PROFILE_0  9
#define BT_LED_PROFILE_1  6
#define BT_LED_PROFILE_2  1

#define BLINK_ON_MS  300
#define BLINK_GAP_MS 200

/* --- state --- */

enum ind_mode {
    MODE_OFF,
    MODE_BOOT,
    MODE_CRITICAL,
};

static const struct device *strip = DEVICE_DT_GET(LED_NODE);
static struct led_rgb pixels[NUM_PIXELS];

static enum ind_mode mode = MODE_OFF;
static uint8_t battery_level = 255;

/* --- colour math --- */

static struct led_rgb battery_color(uint8_t level, uint8_t bright) {
    struct led_rgb c = {.r = 0, .g = 0, .b = 0};

    if (level > 100) {
        level = 100;
    }

    if (level >= 50) {
        c.r = (uint8_t)((uint16_t)bright * (100 - level) / 50);
        c.g = bright;
    } else {
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
    set_all((struct led_rgb){.r = 0, .g = 0, .b = 0});
    push();
}

/* Return what color a given LED should be in the current mode */
static struct led_rgb mode_color_for_led(void) {
    switch (mode) {
    case MODE_BOOT:
        if (battery_level <= 100) {
            return battery_color(battery_level, BOOT_BRIGHT);
        }
        return (struct led_rgb){.r = BOOT_BRIGHT / 4,
                                .g = BOOT_BRIGHT / 8, .b = 0};
    case MODE_CRITICAL:
        return (struct led_rgb){.r = CRIT_BRIGHT, .g = 0, .b = 0};
    case MODE_OFF:
    default:
        return (struct led_rgb){.r = 0, .g = 0, .b = 0};
    }
}

static void refresh_leds(void) {
    set_all(mode_color_for_led());
    push();
}

/* --- boot timer --- */

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
    mode = (battery_level <= CRIT_PCT) ? MODE_CRITICAL : MODE_OFF;
    refresh_leds();
}

/* --- BT profile blink (central only) --- */

#if IS_BT_CENTRAL

static const int bt_profile_leds[] = {
    BT_LED_PROFILE_0,
    BT_LED_PROFILE_1,
    BT_LED_PROFILE_2,
};

static int bt_blink_led = -1;
static struct led_rgb bt_blink_color;
static int bt_blinks_remaining;
static bool bt_blink_is_on;

/* Previous state for detecting what changed */
static int prev_profile_index = -1;
static bool prev_connected;
static bool prev_open;

static void bt_blink_tick_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(bt_blink_tick_work, bt_blink_tick_handler);

static void bt_blink_tick_handler(struct k_work *work) {
    if (bt_blink_led < 0) {
        return;
    }

    if (bt_blink_is_on) {
        /* Turn off */
        pixels[bt_blink_led] = mode_color_for_led();
        push();
        bt_blink_is_on = false;
        bt_blinks_remaining--;

        if (bt_blinks_remaining > 0) {
            k_work_schedule(&bt_blink_tick_work, K_MSEC(BLINK_GAP_MS));
        } else {
            bt_blink_led = -1;
        }
    } else {
        /* Turn on */
        pixels[bt_blink_led] = bt_blink_color;
        push();
        bt_blink_is_on = true;
        k_work_schedule(&bt_blink_tick_work, K_MSEC(BLINK_ON_MS));
    }
}

static void start_blink(int led, struct led_rgb color, int count) {
    /* Cancel any running blink */
    k_work_cancel_delayable(&bt_blink_tick_work);

    /* Restore previous blink LED if switching to a different one */
    if (bt_blink_led >= 0 && bt_blink_led != led) {
        pixels[bt_blink_led] = mode_color_for_led();
    }

    bt_blink_led = led;
    bt_blink_color = color;
    bt_blinks_remaining = count;
    bt_blink_is_on = true;

    pixels[led] = color;
    push();

    k_work_schedule(&bt_blink_tick_work, K_MSEC(BLINK_ON_MS));
}

static int on_ble_profile_changed(const zmk_event_t *eh) {
    int index = zmk_ble_active_profile_index();
    bool connected = zmk_ble_active_profile_is_connected();
    bool open = zmk_ble_active_profile_is_open();

    /* Only handle profiles 0-2 */
    if (index < 0 || index >= (int)ARRAY_SIZE(bt_profile_leds)) {
        prev_profile_index = index;
        prev_connected = connected;
        prev_open = open;
        return ZMK_EV_EVENT_BUBBLE;
    }

    int led = bt_profile_leds[index];
    struct led_rgb blue = {.r = 0, .g = 0, .b = BOOT_BRIGHT};
    struct led_rgb red = {.r = BOOT_BRIGHT, .g = 0, .b = 0};

    if (index != prev_profile_index) {
        /* Profile switched → 1 blue blink */
        start_blink(led, blue, 1);
    } else if (connected && !prev_connected) {
        /* Just connected → 2 blue blinks */
        start_blink(led, blue, 2);
    } else if (open && !prev_open) {
        /* Just cleared → 1 red blink */
        start_blink(led, red, 1);
    }

    prev_profile_index = index;
    prev_connected = connected;
    prev_open = open;

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(led_ind_ble, on_ble_profile_changed);
ZMK_SUBSCRIPTION(led_ind_ble, zmk_ble_active_profile_changed);

#endif /* IS_BT_CENTRAL */

/* --- ZMK battery event --- */

static void battery_work_handler(struct k_work *work);
static K_WORK_DEFINE(battery_work, battery_work_handler);

static uint8_t pending_battery_level;

static void battery_work_handler(struct k_work *work) {
    battery_level = pending_battery_level;
    LOG_DBG("Battery: %u%%", battery_level);

    if (battery_level <= CRIT_PCT) {
        mode = MODE_CRITICAL;
        refresh_leds();
    } else if (mode == MODE_CRITICAL) {
        mode = MODE_OFF;
        refresh_leds();
    } else if (mode == MODE_BOOT) {
        refresh_leds();
    }
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
    k_timer_start(&boot_timer, K_SECONDS(BOOT_DUR_S), K_NO_WAIT);

#if IS_BT_CENTRAL
    prev_profile_index = zmk_ble_active_profile_index();
    prev_connected = zmk_ble_active_profile_is_connected();
    prev_open = zmk_ble_active_profile_is_open();
#endif

    LOG_INF("LED indicators: boot display active");
}

static int led_indicators_init(void) {
    if (!device_is_ready(strip)) {
        LOG_ERR("LED strip device not ready");
        return -ENODEV;
    }

#if IS_ENABLED(CONFIG_ZMK_EXT_POWER)
    const struct device *ext_power = device_get_binding("EXT_POWER");
    if (ext_power != NULL) {
        ext_power_enable(ext_power);
    }
#endif

    k_work_schedule(&boot_work, K_SECONDS(2));
    return 0;
}

SYS_INIT(led_indicators_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
