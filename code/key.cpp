#include <cstdint>
#include <cstdio>

#include "cmsis_os2.h"
#include "main.h"

extern "C" {
#include "bits_button/bits_button.h"
#include "storage/config_store.h"
#include "storage/storage_service.h"
}

namespace {

enum user_button_t : uint8_t {
    USER_BUTTON_0 = 0,
    USER_BUTTON_1,
    USER_BUTTON_MAX,
};

const bits_btn_obj_param_t default_param = {
    .long_press_start_time_ms = BITS_BTN_LONG_PRESS_START_TIME_MS,
    .long_press_period_trigger_ms = BITS_BTN_LONG_PRESS_PERIOD_TRIGGER_MS,
    .time_window_time_ms = BITS_BTN_TIME_WINDOW_TIME_MS,
};

button_obj_t btns[] = {
    BITS_BUTTON_INIT(USER_BUTTON_0, 0, &default_param),
    BITS_BUTTON_INIT(USER_BUTTON_1, 0, &default_param),
};

uint32_t key_press_count[USER_BUTTON_MAX] = {0, 0};
bool storage_ready = false;
bool storage_write_ok = false;

const char *button_name(uint16_t key_id)
{
    switch (key_id) {
        case USER_BUTTON_0:
            return "key0";
        case USER_BUTTON_1:
            return "key1";
        default:
            return "key?";
    }
}

const char *button_count_key(uint16_t key_id)
{
    switch (key_id) {
        case USER_BUTTON_0:
            return "key0_count";
        case USER_BUTTON_1:
            return "key1_count";
        default:
            return "";
    }
}

uint8_t read_key_state(struct button_obj_t *btn)
{
    switch (btn->key_id) {
        case USER_BUTTON_0:
            return (uint8_t)HAL_GPIO_ReadPin(key0_GPIO_Port, key0_Pin);
        case USER_BUTTON_1:
            return (uint8_t)HAL_GPIO_ReadPin(key1_GPIO_Port, key1_Pin);
        default:
            return 1;
    }
}

uint8_t should_buffer_result(bits_btn_result_t result)
{
    return (uint8_t)((result.event == BTN_EVENT_PRESSED) || (result.event == BTN_EVENT_FINISH));
}

void load_saved_counts()
{
    uint32_t value = 0;

    if (!storage_ready) {
        return;
    }

    if (config_get_u32("key0_count", &value)) {
        key_press_count[USER_BUTTON_0] = value;
    }

    value = 0;
    if (config_get_u32("key1_count", &value)) {
        key_press_count[USER_BUTTON_1] = value;
    }
}

void persist_count(uint16_t key_id)
{
    const char *key = button_count_key(key_id);

    if (!storage_ready || !storage_write_ok || key[0] == '\0') {
        return;
    }

    if (!config_set_u32(key, key_press_count[key_id])) {
        std::printf("[key] %s count save failed\r\n", button_name(key_id));
        return;
    }

    if (!config_commit()) {
        std::printf("[key] %s commit failed\r\n", button_name(key_id));
    }
}

} // namespace

extern "C" void keyTask(void *)
{
    bits_btn_config_t config = {
        .btns = btns,
        .btns_cnt = ARRAY_SIZE(btns),
        .btns_combo = NULL,
        .btns_combo_cnt = 0,
        .read_button_level_func = read_key_state,
        .bits_btn_result_cb = NULL,
        .bits_btn_debug_printf = NULL,
    };

    storage_ready = storage_init();
    storage_write_ok = storage_ready;
    std::printf("[key] storage %s\r\n", storage_ready ? "ready" : "init failed");
    if (storage_ready) {
        load_saved_counts();
    }

    if (bits_button_init(&config) != BITS_BTN_OK) {
        std::printf("[key] bits_button init failed\r\n");
        for (;;) {
            osDelay(1000);
        }
    }

    bits_btn_register_result_filter_callback(should_buffer_result);
    std::printf("[key] task ready, key0=%lu key1=%lu\r\n",
                (unsigned long)key_press_count[USER_BUTTON_0],
                (unsigned long)key_press_count[USER_BUTTON_1]);

    for (;;) {
        bits_button_ticks();

        bits_btn_result_t result = {};
        while (bits_button_get_key_result(&result)) {
            if (result.event == BTN_EVENT_PRESSED) {
                std::printf("[key] %s down\r\n", button_name(result.key_id));
                continue;
            }

            if (result.event == BTN_EVENT_FINISH && result.key_value == BITS_BTN_SINGLE_CLICK_KV
                && result.key_id < USER_BUTTON_MAX) {
                key_press_count[result.key_id] += 1;
                persist_count(result.key_id);
                std::printf("[key] %s click, count=%lu\r\n",
                            button_name(result.key_id),
                            (unsigned long)key_press_count[result.key_id]);
            }
        }

        osDelay(BITS_BTN_TICKS_INTERVAL);
    }
}
