#include <cstdint>
#include <cstdio>

#include "cmsis_os2.h"
#include "main.h"
#include "ui_input.hpp"

extern "C" {
#include "bits_button/bits_button.h"
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
    if (result.event == BTN_EVENT_FINISH) {
        return 1;
    }

    return (uint8_t)((result.event == BTN_EVENT_LONG_PRESS)
                     && (result.key_value == BITS_BTN_LONG_PRESS_START_KV));
}

bool publish_ui_event(uint16_t key_id, bits_btn_result_t result)
{
    UiInputEvent event = UiInputEvent::NextItem;

    if (result.event == BTN_EVENT_LONG_PRESS) {
        event = (key_id == USER_BUTTON_0) ? UiInputEvent::Cancel : UiInputEvent::Confirm;
        return ui_input_publish(event);
    }

    if (result.event != BTN_EVENT_FINISH) {
        return false;
    }

    switch (result.key_value) {
        case BITS_BTN_SINGLE_CLICK_KV:
            event = (key_id == USER_BUTTON_0) ? UiInputEvent::PrevItem : UiInputEvent::NextItem;
            return ui_input_publish(event);
        case BITS_BTN_DOUBLE_CLICK_KV:
            event = (key_id == USER_BUTTON_0) ? UiInputEvent::PrevPage : UiInputEvent::NextPage;
            return ui_input_publish(event);
        default:
            return false;
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

    ui_input_init();

    if (bits_button_init(&config) != BITS_BTN_OK) {
        std::printf("[key] bits_button init failed\r\n");
        for (;;) {
            osDelay(1000);
        }
    }

    bits_btn_register_result_filter_callback(should_buffer_result);
    std::printf("[key] task ready: K0(prev/item,prev/page,cancel) K1(next/item,next/page,confirm)\r\n");

    for (;;) {
        bits_button_ticks();

        bits_btn_result_t result = {};
        while (bits_button_get_key_result(&result)) {
            if (result.key_id >= USER_BUTTON_MAX) {
                continue;
            }

            if (publish_ui_event(result.key_id, result)) {
                if (result.event == BTN_EVENT_LONG_PRESS) {
                    std::printf("[key] %s long-press -> %s\r\n",
                                button_name(result.key_id),
                                (result.key_id == USER_BUTTON_0) ? "cancel" : "confirm");
                } else if (result.key_value == BITS_BTN_SINGLE_CLICK_KV) {
                    std::printf("[key] %s short -> %s\r\n",
                                button_name(result.key_id),
                                (result.key_id == USER_BUTTON_0) ? "prev-item" : "next-item");
                } else if (result.key_value == BITS_BTN_DOUBLE_CLICK_KV) {
                    std::printf("[key] %s double -> %s\r\n",
                                button_name(result.key_id),
                                (result.key_id == USER_BUTTON_0) ? "prev-page" : "next-page");
                }
            }
        }

        osDelay(BITS_BTN_TICKS_INTERVAL);
    }
}
