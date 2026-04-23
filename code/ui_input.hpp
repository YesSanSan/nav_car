#pragma once

#include <cstdint>

enum class UiInputEvent : uint8_t {
    PrevItem = 0,
    NextItem,
    PrevPage,
    NextPage,
    Cancel,
    Confirm,
};

void        ui_input_init();
bool        ui_input_publish(UiInputEvent event);
bool        ui_input_poll(UiInputEvent &event, uint32_t timeout_ms = 0);
const char *ui_input_name(UiInputEvent event);
