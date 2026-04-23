#include "ui_input.hpp"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

namespace {

constexpr size_t kQueueDepth = 16;

StaticQueue_t s_queue_struct;
uint8_t       s_queue_storage[kQueueDepth * sizeof(UiInputEvent)];
QueueHandle_t s_queue = nullptr;

QueueHandle_t ensure_queue()
{
    taskENTER_CRITICAL();
    if (s_queue == nullptr) {
        s_queue = xQueueCreateStatic(kQueueDepth, sizeof(UiInputEvent), s_queue_storage, &s_queue_struct);
    }
    taskEXIT_CRITICAL();
    return s_queue;
}

} // namespace

void ui_input_init()
{
    (void)ensure_queue();
}

bool ui_input_publish(UiInputEvent event)
{
    QueueHandle_t queue = ensure_queue();
    if (queue == nullptr) {
        return false;
    }

    return xQueueSend(queue, &event, 0) == pdPASS;
}

bool ui_input_poll(UiInputEvent &event, uint32_t timeout_ms)
{
    QueueHandle_t queue = ensure_queue();
    if (queue == nullptr) {
        return false;
    }

    return xQueueReceive(queue, &event, pdMS_TO_TICKS(timeout_ms)) == pdPASS;
}

const char *ui_input_name(UiInputEvent event)
{
    switch (event) {
        case UiInputEvent::PrevItem:
            return "prev-item";
        case UiInputEvent::NextItem:
            return "next-item";
        case UiInputEvent::PrevPage:
            return "prev-page";
        case UiInputEvent::NextPage:
            return "next-page";
        case UiInputEvent::Cancel:
            return "cancel";
        case UiInputEvent::Confirm:
            return "confirm";
        default:
            return "unknown";
    }
}
