#include "bits_button.h"

#include <stdatomic.h>
#include <string.h>

static bits_button_t bits_btn_entity;
static bits_btn_debug_printf_func debug_printf = NULL;
static void debug_print_binary(key_value_type_t num);

typedef enum {
    BTN_STATE_IDLE,
    BTN_STATE_PRESSED,
    BTN_STATE_LONG_PRESS,
    BTN_STATE_RELEASE,
    BTN_STATE_RELEASE_WINDOW,
    BTN_STATE_FINISH
} bits_btn_state_t;

static bits_btn_event_t state_to_event(bits_btn_state_t state)
{
    switch (state) {
        case BTN_STATE_PRESSED: return BTN_EVENT_PRESSED;
        case BTN_STATE_LONG_PRESS: return BTN_EVENT_LONG_PRESS;
        case BTN_STATE_RELEASE: return BTN_EVENT_RELEASE;
        case BTN_STATE_FINISH: return BTN_EVENT_FINISH;
        default: return (bits_btn_event_t)0;
    }
}

#ifndef BITS_BTN_BUFFER_SIZE
#define BITS_BTN_BUFFER_SIZE        10
#endif

typedef struct
{
    bits_btn_result_t buffer[BITS_BTN_BUFFER_SIZE];
    atomic_size_t read_idx;
    atomic_size_t write_idx;
} bits_btn_ring_buffer_t;

static atomic_size_t overwrite_count = 0;
static bits_btn_ring_buffer_t ring_buffer;
static bits_btn_result_user_filter_callback bits_btn_result_user_filter_cb = NULL;

static void bits_btn_init_buffer_c11(void)
{
    bits_btn_ring_buffer_t *buf = &ring_buffer;

    atomic_init(&buf->read_idx, 0);
    atomic_init(&buf->write_idx, 0);
    atomic_init(&overwrite_count, 0);
}

static uint8_t bits_btn_is_buffer_empty_c11(void)
{
    bits_btn_ring_buffer_t *buf = &ring_buffer;
    size_t current_read = atomic_load_explicit(&buf->read_idx, memory_order_relaxed);
    size_t current_write = atomic_load_explicit(&buf->write_idx, memory_order_relaxed);
    return current_read == current_write;
}

static uint8_t bits_btn_is_buffer_full_c11(void)
{
    bits_btn_ring_buffer_t *buf = &ring_buffer;
    size_t current_write = atomic_load_explicit(&buf->write_idx, memory_order_relaxed);
    size_t current_read = atomic_load_explicit(&buf->read_idx, memory_order_relaxed);
    size_t next_write = (current_write + 1) % BITS_BTN_BUFFER_SIZE;
    return next_write == current_read;
}

static size_t get_bits_btn_buffer_used_count_c11(void)
{
    bits_btn_ring_buffer_t *buf = &ring_buffer;
    size_t current_write = atomic_load_explicit(&buf->write_idx, memory_order_relaxed);
    size_t current_read = atomic_load_explicit(&buf->read_idx, memory_order_relaxed);

    if (current_write >= current_read) {
        return current_write - current_read;
    }
    return BITS_BTN_BUFFER_SIZE - current_read + current_write;
}

static size_t get_bits_btn_buffer_capacity_c11(void)
{
    return BITS_BTN_BUFFER_SIZE - 1;
}

static void bits_btn_clear_buffer_c11(void)
{
    bits_btn_ring_buffer_t *buf = &ring_buffer;
    atomic_store_explicit(&buf->write_idx, 0, memory_order_release);
    atomic_store_explicit(&buf->read_idx, 0, memory_order_release);
}

static size_t get_bits_btn_buffer_overwrite_count_c11(void)
{
    return atomic_load_explicit(&overwrite_count, memory_order_relaxed);
}

static uint8_t bits_btn_write_buffer_overwrite_c11(bits_btn_result_t *result)
{
    bits_btn_ring_buffer_t *buf = &ring_buffer;

    if (result == NULL) {
        return false;
    }

    size_t current_write = atomic_load_explicit(&buf->write_idx, memory_order_relaxed);
    size_t next_write = (current_write + 1) % BITS_BTN_BUFFER_SIZE;
    size_t current_read = atomic_load_explicit(&buf->read_idx, memory_order_acquire);

    if (next_write == current_read) {
        atomic_fetch_add_explicit(&overwrite_count, 1, memory_order_relaxed);
        atomic_store_explicit(&buf->read_idx, (current_read + 1) % BITS_BTN_BUFFER_SIZE, memory_order_release);
    }

    buf->buffer[current_write] = *result;
    atomic_store_explicit(&buf->write_idx, next_write, memory_order_release);
    return true;
}

static uint8_t bits_btn_read_buffer_c11(bits_btn_result_t *result)
{
    bits_btn_ring_buffer_t *buf = &ring_buffer;
    size_t current_write = atomic_load_explicit(&buf->write_idx, memory_order_acquire);
    size_t current_read = atomic_load_explicit(&buf->read_idx, memory_order_relaxed);

    if (current_read == current_write) {
        return false;
    }

    *result = buf->buffer[current_read];
    atomic_store_explicit(&buf->read_idx, (current_read + 1) % BITS_BTN_BUFFER_SIZE, memory_order_release);
    return true;
}

static uint8_t bits_btn_peek_buffer_c11(bits_btn_result_t *result)
{
    bits_btn_ring_buffer_t *buf = &ring_buffer;
    size_t current_write = atomic_load_explicit(&buf->write_idx, memory_order_acquire);
    size_t current_read = atomic_load_explicit(&buf->read_idx, memory_order_relaxed);

    if (current_read == current_write) {
        return false;
    }

    *result = buf->buffer[current_read];
    return true;
}

static const bits_btn_buffer_ops_t c11_buffer_ops = {
    .init = bits_btn_init_buffer_c11,
    .write = bits_btn_write_buffer_overwrite_c11,
    .read = bits_btn_read_buffer_c11,
    .is_empty = bits_btn_is_buffer_empty_c11,
    .is_full = bits_btn_is_buffer_full_c11,
    .get_buffer_used_count = get_bits_btn_buffer_used_count_c11,
    .clear = bits_btn_clear_buffer_c11,
    .get_buffer_overwrite_count = get_bits_btn_buffer_overwrite_count_c11,
    .get_buffer_capacity = get_bits_btn_buffer_capacity_c11,
    .peek = bits_btn_peek_buffer_c11,
};

static const bits_btn_buffer_ops_t *bits_btn_buffer_ops = &c11_buffer_ops;

void bits_btn_register_result_filter_callback(bits_btn_result_user_filter_callback cb)
{
    bits_btn_result_user_filter_cb = cb;
}

static int _get_btn_index_by_key_id(uint16_t key_id)
{
    bits_button_t *button = &bits_btn_entity;
    for (size_t i = 0; i < button->btns_cnt; i++) {
        if (button->btns[i].key_id == key_id) {
            return (int)i;
        }
    }

    return -1;
}

static uint32_t get_button_tick(void)
{
    return bits_btn_entity.btn_tick;
}

uint8_t bits_btn_is_buffer_empty(void)
{
    return bits_btn_buffer_ops->is_empty();
}

uint8_t bits_btn_is_buffer_full(void)
{
    return bits_btn_buffer_ops->is_full();
}

size_t get_bits_btn_buffer_used_count(void)
{
    return bits_btn_buffer_ops->get_buffer_used_count();
}

static void bits_btn_clear_buffer(void)
{
    bits_btn_buffer_ops->clear();
}

size_t get_bits_btn_buffer_overwrite_count(void)
{
    return bits_btn_buffer_ops->get_buffer_overwrite_count();
}

size_t get_bits_btn_buffer_capacity(void)
{
    return bits_btn_buffer_ops->get_buffer_capacity();
}

static void sort_combo_buttons_in_init(bits_button_t *button)
{
    const uint16_t cnt = button->btns_combo_cnt;

    if (cnt <= 1) {
        return;
    }

    for (uint16_t i = 0; i < cnt; i++) {
        button->combo_sorted_indices[i] = i;
    }

    for (uint16_t i = 1; i < cnt; i++) {
        const uint16_t temp_idx = button->combo_sorted_indices[i];
        const uint8_t temp_keys = button->btns_combo[temp_idx].key_count;
        int16_t j = i - 1;

        while (j >= 0 && button->btns_combo[button->combo_sorted_indices[j]].key_count < temp_keys) {
            button->combo_sorted_indices[j + 1] = button->combo_sorted_indices[j];
            j--;
        }
        button->combo_sorted_indices[j + 1] = temp_idx;
    }
}

int32_t bits_button_init(const bits_btn_config_t *config)
{
    bits_button_t *button = &bits_btn_entity;

    if (config == NULL) {
        return BITS_BTN_ERR_INVALID_PARAM;
    }

    debug_printf = config->bits_btn_debug_printf;

    if ((config->btns == NULL)
        || (config->btns_cnt == 0)
        || (config->read_button_level_func == NULL)
        || (config->btns_combo_cnt > 0 && config->btns_combo == NULL)) {
        return BITS_BTN_ERR_INVALID_PARAM;
    }

    if (config->btns_cnt > BITS_BTN_MAX_BUTTONS) {
        return BITS_BTN_ERR_TOO_MANY_BUTTONS;
    }

    if (config->btns_combo_cnt > BITS_BTN_MAX_COMBO_BUTTONS) {
        return BITS_BTN_ERR_TOO_MANY_COMBOS;
    }

    for (uint16_t i = 0; i < config->btns_cnt; i++) {
        if (config->btns[i].param == NULL) {
            return BITS_BTN_ERR_BTN_PARAM_NULL;
        }
    }

    for (uint16_t i = 0; i < config->btns_combo_cnt; i++) {
        if (config->btns_combo[i].btn.param == NULL) {
            return BITS_BTN_ERR_COMBO_PARAM_NULL;
        }
        if (config->btns_combo[i].key_single_ids == NULL || config->btns_combo[i].key_count == 0) {
            return BITS_BTN_ERR_COMBO_KEYS_INVALID;
        }
    }

    memset(button, 0, sizeof(bits_button_t));
    button->btns = config->btns;
    button->btns_cnt = config->btns_cnt;
    button->btns_combo = config->btns_combo;
    button->btns_combo_cnt = config->btns_combo_cnt;
    button->_read_button_level = config->read_button_level_func;
    button->bits_btn_result_cb = config->bits_btn_result_cb;

    for (uint16_t i = 0; i < config->btns_combo_cnt; i++) {
        button_obj_combo_t *combo = &button->btns_combo[i];
        combo->combo_mask = 0;

        for (uint16_t j = 0; j < combo->key_count; j++) {
            int idx = _get_btn_index_by_key_id(combo->key_single_ids[j]);
            if (idx == -1) {
                return BITS_BTN_ERR_INVALID_COMBO_ID;
            }
            combo->combo_mask |= ((button_mask_type_t)1UL << idx);
        }
    }

    sort_combo_buttons_in_init(button);
    bits_btn_buffer_ops->init();
    return BITS_BTN_OK;
}

uint8_t bits_button_get_key_result(bits_btn_result_t *result)
{
    return bits_btn_buffer_ops->read(result);
}

uint8_t bits_button_peek_key_result(bits_btn_result_t *result)
{
    return bits_btn_buffer_ops->peek(result);
}

void bits_button_reset_states(void)
{
    bits_button_t *button = &bits_btn_entity;

    if (button->_read_button_level == NULL || button->btns == NULL) {
        return;
    }

    for (size_t i = 0; i < button->btns_cnt; i++) {
        button->btns[i].current_state = BTN_STATE_IDLE;
        button->btns[i].last_state = BTN_STATE_IDLE;
        button->btns[i].state_bits = 0;
        button->btns[i].state_entry_time = 0;
        button->btns[i].long_press_period_trigger_cnt = 0;
    }

    if (button->btns_combo != NULL && button->btns_combo_cnt > 0) {
        for (size_t i = 0; i < button->btns_combo_cnt; i++) {
            button_obj_combo_t *combo = &button->btns_combo[i];
            combo->btn.current_state = BTN_STATE_IDLE;
            combo->btn.last_state = BTN_STATE_IDLE;
            combo->btn.state_bits = 0;
            combo->btn.state_entry_time = 0;
            combo->btn.long_press_period_trigger_cnt = 0;
        }
    }

    button_mask_type_t current_physical_mask = 0;
    for (size_t i = 0; i < button->btns_cnt; i++) {
        uint8_t read_gpio_level = button->_read_button_level(&button->btns[i]);
        if (read_gpio_level == button->btns[i].active_level) {
            current_physical_mask |= ((button_mask_type_t)1UL << i);
        }
    }

    button->current_mask = current_physical_mask;
    button->last_mask = current_physical_mask;
    button->state_entry_time = get_button_tick();
    bits_btn_clear_buffer();
}

static void __append_bit(state_bits_type_t *state_bits, uint8_t bit)
{
    *state_bits = (*state_bits << 1) | bit;
}

static uint8_t __check_if_the_bits_match(const key_value_type_t *state_bits, key_value_type_t target, uint8_t target_bits_number)
{
    key_value_type_t mask = ((key_value_type_t)1 << target_bits_number) - 1;
    return (((*state_bits) & mask) == target) ? 1 : 0;
}

static void bits_btn_report_event(struct button_obj_t *button, bits_btn_result_t *result)
{
    bits_btn_result_callback btn_result_cb = bits_btn_entity.bits_btn_result_cb;

    if (result == NULL) {
        return;
    }

    if (debug_printf) {
        debug_printf("key id[%d],event:%d, long trigger_cnt:%d, key_value:", result->key_id, result->event, result->long_press_period_trigger_cnt);
        debug_print_binary(result->key_value);
    }

    if (bits_btn_result_user_filter_cb == NULL) {
        if (result->event == BTN_EVENT_LONG_PRESS || result->event == BTN_EVENT_FINISH) {
            bits_btn_buffer_ops->write(result);
        }
    } else if (bits_btn_result_user_filter_cb(*result)) {
        bits_btn_buffer_ops->write(result);
    }

    if (btn_result_cb) {
        btn_result_cb(button, *result);
    }
}

static void update_button_state_machine(struct button_obj_t *button, uint8_t btn_pressed)
{
    uint32_t current_time = get_button_tick();
    uint32_t time_diff = current_time - button->state_entry_time;
    bits_btn_result_t result = {0};
    result.key_id = button->key_id;

    if (button->param == NULL) {
        return;
    }

    switch (button->current_state) {
        case BTN_STATE_IDLE:
            if (btn_pressed) {
                __append_bit(&button->state_bits, 1);
                button->current_state = BTN_STATE_PRESSED;
                button->state_entry_time = current_time;
                result.key_value = button->state_bits;
                result.event = state_to_event((bits_btn_state_t)button->current_state);
                bits_btn_report_event(button, &result);
            }
            break;
        case BTN_STATE_PRESSED:
            if (time_diff * BITS_BTN_TICKS_INTERVAL > button->param->long_press_start_time_ms) {
                __append_bit(&button->state_bits, 1);
                button->current_state = BTN_STATE_LONG_PRESS;
                button->state_entry_time = current_time;
                button->long_press_period_trigger_cnt = 0;
                result.key_value = button->state_bits;
                result.event = state_to_event((bits_btn_state_t)button->current_state);
                bits_btn_report_event(button, &result);
            } else if (btn_pressed == 0) {
                button->current_state = BTN_STATE_RELEASE;
            }
            break;
        case BTN_STATE_LONG_PRESS:
            if (btn_pressed == 0) {
                button->long_press_period_trigger_cnt = 0;
                button->current_state = BTN_STATE_RELEASE;
            } else if (time_diff * BITS_BTN_TICKS_INTERVAL > button->param->long_press_period_trigger_ms) {
                button->state_entry_time = current_time;
                button->long_press_period_trigger_cnt++;

                if (__check_if_the_bits_match(&button->state_bits, 0b011, 3)) {
                    __append_bit(&button->state_bits, 1);
                }

                result.key_value = button->state_bits;
                result.event = state_to_event((bits_btn_state_t)button->current_state);
                result.long_press_period_trigger_cnt = button->long_press_period_trigger_cnt;
                bits_btn_report_event(button, &result);
            }
            break;
        case BTN_STATE_RELEASE:
            __append_bit(&button->state_bits, 0);
            result.key_value = button->state_bits;
            result.event = BTN_EVENT_RELEASE;
            bits_btn_report_event(button, &result);
            button->current_state = BTN_STATE_RELEASE_WINDOW;
            button->state_entry_time = current_time;
            break;
        case BTN_STATE_RELEASE_WINDOW:
            if (btn_pressed) {
                button->current_state = BTN_STATE_IDLE;
                button->state_entry_time = current_time;
            } else if (time_diff * BITS_BTN_TICKS_INTERVAL > button->param->time_window_time_ms) {
                button->current_state = BTN_STATE_FINISH;
            }
            break;
        case BTN_STATE_FINISH:
            result.key_value = button->state_bits;
            result.event = BTN_EVENT_FINISH;
            bits_btn_report_event(button, &result);
            button->state_bits = 0;
            button->current_state = BTN_STATE_IDLE;
            break;
        default:
            break;
    }

    if (button->last_state != button->current_state) {
        button->last_state = button->current_state;
    }
}

static void handle_button_state(struct button_obj_t *button, button_mask_type_t current_mask, button_mask_type_t btn_mask)
{
    uint8_t pressed = (current_mask & btn_mask) == btn_mask ? 1 : 0;
    update_button_state_machine(button, pressed);
}

static void dispatch_combo_buttons(bits_button_t *button, button_mask_type_t *suppression_mask)
{
    if (button->btns_combo_cnt == 0) {
        return;
    }

    button_mask_type_t activated_mask = 0;

    for (uint16_t i = 0; i < button->btns_combo_cnt; i++) {
        uint16_t combo_index = button->combo_sorted_indices[i];
        button_obj_combo_t *combo = &button->btns_combo[combo_index];
        button_mask_type_t combo_mask = combo->combo_mask;

        if (activated_mask & combo_mask) {
            continue;
        }

        handle_button_state(&combo->btn, button->current_mask, combo_mask);

        if ((button->current_mask & combo_mask) == combo_mask || combo->btn.state_bits) {
            activated_mask |= combo_mask;
            if (combo->suppress) {
                *suppression_mask |= combo_mask;
            }
        }
    }
}

static void dispatch_unsuppressed_buttons(bits_button_t *button, button_mask_type_t suppression_mask)
{
    for (size_t i = 0; i < button->btns_cnt; i++) {
        button_mask_type_t btn_mask = ((button_mask_type_t)1UL << i);
        if (suppression_mask & btn_mask) {
            continue;
        }
        handle_button_state(&button->btns[i], button->current_mask, btn_mask);
    }
}

void bits_button_ticks(void)
{
    bits_button_t *button = &bits_btn_entity;

    if (button->_read_button_level == NULL || button->btns == NULL) {
        return;
    }

    uint32_t current_time = get_button_tick();
    button->btn_tick++;

    button_mask_type_t new_mask = 0;
    for (size_t i = 0; i < button->btns_cnt; i++) {
        uint8_t read_gpio_level = button->_read_button_level(&button->btns[i]);
        if (read_gpio_level == button->btns[i].active_level) {
            new_mask |= ((button_mask_type_t)1UL << i);
        }
    }

    button->current_mask = new_mask;

    if (button->last_mask != new_mask) {
        button->state_entry_time = current_time;
        button->last_mask = new_mask;
    }

    if ((current_time - button->state_entry_time) * BITS_BTN_TICKS_INTERVAL < BITS_BTN_DEBOUNCE_TIME_MS) {
        return;
    }

    button_mask_type_t suppressed_mask = 0;
    dispatch_combo_buttons(button, &suppressed_mask);
    dispatch_unsuppressed_buttons(button, suppressed_mask);
}

static void debug_print_binary(key_value_type_t num)
{
    if (debug_printf == NULL) {
        return;
    }

    debug_printf("0b");
    int leading_zero = 1;

    for (int i = (int)(sizeof(key_value_type_t) * 8 - 1); i >= 0; i--) {
        if ((num >> i) & 1U) {
            debug_printf("1");
            leading_zero = 0;
        } else if (!leading_zero) {
            debug_printf("0");
        }
    }

    if (leading_zero) {
        debug_printf("0");
    }

    debug_printf("\r\n");
}
