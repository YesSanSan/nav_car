#ifndef __BITS_BUTTON_H__
#define __BITS_BUTTON_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>

#ifndef BITS_BTN_MAX_COMBO_BUTTONS
#define BITS_BTN_MAX_COMBO_BUTTONS  8
#endif

typedef uint32_t key_value_type_t;
typedef uint32_t state_bits_type_t;
typedef state_bits_type_t button_mask_type_t;

#define BITS_BTN_MAX_BUTTONS      (sizeof(button_mask_type_t) * 8)

typedef enum {
    BITS_BTN_OK                       =  0,
    BITS_BTN_ERR_INVALID_COMBO_ID     = -1,
    BITS_BTN_ERR_INVALID_PARAM        = -2,
    BITS_BTN_ERR_TOO_MANY_COMBOS      = -3,
    BITS_BTN_ERR_BUFFER_OPS_NULL      = -4,
    BITS_BTN_ERR_TOO_MANY_BUTTONS     = -5,
    BITS_BTN_ERR_BTN_PARAM_NULL       = -6,
    BITS_BTN_ERR_COMBO_PARAM_NULL     = -7,
    BITS_BTN_ERR_COMBO_KEYS_INVALID   = -8,
} bits_btn_error_t;

typedef enum {
    BTN_EVENT_PRESSED    = 1,
    BTN_EVENT_LONG_PRESS = 2,
    BTN_EVENT_RELEASE    = 3,
    BTN_EVENT_FINISH     = 5,
} bits_btn_event_t;

#ifndef BITS_BTN_TICKS_INTERVAL
#define BITS_BTN_TICKS_INTERVAL              5
#endif

#ifndef BITS_BTN_DEBOUNCE_TIME_MS
#define BITS_BTN_DEBOUNCE_TIME_MS             (40)
#endif

#define BITS_BTN_LONG_PRESS_START_TIME_MS     (1000)
#define BITS_BTN_LONG_PRESS_PERIOD_TRIGGER_MS (1000)
#define BITS_BTN_TIME_WINDOW_TIME_MS          (300)

#define BITS_BTN_NONE_PRESS_KV              0
#define BITS_BTN_SINGLE_CLICK_KV            0b010
#define BITS_BTN_DOUBLE_CLICK_KV            0b01010

#define BITS_BTN_SINGLE_CLICK_THEN_LONG_PRESS_KV     0b01011
#define BITS_BTN_DOUBLE_CLICK_THEN_LONG_PRESS_KV     0b0101011

#define BITS_BTN_LONG_PRESS_START_KV       0b011
#define BITS_BTN_LONG_PRESS_HOLD_KV        0b0111
#define BITS_BTN_LONG_PRESS_HOLD_END_KV    0b01110

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#endif

#ifndef true
#define true 1
#endif

#ifndef false
#define false 0
#endif

#define BITS_BUTTON_INIT(_key_id, _active_level, _param)                                    \
{                                                                                           \
    .active_level = _active_level, .current_state = 0, .last_state = 0, .key_id = _key_id, \
    .long_press_period_trigger_cnt = 0, .state_entry_time = 0,                              \
    .state_bits = 0, .param = _param                                                        \
}

#define BITS_BUTTON_COMBO_INIT(_key_id, _active_level, _param, _key_single_ids, _key_count, _single_key_suppress) \
{                                                                                                                    \
    .suppress = _single_key_suppress, .key_count = _key_count, .key_single_ids = _key_single_ids, .combo_mask = 0, \
    .btn = BITS_BUTTON_INIT(_key_id, _active_level, _param)                                                          \
}

typedef struct bits_btn_result
{
    uint8_t event;
    uint16_t key_id;
    uint16_t long_press_period_trigger_cnt;
    state_bits_type_t key_value;
} bits_btn_result_t;

typedef struct bits_btn_obj_param
{
    uint16_t long_press_start_time_ms;
    uint16_t long_press_period_trigger_ms;
    uint16_t time_window_time_ms;
} bits_btn_obj_param_t;

typedef struct button_obj_t {
    uint8_t active_level : 1;
    uint8_t current_state : 3;
    uint8_t last_state : 3;
    uint16_t key_id;
    uint16_t long_press_period_trigger_cnt;
    uint32_t state_entry_time;
    state_bits_type_t state_bits;
    const bits_btn_obj_param_t *param;
} button_obj_t;

typedef uint8_t (*bits_btn_read_button_level)(struct button_obj_t *btn);
typedef void (*bits_btn_result_callback)(struct button_obj_t *btn, struct bits_btn_result button_result);
typedef int (*bits_btn_debug_printf_func)(const char*, ...);
typedef uint8_t (*bits_btn_result_user_filter_callback)(bits_btn_result_t button_result);

typedef struct button_obj_combo
{
    uint8_t suppress;
    uint8_t key_count;
    uint16_t *key_single_ids;
    button_mask_type_t combo_mask;

    button_obj_t btn;
} button_obj_combo_t;

typedef struct bits_button
{
    button_obj_t *btns;
    uint16_t btns_cnt;
    button_obj_combo_t *btns_combo;
    uint16_t btns_combo_cnt;

    button_mask_type_t current_mask;
    button_mask_type_t last_mask;
    uint32_t state_entry_time;
    uint32_t btn_tick;
    bits_btn_read_button_level _read_button_level;
    bits_btn_result_callback bits_btn_result_cb;

    uint16_t combo_sorted_indices[BITS_BTN_MAX_COMBO_BUTTONS];
} bits_button_t;

typedef struct
{
    void (*init)(void);
    uint8_t (*write)(bits_btn_result_t *result);
    uint8_t (*read)(bits_btn_result_t *result);
    uint8_t (*is_empty)(void);
    uint8_t (*is_full)(void);
    size_t (*get_buffer_used_count)(void);
    void (*clear)(void);
    size_t (*get_buffer_overwrite_count)(void);
    size_t (*get_buffer_capacity)(void);
    uint8_t (*peek)(bits_btn_result_t *result);
} bits_btn_buffer_ops_t;

typedef struct
{
    button_obj_t *btns;
    uint16_t btns_cnt;
    button_obj_combo_t *btns_combo;
    uint16_t btns_combo_cnt;
    bits_btn_read_button_level read_button_level_func;
    bits_btn_result_callback bits_btn_result_cb;
    bits_btn_debug_printf_func bits_btn_debug_printf;
} bits_btn_config_t;

int32_t bits_button_init(const bits_btn_config_t *config);
void bits_button_ticks(void);
uint8_t bits_button_get_key_result(bits_btn_result_t *result);
uint8_t bits_button_peek_key_result(bits_btn_result_t *result);
void bits_button_reset_states(void);
size_t get_bits_btn_buffer_overwrite_count(void);
size_t get_bits_btn_buffer_used_count(void);
uint8_t bits_btn_is_buffer_full(void);
uint8_t bits_btn_is_buffer_empty(void);

#ifdef BITS_BTN_USE_USER_BUFFER
void bits_button_set_buffer_ops(const bits_btn_buffer_ops_t *user_buffer_ops);
#endif

size_t get_bits_btn_buffer_capacity(void);
void bits_btn_register_result_filter_callback(bits_btn_result_user_filter_callback cb);

#ifdef __cplusplus
}
#endif

#endif
