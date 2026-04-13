#include <atomic>
#include <cstdint>

#include "stm32h7xx_hal.h"

using ControlOuputHelper = void (*)(float &);

class PidTool {
public:
    float caculate(float target, float current);

    float Kp = 1.2f;
    float Ki = 0.15f;
    float Kd = 0.0f;

    float integral = 0;

private:
    float last_cur = 0;
};

class Motor {
public:
    explicit Motor(TIM_HandleTypeDef *encoder_tim_ptr, ControlOuputHelper control_output_helper, uint16_t encoder_ppr, int8_t encoder_direction)
        : encoder_ppr(encoder_ppr), encoder_direction(encoder_direction),
          id(id_cnt++), encoder_tim_ptr(encoder_tim_ptr), control_output_helper(control_output_helper) {}
    void update(int32_t delta_time, float target_speed);

    PidTool pid;

    static uint8_t id_cnt;

    uint8_t  gear_ratio;
    uint16_t encoder_ppr;
    int8_t   encoder_direction;

    float speed_rpm;

private:
    uint8_t id;

    TIM_HandleTypeDef *encoder_tim_ptr;
    ControlOuputHelper control_output_helper;

    int16_t current_encoder_value;
    int16_t previous_encoder_value = 0;
    int32_t delta_encoder_value;

    static const uint8_t speed_buf_size = 8;

    uint8_t speed_buf_idx = 0;
    float   speed_filter_buf[speed_buf_size];

    float speed_filt = 0;

    float real_target          = 0;
    float target_step          = 100;
    float current_acceleration = 0;
    float jerk_step            = 50;
    float max_accel_limit      = 100;
};
