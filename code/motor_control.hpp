#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "stm32h7xx_hal.h"

using ControlOuputHelper = void (*)(float &);

constexpr uint32_t control_loop_ms  = 1; // 控制周期
constexpr float    pid_output_limit = 10000.0f;
constexpr float    pwm_brake_output = pid_output_limit + 1.0f;
constexpr float    max_wheel_speed  = 5.0f; // m/s

constexpr float motor_speed_to_rpm_factor(float wheel_size_mm) {
    return 60.0f / (3.1415926f * (wheel_size_mm / 1000.0f));
}

void motor_set_hbridge_output(TIM_HandleTypeDef *pwm_tim_ptr, uint32_t positive_channel, uint32_t negative_channel, float &pwm_output);
void motor_start_encoder(TIM_HandleTypeDef *encoder_tim_ptr);
void motor_start_hbridge_pwm(TIM_HandleTypeDef *pwm_tim_ptr);
void motor_limit_wheel_speeds(float *wheel_speeds, std::size_t wheel_count, float max_abs_speed);

struct MotorPidParams {
    float Kp;
    float Ki;
    float Kd;
};

class PidTool {
public:
    float caculate(float target, float current);
    void  reset();

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

std::size_t motor_count();
Motor      *motor_at(std::size_t index);
void        motor_apply_default_pid();

bool motor_get_pid_params(std::size_t index, MotorPidParams *params);
bool motor_set_pid_params(std::size_t index, const MotorPidParams &params);
bool motor_load_pid_params();
bool motor_save_pid_params();
