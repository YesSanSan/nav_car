#include "motor_control.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

#include "adc_history.hpp"
#include "pid_tuning/pid_tuning.hpp"
#include "storage/config_store.h"
#include "storage/storage_service.h"

namespace {

constexpr float motor_rated_voltage         = 12.0f;
constexpr float max_expected_battery_voltage = 25.2f; // 6S 满电
constexpr float min_valid_battery_voltage    = 6.0f;
constexpr char  motor_pid_config_key[]       = "motor_pid";
constexpr uint32_t motor_pid_config_magic    = 0x31504944UL; // PID1
constexpr uint16_t motor_pid_config_version  = 1;
constexpr std::size_t motor_pid_config_max_count = 4;

struct MotorPidConfigStore {
    uint32_t magic;
    uint16_t version;
    uint16_t count;
    MotorPidParams params[motor_pid_config_max_count];
};

float encoder_delta_to_rpm(int32_t delta_encoder_value, int32_t delta_time, uint16_t encoder_ppr, int8_t encoder_direction) {
    return static_cast<float>(delta_encoder_value) * (60.0f * 1000000.0f / (static_cast<float>(encoder_ppr) * static_cast<float>(delta_time))) *
           static_cast<float>(encoder_direction);
}

float get_pwm_compensation_voltage() {
    const float battery_voltage = adc_get_latest_voltage();
    if (std::isfinite(battery_voltage) && battery_voltage >= min_valid_battery_voltage) {
        return battery_voltage;
    }

    return max_expected_battery_voltage;
}

float apply_battery_voltage_compensation(float pwm_output) {
    if (pwm_output == pwm_brake_output) {
        return pwm_output;
    }

    const float battery_voltage  = get_pwm_compensation_voltage();
    const float voltage_scale    = motor_rated_voltage / battery_voltage;
    const float max_pwm          = std::min(pid_output_limit, pid_output_limit * voltage_scale);
    const float compensated_pwm  = pwm_output * voltage_scale;

    return std::clamp(compensated_pwm, -max_pwm, max_pwm);
}

float apply_deadband(float out, float dead_pct) {
    if (out > -dead_pct && out < dead_pct) return 0.0f;
    return out;
}

bool pid_params_are_valid(const MotorPidParams &params) {
    return std::isfinite(params.Kp) && std::isfinite(params.Ki) && std::isfinite(params.Kd);
}

bool ensure_storage_ready() {
    return storage_is_ready() || storage_init();
}

} // namespace

void motor_set_hbridge_output(TIM_HandleTypeDef *pwm_tim_ptr, uint32_t positive_channel, uint32_t negative_channel, float &pwm_output) {
    if (pwm_output == pwm_brake_output) {
        __HAL_TIM_SET_COMPARE(pwm_tim_ptr, positive_channel, static_cast<uint16_t>(pid_output_limit));
        __HAL_TIM_SET_COMPARE(pwm_tim_ptr, negative_channel, static_cast<uint16_t>(pid_output_limit));
        return;
    }

    __HAL_TIM_SET_COMPARE(pwm_tim_ptr, positive_channel, static_cast<uint16_t>(pwm_output > 0.0f ? pwm_output : 0.0f));
    __HAL_TIM_SET_COMPARE(pwm_tim_ptr, negative_channel, static_cast<uint16_t>(-pwm_output > 0.0f ? -pwm_output : 0.0f));
}

void motor_start_encoder(TIM_HandleTypeDef *encoder_tim_ptr) {
    HAL_TIM_Encoder_Start(encoder_tim_ptr, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(encoder_tim_ptr, TIM_CHANNEL_2);
}

void motor_start_hbridge_pwm(TIM_HandleTypeDef *pwm_tim_ptr) {
    HAL_TIM_PWM_Start(pwm_tim_ptr, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(pwm_tim_ptr, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(pwm_tim_ptr, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(pwm_tim_ptr, TIM_CHANNEL_4);
    __HAL_TIM_MOE_ENABLE(pwm_tim_ptr);
}

void motor_limit_wheel_speeds(float *wheel_speeds, std::size_t wheel_count, float max_abs_speed) {
    if (wheel_speeds == nullptr || wheel_count == 0 || max_abs_speed <= 0.0f) {
        return;
    }

    float max_val = 0.0f;
    for (std::size_t i = 0; i < wheel_count; ++i) {
        max_val = std::max(max_val, std::fabsf(wheel_speeds[i]));
    }

    if (max_val <= max_abs_speed) {
        return;
    }

    const float scale = max_abs_speed / max_val;
    for (std::size_t i = 0; i < wheel_count; ++i) {
        wheel_speeds[i] *= scale;
    }
}

float PidTool::caculate(float target, float current) {
    if (target == 0) {
        integral = 0;
        last_cur = 0;
        return pwm_brake_output;
    }

    float error      = target - current;
    float derivative = current - last_cur;
    last_cur         = error;

    float output = Kp * error + integral - Kd * derivative;

    // 限制PWM输出范围
    if (output > pid_output_limit) {
        output = pid_output_limit;
    }
    if (output < -pid_output_limit) {
        output = -pid_output_limit;
    }

    if (output > -100 && output < 100) {
        integral *= 0.5;
    } else {
        integral *= 0.999;
    }

    if (integral > -pid_output_limit && integral < pid_output_limit) {
        integral += error * Ki;
    }

    if (std::fabsf(target) < 0.1f && std::fabsf(current) < 0.1f) {
        integral = 0;
        last_cur = 0;
        return 0.0f;
    }

    output = apply_deadband(output, 200);

    return output;
}

void PidTool::reset() {
    integral = 0.0f;
    last_cur = 0.0f;
}

void Motor::update(int32_t delta_time, float target_rpm) {
    {
        // 1. 计算当前时刻期望的临时加速度
        float desired_acceleration = target_rpm - real_target;

        // 2. 计算加速度的变化量 (Delta Acceleration)
        float delta_accel = desired_acceleration - current_acceleration;

        // 3. 限制加速度的变化量 (Jerk 限制)
        if (std::fabsf(delta_accel) > jerk_step) {
            current_acceleration += jerk_step * (delta_accel > 0 ? 1 : -1);
        } else {
            current_acceleration = desired_acceleration;
        }

        // 4. 限制加速度的最大值
        if (std::fabsf(current_acceleration) > max_accel_limit) {
            current_acceleration = max_accel_limit * (current_acceleration > 0 ? 1 : -1);
        }

        // 5. 应用加速度更新目标转速
        real_target += current_acceleration;

        // 6. 最终边界检查：防止在目标值附近震荡
        if (std::fabsf(real_target - target_rpm) < 0.1f) {
            real_target           = target_rpm;
            current_acceleration  = 0;
        }
    }

    // 读取当前编码器计数值
    current_encoder_value = static_cast<int16_t>(__HAL_TIM_GET_COUNTER(encoder_tim_ptr));

    // 差分计算转速
    delta_encoder_value = current_encoder_value - previous_encoder_value;

    // 考虑溢出回绕情况
    if (delta_encoder_value > 32767)
        delta_encoder_value -= 65536;
    else if (delta_encoder_value < -32768)
        delta_encoder_value += 65536;

    previous_encoder_value = current_encoder_value;

    speed_rpm = encoder_delta_to_rpm(delta_encoder_value, delta_time, encoder_ppr, encoder_direction);

    speed_filter_buf[speed_buf_idx++] = speed_rpm;
    if (speed_buf_idx >= speed_buf_size) speed_buf_idx = 0;

    float total_speed = 0;
    for (uint8_t i = 0; i < speed_buf_size; i++) {
        total_speed += speed_filter_buf[i];
    }
    speed_rpm = total_speed / speed_buf_size;

    float pwm_output = pid.caculate(real_target, speed_rpm);

    static std::array<uint8_t, 5> cnt = {0, 1, 2, 3};
    if (id == 1 && cnt[id - 1] % 20 == 0) {
        // printf("volt=%.3f\n", adc_get_latest_voltage());
    }
    if (cnt[id - 1]++ % 20 == 0) {
        // printf("id%d,target%d=%d.%d,target_rpm%d=%d.%d,int%d=%d.%d,real_rpm%d=%d.%d,enc%d=%d,pwm%d=%d.%d\n",
        //        id, id, static_cast<int>(real_target), std::abs(static_cast<int>(real_target * 1000)) % 1000,
        //        id, static_cast<int>(real_target), std::abs(static_cast<int>(real_target * 1000)) % 1000,
        //        id, static_cast<int>(pid.integral), std::abs(static_cast<int>(pid.integral * 1000)) % 1000,
        //        id, static_cast<int>(speed_rpm), std::abs(static_cast<int>(speed_rpm * 1000)) % 1000,
        //        id, current_encoder_value,
        //        id, static_cast<int>(pwm_output), std::abs(static_cast<int>(pwm_output * 1000)) % 1000);
    }

    float compensated_pwm_output = apply_battery_voltage_compensation(pwm_output);
    const PidTuningMotorRuntimeSample tuning_sample = {
        .motor_index = static_cast<uint8_t>(id - 1U),
        .motor_count = static_cast<uint8_t>(motor_count()),
        .reserved = 0,
        .sequence = 0,
        .time_us = 0,
        .target_rpm = target_rpm,
        .real_target_rpm = real_target,
        .speed_rpm = speed_rpm,
        .pid_integral = pid.integral,
        .pwm_output = pwm_output,
        .compensated_pwm_output = compensated_pwm_output,
        .battery_voltage = adc_get_latest_voltage(),
        .encoder_value = current_encoder_value,
    };
    pid_tuning_update_motor_sample(static_cast<uint8_t>(id - 1U), tuning_sample);
    control_output_helper(compensated_pwm_output);
}

uint8_t Motor::id_cnt = 1;

bool motor_get_pid_params(std::size_t index, MotorPidParams *params) {
    Motor *motor = motor_at(index);
    if (motor == nullptr || params == nullptr) {
        return false;
    }

    params->Kp = motor->pid.Kp;
    params->Ki = motor->pid.Ki;
    params->Kd = motor->pid.Kd;
    return true;
}

bool motor_set_pid_params(std::size_t index, const MotorPidParams &params) {
    Motor *motor = motor_at(index);
    if (motor == nullptr || !pid_params_are_valid(params)) {
        return false;
    }

    motor->pid.Kp = params.Kp;
    motor->pid.Ki = params.Ki;
    motor->pid.Kd = params.Kd;
    motor->pid.reset();
    return true;
}

bool motor_load_pid_params() {
    MotorPidConfigStore store = {};
    size_t actual_len = 0;

    if (!ensure_storage_ready()) {
        return false;
    }

    if (!config_get_blob(motor_pid_config_key, &store, sizeof(store), &actual_len) || actual_len != sizeof(store)) {
        return false;
    }

    if (store.magic != motor_pid_config_magic || store.version != motor_pid_config_version || store.count == 0 ||
        store.count > motor_pid_config_max_count) {
        return false;
    }

    const std::size_t count = std::min<std::size_t>(store.count, motor_count());
    for (std::size_t i = 0; i < count; ++i) {
        if (!motor_set_pid_params(i, store.params[i])) {
            return false;
        }
    }

    return true;
}

bool motor_save_pid_params() {
    MotorPidConfigStore store = {};
    const std::size_t count = std::min<std::size_t>(motor_count(), motor_pid_config_max_count);

    if (count == 0 || !ensure_storage_ready()) {
        return false;
    }

    store.magic = motor_pid_config_magic;
    store.version = motor_pid_config_version;
    store.count = static_cast<uint16_t>(count);

    for (std::size_t i = 0; i < count; ++i) {
        if (!motor_get_pid_params(i, &store.params[i])) {
            return false;
        }
    }

    return config_set_blob(motor_pid_config_key, &store, sizeof(store)) && config_commit();
}
