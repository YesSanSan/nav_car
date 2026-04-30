#include "motor_control.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "cmsis_os2.h"

#include "adc_history.hpp"
#include "main.h"


// #define WHEEL_SIZE          124                                             // mm 两轮底盘的大轮子
#define WHEEL_SIZE          70                                              // mm 麦轮
#define SPEED2RPM           (60.0f / (3.1415926f * (WHEEL_SIZE / 1000.0f))) // m/s -> 电机轴 rpm
#define RPM2SPEED           (1.0 / SPEED2RPM)
#define ENCODER2RPM(t, ppr) (60.f * 1000000.f / ((ppr) * (t))) // 电机轴 rpm


extern TIM_HandleTypeDef htim1; // PWM输出
extern TIM_HandleTypeDef htim2; // QEI编码器
extern TIM_HandleTypeDef htim3; // QEI编码器
extern TIM_HandleTypeDef htim4; // QEI编码器
extern TIM_HandleTypeDef htim5; // QEI编码器
extern TIM_HandleTypeDef htim8; // PWM输出
extern TIM_HandleTypeDef htim12; // 微秒时间

extern std::atomic<float> speed_; // 目标速度
extern std::atomic<bool>  pause_; // 暂停标志

extern std::atomic<float> cmd_x;
extern std::atomic<float> cmd_y;
extern std::atomic<float> cmd_z;


constexpr uint32_t control_loop_ms  = 1; // 控制周期
constexpr float    pid_output_limit = 10000;
constexpr float    pwm_brake_output = pid_output_limit + 1.0f;
constexpr float    motor_rated_voltage = 12.0f;
constexpr float    max_expected_battery_voltage = 25.2f; // 6S 满电
constexpr float    min_valid_battery_voltage = 6.0f;
constexpr float    max_wheel_speed = 5.0f; // m/s

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

    const float battery_voltage = get_pwm_compensation_voltage();
    const float voltage_scale = motor_rated_voltage / battery_voltage;
    const float max_pwm = std::min(pid_output_limit, pid_output_limit * voltage_scale);
    const float compensated_pwm = pwm_output * voltage_scale;

    return std::clamp(compensated_pwm, -max_pwm, max_pwm);
}

static void Motor_SetOutput_1(float &pwm_output) {
    if (pwm_output == pwm_brake_output) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)pid_output_limit);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint16_t)pid_output_limit);
        return;
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)(pwm_output > 0 ? pwm_output : 0));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint16_t)(-pwm_output > 0 ? -pwm_output : 0));
}
static void Motor_SetOutput_2(float &pwm_output) {
    if (pwm_output == pwm_brake_output) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint16_t)pid_output_limit);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint16_t)pid_output_limit);
        return;
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint16_t)(pwm_output > 0 ? pwm_output : 0));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint16_t)(-pwm_output > 0 ? -pwm_output : 0));
}
static void Motor_SetOutput_3(float &pwm_output) {
    if (pwm_output == pwm_brake_output) {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, (uint16_t)pid_output_limit);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, (uint16_t)pid_output_limit);
        return;
    }
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, (uint16_t)(pwm_output > 0 ? pwm_output : 0));
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, (uint16_t)(-pwm_output > 0 ? -pwm_output : 0));
}
static void Motor_SetOutput_4(float &pwm_output) {
    if (pwm_output == pwm_brake_output) {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (uint16_t)pid_output_limit);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, (uint16_t)pid_output_limit);
        return;
    }
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (uint16_t)(pwm_output > 0 ? pwm_output : 0));
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, (uint16_t)(-pwm_output > 0 ? -pwm_output : 0));
}

float apply_deadband(float out, float dead_pct) {
    if (out > -dead_pct && out < dead_pct) return 0.0f;
    return out;
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

extern UART_HandleTypeDef huart2;

void Motor::update(int32_t delta_time, float target_rpm) {
    // __HAL_TIM_SET_COMPARE(&htim1, pwm_channel, (uint16_t)std::fabsf(map(target_rpm, 0, 1, 5000, 7000 - 10)));
    // return;

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
            real_target = target_rpm;
            current_acceleration = 0;
        }
    }

    // 读取当前编码器计数值
    current_encoder_value = (int16_t)__HAL_TIM_GET_COUNTER(encoder_tim_ptr); // 强制转换为有符号值

    // 差分计算转速
    delta_encoder_value = current_encoder_value - previous_encoder_value;

    // 考虑溢出回绕情况
    if (delta_encoder_value > 32767)
        delta_encoder_value -= 65536;
    else if (delta_encoder_value < -32768)
        delta_encoder_value += 65536;

    previous_encoder_value = current_encoder_value;

    speed_rpm = (float)delta_encoder_value * ENCODER2RPM(delta_time, encoder_ppr) * encoder_direction;

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
    control_output_helper(compensated_pwm_output);
    // float out = 8000;
    // control_output_helper(out);
}

uint8_t Motor::id_cnt = 1;
Motor   motor1(&htim2, Motor_SetOutput_1, 13 * 2 * 30, -1);
Motor   motor2(&htim3, Motor_SetOutput_2, 13 * 2 * 30, -1);
Motor   motor3(&htim4, Motor_SetOutput_3, 13 * 2 * 30, -1);
Motor   motor4(&htim5, Motor_SetOutput_4, 13 * 2 * 30, -1);

constexpr float motor1_output_direction = 1.0f;  // 左前
constexpr float motor2_output_direction = -1.0f; // 右前
constexpr float motor3_output_direction = 1.0f; // 左后
constexpr float motor4_output_direction = -1.0f;  // 右后

void update_mecanum_wheels(uint16_t delta_time, float vx, float vy, float w) {
    // L/W 为车体中心到前后/左右轮中心的距离，单位 m。
    constexpr float L = 0.209f;
    constexpr float W = 0.252f;
    constexpr float k = L + W;

    // 轮序：1 左前，2 右前，3 左后，4 右后。
    std::array<float, 4> wheel_speed = {
        vx - vy - k * w,
        vx + vy + k * w,
        vx + vy - k * w,
        vx - vy + k * w,
    };

    float max_val = 0.0f;
    for (float speed : wheel_speed) {
        max_val = std::max(max_val, std::fabsf(speed));
    }

    float scale = 1.0f;
    if (max_val > max_wheel_speed) {
        scale = max_wheel_speed / max_val;
    }

    motor1.update(delta_time, wheel_speed[0] * scale * motor1_output_direction * SPEED2RPM);
    motor2.update(delta_time, wheel_speed[1] * scale * motor2_output_direction * SPEED2RPM);
    motor3.update(delta_time, wheel_speed[2] * scale * motor3_output_direction * SPEED2RPM);
    motor4.update(delta_time, wheel_speed[3] * scale * motor4_output_direction * SPEED2RPM);
}

extern "C" void motorControlTask(void *) {
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    __HAL_TIM_MOE_ENABLE(&htim1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
    __HAL_TIM_MOE_ENABLE(&htim8);

    float speed = 20;
    bool  pause = false;

    // 输入速度 (m/s, m/s, rad/s)
    float vx;
    float vy;
    float w;

    motor1.pid.Kp = 1.2f;
    motor1.pid.Ki = 0.15f;
    motor1.pid.Kd = 0.0f;

    motor2.pid.Kp = 1.2f;
    motor2.pid.Ki = 0.15f;
    motor2.pid.Kd = 0.0f;

    motor3.pid.Kp = 1.2f;
    motor3.pid.Ki = 0.15f;
    motor3.pid.Kd = 3.0f;

    motor4.pid.Kp = 1.2f;
    motor4.pid.Ki = 0.12f;
    motor4.pid.Kd = 0.5f;

    uint16_t last_time, cur_time, delta_time;
    last_time = __HAL_TIM_GET_COUNTER(&htim12) - 1;
    for (;;) {
        {
            speed = speed_.load();
            pause = pause_.load();
        }

        {
            vx = cmd_x.load();
            vy = cmd_y.load();
            w  = cmd_z.load();
        }

        cur_time   = __HAL_TIM_GET_COUNTER(&htim12);
        delta_time = (uint16_t)(cur_time - last_time);
        last_time  = cur_time;

        if (vx == 0 && vy == 0 && w == 0) {
            if (pause) {
                speed = 0;
            }

            update_mecanum_wheels(delta_time, speed, 0.0f, 0.0f);
        } else {
            update_mecanum_wheels(delta_time, vx, vy, w);
        }

        osDelay(control_loop_ms);
    }
}
