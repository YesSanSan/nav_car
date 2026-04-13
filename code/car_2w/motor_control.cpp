#include "motor_control.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "cmsis_os2.h"

#include "main.h"


// #define WHEEL_SIZE 70 // mm 麦轮
#define WHEEL_SIZE          124                                             // mm 两轮底盘的大轮子
#define SPEED2RPM           (60.0f / (3.1415926f * (WHEEL_SIZE / 1000.0f))) // m/s -> 电机轴 rpm
#define RPM2SPEED           (1.0 / SPEED2RPM)
#define ENCODER2RPM(t, ppr) (60.f * 1000000.f / ((ppr) * (t))) // 电机轴 rpm


extern TIM_HandleTypeDef htim1;  // PWM输出
extern TIM_HandleTypeDef htim2;  // QEI编码器
extern TIM_HandleTypeDef htim3;  // QEI编码器
extern TIM_HandleTypeDef htim4;  // QEI编码器
extern TIM_HandleTypeDef htim5;  // QEI编码器
extern TIM_HandleTypeDef htim8;  // PWM输出
extern TIM_HandleTypeDef htim12; // 微秒时间

extern std::atomic<float> speed_; // 目标速度
extern std::atomic<bool>  pause_; // 暂停标志

extern std::atomic<float> cmd_x;
extern std::atomic<float> cmd_y;
extern std::atomic<float> cmd_z;

extern volatile std::atomic<float> volt;

constexpr uint32_t control_loop_ms  = 1; // 控制周期
constexpr float    pid_output_limit = 9999;

static void Motor_SetOutput_1(float &pwm_output) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)(pwm_output > 0 ? pwm_output : 0));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint16_t)(-pwm_output > 0 ? -pwm_output : 0));
}
static void Motor_SetOutput_2(float &pwm_output) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint16_t)(pwm_output > 0 ? pwm_output : 0));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint16_t)(-pwm_output > 0 ? -pwm_output : 0));
}

float map(float value, float old_min, float old_max, float new_min, float new_max) {
    return new_min + (value - old_min) * (new_max - new_min) / (old_max - old_min);
}

void climp_pwm(float &pwm_output) {
    printf("%d  ", (uint16_t)std::fabsf(pwm_output));
    pwm_output = map(pwm_output, 0, 9999, 5000, 6995);
    printf("%d\n", (uint16_t)std::fabsf(pwm_output));
}

float apply_deadband(float out, float dead_pct) {
    if (out > -dead_pct && out < dead_pct) return 0.0f;
    return out;
}

float PidTool::caculate(float target, float current) {
    if (target == 0) {
        return 0;
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
        // jerk_step 是你允许加速度每步改变的最大值
        if (std::fabsf(delta_accel) > jerk_step) {
            current_acceleration += jerk_step * (delta_accel > 0 ? 1 : -1);
        } else {
            current_acceleration = desired_acceleration;
        }

        // 4. 限制加速度的最大值 (可选，防止加速度过大)
        if (std::fabsf(current_acceleration) > max_accel_limit) {
            current_acceleration = max_accel_limit * (current_acceleration > 0 ? 1 : -1);
        }

        // 5. 应用加速度更新目标转速
        real_target += current_acceleration;

        // 6. 最终边界检查：防止在目标值附近震荡
        if (std::fabsf(real_target - target_rpm) < 0.1f) { // 0.1f 为死区阈值
            real_target          = target_rpm;
            current_acceleration = 0; // 到达目标，加速度归零
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
        printf("volt=%.3f\n", volt.load());
    }
    if (cnt[id - 1]++ % 20 == 0) {
        printf("id%d,target%d=%d.%d,target_rpm%d=%d.%d,int%d=%d.%d,real_rpm%d=%d.%d,enc%d=%d,pwm%d=%d.%d\n",
               id, id, static_cast<int>(real_target), std::abs(static_cast<int>(real_target * 1000)) % 1000,
               id, static_cast<int>(real_target), std::abs(static_cast<int>(real_target * 1000)) % 1000,
               id, static_cast<int>(pid.integral), std::abs(static_cast<int>(pid.integral * 1000)) % 1000,
               id, static_cast<int>(speed_rpm), std::abs(static_cast<int>(speed_rpm * 1000)) % 1000,
               id, current_encoder_value,
               id, static_cast<int>(pwm_output), std::abs(static_cast<int>(pwm_output * 1000)) % 1000);
    }

    control_output_helper(pwm_output);
    // float out = 8000;
    // control_output_helper(out);
}

uint8_t Motor::id_cnt = 1;
Motor   motor1(&htim2, Motor_SetOutput_1, 500 * 2 * 30, -1);
Motor   motor2(&htim3, Motor_SetOutput_2, 500 * 2 * 30, -1);

extern "C" void motorControlTask(void *) {
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    __HAL_TIM_MOE_ENABLE(&htim1);

    float speed = 20;
    bool  pause = false;

    // 输入速度 (m/s, m/s, rad/s)
    float vx;
    float vy;
    float w;


    float w_R;
    float w_L;


    // motor1.pid.Kp = 1000;
    // motor1.pid.Ki = 10;
    // motor1.pid.Kd = 0;

    // motor2.pid.Kp = 1000;
    // motor2.pid.Ki = 10;
    // motor2.pid.Kd = 0;

    motor1.pid.Kp = 200;
    motor1.pid.Ki = 3;
    motor1.pid.Kd = 0.8;

    motor2.pid.Kp = 500;
    motor2.pid.Ki = 3;
    motor2.pid.Kd = 0.5;

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

        constexpr float L = 0.38;

        float wR_raw = vx + 0.5f * L * w;
        float wL_raw = vx - 0.5f * L * w;

        float max_val = std::max(std::abs(wR_raw), std::abs(wL_raw));
        float limit   = 5.f;

        float scale = 1.0f;
        if (max_val > limit) {
            scale = limit / max_val; // 比例缩小
        }

        w_R = wR_raw * scale;
        w_L = wL_raw * scale;

        cur_time   = __HAL_TIM_GET_COUNTER(&htim12);
        delta_time = (uint16_t)(cur_time - last_time);
        last_time  = cur_time;

        if (vx == 0 && vy == 0 && w == 0) {
            if (pause) {
                speed = 0;
            }

            motor1.update(delta_time, speed * SPEED2RPM);
            motor2.update(delta_time, -speed * SPEED2RPM);
        } else {
            motor1.update(delta_time, w_L * SPEED2RPM);
            motor2.update(delta_time, -w_R * SPEED2RPM);
        }

        osDelay(control_loop_ms);
    }
}
