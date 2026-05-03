#include "motor_control.hpp"

#include <array>
#include <atomic>
#include <cstdint>

#include "cmsis_os2.h"

#include "main.h"

extern TIM_HandleTypeDef htim1;  // PWM输出
extern TIM_HandleTypeDef htim2;  // QEI编码器
extern TIM_HandleTypeDef htim3;  // QEI编码器
extern TIM_HandleTypeDef htim12; // 微秒时间

extern std::atomic<float> speed_; // 目标速度
extern std::atomic<bool>  pause_; // 暂停标志

extern std::atomic<float> cmd_x;
extern std::atomic<float> cmd_y;
extern std::atomic<float> cmd_z;

namespace {

constexpr float wheel_size_mm = 124.0f; // 两轮底盘的大轮子
constexpr float speed_to_rpm  = motor_speed_to_rpm_factor(wheel_size_mm);
constexpr float wheel_base_m  = 0.38f;
constexpr std::array<MotorPidParams, 2> default_pid_params = {{
    {200.0f, 3.0f, 0.8f},
    {500.0f, 3.0f, 0.5f},
}};

void Motor_SetOutput_1(float &pwm_output) {
    motor_set_hbridge_output(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, pwm_output);
}

void Motor_SetOutput_2(float &pwm_output) {
    motor_set_hbridge_output(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4, pwm_output);
}

} // namespace

Motor motor1(&htim2, Motor_SetOutput_1, 500 * 2 * 30, -1);
Motor motor2(&htim3, Motor_SetOutput_2, 500 * 2 * 30, -1);
std::array<Motor *, 2> motors = {&motor1, &motor2};

std::size_t motor_count() {
    return motors.size();
}

Motor *motor_at(std::size_t index) {
    if (index >= motors.size()) {
        return nullptr;
    }

    return motors[index];
}

void motor_apply_default_pid() {
    for (std::size_t i = 0; i < default_pid_params.size(); ++i) {
        (void)motor_set_pid_params(i, default_pid_params[i]);
    }
}

extern "C" void motorControlTask(void *) {
    motor_start_encoder(&htim2);
    motor_start_encoder(&htim3);
    motor_start_hbridge_pwm(&htim1);

    float speed = 20.0f;
    bool  pause = false;

    // 输入速度 (m/s, m/s, rad/s)
    float vx = 0.0f;
    float vy = 0.0f;
    float w  = 0.0f;

    motor_apply_default_pid();
    (void)motor_load_pid_params();

    uint16_t last_time, cur_time, delta_time;
    last_time = __HAL_TIM_GET_COUNTER(&htim12) - 1;
    for (;;) {
        speed = speed_.load();
        pause = pause_.load();

        vx = cmd_x.load();
        vy = cmd_y.load();
        w  = cmd_z.load();

        const float wR_raw = vx + 0.5f * wheel_base_m * w;
        const float wL_raw = vx - 0.5f * wheel_base_m * w;
        float       wheel_speed[2] = {wL_raw, wR_raw};

        motor_limit_wheel_speeds(wheel_speed, 2, max_wheel_speed);

        cur_time   = __HAL_TIM_GET_COUNTER(&htim12);
        delta_time = static_cast<uint16_t>(cur_time - last_time);
        last_time  = cur_time;

        if (vx == 0.0f && vy == 0.0f && w == 0.0f) {
            if (pause) {
                speed = 0.0f;
            }

            motor1.update(delta_time, speed * speed_to_rpm);
            motor2.update(delta_time, -speed * speed_to_rpm);
        } else {
            motor1.update(delta_time, wheel_speed[0] * speed_to_rpm);
            motor2.update(delta_time, -wheel_speed[1] * speed_to_rpm);
        }

        osDelay(control_loop_ms);
    }
}
