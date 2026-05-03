#include "motor_control.hpp"

#include <array>
#include <atomic>
#include <cstdint>

#include "cmsis_os2.h"

#include "main.h"

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

namespace {

constexpr float wheel_size_mm = 70.0f; // 麦轮
constexpr float speed_to_rpm  = motor_speed_to_rpm_factor(wheel_size_mm);
constexpr std::array<MotorPidParams, 4> default_pid_params = {{
    {1.2f, 0.15f, 0.0f},
    {1.2f, 0.15f, 0.0f},
    {1.2f, 0.15f, 3.0f},
    {1.2f, 0.12f, 0.5f},
}};

void Motor_SetOutput_1(float &pwm_output) {
    motor_set_hbridge_output(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, pwm_output);
}

void Motor_SetOutput_2(float &pwm_output) {
    motor_set_hbridge_output(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4, pwm_output);
}

void Motor_SetOutput_3(float &pwm_output) {
    motor_set_hbridge_output(&htim8, TIM_CHANNEL_1, TIM_CHANNEL_2, pwm_output);
}

void Motor_SetOutput_4(float &pwm_output) {
    motor_set_hbridge_output(&htim8, TIM_CHANNEL_3, TIM_CHANNEL_4, pwm_output);
}

constexpr float motor1_output_direction = 1.0f;  // 左前
constexpr float motor2_output_direction = -1.0f; // 右前
constexpr float motor3_output_direction = 1.0f;  // 左后
constexpr float motor4_output_direction = -1.0f; // 右后

} // namespace

Motor motor1(&htim2, Motor_SetOutput_1, 13 * 2 * 30, -1);
Motor motor2(&htim3, Motor_SetOutput_2, 13 * 2 * 30, -1);
Motor motor3(&htim4, Motor_SetOutput_3, 13 * 2 * 30, -1);
Motor motor4(&htim5, Motor_SetOutput_4, 13 * 2 * 30, -1);
std::array<Motor *, 4> motors = {&motor1, &motor2, &motor3, &motor4};

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

static void update_mecanum_wheels(uint16_t delta_time, float vx, float vy, float w) {
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

    motor_limit_wheel_speeds(wheel_speed.data(), wheel_speed.size(), max_wheel_speed);

    motor1.update(delta_time, wheel_speed[0] * motor1_output_direction * speed_to_rpm);
    motor2.update(delta_time, wheel_speed[1] * motor2_output_direction * speed_to_rpm);
    motor3.update(delta_time, wheel_speed[2] * motor3_output_direction * speed_to_rpm);
    motor4.update(delta_time, wheel_speed[3] * motor4_output_direction * speed_to_rpm);
}

extern "C" void motorControlTask(void *) {
    motor_start_encoder(&htim2);
    motor_start_encoder(&htim3);
    motor_start_encoder(&htim4);
    motor_start_encoder(&htim5);
    motor_start_hbridge_pwm(&htim1);
    motor_start_hbridge_pwm(&htim8);

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

        cur_time   = __HAL_TIM_GET_COUNTER(&htim12);
        delta_time = static_cast<uint16_t>(cur_time - last_time);
        last_time  = cur_time;

        if (vx == 0.0f && vy == 0.0f && w == 0.0f) {
            if (pause) {
                speed = 0.0f;
            }

            update_mecanum_wheels(delta_time, speed, 0.0f, 0.0f);
        } else {
            update_mecanum_wheels(delta_time, vx, vy, w);
        }

        osDelay(control_loop_ms);
    }
}
