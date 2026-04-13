#include "motor_control.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "cmsis_os2.h"

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
extern TIM_HandleTypeDef htim6; // 微秒时间
extern TIM_HandleTypeDef htim8; // PWM输出

extern std::atomic<float> speed_; // 目标速度
extern std::atomic<bool>  pause_; // 暂停标志

extern std::atomic<float> cmd_x;
extern std::atomic<float> cmd_y;
extern std::atomic<float> cmd_z;


constexpr uint32_t control_loop_ms  = 1; // 控制周期
constexpr float    pid_output_limit = 11999;

static void Motor_SetOutput_1(float &pwm_output) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)(pwm_output > 0 ? pwm_output : 0));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint16_t)(-pwm_output > 0 ? -pwm_output : 0));
}
static void Motor_SetOutput_2(float &pwm_output) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint16_t)(pwm_output > 0 ? pwm_output : 0));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint16_t)(-pwm_output > 0 ? -pwm_output : 0));
}
static void Motor_SetOutput_3(float &pwm_output) {
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, (uint16_t)(pwm_output > 0 ? pwm_output : 0));
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, (uint16_t)(-pwm_output > 0 ? -pwm_output : 0));
}
static void Motor_SetOutput_4(float &pwm_output) {
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (uint16_t)(pwm_output > 0 ? pwm_output : 0));
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, (uint16_t)(-pwm_output > 0 ? -pwm_output : 0));
}

float apply_deadband(float out, float dead_pct) {
    if (out > -dead_pct && out < dead_pct) return 0.0f;
    return out;
}

float PidTool::caculate(float target, float current) {
    float error      = target - current;
    float derivative = error - last_error;
    last_error       = error;

    float output = Kp * error + integral + Kd * derivative;

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

    output = apply_deadband(output, 200);

    return output;
}

extern UART_HandleTypeDef huart2;

void Motor::update(int32_t delta_time, float target_rpm) {
    // __HAL_TIM_SET_COMPARE(&htim1, pwm_channel, (uint16_t)std::fabsf(map(target_rpm, 0, 1, 5000, 7000 - 10)));
    // return;

    {
        float delta_target = target_rpm - real_target;
        if (std::fabsf(delta_target) > target_step) {
            real_target += target_step * (delta_target > 0 ? 1 : -1);
        } else {
            real_target = target_rpm;
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

    static std::array<uint8_t, 4> cnt = {0, 1, 0, 0};
    if (cnt[id]++ % 2 == 0) {
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
Motor   motor1(&htim2, Motor_SetOutput_1, 13 * 4, 1);
Motor   motor2(&htim3, Motor_SetOutput_2, 13 * 4, 1);
Motor   motor3(&htim4, Motor_SetOutput_3, 13 * 4, 1);
Motor   motor4(&htim5, Motor_SetOutput_4, 13 * 4, 1);

extern "C" void motorControlTask(void *) {

    float speed = 20;
    bool  pause = false;

    // 输入速度 (m/s, m/s, rad/s)
    float vx;
    float vy;
    float w;

    float w1, w2, w3, w4;

    motor3.pid.Kp = 1.2;
    motor3.pid.Ki = 0.15;
    motor3.pid.Kd = 3;

    motor4.pid.Kp = 1.2;
    motor4.pid.Ki = 0.12;
    motor4.pid.Kd = 0.5;

    uint16_t last_time, cur_time, delta_time;
    last_time = __HAL_TIM_GET_COUNTER(&htim6) - 1;
    for (;;) {
        {
            speed = speed_.load();
            pause = pause_.load();
        }

        {
            vx = -cmd_x.load();
            vy = cmd_y.load();
            w  = -cmd_z.load();
        }

        // 20.9cm
        // 25.2cm
        // 8cm
        // L：前后轮中心间距的一半（车体中心到前后轮的距离）
        // W：左右轮中心间距的一半（车体中心到左右轮的距离）
        // r：轮子半径
        constexpr float L = 0.209;
        constexpr float W = 0.252;
        constexpr float k = (L + W);
        // constexpr float r = 0.08;

        w1 = (vx - vy - k * w) /*/ r*/;
        w2 = -(vx + vy + k * w) /*/ r*/;
        w3 = -(vx - vy + k * w) /*/ r*/;
        w4 = (vx + vy - k * w) /*/ r*/;

        cur_time   = __HAL_TIM_GET_COUNTER(&htim6);
        delta_time = (uint16_t)(cur_time - last_time);
        last_time  = cur_time;

        if (vx == 0 && vy == 0 && w == 0) {
            if (pause) {
                speed = 0;
            }

            motor1.update(delta_time, speed);
            motor2.update(delta_time, speed);
            motor3.update(delta_time, speed);
            motor4.update(delta_time, speed);
        } else {
            motor1.update(delta_time, w1);
            motor2.update(delta_time, w2);
            motor3.update(delta_time, w3);
            motor4.update(delta_time, w4);
        }

        osDelay(control_loop_ms);
    }
}
