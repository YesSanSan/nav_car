#include "cmsis_os2.h"
#include "main.h"
#include "stm32h7xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

#include "crc.hpp"
#include "main.h"
#include "motor_control.hpp"
#include <cstdint>


#define WHEEL_SIZE          124                                             // mm 两轮底盘的大轮子
#define SPEED2RPM           (60.0f / (3.1415926f * (WHEEL_SIZE / 1000.0f))) // m/s -> 电机轴 rpm
#define RPM2SPEED           (1.0 / SPEED2RPM)
#define ENCODER2RPM(t, ppr) (60.f * 1000000.f / ((ppr) * (t))) // 电机轴 rpm


extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef  htim12;
extern volatile uint64_t  timestamp_us; // 全局 64-bit 时间戳（微秒）

extern Motor motor1;
extern Motor motor2;

extern volatile std::atomic<float> volt;

struct [[gnu::packed]] SpeedMsg {
    uint8_t  header = 0x7E;
    uint8_t  length;
    float    v;     // 线速度
    float    omega; // 角速度
    float    volt;  // 电池电压
    uint64_t time;  // 时间戳 (us)
    uint16_t crc16;
};

uint64_t GetTimestampUs();


extern "C" void encoderSendTask(void *) {
    SpeedMsg msg;
    msg.length = sizeof(SpeedMsg);

    while (true) {
        float v1 = motor1.speed_rpm * RPM2SPEED * -1;
        float v2 = motor2.speed_rpm * RPM2SPEED;

        float v = (v1 + v2) / 2;
        float w = (v1 - v2) / 0.38;

        uint64_t t = GetTimestampUs();

        msg.v     = -v * 1.018;
        msg.omega = w * 1.018;
        msg.volt  = volt;
        msg.time  = t;

        // msg.v     = 0;
        // msg.omega = 0;
        // msg.time  = 0xabcdef8912345678;

        msg.crc16 = GetCRC16((uint8_t *)&msg, sizeof(SpeedMsg) - 2);

        HAL_UART_Transmit(&huart2, (uint8_t *)&msg, sizeof(SpeedMsg), HAL_MAX_DELAY);

        osDelay(10);
    }
}

uint64_t GetTimestampUs() {
    uint64_t base;
    uint32_t cnt;

    taskENTER_CRITICAL(); // 或 __disable_irq();

    base = timestamp_us;
    cnt  = __HAL_TIM_GET_COUNTER(&htim12);
    // 如果刚好读到CNT但溢出中断未执行
    if (__HAL_TIM_GET_FLAG(&htim12, TIM_FLAG_UPDATE) != RESET) {
        // 加上65536
        base += 65536ULL;
        cnt = __HAL_TIM_GET_COUNTER(&htim12); // CNT重新读取
    }
    taskEXIT_CRITICAL(); // 或 __enable_irq();

    return base + cnt;
}
