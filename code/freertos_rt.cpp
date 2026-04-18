#include "main.h"

#include "FreeRTOS.h"
#include "task.h"


extern volatile uint32_t runtime_base32;
extern TIM_HandleTypeDef htim12;

/**
 * @brief  初始化用于统计运行时间的定时器
 * @note   该函数由 FreeRTOS 内部在启动调度器时自动调用
 */
extern "C" void configureTimerForRunTimeStats(void) {
    // 启动 TIM1
    // HAL_TIM_Base_Start(&htim1);
    // HAL_TIM_Base_Start_IT(&htim1); // 开启中断模式启动
}

/**
 * @brief  获取当前计数器的值
 * @return 返回当前的高频计数值
 */
extern "C" unsigned long getRunTimeCounterValue(void) {
    uint64_t base;
    uint32_t cnt;

    UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
    
    base = runtime_base32;
    cnt  = __HAL_TIM_GET_COUNTER(&htim12);
    // 如果刚好读到CNT但溢出中断未执行
    if (__HAL_TIM_GET_FLAG(&htim12, TIM_FLAG_UPDATE) != RESET) {
        base += 65536ULL;
        cnt = __HAL_TIM_GET_COUNTER(&htim12);
    }
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);

    return base + cnt;
}
