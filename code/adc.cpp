#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include <FreeRTOS.h>
#include <stdint.h>
#include <stm32h7xx_hal.h>

#include <cmsis_os2.h>
#include <semphr.h>
#include <task.h>

#include "ring_bufffer.hpp"

extern ADC_HandleTypeDef hadc1;
extern TaskHandle_t      adcHandle;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim12;

volatile std::atomic<float> volt = 0;

namespace {

constexpr size_t kVoltageHistorySize = 160;

std::array<float, kVoltageHistorySize> voltage_history{};
std::atomic<size_t>                    voltage_history_head{0};

float   history_accumulator = 0.0f;
uint8_t history_accumulator_count = 0;

void push_voltage_sample(float voltage)
{
    history_accumulator += voltage;
    history_accumulator_count++;

    if (history_accumulator_count < 4) {
        return;
    }

    const float averaged_voltage = history_accumulator / static_cast<float>(history_accumulator_count);
    const size_t head = voltage_history_head.load(std::memory_order_relaxed);

    voltage_history[head % kVoltageHistorySize] = averaged_voltage;
    voltage_history_head.store(head + 1, std::memory_order_release);

    history_accumulator = 0.0f;
    history_accumulator_count = 0;
}

} // namespace

// 硬件配置常量
constexpr uint32_t ADC_BUF_SIZE = 64; // DMA 双缓冲大小 (Half + Full)

// DMA 专用缓冲区，后续可配合 MPU 单独设为 non-cacheable。
alignas(32)
    __attribute__((section(".dma_buffer")))
    uint16_t adc_dma_raw[ADC_BUF_SIZE];

RingBuffer<uint16_t, 512> app_buffer;
SemaphoreHandle_t         adc_sem = nullptr;

/* 
 * ADC 电压转换函数 (线性校准)
 * 方程: Voltage = RawADC * 0.00055413 + (-0.02217556)
 */
float getCalibratedVoltage(int rawAdc) {
    // 建议对 rawAdc 进行多次采样取平均后再计算
    // const float slope = 0.00055413f;
    // const float offset = -0.02217556f;
    const float slope = 0.00055208f;
    const float offset = 0.01633f;
    
    float voltage = ((float)rawAdc * slope) + offset;
    
    return voltage;
}

float adc_get_latest_voltage()
{
    return volt.load(std::memory_order_relaxed);
}

size_t adc_copy_voltage_history(float *buffer, size_t capacity)
{
    if (buffer == nullptr || capacity == 0) {
        return 0;
    }

    const size_t total_pushed = voltage_history_head.load(std::memory_order_acquire);
    const size_t available = std::min(total_pushed, kVoltageHistorySize);
    const size_t count = std::min(available, capacity);
    const size_t start = total_pushed - count;

    for (size_t i = 0; i < count; ++i) {
        buffer[i] = voltage_history[(start + i) % kVoltageHistorySize];
    }

    return count;
}

extern "C" void adcTask(void *pvParameters) {
    adc_sem         = xSemaphoreCreateBinary();
    uint16_t sample = 0;

    printf("adc task start\n");

    // HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    // if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_raw, ADC_BUF_SIZE) != HAL_OK) {
    //     while(true) osDelay(1000);
    // }
    // if (HAL_TIM_Base_Start(&htim6) != HAL_OK) {
    //     while(true) osDelay(1000);
    // }

    // 1. 先停止（确保处于干净状态）
    // HAL_ADC_Stop_DMA(&hadc1);

    // 2. ADC 校准 (必须在 ADEN=0 时)
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
        // 处理错误
    }

    // 2. 强制设置 ADC_CFGR 寄存器的 DMNGT 位为 3 (Circular Mode)
    // 00: DFSDM mode, 01: DMA One-shot, 11: DMA Circular
    // hadc1.Instance->CFGR &= ~ADC_CFGR_DMNGT;  // 先清除
    // hadc1.Instance->CFGR |= (0x3UL << ADC_CFGR_DMNGT_Pos); // 设置为 3 (Circular)

    // 3. 再次确保数据管理模式正确
    // hadc1.Instance->CFGR &= ~ADC_CFGR_DMNGMT;
    // hadc1.Instance->CFGR |= ADC_CFGR_DMNGMT_1 | ADC_CFGR_DMNGMT_0; // 11: Circular

    // 4. 以轮询模式试运行 (排除 DMA 干扰)
    // HAL_ADC_Start(&hadc1);
    // if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
    //     uint32_t val = HAL_ADC_GetValue(&hadc1);
    //     printf("adc: %lu\n", val);
    //     // 如果能运行到这里，说明 ADC 硬件是活的！
    // }
    // HAL_ADC_Stop(&hadc1);
    
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    
    // 5. 重新启动 DMA 模式
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_raw, ADC_BUF_SIZE);

    while (true) {
        if (xSemaphoreTake(adc_sem, portMAX_DELAY) == pdTRUE) {
            // 从环形缓冲处理数据
            while (app_buffer.available() > 0) {
                if (app_buffer.read(sample)) {
                    // TODO: 计算逻辑 (如 1kHz 采样后的滤波)
                    // auto cur_time = __HAL_TIM_GET_COUNTER(&htim12);
                    float current_voltage = getCalibratedVoltage(sample);
                    volt = current_voltage;
                    push_voltage_sample(current_voltage);
                    // printf("adc=%u,volt=%.3f\n", sample, getCalibratedVoltage(sample));
                }
            }
        }
    }
}

// --- 中断回调 (由 HAL_DMA_IRQHandler 调用) ---
extern "C" {

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) {
        // 后半段数据: adc_dma_raw[ADC_BUF_SIZE/2] 到 [ADC_BUF_SIZE-1]
        // 如果数据在 D1 RAM，需要 Invalidate D-Cache:
        // SCB_InvalidateDCache_by_Addr((uint32_t*)&adc_dma_raw[ADC_BUF_SIZE/2], ADC_BUF_SIZE);

        auto data_segment = std::span{adc_dma_raw}.subspan(ADC_BUF_SIZE / 2);
        app_buffer.write_span(data_segment);

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(adc_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) {
        // 前半段数据
        auto data_segment = std::span{adc_dma_raw}.subspan(0, ADC_BUF_SIZE / 2);
        app_buffer.write_span(data_segment);

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(adc_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
}

// extern "C" void adcTask(void *) {

//     HAL_ADC_Start_DMA(&hadc1, reinterpret_cast<uint32_t *>(adc_data.data()), ADC_CH_NUM); // 启动ADC + DMA

//     uint8_t  cnt  = 0;
//     uint16_t ph   = 0;
//     uint16_t turb = 0;
//     while (true) {
//         ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
//         memcpy(&adc_data_lock, &adc_data, sizeof(adc_data));

//         ph   = 0.8 * ph + adc_data_lock[0];
//         turb = 0.8 * turb + adc_data_lock[2];

//         if (cnt++ % 4 == 0) {
//             ph_   = ph;
//             turb_ = turb;
//             // printf("adc1=%d, adc2=%d, adc3=%d\n", adc_data[0], adc_data[1], adc_data[2]);
//         }
//         osDelay(10);
//     }
// }
