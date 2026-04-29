#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "FreeRTOS.h"
#include "adc_history.hpp"
#include "cmsis_os2.h"
#include "main.h"
#include "semphr.h"

extern TIM_HandleTypeDef htim17;
extern DMA_HandleTypeDef hdma_tim17_up;

namespace {

struct Rgb {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

constexpr uint32_t kTimerPeriodTicks = 300;
constexpr uint32_t kDuty0 = 72;  // 0.30 us at 240 MHz timer clock
constexpr uint32_t kDuty1 = 160; // 0.667 us at 240 MHz timer clock
constexpr size_t   kLedCount = 2;
constexpr size_t   kBitsPerLed = 24;
constexpr size_t   kLeadLowSlots = 4;
constexpr size_t   kResetSlots = 256; // 320 us reset low time at 800 kHz, datasheet requires >280 us.
constexpr size_t   kDmaWordCount = kLeadLowSlots + kLedCount * kBitsPerLed + kResetSlots;
constexpr uint32_t kRefreshMs = 250;
constexpr uint8_t  kFrameRepeats = 2;
constexpr uint8_t  kMaxCellCount = 6;

constexpr std::array<Rgb, 64> kBatteryGradient = {{
    {0, 204, 0},   {0, 202, 0},   {0, 200, 0},   {0, 198, 0},
    {0, 196, 0},   {0, 193, 0},   {21, 191, 0},  {53, 189, 0},
    {72, 186, 0},  {86, 184, 0},  {98, 182, 0},  {109, 179, 0},
    {118, 176, 0}, {127, 174, 0}, {134, 171, 0}, {142, 168, 0},
    {148, 166, 0}, {155, 163, 0}, {161, 160, 0}, {167, 157, 0},
    {172, 154, 0}, {177, 151, 0}, {182, 148, 0}, {187, 145, 0},
    {191, 141, 0}, {195, 138, 0}, {199, 135, 0}, {203, 132, 0},
    {207, 128, 0}, {211, 125, 0}, {214, 121, 0}, {217, 118, 0},
    {220, 114, 0}, {223, 110, 0}, {226, 107, 0}, {229, 103, 0},
    {231, 99, 0},  {233, 95, 0},  {236, 91, 0},  {238, 87, 0},
    {240, 83, 0},  {242, 79, 0},  {243, 75, 0},  {245, 71, 0},
    {246, 66, 0},  {248, 61, 0},  {249, 56, 0},  {250, 51, 0},
    {251, 46, 0},  {252, 40, 0},  {253, 34, 0},  {253, 26, 0},
    {254, 17, 0},  {254, 4, 0},   {254, 0, 0},   {255, 0, 0},
    {255, 0, 0},   {255, 0, 0},   {255, 0, 0},   {254, 0, 0},
    {254, 0, 0},   {254, 0, 0},   {253, 0, 0},   {252, 0, 0},
}};

struct VoltagePoint {
    float   volts;
    uint8_t percent;
};

constexpr std::array<VoltagePoint, 11> kLiIonCurve = {{
    {3.00f, 0},   {3.30f, 5},   {3.50f, 10},  {3.60f, 20},
    {3.70f, 35},  {3.80f, 50},  {3.90f, 65},  {4.00f, 80},
    {4.10f, 90},  {4.15f, 95},  {4.20f, 100},
}};

alignas(32)
    __attribute__((section(".dma_buffer")))
    uint16_t ws2812_dma_words[kDmaWordCount];

SemaphoreHandle_t transfer_done_sem = nullptr;
std::atomic<bool> transfer_busy{false};

uint8_t estimate_cell_count(float pack_voltage)
{
    if (pack_voltage <= 0.2f) {
        return 1;
    }

    for (uint8_t cells = 1; cells <= kMaxCellCount; ++cells) {
        const float per_cell = pack_voltage / static_cast<float>(cells);
        if (per_cell <= 4.35f) {
            return cells;
        }
    }

    return kMaxCellCount;
}

uint8_t estimate_percent_from_cell_voltage(float cell_voltage)
{
    if (cell_voltage <= kLiIonCurve.front().volts) {
        return 0;
    }
    if (cell_voltage >= kLiIonCurve.back().volts) {
        return 100;
    }

    for (size_t i = 1; i < kLiIonCurve.size(); ++i) {
        const auto low = kLiIonCurve[i - 1];
        const auto high = kLiIonCurve[i];

        if (cell_voltage <= high.volts) {
            const float t = (cell_voltage - low.volts) / (high.volts - low.volts);
            const float percent = static_cast<float>(low.percent)
                                  + t * static_cast<float>(high.percent - low.percent);
            return static_cast<uint8_t>(std::clamp(percent + 0.5f, 0.0f, 100.0f));
        }
    }

    return 100;
}

Rgb scale_rgb(Rgb color, uint8_t brightness)
{
    return {
        static_cast<uint8_t>((static_cast<uint16_t>(color.r) * brightness) / 255U),
        static_cast<uint8_t>((static_cast<uint16_t>(color.g) * brightness) / 255U),
        static_cast<uint8_t>((static_cast<uint16_t>(color.b) * brightness) / 255U),
    };
}

Rgb color_for_battery_percent(uint8_t percent)
{
    const uint8_t index = static_cast<uint8_t>(((100U - percent) * (kBatteryGradient.size() - 1U) + 50U) / 100U);
    return kBatteryGradient[index];
}

void encode_byte(uint8_t value, size_t &offset)
{
    for (int bit = 7; bit >= 0; --bit) {
        ws2812_dma_words[offset++] = ((value & (1U << bit)) != 0U) ? kDuty1 : kDuty0;
    }
}

void encode_led(Rgb color, size_t &offset)
{
    encode_byte(color.g, offset);
    encode_byte(color.r, offset);
    encode_byte(color.b, offset);
}

void build_frame(Rgb first, Rgb second)
{
    size_t offset = 0;
    while (offset < kLeadLowSlots) {
        ws2812_dma_words[offset++] = 0;
    }

    encode_led(first, offset);
    encode_led(second, offset);

    while (offset < kDmaWordCount) {
        ws2812_dma_words[offset++] = 0;
    }
}

void dma_transfer_complete(DMA_HandleTypeDef *)
{
    __HAL_TIM_DISABLE_DMA(&htim17, TIM_DMA_UPDATE);
    __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 0);
    transfer_busy.store(false, std::memory_order_release);

    if (transfer_done_sem != nullptr) {
        BaseType_t higher_priority_task_woken = pdFALSE;
        xSemaphoreGiveFromISR(transfer_done_sem, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

bool show_frame(Rgb first, Rgb second)
{
    if (transfer_busy.exchange(true, std::memory_order_acq_rel)) {
        return false;
    }

    build_frame(first, second);

    if (transfer_done_sem != nullptr) {
        xSemaphoreTake(transfer_done_sem, 0);
    }

    hdma_tim17_up.XferCpltCallback = dma_transfer_complete;
    hdma_tim17_up.XferErrorCallback = dma_transfer_complete;

    __HAL_TIM_DISABLE_DMA(&htim17, TIM_DMA_UPDATE | TIM_DMA_CC1);
    HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COUNTER(&htim17, 0);
    HAL_TIM_GenerateEvent(&htim17, TIM_EVENTSOURCE_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&htim17, TIM_FLAG_UPDATE | TIM_FLAG_CC1);

    if (HAL_DMA_Start_IT(&hdma_tim17_up,
                         reinterpret_cast<uint32_t>(ws2812_dma_words),
                         reinterpret_cast<uint32_t>(&htim17.Instance->CCR1),
                         kDmaWordCount)
        != HAL_OK) {
        transfer_busy.store(false, std::memory_order_release);
        return false;
    }

    __HAL_TIM_ENABLE_DMA(&htim17, TIM_DMA_UPDATE);

    if (HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1) != HAL_OK) {
        HAL_DMA_Abort(&hdma_tim17_up);
        __HAL_TIM_DISABLE_DMA(&htim17, TIM_DMA_UPDATE);
        transfer_busy.store(false, std::memory_order_release);
        return false;
    }

    if (transfer_done_sem != nullptr) {
        xSemaphoreTake(transfer_done_sem, pdMS_TO_TICKS(20));
    }

    osDelay(1);
    HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 0);

    return true;
}

void show_stable_frame(Rgb first, Rgb second)
{
    for (uint8_t i = 0; i < kFrameRepeats; ++i) {
        show_frame(first, second);
        osDelay(1);
    }
}

bool reconfigure_dma_for_tim17_update()
{
    HAL_DMA_Abort(&hdma_tim17_up);
    HAL_DMA_DeInit(&hdma_tim17_up);

    hdma_tim17_up.Init.Request = DMA_REQUEST_TIM17_UP;
    hdma_tim17_up.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tim17_up.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    if (HAL_DMA_Init(&hdma_tim17_up) != HAL_OK) {
        return false;
    }

    __HAL_LINKDMA(&htim17, hdma[TIM_DMA_ID_UPDATE], hdma_tim17_up);
    return true;
}

} // namespace

extern "C" void ws2812Task(void *)
{
    transfer_done_sem = xSemaphoreCreateBinary();

    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

    __HAL_TIM_ENABLE_OCxPRELOAD(&htim17, TIM_CHANNEL_1);

    if (!reconfigure_dma_for_tim17_update()) {
        std::printf("[ws2812] DMA reconfigure to TIM17_UP failed\r\n");
        for (;;) {
            osDelay(1000);
        }
    }

    std::printf("[ws2812] TIM17 CH1 ready: ARR=%lu, DMA words=%u, request=UPDATE\r\n",
                static_cast<unsigned long>(__HAL_TIM_GET_AUTORELOAD(&htim17)),
                static_cast<unsigned>(kDmaWordCount));

    for (;;) {
        const float pack_voltage = adc_get_latest_voltage();
        const uint8_t cells = estimate_cell_count(pack_voltage);
        const float cell_voltage = pack_voltage / static_cast<float>(cells);
        const uint8_t percent = estimate_percent_from_cell_voltage(cell_voltage);
        const Rgb color = scale_rgb(color_for_battery_percent(percent), 3);

        show_stable_frame({0, 0, 0}, color);
        osDelay(kRefreshMs);
    }
}
