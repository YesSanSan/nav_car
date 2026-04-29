#include <array>
#include <cstdint>
#include <cstdio>

#include "cmsis_os2.h"
#include "main.h"

extern TIM_HandleTypeDef htim16;

namespace {

struct Tone {
    uint16_t hz;
    uint16_t ms;
};

[[maybe_unused]] constexpr uint16_t R  = 0;
[[maybe_unused]] constexpr uint16_t C4 = 262;
[[maybe_unused]] constexpr uint16_t D4 = 294;
[[maybe_unused]] constexpr uint16_t E4 = 330;
[[maybe_unused]] constexpr uint16_t F4 = 349;
[[maybe_unused]] constexpr uint16_t G4 = 392;
[[maybe_unused]] constexpr uint16_t A4 = 440;
[[maybe_unused]] constexpr uint16_t B4 = 494;
[[maybe_unused]] constexpr uint16_t C5 = 523;
[[maybe_unused]] constexpr uint16_t D5 = 587;
[[maybe_unused]] constexpr uint16_t E5 = 659;
[[maybe_unused]] constexpr uint16_t F5 = 698;
[[maybe_unused]] constexpr uint16_t G5 = 784;

// clang-format off
[[maybe_unused]]constexpr std::array<Tone, 32> kBootTheme = {{
    {C5, 180}, {E5, 180}, {G5, 220}, { R,  60}, {E5, 160}, {G5, 160}, {C5, 360}, { R, 120},
    {G4, 160}, {C5, 160}, {D5, 220}, {E5, 220}, { R,  80}, {D5, 160}, {C5, 300}, { R, 120},
    {E4, 180}, {G4, 180}, {C5, 240}, { R,  80}, {D5, 160}, {E5, 160}, {G5, 360}, { R, 120},
    {F5, 160}, {E5, 160}, {D5, 180}, {C5, 180}, {G4, 180}, {C5, 180}, { R, 120}, {C5, 420},
}};

[[maybe_unused]]constexpr std::array<Tone, 40> kCruiseTheme = {{
    {A4, 140}, { R,  30}, {A4, 140}, {C5, 180}, {D5, 220}, { R,  60}, {D5, 140}, {E5, 140},
    {F5, 220}, {E5, 180}, {D5, 180}, {C5, 180}, {A4, 260}, { R, 120}, {C5, 140}, {D5, 140},
    {E5, 220}, { R,  40}, {E5, 140}, {F5, 140}, {G5, 280}, { R, 100}, {G5, 120}, {F5, 120},
    {E5, 180}, {D5, 180}, {C5, 180}, {A4, 220}, {G4, 220}, { R, 100}, {A4, 180}, {C5, 180},
    {D5, 260}, { R,  60}, {E5, 160}, {D5, 160}, {C5, 220}, {A4, 220}, { R, 120}, {A4, 360},
}};

[[maybe_unused]]constexpr std::array<Tone, 28> kLowBatteryTheme = {{
    {E5, 120}, { R,  60}, {E5, 120}, { R, 180}, {D5, 120}, { R,  60}, {D5, 120},
    { R, 260}, {C5, 180}, { R,  80}, {G4, 220}, { R, 260}, {E5, 120}, { R,  60},
    {E5, 120}, { R, 180}, {D5, 120}, { R,  60}, {D5, 120}, { R, 260}, {C5, 180},
    { R,  80}, {G4, 220}, { R, 320}, {F4, 140}, {E4, 140}, {D4, 180}, {C4, 320},
}};
constexpr std::array<Tone, 4> kStartupTheme = {{    
    {C4,  300}, {E4,  300}, {G4,  250}, {C5, 400}    
}};
constexpr std::array<Tone, 3> kArmedTheme = {{    
    {C5, 100}, { R,  50}, {E5, 150}    
}};
[[maybe_unused]]constexpr std::array<Tone, 5> kSystemReadyTheme = {{
    {G4, 120}, { R,  40}, {C5, 120}, { R,  40}, {E5, 250}  
}};
[[maybe_unused]]constexpr std::array<Tone, 8> kWarningTheme = {{
    {G5, 100}, {R, 50}, {G5, 100}, {R, 50}, 
    {E5, 100}, {R, 50}, {C5, 250}, {R, 50}
}};
// clang-format on

uint16_t tune_to_resonant_band(uint16_t hz) {
    if (hz == R) {
        return R;
    }

    uint32_t tuned = static_cast<uint32_t>(hz) * 8U;
    while (tuned > 6200U) {
        tuned /= 2U;
    }
    while (tuned < 2500U) {
        tuned *= 2U;
    }

    return static_cast<uint16_t>(tuned);
}

void stop_tone() {
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 0);
    HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
}

void play_tone(uint16_t hz) {
    hz = tune_to_resonant_band(hz);

    if (hz == R) {
        stop_tone();
        return;
    }

    constexpr uint32_t timer_hz = 1000000U;
    const uint32_t     period   = (timer_hz / hz) - 1U;
    const uint32_t     pulse    = (period + 1U) / 2U;

    HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
    __HAL_TIM_SET_AUTORELOAD(&htim16, period);
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, pulse);
    __HAL_TIM_SET_COUNTER(&htim16, 0);
    HAL_TIM_GenerateEvent(&htim16, TIM_EVENTSOURCE_UPDATE);
    HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
}

template <size_t N>
void play_fragment(const std::array<Tone, N> &fragment) {
    for (const auto &tone : fragment) {
        play_tone(tone.hz);
        osDelay(tone.ms);
        stop_tone();
        osDelay(12);
    }
}

} // namespace

extern "C" void beeperTask(void *) {
    std::printf("[beeper] TIM16 CH1 ready: 1 MHz base clock\r\n");
    stop_tone();

    // for (;;) {
    // play_fragment(kBootTheme);
    // osDelay(1200);
    // play_fragment(kCruiseTheme);
    // osDelay(2000);
    play_fragment(kStartupTheme);
    while (1) {
        osDelay(1000);
    }
    // }
}
