#pragma once

#include <cstdint>

#include "motor_control.hpp"
#include "stm32h7xx_hal.h"

constexpr uint8_t pid_tuning_frame_header = 0xA5;

enum class PidTuningFrameType : uint8_t {
    SetPid = 0x01,
    SavePid = 0x02,
    LoadPid = 0x03,
    ReadPid = 0x04,
    DefaultPid = 0x05,
    SetSpeed = 0x06,
    Pause = 0x07,
    EnterBootloader = 0x08,
    TelemetryKeepalive = 0x09,
    SetTelemetryConfig = 0x0A,

    Ack = 0x80,
    PidParams = 0x81,
    MotorTelemetry = 0x82,
};

struct [[gnu::packed]] PidTuningMotorRuntimeSample {
    uint8_t motor_index;
    uint8_t motor_count;
    uint16_t reserved;
    uint32_t sequence;
    uint64_t time_us;
    float target_rpm;
    float real_target_rpm;
    float speed_rpm;
    float pid_integral;
    float pwm_output;
    float compensated_pwm_output;
    float battery_voltage;
    int32_t encoder_value;
};

void pid_tuning_init();
void pid_tuning_poll();
bool pid_tuning_on_uart_rx(UART_HandleTypeDef *huart);
bool pid_tuning_on_uart_error(UART_HandleTypeDef *huart);
void pid_tuning_update_motor_sample(uint8_t motor_index, const PidTuningMotorRuntimeSample &sample);
