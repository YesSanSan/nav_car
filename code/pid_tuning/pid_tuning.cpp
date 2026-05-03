#include "pid_tuning/pid_tuning.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "crc.hpp"
#include "main.h"
#include "task.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

extern std::atomic<float> speed_;
extern std::atomic<bool>  pause_;

#if defined(PID_TUNING_USE_UART3)
UART_HandleTypeDef *pid_tuning_uart = &huart3;
USART_TypeDef *pid_tuning_uart_instance = USART3;
#elif defined(PID_TUNING_USE_UART1)
UART_HandleTypeDef *pid_tuning_uart = &huart1;
USART_TypeDef *pid_tuning_uart_instance = USART1;
#else
UART_HandleTypeDef *pid_tuning_uart = nullptr;
USART_TypeDef *pid_tuning_uart_instance = nullptr;
#endif

namespace {

constexpr uint8_t ack_ok = 0;
constexpr uint8_t ack_err = 1;
constexpr std::size_t max_motors = 4;
constexpr std::size_t rx_ring_size = 256;
constexpr std::size_t max_frame_size = 96;
constexpr uint32_t telemetry_period_ms = 5;
// Keep USART1 quiet after the tuning tool disconnects so bootloader entry can write reliably.
constexpr uint32_t telemetry_active_window_ms = 1000;
constexpr uint32_t bootloader_request_magic = 0x55475044U;
// Matches the bootloader's D3 SRAM request flag and avoids the app .bss region.
constexpr uintptr_t bootloader_shared_request_flag_address = 0x3800FFF0U;

struct [[gnu::packed]] SetPidPayload {
    uint8_t motor_index;
    uint8_t reserved[3];
    float Kp;
    float Ki;
    float Kd;
};

struct [[gnu::packed]] SetSpeedPayload {
    float speed;
};

struct [[gnu::packed]] PausePayload {
    uint8_t pause;
};

struct [[gnu::packed]] TelemetryConfigPayload {
    uint8_t motor_mask;
    uint8_t signal_mask;
    uint16_t reserved;
};

struct [[gnu::packed]] TelemetryHeaderPayload {
    uint8_t motor_index;
    uint8_t motor_count;
    uint16_t signal_mask;
    uint32_t sequence;
    uint64_t time_us;
};

struct [[gnu::packed]] AckPayload {
    uint8_t command;
    uint8_t status;
    uint8_t motor_count;
    uint8_t reserved;
};

struct [[gnu::packed]] PidParamsPayload {
    uint8_t motor_count;
    uint8_t reserved[3];
    MotorPidParams params[max_motors];
};

static_assert(sizeof(SetPidPayload) == 16);
static_assert(sizeof(TelemetryConfigPayload) == 4);
static_assert(sizeof(TelemetryHeaderPayload) == 16);
static_assert(sizeof(AckPayload) == 4);
static_assert(sizeof(PidParamsPayload) == 52);
static_assert(sizeof(PidTuningMotorRuntimeSample) == 48);

uint8_t rx_byte = 0;
std::array<uint8_t, rx_ring_size> rx_ring = {};
volatile uint16_t rx_head = 0;
volatile uint16_t rx_tail = 0;

std::array<uint8_t, max_frame_size> parse_buf = {};
std::size_t parse_len = 0;

std::array<PidTuningMotorRuntimeSample, max_motors> latest_samples = {};
std::array<bool, max_motors> latest_sample_valid = {};
uint32_t last_telemetry_ms = 0;
uint32_t telemetry_sequence = 0;
uint32_t telemetry_active_until_ms = 0;
bool tuning_uart_ready = false;
bool telemetry_enabled = false;
uint8_t telemetry_motor_mask = 0x0F;
uint8_t telemetry_signal_mask = 0xFF;

uint8_t current_motor_count_u8() {
    return static_cast<uint8_t>(std::min<std::size_t>(motor_count(), max_motors));
}

bool rx_ring_push(uint8_t value) {
    const uint16_t next = static_cast<uint16_t>((rx_head + 1U) % rx_ring_size);
    if (next == rx_tail) {
        return false;
    }

    rx_ring[rx_head] = value;
    rx_head = next;
    return true;
}

bool rx_ring_pop(uint8_t *value) {
    if (value == nullptr || rx_tail == rx_head) {
        return false;
    }

    taskENTER_CRITICAL();
    if (rx_tail == rx_head) {
        taskEXIT_CRITICAL();
        return false;
    }

    *value = rx_ring[rx_tail];
    rx_tail = static_cast<uint16_t>((rx_tail + 1U) % rx_ring_size);
    taskEXIT_CRITICAL();
    return true;
}

void drop_parse_bytes(std::size_t count) {
    if (count >= parse_len) {
        parse_len = 0;
        return;
    }

    std::memmove(parse_buf.data(), parse_buf.data() + count, parse_len - count);
    parse_len -= count;
}

bool send_frame(PidTuningFrameType type, const void *payload, std::size_t payload_len) {
    std::array<uint8_t, max_frame_size> frame = {};
    const std::size_t frame_len = 1U + 1U + 1U + payload_len + sizeof(uint16_t);

    if (!tuning_uart_ready || frame_len > frame.size()) {
        return false;
    }

    frame[0] = pid_tuning_frame_header;
    frame[1] = static_cast<uint8_t>(frame_len);
    frame[2] = static_cast<uint8_t>(type);
    if (payload_len > 0 && payload != nullptr) {
        std::memcpy(&frame[3], payload, payload_len);
    }

    const uint16_t crc = GetCRC16(frame.data(), static_cast<uint32_t>(frame_len - sizeof(uint16_t)));
    frame[frame_len - 2U] = static_cast<uint8_t>(crc & 0xFFU);
    frame[frame_len - 1U] = static_cast<uint8_t>((crc >> 8U) & 0xFFU);

    return pid_tuning_uart != nullptr && HAL_UART_Transmit(pid_tuning_uart, frame.data(), static_cast<uint16_t>(frame_len), 5) == HAL_OK;
}

bool send_telemetry_sample(const PidTuningMotorRuntimeSample &sample) {
    std::array<uint8_t, max_frame_size> payload = {};
    TelemetryHeaderPayload header = {
        .motor_index = sample.motor_index,
        .motor_count = sample.motor_count,
        .signal_mask = telemetry_signal_mask,
        .sequence = sample.sequence,
        .time_us = sample.time_us,
    };
    std::memcpy(payload.data(), &header, sizeof(header));
    std::size_t offset = sizeof(header);

    const float values[] = {
        sample.target_rpm,
        sample.real_target_rpm,
        sample.speed_rpm,
        sample.pid_integral,
        sample.pwm_output,
        sample.compensated_pwm_output,
        sample.battery_voltage,
    };

    for (std::size_t i = 0; i < sizeof(values) / sizeof(values[0]); ++i) {
        if ((telemetry_signal_mask & (1U << i)) == 0U) {
            continue;
        }
        if (offset + sizeof(float) > payload.size()) {
            return false;
        }
        std::memcpy(payload.data() + offset, &values[i], sizeof(float));
        offset += sizeof(float);
    }

    return send_frame(PidTuningFrameType::MotorTelemetry, payload.data(), offset);
}

void send_ack(PidTuningFrameType command, uint8_t status) {
    const AckPayload payload = {
        .command = static_cast<uint8_t>(command),
        .status = status,
        .motor_count = current_motor_count_u8(),
        .reserved = 0,
    };
    (void)send_frame(PidTuningFrameType::Ack, &payload, sizeof(payload));
}

void send_pid_params() {
    PidParamsPayload payload = {};
    const std::size_t count = std::min<std::size_t>(motor_count(), max_motors);

    payload.motor_count = static_cast<uint8_t>(count);
    for (std::size_t i = 0; i < count; ++i) {
        MotorPidParams params = {};
        if (motor_get_pid_params(i, &params)) {
            payload.params[i] = params;
        }
    }

    (void)send_frame(PidTuningFrameType::PidParams, &payload, sizeof(payload));
}

void refresh_telemetry_window() {
    telemetry_enabled = true;
    telemetry_active_until_ms = HAL_GetTick() + telemetry_active_window_ms;
}

template <typename Payload>
bool read_payload(const uint8_t *payload, std::size_t payload_len, Payload *out) {
    if (payload == nullptr || out == nullptr || payload_len != sizeof(Payload)) {
        return false;
    }

    std::memcpy(out, payload, sizeof(Payload));
    return true;
}

void handle_set_pid(const uint8_t *payload, std::size_t payload_len) {
    refresh_telemetry_window();

    SetPidPayload parsed = {};
    if (!read_payload(payload, payload_len, &parsed)) {
        send_ack(PidTuningFrameType::SetPid, ack_err);
        return;
    }

    const MotorPidParams params = {
        .Kp = parsed.Kp,
        .Ki = parsed.Ki,
        .Kd = parsed.Kd,
    };

    send_ack(PidTuningFrameType::SetPid, motor_set_pid_params(parsed.motor_index, params) ? ack_ok : ack_err);
}

void handle_set_speed(const uint8_t *payload, std::size_t payload_len) {
    refresh_telemetry_window();

    SetSpeedPayload parsed = {};
    if (!read_payload(payload, payload_len, &parsed)) {
        send_ack(PidTuningFrameType::SetSpeed, ack_err);
        return;
    }

    speed_.store(parsed.speed);
    send_ack(PidTuningFrameType::SetSpeed, ack_ok);
}

void handle_pause(const uint8_t *payload, std::size_t payload_len) {
    refresh_telemetry_window();

    PausePayload parsed = {};
    if (!read_payload(payload, payload_len, &parsed)) {
        send_ack(PidTuningFrameType::Pause, ack_err);
        return;
    }

    pause_.store(parsed.pause != 0);
    send_ack(PidTuningFrameType::Pause, ack_ok);
}

void handle_telemetry_config(const uint8_t *payload, std::size_t payload_len) {
    refresh_telemetry_window();

    TelemetryConfigPayload parsed = {};
    if (!read_payload(payload, payload_len, &parsed)) {
        send_ack(PidTuningFrameType::SetTelemetryConfig, ack_err);
        return;
    }

    telemetry_motor_mask = static_cast<uint8_t>(parsed.motor_mask & ((1U << max_motors) - 1U));
    telemetry_signal_mask = parsed.signal_mask;
    send_ack(PidTuningFrameType::SetTelemetryConfig, ack_ok);
}

void handle_enter_bootloader() {
    send_ack(PidTuningFrameType::EnterBootloader, ack_ok);
    tuning_uart_ready = false;
    telemetry_enabled = false;
    taskENTER_CRITICAL();
    // D3 SRAM survives NVIC_SystemReset(), so the bootloader can see the request.
    *reinterpret_cast<volatile uint32_t *>(bootloader_shared_request_flag_address) = bootloader_request_magic;
    taskEXIT_CRITICAL();
    HAL_Delay(20);
    NVIC_SystemReset();
}

void handle_frame(PidTuningFrameType type, const uint8_t *payload, std::size_t payload_len) {
    switch (type) {
        case PidTuningFrameType::SetPid:
            handle_set_pid(payload, payload_len);
            break;
        case PidTuningFrameType::SavePid:
            refresh_telemetry_window();
            send_ack(type, motor_save_pid_params() ? ack_ok : ack_err);
            break;
        case PidTuningFrameType::LoadPid:
            refresh_telemetry_window();
            send_ack(type, motor_load_pid_params() ? ack_ok : ack_err);
            send_pid_params();
            break;
        case PidTuningFrameType::ReadPid:
            refresh_telemetry_window();
            send_ack(type, ack_ok);
            send_pid_params();
            break;
        case PidTuningFrameType::DefaultPid:
            refresh_telemetry_window();
            motor_apply_default_pid();
            send_ack(type, ack_ok);
            send_pid_params();
            break;
        case PidTuningFrameType::SetSpeed:
            handle_set_speed(payload, payload_len);
            break;
        case PidTuningFrameType::Pause:
            handle_pause(payload, payload_len);
            break;
        case PidTuningFrameType::EnterBootloader:
            if (payload_len == 0U) {
                handle_enter_bootloader();
            } else {
                send_ack(type, ack_err);
            }
            break;
        case PidTuningFrameType::TelemetryKeepalive:
            if (payload_len == 0U) {
                refresh_telemetry_window();
            }
            break;
        case PidTuningFrameType::SetTelemetryConfig:
            handle_telemetry_config(payload, payload_len);
            break;
        default:
            break;
    }
}

void append_parse_byte(uint8_t value) {
    if (parse_len >= parse_buf.size()) {
        parse_len = 0;
    }

    parse_buf[parse_len++] = value;

    for (;;) {
        while (parse_len > 0 && parse_buf[0] != pid_tuning_frame_header) {
            drop_parse_bytes(1);
        }

        if (parse_len < 3) {
            return;
        }

        const std::size_t frame_len = parse_buf[1];
        if (frame_len < 5 || frame_len > parse_buf.size()) {
            drop_parse_bytes(1);
            continue;
        }

        if (parse_len < frame_len) {
            return;
        }

        const uint16_t expected_crc = GetCRC16(parse_buf.data(), static_cast<uint32_t>(frame_len - sizeof(uint16_t)));
        const uint16_t received_crc = static_cast<uint16_t>(parse_buf[frame_len - 2U]) |
                                      (static_cast<uint16_t>(parse_buf[frame_len - 1U]) << 8U);

        if (expected_crc == received_crc) {
            const auto type = static_cast<PidTuningFrameType>(parse_buf[2]);
            handle_frame(type, &parse_buf[3], frame_len - 5U);
            drop_parse_bytes(frame_len);
        } else {
            drop_parse_bytes(1);
        }
    }
}

void process_rx_bytes() {
    uint8_t value = 0;
    while (rx_ring_pop(&value)) {
        append_parse_byte(value);
    }
}

void send_telemetry() {
    const uint32_t now_ms = HAL_GetTick();
    if (!tuning_uart_ready || !telemetry_enabled || now_ms - last_telemetry_ms < telemetry_period_ms) {
        return;
    }
    if (static_cast<int32_t>(now_ms - telemetry_active_until_ms) >= 0) {
        telemetry_enabled = false;
        return;
    }

    last_telemetry_ms = now_ms;

    std::array<PidTuningMotorRuntimeSample, max_motors> samples = {};
    std::array<bool, max_motors> valid = {};

    taskENTER_CRITICAL();
    samples = latest_samples;
    valid = latest_sample_valid;
    taskEXIT_CRITICAL();

    for (std::size_t i = 0; i < samples.size(); ++i) {
        if (!valid[i] || (telemetry_motor_mask & (1U << i)) == 0U) {
            continue;
        }

        samples[i].motor_count = current_motor_count_u8();
        samples[i].sequence = telemetry_sequence++;
        samples[i].time_us = static_cast<uint64_t>(now_ms) * 1000ULL;
        (void)send_telemetry_sample(samples[i]);
    }
}

} // namespace

void pid_tuning_init() {
    if (pid_tuning_uart == nullptr) {
        tuning_uart_ready = false;
        return;
    }
    tuning_uart_ready = true;
    (void)HAL_UART_Receive_IT(pid_tuning_uart, &rx_byte, 1);
}

void pid_tuning_poll() {
    process_rx_bytes();
    send_telemetry();
}

bool pid_tuning_on_uart_rx(UART_HandleTypeDef *huart) {
    if (huart == nullptr || huart->Instance != pid_tuning_uart_instance) {
        return false;
    }

    (void)rx_ring_push(rx_byte);
    (void)HAL_UART_Receive_IT(pid_tuning_uart, &rx_byte, 1);
    return true;
}

bool pid_tuning_on_uart_error(UART_HandleTypeDef *huart) {
    if (huart == nullptr || huart->Instance != pid_tuning_uart_instance) {
        return false;
    }

    (void)HAL_UART_Receive_IT(pid_tuning_uart, &rx_byte, 1);
    return true;
}

void pid_tuning_update_motor_sample(uint8_t motor_index, const PidTuningMotorRuntimeSample &sample) {
    if (motor_index >= max_motors) {
        return;
    }

    taskENTER_CRITICAL();
    latest_samples[motor_index] = sample;
    latest_sample_valid[motor_index] = true;
    taskEXIT_CRITICAL();
}
