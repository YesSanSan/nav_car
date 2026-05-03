#include <array>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "cmsis_os2.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_uart.h"

#include "crc.hpp"
#include "main.h"
#include "pid_tuning/pid_tuning.hpp"

extern UART_HandleTypeDef huart2;

#define SERIAL_HEADER    0xD4
#define RECEIVE_BUF_SIZE 256
#define PACKET_SIZE      (1 + 1 + 4 + 4 + 4 + 2) // header+len+x+y+z+crc16=16字节

struct [[gnu::packed]] ReceivePacket_t {
    uint8_t  header;
    uint8_t  length;
    float    x;
    float    y;
    float    z;
    uint16_t crc16;
};

std::atomic<float> cmd_x = 0;
std::atomic<float> cmd_y = 0;
std::atomic<float> cmd_z = 0;

std::atomic<float> speed_ = 0;
std::atomic<bool>  pause_ = true;

static std::array<uint8_t, RECEIVE_BUF_SIZE> fifo_buffer;
static uint16_t        fifo_len = 0;
static ReceivePacket_t receive_packet;
static uint8_t         uart_rx_byte = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (pid_tuning_on_uart_rx(huart)) {
        return;
    }

    if (huart->Instance == USART2) {
        if (fifo_len < RECEIVE_BUF_SIZE) {
            fifo_buffer[fifo_len++] = uart_rx_byte;
        }

        HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (pid_tuning_on_uart_error(huart)) {
        return;
    }

    if (huart->Instance == USART2) {
        HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
    }
}

static void ProcessReceiveData() {
    if (fifo_len < PACKET_SIZE) {
        return;
    }

    int last_header_index = -1;
    for (int i = 0; i < fifo_len; i++) {
        if (fifo_buffer[i] == SERIAL_HEADER && fifo_buffer[i + 1] == PACKET_SIZE) {
            last_header_index = i;
        }
    }

    if (last_header_index == -1) {
        fifo_len = 0;
        return;
    }

    if (fifo_len - last_header_index < PACKET_SIZE) {
        return;
    }

    uint8_t *frame = &fifo_buffer[last_header_index];
    const uint16_t calc_crc = GetCRC16(frame, PACKET_SIZE - 2);
    const uint16_t recv_crc = *(uint16_t *)&frame[PACKET_SIZE - 2];

    if (calc_crc == recv_crc) {
        memcpy(&receive_packet, frame, PACKET_SIZE);
        cmd_x.store(receive_packet.x);
        cmd_y.store(receive_packet.y);
        cmd_z.store(receive_packet.z);
    } else {
        printf("CRC failed (header@%d)\r\n", last_header_index);
    }

    fifo_len = 0;
}

extern "C" void uartCommandTask(void *) {
    HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
    pid_tuning_init();

    for (;;) {
        ProcessReceiveData();
        pid_tuning_poll();
        osDelay(1);
    }
}
