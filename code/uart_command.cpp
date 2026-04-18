#include <array>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_uart.h"


#include "task.h"

#include "crc.hpp"
#include "main.h"
#include "motor_control.hpp"
#include "sfud/sfud.h"

extern QSPI_HandleTypeDef hqspi;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

#define RX_BUF_SIZE    128
#define CMD_QUEUE_SIZE 8 // 最多缓存8条命令

#define SERIAL_HEADER    0xD4
#define RECEIVE_BUF_SIZE 256
#define PACKET_SIZE      (1 + 1 + 4 + 4 + 4 + 2) // header+len+x+y+z+crc16=16字节

// #define UART_DBG

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

static std::array<uint8_t, RECEIVE_BUF_SIZE> fifo_buffer;

static uint16_t        fifo_len = 0;
static ReceivePacket_t receive_packet;

uint8_t           uart_rx_byte1;
uint8_t           uart_rx_byte2;
char              uart_rx_buf[RX_BUF_SIZE];
volatile uint16_t uart_rx_index = 0;

// ---- 命令队列结构 ----
char             cmd_queue[CMD_QUEUE_SIZE][RX_BUF_SIZE];
volatile uint8_t cmd_head = 0;
volatile uint8_t cmd_tail = 0;

std::atomic<float> speed_ = 0;
std::atomic<bool>  pause_ = true;

extern Motor motor1;
extern Motor motor2;
// extern Motor motor3;
// extern Motor motor4;

std::array<Motor *, 2> motors = {&motor1, &motor2};
// std::array<Motor *, 4> motors = {&motor1, &motor2, &motor3, &motor4};

// ---- 判断队列是否为空 ----
static inline bool cmdQueueEmpty() {
    return cmd_head == cmd_tail;
}

// ---- 判断队列是否满 ----
static inline bool cmdQueueFull() {
    return ((cmd_tail + 1) % CMD_QUEUE_SIZE) == cmd_head;
}

// ---- 入队 ----
void cmdQueuePush(const char *cmd) {
    if (!cmdQueueFull()) {
        strncpy(cmd_queue[cmd_tail], cmd, RX_BUF_SIZE - 1);
        cmd_queue[cmd_tail][RX_BUF_SIZE - 1] = '\0';
        cmd_tail                             = (cmd_tail + 1) % CMD_QUEUE_SIZE;
    } else {
        // printf("Warning: cmd queue full, drop cmd: %s\r\n", cmd);
    }
}

// ---- 出队 ----
bool cmdQueuePop(char *out) {
    if (cmdQueueEmpty())
        return false;
    strncpy(out, cmd_queue[cmd_head], RX_BUF_SIZE);
    cmd_head = (cmd_head + 1) % CMD_QUEUE_SIZE;
    return true;
}

// ---- 串口接收回调 ----
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
#ifdef UART_DBG
    if (huart->Instance == USART1) {
        if (uart_rx_byte2 == '\n' || uart_rx_byte2 == '\r') {
            uart_rx_buf[uart_rx_index] = '\0';
            if (uart_rx_index > 0) {
                cmdQueuePush(uart_rx_buf); // 入队
            }
            uart_rx_index = 0;
        } else {
            if (uart_rx_index < RX_BUF_SIZE - 1) {
                uart_rx_buf[uart_rx_index] = uart_rx_byte2;
                uart_rx_index              = uart_rx_index + 1;
            }
        }

        // 再次启动接收中断
        HAL_UART_Receive_IT(&huart1, &uart_rx_byte2, 1);
    } else
#endif
        if (huart->Instance == USART2) {
        // printf("rx: %x\n", uart_rx_byte);
        if (fifo_len < RECEIVE_BUF_SIZE) {
            fifo_buffer[fifo_len++] = uart_rx_byte1;
        }

        // 继续接收下一个字节
        HAL_UART_Receive_IT(&huart2, &uart_rx_byte1, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // 获取当前错误码
        uint32_t err = HAL_UART_GetError(huart);

        // 打印错误码（调试用）
        // printf("UART1 Error: 0x%08X\r\n", err);

        // 清除错误标志并重新启动接收中断
        // 注意：HAL_UART_Receive_IT 内部会尝试清除部分状态位
        // 如果依然锁死，可以考虑先调用 HAL_UART_AbortReceive(huart);
        HAL_UART_Receive_IT(&huart1, &uart_rx_byte2, 1);
    } else if (huart->Instance == USART2) {
        HAL_UART_Receive_IT(&huart2, &uart_rx_byte1, 1);
    }
}

void ProcessReceiveData() {
    if (fifo_len < PACKET_SIZE)
        return;

    int last_header_index = -1;

    // 找到最后一个 SERIAL_HEADER 的位置
    for (int i = 0; i < fifo_len; i++) {
        if (fifo_buffer[i] == SERIAL_HEADER && fifo_buffer[i + 1] == PACKET_SIZE) {
            last_header_index = i;
        }
    }

    // 没找到帧头
    if (last_header_index == -1) {
        printf("no header !!!\n");
        // for (int i = 0; i < fifo_len; i++) {
        //     printf("%x ", fifo_buffer[i]);
        // }
        // printf("\n");
        fifo_len = 0; // 全部丢弃
        return;
    }

    // 如果剩余数据不足一帧
    if (fifo_len - last_header_index < PACKET_SIZE) {
        return; // 等待更多数据
    }


    // 校验CRC
    uint8_t *frame    = &fifo_buffer[last_header_index];
    uint16_t calc_crc = GetCRC16(frame, PACKET_SIZE - 2);
    uint16_t recv_crc = *(uint16_t *)&frame[PACKET_SIZE - 2];

    // printf("len: %d, idx: %d, crc: %x\n", fifo_len, last_header_index, calc_crc);

    // print_task_stack("uart_command", &huart1);

    if (calc_crc == recv_crc) {
        memcpy(&receive_packet, frame, PACKET_SIZE);

        {
            cmd_x.store(receive_packet.x);
            cmd_y.store(receive_packet.y);
            cmd_z.store(receive_packet.z);
        }

        // printf("RX OK: X=%.3f, Y=%.3f, Z=%.3f\r\n",
        //        receive_packet.x, receive_packet.y, receive_packet.z);
        // for (int i = 0; i < fifo_len; i++) {
        //     printf("%x ", fifo_buffer[i]);
        // }
        // printf("\n");

        // 保留最新帧后丢掉旧数据
        fifo_len = 0;
    } else {
        printf("CRC failed (header@%d)\r\n", last_header_index);
        for (int i = 0; i < fifo_len; i++) {
            printf("%x ", fifo_buffer[i]);
        }
        printf("\n");
        // CRC 错误也丢掉旧数据
        fifo_len = 0;
    }
}

#define SFUD_DEMO_TEST_BUFFER_SIZE 1024

static void sfud_demo(uint32_t addr, size_t size, uint8_t *data);

static uint8_t sfud_demo_test_buf[SFUD_DEMO_TEST_BUFFER_SIZE];

/**
 * SFUD demo for the first flash device test.
 *
 * @param addr flash start address
 * @param size test flash size
 * @param size test flash data buffer
 */
static void sfud_demo(uint32_t addr, size_t size, uint8_t *data) {
    sfud_err          result = SFUD_SUCCESS;
    const sfud_flash *flash  = sfud_get_device_table() + 0;
    size_t            i;
    /* prepare write data */
    for (i = 0; i < size; i++) {
        data[i] = i;
    }
    /* erase test */
    result = sfud_erase(flash, addr, size);
    if (result == SFUD_SUCCESS) {
        printf("Erase the %s flash data finish. Start from 0x%08X, size is %u.\r\n", flash->name, addr,
               size);
    } else {
        printf("Erase the %s flash data failed.\r\n", flash->name);
        return;
    }
    /* write test */
    result = sfud_write(flash, addr, size, data);
    if (result == SFUD_SUCCESS) {
        printf("Write the %s flash data finish. Start from 0x%08X, size is %u.\r\n", flash->name, addr,
               size);
    } else {
        printf("Write the %s flash data failed.\r\n", flash->name);
        return;
    }
    /* read test */
    result = sfud_read(flash, addr, size, data);
    if (result == SFUD_SUCCESS) {
        printf("Read the %s flash data success. Start from 0x%08X, size is %u. The data is:\r\n", flash->name, addr,
               size);
        printf("Offset (h) 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\r\n");
        for (i = 0; i < size; i++) {
            if (i % 16 == 0) {
                printf("[%08X] ", addr + i);
            }
            printf("%02X ", data[i]);
            if (((i + 1) % 16 == 0) || i == size - 1) {
                printf("\r\n");
            }
        }
        printf("\r\n");
    } else {
        printf("Read the %s flash data failed.\r\n", flash->name);
    }
    /* data check */
    for (i = 0; i < size; i++) {
        if (data[i] != i % 256) {
            printf("Read and check write data has an error. Write the %s flash data failed.\r\n", flash->name);
            break;
        }
    }
    if (i == size) {
        printf("The %s flash test is success.\r\n", flash->name);
    }
}


extern "C" void uartCommandTask(void *) {
#if 0
    if (sfud_init() == SFUD_SUCCESS) {
        sfud_qspi_fast_read_enable(sfud_get_device(SFUD_W25Q256FV_DEVICE_INDEX), 2);
        // sfud_demo(0, sizeof(sfud_demo_test_buf), sfud_demo_test_buf);
    }
#endif

    HAL_UART_Receive_IT(&huart2, &uart_rx_byte1, 1);

#ifdef UART_DBG
    HAL_UART_Receive_IT(&huart1, &uart_rx_byte2, 1);
    char cmd_buf[RX_BUF_SIZE];
#endif

    for (;;) {
        ProcessReceiveData();
#ifdef UART_DBG // 若开启串口调参
        if (cmdQueuePop(cmd_buf)) {

            // ---- 解析 PID 命令 ----
            if (strncmp(cmd_buf, "PID,", 4) == 0) {
                unsigned int wheel;
                float        kp, ki, kd;
                if (sscanf(cmd_buf, "PID,%u,%f,%f,%f", &wheel, &kp, &ki, &kd) == 4) {
                    if (wheel < motors.max_size()) {
                        (*motors[wheel]).pid.Kp = kp;
                        (*motors[wheel]).pid.Ki = ki;
                        (*motors[wheel]).pid.Kd = kd;
                        // printf("Wheel %d PID set: Kp=%.3f Ki=%.3f Kd=%.3f\r\n", wheel, kp, ki, kd);
                    }
                } else {
                    // int res = sscanf(cmd_buf, "PID,%u,%f,%f,%f", &wheel, &kp, &ki, &kd);
                    // printf("PID command format error: %s ### %d\r\n", cmd_buf, res);
                }
            }
            // ---- 解析 SPEED 命令 ----
            else if (strncmp(cmd_buf, "SPEED,", 6) == 0) {
                float val = atof(&cmd_buf[6]);
                speed_.store(val);
                // printf("Set sp/eed = %.2f\r\n", val);
            }
            // ---- 解析 pause 命令 ----
            else if (strncmp(cmd_buf, "pause ", 6) == 0) {
                int val = atoi(&cmd_buf[6]);
                pause_.store(val != 0);
                // printf("Pause = %d\r\n", val);
            }
            // ---- 未知命令 ----
            else {
                // printf("Unknown command: %s\r\n", cmd_buf);
            }
        }
#endif
        osDelay(10);
    }
}
