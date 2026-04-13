#include "stdio.h"
#include "stm32h7xx_hal.h"


extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

/* 支持 printf 输出到 UART1 */
int _write(int file __attribute((unused)), char *ptr, int len) // NOLINT(bugprone-reserved-identifier)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    // HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}
