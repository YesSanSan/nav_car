#include "stdio.h"
#include "stm32h7xx_hal.h"


extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

/* USART1 is reserved for the binary PID tuning protocol.  Raw printf bytes on
 * the same UART corrupt framed telemetry and cause CRC errors in the host tool.
 */
int _write(int file __attribute((unused)), char *ptr, int len) // NOLINT(bugprone-reserved-identifier)
{
#if defined(PID_TUNING_USE_UART1)
    (void)ptr;
    (void)huart1;
#else
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
#endif
    // HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}
