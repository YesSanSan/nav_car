#include <stdarg.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "main.h"
#include "semphr.h"

#include "sfud.h"


extern QSPI_HandleTypeDef hqspi;

/* 1. 互斥锁，确保 FreeRTOS 环境下线程安全 */
static SemaphoreHandle_t qspi_sem = NULL;

static void spi_lock(const struct __sfud_spi *spi) {
    if (qspi_sem) xSemaphoreTake(qspi_sem, portMAX_DELAY);
}

static void spi_unlock(const struct __sfud_spi *spi) {
    if (qspi_sem) xSemaphoreGive(qspi_sem);
}

static uint32_t qspi_instruction_mode_from_lines(uint8_t lines) {
    switch (lines) {
        case 1:
            return QSPI_INSTRUCTION_1_LINE;
        case 2:
            return QSPI_INSTRUCTION_2_LINES;
        default:
            return 0;
    }
}

static uint32_t qspi_address_mode_from_lines(uint8_t lines) {
    switch (lines) {
        case 0:
            return QSPI_ADDRESS_NONE;
        case 1:
            return QSPI_ADDRESS_1_LINE;
        case 2:
            return QSPI_ADDRESS_2_LINES;
        default:
            return 0;
    }
}

static uint32_t qspi_data_mode_from_lines(uint8_t lines) {
    switch (lines) {
        case 0:
            return QSPI_DATA_NONE;
        case 1:
            return QSPI_DATA_1_LINE;
        case 2:
            return QSPI_DATA_2_LINES;
        default:
            return 0;
    }
}

/**
 * QSPI 硬件命令转换函数
 */
static void qspi_cmd_config(QSPI_CommandTypeDef *sCommand,
                            uint32_t             instruction,
                            uint32_t             address,
                            uint32_t             dummy_cycles,
                            uint32_t             instruction_mode,
                            uint32_t             address_mode,
                            uint32_t             address_size,
                            uint32_t             data_mode) {
    sCommand->Instruction = instruction;
    sCommand->Address     = address;
    sCommand->DummyCycles = dummy_cycles;

    sCommand->InstructionMode = instruction_mode;
    sCommand->AddressMode     = address_mode;
    sCommand->AddressSize     = address_size;
    sCommand->DataMode        = data_mode;

    // 以下为 H7 QUADSPI 特有且必须配置的成员
    sCommand->SIOOMode           = QSPI_SIOO_INST_EVERY_CMD;
    sCommand->AlternateByteMode  = QSPI_ALTERNATE_BYTES_NONE;
    sCommand->AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS; // 默认值
    sCommand->DdrMode            = QSPI_DDR_MODE_DISABLE;       // 替换掉报错的 DtrMode
    sCommand->DdrHoldHalfCycle   = 0;
}

/**
 * 2. 基础的 SPI 读写 (用于 SFUD 发送控制命令、发送擦除指令等)
 */
static sfud_err spi_write_read(const sfud_spi *spi, const uint8_t *write_buf, size_t write_size, uint8_t *read_buf, size_t read_size) {
    QSPI_CommandTypeDef sCommand = {0};
    HAL_StatusTypeDef   status;

    // 初始化命令基础配置
    sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
    sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
    sCommand.AddressMode       = QSPI_ADDRESS_NONE; // 默认无地址阶段
    sCommand.DataMode          = QSPI_DATA_NONE;    // 默认无数据阶段
    sCommand.DummyCycles       = 0;

    if (write_size > 0) {
        sCommand.Instruction = write_buf[0]; // 第一个字节永远是指令

        /* 智能解析 SFUD 的字节流 */
        if (write_size >= 4 && write_size <= 5) {
            // 这通常是 擦除指令 (Instr + 3/4 Byte Address)
            sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
            sCommand.AddressSize = (write_size == 5) ? QSPI_ADDRESS_32_BITS : QSPI_ADDRESS_24_BITS;
            // 组装地址：大端模式
            if (write_size == 5)
                sCommand.Address = (write_buf[1] << 24) | (write_buf[2] << 16) | (write_buf[3] << 8) | write_buf[4];
            else
                sCommand.Address = (write_buf[1] << 16) | (write_buf[2] << 8) | write_buf[3];

            sCommand.DataMode = QSPI_DATA_NONE;
        } else if (write_size > 5) {
            // 这通常是 页写入 (Instr + 3/4 Byte Address + Data)
            // 根据已进入 4字节模式判断地址长度
            sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
            sCommand.AddressSize = QSPI_ADDRESS_32_BITS; // W25Q256 既然已进入4B模式，统一用32位
            sCommand.Address     = (write_buf[1] << 24) | (write_buf[2] << 16) | (write_buf[3] << 8) | write_buf[4];

            sCommand.DataMode = QSPI_DATA_1_LINE;
            sCommand.NbData   = write_size - 5;
        } else if (write_size > 1 && write_size < 4) {
            // 其他小型写操作（如写状态寄存器）
            sCommand.DataMode = QSPI_DATA_1_LINE;
            sCommand.NbData   = write_size - 1;
        }

        // 发送命令
        status = HAL_QSPI_Command(&hqspi, &sCommand, 500);
        if (status != HAL_OK) return SFUD_ERR_WRITE;

        // 如果有数据要发
        if (sCommand.DataMode != QSPI_DATA_NONE) {
            uint8_t *ptr = (write_size > 5) ? (uint8_t *)&write_buf[5] : (uint8_t *)&write_buf[1];
            status = HAL_QSPI_Transmit(&hqspi, ptr, 500);
            if (status != HAL_OK) return SFUD_ERR_WRITE;
        }
    }

    if (read_size > 0) {
        // SFUD 读取操作（读取状态寄存器、读取数据等）
        // 如果前面只是发了指令（write_size=1），这里就是读取返回
        sCommand.Instruction = write_buf[0];
        sCommand.DataMode    = QSPI_DATA_1_LINE;
        sCommand.NbData      = read_size;

        if (HAL_QSPI_Command(&hqspi, &sCommand, 500) != HAL_OK) return SFUD_ERR_READ;
        if (HAL_QSPI_Receive(&hqspi, read_buf, 500) != HAL_OK) return SFUD_ERR_READ;
    }

    return SFUD_SUCCESS;
}

#ifdef SFUD_USING_QSPI
/**
 * 3. 高速 QSPI 读取数据加速接口
 */
static sfud_err qspi_read(const struct __sfud_spi *spi, uint32_t addr, sfud_qspi_read_cmd_format *qspi_read_cmd_format,
                          uint8_t *read_buf, size_t read_size) {

    QSPI_CommandTypeDef sCommand = {0};

    uint32_t inst_mode = qspi_instruction_mode_from_lines(qspi_read_cmd_format->instruction_lines);
    uint32_t addr_mode = qspi_address_mode_from_lines(qspi_read_cmd_format->address_lines);
    uint32_t data_mode = qspi_data_mode_from_lines(qspi_read_cmd_format->data_lines);

    uint32_t addr_size = (qspi_read_cmd_format->address_size == 32) ? QSPI_ADDRESS_32_BITS : QSPI_ADDRESS_24_BITS;

    if (inst_mode == 0U || addr_mode == 0U || data_mode == 0U) {
        return SFUD_ERR_READ;
    }

    qspi_cmd_config(&sCommand, qspi_read_cmd_format->instruction, addr, qspi_read_cmd_format->dummy_cycles,
                    inst_mode, addr_mode, addr_size, data_mode);

    sCommand.NbData = read_size;

    if (HAL_QSPI_Command(&hqspi, &sCommand, 1000) != HAL_OK) return SFUD_ERR_READ;

    if (HAL_QSPI_Receive(&hqspi, read_buf, 1000) != HAL_OK) return SFUD_ERR_READ;

    return SFUD_SUCCESS;
}
#endif

/* 4. 初始化端口 */
sfud_err sfud_spi_port_init(sfud_flash *flash) {
    sfud_err result = SFUD_SUCCESS;

    // 创建信号量
    if (qspi_sem == NULL) {
        qspi_sem = xSemaphoreCreateMutex();
    }

    /* 绑定底层接口 */
    flash->spi.wr = spi_write_read;
#ifdef SFUD_USING_QSPI
    flash->spi.qspi_read = qspi_read;
#endif
    flash->spi.lock      = spi_lock;
    flash->spi.unlock    = spi_unlock;
    flash->spi.user_data = &hqspi;
    flash->retry.times   = 10000;

    return result;
}

/**
 * 打印调试信息
 */
void sfud_log_debug(const char *file, const long line, const char *format, ...) {
#ifndef NDEBUG
    va_list args;
    printf("[SFUD](%s:%ld) ", file, line);
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    printf("\r\n");
#endif
}

void sfud_log_info(const char *format, ...) {
    va_list args;
    printf("[SFUD] ");
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    printf("\r\n");
}
