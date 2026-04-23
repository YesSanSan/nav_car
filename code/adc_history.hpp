#pragma once

#include <cstddef>

float  adc_get_latest_voltage();
size_t adc_copy_voltage_history(float *buffer, size_t capacity);
