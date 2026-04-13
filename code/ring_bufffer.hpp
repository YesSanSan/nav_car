#pragma once
#include <cstdint>
#include <concepts>
#include <span>
#include <array>
#include <atomic>

template <typename T, std::size_t Size>
requires std::integral<T> && (Size > 0 && (Size & (Size - 1)) == 0) // 要求2的幂次，优化取模
class RingBuffer {
public:
    static constexpr std::size_t Mask = Size - 1;

    // 写入多个元素 (由 DMA 中断调用)
    bool write_span(std::span<const T> data) {
        for (const auto& val : data) {
            std::size_t next = (head_.load(std::memory_order_relaxed) + 1) & Mask;
            if (next == tail_.load(std::memory_order_acquire)) {
                return false; // 溢出
            }
            buffer_[head_.load(std::memory_order_relaxed)] = val;
            head_.store(next, std::memory_order_release);
        }
        return true;
    }

    // 读取单个元素
    bool read(T& out_data) {
        std::size_t current_tail = tail_.load(std::memory_order_relaxed);
        if (current_tail == head_.load(std::memory_order_acquire)) {
            return false;
        }
        out_data = buffer_[current_tail];
        tail_.store((current_tail + 1) & Mask, std::memory_order_release);
        return true;
    }

    std::size_t available() const {
        return (head_.load(std::memory_order_acquire) - tail_.load(std::memory_order_acquire)) & Mask;
    }

private:
    std::array<T, Size> buffer_{};
    std::atomic<std::size_t> head_{0};
    std::atomic<std::size_t> tail_{0};
};
