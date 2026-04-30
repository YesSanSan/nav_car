#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "cmsis_os2.h"
#include "main.h"
#include "adc_history.hpp"
#include "ui_input.hpp"

extern SPI_HandleTypeDef hspi4;
extern TIM_HandleTypeDef htim15;

namespace {

constexpr uint16_t kPanelWidth = 172;
constexpr uint16_t kPanelHeight = 320;
constexpr uint16_t kPanelXOffset = 34;
constexpr uint16_t kPanelYOffset = 0;
constexpr uint32_t kFramePixelCount = static_cast<uint32_t>(kPanelWidth) * static_cast<uint32_t>(kPanelHeight);
constexpr uint32_t kSpiDmaChunkBytes = 4096;
constexpr uint32_t kSpiDmaTimeoutMs = 100;
constexpr uint32_t kBacklightPercent = 90;
constexpr bool     kBacklightActiveHigh = true;
constexpr uint32_t kVoltagePageRefreshMs = 250;
constexpr size_t   kGraphHistoryCapacity = 160;

enum class Page : uint8_t {
    Qr = 0,
    Voltage,
    Settings,
    Count,
};

struct UiState {
    Page current_page = Page::Qr;
    std::array<uint8_t, static_cast<size_t>(Page::Count)> selected_index = {0, 0, 0};
    char status_line[32] = "DBL CLICK TO CHANGE PAGE";
};

alignas(32)
    __attribute__((section(".dma_buffer")))
    uint16_t framebuffer[kFramePixelCount];

alignas(32)
    __attribute__((section(".sram1_buffer")))
    float graph_history[kGraphHistoryCapacity];

UiState ui_state;
DMA_HandleTypeDef spi4_tx_dma;
osSemaphoreId_t   spi4_tx_dma_done = nullptr;
volatile bool     spi4_tx_dma_error = false;
bool              spi4_tx_dma_ready = false;

constexpr uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return static_cast<uint16_t>(((r & 0xF8U) << 8) | ((g & 0xFCU) << 3) | (b >> 3));
}

constexpr uint16_t to_panel_color(uint16_t color)
{
    return static_cast<uint16_t>((color >> 8) | (color << 8));
}

constexpr uint16_t kColorBg = to_panel_color(rgb565(9, 18, 26));
constexpr uint16_t kColorPanel = to_panel_color(rgb565(19, 34, 45));
constexpr uint16_t kColorPanelAlt = to_panel_color(rgb565(27, 48, 64));
constexpr uint16_t kColorAccent = to_panel_color(rgb565(0, 200, 160));
constexpr uint16_t kColorAccentSoft = to_panel_color(rgb565(40, 110, 100));
constexpr uint16_t kColorDanger = to_panel_color(rgb565(220, 80, 80));
constexpr uint16_t kColorText = to_panel_color(rgb565(235, 242, 245));
constexpr uint16_t kColorTextMuted = to_panel_color(rgb565(150, 168, 180));
constexpr uint16_t kColorWhite = to_panel_color(rgb565(255, 255, 255));
constexpr uint16_t kColorBlack = to_panel_color(rgb565(0, 0, 0));
constexpr uint16_t kColorGraph = to_panel_color(rgb565(255, 187, 0));

constexpr uint16_t make_glyph(uint8_t r0, uint8_t r1, uint8_t r2, uint8_t r3, uint8_t r4)
{
    return static_cast<uint16_t>((r0 << 12) | (r1 << 9) | (r2 << 6) | (r3 << 3) | r4);
}

uint16_t glyph_bits(char c)
{
    switch (c) {
        case 'A': return make_glyph(0b010, 0b101, 0b111, 0b101, 0b101);
        case 'B': return make_glyph(0b110, 0b101, 0b110, 0b101, 0b110);
        case 'C': return make_glyph(0b011, 0b100, 0b100, 0b100, 0b011);
        case 'D': return make_glyph(0b110, 0b101, 0b101, 0b101, 0b110);
        case 'E': return make_glyph(0b111, 0b100, 0b110, 0b100, 0b111);
        case 'F': return make_glyph(0b111, 0b100, 0b110, 0b100, 0b100);
        case 'G': return make_glyph(0b011, 0b100, 0b101, 0b101, 0b011);
        case 'H': return make_glyph(0b101, 0b101, 0b111, 0b101, 0b101);
        case 'I': return make_glyph(0b111, 0b010, 0b010, 0b010, 0b111);
        case 'J': return make_glyph(0b001, 0b001, 0b001, 0b101, 0b010);
        case 'K': return make_glyph(0b101, 0b101, 0b110, 0b101, 0b101);
        case 'L': return make_glyph(0b100, 0b100, 0b100, 0b100, 0b111);
        case 'M': return make_glyph(0b101, 0b111, 0b111, 0b101, 0b101);
        case 'N': return make_glyph(0b101, 0b111, 0b111, 0b111, 0b101);
        case 'O': return make_glyph(0b010, 0b101, 0b101, 0b101, 0b010);
        case 'P': return make_glyph(0b110, 0b101, 0b110, 0b100, 0b100);
        case 'Q': return make_glyph(0b010, 0b101, 0b101, 0b111, 0b011);
        case 'R': return make_glyph(0b110, 0b101, 0b110, 0b101, 0b101);
        case 'S': return make_glyph(0b011, 0b100, 0b010, 0b001, 0b110);
        case 'T': return make_glyph(0b111, 0b010, 0b010, 0b010, 0b010);
        case 'U': return make_glyph(0b101, 0b101, 0b101, 0b101, 0b111);
        case 'V': return make_glyph(0b101, 0b101, 0b101, 0b101, 0b010);
        case 'W': return make_glyph(0b101, 0b101, 0b111, 0b111, 0b101);
        case 'X': return make_glyph(0b101, 0b101, 0b010, 0b101, 0b101);
        case 'Y': return make_glyph(0b101, 0b101, 0b010, 0b010, 0b010);
        case 'Z': return make_glyph(0b111, 0b001, 0b010, 0b100, 0b111);
        case '0': return make_glyph(0b111, 0b101, 0b101, 0b101, 0b111);
        case '1': return make_glyph(0b010, 0b110, 0b010, 0b010, 0b111);
        case '2': return make_glyph(0b110, 0b001, 0b111, 0b100, 0b111);
        case '3': return make_glyph(0b110, 0b001, 0b111, 0b001, 0b110);
        case '4': return make_glyph(0b101, 0b101, 0b111, 0b001, 0b001);
        case '5': return make_glyph(0b111, 0b100, 0b111, 0b001, 0b110);
        case '6': return make_glyph(0b011, 0b100, 0b111, 0b101, 0b111);
        case '7': return make_glyph(0b111, 0b001, 0b010, 0b010, 0b010);
        case '8': return make_glyph(0b111, 0b101, 0b111, 0b101, 0b111);
        case '9': return make_glyph(0b111, 0b101, 0b111, 0b001, 0b110);
        case ':': return make_glyph(0b000, 0b010, 0b000, 0b010, 0b000);
        case '.': return make_glyph(0b000, 0b000, 0b000, 0b000, 0b010);
        case '-': return make_glyph(0b000, 0b000, 0b111, 0b000, 0b000);
        case '/': return make_glyph(0b001, 0b001, 0b010, 0b100, 0b100);
        case '>': return make_glyph(0b100, 0b010, 0b001, 0b010, 0b100);
        case '<': return make_glyph(0b001, 0b010, 0b100, 0b010, 0b001);
        case ' ': return 0;
        default: return make_glyph(0b111, 0b001, 0b011, 0b000, 0b010);
    }
}

const char *page_title(Page page)
{
    switch (page) {
        case Page::Qr:
            return "QR PLACEHOLDER";
        case Page::Voltage:
            return "BATTERY VOLT";
        case Page::Settings:
            return "SETTINGS";
        default:
            return "PAGE";
    }
}

uint8_t page_item_count(Page page)
{
    switch (page) {
        case Page::Qr:
            return 3;
        case Page::Voltage:
            return 3;
        case Page::Settings:
            return 1;
        default:
            return 1;
    }
}

void set_status(const char *text)
{
    std::snprintf(ui_state.status_line, sizeof(ui_state.status_line), "%s", text);
}

void advance_page(int delta)
{
    const int page_count = static_cast<int>(Page::Count);
    int       current = static_cast<int>(ui_state.current_page);
    current = (current + delta + page_count) % page_count;
    ui_state.current_page = static_cast<Page>(current);
    std::snprintf(ui_state.status_line, sizeof(ui_state.status_line), "PAGE %d/%d",
                  current + 1,
                  page_count);
}

void move_selection(int delta)
{
    const size_t  page_index = static_cast<size_t>(ui_state.current_page);
    const uint8_t count = page_item_count(ui_state.current_page);
    if (count == 0) {
        return;
    }

    int selected = static_cast<int>(ui_state.selected_index[page_index]);
    selected = (selected + delta + count) % count;
    ui_state.selected_index[page_index] = static_cast<uint8_t>(selected);

    std::snprintf(ui_state.status_line, sizeof(ui_state.status_line), "ITEM %d/%d",
                  selected + 1,
                  count);
}

void apply_input(UiInputEvent event)
{
    switch (event) {
        case UiInputEvent::PrevItem:
            move_selection(-1);
            break;
        case UiInputEvent::NextItem:
            move_selection(1);
            break;
        case UiInputEvent::PrevPage:
            advance_page(-1);
            break;
        case UiInputEvent::NextPage:
            advance_page(1);
            break;
        case UiInputEvent::Cancel:
            set_status("CANCEL");
            break;
        case UiInputEvent::Confirm:
            set_status("CONFIRM");
            break;
    }
}

void fill_screen(uint16_t color)
{
    std::fill(std::begin(framebuffer), std::end(framebuffer), color);
}

void draw_pixel(int16_t x, int16_t y, uint16_t color)
{
    if (x < 0 || y < 0 || x >= static_cast<int16_t>(kPanelWidth) || y >= static_cast<int16_t>(kPanelHeight)) {
        return;
    }

    framebuffer[static_cast<size_t>(y) * kPanelWidth + static_cast<size_t>(x)] = color;
}

void fill_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    const int16_t x0 = std::max<int16_t>(0, x);
    const int16_t y0 = std::max<int16_t>(0, y);
    const int16_t x1 = std::min<int16_t>(kPanelWidth, x + w);
    const int16_t y1 = std::min<int16_t>(kPanelHeight, y + h);

    for (int16_t row = y0; row < y1; ++row) {
        for (int16_t col = x0; col < x1; ++col) {
            framebuffer[static_cast<size_t>(row) * kPanelWidth + static_cast<size_t>(col)] = color;
        }
    }
}

void draw_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    fill_rect(x, y, w, 1, color);
    fill_rect(x, y + h - 1, w, 1, color);
    fill_rect(x, y, 1, h, color);
    fill_rect(x + w - 1, y, 1, h, color);
}

void draw_line(int x0, int y0, int x1, int y1, uint16_t color)
{
    int dx = std::abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    while (true) {
        draw_pixel(static_cast<int16_t>(x0), static_cast<int16_t>(y0), color);
        if (x0 == x1 && y0 == y1) {
            break;
        }
        const int e2 = err * 2;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void draw_char(int16_t x, int16_t y, char c, uint16_t color, uint8_t scale)
{
    const uint16_t glyph = glyph_bits(c);
    for (uint8_t row = 0; row < 5; ++row) {
        for (uint8_t col = 0; col < 3; ++col) {
            const uint16_t mask = static_cast<uint16_t>(1U << (14U - (row * 3U + col)));
            if ((glyph & mask) != 0U) {
                fill_rect(x + static_cast<int16_t>(col * scale),
                          y + static_cast<int16_t>(row * scale),
                          scale,
                          scale,
                          color);
            }
        }
    }
}

void draw_text(int16_t x, int16_t y, const char *text, uint16_t color, uint8_t scale = 2)
{
    if (text == nullptr) {
        return;
    }

    int16_t cursor_x = x;
    for (size_t i = 0; text[i] != '\0'; ++i) {
        draw_char(cursor_x, y, text[i], color, scale);
        cursor_x += static_cast<int16_t>(4 * scale);
    }
}

void draw_chip(int16_t x, int16_t y, int16_t w, const char *label, bool selected)
{
    const uint16_t fill = selected ? kColorAccent : kColorPanelAlt;
    const uint16_t text = selected ? kColorBg : kColorTextMuted;
    fill_rect(x, y, w, 20, fill);
    draw_rect(x, y, w, 20, selected ? kColorWhite : kColorAccentSoft);
    draw_text(x + 8, y + 7, label, text, 1);
}

void draw_page_dots(Page page)
{
    for (uint8_t i = 0; i < static_cast<uint8_t>(Page::Count); ++i) {
        const int16_t x = static_cast<int16_t>(kPanelWidth / 2 - 18 + i * 18);
        const int16_t y = 26;
        fill_rect(x, y, 10, 10,
                  (i == static_cast<uint8_t>(page)) ? kColorAccent : kColorPanelAlt);
    }
}

void draw_header(Page page)
{
    fill_rect(0, 0, kPanelWidth, 38, kColorPanel);
    draw_text(10, 10, page_title(page), kColorText, 2);
    draw_page_dots(page);
}

void draw_footer()
{
    fill_rect(0, kPanelHeight - 52, kPanelWidth, 52, kColorPanel);
    draw_text(8, kPanelHeight - 42, "K0< ITEM  K1> ITEM", kColorTextMuted, 1);
    draw_text(8, kPanelHeight - 30, "DBL PAGE  HOLD X/OK", kColorTextMuted, 1);
    draw_text(8, kPanelHeight - 17, ui_state.status_line, kColorText, 1);
}

void draw_qr_placeholder(uint8_t selected)
{
    draw_header(Page::Qr);

    fill_rect(16, 52, 140, 140, kColorWhite);
    draw_rect(16, 52, 140, 140, kColorAccentSoft);

    constexpr int module_size = 4;
    constexpr int qr_size = 29;
    const int qr_origin_x = 28;
    const int qr_origin_y = 64;

    auto draw_finder = [&](int ox, int oy) {
        fill_rect(static_cast<int16_t>(ox), static_cast<int16_t>(oy), 7 * module_size, 7 * module_size, kColorBlack);
        fill_rect(static_cast<int16_t>(ox + module_size), static_cast<int16_t>(oy + module_size),
                  5 * module_size, 5 * module_size, kColorWhite);
        fill_rect(static_cast<int16_t>(ox + 2 * module_size), static_cast<int16_t>(oy + 2 * module_size),
                  3 * module_size, 3 * module_size, kColorBlack);
    };

    draw_finder(qr_origin_x, qr_origin_y);
    draw_finder(qr_origin_x + (qr_size - 7) * module_size, qr_origin_y);
    draw_finder(qr_origin_x, qr_origin_y + (qr_size - 7) * module_size);

    for (int row = 0; row < qr_size; ++row) {
        for (int col = 0; col < qr_size; ++col) {
            const bool in_top_left = row < 7 && col < 7;
            const bool in_top_right = row < 7 && col >= qr_size - 7;
            const bool in_bottom_left = row >= qr_size - 7 && col < 7;
            if (in_top_left || in_top_right || in_bottom_left) {
                continue;
            }

            const uint32_t pattern = static_cast<uint32_t>((row * 17) ^ (col * 29) ^ 0x5A);
            if ((pattern & 0x3U) == 0U) {
                fill_rect(static_cast<int16_t>(qr_origin_x + col * module_size),
                          static_cast<int16_t>(qr_origin_y + row * module_size),
                          module_size,
                          module_size,
                          kColorBlack);
            }
        }
    }

    draw_text(30, 204, "PLACEHOLDER IMAGE", kColorText, 2);
    draw_chip(16, 232, 42, "VIEW", selected == 0);
    draw_chip(65, 232, 42, "SAVE", selected == 1);
    draw_chip(114, 232, 42, "SEND", selected == 2);
}

void draw_voltage_page(uint8_t selected)
{
    draw_header(Page::Voltage);

    const float latest_voltage = adc_get_latest_voltage();
    char        voltage_text[24] = {};
    std::snprintf(voltage_text, sizeof(voltage_text), "VOLT: %.2fV", latest_voltage);
    draw_text(16, 54, voltage_text, kColorText, 2);

    const int16_t graph_x = 12;
    const int16_t graph_y = 88;
    const int16_t graph_w = 148;
    const int16_t graph_h = 128;

    fill_rect(graph_x, graph_y, graph_w, graph_h, kColorPanel);
    draw_rect(graph_x, graph_y, graph_w, graph_h, kColorAccentSoft);

    for (int i = 1; i < 4; ++i) {
        const int16_t y = static_cast<int16_t>(graph_y + (graph_h * i) / 4);
        fill_rect(graph_x + 1, y, graph_w - 2, 1, kColorPanelAlt);
    }

    const size_t history_count = adc_copy_voltage_history(graph_history, kGraphHistoryCapacity);
    if (history_count >= 2) {
        float min_v = graph_history[0];
        float max_v = graph_history[0];

        for (size_t i = 1; i < history_count; ++i) {
            min_v = std::min(min_v, graph_history[i]);
            max_v = std::max(max_v, graph_history[i]);
        }

        if ((max_v - min_v) < 0.2f) {
            max_v += 0.1f;
            min_v -= 0.1f;
        }

        const float padded_min = min_v - 0.05f;
        const float padded_max = max_v + 0.05f;
        const float span = std::max(0.1f, padded_max - padded_min);

        for (size_t i = 1; i < history_count; ++i) {
            const int x0 = graph_x + 1 + static_cast<int>((i - 1) * (graph_w - 3) / (history_count - 1));
            const int x1 = graph_x + 1 + static_cast<int>(i * (graph_w - 3) / (history_count - 1));
            const int y0 = graph_y + graph_h - 2
                           - static_cast<int>(((graph_history[i - 1] - padded_min) / span) * (graph_h - 4));
            const int y1 = graph_y + graph_h - 2
                           - static_cast<int>(((graph_history[i] - padded_min) / span) * (graph_h - 4));

            draw_line(x0, std::clamp(y0, graph_y + 1, graph_y + graph_h - 2),
                      x1, std::clamp(y1, graph_y + 1, graph_y + graph_h - 2),
                      kColorGraph);
        }

        char range_text[24] = {};
        std::snprintf(range_text, sizeof(range_text), "%.2f-%.2fV", padded_min, padded_max);
        draw_text(24, 226, range_text, kColorTextMuted, 2);
    } else {
        draw_text(32, 140, "WAIT ADC DATA", kColorTextMuted, 2);
    }

    draw_chip(16, 252, 42, "AUTO", selected == 0);
    draw_chip(65, 252, 42, "HOLD", selected == 1);
    draw_chip(114, 252, 42, "CLR", selected == 2);
}

void draw_settings_page(uint8_t selected)
{
    (void)selected;
    draw_header(Page::Settings);

    fill_rect(16, 68, 140, 160, kColorPanel);
    draw_rect(16, 68, 140, 160, kColorAccentSoft);
    draw_text(36, 110, "EMPTY PAGE", kColorText, 2);
    draw_text(30, 138, "LVGL READY SLOT", kColorTextMuted, 2);
    draw_text(26, 166, "ADD ITEMS LATER", kColorTextMuted, 2);
}

void render_page()
{
    fill_screen(kColorBg);

    switch (ui_state.current_page) {
        case Page::Qr:
            draw_qr_placeholder(ui_state.selected_index[static_cast<size_t>(Page::Qr)]);
            break;
        case Page::Voltage:
            draw_voltage_page(ui_state.selected_index[static_cast<size_t>(Page::Voltage)]);
            break;
        case Page::Settings:
            draw_settings_page(ui_state.selected_index[static_cast<size_t>(Page::Settings)]);
            break;
        default:
            break;
    }

    draw_footer();
}

void st7789_select()
{
    HAL_GPIO_WritePin(tft_nss_GPIO_Port, tft_nss_Pin, GPIO_PIN_RESET);
}

void st7789_deselect()
{
    HAL_GPIO_WritePin(tft_nss_GPIO_Port, tft_nss_Pin, GPIO_PIN_SET);
}

void clear_spi4_dma_done()
{
    if (spi4_tx_dma_done == nullptr) {
        return;
    }

    while (osSemaphoreAcquire(spi4_tx_dma_done, 0) == osOK) {
    }
}

bool init_spi4_tx_dma()
{
    if (spi4_tx_dma_ready) {
        return true;
    }

    if (spi4_tx_dma_done == nullptr) {
        spi4_tx_dma_done = osSemaphoreNew(1, 0, nullptr);
        if (spi4_tx_dma_done == nullptr) {
            std::printf("[lcd] SPI4 DMA semaphore create failed\r\n");
            return false;
        }
    }

    __HAL_RCC_DMA1_CLK_ENABLE();

    spi4_tx_dma.Instance = DMA1_Stream1;
    spi4_tx_dma.Init.Request = DMA_REQUEST_SPI4_TX;
    spi4_tx_dma.Init.Direction = DMA_MEMORY_TO_PERIPH;
    spi4_tx_dma.Init.PeriphInc = DMA_PINC_DISABLE;
    spi4_tx_dma.Init.MemInc = DMA_MINC_ENABLE;
    spi4_tx_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    spi4_tx_dma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    spi4_tx_dma.Init.Mode = DMA_NORMAL;
    spi4_tx_dma.Init.Priority = DMA_PRIORITY_HIGH;
    spi4_tx_dma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    if (HAL_DMA_Init(&spi4_tx_dma) != HAL_OK) {
        std::printf("[lcd] SPI4 TX DMA init failed\r\n");
        return false;
    }

    __HAL_LINKDMA(&hspi4, hdmatx, spi4_tx_dma);

    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    HAL_NVIC_SetPriority(SPI4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SPI4_IRQn);

    spi4_tx_dma_ready = true;
    std::printf("[lcd] SPI4 TX DMA ready (DMA1_Stream1)\r\n");
    return true;
}

bool spi4_transmit_dma_blocking(const uint8_t *data, uint16_t length)
{
    if (data == nullptr || length == 0U) {
        return true;
    }

    if (!init_spi4_tx_dma()) {
        return false;
    }

    clear_spi4_dma_done();
    spi4_tx_dma_error = false;

    const HAL_StatusTypeDef start_status = HAL_SPI_Transmit_DMA(&hspi4, data, length);
    if (start_status != HAL_OK) {
        std::printf("[lcd] SPI4 DMA start failed: status=%d err=0x%08lx\r\n",
                    static_cast<int>(start_status),
                    static_cast<unsigned long>(hspi4.ErrorCode));
        return false;
    }

    const osStatus_t wait_status = osSemaphoreAcquire(spi4_tx_dma_done, kSpiDmaTimeoutMs);
    if (wait_status != osOK || spi4_tx_dma_error || hspi4.ErrorCode != HAL_SPI_ERROR_NONE) {
        HAL_SPI_Abort(&hspi4);
        std::printf("[lcd] SPI4 DMA transfer failed: wait=%d dma_err=%u spi_err=0x%08lx\r\n",
                    static_cast<int>(wait_status),
                    static_cast<unsigned>(spi4_tx_dma_error),
                    static_cast<unsigned long>(hspi4.ErrorCode));
        return false;
    }

    return true;
}

void st7789_write_command(uint8_t command)
{
    HAL_GPIO_WritePin(tft_rs_GPIO_Port, tft_rs_Pin, GPIO_PIN_RESET);
    st7789_select();
    HAL_SPI_Transmit(&hspi4, &command, 1, HAL_MAX_DELAY);
    st7789_deselect();
}

void st7789_write_data(const uint8_t *data, uint16_t length)
{
    if (data == nullptr || length == 0) {
        return;
    }

    HAL_GPIO_WritePin(tft_rs_GPIO_Port, tft_rs_Pin, GPIO_PIN_SET);
    st7789_select();
    HAL_SPI_Transmit(&hspi4, const_cast<uint8_t *>(data), length, HAL_MAX_DELAY);
    st7789_deselect();
}

void st7789_write_u8(uint8_t command, uint8_t data)
{
    st7789_write_command(command);
    st7789_write_data(&data, 1);
}

void st7789_set_address_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    x0 = static_cast<uint16_t>(x0 + kPanelXOffset);
    x1 = static_cast<uint16_t>(x1 + kPanelXOffset);
    y0 = static_cast<uint16_t>(y0 + kPanelYOffset);
    y1 = static_cast<uint16_t>(y1 + kPanelYOffset);

    uint8_t data[4] = {};

    st7789_write_command(0x2A);
    data[0] = static_cast<uint8_t>(x0 >> 8);
    data[1] = static_cast<uint8_t>(x0 & 0xFF);
    data[2] = static_cast<uint8_t>(x1 >> 8);
    data[3] = static_cast<uint8_t>(x1 & 0xFF);
    st7789_write_data(data, sizeof(data));

    st7789_write_command(0x2B);
    data[0] = static_cast<uint8_t>(y0 >> 8);
    data[1] = static_cast<uint8_t>(y0 & 0xFF);
    data[2] = static_cast<uint8_t>(y1 >> 8);
    data[3] = static_cast<uint8_t>(y1 & 0xFF);
    st7789_write_data(data, sizeof(data));

    st7789_write_command(0x2C);
}

void st7789_reset()
{
    HAL_GPIO_WritePin(tft_rst_GPIO_Port, tft_rst_Pin, GPIO_PIN_RESET);
    osDelay(20);
    HAL_GPIO_WritePin(tft_rst_GPIO_Port, tft_rst_Pin, GPIO_PIN_SET);
    osDelay(120);
}

void st7789_init()
{
    static const uint8_t porch[] = {0x0C, 0x0C, 0x00, 0x33, 0x33};
    static const uint8_t pwctrl[] = {0xA4, 0xA1};
    static const uint8_t positive_gamma[] = {0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54,
                                             0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23};
    static const uint8_t negative_gamma[] = {0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44,
                                             0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23};

    st7789_reset();
    st7789_write_command(0x01);
    osDelay(150);

    st7789_write_u8(0x36, 0x00);
    st7789_write_u8(0x3A, 0x05);

    st7789_write_command(0xB2);
    st7789_write_data(porch, sizeof(porch));

    st7789_write_u8(0xB7, 0x35);
    st7789_write_u8(0xBB, 0x19);
    st7789_write_u8(0xC0, 0x2C);
    st7789_write_u8(0xC2, 0x01);
    st7789_write_u8(0xC3, 0x12);
    st7789_write_u8(0xC4, 0x20);
    st7789_write_u8(0xC6, 0x0F);

    st7789_write_command(0xD0);
    st7789_write_data(pwctrl, sizeof(pwctrl));

    st7789_write_command(0xE0);
    st7789_write_data(positive_gamma, sizeof(positive_gamma));

    st7789_write_command(0xE1);
    st7789_write_data(negative_gamma, sizeof(negative_gamma));

    st7789_write_command(0x21);
    st7789_write_command(0x11);
    osDelay(120);
    st7789_write_command(0x29);
    osDelay(20);
}

void st7789_flush_framebuffer()
{
    if (!init_spi4_tx_dma()) {
        return;
    }

    auto *bytes = reinterpret_cast<uint8_t *>(framebuffer);
    const uint32_t total_bytes = kFramePixelCount * sizeof(uint16_t);

    st7789_set_address_window(0, 0, kPanelWidth - 1, kPanelHeight - 1);

    HAL_GPIO_WritePin(tft_rs_GPIO_Port, tft_rs_Pin, GPIO_PIN_SET);
    st7789_select();

    bool ok = true;
    for (uint32_t offset = 0; offset < total_bytes; offset += kSpiDmaChunkBytes) {
        const uint16_t chunk = static_cast<uint16_t>(std::min<uint32_t>(kSpiDmaChunkBytes, total_bytes - offset));
        if (!spi4_transmit_dma_blocking(bytes + offset, chunk)) {
            ok = false;
            break;
        }
    }

    st7789_deselect();

    if (!ok) {
        std::printf("[lcd] ST7789 framebuffer flush aborted\r\n");
    }
}

void set_backlight_percent(uint32_t percent)
{
    const uint32_t clamped = std::min<uint32_t>(percent, 100);
    const uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim15);
    uint32_t pulse = (period * clamped) / 100U;

    if (!kBacklightActiveHigh) {
        pulse = period - pulse;
    }

    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, pulse);
}

void init_backlight()
{
    HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
    set_backlight_percent(0);
}

} // namespace

extern "C" void lcd_dma1_stream1_irq_handler(void)
{
    HAL_DMA_IRQHandler(&spi4_tx_dma);
}

extern "C" void lcd_spi4_irq_handler(void)
{
    HAL_SPI_IRQHandler(&hspi4);
}

extern "C" void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi4 && spi4_tx_dma_done != nullptr) {
        osSemaphoreRelease(spi4_tx_dma_done);
    }
}

extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi4 && spi4_tx_dma_done != nullptr) {
        spi4_tx_dma_error = true;
        osSemaphoreRelease(spi4_tx_dma_done);
    }
}

extern "C" void lcdTask(void *)
{
    ui_input_init();
    init_backlight();

    osDelay(30);
    st7789_init();
    set_status("SCREEN READY");
    render_page();
    st7789_flush_framebuffer();
    set_backlight_percent(kBacklightPercent);

    std::printf("[lcd] ST7789 ready (%ux%u, xoff=%u, yoff=%u)\r\n",
                kPanelWidth,
                kPanelHeight,
                kPanelXOffset,
                kPanelYOffset);

    uint32_t last_voltage_refresh = osKernelGetTickCount();

    for (;;) {
        bool         dirty = false;
        UiInputEvent event = UiInputEvent::NextItem;

        if (ui_input_poll(event, 40)) {
            apply_input(event);
            dirty = true;
        }

        const uint32_t now = osKernelGetTickCount();
        if (ui_state.current_page == Page::Voltage && (now - last_voltage_refresh) >= kVoltagePageRefreshMs) {
            last_voltage_refresh = now;
            dirty = true;
        }

        if (!dirty) {
            continue;
        }

        render_page();
        st7789_flush_framebuffer();
    }
}
