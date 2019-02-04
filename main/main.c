#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include <esp_log.h>

// SAMPLE_RATE: Sample rate is set to get about 800MHz bit rate
#define SAMPLE_RATE       (90000)
// I2S_NUM: I2S peripheral to use (0 or 1)
#define I2S_NUM           (0)
// BITS_PER_BIT: When encoding to bits in the i2s bus, how many i2s bits are required per color bit
#define BITS_PER_BIT      (4)
// COLOR_DEPTH: How many bits represent an individual colour
#define COLOR_DEPTH       (8)
// COLORS: How many colors per LED
#define COLORS            (3)
// LEDS: The number of LEDs in the NEOPixel chain
#define LEDS              (64)

#define DATA_SIZE_BYTES ((BITS_PER_BIT * COLORS * LEDS))
// RESET_SIZE_BYTES: a fairly arbitrary "large enough" number to invoke a reset of the neopixels
#define RESET_SIZE_BYTES (24)
#define BUFFER_SIZE_BYTES (DATA_SIZE_BYTES + RESET_SIZE_BYTES)

// A low is represented by one high bit followed by three low
#define LOW  0x08
// A high is represented by two high bits followed by two lows (also 0x0E works here)
#define HIGH 0x0C

// The driver doesn't support 8 bits per sample, so use 16.
#define BITS_PER_SAMPLE I2S_BITS_PER_SAMPLE_16BIT

#define LOG_TAG "neoi2smain"

// How many bytes long is a color when it has been packed
#define COLOR_WIDTH ((COLOR_DEPTH * BITS_PER_BIT) >> 3)

// graphics_buffer is the encoded data to be written to the I2S dma buffer.
uint8_t * graphics_buffer;

// set_color sets a color of a pixel.
// color is expected to be in RGB format 0xRRGGBB
// px is the index of the pixel to update
static inline void set_color(uint8_t * buf, uint16_t px, uint32_t color) {
    uint8_t offset = 0;
    for(int bit = 0; COLOR_DEPTH * COLORS > (bit+1) ; bit += 2) {
        int px_off = (px * COLOR_WIDTH * COLORS);
        // WS2812 expects data in GGRRBB format
        int col_byte_off = (COLORS - 1 - (offset / COLOR_WIDTH)) * COLOR_WIDTH;
        int bit_off = offset % COLOR_WIDTH;
        int tmp_off = px_off + col_byte_off + bit_off;

        buf[tmp_off]  = 0x00;
        buf[tmp_off]  = ((color & (0x1 << bit)) ? HIGH : LOW);
        buf[tmp_off] |= ((color & (0x1 << (bit + 1))) ? HIGH : LOW)  << 4;

        offset ++;
    }
}

// draw writes the graphics_buffer to the i2s dma buffer
static inline void draw() {
    size_t i2s_bytes_written;
    // FIXME: At the moment, if we don't clear the dma buffer, then the LEDs just get progressively brighter...
    i2s_zero_dma_buffer(I2S_NUM);
    esp_err_t err = i2s_write(I2S_NUM, graphics_buffer, BUFFER_SIZE_BYTES, &i2s_bytes_written, 100);
    ESP_LOGI(LOG_TAG, "Drawing. Wrote %d bytes. Result: %d", i2s_bytes_written, err);
}

// setup_buffer allocates the graphics_buffer and fills it with neopixel zero values
static void setup_buffer()
{
    graphics_buffer = malloc(BUFFER_SIZE_BYTES);

    for(int i = 0; i < BUFFER_SIZE_BYTES; i ++) {
        graphics_buffer[i] = 0x00;
    }

    // fill with "zeros"
    for(int i = 0; i < DATA_SIZE_BYTES; i ++) {
        graphics_buffer[i] = (LOW << 4) | LOW;
    }

    // put some empty, actually zero, values on the end for a reset.
    for(int i = 0; i < RESET_SIZE_BYTES; i++) {
        graphics_buffer[DATA_SIZE_BYTES + i] = 0x00;
    }

    i2s_zero_dma_buffer(I2S_NUM);
}

// fill_with_pattern fills the buffer with a test pattern
void fill_with_pattern(uint8_t offset) {
    ESP_LOGI(LOG_TAG, "Filling with pattern, offset %d.",offset);
    for(int i = 0; i < LEDS; i++) {
        set_color(graphics_buffer, i, 0x00000A << (8 * ((i+offset) % 3)));
    }
}


void app_main()
{
    ESP_LOGI(LOG_TAG, "Sample rate: %d. BufferSize: %d bytes", SAMPLE_RATE, BUFFER_SIZE_BYTES);

    i2s_config_t i2s_config = {
            .mode = I2S_MODE_MASTER | I2S_MODE_TX,
            .sample_rate = SAMPLE_RATE,
            .bits_per_sample = BITS_PER_SAMPLE,
            // Although we're only dealing with one data stream, we must use _FMT_RIGHT_LEFT
            // if we use any of the others, the signal is sent twice.
            .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
            .communication_format = I2S_COMM_FORMAT_I2S,
            .dma_buf_count = 2,
            // dma_buf_len appears to be the number of samples in the buffer
            .dma_buf_len = BUFFER_SIZE_BYTES / 2,
            .use_apll = true,
            .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    };

    i2s_pin_config_t pin_config = {
            .data_out_num = 23,
            // Not used
            .bck_io_num = -1,
            .ws_io_num = -1,
            .data_in_num = -1
    };

    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM, &pin_config);

    setup_buffer();

    uint8_t offset = 0;
    while (1) {
            fill_with_pattern(offset++);
            vTaskDelay(500/portTICK_PERIOD_MS);
            draw();
    }

}
