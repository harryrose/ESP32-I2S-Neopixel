//
// Created by harry on 08/02/19.
//
#include "freertos/FreeRTOS.h"
#include "neopixel.h"
#include <esp_log.h>
#include <stdio.h>
#include <string.h>

#define NEO_LOG_TAG "neopixel"

// SAMPLE_RATE: Sample rate is set to get about 800MHz bit rate
#define SAMPLE_RATE       (90000)
// COLOR_DEPTH: How many bits represent an individual colour
#define COLOR_DEPTH       (8)
// BITS_PER_BIT: When encoding to bits in the i2s bus, how many i2s bits are required per color bit
#define BITS_PER_BIT      (4)
// COLORS: How many colors per LED
#define COLORS            (3)
// The driver doesn't support 8 bits per sample, so use 16.
#define BITS_PER_SAMPLE I2S_BITS_PER_SAMPLE_16BIT
// How many bytes long is a color when it has been packed
#define COLOR_WIDTH ((COLOR_DEPTH * BITS_PER_BIT) >> 3)
// A low is represented by one high bit followed by three low
#define LOW  0x08
// A high is represented by two high bits followed by two lows (also 0x0E works here)
#define HIGH 0x0E

void neo_init(neo_display_t *d) {
    d->buffer_size = d->width * d->height * COLORS * BITS_PER_BIT + 128;
    d->buffer = (uint8_t *)malloc(d->buffer_size);
    memset(d->buffer, 0, d->buffer_size);
    i2s_config_t i2s_config = {
            .mode = I2S_MODE_MASTER | I2S_MODE_TX,
            .sample_rate = SAMPLE_RATE,
            .bits_per_sample = BITS_PER_SAMPLE,
            // Although we're only dealing with one data stream, we must use _FMT_RIGHT_LEFT
            // if we use any of the others, the signal is sent twice.
            .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
            .communication_format = I2S_COMM_FORMAT_I2S,
            .dma_buf_count = 4,
            // dma_buf_len appears to be the number of samples in the buffer
            .dma_buf_len = d->buffer_size / 2,
            .use_apll = true,
            .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    };

    i2s_pin_config_t pin_config = {
            .data_out_num = d->pin,
            // Not used
            .bck_io_num = -1,
            .ws_io_num = -1,
            .data_in_num = -1
    };


    i2s_driver_install(d->port, &i2s_config, 0, NULL);
    i2s_set_pin(d->port, &pin_config);
    i2s_zero_dma_buffer(d->port);
}

void neo_free(neo_display_t *d) {
    free(d->buffer);
}

inline void neo_set_absolute(neo_display_t *d, uint16_t px, uint32_t color) {
    uint8_t offset = 0;
    for(int bit = 0; COLOR_DEPTH * COLORS > (bit+1) ; bit += 2) {
        int px_off = (px * COLOR_WIDTH * COLORS);
        // WS2812 expects data in GGRRBB format
        int col_byte_off = (COLORS - 1 - (offset / COLOR_WIDTH)) * COLOR_WIDTH;
        int bit_off = offset % COLOR_WIDTH;
        int tmp_off = px_off + col_byte_off + bit_off;

        d->buffer[tmp_off]  = 0x00;
        d->buffer[tmp_off]  = ((color & (0x1 << bit)) ? HIGH : LOW);
        d->buffer[tmp_off] |= ((color & (0x1 << (bit + 1))) ? HIGH : LOW)  << 4;

        offset ++;
    }
}

void neo_clear(neo_display_t *d){
    for(int i = 0; i < d->width * d->height; i++) {
        neo_set_absolute(d, i, 0);
    }
}

inline void neo_set(neo_display_t *d, uint16_t x, uint16_t y, uint32_t color){
    neo_set_absolute(d, y * d->width + x, color);
}

void neo_draw(neo_display_t *d){
    size_t i2s_bytes_written;
    // FIXME: At the moment, if we don't clear the dma buffer, then the LEDs just get progressively brighter...
    i2s_zero_dma_buffer(d->port);
    esp_err_t err = i2s_write(d->port, d->buffer, d->buffer_size, &i2s_bytes_written, 100);
}
