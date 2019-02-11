//
// Created by harry on 08/02/19.
//

#ifndef ESP32_I2S_NEOPIXEL_NEOPIXEL_H
#define ESP32_I2S_NEOPIXEL_NEOPIXEL_H

#include <driver/i2s.h>
#include <stdint.h>
#include <stdio.h>

typedef struct {
    i2s_port_t port;
    uint16_t width;
    uint16_t height;
    int pin;
    uint8_t *buffer;
    uint16_t buffer_size;
} neo_display_t;

void neo_init(neo_display_t *);
void neo_free(neo_display_t *);
void neo_clear(neo_display_t *);
void neo_set(neo_display_t *,uint16_t, uint16_t,uint32_t);
void neo_draw(neo_display_t *);


#endif //ESP32_I2S_NEOPIXEL_NEOPIXEL_H
