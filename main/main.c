#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include <esp_log.h>
#include <math.h>

#include "neopixel.h"

// I2S_NUM: I2S peripheral to use (0 or 1)
#define I2S_NUM           (0)

#define DATA_PIN 23
// LEDS: The number of LEDs in the NEOPixel chain
#define LEDS              (64)


#define LOG_TAG "neoi2smain"

float centre_rx, centre_ry,centre_gx, centre_gy,centre_bx, centre_by;
float r_dx, r_dy, g_dx, g_dy, b_dx, b_dy;

void app_main()
{
    neo_display_t t = {
            .port = I2S_NUM,
            .pin = DATA_PIN,
            .width = 8,
            .height = 8,
    };

    centre_rx = 2.0;
    centre_ry = 2.0;
    centre_gx = 6.0;
    centre_gy = 6.0;
    centre_bx = 3.0;
    centre_by = 5.0;

    r_dx = 0.01;
    r_dy = 0.03;

    g_dx = -0.02;
    g_dy = 0.02;

    b_dx = 0.03;
    b_dy = 0.05;

    neo_init(&t);
    neo_clear(&t);

#define square(x) ((x) * (x))
#define dist(centrex, centrey, pxx, pxy) (square(((centrex)-(pxx))/8) + square(((centrey) - (pxy))/8))
#define brightness(centrex, centrey, pxx, pxy) (0xff & (int)(128 / (1 + 64 * dist(centrex, centrey, pxx, pxy))))

#define advance(centrex, centrey, dx, dy) do {\
    centrex += dx; \
    if(centrex >= 8) { \
        centrex = 8 - (centrex-8); \
        dx = -dx;\
    }\
    if(centrex <= 0) {\
        centrex = -centrex;\
        dx = -dx;\
    }\
    centrey += dy;\
    if(centrey >= 8) {\
        centrey = 8 - (centrey-8);\
        dy = -dy;\
    }\
    if(centrey <= 0) {\
        centrey = -centrey;\
        dy = -dy;\
    }\
}while(0)

    while(1) {
        for(int x = 0; x < t.width; x ++) {
            for(int y = 0; y < t.width; y++) {
                neo_set(&t, x, y,
                    brightness(centre_rx, centre_ry, x, y) << 16 | brightness(centre_gx, centre_gy, x, y) << 8 | brightness(centre_bx, centre_by, x, y)
                );
            }
        }

        advance(centre_rx, centre_ry, r_dx, r_dy);
        advance(centre_gx, centre_gy, g_dx, g_dy);
        advance(centre_bx, centre_by, b_dx, b_dy);

        neo_draw(&t);
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}
