#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdint.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

#define PIN_X     27
#define PIN_Y     26
#define CHANNEL_X      1
#define CHANNEL_Y      0

#define AVG_WINDOW       8
#define TWEAK_SAMPLES    200
#define DEADZONE_LIMIT   30
#define SAMPLE_INTERVAL  10
#define Q_LEN            32
#define SENS_NUM         35
#define SENS_DEN         100

typedef struct {
    int axis;
    int value;
} read_adc_t;

static QueueHandle_t queueADC;

static inline int limit(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline int map_adc(int mean12bits, uint16_t ref12bits) {
    int centralized = mean12bits - (int)ref12bits;

    int64_t num = (int64_t)centralized * 255 * SENS_NUM;
    int64_t den = 2048LL * SENS_DEN;

    int resultado = (centralized >= 0)
        ? (int)((num + den / 2) / den)
        : (int)((num - den / 2) / den);

    if (resultado > -DEADZONE_LIMIT && resultado < DEADZONE_LIMIT)
        resultado = 0;

    return limit(resultado, -255, 255);
}

static inline void usb_send(int axis, int16_t value) {
    putchar_raw(0xFF);
    putchar_raw((uint8_t)(axis ? 1 : 0));
    putchar_raw((uint8_t)(value & 0xFF));
    putchar_raw((uint8_t)((value >> 8) & 0xFF));
}

static void initialize_adc(void) {
    adc_init();
    adc_gpio_init(PIN_X);
    adc_gpio_init(PIN_Y);
}

static uint16_t tweak_read(uint channelint) {
    uint32_t sum = 0;
    for (int i = 0; i < TWEAK_SAMPLES; i++) {
        adc_select_input(channelint);
        (void)adc_read();
        sum += adc_read();
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return (uint16_t)(sum / TWEAK_SAMPLES);
}

static void task_x(void *p) {
    const int axis = 1;
    const uint channelint = CHANNEL_X;
    uint16_t ref_center = tweak_read(channelint);  // was ref_x

    int buffer[AVG_WINDOW] = {0};
    int sum = 0, idx = 0, filled = 0;

    for (;;) {
        adc_select_input(channelint);
        (void)adc_read();
        uint16_t dataread = adc_read();

        sum -= buffer[idx];
        buffer[idx] = (int)dataread;
        sum += buffer[idx];
        idx = (idx + 1) % AVG_WINDOW;
        if (filled < AVG_WINDOW) filled++;

        int mean = sum / (filled ? filled : 1);
        int value = -map_adc(mean, ref_center);  // pass local

        read_adc_t read = { .axis = axis, .value = value };
        xQueueSend(queueADC, &read, 0);

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL));
    }
}


static void task_y(void *p) {
    const int axis = 0;
    const uint channelint = CHANNEL_Y;
    uint16_t ref_center = tweak_read(channelint);  // was ref_y

    int buffer[AVG_WINDOW] = {0};
    int sum = 0, idx = 0, filled = 0;

    for (;;) {
        adc_select_input(channelint);
        (void)adc_read();
        uint16_t dataread = adc_read();

        sum -= buffer[idx];
        buffer[idx] = (int)dataread;
        sum += buffer[idx];
        idx = (idx + 1) % AVG_WINDOW;
        if (filled < AVG_WINDOW) filled++;

        int mean = sum / (filled ? filled : 1);
        int value = map_adc(mean, ref_center);  // keep Y inverted fix

        read_adc_t read = { .axis = axis, .value = value };
        xQueueSend(queueADC, &read, 0);

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL));
    }
}

static void task_usb(void *p) {
    (void)p;
    for (;;) {
        read_adc_t read;
        if (xQueueReceive(queueADC, &read, portMAX_DELAY) == pdTRUE) {
            usb_send(read.axis, (int16_t)read.value);
        }
    }
}

int main(void) {
    stdio_init_all();
    initialize_adc();

    queueADC = xQueueCreate(Q_LEN, sizeof(read_adc_t));
    xTaskCreate(task_x, "task_x", 2048, NULL, 2, NULL);
    xTaskCreate(task_y, "task_y", 2048, NULL, 2, NULL);
    xTaskCreate(task_usb, "task_usb", 2048, NULL, 2, NULL);

    vTaskStartScheduler();

    while (true) {
        tight_loop_contents();
    }
}
