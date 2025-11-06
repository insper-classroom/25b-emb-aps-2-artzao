#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdint.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"


// ========================= PINS =========================


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

#define GPIO_BTN1        2
#define GPIO_BTN2        3
#define GPIO_BTN3        6
#define GPIO_BTN4        7


// ========================= Key / command types =========================


typedef enum {
    KEY_LMB  = 1,
    KEY_RMB  = 2,
    KEY_SHIFT= 3,
    KEY_CTRL = 4,
} key_type_t;

// Internal: logical buttons
typedef enum {
    BTN1 = 1,
    BTN2 = 2,
    BTN3 = 3,
    BTN4 = 4,
} btn_id_t;

// Button event sent from ISR -> task
typedef struct {
    uint8_t btn;    // 1..4
    uint8_t press;  // 1=press, 0=release
} btn_event_t;


// ========================= Queues ================================


typedef struct {
    int axis;
    int value;
} read_adc_t;

static QueueHandle_t queueADC;
static QueueHandle_t queueBTN;


// ========================= Helpers ===============================


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


// ========================= USB/Host I/O ===========================


static inline void usb_send_axis(int axis, int16_t value) {
    putchar_raw(0xFF);
    putchar_raw((uint8_t)(axis ? 1 : 0));
    putchar_raw((uint8_t)(value & 0xFF));
    putchar_raw((uint8_t)((value >> 8) & 0xFF));
}

// BUTTON PACKET: 0xFE, key_type, flags, checksum
static inline uint8_t sum8(uint8_t a, uint8_t b) { return (uint8_t)((a + b) & 0xFF); }

static inline void usb_send_button(key_type_t key, uint8_t press) {
    const uint8_t hdr = 0xFE;
    const uint8_t flags = (press ? 1u : 0u);
    const uint8_t csum = sum8((uint8_t)key, flags);
    putchar_raw(hdr);
    putchar_raw((uint8_t)key);
    putchar_raw(flags);
    putchar_raw(csum);
}

// ========================= ADC / Joystick =========================

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
            usb_send_axis(read.axis, (int16_t)read.value);
        }
    }
}


// ========================= Buttons (GPIO + IRQ) ====================


// Map GPIO
static inline btn_id_t gpio_to_btn(uint gpio) {
    switch (gpio) {
        case GPIO_BTN1: return BTN1;
        case GPIO_BTN2: return BTN2;
        case GPIO_BTN3: return BTN3;
        case GPIO_BTN4: return BTN4;
        default:        return 0;
    }
}

// Shared ISR callback: active-low buttons
static void btn_isr(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    btn_id_t id = gpio_to_btn(gpio);
    if (!id) return;

    // FALL = press, RISE = release (because pull-up, active-low)
    if (events & GPIO_IRQ_EDGE_FALL) {
        btn_event_t ev = { .btn = (uint8_t)id, .press = 1 };
        xQueueSendFromISR(queueBTN, &ev, &xHigherPriorityTaskWoken);
    }
    if (events & GPIO_IRQ_EDGE_RISE) {
        btn_event_t ev = { .btn = (uint8_t)id, .press = 0 };
        xQueueSendFromISR(queueBTN, &ev, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void buttons_init(void) {
    const uint btn_pins[] = { GPIO_BTN1, GPIO_BTN2, GPIO_BTN3, GPIO_BTN4 };
    for (size_t i = 0; i < 4; i++) {
        gpio_init(btn_pins[i]);
        gpio_set_dir(btn_pins[i], GPIO_IN);
        gpio_pull_up(btn_pins[i]);
    }

    // Register one shared callback, enable edges
    gpio_set_irq_enabled_with_callback(btn_pins[0],
        GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &btn_isr);
    for (size_t i = 1; i < 4; i++) {
        gpio_set_irq_enabled(btn_pins[i], GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
    }
}

// Task that converts button events → key packets
static void task_buttons(void *p) {
    (void)p;
    for (;;) {
        btn_event_t ev;
        if (xQueueReceive(queueBTN, &ev, portMAX_DELAY) == pdTRUE) {
            // Mapping based on the earlier pattern:
            // BTN1 → LMB, BTN2 → RMB, BTN3 → SHIFT, BTN4 → CTRL
            key_type_t key = KEY_LMB;
            switch (ev.btn) {
                case BTN1: key = KEY_LMB;  break;
                case BTN2: key = KEY_RMB;  break;
                case BTN3: key = KEY_SHIFT;break;
                case BTN4: key = KEY_CTRL; break;
                default: continue;
            }
            usb_send_button(key, ev.press);
        }
    }
}


// ========================= Main ================================


int main(void) {
    stdio_init_all();
    initialize_adc();

    queueADC = xQueueCreate(Q_LEN, sizeof(read_adc_t));
    queueBTN = xQueueCreate(8, sizeof(btn_event_t));

    buttons_init();

    xTaskCreate(task_x, "task_x", 2048, NULL, 2, NULL);
    xTaskCreate(task_y, "task_y", 2048, NULL, 2, NULL);
    xTaskCreate(task_usb, "task_usb", 2048, NULL, 2, NULL);
    xTaskCreate(task_buttons, "task_buttons", 1024, NULL, 2, NULL);

    vTaskStartScheduler();

    while (true) {
        tight_loop_contents();
    }
}
