#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"

// ========================= PINOS =========================

#define GPIO_BTN_VERDE     15
#define GPIO_BTN_AZUL      14
#define GPIO_BTN_AMARELO   13
#define GPIO_BTN_VERMELHO  12

#define GPIO_JOYSTICK_Y    26  // ADC0
#define GPIO_JOYSTICK_X    27  // ADC1
#define ADC_CHANNEL_Y      0
#define ADC_CHANNEL_X      1

#define I2C_PORT           i2c0
#define I2C_SDA_PIN        4
#define I2C_SCL_PIN        5
#define I2C_FREQ           400000

#define UART_ID            uart0
#define UART_BAUD_RATE     115200
#define UART_TX_PIN        0
#define UART_RX_PIN        1

// ========================= CONFIG =========================

#define CALIBRACAO_AMOSTRAS 100
#define AVG_BUFFER_SIZE    3
#define JOYSTICK_DELAY_MS  10

#define MARGEM_RUIDO       15
#define MARGEM_ATIVA       40

#define BUTTON_DELAY_MS    50
#define BUTTON_DEBOUNCE_MS 100
#define IMU_DELAY_MS       30
#define QUEUE_SIZE_INPUT   32
#define QUEUE_SIZE_IMU     16

#define PRIORIDADE_UART    3
#define PRIORIDADE_INPUT   2
#define PRIORIDADE_IMU     1

#define TIMEOUT_QUEUE_CRITICO pdMS_TO_TICKS(10)

// ========================= PROTOCOLO =========================

#define EVENTO_TECLA       0
#define EVENTO_MOUSE       1

#define HEADER_TECLA       0x54
#define HEADER_MOUSE       0x4D

#define TECLA_W            0x57
#define TECLA_A            0x41
#define TECLA_S            0x53
#define TECLA_D            0x44
#define TECLA_SPACE        0x20
#define TECLA_E            0x45
#define TECLA_TAB          0x09
#define TECLA_CTRL         0x11

#define BOTAO_VERDE_TECLA      TECLA_TAB
#define BOTAO_AZUL_TECLA       TECLA_SPACE
#define BOTAO_AMARELO_TECLA    TECLA_E
#define BOTAO_VERMELHO_TECLA   TECLA_CTRL

#define IMU_MOUSE_THRESHOLD 3

// ========================= MPU6050 =========================

#define MPU6050_ADDR       0x68
#define MPU6050_PWR_MGMT   0x6B
#define MPU6050_ACCEL_XOUT 0x3B

// ========================= TIPOS =========================

typedef struct {
    uint8_t tipo;
    uint8_t id;
    int16_t valor;
    int16_t valor2;
} evento_t;

typedef struct {
    uint16_t center_x;
    uint16_t center_y;
    int      th_desativa;
    int      th_ativa;
} joystick_calib_t;

// ========================= FILAS =========================

QueueHandle_t xQueueEventosInput;
QueueHandle_t xQueueEventosIMU;

// ========================= JOYSTICK / TECLAS =========================

static void calibrar_joystick(joystick_calib_t *calib) {
    if (calib == NULL) {
        return;
    }

    printf("Calibrando joystick...\n");
    sleep_ms(500);

    uint32_t soma_x = 0;
    uint32_t soma_y = 0;
    int ruido_max_x = 0;
    int ruido_max_y = 0;

    for (int i = 0; i < CALIBRACAO_AMOSTRAS; i++) {
        adc_select_input(ADC_CHANNEL_X);
        busy_wait_us(10);
        uint16_t raw_x = adc_read();
        soma_x += raw_x;

        adc_select_input(ADC_CHANNEL_Y);
        busy_wait_us(10);
        uint16_t raw_y = adc_read();
        soma_y += raw_y;

        sleep_ms(10);
    }

    calib->center_x = (uint16_t)(soma_x / CALIBRACAO_AMOSTRAS);
    calib->center_y = (uint16_t)(soma_y / CALIBRACAO_AMOSTRAS);

    for (int i = 0; i < CALIBRACAO_AMOSTRAS; i++) {
        adc_select_input(ADC_CHANNEL_X);
        busy_wait_us(10);
        uint16_t raw_x = adc_read();
        int desvio_x = (raw_x > calib->center_x) ? (raw_x - calib->center_x) : (calib->center_x - raw_x);
        if (desvio_x > ruido_max_x) {
            ruido_max_x = desvio_x;
        }

        adc_select_input(ADC_CHANNEL_Y);
        busy_wait_us(10);
        uint16_t raw_y = adc_read();
        int desvio_y = (raw_y > calib->center_y) ? (raw_y - calib->center_y) : (calib->center_y - raw_y);
        if (desvio_y > ruido_max_y) {
            ruido_max_y = desvio_y;
        }

        sleep_ms(10);
    }

    int ruido_max = (ruido_max_x > ruido_max_y) ? ruido_max_x : ruido_max_y;
    calib->th_desativa = ruido_max + MARGEM_RUIDO;
    calib->th_ativa    = calib->th_desativa + MARGEM_ATIVA;

    printf("Calibração: cx=%d cy=%d th_des=%d th_atv=%d\n",
           calib->center_x, calib->center_y, calib->th_desativa, calib->th_ativa);
}

static void ler_joystick(const joystick_calib_t *calib, int *delta_x, int *delta_y) {
    static int buffer_x[AVG_BUFFER_SIZE] = {0};
    static int buffer_y[AVG_BUFFER_SIZE] = {0};
    static int index_x = 0;
    static int index_y = 0;

    adc_select_input(ADC_CHANNEL_X);
    (void)adc_read();
    uint16_t raw_x = adc_read();
    buffer_x[index_x] = (int)raw_x;
    index_x = (index_x + 1) % AVG_BUFFER_SIZE;

    int soma_x = 0;
    for (int i = 0; i < AVG_BUFFER_SIZE; i++) {
        soma_x += buffer_x[i];
    }
    int media_x = soma_x / AVG_BUFFER_SIZE;

    adc_select_input(ADC_CHANNEL_Y);
    (void)adc_read();
    uint16_t raw_y = adc_read();
    buffer_y[index_y] = (int)raw_y;
    index_y = (index_y + 1) % AVG_BUFFER_SIZE;

    int soma_y = 0;
    for (int i = 0; i < AVG_BUFFER_SIZE; i++) {
        soma_y += buffer_y[i];
    }
    int media_y = soma_y / AVG_BUFFER_SIZE;

    *delta_x = media_x - (int)calib->center_x;
    *delta_y = media_y - (int)calib->center_y;
}

static void enviar_press_tecla(uint8_t tecla) {
    evento_t evento = {
        .tipo = EVENTO_TECLA,
        .id = tecla,
        .valor = 1,
        .valor2 = 0
    };
    BaseType_t ok = xQueueSend(xQueueEventosInput, &evento, TIMEOUT_QUEUE_CRITICO);
    if (ok != pdTRUE) {
        printf("[ERRO] Fila cheia! PRESS 0x%02X\n", tecla);
    }
}

static void enviar_release_tecla(uint8_t tecla) {
    evento_t evento = {
        .tipo = EVENTO_TECLA,
        .id = tecla,
        .valor = 0,
        .valor2 = 0
    };
    BaseType_t ok = xQueueSend(xQueueEventosInput, &evento, TIMEOUT_QUEUE_CRITICO);
    if (ok != pdTRUE) {
        printf("[ERRO] Fila cheia! RELEASE 0x%02X\n", tecla);
    }
}

static void atualiza_tecla_virtual(uint8_t tecla, bool *estado_atual, bool estado_desejado) {
    if (!(*estado_atual) && estado_desejado) {
        enviar_press_tecla(tecla);
        *estado_atual = true;
    } else if (*estado_atual && !estado_desejado) {
        enviar_release_tecla(tecla);
        *estado_atual = false;
    }
}

// ========================= TASK: JOYSTICK =========================

void task_joystick(void *p) {
    const joystick_calib_t *calib = (const joystick_calib_t *)p;

    vTaskDelay(pdMS_TO_TICKS(2000));

    static bool w_pressionada = false;
    static bool a_pressionada = false;
    static bool s_pressionada = false;
    static bool d_pressionada = false;

    printf("Task joystick: th_des=%d th_atv=%d\n",
           calib->th_desativa, calib->th_ativa);

    while (1) {
        int delta_x, delta_y;
        ler_joystick(calib, &delta_x, &delta_y);

        int abs_x = (delta_x < 0) ? -delta_x : delta_x;
        int abs_y = (delta_y < 0) ? -delta_y : delta_y;

        bool w_desejada = w_pressionada;
        bool s_desejada = s_pressionada;
        bool a_desejada = a_pressionada;
        bool d_desejada = d_pressionada;

        if (abs_y > calib->th_ativa) {
            if (delta_y < 0) {
                w_desejada = true;
                s_desejada = false;
            } else {
                s_desejada = true;
                w_desejada = false;
            }
        } else if (abs_y < calib->th_desativa) {
            w_desejada = false;
            s_desejada = false;
        }

        if (abs_x > calib->th_ativa) {
            if (delta_x < 0) {
                a_desejada = true;
                d_desejada = false;
            } else {
                d_desejada = true;
                a_desejada = false;
            }
        } else if (abs_x < calib->th_desativa) {
            a_desejada = false;
            d_desejada = false;
        }

        atualiza_tecla_virtual(TECLA_W, &w_pressionada, w_desejada);
        atualiza_tecla_virtual(TECLA_A, &a_pressionada, a_desejada);
        atualiza_tecla_virtual(TECLA_S, &s_pressionada, s_desejada);
        atualiza_tecla_virtual(TECLA_D, &d_pressionada, d_desejada);

        vTaskDelay(pdMS_TO_TICKS(JOYSTICK_DELAY_MS));
    }
}

// ========================= TASK: BOTÕES =========================

void task_botoes(void *p) {
    (void)p;

    gpio_init(GPIO_BTN_VERDE);
    gpio_set_dir(GPIO_BTN_VERDE, GPIO_IN);
    gpio_pull_up(GPIO_BTN_VERDE);

    gpio_init(GPIO_BTN_AZUL);
    gpio_set_dir(GPIO_BTN_AZUL, GPIO_IN);
    gpio_pull_up(GPIO_BTN_AZUL);

    gpio_init(GPIO_BTN_AMARELO);
    gpio_set_dir(GPIO_BTN_AMARELO, GPIO_IN);
    gpio_pull_up(GPIO_BTN_AMARELO);

    gpio_init(GPIO_BTN_VERMELHO);
    gpio_set_dir(GPIO_BTN_VERMELHO, GPIO_IN);
    gpio_pull_up(GPIO_BTN_VERMELHO);

    struct {
        uint gpio;
        uint8_t tecla;
    } botoes[4] = {
        {GPIO_BTN_VERDE,    BOTAO_VERDE_TECLA},
        {GPIO_BTN_AZUL,     BOTAO_AZUL_TECLA},
        {GPIO_BTN_AMARELO,  BOTAO_AMARELO_TECLA},
        {GPIO_BTN_VERMELHO, BOTAO_VERMELHO_TECLA}
    };

    bool estado_anterior[4] = {true, true, true, true};
    uint32_t ultimo_tempo[4] = {0, 0, 0, 0};

    for (int i = 0; i < 4; i++) {
        int gpio_val = gpio_get(botoes[i].gpio);
        estado_anterior[i] = (gpio_val == 0);
    }

    while (1) {
        uint32_t tempo_atual = to_ms_since_boot(get_absolute_time());

        for (int i = 0; i < 4; i++) {
            int gpio_val = gpio_get(botoes[i].gpio);
            bool estado_atual = (gpio_val == 0);

            if (estado_atual != estado_anterior[i]) {
                uint32_t tempo_debounce = tempo_atual - ultimo_tempo[i];
                if (tempo_debounce >= BUTTON_DEBOUNCE_MS) {
                    evento_t evento = {
                        .tipo = EVENTO_TECLA,
                        .id = botoes[i].tecla,
                        .valor = estado_atual ? 1 : 0,
                        .valor2 = 0
                    };

                    BaseType_t resultado = xQueueSend(xQueueEventosInput, &evento, TIMEOUT_QUEUE_CRITICO);
                    if (resultado == pdTRUE) {
                        ultimo_tempo[i] = tempo_atual;
                        estado_anterior[i] = estado_atual;
                    } else {
                        printf("[ERRO] Fila cheia! Botão GP%d tecla=0x%02X\n",
                               botoes[i].gpio, botoes[i].tecla);
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(BUTTON_DELAY_MS));
    }
}

// ========================= IMU / MPU6050 =========================

static bool imu_init(void) {
    uint8_t buf[2] = {MPU6050_PWR_MGMT, 0x00};
    int ret = i2c_write_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);
    return (ret == 2);
}

static int16_t imu_read_16bit(uint8_t reg) {
    uint8_t data[2] = {0, 0};
    int ret;

    ret = i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    if (ret < 0) {
        return 0;
    }

    ret = i2c_read_blocking(I2C_PORT, MPU6050_ADDR, data, 2, false);
    if (ret < 0) {
        return 0;
    }

    int16_t valor = (int16_t)((data[0] << 8) | data[1]);
    return valor;
}

static int16_t calcular_pitch(int16_t accel_x, int16_t accel_y, int16_t accel_z) {
    (void)accel_y;
    (void)accel_z;
    int32_t pitch_raw = (int32_t)accel_x * 90 / 16384;
    if (pitch_raw > 90) pitch_raw = 90;
    if (pitch_raw < -90) pitch_raw = -90;
    return (int16_t)pitch_raw;
}

static int16_t calcular_roll(int16_t accel_x, int16_t accel_y, int16_t accel_z) {
    (void)accel_x;
    (void)accel_z;
    int32_t roll_raw = (int32_t)accel_y * 90 / 16384;
    if (roll_raw > 90) roll_raw = 90;
    if (roll_raw < -90) roll_raw = -90;
    return (int16_t)roll_raw;
}

// ========================= TASK: IMU =========================

void task_imu(void *p) {
    (void)p;

    bool imu_conectado = false;

    printf("[IMU] Init I2C0 SDA=%d SCL=%d freq=%d\n",
           I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ);
    i2c_init(I2C_PORT, I2C_FREQ);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    vTaskDelay(pdMS_TO_TICKS(100));

    printf("[IMU] Tentando inicializar MPU6050 (0x%02X)\n", MPU6050_ADDR);
    if (imu_init()) {
        vTaskDelay(pdMS_TO_TICKS(100));

        int16_t test_reads[3] = {0};
        for (int i = 0; i < 3; i++) {
            test_reads[i] = imu_read_16bit(MPU6050_ACCEL_XOUT);
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        bool leituras_validas = false;
        for (int i = 0; i < 3; i++) {
            if (test_reads[i] != 0) {
                leituras_validas = true;
                break;
            }
        }

        if (leituras_validas || true) {
            imu_conectado = true;
            printf("[IMU] OK leituras: %d %d %d\n",
                   test_reads[0], test_reads[1], test_reads[2]);
        } else {
            printf("[IMU] Aviso: leituras nulas, seguindo sem IMU\n");
        }
    } else {
        printf("[IMU] Erro init IMU\n");
    }

    if (!imu_conectado) {
        printf("[IMU] Desabilitado (sem sensor)\n");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    static int16_t pitch_history[3] = {0, 0, 0};
    static int16_t roll_history[3] = {0, 0, 0};
    static int history_index = 0;

    printf("[IMU] Loop leitura iniciado\n");

    while (1) {
        int16_t accel_x = imu_read_16bit(MPU6050_ACCEL_XOUT);
        int16_t accel_y = imu_read_16bit(MPU6050_ACCEL_XOUT + 2);
        int16_t accel_z = imu_read_16bit(MPU6050_ACCEL_XOUT + 4);

        printf("[IMU RAW] ax=%d ay=%d az=%d\n", accel_x, accel_y, accel_z);

        int16_t pitch = calcular_pitch(accel_x, accel_y, accel_z);
        int16_t roll  = calcular_roll(accel_x, accel_y, accel_z);

        pitch_history[history_index] = pitch;
        roll_history[history_index]  = roll;
        history_index = (history_index + 1) % 3;

        int32_t pitch_avg = 0;
        int32_t roll_avg  = 0;
        for (int i = 0; i < 3; i++) {
            pitch_avg += pitch_history[i];
            roll_avg  += roll_history[i];
        }
        pitch_avg /= 3;
        roll_avg  /= 3;

        int16_t delta_x = (int16_t)(roll_avg / 3);
        int16_t delta_y = (int16_t)(pitch_avg / 3);

        int16_t abs_delta_x = (delta_x < 0) ? -delta_x : delta_x;
        int16_t abs_delta_y = (delta_y < 0) ? -delta_y : delta_y;

        if (abs_delta_x >= IMU_MOUSE_THRESHOLD || abs_delta_y >= IMU_MOUSE_THRESHOLD) {
            printf("[IMU DBG] dx=%d dy=%d ax=%d ay=%d\n",
                   delta_x, delta_y, abs_delta_x, abs_delta_y);

            if (delta_x > 127)  delta_x = 127;
            if (delta_x < -128) delta_x = -128;
            if (delta_y > 127)  delta_y = 127;
            if (delta_y < -128) delta_y = -128;

            evento_t evento_mouse = {
                .tipo   = EVENTO_MOUSE,
                .id     = 0,
                .valor  = delta_x,
                .valor2 = delta_y
            };

            BaseType_t sent = xQueueSend(xQueueEventosIMU, &evento_mouse, 0);
            if (sent != pdTRUE) {
                printf("[IMU] Fila IMU cheia, evento descartado\n");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(IMU_DELAY_MS));
    }
}

// ========================= TASK: UART ENVIO =========================

void task_uart_envio(void *p) {
    (void)p;

    uint32_t ultimo_heartbeat = 0;
    const uint32_t HEARTBEAT_INTERVAL_MS = 5000;

    while (1) {
        evento_t evento;
        uint32_t tempo_atual = to_ms_since_boot(get_absolute_time());

        if (tempo_atual - ultimo_heartbeat >= HEARTBEAT_INTERVAL_MS) {
            uint8_t heartbeat[4] = {0x54, 'H', 0, 0xFF};
#ifdef USE_USB_CDC
            for (int i = 0; i < 4; i++) {
                putchar_raw(heartbeat[i]);
            }
#else
            uart_write_blocking(UART_ID, heartbeat, 4);
#endif
            ultimo_heartbeat = tempo_atual;
        }

        bool evento_processado = false;

        if (xQueueReceive(xQueueEventosInput, &evento, 0) == pdTRUE) {
            evento_processado = true;
        } else if (xQueueReceive(xQueueEventosIMU, &evento, pdMS_TO_TICKS(10)) == pdTRUE) {
            evento_processado = true;
        }

        if (evento_processado) {
            uint8_t mensagem[4] = {0};

            switch (evento.tipo) {
                case EVENTO_TECLA: {
                    mensagem[0] = HEADER_TECLA;
                    mensagem[1] = evento.id;
                    mensagem[2] = (uint8_t)evento.valor;
                    mensagem[3] = 0xFF;
                    break;
                }

                case EVENTO_MOUSE: {
                    mensagem[0] = HEADER_MOUSE;
                    int8_t dx = (int8_t)evento.valor;
                    int8_t dy = (int8_t)evento.valor2;
                    mensagem[1] = (uint8_t)dx;
                    mensagem[2] = (uint8_t)dy;
                    mensagem[3] = 0xFF;
                    printf("[UART IMU] dx=%d dy=%d -> [%02X %02X %02X %02X]\n",
                           dx, dy, mensagem[0], mensagem[1], mensagem[2], mensagem[3]);
                    break;
                }

                default:
                    continue;
            }

#ifdef USE_USB_CDC
            for (int i = 0; i < 4; i++) {
                putchar_raw(mensagem[i]);
            }
#else
            uart_write_blocking(UART_ID, mensagem, sizeof(mensagem));
#endif
        }
    }
}

// ========================= MAIN =========================

int main(void) {
    stdio_init_all();

#ifndef USE_USB_CDC
    printf("Init UART0 TX=%d RX=%d baud=%d\n",
           UART_TX_PIN, UART_RX_PIN, UART_BAUD_RATE);
    uart_init(UART_ID, UART_BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    sleep_ms(100);
#else
    sleep_ms(500);
#endif

    uint8_t msg_init[4] = {0x54, 'I', 1, 0xFF};
#ifdef USE_USB_CDC
    for (int i = 0; i < 4; i++) {
        putchar_raw(msg_init[i]);
    }
    printf("USB-CDC pronto\n");
#else
    uart_write_blocking(UART_ID, msg_init, 4);
    printf("UART pronta\n");
#endif

    sleep_ms(200);

    adc_init();
    adc_gpio_init(GPIO_JOYSTICK_X);
    adc_gpio_init(GPIO_JOYSTICK_Y);
    sleep_ms(50);

    joystick_calib_t *joystick_calib = (joystick_calib_t *)pvPortMalloc(sizeof(joystick_calib_t));
    if (joystick_calib == NULL) {
        printf("[ERRO] pvPortMalloc joystick_calib\n");
        while (1) {
            tight_loop_contents();
        }
    }
    calibrar_joystick(joystick_calib);

    xQueueEventosInput = xQueueCreate(QUEUE_SIZE_INPUT, sizeof(evento_t));
    xQueueEventosIMU   = xQueueCreate(QUEUE_SIZE_IMU,   sizeof(evento_t));

    if (xQueueEventosInput == NULL || xQueueEventosIMU == NULL) {
        printf("[ERRO] Falha criar filas\n");
        while (1) {
            tight_loop_contents();
        }
    }

    printf("Filas criadas: INPUT=%d IMU=%d\n", QUEUE_SIZE_INPUT, QUEUE_SIZE_IMU);

    xTaskCreate(task_uart_envio, "UART Envio", 256, NULL,           PRIORIDADE_UART,  NULL);
    xTaskCreate(task_joystick,   "Joystick",   256, joystick_calib, PRIORIDADE_INPUT, NULL);
    xTaskCreate(task_botoes,     "Botoes",     256, NULL,           PRIORIDADE_INPUT, NULL);
    xTaskCreate(task_imu,        "IMU",        512, NULL,           PRIORIDADE_IMU,   NULL);

    printf("Sistema iniciado\n");

    vTaskStartScheduler();

    while (1) {
        tight_loop_contents();
    }
}
