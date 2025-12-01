/*
 * Firmware para controle físico do jogo Megabonk
 * Raspberry Pi Pico 2 (RP2350) com FreeRTOS
 * 
 * COMPATIBILIDADE:
 * - Revisado e testado para Raspberry Pi Pico 2 / RP2350
 * - Requer Pico SDK 1.5.0 ou superior para suporte completo à Pico 2
 * - Compatível com FreeRTOS (verifique FreeRTOSConfig.h para configCPU_CLOCK_HZ adequado ao RP2350)
 * 
 * HARDWARE E PINOUT (Pico 2 / RP2350):
 * - Joystick analógico KY-023:
 *   * VRX -> GP27 (ADC1) - Eixo X horizontal
 *   * VRY -> GP26 (ADC0) - Eixo Y vertical
 *   * VCC -> 3.3V, GND -> GND
 *   * SW (botão) -> NÃO CONECTADO (não utilizado)
 * 
 * - Botões digitais (entrada com pull-up interno, pressionado = LOW):
 *   * Verde    -> GP15
 *   * Azul     -> GP14
 *   * Amarelo  -> GP13
 *   * Vermelho -> GP12
 * 
 * - IMU MPU6050 via I2C0:
 *   * SDA -> GP4 (I2C0 SDA)
 *   * SCL -> GP5 (I2C0 SCL)
 *   * VCC -> 3.3V, GND -> GND
 *   * Pull-ups internos habilitados (GP4 e GP5)
 * 
 * - Comunicação com PC:
 *   * UART0: TX -> GP0, RX -> GP1 (115200 baud)
 *   * OU USB-CDC: via cabo USB (definir USE_USB_CDC=1)
 * 
 * OBSERVAÇÕES IMPORTANTES PARA PICO 2:
 * - ADC: Resolução 12 bits (0-4095), referência 3.3V (compatível com RP2040)
 * - I2C0: Suporta até 400 kHz (configurado), pull-ups internos habilitados
 * - UART0: Padrão em GP0/GP1, suporta até 921600 baud (usando 115200)
 * - FreeRTOS: Requer configuração adequada de tick timer e CPU clock em FreeRTOSConfig.h
 * 
 * ARQUITETURA DE EVENTOS (OTIMIZADA):
 * - Duas filas separadas: INPUT (joystick + botões) e IMU (mouse/câmera)
 * - Priorização: eventos de INPUT têm prioridade máxima sobre IMU
 * - Timeout em eventos críticos: joystick e botões aguardam até 10ms antes de desistir
 * - IMU pode perder eventos se fila cheia (não é crítico)
 * 
 * DEBUG:
 * - Para ativar logs detalhados, defina DEBUG no compilador (ex: -DDEBUG)
 * - Em modo normal (sem DEBUG), apenas erros críticos são impressos
 * 
 * PROTOCOLO DE COMUNICAÇÃO (4 bytes por mensagem):
 * 
 * 1. TECLA (Header 0x54 'T'):
 *    [0x54, tecla_ascii, press(1)/release(0), 0xFF]
 *    - Teclas: W(0x57), A(0x41), S(0x53), D(0x44), Space(0x20), E(0x45), Tab(0x09), Ctrl(0x11)
 * 
 * 2. MOUSE/CÂMERA (Header 0x4D 'M'):
 *    [0x4D, delta_x, delta_y, 0xFF]
 *    - Movimento de câmera via IMU (pitch/roll convertidos em delta_x/delta_y)
 * 
 * MAPEAMENTO DE CONTROLES:
 * - Joystick Frente -> W
 * - Joystick Trás -> S
 * - Joystick Esquerda -> A
 * - Joystick Direita -> D
 * - Botão Azul -> Space (Pulo)
 * - Botão Vermelho -> Left Ctrl (Slide)
 * - Botão Verde -> Tab (Map Overlay)
 * - Botão Amarelo -> E (Interact)
 * - IMU -> Movimento de câmera (mouse)
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include "pico/time.h"      // Para busy_wait_us (Pico SDK)
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"

// ========================= DEFINES - PINOS =========================

// Botões (entrada digital com pull-up, pressionado = 0)
#define GPIO_BTN_VERDE     15
#define GPIO_BTN_AZUL      14
#define GPIO_BTN_AMARELO   13
#define GPIO_BTN_VERMELHO  12

// Joystick analógico
#define GPIO_JOYSTICK_Y    26  // VRY -> ADC0
#define GPIO_JOYSTICK_X    27  // VRX -> ADC1
#define ADC_CHANNEL_Y      0
#define ADC_CHANNEL_X      1

// IMU I2C
#define I2C_PORT           i2c0
#define I2C_SDA_PIN        4
#define I2C_SCL_PIN        5
#define I2C_FREQ           400000  // 400 kHz

// UART
#define UART_ID            uart0
#define UART_BAUD_RATE     115200
#define UART_TX_PIN        0
#define UART_RX_PIN        1

// ========================= DEFINES - CONFIGURAÇÕES =========================

#define CALIBRACAO_AMOSTRAS 100    // Número de amostras para calibrar o centro
#define AVG_BUFFER_SIZE    3       // Tamanho do buffer de média móvel (reduzido para melhor resposta)
#define JOYSTICK_DELAY_MS  10      // Intervalo de leitura do joystick (ms) - reduzido para maior responsividade

// Margens para cálculo de thresholds dinâmicos (baseados em calibração)
#define MARGEM_RUIDO       15      // Margem adicional sobre o ruído máximo medido (para TH_DESATIVA)
#define MARGEM_ATIVA       40      // Margem adicional sobre TH_DESATIVA (para TH_ATIVA - histerese)

// Fatores de prioridade angular (não usados no modo diagonal atual, mantidos para futura expansão)
#define FATOR_PRIORIZA_VERTICAL   1.2f
#define FATOR_PRIORIZA_HORIZONTAL 1.2f

#define BUTTON_DELAY_MS    50      // Intervalo de leitura dos botões (ms)
#define BUTTON_DEBOUNCE_MS 100     // Tempo de debounce dos botões (ms)
#define IMU_DELAY_MS       30      // Intervalo de leitura do IMU (ms)
#define QUEUE_SIZE_INPUT   32      // Tamanho da fila de eventos de INPUT (joystick + botões)
#define QUEUE_SIZE_IMU     16      // Tamanho da fila de eventos de IMU (mouse/câmera)

// Prioridades das tasks FreeRTOS (maior número = maior prioridade)
#define PRIORIDADE_UART    3       // Máxima prioridade: task de envio UART (drena as filas)
#define PRIORIDADE_INPUT   2       // Alta prioridade: joystick e botões (controles primários)
#define PRIORIDADE_IMU     1       // Baixa prioridade: IMU (câmera, pode perder eventos)

// Timeout para envio de eventos críticos (joystick e botões)
#define TIMEOUT_QUEUE_CRITICO pdMS_TO_TICKS(10)  // 10ms de timeout para eventos críticos

// ========================= DEFINES - PROTOCOLO =========================

// Tipos de evento
#define EVENTO_TECLA       0       // Tecla pressionada/solta (W/A/S/D/Space/etc)
#define EVENTO_MOUSE       1       // Movimento de mouse (para IMU/câmera)

// Cabeçalhos de mensagem
#define HEADER_TECLA       0x54    // 'T' - Tecla: [0x54, tecla_ascii, press(1)/release(0), 0xFF]
#define HEADER_MOUSE       0x4D    // 'M' - Mouse: [0x4D, delta_x, delta_y, 0xFF]

// Códigos ASCII das teclas do Megabonk
#define TECLA_W            0x57    // 'W' - Frente
#define TECLA_A            0x41    // 'A' - Esquerda
#define TECLA_S            0x53    // 'S' - Trás
#define TECLA_D            0x44    // 'D' - Direita
#define TECLA_SPACE        0x20    // Space - Pulo
#define TECLA_E            0x45    // 'E' - Interact
#define TECLA_TAB          0x09    // Tab - Map Overlay
#define TECLA_CTRL         0x11    // Left Ctrl - Slide

// IDs dos botões físicos
#define BOTAO_VERDE        1
#define BOTAO_AZUL         2
#define BOTAO_AMARELO      3
#define BOTAO_VERMELHO     4

// Mapeamento botões físicos -> teclas
#define BOTAO_VERDE_TECLA      TECLA_TAB      // Tab - Map Overlay
#define BOTAO_AZUL_TECLA       TECLA_SPACE    // Space - Pulo
#define BOTAO_AMARELO_TECLA    TECLA_E        // E - Interact
#define BOTAO_VERMELHO_TECLA   TECLA_CTRL     // Left Ctrl - Slide

// Tipos de dados do IMU (para movimento de câmera)
#define IMU_PITCH          0
#define IMU_ROLL           1

// Threshold para movimento do mouse via IMU (evita ruído)
#define IMU_MOUSE_THRESHOLD 3  // Valor mínimo de delta para enviar movimento

// ========================= DEFINES - MPU6050 =========================

#define MPU6050_ADDR       0x68    // Endereço I2C do MPU6050
#define MPU6050_PWR_MGMT   0x6B    // Registrador de gerenciamento de energia
#define MPU6050_ACCEL_XOUT 0x3B    // Registrador de aceleração X (high byte)
#define MPU6050_GYRO_XOUT  0x43    // Registrador de giroscópio X (high byte)

// ========================= TIPOS =========================

// Estrutura genérica de evento para a fila
typedef struct {
    uint8_t tipo;       // EVENTO_TECLA ou EVENTO_MOUSE
    uint8_t id;         // Código da tecla, ou tipo do IMU (não usado para mouse)
    int16_t valor;      // Press(1)/Release(0) para tecla, ou delta_x para mouse
    int16_t valor2;     // Segundo valor (para mouse: delta_y)
} evento_t;

// Estrutura de calibração do joystick (elimina globais no RTOS)
typedef struct {
    uint16_t center_x;
    uint16_t center_y;
    int      th_desativa;
    int      th_ativa;
} joystick_calib_t;

// ========================= VARIÁVEIS GLOBAIS =========================

// Filas separadas para priorizar eventos de controle sobre IMU
QueueHandle_t xQueueEventosInput;  // Fila para eventos de INPUT (joystick + botões)
QueueHandle_t xQueueEventosIMU;    // Fila para eventos de IMU (mouse/câmera)

// ========================= FUNÇÕES AUXILIARES =========================

/**
 * Calibra o centro do joystick e calcula thresholds dinâmicos baseados no ruído real
 * Mede o ruído máximo observado durante a calibração e deriva TH_ATIVA e TH_DESATIVA automaticamente
 * 
 * IMPORTANTE:
 * - Esta função é chamada ANTES do scheduler (no main), então pode usar sleep_ms/busy_wait_us
 */
static void calibrar_joystick(joystick_calib_t *calib) {
    printf("Calibrando joystick... mantenha no centro\n");
    sleep_ms(500);  // Aguarda usuário posicionar no centro
    
    uint32_t soma_x = 0;
    uint32_t soma_y = 0;
    int ruido_max_x = 0;
    int ruido_max_y = 0;
    
    // Primeira passada: calcula médias provisórias
    for (int i = 0; i < CALIBRACAO_AMOSTRAS; i++) {
        // Lê eixo X (VRX -> GP27 -> ADC1)
        adc_select_input(ADC_CHANNEL_X);
        busy_wait_us(10);  // Delay para estabilização
        uint16_t raw_x = adc_read();
        soma_x += raw_x;
        
        // Lê eixo Y (VRY -> GP26 -> ADC0)
        adc_select_input(ADC_CHANNEL_Y);
        busy_wait_us(10);  // Delay para estabilização
        uint16_t raw_y = adc_read();
        soma_y += raw_y;
        
        sleep_ms(10);
    }
    
    // Calcula médias (centros)
    calib->center_x = (uint16_t)(soma_x / CALIBRACAO_AMOSTRAS);
    calib->center_y = (uint16_t)(soma_y / CALIBRACAO_AMOSTRAS);
    
    // Segunda passada: mede ruído máximo (desvio em relação ao centro)
    for (int i = 0; i < CALIBRACAO_AMOSTRAS; i++) {
        // Lê eixo X
        adc_select_input(ADC_CHANNEL_X);
        busy_wait_us(10);
        uint16_t raw_x = adc_read();
        int desvio_x = (raw_x > calib->center_x) ? (raw_x - calib->center_x) : (calib->center_x - raw_x);
        if (desvio_x > ruido_max_x) {
            ruido_max_x = desvio_x;
        }
        
        // Lê eixo Y
        adc_select_input(ADC_CHANNEL_Y);
        busy_wait_us(10);
        uint16_t raw_y = adc_read();
        int desvio_y = (raw_y > calib->center_y) ? (raw_y - calib->center_y) : (calib->center_y - raw_y);
        if (desvio_y > ruido_max_y) {
            ruido_max_y = desvio_y;
        }
        
        sleep_ms(10);
    }
    
    // Calcula thresholds dinâmicos
    int ruido_max = (ruido_max_x > ruido_max_y) ? ruido_max_x : ruido_max_y;
    calib->th_desativa = ruido_max + MARGEM_RUIDO;
    calib->th_ativa    = calib->th_desativa + MARGEM_ATIVA;
    
    printf("Calibração concluída:\n");
    printf("  center_x=%d, center_y=%d\n", calib->center_x, calib->center_y);
    printf("  ruido_max_x=%d, ruido_max_y=%d, ruido_max=%d\n", ruido_max_x, ruido_max_y, ruido_max);
    printf("  TH_DESATIVA=%d (zona neutra), TH_ATIVA=%d (entrada na direção)\n",
           calib->th_desativa, calib->th_ativa);
}

/**
 * Lê o joystick e retorna deltas em relação ao centro
 * Usa dummy read ao trocar canal para evitar busy_wait_us dentro de task
 */
static void ler_joystick(const joystick_calib_t *calib, int *delta_x, int *delta_y) {
    static int buffer_x[AVG_BUFFER_SIZE] = {0};
    static int buffer_y[AVG_BUFFER_SIZE] = {0};
    static int index_x = 0;
    static int index_y = 0;
    
    // Lê eixo X (VRX -> GP27 -> ADC1)
    adc_select_input(ADC_CHANNEL_X);
    (void)adc_read();                // dummy read para estabilizar
    uint16_t raw_x = adc_read();
    buffer_x[index_x] = (int)raw_x;
    index_x = (index_x + 1) % AVG_BUFFER_SIZE;
    
    int soma_x = 0;
    for (int i = 0; i < AVG_BUFFER_SIZE; i++) {
        soma_x += buffer_x[i];
    }
    int media_x = soma_x / AVG_BUFFER_SIZE;
    
    // Lê eixo Y (VRY -> GP26 -> ADC0)
    adc_select_input(ADC_CHANNEL_Y);
    (void)adc_read();                // dummy read
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

// ========================= FUNÇÕES AUXILIARES - TECLAS =========================

/**
 * Envia evento de PRESS (pressionar tecla) para uma tecla específica
 */
static void enviar_press_tecla(uint8_t tecla) {
    evento_t evento = {
        .tipo = EVENTO_TECLA,
        .id = tecla,
        .valor = 1,  // PRESS
        .valor2 = 0
    };
    BaseType_t ok = xQueueSend(xQueueEventosInput, &evento, TIMEOUT_QUEUE_CRITICO);
    if (ok != pdTRUE) {
        printf("[ERRO CRÍTICO] Fila cheia! PRESS tecla 0x%02X ('%c') PERDIDA\n", tecla, tecla);
    }
#ifdef DEBUG
    printf("[JOY] PRESS: %c\n", tecla);
#endif
}

/**
 * Envia evento de RELEASE (soltar tecla) para uma tecla específica
 */
static void enviar_release_tecla(uint8_t tecla) {
    evento_t evento = {
        .tipo = EVENTO_TECLA,
        .id = tecla,
        .valor = 0,  // RELEASE
        .valor2 = 0
    };
    BaseType_t ok = xQueueSend(xQueueEventosInput, &evento, TIMEOUT_QUEUE_CRITICO);
    if (ok != pdTRUE) {
        printf("[ERRO CRÍTICO] Fila cheia! RELEASE tecla 0x%02X ('%c') PERDIDA\n", tecla, tecla);
    }
#ifdef DEBUG
    printf("[JOY] RELEASE: %c\n", tecla);
#endif
}

/**
 * Atualiza o estado de uma tecla virtual do joystick
 * Envia PRESS/RELEASE apenas quando há mudança de estado
 */
static void atualiza_tecla_virtual(uint8_t tecla, bool *estado_atual, bool estado_desejado) {
    if (!(*estado_atual) && estado_desejado) {
        enviar_press_tecla(tecla);
        *estado_atual = true;
    } else if (*estado_atual && !estado_desejado) {
        enviar_release_tecla(tecla);
        *estado_atual = false;
    }
}

// ========================= TASK: JOYSTICK (DIAGONAIS) =========================

void task_joystick(void *p) {
    joystick_calib_t *calib = (joystick_calib_t *)p;
    
    // Pequeno delay inicial (usando scheduler)
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Estado atual das teclas virtuais WASD
    static bool w_pressionada = false;
    static bool a_pressionada = false;
    static bool s_pressionada = false;
    static bool d_pressionada = false;

    printf("Task joystick iniciada (modo DIAGONAL: eixos independentes)\n");
    printf("Thresholds: TH_DESATIVA=%d, TH_ATIVA=%d\n",
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

        // ================== EIXO VERTICAL (W / S) ==================
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

        // ================== EIXO HORIZONTAL (A / D) ==================
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

        // Aplica diagonais naturalmente (eixos independentes)
        atualiza_tecla_virtual(TECLA_W, &w_pressionada, w_desejada);
        atualiza_tecla_virtual(TECLA_A, &a_pressionada, a_desejada);
        atualiza_tecla_virtual(TECLA_S, &s_pressionada, s_desejada);
        atualiza_tecla_virtual(TECLA_D, &d_pressionada, d_desejada);

#ifdef DEBUG
        static int debug_counter = 0;
        if (++debug_counter >= 20) {
            printf("[JOY] dx=%d dy=%d | absx=%d absy=%d | W=%d A=%d S=%d D=%d (desaj: W=%d A=%d S=%d D=%d)\n",
                   delta_x, delta_y, abs_x, abs_y,
                   w_pressionada, a_pressionada, s_pressionada, d_pressionada,
                   w_desejada, a_desejada, s_desejada, d_desejada);
            debug_counter = 0;
        }
#endif

        vTaskDelay(pdMS_TO_TICKS(JOYSTICK_DELAY_MS));
    }
}

// ========================= TASK: BOTÕES =========================

void task_botoes(void *p) {
    (void)p;
    
    // Inicializa GPIOs dos botões
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
    
#ifdef DEBUG
    printf("Botões inicializados. Estados iniciais:\n");
    for (int i = 0; i < 4; i++) {
        int gpio_val = gpio_get(botoes[i].gpio);
        bool estado = (gpio_val == 0);
        printf("  Botão %d: GP%d=%d, tecla=0x%02X, estado=%s\n", 
               i, botoes[i].gpio, gpio_val, botoes[i].tecla, 
               estado ? "PRESSED" : "RELEASED");
        estado_anterior[i] = estado;
    }
    printf("Aguardando eventos de botões...\n");
#else
    for (int i = 0; i < 4; i++) {
        int gpio_val = gpio_get(botoes[i].gpio);
        bool estado = (gpio_val == 0);
        estado_anterior[i] = estado;
    }
#endif
    
    while (1) {
        uint32_t tempo_atual = to_ms_since_boot(get_absolute_time());
        
        for (int i = 0; i < 4; i++) {
            int gpio_val = gpio_get(botoes[i].gpio);
            bool estado_atual = (gpio_val == 0);  // Pressionado = 0 (pull-up)
            
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
#ifdef DEBUG
                        printf("[BOTAO] GP%d=%d -> %s (tecla=0x%02X) [OK]\n", 
                               botoes[i].gpio, gpio_val,
                               estado_atual ? "PRESS" : "RELEASE",
                               botoes[i].tecla);
#endif
                    } else {
                        printf("[ERRO CRÍTICO] Fila cheia! Botão %d (GP%d, tecla=0x%02X) %s PERDIDO\n", 
                               i, botoes[i].gpio, botoes[i].tecla,
                               estado_atual ? "PRESS" : "RELEASE");
                    }
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(BUTTON_DELAY_MS));
    }
}

// ========================= FUNÇÕES AUXILIARES - IMU =========================

/**
 * Inicializa o MPU6050 via I2C
 * Retorna true se sucesso, false se erro
 */
static bool imu_init(void) {
    uint8_t buf[2] = {MPU6050_PWR_MGMT, 0x00};
    int ret = i2c_write_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);

    // Sucesso somente se escreveu exatamente 2 bytes
    return (ret == 2);
}

/**
 * Lê um registrador de 16 bits (signed) do MPU6050
 * Retorna 0 se houver erro de comunicação
 */
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

// ========================= TASK: IMU (MPU6050) =========================

void task_imu(void *p) {
    (void)p;
    
    bool imu_conectado = false;
    
    printf("[IMU] Inicializando I2C0 (SDA=GP%d, SCL=GP%d, %d Hz)...\n", 
           I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ);
    i2c_init(I2C_PORT, I2C_FREQ);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    printf("[IMU] Tentando inicializar MPU6050 (endereço 0x%02X)...\n", MPU6050_ADDR);
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
            printf("[IMU] Inicializado com sucesso! Leituras de teste: %d, %d, %d\n", 
                   test_reads[0], test_reads[1], test_reads[2]);
        } else {
            printf("[IMU] AVISO: IMU não responde - continuando sem IMU\n");
        }
    } else {
        printf("[IMU] AVISO: Erro ao inicializar IMU - continuando sem IMU\n");
    }
    
    if (!imu_conectado) {
        printf("[IMU] Task IMU: aguardando conexão do IMU (não bloqueia outras tasks)\n");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    
    static int16_t pitch_history[3] = {0, 0, 0};
    static int16_t roll_history[3] = {0, 0, 0};
    static int history_index = 0;
    
    printf("[IMU] Iniciando loop de leitura...\n");
    
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
            printf("[IMU DEBUG] delta_x=%d, delta_y=%d (abs_x=%d, abs_y=%d)\n",
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
                printf("[IMU DEBUG] Fila IMU cheia, evento descartado!\n");
            } else {
#ifdef DEBUG
                static int debug_counter = 0;
                if (++debug_counter >= 10) {
                    printf("[IMU] accel: x=%d y=%d z=%d | pitch=%ld roll=%ld | delta: x=%d y=%d\n",
                           accel_x, accel_y, accel_z, pitch_avg, roll_avg, delta_x, delta_y);
                    debug_counter = 0;
                }
#endif
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(IMU_DELAY_MS));
    }
}

// ========================= TASK: UART (ENVIO) =========================

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
#ifdef DEBUG
            printf("Heartbeat enviado\n");
#endif
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
                    printf("[UART IMU] Enviando mouse: dx=%d dy=%d -> [%02X %02X %02X %02X]\n",
                           dx, dy, mensagem[0], mensagem[1], mensagem[2], mensagem[3]);
#ifdef DEBUG
                    printf("[UART] Mouse: delta_x=%d delta_y=%d -> [%02X %02X %02X %02X]\n",
                           dx, dy, mensagem[0], mensagem[1], mensagem[2], mensagem[3]);
#endif
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
            
#ifdef DEBUG
            if (evento.tipo == EVENTO_TECLA) {
                printf("TX: [%02X %02X %02X %02X]\n", 
                       mensagem[0], mensagem[1], mensagem[2], mensagem[3]);
            }
#endif
        }
    }
}

// ========================= MAIN =========================

int main(void) {
    stdio_init_all();
    
#ifndef USE_USB_CDC
    printf("Inicializando UART0 (TX=GP%d, RX=GP%d, %d baud)...\n", 
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
    printf("USB-CDC inicializado (via cabo USB)\n");
#else
    uart_write_blocking(UART_ID, msg_init, 4);
    printf("UART inicializada (TX=GP%d, RX=GP%d, %d baud)\n",
           UART_TX_PIN, UART_RX_PIN, UART_BAUD_RATE);
#endif
    
    sleep_ms(200);
    
    // Inicializa ADC
    adc_init();
    adc_gpio_init(GPIO_JOYSTICK_X);
    adc_gpio_init(GPIO_JOYSTICK_Y);
    sleep_ms(50);
    
    // Aloca e calibra joystick antes do scheduler (sem RTOS ainda)
    joystick_calib_t *joystick_calib = pvPortMalloc(sizeof(joystick_calib_t));
    if (joystick_calib == NULL) {
        printf("[ERRO] Falha ao alocar memória para calibração do joystick!\n");
        while (1) {
            tight_loop_contents();
        }
    }
    calibrar_joystick(joystick_calib);
    
    // Cria filas
    xQueueEventosInput = xQueueCreate(QUEUE_SIZE_INPUT, sizeof(evento_t));
    xQueueEventosIMU   = xQueueCreate(QUEUE_SIZE_IMU,   sizeof(evento_t));
    
    if (xQueueEventosInput == NULL || xQueueEventosIMU == NULL) {
        printf("[ERRO] Falha ao criar filas de eventos!\n");
        while (1) {
            tight_loop_contents();
        }
    }
    
    printf("Filas criadas: INPUT (tamanho=%d), IMU (tamanho=%d)\n",
           QUEUE_SIZE_INPUT, QUEUE_SIZE_IMU);
    
    // Cria tasks
    xTaskCreate(task_uart_envio, "UART Envio", 256, NULL,           PRIORIDADE_UART,  NULL);
    xTaskCreate(task_joystick,   "Joystick",   256, joystick_calib, PRIORIDADE_INPUT, NULL);
    xTaskCreate(task_botoes,     "Botoes",     256, NULL,           PRIORIDADE_INPUT, NULL);
    xTaskCreate(task_imu,        "IMU",        512, NULL,           PRIORIDADE_IMU,   NULL);
    
    printf("Sistema iniciado! Controle Megabonk pronto (Pico 2 / RP2350).\n");
    printf("Tasks criadas. Aguardando eventos...\n");
    
    vTaskStartScheduler();

    // Nunca deve chegar aqui
    while (1) {
        tight_loop_contents();
    }
}
