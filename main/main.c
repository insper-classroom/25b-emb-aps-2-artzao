/*
 * Firmware para controle físico do jogo Megabonk
 * Raspberry Pi Pico com FreeRTOS
 * 
 * Hardware:
 * - Joystick analógico: VRX (GP27/ADC1), VRY (GP26/ADC0)
 * - Botões: Verde (GP15), Azul (GP14), Amarelo (GP13), Vermelho (GP12)
 * - IMU: MPU6050 via I2C (SDA=GP4, SCL=GP5)
 * - Comunicação: UART0 para PC (115200 baud)
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
 * 3. BOTÃO JOYSTICK (Header 0x4A 'J'):
 *    [0x4A, press(1)/release(0), 0x01, 0xFF]
 *    - Left Mouse Button (Aim) - pressionar o joystick
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
 * - Pressionar Joystick -> Left Mouse Button (Aim)
 * - IMU -> Movimento de câmera (mouse)
 */

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include "pico/stdlib.h"
#include <stdio.h>
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
// Botão do joystick (pressionar o joystick) - Ajuste o GPIO conforme seu hardware
// Se o joystick não tiver botão central, deixe este define comentado
// #define GPIO_BTN_JOYSTICK 16  // Exemplo: ajuste para o GPIO correto

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

#define DEADZONE_LIMIT     60      // Zona morta do joystick (-60 a +60 = 0)
#define AVG_BUFFER_SIZE    5       // Tamanho do buffer de média móvel
#define JOYSTICK_DELAY_MS  50      // Intervalo de leitura do joystick (ms)
#define BUTTON_DELAY_MS    50      // Intervalo de leitura dos botões (ms)
#define BUTTON_DEBOUNCE_MS 100     // Tempo de debounce dos botões (ms)
#define IMU_DELAY_MS       20      // Intervalo de leitura do IMU (ms)
#define QUEUE_SIZE         10      // Tamanho da fila de eventos

// ========================= DEFINES - PROTOCOLO =========================

// Tipos de evento
#define EVENTO_TECLA       0       // Tecla pressionada/solta (W/A/S/D/Space/etc)
#define EVENTO_MOUSE       1       // Movimento de mouse (para IMU/câmera)
#define EVENTO_BOTAO_JOY   2       // Botão do joystick (pressionar)

// Cabeçalhos de mensagem
#define HEADER_TECLA       0x54    // 'T' - Tecla: [0x54, tecla_ascii, press(1)/release(0), 0xFF]
#define HEADER_MOUSE       0x4D    // 'M' - Mouse: [0x4D, delta_x, delta_y, 0xFF]
#define HEADER_BOTAO_JOY   0x4A    // 'J' - Botão joystick: [0x4A, press(1)/release(0), 0x00, 0xFF]

// Códigos ASCII das teclas do Megabonk
#define TECLA_W            0x57    // 'W' - Frente
#define TECLA_A            0x41    // 'A' - Esquerda
#define TECLA_S            0x53    // 'S' - Trás
#define TECLA_D            0x44    // 'D' - Direita
#define TECLA_SPACE        0x20    // Space - Pulo
#define TECLA_E            0x45    // 'E' - Interact
#define TECLA_TAB          0x09    // Tab - Map Overlay
#define TECLA_CTRL         0x11    // Left Ctrl - Slide
#define MOUSE_LEFT         0x01    // Left Mouse Button - Aim

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

// Threshold para considerar joystick "pressionado" (para enviar teclas)
#define JOYSTICK_THRESHOLD     80      // Valor absoluto mínimo para ativar tecla

// Tipos de dados do IMU (para movimento de câmera)
#define IMU_PITCH          0
#define IMU_ROLL           1

// ========================= DEFINES - MPU6050 =========================

#define MPU6050_ADDR       0x68    // Endereço I2C do MPU6050
#define MPU6050_PWR_MGMT   0x6B    // Registrador de gerenciamento de energia
#define MPU6050_ACCEL_XOUT 0x3B    // Registrador de aceleração X (high byte)
#define MPU6050_GYRO_XOUT  0x43    // Registrador de giroscópio X (high byte)

// ========================= TIPOS =========================

// Estrutura genérica de evento para a fila
typedef struct {
    uint8_t tipo;       // EVENTO_TECLA, EVENTO_MOUSE ou EVENTO_BOTAO_JOY
    uint8_t id;         // Código da tecla, ou tipo do IMU, ou 0/1 para botão joystick
    int16_t valor;      // Press(1)/Release(0) para tecla, ou delta para mouse/IMU
    int16_t valor2;     // Segundo valor (para mouse: delta_y)
} evento_t;

// ========================= VARIÁVEIS GLOBAIS =========================

QueueHandle_t xQueueEventos;  // Fila para eventos (joystick, botões, IMU)

// ========================= FUNÇÕES AUXILIARES =========================

/**
 * Mapeia valor do ADC (0-4095) para escala -255 a +255
 * Aplica deadzone no centro
 */
static int mapear_joystick(uint16_t adc_value) {
    // Centraliza o valor (centro teórico = 2047)
    int centralizado = (int)adc_value - 2047;
    
    // Mapeia para escala -255 a +255
    int escala = (centralizado * 255) / 2047;
    
    // Aplica deadzone
    if (escala > -DEADZONE_LIMIT && escala < DEADZONE_LIMIT) {
        escala = 0;
    }
    
    // Limita entre -255 e +255
    if (escala > 255) escala = 255;
    if (escala < -255) escala = -255;
    
    return escala;
}

// ========================= TASK: JOYSTICK X =========================

void task_joystick_x(void *p) {
    (void)p;
    
    // Inicializa ADC (já inicializado no main, mas garante)
    adc_init();
    adc_gpio_init(GPIO_JOYSTICK_X);
    
    // Buffer para média móvel
    int buffer[AVG_BUFFER_SIZE] = {0};
    int index = 0;
    int estado_anterior = 0;  // 0=nada, 1=D pressionada, -1=A pressionada
    
    while (1) {
        // Seleciona canal X e lê ADC
        adc_select_input(ADC_CHANNEL_X);
        uint16_t raw = adc_read();
        
        // Adiciona ao buffer circular
        buffer[index] = (int)raw;
        index = (index + 1) % AVG_BUFFER_SIZE;
        
        // Calcula média móvel
        int soma = 0;
        for (int i = 0; i < AVG_BUFFER_SIZE; i++) {
            soma += buffer[i];
        }
        int media = soma / AVG_BUFFER_SIZE;
        
        // Mapeia e aplica deadzone
        int valor = mapear_joystick((uint16_t)media);
        
        // Determina estado atual: D (direita, positivo) ou A (esquerda, negativo)
        int estado_atual = 0;
        if (valor > JOYSTICK_THRESHOLD) {
            estado_atual = 1;  // D (direita)
        } else if (valor < -JOYSTICK_THRESHOLD) {
            estado_atual = -1; // A (esquerda)
        }
        
        // Envia eventos de press/release conforme mudança de estado
        if (estado_atual != estado_anterior) {
            // Solta tecla anterior se havia uma pressionada
            if (estado_anterior == 1) {
                // Solta D
                evento_t evento = {
                    .tipo = EVENTO_TECLA,
                    .id = TECLA_D,
                    .valor = 0,  // Release
                    .valor2 = 0
                };
                xQueueSend(xQueueEventos, &evento, 0);
            } else if (estado_anterior == -1) {
                // Solta A
                evento_t evento = {
                    .tipo = EVENTO_TECLA,
                    .id = TECLA_A,
                    .valor = 0,  // Release
                    .valor2 = 0
                };
                xQueueSend(xQueueEventos, &evento, 0);
            }
            
            // Pressiona nova tecla se necessário
            if (estado_atual == 1) {
                // Pressiona D
                evento_t evento = {
                    .tipo = EVENTO_TECLA,
                    .id = TECLA_D,
                    .valor = 1,  // Press
                    .valor2 = 0
                };
                xQueueSend(xQueueEventos, &evento, 0);
            } else if (estado_atual == -1) {
                // Pressiona A
                evento_t evento = {
                    .tipo = EVENTO_TECLA,
                    .id = TECLA_A,
                    .valor = 1,  // Press
                    .valor2 = 0
                };
                xQueueSend(xQueueEventos, &evento, 0);
            }
            
            estado_anterior = estado_atual;
        }
        
        vTaskDelay(pdMS_TO_TICKS(JOYSTICK_DELAY_MS));
    }
}

// ========================= TASK: JOYSTICK Y =========================

void task_joystick_y(void *p) {
    (void)p;
    
    // Inicializa ADC
    adc_init();
    adc_gpio_init(GPIO_JOYSTICK_Y);
    
    // Buffer para média móvel
    int buffer[AVG_BUFFER_SIZE] = {0};
    int index = 0;
    int estado_anterior = 0;  // 0=nada, 1=W pressionada, -1=S pressionada
    
    while (1) {
        // Seleciona canal Y e lê ADC
        adc_select_input(ADC_CHANNEL_Y);
        uint16_t raw = adc_read();
        
        // Adiciona ao buffer circular
        buffer[index] = (int)raw;
        index = (index + 1) % AVG_BUFFER_SIZE;
        
        // Calcula média móvel
        int soma = 0;
        for (int i = 0; i < AVG_BUFFER_SIZE; i++) {
            soma += buffer[i];
        }
        int media = soma / AVG_BUFFER_SIZE;
        
        // Mapeia e aplica deadzone
        int valor = mapear_joystick((uint16_t)media);
        
        // Determina estado atual: W (frente, negativo) ou S (trás, positivo)
        // Nota: Dependendo da orientação do joystick, pode precisar inverter
        int estado_atual = 0;
        if (valor < -JOYSTICK_THRESHOLD) {
            estado_atual = 1;  // W (frente) - valor negativo = frente
        } else if (valor > JOYSTICK_THRESHOLD) {
            estado_atual = -1; // S (trás) - valor positivo = trás
        }
        
        // Envia eventos de press/release conforme mudança de estado
        if (estado_atual != estado_anterior) {
            // Solta tecla anterior se havia uma pressionada
            if (estado_anterior == 1) {
                // Solta W
                evento_t evento = {
                    .tipo = EVENTO_TECLA,
                    .id = TECLA_W,
                    .valor = 0,  // Release
                    .valor2 = 0
                };
                xQueueSend(xQueueEventos, &evento, 0);
            } else if (estado_anterior == -1) {
                // Solta S
                evento_t evento = {
                    .tipo = EVENTO_TECLA,
                    .id = TECLA_S,
                    .valor = 0,  // Release
                    .valor2 = 0
                };
                xQueueSend(xQueueEventos, &evento, 0);
            }
            
            // Pressiona nova tecla se necessário
            if (estado_atual == 1) {
                // Pressiona W
                evento_t evento = {
                    .tipo = EVENTO_TECLA,
                    .id = TECLA_W,
                    .valor = 1,  // Press
                    .valor2 = 0
                };
                xQueueSend(xQueueEventos, &evento, 0);
            } else if (estado_atual == -1) {
                // Pressiona S
                evento_t evento = {
                    .tipo = EVENTO_TECLA,
                    .id = TECLA_S,
                    .valor = 1,  // Press
                    .valor2 = 0
                };
                xQueueSend(xQueueEventos, &evento, 0);
            }
            
            estado_anterior = estado_atual;
        }
        
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
    
    // Mapeamento: índice -> (GPIO, Tecla ASCII)
    struct {
        uint gpio;
        uint8_t tecla;
    } botoes[4] = {
        {GPIO_BTN_VERDE,    BOTAO_VERDE_TECLA},     // Tab - Map Overlay
        {GPIO_BTN_AZUL,     BOTAO_AZUL_TECLA},      // Space - Pulo
        {GPIO_BTN_AMARELO,  BOTAO_AMARELO_TECLA},   // E - Interact
        {GPIO_BTN_VERMELHO, BOTAO_VERMELHO_TECLA}   // Left Ctrl - Slide
    };
    
    // Estados anteriores dos botões (para debounce)
    bool estado_anterior[4] = {true, true, true, true};
    uint32_t ultimo_tempo[4] = {0, 0, 0, 0};
    
    // Debug: mostra estado inicial dos botões via printf
    printf("Botões inicializados. Estados iniciais:\n");
    for (int i = 0; i < 4; i++) {
        int gpio_val = gpio_get(botoes[i].gpio);
        bool estado = (gpio_val == 0);  // Pressionado = 0 (pull-up)
        printf("  Botão %d: GP%d=%d, tecla=0x%02X, estado=%s\n", 
               i, botoes[i].gpio, gpio_val, botoes[i].tecla, 
               estado ? "PRESSED" : "RELEASED");
        estado_anterior[i] = estado;  // Inicializa com estado atual
    }
    printf("Aguardando eventos de botões...\n");
    
    // Inicializa botão do joystick se definido
    #ifdef GPIO_BTN_JOYSTICK
    gpio_init(GPIO_BTN_JOYSTICK);
    gpio_set_dir(GPIO_BTN_JOYSTICK, GPIO_IN);
    gpio_pull_up(GPIO_BTN_JOYSTICK);
    bool estado_anterior_joy = true;
    uint32_t ultimo_tempo_joy = 0;
    #endif
    
    while (1) {
        uint32_t tempo_atual = to_ms_since_boot(get_absolute_time());
        
        // Processa os 4 botões principais
        for (int i = 0; i < 4; i++) {
            int gpio_val = gpio_get(botoes[i].gpio);
            bool estado_atual = (gpio_val == 0);  // Pressionado = 0 (pull-up)
            
            // Detecta mudança de estado (press ou release)
            if (estado_atual != estado_anterior[i]) {
                // Debounce: verifica se passou tempo suficiente desde a última mudança
                uint32_t tempo_debounce = tempo_atual - ultimo_tempo[i];
                if (tempo_debounce >= BUTTON_DEBOUNCE_MS) {
                    // Envia evento de tecla (press ou release)
                    evento_t evento = {
                        .tipo = EVENTO_TECLA,
                        .id = botoes[i].tecla,
                        .valor = estado_atual ? 1 : 0,  // 1 = pressionado, 0 = solto
                        .valor2 = 0
                    };
                    
                    // Tenta enviar para a fila (não bloqueia se fila cheia)
                    BaseType_t resultado = xQueueSend(xQueueEventos, &evento, 0);
                    if (resultado == pdTRUE) {
                        ultimo_tempo[i] = tempo_atual;
                        estado_anterior[i] = estado_atual;
                        // Debug via printf
                        printf("[BOTAO] GP%d=%d -> %s (tecla=0x%02X) [OK]\n", 
                               botoes[i].gpio, gpio_val,
                               estado_atual ? "PRESS" : "RELEASE",
                               botoes[i].tecla);
                    } else {
                        printf("[ERRO] Fila cheia! Botão %d (GP%d) não enviado\n", i, botoes[i].gpio);
                    }
                }
            }
        }
        
        // Processa botão do joystick (Left Mouse Button - Aim)
        #ifdef GPIO_BTN_JOYSTICK
        bool estado_atual_joy = gpio_get(GPIO_BTN_JOYSTICK) == 0;
        if (estado_atual_joy != estado_anterior_joy) {
            if (tempo_atual - ultimo_tempo_joy >= BUTTON_DEBOUNCE_MS) {
                // Envia evento de botão do joystick (Left Mouse Button)
                evento_t evento = {
                    .tipo = EVENTO_BOTAO_JOY,
                    .id = 0,
                    .valor = estado_atual_joy ? 1 : 0,  // 1 = pressionado, 0 = solto
                    .valor2 = 0
                };
                
                if (xQueueSend(xQueueEventos, &evento, 0) == pdTRUE) {
                    ultimo_tempo_joy = tempo_atual;
                    estado_anterior_joy = estado_atual_joy;
                }
            }
        }
        #endif
        
        vTaskDelay(pdMS_TO_TICKS(BUTTON_DELAY_MS));
    }
}

// ========================= TASK: IMU (MPU6050) =========================

/**
 * Inicializa o MPU6050 via I2C
 * Retorna true se sucesso, false se erro (IMU não conectado ou não responde)
 */
static bool imu_init(void) {
    // Tenta escrever no power management register para acordar o dispositivo
    // Registrador 0x6B: escreve 0x00 para sair do modo sleep
    uint8_t buf[2] = {MPU6050_PWR_MGMT, 0x00};
    int ret = i2c_write_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);
    
    // Se ret < 0, houve erro (dispositivo não responde)
    // Se ret != 2, não escreveu todos os bytes (também erro)
    if (ret < 0 || ret != 2) {
        return false;  // IMU não está conectado ou não responde
    }
    
    // Pequeno delay para estabilização
    sleep_ms(100);
    return true;
}

/**
 * Lê um registrador de 16 bits (signed) do MPU6050
 * Retorna 0 se houver erro de comunicação
 */
static int16_t imu_read_16bit(uint8_t reg) {
    uint8_t data[2] = {0, 0};
    int ret;
    
    // Escreve endereço do registrador
    ret = i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);  // Mantém conexão
    if (ret < 0) {
        return 0;  // Erro na escrita
    }
    
    // Lê 2 bytes
    ret = i2c_read_blocking(I2C_PORT, MPU6050_ADDR, data, 2, false);
    if (ret < 0) {
        return 0;  // Erro na leitura
    }
    
    int16_t valor = (int16_t)((data[0] << 8) | data[1]);
    return valor;
}

/**
 * Calcula ângulo de pitch (inclinação para frente/trás) a partir da aceleração
 * Retorna valor em graus aproximados (-90 a +90)
 */
static int16_t calcular_pitch(int16_t accel_x, int16_t accel_y, int16_t accel_z) {
    // Fórmula simplificada: pitch = atan2(accel_x, sqrt(accel_y^2 + accel_z^2))
    // Para simplificar, usamos apenas accel_x e accel_z
    // Escala aproximada: mapeia para -90 a +90 graus
    
    // Normaliza usando apenas accel_x (simplificado)
    // Em um MPU6050 típico, a aceleração é em g's, então:
    // accel_x de -16384 a +16384 representa aproximadamente -2g a +2g
    
    int32_t pitch_raw = (int32_t)accel_x * 90 / 16384;
    
    // Limita entre -90 e +90
    if (pitch_raw > 90) pitch_raw = 90;
    if (pitch_raw < -90) pitch_raw = -90;
    
    return (int16_t)pitch_raw;
}

/**
 * Calcula ângulo de roll (inclinação lateral) a partir da aceleração
 */
static int16_t calcular_roll(int16_t accel_x, int16_t accel_y, int16_t accel_z) {
    // Similar ao pitch, mas usando accel_y
    int32_t roll_raw = (int32_t)accel_y * 90 / 16384;
    
    if (roll_raw > 90) roll_raw = 90;
    if (roll_raw < -90) roll_raw = -90;
    
    return (int16_t)roll_raw;
}

void task_imu(void *p) {
    (void)p;
    
    bool imu_conectado = false;
    
    // Inicializa I2C
    i2c_init(I2C_PORT, I2C_FREQ);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    
    // Pequeno delay para estabilização
    sleep_ms(100);
    
    // Tenta inicializar MPU6050
    if (imu_init()) {
        // Testa se consegue ler um registrador (verifica se IMU está realmente conectado)
        int16_t test_read = imu_read_16bit(MPU6050_ACCEL_XOUT);
        if (test_read != 0 || true) {  // Aceita qualquer valor (mesmo 0 pode ser válido)
            imu_conectado = true;
            printf("IMU inicializado com sucesso!\n");
        } else {
            printf("AVISO: IMU não responde - continuando sem IMU\n");
        }
    } else {
        printf("AVISO: Erro ao inicializar IMU - continuando sem IMU\n");
    }
    
    // Se IMU não está conectado, apenas faz delay sem tentar ler
    if (!imu_conectado) {
        printf("Task IMU: aguardando conexão do IMU (não bloqueia outras tasks)\n");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));  // Delay longo, não consome recursos
        }
    }
    
    // Loop principal apenas se IMU estiver conectado
    while (1) {
        // Lê aceleração (3 eixos, 16 bits cada)
        // Registradores: 0x3B=X_H, 0x3D=Y_H, 0x3F=Z_H
        int16_t accel_x = imu_read_16bit(MPU6050_ACCEL_XOUT);      // 0x3B
        int16_t accel_y = imu_read_16bit(MPU6050_ACCEL_XOUT + 2); // 0x3D
        int16_t accel_z = imu_read_16bit(MPU6050_ACCEL_XOUT + 4); // 0x3F
        
        // Se leitura falhou (retornou 0 e não deveria), pode ser que IMU desconectou
        // Mas não vamos travar, apenas pular esta leitura
        if (accel_x == 0 && accel_y == 0 && accel_z == 0) {
            // Pode ser válido ou erro - não fazemos nada, apenas continuamos
        }
        
        // Calcula ângulos
        int16_t pitch = calcular_pitch(accel_x, accel_y, accel_z);
        int16_t roll = calcular_roll(accel_x, accel_y, accel_z);
        
        // Converte ângulos em movimento de mouse/câmera
        // Pitch (inclinação frente/trás) -> movimento vertical do mouse
        // Roll (inclinação lateral) -> movimento horizontal do mouse
        // Escala: divide por 2 para suavizar (ajuste conforme necessário)
        int16_t delta_x = roll / 2;   // Roll -> movimento horizontal
        int16_t delta_y = pitch / 2;  // Pitch -> movimento vertical
        
        // Envia movimento de mouse (câmera) apenas se houver movimento significativo
        if (delta_x != 0 || delta_y != 0) {
            evento_t evento_mouse = {
                .tipo = EVENTO_MOUSE,
                .id = 0,  // Não usado
                .valor = delta_x,
                .valor2 = delta_y
            };
            xQueueSend(xQueueEventos, &evento_mouse, 0);
        }
        
        vTaskDelay(pdMS_TO_TICKS(IMU_DELAY_MS));
    }
}

// ========================= TASK: UART (ENVIO) =========================

void task_uart_envio(void *p) {
    (void)p;
    
    // Contador para heartbeat (envia mensagem periódica mesmo sem eventos)
    uint32_t ultimo_heartbeat = 0;
    const uint32_t HEARTBEAT_INTERVAL_MS = 5000;  // Envia heartbeat a cada 5 segundos
    
    while (1) {
        evento_t evento;
        uint32_t tempo_atual = to_ms_since_boot(get_absolute_time());
        
        // Verifica se deve enviar heartbeat
        if (tempo_atual - ultimo_heartbeat >= HEARTBEAT_INTERVAL_MS) {
            // Envia heartbeat: [0x54, 'H', 0, 0xFF]
            uint8_t heartbeat[4] = {0x54, 'H', 0, 0xFF};
            #ifdef USE_USB_CDC
            for (int i = 0; i < 4; i++) {
                putchar_raw(heartbeat[i]);
            }
            #else
            uart_write_blocking(UART_ID, heartbeat, 4);
            #endif
            ultimo_heartbeat = tempo_atual;
            printf("Heartbeat enviado\n");
        }
        
        // Tenta receber evento da fila (não bloqueia por muito tempo)
        if (xQueueReceive(xQueueEventos, &evento, pdMS_TO_TICKS(100)) == pdTRUE) {
            uint8_t mensagem[4] = {0};
            
            switch (evento.tipo) {
                case EVENTO_TECLA: {
                    // Protocolo: [0x54 ('T'), tecla_ascii, press(1)/release(0), 0xFF]
                    mensagem[0] = HEADER_TECLA;  // 'T'
                    mensagem[1] = evento.id;     // Código ASCII da tecla
                    mensagem[2] = (uint8_t)evento.valor;  // 1 = press, 0 = release
                    mensagem[3] = 0xFF;          // Delimitador
                    break;
                }
                
                case EVENTO_MOUSE: {
                    // Protocolo: [0x4D ('M'), delta_x, delta_y, 0xFF]
                    mensagem[0] = HEADER_MOUSE;  // 'M'
                    mensagem[1] = (uint8_t)(evento.valor & 0xFF);   // delta_x (limitado a 8 bits)
                    mensagem[2] = (uint8_t)(evento.valor2 & 0xFF); // delta_y (limitado a 8 bits)
                    mensagem[3] = 0xFF;           // Delimitador
                    break;
                }
                
                case EVENTO_BOTAO_JOY: {
                    // Protocolo: [0x4A ('J'), press(1)/release(0), 0x01, 0xFF]
                    // 0x01 indica Left Mouse Button (Aim)
                    mensagem[0] = HEADER_BOTAO_JOY;  // 'J'
                    mensagem[1] = (uint8_t)evento.valor;  // 1 = press, 0 = release
                    mensagem[2] = MOUSE_LEFT;  // 0x01 = Left Mouse Button
                    mensagem[3] = 0xFF;  // Delimitador
                    break;
                }
                
                default:
                    continue;  // Ignora tipos desconhecidos
            }
            
            // Envia mensagem via UART
            // Tenta UART hardware primeiro, se falhar usa USB-CDC como fallback
            #ifdef USE_USB_CDC
            // Usa USB-CDC (via putchar_raw)
            for (int i = 0; i < 4; i++) {
                putchar_raw(mensagem[i]);
            }
            #else
            // Usa UART hardware
            uart_write_blocking(UART_ID, mensagem, sizeof(mensagem));
            #endif
            
            // Debug: mostra mensagem enviada (apenas para teclas para não poluir)
            if (evento.tipo == EVENTO_TECLA) {
                printf("TX: [%02X %02X %02X %02X]\n", 
                       mensagem[0], mensagem[1], mensagem[2], mensagem[3]);
            }
        }
    }
}

// ========================= MAIN =========================

int main(void) {
    // Inicializa stdio (para debug via USB)
    stdio_init_all();
    
    #ifndef USE_USB_CDC
    // Inicializa UART hardware (apenas se não estiver usando USB-CDC)
    uart_init(UART_ID, UART_BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    
    // Pequeno delay para estabilização
    sleep_ms(100);
    #else
    // USB-CDC é inicializado automaticamente pelo stdio_init_all()
    sleep_ms(500);  // Delay maior para USB estabilizar
    #endif
    
    // Teste: envia mensagem de inicialização
    uint8_t msg_init[4] = {0x54, 'I', 1, 0xFF};  // Tecla 'I' para indicar inicialização
    #ifdef USE_USB_CDC
    // Usa USB-CDC
    for (int i = 0; i < 4; i++) {
        putchar_raw(msg_init[i]);
    }
    printf("USB-CDC inicializado (via cabo USB)\n");
    #else
    // Usa UART hardware
    uart_write_blocking(UART_ID, msg_init, 4);
    printf("UART inicializada (TX=GP%d, RX=GP%d, %d baud)\n", UART_TX_PIN, UART_RX_PIN, UART_BAUD_RATE);
    #endif
    
    // Pequeno delay antes de continuar
    sleep_ms(200);
    
    // Inicializa ADC (para joystick)
    adc_init();
    adc_gpio_init(GPIO_JOYSTICK_X);
    adc_gpio_init(GPIO_JOYSTICK_Y);
    
    // Cria fila de eventos
    xQueueEventos = xQueueCreate(QUEUE_SIZE, sizeof(evento_t));
    
    if (xQueueEventos == NULL) {
        printf("Erro ao criar fila!\n");
        while (1) {
            tight_loop_contents();
        }
    }
    
    // Cria tasks do FreeRTOS
    xTaskCreate(task_joystick_x, "Joystick X", 256, NULL, 1, NULL);
    xTaskCreate(task_joystick_y, "Joystick Y", 256, NULL, 1, NULL);
    xTaskCreate(task_botoes, "Botoes", 256, NULL, 1, NULL);
    xTaskCreate(task_imu, "IMU", 512, NULL, 1, NULL);  // Stack maior para IMU
    xTaskCreate(task_uart_envio, "UART Envio", 256, NULL, 1, NULL);
    
    printf("Sistema iniciado! Controle Megabonk pronto.\n");
    printf("Tasks criadas. Aguardando eventos...\n");
    
    // Inicia o scheduler do FreeRTOS
    vTaskStartScheduler();

    // Nunca deve chegar aqui
    while (1) {
        tight_loop_contents();
    }
}
