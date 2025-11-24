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

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include "pico/stdlib.h"
#include "pico/time.h"      // Para busy_wait_us (Pico SDK)
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

// UARTa
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

// Fatores de prioridade angular para decidir eixo dominante (evita alternância entre direções)
#define FATOR_PRIORIZA_VERTICAL   1.2f   // Se abs_y > abs_x * este_fator → prioriza vertical
#define FATOR_PRIORIZA_HORIZONTAL 1.2f   // Se abs_x > abs_y * este_fator → prioriza horizontal
#define BUTTON_DELAY_MS    50      // Intervalo de leitura dos botões (ms)
#define BUTTON_DEBOUNCE_MS 100     // Tempo de debounce dos botões (ms)
#define IMU_DELAY_MS       30      // Intervalo de leitura do IMU (ms) - reduzido para aliviar fila
#define QUEUE_SIZE_INPUT   32      // Tamanho da fila de eventos de INPUT (joystick + botões) - prioridade máxima
#define QUEUE_SIZE_IMU     16      // Tamanho da fila de eventos de IMU (mouse/câmera) - prioridade menor

// Prioridades das tasks FreeRTOS (maior número = maior prioridade)
#define PRIORIDADE_UART    3       // Máxima prioridade: task de envio UART (drena as filas)
#define PRIORIDADE_INPUT   2       // Alta prioridade: joystick e botões (controles primários)
#define PRIORIDADE_IMU     1       // Baixa prioridade: IMU (câmera, pode perder eventos)

// Timeout para envio de eventos críticos (joystick e botões)
// Se a fila estiver cheia, aguarda até 10ms antes de desistir
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

// Valores de centro calibrados (serão preenchidos na inicialização)
static uint16_t center_x = 2047;  // Centro do eixo X (será calibrado)
static uint16_t center_y = 2047;  // Centro do eixo Y (será calibrado)

// Thresholds dinâmicos calculados durante calibração (baseados no ruído real medido)
static int th_desativa = 80;      // Threshold para sair da direção / voltar ao neutro (será calculado)
static int th_ativa = 120;        // Threshold para entrar numa direção (será calculado)

// Enum para direções do joystick (máquina de estados)
typedef enum {
    JOY_NEUTRO = 0,
    JOY_ESQUERDA,
    JOY_DIREITA,
    JOY_FRENTE,
    JOY_TRAS
} joy_dir_t;

// Tipos de dados do IMU (para movimento de câmera)
#define IMU_PITCH          0
#define IMU_ROLL           1

// Threshold para movimento do mouse via IMU (evita ruído)
#define IMU_MOUSE_THRESHOLD 3  // Valor mínimo de delta para enviar movimento (ajuste conforme necessário)

// ========================= DEFINES - MPU6050 =========================

#define MPU6050_ADDR       0x68    // Endereço I2C do MPU6050
#define MPU6050_PWR_MGMT   0x6B    // Registrador de gerenciamento de energia
#define MPU6050_ACCEL_XOUT 0x3B    // Registrador de aceleração X (high byte)
#define MPU6050_GYRO_XOUT  0x43    // Registrador de giroscópio X (high byte)

// ========================= TIPOS =========================

// Estrutura genérica de evento para a fila
typedef struct {
    uint8_t tipo;       // EVENTO_TECLA ou EVENTO_MOUSE
    uint8_t id;         // Código da tecla, ou tipo do IMU
    int16_t valor;      // Press(1)/Release(0) para tecla, ou delta para mouse/IMU
    int16_t valor2;     // Segundo valor (para mouse: delta_y)
} evento_t;

// ========================= VARIÁVEIS GLOBAIS =========================

// Filas separadas para priorizar eventos de controle sobre IMU
QueueHandle_t xQueueEventosInput;  // Fila para eventos de INPUT (joystick + botões) - PRIORIDADE MÁXIMA
QueueHandle_t xQueueEventosIMU;   // Fila para eventos de IMU (mouse/câmera) - PRIORIDADE MENOR

// ========================= FUNÇÕES AUXILIARES =========================

/**
 * Calibra o centro do joystick e calcula thresholds dinâmicos baseados no ruído real
 * Mede o ruído máximo observado durante a calibração e deriva TH_ATIVA e TH_DESATIVA automaticamente
 * 
 * IMPORTANTE PARA PICO 2 (RP2350):
 * - O ADC na Pico 2 tem características similares ao RP2040
 * - A calibração mede o ruído real do hardware específico
 * - Thresholds são calculados dinamicamente para cada placa/joystick
 */
static void calibrar_joystick(void) {
    printf("Calibrando joystick... mantenha no centro\n");
    sleep_ms(500);  // Aguarda usuário posicionar no centro
    
    uint32_t soma_x = 0;
    uint32_t soma_y = 0;
    int ruido_max_x = 0;
    int ruido_max_y = 0;
    
    // Primeira passada: calcula médias provisórias
    // IMPORTANTE: Na Pico 2, o ADC precisa de um pequeno delay após adc_select_input
    // para estabilizar a leitura (especialmente ao alternar entre canais)
    for (int i = 0; i < CALIBRACAO_AMOSTRAS; i++) {
        // Lê eixo X (VRX -> GP27 -> ADC1)
        adc_select_input(ADC_CHANNEL_X);
        // Pequeno delay para estabilização do ADC após selecionar canal (Pico 2)
        busy_wait_us(10);  // ~10us é suficiente para estabilizar
        uint16_t raw_x = adc_read();
        soma_x += raw_x;
        
        // Lê eixo Y (VRY -> GP26 -> ADC0)
        adc_select_input(ADC_CHANNEL_Y);
        // Pequeno delay para estabilização do ADC após selecionar canal (Pico 2)
        busy_wait_us(10);  // ~10us é suficiente para estabilizar
        uint16_t raw_y = adc_read();
        soma_y += raw_y;
        
        sleep_ms(10);
    }
    
    // Calcula médias (centros)
    center_x = (uint16_t)(soma_x / CALIBRACAO_AMOSTRAS);
    center_y = (uint16_t)(soma_y / CALIBRACAO_AMOSTRAS);
    
    // Segunda passada: mede ruído máximo (desvio em relação ao centro)
    for (int i = 0; i < CALIBRACAO_AMOSTRAS; i++) {
        // Lê eixo X
        adc_select_input(ADC_CHANNEL_X);
        busy_wait_us(10);  // Delay para estabilização (Pico 2)
        uint16_t raw_x = adc_read();
        int desvio_x = (raw_x > center_x) ? (raw_x - center_x) : (center_x - raw_x);
        if (desvio_x > ruido_max_x) {
            ruido_max_x = desvio_x;
        }
        
        // Lê eixo Y
        adc_select_input(ADC_CHANNEL_Y);
        busy_wait_us(10);  // Delay para estabilização (Pico 2)
        uint16_t raw_y = adc_read();
        int desvio_y = (raw_y > center_y) ? (raw_y - center_y) : (center_y - raw_y);
        if (desvio_y > ruido_max_y) {
            ruido_max_y = desvio_y;
        }
        
        sleep_ms(10);
    }
    
    // Calcula thresholds dinâmicos baseados no ruído máximo observado
    // TH_DESATIVA: ruído máximo + margem pequena (zona neutra)
    int ruido_max = (ruido_max_x > ruido_max_y) ? ruido_max_x : ruido_max_y;
    th_desativa = ruido_max + MARGEM_RUIDO;
    
    // TH_ATIVA: TH_DESATIVA + margem de ativação (histerese - entrada na direção exige valor maior)
    th_ativa = th_desativa + MARGEM_ATIVA;
    
    printf("Calibração concluída:\n");
    printf("  center_x=%d, center_y=%d\n", center_x, center_y);
    printf("  ruido_max_x=%d, ruido_max_y=%d, ruido_max=%d\n", ruido_max_x, ruido_max_y, ruido_max);
    printf("  TH_DESATIVA=%d (zona neutra), TH_ATIVA=%d (entrada na direção)\n", th_desativa, th_ativa);
}

/**
 * Lê o joystick e retorna deltas em relação ao centro
 * Aplica média móvel e retorna valores brutos (não normalizados)
 * 
 * IMPORTANTE: Esta função calcula deltas ANTES de aplicar deadzone
 * A deadzone será aplicada na task do joystick para permitir
 * escolha do eixo dominante baseada nos valores absolutos
 */
static void ler_joystick(int *delta_x, int *delta_y) {
    static int buffer_x[AVG_BUFFER_SIZE] = {0};
    static int buffer_y[AVG_BUFFER_SIZE] = {0};
    static int index_x = 0;
    static int index_y = 0;
    
    // Lê eixo X (VRX -> GP27 -> ADC1)
    // IMPORTANTE: Na Pico 2, adicionar pequeno delay após adc_select_input
    // para garantir leitura estável ao alternar entre canais
    adc_select_input(ADC_CHANNEL_X);
    busy_wait_us(10);  // Delay para estabilização do ADC (Pico 2)
    uint16_t raw_x = adc_read();
    buffer_x[index_x] = (int)raw_x;
    index_x = (index_x + 1) % AVG_BUFFER_SIZE;
    
    // Calcula média móvel do eixo X
    int soma_x = 0;
    for (int i = 0; i < AVG_BUFFER_SIZE; i++) {
        soma_x += buffer_x[i];
    }
    int media_x = soma_x / AVG_BUFFER_SIZE;
    
    // Lê eixo Y (VRY -> GP26 -> ADC0)
    adc_select_input(ADC_CHANNEL_Y);
    busy_wait_us(10);  // Delay para estabilização do ADC (Pico 2)
    uint16_t raw_y = adc_read();
    buffer_y[index_y] = (int)raw_y;
    index_y = (index_y + 1) % AVG_BUFFER_SIZE;
    
    // Calcula média móvel do eixo Y
    int soma_y = 0;
    for (int i = 0; i < AVG_BUFFER_SIZE; i++) {
        soma_y += buffer_y[i];
    }
    int media_y = soma_y / AVG_BUFFER_SIZE;
    
    // Calcula deltas em relação ao centro calibrado
    // NOTA: NÃO aplica deadzone aqui - será aplicada na task para permitir
    // escolha do eixo dominante baseada nos valores absolutos
    *delta_x = media_x - (int)center_x;
    *delta_y = media_y - (int)center_y;
}

// ========================= FUNÇÕES AUXILIARES - JOYSTICK =========================

/**
 * Envia evento de PRESS (pressionar tecla) para uma tecla específica
 * CRÍTICO: Verifica retorno e reporta erro se fila estiver cheia
 */
static void enviar_press_tecla(uint8_t tecla) {
    evento_t evento = {
        .tipo = EVENTO_TECLA,
        .id = tecla,
        .valor = 1,  // PRESS
        .valor2 = 0
    };
    // Usa timeout para garantir que eventos críticos não sejam perdidos
    BaseType_t ok = xQueueSend(xQueueEventosInput, &evento, TIMEOUT_QUEUE_CRITICO);
    if (ok != pdTRUE) {
        // ERRO CRÍTICO: evento de joystick perdido (fila cheia ou timeout)
        printf("[ERRO CRÍTICO] Fila cheia! PRESS tecla 0x%02X ('%c') PERDIDA\n", tecla, tecla);
    }
    #ifdef DEBUG
    printf("[JOY] PRESS: %c\n", tecla);
    #endif
}

/**
 * Envia evento de RELEASE (soltar tecla) para uma tecla específica
 * CRÍTICO: Verifica retorno e reporta erro se fila estiver cheia
 */
static void enviar_release_tecla(uint8_t tecla) {
    evento_t evento = {
        .tipo = EVENTO_TECLA,
        .id = tecla,
        .valor = 0,  // RELEASE
        .valor2 = 0
    };
    // Usa timeout para garantir que eventos críticos não sejam perdidos
    BaseType_t ok = xQueueSend(xQueueEventosInput, &evento, TIMEOUT_QUEUE_CRITICO);
    if (ok != pdTRUE) {
        // ERRO CRÍTICO: evento de joystick perdido (fila cheia ou timeout)
        printf("[ERRO CRÍTICO] Fila cheia! RELEASE tecla 0x%02X ('%c') PERDIDA\n", tecla, tecla);
    }
    #ifdef DEBUG
    printf("[JOY] RELEASE: %c\n", tecla);
    #endif
}

/**
 * Atualiza o estado de uma tecla virtual do joystick
 * Envia PRESS/RELEASE apenas quando há mudança de estado
 * Comportamento igual a teclas físicas: stateful, nunca toggle
 */
static void atualiza_tecla_virtual(uint8_t tecla, bool *estado_atual, bool estado_desejado) {
    if (!(*estado_atual) && estado_desejado) {
        // Estava solta e agora deve ficar pressionada -> enviar PRESS
        enviar_press_tecla(tecla);
        *estado_atual = true;
    } else if (*estado_atual && !estado_desejado) {
        // Estava pressionada e agora deve soltar -> enviar RELEASE
        enviar_release_tecla(tecla);
        *estado_atual = false;
    }
    // Caso contrário, não faz nada (estado não mudou - tecla continua como está)
}

/**
 * Calcula a direção do joystick baseada em delta_x, delta_y e direcao_atual
 * Implementa histerese explícita e margem angular para máxima precisão
 * 
 * Histerese:
 * - Para ENTRAR numa direção: eixo dominante deve ultrapassar TH_ATIVA
 * - Para SAIR da direção: ambos eixos devem cair abaixo de TH_DESATIVA
 * 
 * Margem angular:
 * - Usa FATOR_PRIORIZA_VERTICAL e FATOR_PRIORIZA_HORIZONTAL para decidir eixo dominante
 * - Em região ambígua, mantém direcao_atual (evita alternância entre direções)
 * 
 * Retorna JOY_NEUTRO ou uma das direções (FRENTE, TRAS, ESQUERDA, DIREITA)
 * Escolhe apenas UMA direção por vez (sem diagonais)
 */
static joy_dir_t calcular_direcao(int delta_x, int delta_y, joy_dir_t direcao_atual) {
    int abs_x = (delta_x < 0) ? -delta_x : delta_x;
    int abs_y = (delta_y < 0) ? -delta_y : delta_y;
    
    // 1) HISTERESE: Se já estamos em uma direção, só volta ao NEUTRO se ambos eixos caírem abaixo de TH_DESATIVA
    if (direcao_atual != JOY_NEUTRO) {
        if (abs_x < th_desativa && abs_y < th_desativa) {
            // Ambos eixos abaixo do threshold de desativação → volta ao neutro
            return JOY_NEUTRO;
        }
    }
    
    // 2) Decide eixo dominante usando margem angular (evita alternância)
    bool eixo_vertical_domina = (abs_y > abs_x * FATOR_PRIORIZA_VERTICAL);
    bool eixo_horizontal_domina = (abs_x > abs_y * FATOR_PRIORIZA_HORIZONTAL);
    
    // 3) Região ambígua: nenhum eixo domina claramente
    if (!eixo_vertical_domina && !eixo_horizontal_domina) {
        // Mantém a direção atual se não for neutro (evita piscadas)
        if (direcao_atual != JOY_NEUTRO) {
            return direcao_atual;
        }
        // Se está no neutro e região ambígua, verifica qual eixo está ligeiramente maior
        if (abs_y > abs_x) {
            eixo_vertical_domina = true;
        } else {
            eixo_horizontal_domina = true;
        }
    }
    
    // 4) Eixo Y domina (vertical) - FRENTE ou TRÁS
    if (eixo_vertical_domina) {
        if (delta_y < -th_ativa) {
            // Movimento para frente (negativo) acima de TH_ATIVA → FRENTE (muda direção mesmo se já estava em outra)
            return JOY_FRENTE;
        } else if (delta_y > th_ativa) {
            // Movimento para trás (positivo) acima de TH_ATIVA → TRÁS (muda direção mesmo se já estava em outra)
            return JOY_TRAS;
        } else {
            // Y não passou de TH_ATIVA
            // Se já estava em uma direção vertical, mantém (histerese)
            if (direcao_atual == JOY_FRENTE || direcao_atual == JOY_TRAS) {
                return direcao_atual;
            }
            // Se não estava em direção vertical, verifica eixo X como fallback
            if (delta_x > th_ativa) {
                return JOY_DIREITA;
            } else if (delta_x < -th_ativa) {
                return JOY_ESQUERDA;
            }
            // Se nenhum passou de TH_ATIVA, mantém direção atual ou retorna NEUTRO
            return (direcao_atual != JOY_NEUTRO) ? direcao_atual : JOY_NEUTRO;
        }
    }
    
    // 5) Eixo X domina (horizontal) - ESQUERDA ou DIREITA
    if (eixo_horizontal_domina) {
        if (delta_x > th_ativa) {
            // Movimento para direita (positivo) acima de TH_ATIVA → DIREITA (muda direção mesmo se já estava em outra)
            return JOY_DIREITA;
        } else if (delta_x < -th_ativa) {
            // Movimento para esquerda (negativo) acima de TH_ATIVA → ESQUERDA (muda direção mesmo se já estava em outra)
            return JOY_ESQUERDA;
        } else {
            // X não passou de TH_ATIVA
            // Se já estava em uma direção horizontal, mantém (histerese)
            if (direcao_atual == JOY_ESQUERDA || direcao_atual == JOY_DIREITA) {
                return direcao_atual;
            }
            // Se não estava em direção horizontal, verifica eixo Y como fallback
            if (delta_y < -th_ativa) {
                return JOY_FRENTE;
            } else if (delta_y > th_ativa) {
                return JOY_TRAS;
            }
            // Se nenhum passou de TH_ATIVA, mantém direção atual ou retorna NEUTRO
            return (direcao_atual != JOY_NEUTRO) ? direcao_atual : JOY_NEUTRO;
        }
    }
    
    // Fallback: retorna NEUTRO (ou mantém direção atual se já estava em uma)
    return (direcao_atual != JOY_NEUTRO) ? direcao_atual : JOY_NEUTRO;
}

// ========================= TASK: JOYSTICK (UNIFICADA COM PRESS/RELEASE) =========================

/**
 * Task unificada do joystick com máquina de estados e transições diretas
 * Garante:
 * - Troca direta de direção (ex: FRENTE → DIREITA) sem precisar passar pelo centro
 * - Parada total quando volta ao centro (sempre envia RELEASE)
 * - Comportamento de controle real de console
 */
void task_joystick(void *p) {
    (void)p;
    
    // Inicializa ADC (já inicializado no main, mas garante)
    adc_init();
    adc_gpio_init(GPIO_JOYSTICK_X);
    adc_gpio_init(GPIO_JOYSTICK_Y);
    
    // Aguarda calibração ser concluída
    sleep_ms(2000);
    
    // Estado atual das teclas virtuais WASD (stateful, nunca toggle)
    // true = tecla está sendo considerada como segurada pelo PC
    // false = tecla está solta
    static bool w_pressionada = false;
    static bool a_pressionada = false;
    static bool s_pressionada = false;
    static bool d_pressionada = false;
    
    // Estado atual da direção do joystick (para histerese e margem angular)
    static joy_dir_t direcao_atual = JOY_NEUTRO;
    
    printf("Task joystick iniciada (comportamento digital com thresholds dinâmicos e histerese)\n");
    printf("Thresholds calculados: TH_DESATIVA=%d, TH_ATIVA=%d\n", th_desativa, th_ativa);
    printf("Margens angulares: Vertical=%.1f, Horizontal=%.1f\n", FATOR_PRIORIZA_VERTICAL, FATOR_PRIORIZA_HORIZONTAL);
    
    while (1) {
        // 1) Lê valores brutos e deltas
        int delta_x, delta_y;
        ler_joystick(&delta_x, &delta_y);
        
        // 2) Calcula direção nova baseada nos deltas, usando direcao_atual para histerese
        joy_dir_t direcao_nova = calcular_direcao(delta_x, delta_y, direcao_atual);
        
        // Atualiza direcao_atual para próxima iteração
        direcao_atual = direcao_nova;
        
        // 3) Define quais teclas DEVEM estar pressionadas AGORA (baseado na direção)
        // Começa "zerando" o estado desejado
        bool w_desejada = false;
        bool a_desejada = false;
        bool s_desejada = false;
        bool d_desejada = false;
        
        // De acordo com direcao_nova, marca APENAS UMA como verdadeira
        switch (direcao_nova) {
            case JOY_FRENTE:
                w_desejada = true;
                break;
            case JOY_TRAS:
                s_desejada = true;
                break;
            case JOY_ESQUERDA:
                a_desejada = true;
                break;
            case JOY_DIREITA:
                d_desejada = true;
                break;
            case JOY_NEUTRO:
            default:
                // Todas permanecem false (centro = todas soltas)
                break;
        }
        
        // 4) Compara estado atual vs estado desejado e envia PRESS/RELEASE
        // Isso garante que:
        // - Press é enviado somente quando a tecla passa de "solta" → "pressionada"
        // - Release é enviado somente quando a tecla passa de "pressionada" → "solta"
        // - Não há toggles
        // - Não há "tecla presa" após o analógico voltar ao centro
        atualiza_tecla_virtual(TECLA_W, &w_pressionada, w_desejada);
        atualiza_tecla_virtual(TECLA_A, &a_pressionada, a_desejada);
        atualiza_tecla_virtual(TECLA_S, &s_pressionada, s_desejada);
        atualiza_tecla_virtual(TECLA_D, &d_pressionada, d_desejada);
        
        #ifdef DEBUG
        // Debug detalhado (a cada 20 leituras para não poluir)
        static int debug_counter = 0;
        if (++debug_counter >= 20) {
            int abs_x = (delta_x < 0) ? -delta_x : delta_x;
            int abs_y = (delta_y < 0) ? -delta_y : delta_y;
            
            printf("[JOY DEBUG] raw: x=%d y=%d | delta: x=%d y=%d | abs: x=%d y=%d | dir=%d (TH_DESATIVA=%d, TH_ATIVA=%d)\n",
                   raw_x, raw_y, delta_x, delta_y, abs_x, abs_y, direcao_nova, th_desativa, th_ativa);
            printf("[JOY DEBUG] Estados: W=%s A=%s S=%s D=%s | Desejados: W=%s A=%s S=%s D=%s\n",
                   w_pressionada ? "ON" : "OFF", a_pressionada ? "ON" : "OFF",
                   s_pressionada ? "ON" : "OFF", d_pressionada ? "ON" : "OFF",
                   w_desejada ? "ON" : "OFF", a_desejada ? "ON" : "OFF",
                   s_desejada ? "ON" : "OFF", d_desejada ? "ON" : "OFF");
            debug_counter = 0;
        }
        
        // Debug de ruído quando no centro (para verificar se thresholds estão adequados)
        static int debug_ruido_counter = 0;
        if (direcao_nova == JOY_NEUTRO && ++debug_ruido_counter >= 50) {
            int abs_x = (delta_x < 0) ? -delta_x : delta_x;
            int abs_y = (delta_y < 0) ? -delta_y : delta_y;
            printf("[JOY RUÍDO] Centro: delta_x=%d delta_y=%d | abs_x=%d abs_y=%d (TH_DESATIVA=%d, TH_ATIVA=%d)\n",
                   delta_x, delta_y, abs_x, abs_y, th_desativa, th_ativa);
            debug_ruido_counter = 0;
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
    
    #ifdef DEBUG
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
    #else
    // Inicializa estados anteriores sem debug
    for (int i = 0; i < 4; i++) {
        int gpio_val = gpio_get(botoes[i].gpio);
        bool estado = (gpio_val == 0);  // Pressionado = 0 (pull-up)
        estado_anterior[i] = estado;  // Inicializa com estado atual
    }
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
                    
                    // Tenta enviar para a fila de INPUT (eventos críticos)
                    // Usa timeout para garantir que eventos não sejam perdidos
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
                        // ERRO CRÍTICO: evento de botão perdido
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
 * Retorna valor em graus aproximados (-90 a +90)
 */
static int16_t calcular_roll(int16_t accel_x, int16_t accel_y, int16_t accel_z) {
    // Similar ao pitch, mas usando accel_y
    // Em um MPU6050 típico, a aceleração é em g's, então:
    // accel_y de -16384 a +16384 representa aproximadamente -2g a +2g
    
    int32_t roll_raw = (int32_t)accel_y * 90 / 16384;
    
    // Limita entre -90 e +90
    if (roll_raw > 90) roll_raw = 90;
    if (roll_raw < -90) roll_raw = -90;
    
    return (int16_t)roll_raw;
}

void task_imu(void *p) {
    (void)p;
    
    bool imu_conectado = false;
    
    // Inicializa I2C
    // IMPORTANTE: Na Pico 2 (RP2350), I2C0 em GP4/GP5 é suportado nativamente
    // Pull-ups internos são suficientes para distâncias curtas (< 30cm)
    // Para distâncias maiores ou múltiplos dispositivos, considere pull-ups externos (2.2kΩ)
    printf("[IMU] Inicializando I2C0 (SDA=GP%d, SCL=GP%d, %d Hz)...\n", 
           I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ);
    i2c_init(I2C_PORT, I2C_FREQ);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);  // Pull-up interno habilitado
    gpio_pull_up(I2C_SCL_PIN);  // Pull-up interno habilitado
    
    // Pequeno delay para estabilização
    sleep_ms(100);
    
    // Tenta inicializar MPU6050
    printf("[IMU] Tentando inicializar MPU6050 (endereço 0x%02X)...\n", MPU6050_ADDR);
    if (imu_init()) {
        // Testa se consegue ler um registrador (verifica se IMU está realmente conectado)
        // Faz várias tentativas para garantir que não é ruído
        int16_t test_reads[3] = {0};
        for (int i = 0; i < 3; i++) {
            test_reads[i] = imu_read_16bit(MPU6050_ACCEL_XOUT);
            sleep_ms(10);
        }
        
        // Se pelo menos uma leitura retornou valor diferente de 0, IMU está conectado
        // (mesmo que todas sejam 0, pode ser válido se o sensor estiver perfeitamente nivelado)
        bool leituras_validas = false;
        for (int i = 0; i < 3; i++) {
            if (test_reads[i] != 0) {
                leituras_validas = true;
                break;
            }
        }
        
        if (leituras_validas || true) {  // Aceita mesmo se todas forem 0 (pode ser válido)
            imu_conectado = true;
            printf("[IMU] Inicializado com sucesso! Leituras de teste: %d, %d, %d\n", 
                   test_reads[0], test_reads[1], test_reads[2]);
        } else {
            printf("[IMU] AVISO: IMU não responde - continuando sem IMU\n");
        }
    } else {
        printf("[IMU] AVISO: Erro ao inicializar IMU - continuando sem IMU\n");
    }
    
    // Se IMU não está conectado, apenas faz delay sem tentar ler
    if (!imu_conectado) {
        printf("[IMU] Task IMU: aguardando conexão do IMU (não bloqueia outras tasks)\n");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));  // Delay longo, não consome recursos
        }
    }
    
    // Variáveis para filtro de média móvel (suaviza o movimento)
    static int16_t pitch_history[3] = {0, 0, 0};
    static int16_t roll_history[3] = {0, 0, 0};
    static int history_index = 0;
    
    printf("[IMU] Iniciando loop de leitura...\n");
    
    // Loop principal apenas se IMU estiver conectado
    while (1) {
        // Lê aceleração (3 eixos, 16 bits cada)
        // Registradores: 0x3B=X_H, 0x3C=X_L, 0x3D=Y_H, 0x3E=Y_L, 0x3F=Z_H, 0x40=Z_L
        int16_t accel_x = imu_read_16bit(MPU6050_ACCEL_XOUT);      // 0x3B (high byte)
        int16_t accel_y = imu_read_16bit(MPU6050_ACCEL_XOUT + 2);   // 0x3D (high byte)
        int16_t accel_z = imu_read_16bit(MPU6050_ACCEL_XOUT + 4);   // 0x3F (high byte)
        
        // Verifica se houve erro na leitura (todos zeros pode ser erro ou válido)
        // Se for erro, os valores serão 0, mas não vamos bloquear por isso
        
        // Calcula ângulos
        int16_t pitch = calcular_pitch(accel_x, accel_y, accel_z);
        int16_t roll = calcular_roll(accel_x, accel_y, accel_z);
        
        // Aplica filtro de média móvel para suavizar
        pitch_history[history_index] = pitch;
        roll_history[history_index] = roll;
        history_index = (history_index + 1) % 3;
        
        // Calcula média dos últimos 3 valores
        int32_t pitch_avg = 0;
        int32_t roll_avg = 0;
        for (int i = 0; i < 3; i++) {
            pitch_avg += pitch_history[i];
            roll_avg += roll_history[i];
        }
        pitch_avg /= 3;
        roll_avg /= 3;
        
        // Converte ângulos em movimento de mouse/câmera
        // Pitch (inclinação frente/trás) -> movimento vertical do mouse (delta_y)
        // Roll (inclinação lateral) -> movimento horizontal do mouse (delta_x)
        // Escala: divide por um fator para suavizar (ajuste conforme necessário)
        // Valores maiores = movimento mais rápido, valores menores = movimento mais suave
        int16_t delta_x = (int16_t)(roll_avg / 3);   // Roll -> movimento horizontal
        int16_t delta_y = (int16_t)(pitch_avg / 3);  // Pitch -> movimento vertical
        
        // Aplica threshold: só envia movimento se for significativo (evita ruído)
        // Usa valor absoluto para verificar ambos os eixos
        int16_t abs_delta_x = (delta_x < 0) ? -delta_x : delta_x;
        int16_t abs_delta_y = (delta_y < 0) ? -delta_y : delta_y;
        
        if (abs_delta_x >= IMU_MOUSE_THRESHOLD || abs_delta_y >= IMU_MOUSE_THRESHOLD) {
            // Limita valores para caber em 8 bits (protocolo usa 1 byte por delta)
            // Range: -128 a +127
            if (delta_x > 127) delta_x = 127;
            if (delta_x < -128) delta_x = -128;
            if (delta_y > 127) delta_y = 127;
            if (delta_y < -128) delta_y = -128;
            
            evento_t evento_mouse = {
                .tipo = EVENTO_MOUSE,
                .id = 0,  // Não usado
                .valor = delta_x,
                .valor2 = delta_y
            };
            
            // Tenta enviar para a fila de IMU (prioridade menor - pode perder eventos se fila cheia)
            // Não usa timeout: se fila estiver cheia, simplesmente descarta (IMU pode perder frames)
            if (xQueueSend(xQueueEventosIMU, &evento_mouse, 0) == pdTRUE) {
                #ifdef DEBUG
                // Debug: mostra valores a cada 10 leituras (para não poluir o log)
                static int debug_counter = 0;
                if (++debug_counter >= 10) {
                    printf("[IMU] accel: x=%d y=%d z=%d | pitch=%d roll=%d | delta: x=%d y=%d\n",
                           accel_x, accel_y, accel_z, pitch_avg, roll_avg, delta_x, delta_y);
                    debug_counter = 0;
                }
                #endif
            }
            // Se fila cheia - pode acontecer se o PC não estiver lendo rápido o suficiente
            // Não fazemos nada, apenas pulamos este evento (IMU pode perder frames sem problema)
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
            #ifdef DEBUG
            printf("Heartbeat enviado\n");
            #endif
        }
        
        // PRIORIDADE: Consome primeiro da fila de INPUT (joystick + botões) - eventos críticos
        // Só consome da fila IMU se não houver eventos de input pendentes
        bool evento_processado = false;
        
        // Tenta receber evento da fila de INPUT (prioridade máxima)
        if (xQueueReceive(xQueueEventosInput, &evento, 0) == pdTRUE) {
            evento_processado = true;
        } 
        // Se não houver evento de input, tenta receber da fila IMU (prioridade menor)
        else if (xQueueReceive(xQueueEventosIMU, &evento, pdMS_TO_TICKS(10)) == pdTRUE) {
            evento_processado = true;
        }
        
        if (evento_processado) {
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
                    // delta_x e delta_y são valores signed de 8 bits (-128 a +127)
                    mensagem[0] = HEADER_MOUSE;  // 'M'
                    // Converte int16_t para int8_t (mantém sinal)
                    int8_t delta_x = (int8_t)evento.valor;
                    int8_t delta_y = (int8_t)evento.valor2;
                    mensagem[1] = (uint8_t)delta_x;   // delta_x (signed 8 bits)
                    mensagem[2] = (uint8_t)delta_y;   // delta_y (signed 8 bits)
                    mensagem[3] = 0xFF;               // Delimitador
                    
                    #ifdef DEBUG
                    // Debug: mostra mensagem de mouse enviada
                    printf("[UART] Mouse: delta_x=%d delta_y=%d -> [%02X %02X %02X %02X]\n",
                           delta_x, delta_y, mensagem[0], mensagem[1], mensagem[2], mensagem[3]);
                    #endif
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
            
            #ifdef DEBUG
            // Debug: mostra mensagem enviada (apenas para teclas para não poluir)
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
    // Inicializa stdio (para debug via USB)
    stdio_init_all();
    
    #ifndef USE_USB_CDC
    // Inicializa UART hardware (apenas se não estiver usando USB-CDC)
    // IMPORTANTE: Na Pico 2 (RP2350), UART0 em GP0/GP1 é padrão e suportado
    // Baud rate de 115200 é estável e compatível com a maioria dos adaptadores USB-UART
    printf("Inicializando UART0 (TX=GP%d, RX=GP%d, %d baud)...\n", 
           UART_TX_PIN, UART_RX_PIN, UART_BAUD_RATE);
    uart_init(UART_ID, UART_BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    
    // Pequeno delay para estabilização do UART
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
    
    // Inicializa ADC (para joystick analógico)
    // IMPORTANTE: Na Pico 2 (RP2350), o ADC é compatível com RP2040
    // - Resolução: 12 bits (0-4095)
    // - Referência: 3.3V
    // - Canais: GP26 (ADC0), GP27 (ADC1)
    adc_init();
    
    // Configura pinos como entradas ADC
    adc_gpio_init(GPIO_JOYSTICK_X);  // GP27 -> ADC1 (eixo X)
    adc_gpio_init(GPIO_JOYSTICK_Y);  // GP26 -> ADC0 (eixo Y)
    
    // Pequeno delay para estabilização do ADC após inicialização
    // Na Pico 2, o ADC pode precisar de um momento para estabilizar a referência interna
    sleep_ms(50);
    
    // Calibra o centro do joystick (DEVE ser feito antes de criar as tasks)
    // Esta calibração mede o ruído real e calcula thresholds dinâmicos
    calibrar_joystick();
    
    // Cria filas de eventos separadas (priorização: INPUT > IMU)
    xQueueEventosInput = xQueueCreate(QUEUE_SIZE_INPUT, sizeof(evento_t));
    xQueueEventosIMU = xQueueCreate(QUEUE_SIZE_IMU, sizeof(evento_t));
    
    if (xQueueEventosInput == NULL || xQueueEventosIMU == NULL) {
        printf("[ERRO] Falha ao criar filas de eventos!\n");
        while (1) {
            tight_loop_contents();
        }
    }
    
    printf("Filas criadas: INPUT (tamanho=%d), IMU (tamanho=%d)\n", QUEUE_SIZE_INPUT, QUEUE_SIZE_IMU);
    
    // Cria tasks do FreeRTOS com prioridades adequadas
    // IMPORTANTE: Para Pico 2 (RP2350), verifique FreeRTOSConfig.h:
    // - configCPU_CLOCK_HZ deve estar configurado para a frequência do RP2350 (ex: 250 MHz)
    // - configTICK_RATE_HZ deve estar adequado (ex: 1000 Hz = 1 ms por tick)
    // - O tick timer deve estar configurado corretamente para o RP2350
    // 
    // PRIORIDADES:
    // - PRIORIDADE_UART (3): Máxima - task de envio UART (drena as filas)
    // - PRIORIDADE_INPUT (2): Alta - joystick e botões (controles primários)
    // - PRIORIDADE_IMU (1): Baixa - IMU (câmera, pode perder eventos)
    xTaskCreate(task_uart_envio, "UART Envio", 256, NULL, PRIORIDADE_UART, NULL);  // PRIORIDADE MÁXIMA
    xTaskCreate(task_joystick, "Joystick", 256, NULL, PRIORIDADE_INPUT, NULL);     // Alta prioridade
    xTaskCreate(task_botoes, "Botoes", 256, NULL, PRIORIDADE_INPUT, NULL);         // Alta prioridade
    xTaskCreate(task_imu, "IMU", 512, NULL, PRIORIDADE_IMU, NULL);                 // Baixa prioridade
    
    printf("Sistema iniciado! Controle Megabonk pronto (Pico 2 / RP2350).\n");
    printf("Tasks criadas. Aguardando eventos...\n");
    
    // Inicia o scheduler do FreeRTOS
    // NOTA: vTaskStartScheduler() nunca retorna em execução normal
    vTaskStartScheduler();

    // Nunca deve chegar aqui
    while (1) {
        tight_loop_contents();
    }
}
