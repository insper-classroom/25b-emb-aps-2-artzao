#!/usr/bin/env python3
"""
Programa para ler UART do controle Megabonk e simular teclas / movimento de câmera via vJoy
Compatível com o firmware main.c atualizado
No Windows: usa pyautogui para teclado e pyvjoy (vJoy) para eixos de câmera.
"""

import serial
import time
import pyautogui
import sys
import pyvjoy   # <<< novo


# Desabilita failsafe e pausa do pyautogui para resposta rápida
pyautogui.PAUSE = 0
pyautogui.FAILSAFE = False

# Modo debug (mostra mensagens recebidas)
DEBUG = False  # Mude para True para ver mensagens de debug

# ========================= DEFINES - PROTOCOLO =========================

# Headers de mensagem (devem bater com main.c)
HEADER_TECLA = 0x54      # 'T' - Tecla
HEADER_MOUSE = 0x4D      # 'M' - Mouse/Câmera
HEADER_BOTAO_JOY = 0x4A  # 'J' - Botão do joystick

# Códigos ASCII das teclas (devem bater com main.c)
TECLA_W = 0x57
TECLA_A = 0x41
TECLA_S = 0x53
TECLA_D = 0x44
TECLA_SPACE = 0x20
TECLA_E = 0x45
TECLA_TAB = 0x09
TECLA_CTRL = 0x11

# Mapeamento de teclas ASCII para nomes do pyautogui
TECLA_MAP = {
    TECLA_W: 'w',
    TECLA_A: 'a',
    TECLA_S: 's',
    TECLA_D: 'd',
    TECLA_SPACE: 'space',
    TECLA_E: 'e',
    TECLA_TAB: 'tab', 
    TECLA_CTRL: 'ctrl'
}

# Estados das teclas (para evitar spam)
teclas_pressionadas = set()

# ========================= ESTADO DO vJoy =========================

# Valores padrão do vJoy (16 bits: 0 .. 32767)
VJOY_MAX = 0x8000 - 1         # 32767
VJOY_CENTER = VJOY_MAX // 2   # ~16383

# Vamos usar os eixos X/Y como câmera (ou RX/RY, se preferir)
# Em vez de X/Y, use RX/RY:
VJOY_AXIS_X = pyvjoy.HID_USAGE_Z   # eixo look horizontal
VJOY_AXIS_Y = pyvjoy.HID_USAGE_RX   # eixo look vertical

# Estado global do joystick
j = None
vjoy_x = VJOY_CENTER
vjoy_y = VJOY_CENTER

# Sensibilidade: quanto 1 "delta" da IMU mexe o eixo do joystick
VJOY_SENS = 300   # ajuste isso se a câmera estiver lenta/rápida demais
DEADZONE = 2     # ignora deltas muito pequenos (ruído)
VJOY_SPRING = 0.15

# ========================= FUNÇÕES DE PROCESSAMENTO =========================

def processar_tecla(tecla_ascii, press_release):
    """
    Processa evento de tecla pressionada/solta
    Args:
        tecla_ascii: Código ASCII da tecla
        press_release: 1 = pressionado, 0 = solto
    """
    WASD = {TECLA_W, TECLA_A, TECLA_S, TECLA_D}
    global ultimo_tempo_wasd
    # Deadzone state
    # Estado da deadzone de WASD (em segundos)
    ultimo_tempo_wasd = 0.0
    WASD_DEADZONE_MS = 0.10  # 100 ms

    

    # ========== DEADZONE PARA JOYSTICK (WASD) ==========
    if tecla_ascii in WASD:
        agora = time.time()

        if press_release == 1:  # press
            # If moved too fast (micro jitters), ignore
            if agora - ultimo_tempo_wasd < WASD_DEADZONE_MS:
                # Ignore this press
                if DEBUG:
                    print(f"[DEADZONE] Ignorando jitter WASD: {tecla_ascii:02X}")
                return  

            # Otherwise accept the press
            ultimo_tempo_wasd = agora

        else:  # release
            # Always allow releases so keys don't get stuck
            ultimo_tempo_wasd = agora


    if tecla_ascii not in TECLA_MAP:
        print(f"Tecla desconhecida: 0x{tecla_ascii:02X}")
        return
    
    nome_tecla = TECLA_MAP[tecla_ascii]
    
    try:
        if press_release == 1:
            # Pressiona tecla
            if nome_tecla not in teclas_pressionadas:
                pyautogui.keyDown(nome_tecla)
                teclas_pressionadas.add(nome_tecla)
                print(f"Pressionou: {nome_tecla}")
        else:
            # Solta tecla
            if nome_tecla in teclas_pressionadas:
                pyautogui.keyUp(nome_tecla)
                teclas_pressionadas.remove(nome_tecla)
                print(f"Soltou: {nome_tecla}")
    except Exception as e:
        print(f"Erro ao processar tecla {nome_tecla}: {e}")


def processar_mouse(delta_x, delta_y):
    """
    Processa movimento de câmera via IMU -> eixos do vJoy (pyvjoy)
    Args:
        delta_x: Movimento horizontal (signed byte)
        delta_y: Movimento vertical (signed byte)
    """
    global j, vjoy_x, vjoy_y

    if j is None:
        # vJoy não inicializado
        return

    try:
        # Converte bytes sem sinal para valores com sinal
        if delta_x > 127:
            delta_x = delta_x - 256
        if delta_y > 127:
            delta_y = delta_y - 256

        if DEBUG:
            print(f"[SIGNED] dx={delta_x}, dy={delta_y}")


        # Axis isolation: choose dominant axis
        # HARD FILTER: if vertical movement is large, kill horizontal movement
        if delta_y > 100 or delta_y < -100:
            delta_x = 0
            print(delta_x, delta_y)
            if DEBUG:
                print(f"Filtered: dx={delta_x}, dy={delta_y}")



        # Aplica deadzone para evitar ruído
        if abs(delta_x) < DEADZONE:
            delta_x = 0
        if abs(delta_y) < DEADZONE:
            delta_y = 0



        # Se não há movimento significativo, centraliza
        if delta_x == 0 and delta_y == 0:
            vjoy_x = VJOY_CENTER
            vjoy_y = VJOY_CENTER
        else:
            # Mapeia delta diretamente ao redor do centro (sem acumular)
            vjoy_x = VJOY_CENTER + delta_x * VJOY_SENS
            vjoy_y = VJOY_CENTER - delta_y * VJOY_SENS  # inverte Y se quiser estilo FPS

        # Garante que fique dentro do range do vJoy
        vjoy_x = max(0, min(VJOY_MAX, vjoy_x))
        vjoy_y = max(0, min(VJOY_MAX, vjoy_y))

        j.set_axis(VJOY_AXIS_X, vjoy_x)
        j.set_axis(VJOY_AXIS_Y, vjoy_y)

        if DEBUG:
            print(f"vJoy: X={vjoy_x}, Y={vjoy_y}, dx={delta_x}, dy={delta_y}")

    except Exception as e:
        print(f"Erro ao mover câmera (vJoy): {e}")



def processar_botao_joystick(press_release):
    """
    Processa botão do joystick.
    Aqui você pode tanto:
      - continuar usando o mouse real (pyautogui.mouseDown), ou
      - mapear para um botão do vJoy (ex: botão 1) para o jogo ver como "fire/aim".
    """
    global j

    try:
        # Exemplo 1: continuar usando clique do mouse real
        # (o jogo precisa aceitar input de mouse real)
        if press_release == 1:
            pyautogui.mouseDown(button='left')
            print("Botão joystick: Pressionado (Aim)")
        else:
            pyautogui.mouseUp(button='left')
            print("Botão joystick: Solto")

        # Exemplo 2 (opcional): também acionar um botão do vJoy (botão 1)
        # if j is not None:
        #     j.set_button(1, 1 if press_release == 1 else 0)

    except Exception as e:
        print(f"Erro ao processar botão joystick: {e}")


# ========================= FUNÇÃO PRINCIPAL =========================

def main():
    global DEBUG, j, vjoy_x, vjoy_y

    # Verifica argumentos de linha de comando
    if '--debug' in sys.argv or '-d' in sys.argv:
        DEBUG = True
        print("Modo DEBUG ativado")

    # Inicializa vJoy
    try:
        j = pyvjoy.VJoyDevice(1)  # usa o device ID 1 configurado no vJoy
        vjoy_x = VJOY_CENTER
        vjoy_y = VJOY_CENTER
        j.set_axis(VJOY_AXIS_X, vjoy_x)
        j.set_axis(VJOY_AXIS_Y, vjoy_y)
        print("vJoy inicializado (Device ID 1).")
    except Exception as e:
        print(f"Falha ao inicializar vJoy / pyvjoy: {e}")
        print("Verifique se o driver vJoy está instalado e se o Device 1 está habilitado.")
        # se quiser, pode dar return aqui:
        # return

    # Solicita porta serial
    if len(sys.argv) > 1:
        # Filtra argumentos de debug
        port_args = [arg for arg in sys.argv[1:] if arg not in ('--debug', '-d')]
        if port_args:
            port = port_args[0]
        else:
            port = input("Porta serial (ex: COM3 ou /dev/ttyACM0): ").strip()
    else:
        port = input("Porta serial (ex: COM3 ou /dev/ttyACM0): ").strip()
    
    if not port:
        print("Porta serial não especificada!")
        return
    
    try:
        # Abre conexão serial
        ser = serial.Serial(port, 115200, timeout=0.1)
        print(f"Conectado à porta {port} (115200 baud)")
        print("Aguardando dados do controle...")
        print("Pressione Ctrl+C para sair")
        if DEBUG:
            print("Modo DEBUG: todas as mensagens serão exibidas\n")
        else:
            print("(Use --debug para ver mensagens detalhadas)\n")
        
        buffer = bytearray()
        
        while True:
            # Lê dados disponíveis
            dados = ser.read(ser.in_waiting or 1)
            if not dados:
                time.sleep(0.01)
                continue
            
            buffer.extend(dados)
            
            # Processa mensagens completas (4 bytes)
            while len(buffer) >= 4:
                # Extrai mensagem de 4 bytes
                mensagem = buffer[:4]
                buffer = buffer[4:]
                
                header = mensagem[0]
                byte1 = mensagem[1]
                byte2 = mensagem[2]
                byte3 = mensagem[3]
                
                # Verifica delimitador final (deve ser 0xFF)
                if byte3 != 0xFF:
                    # Mensagem inválida, descarta e procura próximo header válido
                    header_pos = -1
                    for i in range(1, len(buffer)):
                        if buffer[i] in (HEADER_TECLA, HEADER_MOUSE, HEADER_BOTAO_JOY):
                            header_pos = i
                            break
                    
                    if header_pos > 0:
                        buffer = buffer[header_pos:]
                    else:
                        buffer.clear()
                    continue
                
                # Debug: mostra mensagem recebida
                if DEBUG:
                    print(f"RX: [{header:02X} {byte1:02X} {byte2:02X} {byte3:02X}]")
                    print(f"Z: [{header:02X} {byte1:02X} {byte2:02X} {byte3:02X}]")
                
                # Processa mensagem conforme o header
                if header == HEADER_TECLA:
                    # Protocolo: [0x54, tecla_ascii, press/release, 0xFF]
                    tecla_ascii = byte1
                    press_release = byte2
                    if DEBUG:
                        print(f"  -> Tecla: 0x{tecla_ascii:02X}, press={press_release}")
                    processar_tecla(tecla_ascii, press_release)
                
                elif header == HEADER_MOUSE:
                    # Protocolo: [0x4D, delta_x, delta_y, 0xFF]
                    delta_x = byte1
                    delta_y = byte2
                    #print(f"RAW UART mouse bytes: dx_byte={delta_x:3d}, dy_byte={delta_y:3d}")
                    if DEBUG:
                        print(f"RAW UART mouse bytes: dx_byte={delta_x:3d}, dy_byte={delta_y:3d}")
                    processar_mouse(delta_x, delta_y)
                
                elif header == HEADER_BOTAO_JOY:
                    # Protocolo: [0x4A, press/release, 0x01, 0xFF]
                    press_release = byte1
                    if DEBUG:
                        print(f"  -> Botão joystick: press={press_release}")
                    processar_botao_joystick(press_release)
                
                else:
                    # Header desconhecido
                    print(f"Header desconhecido: 0x{header:02X} (bytes: {byte1:02X} {byte2:02X} {byte3:02X})")
    
    except serial.SerialException as e:
        print(f"Erro de comunicação serial: {e}")
        print("Verifique se a porta está correta e se o dispositivo está conectado.")
    
    except KeyboardInterrupt:
        print("\n\nSaindo...")
    
    except Exception as e:
        print(f"Erro inesperado: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Solta todas as teclas pressionadas
        for tecla in list(teclas_pressionadas):
            try:
                pyautogui.keyUp(TECLA_MAP.get(tecla, tecla))
            except:
                pass
        teclas_pressionadas.clear()
        
        # Centraliza eixos novamente ao sair
        if j is not None:
            try:
                j.set_axis(VJOY_AXIS_X, VJOY_CENTER)
                j.set_axis(VJOY_AXIS_Y, VJOY_CENTER)
            except:
                pass

        # Fecha conexão serial
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Conexão serial fechada.")

if __name__ == "__main__":
    main()