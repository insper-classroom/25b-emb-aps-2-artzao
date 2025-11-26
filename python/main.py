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
import pyvjoy

# Desabilita failsafe e pausa do pyautogui para resposta rápida
pyautogui.PAUSE = 0
pyautogui.FAILSAFE = False

# =============== DEBUG ===============
# Força debug ligado por enquanto
DEBUG = True   # <<< GARANTE LOGS

# ========================= DEFINES - PROTOCOLO =========================

HEADER_TECLA = 0x54      # 'T' - Tecla
HEADER_MOUSE = 0x4D      # 'M' - Mouse/Câmera
HEADER_BOTAO_JOY = 0x4A  # 'J' - Botão do joystick

TECLA_W = 0x57
TECLA_A = 0x41
TECLA_S = 0x53
TECLA_D = 0x44
TECLA_SPACE = 0x20
TECLA_E = 0x45
TECLA_TAB = 0x09
TECLA_CTRL = 0x11

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

teclas_pressionadas = set()

# ========================= ESTADO DO vJoy ========================

VJOY_MAX = 0x8000 - 1         # 32767
VJOY_CENTER = VJOY_MAX // 2   # ~16383

# Use RX / RY como câmera (right stick)
VJOY_AXIS_X = pyvjoy.HID_USAGE_Z   # look horizontal
VJOY_AXIS_Y = pyvjoy.HID_USAGE_RX   # look vertical

j = None
vjoy_x = VJOY_CENTER
vjoy_y = VJOY_CENTER

# Sensibilidade e deadzone
VJOY_SENS = 300   # ajuste depois se ficar muito forte/fraco
DEADZONE  = 2      # ignora ruído pequeno

# ========================= FUNÇÕES DE PROCESSAMENTO =========================

def processar_tecla(tecla_ascii, press_release):
    if tecla_ascii not in TECLA_MAP:
        print(f"Tecla desconhecida: 0x{tecla_ascii:02X}")
        return

    nome_tecla = TECLA_MAP[tecla_ascii]

    try:
        if press_release == 1:
            if nome_tecla not in teclas_pressionadas:
                pyautogui.keyDown(nome_tecla)
                teclas_pressionadas.add(nome_tecla)
                print(f"Pressionou: {nome_tecla}")
        else:
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
        delta_x: Movimento horizontal (byte sem sinal)
        delta_y: Movimento vertical (byte sem sinal)
    """
    global j, vjoy_x, vjoy_y

    if j is None:
        # vJoy não inicializado
        return

    try:
        # Guarda os bytes crus só pra debug
        raw_dx = delta_x
        raw_dy = delta_y

        # Converte bytes sem sinal (0..255) para valores com sinal (-128..127)
        if delta_x > 127:
            delta_x = delta_x - 256
        if delta_y > 127:
            delta_y = delta_y - 256

        if DEBUG:
            print(f"RAW UART mouse bytes: dx_byte={raw_dx:3d}, dy_byte={raw_dy:3d}")
            print(f"[SIGNED] dx={delta_x}, dy={delta_y}")

        # ---------------- DOMINÂNCIA VERTICAL ----------------
        # Se o movimento vertical é mais forte que o horizontal,
        # consideramos que o usuário quer só olhar pra cima/baixo
        # e zeramos o eixo horizontal.
        if abs(delta_y) > abs(delta_x) and abs(delta_y) >= 2:
            delta_x = 0
            if DEBUG:
                print(f"VERT_DOM: forçando dx=0, dy={delta_y}")

        # ---------------- DEADZONE ----------------
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
            vjoy_x = VJOY_CENTER + int(delta_x * VJOY_SENS)
            vjoy_y = VJOY_CENTER - int(delta_y * VJOY_SENS)  # inverte Y estilo FPS

        # Garante que fique dentro do range do vJoy
        vjoy_x = max(0, min(VJOY_MAX, vjoy_x))
        vjoy_y = max(0, min(VJOY_MAX, vjoy_y))

        j.set_axis(VJOY_AXIS_X, vjoy_x)
        j.set_axis(VJOY_AXIS_Y, vjoy_y)

        if DEBUG:
            print(f"vJoy: X={vjoy_x}, Y={vjoy_y}, dx={delta_x}, dy={delta_y}")
            print()

    except Exception as e:
        print(f"Erro ao mover câmera (vJoy): {e}")
# ========================= FUNÇÃO PRINCIPAL =========================

def main():
    global DEBUG, j, vjoy_x, vjoy_y

    # Só para ver como o script está sendo chamado
    print("sys.argv =", sys.argv)

    # Mesmo assim, ainda respeita -d / --debug se você quiser desativar depois
    if '--debug' in sys.argv or '-d' in sys.argv:
        DEBUG = True
        print("Modo DEBUG ativado")
    else:
        # Neste momento forçamos DEBUG=True lá em cima,
        # mas mantemos essa mensagem para clareza.
        print("Rodando com DEBUG =", DEBUG)

    # Inicializa vJoy
    try:
        j = pyvjoy.VJoyDevice(1)
        vjoy_x = VJOY_CENTER
        vjoy_y = VJOY_CENTER
        j.set_axis(VJOY_AXIS_X, vjoy_x)
        j.set_axis(VJOY_AXIS_Y, vjoy_y)
        print("vJoy inicializado (Device ID 1).")
    except Exception as e:
        print(f"Falha ao inicializar vJoy / pyvjoy: {e}")
        print("Verifique vJoyConf: Device 1 habilitado, RX/RY marcados.")
        return

    # Porta serial
    if len(sys.argv) > 1:
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
        ser = serial.Serial(port, 115200, timeout=0.1)
        print(f"Conectado à porta {port} (115200 baud)")
        print("Aguardando dados do controle...")
        print("Pressione Ctrl+C para sair")

        buffer = bytearray()

        while True:
            dados = ser.read(ser.in_waiting or 1)
            if not dados:
                time.sleep(0.01)
                continue

            buffer.extend(dados)

            while len(buffer) >= 4:
                mensagem = buffer[:4]
                buffer = buffer[4:]

                header = mensagem[0]
                byte1  = mensagem[1]
                byte2  = mensagem[2]
                byte3  = mensagem[3]

                # Verifica delimitador final (0xFF)
                if byte3 != 0xFF:
                    # descarta até próximo possível header
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

                if DEBUG:
                    print(f"RX: [{header:02X} {byte1:02X} {byte2:02X} {byte3:02X}]", flush=True)

                if header == HEADER_TECLA:
                    tecla_ascii = byte1
                    press_release = byte2
                    if DEBUG:
                        print(f"  -> Tecla: 0x{tecla_ascii:02X}, press={press_release}", flush=True)
                    processar_tecla(tecla_ascii, press_release)

                elif header == HEADER_MOUSE:
                    dx_byte = byte1
                    dy_byte = byte2
                    if DEBUG:
                        print(f"RAW UART mouse bytes: dx_byte={dx_byte:3d}, dy_byte={dy_byte:3d}", flush=True)
                    processar_mouse(dx_byte, dy_byte)

                elif header == HEADER_BOTAO_JOY:
                    press_release = byte1
                    if DEBUG:
                        print(f"  -> Botão joystick: press={press_release}", flush=True)
                    processar_botao_joystick(press_release)

                else:
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
        for tecla in list(teclas_pressionadas):
            try:
                pyautogui.keyUp(TECLA_MAP.get(tecla, tecla))
            except:
                pass
        teclas_pressionadas.clear()

        if j is not None:
            try:
                j.set_axis(VJOY_AXIS_X, VJOY_CENTER)
                j.set_axis(VJOY_AXIS_Y, VJOY_CENTER)
            except:
                pass

        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Conexão serial fechada.")

if __name__ == "__main__":
    main()
