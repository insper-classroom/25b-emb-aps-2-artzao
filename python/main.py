#!/usr/bin/env python3

import sys
import glob
import serial
import pyautogui
pyautogui.PAUSE = 0  # Remove delay between actions
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
from time import sleep

pyautogui.FAILSAFE = False

# (opcional) um pequeno intervalo entre ações ajuda a suavizar

# === ADDED/CHANGED ===
# Button protocol constants to match the firmware
PKT_AXIS_HDR   = 0xFF
PKT_BUTTON_HDR = 0xFE

KEY_LMB   = 1
KEY_RMB   = 2
KEY_SHIFT = 3
KEY_CTRL  = 4



def move_mouse(axis, value):
    """Move o mouse de acordo com o eixo e valor recebidos."""
    if axis == 0:
        pyautogui.moveRel(value, 0)
    elif axis == 1:
        pyautogui.moveRel(0, value)

# === ADDED/CHANGED ===
def handle_button(key_type: int, press: bool):
    """Converte o pacote de botão em uma ação pyautogui."""
    try:
        if key_type == KEY_LMB:
            if press:
                pyautogui.mouseDown(button='left')
            else:
                pyautogui.mouseUp(button='left')
        elif key_type == KEY_RMB:
            if press:
                pyautogui.mouseDown(button='right')
            else:
                pyautogui.mouseUp(button='right')
        elif key_type == KEY_SHIFT:
            if press:
                pyautogui.keyDown('shift')
            else:
                pyautogui.keyUp('shift')
        elif key_type == KEY_CTRL:
            if press:
                pyautogui.keyDown('ctrl')
            else:
                pyautogui.keyUp('ctrl')
        # else: desconhecido → ignore
    except Exception as e:
        # Evita quebrar o loop caso o sistema recuse eventos (ex: tela protegida)
        print(f"[WARN] Falha em aplicar ação de botão: {e}", file=sys.stderr)

def parse_axis_data(data):
    """Interpreta os dados recebidos do buffer (axis + valor)."""
    axis = data[0]
    value = int.from_bytes(data[1:3], byteorder='little', signed=True)
    return axis, value


def controle(ser):
    """
    Loop principal que lê bytes da porta serial:
      - 0xFF → pacote de eixo (axis + valor int16)
      - 0xFE → pacote de botão (key_type + flags + checksum)
    """
    while True:
        sync_byte = ser.read(size=1)
        if not sync_byte:
            continue

        hdr = sync_byte[0]

        if hdr == PKT_AXIS_HDR:
            # Ler 3 bytes (axis + valor(2b, little-endian, signed))
            data = ser.read(size=3)
            if len(data) < 3:
                continue
            axis, value = parse_axis_data(data)
            move_mouse(axis, value)

        # === ADDED/CHANGED ===
        elif hdr == PKT_BUTTON_HDR:
            # Ler 3 bytes (key_type, flags, checksum)
            data = ser.read(size=3)
            if len(data) < 3:
                continue
            key_type, flags, checksum = data[0], data[1], data[2]
            # Verifica checksum simples
            if ((key_type + flags) & 0xFF) != checksum:
                # checksum ruim → descarta
                print(f"[WARN] Checksum inválido: key={key_type} flags={flags} csum={checksum}", file=sys.stderr)
                continue
            press = bool(flags & 0x01)
            handle_button(key_type, press)

        else:
            # Byte desconhecido → continue procurando header
            continue


def serial_ports():
    """Retorna uma lista das portas seriais disponíveis na máquina."""
    ports = []
    if sys.platform.startswith('win'):
        # Windows
        for i in range(1, 256):
            port = f'COM{i}'
            try:
                s = serial.Serial(port)
                s.close()
                ports.append(port)
            except (OSError, serial.SerialException):
                pass
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # Linux/Cygwin
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        # macOS
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Plataforma não suportada para detecção de portas seriais.')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result


def parse_data(data):
    """Interpreta os dados recebidos do buffer (axis + valor)."""
    axis = data[0]
    value = int.from_bytes(data[1:3], byteorder='little', signed=True)
    return axis, value

def conectar_porta(port_name, root, botao_conectar, status_label, mudar_cor_circulo):
    """Abre a conexão com a porta selecionada e inicia o loop de leitura."""
    if not port_name:
        messagebox.showwarning("Aviso", "Selecione uma porta serial antes de conectar.")
        return

    ser = None
    try:
        ser = serial.Serial(port_name, 115200, timeout=1)
        status_label.config(text=f"Conectado em {port_name}", foreground="green")
        mudar_cor_circulo("green")
        botao_conectar.config(text="Conectado")
        root.update()

        # Inicia o loop de leitura (bloqueante).
        controle(ser)

    except KeyboardInterrupt:
        print("Encerrando via KeyboardInterrupt.")
    except Exception as e:
        messagebox.showerror("Erro de Conexão", f"Não foi possível conectar em {port_name}.\nErro: {e}")
        mudar_cor_circulo("red")
    finally:
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        status_label.config(text="Conexão encerrada.", foreground="red")
        mudar_cor_circulo("red")



def criar_janela():
    root = tk.Tk()
    root.title("Controle de Mouse")
    root.geometry("400x250")
    root.resizable(False, False)

    # Dark mode color settings
    dark_bg = "#2e2e2e"
    dark_fg = "#ffffff"
    accent_color = "#007acc"
    root.configure(bg=dark_bg)

    style = ttk.Style(root)
    style.theme_use("clam")
    style.configure("TFrame", background=dark_bg)
    style.configure("TLabel", background=dark_bg, foreground=dark_fg, font=("Segoe UI", 11))
    style.configure("TButton", font=("Segoe UI", 10, "bold"),
                    foreground=dark_fg, background="#444444", borderwidth=0)
    style.map("TButton", background=[("active", "#555555")])
    style.configure("Accent.TButton", font=("Segoe UI", 12, "bold"),
                    foreground=dark_fg, background=accent_color, padding=6)
    style.map("Accent.TButton", background=[("active", "#005f9e")])

    style.configure("TCombobox",
                    fieldbackground=dark_bg,
                    background=dark_bg,
                    foreground=dark_fg,
                    padding=4)
    style.map("TCombobox", fieldbackground=[("readonly", dark_bg)])

    frame_principal = ttk.Frame(root, padding="20")
    frame_principal.pack(expand=True, fill="both")

    titulo_label = ttk.Label(frame_principal, text="Controle de Mouse", font=("Segoe UI", 14, "bold"))
    titulo_label.pack(pady=(0, 10))

    porta_var = tk.StringVar(value="")

    status_label = tk.Label(root, text="Aguardando seleção de porta...", font=("Segoe UI", 11),
                            bg=dark_bg, fg=dark_fg)

    def mudar_cor_circulo(cor):
        circle_canvas.itemconfig(circle_item, fill=cor)

    botao_conectar = ttk.Button(
        frame_principal,
        text="Conectar e Iniciar Leitura",
        style="Accent.TButton",
        command=lambda: conectar_porta(porta_var.get(), root, botao_conectar, status_label, mudar_cor_circulo)
    )
    botao_conectar.pack(pady=10)

    footer_frame = tk.Frame(root, bg=dark_bg)
    footer_frame.pack(side="bottom", fill="x", padx=10, pady=(10, 0))

    status_label = tk.Label(footer_frame, text="Aguardando seleção de porta...", font=("Segoe UI", 11),
                            bg=dark_bg, fg=dark_fg)
    status_label.grid(row=0, column=0, sticky="w")

    portas_disponiveis = serial_ports()
    if portas_disponiveis:
        porta_var.set(portas_disponiveis[0])
    port_dropdown = ttk.Combobox(footer_frame, textvariable=porta_var,
                                 values=portas_disponiveis, state="readonly", width=10)
    port_dropdown.grid(row=0, column=1, padx=10)

    circle_canvas = tk.Canvas(footer_frame, width=20, height=20, highlightthickness=0, bg=dark_bg)
    circle_item = circle_canvas.create_oval(2, 2, 18, 18, fill="red", outline="")
    circle_canvas.grid(row=0, column=2, sticky="e")

    footer_frame.columnconfigure(1, weight=1)

    root.mainloop()


if __name__ == "__main__":
    criar_janela()
