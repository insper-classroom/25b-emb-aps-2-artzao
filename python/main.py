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
BUTTON_MAP = {
    2: 'w',
    3: 'a',
    4: 's',
    5: 'd',
    # Exemplos extras:
    # 6: 'space',
    # 7: 'esc',
}

button_states = {}  # {button_id: 0/1}

def move_mouse(axis, value):
    """Move o mouse de acordo com o eixo e valor recebidos."""
    if axis == 0:
        pyautogui.moveRel(value, 0)
    elif axis == 1:
        pyautogui.moveRel(0, value)


def handle_button(button_id, value):
    """
    Trata botões:
      value = 1 -> keyDown
      value = 0 -> keyUp
    Evita eventos duplicados comparando com o último estado conhecido.
    """
    if button_id not in BUTTON_MAP:
        return  # botão sem mapeamento → ignora
    key = BUTTON_MAP[button_id]

    prev = button_states.get(button_id, None)
    # Só envia evento se mudou o estado (debounce lógico)
    if prev != value:
        if value == 1:
            pyautogui.keyDown(key)
        else:
            pyautogui.keyUp(key)
        button_states[button_id] = value

def handle_input(axis, value):
    """
    Direciona o pacote para mouse (axis 0/1) ou botões (axis >=2).
    Para botões, espera-se value em {0,1}.
    """
    if axis in (0, 1):
        move_mouse(axis, value)
    else:
        # Trata como botão
        handle_button(axis, 1 if value != 0 else 0)

def controle(ser):
    """
    Loop principal que lê bytes da porta serial em loop infinito.
    Aguarda o byte 0xFF e então lê 3 bytes: axis (1 byte) + valor (2 bytes).
    """
    while True:
        # Aguardar byte de sincronização
        sync_byte = ser.read(size=1)
        if not sync_byte:
            continue
        if sync_byte[0] == 0xFF:
            # Ler 3 bytes (axis + valor(2b))
            data = ser.read(size=3)
            if len(data) < 3:
                continue
            print(data)
            axis, value = parse_data(data)
            handle_input(axis, value)


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

        controle(ser)

    except KeyboardInterrupt:
        print("Encerrando via KeyboardInterrupt.")
    except Exception as e:
        messagebox.showerror("Erro de Conexão", f"Não foi possível conectar em {port_name}.\nErro: {e}")
        mudar_cor_circulo("red")
    finally:
        try:
            if ser and ser.is_open:
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

    # Dark mode
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

    status_label = None  # declarado antes para fechar sobre ele na lambda

    def start_connect():
        conectar_porta(porta_var.get(), root, botao_conectar, status_label, mudar_cor_circulo)

    botao_conectar = ttk.Button(
        frame_principal,
        text="Conectar e Iniciar Leitura",
        style="Accent.TButton",
        command=start_connect
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

    def mudar_cor_circulo(cor):
        circle_canvas.itemconfig(circle_item, fill=cor)

    root.mainloop()


if __name__ == "__main__":
    criar_janela()
