#!/usr/bin/env python3
"""
Script simples para testar comunicação UART
Mostra todos os bytes recebidos em hexadecimal
"""

import serial
import sys

def main():
    if len(sys.argv) < 2:
        print("Uso: python test_uart.py <porta>")
        print("Exemplo: python test_uart.py COM3")
        return
    
    port = sys.argv[1]
    
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        print(f"Conectado à porta {port} (115200 baud)")
        print("Aguardando dados... (pressione Ctrl+C para sair)\n")
        
        buffer = bytearray()
        byte_count = 0
        
        while True:
            dados = ser.read(ser.in_waiting or 1)
            if dados:
                buffer.extend(dados)
                byte_count += len(dados)
                
                # Mostra bytes recebidos
                for byte in dados:
                    print(f"{byte:02X} ", end='', flush=True)
                
                # Tenta identificar mensagens válidas
                while len(buffer) >= 4:
                    header = buffer[0]
                    byte1 = buffer[1]
                    byte2 = buffer[2]
                    byte3 = buffer[3]
                    
                    if byte3 == 0xFF:  # Delimitador válido
                        if header == 0x54:  # 'T' - Tecla
                            print(f"\n[TECLA] Tecla=0x{byte1:02X} ({chr(byte1) if 32 <= byte1 < 127 else '?'}), Press={byte2}")
                        elif header == 0x4D:  # 'M' - Mouse
                            print(f"\n[MOUSE] dx={byte1}, dy={byte2}")
                        elif header == 0x4A:  # 'J' - Botão joystick
                            print(f"\n[JOY] Press={byte1}")
                        else:
                            print(f"\n[UNKNOWN] Header=0x{header:02X}")
                    
                    buffer = buffer[4:]
            
            else:
                time.sleep(0.01)
    
    except serial.SerialException as e:
        print(f"Erro de comunicação serial: {e}")
    
    except KeyboardInterrupt:
        print(f"\n\nTotal de bytes recebidos: {byte_count}")
        print("Saindo...")
    
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    import time
    main()

