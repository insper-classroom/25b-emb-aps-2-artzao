#!/usr/bin/env python3
"""
Testa múltiplos baud rates para encontrar o correto
"""

import serial
import sys
import time

BAUD_RATES = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]

def test_baudrate(port, baud):
    """Testa um baud rate específico"""
    try:
        ser = serial.Serial(port, baud, timeout=0.5)
        time.sleep(0.1)  # Pequeno delay para estabilização
        
        # Limpa buffer
        ser.reset_input_buffer()
        
        # Aguarda dados por 2 segundos
        start_time = time.time()
        bytes_received = 0
        sample_bytes = []
        
        while time.time() - start_time < 2.0:
            if ser.in_waiting > 0:
                byte = ser.read(1)[0]
                bytes_received += 1
                if len(sample_bytes) < 10:  # Guarda primeiros 10 bytes como amostra
                    sample_bytes.append(byte)
            time.sleep(0.01)
        
        ser.close()
        
        if bytes_received > 0:
            return True, bytes_received, sample_bytes
        return False, 0, []
    
    except Exception as e:
        return None, 0, str(e)

def main():
    if len(sys.argv) < 2:
        print("Uso: python test_baudrate.py <porta>")
        print("Exemplo: python test_baudrate.py COM3")
        return
    
    port = sys.argv[1]
    
    print(f"Testando porta {port} com diferentes baud rates...")
    print("Pressione os botões ou mova o joystick durante o teste\n")
    
    results = []
    
    for baud in BAUD_RATES:
        print(f"Testando {baud} baud... ", end='', flush=True)
        success, count, data = test_baudrate(port, baud)
        
        if success is None:
            print(f"ERRO: {data}")
        elif success:
            print(f"✓ Recebeu {count} bytes!")
            if data:
                hex_str = ' '.join([f'{b:02X}' for b in data])
                print(f"  Amostra: {hex_str}")
            results.append((baud, count, data))
        else:
            print("✗ Nenhum dado")
        
        time.sleep(0.2)  # Pequeno delay entre testes
    
    print("\n" + "="*50)
    print("RESUMO:")
    print("="*50)
    
    if results:
        print("\nBaud rates que receberam dados:")
        for baud, count, data in results:
            print(f"  {baud} baud: {count} bytes")
            if data:
                hex_str = ' '.join([f'{b:02X}' for b in data])
                print(f"    Primeiros bytes: {hex_str}")
        
        # Recomenda o baud rate com mais dados
        best = max(results, key=lambda x: x[1])
        print(f"\n✓ RECOMENDAÇÃO: Use {best[0]} baud (recebeu mais dados)")
    else:
        print("\n✗ NENHUM baud rate recebeu dados!")
        print("\nPossíveis problemas:")
        print("  1. Firmware não está rodando")
        print("  2. Pinos UART não estão conectados corretamente")
        print("  3. TX/RX podem estar invertidos")
        print("  4. GND não está conectado")
        print("  5. Porta serial está incorreta")

if __name__ == "__main__":
    main()

