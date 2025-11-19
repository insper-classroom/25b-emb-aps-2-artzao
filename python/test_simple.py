#!/usr/bin/env python3
"""
Teste muito simples - apenas mostra TODOS os bytes recebidos
Útil para verificar se há comunicação básica
"""

import serial
import sys
import time

def main():
    if len(sys.argv) < 2:
        print("Uso: python test_simple.py <porta>")
        print("Exemplo: python test_simple.py COM3")
        return
    
    port = sys.argv[1]
    
    try:
        print(f"Tentando conectar à porta {port}...")
        ser = serial.Serial(port, 115200, timeout=1)
        print(f"✓ Conectado! Aguardando dados...\n")
        print("Qualquer byte recebido será mostrado abaixo:\n")
        
        byte_count = 0
        start_time = time.time()
        
        while True:
            if ser.in_waiting > 0:
                byte = ser.read(1)[0]
                byte_count += 1
                print(f"[{byte_count:04d}] 0x{byte:02X} ({byte:3d}) '{chr(byte) if 32 <= byte < 127 else '.'}'")
                
                # A cada 4 bytes, tenta identificar mensagem
                if byte_count % 4 == 0:
                    print("  <- Possível mensagem completa")
            else:
                elapsed = time.time() - start_time
                if elapsed > 5 and byte_count == 0:
                    print(f"\n⚠ Nenhum dado recebido em {elapsed:.1f} segundos!")
                    print("   Verifique:")
                    print("   1. Se o firmware está rodando no Pico")
                    print("   2. Se os pinos UART estão conectados corretamente")
                    print("   3. Se a porta serial está correta")
                    print("   4. Se o baud rate está correto (115200)")
                    start_time = time.time()  # Reset timer
            
            time.sleep(0.01)
    
    except serial.SerialException as e:
        print(f"✗ Erro ao abrir porta serial: {e}")
        print("\nPossíveis causas:")
        print("  - Porta não existe ou está incorreta")
        print("  - Porta já está em uso por outro programa")
        print("  - Dispositivo não está conectado")
        print("\nNo Windows, portas são COM1, COM2, COM3, etc.")
        print("No Linux/Mac, portas são /dev/ttyACM0, /dev/ttyUSB0, etc.")
    
    except KeyboardInterrupt:
        print(f"\n\nTotal de bytes recebidos: {byte_count}")
        if byte_count == 0:
            print("⚠ NENHUM dado foi recebido!")
            print("\nIsso indica que:")
            print("  1. O firmware pode não estar rodando")
            print("  2. Os pinos UART podem estar errados")
            print("  3. A conexão física pode estar com problema")
        print("\nSaindo...")
    
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()

