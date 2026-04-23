import socket

# Mudar para o IP da máquina onde o servidor está a correr (127.0.0.1 se for local)
SERVER_IP = "127.0.0.1" 
SERVER_PORT = 12345

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Timeout estrito de 2 segundos.
sock.settimeout(2.0) 

print("--- TERMINAL DO ROBÔ ---")
print("Comandos válidos: IWP, OWP, MAP, MBP, CTL, PING, EXIT")

while True:
    try:
        msg = input("Enviar> ").strip().upper()
        if not msg: 
            continue
            
        # Comando para sair limpo do simulador
        if msg == "EXIT" or msg == "SAIR":
            print("[SISTEMA] A encerrar o simulador do robô. Boa sorte na prova.")
            break
        
        sock.sendto(msg.encode('utf-8'), (SERVER_IP, SERVER_PORT))
        data, addr = sock.recvfrom(1024)
        print(f"Recebido: {data.decode('utf-8')}")
        
    except socket.timeout:
        print("[ERRO] Timeout. O pacote perdeu-se ou o servidor não respondeu a tempo.")
    except ConnectionResetError:
        # Tratamento específico para o WinError 10054
        print("[ERRO REDE] O servidor de destino rejeitou o pacote. Provavelmente está desligado.")
    except KeyboardInterrupt:
        # Tratamento limpo para o Ctrl+C
        print("\n[SISTEMA] Interrupção manual (Ctrl+C). A sair...")
        break
    except Exception as e:
        print(f"[ERRO] Falha de socket não prevista: {e}")

# Fechar o socket e libertar a porta ao sair
sock.close()