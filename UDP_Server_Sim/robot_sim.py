import socket

# Mudar para o IP da máquina onde o servidor está a correr (127.0.0.1 se for local)
SERVER_IP = "127.0.0.1" 
SERVER_PORT = 12345

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Timeout estrito de 2 segundos. Em robótica real, não podes bloquear a thread de controlo.
sock.settimeout(2.0) 

print("--- TERMINAL DO ROBÔ ---")
print("Comandos válidos: IWP, OWP, MAP, MBP, CTL, PING")

while True:
    msg = input("Enviar> ").strip().upper()
    if not msg: 
        continue
    
    try:
        sock.sendto(msg.encode('utf-8'), (SERVER_IP, SERVER_PORT))
        data, addr = sock.recvfrom(1024)
        print(f"Recebido: {data.decode('utf-8')}")
    except socket.timeout:
        print("[ERRO] Timeout. O pacote perdeu-se ou o servidor caiu.")
    except Exception as e:
        print(f"[ERRO] Falha de socket: {e}")
