import socket
import time
import threading

# Configurações de Rede
UDP_IP = "0.0.0.0"
UDP_PORT = 12345

class FNRServer:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP, UDP_PORT))
        self.state = "STOPPED" # Estados: STOPPED ou RUNNING
        self.start_time = 0
        self.duration = 600 # 10 minutos por tentativa
        
        # Estado estático das peças para teste
        self.iwp = "RGBR"
        self.owp = "XXXX"
        self.map = "BXXG"
        self.mbp = "XXXX"

    def get_time_left(self):
        if self.state == "STOPPED": 
            return "T600"
        elapsed = int(time.time() - self.start_time)
        left = max(0, self.duration - elapsed)
        return f"T{left:03d}"

    def process_request(self, msg):
        # Regra de bloqueio pré-início
        if self.state == "STOPPED" and msg != "PING":
            return "STOP"
        
        # Máquina de estados
        if msg == "IWP": return self.iwp
        if msg == "OWP": return self.owp
        if msg == "MAP": return self.map
        if msg == "MBP": return self.mbp
        if msg == "CTL": return self.get_time_left()
        if msg == "PING": return "PONG"
        
        return "STOP" # Fallback

    def run(self):
        print(f"[REDE] Servidor escutando na porta {UDP_PORT}...")
        while True:
            data, addr = self.sock.recvfrom(1024)
            msg = data.decode('utf-8').strip()
            resp = self.process_request(msg)
            self.sock.sendto(resp.encode('utf-8'), addr)
            print(f"[LOG] {addr[0]} pediu '{msg}' -> Resposta: '{resp}'")

# Inicialização assíncrona para permitir controlo via terminal
server = FNRServer()
t = threading.Thread(target=server.run, daemon=True)
t.start()

print("\n--- PAINEL DE CONTROLO DO JÚRI ---")
print("Comandos: 'start' (inicia relógio), 'stop' (bloqueia robô), 'exit' (fecha).")

while True:
    cmd = input("> ").strip().lower()
    if cmd == "start":
        server.state = "RUNNING"
        server.start_time = time.time()
        print("[ESTADO] Corrida iniciada. Robô autorizado a mover-se.")
    elif cmd == "stop":
        server.state = "STOPPED"
        print("[ESTADO] Corrida parada. Robô bloqueado (STOP).")
    elif cmd == "exit":
        break