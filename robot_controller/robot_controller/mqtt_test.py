import socket
import time

s = socket.socket()
s.bind(('0.0.0.0', 3333))
s.listen(1)
print("[SERVER] Ready on port 3333...")

conn, addr = s.accept()
print("[SERVER] Connected by", addr)

while True:
    data = conn.recv(1024)
    if not data:
        print("[SERVER] Connection closed")
        break
    print("[SERVER] Got from ESP32:", data.decode().strip())
    conn.sendall(b"Hello from Laptop\n")
