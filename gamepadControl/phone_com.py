import socket

HOST = '127.0.0.1'  # Localhost (The USB Tunnel)
PORT = 6000         # Must match Android

print("Connecting to Phone...")
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    print("Connected! Waiting for data...")
    
    while True:
        data = s.recv(1024)
        if not data:
            break
        # Decode bytes to string
        message = data.decode('utf-8').strip()
        print(f"Received from Robot: {message}")