import socket
import cv2
import numpy as np

HOST = '0.0.0.0'
PORT = 5000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((HOST, PORT))
print(f"Receiver started on port {PORT}")

cv2.namedWindow('Video Stream', cv2.WINDOW_NORMAL)

while True:
    data, addr = sock.recvfrom(65536)
    if len(data) < 100:  # 跳过无效数据包
        continue
        
    img = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
    
    if img is None:
        print("ERROR: Failed to decode image (size=", len(data), ")")
        continue
        
    cv2.imshow('Video Stream', img)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
