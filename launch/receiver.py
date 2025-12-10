import socket
import cv2
import numpy as np

HOST = '0.0.0.0'
PORT = 5000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((HOST, PORT))
print(f"Receiver started on port {PORT}")

# 关键修复：动态获取图像尺寸（不要硬编码）
while True:
    data, addr = sock.recvfrom(65536)
    if len(data) < 100:  # 跳过无效数据包
        continue
        
    # 修复1：解码为BGR（标准格式）
    img = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
    
    if img is None:
        print("ERROR: Failed to decode image (size=", len(data), ")")
        continue
        
    # 修复2：动态显示（不缩放！）
    cv2.imshow('Video Stream', img)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()