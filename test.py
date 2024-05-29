# import socket
# import time


# # 设置目标地址和端口
# HOST = '192.168.31.68'  # 目标地址
# PORT = 12310  # 目标端口

# # 创建socket对象
# s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# #try:
#     # 连接到目标地址和端口
# print("Connecting to server...")
# s.connect((HOST, PORT))
# print("Connected to server!")
# while True:
#     # 获取数据 横滚 俯仰 偏航 融合状态
#     # ROL, PIT, YAW, FUSION_STA = getdata()
#     ROL, PIT, YAW, FUSION_STA = 0, 0, 0, 1
#     print("ROL:", ROL, "PIT:", PIT, "YAW:", YAW, "FUSION_STA:", FUSION_STA)
    
#     # 构造帧头
#     #HEAD = 0xAAFF
#     HEAD = 0xAB
#     # 源地址，这里假设源地址为0x00
#     #S_ADDR = 0x00
#     S_ADDR = 0x01
#     # 目标地址，这里假设目标地址为0x01
#     #D_ADDR = 0x01
#     D_ADDR = 0xFE
#     # 功能码，这里假设为0x03
#     ID = 0xF1
#     # 数据内容
#     DATA = ROL, PIT, YAW, FUSION_STA
#     # 数据长度，根据数据内容的长度计算
#     #LEN = len(DATA)
#     LEN = len(DATA)
#     # 计算和校验，这里简单地将数据内容按位求和
#     SC = HEAD
#     AC = SC
#     SC += D_ADDR
#     AC += SC
#     SC += ID
#     AC += SC
#     SC += LEN
#     AC += SC
#     for value in DATA:
#         SC += value
#         AC += SC

#     SC &= 0xFF
#     AC &= 0xFF
#     print("SC:", SC, "AC:", AC)
#     # 构造帧
#     #frame = bytearray([HEAD >> 8, HEAD & 0xFF, D_ADDR, ID, LEN])
#     frame = bytearray([HEAD, S_ADDR, D_ADDR, ID, LEN])
#     for value in DATA: 
#         frame.append(value)

#     frame.extend([SC, AC])
    
#     print("Frame:", frame)
#     # 发送帧
#     s.sendto(frame, (HOST, PORT))
#     print("Frame sent:", frame)
    
#     print("Frame sent successfully!")
#     time.sleep(1)
# s.close()

# #except Exception as e:
#     #print("Error:", e)
# #finally:
#     # 关闭socket连接
#     #s.close()

import socket

# 创建UDP套接字
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 绑定要监听的IP地址和端口
listen_addr = ('', 12300)  # 在所有网络接口上监听，端口号为12345
udp_socket.bind(listen_addr)

print("UDP服务器已启动，正在监听端口", listen_addr[1])

while True:
    # 接收数据
    # data, addr = udp_socket.recvfrom(1024)  # 每次最多接收1024字节的数据
    # print(data, addr)

    # udp_socket.sendto(data, ('192.168.31.68', 12312))

    def calSCAC(data):
        SC = 0x00
        AC = 0x00
        for i in range(len(data)):
            SC += data[i] & 0xFF
            AC += SC & 0xFF
            SC &= 0xFF
            AC &= 0xFF
        return SC, AC

    # HEAD = 0xAA
    # D_ADDR = 0xFF
    # ID = 0xF1
    # LEN = 8
    # DATA = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0xAB, 0x01]

    # SC, AC = calSCAC([HEAD, D_ADDR, ID, LEN, DATA[0], DATA[1], DATA[2], DATA[3], DATA[4], DATA[5], DATA[6], DATA[7]])

    # SC &= 0xFF
    # AC &= 0xFF

    # # SC = 73
    # # AC = 49

    # # SC = 0x49
    # # AC = 0x31

    # protocol_data = bytes([HEAD, D_ADDR, ID, LEN])
    # protocol_data += bytes(DATA)
    # protocol_data += bytes([SC, AC])
    
    # # protocol_data = B'\xaa\xff\xf1\x08\x4c\x04\xb0\x04\x14\x05\x00\x00\xbf\xae'

    # # 发送数据
    # udp_socket.sendto(protocol_data, ('192.168.31.68', 12312))
    # print("数据已发送:", protocol_data)
    # import time
    # time.sleep(1)
    
    # break

    HEAD = 0xAA
    D_ADDR = 0xFF
    ID = 0x03
    LEN = 7
    DATA = [0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x01]

    SC, AC = calSCAC([HEAD, D_ADDR, ID, LEN, DATA[0], DATA[1], DATA[2], DATA[3], DATA[4], DATA[5], DATA[6]])

    SC &= 0xFF
    AC &= 0xFF

    protocol_data = bytes([HEAD, D_ADDR, ID, LEN])
    protocol_data += bytes(DATA)
    protocol_data += bytes([SC, AC])
    
    # 发送数据
    udp_socket.sendto(protocol_data, ('192.168.31.68', 12312))
    print("数据已发送:", protocol_data)

    continue

    data, addr = udp_socket.recvfrom(1024)  # 每次最多接收1024字节的数据
    print(data, addr)

    HEAD = 0xAA
    D_ADDR = 0xFF
    ID = 0x00
    LEN = 3
    DATA = data[2],data[6],data[7]


    
    SC, AC = calSCAC([HEAD, D_ADDR, ID, LEN, data[2], data[6], data[7]])

    protocol_data = bytes([HEAD, D_ADDR, ID, LEN])
    protocol_data += bytes(DATA)
    protocol_data += bytes([SC, AC])
    
    # 发送数据
    udp_socket.sendto(protocol_data, ('192.168.31.68', 12312))
    print("数据已发送:", protocol_data)

    break

# 关闭套接字
udp_socket.close()