import bluetooth
import struct 
# 24:62:AB:D2:A7:06

sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
bd_addr = "24:62:AB:D2:A7:06"
port = 1
sock.connect((bd_addr, port))


# port = 1
# server_sock.bind((bd_addr,0))
# server_sock.listen(1)
# cient_sock, address = server_sock.accept()
# print("Accepted connection from ",address)
while True: 
    data = sock.recv(1024)
    # print(data)
    # struct.unpack("<Q", data)  
    print(data) 
    # print("received [%s]" % data)

client_sock.close()
server_sock.close()