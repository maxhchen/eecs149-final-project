import bluetooth 
import time
# 24:62:AB:D2:A7:06
bd_addr = "24:62:AB:D2:A7:06"
port = 1
sock=bluetooth.BluetoothSocket(bluetooth.RFCOMM)
sock.connect((bd_addr, port))


sock.bind(bd_addr)
sock.listen(1)

# server_sock=bluetooth.BluetoothSocket(bluetooth.RFCOMM)
# server_sock.bind(("24:62:AB:D2:A7:06", port))
# server_sock.listen(1)
# client_sock, address = server_sock.accept()

while True: 
    # data = "hello!\n"
    # sock.send(data.encode())
    receive_data = sock.recv(4096)
    # # print(ord(receive_data.decode()))
    decode_data = receive_data.decode().strip().split()
    print(decode_data)
    print(ord(decode_data[0]))
    time.sleep(0.5);
    # int_data = int(decode_data)
    # print(int_data)
# sock.close()