import bluetooth 
import time
import numpy as np
# 24:62:AB:D2:A7:06
bd_addr = "24:62:AB:D2:A7:06"
port = 1
sock=bluetooth.BluetoothSocket(bluetooth.RFCOMM)
sock.connect((bd_addr, port))


# sock.bind(bd_addr)
# sock.listen(1)

# server_sock=bluetooth.BluetoothSocket(bluetooth.RFCOMM)
# server_sock.bind(("24:62:AB:D2:A7:06", port))
# server_sock.listen(1)
# client_sock, address = server_sock.accept()
split = 0
held = 0
delimiter = '-'

while True: 
    # data = "hello!\n"
    # sock.send(data.encode())
    receive_data = sock.recv(4096)
    decoded_data = receive_data.decode()
    split_data = np.array(decoded_data.strip().split(delimiter))[1:]
    split_data = split_data[split_data != '']
    # print(split_data)
    float_data = split_data.astype(float)
    # print(float_data)
    if (len(float_data) != 0):
        print(np.round(np.mean(float_data), 2))
    time.sleep(0.25)
    # time.sleep(1)
    # print(type(decoded_data))

    # if '.' not in decoded_data:
    #     if split == 0:
    #         print(decoded_data)
    #     elif split == 1:
    #         print(held + decoded_data)
    #         split = 0
    #         held = 0
    # else:
    #     split = 1
    #     held = decoded_data
 
    # print(receive_data.decode())
    # # print(ord(receive_data.decode()))
    # decode_data = receive_data.decode().strip().split()
    # print(decode_data)
    # print(ord(decode_data[0]))
    # time.sleep(0.5);
    # int_data = int(decode_data)
    # print(int_data)
# sock.close()